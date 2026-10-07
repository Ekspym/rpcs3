#include "stdafx.h"
#include "rb_input.h"

#include "Emu/Cell/PPUThread.h"
#include "Emu/Memory/vm.h"

#include <cstdlib>
#include <chrono>
#include <cstring>

#ifdef _WIN32
#include <Windows.h>
#endif

LOG_CHANNEL(rb_native_log, "RbNative");

extern void ppu_execute_syscall(ppu_thread& ppu, u64 code);

// Hybrid (RB_NATIVE=<t6_native.dll>): the game's PPU code lifted to C++ by ps3recomp from the user's own EBOOT
// (recomp_t6/hybrid) runs the frame's task steps natively; RPCS3 keeps everything else (SPU, RSX, sound, lv2,
// firmware libraries) and runs whatever the lifted code calls that has no lifted function.
namespace rb::native
{
	// Must match ppu_context of ps3recomp's lifted code (ppu_recomp.h) field by field
	struct ppu_context
	{
		u64 gpr[32];
		f64 fpr[32];
		alignas(16) u8 vr[32][16]; // raw big-endian guest bytes
		u32 cr;
		u64 lr;
		u64 ctr;
		u32 xer;
		u32 fpscr;
		u32 vscr;
		u64 cia;
		u64 thread_id;
		u64 reserve_addr;
		u64 reserve_value;
		int reserve_valid;
	};

	struct host_api
	{
		u8* base;
		void (*call_guest)(ppu_context* ctx, u32 addr);
		void (*syscall)(ppu_context* ctx);
		u64 (*timebase)();
	};

	using init_fn = int (*)(const host_api*);
	using run_fn = int (*)(u32 addr, ppu_context* ctx);
	using size_fn = unsigned (*)();

	run_fn g_run = nullptr;
	thread_local ppu_thread* g_ppu = nullptr;

	void to_ctx(const ppu_thread& ppu, ppu_context& c)
	{
		std::memcpy(c.gpr, ppu.gpr, sizeof(c.gpr));
		std::memcpy(c.fpr, ppu.fpr, sizeof(c.fpr));

		// RPCS3 keeps vector registers byte-reversed (element 0 in the high bytes)
		for (u32 i = 0; i < 32; i++)
		{
			for (u32 b = 0; b < 16; b++)
			{
				c.vr[i][b] = ppu.vr[i]._u8[15 - b];
			}
		}

		c.cr = ppu.cr.pack();
		c.lr = ppu.lr;
		c.ctr = ppu.ctr;
		c.xer = (ppu.xer.so ? 0x80000000u : 0) | (ppu.xer.ov ? 0x40000000u : 0) | (ppu.xer.ca ? 0x20000000u : 0) | (ppu.xer.cnt & 0x7f);
		c.fpscr = ppu.fpscr.bits.pack();
		c.vscr = (ppu.nj ? 0x10000u : 0) | ((ppu.sat._u64[0] | ppu.sat._u64[1]) ? 1u : 0);
		c.cia = ppu.cia;
		c.thread_id = ppu.id;
	}

	void from_ctx(const ppu_context& c, ppu_thread& ppu)
	{
		std::memcpy(ppu.gpr, c.gpr, sizeof(c.gpr));
		std::memcpy(ppu.fpr, c.fpr, sizeof(c.fpr));

		for (u32 i = 0; i < 32; i++)
		{
			for (u32 b = 0; b < 16; b++)
			{
				ppu.vr[i]._u8[15 - b] = c.vr[i][b];
			}
		}

		ppu.cr.unpack(c.cr);
		ppu.lr = c.lr;
		ppu.ctr = c.ctr;
		ppu.xer.so = !!(c.xer & 0x80000000u);
		ppu.xer.ov = !!(c.xer & 0x40000000u);
		ppu.xer.ca = !!(c.xer & 0x20000000u);
		ppu.xer.cnt = static_cast<u8>(c.xer & 0x7f);
		ppu.fpscr.bits.unpack(c.fpscr);
		ppu.nj = !!(c.vscr & 0x10000u);

		if (!(c.vscr & 1))
		{
			ppu.sat = v128{};
		}
	}

	// Back in the lifted code after RPCS3 ran something (syscall, firmware or HLE function): like RPCS3's own PPU code
	// at the next block, the thread leaves the "waiting / not accessing guest memory" state (cpu_flag::wait set by a
	// syscall or a blocking call) and serves pending suspend requests. Without this the lifted code kept running while
	// RPCS3 considered the thread out of guest memory: SPU atomic operations and suspend_all ran concurrently with it.
	void resume(ppu_thread& ppu)
	{
		if (ppu.state)
		{
			ppu.check_state();
		}
	}

	// The lifted code reached an address without a lifted function: RPCS3 runs it with the same registers
	void host_call_guest(ppu_context* ctx, u32 addr)
	{
		ppu_thread& ppu = *g_ppu;
		from_ctx(*ctx, ppu);
		ppu.fast_call(addr, ppu.gpr[2]);
		resume(ppu);
		to_ctx(ppu, *ctx);
	}

	void host_syscall(ppu_context* ctx)
	{
		ppu_thread& ppu = *g_ppu;
		from_ctx(*ctx, ppu);
		ppu_execute_syscall(ppu, ppu.gpr[11]);
		resume(ppu);
		to_ctx(ppu, *ctx);
	}

	u64 host_timebase()
	{
		return rb::guest_timebase();
	}

	bool init()
	{
		static const bool ok = []()
		{
#ifdef _WIN32
			const char* path = std::getenv("RB_NATIVE");

			if (!path || !*path)
			{
				return false;
			}

			const HMODULE dll = LoadLibraryA(path);

			if (!dll)
			{
				rb_native_log.error("Native code %s cannot be loaded (error %u)", path, GetLastError());
				return false;
			}

			const auto init_f = reinterpret_cast<init_fn>(GetProcAddress(dll, "t6n_init"));
			const auto size_f = reinterpret_cast<size_fn>(GetProcAddress(dll, "t6n_context_size"));
			g_run = reinterpret_cast<run_fn>(GetProcAddress(dll, "t6n_run"));

			if (!init_f || !size_f || !g_run)
			{
				rb_native_log.error("Native code %s: missing exports", path);
				return false;
			}

			if (size_f() != sizeof(ppu_context))
			{
				rb_native_log.error("Native code %s: context size %u, expected %u", path, size_f(), sizeof(ppu_context));
				return false;
			}

			static const host_api api{vm::g_base_addr, host_call_guest, host_syscall, host_timebase};
			const int n = init_f(&api);

			// RB_NATIVE_EXCLUDE_FN=<addr>,...: indirect calls to these functions run in RPCS3 (subtree included)
			if (const char* ex = std::getenv("RB_NATIVE_EXCLUDE_FN"); ex && *ex)
			{
				std::vector<u32> list;

				for (const auto& v : fmt::split(ex, {","}))
				{
					list.push_back(static_cast<u32>(std::stoul(v, nullptr, 16)));
				}

				if (const auto excl = reinterpret_cast<void (*)(const u32*, unsigned)>(GetProcAddress(dll, "t6n_exclude")))
				{
					excl(list.data(), static_cast<unsigned>(list.size()));
					rb_native_log.notice("Native: %u functions left to RPCS3 (%s)", list.size(), ex);
				}
			}
			rb_native_log.success("Native code %s loaded: %d lifted functions", path, n);
			return n > 0;
#else
			return false;
#endif
		}();

		return ok;
	}

	// Runs the lifted function at addr on this PPU thread's registers; false when native code is off or has no such
	// function (the caller runs it in RPCS3)
	bool call(ppu_thread& ppu, u32 addr)
	{
		if (!init())
		{
			return false;
		}

		ppu_thread* const prev = g_ppu;
		g_ppu = &ppu;
		resume(ppu);
		ppu_context ctx;
		to_ctx(ppu, ctx);

		// A call: the callee returns to LR; the lifted function returns to its host caller
		const bool ran = g_run(addr, &ctx) != 0;

		// RB_NATIVE_DELAY_US=<us>: busy wait after every native task step (timing experiments)
		static const u64 delay_us = std::getenv("RB_NATIVE_DELAY_US") ? std::strtoull(std::getenv("RB_NATIVE_DELAY_US"), nullptr, 10) : 0;

		if (delay_us)
		{
			const auto until = std::chrono::steady_clock::now() + std::chrono::microseconds(delay_us);
			while (std::chrono::steady_clock::now() < until) {}
		}

		if (ran)
		{
			from_ctx(ctx, ppu);
		}

		g_ppu = prev;
		return ran;
	}
}
