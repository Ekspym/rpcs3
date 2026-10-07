#include "stdafx.h"
#include "rb_input.h"

#include "Emu/Cell/PPUThread.h"
#include "Emu/Memory/vm.h"
#include "Emu/Memory/vm_reservation.h"
#include "Emu/Memory/vm_ptr.h"

#include <cstdlib>
#include <vector>
#include <chrono>
#include <cstring>

#ifdef _WIN32
#include <Windows.h>
#endif

LOG_CHANNEL(rb_native_log, "RbNative");

extern void ppu_execute_syscall(ppu_thread& ppu, u64 code);
extern u32 ppu_lwarx(ppu_thread& ppu, u32 addr);
extern u64 ppu_ldarx(ppu_thread& ppu, u32 addr);
extern bool ppu_stwcx(ppu_thread& ppu, u32 addr, u32 reg_value);
extern bool ppu_stdcx(ppu_thread& ppu, u32 addr, u64 reg_value);

// Hybrid (RB_NATIVE=<t6_native.dll>): the game's PPU code lifted to C++ by ps3recomp from the user's own EBOOT
// (recomp_t6/hybrid) runs the frame's task steps natively; RPCS3 keeps everything else (SPU, RSX, sound, lv2,
// firmware libraries) and runs whatever the lifted code calls that has no lifted function.
namespace rb::native
{
	void pad_record();
	void pad_replay();
	void pad_off();

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
		u64 (*load_reserve)(u32 addr, u32 size);           // lwarx/ldarx: RPCS3's ppu_lwarx/ppu_ldarx
		int (*store_cond)(u32 addr, u64 value, u32 size);  // stwcx/stdcx: RPCS3's ppu_stwcx/ppu_stdcx
		void (*check_state)();                             // polled on the lifted code's backward branches
	};

	using init_fn = int (*)(const host_api*);
	using run_fn = int (*)(u32 addr, ppu_context* ctx);
	using size_fn = unsigned (*)();

	run_fn g_run = nullptr;
	void (*g_bind)(volatile u32* state) = nullptr;
	std::vector<u32> g_hooked;

	void set_hooked(const std::vector<u32>& addrs)
	{
		g_hooked = addrs;
	}
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

	// lwarx/ldarx and stwcx./stdcx. of the lifted code go through the same functions RPCS3's own compiled PPU code
	// calls (reservation address and time in the ppu_thread, locking, notification of waiting SPUs). A host CAS did
	// not move the reservation time nor wake the SPURS kernel waiting for a change of the line (jobs queued by the
	// native main thread were never picked up: the loading screen waited forever).
	u64 host_load_reserve(u32 addr, u32 size)
	{
		return size == 4 ? ppu_lwarx(*g_ppu, addr) : ppu_ldarx(*g_ppu, addr);
	}

	int host_store_cond(u32 addr, u64 value, u32 size)
	{
		return (size == 4 ? ppu_stwcx(*g_ppu, addr, static_cast<u32>(value)) : ppu_stdcx(*g_ppu, addr, value)) ? 1 : 0;
	}

	// A loop in the lifted code found the thread's state word set (suspend_all, pause, memory lock request): serve it
	// like RPCS3's own PPU code does at every block
	void host_check_state()
	{
		g_ppu->check_state();
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
			g_bind = reinterpret_cast<void (*)(volatile u32*)>(GetProcAddress(dll, "t6n_bind_thread"));

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

			static const host_api api{vm::g_base_addr, host_call_guest, host_syscall, host_timebase, host_load_reserve, host_store_cond, host_check_state};
			const int n = init_f(&api);

			// Functions with hooks or patches (HLE replacements of the rollback modules, game patches) and
			// RB_NATIVE_EXCLUDE_FN=<addr>,...: calls to them from the lifted code run in RPCS3 (subtree included)
			{
				std::vector<u32> list = g_hooked;
				const char* ex = std::getenv("RB_NATIVE_EXCLUDE_FN");

				for (const auto& v : fmt::split(ex ? ex : "", {","}))
				{
					list.push_back(static_cast<u32>(std::stoul(v, nullptr, 16)));
				}

				if (const auto excl = reinterpret_cast<void (*)(const u32*, unsigned)>(GetProcAddress(dll, "t6n_exclude")))
				{
					excl(list.data(), static_cast<unsigned>(list.size()));
					rb_native_log.notice("Native: %u functions left to RPCS3 (%u hooked)", list.size(), g_hooked.size());
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


	// RB_NATIVE_DIFF_AFTER_S=<s>: from that many seconds after the native code was loaded, every frame runs twice from
	// the same state - lifted code, then RPCS3 - and the game's data and heaps are compared (through the unprotected
	// view, so RSX page protection is not triggered); differing words are logged, RPCS3's result is kept.
	struct diff_state
	{
		bool on = false;
		std::chrono::steady_clock::time_point start{};
		u64 after_s = 0;
		u32 frames = 0;
		u32 reported = 0;
	};

	diff_state g_diff;

	u32 bswap32(u32 v)
	{
		return (v >> 24) | ((v >> 8) & 0xff00u) | ((v << 8) & 0xff0000u) | (v << 24);
	}

	// Game data and heaps; not the RSX command buffers / draw request buffers (0x30000000-0x36000000) and not the draw
	// manager (0x00D2C000-0x00D2E000): restoring those desynchronised the RSX
	constexpr u32 diff_ranges[][2] = {{0x00900000u, 0x00D2C000u}, {0x00D2E000u, 0x00D80000u}, {0x10000000u, 0x14000000u}, {0x36000000u, 0x37600000u}};

	void diff_copy(std::vector<u8>& out)
	{
		out.clear();

		for (const auto& r : diff_ranges)
		{
			for (u32 page = r[0]; page < r[1]; page += 4096)
			{
				const usz at = out.size();
				out.resize(at + 4096);

				if (vm::check_addr(page))
				{
					std::memcpy(out.data() + at, vm::g_sudo_addr + page, 4096);
				}
				else
				{
					std::memset(out.data() + at, 0, 4096);
				}
			}
		}
	}

	void diff_restore(const std::vector<u8>& in)
	{
		usz at = 0;

		for (const auto& r : diff_ranges)
		{
			for (u32 page = r[0]; page < r[1]; page += 4096, at += 4096)
			{
				if (vm::check_addr(page, vm::page_writable))
				{
					std::memcpy(vm::g_sudo_addr + page, in.data() + at, 4096);
				}
			}
		}
	}

	u32 diff_addr(usz offset)
	{
		for (const auto& r : diff_ranges)
		{
			const usz size = r[1] - r[0];

			if (offset < size)
			{
				return r[0] + static_cast<u32>(offset);
			}

			offset -= size;
		}

		return 0;
	}

	bool diff_active()
	{
		static const bool init = []()
		{
			if (const char* v = std::getenv("RB_NATIVE_DIFF_AFTER_S"); v && *v)
			{
				g_diff.on = true;
				g_diff.after_s = std::strtoull(v, nullptr, 10);
				g_diff.start = std::chrono::steady_clock::now();
			}

			return true;
		}();

		(void)init;
		return g_diff.on && std::chrono::steady_clock::now() - g_diff.start > std::chrono::seconds(g_diff.after_s);
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

		if (g_bind)
		{
			g_bind(reinterpret_cast<volatile u32*>(&ppu.state));
		}

		resume(ppu);
		ppu_context ctx;
		to_ctx(ppu, ctx);

		if (diff_active() && g_diff.reported < 20)
		{
			if (g_diff.frames == 0)
			{
				rb_native_log.success("Diff: native vs RPCS3 frame comparison started");
			}

			static std::vector<u8> before, native_after, rpcs3_after;
			diff_copy(before);
			const ppu_context regs_in = ctx;

			pad_record();

			if (!g_run(addr, &ctx))
			{
				pad_off();
				g_ppu = prev;
				return false;
			}

			diff_copy(native_after);
			const ppu_context regs_native = ctx;

			// Same frame in RPCS3 from the same state
			diff_restore(before);
			from_ctx(regs_in, ppu);
			pad_replay();
			ppu.fast_call(addr, 0x0096A8B0);
			resume(ppu);
			pad_off();
			diff_copy(rpcs3_after);
			g_diff.frames++;

			// Words both runs wrote differently (other threads' writes during either run show up too: noise)
			u32 shown = 0;

			for (usz i = 0; i + 4 <= rpcs3_after.size(); i += 4)
			{
				if (std::memcmp(native_after.data() + i, rpcs3_after.data() + i, 4) != 0 &&
					(std::memcmp(native_after.data() + i, before.data() + i, 4) != 0 || std::memcmp(rpcs3_after.data() + i, before.data() + i, 4) != 0))
				{
					if (shown++ < 40)
					{
						u32 b, n, r;
						std::memcpy(&b, before.data() + i, 4);
						std::memcpy(&n, native_after.data() + i, 4);
						std::memcpy(&r, rpcs3_after.data() + i, 4);
						rb_native_log.warning("Diff frame %u: 0x%08x before %08x native %08x rpcs3 %08x", g_diff.frames, diff_addr(i),
							bswap32(b), bswap32(n), bswap32(r));
					}
				}
			}

			if (shown)
			{
				g_diff.reported++;
				rb_native_log.warning("Diff frame %u: %u differing words (r3 native %x rpcs3 %x)", g_diff.frames, shown, regs_native.gpr[3], ppu.gpr[3]);
			}
			else if (g_diff.frames % 60 == 0)
			{
				rb_native_log.notice("Diff: %u frames identical so far", g_diff.frames);
			}

			g_ppu = prev;
			return true;
		}

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
