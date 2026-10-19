#include "stdafx.h"
#include "rb_checksum.h"
#include "rb_input.h"

#include "Emu/CPU/CPUThread.h"
#include "Emu/Cell/PPUThread.h"
#include "Emu/Memory/vm.h"
#include "Utilities/File.h"
#include "Utilities/StrUtil.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <vector>

LOG_CHANNEL(rb_log, "Rollback");

namespace rb
{
	namespace
	{
		constexpr char file_magic[4] = {'R', 'B', 'H', 'S'};
		constexpr char file_magic_dump[4] = {'R', 'B', 'D', 'M'};
		constexpr u32 file_version = 1;
		constexpr u32 page_size = 4096;

		// Hashed guest address ranges: main memory, user64k/user1m areas and thread stacks.
		// RSX local memory (0xC0000000) and raw SPU areas are left out on purpose.
		constexpr std::pair<u32, u32> hashed_ranges[] =
		{
			{0x00010000, 0xC0000000},
			{0xD0000000, 0xE0000000},
		};

		struct page_hash
		{
			u32 page; // guest address / 4096
			u32 pad;
			u64 hash;
		};

		struct sample_header
		{
			u64 game_frame;
			u64 rsx_frame;
			u32 page_count;
			u32 time_us; // time spent hashing
		};

		struct watch_t
		{
			std::string name;
			bool r13_relative;
			s64 offset;
		};

		fs::file g_out;
		u64 g_every = 60;
		u64 g_from = 0;
		std::once_flag g_init;

		fs::file g_watch_out;
		std::vector<watch_t> g_watches;

		std::vector<std::pair<u32, u32>> g_state_ranges;

		void init_state_ranges()
		{
			const char* list = std::getenv("RB_STATE_RANGES");

			for (const std::string& item : fmt::split(list ? list : "", {","}))
			{
				const auto parts = fmt::split(item, {"-"});

				if (parts.size() == 2)
				{
					const u32 begin = static_cast<u32>(std::strtoul(parts[0].c_str(), nullptr, 16)) & ~3u;
					const u32 end = static_cast<u32>(std::strtoul(parts[1].c_str(), nullptr, 16)) & ~3u;
					g_state_ranges.emplace_back(begin, end);
					rb_log.success("State hash range 0x%08x-0x%08x", begin, end);
				}
			}
		}

		// Hash of the configured game state ranges, heap pointers skipped
		u64 hash_state()
		{
			u64 h = 0xCBF29CE484222325;

			for (const auto& [begin, end] : g_state_ranges)
			{
				for (u32 addr = begin; addr < end; addr += 4)
				{
					if (!vm::check_addr<4>(addr, vm::page_readable))
					{
						continue;
					}

					const u32 value = *vm::get_super_ptr<u32>(addr);

					if (value - 0x30000000u < 0x08000000u)
					{
						continue;
					}

					h = (h ^ value ^ (u64{addr} << 32)) * 0x100000001B3;
				}
			}

			return h;
		}

		std::vector<u64> g_dump_frames;
		std::string g_dump_prefix;
		std::vector<std::pair<u32, u32>> g_dump_ranges; // RB_DUMP_RANGES=0xBEGIN-0xEND,...: only these pages (smaller files)

		void init_dump()
		{
			const char* frames = std::getenv("RB_DUMP_FRAMES");
			const char* prefix = std::getenv("RB_DUMP_PREFIX");

			if (!frames || !*frames || !prefix || !*prefix)
			{
				return;
			}

			for (const std::string& item : fmt::split(frames, {","}))
			{
				g_dump_frames.push_back(std::strtoull(item.c_str(), nullptr, 10));
			}

			g_dump_prefix = out_path(prefix);
			rb_log.success("Dumping memory at %u game frames to '%s_<frame>.bin'", g_dump_frames.size(), g_dump_prefix);

			if (const char* ranges = std::getenv("RB_DUMP_RANGES"); ranges && *ranges)
			{
				for (const std::string& item : fmt::split(ranges, {","}))
				{
					const auto parts = fmt::split(item, {"-"});

					if (parts.size() == 2)
					{
						g_dump_ranges.emplace_back(static_cast<u32>(std::strtoul(parts[0].c_str(), nullptr, 16)), static_cast<u32>(std::strtoul(parts[1].c_str(), nullptr, 16)));
						rb_log.success("Dump range 0x%08x-0x%08x", g_dump_ranges.back().first, g_dump_ranges.back().second);
					}
				}
			}
		}

		// File: "RBDM", u32 version 1, u64 game frame, then records of u32 page index + 4096 bytes
		void dump_memory(u64 game_frame)
		{
			// The same frame can be reached again after a state restore: number the passes
			std::string path = fmt::format("%s_%u.bin", g_dump_prefix, game_frame);
			for (u32 pass = 1; fs::is_file(path); pass++)
			{
				path = fmt::format("%s_%u_pass%u.bin", g_dump_prefix, game_frame, pass);
			}
			fs::file f(path, fs::rewrite);

			if (!f)
			{
				rb_log.error("Dump file '%s' cannot be created", path);
				return;
			}

			std::vector<u8> buf;
			buf.reserve(200u << 20);

			const auto append = [&](const void* data, usz size)
			{
				const u8* p = static_cast<const u8*>(data);
				buf.insert(buf.end(), p, p + size);
			};

			append(file_magic_dump, 4);
			append(&file_version, sizeof(file_version));
			append(&game_frame, sizeof(game_frame));

			for (const auto& [begin, end] : hashed_ranges)
			{
				for (u64 addr = begin; addr < end; addr += page_size)
				{
					if (!vm::check_addr(static_cast<u32>(addr), vm::page_readable))
					{
						continue;
					}

					if (!g_dump_ranges.empty() && std::none_of(g_dump_ranges.begin(), g_dump_ranges.end(), [&](const auto& r) { return addr >= r.first && addr < r.second; }))
					{
						continue;
					}

					const u32 page = static_cast<u32>(addr / page_size);
					append(&page, sizeof(page));
					append(vm::g_sudo_addr + addr, page_size);
				}
			}

			f.write(buf.data(), buf.size());
			rb_log.success("Dumped %u MiB of memory at game frame %u to '%s'", buf.size() >> 20, game_frame, path);
		}

		u64 env_u64(const char* name, u64 def)
		{
			const char* value = std::getenv(name);
			return value && *value ? std::strtoull(value, nullptr, 10) : def;
		}

		void init_watch()
		{
			const char* list = std::getenv("RB_WATCH");
			const char* path_env = std::getenv("RB_WATCH_OUT");

			if (!list || !*list || !path_env || !*path_env)
			{
				return;
			}

			const std::string path = out_path(path_env);

			for (const std::string& item : fmt::split(list, {","}))
			{
				watch_t w{item, false, 0};
				std::string_view s = item;

				if (s.starts_with("r13"))
				{
					w.r13_relative = true;
					s.remove_prefix(3);
				}

				const bool negative = s.starts_with('-');
				if (negative || s.starts_with('+'))
				{
					s.remove_prefix(1);
				}

				w.offset = static_cast<s64>(std::strtoull(std::string(s).c_str(), nullptr, 16));
				if (negative)
				{
					w.offset = -w.offset;
				}

				g_watches.push_back(std::move(w));
			}

			if (!g_watch_out.open(path, fs::rewrite))
			{
				rb_log.error("Watch file '%s' cannot be created", path);
				g_watches.clear();
				return;
			}

			std::string header = "game_frame,rsx_frame";
			for (const watch_t& w : g_watches)
			{
				header += "," + w.name;
			}
			if (!g_state_ranges.empty())
			{
				header += ",state_hash";
			}
			header += "\n";
			g_watch_out.write(header.data(), header.size());
			rb_log.success("Watching %u values, writing to '%s'", g_watches.size(), path);
		}

		void write_watch(u64 game_frame)
		{
			const auto ppu = get_current_cpu_thread<ppu_thread>();
			std::string line = fmt::format("%u,%u", game_frame, frame());

			for (const watch_t& w : g_watches)
			{
				const u64 base = w.r13_relative ? (ppu ? ppu->gpr[13] : 0) : 0;
				const u32 addr = static_cast<u32>(base + w.offset);

				if (vm::check_addr<4>(addr, vm::page_readable))
				{
					fmt::append(line, ",0x%08x", +*vm::get_super_ptr<u32>(addr));
				}
				else
				{
					line += ",-";
				}
			}

			if (!g_state_ranges.empty())
			{
				fmt::append(line, ",%016x", hash_state());
			}

			line += "\n";
			g_watch_out.write(line.data(), line.size());
		}

		void init()
		{
			init_state_ranges();
			init_watch();
			init_dump();

			const char* path_env = std::getenv("RB_HASH");

			if (!path_env || !*path_env)
			{
				return;
			}

			const std::string path = out_path(path_env);

			g_every = std::max<u64>(env_u64("RB_HASH_EVERY", 60), 1);
			g_from = env_u64("RB_HASH_FROM", 0);

			if (!g_out.open(path, fs::rewrite))
			{
				rb_log.error("Checksum file '%s' cannot be created", path);
				return;
			}

			g_out.write(file_magic, sizeof(file_magic));
			g_out.write(&file_version, sizeof(file_version));
			rb_log.success("Writing memory checksums to '%s' (every %u game frames from frame %u)", path, g_every, g_from);
		}

		// Four independent multiply-xor lanes over 64-bit words, folded at the end
		u64 hash_page(const u8* data)
		{
			const u64* words = reinterpret_cast<const u64*>(data);
			u64 h0 = 0x9E3779B97F4A7C15, h1 = 0xC2B2AE3D27D4EB4F, h2 = 0x165667B19E3779F9, h3 = 0x27D4EB2F165667C5;

			for (u32 i = 0; i < page_size / 8; i += 4)
			{
				h0 = (h0 ^ words[i + 0]) * 0x100000001B3;
				h1 = (h1 ^ words[i + 1]) * 0x100000001B3;
				h2 = (h2 ^ words[i + 2]) * 0x100000001B3;
				h3 = (h3 ^ words[i + 3]) * 0x100000001B3;
			}

			u64 h = h0;
			h = (h ^ (h1 >> 7 | h1 << 57)) * 0x9E3779B97F4A7C15;
			h = (h ^ (h2 >> 19 | h2 << 45)) * 0x9E3779B97F4A7C15;
			h = (h ^ (h3 >> 31 | h3 << 33)) * 0x9E3779B97F4A7C15;
			return h ^ (h >> 29);
		}

		void hash_memory(std::vector<page_hash>& out)
		{
			for (const auto& [begin, end] : hashed_ranges)
			{
				for (u64 addr = begin; addr < end; addr += page_size)
				{
					if (!vm::check_addr(static_cast<u32>(addr), vm::page_readable))
					{
						continue;
					}

					out.push_back({static_cast<u32>(addr / page_size), 0, hash_page(vm::g_sudo_addr + addr)});
				}
			}
		}
	}

	void checksum_on_frame(u64 game_frame)
	{
		std::call_once(g_init, init);

		if (!g_watches.empty())
		{
			write_watch(game_frame);
		}

		if (std::find(g_dump_frames.begin(), g_dump_frames.end(), game_frame) != g_dump_frames.end())
		{
			dump_memory(game_frame);
		}

		if (!g_out || game_frame < g_from || (game_frame - g_from) % g_every)
		{
			return;
		}

		std::vector<page_hash> pages;
		pages.reserve(0x10000);

		const auto start = std::chrono::steady_clock::now();

		// No cpu_thread::suspend_all here: it deadlocked the game twice (game frames 1020 and 6600).
		// Other threads keep running while hashing, so pages they write show up as noise.
		hash_memory(pages);

		const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();

		const sample_header header{game_frame, frame(), ::size32(pages), static_cast<u32>(elapsed)};
		g_out.write(&header, sizeof(header));
		g_out.write(pages.data(), pages.size() * sizeof(page_hash));
	}

	void dump_memory_now(const std::string& prefix, u64 game_frame)
	{
		if (g_dump_prefix.empty())
		{
			g_dump_prefix = out_path(prefix);
		}

		dump_memory(game_frame);
		rb_log.success("Memory dumped at game frame %u (%s_%u.bin)", game_frame, g_dump_prefix, game_frame);
	}
}
