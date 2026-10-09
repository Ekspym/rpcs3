#pragma once
#include "rsx_replay.h"

#include <map>

namespace rsx
{
	class thread;
	namespace capture
	{
		void capture_draw_memory(thread* rsx);
		void capture_image_in(thread* rsx, frame_capture_data::replay_command& replay_command);
		void capture_buffer_notify(thread* rsx, frame_capture_data::replay_command& replay_command);
		void capture_display_tile_state(thread* rsx, frame_capture_data::replay_command& replay_command);

		// Rollback diagnostics (RB_RSX_READS=1): main memory pages the RSX reads in a draw (fragment program, textures,
		// vertex and index data, the command buffer around GET), collected for the rollback module
		bool rb_reads_enabled();
		void rb_note_draw_reads(thread* rsx);
		std::map<u32, u32> rb_take_read_pages();
	}
}
