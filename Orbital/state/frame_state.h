#pragma once

#include "../util.h"
#include "base_state.h"
#include "dirct_state.h"
#include "point_state.h"

namespace orbital::state {
	class frame: public state::point, state::dirct {
		friend class object::frame;
	protected:
		const object::frame &parent;
		const absolute_phase::frame_ptr p_data;

	public:
		frame(frame &&) = default;
		frame(const frame &) = default;
		virtual ~frame() = default;

		frame(const object::frame &parent, const timesystem::time_ptr &p_time, const absolute_phase::frame_ptr &p_data);

		const object::frame &get_parent() const { return parent; }
		absolute_phase::frame_ptr get_frameptr() const { return p_data; }

		operator absolute_phase::frame_ptr() const { return get_frameptr(); }
	};
}