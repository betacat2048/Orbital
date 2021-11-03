#pragma once

#include "base_state.h"
#include "../absolute_phase/absolute_point_phase.h"

namespace orbital::state {
	class point: public virtual base_state, public virtual absolute_phase::point {
	public:
		//point():relatively_phase::point(), absolute_phase::point(), state::base_state() {}
	};
}