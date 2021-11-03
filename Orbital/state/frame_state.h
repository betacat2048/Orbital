#pragma once

#include "drict_state.h"
#include "point_state.h"
#include "../absolute_phase/absolute_frame_phase.h"

namespace orbital::state {
	class frame: public state::point, public state::dirct, public absolute_phase::frame {
	public:
		//frame()
		//	: relatively_phase::point(), relatively_phase::dirct(), 
		//	absolute_phase::point(), absolute_phase::dirct(), 
		//	state::base_state(), 
		//	absolute_phase::frame(), state::point(), state::dirct() {}

	};
}