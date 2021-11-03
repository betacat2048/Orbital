#pragma once

#include "base_state.h"
#include "../absolute_phase/absolute_dirct_phase.h"

namespace orbital::state {
	class dirct: public virtual base_state, public virtual absolute_phase::dirct {
	public:
		//dirct():relatively_phase::dirct(), absolute_phase::dirct(), state::base_state() {}
	};
}