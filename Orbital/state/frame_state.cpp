#include "frame_state.h"
#include "../objects/frame.h"
#include "../absolute_phase/absolute_frame_phase.h"

using namespace orbital;

orbital::state::frame::frame(const object::frame &parent, const timesystem::time_ptr &p_time, const absolute_phase::frame_ptr &p_data)
	:base_state(p_time), point(parent, p_time, p_data), dirct(parent, p_time, p_data), parent(parent), p_data(p_data) {}
