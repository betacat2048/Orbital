#pragma once
#include "dirct.h"
#include "point.h"
#include "../state/frame_state.h"

namespace orbital::object {

	// interface for framework
	class frame :public object::point, public object::dirct {
	protected:

	public:
		frame(frame &&) = default;
		frame(const frame &) = default;
		frame(const std::string &name) :point(name), dirct(name) { }

		// bind up point and dirct
		frame(const object::point &p, const object::dirct &d) :object::point(p), object::dirct(d) { }
		// bind up point and dirct
		frame(object::point &&p, object::dirct &&d) :object::point(std::move(p)), object::dirct(std::move(d)) { }

		virtual bool inertial() const { return point::inertial() && dirct::inertial(); }

		virtual state::frame frame_state_at(const timesystem::time_ptr &) const = 0;
		state::frame operator()(const timesystem::time_ptr &pt) const { return frame_state_at(pt); }
		virtual state::point point_state_at(const timesystem::time_ptr &pt) const { return frame_state_at(pt); }
		virtual state::dirct dirct_state_at(const timesystem::time_ptr &pt) const { return frame_state_at(pt); }
	};
}