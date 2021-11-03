#pragma once
#include "util.h"
//#include "state.h"
#include "direction.h"
#include "timesystem.h"

namespace orbital {

	// interface for point
	class point {
	protected:
		std::string name;

	public:
		point(point &&) = default;
		point(const point &) = default;
		point(const std::string &name):name(name) { }

		virtual bool inertial() const = 0;
		virtual point_state operator()(const std::shared_ptr<frame_state> &) const = 0;
	};

	/*
	// state of a certain point at a certain time
	class point_state: public state {
		friend class point;
	protected:
		const point &parent;
		const vec9 pos_vel_acc;

		point_state(const std::shared_ptr<frame_state> &frame_s, const point &parent, const vec9 &pos_vel_acc):state(frame_s), parent(parent), pos_vel_acc(pos_vel_acc) { }
	public:
		point_state(point_state &&) = default;
		point_state(const point_state &) = default;

		point_state under(const std::shared_ptr<frame_state> &frame_s);
	};
	*/
}