#pragma once
#include "../util.h"
#include "../state/point_state.h"
#include "../timesystem/timepoint.h"
#include "../absolute_phase/absolute_point_phase.h"

namespace orbital::object {
	// interface for point
	class point {
	protected:
		std::string point_name;

	public:
		point(point &&) = default;
		point(const point &) = default;
		virtual ~point() = default;
		
		point(const std::string &name):point_name(name) { }

		virtual bool inertial() const = 0;

		std::string get_point_name() const { return point_name; }

		virtual state::point point_state_at(const timesystem::time_ptr &) const = 0;
		state::point operator()(const timesystem::time_ptr &pt) const { return point_state_at(pt); }
	};

	class inertial_point: public object::point {
		absolute_phase::point_diff inital_point;
		timesystem::time_ptr inital_time;
	public:
		inertial_point(inertial_point &&) = default;
		inertial_point(const inertial_point &) = default;
		virtual ~inertial_point() = default;

		inertial_point(const std::string &name, const timesystem::time_ptr &t, const absolute_phase::point_ptr &p):point(name), inital_time(t) {
			inital_point = p->different_from(absolute_phase::point::root()); // get the differece from the S.S.B. (this would reduce the dependency and ensure the point is intertial)
			inital_point.acceleration = vec3::Zero(); //set the acceleration to zero
		}

		bool inertial() const { return true; }

		virtual state::point point_state_at(const timesystem::time_ptr &t) const {
			return state::point(*this, t, absolute_phase::point::make_node(
				inital_point + relatively_phase::point(vec3(inital_point.velocity * (t->second() - inital_time->second()))) // calculate new position base on velocity
				));
		}
	};
}