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

		point(const std::string &name) :point_name(name) { }

		virtual bool inertial() const = 0;

		std::string get_point_name() const { return point_name; }

		virtual state::point point_state_at(const timesystem::time_ptr &) const = 0;
		state::point operator()(const timesystem::time_ptr &pt) const { return point_state_at(pt); }
	};


	class fixed_point : public object::point {
	protected:
		vec3 pos;
	public:
		fixed_point(fixed_point &&) = default;
		fixed_point(const fixed_point &) = default;
		virtual ~fixed_point() = default;

		fixed_point(const std::string &name, const vec3 &pos) :point(name), pos(pos) { }

		bool inertial() const { return true; }

		virtual state::point point_state_at(const timesystem::time_ptr &t) const { return { *this, t, absolute_phase::point::make_node({pos}) }; }
	};

	class inertial_point : public object::fixed_point {
		timesystem::time_ptr t0;
		vec3 vel;
	public:
		inertial_point(inertial_point &&) = default;
		inertial_point(const inertial_point &) = default;
		virtual ~inertial_point() = default;

		inertial_point(const std::string &name, const timesystem::time_ptr &t, const vec3 &pos, const vec3 &vel) :fixed_point(name, pos), t0(t), vel(vel) { }

		bool inertial() const { return true; }

		virtual state::point point_state_at(const timesystem::time_ptr &t) const {
			return { *this, t, absolute_phase::point::make_node(relatively_phase::point(pos + ( t->seconds() - t0->seconds() ) * vel, vel)) };
		}
	};
}