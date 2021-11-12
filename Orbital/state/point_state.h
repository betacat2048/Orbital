#pragma once
#include "../util.h"
#include "base_state.h"

namespace orbital::state {
	class point: public virtual base_state {
		friend class object::point;
	protected:
		const object::point &parent;
		const absolute_phase::point_ptr p_data;

	public:
		point(point &&) = default;
		point(const point &) = default;
		virtual ~point() = default;

		point(const object::point &parent, const timesystem::time_ptr &p_time, const absolute_phase::point_ptr &p_data)
			:base_state(p_time), parent(parent), p_data(p_data) {}

		const object::point &get_parent() const { return parent; }
		absolute_phase::point_ptr get_pointptr() const { return p_data; }

		operator absolute_phase::point_ptr() const { return get_pointptr(); }
	};
}