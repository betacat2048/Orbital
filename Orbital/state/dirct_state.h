#pragma once
#include "../util.h"
#include "base_state.h"

namespace orbital::state {
	class dirct: public virtual base_state {
		friend class object::dirct;
	protected:
		const object::dirct &parent;
		const absolute_phase::dirct_ptr p_data;

	public:
		dirct(dirct &&) = default;
		dirct(const dirct &) = default;
		virtual ~dirct() = default;

		dirct(const object::dirct &parent, const timesystem::time_ptr &p_time, const absolute_phase::dirct_ptr &p_data)
			:base_state(p_time), parent(parent), p_data(p_data) {}

		const object::dirct &get_parent() const { return parent; }
		absolute_phase::dirct_ptr get_dirctptr() const { return p_data; }

		operator absolute_phase::dirct_ptr() const { return get_dirctptr(); }
	};
}