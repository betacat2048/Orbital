#pragma once
#include "../util.h"
#include "../timesystem/timepoint.h"
#include "../timesystem/TDB.h"


namespace orbital::state {
	// state of sth. where it reference to a certain trans_base framework and a certain time
	class base_state {
	protected:
		const timesystem::time_ptr p_time;

	public:
		base_state(base_state &&) = default;
		base_state(const base_state &) = default;
		virtual ~base_state() = default;

		base_state(const timesystem::time_ptr &p_time): p_time(p_time) {}

		timesystem::time_ptr get_timeptr() const { return p_time; }
		operator timesystem::time_ptr() const { return get_timeptr(); }

		bool has_same_time_with(const base_state &o) const {
			// if the time_ptr is same, then return true
			// else if both p_time and o.p_time aren't nullptr, return if their time_points are same
			return p_time == o.p_time || ( ( p_time && o.p_time ) && *p_time == *o.p_time );
		}
	};

}
