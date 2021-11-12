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
	};

}
