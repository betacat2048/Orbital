#pragma once
#include "../util.h"
#include "../timesystem/timepoint.h"
#include "../timesystem/TDB.h"


namespace orbital::state {
	// state of sth. where it reference to a certain trans_base framework and a certain time
	class base_state {
	protected:
		const std::shared_ptr<timesystem::BarycentricDynamicalTime> p_time;
		const std::shared_ptr<state::frame> p_base_frame = nullptr;

	public:
		base_state(base_state &&) = default;
		base_state(const base_state &) = default;
		virtual ~base_state() = default;

		//create a state refence to a certain frame state
		base_state(const std::shared_ptr<state::frame> &p);
		
		//create a state refence to a certain time point (the trans_base frame would be nullptr)
		base_state(const std::shared_ptr<timesystem::BarycentricDynamicalTime> &);

		const state::frame &get_baseframe() const { return *p_base_frame; }
		std::shared_ptr<state::frame> get_frame_p() const { return p_base_frame; }

		const timesystem::BarycentricDynamicalTime &get_time() const { return *p_time; }
		std::shared_ptr<timesystem::BarycentricDynamicalTime> get_time_p() const { return p_time; }
	};

}
