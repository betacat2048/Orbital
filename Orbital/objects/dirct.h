#pragma once
#include "util.h"
#include "phase.h"
//#include "state.h"
#include "timesystem.h"

namespace orbital {

	// interface for dirct
	class dirct {
	protected:
		std::string name;
	public:

		virtual bool inertial() const = 0; // return if this dirction is inertial (which means its angular velocity and the angular acceleration are all ZERO)
		virtual dirct_state operator()(const std::shared_ptr<frame_state> &) const = 0;
	};


	/*
	// state of a certain orientation at a certain time
	class dirct_state: public state {
		friend class dirct;
	protected:
		const dirct &parent;
		quaternion rotation;
		vec3 omega, ang_acc;
		mat99 trans_mat;
	public:
		dirct_state(dirct_state &&) = default;
		dirct_state(const dirct_state &) = default;

		dirct_state reduce_under(const std::shared_ptr<frame_state> &frame_s);
	};
	*/
}