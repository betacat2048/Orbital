#pragma once
#include "../util.h"
#include "../state/dirct_state.h"

namespace orbital::object {

	// interface for dirct
	class dirct {
	protected:
		std::string dirct_name;

	public:
		dirct(dirct &&) = default;
		dirct(const dirct &) = default;
		virtual ~dirct() = default;

		dirct(const std::string &name):dirct_name(name) {}

		virtual bool inertial() const = 0;
		
		std::string get_dirct_name() const { return dirct_name; }

		virtual state::dirct dirct_state_at(const timesystem::time_ptr &) const = 0;
		state::dirct operator()(const timesystem::time_ptr &pt) const { return dirct_state_at(pt); }
	};
}