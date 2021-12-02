#pragma once
#include "../util.h"
#include "timepoint.h"

namespace orbital::timesystem {
	//Barycentric Dynamical Time (also konw as TDB)
	class BarycentricDynamicalTime: public timepoint {
	protected:
		const value_t seconds_from_J2000;
	public:
		BarycentricDynamicalTime(BarycentricDynamicalTime &&) = default;
		BarycentricDynamicalTime(const BarycentricDynamicalTime &) = default;
		BarycentricDynamicalTime(const value_t &seconds_from_J2000):seconds_from_J2000(seconds_from_J2000) {}

		// overrides
		value_t seconds() const { return seconds_from_J2000; }
		virtual BarycentricDynamicalTime toTDB() const { return *this; }

		// build factor
		static BarycentricDynamicalTime fromJD(const value_t &Julian_Date) { return BarycentricDynamicalTime(Julian_Date - JD_of_J2000_epoch); }
		static BarycentricDynamicalTime fromMJD(const value_t &Modified_Julian_Date) { return BarycentricDynamicalTime(Modified_Julian_Date + ( MJD_shift - JD_of_J2000_epoch )); }

		// shift by seconds
		BarycentricDynamicalTime operator+(const value_t &o) const { return BarycentricDynamicalTime(seconds_from_J2000 + o); }
		BarycentricDynamicalTime operator-(const value_t &o) const { return BarycentricDynamicalTime(seconds_from_J2000 - o); }

		// difference in seconds
		value_t operator-(const BarycentricDynamicalTime &o) const { return seconds_from_J2000 - o.seconds_from_J2000; }

		static time_ptr make_node(const BarycentricDynamicalTime &o) { return std::make_shared<BarycentricDynamicalTime>(o); }
		static time_ptr make_node(BarycentricDynamicalTime &&o) { return std::make_shared<BarycentricDynamicalTime>(std::move(o)); }
		static time_ptr make_node(const value_t &seconds_from_J2000) { return std::make_shared<BarycentricDynamicalTime>(seconds_from_J2000); }
	};
	using TDB = BarycentricDynamicalTime;
}