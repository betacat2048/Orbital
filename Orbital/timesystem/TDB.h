#pragma once
#include "../util.h"
#include "timepoint.h"

namespace orbital::timesystem {
	//Barycentric Dynamical Time (also konw as TDB)
	class BarycentricDynamicalTime: public timepoint {
	protected:
		const value_t JD2000;
	public:
		BarycentricDynamicalTime(BarycentricDynamicalTime &&) = default;
		BarycentricDynamicalTime(const BarycentricDynamicalTime &) = default;
		BarycentricDynamicalTime(const value_t &days_from_J2000):JD2000(days_from_J2000) {}

		value_t century() const { return JD2000 / dpc; }
		value_t days() const { return JD2000; }
		value_t second() const { return spd * JD2000; }
		value_t JulianDate() const { return JD2000 + j2000; }
		value_t ModifiedJulianDate() const { return JulianDate() - mjd; }

		// date shift
		BarycentricDynamicalTime operator+(const value_t &o) const { return BarycentricDynamicalTime(JD2000 + o); }
		BarycentricDynamicalTime operator-(const value_t &o) const { return BarycentricDynamicalTime(JD2000 - o); }

		// date different
		value_t operator-(const BarycentricDynamicalTime &o) const { return JD2000 - o.JD2000; }
	};
}