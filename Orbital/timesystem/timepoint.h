#pragma once
#include "../util.h"

namespace orbital::timesystem {
	class timepoint {
	public:
		static constexpr value_t dpc = 36525.; // days per century
		static constexpr value_t spd = 86400.; // seconds per day
		static constexpr value_t j2000 = 2451545.0; // Julian Date at J2000
		static constexpr value_t mjd = 2400000.5; // Modified Julian Date shift from Julian Date

		virtual value_t century() const = 0; // return the centuries from J2000 under Barycentric Dynamical Time
		virtual value_t days() const = 0; // return the days from J2000 under Barycentric Dynamical Time
		virtual value_t second() const = 0; // return the seconds from J2000 under Barycentric Dynamical Time
		virtual value_t JulianDate() const = 0;// return the JD dates under Barycentric Dynamical Time
		virtual value_t ModifiedJulianDate() const = 0;// return the MJD (dates from 1858/11/17 00:00) under Barycentric Dynamical Time
	};
}