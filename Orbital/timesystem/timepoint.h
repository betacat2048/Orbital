#pragma once
#include "../util.h"

namespace orbital::timesystem {
	class timepoint {
	public:
		virtual ~timepoint() = default;

		static constexpr value_t DaysPerCentury = 36525.; // days per century
		static constexpr value_t SecondsPerDay = 86400.; // seconds per day
		static constexpr value_t JD_of_J2000_epoch = 2451545.0; // Julian Date at J2000
		static constexpr value_t MJD_shift = 2400000.5; // Modified Julian Date shift from Julian Date

		virtual BarycentricDynamicalTime toTDB() const = 0; // cover into TDB

		virtual value_t days() const { return seconds() / SecondsPerDay; } // return the days from J2000
		virtual value_t seconds() const = 0; // return the seconds from J2000
		virtual value_t centurys() const { return days() / DaysPerCentury; } // return the centuries from J2000
		virtual value_t JulianDate() const { return days() + JD_of_J2000_epoch; }// return the JD dates
		virtual value_t ModifiedJulianDate() const { return JulianDate() - MJD_shift; }// return the MJD (dates from 1858/11/17 00:00)

		virtual bool operator==(const timepoint &o) const { return days() == o.days(); }
		virtual bool operator!=(const timepoint &o) const { return days() != o.days(); }
		virtual bool operator<=(const timepoint &o) const { return days() <= o.days(); }
		virtual bool operator>=(const timepoint &o) const { return days() >= o.days(); }
		virtual bool operator<(const timepoint &o) const { return days() < o.days(); }
		virtual bool operator>(const timepoint &o) const { return days() > o.days(); }
	};
}