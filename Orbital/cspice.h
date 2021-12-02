#pragma once
#include <stack>
#include <chrono>
#include <string>
#include <exception>
#include <filesystem>

#include "objects/point.h"
#include "objects/dirct.h"
#include "objects/frame.h"
#include "timesystem/TDB.h"

#include <cspice/SpiceZdf.h>

namespace orbital {
	// the holder of cspice kernel ( automatic load cspice kernel when progeam start, then unload them when program end )
	class cspice_kernel_holder {
	private:
		std::stack<std::filesystem::path> kernels;

		cspice_kernel_holder(const std::filesystem::path &kernels_folder);
		~cspice_kernel_holder();
		static cspice_kernel_holder holder;
	};

	class cspice_error :public std::runtime_error {
	public:
		cspice_error(std::string msg) :runtime_error("cspice error: " + msg) {}
	};

	namespace timesystem {
		class InternationalAtomicTime : public timepoint {
		protected:
			const value_t seconds_from_J2000;
		public:
			static constexpr value_t JD_of_UNIX_epoch = 2440587.5000000; // Julian Date at UNIX epoch (1970-01-01 00:00:00)

			InternationalAtomicTime(InternationalAtomicTime &&) = default;
			InternationalAtomicTime(const InternationalAtomicTime &) = default;
			InternationalAtomicTime(const value_t &seconds_from_J2000) :seconds_from_J2000(seconds_from_J2000) {}

			InternationalAtomicTime(const BarycentricDynamicalTime &);
			InternationalAtomicTime(const std::chrono::system_clock::time_point &tp) :InternationalAtomicTime(std::chrono::duration_cast<std::chrono::seconds>( tp.time_since_epoch() ).count() + ( ( JD_of_UNIX_epoch - JD_of_J2000_epoch ) * SecondsPerDay )) {}

			// overrides
			value_t seconds() const { return seconds_from_J2000; }
			virtual BarycentricDynamicalTime toTDB() const;

			// shift by seconds
			InternationalAtomicTime operator+(const value_t &o) const { return InternationalAtomicTime(seconds_from_J2000 + o); }
			InternationalAtomicTime operator-(const value_t &o) const { return InternationalAtomicTime(seconds_from_J2000 - o); }

			// difference in seconds
			value_t operator-(const InternationalAtomicTime &o) const { return seconds_from_J2000 - o.seconds_from_J2000; }

			static time_ptr make_node(const InternationalAtomicTime &o) { return std::make_shared<InternationalAtomicTime>(o); }
			static time_ptr make_node(const BarycentricDynamicalTime &o) { return std::make_shared<InternationalAtomicTime>(o); }
			static time_ptr make_node(InternationalAtomicTime &&o) { return std::make_shared<InternationalAtomicTime>(std::move(o)); }
			static time_ptr make_node(const value_t &seconds_from_J2000) { return std::make_shared<InternationalAtomicTime>(seconds_from_J2000); }
			static time_ptr make_node(const std::chrono::system_clock::time_point &tp) { return std::make_shared<InternationalAtomicTime>(tp); }
		};
		using TAI = InternationalAtomicTime;

		class TerrestrialDynamicalTime : public timepoint {
		protected:
			const value_t seconds_from_J2000;
		public:
			TerrestrialDynamicalTime(TerrestrialDynamicalTime &&) = default;
			TerrestrialDynamicalTime(const TerrestrialDynamicalTime &) = default;
			TerrestrialDynamicalTime(const value_t &seconds_from_J2000) :seconds_from_J2000(seconds_from_J2000) {}

			TerrestrialDynamicalTime(const BarycentricDynamicalTime &);

			// overrides
			value_t seconds() const { return seconds_from_J2000; }
			virtual BarycentricDynamicalTime toTDB() const;

			// shift by seconds
			TerrestrialDynamicalTime operator+(const value_t &o) const { return TerrestrialDynamicalTime(seconds_from_J2000 + o); }
			TerrestrialDynamicalTime operator-(const value_t &o) const { return TerrestrialDynamicalTime(seconds_from_J2000 - o); }

			// difference in seconds
			value_t operator-(const TerrestrialDynamicalTime &o) const { return seconds_from_J2000 - o.seconds_from_J2000; }

			static time_ptr make_node(const TerrestrialDynamicalTime &o) { return std::make_shared<TerrestrialDynamicalTime>(o); }
			static time_ptr make_node(const BarycentricDynamicalTime &o) { return std::make_shared<TerrestrialDynamicalTime>(o); }
			static time_ptr make_node(TerrestrialDynamicalTime &&o) { return std::make_shared<TerrestrialDynamicalTime>(std::move(o)); }
			static time_ptr make_node(const value_t &seconds_from_J2000) { return std::make_shared<TerrestrialDynamicalTime>(seconds_from_J2000); }
		};
		using TDT = TerrestrialDynamicalTime;

		class CoordinatedUniversalTime : public timepoint {
		protected:
			value_t seconds_from_J2000;
		public:
			CoordinatedUniversalTime(CoordinatedUniversalTime &&) = default;
			CoordinatedUniversalTime(const CoordinatedUniversalTime &) = default;
			CoordinatedUniversalTime(const value_t &seconds_from_J2000) :seconds_from_J2000(seconds_from_J2000) {}

			CoordinatedUniversalTime(const BarycentricDynamicalTime &);

			// overrides
			value_t seconds() const { return seconds_from_J2000; }
			virtual BarycentricDynamicalTime toTDB() const;

			// shift by seconds
			CoordinatedUniversalTime operator+(const value_t &o) const { return CoordinatedUniversalTime(seconds_from_J2000 + o); }
			CoordinatedUniversalTime operator-(const value_t &o) const { return CoordinatedUniversalTime(seconds_from_J2000 - o); }

			// difference in seconds
			value_t operator-(const CoordinatedUniversalTime &o) const { return seconds_from_J2000 - o.seconds_from_J2000; }

			static time_ptr make_node(const CoordinatedUniversalTime &o) { return std::make_shared<CoordinatedUniversalTime>(o); }
			static time_ptr make_node(const BarycentricDynamicalTime &o) { return std::make_shared<CoordinatedUniversalTime>(o); }
			static time_ptr make_node(CoordinatedUniversalTime &&o) { return std::make_shared<CoordinatedUniversalTime>(std::move(o)); }
			static time_ptr make_node(const value_t &seconds_from_J2000) { return std::make_shared<CoordinatedUniversalTime>(seconds_from_J2000); }
		};
		using UTC = CoordinatedUniversalTime;
	}

	namespace object {
		class cspice_point : public object::point {
		protected:
			SpiceInt NAIF_ID;
			/*****************************************************************************************
			*	NAIF ID     NAME
			*	_______    ________________________
			*	  0        SOLAR SYSTEM BARYCENTER
			*	  1        MERCURY BARYCENTER
			*	  2        VENUS BARYCENTER
			*	  3        EARTH-MOON BARYCENTER
			*	  4        MARS BARYCENTER
			*	  5        JUPITER BARYCENTER
			*	  6        SATURN BARYCENTER
			*	  7        URANUS BARYCENTER
			*	  8        NEPTUNE BARYCENTER
			*	  9        PLUTO BARYCENTER
			*	  10       SUN
			* 
			*	 P99       the main planet of the P-th planet's barycenter
			*	 PNN       P in [1, 9]
			*              NN in [01, 98]
			*              the NN-th satellites of the planet P-th planet
			*****************************************************************************************/
		public:
			cspice_point(cspice_point &&) = default;
			cspice_point(const cspice_point &) = default;
			virtual ~cspice_point() = default;

			cspice_point(const std::string &NAIF_name);
			cspice_point(const SpiceInt &NAIF_ID);

			virtual bool inertial() const { 
				return NAIF_ID == 0; // *this is inertial only if *this is SOLAR SYSTEM BARYCENTER (NAIF code = 0)
			}
			virtual state::point point_state_at(const timesystem::time_ptr &) const;

			SpiceInt get_id() const { return NAIF_ID; }

			static cspice_point SUN_barycenter;
			static cspice_point MERCURY_barycenter;
			static cspice_point VENUS_barycenter;
			static cspice_point EARTHMOON_barycenter;
			static cspice_point MARS_barycenter;
			static cspice_point JUPITER_barycenter;
			static cspice_point SATURN_barycenter;
			static cspice_point URANUS_barycenter;
			static cspice_point NEPTUNE_barycenter;
			static cspice_point PLUTO_barycenter;

			static cspice_point SUN_center_of_mass;
			static cspice_point MERCURY_center_of_mass;
			static cspice_point VENUS_center_of_mass;
			static cspice_point EARTH_center_of_mass;
			static cspice_point MOON_center_of_mass;
			static cspice_point MARS_center_of_mass;
			static cspice_point JUPITER_center_of_mass;
			static cspice_point SATURN_center_of_mass;
			static cspice_point URANUS_center_of_mass;
			static cspice_point NEPTUNE_center_of_mass;
			static cspice_point PLUTO_center_of_mass;

		};
	}
}