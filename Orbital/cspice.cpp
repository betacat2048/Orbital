#include "cspice.h"
#include <cspice/SpiceUsr.h>

#pragma comment(lib, "cspice.lib")
#pragma comment(lib, "csupport.lib")

using namespace std;
using namespace orbital;

// inital the holder
orbital::cspice_kernel_holder orbital::cspice_kernel_holder::holder("kernel");

// load kernels
orbital::cspice_kernel_holder::cspice_kernel_holder(const std::filesystem::path &kernels_folder) {
	// loop throught all files under kernels_folder
	for ( auto &p : filesystem::recursive_directory_iterator(kernels_folder) ) {
		if ( p.is_regular_file() ) {

		#ifdef _DEBUG
			cout << "loading " << p.path() << '\n';
			errdev_c("SET", 0, (char *)"SCREEN");
			errprt_c("SET", 0, (char *)"ALL");
			erract_c("SET", 0, (char *)"ABORT");
		#else
			errdev_c("SET", 0, (char *)"err.log");
			errprt_c("SET", 0, (char *)"ALL");
			erract_c("SET", 0, (char *)"RETURN");
		#endif // _DEBUG

			kernels.emplace(p.path()); // add p into the kernels path stack
			furnsh_c(kernels.top().string().c_str()); // call cspice API to load kernel
		}
	}
}

// unload kernels
orbital::cspice_kernel_holder::~cspice_kernel_holder() {
	while ( !kernels.empty() ) {
		unload_c(kernels.top().string().c_str()); // call cspice API to unload kernel
		kernels.pop(); // pop stack
	}
}

orbital::timesystem::InternationalAtomicTime::InternationalAtomicTime(const BarycentricDynamicalTime &tdb) :seconds_from_J2000(unitim_c(tdb.seconds(), "TDB", "TAI")) {}

orbital::timesystem::BarycentricDynamicalTime orbital::timesystem::InternationalAtomicTime::toTDB() const { return BarycentricDynamicalTime(unitim_c(seconds(), "TAI", "TDB")); }

orbital::timesystem::TerrestrialDynamicalTime::TerrestrialDynamicalTime(const BarycentricDynamicalTime &tdb) : seconds_from_J2000(unitim_c(tdb.seconds(), "TDB", "TDT")) {}

orbital::timesystem::BarycentricDynamicalTime orbital::timesystem::TerrestrialDynamicalTime::toTDB() const { return BarycentricDynamicalTime(unitim_c(seconds(), "TDT", "TDB")); }

orbital::timesystem::CoordinatedUniversalTime::CoordinatedUniversalTime(const BarycentricDynamicalTime &tdb) {
	SpiceDouble delta;
	deltet_c(tdb.seconds(), "ET", &delta);
	seconds_from_J2000 = tdb.seconds() - delta;
}

orbital::timesystem::BarycentricDynamicalTime orbital::timesystem::CoordinatedUniversalTime::toTDB() const {
	SpiceDouble delta;
	deltet_c(seconds(), "UTC", &delta);
	return BarycentricDynamicalTime(seconds() + delta);
}


orbital::object::cspice_point::cspice_point(const std::string &NAIF_name) :point("") {
	constexpr size_t max_name_len = 256;
	SpiceChar name[max_name_len];
	SpiceBoolean found;

	bods2c_c(NAIF_name.c_str(), &NAIF_ID, &found);

	if ( !found )
		throw cspice_error{ "cannot get NAIF_ID from '" + NAIF_name + "'" };

	bodc2s_c(NAIF_ID, max_name_len, name);
	this->point_name = name;
}

orbital::object::cspice_point::cspice_point(const SpiceInt &NAIF_ID) :point(""), NAIF_ID(NAIF_ID) {
	constexpr size_t max_name_len = 256;
	SpiceChar name[max_name_len];

	bodc2s_c(NAIF_ID, max_name_len, name);
	this->point_name = name;
}

state::point orbital::object::cspice_point::point_state_at(const timesystem::time_ptr &pt) const {
	SpiceDouble lt;
	SpiceDouble posvel[6];

	spkez_c(NAIF_ID, pt->toTDB().seconds(), "J2000", "NONE", 0, posvel, &lt);

	return { *this,
		pt,
		absolute_phase::point::make_node(
			relatively_phase::point{
				vec3{posvel[0],posvel[1],posvel[2]},
				vec3{posvel[3],posvel[4],posvel[5]},
				vec3_NaN
			}
	)
	};
}
