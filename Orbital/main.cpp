#include <iostream>
#include <chrono>
#include <string>
#include <math.h>
#include <vector>
#include <tuple>
#include <string>

#include <functional>
#include <random>
#include <memory>

#include <Eigen/Dense>

#include <boost/math/tools/roots.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/next.hpp> // For float_distance.
#include <boost/math/special_functions/cbrt.hpp> // For boost::math::cbrt.


#include <cspice/SpiceUsr.h>

#include "space.h"

#ifdef _DEBUG

#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#endif // _DEBUG


using namespace orbital;

boost::uintmax_t it = 0;

value_t mean2elliptic(value_t M, value_t e = 0) {
	using namespace boost::math::tools;
	using namespace boost::math::long_double_constants;

	/**
	static constexpr size_t ord = 4, sep = 2;
	static constexpr double coeff_table[sep][sep][ord][ord] ={
		{
			{{0.04166516827561763,0.8330431271094522,0.1722595017809713,-0.0512436841509584},{-0.39306840991952485,2.590158329496205,-1.6271521446909338,0.3162169310255297},{0.21078887785603048,0.1757065454713379,0,0},{1.053304986444104,-1.3445384837578394,0,0}},
			{{-0.22620954564205367,1.2388268604279764,-0.07855956691546745,0.007875849079899262},{1.4670383987121736,0.26743130438363627,-0.35518486516654046,0.04133280010717042},{-0.7974652145971981,0.17811523691315462,0,0},{-0.14312733691997792,0.1055547642182855,0,0}}
		},
		{
			{{0.14773937640632193,2.0095026569056977,-1.3958387761690103,0.3167025148523204},{-2.818866021301129,3.3662131835737954,0.09053151243179873,0.05987348942751839},{5.745378579921496,-5.251974444195626,0,0},{-2.3783317558608275,2.0494980426603338,0,0}},
			{{-0.7689301689443863,1.8452756142369147,-0.29436149916125487,0.032789074547786984},{2.8851478866383804,-1.2517923635029544,0.1654775836612348,-0.018478540146222466},{-1.2097952940529177,0.37895669946746885,0,0},{0.2703121247236524,-0.08315027581461518,0,0}}
		}
	};

	/**/

	static constexpr size_t ord = 4, sep = 4;
	static constexpr double coeff_table[sep][sep][ord][ord] = {
		{
			{{0.0012483724733875108,0.9919878118056848,0.006149946950965943,0.006578841938249232},{-0.022883739996147567,1.1529903788513847,-0.1202373609996659,-0.28993373637520525},{0.021215556489776244,0.7526349352843683,0,0},{0.30431638285518403,-0.00933712668366061,0,0}},
			{{0.020646259483659196,0.9521140613846486,0.036217197730783864,-0.008950507344668516},{-0.5054256670952778,2.2071370086286266,-0.9766347269551157,0.1144527613079234},{1.2720594079149574,-0.7804562894045538,0,0},{0.3116532590757686,-0.5959017641841649,0,0}},
			{{-0.022166190097144753,1.017077294128553,0.0000207908813922332,-0.0014733691939240075},{-0.21811256544131274,1.969830927823781,-0.924895270621506,0.11026164664371796},{0.9371058621841328,-0.6573052339780364,0,0},{-1.8523632166296125,0.9071052308812606,0,0}},
			{{-0.1146260796510516,1.1121659198196796,-0.03596560945771693,0.0037680554728151375},{0.9689065172016842,0.992372337847656,-0.6219400052486344,0.06645139163864858},{-2.214910909754668,0.6905491325877181,0,0},{1.0657430115338329,-0.3216442616688267,0,0}}
		},
		{
			{{0.031583110329208776,0.6439824888617103,0.6800855210028967,-0.21082421913396018},{0.047247689731879323,1.9382759319186547,-2.4760618348724877,0.38807568908689544},{-0.9545812241913492,3.2483068817149663,0,0},{1.483799194027057,-2.9886706499296016,0,0}},
			{{0.0194146027514068,0.922611226868551,0.11306618409974681,-0.05117180551860956},{-0.833643396948283,2.769361153412463,-1.4659527577649976,0.3236052952357645},{2.599550809142773,-1.9060047538126565,0,0},{-1.5557899800596147,0.9540281219658374,0,0}},
			{{-0.43650643524061417,1.5100695367543562,-0.18497934472122718,0.019599657448064273},{1.6291201167523852,-0.08365482443052791,-0.1963937336976986,0.030292148296292138},{-0.08900030564526762,-0.19505668243520422,0,0},{-0.44849162003805537,0.281563765695012,0,0}},
			{{-0.5591140284047698,1.5568114717372454,-0.18124078630429222,0.0192698778847371},{2.8872758390878133,-0.95611264135501,0.01813442143320876,-0.0017684065866897685},{-1.9407074764709809,0.6125555287149658,0,0},{0.7083620307867307,-0.22207666073515198,0,0}}
		},
		{
			{{0.3180336715581032,-1.8762095659484803,6.9955250514442735,-4.695901343752972},{-0.2183503710677088,5.834666250922318,-14.835442273828475,9.058519150379933},{-2.0160004625920678,6.8174318884810585,0,0},{2.3989552535228995,-5.813252231025009,0,0}},
			{{-0.6230305634474702,2.1328343879447544,-0.6664836604658182,0.11001460717096762},{0.7969897238084476,0.19335312337787613,0.0891375973780039,0.004730842095018619},{1.6390738579025808,-1.5034271621932869,0,0},{-0.9967529526635985,0.7515570084601072,0,0}},
			{{-0.8224133118952164,1.9700462654153261,-0.36831380715942147,0.044760088138723605},{2.695140122320007,-1.175424877058799,0.18669470288099577,-0.022162469811238827},{-0.902909060778665,0.2117823356851715,0,0},{0.12595829172659023,-0.004038929971444524,0,0}},
			{{-0.6815950970783535,1.7037768301190974,-0.23323336942022752,0.024896707568770885},{3.013053632549016,-1.227993834971784,0.12916664275311787,-0.013780103788454828},{-1.5140621757665629,0.48061804683084386,0,0},{0.418275237899401,-0.13248304012216866,0,0}}
		},
		{
			{{2.733370440900174,-0.48385814709854813,-8.79748004250742,4.627396864793672},{-12.754803840501632,21.62895816101012,5.130211660399395,-2.576743986245724},{16.71433051679349,-27.891001068334,0,0},{-6.1108922298169555,9.903741838837103,0,0}},
			{{-1.2537085158105907,2.9738333650630473,-1.0958241569611427,0.21048441809315052},{2.656995067607769,-1.6426638962350852,0.6880558005909847,-0.1351228517178275},{-0.38779981578043277,-0.1359471871209349,0,0},{-0.07562358632185351,0.1318657517371867,0,0}},
			{{-0.8853718908234034,2.044680997290272,-0.3950356450329936,0.04908502266073675},{2.862669039710872,-1.340163637770135,0.22596349026998525,-0.02843623870455387},{-1.0644431946422337,0.3288213722533549,0,0},{0.2039869216193324,-0.059110614203664515,0,0}},
			{{-0.5921525240193959,1.6622583366888326,-0.22697501312344012,0.02425134336758735},{2.7119875798712347,-1.1172353535137067,0.12172283771153794,-0.013017192064757974},{-1.1421269496595683,0.3634661368045464,0,0},{0.25042089836834336,-0.07962647068411272,0,0}}
		}
	};

	if ( e >= 1 ) throw std::invalid_argument("e should less than 1");

	auto dm = remainder(M, two_pi);
	bool neg = dm < 0;
	value_t guess = 0., err = 1.;

	if ( neg ) dm = -dm;

	auto ie = static_cast<unsigned>( e / 1. * sep );
	if ( ie >= sep ) ie = sep - 1;
	auto im = static_cast<unsigned>( dm / pi * sep );
	if ( im >= sep ) im = sep - 1;

	//err = (im == 0) ? (ie == (sep - 1) ? 0.8 : 0.06) : 0.04;
	err = ( im == 0 ) ? ( ie == ( sep - 1 ) ? 0.65 : 0.045 ) : 0.002;

	const auto &coeff = coeff_table[ie][im];

	const auto dm2 = dm * dm;
	const auto dm3 = dm * dm2; sep;

	guess  //calculate the guess point of the solution
		= ( 1 * coeff[0][0] + dm * coeff[0][1] + dm2 * coeff[0][2] + dm3 * coeff[0][3] ) * 1
		+ ( 1 * coeff[1][0] + dm * coeff[1][1] + dm2 * coeff[1][2] + dm3 * coeff[1][3] ) * e
		+ ( 1 * coeff[2][0] + dm * coeff[2][1] ) * e * e
		+ ( 1 * coeff[3][0] + dm * coeff[3][1] ) * e * e * e;

	if ( neg )
		guess = M + dm - guess;
	else
		guess = M - dm + guess;

	boost::uintmax_t maxit = 64;
	auto res = halley_iterate(
		[&e, &M](const value_t &E) { auto esinE = e * sin(E);  return std::make_tuple(E - esinE - M, 1 - e * cos(E), esinE); },
		guess, guess - err, guess + err, static_cast<int>( std::numeric_limits<value_t>::digits ), maxit);

	it = maxit;
	return res;
}

using namespace Eigen;
using namespace std;

std::mt19937 gen(std::random_device{}( ));

class testclass : std::enable_shared_from_this<testclass> {
	using node_t = testclass;
	using ptr_t = std::shared_ptr<node_t>;
	int data;
	ptr_t prev;
	testclass(testclass &&) = default;
	testclass(const testclass &) = default;
	testclass &operator= (testclass &&) = default;
	testclass &operator= (const testclass &) = default;

	testclass(const ptr_t &prev, const int &data) :prev(prev), data(data) { }
public:


	static ptr_t make_node(const ptr_t &prev, const int &data) {
		return ptr_t(new testclass(prev, data));
	}
	static ptr_t make_node(const int &data) {
		return make_node(nullptr, data);
	}

	ptr_t make_child(const int &data) { return make_node(shared_from_this(), data); }

	int get_data() const { return data; }
	void set_data(int n) { data = n; }
};

void phase_test(value_t ct) {
	using namespace boost::math::long_double_constants;

	auto SSB = absolute_phase::frame::root();

	auto earthmoon_MeanOrbit = absolute_phase::dirct::make_node(
		SSB,
		relatively_phase::dirct(make_quaternion(-11.3 * ( pi / 180. ), vec3(0, 0, 1)) * make_quaternion(2.3 * ( pi / 180. ), vec3(1, 0, 0)))
	);

	auto earth_MeanEquator = earthmoon_MeanOrbit->make_next_node(
		relatively_phase::dirct(make_quaternion(1.35 * ( pi / 180. ), vec3(0, 0, 1)) * make_quaternion(23.4 * ( pi / 180. ), vec3(1, 0, 0)))
	);

	auto moon_MeanOrbit = earth_MeanEquator->make_next_node(
		relatively_phase::dirct(make_quaternion(56.5 * ( pi / 180. ), vec3(0, 0, 1)) * make_quaternion(18. * ( pi / 180. ), vec3(1, 0, 0)))
	);

	auto earthmoon_CenterPoint = absolute_phase::point::make_node(
		SSB,
		relatively_phase::point(
			vec3(0.980614658546613 * cos(ct / 12.) + 0.19578828907141047 * sin(ct / 12.), -0.19594614424251772 * cos(ct / 12.) + 0.979824670586842 * sin(ct / 12.), 0.04013179253255973 * sin(ct / 12.)),
			vec3(0.016315690755950872 * cos(ct / 12.) - 0.08171788821221775 * sin(ct / 12.), 0.08165205588223683 * cos(ct / 12.) + 0.01632884535354314 * sin(ct / 12.), 0.0033443160443799775 * cos(ct / 12.)),
			vec3(-0.006809824017684812 * cos(ct / 12.) - 0.0013596408963292392 * sin(ct / 12.), 0.001360737112795262 * cos(ct / 12.) - 0.006804337990186402 * sin(ct / 12.), -0.00027869300369833146 * sin(ct / 12.))
		)
	);

	auto moon_CenterPoint = earthmoon_CenterPoint->make_next_node(
		{
			moon_MeanOrbit,
			relatively_phase::point(
				vec3(0.5 * cos(ct + 12. * ( pi / 180. )), 0.5 * sin(ct + 12. * ( pi / 180. )), 0.),
				vec3(-0.5 * sin(ct + 12. * ( pi / 180. )), 0.5 * cos(ct + 12. * ( pi / 180. )), 0.),
				vec3(-0.5 * cos(ct + 12. * ( pi / 180. )), -0.5 * sin(ct + 12. * ( pi / 180. )), 0.)
			)
		}
	);

	auto tmp_point = earthmoon_CenterPoint->make_next_node(
		{
			moon_MeanOrbit,
			relatively_phase::point(
				vec3(-0.1 * cos(ct + 12. * ( pi / 180. )), -0.1 * sin(ct + 12. * ( pi / 180. )), 0.),
				vec3(0.1 * sin(ct + 12. * ( pi / 180. )), -0.1 * cos(ct + 12. * ( pi / 180. )), 0.),
				vec3(0.1 * cos(ct + 12. * ( pi / 180. )), 0.1 * sin(ct + 12. * ( pi / 180. )), 0.))
		}
	);

	auto tmp_dirct = earth_MeanEquator->make_next_node(
		{
			make_quaternion(( 30. * ct + 5. * cos(ct) ) * 360. * ( pi / 180. ), vec3(0, 0, 1)),
			360. * ( pi / 180. ) * ( 30. - 5. * sin(ct) ) * vec3(0, 0, 1),
			-1800. * ( pi / 180. ) * cos(ct) * vec3(0, 0, 1)
		}
	);

	auto earth_FixBody = absolute_phase::frame::make_node(tmp_dirct, tmp_point);

	double phi = 35 * ( pi / 180 );
	double lambda = 45 * ( pi / 180 );

	auto localstation_ENT = absolute_phase::frame::make_node(
		earth_FixBody,
		relatively_phase::frame(
			relatively_phase::point(vec3(0.11 * cos(phi) * cos(lambda), 0.11 * cos(phi) * sin(lambda), 0.11 * sin(phi))),
			relatively_phase::dirct(make_quaternion(lambda, { 0,0,1 }) * make_quaternion(-phi, { 0,1,0 }) * make_quaternion(half_pi, { 0,1,0 }) * make_quaternion(half_pi, { 0,0,1 }))
		)
	);


	auto satt = absolute_phase::point::make_node(
		earthmoon_CenterPoint,
		{
			SSB,
			relatively_phase::point(
				vec3(0.25 * cos(345600. * ct * ( pi / 180. )), 0.25 * sin(345600. * ct * ( pi / 180. )), 0),
				vec3(-1507.96 * sin(345600. * ct * ( pi / 180. )), 1507.96 * cos(345600. * ct * ( pi / 180. )), 0.),
				vec3(-9.09583e6 * cos(345600. * ct * ( pi / 180. )), -9.09583e6 * sin(345600. * ct * ( pi / 180. )), 0.)
			)

		}
	);

	cout << earthmoon_CenterPoint->different_from(absolute_phase::frame::root()).posvelacc().transpose() << endl;
	cout << earth_FixBody->point_diff_from(earthmoon_CenterPoint).reduce_under(SSB).posvelacc().transpose() << endl;
	auto ussb = earth_FixBody->different_from(SSB);
	cout << ussb.posvelacc().transpose() << endl;
	cout << localstation_ENT->different_from(SSB).posvelacc().transpose() << endl;
	cout << satt->different_from(SSB).posvelacc().transpose() << endl;
	cout << satt->different_from(localstation_ENT).posvelacc().transpose() << endl;
	cout << "\n" << endl;
	cout << satt->distanceTo(localstation_ENT) << endl;
	cout << "\n" << endl;
}


int main(void) {
#ifdef _DEBUG
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif // _DEBUG

	using namespace boost::math::long_double_constants;
	
	phase_test(55.6 * pi / 10);

	return 0;

	auto s = std::chrono::high_resolution_clock::now();
	size_t c = 0;
	value_t err = 0;
	value_t err_sum = 0;
	value_t max_E = 0;
	value_t max_e = 0;
	boost::uintmax_t mit = 0;
	value_t it_sum = 0;
	value_t maxit_E = 0;
	value_t maxit_e = 0;
	for ( value_t e = 0; e < 1; e += 0.00025 ) {
		for ( value_t i = -5; i <= 5; i += 0.0005 ) {
			auto er = abs(mean2elliptic(i - e * sin(i), e) - i);
			if ( er > err )err = er, max_E = i, max_e = e;
			if ( it > mit )mit = it, maxit_E = i, maxit_e = e;
			err_sum += er;
			it_sum += it;
			++c;
		}
	}
	auto e = std::chrono::high_resolution_clock::now();
	std::cout << std::fixed << std::setprecision(26);
	std::cout << err << std::endl;
	std::cout << max_E << std::endl;
	std::cout << max_e << std::endl;
	std::cout << err_sum / c << std::endl;
	std::cout << std::endl;
	std::cout << mit << std::endl;
	std::cout << maxit_E << std::endl;
	std::cout << maxit_e << std::endl;
	std::cout << it_sum / c << std::endl;
	std::cout << std::endl;
	std::cout << c << std::endl;
	std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>( e - s ).count() / c << std::endl;
}


