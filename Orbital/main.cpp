#include <iostream>
#include <chrono>
#include <string>
#include <math.h>
#include <vector>
#include <tuple>
#include <fstream>

#include <functional>
#include <random>
#include <memory>

#include <Eigen/Dense>

#include <boost/math/tools/roots.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/next.hpp> // For float_distance.
#include <boost/math/special_functions/cbrt.hpp> // For boost::math::cbrt.

#include "space.h"
#include "cspice.h"

#include "kepler.h"

#ifdef _DEBUG

#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#endif // _DEBUG

// CSPICE
#include <cspice/SpiceUsr.h>
#pragma comment(lib, "cspice.lib")
#pragma comment(lib, "csupport.lib")

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

	testclass(const ptr_t &prev, const int &data) :prev(prev), data(data) {}
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
	size_t col;

	tuple<value_t, value_t, value_t> ref_datas[] =
	{ {0,0.,3506102.133176311384886503219604},{60,0.2552432794628469758866079942628,3563832.837858118880684824237894},{120,0.4949794990948845037413695823078,3729938.6452830169536725574415739},{180,0.7090471496101336513281697646320,3986511.373115476080276253197664},{240,0.8940213810101583115544213527919,4311800.8290852322391853358737484},{300,1.0512564868809008270187599703726,4685796.5053554186797304515320604},{360,1.1843062441139831604654612568280,5092600.7406223242809207149777571},{420,1.2971950862154086610589718308274,5520549.259385817646157024446764},{480,1.3936084896722606593650389197817,5961429.876192129169326218604192},{540,1.4766369755667324175911257163553,6409574.313793141826438238626995},{600,1.5487717746682248357053398557402,6861103.745940989127072145131505},{660,1.6119886182037341809941846335852,7313373.699125085891162833927526},{720,1.6678477947664860260148200050644,7764586.358534600915801399279869},{780,1.7175842307628977261254850534719,8213525.508035470418668784096221},{840,1.7621808606262720107980471134236,8659376.397494897610862480370119},{900,1.8024258428699547423884813646154,9101603.086277530866553916278780},{960,1.8389563393382123445197256563207,9539864.417282695769193973006816},{1020,1.8722917894461128712169650384298,9973955.993721066344394395725311},{1080,1.9028591985936306252397324695136,10403769.762755047170684636233022},{1140,1.9310124277107505763175990946023,10829265.619275240761136487933931},{1200,1.9570469916603907464362996121875,11250451.292009669314795426189672},{1260,1.9812114904573210029142644520887,11667367.991264872869572642243896},{1320,2.0037165053498451732025041939543,12080080.102710597653919947445502},{1380,2.0247415751150376603026118257105,12488667.748206764010148906920745},{1440,2.0444407088074394381787758138700,12893221.395482625668576324694765},{1500,2.0629467747891560119412393840434,13293837.943417203450873565251053},{1560,2.0803750206363750342719993325228,13690617.877575830325634967196538},{1620,2.0968259159067262734473809014342,14083663.206874398060959647599227},{1680,2.1123874635353082186815116508127,14473075.973443407889795224634736},{1740,2.1271370913135864785375643440569,14858957.185016129999763534349106},{1800,2.1411432092673318212333568175469,15241406.059882083291355054241287},{1860,2.1544664994667215824105267008354,15620519.503643093341205741517026},{1920,2.1671609902006791865266388306737,15996391.758106877735960808516762},{1980,2.1792749553173146661257891519605,16369114.178008794836142316049829},{2040,2.1908516709913968885860891333489,16738775.102506456672789869715990},{2100,2.2019300555832377417782380952134,17105459.796690465228307154867161},{2160,2.2125452131262177151892418960962,17469250.444509631202259226414473},{2220,2.2227288969704900466281191120550,17830226.179100884306248065855794},{2280,2.2325099069561744734109213169040,18188463.139952706846729318631835},{2340,2.2419144309935931457764887038387,18544034.548920913487320089560142},{2400,2.2509663399425207152550379128684,18897010.799072353306857183495340},{2460,2.2596874430943125422959997182839,19247459.551814881350300038242219},{2520,2.2680977102839460489882745733901,19595445.838900756759277519802448},{2580,2.2762154656274661854832441110661,19941032.166750166486786919627456},{2640,2.2840575570429218456378157255711,20284278.621198302240237404453600},{2700,2.2916395050299816605320494552236,20625242.971271481415834176167078},{2760,2.2989756336240768119391715664798,20963980.77098141501413486405483},{2820,2.3060791859807903181642774418644,21300545.458420867918598191488157},{2880,2.3129624266662252307473990120428,21634988.451667474511959294076136},{2940,2.3196367324139734256199853317038,21967359.241173599709364379840516},{3000,2.3261126728470508928351361093388,22297705.478449665128291068976054},{3060,2.3324000824440888635874203987518,22626073.06094612376022584265779},{3120,2.3385081248454162142781781558480,22952506.213112597084631837487646},{3180,2.3444453504401916068057693039547,23277047.563667012317304139244598},{3240,2.3502197480453811377941583746913,23599738.21914727524092556308993},{3300,2.3558387913770305649667850559220,23920617.833845943376827555349991},{3360,2.3613094809205388399841160877395,24239724.67624907086694974200391},{3420,2.3666383817268740293611246354090,24557095.692111920492860865333243},{3480,2.3718316575934755190154036835573,24872766.564313231161875080990874},{3540,2.3768951020302177474349720647759,25186771.769633477352979035837545},{3600,2.3818341663607077811086396616051,25499144.63260260843894001927352},{3660,2.3866539852659766079332018555929,25809917.376563673536170694628460},{3720,2.3913594000404469142103845685496,26119121.17209330327539696411825},{3780,2.3959549797977638698667141655342,26426786.182918795955303785855352},{3840,2.4004450408361848984005017541519,26732941.60946468632040155728777},{3900,2.4048336643489002911789820701301,27037615.730157757882792326054289},{3960,2.4091247126435082971954779339255,27340835.940613490101548897100724},{4020,2.4133218440164081196626847712794,27642628.79082073034260653850633},{4080,2.4174285264116922223926786821663,27943020.020436659982817443219864},{4140,2.4214480499799858682965959051215,28242034.592297124356052155553890},{4200,2.4253835386402243763181141297313,28539696.72424259396687806689437},{4260,2.4292379607364163305682120471655,28836029.91935447728011499818558},{4320,2.4330141388718027460828794138161,29131056.994690759007967385933770},{4380,2.4367147589942948704615009114620,29424800.108605169288659893889314},{4440,2.4403423787995362087761087301119,29717280.78672921295245241284464},{4500,2.4438994355112532071382933509594,30008519.94669184420097848055446},{4560,2.4473882530926364348169752821361,30298537.92164684987985117573050},{4620,2.4508110489372156813044442464961,30587354.482674192670562039114586},{4680,2.4541699400830043529788566065946,30874988.860117323980903798933366},{4740,2.4574669489894934244629449687251,31161459.763915236377388926621172},{4800,2.4607040089133466820460092798982,31446785.402983973349808503811194},{4860,2.4638829689152973104612825097926,31730983.50369940719051343978924},{4920,2.4670055985277526896593108233667,32014071.32752987164443835696911},{4980,2.4700735921099266391434557231939,32296065.68786436116799009594605},{5040,2.4730885729149045700897284360303,32576982.96607930071134049767066},{5100,2.4760520968908769161982189654582,32856839.12688430170515904130944},{5160,2.4789656562368209655910302635740,33135649.73298495382539729856027},{5220,2.4818306827311491268474184547784,33413429.95909843044566866132586},{5280,2.4846485508502484880904995686082,33690194.60535570865657428201849},{5340,2.4874205806924043588165054407171,33965958.11012185824741026625007},{5400,2.4901480407212894839418712419439,34240734.56226460417069268078932},{5460,2.4928321503420309075591431971209,34514537.71289905728391873019483},{5520,2.4954740823217917430602926040276,34787380.98663522125364601617944},{5580,2.4980749650658360250226344731005,35059277.49235325062785677188150},{5640,2.5006358847591621230381527293318,35330240.03353002360533578235029},{5700,2.5031578873829872777688207433151,35600281.11813925503283017518433},{5760,2.5056419806146344770393785251151,35869412.96814612323613395634794},{5820,2.5080891356187064985849420834349,36137647.52861618141929399318826} };

	/*/
	ofstream fout{ "err.csv" ,std::ios::out | std::ios::trunc };
	for ( auto e = 0.5; e <= 1.5; e += 0.01 )
		for ( auto p_rate = 0.1; p_rate < 10; p_rate += 0.1 ) {
			auto p = 6371000 * p_rate;
			planeKepler obt(p, e, 3.986004418e14);
			for ( auto chi = -10000; chi <= 10000; chi += 100 ) {
				if ( e == 1 )
					cout << "??" << endl;

				auto t = obt.dt_by_universal_anomaly(chi);
				auto _chi = obt.universal_anomaly_by_dt(t);

				auto abs_err = chi - _chi;
				auto rel_err = abs_err / ( _chi + 1e-16 );

				fout << setprecision(32);
				fout << e << ", ";
				fout << p << ", ";
				fout << chi << ", ";
				fout << t * (1 + obt.e()) / obt.p() << ", ";
				fout << t * obt.mu() << ", ";
				fout << abs_err << "\n";
			}
		}

	return 0;
	/**/

	double count = 0;
	double total_abs_err = 0, max_abs_err = 0, max_abs_err_v0, max_abs_err_t;
	double total_rel_err = 0, max_rel_err = 0, max_rel_err_v0, max_rel_err_t;

	auto start_t = chrono::high_resolution_clock::now();
	for ( double v0 = 7800; v0 < 15000; v0 += 200 ) {
		planeKepler obt1(3.986004418e14, vec2(6371000, 0), vec2(0, v0));
		//cout << sqrt(abs(obt1.a())) << endl;
		for ( value_t t = -150000.; t < 150000.; t += 1 ) {

			auto theta = obt1.true_anomaly_by_dt(t);
			auto t_ = obt1.dt_by_true_anomaly(theta);

			auto chi = obt1.anomaly_universal_by_true(theta);
			auto _t = obt1.dt_by_universal_anomaly(chi);
			auto _chi = obt1.universal_anomaly_by_dt(t);

			//auto err = obt1.dt_by_universal_anomaly(_chi) - t;
			//if ( obt1.is_ellipse() ) err = remainder(err, obt1.T());

			auto abs_err = chi - _chi;
			if ( obt1.is_ellipse() ) abs_err = remainder(abs_err, sqrt(abs(obt1.a())) * boost::math::long_double_constants::two_pi);
			auto rel_err = abs_err / ( _chi + 1e-16 );

			//if ( err * err > 1 )
			//	cout << "err" << endl;
			if ( isnan(abs_err) )
				cout << "NaN @ t=" << t << " v0=" << v0 << endl;
			else {
				if ( abs(abs_err) > abs(max_abs_err) ) {
					max_abs_err = abs_err;
					max_abs_err_v0 = v0;
					max_abs_err_t = t;
				}
				if ( abs(rel_err) > abs(max_rel_err) ) {
					max_rel_err = rel_err;
					max_rel_err_v0 = v0;
					max_rel_err_t = t;
				}
				total_abs_err += abs(abs_err);
				total_rel_err += abs(rel_err);
				count += 1;
			}
			//cout << ( t - _t ) << "\n";
			//cout << t << '\t';
			//cout << err << '\n';
		}
	}
	auto end_t = chrono::high_resolution_clock::now();
	cout << fixed << setprecision(3) << ( std::chrono::duration_cast<std::chrono::nanoseconds>( end_t - start_t ).count() / count ) << "ns" << endl;

	cout << scientific << ( total_abs_err / count ) << endl;
	cout << scientific << "Max abs err=" << max_abs_err << "\t";
	cout << fixed << "@ v0=" << max_abs_err_v0 << "\t";
	cout << fixed << "t=" << max_abs_err_t << endl;

	cout << scientific << ( total_rel_err / count ) << endl;
	cout << scientific << "Max rel err=" << max_rel_err << "\t";
	cout << fixed << "@ v0=" << max_rel_err_v0 << "\t";
	cout << fixed << "t=" << max_rel_err_t << endl;

	return 0;



	using namespace boost::math::long_double_constants;

	//auto target = object::cspice_point("2003451");
	//auto now_tpr = timesystem::TAI::make_node(chrono::system_clock::now());
	//auto s = target(now_tpr);
	//cout << fixed << setprecision(32) << now_tpr->seconds() << endl;
	//cout << fixed << setprecision(32) << now_tpr->toTDB().seconds() << endl;
	//cout << fixed << setprecision(32) << now_tpr->toTDB().JulianDate() << endl;
	//cout << scientific << right << setprecision(15) << setw(25) << s.get_pointptr()->different_from(absolute_phase::frame::root()).posvelacc() << endl;
	//return 0;

	SpiceDouble et;
	SpiceDouble lt;
	SpiceDouble posvel[6];
	SpiceDouble xform[6][6];

	/**
	et = timesystem::TAI(chrono::system_clock::now()).toTDB().seconds();
	cout << "TDB: " << fixed << setprecision(9) << timesystem::TDB(et).JulianDate() << "\n\t";

	sxform_c("IAU_MARS", "J2000", et, xform);

	Matrix < SpiceDouble, 6, 6> eigen_xform(&xform[0][0]);
	cout << eigen_xform << endl;

	return 0;
	/**/

	constexpr size_t max_name_len = 256;
	SpiceChar name[max_name_len];
	SpiceBoolean found;
	SpiceInt code;

	cout << "FRAMES:" << endl;
	for ( code = 0, col = 0; code < 100000; ++code ) {
		frmnam_c(code, max_name_len, name);
		if ( !iswhsp_c(name) ) {
			cout << right << setw(6) << code << ": " << left << setw(25) << name << "    \t";
			if ( ( ++col ) % 5 == 0 )
				cout << endl;
		}
	}
	cout << "\n\n" << endl;

	for ( code = 0, col = 0; code < 10000000; ++code ) {
		bodc2n_c(code, max_name_len, name, &found);
		if ( found ) {
			cout << right << setw(6) << code << ": " << left << setw(25) << name << "    \t";
			if ( ( ++col ) % 5 == 0 )
				cout << endl;
		}
	}
	return 0;

	code = 2000029;
	bodc2s_c(code, max_name_len, name);
	cout << "\ncode: " << code << "\tname: " << name << "\n" << endl;
	et = timesystem::TAI(chrono::system_clock::now()).toTDB().seconds();

	for ( size_t i = 0; i < 10; ++i, et += 3600. ) {
		spkez_c(code, et, "ECLIPJ2000", "NONE", 0, posvel, &lt);
		cout << "TDB: " << fixed << setprecision(9) << timesystem::TDB(et).JulianDate() << "\n\t";
		cout << scientific << right << setprecision(15) << setw(25) << posvel[0] << "\t";
		cout << scientific << right << setprecision(15) << setw(25) << posvel[1] << "\t";
		cout << scientific << right << setprecision(15) << setw(25) << posvel[2] << "\n\t";
		cout << scientific << right << setprecision(15) << setw(25) << posvel[3] << "\t";
		cout << scientific << right << setprecision(15) << setw(25) << posvel[4] << "\t";
		cout << scientific << right << setprecision(15) << setw(25) << posvel[5] << "\n";
		cout << "\n" << endl;
	}

	//for ( size_t i = 0; i < 10000000; ++i ) {
	//	code = i;
	//	bodc2n_c(code, max_name_len, name, &found);
	//	if ( found )
	//		cout << code << "\t" << name << endl;
	//}

	//bods2c_c("IAU_EARTH", &code, &found);
	//cout << code << endl;

	return 0;
	/**
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
	/**/
}


