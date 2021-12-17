#pragma once
#include <cmath>
#include <tuple>
#include <limits>
#include <string>
#include <utility>
#include <optional>
#include <exception>
#include <functional>

template<class T>
class float_comp {
	const std::optional<int> rel_digits, abs_digits;
public:
	constexpr float_comp(const std::optional<int> &rel_digits = endianness, const std::optional<int> &abs_digits = std::nullopt) :rel_digits(rel_digits), abs_digits(abs_digits) {}
	bool operator()(const T &a, const T &b) const {
		return ( rel_digits && ( is_rel_similar(a, b, *rel_digits) ) ) || ( abs_digits && is_abs_similar(a, b, *abs_digits) );
	}
	bool operator()(const T &x) const { return ( *this )( x, 0 ); }

	static constexpr auto min_exp = std::numeric_limits<T>::min_exponent - 1; // ilogb(numeric_limits<double>::min())
	static constexpr auto endianness = std::numeric_limits<T>::digits - 1;
	static constexpr auto nan = std::numeric_limits<T>::quiet_NaN();
	static constexpr auto inf = std::numeric_limits<T>::infinity();
	static constexpr auto min = std::numeric_limits<T>::min();


	static T get_eps(const int &exp) {
		if ( exp < min_exp ) return min; // in case the exp is too small for float-point
		else return ldexp(1, exp);
	}
	static T get_eps(const T &x, const int &digits) {
		if ( isinf(x) || isnan(x) ) return nan;
		if ( x == 0 ) return min;
		return get_eps(ilogb(x) - digits);
	}
	static bool is_rel_similar(const T &a, const T &b, const int &digits) {
		if ( digits < 0 ) return false;
		if ( a == b ) return true;// in case both a and b are zero
		auto diff_digits = std::ilogb(a - b);
		return ( diff_digits <= std::max(std::ilogb(a) - digits, min_exp) ) && ( diff_digits <= std::max(std::ilogb(b) - digits, min_exp) );
	}
	static bool is_abs_similar(const T &a, const T &b, const int &digits) {
		if ( digits < 0 ) return false;
		return std::ilogb(a - b) <= std::max(-digits, min_exp);
	}
};

template<class T, class R>
struct bracket_type_helpper {
	using input_t = T;
	using output_t = R;

	using state_t = std::tuple<std::pair<T, R> &, std::pair<T, R> &, std::pair<T, R> &, std::pair<T, R> &>;
	using step_t = std::tuple<std::pair<T, R>, std::pair<T, R>, std::pair<T, R>>;
	using predict_t = std::pair<T, std::optional<R>>;

	static constexpr T T_nan = std::numeric_limits<T>::quiet_NaN();
	static constexpr T T_inf = std::numeric_limits<T>::infinity();
	static constexpr R R_nan = std::numeric_limits<R>::quiet_NaN();
	static constexpr R R_inf = std::numeric_limits<R>::infinity();

};

template<class Cr, class Cx, class Ce, class T, class R>
class bracket_precision_controller : public bracket_type_helpper<T, R> {
	size_t maxit;
	const std::optional<Cr> range_comp;
	const std::optional<Cx> result_comp;
	const std::optional<Ce> error_comp;
public:
	constexpr bracket_precision_controller(const size_t &maxit,
										   const std::optional<Cr> &range_comp,
										   const std::optional<Cx> &result_comp = std::nullopt,
										   const std::optional<Ce> &error_comp = std::nullopt)
		:maxit(maxit), result_comp(result_comp), range_comp(range_comp), error_comp(error_comp) {}

	bool operator()(const typename bracket_type_helpper<T, R>::state_t &pkg) {
		const auto &[a, x, prev_x, b] = pkg;

		return !( 0 == maxit-- ) && // iteration count limit
			!( result_comp && result_comp.value()( x.first, prev_x.first ) ) && // if the solution didn't move much (convergence)
			!( range_comp && range_comp.value()( a.first, b.first ) ) && // if the bracket is small enough
			!( error_comp && error_comp.value()( x.second ) ); // if the error is small enough
	}
};

template<class F, class T, class R>
class bisect_predictor : public bracket_type_helpper<T, R> {
	const F f;
public:
	bisect_predictor(const F &f) :f(f) {}
	R operator()(const T &x) const { return f(x); }
	typename bracket_type_helpper<T, R>::predict_t operator()(const typename bracket_type_helpper<T, R>::state_t &pkg) const {
		const auto &[a, x, prev_x, b] = pkg;
		return { ( a.first + b.first ) / 2. ,std::nullopt };
	}
};

template<class F, class T, class R>
class regula_falsi_predictor : public bracket_type_helpper<T, R> {
	const F f;
public:
	regula_falsi_predictor(const F &f) :f(f) {}
	R operator()(const T &x) const { return f(x); }
	typename bracket_type_helpper<T, R>::predict_t operator()(const typename bracket_type_helpper<T, R>::state_t &pkg) const {
		const auto &[a, x, prev_x, b] = pkg;
		return { ( b.first * a.second - a.first * b.second ) / ( a.second - b.second ) ,std::nullopt };
	}
};

template<class F0, class F1, class T, class R>
class newton_raphson_predictor : public bracket_type_helpper<T, R> {
	const F0 f0;
	const F1 f1;
public:
	newton_raphson_predictor(const F0 &f0, const F1 &f1) :f0(f0), f1(f1) {}
	R operator()(const T &x) const { return f0(x); }
	typename bracket_type_helpper<T, R>::predict_t operator()(const typename bracket_type_helpper<T, R>::state_t &pkg) const {
		const auto &[a, x, prev_x, b] = pkg;
		const auto y0 = x.second;
		const auto y1 = f1(x.first);

		if ( y1 != 0 ) return { x.first - y0 / y1, std::nullopt };
		return{ bracket_type_helpper<T, R>::T_nan,std::nullopt };
	}
};

template<class F0, class F1, class F2, class T, class R>
class halley_predictor : public bracket_type_helpper<T, R> {
	const F0 f0;
	const F1 f1;
	const F2 f2;
public:
	halley_predictor(const F0 &f0, const F1 &f1, const F2 &f2) :f0(f0), f1(f1), f2(f2) {}
	R operator()(const T &x) const { return f0(x); }
	typename bracket_type_helpper<T, R>::predict_t operator()(const typename bracket_type_helpper<T, R>::state_t &pkg) const {
		const auto &[a, x, prev_x, b] = pkg;
		const auto y0 = x.second;
		const auto y1 = f1(x.first);
		const auto y2 = f2(x.first);

		if ( auto dom = 2 * y1 * y1 - x.second * y2; dom != 0 ) return { x.first - 2 * y0 * y1 / dom, std::nullopt };
		if ( y1 != 0 ) return { x.first - y0 / y1, std::nullopt };
		return{ bracket_type_helpper<T, R>::T_nan,std::nullopt };
	}
};


template<class Predictor>
class bracket_stepper : public bracket_type_helpper<typename Predictor::input_t, typename Predictor::output_t> {
	const Predictor predictor;
	using T = typename Predictor::input_t;
	using R = typename Predictor::output_t;
public:
	bracket_stepper(const Predictor &predictor) :predictor(predictor) {}
	R operator()(const T &x) const { return predictor(x); }
	void operator()(const typename bracket_type_helpper<T, R>::state_t &in_pkg,
					typename bracket_type_helpper<T, R>::step_t &out_pkg) const {
		auto &[a, x, b] = out_pkg; // unpackage out_pkg

		const auto [res, opt] = predictor(in_pkg); // call predictor with x_n to get root predict
		if ( std::isnan(res) || std::isinf(res) || res <= a.first || b.first <= res ) return; // the result cannot refine by this predict

		x = { res, opt.value_or(predictor(res)) }; // get x_{n + 1} by predict (if the predictor didn't generate f(x_{n + 1}), then call it)
		if ( std::signbit(a.second) != std::signbit(x.second) )
			b = x; // root in [a, x]
		else
			a = x; // root in [x, b]
	}
};

template<class MainStepper, class ... Steppers>
class mixed_stepper : public bracket_type_helpper<typename MainStepper::input_t, typename MainStepper::output_t> {
	std::tuple<const MainStepper, const Steppers...> steppers;
	using T = typename MainStepper::input_t;
	using R = typename MainStepper::output_t;
public:
	mixed_stepper(const MainStepper &major, const Steppers &...backups) :steppers(major, backups...) {}
	R operator()(const T &x) const { return  std::get<0>(steppers)( x ); }
	void operator()(const typename bracket_type_helpper<T, R>::state_t &in_pkg, typename bracket_type_helpper<T, R>::step_t &out_pkg) const {
		step_impl(in_pkg, out_pkg, std::index_sequence_for<MainStepper, Steppers...>{});
	}
private:
	template<std::size_t... Is>
	void step_impl(const typename bracket_type_helpper<T, R>::state_t &in_pkg,
				   typename bracket_type_helpper<T, R>::step_t &out_pkg,
				   std::index_sequence<Is...>) const {
		( std::get<Is>(steppers) ( in_pkg, out_pkg ), ... );
	}
};


template<class T, class R, class Stepper, class Controller>
class bracket_solver {
	std::pair<T, R> a, x, b;
	const Stepper &stepper;
	Controller controller;

	void nan_check(const std::pair<T, R> &p) {
		if ( std::isnan(p.first) || std::isnan(p.second) ) throw std::runtime_error("unexpected NaN: in bracket_solver func(" + std::to_string(p.first) + ") returns " + std::to_string(p.second));
	}
public:
	static constexpr T T_nan = std::numeric_limits<T>::quiet_NaN();
	static constexpr T T_inf = std::numeric_limits<T>::infinity();
	static constexpr R R_nan = std::numeric_limits<R>::quiet_NaN();
	static constexpr R R_inf = std::numeric_limits<R>::infinity();

	const T T_eps = exp2(1 - std::numeric_limits<T>::digits);
	const R R_eps = exp2(1 - std::numeric_limits<R>::digits);

	// inital solver with guess, and bracket as [min, max]
	bracket_solver(const Stepper &stepper, const T &guess, const T &min, const T &max, const Controller &controller)
		:stepper(stepper), a(min, stepper(min)), x(guess, stepper(guess)), b(max, stepper(max)), controller(controller) {
		// sign check
		if ( std::signbit(a.second) == std::signbit(b.second) ) throw std::runtime_error("inital bracket doesn't contain root ( func(min) and func(max) have same sign )");
		// inf check
		if ( std::isinf(a.first) || std::isinf(b.first) ) throw std::domain_error("inital bracket's bounds contain infty");
		if ( std::isinf(x.first) ) throw std::domain_error("inital value is infty");
		// NaN check
		nan_check(a), nan_check(x), nan_check(b);

		if ( a.first > b.first ) // ensure a.first <= b.first
			std::swap(a, b);
	}
	// inital solver with guess = (min + max)/2, and bracket as [min, max]
	bracket_solver(const Stepper &stepper, const T &min, const T &max, const Controller &controller) :bracket_solver(stepper, ( min + max ) / 2, min, max, controller) {}
	// inital solver with guess, the bracket is searched automatically
	bracket_solver(const Stepper &stepper, const T &guess, T ra, T rb, const T &factor, const Controller &controller, const bool &mono = false, const bool &inc = true)
		:stepper(stepper), x(guess, stepper(guess)), controller(controller) {
		if ( std::isnan(guess) || std::isnan(ra) || std::isnan(rb) ) throw std::domain_error("guess and r cannot be nan");
		if ( std::isinf(guess) || std::isinf(ra) || std::isinf(rb) ) throw std::domain_error("guess and r cannot be infty");

		ra = std::abs(ra), a = { guess - ra, stepper(guess - ra) };
		rb = std::abs(rb), b = { guess + rb, stepper(guess + rb) };


		if ( mono ) {
			while ( std::signbit(a.second) == std::signbit(b.second) )
				if ( a.second > 0 )
					ra *= factor, a = { guess - ra, stepper(guess - ra) };
				else
					rb *= factor, b = { guess + rb, stepper(guess + rb) };
		}
		else {
			while ( std::signbit(a.second) == std::signbit(b.second) ) {
				ra *= factor, a = { guess - ra, stepper(guess - ra) };
				rb *= factor, b = { guess + rb, stepper(guess + rb) };
			}
		}
	}

	bracket_solver &solve() {
		std::pair<T, R> prev_x{ T_nan, R_nan };
		const auto pkg = std::tie(a, x, prev_x, b);

		while ( controller(pkg) ) {
			std::tuple res{ a, x, b };

			stepper(pkg, res);
			nan_check(x);

			prev_x = x, std::tie(a, x, b) = res;

			// in case that the root is on the boudary
			if ( a.second == 0 ) { x = a; }
			if ( b.second == 0 ) { x = b; }
			if ( x.second == 0 ) { a = x, b = x; break; } // found the precise root value
		}
		return *this;
	}

	T get_result() const { return x.first; }
	R get_error() const { return x.second; }
	std::pair<T, T> get_range() const { return { a.first, b.first }; }
};

// x^3 + 3 * p * x + 2 * q ==0
template<class T>
T _cubic_equation_root(const T &p, const T &q) {
	auto tmp = p * p * p + q * q;
	tmp = std::cbrt(std::sqrt(tmp) - q);
	return tmp - p / tmp;
}

// a * x^3 + c * x + d * q ==0
template<class T>
T _cubic_equation_root(const T &a, const T &c, const T &d) {
	return _cubic_equation_root<T>(c / 3 / a, d / 2 / a);
}

//x^3 + b * x^2 + c * x + d * q ==0
template<class T>
T cubic_equation_root(const T &b, const T &c, const T &d) {
	return _cubic_equation_root<T>(c - b * b / 3, 2 * b * b * b / 27 - b * c / 3 + d);
}

//a * x^3 + b * x^2 + c * x + d * q ==0
template<class T>
T cubic_equation_root(const T &a, const T &b, const T &c, const T &d) {
	return cubic_equation_root<T>(b / a, c / a, d / a);
}