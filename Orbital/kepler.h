#pragma once
#include <cmath>

#include "util.h"


namespace orbital {
	namespace stumpff {
		inline value_t S(const value_t &z) {
			if ( z == 0 ) return 1 / 6.;
			if ( z > 0 ) {
				auto sqrt_z = sqrt(z);
				return 1 / z - sin(sqrt_z) / ( z * sqrt_z );
			}
			else {
				auto sqrt_z = sqrt(-z);
				return 1 / z - sinh(sqrt_z) / ( z * sqrt_z );
			}
		}

		inline value_t C(const value_t &z) {
			if ( z == 0 ) return 1 / 2.;
			if ( z > 0 ) return ( 1 - cos(sqrt(z)) ) / z;
			else return ( 1 - cosh(sqrt(-z)) ) / z;
		}
	}

	class planeKepler {
		value_t _p, _e, _mu;
	public:
		enum orbit_type {
			ERROR = 0,
			ellipse = 1,
			hyperbola = 2,
			parabola = 4,
			line = 8
		};

		planeKepler(planeKepler &&) = default;
		planeKepler(const planeKepler &) = default;
		planeKepler &operator= (planeKepler &&) = default;
		planeKepler &operator= (const planeKepler &) = default;
		planeKepler() :_p(constants::nan), _e(constants::nan), _mu(constants::nan) {}

		planeKepler(const value_t &semi_latus_rectum,
					const value_t &eccentricity = 0.,
					const value_t &StandardGravitationalParameter = constants::nan) :_p(semi_latus_rectum), _e(eccentricity), _mu(StandardGravitationalParameter) {}
		planeKepler(const value_t &StandardGravitationalParameter, vec2 pos, vec2 vel) :_mu(StandardGravitationalParameter) {
			auto A2 = vel.squaredNorm() * pos.squaredNorm() - square(pos.dot(vel));
			_p = A2 / StandardGravitationalParameter;
			_e = sqrt(1 + _p * ( vel.squaredNorm() / StandardGravitationalParameter - 2 / pos.norm() ));
		}
		planeKepler(const value_t &StandardGravitationalParameter, vec3 pos, vec3 vel) :planeKepler(StandardGravitationalParameter, vec3_to_vec2(pos), vec3_to_vec2(vel)) {}

		static planeKepler get_by_perigee(const value_t &StandardGravitationalParameter, const value_t &r_perigee, const value_t &v_perigee) {
			auto e_add_one = r_perigee * v_perigee * v_perigee / StandardGravitationalParameter;
			return planeKepler(
				r_perigee * e_add_one,
				e_add_one - 1,
				StandardGravitationalParameter
			);
		}

		orbit_type type() const {
			if ( e() < 0 || ( p() < 0 || ( e() != 1 && p() == 0 ) ) || ( mu() <= 0 && !std::isnan(mu()) ) ) return ERROR;
			if ( e() < 1 ) return ellipse;
			if ( e() > 1 ) return hyperbola;
			return p() == 0 ? line : parabola;
		}
		bool type_in(const int &filter) const { return ( filter & type() ) != 0; }
		bool is_ellipse() const { return type() == ellipse; }
		bool is_hyperbola() const { return type() == hyperbola; }
		bool is_parabola() const { return type() == parabola; }
		bool is_line() const { return type() == line; }

		value_t e() const { return _e; }
		value_t p() const { return _p; }
		value_t mu() const { return _mu; }
		value_t eccentricity() const { return e(); }
		value_t semi_latus_rectum() const { return p(); }
		value_t standard_gravitational_parameter() const { return mu(); }

		value_t a() const { return p() / ( 1 - square(e()) ); }
		value_t H() const { return ( square(e()) - 1 ) * mu() / ( 2 * p() ); }
		vec3 h_vec3() const { return vec3(0, 0, 1) * norm_h(); }
		value_t norm_h() const { return sqrt(squaredNorm_h()); }
		value_t squaredNorm_h() const { return p() * mu(); }
		value_t energy() const { return H(); }
		value_t semi_major_axes() const { return a(); }
		value_t semi_minor_axes() const { return a() * sqrt(abs(1 - square(e()))); }
		value_t Hamilton_constant() const { return H(); }
		value_t norm_of_angular_momentum() const { return norm_h(); }
		vec3 angular_momentum() const { return h_vec3(); }

		value_t r_min() const { return p() / ( 1 + e() ); }
		value_t r_max() const { return type() == ellipse ? p() / ( 1 - e() ) : constants::inf; }
		value_t perigee_distance() const { return r_min(); }
		value_t apogee_distance() const { return r_max(); }

		value_t LRL_norm() const { return e() * mu(); }
		vec2 LRL_vec() const { return vec2(1, 0) * LRL_norm(); }
		vec3 LRL_vec3() const { return vec3(1, 0, 0) * LRL_norm(); }

		value_t T() const { return type() == ellipse ? constants::two_pi * sqrt(cube(a()) / mu()) : constants::nan; }
		value_t n() const { return type_in(ellipse | hyperbola) ? sqrt(mu() / cube(abs(a()))) : constants::nan; }
		value_t vc() const { return type() == ellipse ? sqrt(mu() / a()) : constants::nan; }
		value_t vinf() const { return type() == hyperbola ? sqrt(-mu() / a()) : constants::nan; }
		value_t sqrt_mu_div_p_cube() const { return sqrt(mu() / cube(p())); }
		value_t orbital_period() const { return T(); }
		value_t period() const { return T(); }
		value_t average_orbital_angular_speed() const { return n(); }
		value_t circle_average_orbital_speed() const { return vc(); }
		value_t hyperbolic_excess_velocity() const { return vinf(); }

		value_t v_by_r(const value_t &r) const { return sqrt(mu() * ( 2 / r + ( square(e()) - 1 ) / p() )); }
		value_t r_by_v(const value_t &v) const { return 2 * p() * mu() / ( mu() * ( 1 - square(e()) ) + p() * square(v) ); }

		value_t r(const value_t &true_anomaly)const { return p() / ( 1 + e() * cos(true_anomaly) ); }
		vec2 r_vec(const value_t &true_anomaly)const { return vec2(cos(true_anomaly), sin(true_anomaly)) * r(true_anomaly); }
		vec3 r_vec3(const value_t &true_anomaly)const { return vec3(cos(true_anomaly), sin(true_anomaly), 0) * r(true_anomaly); }

		vec2 r_vec_by_universal_anomaly(const value_t &universal_anomaly)const {

		}

		value_t anomaly_mean_by_true(const value_t &true_anomaly)const {
			return anomaly_mean_by_eccentric(anomaly_eccentric_by_true(true_anomaly));
		}
		value_t anomaly_mean_by_eccentric(const value_t &eccentric_anomaly)const {
			if ( type() == ellipse )
				return eccentric_anomaly - e() * sin(eccentric_anomaly);
			if ( type() == hyperbola )
				return e() * sinh(eccentric_anomaly) - eccentric_anomaly;
			return constants::nan;
		}
		value_t anomaly_eccentric_by_mean(const value_t &mean_anomaly) const;
		value_t anomaly_eccentric_by_true(const value_t &true_anomaly)const {
			if ( type() == ellipse )
				return 2 * atan(sqrt(( 1 - e() ) / ( 1 + e() )) * tan(true_anomaly / 2));
			if ( type() == hyperbola )
				return 2 * atanh(sqrt(( e() - 1 ) / ( e() + 1 )) * tan(true_anomaly / 2));
			return constants::nan;
		}
		value_t anomaly_true_by_eccentric(const value_t &eccentric_anomaly)const {
			if ( type() == ellipse )
				return 2 * atan(sqrt(( 1 + e() ) / ( 1 - e() )) * tan(eccentric_anomaly / 2));
			if ( type() == hyperbola )
				return 2 * atan(sqrt(( e() + 1 ) / ( e() - 1 )) * tanh(eccentric_anomaly / 2));
			return constants::nan;
		}
		value_t anomaly_true_by_mean(const value_t &mean_anomaly) const {
			return anomaly_true_by_eccentric(anomaly_eccentric_by_mean(mean_anomaly));
		}

		value_t anomaly_universal_by_eccentric(const value_t &eccentric_anomaly)const {
			if ( type_in(ellipse | hyperbola) )
				return sqrt(abs(a())) * eccentric_anomaly;
			return constants::nan;
		}
		value_t anomaly_universal_by_true(const value_t &true_anomaly)const {
			if ( type() == parabola ) {
				return norm_h() / sqrt(mu()) * tan(true_anomaly / 2.);
			}
			if ( type_in(ellipse | hyperbola) )
				return anomaly_universal_by_eccentric(anomaly_eccentric_by_true(true_anomaly));
			return constants::nan;
		}


		value_t dt_by_true_anomaly(const value_t &true_anomaly)const {
			if ( type() == parabola ) {
				auto tanv2 = tan(true_anomaly / 2);
				return ( tanv2 + cube(tanv2) / 3 ) / ( 2 * sqrt_mu_div_p_cube() );
			}
			if ( type_in(ellipse | hyperbola) )
				return dt_by_eccentric_anomaly(anomaly_eccentric_by_true(true_anomaly));
			return constants::nan;
		}
		value_t dt_by_eccentric_anomaly(const value_t &eccentric_anomaly)const { return dt_by_mean_anomaly(anomaly_mean_by_eccentric(eccentric_anomaly)); }
		value_t dt_by_mean_anomaly(const value_t &mean_anomaly)const { return mean_anomaly / n(); }
		value_t dt_by_universal_anomaly(const value_t &universal_anomaly)const {
			auto chi2 = square(universal_anomaly);
			auto z = ( 1 - square(e()) ) * chi2 / p();
			return universal_anomaly / sqrt(mu()) * ( r_min() + e() * chi2 * stumpff::S(z) );
		}

		value_t true_anomaly_by_dt(const value_t &dt)const {
			if ( type() == parabola ) {
				auto tmp = 3 * sqrt_mu_div_p_cube() * dt;
				tmp = cbrt(tmp + sqrt(tmp * tmp + 1));
				return 2 * atan(tmp - 1 / tmp);
			}
			if ( type_in(ellipse | hyperbola) )
				return anomaly_true_by_eccentric(eccentric_anomaly_by_dt(dt));
			return constants::nan;
		}
		value_t eccentric_anomaly_by_dt(const value_t &dt)const { return anomaly_eccentric_by_mean(mean_anomaly_by_dt(dt)); }
		value_t mean_anomaly_by_dt(const value_t &dt)const { return n() * dt; }
		value_t universal_anomaly_by_dt(const value_t &dt)const;
	};
}