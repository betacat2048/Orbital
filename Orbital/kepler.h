#pragma once

#include "util.h"

namespace orbital {
	class planeKepler {
	public:
		value_t energy, eccentricity, StandardGravitationalParameter;

		planeKepler(planeKepler &&) = default;
		planeKepler(const planeKepler &) = default;
		planeKepler &operator= (planeKepler &&) = default;
		planeKepler &operator= (const planeKepler &) = default;
		planeKepler() :energy(constants::nan), eccentricity(constants::nan), StandardGravitationalParameter(constants::nan) {}

		planeKepler(const value_t &energy,
					const value_t &eccentricity = 0.,
					const value_t &StandardGravitationalParameter = constants::nan) :energy(energy), eccentricity(eccentricity), StandardGravitationalParameter(StandardGravitationalParameter) {}
		planeKepler(const value_t &StandardGravitationalParameter, vec2 pos, vec2 vel) :StandardGravitationalParameter(StandardGravitationalParameter) {
			energy = vel.squaredNorm() / 2. - StandardGravitationalParameter / pos.norm();
			auto d = pos.dot(vel);
			auto A2 = vel.squaredNorm() * pos.squaredNorm() - d * d;
			eccentricity = sqrt(1 + 2 * energy * A2 / ( StandardGravitationalParameter * StandardGravitationalParameter ));
		}

		value_t H() const { return energy; }
		value_t e() const { return eccentricity; }
		value_t mu() const { return StandardGravitationalParameter; }
		value_t a() const { return -mu() / ( 2. * H() ); }
		value_t p() const { return a() * ( 1 - e() * e() ); }
		value_t squared_A() const { return p() * mu(); }
		value_t A() const { return sqrt(squared_A()); }
		vec3 A_vec3() const { return vec3(0, 0, 1) * A(); }

		value_t r_min() const { return p() / ( 1 + e() ); }
		value_t r_max() const { return 0 <= e() && e() < 1 ? p() / ( 1 - e() ) : constants::inf; }

		value_t LRL_norm() const { return e() * mu(); }
		vec2 LRL_vec() const { return vec2(1, 0) * LRL_norm(); }
		vec3 LRL_vec3() const { return vec3(1, 0, 0) * LRL_norm(); }

		value_t T() const { return constants::two_pi * sqrt(cube(abs(a())) / mu()); }
		value_t n() const { return sqrt(mu() / cube(abs(a()))); }

		value_t v_by_r(const value_t &r) const { return sqrt(mu() * ( 2 / r - 1 / a() )); }
		value_t vc() const { return sqrt(mu() / a()); }

		value_t r(const value_t &true_anomaly)const { return p() / ( 1 + e() * cos(true_anomaly) ); }
		vec2 r_vec(const value_t &true_anomaly)const { return vec2(cos(true_anomaly), sin(true_anomaly)) * r(true_anomaly); }
		vec3 r_vec3(const value_t &true_anomaly)const { return vec3(cos(true_anomaly), sin(true_anomaly), 0) * r(true_anomaly); }

		value_t anomaly_mean_by_true(const value_t &true_anomaly)const {
			return anomaly_mean_by_eccentric(anomaly_eccentric_by_true(true_anomaly));
		}
		value_t anomaly_mean_by_eccentric(const value_t &eccentric_anomaly)const {
			if ( 0 <= e() && e() < 1 )
				return eccentric_anomaly - e() * sin(eccentric_anomaly);
			if ( 1 < e() )
				return e() * sinh(eccentric_anomaly) - eccentric_anomaly;
			return constants::nan;
		}
		value_t anomaly_eccentric_by_mean(const value_t &mean_anomaly) const;
		value_t anomaly_eccentric_by_true(const value_t &true_anomaly)const {
			if ( 0 <= e() && e() < 1 )
				return 2 * atan(sqrt(( 1 - e() ) / ( 1 + e() )) * tan(true_anomaly / 2));
			if ( 1 < e() )
				return 2 * atanh(sqrt(( e() - 1 ) / ( e() + 1 )) * tan(true_anomaly / 2));
			return constants::nan;
		}
		value_t anomaly_true_by_eccentric(const value_t &eccentric_anomaly)const {
			if ( 0 <= e() && e() < 1 )
				return 2 * atan(sqrt(( 1 + e() ) / ( 1 - e() )) * tan(eccentric_anomaly / 2));
			if ( 1 < e() )
				return 2 * atan(sqrt(( e() + 1 ) / ( e() - 1 )) * tanh(eccentric_anomaly / 2));
			return constants::nan;
		}
		value_t anomaly_true_by_mean(const value_t &mean_anomaly) const {
			return anomaly_true_by_eccentric(anomaly_eccentric_by_mean(mean_anomaly));
		}

		value_t t(const value_t &true_anomaly)const {
			if ( e() >= 0 ) {
				if ( e() != 1 )
					return anomaly_mean_by_true(true_anomaly) / n();
				// parabola case ( e==1 )
				auto tanv2 = tan(true_anomaly / 2);
				return ( tanv2 + cube(tanv2) / 3 ) / ( 2 * sqrt(mu() / cube(p())) );
			}
			return constants::nan;
		}
	};
}