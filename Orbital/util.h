#pragma once
#include <tuple>
#include <limits>
#include <memory>

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/math/constants/constants.hpp>

namespace orbital {
	using value_t = long double;
	namespace constants {
		inline constexpr value_t nan = std::numeric_limits<value_t>::quiet_NaN();
		inline constexpr value_t inf = std::numeric_limits<value_t>::infinity();

		inline constexpr value_t pi = boost::math::constants::pi<value_t>();
		inline constexpr value_t two_pi = boost::math::constants::two_pi<value_t>();
		inline constexpr value_t half_pi = boost::math::constants::half_pi<value_t>();

		inline constexpr value_t third_pi = boost::math::constants::third_pi<value_t>();
		inline constexpr value_t two_thirds_pi = boost::math::constants::two_thirds_pi<value_t>();
		inline constexpr value_t four_thirds_pi = boost::math::constants::four_thirds_pi<value_t>();

		inline constexpr value_t quarter_pi = boost::math::constants::quarter_pi<value_t>();
		inline constexpr value_t three_quarters_pi = boost::math::constants::three_quarters_pi<value_t>();

		inline constexpr value_t sixth_pi = boost::math::constants::sixth_pi<value_t>();

		inline constexpr value_t one_div_two_pi = boost::math::constants::one_div_two_pi<value_t>();
		inline constexpr value_t one_div_pi = boost::math::constants::one_div_pi<value_t>();
		inline constexpr value_t two_div_pi = boost::math::constants::two_div_pi<value_t>();

		inline constexpr value_t one_div_root_pi = boost::math::constants::one_div_root_pi<value_t>();
		inline constexpr value_t two_div_root_pi = boost::math::constants::two_div_root_pi<value_t>();
		inline constexpr value_t one_div_root_two_pi = boost::math::constants::one_div_root_two_pi<value_t>();

		inline constexpr value_t root_pi = boost::math::constants::root_pi<value_t>();
		inline constexpr value_t root_half_pi = boost::math::constants::root_half_pi<value_t>();
		inline constexpr value_t root_two_pi = boost::math::constants::root_two_pi<value_t>();

		inline constexpr value_t root_one_div_pi = boost::math::constants::root_one_div_pi<value_t>();
	}

	constexpr value_t epsilon = Eigen::NumTraits<value_t>::dummy_precision();
	inline bool isApproxZero(const value_t &v) { return std::abs(v) < epsilon; }

#define vec_helper(n) using vec##n = Eigen::Matrix<value_t, n, 1>
	vec_helper(2);
	vec_helper(3);
	vec_helper(6);
	vec_helper(9);
#undef vec_helper

#define mat_helper(m, n) using mat##m##n = Eigen::Matrix<value_t, m, n>
	mat_helper(3, 3);
	mat_helper(3, 6);
	mat_helper(3, 9);
	mat_helper(6, 3);
	mat_helper(6, 6);
	mat_helper(6, 9);
	mat_helper(9, 6);
	mat_helper(9, 9);
#undef mat_helper

	const vec3 vec3_NaN = vec3(constants::nan, constants::nan, constants::nan);
	inline vec3 vec2_to_vec3(const vec2 &v) { return vec3(v.x(), v.y(), 0); }
	inline vec2 vec3_to_vec2(const vec3 &v) { return vec2(v.x(), v.y()); }

	using quaternion = Eigen::Quaternion<value_t>;
	using angleaxis = Eigen::AngleAxis<value_t>;

	inline quaternion make_quaternion(const value_t &angle, const vec3 &polar) { return  quaternion(angleaxis(angle, polar.normalized())); }
	inline quaternion make_quaternion(const std::pair<value_t, vec3> &angle_polar) { return make_quaternion(angle_polar.first, angle_polar.second); }
	inline quaternion make_quaternion(const value_t &angle, const value_t &x, const value_t &y, const value_t &z) { return make_quaternion(angle, vec3(x, y, z)); }
	inline mat33 crossMat(const vec3 &x) {
		mat33 res{
			{ 0, -x(2), x(1)},
			{x(2), 0, -x(0)},
			{-x(1), x(0), 0}
		};
		return res;
	}
	inline constexpr auto square(const value_t &x) { return x * x; }
	inline constexpr auto cube(const value_t &x) { return x * x * x; }

	namespace object {
		class point;
		class dirct;
		class frame;
	}

	namespace relatively_phase {
		class point;
		class dirct;
		class frame;
	}

	namespace absolute_phase {
		class point;
		class point_diff; // class repersent the absolute difference between points
		class dirct;
		class frame;

		using point_ptr = std::shared_ptr<const point>;
		using dirct_ptr = std::shared_ptr<const dirct>;
		using frame_ptr = std::shared_ptr<const frame>;
	}

	namespace state {
		class base_state;

		class point;
		class dirct;
		class frame;
	}

	namespace timesystem {
		class timepoint;
		class BarycentricDynamicalTime;
		using time_ptr = std::shared_ptr<const timepoint>;
	}


	// the helper class for enable_inheritable_shared_from_this<T>, which hold the info of shared_ptr
	class inheritable_shared_helper_base : public std::enable_shared_from_this<inheritable_shared_helper_base> {
	public:
		virtual ~inheritable_shared_helper_base() {} // empty virtual deconstructor
	};

	template <class T>
	class enable_inheritable_shared_from_this : virtual public inheritable_shared_helper_base {
	public:
		// share from this
		std::shared_ptr<T> shared_from_this() const {
			return std::dynamic_pointer_cast<T>( inheritable_shared_helper_base::shared_from_this() );
		}

		// downcasted helper method when a derived class doesn't inherit directly
		template <class Down>
		std::shared_ptr<Down> downcasted_shared_from_this() const {
			return std::dynamic_pointer_cast<Down>( inheritable_shared_helper_base::shared_from_this() );
		}
	};
}