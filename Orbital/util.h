#pragma once
#include <memory>

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace orbital {
	using value_t = long double;

#define vec_helper(n) using vec##n = Eigen::Matrix<value_t, n, 1>
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

	using quaternion = Eigen::Quaternion<value_t>;
	using angleaxis = Eigen::AngleAxis<value_t>;

	inline quaternion make_quaternion(const value_t &angle, const vec3 &polar) { return  quaternion(angleaxis(angle, polar.normalized())); }
	inline quaternion make_quaternion(const value_t &angle, const value_t &x, const value_t &y, const value_t &z) { return make_quaternion(angle, vec3(x, y, z)); }
	inline mat33 crossMat(const vec3& x){
		mat33 res{
			{ 0, -x(2), x(1)},
			{x(2), 0, -x(0)},
			{-x(1), x(0), 0}
		};
		return res;
	}

	class point;
	class dirct;
	class frame;

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
}