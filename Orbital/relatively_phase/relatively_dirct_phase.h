#pragma once

#include <tuple>
#include "../util.h"
#include "relatively_point_phase.h"

namespace orbital::relatively_phase {
	class dirct {
	public:
		quaternion posture;
		vec3 angular_velocity;
		vec3 angular_acceleration;



		dirct(dirct &&) = default;
		dirct(const dirct &) = default;
		//dirct(dirct &&o):posture(std::move(o.posture)), angular_velocity(std::move(o.angular_velocity)), angular_acceleration(std::move(o.angular_acceleration)) { std::cout << "called moving constructor of relatively_phase::dirct" << std::endl; }
		//dirct(const dirct &o):posture(o.posture), angular_velocity(o.angular_velocity), angular_acceleration(o.angular_acceleration) { std::cout << "called copying constructor of relatively_phase::dirct" << std::endl; }
		dirct &operator=(dirct &&) = default;
		dirct &operator=(const dirct &) = default;
		virtual ~dirct() = default;



		/// <summary> create a relatively direction by given <paramref name = "posture"/>, <paramref name = "angular_velocity"/>, and <paramref name = "angular_acceleration"/> </summary>
		/// <param name = 'posture'> a quaternion that repersent the posture of the direction relatively, default is quaternion::Identity() </param>
		/// <param name = 'angular_velocity'> a vec3 that repersent the angular velocity of the direction (base) relatively, default is vec3::Zero() </param>
		/// <param name = 'angular_acceleration'> a vec3 that repersent the angular acceleration of the direction (base) relatively, default is vec3::Zero() </param>
		dirct(const quaternion &posture = quaternion::Identity(), const vec3 &angular_velocity = vec3::Zero(), const vec3 &angular_acceleration = vec3::Zero())
			: posture(posture), angular_velocity(angular_velocity), angular_acceleration(angular_acceleration) { }

		/// <summary> create a relatively direction by given <paramref name = "angle_polar"/>, <paramref name = "angular_velocity"/>, and <paramref name = "angular_acceleration"/> </summary>
		/// <param name = 'angle_polar'> a std::pair that repersent the posture, where the first is angle to rotate, and second is the polar of rotation </param>
		/// <param name = 'angular_velocity'> a vec3 that repersent the angular velocity of the direction (base) relatively, default is vec3::Zero() </param>
		/// <param name = 'angular_acceleration'> a vec3 that repersent the angular acceleration of the direction (base) relatively, default is vec3::Zero() </param>
		dirct(const std::pair<value_t, vec3> &angle_polar, const vec3 &angular_velocity = vec3::Zero(), const vec3 &angular_acceleration = vec3::Zero())
			: dirct(make_quaternion(angle_polar.first, angle_polar.second), angular_velocity, angular_acceleration) { }



		/// <summary> a static method that return a identity relatively direction (the identity element under *)</summary>
		static dirct identity() { return dirct(); }



		// check  if *this is exactly equal to other (NOTICE: this method usually return false due to float-point error)
		bool operator==(const dirct &other) const {
			return posture == other.posture && angular_velocity == other.angular_velocity && angular_acceleration == other.angular_acceleration;
		}

		// check if *this is approximately equal to other (the posture's angle different between *this and other < prec, and the norm of difference of angular_velocity and angular_acceleration < prec)
		bool isApprox(const dirct &other, const value_t &prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return posture.angularDistance(other.posture) < prec &&
				( angular_velocity - other.angular_velocity ).squaredNorm() + ( angular_acceleration - other.angular_acceleration ).squaredNorm() < prec * prec;
		}
		// check if *this is approximately equal to identity
		bool isApproxIdentity(const value_t &prec = Eigen::NumTraits<value_t>::dummy_precision()) const { return isApprox(relatively_phase::dirct::identity(), prec); }



		// concatenates two dirct
		dirct &operator*=(const dirct &other) {
			auto trans = posture.toRotationMatrix(); // transfer matrix
			auto omega = trans * other.angular_velocity;

			// angular_acceleration = local_angular_acceleration + base_angular_velocity \cross angular_velocity
			angular_acceleration += trans * other.angular_acceleration + angular_velocity.cross(omega);
			// angular_velocity = local_angular_velocity + base_angular_velocity
			angular_velocity += omega;
			// posture = base_posture * local_posture
			posture *= other.posture;

			return *this;
		}
		// concatenates two dirct NOTICE: this operator satisfy associative law but not the commutative law
		dirct operator*(const dirct &other) const { auto tmp = *this; return tmp *= other; }



		// get inverse rotation of (*this) (for any x, this->inverse() * (*this) * x == x)
		dirct inverse() const {
			auto new_posture = posture.inverse();
			auto trans = new_posture.toRotationMatrix();

			return dirct(new_posture, -trans * angular_velocity, -trans * angular_acceleration);
		}



		// act *this on the point x, and return the new point 
		//     if *this is the base1 repersent under base0 and x is the position of a point repersent under base1,
		//     then (*this) * x is the position of x repersent under base0
		point operator*(const point &x) const {
			auto trans = posture.toRotationMatrix();

			vec3 pos = trans * x.position;
			vec3 local_vel = trans * x.velocity;
			vec3 base_vel = angular_velocity.cross(pos);
			vec3 vel = local_vel + base_vel;
			vec3 acc = trans * x.acceleration			// by motion under the trans_base
				+ 2 * angular_velocity.cross(local_vel)	// Coriolis acceleration
				+ angular_acceleration.cross(pos)		// Euler acceleration
				+ angular_velocity.cross(base_vel);		// Centrifugal acceleration

			return point(pos, vel, acc);
		}
		point trans_pos(const point &x) const { return ( *this ) * x; }



		// get the pos to pos sub block of phase transfer matrix
		mat33 pos_pos() const { return posture.toRotationMatrix(); }

		// get the vel to pos sub block of phase transfer matrix (zero) (NOT recommended)
		mat33 pos_vel() const { return mat33::Zero(); }
		// get the acc to pos sub block of phase transfer matrix (zero) (NOT recommended)
		mat33 pos_acc() const { return mat33::Zero(); }
		// get the pos to vel sub block of phase transfer matrix (velocity cased by rotation of trans_base) (NOT recommended, this is only a part of velocity transfer)
		mat33 vel_pos() const { return crossMat(angular_velocity) * posture.toRotationMatrix(); }
		// get the vel to vel sub block of phase transfer matrix (velocity cased by motion on the trans_base) (NOT recommended, this is only a part of velocity transfer)
		mat33 vel_vel() const { return posture.toRotationMatrix(); }
		// get the vel to vel sub block of phase transfer matrix (zero)(NOT recommended)
		mat33 vel_acc() const { return mat33::Zero(); }
		// get the pos to acc sub block of phase transfer matrix (Euler acceleration + Centrifugal acceleration) (NOT recommended, this is only a part of acceleration transfer)
		mat33 acc_pos() const { return ( crossMat(angular_acceleration) + crossMat(angular_velocity) * crossMat(angular_velocity) ) * posture.toRotationMatrix(); }
		// get the vel to acc sub block of phase transfer matrix (Coriolis acceleration) (NOT recommended, this is only a part of acceleration transfer)
		mat33 acc_vel() const { return 2 * crossMat(angular_velocity) * posture.toRotationMatrix(); }
		// get the acc to acc sub block of phase transfer matrix (by motion under the trans_base) (NOT recommended, this is only a part of acceleration transfer)
		mat33 acc_acc() const { return posture.toRotationMatrix(); }

		// get the posvel to pos sub block of phase transfer matrix
		mat36 pos_posvel() const { mat36 tmp; tmp.block<3, 3>(0, 0) = pos_pos(); tmp.block<3, 3>(0, 3) = pos_vel(); return tmp; }
		// get the posvel to vel sub block of phase transfer matrix
		mat36 vel_posvel() const { mat36 tmp; tmp.block<3, 3>(0, 0) = vel_pos(); tmp.block<3, 3>(0, 3) = vel_vel(); return tmp; }
		// get the posvel to posvel sub block of phase transfer matrix
		mat66 posvel_posvel() const {
			mat66 tmp;
			mat33 trans = posture.toRotationMatrix();
			tmp.block<3, 3>(0, 0) = trans; tmp.block<3, 3>(0, 3) = pos_vel();
			tmp.block<3, 3>(3, 0) = vel_pos(); tmp.block<3, 3>(3, 3) = trans;
			return tmp;
		}

		// get the posvelacc to pos sub block of phase transfer matrix
		mat39 pos_posvelacc() const { mat39 tmp; tmp.block<3, 3>(0, 0) = pos_pos(); tmp.block<3, 3>(0, 3) = pos_vel(); tmp.block<3, 3>(0, 6) = pos_acc(); return tmp; }
		// get the posvelacc to vel sub block of phase transfer matrix
		mat39 vel_posvelacc() const { mat39 tmp; tmp.block<3, 3>(0, 0) = vel_pos(); tmp.block<3, 3>(0, 3) = vel_vel(); tmp.block<3, 3>(0, 6) = vel_acc(); return tmp; }
		// get the posvelacc to acc sub block of phase transfer matrix
		mat39 acc_posvelacc() const {
			mat39 tmp;
			mat33 trans = posture.toRotationMatrix();
			mat33 angular_velocity_cross = crossMat(angular_velocity);
			mat33 angular_velocity_cross_trans = angular_velocity_cross * trans;

			tmp.block<3, 3>(0, 0) = crossMat(angular_acceleration) * trans + angular_velocity_cross * angular_velocity_cross_trans;
			tmp.block<3, 3>(0, 3) = 2 * angular_velocity_cross_trans;
			tmp.block<3, 3>(0, 6) = trans;

			return tmp;
		}
		// get the posvelacc to posvel sub block of phase transfer matrix
		mat69 posvel_posvelacc() const {
			mat69 tmp;
			mat33 trans = posture.toRotationMatrix();
			tmp.block<3, 3>(0, 0) = trans; tmp.block<3, 3>(0, 3) = pos_vel(); tmp.block<3, 3>(0, 6) = pos_acc();
			tmp.block<3, 3>(3, 0) = vel_pos(); tmp.block<3, 3>(3, 3) = trans; tmp.block<3, 3>(3, 6) = vel_acc();
			return tmp;

		}
		// get the posvelacc to posvelacc sub block of phase transfer matrix
		mat99 posvelacc_posvelacc() const {
			mat99 tmp;
			mat33 trans = posture.toRotationMatrix();
			mat33 angular_velocity_cross = crossMat(angular_velocity);
			mat33 angular_velocity_cross_trans = angular_velocity_cross * trans;

			tmp.block<3, 3>(0, 0) = trans;
			tmp.block<3, 3>(0, 3) = mat33::Zero();
			tmp.block<3, 3>(0, 6) = mat33::Zero();

			tmp.block<3, 3>(3, 0) = angular_velocity_cross_trans;
			tmp.block<3, 3>(3, 3) = trans;
			tmp.block<3, 3>(3, 6) = mat33::Zero();

			tmp.block<3, 3>(6, 0) = crossMat(angular_acceleration) * trans + angular_velocity_cross * angular_velocity_cross_trans;
			tmp.block<3, 3>(6, 3) = 2 * angular_velocity_cross_trans;
			tmp.block<3, 3>(6, 6) = trans;

			return tmp;
		}
	};

}