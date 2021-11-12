#pragma once
#include "../util.h"

namespace orbital::relatively_phase {
	class point {
	public:
		vec3 position; // position of the point
		vec3 velocity; // velocity of the point
		vec3 acceleration; // acceleration of the point



		point(point &&) = default;
		point(const point &) = default;
		//point(point &&o):position(std::move(o.position)), velocity(std::move(o.velocity)), acceleration(std::move(o.acceleration)) { std::cout << "called moving constructor of relatively_phase::point" << std::endl; }
		//point(const point &o):position(o.position), velocity(o.velocity), acceleration(o.acceleration) { std::cout << "called copying constructor of relatively_phase::point" << std::endl; }
		point &operator=(point &&) = default;
		point &operator=(const point &) = default;
		virtual ~point() = default;



		/// <summary> create a relatively point by given <paramref name = "position"/>, <paramref name = "velocity"/>, and <paramref name = "acceleration"/> </summary>
		/// <param name = 'position'> a vec3 that repersent the position of the point relatively, default is vec3::Zero() </param>
		/// <param name = 'velocity'> a vec3 that repersent the velocity of the point relatively, default is vec3::Zero() </param>
		/// <param name = 'acceleration'> a vec3 that repersent the acceleration of the point relatively, default is vec3::Zero() </param>
		point(const vec3 &position = vec3::Zero(), const vec3 &velocity = vec3::Zero(), const vec3 &acceleration = vec3::Zero())
			:position(position), velocity(velocity), acceleration(acceleration) { }

		/// <summary> create a relatively point by given <paramref name = "posvel"/></summary>
		/// <param name = 'posvel'> a vec6 that repersent the position and velocity of the point relatively, the acceleration is vec3::Zero() </param>
		point(const vec6 &posvel) : point(posvel.segment<3>(0), posvel.segment<3>(3)) { }

		/// <summary> create a relatively point by given <paramref name = "posvelacc"/></summary>
		/// <param name = 'posvelacc'> a vec6 that repersent the position, velocity and acceleration of the point relatively</param>
		point(const vec9 &posvelacc) : point(posvelacc.segment<3>(0), posvelacc.segment<3>(3), posvelacc.segment<3>(6)) { }



		/// <summary> a static method that return a zero relatively point (the identity element under +)</summary>
		static point zero() { return point(); }



		// check  if *this is exactly equal to other (NOTICE: this method usually return false due to float-point error)
		bool operator==(const point &other) const { return position == other.position && velocity == other.velocity && acceleration == other.acceleration; }

		// check if *this is approximately equal to zero (posvelacc().norm() < prec)
		bool isApproxZero(const value_t &prec = Eigen::NumTraits<value_t>::dummy_precision()) const { 
			return  position.squaredNorm() + velocity.squaredNorm() + acceleration.squaredNorm() < prec * prec;
		}

		// check if *this is approximately equal to other ((*this - other).posvelacc().norm() < prec)
		bool isApprox(const point &other, const value_t &prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return ( other - *this ).isApproxZero(prec);
		}



		// add up two point (user should ensure these two point are under same trans_base)
		point &operator+=(const point &other) {
			position += other.position;
			velocity += other.velocity;
			acceleration += other.acceleration;

			return *this;
		}
		// satisfy the commutative law
		point operator+(point &&other) const { return other += *this; }
		point operator+(const point &other) const { auto tmp = *this; return tmp += other; }


		// subtract with another point (user should ensure these two point are under same trans_base)
		point &operator-=(const point &other) {
			position -= other.position;
			velocity -= other.velocity;
			acceleration -= other.acceleration;

			return *this;
		}
		point operator-(const point &other) const { auto tmp = *this; return tmp -= other; }



		point trans_pos(const point &x) const { return *this + x; }



		// get the negtive of (*this)
		point inverse() const { return point(-position, -velocity, -acceleration); }
		// get the negtive of (*this)
		point operator-() const { return this->inverse(); }



		// get the norm of position
		value_t pos_norm() const { return position.norm(); }
		// get the norm of velocity
		value_t vel_norm() const { return velocity.norm(); }
		// get the norm of acceleration
		value_t acc_norm() const { return acceleration.norm(); }


		// get the distance of this position (same to pos_norm())
		value_t distance() const { return pos_norm(); }
		// get the speed of this position (same to vel_norm())
		value_t speed() const { return vel_norm(); }



		// get the unit vector in direction of position
		vec3 pos_dir() const { return position.normalized(); }
		// get the unit vector in direction of velocity
		vec3 vel_dir() const { return velocity.normalized(); }
		// get the unit vector in direction of acceleration
		vec3 acc_dir() const { return acceleration.normalized(); }



		vec3 pos() const { return position; }
		vec3 vel() const { return velocity; }
		vec3 acc() const { return acceleration; }
		vec6 posvel() const { vec6 res; res.segment<3>(0) = position; res.segment<3>(3) = velocity; return res; }
		vec9 posvelacc() const { vec9 res; res.segment<3>(0) = position; res.segment<3>(3) = velocity; res.segment<3>(6) = acceleration; return res; }
	};
}