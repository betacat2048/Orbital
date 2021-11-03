#pragma once

#include "../util.h"
#include "relatively_point_phase.h"
#include "relatively_dirct_phase.h"

namespace orbital::relatively_phase {
	class frame
		: public virtual relatively_phase::point, public virtual relatively_phase::dirct {
	public:
		frame(frame &&) = default;
		frame(const frame &) = default;
		frame &operator=(frame &&) = default;
		frame &operator=(const frame &) = default;
		virtual ~frame() = default;

		frame(point &&pos, dirct &&rot): point(std::move(pos)), dirct(std::move(rot)) {}
		frame(const point &pos = point::zero(), const dirct &rot = dirct::identity()): frame(point(pos), dirct(rot)) {}
		
		frame(dirct &&rot, point &&pos): frame(std::move(pos), std::move(rot)) {}
		frame(const dirct &rot, const point &pos = point::zero()): frame(point(pos), dirct(rot)) {}

		/// <summary> a static method that return a relatively frame that not change any thing (the identity element under <<)</summary>
		static frame zero() { return frame(); }
		/// <summary> a static method that return a relatively frame that not change any thing (the identity element under <<)</summary>
		static frame identity() { return frame(); }


		// take act on x
		point operator<<(const point &x) { return (*this) + (*this) * x; }
		// take act on x
		point trans_pos(const point &x) const {
			return this->point::trans_pos(this->dirct::trans_pos(x));
		}

		// concatenates frame relate
		frame &operator<<=(const frame &other) {
		// x = (*this) + (*this) * (other + other * x_)
		// so x
			(*this) += (*this) * static_cast<point>(other);
			(*this) *= other;
			return *this;
		}
		frame operator<<(const frame &other) const { auto tmp = *this; return tmp <<= other; }

		// return the inverse of (*this) for any x, this->inverse() << (*this) << x == x
		frame inverse() const {
			// this->inverse() + this->inverse() * ((*this) + (*this) * x) == x
			// (this->inverse() + this->inverse() * (*this)) + (this->inverse() * (*this)) * x == x
			// so we can get:
			// this->inverse().base_view() == his->dirct::inverse()
			// this->inverse().position_view() == (this->inverse().base_view() * this->position_view()).inverse()
			auto rot = this->dirct::inverse();
			return frame((rot * static_cast<point>(*this)).inverse(), rot);
		}
	};
}