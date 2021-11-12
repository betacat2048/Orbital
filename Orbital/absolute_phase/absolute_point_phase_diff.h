#pragma once
#include "../util.h"
#include "../relatively_phase/relatively_point_phase.h"
#include "absolute_dirct_phase.h"

namespace orbital::absolute_phase {
	class point_diff : public virtual relatively_phase::point {
	protected:
		dirct_ptr refer_dirct; // the reference direction of *this (the differece between point would be repersent under such direction)
	public:
		// default copy constructors
		point_diff(point_diff &&) = default;
		point_diff(const point_diff &) = default;
		point_diff &operator=(point_diff &&) = default;
		point_diff &operator=(const point_diff &) = default;
		virtual ~point_diff() = default;


		// create a point_diff which is rel_pos when repersent under refer_dirct
		point_diff(const dirct_ptr &refer_dirct, relatively_phase::point &&rel_pos) :refer_dirct(refer_dirct), point(std::move(rel_pos)) { }
		// create a point_diff which is rel_pos when repersent under refer_dirct
		point_diff(const dirct_ptr &refer_dirct, const relatively_phase::point &rel_pos = relatively_phase::point::zero()) :point_diff(refer_dirct, point(rel_pos)) { }

		// create a point_diff which is rel_pos when repersent under direct::root()
		point_diff(relatively_phase::point &&rel_pos) :point_diff(dirct::root(), std::move(rel_pos)) { }
		// create a point_diff which is rel_pos when repersent under direct::root()
		point_diff(const relatively_phase::point &rel_pos = relatively_phase::point::zero()) :point_diff(point(rel_pos)) { }


		/// <summary> a static method that return a point_diff that means no shift at all</summary>
		static point_diff zero() { return point_diff(); }


		// get the absolute dirct which (*this) refer to. (the differece between point would be repersent under such direction)
		dirct_ptr get_refer_dirct() const { return refer_dirct; }


		// get a new point_diff that is same to (*this), but repersent under new_refer_dirct
		point_diff reduce_under(const dirct_ptr &new_refer_dirct) const {
			return point_diff(new_refer_dirct, relatively_dirct_phase_between(new_refer_dirct, refer_dirct) * ( *this ));
		}


		// shift (*this) with other, this operation won't change this's refer_dirct
		point_diff &operator+=(const point_diff &other) {
			this->relatively_phase::point::operator+=(relatively_dirct_phase_between(refer_dirct, other.refer_dirct) * other); // get the other's rel_pos reduce_under this->refer_dirct, and add to (*this)
			return *this;
		}

		// return a point_diff that repersent (*this) + other ( NOTICE: the new refer_dirct == this->get_refer_dirct() )
		point_diff operator+(const point_diff &other)const { auto tmp = *this;	return tmp += other; }
		// return a point_diff that repersent (*this) + other ( NOTICE: the new refer_dirct == other->get_refer_dirct() )
		point_diff operator+(point_diff &&other)const { return other += *this; }


		// check if two dirct are approximately same
		bool isApprox(const point_diff &other, const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return relatively_phase::point::isApprox(relatively_dirct_phase_between(refer_dirct, other.refer_dirct) * other);
		}
		// check if (*this) is approximately same to root
		bool isApproxZero(const value_t &prec = Eigen::NumTraits<value_t>::dummy_precision()) const { return relatively_phase::point::isApproxZero(); }



		// return a point_diff such that ((*this) + inverse()).isApproxZero() == true
		point_diff inverse()const { return { refer_dirct, this->relatively_phase::point::inverse() }; }

		// shift (*this) with other.inverse(), this operation won't change this's refer_dirct
		point_diff &operator-=(const point_diff &other) { return *this += other.inverse(); }
		// return a point_diff that repersent (*this) - other ( NOTICE: the new refer_dirct == this->get_refer_dirct() )
		point_diff operator-(const point_diff &other)const { auto tmp = *this;	return tmp -= other; }
	};
}