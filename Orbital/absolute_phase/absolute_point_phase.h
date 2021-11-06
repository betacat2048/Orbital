#pragma once

#include <iostream>

#include "../util.h"
#include "absolute_point_phase_diff.h"

namespace orbital::absolute_phase {
	class point: public absolute_phase::point_diff {
	protected:
		// notice that point would form a tree, and its root is S.S.B. (Solar System Barycenter) reduce_under J2000.
		const point_ptr prev_pos; //pointer refer to the pos_base point, where nullptr is the root
		const size_t depth; //depth of the reference tree(of position), where the root(nullptr) is 0
	public:
		// default copy constructors
		point(point &&) = default;
		point(const point &) = default;
		point &operator=(point &&) = default;
		point &operator=(const point &) = default;
		virtual ~point() = default;


		point(const point_ptr &prev_pos, absolute_phase::point_diff &&pos_shift)
			: relatively_phase::point(pos_shift), prev_pos(prev_pos), point_diff(std::move(pos_shift)), depth(prev_pos ? prev_pos->depth + 1 : 1) {}

		point(const point_ptr &prev_pos, const absolute_phase::point_diff &pos_shift = point_diff::zero())
			: point(prev_pos, point_diff(pos_shift)) {}

		point(const absolute_phase::point_diff &pos_shift = point_diff::zero())
			: point(point::root(), point_diff(pos_shift)) {}

		/// <summary> create a absolute_phase::point from <paramref name = "base_frame"/> and <paramref name = "rel_pos"/> </summary>
		/// <param name='base_frame'> the base frame of the <paramref name = "rel_pos"/> which contain a orign and a direction </param>
		/// <param name='rel_pos'> the relative position (and vel, acc) of point reduce_under <paramref name = "base_frame"/> (repersent in the direction of <paramref name = "base_frame"/>) </param>
		point(const frame_ptr &base_frame, const relatively_phase::point &rel_pos);


		/// <summary> a static method that return a absolute point same to root()'s position (notice this would have depth == 1)</summary>
		static point zero() { return point(); }

		// a static method that return the root of all relatively direction
		static point_ptr root() { return nullptr; }


		// get the shared_ptr to the origin position of (*this)
		point_ptr get_prev_pos() const { return prev_pos; }

		size_t distanceTo(const point_ptr &) const;

		// return the difference between (*this) and (*from), in other words, (*from) + this->point_diff_from(from) repersent same the point repersented byu (*this)
		// Notice: the point_diff returned won't change base, so the choice of its refer_drict implementation-defined behavior
		point_diff point_diff_from(const point_ptr &from = point::root()) const;

		// return a point_diff, which has refer_drict equals new_base->refer_dirct (if the new_base is root(), the result would refer to drict::root())
		point_diff different_from(const point_ptr &new_base) const;
		// return a point_diff, which has refer_drict equals new_base (new_base as std::shared_ptr<const dirct> refer_dirct)
		point_diff different_from(const frame_ptr &new_base) const;

		point_ptr reduce_under(const point_ptr &new_base) { return std::make_shared<const point>(new_base, different_from(new_base)); }
		point_ptr reduce_under(const frame_ptr &new_base);

		// check if two point are same
		bool isApprox(const point_ptr &other, const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return point_diff_from(other).isApproxZero(prec);
		}
		bool isApprox(const point &other, const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return isApprox(std::make_shared<const point>(other), prec);
		}
	private:
		/// <summary> this is a helper method for point_diff_from, it would change the base of <paramref name = "rel_point"/> and update all of three parameter </summary>
		/// <param name='rel_point'>rel_point of a point base on (*ptr) ==> rel_point of the same point but base on *(ptr->prev_dirct.get())</param>
		/// <param name='ptr'>a pointer to the base for rel_drict; updated after call: ptr ==> ptr->prev_dirct.get()</param>
		/// <param name='depth'>the depth of the node refer by ptr; updated after call: depth ==> depth - 1</param>
		static inline void reduce_level(absolute_phase::point_diff &pos_shift, const point *&ptr, size_t &depth) {
			pos_shift = (*ptr) + pos_shift;
			ptr = ptr->prev_pos.get();
			--depth;
		}

	};
}