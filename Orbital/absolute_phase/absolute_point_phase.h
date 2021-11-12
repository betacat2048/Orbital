#pragma once

#include <iostream>

#include "../util.h"
#include "absolute_point_phase_diff.h"

namespace orbital::absolute_phase {
	class point : public enable_inheritable_shared_from_this<const point>, protected absolute_phase::point_diff {
	protected:
		// notice that point would form a tree, and its root is S.S.B. (Solar System Barycenter) under J2000.
		const point_ptr prev_pos; //pointer refer to the pos_base point, where nullptr is the root
		const size_t depth; //depth of the reference tree(of position), where the root(nullptr) is 0

		// default copy constructors
		point(point &&) = default;
		point(const point &) = default;

		// build an absolute point by take pos_shift (relatively_phase::point_diff) shift on the prev_pos
		point(const point_ptr &prev_pos, absolute_phase::point_diff &&pos_shift)
			: relatively_phase::point(pos_shift), prev_pos(prev_pos), point_diff(std::move(pos_shift)), depth(prev_pos ? prev_pos->depth + 1 : 1) { }
	public:
		virtual ~point() = default;


		// a static method that return the root of all relatively direction
		static point_ptr root() { return nullptr; }

		// return the distance from p to root
		static size_t distanceToRoot(const point_ptr &p) { return  p ? p->depth : 0; }



		// build an absolute point that equals pos_shift shifted from prev_pos
		static point_ptr make_node(const point_ptr &prev_pos, absolute_phase::point_diff &&pos_shift) { return point_ptr(new point(prev_pos, std::move(pos_shift))); }

		// build an absolute point that equals pos_shift shifted from prev_pos
		static point_ptr make_node(const point_ptr &prev_pos, const absolute_phase::point_diff &pos_shift = point_diff::zero()) { return make_node(prev_pos, point_diff(pos_shift)); }

		// build an absolute point that equals pos_shift shifted from root()
		static point_ptr make_node(const absolute_phase::point_diff &pos_shift = point_diff::zero()) { return make_node(point::root(), point_diff(pos_shift)); }

		/// <summary> create a absolute_phase::point from <paramref point_name = "base_frame"/> and <paramref point_name = "rel_pos"/> </summary>
		/// <param point_name='base_frame'> the base frame of the <paramref point_name = "rel_pos"/> which contain a orign and a direction </param>
		/// <param point_name='rel_pos'> the relative position (and vel, acc) of point reduce_under <paramref point_name = "base_frame"/> (repersent in the direction of <paramref point_name = "base_frame"/>) </param>
		static point_ptr make_node(const frame_ptr &base_frame, const relatively_phase::point &rel_pos);


		// build an absolute point that equals pos_shift shifted from (*this)
		point_ptr make_next_node(absolute_phase::point_diff &&pos_shift) const { return make_node(shared_from_this(), std::move(pos_shift)); }
		// build an absolute point that equals pos_shift shifted from (*this)
		point_ptr make_next_node(const absolute_phase::point_diff &pos_shift) const { return make_next_node(absolute_phase::point_diff(pos_shift)); }


		// get the shared_ptr to the origin position of (*this)
		point_ptr get_prev_pos() const { return prev_pos; }



		// get the distance between two dirct (if distance between two absolute dirct is too large, the floating-error maybe significant)
		size_t distanceTo(const point_ptr &) const;



		// return the difference between (*this) and (*from), in other words, make_node(from, this->point_diff_from(from)) repersent same the point repersented by (*this)
		//    Notice: the refer_drict of returned point_diff is implementation-defined behavior
		point_diff point_diff_from(const point_ptr &from = point::root()) const;


		// return a point_diff repersent the difference from *from to *this, which has refer_drict equals new_base->refer_dirct (if the new_base is root(), the result would refer to drict::root())
		point_diff different_from(const point_ptr &new_base) const;

		// return a point_diff repersent the difference from *from to *this, which has refer_drict equals static_cast<dirct_ptr>(new_base)
		point_diff different_from(const frame_ptr &new_base) const;



		// build a new absolute_phase::point, which repersent the same points as (*this), but has get_prev_pos() == new_base
		point_ptr reduce_under(const point_ptr &new_base) { return make_node(new_base, different_from(new_base)); }

		// build a new absolute_phase::point, which repersent the same points as (*this), but has get_prev_pos() == static_cast<point_ptr>(new_base)
		point_ptr reduce_under(const frame_ptr &new_base);



		// return if (*this) is intertial point (return true if the norm of the acceleration of (*this) under S.S.B is less than prec)
		bool isInertial(const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return point_diff_from().acc_norm() < prec;
		}

		// check if two point are same (the norm of their difference is less than prec)
		bool isApprox(const point_ptr &other, const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return point_diff_from(other).isApproxZero(prec);
		}

		// check if this is same to the root
		bool isApproxZero(const value_t &prec = Eigen::NumTraits<value_t>::dummy_precision()) const { return isApprox(root(), prec); }

	private:
		/// <summary> this is a helper method for point_diff_from, it would change the base of <paramref point_name = "rel_point"/> and update all of three parameter </summary>
		/// <param point_name='rel_point'>rel_point of a point base on (*ptr) ==> rel_point of the same point but base on *(ptr->prev_dirct.get())</param>
		/// <param point_name='ptr'>a pointer to the base for rel_drict; updated after call: ptr ==> ptr->prev_dirct.get()</param>
		/// <param point_name='depth'>the depth of the node refer by ptr; updated after call: depth ==> depth - 1</param>
		static inline void reduce_level(absolute_phase::point_diff &pos_shift, const point *&ptr, size_t &depth) {
			pos_shift = ( *ptr ) + pos_shift;
			ptr = ptr->prev_pos.get();
			--depth;
		}
	};
}