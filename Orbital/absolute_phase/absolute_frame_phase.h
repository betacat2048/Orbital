#pragma once
#include "../util.h"
#include "absolute_dirct_phase.h"
#include "absolute_point_phase.h"
#include "../relatively_phase/relatively_frame_phase.h"

namespace orbital::absolute_phase {
	class frame
		: public enable_inheritable_shared_from_this<const frame>, public absolute_phase::point, public absolute_phase::dirct, public relatively_phase::frame {

		frame(frame &&) = default;
		frame(const frame &) = default;

		frame(const dirct_ptr &prev_dirct, relatively_phase::dirct &&rel_direct, const point_ptr &prev_pos, absolute_phase::point_diff &&pos_shift) :
			relatively_phase::dirct(std::move(rel_direct)), // moving construct the relatively_phase::dirct(&&) (the virtual inherit by absolute_phase::dirct)
			relatively_phase::point(std::move(pos_shift)), // moving construct the relatively_phase::point(&&) (the virtual inherit by absolute_phase::point)
			absolute_phase::dirct(prev_dirct, std::move(rel_direct)), // moving construct the absolute_phase::dirct (constructor of relatively_phase::dirct(&&) would be block due to the virtual inherit)
			absolute_phase::point(prev_pos, std::move(pos_shift)), // moving construct the absolute_phase::point (constructor of relatively_phase::point(&&) would be block due to the virtual inherit)
			relatively_phase::frame(std::move(rel_direct), std::move(pos_shift)) { } // construct relatively_phase::frame. Actually, it do nothng as both drict() and point() are blocked, due to virtual inherit

	public:
		virtual ~frame() = default;

		static frame_ptr make_node(const dirct_ptr &prev_dirct, relatively_phase::dirct &&rel_direct, const point_ptr &prev_pos, absolute_phase::point_diff &&pos_shift) {
			return frame_ptr(new frame(prev_dirct, std::move(rel_direct), prev_pos, std::move(pos_shift)));
		}
		static frame_ptr make_node(const dirct_ptr &prev_dirct, const relatively_phase::dirct &rel_direct, const point_ptr &prev_pos, const absolute_phase::point_diff &pos_shift) {
			return make_node(prev_dirct, relatively_phase::dirct(rel_direct), prev_pos, absolute_phase::point_diff(pos_shift));
		}
		static frame_ptr make_node(const dirct_ptr &base, const point_ptr &origin) {
			if ( origin || base )
				return make_node(
					base ? base->get_prev_dirct() : absolute_phase::dirct::root(), base ? static_cast<relatively_phase::dirct>( *base ) : relatively_phase::dirct::identity(),
					origin ? origin->get_prev_pos() : absolute_phase::point::root(), origin ? static_cast<absolute_phase::point_diff>( *origin ) : absolute_phase::point_diff::zero()
				);
			else
				return root();
		}
		static frame_ptr make_node(const frame_ptr &base, const relatively_phase::frame &frame) { return make_node(base, frame, base, absolute_phase::point_diff(base, frame)); }

		// a static method that return the root of all relatively direction
		static frame_ptr root() { return nullptr; }


		relatively_phase::frame different_from(const frame_ptr &new_base = frame::root()) const {
			return relatively_phase::frame(point::different_from(new_base), dirct::different_from(new_base));
		}

		frame_ptr reduce_under(const frame_ptr &new_base = frame::root()) { return make_node(new_base, different_from(new_base)); }

		bool isInertial(const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const { return point::isInertial() && dirct::isInertial(); }

		// check if two point are same (the norm of their difference is less than prec)
		bool isApprox(const frame_ptr &other, const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return dirct::isApprox(other, prec) && point::isApprox(other, prec);
		}

		// check if this is same to the root
		bool isApproxZero(const value_t &prec = Eigen::NumTraits<value_t>::dummy_precision()) const { return dirct::isApproxIdentity(prec) && point::isApproxZero(prec); }
		// check if this is same to the root
		bool isApproxIdentity(const value_t &prec = Eigen::NumTraits<value_t>::dummy_precision()) const { return dirct::isApproxIdentity(prec) && point::isApproxZero(prec); }

	protected:
		frame &operator<<=(const relatively_phase::frame &o) { relatively_phase::frame::operator<<=(o); return *this; }
	public:
		frame operator<<(const relatively_phase::frame &o) const { absolute_phase::frame tmp = *this; tmp <<= o; return tmp; }

	};
}