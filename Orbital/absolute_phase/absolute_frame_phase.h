#pragma once
#include "../util.h"
#include "absolute_dirct_phase.h"
#include "absolute_point_phase.h"
#include "../relatively_phase/relatively_frame_phase.h"

namespace orbital::absolute_phase {
	class frame
		: public virtual absolute_phase::point, public virtual absolute_phase::dirct, public relatively_phase::frame {
	public:
		frame(frame &&) = default;
		frame(const frame &) = default;
		frame &operator=(frame &&) = default;
		frame &operator=(const frame &) = default;
		virtual ~frame() = default;

		frame(point &&origin, dirct &&base):
			relatively_phase::dirct(std::move(base)), // moving construct the relatively_phase::dirct (the virtual inherit by absolute_phase::dirct)
			relatively_phase::point(std::move(origin)), // moving construct the relatively_phase::point (the virtual inherit by absolute_phase::point)
			point(std::move(origin)), // moving construct the absolute_phase::point (constructor of relatively_phase::point would be block due to the virtual inherit)
			dirct(std::move(base)),// moving construct the absolute_phase::dirct (constructor of relatively_phase::dirct would be block due to the virtual inherit)
			relatively_phase::frame(std::move(origin), std::move(base)) {} // construct relatively_phase::frame. Actually, it do nothng as both drict() and point() are blocked, due to virtual inherit

		frame(const point &origin = point::zero(), const dirct &base = dirct::identity()):frame(point(origin), dirct(base)) {}


		frame(dirct &&base, point &&origin):frame(std::move(origin), std::move(base)) {}
		frame(const dirct &base, const point &origin = point::zero()):frame(origin, base) {}


		frame(const point_ptr &origin, const dirct_ptr &base = dirct::root()):frame(origin ? *origin : point::zero(), base ? *base : dirct::identity()) {}
		frame(const dirct_ptr &base, const point_ptr &origin = point::root()):frame(origin, base) {}

		frame(const frame_ptr &base, const relatively_phase::frame &frame): frame(point(base, frame), dirct(base, frame)) {}


		// a static method that return the root of all relatively direction
		static frame_ptr root() { return nullptr; }


		relatively_phase::frame different_from(const frame_ptr &new_base = frame::root()) const {
			return relatively_phase::frame(point::different_from(new_base), dirct::different_from(new_base));
		}

		frame_ptr reduce_under(const frame_ptr &new_base = frame::root()) { return std::make_shared<const frame>(new_base, different_from(new_base)); }


		frame &operator<<=(const relatively_phase::frame &o) { relatively_phase::frame::operator<<=(o); return *this; }
		frame operator<<(const relatively_phase::frame &o) const { auto tmp = *this; return tmp *= o; }
	};
}