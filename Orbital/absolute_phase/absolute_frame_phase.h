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
		virtual ~frame() = default;

		frame(const point &origin = point::zero(),
			const dirct &base = dirct::identity())
			:point(origin),
			dirct(base),
			relatively_phase::frame(origin, base),
			relatively_phase::point(origin),
			relatively_phase::dirct(base) {}

		frame(const dirct &base, const point &origin = point::zero()):frame(origin, base) {}

		frame(const std::shared_ptr<const point> &origin, const std::shared_ptr <const dirct> &base = dirct::root()):frame(origin ? *origin : point::zero(), base ? *base : dirct::identity()) {}
		frame(const std::shared_ptr<const dirct> &base, const std::shared_ptr<const point> &origin = point::root()):frame(origin, base) {}

		frame(const std::shared_ptr<const frame> &base, const relatively_phase::frame &frame): frame(point(base, frame), dirct(base, frame)) {}


		// a static method that return the root of all relatively direction
		static std::shared_ptr<const frame> root() { return nullptr; }


		relatively_phase::frame different_from(const std::shared_ptr<const frame> &new_base = frame::root()) const {
			return relatively_phase::frame(point::different_from(new_base), dirct::different_from(new_base));
		}

		std::shared_ptr<const frame> reduce_under(const std::shared_ptr<const frame> &new_base = frame::root()) { return std::make_shared<const frame>(new_base, different_from(new_base)); }
	};
}