#pragma once
#include "util.h"
#include "point.h"
#include "direction.h"

namespace orbital {

	// interface for framework
	class frame {
	protected:
		std::string name;
		const point &center;
		const dirct &orient;
	public:

		virtual bool inertial() const { return center.inertial() && orient.inertial(); }
		virtual frame_state operator()(const std::shared_ptr<frame_state> &) const;
	};

	/*
	// state of a certain framework at a certain time
	class frame_state: public state, public point_state, public dirct_state {
		friend class frame;
	protected:
		const frame &parent;

		frame_state(const std::shared_ptr<frame_state> &frame_s, const frame &frame, const point_state &center, const dirct_state &orient):state(frame_s), parent(frame), point_state(center), dirct_state(orient) { }
	public:
		frame_state(frame_state &&) = default;
		frame_state(const frame_state &) = default;

		frame_state under(const std::shared_ptr<frame_state> &frame_s);
	};
	*/
}