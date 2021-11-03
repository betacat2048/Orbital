#pragma once
#include <memory>
#include <vector>

// include the over all part
#include "util.h"

// include all relatively_phase
#include "relatively_phase/relatively_point_phase.h"
#include "relatively_phase/relatively_dirct_phase.h"
#include "relatively_phase/relatively_frame_phase.h"

// include all absolute_phase
#include "absolute_phase/absolute_dirct_phase.h"
#include "absolute_phase/absolute_point_phase.h"
#include "absolute_phase/absolute_frame_phase.h"



//namespace orbital {
//	class gravity_model {
//		const std::vector<std::vector<value_t>> coeff;
//		const value_t mass;
//	public:
//		gravity_model(const value_t &mass = 0):coeff(), mass(mass) { }
//		gravity_model(const std::vector<std::vector<value_t>> &coeff, const value_t &mass = 0):coeff(coeff), mass(mass) { }
//		vec3 acceleration(const vec3 &pos, const size_t &ord = 0) const;
//		value_t potential(const vec3 &pos, const size_t &ord = 0) const;
//		vec3 operator()(const vec3 &pos, const size_t &ord = 0) const { return acceleration(pos, ord); }
//		operator value_t() const { return mass; }
//	};
//}
