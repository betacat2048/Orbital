#pragma once
#include "../util.h"
#include "../relatively_phase/relatively_point_phase.h"
#include "absolute_dirct_phase.h"

namespace orbital::absolute_phase {
	class point_diff: public virtual relatively_phase::point {
	protected:
		dirct_ptr refer_dirct;
	public:
		// default copy constructors
		point_diff(point_diff &&) = default;
		point_diff(const point_diff &) = default;
		point_diff &operator=(point_diff &&) = default;
		point_diff &operator=(const point_diff &) = default;
		virtual ~point_diff() = default;

		point_diff(const dirct_ptr &refer_dirct, relatively_phase::point &&rel_pos):refer_dirct(refer_dirct), point(std::move(rel_pos)) {}
		point_diff(const dirct_ptr &refer_dirct, const relatively_phase::point &rel_pos = relatively_phase::point::zero()):point_diff(refer_dirct, point(rel_pos)) {}

		point_diff(relatively_phase::point &&rel_pos):point_diff(dirct::root(), std::move(rel_pos)) {}
		point_diff(const relatively_phase::point &rel_pos = relatively_phase::point::zero()):point_diff(point(rel_pos)) {}


		/// <summary> a static method that return point_diff means a zero different</summary>
		static point_diff zero() { return point_diff(); }


		dirct_ptr get_refer_dirct() const { return refer_dirct; }

		point_diff reduce_under(const dirct_ptr &new_refer_dirct) const {
			return point_diff(new_refer_dirct, relatively_dirct_phase_between(new_refer_dirct, refer_dirct) * (*this));
		}

		point_diff &operator+=(const point_diff &other) {
			this->relatively_phase::point::operator+=(relatively_dirct_phase_between(refer_dirct, other.refer_dirct) * other); // get the other's rel_pos reduce_under this->refer_dirct, and add to (*this)
			return *this;
		}
		point_diff operator+ (point_diff &&other)const { return other += *this; }
		point_diff operator+ (const point_diff &other)const { auto tmp = *this;	return tmp += other; }

		point_diff inverse()const { return { refer_dirct, this->relatively_phase::point::inverse() }; }

		point_diff &operator+=(const relatively_phase::point &o) { relatively_phase::point::operator+=(o); return *this; }
		point_diff operator+(const relatively_phase::point &o) const { auto tmp = *this; return tmp += o; }
		point_diff operator+(relatively_phase::point &&o) const { auto tmp = *this; return tmp += o; }
	};
}