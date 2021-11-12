#pragma once
#include "../util.h"
#include "../relatively_phase/relatively_dirct_phase.h"

namespace orbital::absolute_phase {
	class dirct : public enable_inheritable_shared_from_this<const dirct>, virtual protected relatively_phase::dirct {
	protected:
		// notice that dirct would form a tree, and its root is S.S.B. (Solar System Barycenter) under J2000.
		dirct_ptr prev_dirct; //pointer refer from the prev_dirct dirct, where nullptr is the root
		size_t depth; //depth of the reference tree, where the root(nullptr) is 0

		// default copy constructors
		dirct(dirct &&) = default;
		dirct(const dirct &) = default;

		// build an absolute dirct by take rel_direct (relatively_phase::dirct) transfer on the prev_dirct
		dirct(const dirct_ptr &prev_dirct, relatively_phase::dirct &&rel_direct)
			: prev_dirct(prev_dirct), relatively_phase::dirct(std::move(rel_direct)), depth(prev_dirct ? prev_dirct->depth + 1 : 1) { }
	public:
		virtual ~dirct() = default;


		// a static method that return the root of all relatively direction
		static dirct_ptr root() { return nullptr; }
		
		// return the distance from p to root
		static size_t distanceToRoot(const dirct_ptr &p) { return  p ? p->depth : 0; }


		// build an absolute dirct that equals prev_dirct concatenates rel_direct
		static dirct_ptr make_node(const dirct_ptr &prev_dirct, relatively_phase::dirct &&rel_direct) { return dirct_ptr(new dirct(prev_dirct, std::move(rel_direct))); }

		// build an absolute dirct that equals prev_dirct concatenates rel_direct
		static dirct_ptr make_node(const dirct_ptr &prev_dirct, const relatively_phase::dirct &rel_direct = relatively_phase::dirct::identity()) { return make_node(prev_dirct, relatively_phase::dirct(rel_direct)); }

		// build an absolute dirct that equals root() concatenates rel_direct
		static dirct_ptr make_node(const relatively_phase::dirct &rel_direct = relatively_phase::dirct::identity()) { return make_node(dirct::root(), rel_direct); }


		// build an absolute dirct that equals this concatenates rel_direct
		dirct_ptr make_next_node(relatively_phase::dirct &&rel_direct) const { return make_node(shared_from_this(), std::move(rel_direct)); }
		
		// build an absolute dirct that equals this concatenates rel_direct
		dirct_ptr make_next_node(const relatively_phase::dirct &rel_direct) const { return make_next_node(relatively_phase::dirct(rel_direct)); }

		// return a shared_ptr of the parent of (*this)
		dirct_ptr get_prev_dirct() const { return prev_dirct; }


		// get the distance between two dirct (if distance between two absolute dirct is too large, the floating-error maybe significant)
		size_t distanceTo(const dirct_ptr &) const;


		// get the relatively_phase::dirct that represent (*this) under (*refer_dirct)
		//     which means: this->isApprox(make_node(refer_dirct, this->different_from(refer_dirct))) == true
		relatively_phase::dirct different_from(const dirct_ptr &refer_dirct = dirct::root()) const;


		// build an absolute dirct that has same direction with (*this), while has get_prev_dirct() == new_prev_dirct
		dirct_ptr reduce_under(const dirct_ptr &new_prev_dirct = dirct::root()) const { return make_node(new_prev_dirct, different_from(new_prev_dirct)); }


		// check if two dirct are approximately same
		bool isApprox(const dirct_ptr &other, const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return different_from(other).isApproxIdentity(prec);
		}
		// check if (*this) is approximately same to root
		bool isApproxIdentity(const value_t &prec = Eigen::NumTraits<value_t>::dummy_precision()) const { return isApprox(root(), prec); }


		// check if this direction is intertial (both angular_velocity and angular_acceleration are zero)
		//    NOTICE: a intertial direction may have a non-intertial prev_dirct
		bool isInertial(const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			auto tmp = different_from(); // get the relatively_phase::dirct of this from root()
			return tmp.angular_velocity.squaredNorm() + tmp.angular_acceleration.squaredNorm() < prec * prec;
		}

	protected:
		// modify *this by concatenates rel_direct after *this
		dirct &operator*=(const relatively_phase::dirct &rel_direct) { relatively_phase::dirct::operator*=(rel_direct); return *this; }

	private:
		/// <summary> this is a helper method for different_from, it would change the base of <paramref point_name = "rel_drict"/> and update all of three parameter </summary>
		/// <param point_name='rel_drict'>rel_drict of a diection base on (*ptr) ==> rel_drict of the same diection but base on *(ptr->prev_dirct.get())</param>
		/// <param point_name='ptr'>a pointer to the base for rel_drict; updated after call: ptr ==> ptr->prev_dirct.get()</param>
		/// <param point_name='depth'>the depth of the node refer by ptr; updated after call: depth ==> depth - 1</param>
		static inline void reduce_level(relatively_phase::dirct &rel_drict, const dirct *&ptr, size_t &depth) {
			rel_drict = ( *ptr ) * rel_drict; //update the transfer, so rel_drict become a rel_dirct form reduce_under *(ptr->prev_dirct.get())
			ptr = ptr->prev_dirct.get();
			--depth;
		}
	};

	// return a relatively_phase::dirct
	// s.t. if x is a vector represented under base (*from), relatively_dirct_phase_between(to, from) * x would result in same vector, but represented under base (*to)
	inline relatively_phase::dirct relatively_dirct_phase_between(const dirct_ptr &to, const dirct_ptr &from) {
		if ( to == from ) return relatively_phase::dirct::identity(); // quick return route
		// at least one of param is not nullptr
		if ( from == nullptr ) return to->different_from().inverse(); // from is the root, so return the inverse of the transfer from (*to) to the root
		return from->different_from(to);
	}
}