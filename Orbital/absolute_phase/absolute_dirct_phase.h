#pragma once
#include "../util.h"
#include "../relatively_phase/relatively_dirct_phase.h"

namespace orbital::absolute_phase {
	class dirct: public virtual relatively_phase::dirct {
	protected:
		// notice that dirct would form a tree, and its root is S.S.B. (Solar System Barycenter) reduce_under J2000.
		dirct_ptr prev_dirct; //pointer refer from the prev_dirct dirct, where nullptr is the root
		size_t depth; //depth of the reference tree, where the root(nullptr) is 0
	public:
		// default copy constructors
		dirct(dirct &&) = default;
		dirct(const dirct &) = default;
		dirct &operator=(dirct &&) = default;
		dirct &operator=(const dirct &) = default;		
		virtual ~dirct() = default;


		// build a dirct by take relatively_phase::dirct transfer on the prev_dirct
		dirct(const dirct_ptr &prev_dirct, relatively_phase::dirct &&rel_direct )
			: prev_dirct(prev_dirct), relatively_phase::dirct(std::move(rel_direct)), depth(prev_dirct ? prev_dirct->depth + 1 : 1) {}
		
		// build a dirct by take relatively_phase::dirct transfer on the prev_dirct
		dirct(const dirct_ptr &prev_dirct, const relatively_phase::dirct &rel_direct = relatively_phase::dirct::identity())
			: dirct(prev_dirct, relatively_phase::dirct(rel_direct)) {}

		// build a dirct by take relatively_phase::dirct transfer on the root
		dirct(const relatively_phase::dirct &rel_direct = relatively_phase::dirct::identity()): dirct(dirct::root(), rel_direct) {}

		
		/// <summary> a static method that return a absolute direction same to root()'s diection (notice this would have depth == 1)</summary>
		static dirct identity() { return dirct(); }



		// a static method that return the root of all relatively direction
		static dirct_ptr root() { return nullptr; }

		// return a shared_ptr of the parent of (*this)
		dirct_ptr get_prev_dirct() const { return prev_dirct; }


		// get the distance between two dirct (if distance is two large, transfer between two dirct may cause signifincant floating error)
		size_t distanceTo(const dirct_ptr &) const;

		// get the relatively_phase::dirct that represent (*this) reduce_under (*new_prev_dirct)
		relatively_phase::dirct different_from(const dirct_ptr &new_prev_dirct = dirct::root()) const;

		// get a share_ptr of absolute_phase::dirct, which has same direction with (*this), while has prev_dirct == new_prev_dirct
		dirct_ptr reduce_under(const dirct_ptr &new_prev_dirct = dirct::root()) const { return std::make_shared<const dirct>(new_prev_dirct, different_from(new_prev_dirct)); }

		// check if two dirct are same
		bool isApprox(const dirct_ptr &other, const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return different_from(other).isApproxIdentity(prec);
		}
		bool isApprox(const dirct &other, const value_t prec = Eigen::NumTraits<value_t>::dummy_precision()) const {
			return isApprox(std::make_shared<const dirct>(other), prec);
		}


		dirct &operator*=(const relatively_phase::dirct &o) { relatively_phase::dirct::operator*=(o); return *this; }
		dirct operator*(const relatively_phase::dirct &o) const { auto tmp = *this; return tmp *= o; }
	private:
		/// <summary> this is a helper method for different_from, it would change the base of <paramref name = "rel_drict"/> and update all of three parameter </summary>
		/// <param name='rel_drict'>rel_drict of a diection base on (*ptr) ==> rel_drict of the same diection but base on *(ptr->prev_dirct.get())</param>
		/// <param name='ptr'>a pointer to the base for rel_drict; updated after call: ptr ==> ptr->prev_dirct.get()</param>
		/// <param name='depth'>the depth of the node refer by ptr; updated after call: depth ==> depth - 1</param>
		static inline void reduce_level(relatively_phase::dirct &rel_drict, const dirct *&ptr, size_t &depth) {
			rel_drict = (*ptr) * rel_drict; //update the transfer, so rel_drict become a rel_dirct form reduce_under *(ptr->prev_dirct.get())
			ptr = ptr->prev_dirct.get();
			--depth;
		}
	};

	// return a relatively_phase::dirct
	// s.t. if x is a vector represented reduce_under base (*from), relatively_dirct_phase_between(to, from) * x would result in same vector, but represented reduce_under base (*to)
	inline relatively_phase::dirct relatively_dirct_phase_between(const dirct_ptr &to, const dirct_ptr &from) {
		if( to == from ) return relatively_phase::dirct::identity(); // quick return route
		// at least one of param is not nullptr
		if( from == nullptr ) return to->different_from().inverse(); // from is the root, so return the inverse of the transfer from (*to) to the root
		return from->different_from(to);
	}

}