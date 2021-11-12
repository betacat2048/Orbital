#include "absolute_dirct_phase.h"

using namespace std;
using namespace Eigen;
using namespace orbital;

size_t orbital::absolute_phase::dirct::distanceTo(const dirct_ptr &o) const {
	if ( o == nullptr ) return depth; // quick return route

	size_t dis = 0;
	auto q = this;	size_t q_depth = q->depth;
	auto p = o.get(); size_t p_depth = p ? p->depth : 0;

	while ( p != q )
		if ( p_depth == q_depth ) {
			p = p->prev_dirct.get(), --p_depth, ++dis;
			q = q->prev_dirct.get(), --q_depth, ++dis;
		}
		else {
			while ( p_depth > q_depth ) p = p->prev_dirct.get(), --p_depth, ++dis;
			while ( p_depth < q_depth ) q = q->prev_dirct.get(), --q_depth, ++dis;
		}

	return dis;
}

relatively_phase::dirct orbital::absolute_phase::dirct::different_from(const dirct_ptr &refer_dirct) const {
	auto q = this;
	auto p = refer_dirct.get();

	//get the depth of p and q
	size_t q_depth = this->depth;
	size_t p_depth = p ? p->depth : 0;

	// creating two identity transfer direct_trans_base for saving transfer from common direct_trans_base
	relatively_phase::dirct Q = relatively_phase::dirct::identity();
	relatively_phase::dirct P = relatively_phase::dirct::identity();

	//find the first common node of p and q
	while ( p != q )
		if ( p_depth == q_depth ) { // if p and q in same depth, but they aren't equal, then reduce both of them
			reduce_level(P, p, p_depth);
			reduce_level(Q, q, q_depth);
		}
		else { // case that p and q are not in same depth, so reduce the deeper one to make them in same depth
			// p is a the deeper node, so reduce p
			while ( p_depth > q_depth )
				reduce_level(P, p, p_depth);
			// q is the deeper node, so reduce q
			while ( p_depth < q_depth )
				reduce_level(Q, q, q_depth);
		}

	return P.inverse() * Q;
}