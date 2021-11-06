#include "absolute_point_phase.h"
#include "absolute_frame_phase.h"

using namespace std;
using namespace Eigen;
using namespace orbital;

absolute_phase::point::point(const absolute_phase::frame_ptr &f, const relatively_phase::point &rel_pos):point(f, { f, rel_pos }) {}

size_t orbital::absolute_phase::point::distanceTo(const point_ptr &o) const {
	if( o == nullptr ) return depth;
	size_t dis = 0;

	auto q = this;
	auto p = o.get();
	size_t q_depth = q->depth;
	size_t p_depth = p ? p->depth : 0;

	while( p != q )
		if( p_depth == q_depth ) {
			p = p->prev_pos.get(), --p_depth, ++dis;
			q = q->prev_pos.get(), --q_depth, ++dis;
		}
		else {
			while( p_depth > q_depth ) p = p->prev_pos.get(), --p_depth, ++dis;
			while( p_depth < q_depth ) q = q->prev_pos.get(), --q_depth, ++dis;
		}

	return size_t();
}

absolute_phase::point_diff orbital::absolute_phase::point::point_diff_from(const point_ptr &new_base) const{
	auto q = this;
	auto p = new_base.get();

	//get the depth of p and q
	size_t q_depth = this->depth;
	size_t p_depth = p ? p->depth : 0;

	// creating two identity transfer pos_trans_base for saving transfer from common pos_trans_base
	absolute_phase::point_diff Q, P;

	while( p != q ) { //finding the common node of p and q
		// notice: as nullptr == nullptr, so p and q won't be null at same time, one of q_depth or p_depth would be non-zero
		if( p_depth == q_depth ) { // if p and q in same depth, but they aren't equal, then reduce both of them
			// both p and q aren't nullptr
			reduce_level(P, p, p_depth);
			reduce_level(Q, q, q_depth);
		}
		else { // case that p and q are not in same depth, so reduce the deeper one to make them in same depth
			// p is a the deeper node, so reduce p
			while( p_depth > q_depth )
				reduce_level(P, p, p_depth); // p won't be nullptr
			// q is the deeper node, so reduce q
			while( p_depth < q_depth )
				reduce_level(Q, q, q_depth); // q won't be nullptr
		}
	}

	//std::cout << "P: " << P.reduce_under(absolute_phase::dirct::root()).posvelacc().transpose() << std::endl;
	//std::cout << "Q: " << Q.reduce_under(absolute_phase::dirct::root()).posvelacc().transpose() << std::endl;
	//std::cout << "P.inverse(): " << P.inverse().reduce_under(absolute_phase::dirct::root()).posvelacc().transpose() << std::endl;
	//std::cout << "P.inverse() + Q: " << (P.inverse() + Q).reduce_under(absolute_phase::dirct::root()).posvelacc().transpose() << std::endl;

	return P.inverse() + Q;
}

absolute_phase::point_diff orbital::absolute_phase::point::different_from(const point_ptr &new_base) const {
	return point_diff_from(new_base).reduce_under(new_base ? new_base->refer_dirct : dirct::root());
}

absolute_phase::point_diff orbital::absolute_phase::point::different_from(const absolute_phase::frame_ptr &new_base) const {
	return point_diff_from(new_base).reduce_under(new_base);
}

absolute_phase::point_ptr orbital::absolute_phase::point::reduce_under(const absolute_phase::frame_ptr &new_base) {
	return std::make_shared<const point>(static_cast<absolute_phase::point_ptr>(new_base), different_from(new_base));
}