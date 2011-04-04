/*
 * state.hpp
 *
 *  Created on: Mar 27, 2011
 *      Author: yasir
 */

#ifndef STATE_HPP_
#define STATE_HPP_
#include <ZGZ.hpp>
using namespace cv;
using namespace Eigen;

namespace ZGZ {

typedef enum indices{
	tx = 0,
	ty,
	tz,
	vx,
	vy,
	vz,
	q0,
	q1,
	q2,
	q3,
	wx,
	wy,
	wz,
}indices;

template <typename stateVector_t, typename stateCovariance_t>
class State {
public:
	stateVector_t x;
	stateCovariance_t P;
	Vector3f t()
	{
		return Vector3f(x.block(tx,0,3,1));
	}

	Matrix3f Rot()
	{
		Quaternion<dataType> 	q(x(q0),x(q1),x(q2),x(q3));
		return q.toRotationMatrix();
	}
	void normalizeP()
	{
		/*
		 J=(r*r+x*x+y*y+z*z)^(-3/2)*...
			[x*x+y*y+z*z         -r*x         -r*y         -r*z;
			-x*r  r*r+y*y+z*z         -x*y         -x*z;
			-y*r         -y*x  r*r+x*x+z*z         -y*z;
			-z*r         -z*x         -z*y  r*r+x*x+y*y];
		 */
		Quaternion<float> q(x(q0),x(q1),x(q2),x(q3));
		float norm = q.norm();




		float r = q.w();
		float qx = q.x();
		float qy = q.y();
		float qz = q.z();

		Matrix4f J;
		J<< qx*qx+qy*qy+qz*qz,		-r*qx,         -r*qy,          -r*qz,
				-qx*r,	 r*r+qy*qy+qz*qz,         -qx*qy,          -qx*qz,
				-qy*r, 			-qy*qx,  r*r+qx*qx+qz*qz,          -qy*qz,
				-qz*r,           -qz*qx,         -qz*qy,   r*r+qx*qx+qy*qy;

		J*= 1.0/std::pow(norm*norm,double(1.5));;

		P.block(tx,q0,3,4)  *= J.transpose();
		P.block(vx,q0,3,4)  *= J.transpose();
		P.block(q0,q0,4,4)  *= J.transpose();
		P.block(wx,q0,3,4) *= J.transpose();

		P.block(q0,tx,4,6) = J* P.block(q0,tx,4,6);
		P.block(q0,wx,4,3) = J* P.block(q0,wx,4,3);

		P.block(q0,q0,4,4) = J* P.block(q0,q0,4,4) * J.transpose();


		x.block(q0,0,4,1) /= norm;

	}

};
}
#endif /* STATE_HPP_ */
