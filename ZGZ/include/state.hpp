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
		Quaternion<filterDataType> 	q(x(q0),x(q1),x(q2),x(q3));
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

		float r = 	x(q0);
		float qx = 	x(q1);
		float qy = 	x(q2);
		float qz = 	x(q3);

		Matrix4f J;
		J<< qx*qx+qy*qy+qz*qz,		-r*qx,	         -r*qy,          -r*qz,
				-qx*r,	 	r*r+qy*qy+qz*qz,         -qx*qy,          -qx*qz,
				-qy*r, 				-qy*qx,  r*r+qx*qx+qz*qz,          -qy*qz,
				-qz*r,           	-qz*qx,         -qz*qy,   r*r+qx*qx+qy*qy;

		J = J * (std::pow(r*r+qx*qx+qy*qy+qz*qz,double(-3/2)));

		//J.transposeInPlace();
		/**
		 * [ Ptt	Ptv 	Ptq*J'	 Ptw
		 *   Pvt	Pvv 	Pvq*J'	 Pvw
		 *   J*Pqt  J*Pqv   J*Pqq*J' J*Pqw
		 *   Pwt	Pwv		Pwq*J'	 Pww	]
		 */

		P.block(tx,q0,3,4)  =  P.block(tx,q0,3,4) * J.transpose();
		P.block(vx,q0,3,4)  =  P.block(vx,q0,3,4) * J.transpose();
		//P.block(q0,q0,4,4)  *= J.transpose();
		P.block(wx,q0,3,4) 	=  P.block(wx,q0,3,4) * J.transpose();

		P.block(q0,tx,4,3)  = J* P.block(q0,tx,4,3);
		P.block(q0,vx,4,3)  = J* P.block(q0,vx,4,3);
		P.block(q0,wx,4,3)  = J* P.block(q0,wx,4,3);

		P.block(q0,q0,4,4)  = J* P.block(q0,q0,4,4) * J.transpose();

		x.block(q0,0,4,1) = x.block(q0,0,4,1) * (1/(std::pow(r*r+qx*qx+qy*qy+qz*qz,double(1/2))));

	}

	Matrix<filterDataType,3,4> FFq(Vector3f& p)
	{
		Matrix<filterDataType,4,3> Pi;
		filterDataType a = x(q0);
		filterDataType b = x(q1);
		filterDataType c = x(q2);
		filterDataType d = x(q3);

		Pi<<-b,-c,-d,a,-d,c,d,a,-b,-c,b,a;

		Vector4f s = 2*Pi*p;
		Matrix<filterDataType,3,4> ffq;
		ffq <<  s(1), -s(0),  s(3), -s(2),
				s(2), -s(3), -s(0),  s(1),
				s(3),  s(2), -s(1), -s(0);

		return ffq;

	}

};
}
#endif /* STATE_HPP_ */
