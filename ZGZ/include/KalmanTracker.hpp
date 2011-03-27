/*
 * KalmanTracker.hpp
 *
 *  Created on: Mar 17, 2011
 *      Author: yasir
 */

/*

 state vector
[	0	1	2	3	4	5	6	7	8	9	10	11	12	]
[ 	x	y	z	vx	vy	vz	q	q1	q2	q3	wx	wy	wz


 */


#ifndef KALMANTRACKER_HPP_
#define KALMANTRACKER_HPP_

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <ZGZ.hpp>

using namespace Eigen;

#define dataType float	//!< Defines the primary datatype of the filter
#define TS (1.0/30.0)
#define W_EPS 0.0000000001
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
		Vector4f Q 	= x.block(q0,0,4,1);
		Quaternion<dataType> 	q(Q(0),Q(1),Q(2),Q(3));
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

		J*= pow(double(norm),double(-1.5));

		P.block(0,q0,3,4)  *= J.transpose();
		P.block(3,q0,3,4)  *= J.transpose();
		P.block(7,q0,4,4)  *= J.transpose();
		P.block(10,q0,3,4) *= J.transpose();

		P.block(q0,0,4,13) = J* P.block(q0,0,4,13);

		x.block(q0,0,4,1) /= norm;

	}

};

#define I3 Matrix<dataType,3,3>::Identity()
#define MatrixXt Matrix<dataType,Dynamic,Dynamic>
#define VectorXt Matrix<dataType,Dynamic,1>

template <
	typename _xSize,
	typename _xCovSize,
	typename _zSize,
	typename _zCovSize,
	typename _zJacSize
	>

class KalmanTracker
{
	private :
		State<_xSize,_xCovSize> _Xk;		//!< State contains Mean AND convariance
		_xCovSize 				_F;			//!< State Transition model
		_zJacSize 				_H;			//!< Observation Model
		_xCovSize 				_Q; 		//!< Model Noise Covariance
		_zCovSize 				_R; 		//!< Observation Covariance
		// Variables for holding intermediate data
		State<_xSize,_xCovSize>		_Xkp1;	//!< X_{k|k-1}
		MatrixXf				_Z;			//!< contains INNOVATION!
	public:
		KalmanTracker(){
			/*
			 * F =  | I Ts*I 0 0 |
			 * 		| 0  I   0 0 |
			 * 		| 0  0   0 0 |
			 * 		| 0  0   0 I |
			 *
			 * The inclusion of quaternions is done in the predict function, before
			 * doing the actual prediction
			 * */
			_F.setZero();
			_F.block(tx,0,3,3) = I3;
			_F.block(tx,vx,3,3) = TS*I3;
			_F.block(vx,vx,3,3) = I3;
			_F.block(wx,wx,3,3) = I3;

			//_Q.setZero();

		}
		_xCovSize& F()					{	return _F;   }
		_zJacSize& H()					{	return _H;  	}
		_xCovSize& Q()					{ 	return _Q; 	}
		_zCovSize& R()					{	return _R;	}
		State<_xSize,_xCovSize>& Xkp1()	{ 	return _Xkp1; }
		State<_xSize,_xCovSize>& Xk() 	{ 	return _Xk; 	}

		void predict()
		{
			dataType wx = _Xk.x(::wx,0);
			dataType wy = _Xk.x(::wy,0);
			dataType wz = _Xk.x(::wz,0);

			Matrix<dataType,4,4> Omega;

			Omega << 	0,	-wx,	-wy,	-wz,
						wx,	  0,	 wz,	-wy,
						wy, -wz, 	  0,	 wx,
						wz,  wy, 	-wz,	 0;

			Matrix<dataType,4,3> Pi;
			dataType a = _Xk.x(q0);
			dataType b = _Xk.x(q1);
			dataType c = _Xk.x(q2);
			dataType d = _Xk.x(q3);

			Pi<<-b,-c,-d,a,-d,c,d,a,-b,-c,b,a;

			_F.block(q0,q0,4,4)  = Matrix4f::Identity()+0.5*TS*Omega;
			_F.block(q0,::wx,4,3) = 0.5*TS*Pi ;

			_Xkp1.x = _F*_Xk.x ;
			_Xkp1.P = _F*_Xk.P*_F.transpose()+_Q;
			_Xkp1.normalizeP();

		}

		void update()
		{
			MatrixXf Ht = _H.transpose();
			_zCovSize S = _H*_Xkp1.P*Ht + _R ;
			std::cerr<<"Inverting"<<std::endl;
			MatrixXf K = _Xkp1.P*Ht*S.fullPivLu().inverse();
			std::cerr<<"Inverting Done"<<std::endl;
			_Xk.x = _Xkp1.x + K*(_Z); //<! @note: _Z is the residual
			_Xk.P = (_xCovSize::Identity() - K*_H)*_Xkp1.P;
			_Xk.normalizeP();
		}

		MatrixXf Jac_pinHole(Vector3f p, dataType fu, dataType fv)
		{
			Matrix<dataType,2,3> J;

			dataType z2 = (p(2)*p(2));

			J<< fu/p(2), 		0 , -fu*p(0)/z2,
					  0,  fv/p(2) , -fv*p(1)/z2;
			return J;
		}
/*
		MatrixXf Jac_toFrame_t(){
			return -_Xkp1.Rot().transpose();
		}

		MatrixXf Jac_toFrame_q(Vector3f p,Vector3f t){
			//VectorXf Q = _Xkp1.x.block(6,0,4,1);

			VectorXf s_star= 2*getPIstar()*(p-t);

			dataType s1 = s_star(0);
			dataType s2 = s_star(1);
			dataType s3 = s_star(2);
			dataType s4 = s_star(3);

			Matrix<dataType,3,4> TFq;

			TFq <<  s2,  s1, -s4,  s3,
					s3,  s4,  s1, -s2,
					s4, -s3,  s2,  s1;

			return TFq;
		}

		MatrixXf getPI()
		{

			return Pi;
		}
*/
		MatrixXf getPIstar()
		{
			dataType a =   _Xk.x(q0);
			dataType b = - _Xk.x(q1);
			dataType c = - _Xk.x(q2);
			dataType d = - _Xk.x(q3);
			Matrix<dataType,4,3> Pi;
			Pi<<-b,-c,-d,a,-d,c,d,a,-b,-c,b,a;
			return Pi;
		}

/*
		MatrixXf Jac_H(Vector3f p,Vector3f t,dataType fu=1,dataType fv=1)
		{
			Matrix<dataType,2,7> J;
			MatrixXf phJ = Jac_pinHole(p,fu,fv);
			J = -_Xkp1.Rot().transpose(), Jac_toFrame_q(p,t);
			J = phJ*J;
			return J;
		}

		void constructH(PatchesVector p,const Mat& K){

			float fu = (float)K.at<double>(0,0);
			float fv = (float)K.at<double>(1,1);

			PatchesVector::iterator iter;
			Vector3f t = _Xkp1.t();
			MatrixXf H(2*p.size(),13);
			int i;
			for(iter=p.begin(),i=0;iter!=p.end();iter++,i++){
				H.block<2,13>(2*i,0) = Jac_H(Vector3f(iter->xyz.x,iter->xyz.y,iter->xyz.z),t,fu,fv);
			}
			_H = H;
		}
*/
		void findMatches(
				const Mat& image,
				const Mat& K,
				PatchesVector& p,
				KeyPointsVector& matches,
				int& NrMatches,
				bool draw,
				Mat& imageWithMatches
				)
		{
			if(matches.size()>0) matches.clear();

			if(draw) imageWithMatches = image.clone();

			PatchesVector::iterator iter;


			int i;
			Matrix<dataType,2,7> r;
			Matrix<dataType,2,2> Uncertainity;
			Matrix<dataType,Dynamic,1> Z(2*p.size());
			Matrix<dataType,Dynamic,Dynamic> H(2*p.size(),13);
			Matrix<dataType,2,1> sigma, m, measured;

			Z.setZero();
			H.setZero();

			int nmatches = 0;
			Point maxLoc;
			double maxVal;
			Mat res;

			Matrix<dataType,3,3> k;
			k.setZero();
			k(0,0) = K.at<double>(0,0);
			k(1,1) = K.at<double>(1,1);
			k(2,2) = K.at<double>(2,2);
			k(0,2) = K.at<double>(0,2);
			k(1,2) = K.at<double>(1,2);

			float fu = K.at<double>(0,0);
			float fv = K.at<double>(1,1);

			bool show = false;
			//Parts of Jacobian that do not depend on the current point
			// should be calculated once.
			MatrixXt PiStar = getPIstar();
			MatrixXt t = _Xkp1.t();
			Matrix<dataType,3,4> TFq;
			Matrix<dataType,2,3> Jac_pin;
			Matrix<dataType,3,3> Jac_t = -_Xkp1.Rot();

			Matrix<dataType,13,13> Pt;
			Pt = _Xkp1.P;
			Matrix<dataType,7,7> P_blocks;
			P_blocks.block<3,3>(0,0)=Pt.block<3,3>(tx,tx);
			P_blocks.block<3,4>(0,3)=Pt.block<3,4>(tx,q0);
			P_blocks.block<4,3>(3,0)=Pt.block<4,3>(q0,tx);
			P_blocks.block<4,4>(3,3)=Pt.block<4,4>(q0,q0);

			//std::cout<<P_blocks<<std::endl;

			Matrix<dataType,3,7> Jac;
			for(iter = p.begin(), i= 0 ; iter!=p.end(); iter++, i ++)
			{
				//-------------------------------------------------------
				// ------------------Calculation of Jacbians ------------
				//------------------------------------------------------

				Vector3f Pt(iter->xyz.x,iter->xyz.y,iter->xyz.z);
				Jac_pin = Jac_pinHole(Pt,fu,fv);

				Vector4f s= 2*PiStar*(Pt-t);
				TFq <<  s(1),  s(0), -s(3),  s(2),
						s(2),  s(3),  s(0), -s(1),
						s(3), -s(2),  s(1),  s(0);

				Jac << Jac_t, TFq;
				r = Jac_pin*Jac;

				//std::cout<<r<<std::endl;
				// 0000000000000000000000000000000000000000000000000000000000
				Uncertainity = r*P_blocks*r.transpose()+Matrix2f::Identity();
				if (show) std::cout<<Uncertainity<<std::endl;

				float xReg = 2.0* sqrt(Uncertainity(0,0));
				float yReg = 2.0* sqrt(Uncertainity(1,1));

				if (show) std::cerr<<"SearchRegion : ["<<xReg<<","<<yReg<<"]"<<std::endl;

				//std::cerr<<"_dx,dy_\n"<<r*_Xkp1.x<<std::endl;

				m = getPrediction(iter->xyz,k);

				if (show)  std::cerr<<"Prediction : \n"<<m<<std::endl;

				if(draw) rectangle(imageWithMatches,Rect(m(0)-1,m(1)-1,3,3),0);

				//std::cout<<m<<std::endl;

				float xOrigin = (m(0) - xReg/2);
				float yOrigin = (m(1) - yReg/2) ;



				if(	(xOrigin < 0 || yOrigin < 0) ||
					(xOrigin > image.cols || yOrigin > image.rows) ||
					(xOrigin + xReg > image.cols|| yOrigin+ yReg > image.rows )
					)
				{
					if (show) std::cerr<<"Skipped "<<std::endl;
					//p.erase(p.begin()+i);
					//if (show) std::cout<<"Now Size : "<<p.size()<<std::endl;
					//iter--;
					continue;
				}

				if (show) std::cerr<<"Image Dims : "<<image.rows << " , "<<image.cols<<std::endl;
				if (show) std::cerr<<"Limits : "<<xOrigin+xReg<<" , "<<yOrigin+yReg<<std::endl;
				res.setTo(-1);
				Rect searchRegion(xOrigin,yOrigin,xReg,yReg);
				if(draw) rectangle(imageWithMatches,searchRegion,255,1);
				//imshow("mat",Mat(image,Rect(xOrigin,yOrigin,xReg,yReg)));
				//imshow("template",iter->texture);
				//cvWaitKey();
				matchTemplate(Mat(image,searchRegion),iter->texture,res,MATCHING_METHOD);
				minMaxLoc(res,NULL,&maxVal,NULL,&maxLoc);

				if (show) std::cout<<"maxVal : "<<maxVal << " at "<<maxLoc.x<<","<<maxLoc.y<<std::endl;

				if(maxVal>MATCHING_THRESHOLD){
					if(PATCH_HEIGHT > xReg )
						measured << xOrigin + maxLoc.x + xReg/2, yOrigin + maxLoc.y + yReg/2 ;
					else
						measured << xOrigin + maxLoc.x + PATCH_WIDTH/2, yOrigin + maxLoc.y + PATCH_HEIGHT/2 ;
					if(draw) {
						rectangle(imageWithMatches,Rect(measured(0)-1,measured(1)-1,3,3),255);
						line(imageWithMatches,Point(m(0),m(1)),Point(measured(0),measured(1)),0);
					}

					Z.block<2,1>(2*nmatches,0) =  measured - m;

					if (show) std::cout<<"Measured"<<std::endl;
					if (show) std::cout<<Z.block<2,1>(2*nmatches,0)<<std::endl;

					//std::cin.get();

					H.block<2,3>(2*nmatches,tx)= r.block<2,3>(0,0);
					H.block<2,4>(2*nmatches,q0)= r.block<2,4>(0,3);
					nmatches++;
					iter->uvd.x = measured(0); iter->uvd.y = measured(1);
					KeyPoint kp(Point2f(measured(0),measured(1)),1,1,1,1,1);
					matches.insert(matches.end(),kp);
				}
			}
			_Z = Z.topRows(2*nmatches);
			_H = H.topRows(2*nmatches);
			_R = MatrixXf(2*nmatches,2*nmatches);
			_R.setIdentity();
			NrMatches = nmatches;

		}

		Vector2f getPrediction(const Point3d& p, const Matrix3f& K)
		{
			Vector3f p1(p.x , p.y , p.z);
			Vector3f P = K*(_Xkp1.Rot().transpose())*(p1-_Xkp1.t());
			return Vector2f(P(0)/P(2),P(1)/P(2));
		}

};
#define xSize 		Matrix<dataType,13,1>
#define xCovSize 	Matrix<dataType,13,13>
#define zSize 		Matrix<dataType,Dynamic,1>
#define zCovSize 	Matrix<dataType,Dynamic,Dynamic>
#define zJacSize 	Matrix<dataType,Dynamic,13>

#define Tracker KalmanTracker<xSize,xCovSize,zSize,zCovSize,zJacSize>
#endif /* KALMANTRACKER_HPP_ */

