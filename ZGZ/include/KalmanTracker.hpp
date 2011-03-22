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
#define TS (1.0/25.0)
#define W_EPS 0.001



template <typename stateVector_t, typename stateCovariance_t>
struct State {
	stateVector_t x;
	stateCovariance_t P;
};

#define I3 Matrix<dataType,3,3>::Identity()

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
		MatrixXf				_Z;
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
			_F.block(0,0,3,3) = I3;
			_F.block(0,3,3,3) = TS*I3;
			_F.block(3,3,3,3) = I3;
			_F.block(10,10,3,3) = I3;

			_Q.setZero();

		}
		_xCovSize& F(){
			return _F;
		}

		State<_xSize,_xCovSize>& Xk(){
			return _Xk;
		}

		_zJacSize& H(){
			return _H;
		}

		State<_xSize,_xCovSize>& Xkp1(){
			return _Xkp1;
		}

		_xCovSize& Q(){
			return _Q;
		}

		_zCovSize& R(){
			return _R;
		}

		void predict()
		{
			dataType wx = 0.5*TS*_Xk.x(4,0);
			dataType wy = 0.5*TS*_Xk.x(5,0);
			dataType wz = 0.5*TS*_Xk.x(6,0);

			Matrix<dataType,4,4> Omega;

			Omega << 	1,	-wx,	-wy,	-wz,
						wx,	  1,	 wz,	-wy,
						wy, -wz, 	  1,	 wx,
						wz,  wy, 	-wz,	 1;

			_F.block(6,6,4,4)  = Omega;
			_F.block(6,10,4,3) = 0.5*getPI() ;

			_Xkp1.x = _F*_Xk.x ;
			_Xkp1.P = _F*_Xk.P*_F.transpose()+_Q;

		}

		void update()
		{
			MatrixXf Ht = _H.transpose();
			_zCovSize S = _H*_Xkp1.P*Ht + _R ;
			MatrixXf K = _Xkp1.P*Ht*S.lu().inverse();
			_Xk.x = _Xkp1.x + K*(_Z-_H*_Xkp1.x);
			_Xk.P = (_xCovSize::Identity() - K*_H)*_Xkp1.P;
		}

		MatrixXf Jac_pinHole(Vector3f p, dataType fu, dataType fv)
		{
			Matrix<dataType,2,3> J;

			dataType z2 = (p(2)*p(2));

			J<< fu/p(2), 		0 , -fu*p(0)/z2,
					  0,  fv/p(2) , -fv*p(1)/z2;
			return J;
		}

		MatrixXf Jac_toFrame_t(){
			return -Rot();
		}

		MatrixXf Jac_toFrame_q(Vector3f p,Vector3f t){
			VectorXf Q = _Xkp1.x.block(6,0,4,1);
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
			dataType a = _Xk.x(6);
			dataType b = _Xk.x(7);
			dataType c = _Xk.x(8);
			dataType d = _Xk.x(9);
			Matrix<dataType,4,3> Pi;
			Pi<<-b,-c,-d,a,-d,c,d,a,-b,-c,b,a;
			return Pi;
		}

		MatrixXf getPIstar()
		{
			dataType a =   _Xk.x(6);
			dataType b = - _Xk.x(7);
			dataType c = - _Xk.x(8);
			dataType d = - _Xk.x(9);
			Matrix<dataType,4,3> Pi;
			Pi<<-b,-c,-d,a,-d,c,d,a,-b,-c,b,a;
			return Pi;
		}


		MatrixXf Jac_H(Vector3f p,Vector3f t,dataType fu=1,dataType fv=1)
		{
			Matrix<dataType,2,13> J;
			MatrixXf phJ = Jac_pinHole(p,fu,fv);
			J<< phJ*Jac_toFrame_t(), Matrix<dataType,2,3>::Zero(),phJ*Jac_toFrame_q(p,t),Matrix<dataType,2,3>::Zero();
			return J;
		}

		void constructH(PatchesVector p,Vector3f t,dataType fu=1, dataType fv=1){

			PatchesVector::iterator iter;
			MatrixXf H(2*p.size(),13);
			int i;
			for(iter=p.begin(),i=0;iter!=p.end();iter++,i++){
				H.block<2,13>(2*i,0) = Jac_H(Vector3f(iter->xyz.x,iter->xyz.y,iter->xyz.z),t,fu,fv);
			}
			_H = H;
		}

		void findMatches(
				Mat& image,
				PatchesVector& p,
				KeyPointsVector& matches,
				int& NrMatches,
				bool draw,
				Mat& imageWithMatches
				)
		{
			if(matches.size()>0) matches.clear();
			PatchesVector::iterator iter;

			if(draw) imageWithMatches = image.clone();
			int i;
			Matrix<float,2,13> r;
			Matrix2f Uncertainity;
			VectorXf Z(2*p.size());
			MatrixXf H(2*p.size(),13);
			Vector2f sigma, m, measured;

			int nmatches = 0;
			Point maxLoc;
			double maxVal;
			Mat res;

			for(iter = p.begin(), i= 0 ; iter!=p.end(); iter++, i ++)
			{
				r = _H.block(2*i,0, 2 ,13);
				Uncertainity = r*_Xkp1.P*r.transpose()+Matrix2f::Identity();
				sigma = Uncertainity.diagonal().array().sqrt();

				float rReg = 3.0*sigma(0);
				float cReg = 3.0*sigma(1);

				std::cerr<<"SearchRegion : ["<<rReg<<","<<cReg<<"]"<<std::endl;

				//std::cerr<<"_dx,dy_\n"<<r*_Xkp1.x<<std::endl;

				m = getPrediction(iter->xyz);

				//m(0) = (m(0)+0.5)*480;
				//m(1) = (m(1)+0.5)*320;
				std::cerr<<"Prediction : \n"<<m<<std::endl;

				if(draw) rectangle(imageWithMatches,Point(m(0),m(1)),Point(m(0)+1,m(1)+1),0);

				//std::cout<<m<<std::endl;

				float rOrigin = (m(0) - rReg/2);
				float cOrigin = (m(1) - cReg/2) ;



				if(	(cOrigin < 0 || rOrigin < 0) ||
					(cOrigin > image.cols || rOrigin > image.rows) ||
					(cOrigin + cReg > image.cols|| rOrigin+ rReg > image.rows )
					)
				{
					std::cerr<<"Skipped "<<std::endl;
					continue;
				}

				res.setTo(0);
				if(draw) rectangle(imageWithMatches,Rect(cOrigin,rOrigin,cReg,rReg),255);
				matchTemplate(Mat(image,Rect(cOrigin,rOrigin,cReg,rReg)),iter->texture,res,MATCHING_METHOD);
				minMaxLoc(res,NULL,&maxVal,NULL,&maxLoc);


				if(maxVal>MATCHING_THRESHOLD){
					measured << rOrigin + maxLoc.x + int(PATCH_WIDTH/2),cOrigin+maxLoc.y + int(PATCH_HEIGHT/2);
					//measured << cOrigin + maxLoc.x , rOrigin+maxLoc.y;
					if(draw) rectangle(imageWithMatches,Point(measured(1),measured(0)),Point(measured(1)+1,measured(0)+1),255);
					Z.block<2,1>(2*nmatches,0) =  measured - Vector2f(iter->uvd.x,iter->uvd.y);
					std::cout<<"Measured"<<std::endl;
					std::cout<<measured<<std::endl;
					std::cout<<"difference"<<std::endl;
					std::cout<<Z.block<2,1>(2*nmatches,0)<<std::endl;

					//std::cin.get();

					H.block<2,13>(2*nmatches,0)= r;
					nmatches++;

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

		Vector2f getPrediction(const Point3d& p)
		{
			Vector3f p1;
			p1 <<  p.x , p.y , p.z;
			Vector3f P = (Rot().transpose())*(p1-t());
			return Vector2f(5.9421434211923247e+02*P(0)/P(2),5.9104053696870778e+02*P(1)/P(2));
		}

		/**
		 * Utility function for returning the translation part of X_{ {k+1} | {k} }
		 * @return
		 */
		Vector3f t()
		{
			return Vector3f(_Xkp1.x.block(0,0,3,1));
		}

		Matrix3f Rot()
		{
			Vector4f Q 	= _Xkp1.x.block(6,0,4,1);
			Quaternion<dataType> 	q(Q(0),Q(1),Q(2),Q(3));
			AngleAxis<dataType> 	angleAxis(q);
			MatrixXf R 	= angleAxis.toRotationMatrix();
			return R;
		}

};
#define xSize 		Matrix<dataType,13,1>
#define xCovSize 	Matrix<dataType,13,13>
#define zSize 		Matrix<dataType,Dynamic,1>
#define zCovSize 	Matrix<dataType,Dynamic,Dynamic>
#define zJacSize 	Matrix<dataType,Dynamic,13>

#define Tracker KalmanTracker<xSize,xCovSize,zSize,zCovSize,zJacSize>
#endif /* KALMANTRACKER_HPP_ */
