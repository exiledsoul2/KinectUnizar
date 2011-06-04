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
#include <state.hpp>
//#include <keyframe.hpp>
#include <evd2by2.hpp>
#include <patch.hpp>
#include <FastMatchTemplate.h>
#include <settings.hpp>
//#include <dataAssociation.hpp>

using namespace Eigen;

#define I3 Matrix<filterDataType,3,3>::Identity()
#define MatrixXt Matrix<filterDataType,Dynamic,Dynamic>
#define VectorXt Matrix<filterDataType,Dynamic,1>

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
			filterDataType wx = _Xk.x(::wx,0);
			filterDataType wy = _Xk.x(::wy,0);
			filterDataType wz = _Xk.x(::wz,0);

			Matrix<filterDataType,4,4> Omega;



			Omega << 	0,	-wx,	-wy,	-wz,
						wx,	  0,	 wz,	-wy,
						wy, -wz, 	  0,	 wx,
						wz,  wy, 	-wx,	 0;

/*			Omega <<  0,  -wx, -wy, -wz,
					  wx,   0, -wz,  wy,
					  wy,  wz,   0, -wx,
					  wz, -wy, wx,   0;
*/

			Matrix<filterDataType,4,3> Pi;
			filterDataType a = _Xk.x(q0);
			filterDataType b = _Xk.x(q1);
			filterDataType c = _Xk.x(q2);
			filterDataType d = _Xk.x(q3);

			Pi<<-b,-c,-d,a,-d,c,d,a,-b,-c,b,a;

			_F.block(q0,q0,4,4)  = Matrix4f::Identity()+0.5*TS*Omega;
			_F.block(q0,::wx,4,3) = 0.5*TS*Pi ;

			//testing remove remove remove !!
			_F.block(tx,::vx,3,3) = TS*_Xk.Rot();
			Vector3f v = TS*_Xk.x.block(::vx,0,3,1);
			_F.block(tx,::q0,3,4) = _Xk.FFq(v);

			//!@note : is this correct?
			//!_Xkp1.x = _F*_Xk.x ;

			//x+ = x+Ts*v
			_Xkp1.x.block(tx,0,3,1) = _Xk.x.block(tx,0,3,1)+TS*_Xk.Rot()*_Xk.x.block(vx,0,3,1); //.
			// const velocity
			_Xkp1.x.block(vx,0,3,1) = _Xk.x.block(vx,0,3,1) ; // + Vector3f::Ones()*0.001;
			// q+ = q + .5 TS omega(w).q
			//_Xkp1.x.block(q0,0,4,1) = _Xk.x.block(q0,0,4,1)+ 0.5*TS*Omega* _Xk.x.block(q0,0,4,1);

			_Xkp1.x.block(q0,0,4,1) = (Matrix4f::Identity()+ 0.5*TS*Omega)* _Xk.x.block(q0,0,4,1);

			_Xkp1.x.block(::wx,0,3,1) = _Xk.x.block(::wx,0,3,1) ; // + Vector3f::Ones()*0.001;

			_Xkp1.P = _F*_Xk.P*_F.transpose()+_Q;

			_Xkp1.normalizeP();

		}

		std::vector<int> onePointRANSACupdate(
				float th,		//!< Therhold is sq. pixels
				float p,		//!< Probability of success
				float epsilon,	//!< outlier ratio
				std::vector<Point2f> predictedPoints,
				std::vector<Point2f> matchedPoints,
				Matrix<filterDataType,Dynamic,1>& Z,
				Matrix<filterDataType,Dynamic,Dynamic>& H,
				const Matrix3f& K,
				std::vector<cv::Point3d>& xyz_prediction,
				int& nrMatches
				)
		{
			int n_hyp = 100 ; //std::ceil(std::log10(1-p)/std::log10(epsilon));
			int bestMatches = 0;
			std::vector<int> bestHypothesis;
			std::vector<int> bestPossibleHighInnovationInliers;

			int _sz = predictedPoints.size();
			srand ( time(NULL) );
			Vector3f p1;
			Vector3f P;
			int index;
			for( int i = 0 ; i < n_hyp ; i++)
			{
				index = rand()%_sz;
				cv::Point2f prediction = predictedPoints[index];
				cv::Point2f match	   = matchedPoints[index];

				Matrix<filterDataType,2,13> Hz;
				Hz= H.block(2*index,0,2,13);

				Matrix2f Sz = Hz*_Xkp1.P*Hz.transpose() + Matrix2f::Identity()*options.tracker.measurementUncertanity;
				MatrixXf Kz = _Xkp1.P*Hz.transpose()*Sz.inverse();
				// 13x13 13x2 2x2
				// The updated state
				VectorXf Xp = _Xkp1.x + Kz*Z.block(2*index,0,2,1);
				// 13x1 -> 13x2 2x1
				// Normalize the quaternion
				Xp.block(::q0,0,4,1) /= Xp.block(::q0,0,4,1).norm();
				// Predict the measurement

				std::vector<cv::Point3d>::iterator point;


				Eigen::Quaternion<float> Q(Xp(q0),Xp(q1),Xp(q2),Xp(q3));
				MatrixXf R = Q.toRotationMatrix();
				Vector3f t = Xp.block(tx,0,3,1);

				int _i=0;
				std::vector<int> hyp;
				std::vector<int> possibleHighInnovationInliers;
				int matches = 0;
				for(point = xyz_prediction.begin(); point!=xyz_prediction.end(); point++,_i++)
				{
					p1 = Vector3f(point->x , point->y , point->z);
					P = K*(R.transpose())*(p1-t);
					Vector2f d = Vector2f(P(0)/P(2),P(1)/P(2))-Vector2f(matchedPoints[_i].x,matchedPoints[_i].y);
					if(d.squaredNorm()<th)
					{
						hyp.push_back(_i);
						matches++;
					}
					else
					{
						possibleHighInnovationInliers.push_back(_i);
					}

				}

				if(matches > bestMatches)
				{
					bestHypothesis = hyp;
					bestMatches = matches;
					bestPossibleHighInnovationInliers = possibleHighInnovationInliers;
				}
				//std::cerr<<"1pRansac : iteration : "<<i<<" Matches : "<<bestMatches<<std::endl;

			}

			std::vector<int>::iterator _it;

			int _m = 0;
			MatrixXf onePoint_Z = MatrixXf(2*(bestMatches),1).setZero();
			MatrixXf onePoint_H = MatrixXf(2*(bestMatches ),13).setZero();
			MatrixXf onePoint_R = MatrixXf(2*bestMatches ,2*bestMatches);

			//std::cerr<<" Hypothesis Size "<< bestHypothesis.size()<<std::endl;
			//std::cerr<<H.rows()<<" "<<H.cols()<<std::endl;
			//std::cerr<<Z.rows()<<" "<<Z.cols()<<std::endl;
			foralliter(_it,bestHypothesis) // bestHypothesis contains INDICES of all the inliers
			{
				    onePoint_Z.block(2*_m,0,2,1) =  Z.block(2*(*_it),0,2,1);
					onePoint_H.block(2*_m,0,2,13) = H.block(2*(*_it),0,2,13);
					_m++;

			}

			onePoint_R.setIdentity();

			onePoint_R *= options.tracker.measurementUncertanity;

		// Rescue Highinnovation inliers //

			// Do a Kalman filter update //

			MatrixXf Ht = onePoint_H.transpose();
			_zCovSize S = onePoint_H*_Xkp1.P*Ht + onePoint_R ;
			std::cerr<<"Inverting 1pRANSAC"<<std::endl;
			std::cerr<<" Points passed "<<_sz << " lowInnov "<< bestHypothesis.size() << std::endl;
			MatrixXf Kt;
			double freq = getTickFrequency();
			double t1 =  (double) getTickCount();
			Kt = (S.ldlt().solve(onePoint_H*_Xkp1.P));
			std::cerr<<((double)getTickCount()-t1)*1000/freq<<" ms "<<std::endl;
			std::cerr<<"Inverting Done"<<std::endl;
			MatrixXf Kx = Kt.transpose();

			State<_xSize,_xCovSize> onePoint_X;

			onePoint_X.x = _Xkp1.x + Kx*(onePoint_Z); //<! @note: _Z is the residual/innovation
			onePoint_X.P = (_xCovSize::Identity() - Kx*onePoint_H)*_Xkp1.P;

			onePoint_X.normalizeP();

			_Xkp1 = onePoint_X;

			// Update complete //

			// for all the points in possibleHighinnovationInliers //

			MatrixXf Rx = onePoint_X.Rot();
			MatrixXf tx = onePoint_X.t();

			MatrixXf Zhi = MatrixXf::Zero(2*_sz,1);		//Temporary for HiInnovInliers
			MatrixXf Hhi = MatrixXf::Zero(2*_sz,13);		//Temporary for hiInnovInliers


			Matrix<filterDataType,3,3> Jac_t = -Rx.transpose();

			int _n = 0;
			std::vector<int>::iterator highInnov;
			foralliter(highInnov,bestPossibleHighInnovationInliers)
			{
				// Predict
				Point3d p = xyz_prediction[*highInnov];
				p1 = Vector3f(p.x, p.y , p.z);
				P = K*(Rx.transpose())*(p1-tx);
				Vector2f d = Vector2f(P(0)/P(2),P(1)/P(2))-Vector2f(matchedPoints[*highInnov].x,matchedPoints[*highInnov].y);
				if(d.squaredNorm()< 9.21*options.tracker.measurementUncertanity)
				{
					//need to recalculate H ?

					bestHypothesis.push_back(*highInnov);
					Zhi.block(2*_n,0,2,1) <<  -d ; // zj - hj Z.block(2*(*highInnov),0,2,1);

					std::cout<<"Innovation \n" << -d<< std::endl;

					//Hhi.block(2*_n,0,2,13) =  H.block(2*(*highInnov),0,2,13);
					Vector3f Pt(p.x,p.y,p.z);
					MatrixXf Jac_pin = Jac_pinHole(Pt,K(0,0),K(1,1));
					Vector4f s= 2*getPIstar()*(Pt-tx);
					Matrix<filterDataType,3,4> TFq ;

					TFq <<  s(1),  s(0), -s(3),  s(2),
							s(2),  s(3),  s(0), -s(1),
							s(3), -s(2),  s(1),  s(0);

					Matrix<filterDataType,3,7> Jac;
					Jac << Jac_t, TFq;

					Matrix<filterDataType,2,7> r = Jac_pin*Jac;


					Hhi.block(2*_n,::tx,2,3)= r.block(0,0,2,3);
					Hhi.block(2*_n,::q0,2,4)= r.block(0,3,2,4);
					_n++;
					std::cout<<Hhi.topRows(_n)<<std::endl;

				}
			}

			_Z = MatrixXf(2*_n,1);
			_Z << Zhi.topRows(2*_n);

			_H = MatrixXf(2*_n,13);
			_H <<Hhi.topRows(2*_n);

			_R = MatrixXf::Identity(2*_n,2*_n);
			_R *= options.tracker.measurementUncertanity;

			if(_n > 0)
				update(); // Update the damned filter here !
			else
				_Xk = _Xkp1;

			//_Xk = _Xkp1;

			std::cerr<<"1-Point RANSAC Matches : (lowInnov) "<<bestMatches<< " (highInnov) "<< bestHypothesis.size() - bestMatches<< std::endl;
			nrMatches = bestMatches;
			return bestHypothesis;
		}

		void update()
		{
			MatrixXf Ht = _H.transpose();
			_zCovSize S = _H*_Xkp1.P*Ht + _R ;
			std::cerr<<"Inverting"<<std::endl;
			MatrixXf Kt;
			double freq = getTickFrequency();
			double t1 =  (double) getTickCount();

			Kt = (S.ldlt().solve(_H*_Xkp1.P));

			std::cerr<<((double)getTickCount()-t1)*1000/freq<<" ms "<<std::endl;
			std::cerr<<"Inverting Done"<<std::endl;
			MatrixXf K = Kt.transpose();

			_Xk.x = _Xkp1.x + K*(_Z); //<! @note: _Z is the residual/innovation
			_Xk.P = (_xCovSize::Identity() - K*_H)*_Xkp1.P;

			_Xk.normalizeP();
			//_Xkp1.P.setZero();
			//_Xkp1.x.setZero();

		}

		MatrixXf Jac_pinHole(Vector3f p, filterDataType fu, filterDataType fv)
		{
			Matrix<filterDataType,2,3> J;

			filterDataType z2 = (p(2)*p(2));

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

			filterDataType s1 = s_star(0);
			filterDataType s2 = s_star(1);
			filterDataType s3 = s_star(2);
			filterDataType s4 = s_star(3);

			Matrix<filterDataType,3,4> TFq;

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
			filterDataType a =  _Xkp1.x(q0);
			filterDataType b =  _Xkp1.x(q1);
			filterDataType c =	_Xkp1.x(q2);
			filterDataType d = 	_Xkp1.x(q3);
			Matrix<filterDataType,4,3> Pi;
			Pi<< 	b , c , d,
					a, d, -c,
					-d, a, b,
					c, -b, a;
			return Pi;
		}

/*
		MatrixXf Jac_H(Vector3f p,Vector3f t,filterDataType fu=1,filterDataType fv=1)
		{
			Matrix<filterDataType,2,7> J;
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
				const Mat& depImage,
				const Mat& K,
				PatchesVector& p ,
				matchesList& matches,
				int& NrMatches,
				bool draw,
				Mat& imageWithMatches
				//Mat& detectorMask
				)
		{
			vector<Point2f> predictedPoints;
			vector<Point2f> measuredPoints;
			std::vector<match> tempMatches;
			std::vector<cv::Point3d> xyz_prediction;

			//detectorMask = Mat(Size(image.cols,image.rows),CV_8U,255);

			if(matches.size()>0) matches.clear();

			if(draw) {
				cvtColor(image,imageWithMatches,CV_GRAY2RGB);
			}

			PatchesVector::iterator iter;

			int i;
			Matrix<filterDataType,2,7> r;
			Matrix<filterDataType,2,2> Uncertainity;
			Matrix<filterDataType,Dynamic,1> Z(2*p.size());
			Matrix<filterDataType,Dynamic,Dynamic> H(2*p.size(),13);
			Matrix<filterDataType,2,1> sigma, m, measured;

			Z.setZero();
			H.setZero();

			int nmatches = 0;
			Point maxLoc;
			double maxVal;
			Mat res;

			Matrix<filterDataType,3,3> k;
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
			Matrix<filterDataType,3,4> TFq;
			Matrix<filterDataType,2,3> Jac_pin;
			Matrix<filterDataType,3,3> Jac_t = -_Xkp1.Rot().transpose();

			Matrix<filterDataType,13,13> Pt;
			Pt = _Xkp1.P.transpose();	//!Why is this transpose? Check @ThisLine
			Matrix<filterDataType,7,7> P_blocks;
			P_blocks.block<3,3>(0,0)=Pt.block<3,3>(tx,tx);
			P_blocks.block<3,4>(0,3)=Pt.block<3,4>(tx,q0);
			P_blocks.block<4,3>(3,0)=Pt.block<4,3>(q0,tx);
			P_blocks.block<4,4>(3,3)=Pt.block<4,4>(q0,q0);

			//std::cout<<P_blocks<<std::endl;

			Matrix<filterDataType,3,7> Jac;

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

				// 0000000000000000000000000000000000000000000000000000000000

				Uncertainity = r*P_blocks*r.transpose() +Matrix2f::Identity(); //!@Tag:ThisLine

				Vector2d eigenValues;
				Matrix2d eigenVectors;

				double theta;

				solve_eig(Uncertainity,eigenValues,eigenVectors,theta);

				if (show) std::cout<<Uncertainity<<std::endl;

				float xReg = (options.matching.restrictedSearch)? options.matching.searchArea : 3.0* sqrt(Uncertainity(0,0));
				float yReg = (options.matching.restrictedSearch)? options.matching.searchArea : 3.0* sqrt(Uncertainity(1,1));

				xReg = (xReg<20)? 20 : xReg;
				yReg = (yReg<20)? 20 : yReg;

				if (show) std::cerr<<"SearchRegion : ["<<xReg<<","<<yReg<<"]"<<std::endl;


				m = getPrediction(iter->xyz,k);


				if (show)  std::cerr<<"Prediction : \n"<<m<<std::endl;

				float xOrigin = (m(0) - xReg/2);
				float yOrigin = (m(1) - yReg/2) ;


				//Let's ignore the points that are at or near the border
				if(	(xOrigin < 0 || yOrigin < 0) ||
					(xOrigin > image.cols || yOrigin > image.rows) ||
					((xOrigin + xReg) > image.cols|| (yOrigin+ yReg) > image.rows )
					)
				{
					if (show) std::cerr<<"Skipped "<<std::endl;
					if(draw) rectangle(imageWithMatches,Rect(m(0)-1,m(1)-1,3,3),CV_RGB(0,0,0));
					continue;
				}


				if (show) std::cerr<<"Image Dims : "<<image.rows << " , "<<image.cols<<std::endl;
				if (show) std::cerr<<"Limits : "<<xOrigin+xReg<<" , "<<yOrigin+yReg<<std::endl;

				res.setTo(-1);
				Rect searchRegion(xOrigin,yOrigin,xReg,yReg);
				if(options.display.searchRegion)
					rectangle(imageWithMatches,searchRegion,CV_RGB(255,255,255),1);
				if(options.display.uncertainityEllipses)
					ellipse(imageWithMatches,Point(m(0),m(1)),cvSize(sqrt(eigenValues(0))*2,sqrt(eigenValues(1))*2),theta,0,360,1);

				Mat texture;
				if(options.matching.affineTransfrom){
					//Calculate affine transformation of the texture
				Vector2f TL = getPrediction(iter->cornersXYZ[0],k);
				Vector2f TR = getPrediction(iter->cornersXYZ[1],k);
				texture = PatchProcessor::getAffineTransformed(*iter,m,TL,TR);
				}
				else
				{	// use the texture as is
					texture = iter->texture;
				}

				//texture = PatchProcessor::normalizePatch(texture);
				Mat target = Mat(image,searchRegion);
				//target = PatchProcessor::normalizePatch(target);
				matchTemplate(target,texture,res,MATCHING_METHOD);
				minMaxLoc(res,NULL,&maxVal,NULL,&maxLoc);

				if (show) std::cout<<"maxVal : "<<maxVal << " at "<<maxLoc.x<<","<<maxLoc.y<<std::endl;

				if(maxVal > MATCHING_THRESHOLD) // It is a match
				{
					measured << (xOrigin + maxLoc.x + PATCH_WIDTH/2), (yOrigin + maxLoc.y + PATCH_HEIGHT/2) ;

					predictedPoints.push_back(Point2f(m(0),m(1)));
					measuredPoints.push_back(Point2f(measured(0),measured(1)));

					Z.block<2,1>(2*nmatches,0) =  (measured - m);

					if (show) std::cout<<"Correction"<<std::endl;
					if (show) std::cout<<Z.block<2,1>(2*nmatches,0)<<std::endl;

					H.block<2,3>(2*nmatches,tx)= r.block<2,3>(0,0);
					H.block<2,4>(2*nmatches,q0)= r.block<2,4>(0,3);

					nmatches++;

					match thisMatch;
					thisMatch.pointidx = i;
					thisMatch.u = measured(0);
					thisMatch.v = measured(1);
					tempMatches.push_back(thisMatch);
					xyz_prediction.push_back(iter->xyz);
				}
				else	// Threshold check failed !
				{
					if(draw) rectangle(imageWithMatches,Rect(m(0)-1,m(1)-1,3,3),CV_RGB(255,0,0));
				}
			} // Done iterating over all points

			//std::vector<bool> inliers = doDataAssociation(predictedPoints,measuredPoints,Z,H,NrMatches);
			std::vector<int> inliers = onePointRANSACupdate(25.0,0.99,0.5,	predictedPoints,measuredPoints,Z,
					H,k,xyz_prediction,NrMatches);

			std::cerr<<" Going to perform an update on "<<inliers.size()<<" inliers"<<std::endl;

			std::vector<int>::iterator inliers_it;
			int idx = 0;
			int count = 0;
			foralliter(inliers_it,inliers)
			{
				//if(*inliers_it == true)	//
				idx = *inliers_it;
				{
					matches.push_back(tempMatches[idx]);
					Rect mask = Rect(tempMatches[idx].u,tempMatches[idx].v,PATCH_HEIGHT,PATCH_WIDTH)-Point(PATCH_HEIGHT/2,PATCH_WIDTH/2);

					//Mat(detectorMask,mask)=Scalar(0);


					if(options.display.matches){
						if ( count > NrMatches )
						rectangle(imageWithMatches,mask,CV_RGB(0,0,255));
						else
							rectangle(imageWithMatches,mask,CV_RGB(0,255,0));

					}

					if(options.display.predictions){
						rectangle(imageWithMatches,
								Rect(predictedPoints[idx],Size(3,3))-Point(1,1),
								CV_RGB(255,0,0));
					}
					if(options.display.matches){
						rectangle(imageWithMatches,Rect(measuredPoints[idx],Size(3,3))-Point(1,1),CV_RGB(0,255,0));
						line(imageWithMatches,predictedPoints[idx],measuredPoints[idx],CV_RGB(0,255,0));
					}
				}
				count++;
				//else{
				//	if(options.display.all)
				//		rectangle(imageWithMatches,Rect(tempMatches[idx].u,tempMatches[idx].v,PATCH_HEIGHT,PATCH_WIDTH)-Point(PATCH_HEIGHT/2,PATCH_WIDTH/2),CV_RGB(255,0,0));
				//}
				//idx++;
			}
		}

		Vector2f getPrediction(const Point3d& p, const Matrix3f& K)
		{
			Vector3f p1(p.x , p.y , p.z);
			Vector3f P = K*(_Xkp1.Rot().transpose())*(p1-_Xkp1.t());
			return Vector2f(P(0)/P(2),P(1)/P(2));
		}

		std::vector<bool>
		doDataAssociation(
				std::vector<cv::Point2f>& predictedPoints,
				std::vector<cv::Point2f>& measuredPoints,
				Matrix<filterDataType,Dynamic,1>& Z,
				Matrix<filterDataType,Dynamic,Dynamic>& H,
				int& NrMatches //! The number of inliers
				)
		{
			int nmatches = predictedPoints.size();
			std::vector<uchar> h ;
			std::vector<bool> result;
			if(!options.matching.fundamentalCheck)
			{
					_Z = Z.topRows(2*nmatches);
					_H = H.topRows(2*nmatches);
					_R = MatrixXf(2*nmatches,2*nmatches);

					NrMatches = nmatches;

					std::vector<bool>::iterator mIter;

					for(int j=0;j<nmatches;j++) result.push_back(true);
			}
			else
			{

				Mat F = findFundamentalMat(Mat(predictedPoints),Mat(measuredPoints),h,CV_FM_RANSAC,2.0,0.99);

				std::cout<<"Fundamental Matrix "<< F <<std::endl;
				std::vector<Vec3f> lines;

				predictedPoints.clear();
				measuredPoints.clear();


				int idx;
				std::vector<uchar>::iterator statusiter;
				int inlierCount = 0;

				Mat hypothesis = Mat(h);

				CvScalar Sum = sum(hypothesis);
				//int inliers = Sum.val[0];

				_Z = Z;
				_H = H;

				for(idx=0,statusiter = h.begin();statusiter!=h.end(); statusiter++,idx++)
				{
					if(int(*statusiter)==1) // is inlier
					{
						_Z(2*inlierCount) = Z(2*idx);
						_Z(2*inlierCount+1) = Z(2*idx+1);

						_H.row(2*inlierCount) = H.row(2*idx);
						_H.row(2*inlierCount+1) = H.row(2*idx+1);

						inlierCount++;
						result.push_back(true);
					}
					else
						result.push_back(false);

				}

				_Z = _Z.topRows(2*inlierCount);
				_H = _H.topRows(2*inlierCount);

				_R = MatrixXf(2*inlierCount,2*inlierCount);
				NrMatches = inlierCount;
			}
			_R.setIdentity();
			_R = _R*options.tracker.measurementUncertanity;
			return result;
		}
		void init()
		{
			//Initalizing the state vector and covariance
			float w_eps = options.tracker.w_eps;
			_Xk.x << 0,0,0,0,0,0,1,0,0,0,w_eps,w_eps,w_eps;
			_Xk.P.setZero();
			//tracker.Xk().P.setIdentity();
			//_Xk.P *= w_eps;
			//_Xk.P.block(3,3,3,3) = I3*options.tracker.v_eps;
			//_Xk.P.bottomRightCorner(3,3)=I3*w_eps;

			// Model Noise covariance
			_Q.setZero();
			_Q.block(vx,vx,3,3) = I3*options.tracker.linearVelocity;
			_Q.block(wx,wx,3,3) = I3*options.tracker.angularVelocity;
			// The measurement Noise covariance depends on the measurement. init later
		}

};
#define xSize 		Matrix<filterDataType,13,1>
#define xCovSize 	Matrix<filterDataType,13,13>
#define zSize 		Matrix<filterDataType,Dynamic,1>
#define zCovSize 	Matrix<filterDataType,Dynamic,Dynamic>
#define zJacSize 	Matrix<filterDataType,Dynamic,13>

#define Tracker KalmanTracker<xSize,xCovSize,zSize,zCovSize,zJacSize>
#endif /* KALMANTRACKER_HPP_ */

