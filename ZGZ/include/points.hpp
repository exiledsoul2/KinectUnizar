/*
 * points.hpp
 *
 *  Created on: Mar 29, 2011
 *      Author: yasir
 */

#ifndef POINTS_HPP_
#define POINTS_HPP_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace std;

#define filterDataType double

#define Vector3t Matrix<filterDataType,3,1>

struct PerPointCameraList
{
	int cameraID;
	filterDataType u;
	filterDataType v;
};


class point3{
public:
	Vector3t xyz;
	int ID;
	std::vector<PerPointCameraList> cameraList;
	point3(Vector3t v, int id){
		xyz = v;
		ID = id;
	}
	point3(cv::Point3d v, int id)
	{
		xyz << v.x , v.y , v. z;
		ID = id;
	}
};

#define GlobalPointList std::vector<point3>


class Kamera
{
private:
	Kamera(){}
public:
	int ID;
	Vector3t t;
	Quaternion<filterDataType> q;
	filterDataType fu;
	filterDataType fv;
	filterDataType k1;
	filterDataType k2;
	Kamera(int id, Vector3t in, Quaternion<filterDataType> qIn, filterDataType fui, filterDataType fvi, filterDataType k1i, filterDataType k2i)
	{
		ID 	= id;
		t 	= in;
		q 	= qIn;
		fu  = fui;
		fv 	= fvi;
		k1 	= k1i;
		k2 	= k2i;
	}
};

#define GlobalCameraList std::vector<Kamera>

#endif /* POINTS_HPP_ */
