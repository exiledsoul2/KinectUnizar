/*
 * camera.cpp
 *
 *  Created on: Feb 27, 2011
 *      Author: yasir
 */

#include <ZGZ.hpp>
#include <camera.hpp>
#include <math.h>
using namespace ZGZ::zcv;
using namespace ZGZ::zEigen;
ZGZ::ZGZ_RETURN_STATUS camera::readCalibration(std::string filename, const string nameK, const string namek1)
{
	FileStorage fs;
	fs.open(filename,FileStorage::READ);
	Mat matK;
	Mat matK1;

	if(!fs.isOpened())
		return ZGZ::ZGZ_ERROR;

	fs[nameK]>>_K;
	fs[namek1]>>_k1;

	_isCalibrated = true;
	return ZGZ::ZGZ_OK;
}

bool camera::isCalibrated(){
	return _isCalibrated;
}

void camera::fastFeatureDetector(int threshold, int nonMaximalSuppression){
	if(_ffd==NULL)
		_ffd = new FastFeatureDetector(threshold,nonMaximalSuppression);
	else{
		delete _ffd;
		_ffd = new FastFeatureDetector(threshold,nonMaximalSuppression);
	}

}

void camera::detectPoints(KeyPointsVector& k){
	if(_ffd)
		_ffd->detect(_currentImage,k);
	else
		std::cout<<"ERROR : Fast Feature Detector is uninitialized";

}

void camera::extractPatches(KeyPointsVector K, std::vector<Patch>& patchList){
	std::vector<KeyPoint>::iterator iter;
	for(iter = K.begin();iter != K.end(); iter++)
	{
		int x = iter->pt.x;
		int y = iter->pt.y;
		int xOrigin = x - (PATCH_HEIGHT-1)/2;
		int yOrigin = y - (PATCH_WIDTH -1)/2;

		// Check if a valid patch can be extracted
		if(xOrigin < 0 || yOrigin < 0) continue;
		if((xOrigin+PATCH_WIDTH)>IMAGE_COLS ||(yOrigin+PATCH_HEIGHT)>IMAGE_ROWS) continue;

		// Check if we have valid depth for the current point
		float depth = _currentDepth.at<unsigned short int>(y ,x);
		if( depth < 1 ) continue;
		//std::cout<<"["<<iter->pt.y<<","<<iter->pt.x<<"]"<<std::endl;
		//if all went well . create the patch and use it.
		Patch patch;
		patch.uvd = Point3d(y,x,depth);
		patch.xyz = toWorldXYZ(patch.uvd);
		patch.texture = Mat(_currentImage, cvRect(xOrigin,yOrigin,PATCH_WIDTH,PATCH_HEIGHT));
		patchList.insert(patchList.end(),patch);

	}
}
void camera::undistortPoints(KeyPointsVector& k){
	//cv::undistortPoints()
}

void camera::showKeyPoints(const KeyPointsVector& k,std::string windowname){
	Mat outImage;
	drawKeypoints(_currentImage,k,outImage);
	imshow(windowname,outImage);
}


Mat& camera::currentImage(){
	return _currentImage;
}
Mat& camera::currentDepth(){
	return _currentDepth;
}

void camera::showPatchesOnImage(KeyPointsVector K,const char* windowName){
	Mat img = _currentImage.clone();
	KeyPointsVector::iterator iter;
	for(iter=K.begin(); iter!=K.end(); iter++)
	{
		int xOrigin = iter->pt.x -(PATCH_HEIGHT-1)/2;
		int yOrigin = iter->pt.y -(PATCH_WIDTH -1)/2;

		Point p1(xOrigin,yOrigin);
		Point p2(xOrigin+PATCH_HEIGHT,yOrigin+PATCH_WIDTH);

		if(_currentDepth.at<unsigned short int>(iter->pt.y,iter->pt.x)>1 )
			rectangle(img,p1,p2,255);
		else
			rectangle(img,p1,p2,0);
	}

	imshow(windowName,img);
}
void camera :: detectAndExtractPatches(KeyPointsVector& kpts,PatchesVector& patchesVector){
	detectPoints(kpts);
	extractPatches(kpts,patchesVector);
}

/*
void camera :: matchKeyPoints(
				const PatchesVector prevImage,

				)
{
	MatrixXf Z(2*prevImage.size(),1);
	// Caculate the Rectangle around the search point and form a ROI for the image
	// then calculate the SSD based correlation and find the maximum
	PatchesVector::iterator iter;
	Vector3f t;
	int i;
	for(iter = prevImage.begin(),i=0; iter!=prevImage.end(); iter++,i++){
		t << iter->xyz.x,iter->xyz.y,iter->xyz.z;
	}
	int xOrigin = currentImage[0].pt.x - (searchRegionSize.height-1)/2;
	int yOrigin = currentImage[0].pt.y - (searchRegionSize.width -1)/2;

	Mat res;
	Rect searchRegion(xOrigin,yOrigin,searchRegionSize.width,searchRegionSize.height);
	matchTemplate(Mat(_currentImage,searchRegion),prevImage[0].texture,res,MATCHING_METHOD);
	Point minLoc;
	Point maxLoc;
	double minVal;
	double maxVal;
	minMaxLoc(res,&minVal,&maxVal,&minLoc,&maxLoc);

	// some  theresholding to determine if it raelly is match

}
*/


Point3d camera::toWorldXYZ(Point3d uvd){
	static const double fx_d = 1.0 / 5.9421434211923247e+02;
	static const double fy_d = 1.0 / 5.9104053696870778e+02;
	static const double cx_d = 3.3930780975300314e+02;
	static const double cy_d = 2.4273913761751615e+02;

	double X = uvd.x;
	double Y = uvd.y;
	const double depth = uvd.z;
	float x = float((X - cx_d) * depth * fx_d);
	float y = float((Y - cy_d) * depth * fy_d);
	return Point3d(x,y,depth);
}
