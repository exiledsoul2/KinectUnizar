/*
 * imagesource.hpp
 *
 *  Created on: Mar 2, 2011
 *      Author: yasir
 */

#ifndef IMAGESOURCE_HPP_
#define IMAGESOURCE_HPP_

#define IMG_SRC_FILE  1
#define IMG_SRC_LIVE  1<<1
#define IMG_SRC_VIDEO 1<<2


#include <opencv2/opencv.hpp>
#include <fstream>

class ImageSource
{
	//TODO: Add reading facilities for other modes
	std::string _path;
	std::string _basename;
	std::string _ext;
	int _indexStart;
	int _indexEnd;
	int _nextIndex;
	int _width;
	int _step;
	std::string _currentImagePath;
public:
	ImageSource(){

	}

	ImageSource(std::string path,std::string basename,std::string ext, int start, int end,int step = 1, int width=4):
		_path(path),
		_basename(basename),
		_ext(ext),
		_indexStart(start),
		_indexEnd(end),
		_nextIndex(start),
		_width(width),
		_step(step)
	{

	}

	void getNextImage(cv::Mat& M, int flags = CV_LOAD_IMAGE_GRAYSCALE){
		if(_nextIndex>_indexEnd)
			return;
		std::stringstream ss;

		ss<<_path<<_basename;
		ss.fill('0'); ss.width(_width);
		ss<<_nextIndex<<"."<<_ext;

		_currentImagePath.assign(ss.str());
		if(_ext == std::string("dep")){
			std::ifstream dep; dep.open(_currentImagePath.c_str(),ios::binary);
			uint16_t buffer[640*480];
			dep.read((char*)buffer,640*480*2);
			M = Mat(Size(640,480),CV_16U,buffer).clone();
			dep.close();
		}
		else	M = cv::imread(_currentImagePath,flags);
		std::cout<<_currentImagePath<<std::endl;

		_nextIndex = _nextIndex + _step;
	}
	bool done()
	{
		return (_nextIndex>_indexEnd);
	}
};

#endif /* IMAGESOURCE_HPP_ */
