/*****************************************************************************
*                                                                            *
*  OpenNI 1.0 Alpha                                                          *
*  Copyright (C) 2010 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  OpenNI is free software: you can redistribute it and/or modify            *
*  it under the terms of the GNU Lesser General Public License as published  *
*  by the Free Software Foundation, either version 3 of the License, or      *
*  (at your option) any later version.                                       *
*                                                                            *
*  OpenNI is distributed in the hope that it will be useful,                 *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
*  GNU Lesser General Public License for more details.                       *
*                                                                            *
*  You should have received a copy of the GNU Lesser General Public License  *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.            *
*                                                                            *
*****************************************************************************/

/**@file KinectCapture.cpp
 * Basic capture functionality for Kinect
 *
 * Captures RGB and ALIGNED depth images from Kinect
 *
 */
/**
 * This is a brief description
 *
 * This one is a longer description
 */


//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>

#define SAMPLE_XML_PATH "./SamplesConfig.xml"

#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}


using namespace xn;
using namespace cv;

#define FRAME_SIZE_DEPTH 614400	//2*640*480
#define FRAME_SIZE_RGB	921600 	//3*640*480

char header[]="P6 640 480 255";
char depthPath[1024];
char rgbPath[1024];
int alignImages = 0;

int writetofile(const XnDepthPixel* depthData, const XnUInt8* rgbData, int frameID)
{
	char filename[1024];

	sprintf(filename,"%s/%04d.dep",depthPath,frameID);
	FILE * dfp = fopen(filename,"wb");
	if(dfp==NULL)
		return -1;
	fwrite(depthData,FRAME_SIZE_DEPTH,1,dfp);
	fclose(dfp);

	sprintf(filename,"%s/%04d.ppm",rgbPath,frameID);
	FILE * rfp = fopen(filename,"wb");
	if(rfp==NULL)
			return -1;
	fprintf(rfp,"%s\n",header);
	fwrite(rgbData,FRAME_SIZE_RGB,1,rfp);
	fclose(rfp);

	return 0;

}

void usage(char* argv0)
{
	printf("%s -o outdir [./capture/] -a \n\toutdir : Specifies the directory in which to capture\n\t -a : if given aligns the images\n",argv0);
}

int parseArgs(int argc, char **argv)
{
	for(int i=1;i<argc;i++)
	{
		if(strcmp(argv[i],"-o")==0)
		{
			strcpy(depthPath,argv[i+1]);
			strcpy(rgbPath,argv[i+1]);
			i++;
		}
		else if(strcmp(argv[i],"-a")==0)
		{
			alignImages = 1;
		}
		else
		{
			printf("Unknown parameter: %s\n",argv[i]);
			return -1;
		}
	}
	return 0;
}

int main(int argc,char *argv[])
{
	strcpy(depthPath,"./capture/");
	strcpy(rgbPath,"./capture/");
	if(parseArgs(argc,argv)<0)
	{
		usage(argv[0]);
		return -1;
	}
	XnStatus nRetVal = XN_STATUS_OK;
	Context context;
	EnumerationErrors errors;
	nRetVal = context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);



	if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (nRetVal);
	}
	else if (nRetVal != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(nRetVal));
		return (nRetVal);
	}

	cvNamedWindow("RGB");
	cvNamedWindow("Depth");

	static IplImage *img = 0;
	if (!img) img = cvCreateImageHeader(cvSize(640,480), 8, 3);

	static IplImage *dep = 0;
	if(!dep) dep = cvCreateImageHeader(cvSize(640,480),16,1);

	DepthGenerator depth;
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
	CHECK_RC(nRetVal, "Find depth generator");

	ImageGenerator image;
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, image);
	CHECK_RC(nRetVal, "Find image generator");

	XnFPSData xnFPS;
	nRetVal = xnFPSInit(&xnFPS, 180);
	CHECK_RC(nRetVal, "FPS Init");

	DepthMetaData depthMD;
	ImageMetaData imageMD;

	if(image.GetFrameSyncCap().CanFrameSyncWith(depth))
	{
		image.GetFrameSyncCap().FrameSyncWith(depth);
	}
	else if(depth.GetFrameSyncCap().CanFrameSyncWith(image))
	{
		depth.GetFrameSyncCap().FrameSyncWith(image);
	}

	if(alignImages)
	{
		nRetVal = depth.GetAlternativeViewPointCap().SetViewPoint(image);
		CHECK_RC(nRetVal," Set viewpoint");
	}

	CvFileStorage * fs = cvOpenFileStorage("calibration_rgb.yaml",0,CV_STORAGE_READ);
	CvMat * cameraMatrix = (CvMat *)cvReadByName(fs,0,"camera_matrix");
	CvMat * distortionCoeffs = (CvMat *)cvReadByName(fs,0,"distortion_coefficients");

	printf("Capture Started ...\n");
	while (!xnOSWasKeyboardHit())
	{
		nRetVal = context.WaitOneUpdateAll(depth);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
			continue;
		}

		xnFPSMarkFrame(&xnFPS);

		depth.GetMetaData(depthMD);
		image.GetMetaData(imageMD);

		cvSetData(img,(unsigned char *)imageMD.Data(), 640*3);
		cvSetData(dep,(unsigned char *)depthMD.Data(), 640*2);
		IplImage* depcvtd = cvCreateImage(cvSize(640,480),8,1);
		cvConvertScale(dep,depcvtd, -255.0/10000.0,255);

		IplImage * undistortedImg =cvCloneImage(img);

		cvUndistort2(img,undistortedImg,cameraMatrix,distortionCoeffs);

		imshow("RGB",undistortedImg);
		imshow("Depth",depcvtd);
		cvWaitKey(5);

		if(writetofile(depthMD.Data(),imageMD.Data(),depthMD.FrameID())<0){
			printf("Could not write to file .. make sure the directory[%s] is present\n",rgbPath);
			return -1;
		}

		//fprintf(stderr,"FPS: %f\n", xnFPSCalc(&xnFPS));
	}

	cvDestroyAllWindows();
	context.Shutdown();
	printf("Capture Completed\n");
	printf("Done");





	cvDestroyAllWindows();
	return 0;
}
