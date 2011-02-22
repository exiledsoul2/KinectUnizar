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




//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>
//#include <conio.h>
//#include <cv.h>
//#include <highgui.h>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "./SamplesConfig.xml"

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace xn;
//using namespace cv;

#define BUFFER_SIZE 600
#define EVERY_NTH_FRAME 2
char header[]="P6 640 480 255";
XnDepthPixel depthBuffer[BUFFER_SIZE][640*480];
XnUInt8 rgbBuffer[BUFFER_SIZE][640*480*3];
int timeStamp= 0;
int chunkCount = 0 ;

typedef struct XnPoint6D
{
	XnPoint3D loc;
	XnUInt8 texture[3];
}XnPoint6D;

XnPoint6D pointsIn3D[640*480];

char buffer[240];

void toRealWorld(DepthGenerator &depth, const XnDepthPixel *pDepthMap, const XnUInt8 *pImageMap)
{
	for(int i=0;i<480;i++){
		for(int j=0;j<640;j++){
			XnPoint3D in = {j,i,pDepthMap[i*640+j]};
			depth.ConvertProjectiveToRealWorld(1,&in,&pointsIn3D[i*640+j].loc);
			pointsIn3D[i*640+j].texture[0]=pImageMap[3*(i*640+j)];
			pointsIn3D[i*640+j].texture[1]=pImageMap[3*(i*640+j)+1];
			pointsIn3D[i*640+j].texture[2]=pImageMap[3*(i*640+j)+2];
		}
	}
}

void dumpData(int howMany = BUFFER_SIZE)
{
	/*
	char filename[256];
	sprintf(filename,"rgbchunk%04d.raw",chunkCount);

	FILE *rgbchunk = fopen(filename,"wb");
	fwrite(rgbBuffer,howMany*640*480*3,1,rgbchunk);
	fclose(rgbchunk);

	sprintf(filename,"depchunk%04d.raw",chunkCount);
	FILE *depchunk = fopen(filename,"wb");
	fwrite(depthBuffer,640*480*howMany*2,1,depchunk);
	fclose(rgbchunk);

	chunkCount++;
	*/
}

void generateFiles()
{
	char filename[256];
	for (int i=0; i<chunkCount;i++)
	{
		sprintf(filename,"rgbchunk%04d.raw",i);
		FILE * rgbChunk = fopen(filename,"rb");
		if(rgbChunk==NULL)
		{
			printf("Could not open %s -- ABORTING\n",filename);
			exit(-1);
		}
		int cnt = fread(rgbBuffer,640*480*3,BUFFER_SIZE,rgbChunk);
		for(int j = 0 ; j < cnt ; j++ )
		{
			sprintf(filename,"rgb%06d.ppm",(BUFFER_SIZE*i + j));
			FILE *image = fopen(filename,"wb");
			fprintf(image,"%s\n",header);
			fwrite(rgbBuffer[j],640*480*3,1,image);
			fclose(image);
		}

		sprintf(filename,"depchunk%04d.raw",i);
		FILE * depChunk = fopen(filename,"rb");
		cnt = fread(depthBuffer,640*480*2,BUFFER_SIZE,depChunk);
		for(int j = 0 ; j < cnt ; j++ )
		{
			sprintf(filename,"dep%06d.raw",(BUFFER_SIZE*i + j));
			FILE *depth = fopen(filename,"wb");
			fwrite(depthBuffer[j],640*480*sizeof(XnUInt16),1,depth);
			fclose(depth);
		}

	}
}

void writeply(const char *name)
{
	FILE * fp= fopen(name,"wb");
	fprintf(fp,"ply\nformat binary_little_endian 1.0\ncomment : created from Kinect depth image\nelement vertex 307200\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n");
	fwrite(pointsIn3D,sizeof(XnPoint6D)*640*480,1,fp);
	//fwrite(data,640*480*3,1,fp);
	fclose(fp);
}

int main()
{
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

	nRetVal = depth.GetAlternativeViewPointCap().SetViewPoint(image);
	CHECK_RC(nRetVal," Set viewpoint");

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

		int frameID = depthMD.FrameID();

		toRealWorld(depth,depthMD.Data(),imageMD.Data());

		sprintf(buffer,"%04d.ply",depthMD.FrameID());

		writeply(buffer);

		/*
		if( ( frameID % EVERY_NTH_FRAME) == 0 )
		{
			timeStamp++;
			const XnDepthPixel* pDepthMap = depthMD.Data();
			memcpy(depthBuffer[timeStamp%BUFFER_SIZE],(unsigned char *)pDepthMap,depthMD.DataSize());

			const XnUInt8* pImageMap = imageMD.Data();
			memcpy(rgbBuffer[timeStamp%BUFFER_SIZE],pImageMap,imageMD.DataSize());
		}
		//if( timeStamp>0 &&  (timeStamp % BUFFER_SIZE) == 0 )
			//dumpData();
		/*
		Mat depthMat(depthMD.YRes(),depthMD.XRes(),CV_16UC1,(XnUInt16*) pDepthMap);
		Mat imageMat(imageMD.YRes(),imageMD.XRes(),CV_8UC3,(XnUInt8*) pImageMap);
		Mat depthNorm, imageNorm;

		imageNorm = imageMat.clone();

		//Assumed the that maximum range is 10000 mm (10 m)
		depthMat.convertTo(depthNorm,CV_8UC1,255.0/10000);
		cvtColor(imageMat,imageNorm,CV_RGB2BGR);
		// NOTE : Nearest is White .. farthest is black
		imshow("RGB_Image",imageNorm);
		imshow("Depth",depthNorm);
		waitKey(15);


		sprintf(buffer,"rgb_%04d.bmp",depthMD.FrameID());
		//imwrite(buffer,img2);

		sprintf(buffer,"dep_%04d.bmp",depthMD.FrameID());
		//imwrite(buffer,dep2);

		show++;
		*/

		//printf("D Frame %d Middle point is: %u. FPS: %f [x y z] (%f %f %f)\n", frameID, depthMD(depthMD.XRes() / 2, depthMD.YRes() / 2), xnFPSCalc(&xnFPS),pnt.X,pnt.Y,pnt.Z);
		//printf("I Frame %d Middle point is: 0. FPS: %f\n", imageMD.FrameID(), xnFPSCalc(&xnFPS));
	}

	dumpData(timeStamp%BUFFER_SIZE);
	//cvDestroyAllWindows();
	context.Shutdown();
	printf("Capture Completed\n");
	printf("Generating files ... ");
	generateFiles();
	printf("Done");
	return 0;
}
