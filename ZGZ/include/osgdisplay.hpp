/*
 *  Created on: May 17, 2011
 *      Author: yasir
 */

#ifndef OSGDISPLAY_HPP_
#define OSGDISPLAY_HPP_

#include <ZGZ.hpp>
#include <patch.hpp>
#include <state.hpp>

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
//#include <osg/Texture2D>
//#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
//#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osgSim/LightPointNode>
#include <osg/MatrixTransform>



class osgDisplay
{
private:
	osgViewer::Viewer viewer;
	osg::Group* rootnode;
	osg::Node* lps;

	osgViewer::ViewerBase::Windows wins;
	osg::StateSet* ss;
	osg::Camera* hudCam;

public:
	osgDisplay()
	{
		osg::Vec3d eye(0.0,0.0,-1.0);
		osg::Vec3d center(0.0,0.0,0.0);
		osg::Vec3d up(0.0,-1.0,0.0);

		viewer.setCameraManipulator ( new osgGA::TrackballManipulator() );
		viewer.getCameraManipulator()->setHomePosition(eye, center, up);
		viewer.setUpViewInWindow ( 640, 0, 640, 480 );

		viewer.getWindows(wins);

		wins[0]->setWindowName("Points and Camera");
		viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;

		ss = geode->getOrCreateStateSet();
		ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		ss->setMode( GL_LINE_SMOOTH, osg::StateAttribute::ON );


		rootnode = new osg::Group;
		rootnode->addChild(geode);

		lps = new osg::Group();
		viewer.realize();
	}

	osg::Node* makeNode(PatchList& p, Eigen::Matrix <float,13,1>& camIn)
	{
		Quaternionf orient(camIn[6],camIn[7],camIn[8],camIn[9]);
		osg::MatrixTransform* transform = new osg::MatrixTransform;

		transform->setDataVariance ( osg::Object::STATIC );
		transform->setMatrix ( osg::Matrix::scale ( .1,.1,.1 ) );

		osgSim::LightPointNode* lpn = new osgSim::LightPointNode();
		lpn->getLightPointList().reserve(p.size());

		for ( unsigned int i=0;i< p.size() ;i++ )
		{
			osgSim::LightPoint lp;
			lp._position.x() = p[i].xyz.x;
			lp._position.y()= p[i].xyz.y;
			lp._position.z() = p[i].xyz.z;
			lp._color.set ( 1.0f,1.0f,1.0f,1.0f );
			lp._radius = .01;

			lpn->addLightPoint(lp);
		}
		transform->addChild(lpn);

		osgSim::LightPointNode* camera = new osgSim::LightPointNode();
		camera->getLightPointList().reserve(1);
		osgSim::LightPoint cam;
		cam._color.set ( 1.0f,0.0f,0.0f,1.0f );
		cam._radius = .1;
		camera->addLightPoint(cam);

		//transform->addChild(camera);

		//axis for camera orientation

		osg::MatrixTransform* cameraOrientation = new osg::MatrixTransform;
		cameraOrientation->setMatrix(osg::Matrix::rotate(orient.w(),orient.x(),orient.y(),orient.z()));
		cameraOrientation->setMatrix(osg::Matrix::scale(0.001,0.001,0.001));
		cameraOrientation->setMatrix(osg::Matrix::translate(camIn[0],camIn[1],camIn[2]));

		cameraOrientation->addChild(camera);

		transform->addChild(cameraOrientation);

		osg::Geode* axis = new osg::Geode();
		osg::Geometry* axisGeometry = new osg::Geometry();

		osg::Vec3Array* points = new osg::Vec3Array;
		points->push_back(osg::Vec3(0.0,0.0,0.0));
		points->push_back(osg::Vec3(10.0,0.0,0.0));
		points->push_back(osg::Vec3(0.0,0.0,0.0));
		points->push_back(osg::Vec3(0.0,10.0,0.0));
		points->push_back(osg::Vec3(0.0,0.0,0.0));
		points->push_back(osg::Vec3(0.0,0.0,10.0));

		axisGeometry->setVertexArray( points );

		osg::DrawElementsUInt* draw = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);
		for( unsigned int i=0; i < 6 ; i++ ) {
		    draw->push_back( i );
		}
		axisGeometry->addPrimitiveSet( draw );

		osg::Vec4Array* pointColors = new osg::Vec4Array;
	    pointColors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );
	    pointColors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
	    pointColors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) );


	    axisGeometry->setColorArray(pointColors);
	    axisGeometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);

	    axis->addDrawable(axisGeometry);
	    transform->addChild(axis);
	    cameraOrientation->addChild(axis);



		osg::Group* group = new osg::Group;
		group->addChild ( transform );
		return group;

	}

	void display(PatchList& p, Eigen::Matrix <float,13,1>& cam)
	{
		rootnode->removeChild(lps);
		//lps->deleteUsingDeleteHandler();
		lps = makeNode(p,cam);
		rootnode->addChild(lps);
		viewer.setSceneData(rootnode);
		viewer.frame();
	}

	void loop()
	{
		while(!viewer.done())
			viewer.frame();
	}
};


#endif /* OSGDISPLAY_HPP_ */
