/*
 * settings.hpp
 *
 *  Created on: Apr 12, 2011
 *      Author: yasir
 */

#ifndef SETTINGS_HPP_
#define SETTINGS_HPP_
enum DetectionMethod{
	HarrisCorners = 0,	//!< Harris Corner Detector
	FastCorners,		//!< FAST Corner Detector
	SIFT,				//!< SIFT Detector (only Detector)
	SURF,				//!< SURF Detector (only Detector)

};

enum AssociationMethod{
	NN = 0,				//!< Nearest Neighbours
	JCBB,				//!< Joint Compatibility
	RJCBB,				//!< Randomized JCBB
};

struct TrackerSettings{
	float linearVelocity;					//<! Variance of linearVelocity sigma-square
	float angularVelocity;					//<! Variance of angularVelocity sigma-square
	AssociationMethod associationMethod;
	float w_eps;								//<! Initial error in angular Velocity
	float v_eps;								//<! Initial error in linear Velocity
	float measurementUncertanity;				//<! Measurement Noise (R)


};

struct DetectorSettings{
	int maxNumFeatures;						//<! The max number of features we want;
	DetectionMethod detectionMethod;		//<! One of DetectionMethod s
	int thershold;							//<! Appropriate Threshold for the detector
	int nonMaximalSuppression;				//<! nonMaximal suppression (FAST CORNERS ONLY)
	bool gridAdapted;						//<! Divides the image into a grid and then detect?
	int gridRows;							//<! Bins in the X directions
	int gridCols;							//<! Bins in the Y directions

};

struct Settings{
	TrackerSettings tracker;
	DetectorSettings detector;
};

static Settings settings;

static void initSettings()
{
	settings.tracker.angularVelocity 		= 1;
	settings.tracker.linearVelocity 		= 1;
	settings.tracker.associationMethod 		= NN;
	settings.tracker.w_eps 					= 1e-15;
	settings.tracker.v_eps 					= 0;
	settings.tracker.measurementUncertanity = 2;

	settings.detector.detectionMethod 		= FastCorners;
	settings.detector.gridAdapted 			= true;
	settings.detector.gridRows 				= 5;
	settings.detector.gridCols 				= 5;
	settings.detector.maxNumFeatures		= 1000;
	settings.detector.nonMaximalSuppression = true;


}

#endif /* SETTINGS_HPP_ */
