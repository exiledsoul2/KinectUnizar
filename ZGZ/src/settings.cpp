/*
 * settings.cpp
 *
 *  Created on: Apr 12, 2011
 *      Author: yasir
 */
#include <settings.hpp>

void initSettings(Settings& options)
{
	options.tracker.angularVelocity 		= 1;
	options.tracker.linearVelocity 			= 1;
	options.tracker.associationMethod 		= NN;
	options.tracker.w_eps 					= 1e-10;
	options.tracker.v_eps 					= 1e-10;
	options.tracker.measurementUncertanity 	= 3*3;

	//options.detector.detectionMethod 			= FastCorners; //Not implemented
	options.detector.gridAdapted 			= true;
	options.detector.gridRows 				= 5;
	options.detector.gridCols 				= 5;
	options.detector.maxNumFeatures			= 200;
	options.detector.nonMaximalSuppression 	= true;
	options.detector.kdtreeCheck			= true;
	options.detector.kdtreeThresh			= 0.0001f;

	options.matching.patchSize				= 15;
	options.matching.restrictedSearch 		= false;
	options.matching.searchArea				= 50;
	options.matching.affineTransfrom		= false;
	options.matching.fundamentalCheck		= false;

	options.display.all 					= false;
	options.display.matches					= true;
	options.display.predictions				= true;
	options.display.uncertainityEllipses	= true;
	options.display.searchRegion			= false;
	options.display.pointsNotSeen			= false;
}


