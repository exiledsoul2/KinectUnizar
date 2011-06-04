#ifndef __SETTINGS_HPP_
#define __SETTINGS_HPP_

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

struct MatchingSettings{
	bool restrictedSearch;					//! Restrict search to @searchArea
	int  searchArea;						//! search area in case of @restrictedSearch
	int  patchSize;							//! size of Texture patch extracted
	bool affineTransfrom;					//! Whether to affine transform the patch or not
	bool fundamentalCheck;					//! Fundamental check for detecting outliers


};

struct DetectorSettings{
	int maxNumFeatures;						//!< The max number of features we want;
	DetectionMethod detectionMethod;		//!< One of DetectionMethod s
	int thershold;							//!< Appropriate Threshold for the detector
	int nonMaximalSuppression;				//!< nonMaximal suppression (FAST CORNERS ONLY)
	bool gridAdapted;						//!< Divides the image into a grid and then detect?
	int gridRows;							//!< Bins in the X directions
	int gridCols;							//!< Bins in the Y directions
	bool kdtreeCheck;						//!< Distance check using kd-tree
	float kdtreeThresh;						//!< Threshold for above distance check

};

struct DisplaySettings{
	bool all;
	bool uncertainityEllipses;
	bool predictions;
	bool matches;
	bool searchRegion;
	bool pointsNotSeen;						//<! Show map point not predicted in the current  frame
};

class Settings{
public:
	TrackerSettings tracker;
	DetectorSettings detector;
	MatchingSettings matching;
	DisplaySettings display;
};

extern Settings options;

void initSettings(Settings& options);

#endif /* SETTINGS_HPP_ */
