/*
 * system.hpp
 *
 *  Created on: Mar 25, 2011
 *      Author: yasir
 */

#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <camera.hpp>
#include <KalmanTracker.hpp>


class TrackingSystem{
public:
	KalmanTracker tracker;

};

#endif /* SYSTEM_HPP_ */
