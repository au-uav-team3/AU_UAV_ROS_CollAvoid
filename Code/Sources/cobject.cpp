/*
The implementation of cobject.h.  For comments on how to use these functions, visit cobject.h.  Comments in this file
are related to implementation, not usage.
*/

#include <math.h>
#include "AU_UAV_ROS/cobject.h"
#include "AU_UAV_ROS/standardFuncs.h" /* for PI, EARTH_RADIUS in meters */

/* Constructor to initialize the latitude, longitude, and collision radius of the CObject */
AU_UAV_ROS::CObject::CObject(double lat, double lon, double cRadius) :
	latitude(lat), longitude(lon), collisionRadius(cRadius) {}

/* Copy constructor */
AU_UAV_ROS::CObject::CObject(const AU_UAV_ROS::CObject& cobj) {
	this->latitude = cobj.latitude;
	this->longitude = cobj.longitude;
	this->collisionRadius = cobj.collisionRadius;
}

/* Modifier functions to change the value of data members */
void AU_UAV_ROS::CObject::setLatitude(double lat) {
	this->latitude = lat;
}

void AU_UAV_ROS::CObject::setLongitude(double lon) {
	this->longitude = lon;
}

void AU_UAV_ROS::CObject::setCollisionRadius(double cRadius) {
	this->collisionRadius = cRadius;
}

/* Accessor functions to get the value of data members */
double AU_UAV_ROS::CObject::getLatitude(void) const {
	return this->latitude;
}

double AU_UAV_ROS::CObject::getLongitude(void) const {
	return this->longitude;
}

double AU_UAV_ROS::CObject::getCollisionRadius(void) const {
	return this->collisionRadius;
}

/* Find the distance between this collision object and a given latitude and longitude */
double AU_UAV_ROS::CObject::findDistance(double lat, double lon) const {
	double latDiff = 0.0, lonDiff = 0.0;
	double squareHalfChord = 0.0, angularDistance = 0.0;

	/* Get difference in latitude and longitude in radians */
	latDiff = (this->latitude - lat) * PI / 180.0;
	lonDiff = (this->longitude - lon) * PI / 180.0;

	/* Haversine math: see http://www.movable-type.co.uk/scripts/latlong.html for more information */

	/* Find the square of half of the chord length between the two points */
	/* sin(lat difference)^2 + cos(lat1) * cos(lat2) * sin(lon difference)^2 */
	squareHalfChord = pow(sin(latDiff / 2), 2) + 
                    	  pow(sin(lonDiff / 2), 2) *
                    	  cos(this->latitude * PI / 180.0) *
                    	  cos(lat * PI / 180.0);

	/* Calulate the angular distance in radians */
	/* 2 * arctan(sqrt(squareHalfChord), sqrt(1 - squareHalfChord) */
	angularDistance = 2 * atan2(sqrt(squareHalfChord), sqrt(1 - squareHalfChord));

	/* Return result in meters */
	return angularDistance * EARTH_RADIUS;
}

double AU_UAV_ROS::CObject::findDistance(const CObject& cobj) const {
	return findDistance(cobj.latitude, cobj.longitude);
}

/* 
Finds the angle between this CObject's latitude and longitude and the final latitude and longitude given as parameters 
*/
double AU_UAV_ROS::CObject::findAngle(double lat, double lon) const {
	double lonDiff = 0.0, angle = 0.0;
	double x = 0.0, y = 0.0;
	double thisLat;

	/* Convert latitudes to radians */
	lat *= PI / 180.0;
	thisLat = this->latitude * PI /180.0;

	lonDiff = (lon - this->longitude) * PI / 180.0; /* convert difference in longitude to radians */
	
	/* Haversine math: see http://www.movable-type.co.uk/scripts/latlong.html for more information */
	y = sin(lonDiff)*cos(lat);
	x = cos(thisLat)*sin(lat)-sin(thisLat)*cos(lat)*cos(lonDiff);

	angle = atan2(y,x) * 180.0 / PI; /* convert final result to degrees */

	return angle;
}

double AU_UAV_ROS::CObject::findAngle(const AU_UAV_ROS::CObject& cobj) const {
	return findAngle(cobj.latitude, cobj.longitude);
}

/* Determine if a collision object is colliding with another in the airspace */
bool AU_UAV_ROS::CObject::isColliding(const AU_UAV_ROS::CObject& cobj) const {
	double distance = findDistance(cobj);

	/* Shorten distance by radius of each object to see if the
	collision fields are intersecting each other */
	distance -= this->collisionRadius;
	distance -= cobj.collisionRadius;

	/* If distance is now zero or less there is a collision */
	return (distance <= 0);
}

/* Overloaded equality operator */
AU_UAV_ROS::CObject& AU_UAV_ROS::CObject::operator=(const AU_UAV_ROS::CObject& cobj) {
  this->latitude = cobj.latitude;
  this->longitude = cobj.longitude;
  this->collisionRadius = cobj.collisionRadius;
  return *this;
}

