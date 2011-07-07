/*
Implementation of pobject.h.  For comments on how to use these functions, visit pobject.h.  Comments in this file
are related to implemenation, not usage.
*/

#include "ros/ros.h"
#include "AU_UAV_ROS/pobject.h"
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/SimulatedPlane.h"	/* used for MAXIMUM_TURNING_ANGLE */

/* Implementation of the default constructor: Member variables are set to zero */
AU_UAV_ROS::PlaneObject::PlaneObject(void) : CObject(0.0, 0.0, 0.0), destination() {
	this->id = 0.0;
	this->altitude = 0.0;
	this->bearing = 0.0;
	this->actualBearing = 0.0;
	this->speed = 0.0;
	this->lastUpdateTime = ros::Time::now().toSec();
}

/* Implementation of an explicit value constructor.  The member variables are set to the values given in the constructor */
AU_UAV_ROS::PlaneObject::PlaneObject(int id, double bearing, double speed, double cRadius, 
				     const AU_UAV_ROS::waypoint &currentPos, const AU_UAV_ROS::waypoint &dest):
		CObject(currentPos.latitude, currentPos.longitude, cRadius) {
	this->id = id;
	this->altitude = currentPos.altitude;
	this->bearing = bearing;
	this->actualBearing = 0.0;	/* used in simulations, where beginning bearing is 0 degrees (usually). May need modification in the field */
	this->speed = speed;
	this->destination = dest;
	this->lastUpdateTime = ros::Time::now().toSec();
}

/* Implementation of an explicit value constructor.  The member variables are set to the values found in the Telemetry Update */
AU_UAV_ROS::PlaneObject::PlaneObject(double cRadius, const AU_UAV_ROS::TelemetryUpdate &msg):CObject(msg.currentLatitude, msg.currentLongitude, cRadius){
	this->id = msg.planeID;
	this->altitude = msg.currentAltitude;
	this->bearing = msg.targetBearing;
	this->actualBearing = 0.0;	/* used in simulations, where beginning bearing is 0 degrees (usually).  May need modification in the field */
	this->speed = msg.groundSpeed;
	this->destination.latitude = msg.destLatitude;
	this->destination.longitude = msg.destLongitude;
	this->destination.altitude = msg.destAltitude;
	this->lastUpdateTime = ros::Time::now().toSec();
}

/* Copy constructor */
AU_UAV_ROS::PlaneObject::PlaneObject(const AU_UAV_ROS::PlaneObject& pobj):
		CObject(pobj.getLatitude(), pobj.getLongitude(), pobj.getCollisionRadius()){
	this->id = pobj.id;
	this->altitude = pobj.altitude;
	this->bearing = pobj.bearing;
	this->actualBearing = pobj.actualBearing;
	this->speed = pobj.speed;
	this->destination = pobj.destination;
	this->lastUpdateTime = pobj.lastUpdateTime;
}

/* The following functions are simple modifiers to update member variables of the PlaneObject */
void AU_UAV_ROS::PlaneObject::setID(int id){
	this->id = id;
}

void AU_UAV_ROS::PlaneObject::setAltitude(double alt) {
	this->altitude = alt;
}

void AU_UAV_ROS::PlaneObject::setBearing(double aBearing){
	this->bearing = aBearing;
}

void AU_UAV_ROS::PlaneObject::setActualBearing(double aBearing){
	this->actualBearing = aBearing;
}

void AU_UAV_ROS::PlaneObject::setSpeed(double aSpeed){
	this->speed = aSpeed;
}

void AU_UAV_ROS::PlaneObject::setDestination(const AU_UAV_ROS::waypoint &dest){
	this->destination = dest;
}

/* Update the time of last update */
void AU_UAV_ROS::PlaneObject::update(void){
	this->lastUpdateTime = ros::Time::now().toSec();
}

/* Update the data members of the Plane object with the values contained in the Telemetry update */
void AU_UAV_ROS::PlaneObject::update(const AU_UAV_ROS::TelemetryUpdate &msg){
	this->setBearing(msg.targetBearing);	/* set bearing to destination */

	//calculate the actual bearing based on our maximum angle change
	//first create a temporary bearing that is the same as bearing but at a different numerical value
	double tempBearing = -1000;
	if((this->bearing) < 0)
	{
		tempBearing = this->bearing + 360;
	}
	else
	{
		tempBearing = this->bearing - 360;
	}
		
	double diff1 = abs(this->actualBearing - this->bearing);
	double diff2 = abs(this->actualBearing - tempBearing);
	
	//check for easy to calculate values first
	if(diff1 < MAXIMUM_TURNING_ANGLE || diff2 < MAXIMUM_TURNING_ANGLE)
	{
		//the difference is less than our maximum angle, set it to the bearing
		this->actualBearing = this->bearing;
	}
	else
	{
		//we have a larger difference than we can turn, so turn our maximum
		double mod;
		if(diff1 < diff2)
		{
			if(this->bearing > this->actualBearing) mod = MAXIMUM_TURNING_ANGLE;
			else mod = 0 - MAXIMUM_TURNING_ANGLE;
		}
		else
		{
			if(tempBearing > this->actualBearing) mod = MAXIMUM_TURNING_ANGLE;
			else mod = 0 - MAXIMUM_TURNING_ANGLE;
		}
	
		//add our mod, either +22.5 or -22.5
		this->actualBearing = this->actualBearing + mod;
	
		//tweak the value to keep it between -180 and 180
		if(this->actualBearing > 180) this->actualBearing = this->actualBearing - 360;
		if(this->actualBearing <= -180) this->actualBearing = this->actualBearing + 360;
	}

	/* update other data members */
	this->setLatitude(msg.currentLatitude); /* call inherited function */
	this->setLongitude(msg.currentLongitude); /* call inherited function */
	this->setAltitude(msg.currentAltitude);
	this->setSpeed(msg.groundSpeed);
	this->lastUpdateTime = ros::Time::now().toSec();
}

/* The following are simple accessor methods to retrieve a data variable */
int AU_UAV_ROS::PlaneObject::getID(void) const {
	return this->id;
}

double AU_UAV_ROS::PlaneObject::getAltitude(void) const {
  	return this->altitude;
}

double AU_UAV_ROS::PlaneObject::getBearing(void) const {
	return this->bearing;
}

double AU_UAV_ROS::PlaneObject::getActualBearing(void) const{
	return this->actualBearing;
}

double AU_UAV_ROS::PlaneObject::getSpeed(void) const {
	return this->speed;
}

double AU_UAV_ROS::PlaneObject::getLastUpdateTime(void) const{
	return this->lastUpdateTime;
}

AU_UAV_ROS::waypoint AU_UAV_ROS::PlaneObject::getDestination(void) const {
	return this->destination;
}

/* Find distance between two PlaneObjects.  The calling plane gives the starting latitude and longitude, 
and the object passed as a parameter gives the final latitude and longitude. */
double AU_UAV_ROS::PlaneObject::findDistance(const AU_UAV_ROS::PlaneObject& pobj) const {
	return AU_UAV_ROS::CObject::findDistance(pobj.getLatitude(), pobj.getLongitude()); /* call inherited function */
}

/* Find angle between two PlaneObjects.  The calling plane gives the starting latitude and longitude, 
and the object passed as a parameter gives the final latitude and longitude. */
double AU_UAV_ROS::PlaneObject::findAngle(const AU_UAV_ROS::PlaneObject& pobj) const {
	return AU_UAV_ROS::CObject::findAngle(pobj.getLatitude(), pobj.getLongitude()); /* call inherited function */
}

/* Overloaded equality operator */
AU_UAV_ROS::PlaneObject& AU_UAV_ROS::PlaneObject::operator=(const AU_UAV_ROS::PlaneObject& pobj) {
	this->id = pobj.id;
  	this->setLatitude(pobj.getLatitude()); /* call inherited function */
  	this->setLongitude(pobj.getLongitude()); /* call inherited function */
	this->altitude = pobj.altitude;
	this->bearing = pobj.bearing;
  	this->setCollisionRadius(pobj.getCollisionRadius()); /* call inherited function */
	this->speed = pobj.speed;
	this->destination = pobj.destination;
  	return *this;
}

/* 
Comparison function for two PlaneObject* parameteters.  Returns a true if the first 
object's distance to the destination is less than the second's, false otherwise.
*/
bool AU_UAV_ROS::cmpDistToDest(const PlaneObject &pobj1, const PlaneObject &pobj2){
	return (findDistance(pobj1.getLatitude(), pobj1.getLongitude(), pobj1.getDestination().latitude, pobj1.getDestination().longitude) < 
			findDistance(pobj2.getLatitude(), pobj2.getLongitude(), pobj2.getDestination().latitude, pobj2.getDestination().longitude));
}
