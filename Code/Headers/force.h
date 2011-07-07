/*
Force

For information about how forces are calculated, visit https://sites.google.com/site/auburn2011uav/.  It is highly recommended that you gain some background about the artificial field approach before delving into the code.

This header file contains a collection of functions used to calculate the force acting on the UAV.  Additionally, removal
of looping is present.
*/

#ifndef FORCE_H
#define FORCE_H

#include <map>

#include "AU_UAV_ROS/pobject.h"
#include "AU_UAV_ROS/vmath.h"

namespace AU_UAV_ROS{
	/*
	The primary method used to calculate and sum all forces acting on one plane.  Returns a mathVector representing the total
	force acting on the plane; the angle of the vector returned is with respect to Cardinal directions.
	*/
	mathVector calculateForces(PlaneObject &pobj1, std::map<int, PlaneObject> &pobjects);

	/*
	Calculates the repulsion force between two planes.
	The distance parameter is the distance between the planes in meters, maxDistance is
	the distance over which field pobj1 is in acts, rAngle is the angle of the repulsive force, 
	fieldAngle is the angle between the bearing of the plane generating the force to the location
	of pobj1, and aAngle is the angle between the bearing of pobj1 and the location of its destination.
	All angles are in the Cartesian plane.
	*/
	static mathVector calculateRepulsionForce(const PlaneObject &pobj1, double distance, 
					double maxDistance, double rAngle, double fieldAngle, double aAngle);
	
	/*
	Calculates the attractive force to the plane's destination.
	*/
	static mathVector calculateAttractionForce(const PlaneObject &pobj1);
 
	/* 
	Modifies the total force acting on pobj1 so that the new path does not differ more than 22.5 degrees from the current 
	bearing.
	*/
	static void makeForceViable(PlaneObject &pobj1, mathVector &tForce);				
									
	/*
	Returns true if pobj1 is looping around it's destination, false otherwise.
	*/			
	static bool inLoop(const PlaneObject &pobj1, mathVector &tForce);

	/*
	Calculates the distance between the center of "circle" generated when a plane is looping around its destination
	and the plane's destination.  The angle parameter is the number of degrees to the left or right of the UAV the center of
	turning is located.  The angle parameter is based off of the turning angle per second.  Because the simulator
	uses straight line paths between each of the waypoints given, a looping UAV actually creates a hexadecagon.  
	For example, in this code 22.5 is our turning angle. Therefore, the angle provided for this method is either 101.25 or -101.25 since 
	[(180-22.5)/2 = 78.75; 78.75 + 22.5 = 101.25].
	*/
	double findLoopDistance(const PlaneObject &pobj1, double angle);
};


#endif
