/*
Collision Avoidance Node

This node controls the collision avoidance algorithm; in our case, an artificial potential field approach was used.  For more information about the algorithm, visit https://sites.google.com/site/auburn2011uav/.  It is highly recommended that you gain some background about the artificial field approach before delving into the code.

The node contains a map which stores information about the UAVs in the airspace, which is updated upon the receipt of a new telemetry update.  From these updates, a new force acting on each UAV is calculated.  Based on that new force, a new waypoint is forwarded to the coordinator for the UAV to begin traveling towards.
*/

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <queue>
#include <map>
#include <cmath>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"
#include "AU_UAV_ROS/standardDefs.h"

//our headers
#include "AU_UAV_ROS/pobject.h"
#include "AU_UAV_ROS/force.h"
#include "AU_UAV_ROS/standardFuncs.h"

/* ROS service clients for calling services from the coordinator */
ros::ServiceClient goToWaypointClient;
ros::ServiceClient requestWaypointInfoClient;

int count; /* keeps count of the number of goToWaypoint services requested */

/* constants */
const double EPSILON = 1e-6; /* used to check floating point numbers for equality */

std::map<int, AU_UAV_ROS::PlaneObject> pobjects; /* map of planes in the airspace.  The key is the plane id of the aircraft */

/* 
This function is run everytime new telemetry information from any plane is recieved.  With the new telemetry update, 
information about the plane is updated, including bearing, speed, current location, etc.  Additionally, we check to see
if the UAV has reached it's current destination, and, if so, update the destination of the UAV.
After updating, the calculateForces function is called to find a the new force acting on the UAV; from this new force,
a next waypoint is found and forwarded to the coordinator.
*/
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr &msg);

/*
Returns a GoToWaypoint service to call the coordiantor with.  The service contains a new waypoint for the UAV to navigate to
based on the current location information contained within the Telemetry update and the direction of the force vector.  The new
waypoint is a second away from the current location.
*/
AU_UAV_ROS::GoToWaypoint updatePath(const AU_UAV_ROS::TelemetryUpdate::ConstPtr &msg, AU_UAV_ROS::mathVector forceVector);

/*
This function checks to see if any planes have been removed from the airspace due to a collision.
To achieve this, the timestamp of the last update from each UAV is compared to the current time.
If the two times differ by more than three seconds, the UAV is deleted.  This function is meant for use with the 
evaluator, and may not be advantageous to use in the live testing due to packet loss, network latency, etc.
*/
void checkCollisions(void);

int main(int argc, char **argv)
{
	//standard ROS startup
	ros::init(argc, argv, "collisionAvoidance");
	ros::NodeHandle n;
	
	//subscribe to telemetry outputs and create clients for the goToWaypoint and requestWaypointInfo services
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	goToWaypointClient = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
	requestWaypointInfoClient = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");
	
	//initialize counting
	count = 0;

	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}

void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr &msg)
{
	AU_UAV_ROS::GoToWaypoint goToWaypointSrv;
	AU_UAV_ROS::RequestWaypointInfo requestWaypointInfoSrv;
	int planeID = msg->planeID;

	/* 
	Remove any planes that have been involved in a collision.
	Note: This function is made for use with the evaluator node, and may not work optimally in the field.
	To check for collisions, it compares the current time with the last update time of each of the UAVs.  
	If the values differ by more than three seconds, it is assumed the plane has been deleted.  However, 
	packet losses / network latency may render issues in the real world.
	*/
	checkCollisions();

	/* request this plane's current normal destination */
	requestWaypointInfoSrv.request.planeID = planeID;
	requestWaypointInfoSrv.request.isAvoidanceWaypoint = false;
	requestWaypointInfoSrv.request.positionInQueue = 0;

	if (!requestWaypointInfoClient.call(requestWaypointInfoSrv)){
		ROS_ERROR("Did not receive a response from the coordinator");
		return;
	}

	/* 
	if (plane has reached current destination waypoint)
		move on to next normal destination waypoint in queue
	*/
	if (findDistance(msg->currentLatitude, msg->currentLongitude, 
					requestWaypointInfoSrv.response.latitude, 
					requestWaypointInfoSrv.response.longitude) < COLLISION_THRESHOLD){

		/* request next normal destination */
		requestWaypointInfoSrv.request.positionInQueue = 1;

		if (!requestWaypointInfoClient.call(requestWaypointInfoSrv)){
			ROS_ERROR("Did not recieve a response from the coordinator");
			return;
		}
	}

	/* 
	Pseudocode for the following code lines.
	if (this plane is not in the map of planes and the telemetry update indicates that the plane is or has been moving towards a destination)
		if (the plane had been flying but we previously detected a collision and removed it)
			return - the plane is dead; the simulator is lagging behind 
		else
			the plane has registered a new TelemetryUpdate 
	else 
		return - the plane has just been initialized but it is not moving torwards a waypoint as of now 
	*/
	if (pobjects.find(planeID) == pobjects.end() && msg->currentWaypointIndex != -1){ 
		
		if (msg->currentWaypointIndex > 0){
			/* This plane is dead; it had previously been moving but was in a collision.
			The simulator is lagging behind and still reporting a telemetry update */
			return;
		}
		else{
			/* 
			a new plane has registered a TelemetryUpdate where the destination is not (0, 0)
			create new PlaneObject, collision radius is set to the distance traveled in one second
			*/
			AU_UAV_ROS::PlaneObject newPlane(MPS_SPEED, *msg); /* */
			pobjects[planeID] = newPlane; /* put the new plane into the map */

			/* update the destination of the PlaneObject with the value found with the requestWaypointInfoSrv call */
			AU_UAV_ROS::waypoint newDest; 

			newDest.latitude = requestWaypointInfoSrv.response.latitude;
			newDest.longitude = requestWaypointInfoSrv.response.longitude;
			newDest.altitude = requestWaypointInfoSrv.response.altitude;

			pobjects[planeID].setDestination(newDest);
		}
	}
	else if (pobjects.find(planeID) == pobjects.end()) /* new plane without waypoint set */
		return; 
	
	/* 
	Note: The requestWaypointInfo service returns a waypoint of -1000, -1000 when the UAV it cannot retrieve a destination from
	queue.

	Pseudocode:
	if (the plane has no destination){
		- for simulations, silence any force from this UAV so it does not affect flight paths by giving it an impossible location
		- update with the new time of latest update to avoid a false detection of a collision
	}
	else{
		update the plane with the new telemetry information

		if (the plane's destination has changed)
			update the map entry of the plane with this information
	}
	*/
	if (requestWaypointInfoSrv.response.latitude == -1000){ /* plane has no waypoints to go to */
		/* 
		useful for simulations, remove in real flights;		
		set location of finished planes to -1000, -1000 so no repulsive force is felt from this plane
		*/
		pobjects[planeID].setLatitude(-1000);
		pobjects[planeID].setLongitude(-1000);

		pobjects[planeID].update(); /* update the time of last update for this plane to acknowledge it is still in the air */
		return; 
	}
	else{
		pobjects[planeID].update(*msg); /* update plane with new position */

		/* if (destination has changed)
			update pobjects[planeID] with new destination */
		if (((pobjects[planeID].getDestination().latitude - requestWaypointInfoSrv.response.latitude) > EPSILON)
				|| ((pobjects[planeID].getDestination().longitude - requestWaypointInfoSrv.response.longitude) > EPSILON)
				|| ((pobjects[planeID].getDestination().latitude - requestWaypointInfoSrv.response.latitude) < EPSILON)
				|| ((pobjects[planeID].getDestination().longitude - requestWaypointInfoSrv.response.longitude) < EPSILON)){
			AU_UAV_ROS::waypoint newDest;

			newDest.latitude = requestWaypointInfoSrv.response.latitude;
			newDest.longitude = requestWaypointInfoSrv.response.longitude;
			newDest.altitude = requestWaypointInfoSrv.response.altitude;

			pobjects[planeID].setDestination(newDest);
		}
	}

	/* Most important line: Find the force acting on this UAV.  The plane is attracted to its waypoint, and repelled from other UAVs */
	AU_UAV_ROS::mathVector force = calculateForces(pobjects[planeID], pobjects);

	/* 
	Create a goToWaypoint service to send to the coordinator based on the force vector just calculated.  The destination will be one
	second away from the current position.
	*/
	goToWaypointSrv = updatePath(msg, force);

	goToWaypointSrv.request.isAvoidanceManeuver = true; 
	goToWaypointSrv.request.isNewQueue = true;

	if (goToWaypointClient.call(goToWaypointSrv)){
		count++;
		ROS_INFO("Received response from service request %d", (count-1));
	}
	else{
		ROS_ERROR("Did not receive response");
	}
}

AU_UAV_ROS::GoToWaypoint updatePath(const AU_UAV_ROS::TelemetryUpdate::ConstPtr &msg, AU_UAV_ROS::mathVector forceVector){
	AU_UAV_ROS::GoToWaypoint goToWaypointSrv; /* destination to go to */
	double forceTheta = forceVector.getDirection(); /* angle of the force vector with respect to North bearing */
	double d = MPS_SPEED/EARTH_RADIUS; /* angular distance traveled in one second */

	AU_UAV_ROS::waypoint currentPosition, newPosition;
	currentPosition.latitude = msg->currentLatitude;
	currentPosition.longitude = msg->currentLongitude;
	currentPosition.altitude = msg->currentAltitude;

	newPosition = calculateCoordinate(currentPosition, forceTheta, d);	/* find new position one second away based on direction of force */

	/* set up new goToWaypoint service */
	goToWaypointSrv.request.planeID = msg->planeID;

	goToWaypointSrv.request.latitude = newPosition.latitude;
	goToWaypointSrv.request.longitude = newPosition.longitude;
	goToWaypointSrv.request.altitude = newPosition.altitude;

	return goToWaypointSrv;
}

void checkCollisions(void){
	std::queue<int> planesToDelete;

	/* Iterate through the list of planes to check for planes that have been removed due to collisions */
	for (std::map<int, AU_UAV_ROS::PlaneObject>::iterator iter = pobjects.begin(); iter != pobjects.end(); iter++){
		/*
		if (we have not heard from this plane for more than three seconds)
			add this plane's ID to the queue of planes to delete 
		*/
		if ((ros::Time::now().toSec() - (iter->second).getLastUpdateTime()) > 3.0){
			planesToDelete.push(iter->first);
		}
	}

	while (!planesToDelete.empty()){
		/* 
		if the plane to delete still exists
			remove it
		*/
		if (pobjects.find(planesToDelete.front()) != pobjects.end())
			pobjects.erase(planesToDelete.front());

		planesToDelete.pop();
	}
}
