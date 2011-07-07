/*
Visualization Node

Outputs information about the flight path of a UAV to map.kml.  The map.kml is located in the flightData folder of the user's AU_UAV_ROS package.
In Google Earth, a NetworkLink can be established so that the display is refreshed every second based on the map.kml file.  By adding this NetworkLink, 
real time tracking can be achieved.

As of now, there are 10 different colors for the paths the UAVs can take.  The destinations of each of the UAVs are marked with a pushpin on the map
with the same color as the UAV path.  When the UAVs enter within 12 meters of each other, a black pushpin is placed on the map in order to indicate
a collision.  When the UAVs enter within 24 meters of each other, a white pushin is placed on the map to signal a conflict.
*/

//standard C++ headers
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <map>
#include <queue>

//ROS headers
#include "ros/ros.h"
#include "ros/package.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"
#include "AU_UAV_ROS/DeleteSimulatedPlane.h"
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/standardDefs.h"

#define NUM_PLANES 100
#define MAX_LINE_TYPES 10

/* Declaration of the data structure used to store information about the UAVs in the air */
namespace AU_UAV_ROS{
	struct PlaneInfo{
		AU_UAV_ROS::TelemetryUpdate lastUpdate;
		AU_UAV_ROS::waypoint dest;
		double lastUpdateTime;
	};
};

std::string planePaths[NUM_PLANES]; /* array of strings to store the paths taken by the UAVs */
std::map<int, AU_UAV_ROS::PlaneInfo> planes; /* map stores information about each of the UAVs.  The map key is the planeID of the UAV */
int maxPlaneID = -1; /* maximum planeID for the UAVs currently in the air */

std::vector<std::string> collisionPts; /* markers for when a UAV enters another's collision zone */
std::vector<std::string> conflictPts; /* markers for when a UAV enters another's conflict zone */

/* colors for fligth paths - formatted in hexidecimal: [transparency, red, green, blue] */
std::string colorComboArray[] = {"7f0000ff", "7f00ff00", "7fff0000", "7f00ffff", "7fff00ff", "7fffff00", "7f777777", "7f007777", "7f770077", "7f007777"};

ros::ServiceClient requestWaypointInfoClient; /* ROS service client to use RequestWaypointInfo service in coordinator */

/*
This code creates a file named map.kml that can be opened with Google earth.  The 
file contains the instructions to create a Google map with the paths taken by
each of the UAVs overlayed, along with markers for any conflict or collision.
*/
void makeKML();

/* 
This function is run everytime new telemetry information from any UAV is recieved.
The planePaths data structure is updated to include the new waypoint.  Additionally,
the last known position of the UAV is updated.
*/
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg);

/*
Checks to see if the distance between the current Telemetry update and the last
known positions of the other UAVs is small enough to be considered a conflict or
collision.  If so, the collisionPts or conflictPts vectors are updated to reflect 
the information.
*/
void markIssues(void);

/* 
Checks to see if any UAVs have removed from the airspace.  If so, the planes map is
updated to refect the new makeup of the airspace.
*/
void checkForRemovals(void);

int main(int argc, char **argv)
{
	atexit(makeKML); // call makeKML before program terminates 

	//standard ROS startup
	ros::init(argc, argv, "visualization");
	ros::NodeHandle n;
	
	//subscribe to telemetry outputs 
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	requestWaypointInfoClient = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");

	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}

void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg){
	std::stringstream ss;
	std::ofstream out;
	AU_UAV_ROS::RequestWaypointInfo requestWaypointInfoSrv;

	/* 
	if (the plane had been flying but we previously detected a collision and removed it)
		return - the plane is dead, but the simulator is lagging behind 
	*/
	if (planes.find(msg->planeID) == planes.end() && msg->currentWaypointIndex > 0)
		return;

	/* update maxPlaneID if necessary */
	if (msg->planeID > maxPlaneID)
		maxPlaneID = msg->planeID;

	/* Force 10 figure decimal precision in output */
	ss.setf(std::ios::fixed);
	ss.precision(10);

	ss<<msg->currentLongitude<<","<<msg->currentLatitude<<","<<msg->currentAltitude<<'\n'; /* use string stream to convert double to string */

	planePaths[msg->planeID]+=ss.str(); /* update path with new waypoint */
	planes[msg->planeID].lastUpdate = *msg; /* update with latest telemetry update */
	planes[msg->planeID].lastUpdateTime = ros::Time::now().toSec(); /* update time of last update */

	checkForRemovals(); /* check to see if any UAVs have been removed from the airspace */

	/* find UAV's destination */

	requestWaypointInfoSrv.request.planeID = msg->planeID;
	requestWaypointInfoSrv.request.isAvoidanceWaypoint = false;
	requestWaypointInfoSrv.request.positionInQueue = 0;

	if (requestWaypointInfoClient.call(requestWaypointInfoSrv)){ /* call was successful */
		AU_UAV_ROS::waypoint dest;

		dest.latitude = requestWaypointInfoSrv.response.latitude;
		dest.longitude = requestWaypointInfoSrv.response.longitude;
		dest.altitude = requestWaypointInfoSrv.response.altitude;

		planes[msg->planeID].dest = dest; /* store destination */
	}
		
	/* check for conflicts / collisions once every second, after every plane has been updated and write the new KML file */
	if (msg->planeID == maxPlaneID){
		markIssues();
		makeKML();
	}
}

void markIssues(void){
	std::stringstream ss;
	double distance;

	/* Force 10 figure decimal precision in output */
	ss.setf(std::ios::fixed);
	ss.precision(10);

	/* check for conflicts / collisions between planes - Note: iter->first returns the plane id of the current element */
	for (std::map<int, AU_UAV_ROS::PlaneInfo>::iterator iter = planes.begin(); iter != planes.end(); iter++){
		for (std::map<int, AU_UAV_ROS::PlaneInfo>::iterator iter2 = planes.begin(); iter2->first < iter->first; iter2++){

			/* find distance between the two UAVs */
			distance = findDistance((iter->second).lastUpdate.currentLatitude, (iter->second).lastUpdate.currentLongitude,
					       (iter2->second).lastUpdate.currentLatitude, (iter2->second).lastUpdate.currentLongitude);
			
			ss.str(""); /* clear string stream before input */

			if (distance < COLLISION_THRESHOLD){ /* update collisionPts */
				ss<<(iter->second).lastUpdate.currentLongitude<<","<<(iter->second).lastUpdate.currentLatitude<<","<<0;
				collisionPts.push_back(ss.str());
			}
			else if (distance < CONFLICT_THRESHOLD){ /* update conflictPts */
				ss<<(iter->second).lastUpdate.currentLongitude<<","<<(iter->second).lastUpdate.currentLatitude<<","<<0;
				conflictPts.push_back(ss.str());
			}
		}
	}	
}

void checkForRemovals(void){
	std::queue<int> planesToDelete;

	/* check if a plane has been removed due to a collision */
	for (std::map<int, AU_UAV_ROS::PlaneInfo>::iterator iter = planes.begin(); iter != planes.end(); iter++){
		/*
		if (we have not heard from this plane for more than three seconds)
			add this plane's ID to the queue of planes to delete 
		*/
		if ((ros::Time::now().toSec() - planes[iter->first].lastUpdateTime) > 3.0){
			planesToDelete.push(iter->first);
		}
	}

	/* remove planes that have been deleted due to a collision */
	while (!planesToDelete.empty()){
		/* 
		if the plane to delete still exists
			remove it
		*/
		if (planes.find(planesToDelete.front()) != planes.end())
			planes.erase(planesToDelete.front());

		planesToDelete.pop();
	}

	/* update the maxPlaneID based on removals from the airspace */
	while (planes.find(maxPlaneID) == planes.end() && maxPlaneID >= 0){
		maxPlaneID--;
	}	
}

void makeKML(){
	std::ofstream out;
	
	out.open((ros::package::getPath("AU_UAV_ROS")+"/flightData/map.kml").c_str()); /* create a new file map.kml - Note: Clears existing file */

	if (out.fail()) /* could not open file, exit function */
		return;

	/* standard KML header information */
	out<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	out<<"<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
	out<<"  <Document>\n";
	out<<"    <name>Paths</name>\n";
	out<<"    <description>Path of our UAVs</description>\n";
	
	/* set up styles - mainly different color types for the paths */
	for(int count = 0; count < MAX_LINE_TYPES; count++)
	{
		out<<"    <Style id=\"lineType"<<count<<"\">\n";
		out<<"      <LineStyle>\n";
		out<<"        <color>"<<colorComboArray[count % MAX_LINE_TYPES].c_str()<<"</color>\n";
		out<<"        <width>4</width>\n";
		out<<"      </LineStyle>\n";
		out<<"      <PolyStyle>\n";
		out<<"        <color>7fffffff</color>\n";
		out<<"      </PolyStyle>\n";
		out<<"    </Style>\n";
	}
	
	/* format to display 10 digit precision */
	out.setf(std::ios::fixed, std::ios::floatfield);
	out.precision(10);

	for (int i = 0; i < NUM_PLANES; ++i){
		if (planePaths[i] != ""){ /* plane registered a path, add its path to the file */
			/* KML formatting */
			out<<"    <Placemark>\n";
			out<<"      <name></name>\n";
			out<<"      <description></description>\n";
			out<<"        <styleUrl>#lineType"<<i % MAX_LINE_TYPES<<"</styleUrl>\n";
			out<<"      <LineString>\n";
			out<<"        <extrude>1</extrude>\n";
			out<<"        <tessellate>1</tessellate>\n";
			out<<"        <altitudeMode>absolute</altitudeMode>\n";
			out<<"        <coordinates>\n";
			out<<planePaths[i]; /* output path */
			out<<"        </coordinates>\n";
			out<<"      </LineString>\n";
			out<<"    </Placemark>\n";
		}
	}
	
	/* put placemarkers where UAV's destinations are */
	for (std::map<int, AU_UAV_ROS::PlaneInfo>::iterator iter = planes.begin(); iter != planes.end(); iter++){
		/*
		Note: iter->first gives the plane id of the current element being iterated on.
		Note: If the plane has no destiantion, (iter->second).dest.latitude will equal -1000
		*/

		if ((iter->second).dest.latitude != -1000){ /* UAV has a destination */
			out<<"    <Placemark>\n";
			out<<"      <name></name>\n";
			out<<"      <description></description>\n";
			out<<"      <Point>\n";
			out<<"        <coordinates>"<<(iter->second).dest.longitude<<","<<
					(iter->second).dest.latitude<<",0</coordinates>\n"; /* output destination coordinates */
			out<<"      </Point>\n";
			out<<"      <Style>\n";
			out<<"        <IconStyle>\n";
			out<<"          <Icon>\n";
			out<<"            <href>http://maps.google.com/mapfiles/kml/pushpin/wht-pushpin.png</href>\n";
			out<<"          </Icon>\n";
			out<<"          <colorMode>normal</colorMode>\n";
			out<<"          <color>"<<colorComboArray[iter->first % MAX_LINE_TYPES]<<"</color>\n"; /* color same as path of UAV */
			out<<"        </IconStyle>\n";
			out<<"      </Style>\n";
			out<<"    </Placemark>\n";
		}
	}

	for (unsigned int i = 0; i < conflictPts.size(); ++i){
		/* add white point to the map where conflicts occured */
		out<<"    <Placemark>\n";
		out<<"      <name></name>\n";
		out<<"      <description></description>\n";
		out<<"      <Point>\n";
		out<<"        <coordinates>"<<conflictPts[i]<<"</coordinates>\n";
		out<<"      </Point>\n";
		out<<"      <Style>\n";
		out<<"        <IconStyle>\n";
		out<<"          <Icon>\n";
		out<<"            <href>http://maps.google.com/mapfiles/kml/pushpin/wht-pushpin.png</href>\n";
		out<<"          </Icon>\n";
		out<<"        </IconStyle>\n";
		out<<"      </Style>\n";
		out<<"    </Placemark>\n";
	}
	
	for (unsigned int i = 0; i < collisionPts.size(); ++i){
		/* add black point to the map where collisions occured */
		out<<"    <Placemark>\n";
		out<<"      <name></name>\n";
		out<<"      <description></description>\n";
		out<<"      <Point>\n";
		out<<"        <coordinates>"<<collisionPts[i]<<"</coordinates>\n";
		out<<"      </Point>\n";
		out<<"      <Style>\n";
		out<<"        <IconStyle>\n";
		out<<"          <Icon>\n";
		out<<"            <href>http://maps.google.com/mapfiles/kml/pushpin/wht-pushpin.png</href>\n";
		out<<"          </Icon>\n";
		out<<"          <colorMode>normal</colorMode>\n";
		out<<"          <color>ff000000</color>\n";	/* override white default */
		out<<"        </IconStyle>\n";
		out<<"      </Style>\n";
		out<<"    </Placemark>\n";
	}

	/* append ending KML information */
	out<<"  </Document>\n";
	out<<"</kml>\n";

	out.close();
}
