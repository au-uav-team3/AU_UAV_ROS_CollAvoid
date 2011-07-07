/*
PlaneObject

This class inherits from CObject, and is used to model the dynamic UAVs in the simulation.  Information about the UAVs in the airspace, including id, bearing, speed, current location, time of last update, and destination are stored within this class.  As this class inherits from CObject, it includes a collision radius member defining the the circular zone other objects should not enter; if another object enters this zone, it is considered a collision. 
*/

#ifndef PLANE_OBJECT_H
#define PLANE_OBJECT_H

#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/cobject.h"
#include "AU_UAV_ROS/standardDefs.h"

namespace AU_UAV_ROS{
	class PlaneObject : public CObject {
  	public:
		/* Default constructor.  Initializes all data members to zero, and updates the time of last update to the current time. */
    		PlaneObject(void);

		/* 
		Note: The cRadius member defines a circular zone other objects should not enter; if another object enters this zone, 
		it is considered a collision.  To check for a collision, use the isColliding() method from CObject.
		*/
		/* 
		Explicit value constructor: Takes a plane id, bearing, speed, collision radius, current position waypoint, and final
		destination waypoint and creates a new PlaneObject.
		*/		
		PlaneObject(int id, double bearing, double speed, double cRadius, 
			    const AU_UAV_ROS::waypoint &currentPos, const AU_UAV_ROS::waypoint &dest);

		/*
		Explicit value constructor: Takes a collision radius and a telemetry update and creates a new PlaneObject.
		*/
		PlaneObject(double cRadius, const AU_UAV_ROS::TelemetryUpdate &msg);

		/* Copy constructor */
		PlaneObject(const PlaneObject& pobj);

		/* Modifier functions: Allow the client to modify the plane's id, altitude, bearing, speed, etc. */
		void setID(int id); 
		void setAltitude(double altitude);
		void setBearing(double bearing);		/* set bearing to destination */
		void setActualBearing(double aBearing); 	/* set actual bearing in the air */
		void setSpeed(double speed);
		void setDestination(const AU_UAV_ROS::waypoint &destination);

		/* Update the lastUpdateTime member of the plane to the current time */
		void update(void);

		/* Update the plane's data members with the information contained within the telemetry update */
		void update(const AU_UAV_ROS::TelemetryUpdate &msg);

		/* Accessor functions: Allow the client to access the plane's id, altitude, bearing, spped, etc. */
		int getID(void) const;
		double getAltitude(void) const;
		double getBearing(void) const;			/* get bearing to destination */
		double getActualBearing(void) const;		/* get actual bearing in the air */
		double getSpeed(void) const;
		double getLastUpdateTime(void) const;
		AU_UAV_ROS::waypoint getDestination(void) const;

		/* 
		Find distance / Cardinal angle between two planes. The calling plane gives the starting latitude and longitude, 
		and the object passed as a parameter gives the final latitude and longitude.
		*/
		double findDistance(const PlaneObject& pobj) const;
		double findAngle(const PlaneObject& pobj) const;

		/* Overloaded equality operator */
		PlaneObject& operator=(const PlaneObject& pobj);
	private:
		/* Private data members */
		int id;
		double altitude;
		double bearing;			/* get bearing to destination */
		double actualBearing;		/* get current bearing in the air */
		double speed;
		double lastUpdateTime;
		AU_UAV_ROS::waypoint destination;
	};

	/* 
	Comparision function for use with std::sort.  The planes are compared based on destination.  
	The function returns true if pobj1 is closer to its destination than pobj2, false otherwise.
	*/	
	bool cmpDistToDest(const PlaneObject &pobj1, const PlaneObject &pobj2);	
};

#endif
