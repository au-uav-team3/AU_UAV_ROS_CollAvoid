/* 
Collision Object

This object is used to model static obstacles in an airspace.  Although not necessary for our current objectives, it may prove useful in the future.  The class stores a current latitude and longitude, along with a collision radius defining the distance in which other objects should not enter.
*/

#ifndef C_OBJECT_H
#define C_OBJECT_H

namespace AU_UAV_ROS{
	class CObject {
	public:
		/* 
		Constructors: Take a latitude, longitude, and collision radius (in meters) defining the distance other objects should not enter.
		*/
		CObject(double lat, double lon, double cRadius);
		CObject(const CObject& cobj);

		/* Modifier functions: Allow the client to modify the data members of this class */
		void setLatitude(double lat);
		void setLongitude(double lon);
		void setCollisionRadius(double cRadius);

		/* Accessor functions: Allow the client to access the data members of this class */
		double getLatitude(void) const;
		double getLongitude(void) const ;
		double getCollisionRadius(void) const;

		/* Find the distance between this collision object and another collision object or a latitude and longitude */
		double findDistance(double lat, double lon) const;
		double findDistance(const CObject& cobj) const;

		/* Find the Cardinal angle between this collision object and another collision object or a latitude and longitude */
		double findAngle(double lat, double lon) const;
		double findAngle(const CObject& cobj) const;

		/* Returns true if a collision object is within the cRadius meters of this collision object, false otherwise */
		bool isColliding(const CObject& cobj) const;

		/* Overloaded equality operator */
		CObject& operator=(const CObject& cobj);
	private:
		/* Data members */
		double latitude;
		double longitude;
		double collisionRadius;
	};
};

#endif

