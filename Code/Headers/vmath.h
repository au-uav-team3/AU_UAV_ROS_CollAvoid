/*
Vector Math

The vector math class handles calculations with vectors used throughout the project (in force.cpp in particular).  Specifically, a polar coordinate system based in the Cartesian plane is utilized.
*/

#ifndef MATH_VECTOR_H
#define MATH_VECTOR_H

namespace AU_UAV_ROS{
	class mathVector {
 	public:
		 
	/* Explicit value constructor - Specify a magnitude and direction in the Cartesian plane for the math vector */
    	mathVector(double m = 0, double d = 0); 

		
	/* Copy constructor */ 
    	mathVector(const mathVector& mV);

	/* Accessor methods: Allow the client to access the magnitude and direction of the math vector */
	double getDirection(void) const;
	double getMagnitude(void) const;

	/* Modifier methods: Allow the client to modify the magnitude and direction of the math vector */
	void setDirection(double d);
    	void setMagnitude(double m);

	/* Overloaded addition operators.  Used to add two vectors. */
    	const mathVector operator+(const mathVector& mV) const;
    	mathVector& operator+=(const mathVector& mV);
	
	/* Overloaded subtraction operators.  Used to subtract two vectors. */
	const mathVector operator-(const mathVector& mV) const;
	mathVector& operator-=(const mathVector& mV);

	/* Overloaded multiplication operators.  Multiplies the magnitude of the vector by a constant */
	const mathVector operator*(double val) const;
	mathVector& operator*=(double val);

	/* Overloaded division operators.  Divides the magnitude of the vector by a constant. */
	const mathVector operator/(double val) const;
	mathVector& operator/=(double val);
		
	/* Overloaded equality operator.  Set two math vectors equal to each other. */
	mathVector& operator=(const mathVector& mV);

	/* Overloaded multiplication operator.  Multiplies the magnitude of the vector by a constant */
	friend const mathVector operator*(double val, const mathVector& mV);

	/* Overloaded division operator.  Divides the magnitude of the vector by a constant. */
	friend const mathVector operator/(double val, const mathVector& mV);

	private:
		/* Private data members specifying the magnitude and direction of the math vector */
		double magnitude;
		double direction;
	};	
};

#endif

