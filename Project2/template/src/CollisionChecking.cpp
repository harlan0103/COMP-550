///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "CollisionChecking.h"

// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    for(int i = 0; i < obstacles.size(); i++) {
    	// x, y is the lower left corner
    	// xMax = x + width
    	// yMax = y + height
    	double a = obstacles[i].x;
    	double b = obstacles[i].y;
    	double width = obstacles[i].width;
    	double height = obstacles[i].height;

    	// xMin <= x <= xMax && yMin <= y <= yMax
    	double aMax = a + width;
    	double bMax = b + height;

    	if(x >= a && x <= aMax && y >= y && y <= bMax) {
    		return false;
    	}
    }
    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    for(int i = 0; i < obstacles.size(); i++) {
    	double a = obstacles[i].x;
    	double b = obstacles[i].y;
    	double width = obstacles[i].width;
    	double height = obstacles[i].height;

    	// Check for ((xMin - r <= x <= xMax + r) && (yMin <= y <= yMax)) || ((xMin <= x <= xMax) && (yMin - r <= y <= yMax + r)) || ce(Euclidean norm) <= r
    	double aMax = a + width;
    	double bMax = b + height;
    	double radiusPower = radius * radius;

    	if(((a - radius <= x && aMax + radius >= x) && (b <= y && bMax >= y)) ||
    		((a <= x && aMax >= x) && (b - radius <= y && bMax + r >= y)) ||
    		((a - x) * (a - x) + (b - y) * (b - y) <= radiusPower) ||	// lower left vertex
    		((aMax - x) * (aMax - x) + (bMax - y) * (bMax - y) <= radiusPower) ||	// lower right vertex
    		((a - x) * (a - x) + (bMax - y) * (bMax - y) <= radiusPower) || // upper left vertex
    		((aMax - x) * (aMax - x) + (b - y) * (b - y) <= radiusPower))	// upper right vertex 
    	{
    		return false;
    	}
    }
    return true;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    return false;
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot> & robots, const std::vector<Rectangle> & obstacles,
               const std::vector<bool> & valid)
{
	int pointCount = 0;
	int validPointCount = 0;
	int circleCount = 0;
	int validCircleCount = 0;

	for(int i = 0; i < robots.size(); i++) {
		if(robots.type == 'p') {
			pointCount++;
			if(valid[i] == 1) {
				validPointCount++;
			}
		}
		else if(robots.type == 'c') {
			circleCount++;
			if(valid[i] == 1) {
				validCircleCount++;
			}
		}
	}

	std::count << "Valid point robots are: " << validPointCount << " , percentage is: " << validPointCount / pointCount << endl;
}
