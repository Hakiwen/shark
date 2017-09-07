#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;



class OurLine 
{
	private:
		int lesserX;
		int greaterX;
		int lesserY;
		int greaterY;
		int currentHeight;
		int currentWidth;
		float angleD;
		float angleP;
		float inAngleP;
		float length;
		Point midPoint;
		Vec4i cvLine;
	public:
		OurLine ();
		OurLine (Vec4i);
		OurLine (Point, float, float);
		float getLength();
		float getAngle();
		Point getMidPoint();
		Vec4i getCvLine();
};