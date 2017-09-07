#include "OurLine.h"

//constructs line segment from endpoints
OurLine::OurLine() {}

OurLine::OurLine(Vec4i cvLine)
{
	this->cvLine = cvLine;
	lesserX = min(cvLine[2], cvLine[0]);
	greaterX = max(cvLine[2], cvLine[0]);
	lesserY = min(cvLine[1], cvLine[3]);
	greaterY = max(cvLine[1], cvLine[3]);
	angleP = atan2(float((cvLine[3] - cvLine[1])),float((cvLine[2] - cvLine[0])));

	angleD = angleP*(180/CV_PI);
	currentWidth = greaterX - lesserX;
	currentHeight = greaterY - lesserY;
	length = sqrt(float(currentHeight*currentHeight + currentWidth*currentWidth));
	midPoint = Point((greaterX - lesserX)/2 + lesserX, (greaterY - lesserY)/2 + lesserY);
}

//construct line segment object from location and quasi-polar coordinates
OurLine::OurLine(Point midPoint, float angleD, float length)
{
	this->midPoint = midPoint;
	this->angleD = angleD;
	this->length = length;
	angleP = angleD*(CV_PI/180);

	cvLine[0] = midPoint.x + (length/2)*cos(angleP);
	cvLine[1] = midPoint.y + (length/2)*sin(angleP);

	cvLine[2] = midPoint.x - (length/2)*cos(angleP);
	cvLine[3] = midPoint.y - (length/2)*sin(angleP);

	lesserX = min(cvLine[2], cvLine[0]);
	greaterX = max(cvLine[2], cvLine[0]);
	lesserY = min(cvLine[1], cvLine[3]);
	greaterY = max(cvLine[1], cvLine[3]);

	currentWidth = greaterX - lesserX;
	currentHeight = greaterY - lesserY;

}  

//returns line length in pixels
float OurLine::getLength()
{
	return length;
}


//returns angle in degrees
float OurLine::getAngle()
{
	return angleD;
}


//returns the Mid Point as an cv::Point
Point OurLine::getMidPoint()
{
	return midPoint;
}

//returns the line as a CV Vec4i
Vec4i OurLine::getCvLine()
{
	return cvLine;
}