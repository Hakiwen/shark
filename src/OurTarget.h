#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "OurLine.h"

class OurTarget
{
	private:
		float depth;
		Point midPoint;
		float maxAngleSeperation;
		int maxSeperation;
		float lineSeperation;
		int width;
		int height;
		float commonAxis;
		float commonAxisR;
		float maxAngle;
		float minAngle;
		float lowerPoint;
		float upperPoint;
		float x1;
		float x2;
		Vec4i newLine1;
		Vec4i newLine2;
		bool  isValidPair;
		RotatedRect  rectTarget;
		float desiredAngle;
		float angleRange;
		cv::Mat adjustedLines;
		cv::Mat originalLines;
		cv::Mat transformedLines;
		cv::Mat outputLines;
	public:
		OurTarget ();
		OurTarget ( OurLine , OurLine , float, int, float, float);
		Point getMidPoint();
		Point getRotatedMidPoint();
		int getWidth();
		int getHeight();
		bool getIsValidPair();
		float getOrientation();
		float getOrientationR();
		RotatedRect getRectTarget();
		cv::Mat getAdjustedLines();
		Vec4i getOutLine1();
		Vec4i getOutLine2();
};

cv::Mat rotate(cv::Mat, float, bool);