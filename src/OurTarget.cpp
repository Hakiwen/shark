#include "OurTarget.h"


OurTarget::OurTarget() {}
OurTarget::OurTarget(OurLine line1, OurLine line2, float maxAngleSeperation, int maxSeperation, float desiredAngle, float angleRange)
{
  this->maxAngleSeperation = maxAngleSeperation;
  this->maxSeperation = maxSeperation;
  this->desiredAngle = desiredAngle;
  this->angleRange = angleRange;
  //lineSeperation = sqrt((line1.getMidPoint().x - line2.getMidPoint().x)*(line1.getMidPoint().x - line2.getMidPoint().x) + (line1.getMidPoint().y - line2.getMidPoint().y)*(line1.getMidPoint().y - line2.getMidPoint().y));
  maxAngle = max(line1.getAngle(), line2.getAngle());
  minAngle = min(line1.getAngle(), line2.getAngle());
  //cout << "line1Angle: " << line1.getAngle() << endl;
  //cout << "line2Angle: " << line2.getAngle() << endl;
  commonAxis = (maxAngle - minAngle)/2 + minAngle;
  //cout << commonAxis << endl;
  commonAxisR = commonAxis*(CV_PI/180);
  
  
  //cout << "l1 angle: " << line1.getAngle() << "l2 angle: "<< line2.getAngle() << " CommonAxis: " << commonAxis << endl;
  originalLines = cv::Mat(2, 4, CV_32F);
  transformedLines = cv::Mat(2, 4, CV_32F);
  originalLines.at<float>(0) = line1.getCvLine()[0];
  originalLines.at<float>(1) = line2.getCvLine()[0];
  originalLines.at<float>(2) = line1.getCvLine()[2];
  originalLines.at<float>(3) = line2.getCvLine()[2];
  originalLines.at<float>(4) = line1.getCvLine()[1];
  originalLines.at<float>(5) = line2.getCvLine()[1];
  originalLines.at<float>(6) = line1.getCvLine()[3];
  originalLines.at<float>(7) = line2.getCvLine()[3];
  
  rotationMatrix = cv::Mat(2, 2, CV_32F);
 
  
  if(commonAxis > 0)
  {
    
    rotationMatrix.at<float>(0) = cos(CV_PI/2 -  (commonAxisR));
    rotationMatrix.at<float>(1) = - sin(CV_PI/2 - (commonAxisR));
    rotationMatrix.at<float>(2) = sin(CV_PI/2 - (commonAxisR));
    rotationMatrix.at<float>(3) = cos(CV_PI/2 - (commonAxisR));
    
    transformedLines = rotationMatrix * originalLines;
  }
  else
  {
    rotationMatrix.at<float>(0) = cos(- CV_PI/2 +  (commonAxisR));
    rotationMatrix.at<float>(1) = - sin( - CV_PI/2 + (commonAxisR));
    rotationMatrix.at<float>(2) = sin( - CV_PI/2 + (commonAxisR));
    rotationMatrix.at<float>(3) = cos( - CV_PI/2 + (commonAxisR));
    
    transformedLines = rotationMatrix.inv() * originalLines;
  }
  lowerPoint = max(min(transformedLines.at<float>(4), transformedLines.at<float>(6)), min(transformedLines.at<float>(5), transformedLines.at<float>(7)));
  upperPoint = min(max(transformedLines.at<float>(4), transformedLines.at<float>(6)), max(transformedLines.at<float>(5), transformedLines.at<float>(7)));
  x1 = (max(transformedLines.at<float>(0) , transformedLines.at<float>(2)) - min(transformedLines.at<float>(0), transformedLines.at<float>(2)))/2 + min(transformedLines.at<float>(0), transformedLines.at<float>(2));
  x2 = (max(transformedLines.at<float>(1) , transformedLines.at<float>(3)) - min(transformedLines.at<float>(1), transformedLines.at<float>(3)))/2 + min(transformedLines.at<float>(1), transformedLines.at<float>(3));
  
  lineSeperation = max(x1, x2) - min(x1, x2);
  //cout<<"x1: " << x1 << " x2: " << x2 <<endl;
  adjustedLines = cv::Mat(2, 5, CV_32F);
  adjustedLines.at<float>(0) = x1;
  adjustedLines.at<float>(1) = x2;
  adjustedLines.at<float>(2) = x1;
  adjustedLines.at<float>(3) = x2;
  adjustedLines.at<float>(4) = lineSeperation/2 + min(x1, x2);
  adjustedLines.at<float>(5) = lowerPoint;
  adjustedLines.at<float>(6) = lowerPoint;
  adjustedLines.at<float>(7) = upperPoint;
  adjustedLines.at<float>(8) = upperPoint;
  adjustedLines.at<float>(9) = (upperPoint - lowerPoint)/2 + lowerPoint;
  
  
  outputLines = cv::Mat(2, 5, CV_32F);
  
  if(commonAxis > 0)
    outputLines = rotationMatrix.inv() * adjustedLines;
  else
    outputLines = rotationMatrix  * adjustedLines;
  
  newLine1[0] = outputLines.at<float>(0);
  newLine1[1] = outputLines.at<float>(5);
  newLine1[2] = outputLines.at<float>(2);
  newLine1[3] = outputLines.at<float>(7);
  newLine2[0] = outputLines.at<float>(1);
  newLine2[1] = outputLines.at<float>(6);
  newLine2[2] = outputLines.at<float>(3);
  newLine2[3] = outputLines.at<float>(8);
  
  midPoint.x = outputLines.at<float>(4);
  midPoint.y = outputLines.at<float>(9);
  
  width = lineSeperation;
  length = upperPoint - lowerPoint;
  
  rectTarget = RotatedRect(midPoint, Size2f(width,length), commonAxis + 90);
  
}


Point OurTarget::getMidPoint()
{
  return midPoint;
}

int OurTarget::getWidth()
{
  return width;
}

int OurTarget::getLength()
{
  return length;
}

float OurTarget::getOrientation()
{
  return commonAxis;
}

bool OurTarget::getIsValidPair()
{
  if((abs(maxAngle - minAngle) < maxAngleSeperation) && (abs(lineSeperation) < maxSeperation))
  { 
    //cout<< "Lines at: (" << midPoint.x << "," << midPoint.y << ") Passed seperation tests" << endl;
    //cout<< "Desire - range: " << desiredAngle - angleRange << " Desire + range: " << desiredAngle + angleRange << endl;
    if ((commonAxis > (desiredAngle - angleRange)) && (commonAxis < (desiredAngle + angleRange)) || (commonAxis > (-desiredAngle - angleRange)) && (commonAxis < (-desiredAngle + angleRange)))
    {
      //cout<< "Lines at: (" << midPoint.x << "," << midPoint.y << ") Passed angle tests" << endl;
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

RotatedRect OurTarget::getRectTarget()
{
  return rectTarget;
}

cv::Mat OurTarget::getAdjustedLines()
{
  return adjustedLines;
}

Vec4i OurTarget::getOutLine1()
{
  return newLine1;
}

Vec4i OurTarget::getOutLine2()
{
  return newLine2;
}