


#include <string>
#include <iomanip>
#include <sstream>
#include <linux/input.h>
#include <fcntl.h>

#include "OurTarget.h"

#include <sl/Camera.hpp>



#include <opencv2/highgui.hpp>

#include <opencv2/video/tracking.hpp>




using namespace std;
using namespace cv;
using namespace sl;

Point ptPIR(0, 0);
int newCoordsPIR = false;

void mouse_callback(int eventPIR, int xPIR,int yPIR, int flagPIR, void *paramPIR)
{
	if (eventPIR == EVENT_LBUTTONDOWN)
	{
		ptPIR.x = xPIR;
		ptPIR.y = yPIR;
		newCoordsPIR = true;
	}
}



int main(int argc, char *argv[])
{
	//ZED initialize and Params
	Camera zed;

	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_VGA;
	init_params.camera_fps = 15;
	init_params.coordinate_units = UNIT_MILLIMETER;
	init_params.depth_minimum_distance = 500;
	init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
	ERROR_CODE err;
	
	err = zed.open(init_params);
	cout <<"zed opened\n";
	if (err != SUCCESS) 
	{
        std::cout << errorCode2str(err) << std::endl;
        zed.close();
        return EXIT_FAILURE; // quit if an error occurred
 	}


	zed.setDepthMaxRangeValue(10000);
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE_FILL;

	//Line finding params
	float rhod = 1;
	float th = 180;
	float thresh = 80;
	float range = 20; //in degrees
	float desiredAngle = 90;
	int searching = 1;
	float depth_value;
	int numTargets = 0;
	
	bool foundTarget = false;
	
	int cannyThresh1 = 10;
	int cannyThresh2 = 40;
	
	int minLineLengthP = 100;
	int maxLineGapP = 5;
	int maxLineSeperation = 20;
	float maxAngleSeperation = 10;

	
	int startXOfRect = 0;
	int colsOfRect;
	int startYOfRect = 0;
	int rowsOfRect;
	float orientation;
	//actual width and height is 2*valueListedBelow

	int widthOfRect;
	int heightOfRect;
	int isTracked = false;
	int framesSinceSeen = 0;
	double ticks = 0;
	
	Point lastMidPoint = Point(0,0);
	
	
	//Eval Vars
	std::chrono::steady_clock::time_point begin;
	std::chrono::steady_clock::time_point end;
	std::chrono::steady_clock::time_point lastStart = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point thisStart;
	std::chrono::steady_clock::time_point thisCycleTime;
	float avgCycleTime = 0;
	float avgNotCycleTime = 0;
	int numCyclesFound = 0;
	int numCyclesNotFound = 0;
	

	//CV Objects
	sl::Mat inFrame_zl, inFrame_zr, inFrame_zd, inFrame_zq;
	cv::Mat inFrame_l, inFrame_r, inFrame_d, inFrame, inFrameCopy,  outFrame, outFrameTemp;
	vector<Vec2f> lines;
	vector<Vec4i> linesP;
	vector<Vec4i> validLinesP;
	namedWindow("InputFrame", WINDOW_AUTOSIZE);
	//setMouseCallback("InputFrame", mouse_callback);
	//namedWindow("OutputFrame", WINDOW_AUTOSIZE);


	//Kalman Filter Nonsense
	int stateDim = 10;
	int measureDim = 6;
	int controlDim = 0;
	unsigned int typeKF = CV_32F;
	KalmanFilter lineTrackingKF(stateDim, measureDim, controlDim);

	//initialize x vector
	cv::Mat state(stateDim, 1, typeKF); //[x,y,z,vx,vy,vz,w,h]

	//initialize y vector
	cv::Mat measurement(measureDim, 1, typeKF);//[mx,my,mz,mw,mh]

	

	//Set C matrix
	
	
	// Transition State Matrix A -
    // Note: set dT at each processing step!
    // [ 1 0 0  0  dT 0  0  0 0 0 ]
    // [ 0 1 0  0  0  dT 0  0 0 0 ]
    // [ 0 0 1  0  0  0  dT 0 0 0 ]
    // [ 0 0 0  1  0  0  0 dT 0 0 ]  
    // [ 0 0 0  0  1  0  0  0 0 0 ]
    // [ 0 0 0  0  0  1  0  0 0 0 ]
    // [ 0 0 0  0  0  0  1  0 0 0 ]
    // [ 0 0 0  0  0  0  0  1 0 0 ]
    // [ 0 0 0  0  0  0  0  0 1 0 ]
    // [ 0 0 0  0  0  0  0  0 0 1 ]
    cv::setIdentity(lineTrackingKF.transitionMatrix);
	
    // Measure Matrix H - This is what you have access to
    // [ 1 0 0 0 0 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 0 0 0 0 ]
    // [ 0 0 1 0 0 0 0 0 0 0 ]
    // [ 0 0 0 1 0 0 0 0 0 0 ]
    // [ 0 0 0 0 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 0 0 0 0 1 ]
    lineTrackingKF.measurementMatrix = cv::Mat::zeros(measureDim, stateDim, typeKF);
    lineTrackingKF.measurementMatrix.at<float>(0,0) = 1.0f;
    lineTrackingKF.measurementMatrix.at<float>(1,1) = 1.0f;
    lineTrackingKF.measurementMatrix.at<float>(2,2) = 1.0f;
    lineTrackingKF.measurementMatrix.at<float>(3,3) = 1.0f;
    lineTrackingKF.measurementMatrix.at<float>(4,8) = 1.0f;
    lineTrackingKF.measurementMatrix.at<float>(5,9) = 1.0f;
    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0      0     0     0     0  0 ]
    // [ 0    Ey  0     0     0      0     0     0     0  0 ]
    // [ 0    0   Ez    0     0      0     0     0     0  0 ]
    // [ 0    0   0     Et    0      0     0     0     0  0 ]
    // [ 0    0   0     0     Evx    0     0     0     0  0 ]
    // [ 0    0   0     0     0      Evy   0     0     0  0 ]
    // [ 0    0   0     0     0      0     Evz   0     0  0 ]
    // [ 0    0   0     0     0      0     0     Evt   0  0 ]
    // [ 0    0   0     0     0      0     0     0    Ew  0 ]
    // [ 0    0   0     0     0      0     0     0     0  Eh] 
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    lineTrackingKF.processNoiseCov = cv::Mat::zeros(stateDim, stateDim, typeKF);
    lineTrackingKF.processNoiseCov.at<float>(0,0) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(1,1) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(2,2) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(3,3) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(4,4) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(5,5) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(6,6) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(7,7) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(8,8) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(9,9) = 1e-2;
    
    
    //Measurement Noise Covairance Martix
    //[ Wx 0  0  0  0  0  ]std::chrono::steady_clock::time_point end;
    //[ 0  Wy 0  0  0  0  ]
    //[ 0  0  Wz 0  0  0  ]
    //[ 0  0  0  Wt 0  0  ]
    //[ 0  0  0  0  Ww 0  ]
    //[ 0  0  0  0  0  Wh ]
    lineTrackingKF.measurementNoiseCov = cv::Mat::zeros(measureDim, measureDim, typeKF);
    lineTrackingKF.measurementNoiseCov.at<float>(0,0) = 1e-2;
    lineTrackingKF.measurementNoiseCov.at<float>(1,1) = 1e-2;
    lineTrackingKF.measurementNoiseCov.at<float>(2,2) = 1e-2;
    lineTrackingKF.measurementNoiseCov.at<float>(3,3) = 1e-2;
    lineTrackingKF.measurementNoiseCov.at<float>(4,4) = 1e-2;
    lineTrackingKF.measurementNoiseCov.at<float>(5,5) = 1e-2;

    
	
	for(;;)
	{	
		thisStart = std::chrono::steady_clock::now();
		//thisCycleTime = (thisStart - lastStart).count();
		cout<< "Total Cycle Time: " << std::chrono::duration_cast<std::chrono::microseconds>(thisStart - lastStart).count() << endl;
		/*if(foundTarget)
		{
		  numCyclesFound ++;
		  avgCycleTime = (avgCycleTime*(numCyclesFound - 1) + std::chrono::duration_cast<std::chrono::microseconds>(thisCycleTime))/numCyclesFound;
		  cout<< "Average Tracking Cycle Time: " << avgCycleTime << endl;
		}
		else
		{
		  numCyclesNotFound ++;
		  avgNotCycleTime = (avgNotCycleTime(numCyclesNotFound - 1) + std::chrono::duration_cast<std::chrono::microseconds>(thisCycleTime))/numCyclesNotFound;
		  cout<< "Average Non Tracking Cycle Time: " << avgNotCycleTime << endl;
		}*/
		lastStart = thisStart;
		begin = std::chrono::steady_clock::now();
		cv::Mat inFrameTemp;
		double precTick = ticks;
		ticks = (double) cv::getTickCount();
		double dT = (ticks - precTick) / cv::getTickFrequency(); //sfleconds
		//Grabs and converts all the images from ZED
		err == zed.grab(runtime_parameters);

		err = zed.retrieveImage(inFrame_zl, VIEW_LEFT);
		inFrame_l = cv::Mat(inFrame_zl.getHeight(), inFrame_zl.getWidth(), CV_8UC4, inFrame_zl.getPtr<sl::uchar1>(sl::MEM_CPU));
		
		err = zed.retrieveMeasure(inFrame_zd);
		inFrame = inFrame_l;
		
		err = zed.retrieveImage(inFrame_zq, VIEW_DEPTH);
		inFrame_d = cv::Mat(inFrame_zq.getHeight(), inFrame_zq.getWidth(), CV_8UC4, inFrame_zq.getPtr<sl::uchar1>(sl::MEM_CPU));
		inFrame_d.copyTo(inFrameCopy);
		end = std::chrono::steady_clock::now();
		cout<< "Time to Grab Images: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << endl;
		
		if(inFrame.empty())
		{
			cout << "Blank frame \n";
			return -1;
		}
		//cout<< isTracked <<endl;
		if (isTracked)
		{
			
			lineTrackingKF.transitionMatrix.at<float>(0,4) = dT;
			lineTrackingKF.transitionMatrix.at<float>(1,5) = dT;
			lineTrackingKF.transitionMatrix.at<float>(2,6) = dT;
			lineTrackingKF.transitionMatrix.at<float>(3,7) = dT;
			//cout<< "dT:" << endl << dT << endl;
			
			state = lineTrackingKF.predict();
			//cout << "State post:" << endl << state << endl;
			
			widthOfRect = state.at<float>(7);
			heightOfRect = state.at<float>(8);
			ptPIR.x = state.at<float>(0) - widthOfRect/2;
			ptPIR.y = state.at<float>(1) - heightOfRect/2;
			orientation = state.at<float>(3);
			//Selects 200x100 slice of image
			//Bounds for X of rect
			if(ptPIR.x > widthOfRect) 
				startXOfRect = ptPIR.x - widthOfRect;
			else
				startXOfRect = 0;
			if(ptPIR.x < (inFrame_l.cols - widthOfRect))
				colsOfRect = widthOfRect*2;
			else
				colsOfRect = inFrame.cols - startXOfRect;
			
			//Bounds for Y of Rect			
			if (ptPIR.y > heightOfRect) 
				startYOfRect = ptPIR.y - heightOfRect;
			else
				startYOfRect = 0;
			if(ptPIR.y < (inFrame.rows - heightOfRect))
				rowsOfRect = heightOfRect*2;
			else
				rowsOfRect = inFrame.rows - startYOfRect;
			
			
			RotatedRect sRect = RotatedRect(Point(state.at<float>(0), state.at<float>(1)), Size2f(state.at<float>(8), state.at<float>(9)), state.at<float>(3) + 90);
			//cout << "Estimated Angle: " << state.at<float>(3) << endl;
			
			Point2f verticesSRect[4];
			sRect.points(verticesSRect);
			for (int i = 0; i < 4; i++)
			line(inFrameCopy, verticesSRect[i], verticesSRect[(i+1)%4], Scalar(255,255,255));
			//rectangle(inFrameCopy, Point(startXOfRect, startYOfRect), Point(startXOfRect + colsOfRect, startYOfRect + rowsOfRect), Scalar(255, 255, 255));
			
			putText(inFrameCopy, to_string(state.at<float>(2)), Point(300, 300), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255 , 255));
			circle(inFrameCopy, Point(state.at<float>(0), state.at<float>(1)), 5, Scalar(255,255,255),3);
			//inFrame(cv::Rect(startXOfRect, startYOfRect, colsOfRect, rowsOfRect)).copyTo(inFrameTemp);
			
			
		  
		}
		else
		{
			inFrame.copyTo(inFrameTemp);
		}
		
		
		
		

		//Shows where the click is made
		

		//Finds edges in the input frame
		begin = std::chrono::steady_clock::now();
		Canny(inFrame, outFrameTemp, cannyThresh1, cannyThresh2, 3);
		//converts image color system to a meaningful output form
		cvtColor(outFrameTemp, outFrame, COLOR_GRAY2BGR);
	    
		//If the line to track has not yet been selected/not tracking    
		//cout<<"startXOfRect: " << startXOfRect << " startYOfRect: " << startYOfRect << " cols: " << colsOfRect << " rows: " << rowsOfRect << endl;
		//cout<< inFrameTemp;
		HoughLinesP( outFrameTemp, linesP, rhod, CV_PI/th, thresh, minLineLengthP, maxLineGapP);
		
		end = std::chrono::steady_clock::now();
		cout<< "Time to Run Transform: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << endl;
		//size_t vL= 0;
		if (linesP.size() > 0)
		{
			begin = std::chrono::steady_clock::now();
			foundTarget = false;
			OurLine ourLines[linesP.size()];
			OurTarget ourTargets[linesP.size()*linesP.size()];
			numTargets = 0;
			lastMidPoint = Point(0,0);
			for( size_t i = 0; i < linesP.size(); i++ )
			{
				/*Vec4i currentLineP;
				//if (isTracked)
				//{
				  
				for( size_t n = 0; n < 4; n ++)
				{
					  if ((n % 2) == 0)
					      currentLineP[n] = linesP[i][n];
					  else
					      currentLineP[n] = linesP[i][n];
				}
				//}
				/*else
				{
				  
				  for( size_t n = 0; n < 4; n ++)
				  {
					  if ((n % 2) == 0)
					      currentLineP[n] = linesP[i][n];
					  else
					      currentLineP[n] = linesP[i][n];
				  }
				}*/
				
				ourLines[i] = OurLine(linesP[i]);
				//line(inFrameCopy, Point(linesP[i][0], linesP[i][1]), Point( linesP[i][2], linesP[i][3]), Scalar(0,255,255), 3 , 8);
				//cout<<"left Pt1: " << Point(linesP[i][0], linesP[i][1]) << " left Pt2: "<< Point( linesP[i][2], linesP[i][3]) << endl;
				Vec4i tempLine = ourLines[i].getCvLine();
				for (size_t n = 0; n < i; n ++)
				{
				  size_t k = i*linesP.size() + n;
				  //cout<<"Line object size: " << sizeof(OurLine) << endl;
				  begin = std::chrono::steady_clock::now();
				  ourTargets[k] = OurTarget(ourLines[i], ourLines[n], maxAngleSeperation, maxLineSeperation, desiredAngle, range);
				  
				  //line(inFrameCopy, ourLines[i].getMidPoint(), ourLines[n].getMidPoint(), Scalar(0, 255,255), 3, 8 );
				  
				  if (ourTargets[k].getIsValidPair() && !foundTarget)
				  {
				    numTargets ++;
				    Point midPoint = ourTargets[k].getMidPoint();
				    cout<< "Number of Targets Found: " << numTargets << endl;
				    foundTarget = true;
				    //if(numTargets > 0 && ((lastMidPoint.x - midPoint.x)*(lastMidPoint.x - midPoint.x) + (lastMidPoint.y - midPoint.y)*(lastMidPoint.y - midPoint.y)) > 4000)
				    //{
					    //for(size_t q = 0; q < numTargets; q++)
					    //{
					    //if ((comm[i] > (90 - range)) && (angleD[i] < (90 + range)) || (angleD[i] > (-90 - range)) && (angleD[i] < (-90 + range)))
					    //{
						    lastMidPoint = midPoint;
						    cv::Mat adjustedLines = ourTargets[k].getAdjustedLines();
						    //cout<< "Pt1: " << Point(ourTargets.getOutLine1()[0],ourTargets.getOutLine1()[1]) << "Pt2: " << Point(ourTargets.getOutLine1()[2],ourTargets.getOutLine1()[3]) << endl;
						    //line(inFrameCopy, Point(currentLineP[0], currentLineP[1]), Point(currentLineP[2], currentLineP[3]), Scalar(0,0,255), 3, 8 );
						    //line(inFrameCopy, Point(ourTargets.getOutLine1()[0],ourTargets.getOutLine1()[1]), Point(ourTargets.getOutLine1()[2],ourTargets.getOutLine1()[3]) , Scalar(0,0,255), 3, 8);
						    //line(inFrameCopy, Point(ourTargets.getOutLine2()[0],ourTargets.getOutLine2()[1]), Point(ourTargets.getOutLine2()[2],ourTargets.getOutLine2()[3]), Scalar(0,0,255), 3, 8);
						    inFrame_zd.getValue(midPoint.x, midPoint.y, & depth_value);
						    
						    RotatedRect rRect = ourTargets[k].getRectTarget();
						    Point2f vertices[4];
						    rRect.points(vertices);
						    for (int i = 0; i < 4; i++)
							line(inFrameCopy, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
						    
						    
						    framesSinceSeen = 0;
						    measurement.at<float>(0) = midPoint.x;
						    measurement.at<float>(1) = midPoint.y;
						    measurement.at<float>(2) = depth_value;
						    measurement.at<float>(3) = ourTargets[k].getOrientation();
						    measurement.at<float>(4) = ourTargets[k].getWidth();
						    measurement.at<float>(5) = ourTargets[k].getHeight();
						    putText(inFrameCopy, to_string(measurement.at<float>(2)), Point(300, 250), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
						    circle(inFrameCopy, Point(measurement.at<float>(0), measurement.at<float>(1)), 5, Scalar(0,255,0),3);
						    //cout << "Angle:" << ourTargets[k].getOrientation() << endl;
						    //cout << "Measurement:" << measurement << endl;
						    if (!isTracked)
						    {
						      lineTrackingKF.errorCovPre.at<float>(0,0) = 1;
						      lineTrackingKF.errorCovPre.at<float>(1,1) = 1;
						      lineTrackingKF.errorCovPre.at<float>(2,2) = 1;
						      lineTrackingKF.errorCovPre.at<float>(3,3) = 1;
						      lineTrackingKF.errorCovPre.at<float>(4,4) = 1;
						      lineTrackingKF.errorCovPre.at<float>(5,5) = 1;
						      lineTrackingKF.errorCovPre.at<float>(6,6) = 1;
						      lineTrackingKF.errorCovPre.at<float>(7,7) = 1;
						      lineTrackingKF.errorCovPre.at<float>(8,8) = 1;
						      lineTrackingKF.errorCovPre.at<float>(9,9) = 1;
						      
						      state.at<float>(0) = measurement.at<float>(0);
						      state.at<float>(1) = measurement.at<float>(1);
						      state.at<float>(2) = measurement.at<float>(2);
						      state.at<float>(3) = measurement.at<float>(3);
						      state.at<float>(4) = 0;
						      state.at<float>(5) = 0;
						      state.at<float>(6) = 0;
						      state.at<float>(7) = 0;
						      state.at<float>(8) = measurement.at<float>(4);
						      state.at<float>(9) = measurement.at<float>(5);
						      
						      lineTrackingKF.statePost = state;
						      
						      
			
						      
						      
						      //cout<<"Initializing filter" << endl;
						      end = std::chrono::steady_clock::now();
						      cout<< "Time to instantiate tracking: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << endl;
						    }
						    else
						    {
						      
						      lineTrackingKF.correct(measurement);
						      end = std::chrono::steady_clock::now();
						      cout<< "Time to correct KF: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << endl;
						      
						      //cout<<"Correcting Estimate" << endl;
						    }
					    //}
					      //}
					isTracked = true;   
				    //}
				}
			      }
			}	  
		}
		else
		{
			framesSinceSeen ++;
			if(framesSinceSeen >= 100)
			  isTracked = false;
		}
	begin = std::chrono::steady_clock::now();
	imshow("InputFrameCopy", inFrameCopy);
	end = std::chrono::steady_clock::now();
	cout<< "Time to output image: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << endl;
	//imshow("OutputFrame", outFrame);

	if (waitKey(5) >= 0)
	    break;
	}
}
