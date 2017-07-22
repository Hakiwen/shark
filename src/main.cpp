

#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <linux/input.h>
#include <fcntl.h>

#include <sl/Camera.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
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
	int searching = 1;
	float depth_value;
	
	int minLineLengthP = 80;
	int maxLineGapP = 3;

	
	int startXOfRect = 0;
	int colsOfRect;
	int startYOfRect = 0;
	int rowsOfRect;

	//actual width and height is 2*valueListedBelow

	int widthOfRect;
	int heightOfRect;
	int isTracked = false;
	int framesSinceSeen = 0;
	double ticks = 0;
	/*
	//user sets rhod, the magnitude precision for the hough transform
	cout << "rho definition\n";
	cin >> rhod;

	//user sets th, the angle precision for the hough transform
	cout << "theta\n";
	cin >> th;

	//set thresh, the vote threshold for the hough transform
	//greater=less lines
	cout << "threshold\n";
	cin >> thresh;

	//set range, the range of angles in radians/pi around horizontal for lines
	cout << "range\n";
	cin >> range;

	//set transform to use
	cout << "0 for HL, 1 for HLP, 2 for HC\n";
	cin >> searching;
	*/


	//CV Objects
	sl::Mat inFrame_zl, inFrame_zr, inFrame_zd;
	cv::Mat inFrame_l, inFrame_r, inFrame_d, inFrame, inFrameCopy,  outFrame, outFrameTemp;
	vector<Vec2f> lines;
	vector<Vec4i> linesP;
	vector<Vec4i> validLinesP;
	namedWindow("InputFrame", WINDOW_AUTOSIZE);
	//setMouseCallback("InputFrame", mouse_callback);
	//namedWindow("OutputFrame", WINDOW_AUTOSIZE);


	//Kalman Filter Nonsense
	int stateDim = 8;
	int measureDim = 5;
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
    // [ 1 0 0  dT 0  0  0 0 ]
    // [ 0 1 0  0  dT 0  0 0 ]
    // [ 0 0 1  0  0  dT 0 0 ]
    // [ 0 0 0  1  0  0  0 0 ]
    // [ 0 0 0  0  1  0  0 0 ]
    // [ 0 0 0  0  0  1  0 0 ]
    // [ 0 0 0  0  0  0  1 0 ]
    // [ 0 0 0  0  0  0  0 1 ]
    cv::setIdentity(lineTrackingKF.transitionMatrix);
	
    // Measure Matrix H - This is what you have access to
    // [ 1 0 0 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 0 0 ]
    // [ 0 0 1 0 0 0 0 0 ]
    // [ 0 0 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 0 0 1 ]
	lineTrackingKF.measurementMatrix = cv::Mat::zeros(measureDim, stateDim, typeKF);
	lineTrackingKF.measurementMatrix.at<float>(0) = 1.0f;
	lineTrackingKF.measurementMatrix.at<float>(9) = 1.0f;
	lineTrackingKF.measurementMatrix.at<float>(18) = 1.0f;
	lineTrackingKF.measurementMatrix.at<float>(30) = 1.0f;
	lineTrackingKF.measurementMatrix.at<float>(39) = 1.0f;
	// Process Noise Covariance Matrix Q
      // [ Ex   0   0     0     0      0     0     0  ]
      // [ 0    Ey  0     0     0      0     0     0  ]
      // [ 0    0   Ez    0     0      0     0     0  ]
      // [ 0    0   0     Ev_x  0      0     0     0  ]
      // [ 0    0   0     0     Ev_y   0     0     0  ]
      // [ 0    0   0     0     0      Ev_z  0     0  ]
      // [ 0    0   0     0     0      0     E_w   0  ]
      // [ 0    0   0     0     0      0     0     E_h]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    lineTrackingKF.processNoiseCov.at<float>(0) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(9) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(18) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(27) = 5.0f;
    lineTrackingKF.processNoiseCov.at<float>(36) = 5.0f;
    lineTrackingKF.processNoiseCov.at<float>(45) = 5.0f;
    lineTrackingKF.processNoiseCov.at<float>(54) = 1e-2;
    lineTrackingKF.processNoiseCov.at<float>(63) = 1e-2;
    
    cv::setIdentity(lineTrackingKF.measurementNoiseCov, cv::Scalar(1e-1));
	
	for(;;)
	{	
		cv::Mat inFrameTemp;
		double precTick = ticks;
		ticks = (double) cv::getTickCount();
		double dT = (ticks - precTick) / cv::getTickFrequency(); //sfleconds
		//Grabs and converts all the images from ZED
		err == zed.grab(runtime_parameters);

		err = zed.retrieveImage(inFrame_zl, VIEW_LEFT);
		inFrame_l = cv::Mat(inFrame_zl.getHeight(), inFrame_zl.getWidth(), CV_8UC4, inFrame_zl.getPtr<sl::uchar1>(sl::MEM_CPU));

		err = zed.retrieveImage(inFrame_zr, VIEW_RIGHT);
		inFrame_r = cv::Mat(inFrame_zr.getHeight(), inFrame_zr.getWidth(), CV_8UC4, inFrame_zr.getPtr<sl::uchar1>(sl::MEM_CPU));
		
		err = zed.retrieveMeasure(inFrame_zd);
		inFrame = inFrame_l;
		inFrame.copyTo(inFrameCopy);
		
		
		if(inFrame.empty())
		{
			cout << "Blank frame \n";
			return -1;
		}
		cout<< isTracked <<endl;
		if (isTracked)
		{
			
			lineTrackingKF.transitionMatrix.at<float>(3) = dT;
			lineTrackingKF.transitionMatrix.at<float>(12) = dT;
			lineTrackingKF.transitionMatrix.at<float>(21) = dT;
			
			//cout<< "dT:" << endl << dT << endl;
			
			state = lineTrackingKF.predict();
			cout << "State post:" << endl << state << endl;
			
			widthOfRect = state.at<float>(6);
			heightOfRect = state.at<float>(7);
			ptPIR.x = state.at<float>(0) - widthOfRect/2;
			ptPIR.y = state.at<float>(1) - heightOfRect/2;
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
			
			if((startYOfRect > 0) && (startXOfRect > 0) && (colsOfRect > 0) && (rowsOfRect > 0))
			{
			  rectangle(inFrameCopy, Point(startXOfRect, startYOfRect), Point(startXOfRect + colsOfRect, startYOfRect + rowsOfRect), Scalar(255, 255, 255));
			  putText(inFrameCopy, to_string(state.at<float>(2)), Point(300, 300), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0 , 255));
			  inFrame(cv::Rect(startXOfRect, startYOfRect, colsOfRect, rowsOfRect)).copyTo(inFrameTemp);
			}
			else
			{
			  isTracked = false;
			  inFrame.copyTo(inFrameTemp);
			}
			
		  
		}
		else
		{
			inFrame.copyTo(inFrameTemp);
		}
		
		
		
		

		//Shows where the click is made
		

		//Finds edges in the input frame
		Canny(inFrameTemp, outFrameTemp, 50, 200, 3);
		//converts image color system to a meaningful output form
		cvtColor(outFrameTemp, outFrame, COLOR_GRAY2BGR);
	    
		//If the line to track has not yet been selected/not tracking    
		cout<<"startXOfRect: " << startXOfRect << " startYOfRect: " << startYOfRect << " cols: " << colsOfRect << " rows: " << rowsOfRect << endl;
		//cout<< inFrameTemp;
		HoughLinesP( outFrameTemp, linesP, rhod, CV_PI/th, thresh, minLineLengthP, maxLineGapP);
		    
		//size_t vL= 0;
		if (linesP.size() > 0)
		{
			for( size_t i = 0; i < linesP.size(); i++ )
			{
				Vec4i currentLineP;
				if (isTracked)
				{
				  
				  for( size_t n = 0; n < 4; n ++)
				  {
					  if ((n % 2) == 0)
					      currentLineP[n] = linesP[i][n] + startXOfRect;
					  else
					      currentLineP[n] = linesP[i][n] + startYOfRect;
				  }
				}
				else
				{
				  
				  for( size_t n = 0; n < 4; n ++)
				  {
					  if ((n % 2) == 0)
					      currentLineP[n] = linesP[i][n];
					  else
					      currentLineP[n] = linesP[i][n];
				  }
				}
				int lesserX = min(currentLineP[2], currentLineP[0]);
				int greaterX = max(currentLineP[2], currentLineP[0]);
				int lesserY = min(currentLineP[1], currentLineP[3]);
				int greaterY = max(currentLineP[1], currentLineP[3]);
				Point midPoint((greaterX - lesserX)/2 + lesserX, (greaterY - lesserY)/2 + lesserY);
				/*if(abs(midPoint.x) > 700 || abs(midPoint.y) > 400)
				{
				  cout<<"lesserX: " << lesserX <<" lesserY: " << lesserY << " greaterX: " << greaterX << " greaterY "<< greaterY << endl;
				  cout<<"bad measurement" << endl;
				  return 0;
				}
				*/
				float angleP = atan2((greaterY - lesserY),(greaterX - lesserX));
				float angleD = angleP*(180/CV_PI);
				int currentWidth = greaterX - lesserX;
				int currentHeight = greaterY - lesserY;
				   
				cout << "Midpoint: (" << midPoint.x << "," << midPoint.y << ")\n";
				if ((angleD > (90 - range)) && (angleD < (90 + range)) || (angleD > (-90 - range)) && (angleD < (-90 + range)))
				{
    
					line(inFrameCopy, Point(currentLineP[0], currentLineP[1]), Point(currentLineP[2], currentLineP[3]), Scalar(0,0,255), 3, 8 );
					inFrame_zd.getValue(midPoint.x, midPoint.y, & depth_value);
					
					framesSinceSeen = 0;
					measurement.at<float>(0) = midPoint.x;
					measurement.at<float>(1) = midPoint.y;
					measurement.at<float>(2) = depth_value;
					measurement.at<float>(3) = currentWidth;
					measurement.at<float>(4) = currentHeight;
					cout << "Angle:" << angleP*(180/CV_PI) << endl;
					cout<< "Measurement:" << measurement << endl;
					if (!isTracked)
					{
					  lineTrackingKF.errorCovPre.at<float>(0) = 1;
					  lineTrackingKF.errorCovPre.at<float>(9) = 1;
					  lineTrackingKF.errorCovPre.at<float>(18) = 1;
					  lineTrackingKF.errorCovPre.at<float>(27) = 1;
					  lineTrackingKF.errorCovPre.at<float>(36) = 1;
					  lineTrackingKF.errorCovPre.at<float>(45) = 1;
					  lineTrackingKF.errorCovPre.at<float>(54) = 1;
					  lineTrackingKF.errorCovPre.at<float>(63) = 1;
					  
					  state.at<float>(0) = measurement.at<float>(0);
					  state.at<float>(1) = measurement.at<float>(1);
					  state.at<float>(2) = measurement.at<float>(2);
					  state.at<float>(3) = 0;
					  state.at<float>(4) = 0;
					  state.at<float>(5) = 0;
					  state.at<float>(6) = measurement.at<float>(3);
					  state.at<float>(7) = measurement.at<float>(4);
					  
					  lineTrackingKF.statePost = state;
					 
					 
					  cout<<"Initializing filter" << endl;
					  
					}
					else
					{
					  lineTrackingKF.correct(measurement);
					  cout<<"Correcting Estimate" << endl;
					}
				}
			    }
			    isTracked = true;
			     
		}
		else
		{
			framesSinceSeen ++;
			if(framesSinceSeen >= 100)
			  isTracked = false;
		}
	imshow("InputFrameCopy", inFrameCopy);
	//imshow("OutputFrame", outFrame);

	if (waitKey(5) >= 0)
	    break;
	}
}
