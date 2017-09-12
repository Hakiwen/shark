


#include <string>
#include <iomanip>
#include <sstream>

#include "OurTarget.h"

#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#ifdef ZED 
#include <sl/Camera.hpp>
#include <opencv2/gpu/gpu.hpp>
#endif

#ifdef GUST
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#endif

using namespace std;
using namespace cv;

#ifdef ZED
using namespace sl;
ERROR_CODE err;
RuntimeParameters runtime_parameters;

cv::Mat findLines(cv::Mat input_frame, int cannyThresh1, int cannyThresh2, float rhod, float th, int minLineLengthP, int maxLineGapP)
{
	cv::Mat linesPMat;
	cv::gpu::GpuMat g_outFrameTemp, g_linesP;
	cv::gpu::HoughLinesBuf hlBuf;
	cv::cvtColor(input_frame, input_frame, CV_RGB2GRAY);

	cv::gpu::CudaMem zero_copy_inFrame(input_frame, cv::gpu::CudaMem::ALLOC_ZEROCOPY);
	cv::gpu::GpuMat g_inFrame = zero_copy_inFrame.createGpuMatHeader();
	
	cv::gpu::Canny(g_inFrame, g_outFrameTemp, cannyThresh1, cannyThresh2, 3);

	cv::gpu::HoughLinesP( g_outFrameTemp, g_linesP, hlBuf, rhod, CV_PI/th, minLineLengthP, maxLineGapP, 20);
	g_linesP.download(linesPMat);
	return linesPMat;
		
}

cv::Vec2f findDepth(OurTarget inputTarget, sl::Mat inputDepth, int numDepthPoints)
{
	cv::Mat rTargetPoints(2, numDepthPoints, CV_32F);
	Point rMidPoint = inputTarget.getRotatedMidPoint();
	float ySeperation = inputTarget.getHeight()/numDepthPoints;
	for(int a = 1; a < numDepthPoints + 1; a ++)
	{
		rTargetPoints.at<float>(0, a - 1) = rMidPoint.x;
		rTargetPoints.at<float>(1, a - 1) = rMidPoint.y - inputTarget.getHeight()/2 + a*inputTarget.getHeight()/ySeperation;
	}
	
	cv::Mat targetPoints = rotate(rTargetPoints, inputTarget.getOrientationR(), 0);
	float depth_array[numDepthPoints];
	for(int a = 0; a < numDepthPoints; a ++)
	{
		inputDepth.getValue(targetPoints.at<float>(0,a), targetPoints.at<float>(1,a), &depth_array[a]);
	}
	float currentDepthSample[numDepthPoints + 1];
	std::vector<std::vector<std::vector<float>>> totalDepthSampleSet;
	for(int sampleDepthSize = 0; sampleDepthSize < numDepthPoints; sampleDepthSize ++)
	{
		std::vector<std::vector<float>> currentDepthSampleSetGroup;
		std::vector<float> currentDepthSampleSet;
		std::vector<int> currentDepthSampleIndexSet;
		int vec_location = 0;
		int d_array_iterator = 0;
		while (vec_location < sampleDepthSize)
		{
			currentDepthSampleSet.push_back(depth_array[d_array_iterator]);
			currentDepthSampleIndexSet.push_back(d_array_iterator);
			vec_location ++;
			d_array_iterator ++;
		}
		currentDepthSampleSetGroup.push_back(currentDepthSampleSet);
		
		while(d_array_iterator < sampleDepthSize)
		{
			if(currentDepthSampleSet[vec_location])
			currentDepthSampleSet.pop_back();
			vec_location --;
			currentDepthSampleSet.push_back(depth_array[d_array_iterator]);
			vec_location ++;

			currentDepthSampleSetGroup.push_back(currentDepthSampleSet);
			d_array_iterator ++;
		}
		float oneBack = 0;
		
		while(oneBack > numDepthPoints - 1)
		{
		}	
	}
}

#endif

Point ptPIR(0, 0);
int newCoordsPIR = false;
bool sim_mode;






void mouse_callback(int eventPIR, int xPIR,int yPIR, int flagPIR, void *paramPIR)
{
	if (eventPIR == EVENT_LBUTTONDOWN)
	{
		ptPIR.x = xPIR;
		ptPIR.y = yPIR;
		newCoordsPIR = true;
	}
}

void write2GUST(cv::Mat state)
{
}


cv::Mat fetchLines()
{
	cv::Mat received;
	return received;
}

float fetchDepth()
{
	float depth_received;
	return depth_received;
}


int main(int argc, char *argv[])
{
#ifdef ZED
		//ocv gpu 
		int availableGPU = cv::gpu::getCudaEnabledDeviceCount();
		cout << availableGPU << " GPUs Found" << endl;
		for(size_t g = 0; g < availableGPU; g ++)
		{
			cv::gpu::DeviceInfo currentDevice = cv::gpu::DeviceInfo(g);
			cout << "Device id: " << g << " name: " << currentDevice.name() << " version: " << currentDevice.majorVersion() << " cores: " << currentDevice.multiProcessorCount() << " mem: " << currentDevice.totalMemory() << "compatible: " << currentDevice.isCompatible() << endl;
			cout << "Can map shared: " << cv::gpu::CudaMem::canMapHostMemory() << endl; 
		}
		cv::gpu::setDevice(0);
		//ALLOC_ZEROCOPY = 2
		Camera zed;
		//ZED initialize and Params
		InitParameters init_params;
		init_params.camera_resolution = RESOLUTION_VGA;
		init_params.camera_fps = 15;
		init_params.coordinate_units = UNIT_MILLIMETER;
		init_params.depth_minimum_distance = 300;
		init_params.depth_mode = DEPTH_MODE_PERFORMANCE;

		
		err = zed.open(init_params);
		cout <<"zed opened\n";
		if (err != SUCCESS) 
		{
		std::cout << errorCode2str(err) << std::endl;
		zed.close();
		return EXIT_FAILURE; // quit if an error occurred
		}


		zed.setDepthMaxRangeValue(10000);
		runtime_parameters.sensing_mode = SENSING_MODE_FILL;
		int numDepthPoints = 20;
		sl::Mat inFrame_zl, inFrame_zr, inFrame_zd, inFrame_zq, inFrame_zdm, inFrame_zcm,  inFrame_zc;
	
#endif
	
	//Line finding params
	float rhod = 1;
	float th = 180;
	float thresh = 80;
	
	int searching = 1;
	int cannyThresh1 = 10;
	int cannyThresh2 = 40;
	int minLineLengthP = 100;
	int maxLineGapP = 10;
	

	float desiredAngle = 90;
	float range = 30; //degrees
	
	float depth_value;
	int numTargets = 0;
	
	bool foundTarget = false;
	int maxLineSeperation = 30;
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
	float avgCycleTime = 0;
	float avgNotCycleTime = 0;
	int numCyclesFound = 0;
	int numCyclesNotFound = 0;
	

	//CV Objects

	cv::Mat inFrame_l, inFrame_r, inFrame_d, inFrame_c, inFrame, inFrameCopy,  outFrame, outFrameTemp, linesPMat, canny_d, canny_l, canny_c;
	vector<Vec2f> lines;
	vector<Vec4i> linesP;
	vector<Vec4i> validLinesP;
	Point estMidPoint;
// 	namedWindow("InputFrame", WINDOW_AUTOSIZE);
// 	namedWindow("Left", WINDOW_AUTOSIZE);
// 	namedWindow("Depth", WINDOW_AUTOSIZE);
// 	namedWindow("Confidence", WINDOW_AUTOSIZE);
//	setMouseCallback("InputFrame", mouse_callback);
//	namedWindow("OutputFrame", WINDOW_AUTOSIZE);


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
	cv::setIdentity(lineTrackingKF.processNoiseCov, cv::Scalar(1e-1));
	//lineTrackingKF.processNoiseCov = cv::Mat::zeros(stateDim, stateDim, typeKF);
	//lineTrackingKF.processNoiseCov.at<float>(0,0) = 1e-2;
	//lineTrackingKF.processNoiseCov.at<float>(1,1) = 1e-2;
	//lineTrackingKF.processNoiseCov.at<float>(2,2) = 1e-2;
	//lineTrackingKF.processNoiseCov.at<float>(3,3) = 1e-2;
	//lineTrackingKF.processNoiseCov.at<float>(4,4) = 1e-2;
	//lineTrackingKF.processNoiseCov.at<float>(5,5) = 1e-2;
	//lineTrackingKF.processNoiseCov.at<float>(6,6) = 1e-2;
	//lineTrackingKF.processNoiseCov.at<float>(7,7) = 1e-2;
	//lineTrackingKF.processNoiseCov.at<float>(8,8) = 1e-2;
	//lineTrackingKF.processNoiseCov.at<float>(9,9) = 1e-2;
	
	
	//Measurement Noise Covairance Martix
	//[ Wx 0  0  0  0  0  ]
	//[ 0  Wy 0  0  0  0  ]
	//[ 0  0  Wz 0  0  0  ]
	//[ 0  0  0  Wt 0  0  ]
	//[ 0  0  0  0  Ww 0  ]
	//[ 0  0  0  0  0  Wh ]
	lineTrackingKF.measurementNoiseCov = cv::Mat::zeros(measureDim, measureDim, typeKF);
	lineTrackingKF.measurementNoiseCov.at<float>(0,0) = 0.043;
	lineTrackingKF.measurementNoiseCov.at<float>(1,1) = 71;
	lineTrackingKF.measurementNoiseCov.at<float>(2,2) = 367;
	lineTrackingKF.measurementNoiseCov.at<float>(3,3) = 1e-1;
	lineTrackingKF.measurementNoiseCov.at<float>(4,4) = 1e-1;
	lineTrackingKF.measurementNoiseCov.at<float>(5,5) = 1909;
	
	int numSamples = 1000;
	std::vector<float> sampledTimeNF;
	sampledTimeNF.reserve(numSamples);
	std::vector<float> sampledTimeF;
	sampledTimeF.reserve(numSamples);
	
	int p = 0;
	while(p < numSamples)
	{	

		p++;

		cv::Mat inFrameTemp;
		double precTick = ticks;
		ticks = (double) cv::getTickCount();
		double dT = (ticks - precTick) / cv::getTickFrequency(); //sfleconds
		//Grabs and converts all the images from ZED
#ifdef ZED
		err = zed.grab(runtime_parameters);
		err = zed.retrieveImage(inFrame_zl, VIEW_LEFT);
		inFrame_l = cv::Mat(inFrame_zl.getHeight(), inFrame_zl.getWidth(), CV_8UC4, inFrame_zl.getPtr<sl::uchar1>(sl::MEM_CPU));
	
		err = zed.retrieveMeasure(inFrame_zdm);
	
	
// 		err = zed.retrieveMeasure(inFrame_zcm, MEASURE_CONFIDENCE);
		
// 		err = zed.retrieveImage(inFrame_zd, VIEW_DEPTH);
// 		inFrame_d = cv::Mat(inFrame_zd.getHeight(), inFrame_zd.getWidth(), CV_8UC4, inFrame_zd.getPtr<sl::uchar1>(sl::MEM_CPU));
		
// 		err = zed.retrieveImage(inFrame_zc , VIEW_CONFIDENCE);
// 		inFrame_c = cv::Mat(inFrame_zc.getHeight(), inFrame_zc.getWidth(), CV_8UC4, inFrame_zc.getPtr<sl::uchar1>(sl::MEM_CPU));
		inFrame = inFrame_l;
		
		inFrame_l.copyTo(inFrameCopy);
		
		


		if(inFrame.empty())
		{
			cout << "Blank frame \n";
			return -1;
		}
#endif
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

			cv::Mat estMidPointMat(2, 1, CV_32F);
			estMidPointMat.at<float>(0) = state.at<float>(0);
			estMidPointMat.at<float>(1) = state.at<float>(1);			
			estMidPointMat = rotate(estMidPointMat, state.at<float>(3)*CV_PI/180, false);
			estMidPoint.x = estMidPointMat.at<float>(0);
			estMidPoint.y = estMidPointMat.at<float>(1);
			//cout << "Est Mid Point " << estMidPoint << endl;
			RotatedRect sRect = RotatedRect(Point(estMidPointMat.at<float>(0), estMidPointMat.at<float>(1)), Size2f(state.at<float>(8), state.at<float>(9)), state.at<float>(3) + 90);
			//cout << "Estimated Angle: " << state.at<float>(3) << endl;
			

#ifdef ZED
			Point2f verticesSRect[4];
			sRect.points(verticesSRect);
			for (int i = 0; i < 4; i++)
				line(inFrameCopy, verticesSRect[i], verticesSRect[(i+1)%4], Scalar(255,255,255));
			//rectangle(inFrameCopy, Point(startXOfRect, startYOfRect), Point(startXOfRect + colsOfRect, startYOfRect + rowsOfRect), Scalar(255, 255, 255));
			
			putText(inFrameCopy, to_string(state.at<float>(2)), Point(300, 300), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255 , 255));
			circle(inFrameCopy, estMidPoint, 5, Scalar(255,255,255),3);
			//inFrame(cv::Rect(startXOfRect, startYOfRect, colsOfRect, rowsOfRect)).copyTo(inFrameTemp);
#endif
		}
		

#ifdef ZED
		linesPMat = findLines(inFrame,  cannyThresh1,  cannyThresh2,  rhod,  th,  minLineLengthP,  maxLineGapP);
			
#else
		linesPMat = fetchLines();
#endif



 		if (linesPMat.total() > 0)
 		{
			Vec4i linesP[20];
			foundTarget = false;
			OurLine ourLines[20];
			OurTarget ourTargets[400];
			numTargets = 0;
			lastMidPoint = Point(0,0);
			for( int i = 0; i < linesPMat.total(); i++ )
			{
				linesP[i] = linesPMat.at<Vec4i>(i);
			}
			int indexClosestToEstimate = -1;
			for(size_t i = 0; i < linesPMat.total(); i++ )
			{

				ourLines[i] = OurLine(linesP[i]);
				//line(inFrameCopy, Point(linesP[i][0], linesP[i][1]), Point( linesP[i][2], linesP[i][3]), Scalar(0,255,255), 3 , 8);
				//cout<< "line: " << linesP[i][0] << "," << linesP[i][1] << ";"<< linesP[i][2] << "," << linesP[i][3] << endl;; 
				//cout<<"left Pt1: " << Point(linesP[i][0], linesP[i][1]) << " left Pt2: "<< Point( linesP[i][2], linesP[i][3]) << endl;
				Vec4i tempLine = ourLines[i].getCvLine();
				float distanceClosestToEstimate = std::numeric_limits<int>::max();
				float distanceCurrentToEstimate = distanceClosestToEstimate;
				
				for (size_t n = 0; n < i; n ++)
				{
					size_t k = i*linesPMat.total() + n;
					ourTargets[k] = OurTarget(ourLines[i], ourLines[n], maxAngleSeperation, maxLineSeperation, desiredAngle, range);
					if (ourTargets[k].getIsValidPair())
					{
						distanceCurrentToEstimate = (state.at<float>(0) - ourTargets[k].getMidPoint().x)*(state.at<float>(0) - ourTargets[k].getMidPoint().x) + (state.at<float>(1) - ourTargets[k].getMidPoint().y)*(state.at<float>(1) - ourTargets[k].getMidPoint().y);
						if( distanceCurrentToEstimate < distanceClosestToEstimate)
						{
							distanceClosestToEstimate = distanceCurrentToEstimate;
							indexClosestToEstimate = k;
						}
					}
				}
			}
			if (indexClosestToEstimate > -1)
			{
				foundTarget = true;
				Point midPoint = ourTargets[indexClosestToEstimate].getMidPoint();
				//cout << "Mid Point " << midPoint << endl;
				Point rotatedMidPoint = ourTargets[indexClosestToEstimate].getRotatedMidPoint();

				lastMidPoint = midPoint;
				cv::Mat adjustedLines = ourTargets[indexClosestToEstimate].getAdjustedLines();
				//inFrame_zdm.getValue(midPoint.x, midPoint.y, & depth_value);
				RotatedRect rRect = ourTargets[indexClosestToEstimate].getRectTarget();
				
#ifdef ZED
				Point2f vertices[4];
				rRect.points(vertices);
				for (int i = 0; i < 4; i++)
					line(inFrameCopy, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
				
				depth_value = findDepth(ourTargets[indexClosestToEstimate], inFrame_zdm, numDepthPoints)[1];
#else
				depth_value = fetchDepth();
#endif
				
				
				
				
				framesSinceSeen = 0;
//					sampledTimeF[p] = ourTargets[k].getHeight();

				measurement.at<float>(0) = rotatedMidPoint.x;
				measurement.at<float>(1) = rotatedMidPoint.y;
				measurement.at<float>(2) = depth_value;
				measurement.at<float>(3) = ourTargets[indexClosestToEstimate].getOrientation();
				measurement.at<float>(4) = ourTargets[indexClosestToEstimate].getWidth();
				measurement.at<float>(5) = ourTargets[indexClosestToEstimate].getHeight();
#ifdef ZED
				putText(inFrameCopy, to_string(measurement.at<float>(2)), Point(300, 250), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
				circle(inFrameCopy, midPoint, 5, Scalar(0,255,0),3);
#endif
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


				}
				else
				{
					lineTrackingKF.correct(measurement);

				}
				isTracked = true;
			}

		}
		else
		{
			framesSinceSeen ++;
			if(framesSinceSeen >= 100)
				isTracked = false;
		}
		



		

// 	imshow("InputFrameCopy", inFrameCopy);
// 	imshow("Depth", inFrame_d);
#ifdef ZED
	imshow("Out", inFrameCopy);
#else
	write2GUST(state);
#endif
// 	imshow("Confidence", inFrame_c);


	//imshow("OutputFrame", outFrame);

	if (waitKey(5) >= 0)
		break;
	}
	
	float sumMeanTimeF = 0;
	float sumMeanSquareTimeF = 0;
	float meanTimeF = 0;
	float meanSquareTimeF = 0;
	float varianceTimeF = 0;
	
	for(size_t v = 0; v < sampledTimeF.size(); v ++)
	{
		sumMeanTimeF += sampledTimeF[v];
		sumMeanSquareTimeF += sampledTimeF[v]*sampledTimeF[v];	
	}
	meanTimeF = sumMeanTimeF/sampledTimeF.size();
	meanSquareTimeF = sumMeanSquareTimeF/sampledTimeF.size();
	varianceTimeF = meanSquareTimeF - meanTimeF*meanTimeF;
	
	cout << "MeanF: " << meanTimeF << endl;
	cout << "VarianceF: " << varianceTimeF << endl;
	cout << "Of : " << sampledTimeF.size() << endl;
	
	float sumMeanTimeNF = 0;
	float sumMeanSquareTimeNF = 0;
	float meanTimeNF = 0;
	float meanSquareTimeNF = 0;
	float varianceTimeNF = 0;
	
	for(size_t v = 0; v < sampledTimeNF.size(); v ++)
	{
		sumMeanTimeNF += sampledTimeNF[v];
		sumMeanSquareTimeNF += sampledTimeNF[v]*sampledTimeNF[v];	
	}
	meanTimeNF = sumMeanTimeNF/sampledTimeNF.size();
	meanSquareTimeNF = sumMeanSquareTimeNF/sampledTimeNF.size();
	varianceTimeNF = meanSquareTimeNF - meanTimeF*meanTimeNF;
	
	cout << "MeanNF: " << meanTimeNF << endl;
	cout << "VarianceNF: " << varianceTimeNF << endl;
	cout << "Of : " << sampledTimeNF.size() << endl;
	
}
