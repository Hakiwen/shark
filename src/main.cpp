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



#ifdef GPU
#include <opencv2/gpu/gpu.hpp>
#endif


using namespace std;
using namespace cv;





#ifdef ZED
using namespace sl;
ERROR_CODE err;
RuntimeParameters runtime_parameters;
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

cv::Mat convert2DisplayDepthMat(cv::Mat * dep_mat)
{
	//Scales mat so that it can be displayed as a grayscale
	cv::Mat depth_display_mat;
	dep_mat->copyTo(depth_display_mat);
	double min, max;
	cv::minMaxIdx(depth_display_mat, &min, &max);
	convertScaleAbs(depth_display_mat, depth_display_mat, 255 / max);
	return depth_display_mat;
}

cv::Mat findLines(cv::Mat input_frame, int cannyThresh1, int cannyThresh2, float rhod, float th, int minLineLengthP, int maxLineGapP)
{

	cv::Mat lines_found_mat;
#ifdef GPU
	cv::gpu::GpuMat g_outFrameTemp, g_lines_found;
	cv::gpu::HoughLinesBuf hlBuf;
	cv::cvtColor(input_frame, input_frame, CV_RGB2GRAY);

	cv::gpu::CudaMem zero_copy_input_frame(input_frame, cv::gpu::CudaMem::ALLOC_ZEROCOPY);
	cv::gpu::GpuMat g_input_frame = zero_copy_input_frame.createGpuMatHeader();
	
	cv::gpu::Canny(g_input_frame, g_outFrameTemp, cannyThresh1, cannyThresh2, 3);

	cv::gpu::HoughLinesP( g_outFrameTemp, g_lines_found, hlBuf, rhod, CV_PI/th, minLineLengthP, maxLineGapP, 20);
	g_lines_found.download(lines_found_mat);
#else
	cv::Mat outFrameTemp;
	cv::cvtColor(input_frame,input_frame, CV_RGB2GRAY);
	cv::Canny(input_frame, outFrameTemp, cannyThresh1, cannyThresh2, 3);
	cv::HoughLinesP(outFrameTemp, lines_found_mat, rhod, CV_PI/th, 60 /*thresh*/, minLineLengthP, maxLineGapP);
	if (lines_found_mat.total() > 21)
	{ 
		lines_found_mat = lines_found_mat.colRange(0, 20);
	}
#endif
	return lines_found_mat;
		
}

cv::Vec2f findDepth(OurTarget inputTarget, cv::Mat *inputDepth, int numDepthPoints)
{
	Point targetMidPoint = inputTarget.getMidPoint();
	cout << targetMidPoint << endl;
	Vec2f determinedDepthStates; 
// 	cout << inputDepth->at<int>(targetMidPoint) << endl;
	determinedDepthStates[0] = inputDepth->at<float>(targetMidPoint);
// 	determinedDepthStates[0] = 1;
	cout << determinedDepthStates[0] << endl;
	return determinedDepthStates;
	//TODO Some flan stuff

}


#if defined ZED
void fetchImages(cv::Mat *img_mat, cv::Mat *dep_mat,  Camera * zed)
{
	sl::Mat input_frame_zl, input_frame_zr, input_frame_zdm, input_frame_zd;
	cv::Mat ocv_dep_mat, img_dep, img_mat_r;
	err = zed->grab(runtime_parameters);
	err = zed->retrieveImage(input_frame_zl, VIEW_LEFT);
	//err = zed->retrieveImage(input_frame_zr, VIEW_RIGHT);
	//err = zed->retrieveImage(input_frame_zd, VIEW_DEPTH);
	//img_dep = cv::Mat(input_frame_zd.getHeight(), input_frame_zd.getWidth(), CV_8UC4, input_frame_zd.getPtr<sl::uchar1>(sl::MEM_CPU));
	//imshow("zed depth map", img_dep);
	*img_mat = cv::Mat(input_frame_zl.getHeight(), input_frame_zl.getWidth(), CV_8UC4, input_frame_zl.getPtr<sl::uchar1>(sl::MEM_CPU));
	//img_mat_r = cv::Mat(input_frame_zr.getHeight(), input_frame_zr.getWidth(), CV_8UC4, input_frame_zr.getPtr<sl::uchar1>(sl::MEM_CPU));
	err = zed->retrieveMeasure(input_frame_zdm);
	*dep_mat = cv::Mat(input_frame_zdm.getHeight(), input_frame_zdm.getWidth(), CV_8UC4, input_frame_zdm.getPtr<sl::uchar1>(sl::MEM_CPU));
	//generateDisparityMap(img_mat,& img_mat_r, &ocv_dep_mat);
	
}
#endif



void predictState(cv::Mat * state, KalmanFilter * line_tracking_kalman_filter, cv::Mat * input_frame_copy,  double dT, bool isTracked)
{
	
	
	line_tracking_kalman_filter->transitionMatrix.at<float>(0,4) = dT;
	line_tracking_kalman_filter->transitionMatrix.at<float>(1,5) = dT;
	line_tracking_kalman_filter->transitionMatrix.at<float>(2,6) = dT;
	line_tracking_kalman_filter->transitionMatrix.at<float>(3,7) = dT;
	//cout<< "dT:" << endl << dT << endl;
	
	if(newCoordsPIR)
	{
		//[x,y,z,th,vx,vy,vz,w,h]
		state->at<float>(0) = ptPIR.x;
		state->at<float>(1) = ptPIR.y;
		state->at<float>(2) = 1000;
		state->at<float>(3) = 0;
		state->at<float>(4) = 0;
		state->at<float>(5) = 0;
		state->at<float>(6) = 0;
		state->at<float>(7) = 0;
		state->at<float>(8) = 20;
		state->at<float>(9) = 100;
		newCoordsPIR = false;
	}
	else if(isTracked)
	{
		* state = line_tracking_kalman_filter->predict();
		//cout << "State post:" << endl << state << endl;
		
		cv::Mat estimated_midpoint_mat(2, 1, CV_32F);
		Point estimated_midpoint;
		estimated_midpoint_mat.at<float>(0) = state->at<float>(0);
		estimated_midpoint_mat.at<float>(1) = state->at<float>(1);			
		//estimated_midpoint_mat = rotate(estimated_midpoint_mat, state->at<float>(3)*CV_PI/180, false);
		estimated_midpoint.x = estimated_midpoint_mat.at<float>(0);
		estimated_midpoint.y = estimated_midpoint_mat.at<float>(1);
		
		RotatedRect sRect = RotatedRect(Point(estimated_midpoint_mat.at<float>(0), estimated_midpoint_mat.at<float>(1)), Size2f(state->at<float>(8), state->at<float>(9)), state->at<float>(3) + 90);
		//cout << "Estimated Angle: " << state.at<float>(3) << endl;
		

	//#ifdef ZED
		Point2f state_rectangle_vertices[4];
		sRect.points(state_rectangle_vertices);
		for (int i = 0; i < 4; i++)
			line(* input_frame_copy, state_rectangle_vertices[i], state_rectangle_vertices[(i+1)%4], Scalar(255,255,255));
		
		//putText(input_frame_copy, to_string(state.at<float>(2)), Point(300, 300), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255 , 255));
		circle(* input_frame_copy, estimated_midpoint, 5, Scalar(255,255,255),3);
	//#endif
	}
// 	cout << "Est Mid Point " << Point(state->at<float>(0), state->at<float>(1)) << endl;
}
	

void lineTracker(cv::Mat lines_found_mat, KalmanFilter * line_tracking_kalman_filter, bool * isTracked, cv::Mat * input_frame_copy, cv::Mat * input_frame_d, cv::Mat * state, cv::Mat * measurement, int * framesSinceSeen, float maximum_angle_difference, float maximum_line_seperation, float desired_angle , float range, int numDepthPoints, double dT)
{
	if (lines_found_mat.total() > 1)
 		{
			//cout << lines_found_mat.total() << endl;
			Vec4i lines_found[20];
			bool found_target = false;
			OurLine our_lines[20];
			OurTarget our_targets[1000];
			int number_valid_targets = 0;
			Point last_mid_point = Point(0,0);
			for( int i = 0; i < lines_found_mat.total(); i++ )
			{
				lines_found[i] = lines_found_mat.at<Vec4i>(i);
			}
			
			
			float next_target_range; 
// 			cout << "isTracked" << *isTracked << endl;
			if(*isTracked)
			{
				//float expected_x_movement = dT*state->at<float>(4);
				//float expected_y_movement = dT*state->at<float>(5);
				float expected_x_movement = line_tracking_kalman_filter->errorCovPre.at<float>(0,0);
				float expected_y_movement = line_tracking_kalman_filter->errorCovPre.at<float>(1,1);
// 				cout << "Posteriori Error x,y: " << expected_x_movement <<","<< expected_y_movement << endl;
				next_target_range = (expected_x_movement + expected_y_movement);
				circle(* input_frame_copy, Point(state->at<float>(0), state->at<float>(1)), next_target_range, Scalar(255,0,0), 1);
			}
			else
			{
				next_target_range = input_frame_copy->total();
			}
			
			int closest_target_to_estimate_index = -1;
			float closest_target_to_estimate_distance = std::numeric_limits<int>::max();
			float current_target_to_estimate_distance;
			for(size_t i = 0; i < lines_found_mat.total(); i++ )
			{

				our_lines[i] = OurLine(lines_found[i]);
// 				line(*input_frame_copy, Point(lines_found[i][0], lines_found[i][1]), Point( lines_found[i][2], lines_found[i][3]), Scalar(0,255,255), 3 , 8);
// 				cout<< "line: " << lines_found[i][0] << "," << lines_found[i][1] << ";"<< lines_found[i][2] << "," << lines_found[i][3] << endl;; 
// 				cout<<"left Pt1: " << Point(lines_found[i][0], lines_found[i][1]) << " left Pt2: "<< Point( lines_found[i][2], lines_found[i][3]) << endl;
// 				Vec4i tempLine = our_lines[i].getCvLine();
				
				
				
				for (size_t n = 0; n < i; n ++)
				{
					size_t k = i*lines_found_mat.total() + n;
// 					cout << "n: " << n << " i: " << i << endl;
					our_targets[k] = OurTarget(our_lines[i], our_lines[n], maximum_angle_difference, maximum_line_seperation, desired_angle, range);
					if (our_targets[k].getIsValidPair())
					{
						current_target_to_estimate_distance = sqrt((state->at<float>(0) - our_targets[k].getMidPoint().x)*(state->at<float>(0) - our_targets[k].getMidPoint().x) + (state->at<float>(1) - our_targets[k].getMidPoint().y)*(state->at<float>(1) - our_targets[k].getMidPoint().y));
						//cout << "current_target_to_estimate_distance" << current_target_to_estimate_distance << endl;
						//cout << "next_target_range" << next_target_range << endl;
// 						cout << our_targets[k].getMidPoint() << endl;
						if( (current_target_to_estimate_distance < closest_target_to_estimate_distance) && (current_target_to_estimate_distance < next_target_range))
						{
							closest_target_to_estimate_distance = current_target_to_estimate_distance;
							closest_target_to_estimate_index = k;
						}
					}
				}
			}

			if (closest_target_to_estimate_index > -1)
			{
				found_target = true;
				Point midpoint = our_targets[closest_target_to_estimate_index].getMidPoint();
				//cout << "Mid Point " << midPoint << endl;
				Point rotatedMidPoint = our_targets[closest_target_to_estimate_index].getRotatedMidPoint();

				last_mid_point = midpoint;
				cv::Mat adjusted_lines = our_targets[closest_target_to_estimate_index].getAdjustedLines();
				//input_frame_zdm.getValue(midPoint.x, midPoint.y, & depth_value);
				RotatedRect rotated_rect_target = our_targets[closest_target_to_estimate_index].getRectTarget();
				
//#ifdef ZED
				Point2f rotated_rect_target_vertices[4];
				rotated_rect_target.points(rotated_rect_target_vertices);
				for (int i = 0; i < 4; i++)
					line(* input_frame_copy, rotated_rect_target_vertices[i], rotated_rect_target_vertices[(i+1)%4], Scalar(0,255,0));
//#endif			
				float depth_value = findDepth(our_targets[closest_target_to_estimate_index], input_frame_d, numDepthPoints)[0];
				
				* framesSinceSeen = 0;
//					sampledTimeF[p] = our_targets[k].getHeight();

				measurement->at<float>(0) = midpoint.x;
				measurement->at<float>(1) = midpoint.y;
				if(depth_value != -1)
				measurement->at<float>(2) = depth_value;
				measurement->at<float>(3) = our_targets[closest_target_to_estimate_index].getOrientation();
				measurement->at<float>(4) = our_targets[closest_target_to_estimate_index].getWidth();
				measurement->at<float>(5) = our_targets[closest_target_to_estimate_index].getHeight();
//#ifdef ZED
				//putText(input_frame_copy, to_string(measurement.at<float>(2)), Point(300, 250), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
				circle(* input_frame_copy, midpoint, 5, Scalar(0,255,0),3);
//#endif
				//cout << "Angle:" << our_targets[k].getOrientation() << endl;
				//cout << "Measurement:" << measurement << endl;
				if (!*isTracked)
				{	
					line_tracking_kalman_filter->errorCovPre.at<float>(0,0) = 1;
					line_tracking_kalman_filter->errorCovPre.at<float>(1,1) = 1;
					line_tracking_kalman_filter->errorCovPre.at<float>(2,2) = 1;
					line_tracking_kalman_filter->errorCovPre.at<float>(3,3) = 1;
					line_tracking_kalman_filter->errorCovPre.at<float>(4,4) = 1;
					line_tracking_kalman_filter->errorCovPre.at<float>(5,5) = 1;
					line_tracking_kalman_filter->errorCovPre.at<float>(6,6) = 1;
					line_tracking_kalman_filter->errorCovPre.at<float>(7,7) = 1;
					line_tracking_kalman_filter->errorCovPre.at<float>(8,8) = 1;
					line_tracking_kalman_filter->errorCovPre.at<float>(9,9) = 1;
					
					state->at<float>(0) = measurement->at<float>(0);
					state->at<float>(1) = measurement->at<float>(1);
					state->at<float>(2) = measurement->at<float>(2);
					state->at<float>(3) = measurement->at<float>(3);
					state->at<float>(4) = 0;
					state->at<float>(5) = 0;
					state->at<float>(6) = 0;
					state->at<float>(7) = 0;
					state->at<float>(8) = measurement->at<float>(4);
					state->at<float>(9) = measurement->at<float>(5);
					
					line_tracking_kalman_filter->statePost = * state;


				}
				else
				{
					line_tracking_kalman_filter->correct(* measurement);

				}
				*isTracked = true;
				cout << *state << endl;
			}

		}
		else
		{
			* framesSinceSeen ++;
			if(* framesSinceSeen >= 100)
				*isTracked = false;
		}
}
	
int main(int argc, char *argv[])
{
	namedWindow("Out");
	setMouseCallback("Out", mouse_callback);
#ifdef GPU
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
#endif

#ifdef ZED
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

	sl::Mat input_frame_zl, input_frame_zr, input_frame_zd, input_frame_zq, input_frame_zdm, input_frame_zcm,  input_frame_zc;
	
#endif
	int numDepthPoints = 20;
	//Line finding params
	float rhod = 1;
	float th = 1800;
	float thresh = 60;
	
	int searching = 1;
	int cannyThresh1 = 10;
	int cannyThresh2 = 40;
	int minLineLengthP = 150;
	int maxLineGapP = 20;

	float desired_angle = 90;

	float range = 30; //degrees

	int maximum_line_seperation = 10;
	float maximum_angle_difference = 10;

	int widthOfRect;
	int heightOfRect;
	bool isTracked = false;
	int framesSinceSeen = 0;
	double ticks = 0;
	
	//Eval Vars
	float avgCycleTime = 0;
	float avgNotCycleTime = 0;
	int numCyclesFound = 0;
	int numCyclesNotFound = 0;
	

	//CV Objects

	cv::Mat input_frame_l, input_frame_r, input_frame_d, inFtrame_c, input_frame, input_frame_copy,  outFrame, outFrameTemp, lines_found_mat;
	cv::vector<Vec2f> lines;
	cv::vector<Vec4i> lines_found;
	cv::vector<Vec4i> validLinesP;

	//Kalman Filter Nonsense
	int state_vector_size = 10;
	int measurement_vector_size = 6;
	int control_vector_size = 0;
	unsigned int kalman_filter_data_type = CV_32F;
	KalmanFilter line_tracking_kalman_filter(state_vector_size, measurement_vector_size, control_vector_size);

	//initialize x vector
	cv::Mat state(state_vector_size, 1, kalman_filter_data_type); //[x,y,z,th,vx,vy,vz,w,h]

	//initialize y vector
	cv::Mat measurement(measurement_vector_size, 1, kalman_filter_data_type);//[mx,my,mz,mw,mh]
	
	
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
	cv::setIdentity(line_tracking_kalman_filter.transitionMatrix);
	
	// Measure Matrix H - This is what you have access to
	// [ 1 0 0 0 0 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 0 0 0 0 ]
	// [ 0 0 1 0 0 0 0 0 0 0 ]
	// [ 0 0 0 1 0 0 0 0 0 0 ]
	// [ 0 0 0 0 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 0 0 0 0 1 ]
	line_tracking_kalman_filter.measurementMatrix = cv::Mat::zeros(measurement_vector_size, state_vector_size, kalman_filter_data_type);
	line_tracking_kalman_filter.measurementMatrix.at<float>(0,0) = 1.0f;
	line_tracking_kalman_filter.measurementMatrix.at<float>(1,1) = 1.0f;
	line_tracking_kalman_filter.measurementMatrix.at<float>(2,2) = 1.0f;
	line_tracking_kalman_filter.measurementMatrix.at<float>(3,3) = 1.0f;
	line_tracking_kalman_filter.measurementMatrix.at<float>(4,8) = 1.0f;
	line_tracking_kalman_filter.measurementMatrix.at<float>(5,9) = 1.0f;
	
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
	cv::setIdentity(line_tracking_kalman_filter.processNoiseCov, cv::Scalar(1e-1));
	//line_tracking_kalman_filter.processNoiseCov = cv::Mat::zeros(state_vector_size, state_vector_size, kalman_filter_data_type);
	//line_tracking_kalman_filter.processNoiseCov.at<float>(0,0) = 1e-2;
	//line_tracking_kalman_filter.processNoiseCov.at<float>(1,1) = 1e-2;
	//line_tracking_kalman_filter.processNoiseCov.at<float>(2,2) = 1e-2;
	//line_tracking_kalman_filter.processNoiseCov.at<float>(3,3) = 1e-2;
	//line_tracking_kalman_filter.processNoiseCov.at<float>(4,4) = 1e-2;
	//line_tracking_kalman_filter.processNoiseCov.at<float>(5,5) = 1e-2;
	//line_tracking_kalman_filter.processNoiseCov.at<float>(6,6) = 1e-2;
	//line_tracking_kalman_filter.processNoiseCov.at<float>(7,7) = 1e-2;
	//line_tracking_kalman_filter.processNoiseCov.at<float>(8,8) = 1e-2;
	//line_tracking_kalman_filter.processNoiseCov.at<float>(9,9) = 1e-2;
	
	
	//Measurement Noise Covairance Martix
	//[ Wx 0  0  0  0  0  ]
	//[ 0  Wy 0  0  0  0  ]
	//[ 0  0  Wz 0  0  0  ]
	//[ 0  0  0  Wt 0  0  ]
	//[ 0  0  0  0  Ww 0  ]
	//[ 0  0  0  0  0  Wh ]
	line_tracking_kalman_filter.measurementNoiseCov = cv::Mat::zeros(measurement_vector_size, measurement_vector_size, kalman_filter_data_type);
	line_tracking_kalman_filter.measurementNoiseCov.at<float>(0,0) = 0.043;
	line_tracking_kalman_filter.measurementNoiseCov.at<float>(1,1) = 71;
	line_tracking_kalman_filter.measurementNoiseCov.at<float>(2,2) = 367;
	line_tracking_kalman_filter.measurementNoiseCov.at<float>(3,3) = 1e-1;
	line_tracking_kalman_filter.measurementNoiseCov.at<float>(4,4) = 1e-1;
	line_tracking_kalman_filter.measurementNoiseCov.at<float>(5,5) = 1909;
	
	bool framesGotten = false;
	while(true)
	{	
// 		cout<< "Beginning of Loop" << endl;
		cv::Mat input_frameTemp;
		double preceding_tick = ticks;
		ticks = (double) cv::getTickCount();
		double dT = (ticks - preceding_tick) / cv::getTickFrequency(); //sfleconds
		//Grabs and converts all the images from ZED
		framesGotten = false;

#ifdef ZED
		fetchImages(&input_frame_l, &input_frame_d, &zed);
#endif			
		input_frame = input_frame_l;
		
		input_frame_l.copyTo(input_frame_copy);
		
		if(input_frame.empty())
		{
			cout << "Blank frame \n";
			return -1;
		}
		
// 		cout << ptPIR << newCoordsPIR << endl;

		predictState(& state,& line_tracking_kalman_filter, & input_frame_copy, dT, isTracked);
		
		lines_found_mat = findLines(input_frame,  cannyThresh1,  cannyThresh2,  rhod,  th,  minLineLengthP,  maxLineGapP);
		
		lineTracker( lines_found_mat, 
				& line_tracking_kalman_filter, 
				& isTracked, 
				& input_frame_copy, 
				& input_frame_d, 
				& state, 
				& measurement,
				& framesSinceSeen, 
				maximum_angle_difference,
				maximum_line_seperation, 
				desired_angle, 
				range, 
				numDepthPoints,
				dT
   			);
		
		imshow("Out", input_frame_copy);
	
#ifdef ZED
		imshow("Out", input_frame_copy);
#endif


		if (waitKey(5) >= 0)
			break;
	}
	

	
}
