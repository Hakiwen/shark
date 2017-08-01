#include <iostream>
#include <sl/Camera.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;
using namespace sl;
//Depth variance may depend upon confidence and pixel location
int main(int argc, char **argv) {

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
	
	sl::Mat inFrame_zl, inFrame_zr, inFrame_zd, inFrame_zq, inFrame_zc;
	cv::Mat inFrame_l, inFrame_r, inFrame_d, inFrame, inFrameCopy,  outFrame, outFrameTemp;
	
	namedWindow("InputFrame", WINDOW_AUTOSIZE);
	
	int numSamples = 10000;
	float sampledDepth[numSamples];
	float sampledConfidence[numSamples];
	cout << "Number of Samples: " << numSamples << endl;
	for(size_t sampled = 0; sampled < numSamples; sampled ++ )
	{
		err == zed.grab(runtime_parameters);

		err = zed.retrieveImage(inFrame_zl, VIEW_LEFT);
		inFrame_l = cv::Mat(inFrame_zl.getHeight(), inFrame_zl.getWidth(), CV_8UC4, inFrame_zl.getPtr<sl::uchar1>(sl::MEM_CPU));
		
		err = zed.retrieveMeasure(inFrame_zd);
		inFrame = inFrame_l;
		
		err = zed.retrieveMeasure(inFrame_zc, MEASURE_CONFIDENCE);
		
		err = zed.retrieveImage(inFrame_zq, VIEW_DEPTH);
		inFrame_d = cv::Mat(inFrame_zq.getHeight(), inFrame_zq.getWidth(), CV_8UC4, inFrame_zq.getPtr<sl::uchar1>(sl::MEM_CPU));
		inFrame.copyTo(inFrameCopy);
		
		
		inFrame_zd.getValue(300, 150, & sampledDepth[sampled]);
		inFrame_zc.getValue(300, 150, & sampledConfidence[sampled]);
		circle(inFrameCopy, Point(300, 150), 5, Scalar(0,255,0),3);
		
		
		if(sampled % 100 == 0)
			cout << "Percent Done: " << 100/(numSamples/sampled) << endl;
		if(inFrame.empty())
		{
			cout << "Blank frame \n";
			return -1;
		}
		
		imshow("InputFrameCopy", inFrameCopy);
		
		if (waitKey(5) >= 0)
		break;
	}
	
	float sumMeanDepth = 0;
	float sumMeanSquareDepth = 0;
	float meanDepth = 0;
	float meanSquareDepth = 0;
	float varianceDepth = 0;
	
	float sumMeanConfidence = 0;
	float sumMeanSquareConfidence = 0;
	float meanConfidence = 0;
	float meanSquareConfidence = 0;
	float varianceConfidence = 0;
	
	for(size_t v = 0; v < numSamples; v ++)
	{
		sumMeanDepth += sampledDepth[v];
		sumMeanSquareDepth += sampledDepth[v]*sampledDepth[v];
		
		sumMeanConfidence += sampledConfidence[v];
		sumMeanSquareConfidence += sampledConfidence[v]*sampledConfidence[v];
	}
	meanDepth = sumMeanDepth/numSamples;
	meanSquareDepth = sumMeanSquareDepth/numSamples;
	varianceDepth = meanSquareDepth - meanDepth*meanDepth;
	
	cout << "Depth Mean: " << meanDepth << endl;
	cout << "Depth Variance: " << varianceDepth << endl;
	
	
	meanConfidence = sumMeanConfidence/numSamples;
	meanSquareConfidence = sumMeanSquareConfidence/numSamples;
	varianceConfidence = meanSquareConfidence - meanConfidence*meanConfidence;
	
	cout << "Confidence Mean: " << meanConfidence << endl;
	cout << "Confidence Variance: " << varianceConfidence << endl;
	
}
