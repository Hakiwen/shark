#include <string>
#include <iomanip>
#include <sstream>

#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <sl/Camera.hpp>
#include <opencv2/gpu/gpu.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>


using namespace std;
using namespace cv;
using namespace sl;
using namespace boost::interprocess;



struct shared_gen_mat
{
	interprocess_mutex mutex;
	offset_ptr<uchar> dat_off;
	uchar dat_arr;
	int height, width;
};

struct shared_state
{
	boost::interprocess::interprocess_mutex mutex;
	float x, y, z, psi;
};

ERROR_CODE err;
RuntimeParameters runtime_parameters;
int main(int argc, char *argv[])
{
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
	sl::Mat inFrame_zl, inFrame_zr, inFrame_zd, inFrame_zq, inFrame_zdm, inFrame_zcm,  inFrame_zc;
// 	cv::Mat inFrame_l, inFrame_d;

	// Boost interprocess config
	// could do segment*(open_or_create) to connect if not created
	
	struct shm_remove
	{
		shm_remove() { shared_memory_object::remove("SharedMemoryImG"); shared_memory_object::remove("SharedMemoryDeP"); shared_memory_object::remove("SharedMemoryState");}
		~shm_remove(){ shared_memory_object::remove("SharedMemoryImG"); shared_memory_object::remove("SharedMemoryDeP"); shared_memory_object::remove("SharedMemoryState");}
	} remover;
   

	shared_memory_object sharedMemImg(open_or_create, "SharedMemoryImG", read_write);
	// might need to check if already truncated in shark
	sharedMemImg.truncate(100000000);

	mapped_region img_region(sharedMemImg, read_write);
	shared_gen_mat *inFrame_l = new (img_region.get_address()) shared_gen_mat;

	shared_memory_object sharedMemDep(open_or_create, "SharedMemoryDeP", read_write);
	sharedMemDep.truncate(100000000);
	

	mapped_region dep_region(sharedMemDep,read_write);
	shared_gen_mat *inFrame_d = new (dep_region.get_address()) shared_gen_mat;
	
	
	shared_memory_object sharedMemState(open_or_create, "SharedMemoryState", read_write);
	sharedMemState.truncate(100000);
	
	mapped_region state_region(sharedMemState, read_write);
	shared_state *state = new (state_region.get_address()) shared_state;
	


	//////////////////////////////
	
	while(true)
	{


		cout<< "-------------------------------------------------------------------------------------------------------------------------------------------------------" << endl;
		err = zed.grab(runtime_parameters);
		scoped_lock<interprocess_mutex> l_lock(inFrame_l->mutex);
		
		//Writes Image Matrix
		err = zed.retrieveImage(inFrame_zl, VIEW_LEFT);
		inFrame_l->dat_off = & inFrame_l->dat_arr;
		memcpy(inFrame_l->dat_off.get(), inFrame_zl.getPtr<sl::uchar1>(sl::MEM_CPU), inFrame_zl.getWidth()*inFrame_zl.getHeight()*inFrame_zl.getPixelBytes());
		inFrame_l->height = inFrame_zl.getHeight();
		inFrame_l->width = inFrame_zl.getWidth();
		//Displays result
		imshow("In", cv::Mat(inFrame_l->height, inFrame_l->width, CV_8UC4, inFrame_l->dat_off.get()));
		waitKey(5);

		//Writes depth Matrix
		scoped_lock<interprocess_mutex> d_lock(inFrame_d->mutex);
		err = zed.retrieveMeasure(inFrame_zd);
		inFrame_d->dat_off = &inFrame_d->dat_arr;
		memcpy(inFrame_d->dat_off.get(), inFrame_zd.getPtr<sl::uchar1>(sl::MEM_CPU), inFrame_zd.getWidth()*inFrame_zd.getHeight()*inFrame_zd.getPixelBytes());
		inFrame_d->height = inFrame_zd.getHeight();
		inFrame_d->width = inFrame_zd.getWidth();
		//Displays Result
		imshow("InDep", cv::Mat(inFrame_d->height, inFrame_d->width, CV_8UC4, inFrame_d->dat_off.get()));
		
		//State In
		scoped_lock<interprocess_mutex> s_lock(state->mutex);
		cout << "[ " << state->x << ", " << state->y << ", " << state->z << ", " << state->psi << "]" << endl;

	}

    // close sharedMemoryStuff 
    // TODO ^^^ unless it is automatically done
}