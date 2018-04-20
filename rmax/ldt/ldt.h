#ifndef GCV_LDT_H
#define GCV_LDT_H

/* Line Detector and Tracker (LDT) */

/// C function prototypes
#if defined(__cplusplus)
extern "C"
{
#endif

#if defined(__cplusplus)
}
#endif


#if defined(__cplusplus)
#include "gcv/gcv.h"
#include "sul/component.h"
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/offset_ptr.hpp>
#include <boost/version.hpp>
/// GcvSample
class gcvLDT : public sulComponent {
public:
	/// constructor takes pointer to vision pipeline and structure for window tracking
    gcvLDT(sulObject *parent = 0, const char *name = 0, struct Ldt_ref *mypipe = 0);

	/// frees memory and pipeline
	virtual ~gcvLDT(); // should not have to have a destructor

	/// override userInitialize function
	virtual int onInit();

	/// override userShutdown function
	virtual int onShutdown();

	/// override the process frame funciton
	virtual int onUpdate(double time);

	struct Ldt_ref *emypipe;
private:
	class gcvImage *mline;
	class gcvImage *medge;
	class gcvImage *mgray;
	class gcvImage *mimg;
	class gcvImage *mimg2;
	class gcvImage *mjpeg;
	class gcvGrabberComponent *mfg;
	class gcvGrabberComponent *mfg2;
	class gcvGrabber_ref* gfg;
	class gcvGrabber_ref* gfg2;
	gcvUchar *mtximage;
    
	struct shared_gen_mat
		{
			boost::interprocess::interprocess_mutex mutex;
			boost::interprocess::offset_ptr<uchar> dat_off;
			uchar dat_arr;
			int height, width;
		};

	struct shared_state
		{
			boost::interprocess::interprocess_mutex mutex;
			float x, y, z, psi;
		};
		
	boost::interprocess::shared_memory_object sharedMemImgL;
	boost::interprocess::shared_memory_object sharedMemImgR;
	boost::interprocess::shared_memory_object sharedMemDep;
	boost::interprocess::shared_memory_object sharedMemState;
	
	boost::interprocess::mapped_region img_region_l;
	boost::interprocess::mapped_region img_region_r;
	boost::interprocess::mapped_region dep_region;
	boost::interprocess::mapped_region state_region;
	
	shared_gen_mat *inFrame_l;
	shared_gen_mat *inFrame_r;
	shared_gen_mat *inFrame_d;
	shared_state *state;
	

};


#endif // _cplusplus

#endif // GCV_LDT_H


