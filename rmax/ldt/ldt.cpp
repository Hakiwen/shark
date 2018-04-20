#include <string.h>
#include <math.h>
#include <stdio.h>
#include "sul/sul.h"
#include "gcv/gcv.h"
#include "gcv/image.h"
#include "rmax/ldt.h"
#include "rmax/ldt_ref.h"
#include "rmax/onboard2_ref.h"
#include "rmax/wdb.h"
#include "rmax/wdb_ref.h"
#include "rmax/si_ref.h"
#include "rmax/motion_ref.h"
#include "esim/cnsl.h"
#include "esim/command.h"
#include "esim/util.h"
#include "esim/rand.h"
#include "esim/sim_ref.h"
#include "rmax/scene_ref.h"
#include "rmax/scene.h"
#include "rmax/realScene_ref.h"
#include "rmax/BMPLoader.h"
#include "rmax/lidar.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "gcv/grabbercomponent.h"
#include "rmax/dataport.h"


/* Line Detector and Tracker */


gcvLDT::gcvLDT(sulObject* parent, const char *name, struct Ldt_ref *mypipe):
sulComponent(parent,name) {
    emypipe   = mypipe;
    mfg       = new gcvGrabberComponent(this,"fg");
    gfg       = mfg->getGrabberRef();
    mfg2      = new gcvGrabberComponent(this,"fg2");
    gfg2      = mfg2->getGrabberRef();
    mimg      = new gcvImage(this,"img");
    mimg2     = new gcvImage(this,"img2");
    medge     = new gcvImage(this,"edge");
    mgray     = new gcvImage(this,"gray");
    mjpeg     = new gcvImage(this,"jpeg");
    mline     = new gcvImage(this,"line");
    mtximage  = new gcvUchar[1024*768*3];
}

/// fgsimple destructor
gcvLDT::~gcvLDT() {
    this->shutdown();
    delete[] mtximage;
}

/// fgsimple initialization code (C++ function)
int gcvLDT::onInit() {


    mfg->setFgStruct(emypipe->fg); 
    mfg->init();
    
    mfg2->setFgStruct(emypipe->fg2);
    mfg2->init();
    
    
//     struct shm_remove
// 	{
// 		shm_remove() 
// 		{  
// 			boost::interprocess::shared_memory_object::remove("SharedMemoryImg"); 
// 			boost::interprocess::shared_memory_object::remove("SharedMemoryDep"); 
// 			boost::interprocess::shared_memory_object::remove("SharedMemoryState");
// 		}
// 		~shm_remove()
// 		{
// 			boost::interprocess::shared_memory_object::remove("SharedMemoryImg"); 
// 			boost::interprocess::shared_memory_object::remove("SharedMemoryDep"); 
// 			boost::interprocess::shared_memory_object::remove("SharedMemoryState");
// 		}
// 	} remover;
    
    
	boost::interprocess::shared_memory_object::remove("SharedMemoryImgL"); 
	boost::interprocess::shared_memory_object::remove("SharedMemoryImgR"); 
	boost::interprocess::shared_memory_object::remove("SharedMemoryDep"); 
	boost::interprocess::shared_memory_object::remove("SharedMemoryState");

	sharedMemImgL = boost::interprocess::shared_memory_object(boost::interprocess::open_or_create, "SharedMemoryImgL", boost::interprocess::read_write);
	sharedMemImgL.truncate(100000000);
	img_region_l = boost::interprocess::mapped_region(sharedMemImgL, boost::interprocess::read_write);
	inFrame_l = new (img_region_l.get_address()) shared_gen_mat;
	
	sharedMemImgR = boost::interprocess::shared_memory_object(boost::interprocess::open_or_create, "SharedMemoryImgR", boost::interprocess::read_write);
	sharedMemImgR.truncate(100000000);
	img_region_r = boost::interprocess::mapped_region(sharedMemImgR, boost::interprocess::read_write);
	inFrame_r = new (img_region_r.get_address()) shared_gen_mat;
    
	sharedMemDep = boost::interprocess::shared_memory_object(boost::interprocess::open_or_create, "SharedMemoryDep", boost::interprocess::read_write);
	sharedMemDep.truncate(100000000);
	dep_region = boost::interprocess::mapped_region(sharedMemDep, boost::interprocess::read_write);
	inFrame_d = new (dep_region.get_address()) shared_gen_mat;
    
	sharedMemState = boost::interprocess::shared_memory_object(boost::interprocess::open_or_create, "SharedMemoryState", boost::interprocess::read_write);
	sharedMemState.truncate(10000);
	state_region = boost::interprocess::mapped_region(sharedMemState, boost::interprocess::read_write);
	state = new (state_region.get_address()) shared_state;

   

#if defined(HAVE_HIGHGUI)
    if (emypipe->showgray)  {cvNamedWindow("gray",1); }
    if (emypipe->showedge)  {cvNamedWindow("edge",1); }
    if (emypipe->showsrc)   {cvNamedWindow("ldt_src",1);  }
    if (emypipe->showsrc2)  {cvNamedWindow("ldt_src2", 1); }
    if (emypipe->showline)  {cvNamedWindow("line", 1);}
    cvWaitKey(1);
#endif


    return SUL_OK;
}

/// fgsimple shutdown code (C++ function)
int gcvLDT::onShutdown() {
	
    mfg->shutdown();
    mfg2->shutdown();
#if defined(HAVE_HIGHGUI)
    cvDestroyWindow("gray");
    cvDestroyWindow("edge");
    cvDestroyWindow("src");
    cvDestroyWindow("jpeg");
    cvDestroyWindow("line");
    cvWaitKey(1);
#endif
    
	boost::interprocess::shared_memory_object::remove("SharedMemoryImgL"); 
	boost::interprocess::shared_memory_object::remove("SharedMemoryImgR"); 
	boost::interprocess::shared_memory_object::remove("SharedMemoryDep"); 
	boost::interprocess::shared_memory_object::remove("SharedMemoryState");
	
    return SUL_OK;
}

/// fgsimple update code (C++ function)
double wdbGetRangeModified( struct wdb_ref *wdb, double terrainAlt, double origin[], double dir[], double max, double angMax, double rangeSigma, double angSigma) {

    double p0[3], p1[3], p2[3], p3[3];
    double t, u, v;

#if 0 //old code
    double a[3], b[3], tmp3[3], nplane[3], q[3], c[3], d[3];
    double tmp1,tmp2,tmp4,angle,angle1,angle2,angle3,angle4,dist, normdir, normnplane;
    double epsilon = 0.1;
    double an, bn, cn, dn;
#endif

    double minRange = 2*max;
    double inc = 0, newInc = 0;

    int i,j;

	/* check ground plane */
	if( dir[2] > 0 && origin[2] < 0 ) {
		inc = acos( dir[2] );
		minRange = ( -origin[2] - terrainAlt )/dir[2];
	}

    /* check wdb objects */
    for(i = 0; i < wdb->numQuads; i++) {
        for(j = 0; j<3; j++) {
            p0[j] = wdb->quads[i][0][j];
            p1[j] = wdb->quads[i][1][j];
            p2[j] = wdb->quads[i][2][j];
            p3[j] = wdb->quads[i][3][j];
        }

        if( intersect_triangle( origin, dir, p0, p2, p1, &t, &u, &v, &newInc ) ) { // check intersection with first triangle
            if (t<minRange) {
                minRange = t;
				inc = newInc;
            }
        } else if( intersect_triangle( origin, dir, p0, p3, p2, &t, &u, &v, &newInc ) ) { // check intersection with second triangle
            if (t<minRange) {
                minRange = t;
				inc = newInc;
            }
        }
    }

    if ((inc > angMax*C_DEG2RAD)||(minRange > max)) {
        return 0;
    } else {
        return minRange;
    }
}
int gcvLDT::onUpdate(double time) {
    struct onboard2_ref *ob2		= &onboard2;
    struct vehicle_ref  *v		= &vehicle;
    struct vehicleMotion_ref *m		= v->motion;
    struct vehicleOutputs_ref *o	= v->outputs;
    struct wdb_ref  *wdb		= &gcswdb;
    struct scenes_ref *s		= &scenes;
    struct scene_ref *s1		= s->s1;
    struct sceneGlobal_ref *sg		= s->global;


	if(emypipe->runLdt){
	    mfg->getFrame(time,mimg);
	    mfg2->getFrame(time, mimg2);
	}
	
	
	if(emypipe->shareshark) 
	{

			

			boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> l_lock(inFrame_l->mutex);
			inFrame_l->dat_off = & inFrame_l->dat_arr;
			memcpy(inFrame_l->dat_off.get(), mimg->data() , mimg->size());
			inFrame_l->height = mimg->dimy();
			inFrame_l->width = mimg->dimx();
			
			boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> r_lock(inFrame_r->mutex);
			//write depth mat as img mat is written above
			inFrame_r->dat_off = & inFrame_r->dat_arr;
			memcpy(inFrame_r->dat_off.get(), mimg2->data() , mimg2->size());
			inFrame_r->height = mimg2->dimy();
			inFrame_r->width = mimg2->dimx();
			
			boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> d_lock(inFrame_d->mutex);
			//write depth mat as img mat is written above
			
			
// 			double view_pos_camera[3] = {2.5,0,0.57};
			double fov_x = 110;
			double fov_y = 61.875;
			double size_pix_x = fov_x/mimg->dimx();
			double size_pix_y = fov_y/mimg->dimy();
			double beamDir[3];
			int dep_size = mimg->dimx()*mimg->dimy();
			float dep_data1[dep_size];
			GLfloat dep_data2[dep_size];
// 			for(int i = 0; i < mimg->dimx()*mimg->dimy(); i++)
// 			{
// 				double theta =  (CV_PI/180)*((size_pix_x)*(i % mimg->dimx()) - fov_x/2 + o->theta);
// 				double psi = (CV_PI/180)*(floor(i/mimg->dimx())*size_pix_y - fov_y/2 + o->psi);
// 				beamDir[0] = cos(psi)*cos(theta);
// 				beamDir[1] = sin(psi);
// 				beamDir[2] = -cos(psi)*sin(theta);
// 				dep_data1[i] = wdbGetRangeModified( wdb, m->env->terrainAlt, o->pos, beamDir, 200, 900, 0, 0);
// 
// 				
// 			}

			
			GLfloat znear = sg->znear;
			GLfloat zfar = sg->vis;
			GLfloat zfactora = (zfar*znear/(znear - zfar));
			GLfloat zfactorb = zfar/(zfar-znear);
// 			std::cout << "s1->x" << s1->winw << std::endl << "s1->y" << s1->winh  << std::endl;
// 			std::cout << "Buffer Size: " << s1->grabberAllocatedSizeDepth << std::endl;
// 			std::cout << "dimx" << mimg->dimx()  << "dimy" << mimg->dimy()  << std::endl;
			


				GLfloat max_pix_depth1 = 0;
				int q1 = 0;
				for(int j = 1; j < mimg->dimy() + 1; j ++)
				{
					for(int i = 0; i < mimg->dimx(); i ++)
					{
						dep_data2[(mimg->dimy() - j)*(mimg->dimx()) + i] = zfactora/((((float) mimg->m_data_depth[(j*(mimg->dimx()) + i)])/(0xFFFFFFFF - 1)) - zfactorb);
						if((j*mimg->dimx() + i) % 10000 == 0)
// 							std::cout << j*mimg->dimx() << std::endl;
						if( !((float) mimg->m_data_depth[i]) == 0)
						{	
// 							std::cout << ( mimg->m_data_depth[i]) << std::endl;
// 							std::cout << ( dep_data2[i]) << std::endl;
							q1 ++;
						}
						if (dep_data2[i] > max_pix_depth1)
							max_pix_depth1 = dep_data2[i];
					}
				}
// 				for(int i = 1; i < dep_size ; i++)
// 				{
// 					dep_data2[i] = zfactora/((((float) mimg->m_data_depth[i])/(0xFFFFFFFF - 1)) - zfactorb);
// 					if((i) % 10000 == 0)
// 						std::cout << i << std::endl;
// 					if( !((float) mimg->m_data_depth[i]) == 0)
// 					{	
// // 						std::cout << ( mimg->m_data_depth[i]) << std::endl;
// // 						std::cout << ( dep_data2[i]) << std::endl;
// 						q1 ++;
// 					}
// 					if (dep_data2[i] > max_pix_depth1)
// 						max_pix_depth1 = dep_data2[i];
// 				}
// 				std::cout << "--------------------------------" << std::endl << (max_pix_depth1) << std::endl << "---------------------" << std::endl;
// 				std::cout << "--------------------------------" << std::endl << q1 << std::endl << "---------------------" << std::endl;
				inFrame_d->dat_off = & inFrame_d->dat_arr;
				memcpy(inFrame_d->dat_off.get(), dep_data2 , dep_size*4);
// 	 			memset(inFrame_d->dat_off.get(), 5, dep_size*4);
				inFrame_d->height = mimg->dimy();
				inFrame_d->width = mimg->dimx();
// 			glutSetWindow(startWindow);
			
			boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> state_lock(state->mutex);
			//read in stat values
			double psi_target = (CV_PI/180)*(fov_y/2 - ((double)state->y)*size_pix_y  + o->psi);
			double theta_target =  (CV_PI/180)*((size_pix_x)*((double)state->x) - fov_x/2 + o->theta);
			double targetNED[3]; 
			targetNED[0] = cos(psi_target)*cos(theta_target)*state->z + o->pos[0];
			targetNED[1] = sin(psi_target)*state->z + o->pos[1];
			targetNED[2] = -cos(psi_target)*sin(theta_target)*state->z + o->pos[2];
			
			std::cout << "N" << targetNED[0]  << " E" << targetNED[1]  << " D" << targetNED[2]  << "NV" << o->pos[0]  << " EV" << o->pos[1]  << " DV" << o->pos[2] << std::endl;;

	}
	
  
    
    if( emypipe->runedge || emypipe->showgray ){
        mgray->resize(mimg->dimx(),mimg->dimy(),1,mimg->depth());
        cvCvtColor(mimg->ipl(), mgray->ipl(), CV_BGR2GRAY);
    }

    if (emypipe->runedge) {
        medge->resize(mgray);


        cvSmooth( mgray->ipl(), medge->ipl(), CV_BLUR, 3, 3, 0, 0 );
        cvNot( mgray->ipl(), medge->ipl() ); 

        // Run the edge detector on grayscale
        cvCanny(mgray->ipl(), medge->ipl(), emypipe->edge_thresh, emypipe->edge_thresh*3, 3);

    }
	
    if (emypipe->runline)
    {
	//creates cvMat to hold line results in a single float precision for two variables per entry, in this case rho and theta of each found line
	//runs line transform, 3rd param: 0 for original, 1 for probablistic, 4th param for rho precision, 5th for theta precision
	CvMat* hlines = cvCreateMat(emypipe->num_hlines, 1, CV_32FC2);
	
	cvHoughLines2(medge->ipl(), hlines, 0,  1, 0.01745, emypipe->line_thresh);
		
	for (int q = 0; q < hlines->rows; q++) 
	{
		//retrieves the rho of the current index from the cvMat
		float* rho_current = (float*)(hlines->data.ptr + (size_t)hlines->step*q);
		//retrieves the theta of the current index by shifting a float over
		float* theta_current = (float*)(hlines->data.ptr + (size_t)hlines->step*q + 4);
		
		//checks if within the angular region specified
		if((*theta_current > (0.5 - (emypipe->ang_range))*CV_PI) && (*theta_current < (0.5 + (emypipe->ang_range))*CV_PI))
		{
		      //calculates line and extreme points to make it seem infinite
		      double a_current = cos(*theta_current), b_current = sin(*theta_current);
		      double x0_current = a_current*(*rho_current), y0_current = b_current*(*rho_current);
		      CvPoint pt1_current = cvPoint(cvRound(x0_current + 1000*(-b_current)), cvRound(y0_current + 1000*(a_current)));
		      CvPoint pt2_current = cvPoint(cvRound(x0_current - 1000*(-b_current)), cvRound(y0_current - 1000*(a_current)));
		      //draws line on src image
		      cvLine(mimg->ipl(), pt1_current, pt2_current, cvScalar(100, 100, 255));
		}
	}
}


    gcvImage *tximg = 0;
    gcvUlong jpegSize = 0;
    if(emypipe->showgray)  {tximg = mgray;}
    if(emypipe->showedge)  {tximg = medge;}
    if(emypipe->showsrc)   {tximg = mimg;} 
    if(emypipe->showline)  {tximg = mline;}

 
    
    if(tximg && emypipe->sendImage && time >= emypipe->lastSend + emypipe->sendDt) {
        emypipe->lastSend = time;

        tximg->saveJpeg(this->mtximage,1024*768*3,emypipe->txquality,&jpegSize);
        dataPortSendMsg(ob2->dataport, (sulbyte*)(this->mtximage), jpegSize, 0);

    }


#if defined(HAVE_HIGHGUI)
    if (emypipe->showgray)  {cvShowImage("gray",mgray->ipl());}
    if (emypipe->showedge)  {cvShowImage("edge", medge->ipl());}
    if (emypipe->showsrc)   {cvShowImage("ldt_src",mimg->ipl());}
    if (emypipe->showsrc2)  {cvShowImage("ldt_src2", mimg2->ipl());}
    if (emypipe->showline)  {cvShowImage("line", mline->ipl());}


    cvWaitKey(1);
#endif



    return SUL_OK;
}



