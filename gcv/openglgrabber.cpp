#include "gcv/gcv.h"

#if defined(HAVE_OPENGLGRABBER)
#include "gcv/openglgrabber.h"
#include "gcv/image.h"
#include "sul/sul.h" 
#include "gcv/gcv_ref.h"
#include "rmax/scene.h"

static int mypatmat(const char *raw, const char *pat) {
    int  i, slraw;

    if ((*pat == '\0') && (*raw == '\0'))          //  if it is end of both
        return( 1 ) ;                              //  strings,then match
    if (*pat == '\0')                              //  if it is end of only
        return( 0 ) ;                              //  pat tehn mismatch
    if (*pat == '*') {                             // if pattern is a '*'
        if (*(pat+1) == '\0')                      //    if it is end of pat
            return( 1 ) ;                          //    then match
        for (i=0,slraw=strlen(raw);i<=slraw;i++)   //    else hunt for match
            if ((*(raw+i) == *(pat+1)) ||          //         or wild card
                (*(pat+1) == '?'))
                if (mypatmat(raw+i+1,pat+2) == 1)    //      if found,match
                    return( 1 ) ;                     //        rest of pat
    } else {
        if (*raw == '\0')                             //  if end of raw then
            return( 0 ) ;                             //     mismatch
        if ((*pat == '?') || (*pat == *raw))          //  if chars match then
            if (mypatmat(raw+1,pat+1) == 1)          //  try & match rest of it
                return( 1 ) ;
    }
    return( 0 ) ;                                     //  no match found
}


gcvOpenglGrabber::gcvOpenglGrabber(sulObject *parent, const char *name, struct gcvGrabber_ref *efg) : gcvGrabber(parent,name,efg) {
}

gcvOpenglGrabber::~gcvOpenglGrabber() {
	this->shutdown();
}

/*
int gcvOpenglGrabber::initScene(const char *src) {

	struct scene_ref *scene = 0;
	if(mypatmat(src,"scene0")) {
		scene = &scene0;
	} else if(mypatmat(src,"scene1")) {
	    scene = &scene1;
	} else if(mypatmat(src,"scene2")) {
	    scene = &scene2;
	}

//	if(scene) {
		//this->initCoord(scene->x, scene->y, scene->winw, scene->winh);
		//return GCV_OK;
	//}
	return GCV_NOTOK;
}*/

int gcvOpenglGrabber::shutdown() {
	/* it would be nice to turn off of the scene part of this - but that wouldn't work if more than
	   one grabber was using it */
	window = atoi( &(mwinName[5]) );
	if( sceneGrabberShutdown( window ) == 0 ) {
		return SUL_OK;
	}
	return SUL_NOTOK;
}

int gcvOpenglGrabber::initWindow(const char *name) {
	sulStrncpy(mwinName,name,MAX_WINDOWNAME_LEN);

	window = atoi( &(mwinName[5]) );

	if( sceneGrabberStart( window ) == 0 ) {
		return SUL_OK;
	}
	
	return SUL_NOTOK;
}
int gcvOpenglGrabber::getStereoFrame(double *time0, gcvImage *img0, double *time1, gcvImage *img1) {
	// this is a fake stereoframe for testing purposes, both images are exactly the same
    efg->times[0] = sulWallTime();
	this->getFrame(img0);
	efg->frames[0]++;
	efg->newframes[0]=1;

	efg->times[1] = sulWallTime();
	img1->resize(img0);
	efg->frames[1]++;
	efg->newframes[1]=1;

	cvCopy( img0->ipl(), img1->ipl(), 0 );
	*time0 = efg->times[0];
	*time1 = efg->times[1];
	return SUL_OK;
}

int gcvOpenglGrabber::getFrame(gcvImage *img) {

	int ret = SUL_NOTOK, winw, winh;
	char *data;
	gcvUint *data_depth;
	if( sceneGrabberFrame( window, &winw, &winh, (void**)&data ) == 0 ) {
#define BYTESPERPIXEL 3
		int rj, k;
		img->resize( winw, winh, 3 );
        efg->dimx = winw;
        efg->dimy = winh;
		for(int j = 0; j < winh; j++) {
			rj = winh-j-1;
			for(int i = 0; i < winw; i++) {
				k = (j*winw + i)*BYTESPERPIXEL;
				PIXIJ(img,i,rj,0) = data[k];
				PIXIJ(img,i,rj,1) = data[k+1];
				PIXIJ(img,i,rj,2) = data[k+2];
			}
		}
		ret = SUL_OK;
		sceneGrabberDepth(window, &winw, &winh, (void**)&(img->m_data_depth));
			
	}
	return ret;
}

#endif // HAVE_OENGLGRABBER


