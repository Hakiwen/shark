#ifndef GCV_OPENGLGRABBER_H
#define GCV_OPENGLGRABBER_H

#include "gcv/gcv.h"
#include "gcv/grabber.h"

// some system include headers are done here
#if defined(HAVE_OPENGLGRABBER)

class gcvOpenglGrabber : public gcvGrabber {
public:
	gcvOpenglGrabber(sulObject *parent = 0 , const char *name = 0, struct gcvGrabber_ref *efg = 0);
	virtual ~gcvOpenglGrabber();
	virtual int getStereoFrame(double *time0, gcvImage *img0, double *time1, gcvImage *img1);
	//int initScene(const char *src);
	//int initCoord(int x, int y, int w, int h);
	int initWindow(const char *name);
	int getFrame(gcvImage *img);
	int shutdown();
private:
	static const int MAX_WINDOWNAME_LEN = 1024;
	char mwinName[MAX_WINDOWNAME_LEN];   /// name of window we want to capture
	int window;
};

#endif // HAVE_OPENGLGRABBER

#endif // GCV_OPENGLGRABBER_H
