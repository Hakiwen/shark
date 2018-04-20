#ifndef GCV_IMAGE_H
#define GCV_IMAGE_H
#ifdef HAVE_OPENCV
#include "opencv/cv.h"
#include "gcv/gcv.h"
#include "sul/object.h"
#include "esim/util.h"
#include <string.h>
#include <stdio.h>

//#if defined(HAVE_GCVLIBPNG)
//#include "gcv/libpng/png.h"
//#endif
// access for contiguous rgb ordering of data
// with this macro J enumerates dimy and I enumerates dimx
#define IPLPIXIJ( IMG, I, J, CHAN ) *( (gcvUchar*)(IMG)->imageData + (IMG)->widthStep*(J) + (IMG)->nChannels*(I) + (CHAN) )
#define PIXIJ( IMG, I, J, CHAN )    *( (gcvUchar*)(IMG)->miplimg->imageData + (IMG)->miplimg->widthStep*(J) + (IMG)->miplimg->nChannels*(I) + (CHAN) )
#define PIXEL( IMG, I, CHAN )       *( (gcvUchar*)(IMG)->miplimg->imageData + (I)*(IMG)->miplimg->nChannels + (CHAN) )
// these return a pointer to the begining of row J
#define ROWJ( IMG, J)               (gcvUchar*)(IMG)->miplimg->imageData + (IMG)->miplimg->widthStep*(J)
#define IPLROWJ( IMG, J)            (gcvUchar*)(IMG)->imageData + (IMG)->widthStep*(J)

#define RCHAN 2
#define GCHAN 1
#define BCHAN 0
#define HCHAN 0
#define SCHAN 1
#define VCHAN 2

#define GCV_DEPTH_1U  IPL_DEPTH_1U
#define GCV_DEPTH_8U  IPL_DEPTH_8U
#define GCV_DEPTH_16U IPL_DEPTH_16U
#define GCV_DEPTH_32F IPL_DEPTH_32F

class gcvImage : public sulObject {
    SUL_DEFINE_TYPE(gcvImage);
public:

    /// constructs an image of x by y pixels with chan number of channels
    gcvImage(sulObject *parent = 0, const char *name = 0, int tdimx=320, int tdimy=240, int tnchannels=3, int depth=GCV_DEPTH_8U );

    /// deallocates and destroys image
    ~gcvImage();

    /// returns x dimension of image
    int dimx() {return miplimg->width;}

    /// returns y dimension of image
    int dimy() {return miplimg->height;}

    /// returns number of channels in image
    int nofChannels() {return miplimg->nChannels;}

    /// returns depth of the image
    int depth() {return miplimg->depth;}

    // returns number of bytes per row of image
    int widthStep() {return miplimg->widthStep;}

    /// returns total number of bytes (aligned) bytes
    gcvUlong size() { return miplimg->widthStep*miplimg->height;}

    /// return number of pixels, useful for iterating over pixles
    gcvUlong nofPixels() {return miplimg->width * miplimg->height;}

    /// returns pointer to raw data
    gcvUchar *data() {return (gcvUchar*)(miplimg->imageData);}

    /// returns pointer to the underlying IplImage
    IplImage *ipl() {return miplimg;}
    
    // returns pointer to underlying depth match
    gcvUint *data_depth() {return m_data_depth;}
    
    /// redimensions and if necessary reallocates image to desired dimensions
    int resize(int tdimx, int tdimy, int tnchannels, int depth=GCV_DEPTH_8U);

    /// redimensions to match given gcvImage
    int resize(gcvImage *img);

    /// redimensions to match given IplImage
    int resize(IplImage *img);

//    gcvUchar& atp(const unsigned int i, const unsigned int channel=0) {

    	//return miplimg->imageData;
        //return mdata[i*mnchannels + channel];
    //}

    /// draw line
    void drawImageAxes();

	void saveRgb(const char *fn);

#if defined(HAVE_HIGHGUI)
    void saveBMP(const char *fn);
    void loadBMP(const char *fn);
#endif

#if defined(HAVE_GCVLIBPNG)
    int savePNG(const char *fileName);
    int savePNG(const char *fileName, int quality);
    int loadPNG(const char *fileName);
#endif


#if defined(HAVE_GCVLIBJPEG)
    /// returns pointer to jpegdata
    int cacheJpeg(int quality=80) {
        if(mjpegIsDirty) {
            this->saveJpeg(mjpegData, mallocSize, quality, &mjpegDataSize);
        }
        this->mjpegIsDirty = false;
        return GCV_OK;
    }
    gcvUchar *jpegData() {return mjpegData;}
    gcvUlong jpegDataSize() {return mjpegDataSize;}


    int saveJpeg(const char *tfn, int quality);
    int saveJpeg(FILE *outfile, int quality);
    int loadJpeg(const char *fn);
    int loadJpeg(FILE *infile);

    int loadJpeg(gcvUchar *buf, gcvUlong bufSize);
	int saveJpeg(gcvUchar *buf, gcvUlong bufSize, int quality, gcvUlong *cbufSize);

#endif

#if HAVE_OPENCV
    //void loadCvImage(const char *tfn);
    //void copyCvImage(struct _IplImage *timg, int flip = 0);
#endif

#if defined(HAVE_S311)
    void copyS311Image(struct ipFg_ref *fg);
#endif

private:
    /// deallocates memory for the image
    void freeImage();

    /// if 1, mdata is deleted when this image is deleted
    bool mowner;
    
    //size of allocated buffer for depth data
    gcvUlong mallocSizeDepth;
    
#if defined(HAVE_GCVLIBJPEG)
private:
    /// jpeg raw data
    gcvUchar *mjpegData;
    /// jpeg raw data buffer size
    gcvUlong mjpegDataSize;
    /// does it have the latest jpeg
    bool mjpegIsDirty;
    /// maximum size of allocated buffer for jpeg data
    gcvUlong mallocSize;
    
public:

#endif



public: // is public but really only used for the pixel access macros.

    /// lock to lock access to image
    //class sulMutex *lock;

    IplImage *miplimg; // this is bgr
    IplImage *miplimgrgb;	// we need to when we want to save to jpeg where we convert to rgb then save
    gcvUint  *m_data_depth;


};

#include <stdio.h>

inline int gcvImage::resize(gcvImage *img) {
	return this->resize(img->dimx(),img->dimy(),img->nofChannels(),img->depth());
}
inline int gcvImage::resize(IplImage *img) {
	return this->resize(img->width,img->height,img->nChannels,img->depth);
}
inline int gcvImage::resize(int tdimx, int tdimy, int tnchannels,int depth) {
    if(miplimg == 0 || miplimg->width != tdimx || miplimg->height != tdimy || miplimg->nChannels != tnchannels || miplimg->depth != depth) {
		this->freeImage();
		this->miplimg    = cvCreateImage(cvSize(tdimx,tdimy),depth,tnchannels);
		this->miplimgrgb = cvCreateImage(cvSize(tdimx,tdimy),depth,tnchannels);  
#if defined(HAVE_GCVLIBJPEG)
		this->mallocSize = miplimg->widthStep*miplimg->height;
		this->mallocSizeDepth = miplimg->width*miplimg->height;
        mjpegData  = new gcvUchar[this->mallocSize]; // this assumes jpeg data is always < raw data
        m_data_depth =(gcvUint *)malloc(mallocSizeDepth*4);
        mowner     = true; // if you allocate memory, make sure this is true
#endif
        //printf("\nimage reallocated and resized to %dx%d",tdimx,tdimy);
    }
#if defined(HAVE_GCVLIBJPEG)
    mjpegIsDirty = true;
#endif
    return GCV_OK;
}

inline void gcvImage::drawImageAxes() {
    int xlen = LIMIT(30,0,dimx());
    int ylen = LIMIT(30,0,dimy());
    int len  = MIN(xlen,ylen);
    if (this->nofChannels() == 3) {
        for (int i = 0; i < len; i++) {
            PIXIJ(this,i,1,0) = 255;
            PIXIJ(this,i,1,1) = 255;
            PIXIJ(this,i,1,2) = 255;
            PIXIJ(this,1,i,0) = 255;
            PIXIJ(this,1,i,1) = 255;
            PIXIJ(this,1,i,2) = 255;
        }
    } else if (this->nofChannels() == 1) {
        for (int i = 0; i < len; i++) {
            PIXIJ(this,i,1,0) = 255;
            PIXIJ(this,1,i,0) = 0;
        }
    } else if (this->nofChannels() == 2) {
        for (int i = 0; i < len; i++) {
            PIXIJ(this,i,1,0) = 255;
            PIXIJ(this,i,1,1) = 0;
            PIXIJ(this,1,i,0) = 0;
            PIXIJ(this,1,i,1) = 255;
        }
    }
}


inline void gcvRgb2Hsv(gcvImage *src, gcvImage *dst) {
    dst->resize(src->dimx(),src->dimy(),3);
    gcvUlong nofPixels = src->nofPixels();
    double var_R, var_G, var_B, var_Min, var_Max, del_Max, V, H, S;

    for (gcvUlong i = 0; i < nofPixels; i++) {

        //PIXEL(dst,i,0) = (gcvUchar)( ( PIXEL(src,i,0) + PIXEL(src,i,1) + PIXEL(src,i,2) )/3.0 );

        // from en.wikipedia.org/wiki/HSV_color_space#Transformation_from_RGB_to_HSV

        var_R = (PIXEL(src,i,RCHAN)/(double)255);
        var_G = (PIXEL(src,i,GCHAN)/(double)255);
        var_B = (PIXEL(src,i,BCHAN)/(double)255);

        var_Min = MIN(var_R, MIN(var_G, var_B));
        var_Max = MAX(var_R, MAX(var_G, var_B));
        del_Max = var_Max-var_Min;

        V = var_Max;

        if (var_Max == var_Min) {
            H = 0;
            S = 0;
        } else {
            S = (var_Max-var_Min)/var_Max;
            if (var_Max == var_R) {
                H = ((var_G-var_B)/del_Max)/6.0;
            }
            if (var_Max == var_G) {
                H = (2+((var_B-var_R)/del_Max))/6.0;
            }
            if (var_Max == var_B) {
                H = (4+((var_R-var_G)/del_Max))/6.0;
            }
            while (H > 1) {
                H = H-1;
            }
            while (H < 0) {
                H = 1+H;
            }
        }

        PIXEL(dst,i,HCHAN) = (gcvUchar)(H*255.0);
        PIXEL(dst,i,SCHAN) = (gcvUchar)(S*255.0);
        PIXEL(dst,i,VCHAN) = (gcvUchar)(V*255.0);
    }
}

inline void gcvRgb2Gray(gcvImage *src, gcvImage *dst) {
    dst->resize(src->dimx(),src->dimy(),1);
    gcvUlong nofPixels = src->nofPixels();
    for (gcvUlong i = 0; i < nofPixels; i++) {
        PIXEL(dst,i,0) = (gcvUchar)( ( PIXEL(src,i,0) + PIXEL(src,i,1) + PIXEL(src,i,2) )/3.0 );
    }
}

inline void gcvRgb2GrayWeightedMax(gcvImage *src, gcvImage *dst, double redCoeff, double greenCoeff, double blueCoeff) {
    gcvUlong nofPixels = src->nofPixels();
    double newpixel;
    double red,green,blue;
    dst->resize(src->dimx(), src->dimy(), 1);

    for (gcvUlong i = 0; i < nofPixels; i++) {
        red     = redCoeff   * PIXEL(src, i, RCHAN);
        green   = greenCoeff * PIXEL(src, i, GCHAN);
        blue    = blueCoeff  * PIXEL(src, i, BCHAN);
        newpixel = MAX( red, green );
        newpixel = MAX( newpixel, blue );
        PIXEL(dst, i, 0) = (gcvUchar)newpixel;
    }
}

inline void gcvCopyImage(gcvImage *src, gcvImage *dst) {
    dst->resize(src->dimx(), src->dimy(), src->nofChannels());
    cvCopy( src->ipl(), dst->ipl(), 0 );
    memcpy(dst->m_data_depth, src->m_data_depth, sizeof(gcvUint)*src->nofPixels());
}

int gcvSwapRB(IplImage *src, IplImage *dst);
#endif //GCV_IMAGE_H

#endif // HAVE_OPENCV
