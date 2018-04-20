#ifdef HAVE_OPENCV

#include <stdio.h>
#include "gcv/gcv.h"
#include "sul/object.h"
#include <stdio.h>
#if defined(HAVE_GCVLIBJPEG)

#include "gcv/libjpeg/jinclude.h"
extern "C" {
#include "gcv/libjpeg/jpeglib.h"
}
#include <setjmp.h>
#endif

#if defined(HAVE_GCVLIBPNG)
// configuring how the libpng files get compiled
#define PNG_NO_MNG_FEATURES
#include "gcv/libpng/png.h"
#endif

#include "gcv/image.h"
#include "sul/sul.h"


#include "opencv/cv.h"
#if defined(HAVE_HIGHGUI)
#include "opencv/highgui.h"
#endif
#if defined(HAVE_CVCAM)
#include "opencv/cvcam.h"
#endif


// only need this for now because we use alison's iplib to read s311 grabber
#if defined(HAVE_S311)
#include "iplib/iplib.h"
#endif

gcvImage::gcvImage(sulObject *parent, const char *name, int tdimx, int tdimy, int tnchannels, int depth) : sulObject(parent,name) {
	mowner     = true;
	mjpegData  = 0;
	mjpegDataSize = 0;
	m_data_depth = 0;
    mjpegIsDirty = false;
    mallocSize = 0;
	//lock      = new sulMutex();
	miplimg = 0;
	miplimgrgb = 0;
	this->resize(tdimx, tdimy, tnchannels, depth);
}

gcvImage::~gcvImage() {
	this->freeImage();
    //delete lock;
}


void gcvImage::freeImage() {
	if(mowner) {
		if(miplimg) {
			cvReleaseImage(&miplimg);
		}
	}
	if(miplimgrgb) {
		cvReleaseImage(&miplimgrgb);
	}
	if(m_data_depth)
	{
		delete[] m_data_depth;
		m_data_depth = 0;
	}
    if(mjpegData) {
        delete[] mjpegData;
		mallocSize = 0;
        mjpegData = 0;
    }
}

#if HAVE_OPENCV
/*
void gcvImage::loadCvImage(const char *tfn) {
//	IplImage *cvimg  = cvLoadImage(tfn,1);
	//this->copyCvImage(cvimg);
	//cvReleaseImage(&cvimg);
}

void gcvImage::copyCvImage(IplImage *timg, int flip) {
	// on windows when we load a jpeg using opencv it comes in bgr format
	// so we reorder the bytes
	if(timg) {
		this->resize(timg->width,timg->height,timg->nChannels);
		int npixels   = this->nofPixels();
//		int nchannels = this->nofChannels();
		for(int i = 0; i < npixels; i++) {
			//mdata[i*nchannels + 0] = timg->imageData[i*timg->nChannels + 0];
			//mdata[i*nchannels + 1] = timg->imageData[i*timg->nChannels + 1];
			//mdata[i*nchannels + 2] = timg->imageData[i*timg->nChannels + 2];
			PIXEL(this,i,0) = timg->imageData[i*timg->nChannels + 0];
			PIXEL(this,i,1) = timg->imageData[i*timg->nChannels + 1];
			PIXEL(this,i,2) = timg->imageData[i*timg->nChannels + 2];

		}
		//memcpy(mdata,timg->imageData,timg->width*timg->height*)
	}
}*/

#endif //HAVE_OPENCV

#if defined(HAVE_S311)
void gcvImage::copyS311Image(struct ipFg_ref *fg) {
	gcvUchar *ptr = fg->pixels;
	this->resize(fg->sizeX,fg->sizeY,3);
	gcvUlong nofPixels = this->nofPixels();

	int pos=0;
		/*
	for(gcvUlong i = 0; i < nofPixels; i++) {
		PIXEL(this,i,0) = ptr[pos+2];
		PIXEL(this,i,1) = ptr[pos+1];
		PIXEL(this,i,2) = ptr[pos+0];
		pos += 3;
	}*/

	// FIXME
	for (int i=0; i< fg->sizeY; i++) {
		for (int j=0; j<fg->sizeX; j++) {
			// Putting data along scan lines but
			// framegrabber scan line one is the lowest part of picture
			// we have to flip the data into correct format
			mdata[3*fg->sizeX*(fg->sizeY-1-i) + 3*j + 0]=ptr[pos+2];
			mdata[3*fg->sizeX*(fg->sizeY-1-i) + 3*j + 1]=ptr[pos+1];
			mdata[3*fg->sizeX*(fg->sizeY-1-i) + 3*j + 2]=ptr[pos+0];
			pos+=3;
		}
	}


}
#endif

void gcvImage::saveRgb(const char *fn) {

	int i,j;
	FILE *fp = fopen(fn,"wb");
	gcvUchar pixel;
	if(fp) {
		for(j=0;j<this->dimy();j++) {
			for(i=0;i<this->dimx();i++) {
				pixel = PIXIJ(this,i,j,RCHAN);
				fwrite(&pixel,sizeof(pixel),1,fp);	
			}
		}
        
		for(j=0;j<this->dimy();j++) {
			for(i=0;i<this->dimx();i++) {
				pixel = PIXIJ(this,i,j,GCHAN);
				fwrite(&pixel,sizeof(pixel),1,fp);
			}
		}
		for(j=0;j<this->dimy();j++) {
			for(i=0;i<this->dimx();i++) {
				pixel = PIXIJ(this,i,j,BCHAN);
				fwrite(&pixel,sizeof(pixel),1,fp);
			}
		}

		fclose(fp);
		//sulInfo(("wrote %s dimx = %d, dimy = %d",fn, this->dimx(), this->dimy()));
	} else {
		sulError(("cannot open %s",fn));
	}
}

#if defined(HAVE_HIGHGUI)
void gcvImage::saveBMP(const char *fn) {

	char fileName[256];
	sprintf(fileName, "%s.bmp", fn);
	//cvSaveImage( fileName, this->ipl() );
}

void gcvImage::loadBMP(const char *fn) {
	IplImage *tmpImg = 0;

	char fileName[256];
	sprintf(fileName, "%s.bmp", fn);

	//tmpImg = cvLoadImage(fileName, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	this->resize(tmpImg);
	cvCopy( this->ipl(), tmpImg, 0 );
}
#endif 

#if defined(HAVE_GCVLIBPNG)
int gcvImage::savePNG(const char *fileName) {
	return savePNG(fileName,Z_DEFAULT_COMPRESSION);
}
int gcvImage::savePNG(const char *fileName, int quality) {
	FILE *fp;
	png_structp png_ptr;
	png_infop info_ptr;
	int irow;
	int colorType;


	if ((fp = fopen(fileName, "wb")) == NULL) {
	  printf("gcvImage::savePNG can't open %s\n",fileName);
	  return GCV_NOTOK;
	}

    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, png_voidp_NULL, NULL, NULL);
    if (!png_ptr) {
       return GCV_NOTOK;
	}

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
       png_destroy_write_struct(&png_ptr, (png_infopp)NULL );
       return GCV_NOTOK;
    }


	if (setjmp(png_ptr->jmpbuf)) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return GCV_NOTOK;
    }	

	png_init_io(png_ptr, fp);

	quality = LIMIT(quality, Z_DEFAULT_COMPRESSION, Z_BEST_COMPRESSION);
	png_set_compression_level(png_ptr, quality);

	if ( this->depth()==GCV_DEPTH_16U || this->depth()==GCV_DEPTH_8U ) {
		switch(this->nofChannels()) {
		case 1:
			colorType = PNG_COLOR_TYPE_GRAY;
			break;
		case 3:
			colorType = PNG_COLOR_TYPE_RGB;
			break;
		default:
			png_error(png_ptr, "imagecv.cpp: color type not supported for png image save.");
		}
		png_set_IHDR(png_ptr, info_ptr, this->dimx(), this->dimy(), 
					 this->depth(),	colorType,	PNG_INTERLACE_NONE, 
					 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
	} else {
		png_error(png_ptr, "imagecv.cpp: color depth not supported for png image save.");
	}

    png_write_info(png_ptr, info_ptr);

	// before we write the image we need to swap the b/r channels (opencv is bgr, libpng is rgb)
	switch (colorType) {
	case PNG_COLOR_TYPE_RGB:
		gcvSwapRB(miplimg, miplimgrgb);
		for( irow=0;irow<this->dimy();irow++ ) {
			png_write_row(png_ptr, IPLROWJ( miplimgrgb, irow) );
		}
		break;
	case PNG_COLOR_TYPE_GRAY:
		for( irow=0;irow<this->dimy();irow++ ) {
			png_write_row(png_ptr, IPLROWJ( miplimg, irow) );
		}
		break;
	}

    png_write_end(png_ptr, NULL);

	fclose(fp);
	return GCV_OK;
}

int gcvImage::loadPNG(const char *fileName) {
	FILE *fp;
	png_structp png_ptr;
	png_infop info_ptr;
	gcvUchar sig[8];
	gcvUlong width, height, rowbytes;
	int channels;
	int bit_depth, color_type;
	gcvUlong i;
	double gamma;


	if ((fp = fopen(fileName, "rb")) == NULL) {
		printf("gcvImage::loadPNG can't open %s\n",fileName);
		return GCV_NOTOK;
	}

	// check that image signature fits png
	fread(sig, 1, 8, fp);
	if (!png_check_sig(sig, 8)) {
		return GCV_NOTOK;   /* bad signature */
	}

    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL,
      NULL);
    if (!png_ptr)
        return GCV_NOTOK;   /* out of memory */
  
    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        return GCV_NOTOK;   /* out of memory */
    }

    if (setjmp(png_ptr->jmpbuf)) {
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        return GCV_NOTOK;
    }

    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, 8);
    png_read_info(png_ptr, info_ptr);

    png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth,
      &color_type, NULL, NULL, NULL);

	if (setjmp(png_jmpbuf(png_ptr))) {
		png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
		return GCV_NOTOK;
	}

	if (color_type == PNG_COLOR_TYPE_PALETTE)
		png_set_expand(png_ptr);
	if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
		png_set_expand(png_ptr);
	if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
		png_set_expand(png_ptr);
	if (bit_depth == 16)
		png_set_strip_16(png_ptr);

	// NOTE if it is a gray scale image we are basically saying, when uncompressing convert it to rgb 24 bit.
	/*
	if (color_type == PNG_COLOR_TYPE_GRAY ||
		color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
		png_set_gray_to_rgb(png_ptr);
		*/

	if (png_get_gAMA(png_ptr, info_ptr, &gamma))
		// I'm using a default display exponent here...
		// could use some other fancy code here, but we 
		// don't really care about precision colors 
		png_set_gamma(png_ptr, 2.2, gamma);

	rowbytes = png_get_rowbytes(png_ptr, info_ptr);
	channels = (int)png_get_channels(png_ptr, info_ptr);

    png_read_update_info(png_ptr, info_ptr);

	
	if (this->resize(width, height, channels, bit_depth) == GCV_NOTOK) {
		png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
		return GCV_NOTOK;
	}

	height=png_ptr->height;
	png_ptr->num_rows = height; /* Make sure this is set correctly */
	
	if (channels == 3) {
		for (i = 0; i < height; i++) {
		   png_read_row(png_ptr, IPLROWJ(this->miplimgrgb, i), png_bytep_NULL);
		}
		gcvSwapRB(miplimgrgb,miplimg);
	} else if (channels == 1) {
		for (i = 0; i < height; i++) {
		   png_read_row(png_ptr, IPLROWJ(this->miplimg, i), png_bytep_NULL);
		}
	}
	
	png_read_end(png_ptr, NULL);

	png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
	fclose(fp);
	
	return GCV_OK;
}
#endif

#if defined(HAVE_GCVLIBJPEG)

/// custom error handler
struct my_error_mgr {
  struct jpeg_error_mgr pub;	// "public" fields
  jmp_buf setjmp_buffer;	// for return to caller
};

typedef struct my_error_mgr * my_error_ptr;

// this routine replaces the standard error routine that calls exit()
METHODDEF(void) my_error_exit (j_common_ptr cinfo) {
  // cinfo->err really points to a my_error_mgr struct, so coerce pointer
  my_error_ptr myerr = (my_error_ptr) cinfo->err;

  // Always display the message.
  // We could postpone this until after returning, if we chose.
  //(*cinfo->err->output_message) (cinfo);

  // Return control to the setjmp point
  longjmp(myerr->setjmp_buffer, 1);
}

// this function is defined in jmemsrc.c
extern "C" void jpeg_mem_src (j_decompress_ptr cinfo, gcvUchar *srcbuf, gcvUlong srcbufSize);
int gcvImage::loadJpeg(gcvUchar *buf, gcvUlong bufSize) {
    struct jpeg_decompress_struct cinfo;
    struct my_error_mgr jerr;
    JSAMPARRAY buffer;		/* Output row buffer */
    int row_stride;		/* physical row width in output buffer */

	if (bufSize == 0) return SUL_NOTOK;

    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_exit;
    if (setjmp(jerr.setjmp_buffer)) {
      jpeg_destroy_decompress(&cinfo);
	  return SUL_NOTOK;
    }
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, buf, bufSize);
    (void) jpeg_read_header(&cinfo, TRUE);
    (void) jpeg_start_decompress(&cinfo);
	this->resize(cinfo.output_width, cinfo.output_height, cinfo.output_components);
    row_stride = this->widthStep();
    buffer = (*cinfo.mem->alloc_sarray)
          ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);
    
    while (cinfo.output_scanline < cinfo.output_height) {
      (void) jpeg_read_scanlines(&cinfo, buffer, 1);
      if(cinfo.output_components == 1 || cinfo.output_components == 2) {
          memcpy(miplimg->imageData + (cinfo.output_scanline-1)*row_stride, buffer[0], row_stride);
      } else {
          memcpy(miplimgrgb->imageData + (cinfo.output_scanline-1)*row_stride, buffer[0], row_stride);
      }
    }
    (void) jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    // make a copy of the raw jpeg data
    bufSize = MIN(bufSize,mallocSize);
    memcpy(mjpegData,buf,bufSize);
    mjpegDataSize = bufSize;
    mjpegIsDirty = false;

	// since we know that the jpeg dcompressor returns rgb we must convert to bgr for opencv purposes
    if(cinfo.output_components == 1 || cinfo.output_components == 2) {
    } else {
        gcvSwapRB(miplimgrgb,miplimg);
    }

    return GCV_OK;

}

extern "C" void jpeg_mem_dest (j_compress_ptr cinfo, gcvUchar *buf, gcvUlong bufSize);

int gcvImage::saveJpeg(gcvUchar *buf, gcvUlong bufSize, int quality, gcvUlong *cbufSize) {
	// unashamedly taken from example.c in the IJG libjpeg distribution
	struct jpeg_compress_struct cinfo;
	struct my_error_mgr jerr;
	JSAMPROW row_pointer[1];
	int row_stride;
	int image_width  = this->dimx();
	int image_height = this->dimy();
    int nofChannels       = this->nofChannels();

    JSAMPLE *image_buffer;
    if(nofChannels == 1 || nofChannels == 2) {
        image_buffer = (gcvUchar*)miplimg->imageData; // remmeber that here we point to the miplim data
    } else {
        // since we know that opencv images are bgr, we must convert to rgb before we can call the jpeg compressor
        gcvSwapRB(miplimg, miplimgrgb);
        image_buffer = (gcvUchar*)miplimgrgb->imageData; // remmeber that here we point to the miplimgrgb data
    }

	// We set up the normal JPEG error routines, then override error_exit.
	cinfo.err = jpeg_std_error(&jerr.pub);
	jerr.pub.error_exit = my_error_exit;
	if (setjmp(jerr.setjmp_buffer)) {
		jpeg_destroy_compress(&cinfo);
		return GCV_NOTOK;
	}

	jpeg_create_compress(&cinfo);

	jpeg_mem_dest(&cinfo, buf, bufSize);

	// First we supply a description of the input image.
	// Four fields of the cinfo struct must be filled in:
	cinfo.image_width  = image_width; 	//
	cinfo.image_height = image_height;
    
	if(nofChannels == 1 || nofChannels == 2) {
		// will only save the first channel
		cinfo.input_components = 1;		//
		cinfo.in_color_space = JCS_GRAYSCALE; 	// colorspace of input image
	} else {
		// will only save the first 3 channels
		cinfo.input_components = 3;		//
		cinfo.in_color_space = JCS_RGB; 	// colorspace of input image
	}

	jpeg_set_defaults(&cinfo);

    if(nofChannels == 1 || nofChannels == 2) {
        jpeg_set_colorspace(&cinfo, JCS_GRAYSCALE);
    } else {
        jpeg_set_colorspace(&cinfo, JCS_RGB);
    }


	// Now you can set any non-default parameters you wish to.
	// Here we just illustrate the use of quality (quantization table) scaling:
	//
	jpeg_set_quality(&cinfo, quality, TRUE );

	// TRUE ensures that we will write a complete interchange-JPEG file.
	// Pass TRUE unless you are very sure of what you're doing.
	jpeg_start_compress(&cinfo, TRUE);

	// Here we use the library's state variable cinfo.next_scanline as the
	// loop counter, so that we don't have to keep track ourselves.
	// To keep things simple, we pass one scanline per call; you can pass
	// more if you wish, though.
	//
	//row_stride = image_width * nofChannels;	// JSAMPLEs per row in image_buffer
	row_stride = this->widthStep();

	while (cinfo.next_scanline < cinfo.image_height) {
		// jpeg_write_scanlines expects an array of pointers to scanlines.
		// Here the array is only one element long, but you could pass
		// more than one scanline at a time if that's more convenient.
		row_pointer[0] = & image_buffer[cinfo.next_scanline * row_stride];
		(void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}

	// Step 6: Finish compression

	jpeg_finish_compress(&cinfo);


	// comptue the size of the compressed jpeg data
    *cbufSize = bufSize - cinfo.dest->free_in_buffer;

	// Step 7: release JPEG compression object

	// This is an important step since it will release a good deal of memory.
	jpeg_destroy_compress(&cinfo);



	return GCV_OK;
}

int gcvImage::loadJpeg(FILE *infile) {

  /* This struct contains the JPEG decompression parameters and pointers to
   * working space (which is allocated as needed by the JPEG library).
   */
  struct jpeg_decompress_struct cinfo;
  /* We use our private extension JPEG error handler.
   * Note that this struct must live as long as the main JPEG parameter
   * struct, to avoid dangling-pointer problems.
   */
  struct my_error_mgr jerr;
  /* More stuff */
  JSAMPARRAY buffer;		/* Output row buffer */
  int row_stride;		/* physical row width in output buffer */

  /* In this example we want to open the input file before doing anything else,
   * so that the setjmp() error recovery below can assume the file is open.
   * VERY IMPORTANT: use "b" option to fopen() if you are on a machine that
   * requires it in order to read binary files.
   */

  if (!infile) {
    printf("gcvImage::loadJpeg infile is null\n");
    return GCV_NOTOK;
  }

  /* Step 1: allocate and initialize JPEG decompression object */

  /* We set up the normal JPEG error routines, then override error_exit. */
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = my_error_exit;
  /* Establish the setjmp return context for my_error_exit to use. */
  if (setjmp(jerr.setjmp_buffer)) {
    /* If we get here, the JPEG code has signaled an error.
     * We need to clean up the JPEG object, close the input file, and return.
     */
    jpeg_destroy_decompress(&cinfo);
    return GCV_NOTOK;
  }
  /* Now we can initialize the JPEG decompression object. */
  jpeg_create_decompress(&cinfo);

  /* Step 2: specify data source (eg, a file) */

  jpeg_stdio_src(&cinfo, infile);

  /* Step 3: read file parameters with jpeg_read_header() */

  (void) jpeg_read_header(&cinfo, TRUE);
  /* We can ignore the return value from jpeg_read_header since
   *   (a) suspension is not possible with the stdio data source, and
   *   (b) we passed TRUE to reject a tables-only JPEG file as an error.
   * See libjpeg.doc for more info.
   */

  /* Step 4: set parameters for decompression */

  /* In this example, we don't need to change any of the defaults set by
   * jpeg_read_header(), so we do nothing here.
   */

  /* Step 5: Start decompressor */

  (void) jpeg_start_decompress(&cinfo);
  /* We can ignore the return value since suspension is not possible
   * with the stdio data source.
   */

  /* We may need to do some setup of our own at this point before reading
   * the data.  After jpeg_start_decompress() we have the correct scaled
   * output image dimensions available, as well as the output colormap
   * if we asked for color quantization.
   * In this example, we need to make an output work buffer of the right size.
   */
   this->resize(cinfo.output_width, cinfo.output_height, cinfo.output_components);
  /* JSAMPLEs per row in output buffer */
  //row_stride = cinfo.output_width * cinfo.output_components;
	row_stride = this->widthStep();
  /* Make a one-row-high sample array that will go away when done with image */
  buffer = (*cinfo.mem->alloc_sarray)
		((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);



  /* Step 6: while (scan lines remain to be read) */
  /*           jpeg_read_scanlines(...); */

  /* Here we use the library's state variable cinfo.output_scanline as the
   * loop counter, so that we don't have to keep track ourselves.
   */

  //gcvUchar *row[1] = {0};
  while (cinfo.output_scanline < cinfo.output_height) {
    /* jpeg_read_scanlines expects an array of pointers to scanlines.
     * Here the array is only one element long, but you could ask for
     * more than one scanline at a time if that's more convenient.
     */
	//  row[0] = mdata + (cinfo.output_scanline-1)*row_stride;
	 // (void) jpeg_read_scanlines(&cinfo, row, 1);
    (void) jpeg_read_scanlines(&cinfo, buffer, 1);
    if(cinfo.output_components == 1 || cinfo.output_components == 2) {
        memcpy(miplimg->imageData + (cinfo.output_scanline-1)*row_stride, buffer[0], row_stride);
    } else {
        memcpy(miplimgrgb->imageData + (cinfo.output_scanline-1)*row_stride, buffer[0], row_stride);
    }
  }

  /* Step 7: Finish decompression */

  (void) jpeg_finish_decompress(&cinfo);
  /* We can ignore the return value since suspension is not possible
   * with the stdio data source.
   */

  /* Step 8: Release JPEG decompression object */

  /* This is an important step since it will release a good deal of memory. */
  jpeg_destroy_decompress(&cinfo);

  /* At this point you may want to check to see whether any corrupt-data
   * warnings occurred (test whether jerr.pub.num_warnings is nonzero).
   */

  // since we know that the jpeg dcompressor returns rgb we must convert to bgr for opencv purposes
  if(cinfo.output_components == 1 || cinfo.output_components == 2) {
  } else {
      gcvSwapRB(miplimgrgb,miplimg);
  }

  /* And we're done! */
  return GCV_OK;

}
int gcvImage::loadJpeg(const char *filename) {

  FILE * infile;

  if ((infile = fopen(filename, "rb")) == NULL) {
    printf("gcvImage::loadJpeg can't open %s\n",filename);
    return GCV_NOTOK;
  }

  int ret = this->loadJpeg(infile);

  // no matter what, close the file
  fclose(infile);

  return ret;

}

int gcvImage::saveJpeg(FILE *outfile, int quality) {
	// unashamedly taken from example.c in the IJG libjpeg distribution
	struct jpeg_compress_struct cinfo;
	struct my_error_mgr jerr;
	JSAMPROW row_pointer[1];
	int row_stride;
	int image_width  = this->dimx();
	int image_height = this->dimy();
    int nofChannels       = this->nofChannels();

    JSAMPLE *image_buffer;
    if(nofChannels == 1 || nofChannels == 2) {
        image_buffer = (gcvUchar*)miplimg->imageData; // remmeber that here we point to the miplim data
    } else {
        // since we know that opencv images are bgr, we must convert to rgb before we can call the jpeg compressor
        gcvSwapRB(miplimg, miplimgrgb);
        image_buffer = (gcvUchar*)miplimgrgb->imageData; // remmeber that here we point to the miplimgrgb data
    }


	// We set up the normal JPEG error routines, then override error_exit.
	cinfo.err = jpeg_std_error(&jerr.pub);
	jerr.pub.error_exit = my_error_exit;
	if (setjmp(jerr.setjmp_buffer)) {
		jpeg_destroy_compress(&cinfo);
		return GCV_NOTOK;
	}

	jpeg_create_compress(&cinfo);

	if (!outfile) {
		printf("gcvImage::saveJpeg outfile is null\n");
		return GCV_NOTOK;
	}

	jpeg_stdio_dest(&cinfo, outfile);

	// First we supply a description of the input image.
	// Four fields of the cinfo struct must be filled in:
	cinfo.image_width  = image_width; 	//
	cinfo.image_height = image_height;
	if(nofChannels == 1 || nofChannels == 2) {
		// will only save the first channel
		cinfo.input_components = 1;		//
		cinfo.in_color_space = JCS_GRAYSCALE; 	// colorspace of input image
	} else {
		// will only save the first 3 channels
		cinfo.input_components = 3;		//
		cinfo.in_color_space = JCS_RGB; 	// colorspace of input image
	}

	jpeg_set_defaults(&cinfo);
    if(nofChannels == 1 || nofChannels == 2) {
        jpeg_set_colorspace(&cinfo, JCS_GRAYSCALE);
    } else {
        jpeg_set_colorspace(&cinfo, JCS_RGB);
    }

	// Now you can set any non-default parameters you wish to.
	// Here we just illustrate the use of quality (quantization table) scaling:
	//
	jpeg_set_quality(&cinfo, quality, TRUE );

	// TRUE ensures that we will write a complete interchange-JPEG file.
	// Pass TRUE unless you are very sure of what you're doing.
	jpeg_start_compress(&cinfo, TRUE);

	// Here we use the library's state variable cinfo.next_scanline as the
	// loop counter, so that we don't have to keep track ourselves.
	// To keep things simple, we pass one scanline per call; you can pass
	// more if you wish, though.
	//
	row_stride = image_width * nofChannels;	// JSAMPLEs per row in image_buffer

	while (cinfo.next_scanline < cinfo.image_height) {
		// jpeg_write_scanlines expects an array of pointers to scanlines.
		// Here the array is only one element long, but you could pass
		// more than one scanline at a time if that's more convenient.
		row_pointer[0] = & image_buffer[cinfo.next_scanline * row_stride];
		(void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}

	// Step 6: Finish compression

	jpeg_finish_compress(&cinfo);

	// Step 7: release JPEG compression object

	// This is an important step since it will release a good deal of memory.
	jpeg_destroy_compress(&cinfo);

	return GCV_OK;
}

int gcvImage::saveJpeg(const char *tfn, int quality) {
	FILE * outfile;

    if(mjpegIsDirty) {
        this->cacheJpeg(quality);
    }

	if ((outfile = fopen(tfn, "wb")) == NULL) {
		sulError(("can't open %s\n", tfn));
		return SUL_NOTOK;
	}

    fwrite(this->jpegData(),sizeof(gcvUchar),this->jpegDataSize(),outfile);
	//ret = this->saveJpeg(outfile, quality);

	// After finish_compress, we can close the output file.
	// no matter what we close the file
	fclose(outfile);

	return SUL_OK;

}

#endif



int gcvSwapRB(IplImage *src, IplImage *dst) {
    int i,j;
    //char *srcptr;
    //char *dstptr;

    if(src->width == dst->width && src->height == dst->height && src->nChannels == 3 && dst->nChannels == 3) {
        for (j=0;j<src->height;j++) {
            //srcptr = src->imageData + j*src->widthStep;
            //dstptr = dst->imageData + j*src->widthStep;
            for(i=0; i<src->width; i++) {
                IPLPIXIJ(dst,i,j,0) = IPLPIXIJ(src,i,j,2);
                IPLPIXIJ(dst,i,j,1) = IPLPIXIJ(src,i,j,1);
                IPLPIXIJ(dst,i,j,2) = IPLPIXIJ(src,i,j,0);
                //srcptr += i*src->nChannels;
                //dstptr += i*src->nChannels;
                //dstptr[0] = srcptr[2];
                //dstptr[1] = srcptr[1];
                //dstptr[2] = srcptr[0];
            }
        }
        return GCV_OK;
    } else {
        return GCV_NOTOK;
    }
}



#endif // HAVE_OPENCV
