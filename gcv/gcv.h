#ifndef GCV_GCV_H
#define GCV_GCV_H

//#include "gcv/gcvconfig.h"
typedef unsigned char gcvUchar;
typedef unsigned int  gcvUint;
typedef unsigned long gcvUlong;
typedef long          gcvPixelIdx;
#include "esim/util.h"
//#include "gcv/gcv_ref.h"
// some C functions
#if defined(__cplusplus)
extern "C"
{
#endif

#define GCV_OK     0
#define GCV_NOTOK -1
#define GCV_FALSE  0
#define GCV_TRUE   1

void gcvUpdate(struct gcv_ref *g, double time);
void initGcv(struct gcv_ref *g);
void gcvInit();

#if defined(__cplusplus)
}
#endif

#if defined(__cplusplus)
/// pre-declerations of structs and classes
class gcvImage;
class gcvImageWindow;
class gcvPipe;

#if HAVE_FLTK2
class gcvImageWindow;
#include "fltk/Widget.h"
#include "fltk/draw.h"
#include <fltk/events.h>
#endif //HAVE_FLTK2



#endif //(__cplusplus)

#endif // GCV_GCV_H


