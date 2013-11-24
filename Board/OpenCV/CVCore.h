#include "../../Common/RobotDefs.h"

#if defined(IS_WIN)

#ifndef CV_CORE_H
#define CV_CORE_H


typedef struct _IplROI
{
	int  coi; /* 0 - no COI (all channels are selected), 1 - 0th channel is selected ...*/
	int  xOffset;
	int  yOffset;
	int  width;
	int  height;
}
IplROI;

typedef struct _IplImage
{
	int  nSize;             /* sizeof(IplImage) */
	int  nChannels;         /* Most of OpenCV functions support 1,2,3 or 4 channels */
	int  alphaChannel;      /* Ignored by OpenCV */
	int  depth;             /* Pixel depth in bits: IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
							IPL_DEPTH_32S, IPL_DEPTH_32F and IPL_DEPTH_64F are supported.  */
	int  dataOrder;         /* 0 - interleaved color channels, 1 - separate color channels.
							cvCreateImage can only create interleaved images */
	int  origin;            /* 0 - top-left origin,
							1 - bottom-left origin (Windows bitmaps style).  */
	int  align;             /* Alignment of image rows (4 or 8).
							OpenCV ignores it and uses widthStep instead.    */
	int  width;             /* Image width in pixels.                           */
	int  height;            /* Image height in pixels.                          */
	struct _IplROI *roi;    /* Image ROI. If NULL, the whole image is selected. */
	int  imageSize;         /* Image data size in bytes
							(==image->height*image->widthStep
							in case of interleaved data)*/
	char *imageData;        /* Pointer to aligned image data.         */
	int  widthStep;         /* Size of aligned image row in bytes.    */
	char *imageDataOrigin;  /* Pointer to very origin of image data
							(not necessarily aligned) -
							needed for correct deallocation */
}
IplImage;

typedef struct _IplTileInfo IplTileInfo;

typedef struct
{
	int width;
	int height;
}
CvSize;


////////////////////////////////////////

#define  CV_MALLOC_ALIGN    16

//void* cvAlignPtr( const void* ptr, int align /*CV_DEFAULT(32)*/ );


void* fastMalloc( size_t size );


void fastFree(void* ptr);

#define cvFree(ptr) (fastFree(*(ptr)), *(ptr)=0)


///////////////////////

#define IPL_DEPTH_SIGN 0x80000000


// Allocates underlying array data
void
cvCreateData( IplImage* img );

// initalize IplImage header, allocated by the user
IplImage*
cvInitImageHeader( IplImage * image, CvSize size, int depth,
				  int channels, int origin, int align );

#define IPL_ORIGIN_BL 1
#define  CV_DEFAULT_IMAGE_ROW_ALIGN  4

// create IplImage header
IplImage* cvCreateImageHeader( CvSize size, int depth, int channels );

IplImage* cvCreateImage( CvSize size, int depth, int channels );
/////////////////////////////


// Deallocates array's data
void
cvReleaseData( IplImage* img );



void
cvReleaseImageHeader( IplImage** image );

void cvReleaseImage( IplImage ** image );

//////////////////////////////

typedef void (__cdecl *CvMouseCallback )(int event, int x, int y, int flags, void* param);


typedef struct CvWindow *CvWindowPtr;

typedef struct CvWindow
{
	int signature;
	HWND hwnd;
	char* name;
	CvWindowPtr prev;
	CvWindowPtr next;
	HWND frame;

	HDC dc;
	HGDIOBJ image;
	int last_key;
	int flags;
	int status;//0 normal, 1 fullscreen (YV)

	CvMouseCallback on_mouse;
	void* on_mouse_param;
}
CvWindow;

typedef struct CvRect
{
	int x;
	int y;
	int width;
	int height;
}
CvRect;

typedef int (__cdecl * CvWin32WindowCallback)(HWND, UINT, WPARAM, LPARAM, int*);



void cvCleanup();

// returns TRUE if there is a problem such as ERROR_IO_PENDING.
int icvGetBitmapData( CvWindow* window, SIZE* size, int* channels, void** data );



#define icvGetWindowLongPtr GetWindowLong
#define CV_USERDATA GWL_USERDATA
#define CV_WINDOW_MAGIC_VAL     0x00420042


#define icvSetWindowLongPtr( hwnd, id, ptr ) SetWindowLong( hwnd, id, (size_t)ptr )

#define icvGetClassLongPtr GetClassLong
#define CV_WINDOW_AUTOSIZE  1
#define CV_HCURSOR GCL_HCURSOR

#define CV_EVENT_MOUSEMOVE      0
#define CV_EVENT_LBUTTONDOWN    1
#define CV_EVENT_RBUTTONDOWN    2
#define CV_EVENT_MBUTTONDOWN    3
#define CV_EVENT_LBUTTONUP      4
#define CV_EVENT_RBUTTONUP      5
#define CV_EVENT_MBUTTONUP      6
#define CV_EVENT_LBUTTONDBLCLK  7
#define CV_EVENT_RBUTTONDBLCLK  8
#define CV_EVENT_MBUTTONDBLCLK  9

#define CV_EVENT_FLAG_LBUTTON   1
#define CV_EVENT_FLAG_RBUTTON   2
#define CV_EVENT_FLAG_MBUTTON   4
#define CV_EVENT_FLAG_CTRLKEY   8
#define CV_EVENT_FLAG_SHIFTKEY  16
#define CV_EVENT_FLAG_ALTKEY    32

void icvUpdateWindowPos( CvWindow* window );

#define CV_HBRBACKGROUND GCL_HBRBACKGROUND


int cvInitSystem( );

#define CV_WINDOW_NORMAL	 	 0



//////////////////////


typedef void CvArr;


typedef struct CvMat
{
	int type;
	int step;

	/* for internal use only */
	int* refcount;
	int hdr_refcount;

	union
	{
		uchar* ptr;
		short* s;
		int* i;
		float* fl;
		double* db;
	} data;

	//#ifdef __cplusplus
#if 1
	union
	{
		int rows;
		int height;
	};

	union
	{
		int cols;
		int width;
	};
#else
	int rows;
	int cols;
#endif
}
CvMat;

#define CV_MAT_MAGIC_VAL    0x42420000

#define CV_CN_MAX     64
#define CV_CN_SHIFT   3
#define CV_DEPTH_MAX  (1 << CV_CN_SHIFT)
#define CV_MAT_CN_MASK          ((CV_CN_MAX - 1) << CV_CN_SHIFT)
#define CV_MAT_CN(flags)        ((((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)
#define CV_MAT_TYPE_MASK        (CV_DEPTH_MAX*CV_CN_MAX - 1)
#define CV_MAT_TYPE(flags)      ((flags) & CV_MAT_TYPE_MASK)

#define CV_MAT_DEPTH_MASK       (CV_DEPTH_MAX - 1)
#define CV_MAT_DEPTH(flags)     ((flags) & CV_MAT_DEPTH_MASK)


/* 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
#define CV_ELEM_SIZE(type) \
	(CV_MAT_CN(type) << ((((sizeof(size_t)/4+1)*16384|0x3a50) >> CV_MAT_DEPTH(type)*2) & 3))

#define CV_AUTOSTEP  0x7fffffff

#define CV_MAT_CONT_FLAG_SHIFT  14
#define CV_MAT_CONT_FLAG        (1 << CV_MAT_CONT_FLAG_SHIFT)

#define INT_MAX       2147483647    /* maximum (signed) int value */

enum { MAGIC_MASK=0xFFFF0000, TYPE_MASK=0x00000FFF, DEPTH_MASK=7 };



#define CV_ARE_DEPTHS_EQ(mat1, mat2) \
	((((mat1)->type ^ (mat2)->type) & CV_MAT_DEPTH_MASK) == 0)

#define CV_8U   0
#define CV_8S   1
#define CV_16U  2
#define CV_16S  3
#define CV_32S  4
#define CV_32F  5
#define CV_64F  6

#define CV_ARE_CNS_EQ(mat1, mat2) \
	((((mat1)->type ^ (mat2)->type) & CV_MAT_CN_MASK) == 0)

#define CV_CVTIMG_FLIP      1
#define CV_MAKETYPE(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))

#define IPL_DEPTH_SIGN 0x80000000
#define IPL2CV_DEPTH(depth) \
	((((CV_8U)+(CV_16U<<4)+(CV_32F<<8)+(CV_64F<<16)+(CV_8S<<20)+ \
	(CV_16S<<24)+(CV_32S<<28)) >> ((((depth) & 0xF0) >> 2) + \
	(((depth) & IPL_DEPTH_SIGN) ? 20 : 0))) & 15)

#define IPL_DATA_ORDER_PIXEL  0
#define IPL_DATA_ORDER_PLANE  1

void updateWindows (CvWindow *window);

CvWindow* icvFindWindowByName( const char* name );

HWND createMainWin (const char *mainClass, const char *name, CvRect rect);


HWND createWin (const char *className, const char *name, CvRect rect, HWND mainhWnd);


int cvWaitKey( int delay );




#endif // CV_CORE_H
#endif // defined(IS_WIN)
