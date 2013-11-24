#include "../../Common/RobotDefs.h"

#if defined(IS_WIN)

#include "CV.h"

static int CV_XADD( int* addr, int delta )
{
	int tmp;
	__asm
	{
		mov edx, addr
		mov eax, delta
		lock xadd [edx], eax
		mov tmp, eax
	}
	return tmp;
}


///////////////////////////////////////////

int Mat_type(const Mat m) { return CV_MAT_TYPE(m.flags); }

size_t Mat_elemSize(Mat m){ return CV_ELEM_SIZE(m.flags); }

int Mat_depth(Mat m)  { return CV_MAT_DEPTH(m.flags); }

int Mat_channels(Mat m) { return CV_MAT_CN(m.flags); }

int Mat_isContinuous(Mat m) { return (m.flags & CV_MAT_CONT_FLAG) != 0; }

Mat initMat()
{
	Mat m;
	m.flags = 0;
	m.rows = 0;
	m.cols = 0;
	m.step = 0;
	m.data = 0;
	m.refcount = 0;
	m.datastart = 0;
	m.dataend = 0;
	return m;
}

void Mat_release (Mat *mat)
{
	if( mat->refcount && CV_XADD(mat->refcount, -1) == 1 )
		fastFree(mat->datastart);
	mat->data = mat->datastart = mat->dataend = 0;
	mat->step = mat->rows = mat->cols = 0;
	mat->refcount = 0;
}


void clearMat(Mat *mat)
{
	Mat_release(mat);
}




static size_t alignSize(size_t sz, int n)
{
	return (sz + n-1) & -n;
}

void Mat_create(Mat *mat, int _rows, int _cols, int _type)
{
	__int64 _nettosize;
	size_t nettosize;
	size_t datasize;

	_type &= TYPE_MASK;

	if( mat->rows == _rows && mat->cols == _cols && /*type() == _type &&*/ mat->data )
		return;

	if( mat->data )
		Mat_release(mat);

	assert( _rows >= 0 && _cols >= 0 );

	if( _rows > 0 && _cols > 0 )
	{
		mat->flags = MAGIC_VAL + CV_MAT_CONT_FLAG + _type;
		mat->rows = _rows;
		mat->cols = _cols;
		mat->step = Mat_elemSize(*mat) * mat->cols;

		_nettosize = (__int64)mat->step * mat->rows;
		nettosize = (size_t)_nettosize;

		if( _nettosize != (__int64)nettosize )
			assert (0);

		datasize = alignSize(nettosize, (int)sizeof (*mat->refcount));

		mat->datastart = mat->data = (uchar*)fastMalloc (datasize + sizeof(*mat->refcount));
		mat->dataend = mat->data + nettosize;
		mat->refcount = (int*)(mat->data + datasize);
		*(mat->refcount) = 1;
	}
}

void Mat_copyTo( const Mat src, Mat* dst )
{
	CvSize sz;
	const uchar* sptr;
	uchar* dptr;

	if( src.data == dst->data )
		return;

	Mat_create (dst, src.rows, src.cols, Mat_type(*dst));
	//dst.create( rows, cols, type() );

	sz.height = src.rows;
	sz.width = src.cols;

	sptr = src.data;
	dptr = dst->data;

	sz.width *= (int)Mat_elemSize(src);

	if( Mat_isContinuous(src) && Mat_isContinuous(*dst) )
	{
		sz.width *= sz.height;
		sz.height = 1;
	}

	for( ; sz.height--; sptr += src.step, dptr += dst->step )
		memcpy( dptr, sptr, sz.width );
}

Mat initMat_params(int _rows, int _cols, int _type, void* _data, size_t _step)
{
	size_t minstep;
	Mat mat;
	mat.flags = MAGIC_VAL + (_type & TYPE_MASK);
	mat.rows = _rows;
	mat.cols = _cols;
	mat.step = _step;
	mat.data = (uchar*)_data;
	mat.refcount = 0;
	mat.datastart = (uchar*)_data;
	mat.dataend = (uchar*)_data;

	minstep = mat.cols * Mat_elemSize(mat);
	if( mat.step == AUTO_STEP )
	{
		mat.step = minstep;
		mat.flags |= CV_MAT_CONT_FLAG;
	}
	else
	{
		if( mat.rows == 1 )
			mat.step = minstep;
		assert( mat.step >= minstep );
		mat.flags |= (mat.step == minstep ? CV_MAT_CONT_FLAG : 0);
	}
	mat.dataend += mat.step*(mat.rows-1) + minstep;
	return mat;
}

Mat initMat_cvMat(const CvMat* m)
{
	size_t minstep;
	Mat mat;
	mat.flags = MAGIC_VAL + (m->type & (CV_MAT_TYPE_MASK|CV_MAT_CONT_FLAG));
	mat.rows = m->rows;
	mat.cols = m->cols;
	mat.step = m->step;
	mat.data = m->data.ptr;
	mat.refcount = 0;
	mat.datastart = m->data.ptr;
	mat.dataend = m->data.ptr;

	minstep = mat.cols * Mat_elemSize(mat);
	if( mat.step == 0 )
		mat.step = minstep;

	mat.dataend += mat.step*(mat.rows-1) + minstep;

	return mat;
}


///////////////////////////////////////////////////////////////////////////////////////////////////





#define CV_MAGIC_MASK       0xFFFF0000

#define CV_IS_MAT_HDR(mat) \
	((mat) != NULL && \
	(((const CvMat*)(mat))->type & CV_MAGIC_MASK) == CV_MAT_MAGIC_VAL && \
	((const CvMat*)(mat))->cols > 0 && ((const CvMat*)(mat))->rows > 0)

#define CV_IS_MAT(mat) \
	(CV_IS_MAT_HDR(mat) && ((const CvMat*)(mat))->data.ptr != NULL)

static Mat cvarrToMat(const CvArr *arr)
{
	if( !CV_IS_MAT(arr) )
		assert (0);
	return initMat_cvMat((const CvMat*)arr );

}

void
cvCopy( const void* srcarr, void* dstarr)
{
	Mat src = cvarrToMat((IplImage*)srcarr);
	Mat dst = cvarrToMat((IplImage*)dstarr);
	assert( Mat_depth(src) == Mat_depth(dst) && src.rows == dst.rows && src.cols == dst.cols );

	assert( Mat_channels(src) == Mat_channels(dst) );

	Mat_copyTo (src, &dst);
}

CvMat*
cvInitMatHeader( CvMat* arr, int rows, int cols,
				int type, void* data, int step )
{
	int pix_size;
	int min_step;

	type = CV_MAT_TYPE( type );
	arr->type = type | CV_MAT_MAGIC_VAL;
	arr->rows = rows;
	arr->cols = cols;
	arr->data.ptr = (uchar*)data;
	arr->refcount = 0;
	arr->hdr_refcount = 0;

	pix_size = CV_ELEM_SIZE(type);
	min_step = arr->cols*pix_size;

	if( step != CV_AUTOSTEP && step != 0 )
	{
		if( step < min_step )
			assert (0);
		arr->step = step;
	}
	else
	{
		arr->step = min_step;
	}

	arr->type = CV_MAT_MAGIC_VAL | type |
		(arr->rows == 1 || arr->step == min_step ? CV_MAT_CONT_FLAG : 0);

	return arr;
}

CvMat*
cvGetMat( const CvArr* array, CvMat* mat )
{
	CvMat* result = 0;
	CvMat* src = (CvMat*)array;

	{
		const IplImage* img = (const IplImage*)src;
		int depth, order;

		depth = IPL2CV_DEPTH( img->depth );
		if( depth < 0 )
			assert (0);

		order = img->dataOrder & (img->nChannels > 1 ? -1 : 0);

		{
			int type = CV_MAKETYPE( depth, img->nChannels );

			if( order != IPL_DATA_ORDER_PIXEL )
				assert (0);

			cvInitMatHeader( mat, img->height, img->width, type,
				img->imageData, img->widthStep );
		}
		result = mat;
	}

	return result;
}

#define CV_MAT_CONT_FLAG_SHIFT  14
#define CV_MAT_CONT_FLAG        (1 << CV_MAT_CONT_FLAG_SHIFT)
#define CV_IS_MAT_CONT(flags)   ((flags) & CV_MAT_CONT_FLAG)

#define  CV_STUB_STEP     (1 << 30)

void icvCvt_Gray2BGR_8u_C1C3R( const uchar* gray, int gray_step,
							  uchar* bgr, int bgr_step, CvSize size )
{
	int i;
	for( ; size.height--; gray += gray_step )
	{
		for( i = 0; i < size.width; i++, bgr += 3 )
		{
			bgr[0] = bgr[1] = bgr[2] = gray[i];
		}
		bgr += bgr_step - size.width*3;
	}
}

void
cvConvertImage( const CvMat* src, CvMat* dst )
{
	int src_cn, dst_cn;

	uchar *s, *d;
	int s_step, d_step;
	int code;
	CvSize size = { src->cols, src->rows };

	src_cn = CV_MAT_CN( src->type );
	dst_cn = CV_MAT_CN( dst->type );

	if( src_cn != 1 && src_cn != 3 && src_cn != 4 )
		assert (0);

	if( CV_MAT_DEPTH( dst->type ) != CV_8U )
		assert (0);

	if( CV_MAT_CN(dst->type) != 1 && CV_MAT_CN(dst->type) != 3 )
		assert (0);

	if( !CV_ARE_DEPTHS_EQ( src, dst ))
	{
		assert (0);
	}

	if( src_cn != dst_cn )
	{
		s = src->data.ptr;
		d = dst->data.ptr;
		s_step = src->step;
		d_step = dst->step;
		code = src_cn*10 + dst_cn;

		if( CV_IS_MAT_CONT(src->type & dst->type) )
		{
			size.width *= size.height;
			size.height = 1;
			s_step = d_step = CV_STUB_STEP;
		}

		switch( code )
		{
		case 13:
			icvCvt_Gray2BGR_8u_C1C3R( s, s_step, d, d_step, size );
			break;
			assert (0);
		}
		src = dst;
	}

	if( src != dst )
	{
		cvCopy( src, dst );
	}
}

void FillBitmapInfo( BITMAPINFO* bmi, int width, int height, int bpp, int origin )
{
	BITMAPINFOHEADER* bmih;

	assert( bmi && width >= 0 && height >= 0 && (bpp == 8 || bpp == 24 || bpp == 32));

	bmih = &(bmi->bmiHeader);

	memset( bmih, 0, sizeof(*bmih));
	bmih->biSize = sizeof(BITMAPINFOHEADER);
	bmih->biWidth = width;
	bmih->biHeight = origin ? abs(height) : -abs(height);
	bmih->biPlanes = 1;
	bmih->biBitCount = (unsigned short)bpp;
	bmih->biCompression = BI_RGB;

	if( bpp == 8 )
	{
		RGBQUAD* palette = bmi->bmiColors;
		int i;
		for( i = 0; i < 256; i++ )
		{
			palette[i].rgbBlue = palette[i].rgbGreen = palette[i].rgbRed = (BYTE)i;
			palette[i].rgbReserved = 0;
		}
	}
}

void cvShowImage( const char* name, const CvArr* arr )
{
	CvWindow* window;
	SIZE size = { 0, 0 };
	int channels = 0;
	void* dst_ptr = 0;
	const int channels0 = 3;
	int origin = 0;
	CvMat stub, dst, *image;
	int changed_size = 0; // philipg

	uchar buffer[sizeof(BITMAPINFO) + 255*sizeof(RGBQUAD)];
	BITMAPINFO* binfo = (BITMAPINFO*)buffer;

	window = icvFindWindowByName(name);
	assert (window);
	assert (arr);

	origin = ((IplImage*)arr)->origin;

	image = cvGetMat( arr, &stub );

	if (window->image)
		// if there is something wrong with these system calls, we cannot display image...
		if (icvGetBitmapData( window, &size, &channels, &dst_ptr ))
			return;

	if( size.cx != image->width || size.cy != image->height || channels != channels0 )
	{
		changed_size = 1;

		DeleteObject( SelectObject( window->dc, window->image ));
		window->image = 0;

		size.cx = image->width;
		size.cy = image->height;
		channels = channels0;

		FillBitmapInfo( binfo, size.cx, size.cy, channels*8, 1 );

		window->image = SelectObject( window->dc, CreateDIBSection(window->dc, binfo,
			DIB_RGB_COLORS, &dst_ptr, 0, 0));
	}

	cvInitMatHeader( &dst, size.cy, size.cx, CV_8UC3,
		dst_ptr, (size.cx * channels + 3) & -4 );
	cvConvertImage( image, &dst  );

	// ony resize window if needed
	if (changed_size)
		icvUpdateWindowPos(window);
	InvalidateRect(window->hwnd, 0, 0);
}

void cvNamedWindow( const char* name, const int windowSize[4] )
{
	HWND hWnd, mainhWnd;
	CvWindow* window;
	DWORD defStyle = WS_VISIBLE | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU;
	int len;
	CvRect rect;

	cvInitSystem();

	// Check the name in the storage
	if( icvFindWindowByName( name ) != 0 )
	{
		return;
	}

	rect.x = windowSize[0];
	rect.y = windowSize[1];
	rect.width = windowSize[2];
	rect.height = windowSize[3];


	mainhWnd = createMainWin ("Main HighGUI class", name, rect);

	if( !mainhWnd )
		assert (0);

	ShowWindow(mainhWnd, SW_SHOW);

	//YV- remove one border by changing the style
	hWnd = createWin ("HighGUI class", "", rect, mainhWnd);

	if( !hWnd )
		assert (0);

	ShowWindow(hWnd, SW_SHOW);

	len = (int)strlen(name);
	window = (CvWindow*)fastMalloc(sizeof(CvWindow) + len + 1);

	window->signature = CV_WINDOW_MAGIC_VAL;
	window->hwnd = hWnd;
	window->frame = mainhWnd;
	window->name = (char*)(window + 1);
	memcpy( window->name, name, len + 1 );
	window->flags = CV_WINDOW_AUTOSIZE;
	window->image = 0;
	window->dc = CreateCompatibleDC(0);
	window->last_key = 0;
	window->status = CV_WINDOW_NORMAL;//YV

	window->on_mouse = 0;
	window->on_mouse_param = 0;


	updateWindows (window);

	icvSetWindowLongPtr( hWnd, CV_USERDATA, window );
	icvSetWindowLongPtr( mainhWnd, CV_USERDATA, window );

	// Recalculate window position
	icvUpdateWindowPos( window );
}

void cvMoveWindow( const char* name, const int x, const int y )
{
	CvWindow* window;
	RECT rect;

	window = icvFindWindowByName(name);
	if(!window)
		return;

	GetWindowRect( window->frame, &rect );
	MoveWindow( window->frame, x, y, rect.right - rect.left, rect.bottom - rect.top, TRUE);
}

#endif // defined(IS_WIN)
