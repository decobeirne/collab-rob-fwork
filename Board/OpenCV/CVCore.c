#include "../../Common/RobotDefs.h"

#if defined(IS_WIN)

#include "CVCore.h"


static const char* highGUIclassName = "HighGUI class";
static const char* mainHighGUIclassName = "Main HighGUI class";
static HINSTANCE hg_hinstance = 0;
static CvWindow* hg_windows = 0;
static CvWin32WindowCallback hg_on_preprocess = 0, hg_on_postprocess = 0;





//void* cvAlignPtr( const void* ptr, int align /*CV_DEFAULT(32)*/ )
//{
//    assert( (align & (align-1)) == 0 );
//    return (void*)( ((size_t)ptr + align - 1) & ~(size_t)(align-1) );
//}

static uchar** alignPtr(uchar** ptr, int n)
{
	return (uchar**)(((size_t)ptr + n-1) & -n);
}

void* fastMalloc( size_t size )
{
	uchar** adata;
	uchar* udata = (uchar*)malloc(size + sizeof(void*) + CV_MALLOC_ALIGN);
	if(!udata)
		assert (0);
	adata = alignPtr((uchar**)udata + 1, CV_MALLOC_ALIGN);
	adata[-1] = udata;
	return adata;
}



void fastFree(void* ptr)
{
	if(ptr)
	{
		uchar* udata = ((uchar**)ptr)[-1];
		assert (udata < (uchar*)ptr &&
			((uchar*)ptr - udata) <= (ptrdiff_t)(sizeof(void*)+CV_MALLOC_ALIGN)); 
		free(udata);
	}
}



// Allocates underlying array data
void cvCreateData( IplImage* img )
{
	{
		if( img->imageData != 0 )
			assert (0);

		{
			img->imageData = img->imageDataOrigin = 
				(char*)fastMalloc( (size_t)img->imageSize );
		}
	}
}

// initalize IplImage header, allocated by the user
IplImage*
cvInitImageHeader( IplImage * image, CvSize size, int depth,
				  int channels, int origin, int align )
{
	memset( image, 0, sizeof( *image ));
	image->nSize = sizeof( *image );

	image->width = size.width;
	image->height = size.height;

	if( image->roi )
	{
		image->roi->coi = 0;
		image->roi->xOffset = image->roi->yOffset = 0;
		image->roi->width = size.width;
		image->roi->height = size.height;
	}

	image->nChannels = max( channels, 1 );
	image->depth = depth;
	image->align = align;
	image->widthStep = (((image->width * image->nChannels *
		(image->depth & ~IPL_DEPTH_SIGN) + 7)/8)+ align - 1) & (~(align - 1));
	image->origin = origin;
	image->imageSize = image->widthStep * image->height;

	return image;
}


// create IplImage header
IplImage *
cvCreateImageHeader( CvSize size, int depth, int channels )
{
	IplImage *img = 0;

	{
		img = (IplImage *)fastMalloc( sizeof( *img ));
		assert( img );
		cvInitImageHeader( img, size, depth, channels, IPL_ORIGIN_BL,
			CV_DEFAULT_IMAGE_ROW_ALIGN );
	}

	return img;
}

IplImage* cvCreateImage( CvSize size, int depth, int channels )
{
	IplImage *img = cvCreateImageHeader( size, depth, channels );
	assert( img );
	cvCreateData( img );

	return img;
}

/////////////////////////////


// Deallocates array's data
void
cvReleaseData( IplImage* img )
{
	{
		{
			char* ptr = img->imageDataOrigin;
			img->imageData = img->imageDataOrigin = 0;
			cvFree( &ptr );

		}
	}
}



void
cvReleaseImageHeader( IplImage** image )
{
	{
		IplImage* img = *image;
		*image = 0;

		{
			cvFree( &img->roi );
			cvFree( &img );
		}
	}
}

void cvReleaseImage( IplImage ** image )
{
	{
		IplImage* img = *image;
		*image = 0;

		cvReleaseData( img );
		cvReleaseImageHeader( &img );
	}
}

//////////////////////////////

static void
cvDestroyAllWindows(void)
{
	HWND mainhWnd;
	HWND hwnd;
	CvWindow* window = hg_windows;

	while( window )
	{
		mainhWnd = window->frame;
		hwnd = window->hwnd;
		window = window->next;

		SendMessage( hwnd, WM_CLOSE, 0, 0 );
		SendMessage( mainhWnd, WM_CLOSE, 0, 0 );
	}
}





static void icvCleanupHighgui(void)
{
	cvDestroyAllWindows();
	UnregisterClass(highGUIclassName, hg_hinstance);
	UnregisterClass(mainHighGUIclassName, hg_hinstance);
}

void cvCleanup()
{
	icvCleanupHighgui();
}

// returns TRUE if there is a problem such as ERROR_IO_PENDING.
int icvGetBitmapData( CvWindow* window, SIZE* size, int* channels, void** data )
{
	BITMAP bmp;
	HGDIOBJ h;

	GdiFlush();

	h = GetCurrentObject( window->dc, OBJ_BITMAP );

	if( size )
		size->cx = size->cy = 0;
	if( data )
		*data = 0;

	if (h == NULL)
		return 1;

	if (GetObject(h, sizeof(bmp), &bmp) == 0)
		return 1;

	if( size )
	{
		size->cx = abs(bmp.bmWidth);
		size->cy = abs(bmp.bmHeight);
	}

	if( channels )
		*channels = bmp.bmBitsPixel/8;

	if( data )
		*data = bmp.bmBits;

	return 0;
}

static void icvScreenToClient( HWND hwnd, RECT* rect )
{
	POINT p;
	p.x = rect->left;
	p.y = rect->top;

	ScreenToClient(hwnd, &p);
	OffsetRect( rect, p.x - rect->left, p.y - rect->top );
}

/* Calculatess the window coordinates relative to the upper left corner of the mainhWnd window */
static RECT icvCalcWindowRect( CvWindow* window )
{
	const int gutter = 1;
	RECT crect, rect;

	assert(window);

	GetClientRect(window->frame, &crect);
	rect = crect;

	rect.top += gutter;
	rect.left += gutter;
	rect.bottom -= gutter;
	rect.right -= gutter;

	return rect;
}

static void icvRemoveWindow( CvWindow* window )
{
	RECT wrect={0,0,0,0};

	if( window->frame )
		GetWindowRect( window->frame, &wrect );

	if( window->hwnd )
		icvSetWindowLongPtr( window->hwnd, CV_USERDATA, 0 );
	if( window->frame )
		icvSetWindowLongPtr( window->frame, CV_USERDATA, 0 );

	if( window->prev )
		window->prev->next = window->next;
	else
		hg_windows = window->next;

	if( window->next )
		window->next->prev = window->prev;

	window->prev = window->next = 0;

	if( window->dc && window->image )
		DeleteObject(SelectObject(window->dc,window->image));

	if( window->dc )
		DeleteDC(window->dc);

	cvFree( &window );
}

static CvWindow* icvWindowByHWND( HWND hwnd )
{
	CvWindow* window = (CvWindow*)icvGetWindowLongPtr( hwnd, CV_USERDATA );
	return window != 0 && hg_windows != 0 &&
		window->signature == CV_WINDOW_MAGIC_VAL ? window : 0;
}

static LRESULT CALLBACK HighGUIProc( HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	LPWINDOWPOS pos;
	int i;
	RECT rect;
	CvWindow* window = icvWindowByHWND(hwnd);
	int nchannels = 3;
	SIZE size = {0,0};
	PAINTSTRUCT paint;
	HDC hdc;
	RGBQUAD table[256];

	if( !window )
		// This window is not mentioned in HighGUI storage
		// Actually, this should be error except for the case of calls to CreateWindow
		return DefWindowProc(hwnd, uMsg, wParam, lParam);

	// Process the message
	switch(uMsg)
	{
	case WM_WINDOWPOSCHANGING:
		{
			pos = (LPWINDOWPOS)lParam;
			rect = icvCalcWindowRect(window);
			pos->x = rect.left;
			pos->y = rect.top;
			pos->cx = rect.right - rect.left + 1;
			pos->cy = rect.bottom - rect.top + 1;
		}
		break;

		//case WM_LBUTTONDOWN:
		//case WM_RBUTTONDOWN:
		//case WM_MBUTTONDOWN:
		//case WM_LBUTTONDBLCLK:
		//case WM_RBUTTONDBLCLK:
		//case WM_MBUTTONDBLCLK:
		//case WM_LBUTTONUP:
		//case WM_RBUTTONUP:
		//case WM_MBUTTONUP:
		//case WM_MOUSEMOVE:
		//    if( window->on_mouse )
		//    {
		//        POINT pt;
		//        RECT rect;
		//        SIZE size = {0,0};

		//        int flags = (wParam & MK_LBUTTON ? CV_EVENT_FLAG_LBUTTON : 0)|
		//                    (wParam & MK_RBUTTON ? CV_EVENT_FLAG_RBUTTON : 0)|
		//                    (wParam & MK_MBUTTON ? CV_EVENT_FLAG_MBUTTON : 0)|
		//                    (wParam & MK_CONTROL ? CV_EVENT_FLAG_CTRLKEY : 0)|
		//                    (wParam & MK_SHIFT ? CV_EVENT_FLAG_SHIFTKEY : 0)|
		//                    (GetKeyState(VK_MENU) < 0 ? CV_EVENT_FLAG_ALTKEY : 0);
		//        int event = uMsg == WM_LBUTTONDOWN ? CV_EVENT_LBUTTONDOWN :
		//                    uMsg == WM_RBUTTONDOWN ? CV_EVENT_RBUTTONDOWN :
		//                    uMsg == WM_MBUTTONDOWN ? CV_EVENT_MBUTTONDOWN :
		//                    uMsg == WM_LBUTTONUP ? CV_EVENT_LBUTTONUP :
		//                    uMsg == WM_RBUTTONUP ? CV_EVENT_RBUTTONUP :
		//                    uMsg == WM_MBUTTONUP ? CV_EVENT_MBUTTONUP :
		//                    uMsg == WM_LBUTTONDBLCLK ? CV_EVENT_LBUTTONDBLCLK :
		//                    uMsg == WM_RBUTTONDBLCLK ? CV_EVENT_RBUTTONDBLCLK :
		//                    uMsg == WM_MBUTTONDBLCLK ? CV_EVENT_MBUTTONDBLCLK :
		//                                               CV_EVENT_MOUSEMOVE;
		//        if( uMsg == WM_LBUTTONDOWN || uMsg == WM_RBUTTONDOWN || uMsg == WM_MBUTTONDOWN )
		//            SetCapture( hwnd );
		//        if( uMsg == WM_LBUTTONUP || uMsg == WM_RBUTTONUP || uMsg == WM_MBUTTONUP )
		//            ReleaseCapture();

		//        pt.x = LOWORD( lParam );
		//        pt.y = HIWORD( lParam );

		//        GetClientRect( window->hwnd, &rect );
		//        icvGetBitmapData( window, &size, 0, 0 );

		//        window->on_mouse( event, pt.x*size.cx/MAX(rect.right - rect.left,1),
		//                                 pt.y*size.cy/MAX(rect.bottom - rect.top,1), flags,
		//                                 window->on_mouse_param );
		//    }
		//    break;

	case WM_PAINT:
		if(window->image != 0)
		{

			// Determine the bitmap's dimensions
			icvGetBitmapData( window, &size, &nchannels, 0 );

			hdc = BeginPaint(hwnd, &paint);
			SetStretchBltMode(hdc, COLORONCOLOR);

			if( nchannels == 1 )
			{

				for(i = 0; i < 256; i++)
				{
					table[i].rgbBlue = (unsigned char)i;
					table[i].rgbGreen = (unsigned char)i;
					table[i].rgbRed = (unsigned char)i;
				}
				SetDIBColorTable(window->dc, 0, 255, table);
			}

			if(window->flags & CV_WINDOW_AUTOSIZE)
			{
				BitBlt( hdc, 0, 0, size.cx, size.cy, window->dc, 0, 0, SRCCOPY );
			}
			else
			{
				GetClientRect(window->hwnd, &rect);
				StretchBlt( hdc, 0, 0, rect.right - rect.left, rect.bottom - rect.top,
					window->dc, 0, 0, size.cx, size.cy, SRCCOPY );
			}
			//DeleteDC(hdc);
			EndPaint(hwnd, &paint);
		}
		else
		{
			return DefWindowProc(hwnd, uMsg, wParam, lParam);
		}
		return 0;

	case WM_ERASEBKGND:
		if(window->image)
			return 0;
		break;

	case WM_DESTROY:

		icvRemoveWindow(window);
		// Do nothing!!!
		//PostQuitMessage(0);
		break;

	case WM_SETCURSOR:
		SetCursor((HCURSOR)icvGetClassLongPtr(hwnd, CV_HCURSOR));
		return 0;

	case WM_KEYDOWN:
		window->last_key = (int)wParam;
		return 0;
	}

	return DefWindowProc(hwnd, uMsg, wParam, lParam);
}


static LRESULT CALLBACK WindowProc( HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	LRESULT ret;
	int was_processed = 0;
	//int ret;

	if( hg_on_preprocess )
	{
		was_processed = 0;
		ret = hg_on_preprocess(hwnd, uMsg, wParam, lParam, &was_processed);
		if( was_processed )
			return ret;
	}
	ret = HighGUIProc(hwnd, uMsg, wParam, lParam);

	if(hg_on_postprocess)
	{
		was_processed = 0;
		ret = hg_on_postprocess(hwnd, uMsg, wParam, lParam, &was_processed);
		if( was_processed )
			return ret;
	}

	return ret;
}

void icvUpdateWindowPos( CvWindow* window )
{
	RECT rect;
	assert(window);

	if( (window->flags & CV_WINDOW_AUTOSIZE) && window->image )
	{

		SIZE size = {0,0};
		icvGetBitmapData( window, &size, 0, 0 );

	}

	rect = icvCalcWindowRect(window);
	MoveWindow(window->hwnd, rect.left, rect.top,
		rect.right - rect.left + 1,
		rect.bottom - rect.top + 1, TRUE );
}

LRESULT CALLBACK
MainWindowProc( HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	RECT cr, tr, wrc;
	HRGN rgn, rgn1, rgn2;
	int ret;
	HDC hdc = (HDC)wParam;
	WINDOWPOS* pos;

	CvWindow* window = icvWindowByHWND( hwnd );

	if( !window )
		return DefWindowProc(hwnd, uMsg, wParam, lParam);

	switch(uMsg)
	{
	case WM_DESTROY:

		icvRemoveWindow(window);
		// Do nothing!!!
		//PostQuitMessage(0);
		break;

		//case WM_GETMINMAXINFO:
		//    if( !(window->flags & CV_WINDOW_AUTOSIZE) )
		//    {
		//        MINMAXINFO* minmax = (MINMAXINFO*)lParam;
		//        RECT rect;
		//        LRESULT retval = DefWindowProc(hwnd, uMsg, wParam, lParam);

		//        minmax->ptMinTrackSize.y = 100;
		//        minmax->ptMinTrackSize.x = 100;

		//        if( window->toolbar.first )
		//        {
		//            GetWindowRect( window->toolbar.first->hwnd, &rect );
		//            minmax->ptMinTrackSize.y += window->toolbar.rows*(rect.bottom - rect.top);
		//            minmax->ptMinTrackSize.x = MAX(rect.right - rect.left + HG_BUDDY_WIDTH, HG_BUDDY_WIDTH*2);
		//        }
		//        return retval;
		//    }
		//    break;

	case WM_WINDOWPOSCHANGED:
		{
			pos = (WINDOWPOS*)lParam;


			if(!(window->flags & CV_WINDOW_AUTOSIZE))
				icvUpdateWindowPos(window);

			break;
		}

	case WM_ACTIVATE:
		if(LOWORD(wParam) == WA_ACTIVE || LOWORD(wParam) == WA_CLICKACTIVE)
			SetFocus(window->hwnd);
		break;

	case WM_ERASEBKGND:
		{

			GetWindowRect(window->hwnd, &cr);
			icvScreenToClient(window->frame, &cr);

			tr.left = tr.top = tr.right = tr.bottom = 0;

			GetClientRect(window->frame, &wrc);

			rgn = CreateRectRgn(0, 0, wrc.right, wrc.bottom);
			rgn1 = CreateRectRgn(cr.left, cr.top, cr.right, cr.bottom);
			rgn2 = CreateRectRgn(tr.left, tr.top, tr.right, tr.bottom);
			ret = CombineRgn(rgn, rgn, rgn1, RGN_DIFF);
			ret = CombineRgn(rgn, rgn, rgn2, RGN_DIFF);

			if(ret != NULLREGION && ret != ERROR)
				FillRgn(hdc, rgn, (HBRUSH)icvGetClassLongPtr(hwnd, CV_HBRBACKGROUND));

			DeleteObject(rgn);
			DeleteObject(rgn1);
			DeleteObject(rgn2);
		}
		return 1;
	}

	return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

int cvInitSystem( )
{
	static int wasInitialized = 0;
	WNDCLASS wndc;

	// check initialization status
	if( !wasInitialized )
	{
		// Initialize the stogare
		hg_windows = 0;

		// Register the class

		wndc.style = CS_OWNDC | CS_VREDRAW | CS_HREDRAW;
		wndc.lpfnWndProc = WindowProc;
		wndc.cbClsExtra = 0;
		wndc.cbWndExtra = 0;
		wndc.hInstance = hg_hinstance;
		wndc.lpszClassName = highGUIclassName;
		wndc.lpszMenuName = highGUIclassName;
		wndc.hIcon = LoadIcon(0, IDI_APPLICATION);
		wndc.hCursor = (HCURSOR)LoadCursor(0, (LPCSTR)(size_t)IDC_CROSS );
		//wndc.hCursor = (HCURSOR)LoadCursor(0, (LPSTR)(size_t)IDC_CROSS );
		wndc.hbrBackground = (HBRUSH)GetStockObject(GRAY_BRUSH);

		RegisterClass(&wndc);

		wndc.lpszClassName = mainHighGUIclassName;
		wndc.lpszMenuName = mainHighGUIclassName;
		wndc.hbrBackground = (HBRUSH)GetStockObject(GRAY_BRUSH);
		wndc.lpfnWndProc = MainWindowProc;

		RegisterClass(&wndc);
		atexit( icvCleanupHighgui );

		wasInitialized = 1;
	}

	return 0;
}


void updateWindows (CvWindow *window)
{
	window->next = hg_windows;
	window->prev = 0;
	if( hg_windows )
		hg_windows->prev = window;
	hg_windows = window;
}

CvWindow* icvFindWindowByName( const char* name )
{
	CvWindow* window = hg_windows;

	for( ; window != 0 && strcmp( name, window->name) != 0; window = window->next )
		;

	return window;
}

HWND createMainWin (const char *mainClass, const char *name, CvRect rect)
{
	DWORD defStyle = WS_VISIBLE | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU;

	HWND mainhWnd = CreateWindow( mainClass, name, defStyle | WS_OVERLAPPED,
		rect.x, rect.y, rect.width, rect.height, 0, 0, hg_hinstance, 0 );

	return mainhWnd;
}

HWND createWin (const char *className, const char *name, CvRect rect, HWND mainhWnd)
{
	DWORD defStyle = WS_VISIBLE | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU;

	HWND hWnd = CreateWindow( className, name,
		(defStyle & ~WS_SIZEBOX) | WS_CHILD, CW_USEDEFAULT, 0, rect.width, rect.height, mainhWnd, 0, hg_hinstance, 0);

	return hWnd;
}

int
cvWaitKey( int delay )
{
	int time0 = GetTickCount();
	CvWindow* window;
	MSG message;
	int is_processed = 0;

	for(;;)
	{
		is_processed = 0;

		if( (delay > 0 && abs((int)(GetTickCount() - time0)) >= delay) || hg_windows == 0 )
			return -1;

		if( delay <= 0 )
			GetMessage(&message, 0, 0, 0);
		else if( PeekMessage(&message, 0, 0, 0, PM_REMOVE) == FALSE )
		{
			Sleep(1);
			continue;
		}

		for( window = hg_windows; window != 0 && is_processed == 0; window = window->next )
		{
			if( window->hwnd == message.hwnd || window->frame == message.hwnd )
			{
				is_processed = 1;
				switch(message.message)
				{
				case WM_DESTROY:
				case WM_CHAR:
					DispatchMessage(&message);
					return (int)message.wParam;

				case WM_SYSKEYDOWN:
					if( message.wParam == VK_F10 )
					{
						is_processed = 1;
						return (int)(message.wParam << 16);
					}
					break;

				case WM_KEYDOWN:
					TranslateMessage(&message);
					if( (message.wParam >= VK_F1 && message.wParam <= VK_F24) ||
						message.wParam == VK_HOME || message.wParam == VK_END ||
						message.wParam == VK_UP || message.wParam == VK_DOWN ||
						message.wParam == VK_LEFT || message.wParam == VK_RIGHT ||
						message.wParam == VK_INSERT || message.wParam == VK_DELETE ||
						message.wParam == VK_PRIOR || message.wParam == VK_NEXT )
					{
						DispatchMessage(&message);
						is_processed = 1;
						return (int)(message.wParam << 16);
					}
				default:
					DispatchMessage(&message);
					is_processed = 1;
					break;
				}
			}
		}

		if( !is_processed )
		{
			TranslateMessage(&message);
			DispatchMessage(&message);
		}
	}
}

#endif // defined(IS_WIN)
