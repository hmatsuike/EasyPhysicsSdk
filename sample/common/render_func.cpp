/*
	Copyright (c) 2012 Hiroshi Matsuike

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:

	1. The origin of this software must not be misrepresented; you must not
	claim that you wrote the original software. If you use this software
	in a product, an acknowledgment in the product documentation would be
	appreciated but is not required.

	2. Altered source versions must be plainly marked as such, and must not be
	misrepresented as being the original software.

	3. This notice may not be removed or altered from any source distribution.
*/

#include "render_func.h"

#include <gl/gl.h>
#include <vector>

using namespace std;
using namespace EasyPhysics;

// context
static HDC s_hDC;
static HGLRC s_hRC;
static HWND s_hWnd;
static HINSTANCE s_hInstance;
static bool s_enableGlWindow;

// local variables
static char s_title[256];
static int s_screenWidth,s_screenHeight;
static EpxMatrix4 s_pMat,s_vMat;
static EpxVector3 s_viewPos,s_lightPos,s_viewTgt;
static float s_lightRadius,s_lightRadX,s_lightRadY;
static float s_viewRadius,s_viewRadX,s_viewRadY,s_viewHeight;

struct MeshBuff {
	float *vtx;
	float *nml;
	int numVtx;
	unsigned short *idx;
	unsigned short *wireIdx;
	int numIdx;

	MeshBuff(): vtx(0), nml(0), numVtx(0), idx(0), wireIdx(0),numIdx(0){}
};

static vector<MeshBuff>* s_meshBuff;

void releaseWindow()
{
	if(s_enableGlWindow) {
		if(s_hRC) {
			wglMakeCurrent(0,0);
			wglDeleteContext(s_hRC);
		}
	}
	if(s_hDC) ReleaseDC(s_hWnd,s_hDC);
	if(s_hWnd) DestroyWindow(s_hWnd);

	UnregisterClass(s_title,s_hInstance);
}

LRESULT CALLBACK WndProc(HWND s_hWnd,UINT	uMsg,WPARAM	wParam,LPARAM lParam)
{
	switch(uMsg) {
		case WM_SYSCOMMAND:
		{
			switch (wParam) {
				case SC_SCREENSAVE:
				case SC_MONITORPOWER:
				return 0;
			}
			break;
		}

		case WM_CLOSE:
		PostQuitMessage(0);
		return 0;

		case WM_SIZE:
		renderResize(LOWORD(lParam),HIWORD(lParam));
		return 0;
	}

	return DefWindowProc(s_hWnd,uMsg,wParam,lParam);
}

bool createWindow(const char* title, int width, int height)
{
	strncpy_s(s_title,title,sizeof(s_title));
	s_title[255] = 0;

	WNDCLASS wc;
	RECT rect;
	rect.left=0;
	rect.right=width;
	rect.top=0;
	rect.bottom=height;

	s_hInstance = GetModuleHandle(NULL);
	wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wc.lpfnWndProc = (WNDPROC) WndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = s_hInstance;
	wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = NULL;
	wc.lpszMenuName = NULL;
	wc.lpszClassName = s_title;

	if(!RegisterClass(&wc)) {
		return false;
	}

	AdjustWindowRectEx(&rect, WS_OVERLAPPEDWINDOW, FALSE, WS_EX_APPWINDOW | WS_EX_WINDOWEDGE);

	if(!(s_hWnd=CreateWindowEx(WS_EX_APPWINDOW|WS_EX_WINDOWEDGE,s_title,s_title,
							WS_OVERLAPPEDWINDOW|WS_CLIPSIBLINGS|WS_CLIPCHILDREN,
							0,0,rect.right-rect.left,rect.bottom-rect.top,
							NULL,NULL,s_hInstance,NULL))) {
		releaseWindow();
		return false;
	}

    static PIXELFORMATDESCRIPTOR pfd = {
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		32,
		0, 0,
		0, 0,
		0, 0,
		0, 0,
		0,
		0, 0, 0, 0,
		32,
		0,
		0,
		PFD_MAIN_PLANE,
		0,
		0, 0, 0
    };
	
	if(!(s_hDC=GetDC(s_hWnd)))
	{
		releaseWindow();
		OutputDebugString("");
		return FALSE;
	}
	
	int pixelformat;
	
    if ( (pixelformat = ChoosePixelFormat(s_hDC, &pfd)) == 0 ){
		OutputDebugString("ChoosePixelFormat Failed....");
        return FALSE;
    }

    if (SetPixelFormat(s_hDC, pixelformat, &pfd) == FALSE){
		OutputDebugString("SetPixelFormat Failed....");
        return FALSE;
    }

	if (!(s_hRC=wglCreateContext(s_hDC))){
		OutputDebugString("Creating HGLRC Failed....");
		return FALSE;
	}
	
	wglMakeCurrent(s_hDC,s_hRC);
	
	// Set Vsync
	BOOL (WINAPI *wglSwapIntervalEXT)(int) = NULL;
	wglSwapIntervalEXT = (BOOL (WINAPI*)(int))wglGetProcAddress("wglSwapIntervalEXT");
	if(wglSwapIntervalEXT) wglSwapIntervalEXT(1);
	
	ShowWindow(s_hWnd,SW_SHOW);
	SetForegroundWindow(s_hWnd);
	SetFocus(s_hWnd);

	renderResize(width, height);
	
	glClearColor(0.0f,0.0f,0.0f,0.0f);
	glClearDepth(1.0f);
	
	return TRUE;
}

void renderInit(const char *title)
{
	s_enableGlWindow = (title != NULL);

	s_screenWidth = DISPLAY_WIDTH;
	s_screenHeight = DISPLAY_HEIGHT;

	if(s_enableGlWindow) {
		if(!createWindow(title,s_screenWidth,s_screenHeight)) {
			MessageBox(NULL,"Can't create gl window.","ERROR",MB_OK|MB_ICONEXCLAMATION);
			assert(0);
		}
	}

	// initalize matrix
	s_pMat = EpxMatrix4::perspective(3.1415f/4.0f, (float)s_screenWidth/(float)s_screenHeight,0.1f, 1000.0f);

	// initalize parameters
	s_lightRadius = 40.0f;
	s_lightRadX = -0.6f;
	s_lightRadY = 0.6f;
	s_viewRadius = 20.0f;
	s_viewRadX = -0.01f;
	s_viewRadY = 0.0f;
	s_viewHeight = 1.0f;

	s_viewTgt = EpxVector3(0.0f,s_viewHeight,0.0f);
	
	s_meshBuff = new vector<MeshBuff>();
}

void renderRelease()
{
	for (EpxUInt32 c=0; c<s_meshBuff->size(); ++c)
	{
		delete[] (*s_meshBuff)[c].vtx;
		delete[] (*s_meshBuff)[c].nml;
		delete[] (*s_meshBuff)[c].idx;
		delete[] (*s_meshBuff)[c].wireIdx;
	}
	s_meshBuff->clear();
	delete s_meshBuff;

	releaseWindow();
}

void renderBegin()
{
	wglMakeCurrent(s_hDC, s_hRC);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glFrontFace(GL_CCW);
    glDepthFunc(GL_LESS);
	glCullFace(GL_BACK);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf((GLfloat*)&s_pMat);

	// create view matrix
	s_viewPos = 
		EpxMatrix3::rotationY(s_viewRadY) * 
		EpxMatrix3::rotationX(s_viewRadX) * 
		EpxVector3(0,0,s_viewRadius);

	s_lightPos = 
		EpxMatrix3::rotationY(s_lightRadY) * 
		EpxMatrix3::rotationX(s_lightRadX) * 
		EpxVector3(0,0,s_lightRadius);

	EpxMatrix4 viewMtx = EpxMatrix4::lookAt(EpxPoint3(s_viewTgt+s_viewPos),EpxPoint3(s_viewTgt),EpxVector3(0.0f,1.0f,0.0f));

	s_vMat = s_pMat * viewMtx;

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf((GLfloat*)&viewMtx);
}

void renderLookAtTarget(const EpxVector3 &viewPos,const EpxVector3 &viewTarget)
{
	s_viewPos = viewPos;
	EpxMatrix4 viewMtx = EpxMatrix4::lookAt(EpxPoint3(viewPos),EpxPoint3(viewTarget),EpxVector3(0.0f,1.0f,0.0f));
	s_vMat = s_pMat * viewMtx;
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf((GLfloat*)&viewMtx);
}

void renderEnd()
{
	SwapBuffers(s_hDC);
}

void renderDebugBegin()
{
	glDepthMask(GL_FALSE);
	glDisable(GL_DEPTH_TEST);
}

void renderDebugEnd()
{
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
}

void renderGetViewAngle(float &angleX,float &angleY,float &radius)
{
	angleX = s_viewRadX;
	angleY = s_viewRadY;
	radius = s_viewRadius;
}

void renderSetViewAngle(float angleX,float angleY,float radius)
{
	s_viewRadX   = angleX;
	s_viewRadY   = angleY;
	s_viewRadius = radius;
}

int renderInitMesh(
	const float *vtx,unsigned int vtxStrideBytes,
	const float *nml,unsigned int nmlStrideBytes,
	const unsigned short *tri,unsigned int triStrideBytes,
	int numVtx,int numTri)
{
//	assert(numMesh<MAX_MESH);
	
	MeshBuff buff;
	buff.vtx = new float [3*numVtx];
	buff.nml = new float [3*numVtx];
	buff.idx = new unsigned short [numTri*3];
	buff.wireIdx = new unsigned short [numTri*6];
	buff.numIdx = numTri*3;
	buff.numVtx = numVtx;

	for(int i=0;i<numVtx;i++) {
		const float *v = (float*)((uintptr_t)vtx + vtxStrideBytes * i);
		const float *n = (float*)((uintptr_t)nml + nmlStrideBytes * i);
		buff.vtx[i*3  ] = v[0];
		buff.vtx[i*3+1] = v[1];
		buff.vtx[i*3+2] = v[2];
		buff.nml[i*3  ] = n[0];
		buff.nml[i*3+1] = n[1];
		buff.nml[i*3+2] = n[2];
	}

	for(int i=0;i<numTri;i++) {
		const unsigned short *idx = (unsigned short*)((uintptr_t)tri + triStrideBytes * i);
		buff.idx[i*3  ] = idx[0];
		buff.idx[i*3+1] = idx[1];
		buff.idx[i*3+2] = idx[2];
		buff.wireIdx[i*6  ] = buff.idx[i*3  ];
		buff.wireIdx[i*6+1] = buff.idx[i*3+1];
		buff.wireIdx[i*6+2] = buff.idx[i*3+1];
		buff.wireIdx[i*6+3] = buff.idx[i*3+2];
		buff.wireIdx[i*6+4] = buff.idx[i*3+2];
		buff.wireIdx[i*6+5] = buff.idx[i*3  ];
	}

	s_meshBuff->push_back(buff);
	return s_meshBuff->size()-1;
}
void renderReleaseMeshAll()
{
	for (EpxUInt32 c=0; c<s_meshBuff->size(); ++c)
	{
		delete[] (*s_meshBuff)[c].vtx;
		delete[] (*s_meshBuff)[c].nml;
		delete[] (*s_meshBuff)[c].idx;
		delete[] (*s_meshBuff)[c].wireIdx;
	}
	s_meshBuff->clear();
}

void renderMesh(
	const EpxTransform3 &transform,
	const EpxVector3 &color,
	int meshId)
{
	assert(meshId>=0&&(EpxUInt32)meshId<s_meshBuff->size());
	
	MeshBuff &buff = (*s_meshBuff)[meshId];

	EpxMatrix4 wMtx = EpxMatrix4(transform);

	glPushMatrix();
	glMultMatrixf((GLfloat*)&wMtx);

	glEnableClientState(GL_VERTEX_ARRAY);
	
	glVertexPointer(3,GL_FLOAT,0,buff.vtx);

	glColor4f(color[0],color[1],color[2],1.0f);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f,1.0f);
	glDrawElements(GL_TRIANGLES,buff.numIdx,GL_UNSIGNED_SHORT,buff.idx);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor4f(0.0f,0.0f,0.0f,1.0f);
	glDrawElements(GL_LINES,buff.numIdx*2,GL_UNSIGNED_SHORT,buff.wireIdx);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glPopMatrix();
}

void renderResize(int width,int height)
{
	glViewport(0,0,width,height);
	s_pMat = EpxMatrix4::perspective(3.1415f/4.0f, (float)width/(float)height,0.1f, 1000.0f);
	s_screenWidth = width;
	s_screenHeight = height;
}

void renderDebugPoint(
	const EpxVector3 &position,
	const EpxVector3 &color)
{
	glColor4f(color[0],color[1],color[2],1.0f);

	glPointSize(5.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)&position);
	glDrawArrays(GL_POINTS,0,1);
	glDisableClientState(GL_VERTEX_ARRAY);
	glPointSize(1.0f);
}

void renderDebugLine(
	const EpxVector3 &position1,
	const EpxVector3 &position2,
	const EpxVector3 &color)
{
	glColor4f(color[0],color[1],color[2],1.0f);
	
	const EpxVector3 points[2] = {
		position1,
		position2,
	};
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)points);
	glDrawArrays(GL_LINES,0,2);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void renderDebugAabb(
	const EpxVector3 &center,
	const EpxVector3 &extent,
	const EpxVector3 &color)
{
	const EpxVector3 points[8] = {
		center + mulPerElem(EpxVector3(-1,-1,-1),extent),
		center + mulPerElem(EpxVector3(-1,-1, 1),extent),
		center + mulPerElem(EpxVector3( 1,-1, 1),extent),
		center + mulPerElem(EpxVector3( 1,-1,-1),extent),
		center + mulPerElem(EpxVector3(-1, 1,-1),extent),
		center + mulPerElem(EpxVector3(-1, 1, 1),extent),
		center + mulPerElem(EpxVector3( 1, 1, 1),extent),
		center + mulPerElem(EpxVector3( 1, 1,-1),extent),
	};
	
	const unsigned short indices[] = {
		0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,0,4,1,5,2,6,3,7,
	};
	
	glColor4f(color[0],color[1],color[2],1.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)points);
	glDrawElements(GL_LINES,24,GL_UNSIGNED_SHORT,indices);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void renderDebugSphere(
	const EasyPhysics::EpxVector3 &position,
	const EasyPhysics::EpxFloat radius,
	const EasyPhysics::EpxVector3 &color)
{
	const EpxVector3 points[] = {
		position + radius * EpxVector3(0.267617,-0.500000,-0.823639),
		position + radius * EpxVector3(-0.700629,-0.500000,-0.509037),
		position + radius * EpxVector3(-0.700629,-0.500000,0.509037),
		position + radius * EpxVector3(0.267617,-0.500000,0.823639),
		position + radius * EpxVector3(0.866025,-0.500000,0.000000),
		position + radius * EpxVector3(0.267617,0.500000,-0.823639),
		position + radius * EpxVector3(-0.700629,0.500000,-0.509037),
		position + radius * EpxVector3(-0.700629,0.500000,0.509037),
		position + radius * EpxVector3(0.267617,0.500000,0.823639),
		position + radius * EpxVector3(0.866025,0.500000,0.000000),
		position + radius * EpxVector3(0.000000,-1.000000,0.000000),
		position + radius * EpxVector3(0.000000,1.000000,0.000000),
	};

	const unsigned short indices[] = {
		0,1,1,2,2,3,3,4,4,0,
		5,6,6,7,7,8,8,9,9,5,
		10,0,10,1,10,2,10,3,10,4,
		11,5,11,6,11,7,11,8,11,9,
		0,5,1,6,2,7,3,8,4,9,
	};

	glColor4f(color[0],color[1],color[2],1.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)points);
	glDrawElements(GL_LINES,50,GL_UNSIGNED_SHORT,indices);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void renderDebug2dBegin()
{
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	EpxMatrix4 proj = EpxMatrix4::orthographic(-s_screenWidth*0.5f,s_screenWidth*0.5f,-s_screenHeight*0.5f,s_screenHeight*0.5f, -10.0f, 10.0f);
	glMultMatrixf((GLfloat*)&proj);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	EpxMatrix4 modelview = EpxMatrix4::translation(EpxVector3(0,0,-1));
	glMultMatrixf((GLfloat*)&modelview);

	glDisable(GL_DEPTH_TEST);
}

void renderDebug2dEnd()
{
	glEnable(GL_DEPTH_TEST);
	glPopMatrix();
}

EpxVector3 renderGetWorldPosition(const EpxVector3 &screenPos)
{
	EpxMatrix4 mvp,mvpInv;
	mvp = s_vMat;
	mvpInv = inverse(mvp);

	EpxVector4 wp(screenPos,1.0f);

	wp[0] /= (0.5f * (float)s_screenWidth);
	wp[1] /= (0.5f * (float)s_screenHeight);

	float w =	mvpInv[0][3] * wp[0] +  
				mvpInv[1][3] * wp[1] +  
				mvpInv[2][3] * wp[2] +  
				mvpInv[3][3];

	wp = mvpInv * wp;
	wp /= w;

	return wp.getXYZ();
}

EpxVector3 renderGetScreenPosition(const EpxVector3 &worldPos)
{
	EpxVector4 sp(worldPos,1.0f);

	EpxMatrix4 mvp;
	mvp = s_vMat;

	sp = mvp * sp;
	sp /= (float)sp[3];
	sp[0] *= (0.5f * (float)s_screenWidth);
	sp[1] *= (0.5f * (float)s_screenHeight);

	return sp.getXYZ();
}

void renderGetScreenSize(int &width,int &height)
{
	width = s_screenWidth;
	height = s_screenHeight;
}

void renderGetViewTarget(EpxVector3 &targetPos)
{
	targetPos = s_viewTgt;
}

void renderSetViewTarget(const EpxVector3 &targetPos)
{
	s_viewTgt = targetPos;
}

void renderGetViewRadius(float &radius)
{
	radius = s_viewRadius;
}

void renderSetViewRadius(float radius)
{
	s_viewRadius = radius;
}

void renderSetContext(HDC hDC,HGLRC hRC)
{
	s_hDC = hDC;
	s_hRC = hRC;
}

void renderGetContext(HDC &hDC,HGLRC &hRC)
{
	hDC = s_hDC;
	hRC = s_hRC;
}

void renderEnableDepthTest()
{
	glEnable(GL_DEPTH_TEST);
}

void renderDisableDepthTest()
{
	glDisable(GL_DEPTH_TEST);
}

void renderWait()
{
	glFinish();
}
