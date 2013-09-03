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

#include "font_render_func.h"

#include <tchar.h>
#include <gl/gl.h>

extern void renderGetContext(HDC &hDC,HGLRC &hRC);

static GLuint base;
static HDC hDC;
static HGLRC hRC;

void fontRenderInit()
{
	HFONT	font;
	HFONT	oldfont;

	base = glGenLists(96);

	renderGetContext(hDC,hRC);

	font = CreateFont(	16,
						0,
						0,
						0,
						0,
						FALSE,
						FALSE,
						FALSE,
						DEFAULT_CHARSET,
						OUT_DEFAULT_PRECIS,
						CLIP_DEFAULT_PRECIS,
						DEFAULT_QUALITY,
						FF_DONTCARE|FIXED_PITCH,
						NULL);

	oldfont = (HFONT)SelectObject(hDC, font);
	wglUseFontBitmaps(hDC, 32, 96, base);
	SelectObject(hDC, oldfont);
	DeleteObject(font);
}

void fontRenderRelease()
{
	glDeleteLists(base, 96);
}

void fontRenderPrint(int sx,int sy,float r,float g,float b,const char *fmt, ...)
{
	char text[256];
	va_list ap;

	if (fmt == NULL)
		return;

	va_start(ap, fmt);
    vsprintf_s(text, fmt, ap);
	va_end(ap);

	glColor3f(r,g,b);
	glRasterPos2f((float)sx,(float)sy);
	glPushAttrib(GL_LIST_BIT);
	glListBase(base - 32);
	glCallLists(strlen(text), GL_UNSIGNED_BYTE, text);
	glPopAttrib();
}
