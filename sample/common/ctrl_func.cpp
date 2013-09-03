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

#include "common.h"
#include "ctrl_func.h"

static int s_keyState[2][BTN_NUM] = {0};
static int s_keySw = 0;

void ctrlInit()
{
	s_keySw = 0;
}

void ctrlRelease()
{
}

void ctrlUpdate()
{
	s_keyState[s_keySw][BTN_SCENE_RESET] 	= GetAsyncKeyState(VK_F1);
	s_keyState[s_keySw][BTN_SCENE_NEXT]  	= GetAsyncKeyState(VK_F2);
	s_keyState[s_keySw][BTN_SIMULATION]  	= GetAsyncKeyState(VK_F3);
	s_keyState[s_keySw][BTN_STEP]        	= GetAsyncKeyState(VK_F4);
	s_keyState[s_keySw][BTN_UP]          	= GetAsyncKeyState(VK_UP);
	s_keyState[s_keySw][BTN_DOWN]        	= GetAsyncKeyState(VK_DOWN);
	s_keyState[s_keySw][BTN_LEFT]        	= GetAsyncKeyState(VK_LEFT);
	s_keyState[s_keySw][BTN_RIGHT]       	= GetAsyncKeyState(VK_RIGHT);
	s_keyState[s_keySw][BTN_ZOOM_IN]     	= GetAsyncKeyState(VK_INSERT);
	s_keyState[s_keySw][BTN_ZOOM_OUT]    	= GetAsyncKeyState(VK_DELETE);
	s_keyState[s_keySw][BTN_PICK]			= GetAsyncKeyState(VK_LBUTTON);

	s_keySw = 1-s_keySw;
}

ButtonStatus ctrlButtonPressed(ButtonID btnId)
{
	if(s_keyState[1-s_keySw][btnId] && !s_keyState[s_keySw][btnId]) {
		return BTN_STAT_DOWN;
	}
	else if(s_keyState[1-s_keySw][btnId] && s_keyState[s_keySw][btnId]) {
		return BTN_STAT_KEEP;
	}
	else if(!s_keyState[1-s_keySw][btnId] && s_keyState[s_keySw][btnId]) {
		return BTN_STAT_UP;
	}
	
	return BTN_STAT_NONE;
}

void ctrlSetScreenSize(int w,int h)
{
}

void ctrlGetCursorPosition(int &cursorX,int &cursorY)
{
	HWND hWnd = ::GetActiveWindow();

	POINT pnt;
	RECT rect;
	::GetCursorPos(&pnt);
	::ScreenToClient(hWnd,&pnt);
	::GetClientRect(hWnd,&rect);
	cursorX = pnt.x - (rect.right - rect.left) / 2;
	cursorY = (rect.bottom-rect.top) / 2 - pnt.y;
}
