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

#ifndef __CTRL_FUNC_H__
#define __CTRL_FUNC_H__

enum ButtonID {
	BTN_SCENE_RESET=0,
	BTN_SCENE_NEXT,
	BTN_SIMULATION,
	BTN_STEP,
	BTN_UP,
	BTN_DOWN,
	BTN_LEFT,
	BTN_RIGHT,
	BTN_ZOOM_IN,
	BTN_ZOOM_OUT,
	BTN_PICK,
	BTN_NUM
};

enum ButtonStatus {
	BTN_STAT_NONE = 0,
	BTN_STAT_DOWN,
	BTN_STAT_UP,
	BTN_STAT_KEEP
};

void ctrlInit();
void ctrlRelease();
void ctrlUpdate();

void ctrlSetScreenSize(int w,int h);
void ctrlGetCursorPosition(int &cursorX,int &cursorY);
bool ctrlGetCursorEnable();

ButtonStatus ctrlButtonPressed(ButtonID btnId);

#endif /* __CTRL_FUNC_H__ */
