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

// 半径1の球
const int sphere_numVertices = 12;
const int sphere_numIndices = 60;
const float sphere_vertices[] = {
0.267617,-0.500000,-0.823639,-0.700629,-0.500000,-0.509037,-0.700629,-0.500000,0.509037,0.267617,-0.500000,0.823639,0.866025,-0.500000,0.000000,0.267617,0.500000,-0.823639,-0.700629,0.500000,-0.509037,-0.700629,0.500000,0.509037,0.267617,0.500000,0.823639,0.866025,0.500000,0.000000,0.000000,-1.000000,0.000000,0.000000,1.000000,0.000000
};
const unsigned short sphere_indices[] = {
0,1,5,5,1,6,1,2,6,6,2,7,2,3,7,7,3,8,3,4,8,8,4,9,4,0,9,9,0,5,1,0,10,2,1,10,3,2,10,4,3,10,0,4,10,5,6,11,6,7,11,7,8,11,8,9,11,9,5,11
};

// サイズ1x1x1のボックス
const int box_numVertices = 8;
const int box_numIndices = 36;
const float box_vertices[] = {
-1.000000,-1.000000,1.000000,1.000000,-1.000000,1.000000,-1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,-1.000000,1.000000,-1.000000,1.000000,1.000000,-1.000000,-1.000000,-1.000000,-1.000000,1.000000,-1.000000,-1.000000
};
const unsigned short box_indices[] = {
0,1,2,2,1,3,2,3,4,4,3,5,4,5,6,6,5,7,6,7,0,0,7,1,1,7,3,3,7,5,6,0,4,4,0,2
};

// 長さ1、半径1の円柱
const int cylinder_numVertices = 14;
const int cylinder_numIndices = 72;
const float cylinder_vertices[] = {
-1.000000,-0.500000,-0.866025,-1.000000,0.500000,-0.866026,-1.000000,1.000000,-0.000000,-1.000000,0.500000,0.866025,-1.000000,-0.500000,0.866025,-1.000000,-1.000000,0.000000,1.000000,-0.500000,-0.866025,1.000000,0.500000,-0.866026,1.000000,1.000000,-0.000000,1.000000,0.500000,0.866025,1.000000,-0.500000,0.866025,1.000000,-1.000000,0.000000,-1.000000,-0.000000,0.000000,1.000000,0.000000,0.000000
};
const unsigned short cylinder_indices[] = {
0,1,6,6,1,7,1,2,7,7,2,8,2,3,8,8,3,9,3,4,9,9,4,10,4,5,10,10,5,11,5,0,11,11,0,6,1,0,12,2,1,12,3,2,12,4,3,12,5,4,12,0,5,12,6,7,13,7,8,13,8,9,13,9,10,13,10,11,13,11,6,13
};

// 幅1、高さ1の三角錐
const int tetrahedron_numVertices = 4;
const int tetrahedron_numIndices = 12;
const float tetrahedron_vertices[] = {
-1.000000,-1.000000,-1.732051,-1.000000,-1.000000,1.732051,2.000000,-1.000000,0.000000,0.000000,1.000000,0.000000
};
const unsigned short tetrahedron_indices[] = {
0,2,1,0,1,3,1,2,3,2,0,3
};