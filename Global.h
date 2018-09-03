//==============================================================================
/*
Filename:	Global.h
Project:	Haptics Showroom
Authors:	Naina Dhingra, Ke Xu, Hannes Bohnengel
Revision:	0.2
Remarks:	These files are tracked with git and are available on the github
repository: https://github.com/hannesb0/haptics-showroom
*/
//==============================================================================

#ifndef GLOBAL_H_INCLUDED
#define GLOBAL_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "chai3d.h"

using namespace chai3d;
using namespace std;

//------------------------------------------------------------------------------
// ENUMERATIONS
//------------------------------------------------------------------------------

enum MyShape { plane, cube, sphere, cylinder, complex3ds };
enum MyTempStatus { heating, cooling, standby };

//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------

struct MyOrientation{
	cVector3d	axis;
	double		rotation;
};

//------------------------------------------------------------------------------
// GLOBAL CONSTANTS
//------------------------------------------------------------------------------

const double scal = 10.0;
const double d = 0.08; // small distance
const double wallDist = scal*0.01;

// size of the rooms
const double roomLength = 30.0*scal;	// x-axis
const double roomWidth = 30.0*scal;	// y-axis
const double roomHeight = 1.5*scal;	// z-axis

// Hall
const double xHall = 3.0*scal;	// x-axis: roomLengthHall
const double yHall = 4.0*scal;	// y-axis: roomWidthHall
// Room1
const double xRoom1 = 4.0*scal;	// x-axis: roomLengthRoom1
const double yRoom1 = 4.0*scal;	// y-axis: roomWidthRoom1
const double xRoom1Cut = 5.0*scal;	// x-axis: roomLengthRoom1Cut
const double yRoom1Cut = 7.0*scal;	// y-axis: roomWidthRoom1Cut

// Room2
const double xRoom2 = 3.0 * scal;	// x-axis: roomLengthRoom2
const double yRoom2 = yHall + yRoom1;	// y-axis: roomWidthRoom2

// Door
const double doorHeight = 2.0*scal;
const double doorWidth = 0.8*scal;
const double xDoor1 = (xHall - doorWidth) / 2;
const double yDoor23 = (yHall - doorWidth) / 2;

// Window
const double windowHeight = 1.2*scal;
const double zWindow = (roomHeight - windowHeight) / 2;
const double windowWidth2 = 3.0*scal;

//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourcesPath+string(p)).c_str())

// concatenates string q and p (e.g. q = "abc", p = "xyz" -> STR_ADD(q,p) = "abcxyz")
#define STR_ADD(q,p)		(char*)((string(q)+string(p)).c_str())

// initial position
#define INITIAL_POSITION	cVector3d(0.0, yHall/2, roomHeight/2)

// maximal number of objects
#define MAX_OBJECT_COUNT		19// 30
#define MAX_AUDIOBUFFER_COUNT	MAX_OBJECT_COUNT
#define MAX_REGIONS_COUNT		MAX_OBJECT_COUNT


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

#define WINDOW_SIZE_W		1000
#define WINDOW_SIZE_H		1000
#define TOOL_RADIUS			0.005*scal
#define TOOL_WORKSPACE		0.2*scal


const double maxStiffness = 200;


#endif