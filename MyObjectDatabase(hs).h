//==============================================================================
/*
Filename:	MyObjectDatabase.h
Project:	Haptics Showroom
Authors:	Naina Dhingra, Ke Xu, Hannes Bohnengel
Revision:	0.2
Remarks:	These files are tracked with git and are available on the github
repository: https://github.com/hannesb0/haptics-showroom
*/
//==============================================================================

#include "MyProperties.h"
#include "Global.h"





vector<cVector3d> wallPositions;
vector<cVector3d> tablePositions;
vector<cVector3d> objPositions;

void initObjPositions()
{
	/*objPositions.push_back(cVector3d(5.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(4.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(3.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(2.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(1.0, (-(roomWidth / 2) + 0.5), 0.265));
	*/

	for (int i = 0; i <= 10; i++) //MAX_OBJECT_COUNT/2
	{
		objPositions.push_back(cVector3d(double(i-5), (-(roomWidth / 2) + 0.5), 0.265));
	}
	for (int i = 0; i < 8; i++)
	{
		objPositions.push_back(cVector3d(double(i-5), ((roomWidth / 2) - 0.5), 0.265));

	}


	/*objPositions.push_back(cVector3d(-1.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(-2.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(-3.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(-4.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(-5.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(-6.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(-7.0, (-(roomWidth / 2) + 0.5), 0.265));
	objPositions.push_back(cVector3d(-8.0, (-(roomWidth / 2) + 0.5), 0.265));

	objPositions.push_back(cVector3d(-1.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(-2.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(-3.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(-4.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(-5.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(-6.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(-7.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(-8.0, ((roomWidth / 2) - 0.5), 0.265));

	
	objPositions.push_back(cVector3d(5.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(4.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(3.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(2.0, ((roomWidth / 2) - 0.5), 0.265));
	objPositions.push_back(cVector3d(1.0, ((roomWidth / 2) - 0.5), 0.265));
	*/

}

vector<double> weights;
// scale object width, length thickness
const double w = 0.5;

void initWeights()
{
	//weights.push_back(-0.1*w);
	//weights.push_back(-0.1*w);
	//weights.push_back(-0.1*w);
	//weights.push_back(-0.1*w);
	

	//weights.push_back(-1 * w);
	weights.push_back(-2 *w);
	weights.push_back(-0.3*w);
	weights.push_back(-0.22*w);
	weights.push_back(-0.5*w);

	weights.push_back(-6.3*w);
	weights.push_back(-0.1*w);
	weights.push_back(-0.33*w);
	weights.push_back(-0.1*w);
	weights.push_back(-2.3*w);

	weights.push_back(-0.2*w);
	weights.push_back(-0.1*w);
	weights.push_back(-3.6*w);
	weights.push_back(-2.5*w);
	weights.push_back(-2.19*w);

	weights.push_back(-8 * w);
	weights.push_back(-8 * w);
	weights.push_back(-1 * w);
	weights.push_back(-1 * w);
	weights.push_back(-1 * w);
	weights.push_back(-1 * w);
	weights.push_back(-1 * w);


}




//------------------------------------------------------------------------------
// Orientations
//------------------------------------------------------------------------------	

// set some orientation
struct MyOrientation orientation1 {
	cVector3d(0.0, 0.0, 0.0), 0.0
};

// set some orientation
struct MyOrientation orientation2 {
	cVector3d(1.0, 1.0, 1.0), 45.0
};

//------------------------------------------------------------------------------
// Objects
//------------------------------------------------------------------------------	

/*
Texture, NormalMap, Audio, Size, Orientation, Shape,  
Temperature, Stiffness, StaticFriction, DynamicFriction, TextureLevel, AudioGain, AudioPitch, AudioPitchOffset);

MyProperties(string setTexture, string setNormalMap, string setAudio, cVector3d setSize, MyOrientation setOrientation, enum MyShape setShape, int setTemperature,
double setStiffness, double setStaticFriction, double setDynamicFriction, double setTextureLevel, double setAudioGain, double setAudioPitchGain, double setAudioPitchOffset);

Texture:	file name for displaying texture
NormalMap:	
Audio:		
...

*/

// scale object width, length thickness
const double a = 2;

// new 16
//
//													     Texture,                          NormalMap,                                  Audio, Impact Audio,                                     Size,                           Orientation,  Shape,     Temperature, Stiffness, StaticFriction, DynamicFriction, TextureLevel,     AudioGain, AudioPitch, AudioPitchOffset);

//TactileSignal_Move_New_Granite.wav

MyProperties Cube_LinedAluminum(	"DisplayImage_LinedAluminum.JPG",	"MacroImage_LinedAluminum.JPG",		"TactileSignal_Move_New_LinedAluMesh.wav",	"TactileSignal_Tap_LinedAluminum.wav",		a*cVector3d(0.25, 0.25, 0.001), orientation1, MyShape(cube),	2, 0.9,		0.1, 0.01,2.0,		1.0, 1.0, 0.0);
MyProperties Cube_CeramicTile(		"DisplayImage_CeramicTile.JPG",		"MacroImage_CeramicTile.JPG",		"TactileSignal_Move_New_Marble.wav",		"TactileSignal_Tap_CeramicTile.wav",		a*cVector3d(0.25, 0.25, 0.007), orientation1, MyShape(cube),	2, 2.2,		0.1, 0.2, 1.0,		1.0, 1.0, 0.0);
MyProperties Cube_CoarseFoam(		"DisplayImage_CoarseFoam.JPG",		"MacroImage_CoarseFoam.JPG",		"TactileSignal_Move_New_CoarseFoam.wav",	 "TactileSignal_Tap_CoarseFoam.wav",		a*cVector3d(0.25, 0.25, 0.02), orientation1, MyShape(cube),		3, 0.2,		1.1, 1.0, 1.0,		1.0, 1.0, 0.0);
MyProperties Cube_Fabric(			"DisplayImage_Fabric.JPG",			"MacroImage_Fabric.JPG",			"TactileSignal_Move_Fabric.wav",			"TactileSignal_Tap_Fabric.wav",				a*cVector3d(0.25, 0.25, 0.005), orientation1, MyShape(cube),	3, 0.1,		0.4, 0.3, 1.0,		1.0, 1.0, 0.0);
MyProperties Cube_FineRubber(		"DisplayImage_FineRubber.JPG",		"MacroImage_FineRubber.JPG",		"TactileSignal_Move_FineRubber.wav",		"TactileSignal_Tap_FineRubber.wav",			a*cVector3d(0.25, 0.25, 0.01), orientation1, MyShape(cube),		3, 0.6,		0.8, 0.2, 1.0,		1.0, 1.0, 0.0);

MyProperties Cube_GraniteTile(		"DisplayImage_GraniteTile.JPG",		"MacroImage_GraniteTile.JPG",		"TactileSignal_Move_GraniteTile.wav",		"TactileSignal_Tap_GraniteTile.wav",		a*cVector3d(0.25, 0.25, 0.02), orientation1, MyShape(cube),		1, 2.5,		0.2, 0.1, 1.0,		4.0, 1.0, 0.0);
MyProperties Cube_Leather(			"DisplayImage_Leather.JPG",			"MacroImage_Leather.JPG",			"TactileSignal_Move_Leather.wav",			"TactileSignal_Tap_Leather.wav",			a*cVector3d(0.25, 0.25, 0.005), orientation1, MyShape(cube),	3, 0.5,		0.4, 0.3, 1.0,		1.0, 1.0, 0.0);
MyProperties Cube_MeshAluminum(		"DisplayImage_MeshAluminum.JPG",	"MacroImage_MeshAluminum.JPG",		"TactileSignal_Move_New_RhombAluMesh.wav",	"TactileSignal_Tap_MeshAluminum.wav",		a*cVector3d(0.25, 0.25, 0.001), orientation1, MyShape(cube),	2, 0.7,		0.3, 0.2, 3.0,		1.0, 1.0, 0.0);
MyProperties Cube_Plastic(			"DisplayImage_Plastic.JPG",			"MacroImage_Plastic.JPG",			"TactileSignal_Move_Plastic.wav",			"TactileSignal_Tap_Plastic.wav",			a*cVector3d(0.25, 0.25, 0.003), orientation1, MyShape(cube),	3, 2.0,		0.1, 0.05,0.1,		1.0, 1.0, 0.0);
//MyProperties Cube_Plastic(			"DisplayImage_Plastic.JPG",			"MacroImage_Plastic.JPG",			"TactileSignal_Move_Plastic.wav",			"TactileSignal_Tap_Plastic.wav",			a*cVector3d(0.50, 0.50, 0.003), orientation1, MyShape(cube),	3, 0.5,		0.01, 0.01, 1.0,		1.0, 1.0, 0.0);
MyProperties Cube_ProfiledWood(		"DisplayImage_ProfiledWood.JPG",	"MacroImage_ProfiledWood.JPG",		"TactileSignal_Move_New_ProfiledWoodenPlate.wav","TactileSignal_Tap_ProfiledWood.wav",	a*cVector3d(0.25, 0.25, 0.025), orientation1, MyShape(cube),	3, 1.0,		0.6, 0.5, 2.0,		1.0, 1.0, 0.0);

MyProperties Cube_Sandpaper(		"DisplayImage_Sandpaper.JPG",		"MacroImage_Sandpaper.JPG",			"TactileSignal_Move_Sandpaper.wav",			"TactileSignal_Tap_Sandpaper.wav",			a*cVector3d(0.25, 0.25, 0.001), orientation1, MyShape(cube),	3, 1.1,		1.2, 1.0, 1.0,		1.0, 1.0, 0.0);
MyProperties Cube_Sponge(			"DisplayImage_Sponge.JPG",			"MacroImage_Sponge.JPG",			"TactileSignal_Move_New_FoamFoil.wav",		"TactileSignal_Tap_Sponge.wav",				a*cVector3d(0.25, 0.25, 0.011), orientation1, MyShape(cube),	3, 0.3,		0.9, 0.8, 1.0,		1.0, 1.0, 0.0);
MyProperties Cube_StainlessSteel(	"DisplayImage_StainlessSteel.JPG",  "MacroImage_StainlessSteel.JPG",	"TactileSignal_Move_New_Steel.wav",			"TactileSignal_Tap_StainlessSteel.wav",		a*cVector3d(0.25, 0.25, 0.001), orientation1, MyShape(cube),	1, 2.6,		0.1, 0.3, 1.0,		1.0, 1.0, 0.0);
MyProperties Cube_StructuredRubber( "DisplayImage_StructuredRubber.JPG","MacroImage_StructuredRubber.JPG",	"TactileSignal_Move_StructuredRubber.wav",	"TactileSignal_Tap_StructuredRubber.wav",	a*cVector3d(0.25, 0.25, 0.01), orientation1, MyShape(cube),		3, 0.6,		1.2, 0.4, 1.0,		1.0, 1.0, 0.0);
MyProperties Cube_Styrofoam(		"DisplayImage_Styrofoam.JPG",		"MacroImage_Styrofoam.JPG",			"TactileSignal_Move_New_Styrofoam.wav",		"TactileSignal_Tap_Styrofoam.wav",			a*cVector3d(0.25, 0.25, 0.05), orientation1, MyShape(cube),		3, 0.8,		0.2, 0.2, 1.0,		1.0, 1.0, 0.0);

MyProperties Cube_WoodOak(			"DisplayImage_WoodOak.JPG",			"MacroImage_WoodOak.JPG",			"TactileSignal_Move_New_WoodOak.wav",		"TactileSignal_Tap_WoodOak.wav",			a*cVector3d(0.25, 0.25, 0.02), orientation1, MyShape(cube),		3, 1.5,		0.1, 0.1, 1.0,		1.0, 1.0, 0.0);
MyProperties Cube_GlowingMetal(		"DisplayImage_GlowingMetal.jpg",	"MacroImage_GlowingMetal.JPG",		"TactileSignal_Move_New_CrushedRock.wav",	"TactileSignal_Tap_CeramicTile.wav",		a*cVector3d(0.25, 0.25, 0.01), orientation1, MyShape(cube),		5, 0.7,		0.1, 0.1, 0.01,		1.0, 1.0, 0.0);
MyProperties Cube_Lava(				"DisplayImage_Lava.jpg",			"MacroImage_Lava.JPG",				"TactileSignal_Move_GraniteTile.waw",		"TactileSignal_Tap_CeramicTile.wav",		a*cVector3d(0.25, 0.25, 0.01), orientation1, MyShape(cube),		4, 0.5,		0.6, 0.5, 0.01,		1.0, 1.0, 0.0);
MyProperties Cube_Ice(				"DisplayImage_Ice.jpg",				"MacroImage_Ice.jpg",				"TactileSignal_Move_Ice.wav",				"TactileSignal_Tap_CeramicTile.wav",		a*cVector3d(0.25, 0.25, 0.01), orientation1, MyShape(cube),		1, 1.9,		0.01, 0.001, 0.01,	1.0, 1.0, 0.0);
/*

"DisplayImage_GlowingMetal.jpg",	"MacroImage_GlowingMetal.JPG",		"TactileSignal_Move_StainlessSteel.wav",	"TactileSignal_Tap_CeramicTile.wav",

MyProperties Cube_Aluminium("G1RhombAluminumMesh.JPG", "G1RhombAluminumMeshNormal.JPG", "RhombicAluminumMesh.wav", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.5, 0.35, 0.01), orientation1, MyShape(cube),
	3, 0.8, 0.3, 0.3, 0.2, 1.0, 0.2, 1.0);

MyProperties Cube_SilverOak("G4WoodTypeSilverOak.JPG", "G4WoodTypeSilverOakNormal.JPG", "WoodTypeSilverOak.wav", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.5, 0.35, 0.01), orientation1, MyShape(cube),
	3, 0.6, 0.2, 0.1, 0.1, 1.0, 0.2, 1.0);


MyProperties Cube_Steel("G3StainlessSteel.JPG", "G3StainlessSteelNormal.JPG", "StainlessSteel.wav", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.5, 0.35, 0.01), orientation1, MyShape(cube),
	3, 0.8, 0.1, 0.1, 0.2, 1.0, 0.2, 1.0);

MyProperties Cube_WoodProfiled("G4ProfiledWoodPlate.JPG", "G4ProfiledWoodPlateNormal.JPG", "ProfiledWoodenPlate.wav", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.5, 0.35, 0.01), orientation1, MyShape(cube),
	3, 0.9, 0.4, 0.4, 0.2, 1.0, 0.2, 1.0);

MyProperties Cube_Rubber("G5ProfiledRubberPlate.JPG", "G5ProfiledRubberPlateNormal.JPG", "ProfiledRubberPlate.wav", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.5, 0.35, 0.01), orientation1, MyShape(cube),
	3, 0.3, 0.9, 0.6, 0.8, 1.0, 0.2, 1.0);


MyProperties Cube_Granite("G2GraniteTypeVeneziano.JPG", "", "Brick.wav", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.5, 0.35, 0.01), orientation1, MyShape(cube),
	2, 1.0, 0.1, 0.1, 0.01, 1.0, 0.2, 1.0);
	
MyProperties Cube_Coarsefoam("G7CoarseFoam.JPG", "G7CoarseFoam_NRM.JPG", "CoarseFoam_3_Z.wav", cVector3d(0.2, 0.2, 0.1), orientation1, MyShape(cube),
	3, 0.6, 0.1, 0.1, 0.1, 1.0, 0.8, 0.8);
*/

MyProperties Property_3ds("whiteWall.jpg", "whiteWallNormal.png", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.2, 0.2, 0.4), orientation1, MyShape(cube),
	3, 0.6, 0.4, 0.4, 0.2, 0.0, 0.2, 0.8);


//------------------------------------------------------------------------------
// Room environment
//------------------------------------------------------------------------------	

MyProperties myFloor("floor1small.jpg", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(roomLength, roomWidth, 0.0), MyOrientation{ cVector3d(0.0, 0.0, 0.0), 0.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myCeiling("ceiling1.jpg", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(roomLength, roomWidth, 0.0), MyOrientation{ cVector3d(0.0, 1.0, 0.0), 180.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myFrontWall("whiteWall.jpg", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(roomHeight, roomWidth, 0.0), MyOrientation{ cVector3d(0.0, 1.0, 0.0), -90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myBackWall("whiteWall.jpg", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(roomHeight, roomWidth, 0.0), MyOrientation{ cVector3d(0.0, 1.0, 0.0), 90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myLeftWall("whiteWall.jpg", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(roomLength, roomHeight, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), -90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myRightWall("whiteWall.jpg", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(roomLength, roomHeight, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), 90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myWindow("window1.jpg", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.6, 0.8, 0.0), MyOrientation{ cVector3d(0.0, 1.0, 0.0), 90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);


//------------------------------------------------------------------------------
// Labels
//------------------------------------------------------------------------------	
#if 1
MyProperties myTitle("title.jpg", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.3, 1.32, 0.0), MyOrientation{ cVector3d(0.0, 1.0, 0.0), 90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myLeftLablePW("ProfiledWood.png", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.615, 0.15, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), -90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myLeftLableSS("StainlessSteel.png", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.615, 0.15, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), -90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myLeftLableCR("CrushedRock.png", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.615, 0.15, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), -90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myLeftLableSO("SilverOak.png", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.615, 0.15, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), -90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myLeftLableAM("AluminiumMesh.png", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.615, 0.15, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), -90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myRightLablePR("ProfiledRubber.png", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.615, 0.15, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), 90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myRightLableGS("GlowingSteel.png", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.615, 0.15, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), 90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myRightLableSL("StiffenedLava.png", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.615, 0.15, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), 90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myRightLableCG("ColdGranite.png", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.615, 0.15, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), 90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myRightLableFW("FrozenWater.png", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.615, 0.15, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), 90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

MyProperties myRightLableCaution("CautionHot.jpg", "", "", "TactileSignal_Tap_CeramicTile.wav", cVector3d(0.3, 0.22, 0.0), MyOrientation{ cVector3d(1.0, 0.0, 0.0), 90.0 }, MyShape(plane),
	3, 0.8, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

#endif