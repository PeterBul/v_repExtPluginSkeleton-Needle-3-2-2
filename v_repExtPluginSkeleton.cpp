// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.2 Rev1 on September 5th 2015
//
// Peter: All functionality is run out of messages: 
// sim_message_eventcallback_simulationabouttostart
// sim_message_eventcallback_simulationended
// sim_message_eventcallback_modulehandle
//
// Vector3f f_ext is the variable that holds the total external force on the needle
// it is calculated relative to the dummy (magnitude * dummy_direction). The other project did
// this but it might have to be changed. It can be found in modelExternalForces(model)

#include <algorithm>
#include <math.h>
#include <vector>
#include <map>

#include "v_repExtPluginSkeleton.h"
#include "luaFunctionData.h"
#include "v_repLib.h"
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

typedef std::vector<Eigen::Vector3f> Vector3fVector;


#ifdef _WIN32
	#ifdef QT_COMPIL
		#include <direct.h>
	#else
		#include <shlwapi.h>
		#pragma comment(lib, "Shlwapi.lib")
	#endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
	#include <unistd.h>
#endif /* __linux || __APPLE__ */

#ifdef __APPLE__
#define _stricmp strcmp
#endif

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

#define PLUGIN_VERSION 2 // 2 since version 3.2.1

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind


// Handles
int dummyHandle;									// This is the handle of the dummy device. That is the virtual 
													// representation of the position of the haptic device in the scene.
int dummyToolTipHandle;								// The tool tip dummy that is used to move the robot.
int phantomHandle;
int needleHandle;
int needleTipHandle;
int extForceGraphHandle;
int lwrTipHandle;									// A dummy that is always connected to the needle tip.
int needleForceGraphHandle;

// CoppeliaSim object parameter IDs
const int RESPONDABLE = 3004;                       // Object parameter id for toggling respondable.
const int RESPONDABLE_MASK = 3019;                  // Object parameter id for toggling respondable mask.
const float FRICTION_COEFFICIENT = 0.03;            // Unit: N/mm ? Delete this?

													// Coefficients for bidirectional Karnopp friction model
const float D_p = 18.45;                            // Positive static friction coefficient. Unit: N/m 
const float D_n = -18.23;                           // Negative static friction coefficient. Unit: N/m
const float b_p = 212.13;                           // Positive damping coefficient. Unit: N-s/m²
const float b_n = -293.08;                          // Negative damping coefficient. Unit: N-s/m²
const float C_p = 10.57;                            // Positive dynamic friction coefficient. Unit: N/m
const float C_n = -11.96;                           // Negative dynamic friction coefficient. Unit: N/m
const float zero_threshold = 5.0e-6;                // (delta v/2 in paper) Threshold on static and dynamic fricion. Unit: m/s

// Config variables: Use these to configurate the details of the execution.
float engine_force_scalar = 1.0;					// How much of the v-rep engine force should be counted.
float model_force_scalar = 1.0;						// How much of the calculated force should be used.
std::string force_model = "kelvin-voigt";			// Which model should be used to model the forces.
bool use_only_z_force_on_engine = true;				// When using the engine for both checking punctures and calculating forces, 
													// Should only z direction be used, or should the full magnitude.
bool constant_puncture_threshold = false;			// Use the same puncture threshold for all tissues.
float puncture_threshold = 1.0e-2;					// Set constant puncture threshold (only used if constant_puncture_threshold==true)

// State variables
bool virtual_fixture = false;						// Is the needle in the tissue/ should the virtual fixture be activated?
float needleVelocity;
float full_penetration_length;
float f_ext_magnitude;								// Magnitude of all external forces on the needle.
float lwr_tip_engine_force_magnitude = 0;			// Magnitude of external forces on the needle_tip created by the physics engine.
Vector3f lwr_tip_enging_force = Vector3f(0.0, 0.0, 0.0); // Force vector of external forces on the needle_tip created by the physics engine.
Vector3f f_ext;										// Total forces on the needle created by the physics engine AND the modeled forces, relative to the dummy.
Vector3f toolTipPoint;
Vector3f needleDirection;


struct sPuncture {
	int handle;
	Vector3f position;
	Vector3f direction;
	std::string name;
	float penetration_length;

	void printPuncture(bool puncture) {
		if (puncture) {
			std::cout << "New puncture: " << name << std::endl;
		}
		else {
			std::cout << "Exit puncture: " << name << std::endl;
		}
		std::cout << "Position: " << position(0) << ", " << position(1) << ", " << position(2) << std::endl;
		std::cout << "Direction: " << direction(0) << ", " << direction(1) << ", " << direction(2) << std::endl;
	}
};

std::vector<sPuncture> punctures;

void addPuncture(int handle);
void checkContacts();
void checkPunctures();
void modelExternalForces(std::string force_model);
void reactivateTissues();
void setForceGraph();
void setRespondable(int handle);
void setUnRespondable(int handle);
void updateNeedleDirection();
void updateNeedleTipPos();
void updateNeedleVelocity();
int checkSinglePuncture(sPuncture puncture);
float B(sPuncture puncture);
float distance3d(Vector3f point1, Vector3f point2);
float generalForce2NeedleTipZ(Vector3f force);
float K(std::string tissueName);
float karnoppModel();
float kelvinVoigtModel();
float getVelocityMagnitude(simFloat* velocities);
float sgn(float x);
Vector3f changeBasis(const float* objectMatrixReferenceFrame, Vector3f vector);
Vector3f simContactInfo2EigenForce(const float* contactInfo);
Vector3f simObjectMatrix2EigenDirection(const float* objectMatrix);


// --------------------------------------------------------------------------------------
// simExtSkeleton_getSensorData: an example of custom Lua command
// --------------------------------------------------------------------------------------
#define LUA_GETSENSORDATA_COMMAND "simExtSkeleton_getSensorData" // the name of the new Lua command

const int inArgs_GETSENSORDATA[]={ // Decide what kind of arguments we need
	3, // we want 3 input arguments
    sim_lua_arg_int,0, // first argument is an integer
    sim_lua_arg_float|sim_lua_arg_table,3, // second argument should be a table of at least 3 float values (use 0 instead of 3 for a table of random size)
    sim_lua_arg_int|sim_lua_arg_table,2, // third argument should be a table of at least 2 integer values (use 0 instead of 2 for a table of random size)
};

void LUA_GETSENSORDATA_CALLBACK(SLuaCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getSensorData")
	p->outputArgCount=0;
	CLuaFunctionData D;
	// If successful the command will return an interger (result), a float table of size 3 (data), and a float (distance). If the command is not successful, it will not return anything
	bool commandWasSuccessful=false;
	int returnResult;
	std::vector<float> returnData;
	float returnDistance;
	if (D.readDataFromLua(p,inArgs_GETSENSORDATA,inArgs_GETSENSORDATA[0],LUA_GETSENSORDATA_COMMAND))
	{ // above function reads in the expected arguments. If the arguments are wrong, it returns false and outputs a message to the simulation status bar
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();

		int sensorIndex=inData->at(0).intData[0]; // the first argument
		std::vector<float>& floatParameters=inData->at(1).floatData; // the second argument
		std::vector<int>& intParameters=inData->at(2).intData; // the third argument

		// Now you can do something with above's arguments. For example:
		if ((sensorIndex>=0)&&(sensorIndex<10))
		{
			commandWasSuccessful=true;
			returnResult=1;
			returnData.push_back(1.0f);
			returnData.push_back(2.0f);
			returnData.push_back(3.0f);
			returnDistance=59.0f;
		}
		else
			simSetLastError(LUA_GETSENSORDATA_COMMAND,"Invalid sensor index."); // output an error message to the simulator's status bar
	}
	if (commandWasSuccessful)
	{ // prepare the return values:
		D.pushOutData(CLuaFunctionDataItem(returnResult));
		D.pushOutData(CLuaFunctionDataItem(returnData));
		D.pushOutData(CLuaFunctionDataItem(returnDistance));
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------
// simExtSkeleton_setPunctureThreshold: the force needed to puncture the tissue
// --------------------------------------------------------------------------------------
#define LUA_SETPUNCTURETHRESHOLD_COMMAND "simExtSkeleton_setPunctureThreshold" // the name of the new Lua command

const int inArgs_SETPUNCTURETHRESHOLD[] = { // Decide what kind of arguments we need
	1, // we want 1 input arguments
	sim_lua_arg_float,0, // first argument is a float
};

void LUA_SETPUNCTURETHRESHOLD_CALLBACK(SLuaCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_setPunctureThreshold")
	p->outputArgCount = 0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p, inArgs_SETPUNCTURETHRESHOLD , inArgs_SETPUNCTURETHRESHOLD[0], LUA_SETPUNCTURETHRESHOLD_COMMAND))
	{ // above function reads in the expected arguments. If the arguments are wrong, it returns false and outputs a message to the simulation status bar
		std::vector<CLuaFunctionDataItem>* inData = D.getInDataPtr();

		puncture_threshold = inData->at(0).intData[0]; // the first argument
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
	// Dynamically load and bind V-REP functions:
	// ******************************************
	// 1. Figure out this plugin's directory:
	char curDirAndFile[1024];
#ifdef _WIN32
	#ifdef QT_COMPIL
		_getcwd(curDirAndFile, sizeof(curDirAndFile));
	#else
		GetModuleFileName(NULL,curDirAndFile,1023);
		PathRemoveFileSpec(curDirAndFile);
	#endif
#elif defined (__linux) || defined (__APPLE__)
	getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

	std::string currentDirAndPath(curDirAndFile);
	// 2. Append the V-REP library's name:
	std::string temp(currentDirAndPath);
#ifdef _WIN32
	temp+="\\v_rep.dll";
#elif defined (__linux)
	temp+="/libv_rep.so";
#elif defined (__APPLE__)
	temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
	// 3. Load the V-REP library:
	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib==NULL)
	{
		std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************

	// Check the version of V-REP:
	// ******************************************
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
	{
		std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************

	std::vector<int> inArgs;

	// Register the new Lua command "simExtSkeleton_getSensorData":
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETSENSORDATA,inArgs);
	simRegisterCustomLuaFunction(LUA_GETSENSORDATA_COMMAND,strConCat("number result,table data,number distance=",LUA_GETSENSORDATA_COMMAND,"(number sensorIndex,table_3 floatParameters,table_2 intParameters)"),&inArgs[0],LUA_GETSENSORDATA_CALLBACK);
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETPUNCTURETHRESHOLD, inArgs);
	simRegisterCustomLuaFunction(LUA_SETPUNCTURETHRESHOLD_COMMAND, strConCat("number threshold"), &inArgs[0], LUA_SETPUNCTURETHRESHOLD_CALLBACK);

	return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
	// Here you could handle various clean-up tasks

	unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
	// Keep following 5 lines at the beginning and unchanged:
	static bool refreshDlgFlag=true;
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
	void* retVal=NULL;

	// Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
	// For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
	// in the V-REP user manual.

	if (message==sim_message_eventcallback_refreshdialogs)
		refreshDlgFlag=true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

	if (message==sim_message_eventcallback_menuitemselected)
	{ // A custom menu bar entry was selected..
		// here you could make a plugin's main dialog visible/invisible
	}

	if (message==sim_message_eventcallback_instancepass)
	{	// This message is sent each time the scene was rendered (well, shortly after) (very often)
		// It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

		int flags=auxiliaryData[0];
		bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message 
		bool instanceSwitched=((flags&64)!=0);

		if (instanceSwitched)
		{
			// React to an instance switch here!!
		}

		if (sceneContentChanged)
		{ // we actualize plugin objects for changes in the scene

			//...

			refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
		}
	}

	if (message==sim_message_eventcallback_mainscriptabouttobecalled)
	{ // The main script is about to be run (only called while a simulation is running (and not paused!))
		
	}

	if (message==sim_message_eventcallback_simulationabouttostart)
	{ // Simulation is about to start
	  // Peter
		
		dummyHandle = simGetObjectHandle("Dummy_device");
		dummyToolTipHandle = simGetObjectHandle("Dummy_tool_tip");
		phantomHandle = simGetObjectHandle("_Phantom");
		needleHandle = simGetObjectHandle("Needle");
		needleTipHandle = simGetObjectHandle("Needle_tip");
		extForceGraphHandle = simGetObjectHandle("Force_Graph");
		needleForceGraphHandle = simGetObjectHandle("Needle_force_graph");

		lwrTipHandle = simGetObjectHandle("LWR_tip");
		full_penetration_length = 0.0;
		

	}

	if (message==sim_message_eventcallback_simulationended)
	{ // Simulation just ended
		reactivateTissues();

	}

	if (message==sim_message_eventcallback_moduleopen)
	{ // A script called simOpenModule (by default the main script). Is only called during simulation.
		if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
		{
			// we arrive here only at the beginning of a simulation
		}
	}

	if (message==sim_message_eventcallback_modulehandle)
	{ // A script called simHandleModule (by default the main script). Is only called during simulation.
		if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
		{
			// we arrive here only while a simulation is running
			
			if (punctures.size() == 0)
			{
				full_penetration_length = 0.0;
				virtual_fixture = false;
			}
			else
				virtual_fixture = true;
			
			updateNeedleTipPos();
			updateNeedleVelocity();
			updateNeedleDirection();
			
			checkPunctures();
			
			checkContacts();
			
			modelExternalForces(force_model);
			
			setForceGraph();
			
		}
	}

	if (message==sim_message_eventcallback_moduleclose)
	{ // A script called simCloseModule (by default the main script). Is only called during simulation.
		if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
		{
			// we arrive here only at the end of a simulation
		}
	}

	if (message==sim_message_eventcallback_instanceswitch)
	{ // We switched to a different scene. Such a switch can only happen while simulation is not running

	}

	if (message==sim_message_eventcallback_broadcast)
	{ // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

	}

	if (message==sim_message_eventcallback_scenesave)
	{ // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

	}

	// You can add many more messages to handle here

	if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
	{ // handle refresh of the plugin's dialogs
		// ...
		refreshDlgFlag=false;
	}

	// Keep following unchanged:
	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	return(retVal);
}




// ------------------------------------------------------------------------- //
// ------------------------------------------------------------------------- //
// ------------------------- Peter Cook Bulukin -----------------------------//
// ------------------------------------------------------------------------- //
// ------------------------------------------------------------------------- //

/**
* @brief Retrieve puncture related to handle.
* @param handle: handle of object to retrieve.
* @return the puncture of the handle. Returns empty puncture if not found.
*/
sPuncture getPunctureFromHandle(int handle)
{
	for (sPuncture puncture : punctures)
	{
		if (puncture.handle == handle)
		{
			return puncture;
		}
	}
	sPuncture puncture;
	return puncture;
}

/**
* @brief Get puncture from tissue name
* @param name: name of tissue (as set in v-rep)
* @return puncture related to name. Empty puncture if not found.
*/
sPuncture getPunctureFromName(std::string name)
{
	for (sPuncture puncture : punctures)
	{
		if (puncture.name == name)
		{
			return puncture;
		}
	}
	sPuncture puncture;
	return puncture;
}

/**
* @brief Calculate length of a puncture
* @param puncture: puncture
* @return penetration distance of puncture. Value bellow zero means distance is "outside" of the tissue.
*/
float punctureLength(sPuncture puncture)
{
	return distance3d(puncture.position, toolTipPoint) * checkSinglePuncture(puncture);
}

/**
* @brief Check if the needle is still in the puncture. This is where full_penetration_length is incremented.
*/
void checkPunctures()
{
	// Iterate through punctures backwards, because if a puncture still is active, all punctures before will also still be active.
	for (auto it = punctures.rbegin(); it != punctures.rend(); ++it)
	{
		float puncture_length = punctureLength(*it);
		// If puncture length is above zero, all punctures before it in the vector will be unchanged.
		if (checkSinglePuncture(*it) > 0)
		{
			full_penetration_length -= it->penetration_length;
			full_penetration_length += puncture_length;
			// This penetration length might have been updated, so update.
			it->penetration_length = puncture_length;
			// Set punctures to be from the first puncture up until the current.
			punctures = std::vector<sPuncture>(it, punctures.rend());
			std::reverse(punctures.begin(), punctures.end());
			return;
		}
		else {
			// The needle isn't puncturing this tissue anymore. Set tissue respondable and print.
			setRespondable(it->handle);
			full_penetration_length -= it->penetration_length;
			it->printPuncture(false);
		}

	}
	// No punctures had length > 0
	punctures.clear();
}



/**
* @brief Set object respondable
* @param handle: handle
*/
void setRespondable(int handle)
{
	simSetObjectIntParameter(handle, RESPONDABLE, 1);
}

/**
* @brief Set object unrespondable
* @param handle: handle
*/
void setUnRespondable(int handle)
{
	simSetObjectIntParameter(handle, RESPONDABLE, 0);
}

/**
* @brief Check if a puncture is still active.
* @param puncture: puncture
* @return 1 if still active, -1 if not.
*/
int checkSinglePuncture(sPuncture puncture) {
	Vector3f current_translation = puncture.position - toolTipPoint;
	// If the dot product of the two vectors are positive (and not to negative because of edge case when distance is around 0), we are still in the tissue.
	if (current_translation.dot(puncture.direction) >= -1)
		return 1;
	return -1;
}

/**
* @brief Update needle velocity
*/
void updateNeedleVelocity()														// To add Low-pass filter, I think a good place to add it would be here.
{
	simFloat needleVelocities[3];
	if (simGetObjectVelocity(lwrTipHandle, needleVelocities, NULL) == -1)
		std::cerr << "Needle tip velocity retrieval failed" << std::endl;
	needleVelocity = getVelocityMagnitude(needleVelocities);
}

/**
* @brief Update needle direction
*/
void updateNeedleDirection()
{
	simFloat objectMatrix[12];
	simGetObjectMatrix(lwrTipHandle, -1, objectMatrix);
	needleDirection = simObjectMatrix2EigenDirection(objectMatrix);

}

/**
* @brief Update needle position
*/
void updateNeedleTipPos()
{
	float needleTipPos[3];
	simGetObjectPosition(lwrTipHandle, -1, needleTipPos);
	toolTipPoint = Vector3f(needleTipPos[0], needleTipPos[1], needleTipPos[2]);
}

/**
* @brief Add new puncture to punctures
* @param handle: handle of tissue that was punctured
*/
void addPuncture(int handle)
{
	float needleMatrix[12];
	simGetObjectMatrix(needleHandle, -1, needleMatrix);
	sPuncture puncture;
	puncture.position = toolTipPoint;
	puncture.direction = Vector3f(needleMatrix[2], needleMatrix[6], needleMatrix[10]);
	puncture.handle = handle;
	puncture.name = simGetObjectName(handle);
	puncture.penetration_length = punctureLength(puncture);
	full_penetration_length += puncture.penetration_length;
	setUnRespondable(handle);
	punctures.push_back(puncture);
	puncture.printPuncture(true);
}

/**
* @brief Check which contacts will result in a puncture
*/
void checkContacts()
{

	lwr_tip_engine_force_magnitude = 0.0;
	lwr_tip_enging_force.setZero();
	for (int a = 0; a<20; a++)
	{
		simInt contactHandles[2];
		simFloat contactInfo[6];
		simGetContactInfo(sim_handle_all, needleHandle , a, contactHandles, contactInfo);
		if (contactHandles[1] < 1000)
		{
			Vector3f force = simContactInfo2EigenForce(contactInfo);
			float force_magnitude;
			if (use_only_z_force_on_engine)
				force_magnitude = generalForce2NeedleTipZ(force);
			else
				force_magnitude = force.norm();

			int respondableValue;
			simGetObjectIntParameter(contactHandles[1], RESPONDABLE, &respondableValue);
			if (respondableValue != 0 && simGetObjectParent(contactHandles[1]) == phantomHandle)
			{
				lwr_tip_engine_force_magnitude += force_magnitude;
				lwr_tip_enging_force += simContactInfo2EigenForce(contactInfo);
			}

				
			// Here we are supposed to use simGetObjectName to use K(), but there is something weird with the sim function that makes v-rep crash.
			if (force_magnitude > constant_puncture_threshold && respondableValue != 0 && simGetObjectParent(contactHandles[1]) == phantomHandle) {
				addPuncture(contactHandles[1]);
				std::cout << "Force magnitude: " << force_magnitude << std::endl;
			}


		}
	}
}

/**
* @brief Sign function
*/
float sgn(float x) {
	if (x > 0) return 1.0;
	if (x < 0) return -1.0;
	return 0.0;
}

/**
* @brief Model external forces that act upon the needle. Updates f_ext_magnitude and f_ext
* @param model: string describing the model that should be used for modeling the forces
*/
void modelExternalForces(std::string model) {

	if (model == "kelvin-voigt") {
		f_ext_magnitude = kelvinVoigtModel();
	}
	else if (model == "karnopp")
	{
		f_ext_magnitude = karnoppModel() * 0.1;
	}
	else
	{
		std::cout << "No valid friction model was chosen, automatically set Kelvin-Voigt" << std::endl;
		f_ext_magnitude = kelvinVoigtModel();
	}
	f_ext_magnitude *= model_force_scalar;
	// Obtain the forces in the z direction in the reference frame of the lwr needle tip.
	// Add the z force to the magnitude of the forces
	if (use_only_z_force_on_engine)
		f_ext_magnitude += generalForce2NeedleTipZ(lwr_tip_enging_force) * engine_force_scalar;
	else
		f_ext_magnitude += lwr_tip_enging_force.norm() * engine_force_scalar;
	// Get direction of the dummy so that the forces get distributed on all the axis. (They did this in the other project, but is this correct?)
	// Shouldn't we rather map all the calculated forces onto the z direction of the needle? The other directions should be handled by the virtual fixture.
	float dummy_object_matrix[12];
	simGetObjectMatrix(dummyHandle, -1, dummy_object_matrix);
	Vector3f dummy_dir = simObjectMatrix2EigenDirection(dummy_object_matrix);
	f_ext = f_ext_magnitude * dummy_dir; // Should we normalize dir?				Peter: Multiplying with dummy dir creates equal force in all directions of the dummy. Is this right?
}

/**
* @brief Distance between two points
* @param point1: point 1
* @param point2: point 2
* @return distance between the points
*/
float distance3d(Vector3f point1, Vector3f point2) {
	return (point1 - point2).norm();
}

/**
* @brief Get the magnitude of a sim velocity vector.
* @param velocity: A pointer to an array of 3 values representing the velocity.
* @return the velocity magnitude
*/
float getVelocityMagnitude(simFloat* velocity) {
	return std::sqrt(std::pow(velocity[0], 2)
		+ std::pow(velocity[1], 2)
		+ std::pow(velocity[2], 2));
}


void setForceGraph()
{
	simSetGraphUserData(extForceGraphHandle, "measured_F", f_ext_magnitude);
	float objectMatrixRefFrame[12];
	simGetObjectMatrix(lwrTipHandle, -1, objectMatrixRefFrame);
	Vector3f extf = changeBasis(objectMatrixRefFrame, lwr_tip_enging_force);
	simSetGraphUserData(needleForceGraphHandle, "x", extf(0));
	simSetGraphUserData(needleForceGraphHandle, "y", extf(1));
	simSetGraphUserData(needleForceGraphHandle, "z", extf(2));
	for (sPuncture puncture : punctures)
	{
		if (puncture.name == "Fat") {
			simSetGraphUserData(extForceGraphHandle, "fat_penetration", puncture.penetration_length);
		}
		else if (puncture.name == "muscle")
		{
			simSetGraphUserData(extForceGraphHandle, "muscle_penetration", puncture.penetration_length);
		}
		else if (puncture.name == "lung")
		{
			simSetGraphUserData(extForceGraphHandle, "lung_penetration", puncture.penetration_length);
		}
		else if (puncture.name == "bronchus")
		{
			simSetGraphUserData(extForceGraphHandle, "bronchus_penetration", puncture.penetration_length);
		}

	}
	simSetGraphUserData(extForceGraphHandle, "full_penetration", full_penetration_length);
}


float karnoppModel()
{
	if (needleVelocity <= -zero_threshold) {
		return full_penetration_length*(C_n*sgn(needleVelocity) + b_n*needleVelocity);
	}
	else if (-zero_threshold < needleVelocity && needleVelocity <= 0) {
		return full_penetration_length * D_n;
	}
	else if (0 < needleVelocity && needleVelocity < zero_threshold) {
		return full_penetration_length * D_p;
	}
	else if (needleVelocity >= zero_threshold) {
		return full_penetration_length * (C_p*sgn(needleVelocity) + b_p*needleVelocity);
	}
	return -1;
}

float B(sPuncture puncture)
{
	if (puncture.name == "Fat")
	{
		return 3.0f * 100.0f;
	}
	else if (puncture.name == "muscle")
	{
		return 3.0f * 100.0f;
	}
	else if (puncture.name == "lung")
	{
		return 3.0f * 100.0f;
	}
	else if (puncture.name == "bone")
	{
		return 30.0f * 100.05;
	}
	else
	{
		return 3.0f * 100.0f;
	}
}

float K(std::string name)
{
	if (name == "Fat")
	{
		return 1.0e-2;
	}
	
	else if (name == "muscle")
	{
		return 1.0e-2;
	}
	else if (name == "lung")
	{
		return 1.0e-2;
	}
	else if (name == "bone")
	{
		return 1.0;
	}
	else
	{
		return 1.0e-2;
	}
}

float kelvinVoigtModel() {
	float f_magnitude = 0.0;
	for (auto puncture_it = punctures.begin(); puncture_it != punctures.end(); puncture_it++)
	{
		f_magnitude += (B(*puncture_it) * puncture_it->penetration_length);
	}
	f_magnitude *= needleVelocity;
	return f_magnitude;
}

Vector3f simObjectMatrix2EigenDirection(const float* objectMatrix)
{
	return Vector3f(objectMatrix[2], objectMatrix[6], objectMatrix[10]);
}


void reactivateTissues()
{
	std::cout << punctures.size() << std::endl;
	for (sPuncture puncture : punctures) {
		simSetObjectIntParameter(puncture.handle, RESPONDABLE, 1);
		std::cout << "Reactivated respondable for object " << simGetObjectName(puncture.handle) << std::endl;
	}
	punctures.clear();
}

Vector3f simContactInfo2EigenForce(const float* contactInfo)
{
	return Vector3f(contactInfo[3], contactInfo[4], contactInfo[5]);
}

Vector3f changeBasis(const float* objectMatrixReferenceFrame, Vector3f vector)
{
	float quat[4];
	simGetQuaternionFromMatrix(objectMatrixReferenceFrame, quat);
	return Quaternionf(quat[0], quat[1], quat[2], quat[3]) * vector;
}

float generalForce2NeedleTipZ(Vector3f force)
{
	float lwr_tip_object_matrix[12];
	simGetObjectMatrix(lwrTipHandle, -1, lwr_tip_object_matrix);
	Vector3f lwr_tip_engine_force_tmp = changeBasis(lwr_tip_object_matrix, lwr_tip_enging_force);
	return lwr_tip_engine_force_tmp.z();
}