	/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "flight_control_sample.hpp"
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <time.h>

extern "C" {
#include <tm_reader.h>
#include <tm_config.h>
//#include <tmr_read_plan.h>
}
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/**
FILE *pf = fopen("position_out.csv", "w");

double whNorth = 75;  // warehouse north in degrees
double whEast = whNorth + 90; // warehouse east (degrees)
double whSouth = whNorth - 180; // warehouse south (degrees)
double whWest = whEast - 180; // warehouse west (degrees)

// North = [0, 1], East = [1, 0], South = [0, -1], West = [-1, 0]
double whNorthVector[2] = {sin(whNorth * DEG2RAD), cos(whNorth * DEG2RAD)}; // unit meter vector, warehouse north
double whEastVector[2] = {cos(whNorth * DEG2RAD), -1 * sin(whNorth * DEG2RAD)};
double whSouthVector[2] = {-1 * whNorthVector[0], -1 * whNorthVector[1]};
double whWestVector[2] = {-1 * whEastVector[0], -1 * whEastVector[1]};

// Define the serial port we're connecting to
e_vbus_index sensor_id = e_vbus1;

typedef struct Pos {
	float x;
	float y;
	float z;
} _Pos;

typedef struct Vel {
	float x;
	float y;
	float z;
} _Vel;

volatile Pos currPos;
volatile Vel currVel;

void getCurrPos(Pos* destPos) {
	destPos->x = currPos.x;
	destPos->y = currPos.y;
	destPos->z = currPos.z;
}

typedef struct droneCoords {
	float x;
	float y;
	float z;
	float yaw;
};

droneCoords currDroneCoords;

bool moveNorth(Vehicle *vehicle, float offsetDesired)
{
  float x = whNorthVector[1] * offsetDesired;
  float y = whNorthVector[0] * offsetDesired;
  moveByPositionOffset(vehicle, x, y, 0, currDroneCoords.yaw);
}

bool moveSouth(Vehicle *vehicle, float offsetDesired)
{
  float x = whSouthVector[1] * offsetDesired;
  float y = whSouthVector[0] * offsetDesired;
  moveByPositionOffset(vehicle, x, y, 0, currDroneCoords.yaw);
}

bool moveEast(Vehicle *vehicle, float offsetDesired)
{
  float x = whEastVector[1] * offsetDesired;
  float y = whEastVector[0] * offsetDesired;
  moveByPositionOffset(vehicle, x, y, 0, currDroneCoords.yaw);
}

bool moveWest(Vehicle *vehicle, float offsetDesired)
{
  float x = whWestVector[1] * offsetDesired;
  float y = whWestVector[0] * offsetDesired;
  moveByPositionOffset(vehicle, x, y, 0, currDroneCoords.yaw);
}

bool turnNorth(Vehicle *vehicle)
{
	currDroneCoords.yaw = whNorth;
	moveByPositionOffset(vehicle, 0, 0, 0, currDroneCoords.yaw);
}

bool turnSouth(Vehicle *vehicle)
{
	currDroneCoords.yaw = whSouth;
	moveByPositionOffset(vehicle, 0, 0, 0, currDroneCoords.yaw);
}

bool turnEast(Vehicle *vehicle)
{
	currDroneCoords.yaw = whEast;
	moveByPositionOffset(vehicle, 0, 0, 0, currDroneCoords.yaw);
}

bool turnWest(Vehicle *vehicle)
{
	currDroneCoords.yaw = whWest;
	moveByPositionOffset(vehicle, 0, 0, 0, currDroneCoords.yaw);
}

/**Callback optimized for velocity, ultrasonic, and motion data. */
/**
int _callback(int data_type, int data_len, char* content) {
	//get position data if motion data collected
	if (e_motion == data_type && NULL != content) {
		motion *m = (motion*) content; // convert content to motion data
		Pos p;
		Vel v;
		p.x = m->position_in_global_x;
		p.y = m->position_in_global_y;
		p.z = m->position_in_global_z;

		v.x = m->velocity_in_global_x;
		v.y = m->velocity_in_global_y;
		v.z = m->velocity_in_global_z;

		currPos.x = p.x;
		currPos.y = p.y;
		currPos.z = p.z;

		currVel.x = v.x;
		currVel.y = v.y;
		currVel.z = v.z;

		fprintf(pf, "%f - %f - %f\n", p.x, p.y, p.z);
		//std::cout << p.x << " " << p.y << " " << std::endl;
	}
	return 0;
}
*/

/*! main
 *
 */
int
main(int argc, char** argv)
{

  currDroneCoords.x = 0;
  currDroneCoords.y = 0;
  currDroneCoords.z = 0;
  currDroneCoords.yaw = whNorth;

  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);


 // reset_config(); //clear previous data subscriptions
  std::cout << "configuring reader...." << std::endl;

  //connect to Guidance, print if error
  int err_code = init_transfer();

  std::cout << "error code: " << err_code << std::endl;
  if (err_code == 0) {
	  std::cout << "Connected to Guidance" << std::endl;
  }
  //:

  //set _callback as the event handler for all Guidance events
  err_code = set_sdk_event_handler(_callback);

  std::cout << "beginning transfer...." << std::endl;
  err_code = start_transfer();
  std::cout << err_code << std::endl;
  std::cout << "Started transfer" << std::endl;

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Monitored Takeoff + Landing                                |"
    << std::endl;
  std::cout
    << "| [b] Monitored Takeoff + DARSHINI  + Landing                    |"
    << std::endl;
  std::cout
    << "| [c] Warehouse coordinate movement command tests                |"
    << std::endl;
  std::cout
    << "| [d] Warehouse coordinate rotation commnd tests                 |"
    << std::endl;
  std::cout
    << "| [e] Warehouse traversal                                        |"
    << std::endl;
  std::cout
    << "| [f] Z axis movement (ascend, descend)                          |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch (inputChar)
  {
    case 'a':
      monitoredTakeoff(vehicle);
      monitoredLanding(vehicle);
      break;
    case 'b':
      monitoredTakeoff(vehicle);
      std::cout << "FORWARD" << std::endl;
      moveByPositionOffset(vehicle, 3, 0, 0, 0);
      std::cout << "rotate" << std::endl;
      moveByPositionOffset(vehicle, 0, 0, 0, 45);
      std::cout << "FORWARD" << std::endl;
      moveByPositionOffset(vehicle, 3, 0, 0, 0);
      std::cout << "rotate" << std::endl;
      moveByPositionOffset(vehicle, 0, 0, 0, 45);
      std::cout << "FORWARD" << std::endl;
      moveByPositionOffset(vehicle, 3, 0, 0, 0);
      std::cout << "rotate" << std::endl;
      moveByPositionOffset(vehicle, 0, 0, 0, 45);
      std::cout << "FORWARD" << std::endl;
      moveByPositionOffset(vehicle, 3, 0, 0, 0);
      std::cout << "rotate" << std::endl;
      moveByPositionOffset(vehicle, 0, 0, 0, 45);
      monitoredLanding(vehicle);
      break;
    case 'c':
      monitoredTakeoff(vehicle);
      std::cout << "Move North" << std::endl;
      moveNorth(vehicle, 5);
      std::cout << "Move East" << std::endl;
      moveEast(vehicle, 5);
      std::cout << "Move West" << std::endl;
      moveWest(vehicle, 5);
      std::cout << "Move South" << std::endl;
      moveSouth(vehicle, 5);
      monitoredLanding(vehicle);
      break;
    case 'd':
      monitoredTakeoff(vehicle);
      std::cout << "Rotate North" << std::endl;
      turnNorth(vehicle);
      std::cout << "Rotate East" << std::endl;
      turnEast(vehicle);
      std::cout << "Rotate South" << std::endl;
      turnSouth(vehicle);
      std::cout << "Rotate West" << std::endl;
      turnWest(vehicle);
      monitoredLanding(vehicle);
      break;
			//std::cout << "Back to home" << std::endl;
      //moveByPositionOffset(vehicle, -6, 0, 0, 0);
      //monitoredLanding(vehicle);
    case 'e':
      monitoredTakeoff(vehicle);
      // Orient w warehouse north (arbitrary decision)
      turnNorth(vehicle);
      // Move to Aisle 1
      moveEast(vehicle, 2.1);
      moveSouth(vehicle, 3.1);
      // Now at start of Aisle 1
      // Traverse Aisle 1
      moveSouth(vehicle, 5);
      // Start moving back to start location
      moveNorth(vehicle, 5 + 3.1);
      moveWest(vehicle, 2.1);
      // Reached start location
      monitoredLanding(vehicle);
      break;
    case 'f':
      monitoredTakeoff(vehicle);
      moveByPositionOffset(vehicle, 0, 0, 3, whSouth);
      moveByPositionOffset(vehicle, 0, 0, -1, whNorth);
      monitoredLanding(vehicle);
    default:
      break;
  }
  fclose(pf);
  err_code = release_transfer();
  return 0;
}
