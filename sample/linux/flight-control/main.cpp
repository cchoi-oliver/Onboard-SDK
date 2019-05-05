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
#include <vector>

extern "C" {
#include <tm_reader.h>
#include <tm_config.h>
#include <tmr_read_plan.h>
}

using std::vector;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

void read_rfid(TMR_Reader * rp, uint32_t timeout, vector<char*>& buffer) {
	TMR_TagReadData ** tags;
	TMR_ReadPlan plan;
	
	int32_t c = 500 * sizeof(TMR_TagReadData);
	int32_t *tagCount = &c;

	//set up data handling
	uint8_t antennaList[1] = {2};
	TMR_RP_init_simple(&plan, 1, antennaList, TMR_TAG_PROTOCOL_GEN2, 1000);
	TMR_paramSet(rp, TMR_PARAM_READ_PLAN, &plan);

	TMR_Status err = TMR_read(rp, timeout, NULL);
	
	if (*tagCount <  0) {
		fprintf(stderr, "could not read tags  %d\n", *tagCount);
	} else {
		
		while (TMR_hasMoreTags(rp) == TMR_SUCCESS) {
			TMR_TagReadData tag_data;
			
			TMR_getNextTag(rp, &tag_data);
			char data[255];

			TMR_bytesToHex(tag_data.tag.epc, tag_data.tag.epcByteCount, data);
			buffer.push_back(data);
		}
	}
}


/*! main
 *
 */
int
main(int argc, char** argv)
{
  /*TMR_Reader r;
  TMR_Reader * rp = &r;

  TMR_Status err = TMR_create(rp, "tmr:///dev/ttyACM0");
	
  if (err != TMR_SUCCESS) {
	fprintf(stderr, "cannot connect reader\n");
  } else {
	printf("reader created\n");
  }
	
  //connect the reader
  err = TMR_connect(rp);

  if (err == TMR_SUCCESS) {
	printf("reader connected\n");
  }

  vector<char*> buffer;
  read_rfid(rp, 1000, buffer);
  printf("reader destroyed\n");
  TMR_destroy(rp);
*/
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
  std::cout << "configuring guidance...." << std::endl;

  //connect to Guidance, print if error
  int err_code = init_transfer();

  std::cout << "error code: " << err_code << std::endl;
  if (err_code == 0) {
	  std::cout << "Connected to Guidance" << std::endl;
  }

  //select data types we want
  select_obstacle_distance();
  select_motion();


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
    << "| [d] Warehouse coordinate rotation command tests                |"
    << std::endl;
  std::cout
    << "| [e] Warehouse traversal                                        |"
    << std::endl;
  std::cout
    << "| [f] Z axis movement (ascend, descend)                          |"
    << std::endl;
  std::cout
    << "| [g] get ganster with dimitri                                   |"
    << std::endl;

  char inputChar = 'g';
  std::cin >> inputChar;

  switch (inputChar)
  {
    case 'a':
      monitoredTakeoff(vehicle);
      monitoredLanding(vehicle);
      break;
    case 'b':
      std::cout << "TAKING OFF" << std::endl;
      monitoredTakeoff(vehicle);
      std::cout << "Rotating Southward" << std::endl;
      turnSouth(vehicle);
      std::cout << "Moving higher" << std::endl;
      moveByPositionOffset(vehicle, 0, 0, 1.5, whSouth);
      std::cout << "Move forward (1)" << std::endl;
      moveSouth(vehicle, 1.5); 
      std::cout << "Move forward (2)" << std::endl;
      moveSouth(vehicle, 1.5); 
      std::cout << "Move forward (3)" << std::endl;
      moveSouth(vehicle, 1.5); 
      std::cout << "Move forward (4)" << std::endl;
      moveSouth(vehicle, 1.5);
      std::cout << "Move forward (5)" << std::endl;
      moveSouth(vehicle, 1.5);
      std::cout << "LANDING" << std::endl;
      monitoredLanding(vehicle);
      break;
    case 'c':
      monitoredTakeoff(vehicle);
      turnNorth(vehicle);
      std::cout << "Move North" << std::endl;
      moveNorth(vehicle, 1);
      std::cout << "Move East" << std::endl;
      moveEast(vehicle, 1);
      std::cout << "Move West" << std::endl;
      moveWest(vehicle, 1);
      std::cout << "Move South" << std::endl;
      moveSouth(vehicle, 1);
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
      moveByPositionOffset(vehicle, 0, 0, 1.5, whSouth);
      moveSouth(vehicle, 1);
      monitoredLanding(vehicle);
      break;
    case 'g':
      std::cout << "TAKING OFF" << std::endl;
      monitoredTakeoff(vehicle);
      std::cout<<"take off complete"<<std::endl;

      std::cout << "Rotating Southward" << std::endl;
      turnSouth(vehicle);
      std::cout << "Moving higher" << std::endl;
      moveByPositionOffset(vehicle, 0, 0, 1.5, whSouth);
      std::cout<<"getting next level high" <<std::endl;
      moveByPositionOffset(vehicle,0,0,2,whSouth);

      std::cout << "Move forward (1)" << std::endl;
      moveSouth(vehicle, 15); 
      /*std::cout << "Move forward (2)" << std::endl;
      moveSouth(vehicle, 1.5); 
      std::cout << "Move forward (3)" << std::endl;
      moveSouth(vehicle, 1.5); 
      std::cout << "Move forward (4)" << std::endl;
      moveSouth(vehicle, 1.5);
      std::cout << "Move forward (5)" << std::endl;
      moveSouth(vehicle, 1.5);*/
      std::cout << "LANDING" << std::endl;
      monitoredLanding(vehicle);


    case 'r': {
	/*TMR_Reader r;
  	TMR_Reader * rp = &r;

  	TMR_Status err = TMR_create(rp, "tmr:///dev/ttyACM0");
	
  	if (err != TMR_SUCCESS) {
		fprintf(stderr, "cannot connect reader\n");
  	} else {
		printf("reader created\n");
 	}
	
	//connect the reader
	err = TMR_connect(rp);

	if (err == TMR_SUCCESS) {
		printf("reader connected\n");
	}

	read_rfid(rp, 250);
	
	TMR_destroy(rp);
	break;*/
    }
    default:
      break;
  }
  fclose(pf);
  fclose(yf);
  err_code = release_transfer();
  std::cout << "ENDING PROGRAM" << std::endl;

  return 0;
}
