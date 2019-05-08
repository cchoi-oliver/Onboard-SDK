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
#include <thread>
#include <vector>

extern "C" {
#include <tm_reader.h>
#include <tm_config.h>
#include <tmr_read_plan.h>
}

using std::vector;
using std::cout;
using std::thread;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using std::endl;

FILE* rf = fopen("rfid.txt", "w");

/** Function for reading from RFID sensor.
  * stores output in buffer of C Strings. */
void read_rfid(TMR_Reader * rp, uint32_t timeout) {
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
			fprintf(stdout, "%s\n", data);
			fprintf(rf, "%s\n", data);
		}
	}
}

/** Function to use as target for separate RFID thread. */
void continuous_read(TMR_Reader * rp) {
	fprintf(rf, "BUFFER BEGIN:\n");

	while (1) {
		cout << "reading\n";
		read_rfid(rp, 4000);
		sleep(3);
	}

}

/*! main
 *
 */
int
main(int argc, char** argv)
{
  TMR_Reader r;
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
  std::cout
    << "| [h] altitude control loop                                      |"
    << std::endl;
  std::cout
    << "| [i] print z coordinates                                        |"
    << std::endl;
  std::cout
    << "| [j] Traverse Aisle (South)                                     |"
    << std::endl;
	std::cout
		<< "| [k] Traverse Aisle while scanning RFID tags									   |"
		<< std::endl;
std::cout
	<<"| [l] Traverse Aisle with guidance control loop                |"
	<< std::endl;

  char inputChar = 'g';
  std::cin >> inputChar;
  fprintf(rf, "TEST\n");
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
    case 'g'://attempting multiple movement commands in a row doesn't work
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
      moveSouth(vehicle, 5);
      std::cout << "sleeping\n";
      sleep(2000);
      std::cout << "ending first sleep\n";
      std::cout << "Move forward (2)" << std::endl;
      moveSouth(vehicle, 1.5);
      sleep(2000);
      cout << "sleeping 2\n";
      std::cout << "Move forward (3)" << std::endl;
      moveSouth(vehicle, 1.5);
      sleep(2000);
      cout << "sleeping 3\n";
      std::cout << "Move forward (4)" << std::endl;
      moveSouth(vehicle, 1.5);
      sleep(2000);
      cout << "sleeping 4\n";
      std::cout << "Move forward (5)" << std::endl;
      moveSouth(vehicle, 1.5);
      sleep(2000);
      cout << "sleeping 5\n";
      std::cout << "LANDING" << std::endl;
      monitoredLanding(vehicle);
      break;
    case 'h'://to test moving up and down
      {
      std::cout << "TAKING OFF" << std::endl;
      monitoredTakeoff(vehicle);
      std::cout<<"take off complete"<<std::endl;

      std::cout << "Rotating Southward" << std::endl;
      turnSouth(vehicle);

	std::cout << "Moving higher" << std::endl;
      	moveByPositionOffset(vehicle, 0, 0, 1.5, whSouth);

      char command = '\0';
      while(std::cin>>command){
	      //start off taking in the next direction
	     std::cout << "type u or d" <<endl;
	     std::cin >> command;
		switch (command){
			case('u'):
				std::cout << "Moving higher" << std::endl;
      				moveByPositionOffset(vehicle, 0, 0, 1.5, whSouth);
				break;
			case('d'):
				std::cout<<"Moving lower"<<std::endl;
				moveByPositionOffset(vehicle, 0, 0, -1.5, whSouth);
				break;

			case('f'):
				std::cout << "moving forward\n";
				moveSouth(vehicle, 2.5);
				break;
		}

      }
      }
      break;
    case ('i')://more accurate up and down
      {
      //thread* rfid_thread_pointer;
	//rfid_thread_pointer = new thread(continuous_read, rp);
      std::cout << "TAKING OFF" << std::endl;
      monitoredTakeoff(vehicle);
      std::cout<<"take off complete"<<std::endl;

      std::cout << "Rotating Southward" << std::endl;
      turnSouth(vehicle);

	std::cout << "Moving higher" << std::endl;
      	moveByPositionOffset(vehicle, 0, 0, 1.5, whSouth);

      float command = 0;
      float z=0;
      float offset=0;
      int sure=0;
      while(1){
	      //start off taking in the next direction
	     z=currPos.z*-1;
	     std::cout << "current Height is: "<< z<<std::endl;
	     std::cout <<"please input the new distance you would like to fly to"<<std::endl;
	     std::cin >> command;
	     command=command;
	     //command holds new vlaue z holds old value
	     if (command < 0) {
	     	break;
	     }
	     if (command >6){
		while (1) {
			std::cout<<"you fucked up"<<std::endl;
		}
	     }
	     offset=command-z;
	     std::cout <<"calculated offset1= "<<offset<<std::endl;
	     std::cout<<"if the offset is ok type 1"<<std::endl;
	     std::cin>>sure;
		
	
	     if (sure ==1) {
	     	while (abs(offset)>.10){
	     		std::cout <<"calculated offset2= "<<offset<<std::endl;
	     		std::cout<<"changing altitude"<<std::endl;
	     		moveByPositionOffset(vehicle, 0, 0, offset, whSouth);
			z=currPos.z*-1;
			offset=command-z;
	     	}
	     }else {
		     return 69;
	     }

      }
      }
      break;



      std::cout<<"trying to print from current position struct"<<std::endl;
      std::cout<<currPos.z<<std::endl;
      break;
case 'l': {//set height and then set traversal distance
/*		  std::cout<<"initializing reader"<<std::endl;
	thread* rfid_thread_pointer;
	rfid_thread_pointer = new thread(continuous_read, rp);	  
	sleep(0.25);
	std::cout<<"rfid sucesfully on"<<std::endl;
*/
	std::cout << "TAKING OFF" << std::endl;
      monitoredTakeoff(vehicle);
      std::cout<<"take off complete"<<std::endl;

      std::cout << "Rotating Southward" << std::endl;
      turnSouth(vehicle);

	std::cout << "Moving higher" << std::endl;
      	moveByPositionOffset(vehicle, 0, 0, 1.5, whSouth);

      float command = 0;
      float z=0;
      float offset=0;
      float sure=0;
	      //start off taking in the next direction
	     z=currPos.z*-1;
	     std::cout << "current Height is: "<< z<<std::endl;
	     std::cout <<"please input the new distance you would like to fly to"<<std::endl;
	     std::cin >> command;
	     command=command;
	     //command holds new vlaue z holds old value
	     if (command < 0) {
	     	return 8;
	     }
	     if (command >6){
		while (1) {
			std::cout<<"you fucked up"<<std::endl;
		}
	     }
	     offset=command-z;
	     
	     std::cout <<"calculated offset1= "<<offset<<std::endl;
	     std::cout<<"if the offset is ok type 1"<<std::endl;
	     std::cin>>sure;
	     if (sure==1){
	     while (offset>.18){
	     	std::cout <<"calculated offset= "<<offset<<std::endl;
	     	std::cout<<"sending traversal command with new offset"<<std::endl;
	     	moveByPositionOffset(vehicle, 0, 0, offset, whSouth);
		std::cout<<"move command executed"<<std::endl;
		z=currPos.z*-1;
		offset=command-z;
	     }
	     }
	     else {
		return 69;
	     }

	     std::cout<<"please input the distance down the ailse you would like to traverse"<<std::endl;
	     float t_dist=0;
	     std::cin>>t_dist;
	     std::cout<<"traversal distance: "<<t_dist<<std::endl;
		std::cout<<"are you sure? type 1 if you is"<<std::endl;
		std::cin>>sure;
	     //calculate initial position
	float xf=0;
	float yf=0;
	float displacement=0;
	float xi=currPos.x;
	float yi=currPos.y;
	float dist_left=t_dist;
	if (sure == 1){
	while (dist_left>0){
      		std::cout << "Traversing" <<std::endl;
      		traverseAisleSouth(vehicle,dist_left , false);
      		//calculate new postions
      		xf=currPos.x;
      		yf=currPos.y;
      		//calulate distance traveled
		displacement=sqrt(pow((xf-xi),2)+pow((yf-yi),2));
		dist_left=t_dist-displacement;
	}
	}else {
		return 69;
	}
      
      std::cout << "Landing" << std::endl;
      monitoredLanding(vehicle);
      break;
	      }

    case 'j'://chang traversal base case
      std::cout << "STARTING " << std::endl;
      monitoredTakeoff(vehicle);
      moveByPositionOffset(vehicle, 0, 0, 0.6, whSouth);
      std::cout <<"takeoff done \n";
      std::cout << "Traversing" <<std::endl;
      traverseAisleSouth(vehicle, 20, false);
      std::cout << "Landing" << std::endl;
      monitoredLanding(vehicle);
      break;
    case 'k'://christian rfid case
	std::cout << "Starting\n";
	//start rfid thread
	thread* rfid_thread_pointer;
	rfid_thread_pointer = new thread(continuous_read, rp);
	
	//char command = '\0';
	sleep(30);
	//run drone movement commands
	//monitoredTakeoff(vehicle);
	//turnSouth(vehicle);
	//moveSouth(vehicle, 5);
	break;
  }

  std::cout << "Destroying reader\n";
  err = TMR_destroy(rp);

	fclose(pf);
  fclose(yf);
  fclose(rf);
  err_code = release_transfer();
  std::cout << "ENDING PROGRAM" << std::endl;

  return 0;
}
