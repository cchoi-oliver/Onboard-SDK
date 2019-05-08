/*! @file flight_control_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
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
#include <vector>

using std::vector;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

FILE *pf = fopen("position_out.csv", "w");
FILE *yf = fopen("yaw_out.csv", "w");

double whNorth = 100;  // warehouse north in degrees //increasing shifts clockwise
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

volatile Pos currPos;
volatile Vel currVel;
volatile Dists currDists;

void getCurrPos(Pos* destPos) {
	destPos->x = currPos.x;
	destPos->y = currPos.y;
	destPos->z = currPos.z;
}

void getCurrDists(Dists* destDists) {
	destDists->d = currDists.d;
	destDists->f = currDists.f;
	destDists->r = currDists.r;
	destDists->b = currDists.b;
	destDists->l = currDists.l;
}

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

bool traverseAisleNorth(Vehicle *vehicle, float offsetDesired, bool toJunction)
{
	turnNorth(vehicle);
	float x = whNorthVector[1] * offsetDesired;
  float y = whNorthVector[0] * offsetDesired;
	traverseAisle(vehicle, x, y, 0, currDroneCoords.yaw, toJunction);
}

bool traverseAisleSouth(Vehicle *vehicle, float offsetDesired, bool toJunction)
{
	turnSouth(vehicle);
	float x = whSouthVector[1] * offsetDesired;
  float y = whSouthVector[0] * offsetDesired;
  traverseAisle(vehicle, x, y, 0, currDroneCoords.yaw, toJunction);
}

typedef struct Data {
	obstacle_distance o;
	Pos p;
	Vel v;
} _Data;

Data currData;

/**Callback optimized for velocity, ultrasonic, and motion data. */
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

		currData.p = p;
		currData.v = v;

		//fprintf(pf, "%f, %f, %f\n", p.x, p.y, p.z);
		//std::cout << p.x << " " << p.y << " " << p.z << std::endl;
	}
	//std::cout << "obstacle data up next!\n";
	if ( e_obstacle_distance == data_type && NULL != content ){
		obstacle_distance *oa = (obstacle_distance*)content;
		currData.o = *oa;
		currDists.d = oa->distance[0];
		currDists.f = oa->distance[1];
		currDists.r = oa->distance[2];
		currDists.b = oa->distance[3];
		currDists.l = oa->distance[4];
		//printf( "obstacle distance:" );
		for ( int i = 0; i < CAMERA_PAIR_NUM; ++i ) {
			//printf( " %u\n",  oa->distance[i] );
			if (oa->distance[i] < 150) {
				//printf("DETECTED OBSTACLE on sensor: %d\n", i);
				//printf("obstacle distance: %d\n", oa->distance[i]);
			}
		}
			//printf( "frame index:%d,stamp:%d\n", oa->frame_index, oa->time_stamp );

		//print to file
	//	std::cout << "obstacle distance " << oa->distance[0] << " " << oa->distance[1] << " " << oa->distance[2] << " " << oa->distance[3] << std::endl;
		//fprintf(stdout,  "obstacle distnace: %u, %u, %u, %u, %u\n", oa->distance[0], oa->distance[1], oa->distance[2], oa->distance[3], oa->distance[4]);

	}

	return 0;
}

/** Function for handling objects within our danger zone. */
bool avoid(Vehicle *vehicle) {
	return true;
}

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool
monitoredTakeoff(Vehicle* vehicle, int timeout)
{
  //@todo: remove this once the getErrorCode function signature changes
  char func[50];
  int  pkgIndex;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    }

    // Telemetry: Subscribe to flight status and mode at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                  TOPIC_STATUS_DISPLAYMODE };
    int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, timeout);
      return false;
    }
  }

  // Start takeoff
  ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(timeout);
  if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(takeoffStatus, func);
    return false;
  }

  // First check: Motors started
  int motorsNotStarted = 0;
  int timeoutCycles    = 20;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::ON_GROUND &&
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted == timeoutCycles)
    {
      std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
      // Cleanup
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
        vehicle->subscribe->removePackage(0, timeout);
      }
      return false;
    }
    else
    {
      std::cout << "Motors spinning...\n";
    }
  }
  else if (vehicle->isLegacyM600())
  {
    while ((vehicle->broadcast->getStatus().flight <
            DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND) &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted < timeoutCycles)
    {
      std::cout << "Successful TakeOff!" << std::endl;
    }
  }
  else // M100
  {
    while ((vehicle->broadcast->getStatus().flight <
            DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted < timeoutCycles)
    {
      std::cout << "Successful TakeOff!" << std::endl;
    }
  }

  // Second check: In air
  int stillOnGround = 0;
  timeoutCycles     = 110;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::IN_AIR &&
           (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround == timeoutCycles)
    {
      std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                   "motors are spinning."
                << std::endl;
      // Cleanup
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
        vehicle->subscribe->removePackage(0, timeout);
      }
      return false;
    }
    else
    {
      std::cout << "Ascending...\n";
    }
  }
  else if (vehicle->isLegacyM600())
  {
    while ((vehicle->broadcast->getStatus().flight <
            DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround < timeoutCycles)
    {
      std::cout << "Aircraft in air!" << std::endl;
    }
  }
  else // M100
  {
    while ((vehicle->broadcast->getStatus().flight !=
            DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround < timeoutCycles)
    {
      std::cout << "Aircraft in air!" << std::endl;
    }
  }

  // Final check: Finished takeoff
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
    {
      sleep(1);
    }

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
          vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE)
      {
        std::cout << "Successful takeoff!\n";
      }
      else
      {
        std::cout
          << "Takeoff finished, but the aircraft is in an unexpected mode. "
             "Please connect DJI GO.\n";
        vehicle->subscribe->removePackage(0, timeout);
        return false;
      }
    }
  }
  else
  {
    float32_t                 delta;
    Telemetry::GlobalPosition currentHeight;
    Telemetry::GlobalPosition deltaHeight =
      vehicle->broadcast->getGlobalPosition();

    do
    {
      sleep(4);
      currentHeight = vehicle->broadcast->getGlobalPosition();
      delta         = fabs(currentHeight.altitude - deltaHeight.altitude);
      deltaHeight.altitude = currentHeight.altitude;
    } while (delta >= 0.009);

    std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";
  }

  // Cleanup
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return true;
}

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there.
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control.
!*/
bool
moveByPositionOffset(Vehicle *vehicle, float xOffsetDesired,
                     float yOffsetDesired, float zOffsetDesired,
                     float yawDesired, float posThresholdInM,
                     float yawThresholdInDeg)
{
  // Set timeout: this timeout is the time you allow the drone to take to finish
  // the
  // mission
  int responseTimeout              = 1;
  int timeoutInMilSec              = 10000;
  int controlFreqInHz              = 50; // Hz
  int cycleTimeInMs                = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
  int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
  int pkgIndex;

  Pos tempPos; // Position struct to hold current value of currPos (global)

  //@todo: remove this once the getErrorCode function signature changes
  char func[50];

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    }

    // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
    // Hz
    pkgIndex                  = 0;
    int       freq            = 50;
    TopicName topicList50Hz[] = { TOPIC_QUATERNION, TOPIC_GPS_FUSED };
    int       numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus =
      vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }

    // Also, since we don't have a source for relative height through subscription,
    // start using broadcast height
    if (!startGlobalPositionBroadcast(vehicle))
    {
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }
  }

  // Wait for data to come in
  sleep(1);

  // Get data
  getCurrPos(&tempPos);

  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition currentBroadcastGP;
  Telemetry::GlobalPosition originBroadcastGP;

  // Convert position offset from first position to local coordinates
  Telemetry::Vector3f localOffset;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    originSubscriptionGPS  = currentSubscriptionGPS;
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentSubscriptionGPS),
                             static_cast<void*>(&originSubscriptionGPS));

    // Get the broadcast GP since we need the height for zCmd
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
  }
  else
  {
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    originBroadcastGP  = currentBroadcastGP;
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentBroadcastGP),
                             static_cast<void*>(&originBroadcastGP));
  }

  // Get initial offset. We will update this in a loop later.
  double xOffsetRemaining = xOffsetDesired - localOffset.x;
  double yOffsetRemaining = yOffsetDesired - localOffset.y;
  double zOffsetRemaining = zOffsetDesired - (-localOffset.z); // TODO: change back if needed
  //double zOffsetRemaining = zOffsetDesired;// - (-localOffset.z); // TODO: change back if needed

  double xOffsetRem = xOffsetDesired - tempPos.x;
  double yOffsetRem = yOffsetDesired - tempPos.y;
  double zOffsetRem = zOffsetDesired - (-tempPos.z);

  // Conversions
  double yawDesiredRad     = DEG2RAD * yawDesired;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

  //! Get Euler angle

  // Quaternion retrieved via subscription
  Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
  // Quaternion retrieved via broadcast
  Telemetry::Quaternion broadcastQ;

  double yawInRad;
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z / DEG2RAD;
  }
  else
  {
    broadcastQ = vehicle->broadcast->getQuaternion();
    yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;
  }

  int   elapsedTimeInMs     = 0;
  int   withinBoundsCounter = 0;
  int   outOfBounds         = 0;
  int   brakeCounter        = 0;
  int   speedFactor         = 2;
  float xCmd, yCmd, zCmd;
  // There is a deadband in position control
  // the z cmd is absolute height
  // while x and y are in relative
  float zDeadband = 0.12;

  if (vehicle->isM100() || vehicle->isLegacyM600())
  {
    zDeadband = 0.12 * 10;
  }

  /*! Calculate the inputs to send the position controller. We implement basic
   *  receding setpoint position control and the setpoint is always 1 m away
   *  from the current position - until we get within a threshold of the goal.
   *  From that point on, we send the remaining distance as the setpoint.
   */
  if (xOffsetDesired > 0)
    xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
  else if (xOffsetDesired < 0)
    xCmd =
      (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
  else
    xCmd = 0;

  if (yOffsetDesired > 0)
    yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
  else if (yOffsetDesired < 0)
    yCmd =
      (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
  else
    yCmd = 0;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    zCmd = currentBroadcastGP.height + zOffsetDesired; //Since subscription cannot give us a relative height, use broadcast.
  }
  else
  {
    //zCmd = zOffsetDesired;
    zCmd = currentBroadcastGP.height + zOffsetDesired;
    //zCmd = (-tempPos.z) + zOffsetDesired;
  }
  int printCounter = 0;
  //! Main closed-loop receding setpoint position control
  while (elapsedTimeInMs < timeoutInMilSec)
  {
    if (printCounter++ % 3 == 0) {
      //std::cout << tempPos.x << " " << tempPos.y << " " << tempPos.x << std::endl;
    }
    fprintf(yf, "%f\n", yawInRad);
    vehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd,
                                         yawDesiredRad / DEG2RAD);

    usleep(cycleTimeInMs * 1000);
    elapsedTimeInMs += cycleTimeInMs;

    //! Get current position in required coordinates and units
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
      yawInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;
      currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
      localOffsetFromGpsOffset(vehicle, localOffset,
                               static_cast<void*>(&currentSubscriptionGPS),
                               static_cast<void*>(&originSubscriptionGPS));

      // Get the broadcast GP since we need the height for zCmd
      currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    }
    else
    {
      broadcastQ         = vehicle->broadcast->getQuaternion();
      yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
      currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
      localOffsetFromGpsOffset(vehicle, localOffset,
                               static_cast<void*>(&currentBroadcastGP),
                               static_cast<void*>(&originBroadcastGP));
      getCurrPos(&tempPos);
    }

    //! See how much farther we have to go
    xOffsetRemaining = xOffsetDesired - localOffset.x;
    yOffsetRemaining = yOffsetDesired - localOffset.y;
    zOffsetRemaining = zOffsetDesired - (-localOffset.z);

    xOffsetRem = xOffsetDesired - tempPos.x;
    yOffsetRem = yOffsetDesired - tempPos.y;
    zOffsetRem = zOffsetDesired - (-tempPos.z);

    //! See if we need to modify the setpoint
    //if (std::abs(xOffsetRemaining) < speedFactor)
    if (std::abs(xOffsetRem) < speedFactor)
    {
      //xCmd = xOffsetRemaining;
      xCmd = xOffsetRem;
    }
    //if (std::abs(yOffsetRemaining) < speedFactor)
    if (std::abs(yOffsetRem) < speedFactor)
    {
      //yCmd = yOffsetRemaining;
      yCmd = yOffsetRem;
    }

    //if (vehicle->isM100() && std::abs(xOffsetRemaining) < posThresholdInM &&
    //    std::abs(yOffsetRemaining) < posThresholdInM &&
    //    std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    if (vehicle->isM100() && std::abs(xOffsetRem) < posThresholdInM &&
        std::abs(yOffsetRem) < posThresholdInM &&
        std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    }
    else if (std::abs(xOffsetRemaining) < posThresholdInM &&
             std::abs(yOffsetRemaining) < posThresholdInM &&
						 std::abs(zOffsetRem) < zDeadband &&
						 //std::abs(zOffsetRemaining) < zDeadband &&
             std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    }
    else
    {
      if (withinBoundsCounter != 0)
      {
        //! 2. Start incrementing an out-of-bounds counter
        outOfBounds += cycleTimeInMs;
      }
    }
    //! 3. Reset withinBoundsCounter if necessary
    if (outOfBounds > outOfControlBoundsTimeLimit)
    {
      withinBoundsCounter = 0;
      outOfBounds         = 0;
    }
    //! 4. If within bounds, set flag and break
    if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
    {
      break;
    }
  }

  //! Set velocity to zero, to prevent any residual velocity from position
  //! command
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (brakeCounter < withinControlBoundsTimeReqmt)
    {
      vehicle->control->emergencyBrake();
      usleep(cycleTimeInMs * 10);
      brakeCounter += cycleTimeInMs;
    }
  }

  /*if (elapsedTimeInMs >= timeoutInMilSec)
  {
    std::cout << "Task timeout!\n";
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      ACK::ErrorCode ack =
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      if (ACK::getError(ack))
      {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
    }
    return ACK::FAIL;
  }*/

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return ACK::SUCCESS;
}

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool
monitoredLanding(Vehicle* vehicle, int timeout)
{
  //@todo: remove this once the getErrorCode function signature changes
  char func[50];
  int  pkgIndex;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    }

    // Telemetry: Subscribe to flight status and mode at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                  TOPIC_STATUS_DISPLAYMODE };
    int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, timeout);
      return false;
    }
  }

  // Start landing
  ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
  if (ACK::getError(landingStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(landingStatus, func);
    return false;
  }

  // First check: Landing started
  int landingNotStarted = 0;
  int timeoutCycles     = 20;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           landingNotStarted < timeoutCycles)
    {
      landingNotStarted++;
      usleep(100000);
    }
  }
  else if (vehicle->isM100())
  {
    while (vehicle->broadcast->getStatus().flight !=
             DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING &&
           landingNotStarted < timeoutCycles)
    {
      landingNotStarted++;
      usleep(100000);
    }
  }

  if (landingNotStarted == timeoutCycles)
  {
    std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
    // Cleanup before return
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
      std::cout << "Error unsubscribing; please restart the drone/FC to get "
                   "back to a clean state.\n";
    }
    return false;
  }
  else
  {
    std::cout << "Landing...\n";
  }

  // Second check: Finished landing
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
             VehicleStatus::FlightStatus::IN_AIR)
    {
      sleep(1);
    }

    if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_P_GPS ||
        vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_ATTITUDE)
    {
      std::cout << "Successful landing!\n";
    }
    else
    {
      std::cout
        << "Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n";
      ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
      if (ACK::getError(ack))
      {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
      return false;
    }
  }
  else if (vehicle->isLegacyM600())
  {
    while (vehicle->broadcast->getStatus().flight >
           DJI::OSDK::VehicleStatus::FlightStatus::STOPED)
    {
      sleep(1);
    }

    Telemetry::GlobalPosition gp;
    do
    {
      sleep(2);
      gp = vehicle->broadcast->getGlobalPosition();
    } while (gp.altitude != 0);

    if (gp.altitude != 0)
    {
      std::cout
        << "Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n";
      return false;
    }
    else
    {
      std::cout << "Successful landing!\n";
    }
  }
  else // M100
  {
    while (vehicle->broadcast->getStatus().flight ==
           DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
    {
      sleep(1);
    }

    Telemetry::GlobalPosition gp;
    do
    {
      sleep(2);
      gp = vehicle->broadcast->getGlobalPosition();
    } while (gp.altitude != 0);

    if (gp.altitude != 0)
    {
      std::cout
        << "Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n";
      return false;
    }
    else
    {
      std::cout << "Successful landing!\n";
    }
  }

  // Cleanup
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return true;
}

// Aisle traversal command
bool
traverseAisle(Vehicle *vehicle, float xTarget, float yTarget, float zTarget,
	      float yawDesired, bool moveToJunction,
	      float xyThresh, float yawThresholdInDeg)
{
	int responseTimeout              = 1;
	int timeoutInMilSec              = 30000; // Long timeout
  	int controlFreqInHz              = 50; // Hz
  	int cycleTimeInMs                = 1000 / controlFreqInHz;
  	int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
  	int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
  	int pkgIndex;

	int distanceThresh  = 130; // Min obj distance to start turning

  float cwYawLim = (yawDesired + 5) * DEG2RAD; // Clockwise limit of yaw
  float ccwYawLim = (yawDesired - 5) * DEG2RAD; // Counter clockwise limit of yaw

	Pos tempPos; // Position struct to hold current value of currPos (global)
	Dists tempDists; // var to hold curr value of obstacle distances
	//Index to direction mappings: 0 - d, 1 - f, 2 - r, 3 - b, 4 - l

	// Wait for data to come in
	sleep(1);

	// Get data
	getCurrPos(&tempPos);
	getCurrDists(&tempDists);

	// Global position retrieved via subscription
	Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
	Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
	// Global position retrieved via broadcast
	Telemetry::GlobalPosition currentBroadcastGP;
	Telemetry::GlobalPosition originBroadcastGP;

	// Convert position offset from first position to local coordinates
	Telemetry::Vector3f localOffset;
	// OSDK version
	currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
	originBroadcastGP  = currentBroadcastGP;
	localOffsetFromGpsOffset(vehicle, localOffset,
                           static_cast<void*>(&currentBroadcastGP),
                           static_cast<void*>(&originBroadcastGP));
  	// TODO: Do we want to input offset into traversAisle and calculate
	// offset desired from there?
	double xOffsetRemaining = xTarget - localOffset.x;
	double yOffsetRemaining = yTarget - localOffset.y;
	double zOffsetRemaining = zTarget - (-localOffset.z);

	double xOffsetRem = xTarget - tempPos.x;
	double yOffsetRem = yTarget - tempPos.y;
	double zOffsetRem = zTarget - (-tempPos.z);

	// Conversions
	double yawDesiredRad     = DEG2RAD * yawDesired;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

	//! Get Euler angle

	// Quaternion retrieved via subscription
	Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
	// Quaternion retrieved via broadcast
	Telemetry::Quaternion broadcastQ;

	double yawInRad;
	broadcastQ = vehicle->broadcast->getQuaternion();
	yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;

	int   elapsedTimeInMs     = 0;
	int   withinBoundsCounter = 0;
	int   outOfBounds         = 0;
	int   brakeCounter        = 0;
	int   speedFactor         = 1; // Changed from 2 to 1
	float xCmd, yCmd, zCmd;

	// There is a deadband in position control
  	// the z cmd is absolute height
  	// while x and y are in relative
  	float zDeadband = 1.2; // since vehicle->isM100()
	/*! Calculate the inputs to send the position controller. We implement basic
   	*  receding setpoint position control and the setpoint is always 1 m away
   	*  from the current position - until we get within a threshold of the goal.
   	*  From that point on, we send the remaining distance as the setpoint.
   	*/
	xCmd = speedFactor;
	yCmd = 0; // No Y coordinate movements - yCmd = 0;
  zCmd = currentBroadcastGP.height + zTarget;

  int printCtr = 0;

	while (elapsedTimeInMs < timeoutInMilSec)
	{
    if (printCtr++ % 100 == 0) {
      std::cout << "Offset Rems: ";
      std::cout << xOffsetRem << ", " << yOffsetRem << ", " << zOffsetRem;
      std::cout << std::endl;
    }
		vehicle->control->positionAndYawCtrlBody(xCmd/2, yCmd, zCmd,
				                         yawDesiredRad / DEG2RAD);
    usleep(cycleTimeInMs * 1000);
	elapsedTimeInMs += cycleTimeInMs;
	//! Get current position in required coordinates and units
	broadcastQ         = vehicle->broadcast->getQuaternion();
	yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
	currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
	localOffsetFromGpsOffset(vehicle, localOffset,
                           static_cast<void*>(&currentBroadcastGP),
                           static_cast<void*>(&originBroadcastGP));
	//! See how much farther we have to go
	xOffsetRemaining = xTarget - localOffset.x;
	yOffsetRemaining = yTarget - localOffset.y;
	zOffsetRemaining = zTarget - (-localOffset.z);
	// GUIDANCE data collection
	getCurrPos(&tempPos);
	xOffsetRem = xTarget - tempPos.x;
	yOffsetRem = yTarget - tempPos.y;
	zOffsetRem = zTarget - (-tempPos.z);

	getCurrDists(&tempDists);
	if (tempDists.l < distanceThresh && tempDists.r < distanceThresh) {
		std::cout << "OBS detected both sides" << std::endl;
	}
  // Left sensor triggered - rotate clockwise
	if (tempDists.l < distanceThresh && yawDesiredRad < cwYawLim) {
		//curr.yaw = curr.yaw + 2;
		yawDesiredRad = yawDesiredRad + (0.2 * DEG2RAD);
		std::cout << "OBS LEFT - ROTATE CW" << std::endl;
	// Right sensor triggered - rotate counter clockwise
  } else if (tempDists.r < distanceThresh && yawDesiredRad > ccwYawLim) {
		//Curr.yaw = curr.yaw - 2;
		yawDesiredRad = yawDesiredRad - (0.2 * DEG2RAD);
		std::cout << "OBS RIGHT - ROTATE CCW" << std::endl;
	}

	//! See if we need to modify the setpoint
    //if (std::abs(xOffsetRemaining) < speedFactor)
    if (std::abs(xOffsetRemaining) < speedFactor / 2)
    {
      xCmd = xOffsetRemaining;
      //xCmd = xOffsetRem;
    }
    if (vehicle->isM100() && std::abs(xOffsetRemaining) < xyThresh &&
        std::abs(yOffsetRemaining) < xyThresh &&
        std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
	    //! 1. We are within bounds; start incrementing our in-bound counter
	    withinBoundsCounter += cycleTimeInMs;
    } else {
	    if (withinBoundsCounter != 0) {
				//! 2. Start incrementing an out-of-bounds counter
            	outOfBounds += cycleTimeInMs;
            }
    }
    //! 3. Reset withinBoundsCounter if necessary
    if (outOfBounds > outOfControlBoundsTimeLimit)
    {
      withinBoundsCounter = 0;
      outOfBounds         = 0;
    }
    //! 4. If within bounds, set flag and break
    if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
    {
      std::cout << "WITHIN BOUNDS" << std::endl;
      break;
    }
	} // End of while loop
	/*if (elapsedTimeInMs >= timeoutInMilSec)
  {
    std::cout << "Task timeout!\n";
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      ACK::ErrorCode ack =
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      if (ACK::getError(ack))
      {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
    }
    return ACK::FAIL;
  }*/
	// If we want to move to a junction, we want to continue moving until
	// we reach the junction.
	if (moveToJunction) {
		std::cout << "Inching towards junction" << std::endl;
		int junctionCounter = 0;
		int junctionCounterMax = 1000;
		int junctionThresh = 400;
		elapsedTimeInMs = 0;
		timeoutInMilSec = 10000;
		xCmd = speedFactor / 2;
		yCmd = 0; // No Y coordinate movements - yCmd = 0;
		while (elapsedTimeInMs < timeoutInMilSec) {
			vehicle->control->positionAndYawCtrlBody(xCmd, yCmd, zCmd,
																							 yawDesiredRad / DEG2RAD);
			usleep(cycleTimeInMs * 1000);
			elapsedTimeInMs += cycleTimeInMs;
			broadcastQ         = vehicle->broadcast->getQuaternion();
      yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
      currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
      localOffsetFromGpsOffset(vehicle, localOffset,
                               static_cast<void*>(&currentBroadcastGP),
                               static_cast<void*>(&originBroadcastGP));
			getCurrDists(&tempDists);
      if (tempDists.l < distanceThresh && tempDists.r < distanceThresh) {
    		std::cout << "OBS detected both sides" << std::endl;
    	}
      // Left sensor triggered - rotate clockwise
    	if (tempDists.l < distanceThresh && yawDesiredRad < cwYawLim) {
    		//curr.yaw = curr.yaw + 2;
    		yawDesiredRad = yawDesiredRad + (0.2 * DEG2RAD);
    		std::cout << "OBS LEFT - ROTATE CW" << std::endl;
    	// Right sensor triggered - rotate counter clockwise
      } else if (tempDists.r < distanceThresh && yawDesiredRad > ccwYawLim) {
    		//Curr.yaw = curr.yaw - 2;
    		yawDesiredRad = yawDesiredRad - (0.2 * DEG2RAD);
    		std::cout << "OBS RIGHT - ROTATE CCW" << std::endl;
    	}
			// TODO: REPLACE FOLLOWING IF STATEMENT TO CHECK FOR DISTANCE SENSOR
			if (tempDists.l > junctionThresh)
			{
			 	//! 1. We are within bounds; start incrementing our in-bound counter
			  junctionCounter += cycleTimeInMs;
				xCmd = speedFactor / 3;
			} else {
				junctionCounter = 0;
				xCmd = speedFactor / 2;
			}
			//! 4. If within bounds, set flag and break
			if (junctionCounter >= junctionCounterMax)
			{
			 	break;
			}
		} // End of while loop
		if (elapsedTimeInMs >= timeoutInMilSec)
	  {
	    std::cout << "Task timeout!\n";
	    if (!vehicle->isM100() && !vehicle->isLegacyM600())
	    {
	      ACK::ErrorCode ack =
	        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
	      if (ACK::getError(ack))
	      {
	        std::cout << "Error unsubscribing; please restart the drone/FC to get "
	                     "back to a clean state.\n";
	      }
	    }
	    return ACK::FAIL;
	  }
	}
	return ACK::SUCCESS;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates.
    Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,
                         void* target, void* origin)
{
  Telemetry::GPSFused*       subscriptionTarget;
  Telemetry::GPSFused*       subscriptionOrigin;
  Telemetry::GlobalPosition* broadcastTarget;
  Telemetry::GlobalPosition* broadcastOrigin;
  double                     deltaLon;
  double                     deltaLat;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    subscriptionTarget = (Telemetry::GPSFused*)target;
    subscriptionOrigin = (Telemetry::GPSFused*)origin;
    deltaLon   = subscriptionTarget->longitude - subscriptionOrigin->longitude;
    deltaLat   = subscriptionTarget->latitude - subscriptionOrigin->latitude;
    deltaNed.x = deltaLat * C_EARTH;
    deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
    deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
  }
  else
  {
    broadcastTarget = (Telemetry::GlobalPosition*)target;
    broadcastOrigin = (Telemetry::GlobalPosition*)origin;
    deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
    deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
    deltaNed.x      = deltaLat * C_EARTH;
    deltaNed.y      = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
    deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
  }
}

Telemetry::Vector3f
toEulerAngle(void* quaternionData)
{
  Telemetry::Vector3f    ans;
  Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

  double q2sqr = quaternion->q2 * quaternion->q2;
  double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
  double t1 =
    +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
  double t2 =
    -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
  double t3 =
    +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
  double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);

  return ans;
}

bool startGlobalPositionBroadcast(Vehicle* vehicle)
{
  uint8_t freq[16];

  /* Channels definition for A3/N3/M600
   * 0 - Timestamp
   * 1 - Attitude Quaternions
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Status
   * 12 - Battery Level
   * 13 - Control Information
   */
  freq[0]  = DataBroadcast::FREQ_HOLD;
  freq[1]  = DataBroadcast::FREQ_HOLD;
  freq[2]  = DataBroadcast::FREQ_HOLD;
  freq[3]  = DataBroadcast::FREQ_HOLD;
  freq[4]  = DataBroadcast::FREQ_HOLD;
  freq[5]  = DataBroadcast::FREQ_50HZ; // This is the only one we want to change
  freq[6]  = DataBroadcast::FREQ_HOLD;
  freq[7]  = DataBroadcast::FREQ_HOLD;
  freq[8]  = DataBroadcast::FREQ_HOLD;
  freq[9]  = DataBroadcast::FREQ_HOLD;
  freq[10] = DataBroadcast::FREQ_HOLD;
  freq[11] = DataBroadcast::FREQ_HOLD;
  freq[12] = DataBroadcast::FREQ_HOLD;
  freq[13] = DataBroadcast::FREQ_HOLD;

  ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreq(freq, 1);
  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }
  else
  {
    return true;
  }
}
