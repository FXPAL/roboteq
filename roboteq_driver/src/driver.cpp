/**
Software License Agreement (BSD)

\file      driver.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
           Mike Irvine <mirvine@clearpathrobotics.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "roboteq_driver/controller.h"
#include "roboteq_driver/channel.h"
#include "geometry_msgs/Twist.h"


#include "ros/ros.h"
#include <math.h>

#include "roboteq_msgs/Command.h"


static const float kMAX_SPEED_MS = 2.0;

static const float kMAX_SPEED = 200;
static const float kMIN_SPEED = -200;
static const float kMAX_ROT_SPEED = 200;

static roboteq::Controller * controller_handle;


// left is steering and right is speed
// open loop version using arbitrary values
void twist_callback_open_loop(const geometry_msgs::Twist tw)
{


    geometry_msgs::Vector3 lin = tw.linear;
    geometry_msgs::Vector3 ang = tw.angular;



    const float speed = lin.x;
    const float rotation = ang.z;



    // let's set 2/ms to 200

    // RIGHT
    float speed_command = speed / kMAX_SPEED_MS *  kMAX_SPEED;

    ROS_INFO("got twist callback %f %f %f", speed, rotation, speed_command);

     if(speed_command >= 0) {
        speed_command = std::min(speed_command, kMAX_SPEED);
        }
    else {
        speed_command = std::max(speed_command, kMIN_SPEED);
        }

     roboteq_msgs::Command SPEED_COMMAND;
     SPEED_COMMAND.commanded_velocity = speed_command;




    // LEFT angular component is radians per second
    // max value is pi / 8


    float rotation_command = rotation / (2*M_PI) * kMAX_ROT_SPEED;
    roboteq_msgs::Command ROTATE_COMMAND;
    ROTATE_COMMAND.commanded_velocity = rotation_command;


    roboteq::Channel * ROTATE_CHANNEL = controller_handle->getChannel(0);
    roboteq::Channel * SPEED_CHNANEL = controller_handle->getChannel(1);

    ROTATE_CHANNEL->cmdCallback(ROTATE_COMMAND);
    SPEED_CHNANEL->cmdCallback(SPEED_COMMAND);

    ROS_INFO("Commands rot: %f  vel: %f", rotation_command, speed_command );

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  std::string port = "/dev/ttyUSB0";
  int32_t baud = 115200;
  std::string channels = "[left]";

  nh.param<std::string>("port", port, port);
  nh.param<int32_t>("baud", baud, baud);
  nh.param<std::string>("channels", channels, channels);

  // Interface to motor controller.
  roboteq::Controller controller(port.c_str(), baud);
  controller_handle = &controller;

  // Setup channels.
  if (nh.hasParam("channels")) {
    XmlRpc::XmlRpcValue channel_namespaces;
    nh.getParam("channels", channel_namespaces);
    ROS_ASSERT(channel_namespaces.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < channel_namespaces.size(); ++i)
    {
      ROS_ASSERT(channel_namespaces[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      controller.addChannel(new roboteq::Channel(1 + i, channel_namespaces[i], &controller));
    }
  } else {
    // Default configuration is a single channel in the node's namespace.
    controller.addChannel(new roboteq::Channel(1, "~", &controller));
  }

   // add handling for twist messages (requires initialized controller)
  ros::Subscriber twistSub = nh.subscribe("/cmd_vel", 1, twist_callback_open_loop);

  // Attempt to connect and run.
  while (ros::ok()) {
    ROS_DEBUG("Attempting connection to %s at %i baud.", port.c_str(), baud);
    controller.connect();
    if (controller.connected()) {
      ros::AsyncSpinner spinner(1);
      spinner.start();
      while (ros::ok()) {
        controller.spinOnce();
      }
      spinner.stop();
    } else {
      ROS_DEBUG("Problem connecting to serial device.");
      ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
      sleep(1);
    }
  }

  return 0;
}
