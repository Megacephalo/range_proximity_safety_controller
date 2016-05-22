/*
 * Copyright (c) 2016, NTU iCeiRA.
 * All right reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file range_proximity_safety_controller/include/range_proximity_safety_controller/safety_controller.h 
 * 
 * @brief range safety controller relying on laser scannings.
 * 
 * This controller uses only laser scaning to serves as a proximity detecting and obstacle avoiding controller.
 * 
 * @author Charly Huang, NTU iCeiRA
 * 
 * Big thanks to Yujin Robot for inspiring this project.
 **/ 

/************************************************************************
 * Ifdefs
 ************************************************************************/

#ifndef _SAFETY_CONTROLLER_H_
#define _SAFETY_CONTROLLER_H_

/*************************************************************************
 * Includes
 *************************************************************************/

#include <string>
#include <boost/concept_check.hpp>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>

namespace beebot
{
  /**
   * @brief keeps track of safety distance and commands the reactive evasive maneuvre accordingly.
   * 
   * The SafetyController keeps track of lethal range from the laser readings, Potential Field with upper bound
   * will be used to determine a gradient of repulsive velocity. Once out of the range of action, the controller 
   * will be shut down.
   * 
   * The Safety Controller should be used in conjuction with Velocity Multiplexer and Velocity Smoother.
   **/
  class SafetyController
  {
  public:
    SafetyController(ros::NodeHandle& nh, std::string& name) :
      nh_(nh),
      name_(name),
      last_event_time_(ros::Time(0)),
      cmdvel_(new geometry_msgs::Twist()){} ;
    ~SafetyController(){} ;
    
    /**
     * Set up necessary publishers/subscrivers and variables
     * @return true, if successful
     */ 
    bool init()
    {
      // how long to keep sending messages after the range is breached.
      double time_to_extend_repulsion ;
      nh_.param("time_to_extend_repulsion", time_to_extend_repulsion, 0.1) ;
      time_to_extend_repulsion_ = ros::Duration(time_to_extend_repulsion) ;
      enable_controller_subscriber_ = nh_.subscribe("enable", 10, &SafetyController::enableCB, this) ;
      disable_controller_subscriber_ = nh_.subscribe("disable", 10, &SafetyController::disableCB, this) ;
      reset_safety_states_subscriber_ = nh_.subscribe("reset", 10, &SafetyController::resetSafetyStateCB, this) ;
      laser_sub_ = nh_.subscribe("scan", 1, &SafetyController::laserScanVigilanteCB, this) ;
      
      velocity_command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel", 10) ;
      
      return true ;
    }

  private:
    ros::NodeHandle nh_ ;
    std::string name_ ;
    ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_ ;
    ros::Subscriber reset_safety_states_subscriber_ ;
    ros::Publisher velocity_command_publisher_ ;
    ros::Subscriber laser_sub_ ;
    ros::Duration time_to_extend_repulsion_ ;
    ros::Time last_event_time_ ;
    bool rangeBreached_ ;
    
    geometry_msgs::TwistPtr cmdvel_ ;  // velocity commands
    
    /**
     * @brief ROS logging output for enabling the controller
     * @param msg incoming topic message
    */ 
    void enableCB(const std_msgs::EmptyConstPtr msg) ;
    
    /**
     * @brief ROS logging output for disabling the controller
     * @param msg incoming topic message
     */
    void disableCB(const std_msgs::EmptyConstPtr msg) ;
    
    /**
     * @brief Keep track of laser readings. Once passing the allowed range, a flag is raised
     * announcing an event
     */ 
    void laserScanVigilanteCB(const sensor_msgs::LaserScan::ConstPtr& msg) ;
    
    /**
     * @brief If an event flag is received, the controller is activated and proceed to evasive maneuvre
     */ 
    void evasiveMan(bool rangeBreached_) ;
    
    /**
     * @brief Reset the controller state back to vigilante state.
     */ 
    void resetSafetyStateCB(const std_msgs::EmptyConstPtr msg) ;
    
  } ;  // Class definition
  
  
} // namespace

#endif
