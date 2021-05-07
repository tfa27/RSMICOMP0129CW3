/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

 /* Modified author: Alejandro Halpern, Andras Nagy, Tim Andersson, Xingjian Lu*/

#ifndef CW3_H_
#define CW3_H_

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// HW1 Includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

// cw3 Includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Char.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <math.h>
#include <stdio.h>
#include <ctype.h>


/** \brief CW 3.
  *
  * \author Dimitrios Kanoulas
  */
class CW3
{
  public:
    /** \brief Empty constructor.
      *
      * \input[in] nh the ROS node
      */
    CW3 (ros::NodeHandle &nh);

    /** \brief Lab 1: initialize the parameters. */
    void
    initParams ();

    /** \brief Load the parameters. */
    void
    updateParams (ros::NodeHandle &nh);

    /** \brief Lab 1: get the world frame.
      *
      * \return the world frame string
      */
    std::string
    getWorldFrame ();

    /** \brief Lab 1: get the robot frame.
      *
      * \return the robot frame string
      */
    std::string
    getRobotFrame ();

    /** \brief Lab 1: create the transformation between robot & world
     *  frames.
     */
    void
    Lab1CreateFrames ();

    /** \brief Lab 1: publish the transformations between robot & world
      * frames.
      */
    void
    Lab1PublishFrames ();

    /** \brief CW1: helper function to implement a non-blocking key getter.
      *
      * \return 1 for key pressed, 0 for no pressed
      */
    int
    kbhit ();

    /** \brief CW1 Q2: makes collision object box
      *
      * \input id name of identifier
      * \input frame name of frame_id
      * \input dim_x box dimensions along x
      * \input dim_y box dimensions along y
      * \input dim_z box dimensions along z
      * \input pos_x centre of box along x
      * \input pos_y centre of box along y
      * \input pos_z centre of box along z
      */
    moveit_msgs::CollisionObject
    cw1Q3MakeBox(std::string id, std::string frame_id,
                        float dim_x, float dim_y, float dim_z,
                        float pos_x, float pos_y, float pos_z);

    /** \brief CW1 Q2: add collision objects: table1, table2, table3, object
      *
      * \input planning_scene_interface the MoveIt! PlanningSceneInterface
      */
    void
    cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface&
                                                      planning_scene_interface);

    /** This function actually works for the last part of q2 and the
      * q3 parts as well.
      */
    void
    cw3q3(int chah,
      moveit::planning_interface::PlanningSceneInterface& p_s_i,
        moveit::planning_interface::MoveGroupInterface& m_g_i);

    /** This function simply exists to execute the correct functions dependent
      * on input
      */
    void
    make_choice (int cha,
      moveit::planning_interface::PlanningSceneInterface& p_s_i,
        moveit::planning_interface::MoveGroupInterface& m_g_i);

    /** This function works for both picking up the object and cylinder.
      * It sets a pre place approach and and post place retreat.
      */
    void
    approach_and_grasp(std::string to_pick,
      moveit::planning_interface::MoveGroupInterface& m_g_i);

    /** This function is executed before approach and grasp and is the main
      * function for the functionality of picking up the cylinder
      */
    void
    pick_up_cylinder(moveit::planning_interface::PlanningSceneInterface& p_s_i,
        moveit::planning_interface::MoveGroupInterface& m_g_i,
            std::string to_pick);

    /** This function is similar to the one above except it's for the object
      */
    void
    move_object(int chah,
      moveit::planning_interface::PlanningSceneInterface& p_s_i,
            moveit::planning_interface::MoveGroupInterface& m_g_i);

    /** This function is used to get the string that makes the options menu
      */
    std::string
    get_options();

    /** This function grips by changing the gripper joint values using MoveIt
      */
    void
    grip(moveit::planning_interface::MoveGroupInterface& m_g_i,
      std::vector<moveit_msgs::Grasp>& grasps, std::string to_pick);

    /** This function moves the cylinder to a position above the correct table
      * and then executes place_object
      */
    void
    place_cylinder(int chah,
      moveit::planning_interface::PlanningSceneInterface& p_s_i,
          moveit::planning_interface::MoveGroupInterface& m_g_i);

    /** Descends the current robot pose and places the object on the table.
      */
    void
    put_it_down(std::string which_table,
      moveit::planning_interface::PlanningSceneInterface& p_s_i,
        moveit::planning_interface::MoveGroupInterface& m_g_i);

    /** This function creates the bumpers for question 2
      */
    void
    cw3q2AddBumper
      (moveit::planning_interface::MoveGroupInterface& move_group,
        moveit::planning_interface::PlanningSceneInterface
          &planning_scene_interface);

    /** This function checks whether the object is touching the bumper
      */
    void
    cw3q2BumperTouched
      (moveit::planning_interface::MoveGroupInterface& move_group,
        moveit::planning_interface::PlanningSceneInterface
          &planning_scene_interface);

    /** This function descends the current robot pose and places the cylinder
      * on the table
      */
    void
    place_object(std::string which_table,
      moveit::planning_interface::PlanningSceneInterface& p_s_i,
        moveit::planning_interface::MoveGroupInterface& m_g_i);

    /** This function publishes the input char for functionality in the
      * cylinder_segment.cpp node
      */
    void
    publishChar(char c);

    /** A simple helper function to initialisea place_location
      */
    std::vector<moveit_msgs::PlaceLocation>
    make_place_location();

    /** This is also another helper function to avoid repeating code
      */
    void
    set_place_location_params
        (std::vector<moveit_msgs::PlaceLocation> &place_location,
              std::map<std::string, geometry_msgs::Pose> &object_pose,
                  std::string &which_table);

    /** \brief convert StampedTransform to homogeneous matrix. */
    Eigen::Matrix4d
    transformToHom (tf::StampedTransform transf);

    /** \brief convert PoseStamped to homogeneous matrix. */
    Eigen::Matrix4d
    transPoseToHom (const geometry_msgs::PoseStamped P);

    /** \brief convert homogeneous matrix to PoseStamped. */
    void
    transHomToPose (Eigen::Matrix4d T);

    /** \brief convert the cylinder pose from
     * the camera frame to the world frame */
    void
    convertCToW();

    void
    printMat (Eigen::Matrix4d T, std::string hom_mat);

////////////////////////////////////////////////////////////////////////////////
    /** \brief Subscribe to the pose of the cylinder w.r.t. the camera frame. */
    void
    cylSubCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
      {this->cylinder_cam_pose_ = *msg;}

    /** \brief Subscribe to the pose of the cylinder w.r.t. the camera frame. */
    void
    cylDimSubCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      this->cylinder_dims.push_back(msg->data.at(0));
      this->cylinder_dims.push_back(msg->data.at(1));
    }

  public:
    /** \brief Node handle. */
    ros::NodeHandle nh_;

    /** \brief World and robot frames. */
    std::string world_frame_, robot_frame_;

    /** \brief Lab 1: TF transforms definition. */
    tf::Transform transf_;

    /** \brief Lab 1: TF transform broadcaster definitions. */
    tf::TransformBroadcaster tranf_br_;

    int internal_counter {0};

    /** \brief Storing the published pose of the cylinder w.r.t. camera frame. */
    geometry_msgs::PoseStamped cylinder_cam_pose_;

    /** \brief Storing the pose of the cylinder w.r.t. world frame. */
    geometry_msgs::Pose cylinder_pose;

    /** \brief Storing the dimensions of the collision objects. */
    std::map<std::string, std::vector<double>> obj_dims;

    /** \brief Storing the published dimensions of the cylinder. */
    std::vector<double> cylinder_dims;


    /** Publishers for the bumper values
     */
    std::vector<ros::Publisher> bumpers_pub_ =
      {nh_.advertise<std_msgs::Bool> ("/bumper1", 1, true),
      nh_.advertise<std_msgs::Bool> ("/bumper2", 1, true),
      nh_.advertise<std_msgs::Bool> ("/bumper3", 1, true)};

    /** Publishes the input choice.
      */
    ros::Publisher choice_pub =
      nh_.advertise<std_msgs::Char> ("/choices", 1, true);

    /**Subscribes to the cylinder pose
      */
    ros::Subscriber cyl_pos_sub_ =
      nh_.subscribe("/cylinder_pose", 1000, &CW3::cylSubCb, this);

    /**Subscribes to the cylinder dimensions
      */
    ros::Subscriber cyl_dim_ =
      nh_.subscribe("/cylinder_size", 1000, &CW3::cylDimSubCb, this);

    /** \brief CW3-Q2: TF listener definition. */
    tf::TransformListener listener_;

    /** \brief CW3-Q2: TF transforms definitions. */
    tf::StampedTransform transf_Q2;

  protected:
    /** \brief Debug mode. */
    bool debug_;

    /** \brief boolean vector for the bumpers */
    std::vector<std_msgs::Bool> bumpers;

    /** \brief vector for storing the bumper positions */
    std::vector<std::vector<double>> bumper_pos_vect;

    geometry_msgs::Pose original_pose;

    double go_down_step {0.05};

    double target_pose_above_step {0.25};

    double target_place_pose_z {0.7};

    /** \brief double range (bumper limit) where the bumpers turn on */
    double epsilon {0.05};

    /** \brief vector for storing the table names */
    std::vector<std::string> table_names {"table1", "table2", "table3"};

    /** \brief vector for storing cubic object name */
    std::vector<std::string> cubic_obj {"object"};

    /** \brief vector for storing all the object names */
    std::vector<std::string> all_ids
        {"table1", "table2", "table3", "object", "cylinder"};
};
#endif
