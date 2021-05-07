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

#include <cw3.h>

CW3::CW3 (ros::NodeHandle &nh):
  debug_ (false)
{
  nh_ = nh;
  initParams ();
  updateParams (nh);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::initParams ()
{
  // Frames
  this->world_frame_ = "/world_frame";

  // Topics
  this->robot_frame_ = "/robot_frame";

	//this->cylinder_pose.position.x = 0.384;
	//this->cylinder_pose.position.y = 0.035;
	//this->cylinder_pose.position.z = 0.083 + 0.07;

  // HW1-Q2: setup the names for frames {0}, {1}, {2}, and {3}
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::updateParams (ros::NodeHandle &nh)
{
  // Frames
  nh.getParam("/frame_params/world_frame", this->world_frame_);
  nh.getParam("/frame_params/robot_frame", this->robot_frame_);
}


////////////////////////////////////////////////////////////////////////////////
std::string
CW3::getWorldFrame ()
{
  return (this->world_frame_);
}

////////////////////////////////////////////////////////////////////////////////
std::string
CW3::getRobotFrame ()
{
  return (this->robot_frame_);
}

////////////////////////////////////////////////////////////////////////////////
int
CW3::kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::Lab1CreateFrames ()
{
  // generate a robot frame attached to the world frame (0,0,0)
  transf_.setOrigin (tf::Vector3(0.0, 0.0, 0.0));
  transf_.setRotation (tf::Quaternion(0.0, 0.0, 0.0, 1.0)); //quaternion

  // Note that the rotation can be given in various forms (rotation matrix,
  // axis-angle, etc), but the function setRotation gets a tf::Quaternion,
  // thus when a different type rotation is available, it should be converted
  // to a tf::Quaternion.
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::Lab1PublishFrames ()
{
  // publish world->robot
  tranf_br_.sendTransform(tf::StampedTransform(transf_,
                                               ros::Time::now(),
                                               world_frame_,
                                               robot_frame_));
}

////////////////////////////////////////////////////////////////////////////////
moveit_msgs::CollisionObject
CW3::cw1Q3MakeBox(std::string id, std::string frame_id,
					float dim_x, float dim_y, float dim_z,
					float pos_x, float pos_y, float pos_z)
{
  // Makes a Box collision object at given location with given dimensions.

  moveit_msgs::CollisionObject collision_object;

  // Add the first table where the cube will originally be kept.
  collision_object.id = id;
  collision_object.header.frame_id = frame_id;

  /* Define the primitive and its dimensions. */
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dim_x;
  collision_object.primitives[0].dimensions[1] = dim_y;
  collision_object.primitives[0].dimensions[2] = dim_z;

  /* Define the pose of the table: center of the cube. */
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x =  pos_x;
  collision_object.primitive_poses[0].position.y =  pos_y;
  collision_object.primitive_poses[0].position.z =  pos_z;

  collision_object.operation = collision_object.ADD;
  return collision_object;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface&
                       planning_scene_interface)
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 4 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table where the cube will originally be kept.
  collision_objects[0] = CW3::cw1Q3MakeBox("table1", "panda_link0",
                                            0.4, 0.2, 0.4,
                                            0.0, 0.5, 0.2);

	this->obj_dims["table1"] = {0.4, 0.2, 0.4};


  // Add the second table where we will be placing the cube.
  collision_objects[1] = CW3::cw1Q3MakeBox("table2", "panda_link0",
                                            0.2, 0.4, 0.4,
					    															-0.5, 0.0, 0.2);

	this->obj_dims["table2"] = {0.2, 0.4, 0.4};

  // Add the second table where we will be placing the cube.
  collision_objects[2] = CW3::cw1Q3MakeBox("table3", "panda_link0",
                                            0.4, 0.2, 0.4,
                                            0.0, -0.5, 0.2);

  this->obj_dims["table3"] = {0.4, 0.2, 0.4};


  // Define the object that we will be manipulating
  collision_objects[3] = CW3::cw1Q3MakeBox("object", "panda_link0",
                                            0.02, 0.02, 0.2,
                                            -0.5, 0.0, 0.5);

  this->obj_dims["object"] = {0.02, 0.02, 0.2};

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::grip(moveit::planning_interface::MoveGroupInterface& m_g_i,
	std::vector<moveit_msgs::Grasp>& grasps, std::string to_pick)
{
	grasps[0].pre_grasp_posture.joint_names.resize(2);
	grasps[0].pre_grasp_posture.joint_names[0] = "panda_finger_joint1";
	grasps[0].pre_grasp_posture.joint_names[1] = "panda_finger_joint2";

	grasps[0].pre_grasp_posture.points.resize(1);
	grasps[0].pre_grasp_posture.points[0].positions.resize(2);
	grasps[0].pre_grasp_posture.points[0].positions[0] = 0.035;
	grasps[0].pre_grasp_posture.points[0].positions[1] = 0.035;
	grasps[0].pre_grasp_posture.points[0].time_from_start=ros::Duration(0.5);

	grasps[0].grasp_posture.joint_names.resize(2);
	grasps[0].grasp_posture.joint_names[0] = "panda_finger_joint1";
	grasps[0].grasp_posture.joint_names[1] = "panda_finger_joint2";

	grasps[0].grasp_posture.points.resize(1);
	grasps[0].grasp_posture.points[0].positions.resize(2);
	grasps[0].grasp_posture.points[0].positions[0] = 0.02;
	grasps[0].grasp_posture.points[0].positions[1] = 0.02;
	grasps[0].grasp_posture.points[0].time_from_start = ros::Duration(0.5);

	m_g_i.pick(to_pick, grasps);

	return;
}


////////////////////////////////////////////////////////////////////////////////
void
CW3::approach_and_grasp(std::string to_pick,
				moveit::planning_interface::MoveGroupInterface& m_g_i)
{

	std::vector<moveit_msgs::Grasp> grasps;
	grasps.resize(1);

	geometry_msgs::Pose next_pose;
	geometry_msgs::PoseStamped curr_pose = m_g_i.getCurrentPose();

	grasps[0].grasp_pose.header.frame_id = "panda_link0";

	grasps[0].grasp_pose.pose.orientation = curr_pose.pose.orientation;

	grasps[0].grasp_pose.pose.position = curr_pose.pose.position;
	grasps[0].grasp_pose.pose.position.z =
			grasps[0].grasp_pose.pose.position.z - this->go_down_step;

	grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";

	grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
	grasps[0].pre_grasp_approach.min_distance = 0.01;
	grasps[0].pre_grasp_approach.desired_distance = 0.08;

	grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";

	grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
	grasps[0].post_grasp_retreat.min_distance = 0.05;
	grasps[0].post_grasp_retreat.desired_distance = 0.1;

	this->grip(m_g_i, grasps, to_pick);

	return;

}


////////////////////////////////////////////////////////////////////////////////
void
CW3::pick_up_cylinder
		(moveit::planning_interface::PlanningSceneInterface& p_s_i,
						moveit::planning_interface::MoveGroupInterface& m_g_i,
										std::string to_pick)
{
	std::map<std::string, moveit_msgs::CollisionObject>
		my_map = p_s_i.getObjects();

	std::map<std::string, geometry_msgs::Pose>
		collision_object_poses =
				p_s_i.getObjectPoses(this->all_ids);

	geometry_msgs::Pose cylinder_pos;

	cylinder_pos = this->cylinder_pose;
	cylinder_pos.position.z = cylinder_pos.position.z + this->go_down_step*2;

	geometry_msgs::PoseStamped curr_pose = m_g_i.getCurrentPose();

	cylinder_pos.position.z = cylinder_pos.position.z + 0.09;

	cylinder_pos.orientation.x = curr_pose.pose.orientation.x;
	cylinder_pos.orientation.y = curr_pose.pose.orientation.y;
	cylinder_pos.orientation.z = curr_pose.pose.orientation.z;
	cylinder_pos.orientation.w = curr_pose.pose.orientation.w;

	m_g_i.setPoseTarget(cylinder_pos);

	m_g_i.move();

	ros::Duration(0.5).sleep();

	this->approach_and_grasp(to_pick, m_g_i);

	this->cw3q2BumperTouched(m_g_i, p_s_i);

	return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::move_object(int chah,
	moveit::planning_interface::PlanningSceneInterface& p_s_i,
				moveit::planning_interface::MoveGroupInterface& m_g_i)
{

	ros::Duration(0.5).sleep();
	std::map<std::string, moveit_msgs::CollisionObject>
		my_map = p_s_i.getObjects();

	std::map<std::string, geometry_msgs::Pose>
		collision_object_poses = p_s_i.getObjectPoses(this->all_ids);

	geometry_msgs::Pose target_pose;
	target_pose.position = collision_object_poses["object"].position;
	target_pose.position.z =
			target_pose.position.z + this->target_pose_above_step;

	geometry_msgs::PoseStamped curr_pose = m_g_i.getCurrentPose();

	target_pose.orientation = curr_pose.pose.orientation;

	m_g_i.setPoseTarget(target_pose);

	std::vector<std::string> go_to;
	for (int i {0}; i < 3; i++)
	{
		if (this->bumpers.at(i).data == false)
		{
			go_to.push_back(this->all_ids.at(i));
		}
	}

	m_g_i.move();

	ros::Duration(0.5).sleep();

	m_g_i.setSupportSurfaceName(this->all_ids.at(chah - '0' - 1));

	this->approach_and_grasp("object", m_g_i);

	this->cw3q2BumperTouched(m_g_i, p_s_i);

	collision_object_poses = p_s_i.getObjectPoses(go_to);

	curr_pose = m_g_i.getCurrentPose();

	target_pose.position = collision_object_poses[go_to.at(0)].position;
	target_pose.position.z = this->target_place_pose_z;
	target_pose.orientation = curr_pose.pose.orientation;

	m_g_i.setPoseTarget(target_pose);

	ros::Duration(0.5).sleep();

	m_g_i.move();

	this->place_object(go_to.at(0), p_s_i, m_g_i);

	ros::Duration(0.5).sleep();

	return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::place_cylinder(int chah,
		moveit::planning_interface::PlanningSceneInterface& p_s_i,
				moveit::planning_interface::MoveGroupInterface& m_g_i)
{

	ros::Duration(1).sleep();

	std::vector<std::string> selected_table;
	selected_table.push_back(this->table_names.at(chah - '0' - 1));

	std::map<std::string, geometry_msgs::Pose>
		collision_object_poses = p_s_i.getObjectPoses(selected_table);

	geometry_msgs::Pose target_pose;

	geometry_msgs::PoseStamped curr_pose = m_g_i.getCurrentPose();
	this->cylinder_pose = curr_pose.pose;

	target_pose.position = curr_pose.pose.position;
	target_pose.position.z =
			this->obj_dims["table1"].at(2) + this->target_pose_above_step;
	target_pose.orientation = curr_pose.pose.orientation;

	m_g_i.setPoseTarget(target_pose);

	m_g_i.move();

	curr_pose = m_g_i.getCurrentPose();
	this->cylinder_pose = curr_pose.pose;

	ros::Duration(1).sleep();

	target_pose.position.x =
			collision_object_poses[selected_table.at(0)].position.x;
	target_pose.position.y =
			collision_object_poses[selected_table.at(0)].position.y;

	m_g_i.setPoseTarget(target_pose);
	m_g_i.move();

	curr_pose = m_g_i.getCurrentPose();
	this->cylinder_pose = curr_pose.pose;

	ros::Duration(1).sleep();

	this->put_it_down((this->table_names.at(chah - '0' - 1)), p_s_i, m_g_i);

	this->cw3q2BumperTouched(m_g_i, p_s_i);

	return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::put_it_down(std::string which_table,
	moveit::planning_interface::PlanningSceneInterface& p_s_i,
		moveit::planning_interface::MoveGroupInterface& m_g_i)
{
	std::vector<std::string> ids {which_table};
	std::map<std::string, geometry_msgs::Pose> object_pose =
			p_s_i.getObjectPoses(ids);

	std::vector<moveit_msgs::PlaceLocation> place_location;
	place_location = this->make_place_location();

	geometry_msgs::PoseStamped curr_pose = m_g_i.getCurrentPose();

	place_location[0].place_pose.pose.orientation = curr_pose.pose.orientation;

	this->set_place_location_params(place_location, object_pose, which_table);

	m_g_i.setSupportSurfaceName(which_table);

	m_g_i.place("cylinder", place_location);

	ros::Duration(1).sleep();

	auto it = find(this->table_names.begin(), this->table_names.end(), which_table);
	int idx {-1};
	if (it != this->table_names.end())
	{
		idx = it - this->table_names.begin();
	}

	this->cylinder_pose.orientation.x = 0;
	this->cylinder_pose.orientation.y = 0;
	this->cylinder_pose.orientation.z = 0;
	this->cylinder_pose.orientation.w = 1;

	this->cylinder_pose.position.x = this->bumper_pos_vect.at(idx).at(0);
	this->cylinder_pose.position.y = this->bumper_pos_vect.at(idx).at(1);
	this->cylinder_pose.position.z = this->bumper_pos_vect.at(idx).at(2) + this->cylinder_dims.at(1)/2.0;

	return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::place_object(std::string which_table,
	moveit::planning_interface::PlanningSceneInterface& p_s_i,
		moveit::planning_interface::MoveGroupInterface& m_g_i)
{
	std::vector<std::string> ids {which_table};
	std::map<std::string, geometry_msgs::Pose> object_pose
	 		= p_s_i.getObjectPoses(ids);

	std::vector<moveit_msgs::PlaceLocation> place_location;
	place_location = this->make_place_location();



	geometry_msgs::PoseStamped curr_pose = m_g_i.getCurrentPose();

	place_location[0].place_pose.pose.orientation.x = 0;
	place_location[0].place_pose.pose.orientation.y = 0;
	place_location[0].place_pose.pose.orientation.z = 0;
	place_location[0].place_pose.pose.orientation.w = 1;

	this->set_place_location_params(place_location, object_pose, which_table);

	m_g_i.setSupportSurfaceName(which_table);

	m_g_i.place("object", place_location);

	this->cw3q2BumperTouched(m_g_i, p_s_i);

	ros::Duration(1).sleep();

	return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw3q3(int chah, moveit::planning_interface::PlanningSceneInterface& p_s_i,
		moveit::planning_interface::MoveGroupInterface& m_g_i)
{
	if (this->bumpers.at((chah - '0' - 1)).data == true)
	{
		this->move_object(chah, p_s_i, m_g_i);
	}
	this->pick_up_cylinder(p_s_i, m_g_i, "cylinder");
	this->place_cylinder(chah, p_s_i, m_g_i);
	ros::Duration(2).sleep();
	return;
}

////////////////////////////////////////////////////////////////////////////////
std::string
CW3::get_options()
{
	std::string options {""};
	options = options + "Options:\n0 - No action\n" +
			"1 || 2 || 3 - Place cylinder at tables 1, 2 or 3\n" +
						"f - activate or deactivate filter\nm - see this list again";
	return options;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::make_choice(int cha,
	moveit::planning_interface::PlanningSceneInterface& p_s_i,
			moveit::planning_interface::MoveGroupInterface& m_g_i)
{
	this->internal_counter++;

	int new_cha;

	if ((cha == '1') || (cha == '2') || (cha == '3'))
	{
		new_cha = 'x';
	} else
	{
		new_cha = cha;
	}

	putchar(tolower(new_cha));

	switch (new_cha)
	{
		case 'm':
			ROS_INFO_STREAM(this->get_options());
			break;

		case 'f':
			this->publishChar(new_cha);
			break;

		case 'p':
			this->publishChar(new_cha);
			break;

		case 'x':
			this->cw3q3(cha, p_s_i, m_g_i);
			break;

		case '\0':
			break;

		case '0':
			break;

		default:
			ROS_INFO_STREAM("Invalid selection, please try again\n");
			break;
	}

}


////////////////////////////////////////////////////////////////////////////////
void
CW3::cw3q2BumperTouched (moveit::planning_interface::MoveGroupInterface&
 move_group,
  moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
  // get the position of the object
  std::map<std::string, geometry_msgs::Pose> object_pose =
   planning_scene_interface.getObjectPoses(this->cubic_obj);

  // bumper message for publish
  std_msgs::Bool bumper2_touched;

  // bumper limit
  // initialize variables for the distance calculation
  double object_dist;
  double cylinder_dist;

  for(std::size_t i = 0; i < this->bumpers.size(); i++)
  {
    // calculate the distance between the bumper and the object position
    object_dist = sqrt(pow(this->bumper_pos_vect[i][0] -
     object_pose[this->cubic_obj[0]].position.x, 2) +
     pow(this->bumper_pos_vect[i][1] -
		 object_pose[this->cubic_obj[0]].position.y, 2) +
     pow(this->bumper_pos_vect[i][2] - (object_pose[this->cubic_obj[0]].position.z -
     this->obj_dims[this->cubic_obj[0]][2]/2.0), 2));

    // calculate the distance between the bumper and the cylinder position
    cylinder_dist = sqrt(pow(this->bumper_pos_vect[i][0] -
     this->cylinder_pose.position.x, 2) +
     pow(this->bumper_pos_vect[i][1] - this->cylinder_pose.position.y, 2) +
     pow(this->bumper_pos_vect[i][2] - (this->cylinder_pose.position.z -
     this->cylinder_dims.at(1)/2.0), 2));

    if (object_dist <= this->epsilon)
    {
      // cubic object is on the bumper
      this->bumpers[i].data = true;
    }
    else if (cylinder_dist <= this->epsilon)
    {
      // cylinder is on the bumper
      this->bumpers[i].data = true;
    }
    else
    {
      this->bumpers[i].data = false;
    }

    // publish the current bumper value
    this->bumpers_pub_[i].publish(this->bumpers[i]);

  }

}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw3q2AddBumper (moveit::planning_interface::MoveGroupInterface&
 move_group,
  moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{

  // get the centre point of the given table
  std::map<std::string, geometry_msgs::Pose> table_poses =
   planning_scene_interface.getObjectPoses(this->table_names);

  // initialize the poses of the bumpers
  std_msgs::Bool this_bool;
  this_bool.data = false;
  for(std::size_t i = 0; i < table_names.size(); i++)
  {
    this->bumper_pos_vect.push_back({table_poses[table_names[i]].position.x,
      table_poses[table_names[i]].position.y,
      table_poses[table_names[i]].position.z +
      this->obj_dims[table_names[i]][2]/2.0});
    this->bumpers.push_back(this_bool);
  }
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::publishChar(char c)
{
  std_msgs::Char msg;
  msg.data = c;
  this->choice_pub.publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
std::vector<moveit_msgs::PlaceLocation>
CW3::make_place_location()
{
  std::vector<moveit_msgs::PlaceLocation> place_location;

  place_location.resize(1);
  place_location[0].place_pose.header.frame_id = "panda_link0";
  return place_location;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::set_place_location_params
		(std::vector<moveit_msgs::PlaceLocation> &place_location,
					std::map<std::string, geometry_msgs::Pose> &object_pose,
							std::string &which_table)
{
  place_location[0].place_pose.pose.position =
			object_pose[which_table].position;
  place_location[0].place_pose.pose.position.z =
			this->obj_dims["table1"].at(2) + this->go_down_step;

  place_location[0].pre_place_approach.direction.header.frame_id="panda_link0";

	place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.01;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  place_location[0].post_place_retreat.direction.header.frame_id="panda_link0";

  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.01;
  place_location[0].post_place_retreat.desired_distance = 0.15;

  place_location[0].post_place_posture.joint_names.resize(2);
  place_location[0].post_place_posture.joint_names[0] = "panda_finger_joint1";
  place_location[0].post_place_posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  place_location[0].post_place_posture.points.resize(1);
  place_location[0].post_place_posture.points[0].positions.resize(2);
  place_location[0].post_place_posture.points[0].positions[0] = 0.04;
  place_location[0].post_place_posture.points[0].positions[1] = 0.04;
  place_location[0].post_place_posture.points[0].time_from_start =
        ros::Duration(0.5);
  return;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d
CW3::transformToHom (tf::StampedTransform transf)
{
  // convert quaternion to rotation matrix
  tf::Matrix3x3 rot_mat;
  rot_mat.setRotation (transf.getRotation());
  // create the homogeneous matrix
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);

  T(0, 0) = rot_mat[0][0];
  T(0, 1) = rot_mat[0][1];
  T(0, 2) = rot_mat[0][2];
  T(0, 3) = transf.getOrigin().getX();

  T(1, 0) = rot_mat[1][0];
  T(1, 1) = rot_mat[1][1];
  T(1, 2) = rot_mat[1][2];
  T(1, 3) = transf.getOrigin().getY();

  T(2, 0) = rot_mat[2][0];
  T(2, 1) = rot_mat[2][1];
  T(2, 2) = rot_mat[2][2];
  T(2, 3) = transf.getOrigin().getZ();

  return T;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d
CW3::transPoseToHom (geometry_msgs::PoseStamped P)
{
  // convert quaternion to rotation matrix
  tf::Quaternion q;
  q.setX(P.pose.orientation.x);
  q.setY(P.pose.orientation.y);
  q.setZ(P.pose.orientation.z);
  q.setW(P.pose.orientation.w);
  // convert quaternion to rotation matrix
  tf::Matrix3x3 rot_mat;
  rot_mat.setRotation (q);
  // create the homogeneous matrix
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);

  T(0, 0) = rot_mat[0][0];
  T(0, 1) = rot_mat[0][1];
  T(0, 2) = rot_mat[0][2];
  T(0, 3) = P.pose.position.x;

  T(1, 0) = rot_mat[1][0];
  T(1, 1) = rot_mat[1][1];
  T(1, 2) = rot_mat[1][2];
  T(1, 3) = P.pose.position.y;

  T(2, 0) = rot_mat[2][0];
  T(2, 1) = rot_mat[2][1];
  T(2, 2) = rot_mat[2][2];
  T(2, 3) = P.pose.position.z;

  return T;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::transHomToPose (Eigen::Matrix4d T)
{
	//convert the rotation matrix to quaternion
	tf::Matrix3x3 rot_mat;
	rot_mat[0][0] = T(0,0);
	rot_mat[0][1] = T(0,1);
	rot_mat[0][2] = T(0,2);

	rot_mat[1][0] = T(1,0);
	rot_mat[1][1] = T(1,1);
	rot_mat[1][2] = T(1,2);

	rot_mat[2][0] = T(2,0);
	rot_mat[2][1] = T(2,1);
	rot_mat[2][2] = T(2,2);

	tf::Quaternion q;
	//rotation Matrix - > quaternion
  rot_mat.getRotation(q);

	// convert back to a PoseStamped
	this->cylinder_pose.position.x = T(0,3);
	this->cylinder_pose.position.y = T(1,3);
	this->cylinder_pose.position.z = T(2,3);

	this->cylinder_pose.orientation.x = q.x();
	this->cylinder_pose.orientation.y = q.y();
	this->cylinder_pose.orientation.z = q.z();
	this->cylinder_pose.orientation.w = q.w();

}

void
CW3::printMat (Eigen::Matrix4d T, std::string hom_mat)
{
  std::string space {" "};
  std::string end_line {"\n"};
  std::string op_message {hom_mat + " \n"};
  for (int i {0}; i < 4; i++)
  {
    for (int j {0}; j < 4; j++)
    {
      // if the number is very small, regard as 0.0
      std::stringstream stream;
      if (abs(T(i, j)) < 0.000000000000001) T(i, j) = 0;
      // print number with one decimal place
      stream << std::fixed << std::setprecision(3) << T(i,j);
      std::string to_add = stream.str();
      op_message = op_message + to_add + space;
    }
    op_message = op_message + end_line;
  }
  ROS_INFO_STREAM(op_message);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::convertCToW()
{

	// get transformation between panda_link0 and panda_hand
    this->listener_.lookupTransform ("panda_link0", "camera_rgb_optical_frame",
                            ros::Time(), this->transf_Q2);

	// convert StampedTransform to homogeneous matrix
	Eigen::Matrix4d T_0_c;
	T_0_c = transformToHom (this->transf_Q2);

	// convert PoseStamped to homogeneous matrix
	Eigen::Matrix4d T_c_cyl;
	T_c_cyl = transPoseToHom (this->cylinder_cam_pose_);

	//Convert from camera to panda_link0
	Eigen::Matrix4d T_0_cyl;
	T_0_cyl = T_0_c * T_c_cyl;

	//convert homogeneouse matrix back to Pose
	this->transHomToPose (T_0_cyl);

}
