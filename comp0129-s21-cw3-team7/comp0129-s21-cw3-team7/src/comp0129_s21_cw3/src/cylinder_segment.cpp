/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Ridhwan Luthra.
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
*   * Neither the name of Ridhwan Luthra nor the names of its
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
*********************************************************************/

/* Author: Ridhwan Luthra */
/* Modified author: Alejandro Halpern, Andras Nagy, Tim Andersson,
                    Xingjian Lu, Dimitrios Kanoulas */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_ros/point_cloud.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      ROS_INFO("Average framerate(%f Hz)\n", double(count)/double(now - last)); \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

class CylinderSegment
{
public:
  CylinderSegment ()
  {

    // ROS node generation
    ros::NodeHandle nh;

    // Initialize subscriber to the raw point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1,
                                        &CylinderSegment::cloudCB, this);

    // Initialize subscriber to the choice key press
    ros::Subscriber choice_sub = nh.subscribe ("/choices", 1,
                                        &CylinderSegment::choiceCallback, this);

    // Initialize the publisher of  pointcloud2 to /pc2
    pc2_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc2", 1);

    // Initialze the publisher of cylinder_pose to /cylinder_pose
    c_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cylinder_pose", 1);

    // Initialize the publisher of cylinder_size to /cylinder_size
    c_size_pub = nh.advertise<std_msgs::Float64MultiArray>("/cylinder_size", 1);

    //searchthis
    pubpub = nh.advertise<sensor_msgs::PointCloud2>("/pc2_result", 1);

    // Spin
    ros::spin ();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the parameters of the cylinder add the cylinder to the
    * planning scene.
    *
    * @param cylinder_pose - cylinder pose containing position and orientation
    */
  void addCylinder (geometry_msgs::Pose cylinder_pose)
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "camera_rgb_optical_frame";
    collision_object.id = "cylinder";

    // Define a cylinder which will be added to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    /* Setting height of cylinder. */
    primitive.dimensions[0] = cylinder_params->height;
    /* Setting radius of cylinder. */
    primitive.dimensions[1] = cylinder_params->radius;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface.applyCollisionObject(collision_object);

    replace = false;
    if (points_not_found)
    {
      // if this is the first time a cylinder is detected
      // add cylinder as collision object
      planning_scene_interface.applyCollisionObject(collision_object);
      this->internal_cylinder_counter++;
      // adding new cylinder happens once when one is first detected
      points_not_found = false;
    } else if (replace)
    {
      // if a key press is detected and a filter is enabled/disabled
      // remove the old cylinder and replace with the updated one
      std::vector<std::string> ids;
      ids.push_back(collision_object.id);
      planning_scene_interface.removeCollisionObjects(ids);
      planning_scene_interface.applyCollisionObject(collision_object);
      // set replace to false until another key (f or p) is pressed
      replace = false;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief calculate position and orientation of cylinder and return as Pose
  */
  geometry_msgs::Pose getCylinderPose()
  {
    // Define a pose for the cylinder (specified relative to frame_id).
    geometry_msgs::Pose cylinder_pose;
    /* Computing and setting quaternion from axis angle representation. */
    Eigen::Vector3d cylinder_z_direction(cylinder_params->direction_vec[0],
                                         cylinder_params->direction_vec[1],
                                         cylinder_params->direction_vec[2]);
    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();
    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
    cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
    cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
    cylinder_pose.orientation.w = cos(angle / 2);

    // Setting the position of cylinder.
    cylinder_pose.position.x = cylinder_params->center_pt[0];
    cylinder_pose.position.y = cylinder_params->center_pt[1];
    cylinder_pose.position.z = cylinder_params->center_pt[2];

    return cylinder_pose;
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud containing just the cylinder, compute its
    *        center point and its height and store in cylinder_params.
    *
    *  @param cloud - Pointcloud containing just the cylinder.
    *  @param cylinder_params - Pointer to the struct AddCylinderParams.
    */
  void extractLocationHeight (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    double max_angle_y = 0.0;
    double min_angle_y = std::numeric_limits<double>::infinity();

    double lowest_point[3];
    double highest_point[3];
    // BEGIN_SUB_TUTORIAL extract_location_height
    // Consider a point inside the point cloud and imagine that point is formed
    // on a XY plane where the perpendicular distance from the plane to the
    // camera is Z. |br|
    // The perpendicular drawn from the camera to the plane hits at center of
    // the XY plane. |br|
    // We have the x and y coordinate of the point which is formed on the XY
    // plane. |br|
    // X is the horizontal axis and Y is the vertical axis. |br|
    // C is the center of the plane which is Z meter away from the center of
    // camera and A is any point on the plane. |br|
    // Now we know Z is the perpendicular distance from the point to the
    // camera. |br|
    // If you need to find the  actual distance d from the point to the camera,
    // you should calculate the hypotenuse-
    // |code_start| hypot(point.z, point.x);\ |code_end| |br|
    // angle the point made horizontally-
    // |code_start| atan2(point.z,point.x);\ |code_end| |br|
    // angle the point made Vertically-
    // |code_start| atan2(point.z, point.y);\ |code_end| |br|
    // Loop over the entire pointcloud.
    for (auto const point : cloud->points)
    {
      /* Find the coordinates of the highest point */
      if (atan2(point.z, point.y) < min_angle_y)
      {
        min_angle_y = atan2(point.z, point.y);
        lowest_point[0] = point.x;
        lowest_point[1] = point.y;
        lowest_point[2] = point.z;
      }
      /* Find the coordinates of the lowest point */
      else if (atan2(point.z, point.y) > max_angle_y)
      {
        max_angle_y = atan2(point.z, point.y);
        highest_point[0] = point.x;
        highest_point[1] = point.y;
        highest_point[2] = point.z;
      }
    }
    /* Store the center point of cylinder */
    cylinder_params->center_pt[0] = (highest_point[0] + lowest_point[0]) / 2;
    cylinder_params->center_pt[1] = (highest_point[1] + lowest_point[1]) / 2;
    cylinder_params->center_pt[2] = (highest_point[2] + lowest_point[2]) / 2;
    /* Store the height of cylinder */
    cylinder_params->height =
        sqrt(pow((lowest_point[0] - highest_point[0]), 2) +
            pow((lowest_point[1] - highest_point[1]), 2) +
                pow((lowest_point[2] - highest_point[2]), 2));
    // END_SUB_TUTORIAL
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given a pointcloud extract the ROI defined by the user.
    *
    * @param cloud - Pointcloud whose ROI needs to be extracted.
    * @param zlower - lower bound on z axis
    * @param zupper - upper bound on z axis
    */
  void passThroughFilter (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          double zlower, double zupper)
  {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    // min and max values in z axis to keep
    pass.setFilterLimits (zlower, zupper);
    pass.filter (*cloud);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud and pointer cloud_normals compute the point
    * normals and store in cloud_normals.
    *
    * @param cloud - Pointcloud.
    * @param cloud_normals - The point normals once computer will be stored in
    *                        this.
    * @param ksearch - parameter for setKSearch
    */
  void computeNormals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                       int ksearch)
  {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr
      tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch (ksearch);
    ne.compute (*cloud_normals);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the point normals and point indices, extract the normals for
    *        the indices.
    *
    * @param cloud_normals - Point normals.
    * @param inliers_plane - Indices whose normals need to be extracted.
    */
  void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                      pcl::PointIndices::Ptr inliers_plane)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud and indices of the plane, remove the plannar
    * region from the pointcloud.
    *
    * @param cloud - Pointcloud.
    * @param inliers_plane - Indices representing the plane.
    * @param maxit - max iteration
    * @param distThresh - distance threshold
    */
  void removePlaneSurface (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr inliers_plane, int maxit,
                           double distThresh)
  {
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
    segmentor.setOptimizeCoefficients (true);
    segmentor.setModelType (pcl::SACMODEL_PLANE);
    segmentor.setMethodType (pcl::SAC_RANSAC);

    /* run at max 1000 iterations before giving up */
    segmentor.setMaxIterations (maxit);

    /* tolerance for variation from model */
    segmentor.setDistanceThreshold (distThresh);
    segmentor.setInputCloud (cloud);

    /* Create the segmentation object for the planar model and set all the
     * parameters
     */
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    segmentor.segment (*inliers_plane, *coefficients_plane);

    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud (cloud);
    extract_indices.setIndices (inliers_plane);

    /* Remove the planar inliers, extract the rest */
    extract_indices.setNegative (true);
    extract_indices.filter (*cloud);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud, pointer to pcl::ModelCoefficients and point
    * normals extract the cylinder from the pointcloud and store the cylinder
    * parameters in coefficients_cylinder.
    *
    * @param cloud - Pointcloud whose plane is removed.
    * @param coefficients_cylinder - Cylinder parameters used to define an
    *                                infinite cylinder will be stored here.
    * @param cloud_normals - Point normals corresponding to the plane on which
    *                        cylinder is kept
    * @param ndw - normal distance weight
    * @param maxit - max iteration
    * @param distThresh - distance threshold
    * @param radLow - lower bound of radius
    * @param radHigh - upper bound of radius
    */
  void extractCylinder (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        pcl::ModelCoefficients::Ptr coefficients_cylinder,
                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                        double ndw, int maxit, double distThresh,
                        double radLow, double radHigh)
  {
    // Create the segmentation object for cylinder segmentation and set all the
    // parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    segmentor.setOptimizeCoefficients (true);
    segmentor.setModelType (pcl::SACMODEL_CYLINDER);
    segmentor.setMethodType (pcl::SAC_RANSAC);

    // Set the normal angular distance weight
    segmentor.setNormalDistanceWeight (ndw);

    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations (maxit);

    // tolerance for variation from model
    segmentor.setDistanceThreshold (distThresh);

    // min max values of radius in meters to consider
    segmentor.setRadiusLimits (radLow, radHigh);
    segmentor.setInputCloud (cloud);
    segmentor.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    segmentor.segment (*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud, apply VoxelGrid and StatisticalOutlierRemoval
    * to the pointcloud
    *
    * @param cloud - Pointcloud to be filtered
    * @param leafx, leafy, leafz - size of leaf on x, y, z direction
    * @param meanK - meanK of StatisticalOutlierRemoval
    * @param stdThresh - standard deviation threshold
    *                     of StatisticalOutlierRemoval
    */
  void filterEnvironment (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          float leafx, float leafy, float leafz,
                          int meanK, double stdThresh)
  {
    // initialise a voxel_grid and apply filter to the cloud
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud (cloud);
    voxel_grid.setLeafSize(leafx,leafy,leafz);
    voxel_grid.filter(*cloud);

    // initialise a StatisticalOutlierRemoval and apply filter to the cloud
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stdThresh);
    sor.filter(*cloud);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud, apply passThroughFilter based on given info
    *
    * @param cloud - Pointcloud to be filtered
    * @param xl, xu - lower bound and upper bound in x axis
    * @param yl, yu - lower bound and upper bound in y axis
    * @param zl, zu - lower bound and upper bound in z axis
    */
  void fastFilterEnvironment (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                              double xl, double xu, double yl, double yu,
                              double zl, double zu)
  {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);

    pass.setFilterFieldName ("x");
    // min and max values in x axis to keep
    pass.setFilterLimits (xl, xu);
    pass.filter (*cloud);

    pass.setFilterFieldName ("y");
    // min and max values in y axis to keep
    pass.setFilterLimits (yl, yu);
    pass.filter (*cloud);

    pass.setFilterFieldName ("z");
    // min and max values in z axis to keep
    pass.setFilterLimits (zl, zu);
    pass.filter (*cloud);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief toggle the environemnt filter and fast environment filter based on
    *         the key pressed
    *
    * @param input - message from the subscribed topic
    */
  void choiceCallback (const std_msgs::Char& input)
  {
    if (input.data == 'f'){
      // toggle environment filter
      this->filter = !this->filter;
      // trigger update of the cylinder in the planning scene
      replace = true;
    }
    if (input.data == 'p'){
      // toggle fast environment filter
      this->fast_filter = !this->fast_filter;
      // trigger update of the cylinder in the planning scene
      replace = true;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief replace the cloud data in the input message and publish it
    *
    * @param cloud - pointer to the filtered point cloud, contains data
    * @param input - pointer to the original point cloud message, contains
    *               header and other information
    */
  void publishPointCloud2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          const sensor_msgs::PointCloud2ConstPtr& input)
  {
    sensor_msgs::PointCloud2 msg = *input;
    pcl::toROSMsg(*cloud, msg);
    pc2_pub.publish(msg);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief print the number pointcloud points to console
    *
    * @param cloud - pointer to the pointcloud of interest
    */
  void printSize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    // ROS_INFO("%d data points (XYZRGB)\n", cloud->width * cloud->height);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief publish cylinder pose as PoseStamped, converted from Pose
    *
    * @param cylinder_pose - cylinder pose
    */
  void publishCylinderPose (geometry_msgs::Pose cylinder_pose)
  {
    geometry_msgs::PoseStamped c_pose;
    c_pose.header.frame_id = "camera_rgb_optical_frame";
    c_pose.header.stamp = ros::Time::now();
    c_pose.pose = cylinder_pose;
    c_pose_pub.publish(c_pose);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief publish cylinder size as Float64MultiArray, converted from Pose
    *
    * @param cylinder_pose - cylinder pose
  */
  void publishCylinderSize ()
  {
    std_msgs::Float64MultiArray c_size;
    c_size.data.resize(2);
    c_size.data[0] = cylinder_params->radius;
    c_size.data[1] = cylinder_params->height;
    c_size_pub.publish(c_size);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief callback function when receive input from camera
    *
    * @param input - the message from the camera topic
  */
  void cloudCB (const sensor_msgs::PointCloud2ConstPtr& input)
  {
    FPS_CALC ("cloudCB");

    // reset timer, mark start of function
    stopwatch.reset();

    // First, convert from sensor_msgs to pcl::PointXYZRGB which is needed for
    // most of the processing.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*input, *cloud);

    // filtering, depending on key pressed, different filters are enabled
    passThroughFilter (cloud, 0.3, 1.1);
    if (this->filter){
      filterEnvironment(cloud, 0.005f, 0.005f, 0.005f, 20, 0.5);
    }
    if (this->fast_filter){
      fastFilterEnvironment(cloud, -0.3, 0.2, -0.09, 0.41, 0.67, 1.17);
    }

    // print time spent on filtering then reset timer
    // ROS_INFO("filtering took %f miliseconds\n", stopwatch.getTime());
    stopwatch.reset();

    // publish filtered pc2
    publishPointCloud2(cloud, input);
    //print out pointcloud size
    printSize(cloud);

    // Declare normals and call function to compute point normals.
    pcl::PointCloud<pcl::Normal>::Ptr
      cloud_normals (new pcl::PointCloud<pcl::Normal>);
    computeNormals (cloud, cloud_normals, 50);

    // inliers_plane will hold the indices of the point cloud that correspond
    // to a plane.
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

    // Detect and eliminate the plane on which the cylinder is kept to ease the
    // process of finding the cylinder.
    removePlaneSurface (cloud, inliers_plane, 1000, 0.01);

    // We had calculated the point normals in a previous call to computeNormals,
    // now we will be extracting the normals that correspond to the plane on
    // which cylinder lies.
    // It will be used to extract the cylinder.
    extractNormals (cloud_normals, inliers_plane);

    // ModelCoefficients will hold the parameters using which we can define a
    // cylinder of infinite length.
    pcl::ModelCoefficients::Ptr
      coefficients_cylinder (new pcl::ModelCoefficients);

    // Extract the cylinder using SACSegmentation.
    extractCylinder (cloud, coefficients_cylinder, cloud_normals,
                      0.1, 10000, 0.05, 0, 1);

    // print time, mark end of segmentation
    // ROS_INFO("segmentation took %f miliseconds\n", stopwatch.getTime());

    if (cloud->points.empty())
    {
      ROS_ERROR_STREAM_NAMED ("cylinder_segment",
                              "Can't find the cylindrical component.");
      return;
    }

    //searchthis
    sensor_msgs::PointCloud2 msg = *input;
    pcl::toROSMsg(*cloud, msg);
    pubpub.publish(msg);

    // Storing Relevant Cylinder Values
    // We define a struct to hold the parameters that are actually needed for
    // defining a collision object completely.

    cylinder_params = new AddCylinderParams();
    // Store the radius of the cylinder.
    cylinder_params->radius = coefficients_cylinder->values[6];
    // Store direction vector of z-axis of cylinder.
    cylinder_params->direction_vec[0] = coefficients_cylinder->values[3];
    cylinder_params->direction_vec[1] = coefficients_cylinder->values[4];
    cylinder_params->direction_vec[2] = coefficients_cylinder->values[5];

    // Extracting Location and Height
    // Compute the center point of the cylinder using standard geometry
    extractLocationHeight(cloud);

    // Use the parameters extracted to calculate the position and orientation
    // of the cylinder
    geometry_msgs::Pose cylinder_pose = getCylinderPose();

    // add the cylinder to the planning scene as a collision object.
    if (this->internal_cylinder_counter == 0)
    {
      addCylinder(cylinder_pose);
    }

    // publish cylinder_pose and cylinder_size
    publishCylinderPose(cylinder_pose);
    publishCylinderSize ();

    // ROS_INFO("rad: %f\n", cylinder_params->radius);
    // ROS_INFO("height: %f\n", cylinder_params->height);
  }

public:
  // publisher for cylinder pose
  ros::Publisher c_pose_pub;
  // publisher for cylinder size
  ros::Publisher c_size_pub;
  // publisher for filtered point cloud
  ros::Publisher pc2_pub;

  // searchthis
  ros::Publisher pubpub;

  int internal_cylinder_counter {0};


private:
  // BEGIN_SUB_TUTORIAL param_struct
  // There are 4 fields and a total of 7 parameters used to define this.
  struct AddCylinderParams
  {
    /* Radius of the cylinder. */
    double radius;
    /* Direction vector towards the z-axis of the cylinder. */
    double direction_vec[3];
    /* Center point of the cylinder. */
    double center_pt[3];
    /* Height of the cylinder. */
    double height;
  };

  // Declare a variable of type AddCylinderParams and store relevant values
  // from ModelCoefficients.
  AddCylinderParams* cylinder_params;
  // END_SUB_TUTORIAL

  // if true, no cylinder was found so far, set to false otherwise
  bool points_not_found = true;
  // if true, filterEnvironment is enabled, false otherwise
  bool filter = false;
  // if true, fastFilterEnvironment is enabled, false otherwise
  bool fast_filter = false;
  // if true, either filterEnvironment or fastFilterEnvironment
  // is enabled/disabled in current spin and the scenen is set to be updated
  // set to false when scene is updated
  bool replace = false;
  // stopwatch to keep track of running time of filtering and segmentation.
  pcl::StopWatch stopwatch;

};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cylinder_segment");

  // Start the segmentor
  CylinderSegment ();

}
