#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h> // also contains SACSegmentationFromNormals on some distros
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZ;

static void toROS(ros::Publisher& pub,
                  const pcl::PointCloud<PointT>::ConstPtr& cloud,
                  const std::string& frame_id)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cylinder_pcd_node");
  ros::NodeHandle nh, pnh("~");

  // Params (defaults match the PCL tutorial)
  std::string pcd_path = "table_scene_mug_stereo_textured.pcd";
  std::string frame_id = "map";
  double z_min = 0.0, z_max = 1.5;
  int normal_k = 50;
  double plane_weight = 0.1, plane_dist = 0.03; int plane_iter = 100;
  double cyl_weight = 0.1, cyl_dist = 0.05; int cyl_iter = 10000;
  double rmin = 0.0, rmax = 0.10;

  pnh.param<std::string>("pcd_path", pcd_path, pcd_path);
  pnh.param<std::string>("frame_id", frame_id, frame_id);
  pnh.param<double>("z_min", z_min, z_min);
  pnh.param<double>("z_max", z_max, z_max);
  pnh.param<int>("normal_k", normal_k, normal_k);
  pnh.param<double>("plane_weight", plane_weight, plane_weight);
  pnh.param<double>("plane_dist", plane_dist, plane_dist);
  pnh.param<int>("plane_iter", plane_iter, plane_iter);
  pnh.param<double>("cyl_weight", cyl_weight, cyl_weight);
  pnh.param<double>("cyl_dist", cyl_dist, cyl_dist);
  pnh.param<int>("cyl_iter", cyl_iter, cyl_iter);
  pnh.param<double>("radius_min", rmin, rmin);
  pnh.param<double>("radius_max", rmax, rmax);

  // Publishers (debug topics)
  auto pub_raw      = nh.advertise<sensor_msgs::PointCloud2>("pcd/debug/raw", 1, true);
  auto pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("pcd/debug/filtered_z", 1, true);
  auto pub_plane    = nh.advertise<sensor_msgs::PointCloud2>("pcd/debug/plane", 1, true);
  auto pub_no_plane = nh.advertise<sensor_msgs::PointCloud2>("pcd/debug/no_plane", 1, true);
  auto pub_cylinder = nh.advertise<sensor_msgs::PointCloud2>("pcd/debug/cylinder", 1, true);
  auto pub_markers  = nh.advertise<visualization_msgs::MarkerArray>("pcd/debug/markers", 1, true);

  // Load PCD (exactly like the tutorial)
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PCDReader reader;
  if (reader.read(pcd_path, *cloud) < 0) {
    ROS_ERROR_STREAM("Couldn't read PCD: " << pcd_path);
    return 1;
  }
  ROS_INFO("Loaded %zu points from %s", cloud->size(), pcd_path.c_str());
  toROS(pub_raw, cloud, frame_id);

  // PassThrough Z [z_min, z_max]
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.filter(*cloud_filtered);
  }
  ROS_INFO("After Z filter [%.2f, %.2f]: %zu pts", z_min, z_max, cloud_filtered->size());
  toROS(pub_filtered, cloud_filtered, frame_id);

  // Normals on filtered cloud
  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud_filtered);
    ne.setSearchMethod(kdtree);
    ne.setKSearch(normal_k);
    ne.compute(*normals);
  }
  ROS_INFO("Normals computed: %zu (K=%d)", normals->size(), normal_k);

  // Plane model with normals (table)
  pcl::ModelCoefficients::Ptr coeff_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  {
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(plane_weight);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(plane_iter);
    seg.setDistanceThreshold(plane_dist);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(normals);
    seg.segment(*inliers_plane, *coeff_plane);
  }
  ROS_INFO("Plane inliers: %zu", inliers_plane->indices.size());

  // Extract plane & save debug
  pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
  {
    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(cloud_filtered);
    ex.setIndices(inliers_plane);
    ex.setNegative(false);
    ex.filter(*cloud_plane);
  }
  ROS_INFO("Plane component: %zu pts", cloud_plane->size());
  toROS(pub_plane, cloud_plane, frame_id);

  // Remove plane → keep rest; also normals of the rest
  pcl::PointCloud<PointT>::Ptr cloud_rest(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr normals_rest(new pcl::PointCloud<pcl::Normal>);
  {
    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(cloud_filtered);
    ex.setIndices(inliers_plane);
    ex.setNegative(true);
    ex.filter(*cloud_rest);

    pcl::ExtractIndices<pcl::Normal> exn;
    exn.setInputCloud(normals);
    exn.setIndices(inliers_plane);
    exn.setNegative(true);
    exn.filter(*normals_rest);
  }
  ROS_INFO("Non-plane points: %zu | normals: %zu", cloud_rest->size(), normals_rest->size());
  toROS(pub_no_plane, cloud_rest, frame_id);

  // Cylinder from normals (mug)
  pcl::ModelCoefficients::Ptr coeff_cyl(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cyl(new pcl::PointIndices);
  {
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(cyl_weight);
    seg.setMaxIterations(cyl_iter);
    seg.setDistanceThreshold(cyl_dist);
    seg.setRadiusLimits(rmin, rmax);
    seg.setInputCloud(cloud_rest);
    seg.setInputNormals(normals_rest);
    seg.segment(*inliers_cyl, *coeff_cyl);
  }
  ROS_INFO("Cylinder inliers: %zu", inliers_cyl->indices.size());

  // Extract cylinder inliers & centroid
  pcl::PointCloud<PointT>::Ptr cloud_cyl(new pcl::PointCloud<PointT>);
  {
    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(cloud_rest);
    ex.setIndices(inliers_cyl);
    ex.setNegative(false);
    ex.filter(*cloud_cyl);
  }
  toROS(pub_cylinder, cloud_cyl, frame_id);

  Eigen::Vector4f c4; 
  if (!cloud_cyl->empty()) {
    pcl::compute3DCentroid(*cloud_cyl, c4);
    ROS_INFO("Cylinder centroid: (%.3f, %.3f, %.3f)", c4[0], c4[1], c4[2]);
  } else {
    ROS_WARN("No cylindrical component found.");
  }

  // Minimal marker (centroid sphere if found)
  visualization_msgs::MarkerArray arr;
  if (!cloud_cyl->empty()) {
    visualization_msgs::Marker s;
    s.header.frame_id = frame_id; s.header.stamp = ros::Time::now();
    s.ns = "pcd_cyl"; s.id = 1; s.type = visualization_msgs::Marker::SPHERE;
    s.action = visualization_msgs::Marker::ADD;
    s.pose.position.x = c4[0]; s.pose.position.y = c4[1]; s.pose.position.z = c4[2];
    s.scale.x = s.scale.y = s.scale.z = 0.06;
    s.color.a = 1.0; s.color.r = 0.1; s.color.g = 1.0; s.color.b = 0.1;
    arr.markers.push_back(s);
  }
  pub_markers.publish(arr);

  // Keep node alive to let RViz subscribe; republish every 2s
  ros::Rate r(0.5);
  while (ros::ok()) {
    toROS(pub_raw, cloud, frame_id);
    toROS(pub_filtered, cloud_filtered, frame_id);
    toROS(pub_plane, cloud_plane, frame_id);
    toROS(pub_no_plane, cloud_rest, frame_id);
    toROS(pub_cylinder, cloud_cyl, frame_id);
    pub_markers.publish(arr);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
