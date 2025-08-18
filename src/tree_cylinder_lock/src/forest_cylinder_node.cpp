// File: src/forest_cylinder_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>                // SACSegmentationFromNormals
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZ;

static void pubCloud(ros::Publisher& pub,
                     const pcl::PointCloud<PointT>::ConstPtr& cloud,
                     const std::string& frame_id)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  pub.publish(msg);
}

static visualization_msgs::Marker makeLine(const Eigen::Vector3f& p,
                                           const Eigen::Vector3f& dir,
                                           int id,
                                           const std::string& frame_id,
                                           double length=5.0)
{
  // Draw a vertical axis line through a point in both directions
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = ros::Time::now();
  m.ns = "cyl_axis";
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.03; // line thickness
  m.color.a = 1.0; m.color.r = 0.1; m.color.g = 0.9; m.color.b = 0.1;

  geometry_msgs::Point a, b;
  Eigen::Vector3f u = dir.normalized();
  Eigen::Vector3f p1 = p - 0.5f*length*u;
  Eigen::Vector3f p2 = p + 0.5f*length*u;
  a.x = p1.x(); a.y = p1.y(); a.z = p1.z();
  b.x = p2.x(); b.y = p2.y(); b.z = p2.z();
  m.points.push_back(a);
  m.points.push_back(b);
  return m;
}

static visualization_msgs::Marker makeSphere(const Eigen::Vector3f& p,
                                             int id,
                                             const std::string& frame_id,
                                             double d=0.15)
{
  visualization_msgs::Marker s;
  s.header.frame_id = frame_id; s.header.stamp = ros::Time::now();
  s.ns = "cyl_centroid"; s.id = id; s.type = visualization_msgs::Marker::SPHERE;
  s.action = visualization_msgs::Marker::ADD;
  s.pose.position.x = p.x(); s.pose.position.y = p.y(); s.pose.position.z = p.z();
  s.scale.x = s.scale.y = s.scale.z = d;
  s.color.a = 1.0; s.color.r = 0.2; s.color.g = 0.2; s.color.b = 1.0;
  return s;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "forest_cylinder_node");
  ros::NodeHandle nh, pnh("~");

  // ── Parameters ────────────────────────────────────────────────────────────────
  std::string pcd_path = "forest2.pcd";
  std::string frame_id = "map";
  // Basic ROI
  double z_min = 0.2, z_max = 7.0;
  // Downsample & denoise
  double voxel_leaf = 0.05; // 5 cm
  bool   use_sor = true;    // statistical outlier removal
  int    sor_mean_k = 24;   double sor_std = 1.5;
  // Normals
  double normal_radius = 0.3; // 15 cm neighborhood
  // Cylinder model
  double cyl_weight = 0.1, cyl_dist = 0.4; int cyl_iter = 20000;
  double rmin = 0.05, rmax = 0.45; // 5–35 cm radius (10–70 cm diameter)
  // Axis prior (vertical) tolerance (degrees)
  double eps_angle_deg = 45.0; // ±15° from vertical
  // Multi-detection loop
  int max_cylinders = 12;
  int min_inliers = 400; // reject tiny inlier sets
  bool use_plane_removal = false; // forests: usually false

  pnh.param<std::string>("pcd_path", pcd_path, pcd_path);
  pnh.param<std::string>("frame_id", frame_id, frame_id);
  pnh.param<double>("z_min", z_min, z_min);
  pnh.param<double>("z_max", z_max, z_max);
  pnh.param<double>("voxel_leaf", voxel_leaf, voxel_leaf);
  pnh.param<bool>("use_sor", use_sor, use_sor);
  pnh.param<int>("sor_mean_k", sor_mean_k, sor_mean_k);
  pnh.param<double>("sor_std", sor_std, sor_std);
  pnh.param<double>("normal_radius", normal_radius, normal_radius);
  pnh.param<double>("cyl_weight", cyl_weight, cyl_weight);
  pnh.param<double>("cyl_dist", cyl_dist, cyl_dist);
  pnh.param<int>("cyl_iter", cyl_iter, cyl_iter);
  pnh.param<double>("radius_min", rmin, rmin);
  pnh.param<double>("radius_max", rmax, rmax);
  pnh.param<double>("eps_angle_deg", eps_angle_deg, eps_angle_deg);
  pnh.param<int>("max_cylinders", max_cylinders, max_cylinders);
  pnh.param<int>("min_inliers", min_inliers, min_inliers);
  pnh.param<bool>("use_plane_removal", use_plane_removal, use_plane_removal);

  // ── Publishers ───────────────────────────────────────────────────────────────
  auto pub_raw       = nh.advertise<sensor_msgs::PointCloud2>("forest/debug/raw", 1, true);
  auto pub_crop      = nh.advertise<sensor_msgs::PointCloud2>("forest/debug/crop", 1, true);
  auto pub_voxel     = nh.advertise<sensor_msgs::PointCloud2>("forest/debug/voxel", 1, true);
  auto pub_filtered  = nh.advertise<sensor_msgs::PointCloud2>("forest/debug/filtered", 1, true);
  auto pub_inliers   = nh.advertise<sensor_msgs::PointCloud2>("forest/debug/cylinder_inliers", 1, true);
  auto pub_remaining = nh.advertise<sensor_msgs::PointCloud2>("forest/debug/remaining", 1, true);
  auto pub_markers   = nh.advertise<visualization_msgs::MarkerArray>("forest/debug/markers", 1, true);

  // ── Load PCD ────────────────────────────────────────────────────────────────
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT>(pcd_path, *cloud) < 0) {
    ROS_ERROR_STREAM("Couldn't read PCD: " << pcd_path);
    return 1;
  }
  ROS_INFO("Loaded %zu points from %s", cloud->size(), pcd_path.c_str());
  pubCloud(pub_raw, cloud, frame_id);

  // ── Crop by Z ───────────────────────────────────────────────────────────────
  pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>);
  {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.filter(*cloud_crop);
  }
  ROS_INFO("After Z crop [%.2f, %.2f]: %zu pts", z_min, z_max, cloud_crop->size());
  pubCloud(pub_crop, cloud_crop, frame_id);

  // ── Voxel downsample ────────────────────────────────────────────────────────
  pcl::PointCloud<PointT>::Ptr cloud_voxel(new pcl::PointCloud<PointT>);
  if (voxel_leaf > 1e-6) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_crop);
    vg.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
    vg.filter(*cloud_voxel);
  } else {
    *cloud_voxel = *cloud_crop;
  }
  ROS_INFO("After voxel (%.3f m): %zu pts", voxel_leaf, cloud_voxel->size());
  pubCloud(pub_voxel, cloud_voxel, frame_id);

  // ── Statistical outlier removal ─────────────────────────────────────────────
  pcl::PointCloud<PointT>::Ptr cloud_filt(new pcl::PointCloud<PointT>);
  if (use_sor) {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_voxel);
    sor.setMeanK(sor_mean_k);
    sor.setStddevMulThresh(sor_std);
    sor.filter(*cloud_filt);
  } else {
    *cloud_filt = *cloud_voxel;
  }
  ROS_INFO("After SOR: %zu pts", cloud_filt->size());
  pubCloud(pub_filtered, cloud_filt, frame_id);

  // ── Compute normals (radius-based, OMP) ─────────────────────────────────────
  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  {
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud_filt);
    ne.setSearchMethod(kdtree);
    ne.setRadiusSearch(normal_radius);
    unsigned int n_threads = std::thread::hardware_concurrency();
    int threads = (n_threads > 0 && n_threads < 128) ? static_cast<int>(n_threads) : 8;
    ne.setNumberOfThreads(threads);
    ne.compute(*normals);
  }
  ROS_INFO("Normals: %zu (radius=%.2f m)", normals->size(), normal_radius);

  // Optionally remove a plane (not recommended in forest unless you know there is one)
  pcl::PointCloud<PointT>::Ptr working(new pcl::PointCloud<PointT>(*cloud_filt));
  pcl::PointCloud<pcl::Normal>::Ptr working_normals(new pcl::PointCloud<pcl::Normal>(*normals));

  if (use_plane_removal) {
    pcl::ModelCoefficients::Ptr coeff_plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segp;
    segp.setOptimizeCoefficients(true);
    segp.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    segp.setNormalDistanceWeight(0.1);
    segp.setMethodType(pcl::SAC_RANSAC);
    segp.setMaxIterations(2000);
    segp.setDistanceThreshold(0.03);
    segp.setInputCloud(working);
    segp.setInputNormals(working_normals);
    segp.segment(*inliers_plane, *coeff_plane);

    if (!inliers_plane->indices.empty()) {
      pcl::ExtractIndices<PointT> ex; ex.setInputCloud(working); ex.setIndices(inliers_plane); ex.setNegative(true);
      pcl::ExtractIndices<pcl::Normal> exn; exn.setInputCloud(working_normals); exn.setIndices(inliers_plane); exn.setNegative(true);
      pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
      pcl::PointCloud<pcl::Normal>::Ptr tmpn(new pcl::PointCloud<pcl::Normal>);
      ex.filter(*tmp); exn.filter(*tmpn);
      working.swap(tmp); working_normals.swap(tmpn);
      ROS_INFO("Plane removed: remaining %zu pts", working->size());
    } else {
      ROS_WARN("No plane removed (as expected in forest).");
    }
  }

  // ── Multi-cylinder extraction loop ──────────────────────────────────────────
  visualization_msgs::MarkerArray markers;
  pcl::PointCloud<PointT>::Ptr remaining(new pcl::PointCloud<PointT>(*working));
  pcl::PointCloud<pcl::Normal>::Ptr remaining_normals(new pcl::PointCloud<pcl::Normal>(*working_normals));

  int found = 0;
  for (int i = 0; i < max_cylinders; ++i) {
    if (remaining->size() < min_inliers) break;

    pcl::ModelCoefficients::Ptr coeff_cyl(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cyl(new pcl::PointIndices);

    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(cyl_weight);
    seg.setMaxIterations(cyl_iter);
    seg.setDistanceThreshold(cyl_dist);
    seg.setRadiusLimits(rmin, rmax);
    // Critical: constrain to near-vertical axis
    seg.setAxis(Eigen::Vector3f(0.f, 0.f, 1.f));
    seg.setEpsAngle(static_cast<float>(eps_angle_deg * M_PI / 180.0));
    seg.setInputCloud(remaining);
    seg.setInputNormals(remaining_normals);
    seg.segment(*inliers_cyl, *coeff_cyl);

    if (inliers_cyl->indices.size() < min_inliers) {
      ROS_INFO("No (more) strong cylinders: inliers=%zu", inliers_cyl->indices.size());
      break;
    }

    // Extract this cylinder's inlier cloud
    pcl::PointCloud<PointT>::Ptr cyl_cloud(new pcl::PointCloud<PointT>);
    {
      pcl::ExtractIndices<PointT> ex;
      ex.setInputCloud(remaining);
      ex.setIndices(inliers_cyl);
      ex.setNegative(false);
      ex.filter(*cyl_cloud);
    }

    // Publish/debug current cylinder inliers
    pubCloud(pub_inliers, cyl_cloud, frame_id);

    // Compute centroid
    Eigen::Vector4f c4; pcl::compute3DCentroid(*cyl_cloud, c4);
    Eigen::Vector3f c = c4.head<3>();
    // Cylinder direction is coeffs[3..5]
    Eigen::Vector3f axis(coeff_cyl->values[3], coeff_cyl->values[4], coeff_cyl->values[5]);

    ROS_INFO("[Cyl %d] inliers=%zu  centroid=(%.2f, %.2f, %.2f)  radius=%.3f  axis=(%.2f,%.2f,%.2f)",
             i, cyl_cloud->size(), c.x(), c.y(), c.z(),
             coeff_cyl->values[6], axis.x(), axis.y(), axis.z());

    // Markers: axis line + centroid
    markers.markers.push_back(makeLine(c, axis, 1000 + i, frame_id, /*length*/ 6.0));
    markers.markers.push_back(makeSphere(c, 2000 + i, frame_id, /*diam*/ 0.20));

    // Remove inliers from remaining (to find more trees)
    {
      pcl::ExtractIndices<PointT> ex;
      ex.setInputCloud(remaining);
      ex.setIndices(inliers_cyl);
      ex.setNegative(true);
      pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
      ex.filter(*tmp);
      remaining.swap(tmp);
    }
    {
      pcl::ExtractIndices<pcl::Normal> exn;
      exn.setInputCloud(remaining_normals);
      exn.setIndices(inliers_cyl);
      exn.setNegative(true);
      pcl::PointCloud<pcl::Normal>::Ptr tmpn(new pcl::PointCloud<pcl::Normal>);
      exn.filter(*tmpn);
      remaining_normals.swap(tmpn);
    }

    ++found;
  }

  ROS_INFO("Total cylinders found: %d", found);
  pub_markers.publish(markers);
  pubCloud(pub_remaining, remaining, frame_id);

  // Keep alive for RViz
  ros::Rate r(0.5);
  while (ros::ok()) {
    pubCloud(pub_raw, cloud, frame_id);
    pubCloud(pub_crop, cloud_crop, frame_id);
    pubCloud(pub_voxel, cloud_voxel, frame_id);
    pubCloud(pub_filtered, cloud_filt, frame_id);
    pub_markers.publish(markers);
    pubCloud(pub_remaining, remaining, frame_id);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
