#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <collision_avoidance/ObstacleInfo.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

class StereoObstacleDetector
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher  pub_obstacle_info_;

  // Operational parameters
  double min_range_;
  double max_range_;
  int min_obstacle_points_;
  double min_obstacle_size_;

public:
  StereoObstacleDetector() : nh_private_("~")
  {
    nh_private_.param("min_range", min_range_, 0.3);
    nh_private_.param("max_range", max_range_, 5.0);
    nh_private_.param("min_obstacle_points", min_obstacle_points_, 100);
    nh_private_.param("min_obstacle_size", min_obstacle_size_, 0.06);
    point_cloud_sub_ = nh_.subscribe<PointCloud>("point_cloud", 1, &StereoObstacleDetector::pointCloudCb, this);
    pub_obstacle_info_ = nh_private_.advertise<collision_avoidance::ObstacleInfo>("obstacle_info", 2, true);
  }

  void pointCloudCb(const PointCloud::ConstPtr& in_cloud)
  {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::copyPointCloud(*in_cloud, *cloud);

    // Remove nans
    std::vector<int> indicies;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indicies);

    // Voxel grid filter
    pcl::ApproximateVoxelGrid<Point> grid;
    grid.setLeafSize(0.05, 0.05, 0.05);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);

    // PassThrough
    pcl::PassThrough<Point> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_range_, max_range_);
    pass.setInputCloud(cloud);
    pass.filter(*cloud);

    // Init
    collision_avoidance::ObstacleInfo o_info;
    o_info.min_points = min_obstacle_points_;
    o_info.min_size = min_obstacle_size_;
    o_info.points = cloud->points.size();
    o_info.width = 0.0;
    o_info.height = 0.0;
    o_info.size = 0.0;
    o_info.distance = -1.0;

    // Count the number of points detected
    if (cloud->points.size() < min_obstacle_points_)
    {
      o_info.detection = false;
    }
    else
    {
      // Obstacle size (x,y)
      Point min_pt, max_pt;
      pcl::getMinMax3D(*cloud, min_pt, max_pt);
      double width  = fabs(max_pt.x - min_pt.x);
      double height = fabs(max_pt.y - min_pt.y);
      double size   = width * height;
      if (size > min_obstacle_size_)
      {
        o_info.width      = width;
        o_info.height     = height;
        o_info.size       = size;
        o_info.distance   = min_pt.z;
        o_info.detection  = true;
      }
      else
        o_info.detection  = false;
    }

    // Publish obstacle info
    if (pub_obstacle_info_.getNumSubscribers() > 0)
    {
      pub_obstacle_info_.publish(o_info);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_obstacle_detector");
  StereoObstacleDetector node;
  ros::spin();
  return 0;
}

