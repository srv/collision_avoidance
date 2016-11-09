#include <math.h>
#include <limits>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/lexical_cast.hpp>

#include <opencv2/plot.hpp>

#include <miniking_ros/AcousticBeam.h>
#include <collision_avoidance/ObstacleInfo.h>


class SonarObstacleDetector
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber acousticbeam_sub_;
  ros::Publisher  pub_obstacle_info_;
  ros::Publisher  intensities_pub_;
  ros::Publisher  peakdetector_pub_;

  // Operational parameters
  double min_range_;
  double max_range_;
  double min_obstacle_size_;

public:
  SonarObstacleDetector() : nh_private_("~")
  {
    // Params
    nh_private_.param("min_range", min_range_, 2.0);
    nh_private_.param("min_obstacle_size", min_obstacle_size_, 0.06);

    // Topics
    acousticbeam_sub_   = nh_.subscribe("/sonar", 1, &SonarObstacleDetector::acousticbeamCb, this);
    pub_obstacle_info_  = nh_private_.advertise<collision_avoidance::ObstacleInfo>("obstacle_info", 2, true);
    intensities_pub_    = nh_private_.advertise<sensor_msgs::Image>("intensities", 2, true);
    peakdetector_pub_   = nh_private_.advertise<sensor_msgs::Image>("peak_detector", 2, true);
  }

  void acousticbeamCb(const miniking_ros::AcousticBeamConstPtr beam)
  {
    // Copy and convert
    std::vector<double> intensities;
    std::vector<unsigned char> input_intensities(beam->intensities);
    for (size_t i=0; i<input_intensities.size(); i++)
    {
      int tmp = (int)input_intensities[i];
      intensities.push_back(boost::lexical_cast<double>(tmp));
    }

    // Remove intensities in the minimum range
    max_range_ = beam->range_max;
    float distance_btw_bins = max_range_/beam->bins;
    int N = round(min_range_/distance_btw_bins);
    intensities.erase(intensities.begin(), intensities.begin() + N);

    // Create the ranges vector
    std::vector<double> ranges;
    for (double r=min_range_; r<=max_range_; r=r+distance_btw_bins)
      ranges.push_back(r);

    // Low pass filter
    std::vector<double> filtered_intensities = lowPassFilter(intensities);

    // Publish debugging images
    if (intensities_pub_.getNumSubscribers() > 0)
    {
      cv::Mat original = drawGraph(ranges, intensities, cv::Scalar(50, 50, 255));
      cv::Mat filtered = drawGraph(ranges, filtered_intensities, cv::Scalar(50, 255, 50));

      // Combine 2 images
      cv::Mat combined;
      double alpha = 0.5;
      double beta = ( 1.0 - alpha );
      cv::addWeighted( original, alpha, filtered, beta, 0.0, combined);

      // Publish
      cv_bridge::CvImage ros_image;
      ros_image.image = combined.clone();
      ros_image.header.stamp = ros::Time::now();
      ros_image.encoding = "bgr8";
      intensities_pub_.publish(ros_image.toImageMsg());
    }

    if (peakdetector_pub_.getNumSubscribers() > 0)
    {
      cv::Mat combined = drawGraph(ranges, filtered_intensities, cv::Scalar(50, 255, 50));

      // Calc peaks
      std::vector<double> x_peak, y_peak;
      peakDetector(ranges, filtered_intensities, x_peak, y_peak);
      for (size_t i=0; i<x_peak.size(); i++)
      {
        // Convert point to image coordinates
        double u = combined.cols * x_peak[i] / (max_range_-min_range_);
        double v = combined.rows * (100.0-y_peak[i]) / 100;
        cv::Point2d p(u,v);
        cv::circle(combined, p, 5, cv::Scalar(50, 255, 50));
      }

      // Publish
      cv_bridge::CvImage ros_image;
      ros_image.image = combined.clone();
      ros_image.header.stamp = ros::Time::now();
      ros_image.encoding = "bgr8";
      peakdetector_pub_.publish(ros_image.toImageMsg());
    }

  }

  std::vector<double> lowPassFilter(const std::vector<double>& input,
                                    const int& order = 10,
                                    const double& gain = 1)
  {

    std::vector<double> output;
    for (size_t n=0; n<input.size(); n++)
    {
      size_t idx = n;
      double sum = 0.0;
      for (size_t k=0; k<order; k++)
      {
        if (idx < 0) break;
        sum += 0.01 * input[idx];
        idx--;
      }
      output.push_back(sum*gain);
    }

    return output;
  }

  void peakDetector(const std::vector<double>& x,
                    const std::vector<double>& y,
                    std::vector<double>& x_peak,
                    std::vector<double>& y_peak,
                    const double& delta = 0.8)
  {
    x_peak.clear();
    y_peak.clear();

    double mn = std::numeric_limits<double>::max();
    double mx = std::numeric_limits<double>::min();
    double mxpos = std::numeric_limits<double>::quiet_NaN();
    double mnpos = std::numeric_limits<double>::quiet_NaN();
    bool lookformax = true;

    for (size_t i=0; i<y.size(); i++)
    {
      if (y[i] > mx)
      {
        mx = y[i];
        mxpos = x[i];
      }
      if (y[i] < mn)
      {
        mn = y[i];
        mnpos = x[i];
      }

      if (lookformax)
      {
        if (y[i] < mx-delta)
        {
          x_peak.push_back(mxpos);
          y_peak.push_back(mx);
          mn = y[i];
          mnpos = x[i];
          lookformax = false;
        }
      }
      else
      {
        if (y[i] < mn+delta)
        {
          mx = y[i];
          mxpos = x[i];
          lookformax = true;
        }
      }
    }
  }

  cv::Mat drawGraph(const std::vector<double>& x,
                    const std::vector<double>& y,
                    const cv::Scalar& line_color,
                    const int& line_width = 2)
  {
    // Convert to cv::Mat
    cv::Mat x_data( x.size(), 1, CV_64F );
    cv::Mat y_data( y.size(), 1, CV_64F );
    for (size_t i=0; i<x.size(); i++)
      x_data.at<double>(i,0) = x[i];
    for (size_t i=0; i<y.size(); i++)
      y_data.at<double>(i,0) = y[i];

    // Draw
    cv::Mat tmp, plot_result;
    cv::Ptr<cv::plot::Plot2d> plot = cv::plot::createPlot2d(x_data, y_data);
    plot->setMaxX(max_range_);
    plot->setMinX(min_range_);
    plot->setMaxY(100);
    plot->setMinY(0);
    plot->setPlotTextColor(cv::Scalar(50, 50, 50));
    plot->setPlotBackgroundColor(cv::Scalar(50, 50, 50));
    plot->setPlotLineColor(line_color);
    plot->setPlotLineWidth(line_width);
    plot->render(tmp);
    cv::flip(tmp, plot_result, 0);
    return plot_result;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sonar_obstacle_detector");
  SonarObstacleDetector node;
  ros::spin();
  return 0;
}

