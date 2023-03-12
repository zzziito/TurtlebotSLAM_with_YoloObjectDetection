#include <ros/ros.h>
#include <image_transport/camera_common.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <laser_geometry/laser_geometry.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
// #include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

 #include <tf/transform_broadcaster.h>

#include <iostream>
using namespace std;

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

using namespace sensor_msgs;
using namespace message_filters;

class YoloTurtle
{
private:
  bool checkpoint;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  Subscriber<Image> image_sub_;
  image_transport::Publisher image_pub_;

  Subscriber<CameraInfo> info_sub_;

  Subscriber<LaserScan> scan_sub_;
  ros::Publisher scan_in_camera_pub_;
  ros::Publisher scan_non_bbox_pub_;
  ros::Publisher scan_in_bbox_pub_;

  Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub_;

  typedef sync_policies::ApproximateTime
    <
    Image,
    CameraInfo, 
    LaserScan, 
    darknet_ros_msgs::BoundingBoxes
    >
    MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  laser_geometry::LaserProjection projector_;

  enum pointClass
  {
    ptOutOfImage = 0, //이미지 외부
    ptOutsideBbox = 1, //Bounding Box 외부
    ptInsideBbox = 2 //Bounding Box 내부
  };

  pointClass getPointClass(const cv::Point2d &point,
                           unsigned imgWidth, unsigned imgHeight,
                           const darknet_ros_msgs::BoundingBoxes &bounding_boxes)
  {
    // cout << point.x << endl;
    // cout << point.y << endl;

    // if (point.x < 0 || point.x >= imgWidth || point.y < 0 || point.y >= imgHeight)
    if (point.x < 0 || point.x >= imgWidth)
    {
      return ptOutOfImage;
    }
    else
    {
      for (unsigned idx = 0; idx < bounding_boxes.bounding_boxes.size(); idx++)
      {
        const darknet_ros_msgs::BoundingBox &boundingBox = bounding_boxes.bounding_boxes.at(idx);
        // if (point.x >= boundingBox.xmin && point.x <= boundingBox.xmax && point.y >= boundingBox.ymin && point.y <= boundingBox.ymax)
        if (point.x >= boundingBox.xmin && point.x <= boundingBox.xmax)
        {
          return ptInsideBbox;
        }
      }
      return ptOutsideBbox;
    }
  }

public:
  YoloTurtle()
      : it_(nh_)
  {
    checkpoint = true;
    std::string image_topic = nh_.resolveName("/image_raw");
    std::string scan_topic = nh_.resolveName("/scan");
    std::string bounding_boxes_topic = nh_.resolveName("/darknet_ros/bounding_boxes");

    std::string info_topic = nh_.resolveName("/camera_info");
    // std::string info_topic = image_transport::getCameraInfoTopic(image_topic);

    image_sub_.subscribe(nh_, image_topic, 1);
    info_sub_.subscribe(nh_, info_topic, 1);
    scan_sub_.subscribe(nh_, scan_topic, 1);
    bounding_boxes_sub_.subscribe(nh_, bounding_boxes_topic, 1);
    sync_.reset(new Sync(MySyncPolicy(80), 
                         image_sub_,info_sub_, scan_sub_, bounding_boxes_sub_));

    sync_->registerCallback(boost::bind(&YoloTurtle::callback,
                                        this, _1, _2, _3,_4));
    image_pub_ = it_.advertise("image_out", 1);
    // 카메라 시야 안에 있는 포인트
    scan_in_camera_pub_ = nh_.advertise<LaserScan>("inside_camera", 1);
    // 바운딩 박스 외부에 있는 포인트
    scan_non_bbox_pub_ = nh_.advertise<LaserScan>("non_bbox", 1);
    // 바운딩 박스 내부에 있는 포인트
    scan_in_bbox_pub_ = nh_.advertise<LaserScan>("in_bbox", 1);
  }

  void callback(
      const ImageConstPtr &image_msg,
      const CameraInfoConstPtr &info_msg,
      const LaserScanConstPtr &scan_msg,
      const darknet_ros_msgs::BoundingBoxesConstPtr &bounding_boxes_msg)
  {

    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    cout << "inside callback" << endl;


    try
    {
      input_bridge = cv_bridge::toCvCopy(image_msg, image_encodings::BGR8);
      image = input_bridge->image;
      cout << "successfully converting image" << endl;
    }
    catch (cv_bridge::Exception &ex)
    {
      ROS_ERROR("[laser_scan_filter] Failed to convert image\n%s", ex.what());
      return;
    }

    cam_model_.fromCameraInfo(info_msg);
    LaserScan scan_in_camera(*scan_msg);
    LaserScan scan_non_bbox(*scan_msg);
    LaserScan scan_in_bbox(*scan_msg);

    std::string error_msg;
    
    
    bool success = tf_listener_.waitForTransform(
        cam_model_.tfFrame(),
        scan_msg->header.frame_id,
        // scan_msg->header.stamp + ros::Duration().fromSec(scan_msg->ranges.size() * double(scan_msg->time_increment)),
        ros::Time(0),
        ros::Duration(5.0),
        ros::Duration(0.0001),
        &error_msg);
    if (!success)
    {
      ROS_WARN("[laser_scan_filter] TF exception:\n%s", error_msg.c_str());
      return;
    }
    sensor_msgs::PointCloud2 laser_cloud;
    try
    {
      projector_.transformLaserScanToPointCloud(cam_model_.tfFrame(), *scan_msg, laser_cloud, tf_listener_);
    }
    catch (tf::TransformException &ex)
    {
      if (checkpoint)
      {
        ROS_WARN_THROTTLE(1, "[laser_scan_filter] Transform unavailable %s", ex.what());
        cout << "transform exception" << endl;
        return;
      }
      else
      {
        ROS_INFO_THROTTLE(.3, "[laser_scan_filter] Ignoring Scan: Waiting for TF");
      }
      return;
    }
    const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
    const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
    const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
    // const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");
    const int z_idx_c = 1;
    if (i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1)
    {
      ROS_INFO_THROTTLE(.3, "[laser_scan_filter] x, y, z and index fields are required, skipping scan");
      return;
    }
    const unsigned i_idx_offset = laser_cloud.fields[unsigned(i_idx_c)].offset;
    const unsigned x_idx_offset = laser_cloud.fields[unsigned(x_idx_c)].offset;
    const unsigned y_idx_offset = laser_cloud.fields[unsigned(y_idx_c)].offset;
    const unsigned z_idx_offset = laser_cloud.fields[unsigned(z_idx_c)].offset;

    const unsigned pstep = laser_cloud.point_step;
    const long int pcount = laser_cloud.width * laser_cloud.height;
    const long int limit = pstep * pcount;

    unsigned imgWidth = info_msg->width;
    unsigned imgHeight = info_msg->height;
    unsigned i_idx, x_idx, y_idx, z_idx;

    int count = 0;
    for (
        i_idx = i_idx_offset,
        x_idx = x_idx_offset,
        y_idx = y_idx_offset,
        z_idx = z_idx_offset;

        x_idx < limit;

        i_idx += pstep,
        x_idx += pstep,
        y_idx += pstep,
        z_idx += pstep
    )
    {

        

        float x = *((float *)(&laser_cloud.data[x_idx]));
        float y = *((float *)(&laser_cloud.data[y_idx]));
        float z = *((float *)(&laser_cloud.data[z_idx]));
        int index = *((int *)(&laser_cloud.data[i_idx]));

      if(z > 0)
      {

        // cout << "x : " << x << endl;
        // cout << "y : " << y << endl;
        cout << "z : " << z << endl;
        cout << "index : " << index << endl;
    
        cv::Point3d pt_cv((double(x)), (double(y)), (double(z)));
        // cv::Point3d pt_cv2((double(x)) + 0.01, (double(y)), (double(z)));
        cv::Point2d uv;
        // cv::Point2d uv((double(x)), (double(y)));
        uv = cam_model_.project3dToPixel(pt_cv);
        
        
        cout << "uv : " << uv << endl; 


        pointClass pt_class = getPointClass(uv, imgWidth, imgHeight, *bounding_boxes_msg);
        cv::Scalar color;
    switch (pt_class)
        {
        case ptOutOfImage:
            scan_in_camera.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
            scan_non_bbox.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
            scan_in_bbox.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
            cout << "ptOutOfImage" << endl;
            break;
        case ptOutsideBbox:
            scan_in_bbox.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
            color = CV_RGB(0, 255, 0);
            cout << "ptOutsideBbox" << endl;
            break;
        case ptInsideBbox:
            scan_non_bbox.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
            color = CV_RGB(255, 0, 0);
            cout << "ptInsideBbox" << endl;
            break;
        }
    if (pt_class != ptOutOfImage)
        {
            cv::Point2d uv2;
            uv2 = cam_model_.project3dToPixel(pt_cv);
            int radius = abs(int(uv.x - uv2.x));
            if (radius < 1)
            {
                radius = 1;
            }
            cv::circle(image, uv, radius, color, -1);
        }
      }
      else
      {
        count ++;
        scan_in_camera.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
        scan_non_bbox.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
        scan_in_bbox.ranges[unsigned(index)] = std::numeric_limits<float>::quiet_NaN();
      }
    }
    cout << count << endl;

    image_pub_.publish(input_bridge->toImageMsg());

    scan_in_camera_pub_.publish(scan_in_camera);
    scan_non_bbox_pub_.publish(scan_non_bbox);
    scan_in_bbox_pub_.publish(scan_in_bbox);
    cout << "published" << endl;
    checkpoint = true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_filter"); 
  cout << "main" << endl;
  YoloTurtle filter;
  cout << "object created" << endl;
  ros::spin();
}
