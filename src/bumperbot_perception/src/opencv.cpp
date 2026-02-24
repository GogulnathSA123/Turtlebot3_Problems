#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>


class ImageConverter
{
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   ros::Subscriber sub_;
   ros::Publisher pub_;

   public:
        
      ImageConverter() : it_(nh_)
      {
        sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imagclb , this);
        pub_ = it_.advertise

      }



     void imagclb()
     {

            
     }

};