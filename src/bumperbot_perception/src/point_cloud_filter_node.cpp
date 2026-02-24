#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


class PointCloudFilter
{
    public:
         ros::NodeHandle nh;
         ros::Subscriber sub;
         ros::Publisher pub;

         PointCloudFilter()
         {
            sub = nh.subscribe("/camera/points",1,&PointCloudFilter::callback,this);

            pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_points",1);
         }

            void callback(const sensor_msgs::PointCloud2ConstPtr& input)
            {
                //Creating an object for storing the cloud points by using smart pointer cloud..
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

                //converting the data from ROS to PCL
                pcl::fromROSMsg(*input,*cloud);

                //creating the passthrough filter

                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered1(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(0.0, 1.0);
                pass.filter(*filtered1);

                //Downsampling the filtered1 cloud point data
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered2(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::VoxelGrid<pcl::PointXYZ> voxel;
                voxel.setInputCloud(filtered1);
                voxel.setLeafSize(0.05f,0.05f,0.05f);
                voxel.filter(*filtered2);

                //Statistical Outlier Removal 

                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered3(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                sor.setInputCloud(filtered2);
                sor.setMeanK(50);
                sor.setStddevMulThresh(1);
                sor.filter(*filtered3);

                sensor_msgs::PointCloud2 output_msg;

                pcl::toROSMsg(*filtered3,output_msg);

                output_msg.header.stamp = ros::Time::now();
                output_msg.header.frame_id = "camera_sensor";
                pub.publish(output_msg);

                ROS_INFO_STREAM("Header frame_id: " << output_msg.header.frame_id);
                ROS_INFO_STREAM("point cloud size =" <<output_msg.data.size());


                





            }
};


int main(int argc,char** argv)
 {
    ros::init(argc,argv,"point_cloud_node");
    PointCloudFilter node;
    ros::spin();
    return 0;
     }
