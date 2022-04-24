#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>

#include <kittiAnalysis/KittiUtils.h>

class KittiFrame {

public:
    std::map <std::string, color> COLOR_TYPE;

    KittiFrame();
    void init();
    void setFrameNum(int frame);
    void loadTrackFile();
    void publishImage();
    void setImagePath(std::string imagePath);   
    void setTrackPath(std::string trackPath);     

   
    void setPclPath(std::string pclPath);
    void loadPclFile();
    void publishPcl();

    void setMyCarPath(std::string myCarPath);
    void adjustMyCar();
    void adjustMyStrip();
    void loadCaliMatrix(float R[4][4], float T[4][4]);
    void adjustMy3dBox();
    void publishMarkerArray();

    void setIMUPath(std::string IMUPath);
    void loadIMUFile();
    void publishIMU();

    TRANSMYCAR getMyCarTrans();
    void publishMyCarPath(vector <position> my_car_path);
    // void adjustMyCarPath(float x, float y);    
    std::map <int , position > getMyObjPos();
    void publishMyObjPath(map <int , vector<position> > my_obj_path);

protected:

    int frame_number;
    MATRIX R0_rect;
    MATRIX Tvelo_cam;
//image
    ros::NodeHandle nh_image;    
    std::string image_path;
    std::string tracking_path;
    image_transport::Publisher pub_image;

//point_cloud
    ros::NodeHandle nh_pcl;
    std::string pcl_path;
    sensor_msgs::PointCloud2 cloud_msg; //点云信息（ROS）
    ros::Publisher pub_pcl;


    visualization_msgs::MarkerArray marker_array; //marker阵列
//Marker myCar
    ros::NodeHandle nh_markers;
    std::string my_car_path;
    visualization_msgs::Marker marker_car; //车子模型信息(ROS)
    ros::Publisher pub_my_car;

//Marker strip
    //ros::NodeHandle nh_strip;
    visualization_msgs::Marker marker_strip_1; // why do i use two marker_strips ?
    visualization_msgs::Marker marker_strip_2;
//Marker 3DBOX
    // vector <visualization_msgs::Marker> 3DBoxMarkers; //it's unnessary.

// IMU&&GPS
    IMUMYCAR my_car_imu;
    TRANSMYCAR my_car_trans;
    std::string IMU_path;
    ros::NodeHandle nh_IMU;
    ros::Publisher pub_IMU;


// tracking
    long tracking_offset;
    std::vector <trackObject> tracking_objects;

// object motion path
    visualization_msgs::MarkerArray marker_array_path;
    ros::NodeHandle nh_path_array;
    ros::Publisher pub_path_array;

    std::map <int , position > obj_center;
};