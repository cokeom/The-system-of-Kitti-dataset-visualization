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
    KittiFrame(); // only run once.
    void init(); // initialize the kittiframe, in main it can run many times.
    void setFrameNum(int frame); // set current frame's id.
    void loadTrackFile(); // load the tracking file
    void publishImage(); // publish Image and draw 2d box
    void setImagePath(std::string imagePath);   
    void setTrackPath(std::string trackPath);     
    void setPclPath(std::string pclPath);
    void loadPclFile(); // load the velodyne data (.bin)
    void publishPcl(); // publish velodyne 
    void setMyCarPath(std::string myCarPath);
    void adjustMyCar(); // set the marker about my car.
    void adjustMyStrip();// set the 90° view lines
    void loadCaliMatrix(float R[4][4], float T[4][4]); // read the Rectify Matrix and the Translation Matrix
    void adjustMy3dBox(); // draw the 3d objects' box
    void publishMarkerArray(); // publish my car, the view lines and 3d object boxes.
    void setIMUPath(std::string IMUPath);
    void loadIMUFile(); // load the IMU file (oxts). I construct a struct called trackObject to store them.
    void publishIMU(); // a purple arrow.
    TRANSMYCAR getMyCarTrans(); // TRANSMYCAR is the struct called trackObject.
    void publishMyCarPath(vector <POS2D> my_car_path); // the trail of my car.
    std::map <int , POS2D > getMyObjPos(); // the position of current objects.
    void publishMyObjPath(map <int , vector<POS2D> > my_obj_path); 
    void adjustObjDistance(float mycar_array[3][8]); // count the distance between my car and other objects.
    void countMinDistance(POS2D pointP, MATRIX box, float& dis_min, POS2D& pos_min_1, POS2D& pos_min_2); // 点到对应3D盒子的俯视图最短距离
    void publishObjDistance();

protected:
    int frame_number;
    MATRIX R0_rect;
    MATRIX Tvelo_cam;
// image
    ros::NodeHandle nh_image;    
    std::string image_path;
    std::string tracking_path;
    image_transport::Publisher pub_image;
// point_cloud
    ros::NodeHandle nh_pcl;
    std::string pcl_path;
    sensor_msgs::PointCloud2 cloud_msg; // velodyne binary file
    ros::Publisher pub_pcl;
    visualization_msgs::MarkerArray marker_array; // marker array for my car, velodyne and 3d boxes.
// Marker myCar
    ros::NodeHandle nh_markers;
    std::string my_car_path;
    visualization_msgs::Marker marker_car; // my car
    ros::Publisher pub_my_car;
// Marker strip
    visualization_msgs::Marker marker_strip_1; // why do i use two marker_strips ?
    visualization_msgs::Marker marker_strip_2;
// Marker 3DBOX
    // vector <visualization_msgs::Marker> 3DBoxMarkers; //it's unnessary.
// IMU&&GPS
    IMUMYCAR my_car_imu;
    TRANSMYCAR my_car_trans;
    std::string IMU_path;
    ros::NodeHandle nh_IMU;
    ros::Publisher pub_IMU;
// tracking
    long tracking_offset; // use for locating the file
    std::vector <trackObject> tracking_objects; // current objects
    std::map<int , MATRIX> location_velo_all; // current objects in velodyne coordinates
// object motion path
    visualization_msgs::MarkerArray marker_array_path;
    ros::NodeHandle nh_path_array;
    ros::Publisher pub_path_array;
    std::map <int , POS2D > obj_center; 
// object distance
    visualization_msgs::MarkerArray marker_array_distance;
    ros::NodeHandle nh_distance_array;
    ros::Publisher pub_distance_array;
};