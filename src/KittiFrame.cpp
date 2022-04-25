#include <kittiAnalysis/KittiFrame.h>
using namespace std;
//#define FLT_MAX         3.402823466e+38F        /* max value */ 
KittiFrame::KittiFrame() { //construct

    tracking_objects.clear();
    image_transport::ImageTransport it( nh_image );
    color color_yellow = {0,255,255};
    color color_red = {0,0,255};
    color color_blue = {255,0,0};
    color color_green = {0,255,0};
    COLOR_TYPE.insert(pair <string, color> ("Car", color_yellow)); // in cv not RGB but BGR  yellow
    COLOR_TYPE.insert(pair <string, color> ("Van", color_red)); // red
    COLOR_TYPE.insert(pair <string, color> ("Pedestrian", color_blue)); // blue
    COLOR_TYPE.insert(pair <string, color> ("Cyclist", color_green)); //green
    pub_image = it.advertise( "camera/image", 1 );

    pub_pcl = nh_pcl.advertise <sensor_msgs::PointCloud2> ( "pcl_output", 20 );

    pub_my_car = nh_markers.advertise<visualization_msgs::MarkerArray> ( "visualization_marker", 30);
    marker_array.markers.clear();

    pub_IMU = nh_IMU.advertise <sensor_msgs::Imu> ("Imu" , 20);
    pub_path_array = nh_path_array.advertise<visualization_msgs::MarkerArray> ( "visualization_marker_path", 30);
    pub_distance_array = nh_distance_array.advertise<visualization_msgs::MarkerArray> ( "visualization_marker_distance", 30);
    tracking_offset = 0;

    R0_rect = CreateMatrix(4,4);
    Tvelo_cam = CreateMatrix(4,4);
}
void KittiFrame::init() {
    marker_array.markers.clear();
    marker_array_path.markers.clear();
    marker_array_distance.markers.clear();
    obj_center.clear();
    location_velo_all.clear();
}
void KittiFrame::setFrameNum(int frame) {
    frame_number = frame;
}
void KittiFrame::setTrackPath(std::string trackPath) {
    tracking_path = trackPath;
}
void KittiFrame::loadTrackFile() {
    tracking_objects.clear();
    fstream infile(tracking_path.c_str(), ios::in);
    if(frame_number == 0)tracking_offset = 0;
    infile.seekg(tracking_offset , ios::beg); // set mark in order to continue reading file. 
    if(!infile) {
        cerr << "open error!!!" << endl;
        exit(1);
    }
    int tmp;
    while(1) {
        trackObject tk_obj;
        if(infile.eof())break;
        infile >> tmp ;
        if(tmp != frame_number)break;
        tk_obj.tk_frame = tmp;

        infile >> tk_obj.tk_id;

       // tracking_offset = infile.tellg();
       // ROS_INFO("tk_frame = %d",tmp);
        infile >> tk_obj.tk_type;

        

        infile >> tk_obj.tk_truncated;
        infile >> tk_obj.tk_occluded;
        infile >> tk_obj.tk_alpha;

        infile >> tk_obj.tk_bbox[0];
        infile >> tk_obj.tk_bbox[1];
        infile >> tk_obj.tk_bbox[2];
        infile >> tk_obj.tk_bbox[3];
    // ROS_INFO("%f 123",tk_obj.tk_bbox[3]);
        infile >> tk_obj.tk_dimensions[0]; // height
        infile >> tk_obj.tk_dimensions[1]; // width
        infile >> tk_obj.tk_dimensions[2]; // length
    // all locations are in camera coordinates
        infile >> tk_obj.tk_location[0]; // x
        infile >> tk_obj.tk_location[1]; // y
        infile >> tk_obj.tk_location[2]; // z
        infile >> tk_obj.rotation_y;
      //  infile >> tk_obj.score; //actually dont have
        tracking_objects.push_back(tk_obj);
    }
    
    tracking_offset = infile.tellg();
    stringstream sstream;
    string tmp_str;
    sstream << tmp;
    sstream >> tmp_str;
 //   ROS_INFO_STREAM(tmp_str);
    tracking_offset -= tmp_str.length();
 //   ROS_INFO("frame_str_length = %d",tmp_str.length());
 //   ROS_INFO("tracking_offset = %d",tracking_offset);
}
void KittiFrame::publishImage() { //连续发布图片(视频)

    
    cv::Mat image = cv::imread( image_path, CV_LOAD_IMAGE_COLOR );
    int object_num = tracking_objects.size();
    for (int i = 0; i < object_num; i++)
    {
        if(tracking_objects[i].tk_type == "DontCare")continue;
        //cv::Scalar color; 
        //cout<<"ooooooooooooo:"<<COLOR_TYPE[tracking_objects[i].tk_type].b<<endl;
        cv::rectangle(image, 
        cv::Point(tracking_objects[i].tk_bbox[0], tracking_objects[i].tk_bbox[1]), 
        cv::Point(tracking_objects[i].tk_bbox[2], tracking_objects[i].tk_bbox[3]),
        cv::Scalar(COLOR_TYPE[tracking_objects[i].tk_type].b, COLOR_TYPE[tracking_objects[i].tk_type].g, COLOR_TYPE[tracking_objects[i].tk_type].r),
        //cv::Scalar(0, 255, 0),
            3,
        cv::LINE_8);
        float mid = (tracking_objects[i].tk_bbox[0] + tracking_objects[i].tk_bbox[2])/2;
        stringstream ss;
        ss << tracking_objects[i].tk_id;
        string tk_id_str = ss.str();
        string str = tracking_objects[i].tk_type + ":" + tk_id_str;
        cv::putText(image, str, cv::Point(mid, tracking_objects[i].tk_bbox[1]), cv::FONT_HERSHEY_SIMPLEX, 1,
        cv::Scalar(COLOR_TYPE[tracking_objects[i].tk_type].b, COLOR_TYPE[tracking_objects[i].tk_type].g, COLOR_TYPE[tracking_objects[i].tk_type].r),
        4,8);
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    pub_image.publish(msg);

}
void KittiFrame::setImagePath(std::string imagePath) {
    image_path = imagePath;
}
void KittiFrame::setPclPath(std::string pclPath) {
    pcl_path = pclPath;
}
void KittiFrame::loadPclFile() {

    fstream infile(pcl_path.c_str(), ios::in|ios::binary);
    
    if(!infile) {
        cerr << "open error!!!" << endl;
        exit(1);
    }

    infile.seekg(0, ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    while (!infile.eof()) {
        // ROS_INFO("I have read the file!!!");
        pcl::PointXYZI point;
        infile.read((char *)&point.x, 3 * sizeof(float));
        infile.read((char *)&point.intensity, sizeof(float));
        cloud->push_back(point);
    }
    infile.close();

    // pcl::VoxelGrid <pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(cloud);
    // pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    // sor.setLeafSize (0.01f, 0.01f, 0.01f); 
    // sor.filter (*cloud_filtered);  

    pcl::toROSMsg(*cloud, cloud_msg);
    //ROS_INFO_STREAM(pcl_path);
}
void KittiFrame::publishPcl() {
    cloud_msg.header.frame_id = "point_cloud";
    pub_pcl.publish(cloud_msg);
}
void KittiFrame::setMyCarPath(std::string myCarPath) {
    my_car_path = myCarPath;
}
void KittiFrame::adjustMyCar() {
    marker_car.header.frame_id = "point_cloud" ;
    marker_car.header.stamp = ros::Time(); //attention
    marker_car.ns = "my_namespace"; 
    marker_car.id = 0;
 //   marker.type = visualization_msgs::Marker::SPHERE;
    marker_car.action = visualization_msgs::Marker::ADD;
    marker_car.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_car.lifetime = ros::Duration(0.1);
    
    marker_car.pose.position.x = 0;
    marker_car.pose.position.y = 0;
    marker_car.pose.position.z = -1.73;

    marker_car.pose.orientation.x = 0.0;
    marker_car.pose.orientation.y = 0.0;
    marker_car.pose.orientation.z = 1.0;
    marker_car.pose.orientation.w = 1.0;

    marker_car.scale.x = 1.2;
    marker_car.scale.y = 1.2;
    marker_car.scale.z = 1.2;

    marker_car.color.a = 1.0; //if not 1.0 , it can't visualize
    marker_car.color.r = 1.0;
    marker_car.color.g = 1.0;
    marker_car.color.b = 1.0;

    marker_car.mesh_resource = my_car_path;
    //ROS_INFO_STREAM(my_car_path);
}
void KittiFrame::adjustMyStrip() {
    marker_strip_1.header.frame_id = "point_cloud" ;
    marker_strip_1.header.stamp = ros::Time(); //attention
    marker_strip_1.ns = "my_namespace"; 
    marker_strip_1.id = 1;
    marker_strip_1.type = visualization_msgs::Marker::LINE_STRIP;
    marker_strip_1.action = visualization_msgs::Marker::ADD;
    marker_strip_1.lifetime = ros::Duration(0);
    
    marker_strip_1.pose.position.x = 0;
    marker_strip_1.pose.position.y = 0;
    marker_strip_1.pose.position.z = 0;

    marker_strip_1.pose.orientation.x = 0.0;
    marker_strip_1.pose.orientation.y = 0.0;
    marker_strip_1.pose.orientation.z = 0.0;
    marker_strip_1.pose.orientation.w = 1.0;

    marker_strip_1.scale.x = 0.2;
    marker_strip_1.scale.y = 0.2;
    marker_strip_1.scale.z = 0.2;

    marker_strip_1.color.a = 1.0; //if not 1.0 , it can't visualize
    marker_strip_1.color.r = 1.0;
    marker_strip_1.color.g = 1.0;
    marker_strip_1.color.b = 1.0;

    marker_strip_2.header.frame_id = "point_cloud" ;
    marker_strip_2.header.stamp = ros::Time(); //attention
    marker_strip_2.ns = "my_namespace"; 
    marker_strip_2.id = 2;
    marker_strip_2.type = visualization_msgs::Marker::LINE_STRIP;
    marker_strip_2.action = visualization_msgs::Marker::ADD;
    marker_strip_2.lifetime = ros::Duration(0);
    
    marker_strip_2.pose.position.x = 0;
    marker_strip_2.pose.position.y = 0;
    marker_strip_2.pose.position.z = 0;

    marker_strip_2.pose.orientation.x = 0.0;
    marker_strip_2.pose.orientation.y = 0.0;
    marker_strip_2.pose.orientation.z = 0.0;
    marker_strip_2.pose.orientation.w = 1.0;

    marker_strip_2.scale.x = 0.2;
    marker_strip_2.scale.y = 0.2;
    marker_strip_2.scale.z = 0.2;

    marker_strip_2.color.a = 1.0; //if not 1.0 , it can't visualize
    marker_strip_2.color.r = 1.0;
    marker_strip_2.color.g = 1.0;
    marker_strip_2.color.b = 1.0;

    geometry_msgs::Point point1,point2,point3;
    point1.x = 0.0;
    point1.y = 0.0;
    point1.z = 0.0;

    point2.x = 15.0;
    point2.y = -15.0;
    point2.z = 0.0;

    point3.x = 15.0;
    point3.y = 15.0;
    point3.z = 0.0;

    marker_strip_1.points.push_back( point2 );
    marker_strip_1.points.push_back( point1 );

    marker_strip_2.points.push_back( point3 );
    marker_strip_2.points.push_back( point1 );
}
void KittiFrame::loadCaliMatrix(float R[4][4], float T[4][4]){
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++){
            R0_rect[i][j] = R[i][j];
            Tvelo_cam[i][j] = T[i][j];
        }
}
void KittiFrame::adjustMy3dBox() {

    for(int i = 0; i < tracking_objects.size(); i++) {

        if(tracking_objects[i].tk_type == "DontCare") continue; 
        //cout<< tracking_objects.size() << "!!!!!!!!!!!!!!!!" <<endl;
        MATRIX corner = Create3DCorner(
        tracking_objects[i].tk_dimensions[0],
        tracking_objects[i].tk_dimensions[1],
        tracking_objects[i].tk_dimensions[2]
        ); // Corner Matrix

        MATRIX rot_y = R_roty (tracking_objects[i].rotation_y); // 3*3 rotation Matrixs

        MATRIX corner_rect = MultiplyMatrix(rot_y, corner); // corner Matrix rectify

        MATRIX location_cam = CreateMatrix(3,8); //create location in camera coordination
        for(int j=0; j<8; j++) {
            location_cam[0][j] = corner_rect[0][j] + tracking_objects[i].tk_location[0];
            location_cam[1][j] = corner_rect[1][j] + tracking_objects[i].tk_location[1];
            location_cam[2][j] = corner_rect[2][j] + tracking_objects[i].tk_location[2];
        }

        MATRIX location_velo = CamToVelo(location_cam, R0_rect, Tvelo_cam); // transform into location in velo coordiation

        location_velo_all[tracking_objects[i].tk_id] = location_velo; // use for counting distance

        visualization_msgs::Marker line_list;
        line_list.header.stamp = ros::Time(); //attention
        line_list.header.frame_id = "point_cloud" ;
        line_list.ns = "my_namespace"; 
        line_list.id = i+3000;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.lifetime = ros::Duration(0.1);

        line_list.scale.x = 0.1;
        // Line list is red
        line_list.color.r = (float)COLOR_TYPE[tracking_objects[i].tk_type].r / 255.0;
        line_list.color.g = (float)COLOR_TYPE[tracking_objects[i].tk_type].g / 255.0;
        line_list.color.b = (float)COLOR_TYPE[tracking_objects[i].tk_type].b / 255.0;

        line_list.color.a = 1.0;

        geometry_msgs::Point p0;
        p0.x = location_velo[0][0];
        p0.y = location_velo[1][0];
        p0.z = location_velo[2][0];
       // cout << p0.x << " " << p0.y << " " << p0.z << endl;
        geometry_msgs::Point p1;
        p1.x = location_velo[0][1];
        p1.y = location_velo[1][1];
        p1.z = location_velo[2][1];
        
        geometry_msgs::Point p2;
        p2.x = location_velo[0][2];
        p2.y = location_velo[1][2];
        p2.z = location_velo[2][2];

        obj_center[tracking_objects[i].tk_id].x = (p0.x + p2.x)/2;
        obj_center[tracking_objects[i].tk_id].y = (p0.y + p2.y)/2;
        // yawn is not used
        geometry_msgs::Point p3;
        p3.x = location_velo[0][3];
        p3.y = location_velo[1][3];
        p3.z = location_velo[2][3];
        geometry_msgs::Point p4;
        p4.x = location_velo[0][4];
        p4.y = location_velo[1][4];
        p4.z = location_velo[2][4];
        geometry_msgs::Point p5;
        p5.x = location_velo[0][5];
        p5.y = location_velo[1][5];
        p5.z = location_velo[2][5];
        geometry_msgs::Point p6;
        p6.x = location_velo[0][6];
        p6.y = location_velo[1][6];
        p6.z = location_velo[2][6];
        geometry_msgs::Point p7;
        p7.x = location_velo[0][7];
        p7.y = location_velo[1][7];
        p7.z = location_velo[2][7];
        
        line_list.points.push_back(p0);
        line_list.points.push_back(p1);
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        line_list.points.push_back(p2);
        line_list.points.push_back(p3);
        line_list.points.push_back(p3);
        line_list.points.push_back(p0);
        
        line_list.points.push_back(p4);
        line_list.points.push_back(p5);
        line_list.points.push_back(p5);
        line_list.points.push_back(p6);
        line_list.points.push_back(p6);
        line_list.points.push_back(p7);
        line_list.points.push_back(p7);
        line_list.points.push_back(p4);

        line_list.points.push_back(p4);
        line_list.points.push_back(p0);
        line_list.points.push_back(p5);
        line_list.points.push_back(p1);
        line_list.points.push_back(p6);
        line_list.points.push_back(p2);
        line_list.points.push_back(p7);
        line_list.points.push_back(p3);   
        marker_array.markers.push_back( line_list );
       //cout<<"markerArray.markers.size()"<<marker_array.markers.size()<<endl; 
        visualization_msgs::Marker box_text;
        box_text.header.stamp = ros::Time(); //attention
        box_text.header.frame_id = "point_cloud" ;
        box_text.ns = "my_namespace"; 
        box_text.id = i+160;
        box_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        box_text.action = visualization_msgs::Marker::ADD;
        box_text.lifetime = ros::Duration(0.1);

        box_text.scale.z = 1;
        //ROS_INFO_STREAM("123123");
        box_text.color.r = (float)COLOR_TYPE[tracking_objects[i].tk_type].r / 255.0;
        box_text.color.g = (float)COLOR_TYPE[tracking_objects[i].tk_type].g / 255.0;
        box_text.color.b = (float)COLOR_TYPE[tracking_objects[i].tk_type].b / 255.0;

        box_text.pose.orientation.w = 1.0;
        box_text.color.a = 1.0;
        //cout << p4.x << " " << p4.y << " " << p4.z << endl;
        box_text.pose.position.x = (p4.x + p7.x)/2;
        box_text.pose.position.y = p4.y; 
        box_text.pose.position.z = (p4.z + p5.z)/2;

        ostringstream str;
        str << tracking_objects[i].tk_id;
        // cout << tracking_objects[i].tk_type + str.str() + "123123" <<endl;
        box_text.text = tracking_objects[i].tk_type +": " + str.str();

        marker_array.markers.push_back( box_text );
        
        // cout<<"markerArray.markers.size() af"<<marker_array.markers.size()<<endl;
    }
}
void KittiFrame::publishMarkerArray() {
    // marker_array.markers.clear();
    marker_array.markers.push_back( marker_car );
    marker_array.markers.push_back( marker_strip_1 );
    marker_array.markers.push_back( marker_strip_2 );
    pub_my_car.publish( marker_array );
}
void KittiFrame::setIMUPath(std::string IMUPath) {
    IMU_path = IMUPath;
}
void KittiFrame::loadIMUFile() {
    fstream infile(IMU_path.c_str(), ios::in);
    
    if(!infile) {
        cerr << "open error!!!" << endl;
        exit(1);
    }

    infile.seekg(0, ios::beg);

    infile >> my_car_imu.oxts_unit[0] >> my_car_imu.oxts_unit[1] >> my_car_imu.oxts_unit[2]; 
    infile >> my_car_imu.angle[0] >> my_car_imu.angle[1] >> my_car_imu.angle[2];
    infile >> my_car_imu.velocity[0] >> my_car_imu.velocity[1] >> my_car_imu.velocity[2] >> my_car_imu.velocity[3] >> my_car_imu.velocity[4];
    infile >> my_car_imu.acceleration[0] >> my_car_imu.acceleration[1] >> my_car_imu.acceleration[2] >> my_car_imu.acceleration[3] >> my_car_imu.acceleration[4] >> my_car_imu.acceleration[5];
    infile >> my_car_imu.angular_rate[0] >> my_car_imu.angular_rate[1] >> my_car_imu.angular_rate[2] >> my_car_imu.angular_rate[3] >> my_car_imu.angular_rate[4] >> my_car_imu.angular_rate[5];
    infile >> my_car_imu.accuracy[0] >> my_car_imu.accuracy[1];
    infile >> my_car_imu.primary_GPS_receiver[0] >> my_car_imu.primary_GPS_receiver[1] >> my_car_imu.primary_GPS_receiver[2] >> my_car_imu.primary_GPS_receiver[3] >> my_car_imu.primary_GPS_receiver[4];
   // ROS_INFO("%f",angle[0]);
    my_car_trans.vl = my_car_imu.velocity[2];
    my_car_trans.vf = my_car_imu.velocity[3];
    my_car_trans.yawn = my_car_imu.angle[2];
    infile.close(); 

}
void KittiFrame::publishIMU() {
    sensor_msgs::Imu IMU_data;
    IMU_data.header.stamp = ros::Time::now();
    IMU_data.header.frame_id = "point_cloud";

    IMU_data.linear_acceleration.x = my_car_imu.acceleration[3]; 
    IMU_data.linear_acceleration.y = my_car_imu.acceleration[4];
    IMU_data.linear_acceleration.z = my_car_imu.acceleration[5];

	//角速度
    IMU_data.angular_velocity.x = my_car_imu.angular_rate[3]; 
    IMU_data.angular_velocity.y = my_car_imu.angular_rate[4]; 
    IMU_data.angular_velocity.z = my_car_imu.angular_rate[5];

    tf::Quaternion q = tf::createQuaternionFromRPY(my_car_imu.angular_rate[0], my_car_imu.angular_rate[1], my_car_imu.angular_rate[2]);
    IMU_data.orientation.x = q.getX();
    IMU_data.orientation.y = q.getY();
    IMU_data.orientation.z = q.getZ();
    IMU_data.orientation.w = q.getW();
  //  ROS_INFO("%f",IMU_data.orientation.w);
  
    pub_IMU.publish(IMU_data);
}
TRANSMYCAR KittiFrame::getMyCarTrans() {
    return my_car_trans;
}
void KittiFrame::publishMyCarPath(vector <position> my_car_path) {
    
    visualization_msgs::Marker marker_my_car;

    marker_my_car.header.frame_id = "point_cloud" ;
    marker_my_car.header.stamp = ros::Time(); //attention
    marker_my_car.ns = "my_namespace"; 
    marker_my_car.id = 2000;
    marker_my_car.type = visualization_msgs::Marker::LINE_STRIP;
    marker_my_car.action = visualization_msgs::Marker::ADD;
    marker_my_car.lifetime = ros::Duration(0);

    marker_my_car.scale.x = 0.2;
    marker_my_car.scale.y = 0.2;
    marker_my_car.scale.z = 0.2;

    marker_my_car.color.a = 1.0; //if not 1.0 , it can't visualize
    marker_my_car.color.r = 1.0;
    marker_my_car.color.g = 0.0;
    marker_my_car.color.b = 0.0;

    int size = my_car_path.size();
    //cout<< size <<endl;
    for(int i=0; i<size; i++) {
        geometry_msgs::Point p;
        p.x = my_car_path[i].x;
        p.y = my_car_path[i].y;
        p.z = 0.;
        marker_my_car.points.push_back(p);
    }
    //cout<< my_car_path[0].x << " " <<my_car_path[0].y << " " <<endl;
    marker_array_path.markers.push_back( marker_my_car );
    pub_path_array.publish(marker_array_path);
    //cout << "markers array size = " <<marker_array_path.markers.size()<<endl;
   
}
std::map <int , position > KittiFrame::getMyObjPos() {
    return obj_center;
}
void KittiFrame::publishMyObjPath(map <int , vector<position> > my_obj_path) {
    int size1 = tracking_objects.size();
    //cout<< "tracking_objects size" << size1 <<endl;
    for(int i=0; i < size1; i++)
    {
        if(tracking_objects[i].tk_id == -1)continue;

        visualization_msgs::Marker marker_my_obj;
        marker_my_obj.header.frame_id = "point_cloud" ;
        marker_my_obj.header.stamp = ros::Time(); //attention
        marker_my_obj.ns = "my_namespace"; 
        marker_my_obj.id = i+5000;
        marker_my_obj.type = visualization_msgs::Marker::LINE_STRIP;
        marker_my_obj.action = visualization_msgs::Marker::ADD;
        marker_my_obj.lifetime = ros::Duration(0.1);

        marker_my_obj.scale.x = 0.2;
        marker_my_obj.scale.y = 0.2;
        marker_my_obj.scale.z = 0.2;

        marker_my_obj.color.a = 1.0; //if not 1.0 , it can't visualize
        marker_my_obj.color.r = 1.0;
        marker_my_obj.color.g = 0.0;
        marker_my_obj.color.b = 0.0;

        int size2 = my_object[tracking_objects[i].tk_id].size();
        //cout<< size2 <<endl;
        //cout << "my_object" << tracking_objects[i].tk_id << " size="<<size2<<endl;
        int j=0;
        if(size2 >= 20)j = size2 - 20; //目的是让记录的点只展现最多20个
        for(; j<size2; j++) {
            geometry_msgs::Point p;
            p.x = my_object[tracking_objects[i].tk_id][j].x;
            p.y = my_object[tracking_objects[i].tk_id][j].y;
            p.z = 0.;
            marker_my_obj.points.push_back(p);
            //cout << "my_object " << tracking_objects[i].tk_id << "point is " <<"("<<p.x <<","<<p.y<<")"<<endl;
        }
        //cout<< my_car_path[0].x << " " <<my_car_path[0].y << " " <<endl;
        marker_array_path.markers.push_back( marker_my_obj );
       
    }
}
void KittiFrame::countMinDistance(position pointP, MATRIX box, float& dis_min, position& pos_min_1, position& pos_min_2) {
    position pointA,pointB;
    position vectAP, vectBP, vectAB, vectBA;
    float dis_curr ;
    for(int k = 0; k < 4; k++) {

        pointA.x = box[0][k % 4];
        pointA.y = box[1][k % 4];
        pointB.x = box[0][(k+1) % 4];
        pointB.y = box[1][(k+1) % 4];

        vectAB.x = pointB.x - pointA.x; // vector (A , B) A->B
        vectAB.y = pointB.y - pointA.y;

        vectBA.x = -1 * vectAB.x;
        vectBA.y = -1 * vectAB.y;


        vectAP.x = pointP.x - pointA.x;
        vectAP.y = pointP.y - pointA.y;

        vectBP.x = pointP.x - pointB.x;
        vectBP.y = pointP.y - pointB.y;

        float inner_ans1 = vectInnerProduct2d(vectAP , vectAB);
        float inner_ans2 = vectInnerProduct2d(vectBP , vectBA);
        float vectAPmold = sqrt(vectAP.x * vectAP.x + vectAP.y * vectAP.y);
        float vectBPmold = sqrt(vectBP.x * vectBP.x + vectBP.y * vectBP.y);
        if(inner_ans1 > 0 && inner_ans2 > 0) { // 锐角三角形
            //float vectAPmold = sqrt(vectAP.x * vectAP.x + vectAP.y * vectAP.y);
            float vectABmold = sqrt(vectAB.x * vectAB.x + vectAB.y * vectAB.y);
            float exter_ans = vectExterProduct2d(vectAP, vectAB);
            dis_curr = vectAPmold * (exter_ans/vectABmold);
            if(dis_curr < dis_min) {
                pos_min_1 = pointP;
                float tmp = inner_ans1 / (vectABmold * vectABmold); 
                pos_min_2.x = tmp * (pointB.x - pointA.x) + pointA.x;
                pos_min_2.y = tmp * (pointB.y - pointA.y) + pointA.y;
                dis_min = dis_curr;
            }
        }
        else{   // 钝角三角形
            dis_curr = min(vectAPmold, vectBPmold);
            if(dis_curr < dis_min) {
                if(dis_curr == vectAPmold) {
                    pos_min_1 = pointP;
                    pos_min_2 = pointA;
                    dis_min = dis_curr;
                }
                else{
                    pos_min_1 = pointP;
                    pos_min_2 = pointB;
                    dis_min = dis_curr;
                }
            }
        } 
    }
}
void KittiFrame::adjustObjDistance(float mycar_array[3][8]) {
    int i;
    for(i = 0; i < tracking_objects.size(); i++) {
        if(tracking_objects[i].tk_id == -1) continue;
        MATRIX mycar = CreateMatrix(3,8); // construct MATRIX struct
        for(int i0=0; i0<3; i0++)
            for(int j0=0; j0<8; j0++){
                mycar[i0][j0] = mycar_array[i0][j0];
        }

        MATRIX obj_curr = location_velo_all[tracking_objects[i].tk_id];

        float dis_min = FLT_MAX;
        position pos_min_A, pos_min_B, pos_curr;

        for(int j = 0; j < 4; j++){ // 4 * 1
            pos_curr.x = mycar[0][j];
            pos_curr.y = mycar[1][j];
            countMinDistance(pos_curr, obj_curr, dis_min, pos_min_A, pos_min_B); // 1 * 4
        }
        for(int j = 0; j < 4; j++) {
            pos_curr.x = obj_curr[0][j];
            pos_curr.y = obj_curr[1][j];
            countMinDistance(pos_curr, mycar, dis_min, pos_min_A, pos_min_B);
        }
    // now we have both the pos_min_AB and dis_min. We can construct Marker now.
# if 1
        visualization_msgs::Marker marker_obj_dis;
        marker_obj_dis.header.frame_id = "point_cloud" ;
        marker_obj_dis.header.stamp = ros::Time(); //attention
        marker_obj_dis.ns = "my_namespace"; 
        marker_obj_dis.id = i+2000;
        marker_obj_dis.type = visualization_msgs::Marker::LINE_STRIP;
        marker_obj_dis.action = visualization_msgs::Marker::ADD;
        marker_obj_dis.lifetime = ros::Duration(0.1);

        marker_obj_dis.scale.x = 0.2;
        marker_obj_dis.scale.y = 0.2;
        marker_obj_dis.scale.z = 0.2;

        marker_obj_dis.color.a = 1.0; //if not 1.0 , it can't visualize
        marker_obj_dis.color.r = 0.0;
        marker_obj_dis.color.g = 0.5;
        marker_obj_dis.color.b = 0.5;
        
        // print the line in Rviz
        geometry_msgs::Point p;
        p.x = pos_min_A.x;
        p.y = pos_min_A.y;
        p.z = 0.;
        marker_obj_dis.points.push_back(p);

        p.x = pos_min_B.x;
        p.y = pos_min_B.y;
        p.z = 0.;
        marker_obj_dis.points.push_back(p);

        marker_array_distance.markers.push_back( marker_obj_dis );

        // print distance in Rviz
        visualization_msgs::Marker dis_text;
        dis_text.header.stamp = ros::Time(); //attention
        dis_text.header.frame_id = "point_cloud" ;
        dis_text.ns = "my_namespace"; 
        dis_text.id = i+4000;
        dis_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        dis_text.action = visualization_msgs::Marker::ADD;
        dis_text.lifetime = ros::Duration(0.1);

        dis_text.scale.z = 1;
        //ROS_INFO_STREAM("123123");
        dis_text.color.r = 0.0;
        dis_text.color.g = 1.0;
        dis_text.color.b = 0.0;

        dis_text.pose.orientation.w = 1.0;
        dis_text.color.a = 1.0;

        dis_text.pose.position.x = (pos_min_A.x + pos_min_B.x)/2;
        dis_text.pose.position.y = (pos_min_A.y + pos_min_B.y)/2; 
        dis_text.pose.position.z = 0.;

        ostringstream str;
        str << dis_min;
        // cout << tracking_objects[i].tk_type + str.str() + "123123" <<endl;
        dis_text.text = str.str();
        marker_array_distance.markers.push_back( dis_text );
# endif
    }
}
void KittiFrame::publishObjDistance() {
    pub_distance_array.publish( marker_array_distance );
}