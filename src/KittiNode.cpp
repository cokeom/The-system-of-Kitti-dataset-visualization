#include <kittiAnalysis/KittiFrame.h>
#include <ros/ros.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
using namespace std;
float r0_rect[4][4] = { 
                       { 9.999239e-01, 9.837760e-03, -7.445048e-03, 0. },
                       {-9.869795e-03, 9.999421e-01, -4.278459e-03, 0. },
                       { 7.402527e-03, 4.351614e-03,  9.999631e-01, 0. },
                       {           0.,           0.,            0., 1.0}
                      };

float tvelo_cam[4][4] = {
                          { 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03},
                          { 1.480249e-02,  7.280733e-04, -9.998902e-01, -7.631618e-02},
                          { 9.998621e-01,  7.523790e-03,  1.480755e-02, -2.717806e-01},
                          {           0.,           0.,            0.,            1.0}    
                        };
int main(int argc, char **argv) {
    std::string DATA_PATH = "/mnt/hgfs/shares/3dObjectDect/2011_09_26/2011_09_26_drive_0005_sync";
    std::string ROOT_DATA_PATH = "/mnt/hgfs/shares/3dObjectDect";
    ros::init(argc, argv, "KittiNode");
    int frame = 0;
    KittiFrame kittiFrame;
    kittiFrame.setTrackPath(ROOT_DATA_PATH + "/training/label_02/0000.txt");
    //ROOT_DATA_PATH + "/training/label_02/0000.txt"
    ros::Rate loop_rate(13);
    while(ros::ok()) {
        std::stringstream buffer;
        buffer << setfill('0') << setw(10) << frame; 
        kittiFrame.init();
        kittiFrame.setImagePath(DATA_PATH + "/image_00/data/" + buffer.str() + ".png");
        kittiFrame.setFrameNum(frame);
        kittiFrame.loadTrackFile();
        kittiFrame.publishImage();

        kittiFrame.setPclPath(DATA_PATH + "/velodyne_points/data/" + buffer.str() + ".bin");
        kittiFrame.loadPclFile();
        kittiFrame.publishPcl();
    
        kittiFrame.setMyCarPath("file://" + ROOT_DATA_PATH + "/Audi_R8/Models/Audi_R8.dae");
        kittiFrame.adjustMyCar();
        kittiFrame.adjustMyStrip();
        kittiFrame.loadCaliMatrix(r0_rect, tvelo_cam);
        kittiFrame.adjustMy3dBox();
        kittiFrame.publishMarkerArray();

        kittiFrame.setIMUPath(DATA_PATH + "/oxts/data/" + buffer.str() + ".txt");
        kittiFrame.loadIMUFile();
        kittiFrame.publishIMU();

        if(frame==153)frame=0;
        else frame++;
        ROS_INFO("Now IM publishing frame! %d",frame);
        loop_rate.sleep();  
    }

    return 0;

}