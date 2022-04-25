// 矩阵乘法以及协调不同帧的数据处理
#ifndef KITTIUTILS_h
#define KITTIUTILS_h
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <map>
using namespace std;
typedef vector< vector<float> > MATRIX;

struct trackObject{
    int tk_frame;
    int tk_id;
    std::string tk_type;
    int tk_truncated;
    int tk_occluded;
    float tk_alpha;
    float tk_bbox[4];
    float tk_dimensions[3]; // height, width, length
    float tk_location[3]; // location x,y,z in camera coordinates (in meters)
    float rotation_y;
    float score;
    };
typedef trackObject TRACKOBJ;

struct imuObject{
    float oxts_unit[3]; // latitude longitude altitude
    float angle[3]; // roll pitch yawn 
    float velocity[5];  // vn ve vl vf vu
    float acceleration[6]; // ax ay az af al au
    float angular_rate[6]; // wx wy wz wf wl wu
    float accuracy[2]; // pos_accuracy vel_accuracy
    int primary_GPS_receiver[5]; // perhaps unuseful
};
typedef imuObject IMUMYCAR;

struct myCarTrans{
    float vf;
    float vl;
    float yawn;
};
typedef myCarTrans TRANSMYCAR;

struct color{
    int b;
    int g;
    int r;
};

struct position {
    float x;
    float y;
//  float z; // we needn't use it 
    float yawn;
};

extern MATRIX CreateMatrix (int m, int n); // create mxn matrix
extern MATRIX Create3DCorner (float h, float w, float l);
extern MATRIX R_roty (float yawn);
extern MATRIX MultiplyMatrix (MATRIX A, MATRIX B);
extern MATRIX InverseMatrix (MATRIX A);
extern MATRIX CamToVelo (MATRIX A, MATRIX R0_rect, MATRIX Tvelo_cam);
extern float vectInnerProduct2d(position a, position b);
extern float vectExterProduct2d(position a, position b); // just return the vector mold
extern vector <position> my_car;
extern map <int , vector<position> > my_object; // 每一个track_id对应一个路径集合
extern void adjustMyCarPos(int frame, int fps, float vf, float vl, float yawn);
void updateObjPath(vector <position>& obj, int fps, float vf, float vl, float yawn);
extern void adjustMyObjPos(int frame, int fps, float vf, float vl, float yawn, map<int, position> obj_curr);


#endif