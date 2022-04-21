#ifndef KITTIUTILS_h
#define KITTIUTILS_h
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
using namespace std;
typedef vector< vector<float> > MATRIX;

extern MATRIX CreateMatrix (int m, int n); // create mxn matrix
extern MATRIX Create3DCorner (float h, float w, float l);
extern MATRIX R_roty (float yawn);
extern MATRIX MultiplyMatrix (MATRIX A, MATRIX B);
extern MATRIX InverseMatrix (MATRIX A);
extern MATRIX CamToVelo (MATRIX A, MATRIX R0_rect, MATRIX Tvelo_cam);
#endif

