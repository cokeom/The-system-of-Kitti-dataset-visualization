#include <kittiAnalysis/KittiUtils.h>
using namespace std;

MATRIX CreateMatrix(int m, int n) {
    vector< vector<float> > v;
    for (int i=0; i<m; i++) {
        vector<float> v1(n,0);
        v.push_back(v1);
    }
    return v;
}


MATRIX R_roty(float yawn) { // rotated by y
    MATRIX R_temp = CreateMatrix(3,3);
    R_temp[0][0] = cos(yawn);
    R_temp[0][1] = 0;
    R_temp[0][2] = sin(yawn);
    R_temp[1][0] = 0;
    R_temp[1][1] = 1;
    R_temp[1][2] = 0;
    R_temp[2][0] = -sin(yawn);
    R_temp[2][1] = 0;
    R_temp[2][2] = cos(yawn);
    return R_temp;
}
MATRIX MultiplyMatrix (MATRIX A, MATRIX B) { // Matrix Mutiply : A@B
    int A_m = A.size();
    int A_n = A[0].size();
    int B_n = B.size();
    int B_s = B[0].size();
    if(A_n != B_n){
        cout << A_m << "x" << A_n;
        cout << B_n << "x" << B_s;
        cerr << "two matrixs cant mutiply !" << endl;
        exit(1);
    }    
    MATRIX C_temp = CreateMatrix(A_m, B_s);
    for (int i = 0; i < A_m; i++)
        for(int k = 0; k < B_s; k++) {
            C_temp[i][k] = 0;
            for(int j = 0; j < B_n; j++){
               C_temp[i][k] += A[i][j] * B[j][k];       
            }
        }
    return C_temp;
}
MATRIX InverseMatrix (MATRIX A) { //用LU分解算法
    int n = A.size();
    if(A.size() != A[0].size()) {
        cerr << "Matrix can't be inversed! " << endl;
        exit(1);
    }
    //MATRIX W,W_n,L,U,L_n,U_n = CreateMatrix(n, n);
    MATRIX W = CreateMatrix(n, n);
    MATRIX W_n = CreateMatrix(n, n);
    MATRIX L = CreateMatrix(n, n);
    MATRIX U = CreateMatrix(n, n);
    MATRIX L_n = CreateMatrix(n, n);
    MATRIX U_n = CreateMatrix(n, n);
    float s;
    for(int i=0; i<n; i++)
        for(int j=0; j<n; j++){
            W[i][j] = A[i][j];
            L[i][j] = 0;
			U[i][j] = 0;
			L_n[i][j] = 0;
			U_n[i][j] = 0;
			W_n[i][j] = 0;
        }
    
    for(int i=0; i<n; i++) 
        L[i][i] = 1.0;

    for(int i=0; i<n; i++) 
        U[0][i] = W[0][i];

    for(int i=1; i<n; i++) 
		L[i][0] = W[i][0] / U[0][0];

    for(int i=1; i<n; i++){
        for(int j=i; j<n; j++){
            s = 0.;
            for(int k=0; k<i; k++){
				s += L[i][k] * U[k][j];
            }
			U[i][j] = W[i][j] - s;
        }

        for(int d=i; d<n; d++){
            s = 0.;
			for(int k=0;k<i;k++)
			{
				s += L[d][k] * U[k][i];
			}
			L[d][i] = (W[d][i] - s) / U[i][i];
        }
    }

    for(int j=0;j<n;j++)  //求L的逆
	{
		for(int i=j;i<n;i++)
		{
			if(i==j) 
				L_n[i][j] = 1 / L[i][j];
			else if(i<j) 
				L_n[i][j] = 0;
			else
			{
				s = 0.;
				for(int k=j;k<i;k++)
				{
					s += L[i][k] * L_n[k][j];
				}
				L_n[i][j] = -L_n[j][j] * s;
			}
		}
	}
    for(int i=0;i<n;i++)  //求U的逆
	{
		for(int j=i;j>=0;j--)
		{
			if(i==j)
				U_n[j][i] = 1 / U[j][i];
			else if(j>i) 
				U_n[j][i] = 0;
			else
			{
				s = 0.;
				for(int k=j+1;k<=i;k++)
				{
					s += U[j][k] * U_n[k][i];
				}
				U_n[j][i] = -1 / U[j][j] * s;
			}
		}
	}
 
 
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<n;j++)
		{
			for(int k=0;k<n;k++)
			{
				W_n[i][j] += U_n[i][k] * L_n[k][j];
			}
		}
	}
    return W_n;
    //return A;
}
MATRIX Create3DCorner (float h, float w, float l) {
    MATRIX A = CreateMatrix(3,8);
    A[0][0] = l/2;
    A[1][0] = 0;
    A[2][0] = w/2;

    A[0][1] = l/2;
    A[1][1] = 0;
    A[2][1] = -w/2;
    
    A[0][2] = -l/2;
    A[1][2] = 0;
    A[2][2] = -w/2;
    
    A[0][3] = -l/2;
    A[1][3] = 0;
    A[2][3] = w/2;
    
    A[0][4] = l/2;
    A[1][4] = -h;
    A[2][4] = w/2;
    
    A[0][5] = l/2;
    A[1][5] = -h;
    A[2][5] = -w/2;
    
    A[0][6] = -l/2;
    A[1][6] = -h;
    A[2][6] = -w/2;
    
    A[0][7] = -l/2;
    A[1][7] = -h;
    A[2][7] = w/2;
    return A;
}
MATRIX CamToVelo (MATRIX A, MATRIX R0_rect, MATRIX Tvelo_cam){
    vector<float> v1(8,1.0); 
    A.push_back(v1); //extend the 3x8 matrix into 4x8 matrix
    MATRIX Inv_RO_rect = InverseMatrix(R0_rect); //inverse Matrix R0_rect
    MATRIX Inv_Tvelo_cam = InverseMatrix(Tvelo_cam); //inverse Matrix Tvelo_cam
    MATRIX Trans_Matrix = MultiplyMatrix(Inv_RO_rect, Inv_Tvelo_cam);
    return MultiplyMatrix(Trans_Matrix, A);
    //return A;
}
// int j_temp = 2;
// void test(){
//     j_temp = 1;
//     cout<<j_temp<<endl;
// }
vector <position> my_car;
void adjustMyCarPos(int frame, int fps, float vf, float vl, float yawn) {
    position temp;
    if(frame == 0 ) {
        my_car.clear();
        temp.x = 0;
        temp.y = 0;
        temp.yawn = yawn;
        my_car.push_back(temp);
        return ;
    }
    float distance = sqrt(vf*vf + vl*vl) * 1.0/fps; 
   
    int size = my_car.size();
    float det_yawn = yawn - my_car[size-1].yawn;

    for(int i=0; i < size; ++i) {
        my_car[i].x = my_car[i].x * cos(det_yawn) + my_car[i].y * sin(det_yawn) - distance;
        my_car[i].y = -1 * my_car[i].x * sin(det_yawn) + my_car[i].y * cos(det_yawn); 
        my_car[i].yawn = 0.0;
        
        //cout<< my_car[i].x << " " << my_car[i].y << " " << distance <<" "<< det_yawn <<endl;
    }
    temp.x = 0;
    temp.y = 0;
    temp.yawn = yawn;
    my_car.push_back(temp);
    return ;
}
