#pragma once

#define _USE_MATH_DEFINES
#include <cstdlib>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <mavros_msgs/Waypoint.h>

#include "llh_conv.h"

#define N_REG 5
#define TH_R 30.0 // Threat checker 최소거리가 얘보다작아야 위협으로인정
#define MAT_SIZE 6
#define ERROR 1.0e-10

// Structures
typedef struct information
{
    // The last received msg's time stamp
    float stamp;
    // Type
    int type;
    // global llh
    double lat;
    double lon;
    double height;
    // common local ned
    double lx;
    double ly;
    double lz;
    // velocity
    float vx;
    float vy;
    float vz;
    // head to
    float hx;
    float hy;
    // head from
    float hfx;
    float hfy;
    // head to (to avoid)
    float hxt;
    float hyt;


} information_t;

typedef struct neighbor_gl
{
    // The last received msg's time stamp
    float stamp;
    // ID
    int id;
    // Type
    int type;
    // global llh
    double lat;
    double lon;
    double height;
    // common local ned
    double lx;
    double ly;
    double lz;
    // velocity
    float vx;
    float vy;
    float vz;
    // Crashing point
    float cx;
    float cy;
    // Priority
    bool pri;
    // Avoidance trig. turn it on when threat detected, and turn it down after avoidance is finished.
    bool avoiding;
    // 1: goto act, 2: pan1, 3: pan2, 4: sink, 5: move to next
    int a_state;
    // termination trig. clear if it becomes true.
    bool terminate;

    
    // set of  which collision or not
    bool pri_checked;
    bool collision;
    bool chk_set[3];
    double v_r_norm;
} neighbor_gl_t;

typedef struct Vector
{
    float x = 0;
    float y = 0;
    float z = 0;
} Vector_t;

// Class
class Avoidance
{
public:
    Avoidance();
    ~Avoidance();
    void initializeSpecification(double, double , double, double);
    bool threat_checker();
    bool priority_check();
    Vector_t avoid();
    void neighbor_managing();
    //void neighbor_update(int id, float stamp, double lat, double lon, double height, float vx, float vy, float vz);
    void neighbor_update(int id, int type, float stamp, double, double, double, float vx, float vy, float vz);
    void myinfo_update(float stamp, int type, double lat, double lon, double height, float vx, float vy, float vz);
    void myinfo_wp_update(mavros_msgs::Waypoint,mavros_msgs::Waypoint);
    Vector_t getdata_myinfo();

private:
    int neighbor_count;
    neighbor_gl_t neighbor[N_REG];
    information_t myinfo;
    bool trigger,chk_wp_set;
    double v0,dt,bank_angle,d_m,rho_min,d_m_more,rate;


    Vector_t vec_calc(int[],int);
    double vec_calc_AV1(double[], double[], double[], double[],double, double,double, double,int,double[][2]);
    double vec_calc_AV2(double[],double[],double[], double, double,int[],int);

    double FindNormOfV_r(double[],double[],double);

    int TransArray2D(double *source,double *target,int row,int col);
    int InverseMatrix(float *work, float *result, int n);
    int CopyArray2D(float *source, float *target, int n);
    int MulArray2D(float *work1, float *work2, float *result, int r1, int c1, int c2);
    int InitMatrix(float *work, int row, int col);
    int PrintMatrix(float *work, int row, int col, char *label);
    Vector_t vector_rotate(Vector_t vector, float angle);
    float find_angle(Vector_t finding);
    Vector_t normalization(Vector_t com_num);
    Vector_t divide_complex(Vector_t nomi_vector, Vector_t deno_vector);
    Vector_t multiply_complex(Vector_t A, Vector_t B);

    void SetAvoidingFalse(int[],int);
    bool CheckCollision(double[], double[], double, double);
    double SaturationFunc(double, double);
    double ThetaRange(double);
    double GetTheta(double[]);    

    double VectorNorm(double[], int);
    void VectorRotateTheta(double[], double);
    void VectorMinus(double[],double[],double[]);
    void VectorPlus(double[],double[],double[]);
    void VectorScaling(double[],double);

    bool chk_is_it_right_side(double th_colhf_wpline,double r_col_hf[2]);
    int kalman_filter(double x[4], double P[4][4]);
};