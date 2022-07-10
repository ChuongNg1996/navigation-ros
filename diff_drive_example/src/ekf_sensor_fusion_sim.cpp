#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <thread>
#include <iostream>
#include <cmath>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3Stamped.h>
//#include "std_msgs/Bool.h"

// ---------------------------------------------------------- //
// ---------          Sensor Variables              --------- //
// ---------------------------------------------------------- //

//--- IMU ---//

// Acceleration
static float xDotDot_imu = 0;
static float yDotDot_imu = 0;
static float zDotDot_imu = 0;

// Velocity
static float xDot_imu = 0;
static float yDot_imu = 0;
static float zDot_imu = 0;

// Position & Orientation
static float xChange_imu = 0;
static float yChange_imu = 0;
static float zChange_imu = 0;
static float phiChange_imu = 0;     // Roll
static float thetaChange_imu = 0;   // Pitch
static float psiChange_imu = 0;     // Yaw

// Relative to Base
// static float xOffset_imu = 0;
// static float yOffset_imu = 0;
// static float zOffset_imu = 0;
// static float phiOffset_imu = 0;
// static float thetaOffset_imu = 0;
// static float psiOffset_imu = 0;


//--- GPS ---//

// Velocity
// static float xVel_gps = 0;
// static float yVel_gps = 0;
// static float zVel_gps = 0;

// Position
static float xChange_gps = 0;
static float yChange_gps = 0;
static float zChange_gps = 0;
static int gps_factor = pow(10,5);

// Relative to Base
static float xOffset_gps = 0;
static float yOffset_gps = -0.15;
static float zOffset_gps = -0.325;


//--- odom ---// 

// Position & Orientation
static float xChange_odom = 0;
static float yChange_odom = 0;
static float zChange_odom = 0;
static float phiChange_odom = 0;     // Roll
static float thetaChange_odom = 0;   // Pitch
static float psiChange_odom = 0;     // Yaw

// Relative to Base
// static float xOffset_odom = 0;
// static float yOffset_odom = 0;
// static float zOffset_odom = 0;
// static float phiOffset_odom = 0;
// static float thetaOffset_odom = 0;
// static float psiOffset_odom = 0;

//--- Sensor Fusion ---//
static float xChange_final = 0;
static float yChange_final = 0;
static float zChange_final = 0;
static float phiChange_final = 0;
static float thetaChange_final = 0;
static float psiChange_final = 0;

// ---------------------------------------------------------- //
// ---------        Algorithm Variables             --------- //
// ---------------------------------------------------------- //

Eigen::MatrixXd x_update(6,1);
Eigen::MatrixXd x_prev(6,1);
Eigen::MatrixXd x_curr_minus(6,1);
Eigen::MatrixXd A(6,6);
Eigen::MatrixXd B(6,6);
Eigen::MatrixXd sys_input(6,1);

Eigen::MatrixXd sub_p_update(6,6);
Eigen::MatrixXd p_update(6,6);
Eigen::MatrixXd p_prev(6,6);
Eigen::MatrixXd p_curr_minus(6,6);
Eigen::MatrixXd Q(6,6);

Eigen::MatrixXd C(6,6);
Eigen::MatrixXd R(6,6);
Eigen::MatrixXd subK(6,6);
Eigen::MatrixXd K(6,6);

Eigen::MatrixXd y_meas(6,1);
Eigen::MatrixXd I(6,6);

static bool imu_ready = 0;
static bool gps_ready = 0;
//static bool gps_vel_ready = 0;
static bool odom_ready = 0;
static bool ekf_ready = 1;


auto time_start = std::chrono::high_resolution_clock::now();
auto time_end= std::chrono::high_resolution_clock::now();
float t_delta = 0;

void imu_callback (const sensor_msgs::Imu& msg)
{
    if (ekf_ready)
    {
        // xDotDot_imu = msg.linear_acceleration.x;
        // yDotDot_imu = msg.linear_acceleration.y;
        // zDotDot_imu = msg.linear_acceleration.z;
        phiChange_imu = msg.orientation.x;
        thetaChange_imu = msg.orientation.y;
        psiChange_imu = msg.orientation.z;
        imu_ready = 1;
    }
}

void gps_callback (const sensor_msgs::NavSatFix& msg)
{
    if (ekf_ready)
    {
        xChange_gps = msg.latitude * gps_factor + xOffset_gps;
        yChange_gps = -msg.longitude * gps_factor + yOffset_gps;
        zChange_gps = msg.altitude + zOffset_gps;
        gps_ready = 1;
    }
}


// void gps_vel_callback (const geometry_msgs::Vector3Stamped& msg)
// {
//     if (ekf_ready)
//     {
//         xVel_gps = msg.vector.x;
//         yVel_gps = msg.vector.y;
//         //zVel_gps = msg.altitude;
//         gps_vel_ready = 1;
//     }
// }

void odom_callback (const nav_msgs::Odometry& msg)
{
    if (ekf_ready)
    {
        xChange_odom = msg.pose.pose.position.x;
        yChange_odom = msg.pose.pose.position.y;
        zChange_odom = msg.pose.pose.position.z;
        phiChange_odom = msg.pose.pose.orientation.x;
        thetaChange_odom = msg.pose.pose.orientation.y;
        psiChange_odom = msg.pose.pose.orientation.z;

        odom_ready = 1;
    }
}

void ekf ()
{
    // time_end= std::chrono::high_resolution_clock::now();
    // t_delta = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start).count(); 
    // t_delta = t_delta/pow(10,9);

    
    A <<    0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0;
    
    B <<    1,  0,  0,  0,  0,  0,
            0,  1,  0,  0,  0,  0,
            0,  0,  1,  0,  0,  0,
            0,  0,  0,  1,  0,  0,
            0,  0,  0,  0,  1,  0,
            0,  0,  0,  0,  0,  1;

    sys_input << xChange_odom, yChange_odom, zChange_odom, phiChange_odom, thetaChange_odom, psiChange_odom;

    // Compute Priori
    x_curr_minus = A*x_prev + B*sys_input;
    // [6x1] = [6x6]*[6x1] + [6x6]*[6x1]
    p_curr_minus = A*p_prev*A.transpose() + Q;
    // [6x6] = [6x6]*[6x6]*[6x6] + [6x6]

    // Kalman filter gain

    subK = C*p_curr_minus*C.transpose() + R;
    // [6x6] = [6x6]*[6x6]*[6x6] + [6x6]
    K = p_curr_minus*C.transpose()*subK.inverse();
    // [6x6] = [6x6]*[6x6]*[6x6]
     

    // Update
    y_meas << xChange_gps, yChange_gps, zChange_gps, phiChange_imu, thetaChange_imu, psiChange_imu;

    x_update = x_curr_minus + K*(y_meas - C*x_curr_minus);
    // [6x6]*([6x1] - [6x6]*[6x1])
    sub_p_update = (I - K*C);
    p_update = (I - K*C)*p_curr_minus*sub_p_update.transpose() + K*R*K.transpose();

    x_prev = x_update;
    p_prev = p_update;

    ekf_ready = 1;
    //time_start = std::chrono::high_resolution_clock::now();
}


int main(int argc, char **argv)
{

    //-------------         Initialization      -------------//
    ros::init(argc, argv, "EKF");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    //-------------         Topics & Service              -------------//
    ros::Subscriber imu_read =  n.subscribe("/imu", 1, imu_callback);
    ros::Subscriber gps_read =  n.subscribe("/gps", 1, gps_callback);
    //ros::Subscriber gps_vel_read =  n.subscribe("/gps/velocity", 1, gps_vel_callback);
    ros::Subscriber odom_red =  n.subscribe("/odom", 1, odom_callback);


    x_prev << 0, 0, 0, 0, 0, 0;

    p_prev <<   pow(10,-5), 0, 0, 0, 0, 0,
                0, pow(10,-5), 0, 0, 0, 0,
                0, 0, pow(10,-5), 0, 0, 0,
                0, 0, 0, pow(10,-5), 0, 0,
                0, 0, 0, 0, pow(10,-5), 0,
                0, 0, 0, 0, 0, pow(10,-5);
           

    Q << pow(10,-5), 0, 0, 0, 0, 0,
         0, pow(10,-5), 0, 0, 0, 0,
         0, 0, pow(10,-5), 0, 0, 0,
         0, 0, 0, pow(10,-5), 0, 0,
         0, 0, 0, 0, pow(10,-5), 0,
         0, 0, 0, 0, 0, pow(10,-5);

    C << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    R << 0.002501, 0, 0, 0, 0, 0,
         0, 0.002501, 0, 0, 0, 0,
         0, 0, 0.002501, 0, 0, 0,
         0, 0, 0, 0.002501, 0, 0,
         0, 0, 0, 0, 0.002501, 0,
         0, 0, 0, 0, 0, 0.002501;

    I << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;              

    ros::Rate loop_rate(30);
    //time_start = std::chrono::high_resolution_clock::now();
    while(ros::ok())
    {
        if (gps_ready && odom_ready && imu_ready)
        {
            ekf_ready = 0;
            gps_ready = 0;
            odom_ready = 0;
            imu_ready = 0;
            
            std::thread ekf_threading (ekf);
            ekf_threading.detach();
        }

        std::cout << "x predicted: " << x_curr_minus.transpose() << std::endl << std::endl << std::endl;
        std::cout << "y mease: " << y_meas.transpose() << std::endl << std::endl << std::endl;
        std::cout << "K: " << std::endl;
        std::cout << K << std::endl;
        std::cout << "EKF Result: " << x_update.transpose() << std::endl << std::endl << std::endl;

        loop_rate.sleep();
    } 

}
/*
Second Counter: https://stackoverflow.com/questions/39971780/c-second-counter
Thread: https://en.cppreference.com/w/cpp/thread/thread/detach
xhat = [x y]T
yhat = C*xhat
C = [1 0
     0 1] 
-> yhat = xhat = [x y]T
Q = Process Noise
R = Measurement Noise
A = {{  1,      delta_t,        0,      0},
     {  0,      1,              0,      0},
     {  0,      0,              1,      delta_t},
     {  0,      0,              0,      1}}
P = P11     P12     P13     P14
    P21     P22     P23     P24
    P31     P32     P33     P34
    P41     P42     P43     P44
A' = {{ 1,          0,          0,          0},
      { delta_t,    1,          0,          0},
      { 0,          0,          1,          0},
      { 0,          0,          delta_t,    1}}
Pesudo Program:
IMU 
    roscallback1: IF previous update of IMU is done -> r, accel from imu -> w. to a variable  -> INIT the new update of IMU
    Main: IF new update of IMU init  
GPS
    roscallback2: ...
 
*/