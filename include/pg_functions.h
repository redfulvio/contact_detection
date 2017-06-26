#include <eigen3/Eigen/Eigen>
#include <math.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <csignal>
#include <algorithm>
#include <string>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>
#include <std_msgs/String.h>

class proto_functions
{
public:

	proto_functions();

	~proto_functions();

	void protoManager();

private:

	ros::NodeHandle n_;
    ros::Subscriber sub_acc_;
    ros::Subscriber sub_gyro_;
    ros::Publisher pub_fig_;
    ros::Publisher filt_data_;

 /********************** accelerations part ************************/   
    
    //matrices for filter
    Eigen::MatrixXf filt_acc_;
    Eigen::MatrixXf old_filt_acc_;

    //matrix for accelerations
    Eigen::MatrixXf matrix_acc_;
    Eigen::MatrixXf single_acc_;

    //matrix for xcorr
    Eigen::MatrixXf database_window_;
    Eigen::MatrixXf database_window1_;

    int database_size_;
    int count_filt_;
    int iia_; //interested imu axis
    int axis_;
    
    float cross_value_;

    //getDatabaseWindow
    void getDatabaseWindow();

    //callAcc
	void callAcc(qb_interface::inertialSensorArray imu_acc);

    //filterData
    void filterData();

    //updateMatrix
    void updateAccMatrix();

    //loadAcc
    void loadAcc();

    //AccControl
    bool AccControl();

    //checkMaxAcc
    float checkMaxAcc(int imu_number);
    
    //CrossCorrelation
    float crossCorrelation(int axis, int d);

    //possibleFinger
    void possibleFinger();

    //answer
    void answer(int direction);


/************************* gyroscopes part **************************/

    //matrix for gyroscopes
    Eigen::MatrixXf matrix_gyro_x_;
    Eigen::MatrixXf matrix_gyro_y_;
    Eigen::MatrixXf matrix_gyro_z_;

    int direction_;

    //callGyro
    void callGyro(qb_interface::inertialSensorArray imu_gyro);

    //updateMatrix
    void updateGyroMatrix();

    //loadGyro
    void loadGyro();

    //GyroControl
    bool GyroControl();


/****************** common and symmetric part *********************/

	qb_interface::inertialSensorArray acc_;
    qb_interface::inertialSensorArray gyro_;

    Eigen::MatrixXf tmp_acc_;
    Eigen::MatrixXf tmp_acc_old_;
    Eigen::MatrixXf tmp_gyro_;
    Eigen::MatrixXf tmp_gyro_old_;

    bool a_acquisition_flag_;
    bool g_acquisition_flag_;
    bool acquisition_flag_;
    bool work_flag_;
    bool wait_flag_;

    int size_;
    int imu_num_;
    int hit_imu_;
    int silence_;

    //waitCall
    void waitCall();

    //silence
    void silence();

};