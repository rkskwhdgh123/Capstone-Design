#ifndef _MYLIB_H_
#define _MYLIB_H_
#include<iostream>
#include "opencv2/opencv.hpp"
#include "dxl.hpp"
#include <fstream>
#include <cmath>
#include <cstdlib>
using namespace std;
using namespace cv;
using namespace cv::dnn;

Mat get_park_roi(Mat frame);
Mat park_area_light_control(Mat gray_Area);
Mat get_park_binary(Mat light_roi);
Mat detect_line(Mat binary,Mat light_roi,vector<Point> &pts);
Mat draw_point(Mat frame2,int *distance,int *max_distance,int *ignore_x,int *x_sum,int *y_sum, vector<Point2f> corners,  Point2f &mid,Point2f &goal);
vector<Point2f> sortPoint(Point2f &p1,Point2f &p2,Point2f &p3,Point2f &p4);

Mat get_way_roi(Mat frame, Net net, double *error,double *way_area,string *sign);
char control_motor(Dxl dxl,double *error,double way_area,string sign, bool *first_stop,string sign_p,bool  p_center_right,bool *second_stop,bool third_stop);
Mat get_parking_roi(Mat frame,Net net,string *sign_p, bool *p_center_right,double *old_x,double *old_y);
char control_motor_second_stop(Dxl dxl,double *old_x,bool *third_stop,string sign_p_right,int p_center_x,double parking_area,bool *fourth_stop,bool *rotate);
Mat right_camera_parking_roi(Mat frame,Net net,string *sign_p,int *p_center_x,double *parking_area);
char control_motor_final(Dxl dxl,string sign_final, int final_center_x,double parking_area_final,bool *go_final,int parking_center_height,bool *corner_find,bool rotate,int width,int height);
Mat get_parking_roi_final(Mat frame,Net net, string *sign_final,int *final_center_x, double *parking_area_final,int *parking_center_height,int *p_width,int *p_height);
char control_motor_real_final(Dxl dxl,Point2f &goal,bool *back_step, bool *turn, bool* go ,bool *final_go,int *ff_num);
#endif