#include <iostream>
#include <opencv2/opencv.hpp>
#include"line.hpp"
using namespace std;
using namespace cv;
using namespace cv::dnn;


int main()
{
	Dxl dxl;
	dxl.dxl_open(); //dxl 오픈.
	float detect_threshold=0.05;
	string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=192.168.54.191 port=8001 sync=false";

	string src2 = "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=192.168.54.191 port=8002 sync=false";


	VideoCapture cap(src, CAP_GSTREAMER);
    VideoWriter writer(dst, 0, (double)15, cv::Size(640,360), true);

	VideoCapture cap2(src2, CAP_GSTREAMER);
    VideoWriter writer2(dst2, 0, (double)15, cv::Size(640,360), true);


	Mat frame;
	Mat frame2;

	auto net = cv::dnn::readNetFromDarknet("yolov4-tiny-stoprightleft.cfg", "yolov4-tiny-stoprightleft_final.weights");
	net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
	net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

	auto net2 = cv::dnn::readNetFromDarknet("yolov3-parking.cfg", "yolov3-parking_final.weights");
	net2.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
	net2.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

	char key=' ';
	double error=0,way_area=0;  // 처음 방향 표지판 찾을때 에러값이랑 표지판 넓이
	string sign; // 처음표지판 찾고 정지했을때 무슨 방향으로 돌지 선택
	string sign_p_right;
	bool first_stop=false;  //이게 true가 되면 표지판으로 방향까지 정하고 회전하는 작업진행
	bool second_stop=false; //이게 true가 되면 회전하다가 주차 영역까지 발견한것.
	bool third_stop=false; //이게 true 가 되면 주차장옆방향까지 보면서 오른쪽 카메라 킬준비 완료
	bool fourth_stop=false;
	bool final_stop=false;

	bool p_center_right=false;
	string sign_p; //주차장 문자열을 저장
	string sign_final;

	int p_center_x;  //오른쪽 카메라로 발견되는 주창영역의 중심값x
	double parking_area;
	double old_x,old_y;
	bool rotate=false;
	bool go_final=false;

	int final_center_x;
	double parking_area_final;
	int parking_center_height;

	bool corner_find_start=false;

	int corner_time=0;
	Point2f goal;
	
	int p_width=100000,p_height=100000;
	bool real_final_circle=false;

	bool back_step=false,turn=false,go=false;
	int back_count=91;
	int go_count_num=270;
	bool final_go=false;
	int ff_num=0;

	while (1)
	{
		cap >> frame;

		if(rotate){
			if(corner_find_start==false)
			frame=get_parking_roi_final(frame,net2,&sign_final,&final_center_x,&parking_area_final,&parking_center_height,&p_width,&p_height);
			else{
				cout<<"goal:   "<<goal<<endl;

				if(real_final_circle){
				circle(frame,goal,5,Scalar(0,255,255),2);
				}

				corner_time+=1;
				if(corner_time<=100){
				Mat park_roi=get_park_roi(frame); 
				Mat park_light_roi=park_area_light_control(park_roi); //이미지 밝기 조절 
				Mat park_binary=get_park_binary(park_light_roi);
					
				vector<Point2f> corners,sortedPoint;
				Mat mask = Mat::zeros(frame.size(),CV_8UC1);
				Mat Area=mask(Rect(0,frame.rows/2,frame.cols,frame.rows/2));
				vector<Point> pts(4);
				Mat Area2=detect_line(park_binary,Area,pts);

			
				fillPoly(Area, pts, cv::Scalar(255), LINE_AA);
				// imshow("Area",Area);
				Point2f mid,points[4];
				int x_sum=0,y_sum=0,ignore_x=0,mid_up_y=0,mid_up_x=0;
				int max_distance=0,distance=0;

				goodFeaturesToTrack(park_light_roi,corners,2,detect_threshold,150,Area); //코너 검출 2개제한,임계값0.05
				Mat dst=draw_point(frame,&distance,&max_distance,&ignore_x,&x_sum,&y_sum,corners,mid,goal); //꼿짓점,중심점 그리기.
				frame=dst;
				}else {corner_time=102; real_final_circle=true; }
			}
			if(real_final_circle==false)
			key=control_motor_final(dxl,sign_final,final_center_x,parking_area_final,&go_final,parking_center_height,&corner_find_start,rotate,p_width,p_height);
			else {

				back_count-=1;
				if(back_count==1)back_step=true;
				else if(back_count<=0)back_count=0;

				go_count_num-=1;

				if(go_count_num==1)final_go=true;
				else if(go_count_num<=0)final_go=true;
				cout<<"numc ount: "<<go_count_num<<endl;

				key=control_motor_real_final(dxl,goal,&back_step, &turn, &go,&final_go,&ff_num );
				}



		}else{



			if(second_stop){ //두번째 정지가 완료 되었으면 실행
				if(third_stop)
				circle(frame, Point(old_x,old_y), 5, Scalar(0,255,0), 2, 8);
				else
				circle(frame, Point(old_x,old_y), 5, Scalar(255,0,0), 2, 8);
				key=control_motor_second_stop(dxl,&old_x,&third_stop,sign_p_right,p_center_x,parking_area,&fourth_stop,&rotate);
			}else {
				if(first_stop==false) //첫번째 정지가 완료 되지 않았을때 실행
				frame=get_way_roi(frame,net,&error, &way_area, &sign); 
			else
				frame=get_parking_roi(frame,net2,&sign_p,&p_center_right,&old_x,&old_y); 
				key=control_motor(dxl,&error,way_area,sign,&first_stop,sign_p, p_center_right,&second_stop,third_stop);

			}
		}
		writer<<frame;	

		if(third_stop){ //세번째 정지까지 완료되었다면 오른쪽 카메라 켜짐
			cap2>>frame2;
			if(rotate==false){
			frame2=right_camera_parking_roi(frame2,net2,&sign_p_right,&p_center_x,&parking_area);  	writer2<<frame2;}
			
			//writer2<<frame2;	
		}

    	if (waitKey(2) == 27) key='q';
		if(key=='q')break;
		
	}

	dxl.dxl_close(); //dxl 닫기.
	return 0;
}
