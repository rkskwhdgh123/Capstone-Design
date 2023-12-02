
#include"line.hpp"

//////////////sign detect//////////
std::vector<std::string> class_names3 = { "stop","right" ,"left"};
const float CONFIDENCE_THRESHOLD3 = 0.5;
const float NMS_THRESHOLD3 = 0.5;
const int NUM_CLASSES3 = 3;

// colors for bounding boxes
const cv::Scalar colors3[] = {
{0, 255, 255},
{255, 255, 0},
{0, 255, 0},
{255, 0, 0}
};
const auto NUM_COLORS3 = sizeof(colors3) / sizeof(colors3[0]);


//////////////parking detect//////////
std::vector<std::string> class_names = { "parking" };
const float CONFIDENCE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.1;
const int NUM_CLASSES = 1;

// colors for bounding boxes
const cv::Scalar colors[] = {
{0, 255, 255},
{255, 255, 0},
{0, 255, 0},
{255, 0, 0}
};
const auto NUM_COLORS = sizeof(colors) / sizeof(colors[0]);



Mat get_park_roi(Mat frame){
    Mat gray_Area;
    Mat Area=frame(Rect(0,frame.rows/2,frame.cols,frame.rows/2));
    cvtColor(Area, gray_Area, COLOR_BGR2GRAY);
    return gray_Area;
}

Mat park_area_light_control(Mat gray_Area){
    Scalar avr=mean(gray_Area);

    Mat light_roi;
    light_roi=gray_Area+(128-avr[0]);

    return light_roi;
}

Mat get_park_binary(Mat light_roi){
    Mat binary;
    threshold(light_roi, binary, 0, 255, THRESH_BINARY_INV|THRESH_OTSU);
    return binary;
}

Mat detect_line(Mat binary,Mat light_roi, vector<Point> &pts){
    Mat img_labels, stats, centroids;
    Point2f pt1,pt2,pt3,pt4;
    int area;
    int numOfLables = connectedComponentsWithStats(binary, img_labels, stats, centroids, 8, CV_32S);
    int big_num=1;
    int big_value=0;

    for (int j = 1; j < numOfLables; j++) {
    int width = stats.at<int>(j, CC_STAT_WIDTH);
    int height = stats.at<int>(j, CC_STAT_HEIGHT);
    area = stats.at<int>(j, CC_STAT_AREA);
    if((area>big_value)&&(area>3000)&&(area<70000)&&(width!=binary.cols)&&(width>2*height)){big_value=area; big_num=j; }
    }

    int left = stats.at<int>(big_num, CC_STAT_LEFT);
    int top = stats.at<int>(big_num, CC_STAT_TOP);
    int width = stats.at<int>(big_num, CC_STAT_WIDTH);
    int height = stats.at<int>(big_num, CC_STAT_HEIGHT);
    if((stats.at<int>(big_num, CC_STAT_AREA)>4000)&&(stats.at<int>(big_num, CC_STAT_AREA)<70000)){
    // rectangle( binary, Point(left, 2*binary.rows/3+top), Point(left + width, 2*binary.rows/3+top+ height),Scalar(125), 2);
    pt1=Point(left+40, top-10);
    pt2=Point(left+ width-40, top-10);
    pt3=Point(left + width+25, top+ height+25);
    pt4=Point(left-25, top+ height+25);

    pts[0]=pt1;
    pts[1]=pt2;
    pts[2]=pt3;
    pts[3]=pt4;
    

    rectangle( binary, Point(left, top-5), Point(left + width, top+ height),Scalar(125), 2);
    }
    double x = centroids.at<double>(big_num, 0);
    double y = centroids.at<double>(big_num, 1);

    // circle(frame, Point(x, 2*frame.rows/3+y), 5, Scalar(255,0,0), 2, 8);
    circle(binary, Point(x, y), 5, Scalar(0,0,255), 1, 8);

    return binary;
}





Mat draw_point(Mat frame2,int *distance,int *max_distance,int *ignore_x,int *x_sum,int *y_sum, vector<Point2f> corners,  Point2f &mid,Point2f &goal){

    Mat dst;
    frame2.copyTo(dst);
    for(unsigned int i=0; i<corners.size(); i++ ){
        float X = corners[i].x; 
        float Y = corners[i].y+frame2.rows/2;

        *distance=Y;
        if(max_distance<distance){max_distance=distance;
        *ignore_x=corners[i].x;
        }
        *x_sum+=X;
        *y_sum+=Y;
        circle(dst,Point(X,Y),5,Scalar(0,0,255),2);
    }

    if(corners.size()==3){mid.x =(*x_sum-*ignore_x)/(corners.size()-1);}
    else mid.x=*x_sum/corners.size();
    mid.y=*y_sum/corners.size();
    if(corners.size()!=1)
    circle(dst,mid,5,Scalar(0,255,0),2);
    goal.x=mid.x;
    goal.y=mid.y;
    
    return dst;
}

vector<Point2f> sortPoint(Point2f &p1,Point2f &p2,Point2f &p3,Point2f &p4){
    Point2f center(0,0);
    center.x=(p1.x+p2.x+p3.x+p4.x)/4;
    center.y=(p1.x+p2.x+p3.x+p4.x)/4;
    vector<pair<double,Point2f>> angles;
    angles.reserve(4);
    angles.emplace_back(atan2(p1.y-center.y,p1.x-center.x),p1);
    angles.emplace_back(atan2(p2.y-center.y,p2.x-center.x),p2);
    angles.emplace_back(atan2(p3.y-center.y,p3.x-center.x),p3);
    angles.emplace_back(atan2(p4.y-center.y,p4.x-center.x),p4);
    sort(angles.begin(),angles.end(),[](const auto& a,const auto& b){return a.first < b.first;});
    vector<Point2f> sortedpoints;
    sortedpoints.reserve(4);
    for(const auto& anglePoint: angles){
        sortedpoints.push_back(anglePoint.second);
    }
    return sortedpoints;
    
}





Mat get_way_roi(Mat frame, Net net, double *error, double *way_area, string *sign){
    auto output_names = net.getUnconnectedOutLayersNames();
    cv::Mat blob;
	std::vector<cv::Mat> detections;
	cv::dnn::blobFromImage(frame, blob, 1 / 255.f, cv::Size(160, 160), cv::Scalar(),
		true, false, CV_32F);
	net.setInput(blob);
	net.forward(detections, output_names);
	std::vector<int> indices[NUM_CLASSES3];
	std::vector<cv::Rect> boxes[NUM_CLASSES3];
	std::vector<float> scores[NUM_CLASSES3];

	for (auto& output : detections)
	{
		const auto num_boxes = output.rows;
		for (int i = 0; i < num_boxes; i++)
		{
			auto x = output.at<float>(i, 0) * frame.cols;
			auto y = output.at<float>(i, 1) * frame.rows;
			auto width = output.at<float>(i, 2) * frame.cols;
			auto height = output.at<float>(i, 3) * frame.rows;
			cv::Rect rect(x - width / 2, y - height / 2, width, height);
			for (int c = 0; c < NUM_CLASSES3; c++)
			{
				auto confidence = *output.ptr<float>(i, 5 + c);
				if (confidence >= CONFIDENCE_THRESHOLD3)
				{
					boxes[c].push_back(rect);
					scores[c].push_back(confidence);
				}
			}
		}
	}

	for (int c = 0; c < NUM_CLASSES3; c++)
		cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD3,
			indices[c]);
	for (int c = 0; c < NUM_CLASSES3; c++)
	{
		for (int i = 0; i < indices[c].size(); ++i)
		{
			const auto color = colors3[c % NUM_COLORS3];
			auto idx = indices[c][i];
			const auto& rect = boxes[c][idx];
			cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width,
				rect.y + rect.height), color, 3);
			circle(frame,Point(rect.x+rect.width/2,rect.y+rect.height/2),5,Scalar(255, 0, 0),2);
            *error=(frame.cols/2)-(rect.x+rect.width/2);
           
            *way_area=rect.width*rect.height;
			std::string label_str = class_names3[c] +": " + cv::format("%.02lf",scores[c][idx]);
            if(class_names3[c]=="stop") *sign="stop";
            else if(class_names3[c]=="left") *sign="left";
            else if(class_names3[c]=="right") *sign="right";
int baseline;
auto label_bg_sz = cv::getTextSize(label_str,
cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height -
baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y),
color, cv::FILLED);
cv::putText(frame, label_str, cv::Point(rect.x, rect.y - baseline - 5),
cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
}
}
return frame;
}

char control_motor(Dxl dxl,double *error,double way_area,string sign, bool *first_stop,string sign_p,bool p_center_right,bool *second_stop,bool third_stop){
    int rpwm,lpwm;
    double gain=0.11;
    char key=' ';


    if(dxl.kbhit() ){
        key=dxl.getch();
    }

    //모터 속도 제어
    int basic_speed=15;

    if(way_area>=18000) {basic_speed=5; gain=0.2;}
  
    rpwm = basic_speed + gain*(*error/2);
    lpwm = basic_speed - gain*(*error/2);
    
    if(way_area>=30000) {
        if(abs(*error)<=15){
            rpwm=0;
            lpwm=0;
            *first_stop=true;
        }
        else if(*error<0){
           rpwm=-1;
           lpwm=1;
        }else if(*error>0){
            rpwm=1;
           lpwm=-1;
        }
    }


    if(*first_stop==true){  //표지판 보고 정지까지 성공했을 경우에 실행
       if(sign == "right"){rpwm = -5; lpwm =5;}
       else if(sign == "left"){rpwm = 5; lpwm =-5;}
    }
    //cout<<"first stop"<<*first_stop<<endl;


    if(sign_p=="parking"){  //주차장을 발견하고
        if(p_center_right==true){  //주차장이 오른쪽에있을경우에 정지
            rpwm=0;
            lpwm=0;
            if(third_stop==false)
            *second_stop=true;
        }
    }
    if(key=='q'){lpwm=0; rpwm=0;}
    //cout<<way_area<<endl;

    dxl.dxl_set_velocity(lpwm, -rpwm);
    usleep(10000);
    return key;
}

//////////////////////

Mat get_parking_roi(Mat frame,Net net, string *sign_p, bool *p_center_right, double *old_x,double *old_y){
    auto output_names = net.getUnconnectedOutLayersNames();
    cv::Mat blob;
	std::vector<cv::Mat> detections;
	cv::dnn::blobFromImage(frame, blob, 1 / 255.f, cv::Size(160, 160), cv::Scalar(),
		true, false, CV_32F);
	net.setInput(blob);
	net.forward(detections, output_names);
	std::vector<int> indices[NUM_CLASSES];
	std::vector<cv::Rect> boxes[NUM_CLASSES];
	std::vector<float> scores[NUM_CLASSES];

	for (auto& output : detections)
	{
		const auto num_boxes = output.rows;
		for (int i = 0; i < num_boxes; i++)
		{
			auto x = output.at<float>(i, 0) * frame.cols;
			auto y = output.at<float>(i, 1) * frame.rows;
			auto width = output.at<float>(i, 2) * frame.cols;
			auto height = output.at<float>(i, 3) * frame.rows;
			cv::Rect rect(x - width / 2, y - height / 2, width, height);
			for (int c = 0; c < NUM_CLASSES; c++)
			{
				auto confidence = *output.ptr<float>(i, 5 + c);
				if (confidence >= CONFIDENCE_THRESHOLD)
				{
					boxes[c].push_back(rect);
					scores[c].push_back(confidence);
				}
			}
		}
	}

	for (int c = 0; c < NUM_CLASSES; c++)
		cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD,
			indices[c]);
	for (int c = 0; c < NUM_CLASSES; c++)
	{
		for (int i = 0; i < indices[c].size(); ++i)
		{
			const auto color = colors[c % NUM_COLORS];
			auto idx = indices[c][i];
			const auto& rect = boxes[c][idx];
			cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width,
				rect.y + rect.height), color, 3);
            double p_center_x=(rect.x+rect.width)/2;
            circle(frame, Point((rect.x + rect.width/2),(rect.y + rect.height/2)), 5, Scalar(0,0,255), 2);
            if(p_center_x>=200) *p_center_right=true;   //주차장 중심점
            
            float x=0;
            if(rect.width*rect.height>=35000) x=1.4;
            else if(rect.width*rect.height>=27000) x=1.3;
            else if(rect.width*rect.height>=20000) x=1.2;
            else x=0.95;
       
            double yeop=240*x;
            double circle_x=(rect.x+rect.width/2)-yeop;
            double circle_y=rect.y+rect.height/2;
            *old_x=circle_x;
            *old_y=circle_y;

			circle(frame,Point(circle_x,circle_y),5,Scalar(255, 0, 0),2);
            if(class_names[c]=="parking") *sign_p="parking";
            else *sign_p=" ";
			std::string label_str = class_names[c] +": " + cv::format("%.02lf",scores[c][idx]);


int baseline;
auto label_bg_sz = cv::getTextSize(label_str,
cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height -
baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y),
color, cv::FILLED);
cv::putText(frame, label_str, cv::Point(rect.x, rect.y - baseline - 5),
cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
}
}
return frame;
}

/////////////////////////////


char control_motor_second_stop(Dxl dxl,double *old_x,bool *third_stop,string sign_p_right,int p_center_x,double parking_area,bool *fourth_stop,bool *rotate){
    int rpwm=0,lpwm=0;
    char key=' ';


    if(dxl.kbhit() ){
        key=dxl.getch();
    }

    if(*third_stop==false){
        if(abs(320-*old_x)<=10){
            rpwm=0;
            lpwm=0;
            *third_stop=true;
        }else if((320-*old_x)<0){
        *old_x-=1.5;
        rpwm=5;
        lpwm=0;
        }else{
        *old_x+=1.5;
        rpwm=0;
        lpwm=5;
        }
    }

    //cout<<"p_center_x"<<p_center_x<<endl;
    //cout<<"*third_stop"<<*third_stop<<endl;
    if(*third_stop==true){
        
        rpwm=30;
        lpwm=30;
    }   
     if(sign_p_right=="parking"){
            cout<<parking_area<<endl;
            if(parking_area>30000){
            *fourth_stop=true;
     }  

     if(*fourth_stop){
        rpwm=0;
        lpwm=0;
        *rotate=true;
        }
     }
    
    


    if(key=='q'){
        rpwm=0;
        lpwm=0;
    }

    dxl.dxl_set_velocity(lpwm, -rpwm);
    usleep(10000);
    return key;
}

Mat right_camera_parking_roi(Mat frame,Net net,string *sign_p, int *p_center_x, double *parking_area){

auto output_names = net.getUnconnectedOutLayersNames();
    cv::Mat blob;
	std::vector<cv::Mat> detections;
	cv::dnn::blobFromImage(frame, blob, 1 / 255.f, cv::Size(160, 160), cv::Scalar(),
		true, false, CV_32F);
	net.setInput(blob);
	net.forward(detections, output_names);
	std::vector<int> indices[NUM_CLASSES];
	std::vector<cv::Rect> boxes[NUM_CLASSES];
	std::vector<float> scores[NUM_CLASSES];

	for (auto& output : detections)
	{
		const auto num_boxes = output.rows;
		for (int i = 0; i < num_boxes; i++)
		{
			auto x = output.at<float>(i, 0) * frame.cols;
			auto y = output.at<float>(i, 1) * frame.rows;
			auto width = output.at<float>(i, 2) * frame.cols;
			auto height = output.at<float>(i, 3) * frame.rows;
			cv::Rect rect(x - width / 2, y - height / 2, width, height);
			for (int c = 0; c < NUM_CLASSES; c++)
			{
				auto confidence = *output.ptr<float>(i, 5 + c);
				if (confidence >= CONFIDENCE_THRESHOLD)
				{
					boxes[c].push_back(rect);
					scores[c].push_back(confidence);
				}
			}
		}
	}

	for (int c = 0; c < NUM_CLASSES; c++)
		cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD,
			indices[c]);
	for (int c = 0; c < NUM_CLASSES; c++)
	{
		for (int i = 0; i < indices[c].size(); ++i)
		{
			const auto color = colors[c % NUM_COLORS];
			auto idx = indices[c][i];
			const auto& rect = boxes[c][idx];
            if((rect.x + rect.width/2 >200)&&(rect.x + rect.width/2<400))
			cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width,rect.y + rect.height), color, 3);


            
            if((rect.x + rect.width/2 >200)&&(rect.x + rect.width/2<400)){
            circle(frame, Point((rect.x + rect.width/2),(rect.y + rect.height/2)), 5, Scalar(0,0,255), 2);
            *p_center_x=(rect.x+rect.width);
            *parking_area=rect.width*rect.height;
            }
            if(class_names[c]=="parking") *sign_p="parking";
            else *sign_p=" ";
			std::string label_str = class_names[c] +": " + cv::format("%.02lf",scores[c][idx]);


        int baseline;
        auto label_bg_sz = cv::getTextSize(label_str,
        cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);

        if((rect.x + rect.width/2 >200)&&(rect.x + rect.width/2<400))
        cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height -
        baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y),
        color, cv::FILLED);
        
        
        if((rect.x + rect.width/2 >200)&&(rect.x + rect.width/2<400))
        cv::putText(frame, label_str, cv::Point(rect.x, rect.y - baseline - 5),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
        }
    }
return frame;
}




char control_motor_final(Dxl dxl,string sign_final, int final_center_x,double parking_area_final, bool *go_final,int parking_center_height,bool *corner_find,bool rotate,int width,int height){
 
    int rpwm=-7,lpwm=7;
    char key=' ';


    if(dxl.kbhit() ){
        key=dxl.getch();
    }

    
    if(sign_final=="parking"){
        if(parking_area_final>50000){
            rpwm=0; lpwm=0;
            *go_final=true;
        }
    }
    //cout<<"parking_area_final"<<parking_area_final<<endl;
    if(*go_final==true){
        rpwm=15; 
        lpwm=15;
        //cout<<"parking_area_final:   "<<parking_area_final<<endl;
        //cout<<"width:    "<<width<<endl;
        //cout<<"height:    "<<height<<endl;
        if(width>height*3.5){
            if(parking_center_height>=150){   //parking_center_height>=290
                *corner_find=true;
                //cout<<"*corner_find=true"<<endl;
            }
       // rpwm=0; lpwm=0;
       
       }
    }
    if(key=='q'){
        rpwm=0;
        lpwm=0;
    }

    if(*corner_find){
        if(width>height*3.5){
            if(parking_center_height>=150){
                rpwm=0;
                lpwm=0;
                //for(int i=0; i<10; i++)
               // cout<<"조건 충족"<<endl;
            }
        }
    }


    dxl.dxl_set_velocity(lpwm, -rpwm);
    usleep(10000);
    return key;

}


Mat get_parking_roi_final(Mat frame,Net net, string *sign_final,int *final_center_x, double *parking_area_final, int *parking_center_height,int *p_width,int *p_height){
     auto output_names = net.getUnconnectedOutLayersNames();
    cv::Mat blob;
	std::vector<cv::Mat> detections;
	cv::dnn::blobFromImage(frame, blob, 1 / 255.f, cv::Size(160, 160), cv::Scalar(),
		true, false, CV_32F);
	net.setInput(blob);
	net.forward(detections, output_names);
	std::vector<int> indices[NUM_CLASSES];
	std::vector<cv::Rect> boxes[NUM_CLASSES];
	std::vector<float> scores[NUM_CLASSES];

	for (auto& output : detections)
	{
		const auto num_boxes = output.rows;
		for (int i = 0; i < num_boxes; i++)
		{
			auto x = output.at<float>(i, 0) * frame.cols;
			auto y = output.at<float>(i, 1) * frame.rows;
			auto width = output.at<float>(i, 2) * frame.cols;
			auto height = output.at<float>(i, 3) * frame.rows;
			cv::Rect rect(x - width / 2, y - height / 2, width, height);
			for (int c = 0; c < NUM_CLASSES; c++)
			{
				auto confidence = *output.ptr<float>(i, 5 + c);
				if (confidence >= CONFIDENCE_THRESHOLD)
				{
					boxes[c].push_back(rect);
					scores[c].push_back(confidence);
				}
			}
		}
	}

	for (int c = 0; c < NUM_CLASSES; c++)
		cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD,
			indices[c]);
	for (int c = 0; c < NUM_CLASSES; c++)
	{
		for (int i = 0; i < indices[c].size(); ++i)
		{
			const auto color = colors[c % NUM_COLORS];
			auto idx = indices[c][i];
			const auto& rect = boxes[c][idx];
            if((rect.x + rect.width/2 >200)&&(rect.x + rect.width/2<400))
			cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width,
				rect.y + rect.height), color, 3);
            double p_center_x=(rect.x+rect.width)/2;
            if((rect.x + rect.width/2 >200)&&(rect.x + rect.width/2<400)){
            circle(frame, Point((rect.x + rect.width/2),(rect.y + rect.height/2)), 5, Scalar(0,0,255), 2);
            //if(p_center_x>=200) *p_center_right=true;   //주차장 중심점
            *final_center_x=rect.x + rect.width/2;
            *parking_area_final=rect.width*rect.height;
            //cout<<"rect.y + rect.height/2"<< rect.y + rect.height/2<<endl;
            *parking_center_height=rect.y + rect.height/2;
            *p_width= rect.width;
            *p_height=rect.height;
            }

            float x=0;
            int center_y=rect.y + rect.height/2;

            // if(center_y>3) x=1.2;
            // else if(center_y) x=1;
            // else if(center_y) x=0.8;
            // else x=0.95;
       
            //double yeop=210*x;

            double yeop_error;
            if(rect.y + rect.height/2>120)
            yeop_error=((rect.y + rect.height/2)-120)*1.33;
            else
            yeop_error=0;

            double circle_x=(rect.x+rect.width/2)-yeop_error;
            double circle_y=rect.y+rect.height/2;
            //*old_x=circle_x;
            //*old_y=circle_y;
           // cout<<circle_y<<endl;
            if((rect.x + rect.width/2 >200)&&(rect.x + rect.width/2<400)){
			//circle(frame,Point(circle_x,circle_y),5,Scalar(255, 0, 0),2);
            if(class_names[c]=="parking") *sign_final="parking";
            else *sign_final=" ";
            }
			std::string label_str = class_names[c] +": " + cv::format("%.02lf",scores[c][idx]);


int baseline;
auto label_bg_sz = cv::getTextSize(label_str,
cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
if((rect.x + rect.width/2 >200)&&(rect.x + rect.width/2<400))
cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height -
baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y),
color, cv::FILLED);
if((rect.x + rect.width/2 >200)&&(rect.x + rect.width/2<400))
cv::putText(frame, label_str, cv::Point(rect.x, rect.y - baseline - 5),
cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
}
}
return frame;
}


char control_motor_real_final(Dxl dxl,Point2f &goal, bool *back_step, bool *turn, bool* go ,bool *final_go,int *ff_num){
    
    int rpwm=-0,lpwm=0;
    int goal_x_error=320-goal.x;
    ff_num+=1;


    char key=' ';
    if(dxl.kbhit() ){
        key=dxl.getch();
    }

    if(*back_step==false){
    rpwm=-5;
    lpwm=-5;

    if(goal.y>270)
    goal.y-=0.4;
    else if(goal.y<220)
    goal.y-=0;
    else
    goal.y-=0.2;
    }

    if(*back_step==true){
        if(goal_x_error<0){
        rpwm=0;
        lpwm=5;
        goal.x-=1.5;

        }else if(goal_x_error>0){
            rpwm=5;
            lpwm=0;
            goal.x+=1.5;
        }

        if(abs(goal_x_error)<3){
            *turn=true;
        }

    }
    
    if(*turn==true){
        rpwm=15;
        lpwm=15;
        cout<<"goal.y    "<<goal.y<<endl;
        if(goal.y>330){
        rpwm=0;
        lpwm=0;
        // goal.y+=1.3;
        }else if(
        goal.y>300){
        goal.y+=1.1;
        }
        else if(goal.y>270)
        goal.y+=0.9;
        else if(goal.y>220)
        goal.y+=0.7;
        else if(goal.y>180)
        goal.y+=0.4;
        else 
        goal.y+=0.1;
    }

    if(*final_go==true){
        rpwm=0;
        lpwm=0;
    }
    cout<<"*final_go   "<<*final_go<<endl;
    if(key=='q'){
        rpwm=0;
        lpwm=0;
    }
    if(*ff_num>=180){
        rpwm=0;
        lpwm=0;
    }
    cout<<"ff_num"<<ff_num<<endl;


    dxl.dxl_set_velocity(lpwm, -rpwm);
    usleep(10000);
    return key;

}