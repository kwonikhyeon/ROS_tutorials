#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/Twist.h"



using namespace cv;

using namespace std;

geometry_msgs::Twist msg;


void ImageCallbak(const sensor_msgs::Image::ConstPtr &img)
{
  double spdCurve = 0;
  cv_bridge::CvImagePtr cv_ptr;

  ROS_INFO("Image(%d, %d)", img->width, img->height);

  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Error to convert!");
    return;
  }


  double sum=0;
  int cnt=0;
  Mat src = cv_ptr->image;
  Mat dst, color_dst,gray;
  Mat img_roi1;
  Mat adapt;

  cvtColor( src, gray, COLOR_BGR2GRAY );
  adaptiveThreshold(gray, adapt, 255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,7,10);
  GaussianBlur(adapt,adapt,Size(7,7),0);
  Canny( adapt, dst, 170, 250, 3 );
  //img_roi1=dst(Rect(0,0,639,100));
  //img_roi1=0;

  cvtColor( dst, color_dst, COLOR_GRAY2BGR );

  vector<Vec3f> circles;
  HoughCircles(dst, circles, CV_HOUGH_GRADIENT, 2, dst.rows / 4, 50, 150);

  if (circles.size()>2){
    msg.linear.x = 0;
    msg.angular.z = 0;
    return;
  }

  vector<Vec4i> lines;
  HoughLinesP( dst, lines, 1, CV_PI/180, 110, 100, 50);
  for( size_t i = 0; i < lines.size(); i++ )
  {
      double m=(double(lines[i][3]-lines[i][1]))/double((lines[i][2]-lines[i][0]));

      if(m<-0.5 || 0.5<m)
      {
        line( color_dst, Point(lines[i][0], lines[i][1]),
        Point( lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
        sum+=m;
        cnt++;
      }
  }


ROS_INFO("count of m = %d\n", cnt);

  if(cnt>=10){
    ROS_INFO("sum of m = %f\n", sum);

    spdCurve = sum / 60;
    msg.linear.x = 0.15;
  }
  else if(cnt>=8 && cnt<10){
    ROS_INFO("sum of m = %f\n", sum);

    spdCurve = sum / 40;
    msg.linear.x = 0.08;
  }
  else{
    int i,j;
    int x1=0,y1=0,x2=0,y2=0, pCnt=0, y_mean=0;
    double mCurve;

    for(pCnt=0, i=290, j=479; j>0; j--){
      if(dst.at<uchar>(j,i) > 127){ pCnt ++; if(pCnt >= 3){x2=i; y2=j; break;}}
      if(j==101){x2=320; y2=0; break;}
    }
    ROS_INFO("%d, %d, %d\n", x2, y2, pCnt);
    for(pCnt=0, i=450, j=479; j>0; j--){
      if(dst.at<uchar>(j,i) > 127){ pCnt ++; if(pCnt >= 3){x1=i; y1=j; break;}}
      if(j==101){x1=480; y1=0; break;}
    }
    ROS_INFO("%d, %d %d\n", x1, y1, pCnt);

    y_mean=(y1+y2)/2;
    ROS_INFO("%d\n", y_mean);
    if(y2-y1){mCurve = double(x2-x1)/double(y2-y1);}
    ROS_INFO("inverse of m = %f\n", mCurve);
    if(mCurve>0)
    {
      spdCurve=mCurve/5; //0.22;
    }
    else if(mCurve<0)
    {
      spdCurve=mCurve/5;   //-0.2;
    }
    if(200-y_mean > 0){
      msg.linear.x = 0.1+double((165-y_mean))/double(2000);
    }
    else{
      msg.linear.x = 0.1+double((165-y_mean))/double(1800);
    }
    ROS_INFO("linear.x_spd = %f\n",0.1+double(190-y_mean)/double(600));
  }

  //imshow( "Source", src );

  imshow( "Detected Lines", color_dst );

  msg.angular.z = spdCurve;





  waitKey(1);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "main");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe("/raspicam_node/image", 1, ImageCallbak);

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Rate rate(10);
  ROS_INFO("Starting to move forward");
  while (ros::ok()) {
          pub.publish(msg);
          ros::spinOnce();
          rate.sleep();
          }
}