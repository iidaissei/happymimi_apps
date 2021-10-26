// RoboCup@Home Follow me 競技用プログラム
// 150411: githubに上げるためにdfollow15に名称を変更した。15は2015年の意味。
// 150308: dfollow4にバージョンアップ
//         台車の制御周期が遅いのでRTのKobukiの関数を止めて、ROSのmove_baseを利用した
//         ~/hydro/src/turtlebot_apps/turtlebot_navigation/param$ 
//          move_base_params.yamlのcontroller_frequencyをデフォルトの20から120にアップしたが
//          ふらつく症状は改善されない。
//         クラス化が不完全だったので、Robotクラス化した。
// 150215: 直交座標系に基づく比例航法を実装したが、性能差はほとんどない。
// 150210: ふらつくのは制御の問題と判明
// 150209: 世界座標系world、ローカル座標系localの導入。標準は世界座標系。
// 150207: 音声認識機能追加。VAIOの場合はマイクをUSB経由で入力するのでLIDAR、マイクが
// うまく機能しない場合がある
// 150126: Huモーメントを試したがあまりうまくいかない。intensityでわけることに成功
// 150124: 差分はレーザ光線一本ごとに計算するとうまくいかない。輪郭の重心で計算すべき
// 150118: LIDARで動いている範囲を検出し、その範囲だけ画像化する。
// 150117: 動いている物体のみを追跡
// 150117: 動作がふらつくので回転速度を0.6[rad/s]と遅くした
//         LIDAR反射強度の導入. 脚の反射強度は広い帯域なので靴を拾っている可能性あり
//         rqt -s kobuki_dashboardでLidarのtopic frequencyがtoo lowと警告されたので、
//         ~/turtlebot/turtlebot/src/turtlebot/turtlebot_bringup/3dsensor.launchに
//         以下を追加
// <arg name="scan_topic" default="kinect_scan"/>
// <node name="laser_driver" pkg="urg_node" type="urg_node"> 
//   <param name="frame_id" value="base_laser_link" />
//   <param name="intensity" type="bool" value="true"/>
//   <param name="min_ang" value="-1.047"/>   
//   <param name="max_ang" value="1.047"/>    
//   <param name="cluster" value="1"/>
//    <param name="skip" value="1"/>
// </node>
// 150115: 比例航法導入
// 150112: LIDAR画像を安定させるために１時刻前の画像と現在の画像のANDを取るようにした
//         識別のパラメータ調整
// 150111: 高速化のためlidarのデータをgray scaleに格納。輝度はintensity情報
//         追跡時にふらつく原因は、一番近い物体を追跡しているため。
//         2個物体を見つけた時はその重心を物体の位置とするように変更。
// 150104: 輪郭抽出により物体を取り出し、その重心、面積、モーメントにより
//         脚と思われる物体を推定している。パラメータの調整が不十分。
//         
// 141210: LIDARのデータを画像化してOpenCVで処理する
// 141210: 切り出しがうまくいかず、凸壁を脚と間違える
// 141203: 何とか追跡できるようになったが、壁を追跡者と間違えたる
// To Do: 人と壁の判別、比例航法の実装

#ifndef DFOLLOW3_HPP
#define DFOLLOW3_HPP

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <std_msgs/String.h>

#include <string>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <boost/regex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
//#include "rospeex_if/rospeex.h"
//static rospeex::Interface interface;





//#define MOMENT_EXEL// 慣性モーメントデータ取得用
//#define DEBUG      // デバッグ用
#define RECORD       //録画用
#define MOVE         //動かさないときはコメントアウト
//#define PROPORTIONAL_NAVI /// 比例航法でないときはコメントアウト
///#define HOME      // 自宅用のパラメータ
const int VIDEO_NO=1;// cam360

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )
#define RAD2DEG(RAD) ( 180.0  * (RAD) / (M_PI))

using namespace std;
using namespace cv;
const double kMovingThreshold = 0.01;   // 移動物検出のしきい値[m] 
const double kFollowMaxDistance = 6.0; // follow distance
const double kFollowDistance = 2.5;     // follow distance
const double kFollowMinDistance = 0.6; // 0.2follow distance大会は0.8
const double kLegWidthMax = 0.3;       // 脚幅の最大値
const double kLegWidthMin = 0.1;       // 脚幅の最大値
const double kFollowAngle  = 180;       // 探す範囲は正面のこの角度[deg]  
const double kGainLinear   = 0.4;       // P制御比例ゲイン（並進）大会のときは0.7
const double kGainTurn     = 0.5;       // P制御比例ゲイン（回転）
const int    kTemplateNumber    = 10;   // テンプレート数     

#ifdef HOME // 家庭用　狭い環境
const double kp                 = 10;
const double kd                 =  5;
const double gain_proportion    = 10;    // 比例航法のゲイン FMT用 0.1 家　10
const double kLinearMaxSpeed    = 0.3;  // FMT用 0.6  家用 0.3 　最大　
const double kTurnMaxSpeed      = 2.4;  // FMT用 0.6　家用 1.2 　最大　3.14
#else  // FMT yamada
const double kKp                = 0.10; // 4, 10 kp: 30, kd: 0 
const double kKd                = 5.25;// 
const double kGainProportion    = 4;  // 比例航法のゲイン FMT用 0.1 家　10
const double kLinearMaxSpeed    = 0.6;  // FMT用 0.6  家用 0.3 　最大　 
const double kTurnMaxSpeed      = 1.0;  // 0.8 FMT用 0.6　家用 1.2 　最大　3.14 
const double kDefaultAveX		= 250;//始めの探索範囲のxの中心位置
const double kDefaultAveY       = 200;//初めの探索範囲のyの中心位置
      int    area               = 20 ;//予測の位置を中心とした探索範囲の大きさ、大きいと人以外を判断してしまう、小さいと見失いやすい
const int    kExtendArea        = 25 ;//人が横切ったあとの探索範囲
const int    kOriginalArea      = area;
      bool   avoid_flg     = false; //とりあえずここで宣言しておくうまくいったらクラスの中にいれる
	  bool   right_avoid_flg    = false; //
	  bool   first_human_detect_flg = false;//
#endif

//double laser_time, laser_last_time, laser_diff_time; //  [s]
//double target_angle;     // target angle [rad] center is 0 [rad]
//double target_distance;  // minimum distance from a robot

const int IMAGE_WIDTH=500, IMAGE_HEIGHT=500;
const int IMAGE360_WIDTH=500, IMAGE360_HEIGHT=500;

const double mToPixel = 50; // mをpixelへ変換 1m == 50 pixel, 1 pixel = 2cm

cv::RNG rng(12345); // 乱数発生器　12345は初期化の種


// 初期化時に塗りつぶす                                                    
//cv::Mat cam360_image(cv::Size(IMAGE360_WIDTH, IMAGE360_HEIGHT),
//		     CV_8UC3, cv::Scalar::all(255));

cv::Mat lidar_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 
   	    CV_8UC3, cv::Scalar::all(255));
cv::Mat lidar_gray_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
	    CV_8U, cv::Scalar::all(255));
cv::Mat human_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
	    CV_8U, cv::Scalar::all(255));
// 1時刻前の画像_
cv::Mat lidar_gray_old_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 
	    CV_8U, cv::Scalar::all(255));



// 脚と推定した輪郭
cv::Mat detect_image;
cv::Mat detect_old_image;

cv::Mat e1_img,e1_old_img,diff_img;
cv::Mat e3_img, e3_old_img, diff3_img;

//yamda
cv::Mat img_dst,img_masked,img_gaussian;

// テンプレートの輪郭
cv::vector<cv::vector<cv::vector<cv::Point> > >template_contours;
cv::vector<cv::vector<cv::Point> > template_contour;
cv::Scalar red(0,0,255), blue(255,0,0), green(0, 255, 0);

//cv::VideoCapture cap(VIDEO_NO); // cam360

cv::VideoWriter writer1("videofile.avi", CV_FOURCC_MACRO('X', 'V', 'I', 'D'), 30.0,
		       cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), true);
cv::VideoWriter writer2("videofile2.avi", CV_FOURCC_MACRO('X', 'V', 'I', 'D'), 30.0,
		       cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), true);
cv::VideoWriter writer3("videofile360.avi", CV_FOURCC_MACRO('X', 'V', 'I', 'D'), 30.0,
			cv::Size(IMAGE360_WIDTH, IMAGE360_HEIGHT), true);


std::vector<std::string> templateFiles;

std_msgs::String voice_command;

enum  Robot_State { STATE_BEGIN, STATE1, STATE2, STATE3, STATE_END, STATE_TEST };
enum  Robot_Condition { SAFE = 0, DANGER = 119, IN_ELEVATOR = 999,LEFT_COLLISION = 1,RIGHT_COLLISION = 2,LEFT_DANGER = 3,RIGHT_DANGER = 4 };

struct Pose {
  double x;
  double y;
  double z;
  double theta;
  double velocity; 
};

class Object {
private:
  double x_;     // position in the world coordinate
  double y_;
  double z_;
  double theta_;

public:
  Object();
  Pose local,last_local; // local coordinate: ROSに合わせて進行方向がx, 左方向がy
  Pose world,last_world; // world coordinate
  bool leg_;   // leg:true  or not:false
  int begin_; // right side laser
  int end_;   // left side laser
  int diff_;
  int intensity_min_, intensity_max_; // 反射強度の最小、最大値　[0:255]
  double distance_,last_distance_; // present, last detected distance
  double angle_,last_angle_; // rad
  double width_;
  double radius_; // radius of an enclose circle
  cv::Point2f image_pos_; // position of an image
  double getX() { return x_; }
  double getY() { return y_; }
  double getZ() { return z_;}
  double getTheta() { return theta_;}
  void   setX(double _x) {x_ = _x;}
  void   setY(double _y) {y_ = _y;}
  void   setZ(double _z) {z_ = _z;}
  void   setTheta(double _theta) {theta_ = _theta;}
 //yamada
  double last_ave_x_,last_ave_y_;//1時刻前の人の中心位置
  double last2_ave_x_,last2_ave_y_;//２時刻前と１時刻前の位置を使って予測したいので宣言してお
  double image_diff_ave_x_,image_diff_ave_y_;
  double image_expect_ave_x_,image_expect_ave_y_;
  double last_linear_speed_;//スムーズに止まるために１時刻前の速度をとっておく
  double avoid_angle_;//障害物があったときの回転する角度
  double return_avoid_angle_;//障害物回避後に回転する角度
  double avoid_angle_speed_;
  bool   last2_ave_flg_;//今の方法だと初めの座標の値が倍になってしまうのでを管理するために宣言 
  bool   chasing_flg_;//人を追いかけているかどうか 
  bool   danger_flg_;//追いかけている最中に障害物


};

class Robot {
private:
  bool   human_lost_;  // 
  bool   second_section_; // true if the robot pass the second section 
  int    state_; // state of robots; 
  double time_, last_time_; // [s]
  double x_,y_,theta_; // position [m], orientation [rad] in the world coordiante
  double vx_, vy_, vth_; // velocity in the world coordinate (odometry)
  double laser_distance_[1081]; // hokuyo lidar UTM-30LX                                     
  double laser_last_distance_[1081];
  double laser_intensities_[1081];
  double laser_last_intensities_[1081];

  double last_x_, last_y_, last_theta_; // １時刻前のx, y, theta
  //geometry_msgs::Twist cmd_speed; // tmp_speed;
  double linear_speed_, angular_speed_; // [m/s]
  geometry_msgs::Twist cmd_vel, zero_cmd_vel;

  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub;
  ros::Publisher find_human_pub;
  ros::Publisher lost_human_pub;//yamada
  ros::Subscriber follow_human_sub;
  ros::Subscriber laser_sub;
  ros::Subscriber odom_sub;


public:
  Robot();
  Pose   local; // local coordinate
  Pose   world;
  int    dataCount_;
  double laser_angle_min_, laser_angle_max_; 
  void   calcHumanPose(int object_num,  Object *object, Object *human_object);
  void   changeToPicture(int dataCount, double laser_angle_min,
			 double laser_angle_max, double laser_angle_increment);
  int    checkCollision();
  int    checkCondition(double theta);
  double checkFrontDistanceMarco(); // by marco
  bool   checkObstacles(double distance);
  string follow_command; // follow command
  double findDirection(double distance);
  bool   findHuman(cv::Mat input_image);
  int    findLegs(cv::Mat input_image, Object *object, cv::Mat result_image,
	   cv::Mat display_image, const string& winname, cv::Scalar color,
	   int contour_min, int contour_max, int width_min, int width_max,
	   double ratio_min, double  ratio_max,double m00_min, double m00_max,
		  double m10_min, double m10_max, double diff_x,  double diff_y);
  void   followHuman(cv::Mat input_image, bool move);
  void   followHumanCallback(const std_msgs::String msg);
  double getAngularSpeed() { return angular_speed_; }
  double getLastX(){ return last_x_; }
  double getLastY(){ return last_y_; }
  double getLastTime() { return last_time_; }
  double getLastTheta(){ return last_theta_; }
  double getLinearSpeed()  { return linear_speed_; }
  double getTheta(){ return theta_; }
  double getTime() { return time_;  }
  double getX(){ return x_;}
  double getY(){ return y_;}
  void   goAhead(double distance); // distanceの距離だけ前進
  void   init(); // initialize
  void   laserCallback(const sensor_msgs::LaserScan laser_scan);
  void   localToWorld(Pose local_pose, Pose *world_pose );
  double median(int no, double *data);
  void   memorizeIntensity();
  void   memorizeOperator();
  void   move();
  void   move(double linear, double angular);
  void   odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void   prepRecord();
  void   prepWindow();
  void   record(cv::VideoWriter writer, cv::Mat image);
  void   setAngularSpeed(double angular);
  void   setLastX(double _last_x){ last_x_ = _last_x; }
  void   setLastY(double _last_y){ last_y_ = _last_y; }
  void   setLastTime(double _last_time) { last_time_ = _last_time; }
  void   setLastTheta(double _last_theta){ last_theta_ = _last_theta; }
  void   setLinearSpeed(double linear);
  void   setPose(double x, double y, double th);
  void   setTheta(double _theta){ theta_ = _theta; }
  void   setTime(double _time) { time_ = _time;  }
  void   setX(double _x){ x_ = _x; }
  void   setY(double _y){ y_ = _y; }
  void   showWindow();
  double SMAfilter(double value, double *data, const int length);
  void   speechCallback(const std_msgs::String& voice);
  void   stop();
  void   test();
  void   turn(double rad);
  void   welcomeMessage();
  //yamada
  double pLinearSpeed(const Object &human_obj);
  void   defaultPos(Object *human_obj);
  void   objectSort(int object_num,Object *object,const Object &human_obj);
  void   reduceSpeed(Object *human_obj,int lost_count);
  };

  Object human;


#endif
