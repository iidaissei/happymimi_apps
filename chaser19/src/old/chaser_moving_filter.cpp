/**
*@file   chaser.cpp
*@brief  人追従プログラム cppファイル
*@author　Kosei Demura , Ryoya Yamada
*@date    2017_9_26
*/

//yamadaというコメントがないところは基本的に出村先生

#include "chaser.hpp"

/*
*@brief   Objectのコンストラクタ
*@details 初期化
*/
Object::Object()
{
    last_distance_ = 0;
    last_angle_    = 0;
    local.x        = 0;
    local.y        = 0;
    intensity_min_ = 0;   // min is 0
    intensity_max_ = 255; // max is 255
    world.x        = 0;
    world.y        = 0;
    last_linear_speed_ = 0.0; // 1時刻前の速度,yamada
    last_human_x_=kDefaultDetectPosX; // yamada
    last_human_y_=kDefaultDetectPosY; // yamada
}

/*
*@brief Robotのコンストラクタ
*@details 初期化
*/
Robot::Robot()
{
    linear_speed_  = 0;
    angular_speed_ = 0;
    robot_x_             = 0;
    robot_y_             = 0;
    robot_theta_         = 0;
    human_lost_    = true;
}

/*
*@brief   単純移動平均フィルタ　Simple Moving Average Filter
*@details 出村先生が書かれた移動平均フィルタを過去の人追従プログラムから移植,これで動体と判断した脚の移動平均をとるように
*@param   value  dataに入れる値
*@param   *data  データの入った配列
*@param   length 配列の大きさ
*/
double Robot::SMAfilter(double value, double *data, const int length)
{
    int count = 0;
    double ave = 0;

    data[0]  = value;
    for (int i=0; i < length; i++) {
        if (data[i] == 999) {
            continue;
        }
        else {
            ave += data[i];
            count++;
        }
    }

    if (count != 0) {
        ave /= count;
    }
    else {
        ave = 999;
    }

    for (int i=length-2; i >= 0; i--) {
        data[i+1] = data[i];
    }
    return ave;
}

/*
*@brief 画像を保存するディレクトリを作成
*@details legs_imageディレクトリを作成し、脚を中心とした矩形を画像として保存,yamada
*/
void Robot::makeDir(){
	    struct stat st;
		    if(stat("legs_image", &st) != 0)
			{
				mkdir("legs_image", 0775);
			}
}

/**
*@brief   追従速度の設定をする関数
*@param   Object &human_obj 人までの距離が欲しいのでObject humanを引数にする
*@return  double　　並進速度[m/s]
*@details 人からkFollowDistance[m]後ろに離れた位置にP制御で追従,yamada
*/
double Robot::followLinearSpeed(const Object &human_obj)
{
    double speed = ((human_obj.distance_-kFollowDistance) * kGainLinear );

    // 速度の上限,下限を設定
    if (speed > kLinearMaxSpeed ) {
        speed = kLinearMaxSpeed;
    }
	// 近づきすぎたら停止
    if (speed < 0.1) {
        speed = 0;
    }
    return speed;
}

/**
*@brief   脚の検索範囲の中心位置を初期位置に戻す関数
*@param   Object *human_obj １時刻前の人の位置を使うのでObject humanを引数にする
*@details 時刻前の脚の位置を初期化する
*人を見失った状態なのでlast_linear_speedを0にする,yamada
*/
void Robot::defaultPos(Object *human_obj)
{
    human_obj->last_human_x_ =  kDefaultDetectPosX; // 1時刻前の人の座標
    human_obj->last_human_y_ =  kDefaultDetectPosY;
    human_obj->last_linear_speed_ = 0.0;      // 1時刻前の速度
}

/**
*@brief   減速する関数
*@param   Object *human_obj 減速するために１時刻前の速度を使うのでObject humanを引数にする
*@details 1時刻前の速度を使って減速する。速度が0.1[m/s]以下になったら停止。yamada
*/
void Robot::reduceSpeed(Object *human_obj)
{
    // 徐々に速度を落とす
    human_obj->last_linear_speed_ = human_obj->last_linear_speed_ * 0.8;
    setLinearSpeed(human_obj->last_linear_speed_);

    if(human_obj->last_linear_speed_ <= 0.1) {
        setLinearSpeed(0);
        setAngularSpeed(0);
        human_obj->distance_ = 999;
        human_obj->angle_    = 999;
    }
}

/**
*@brief   並進速度をlinear_speed_に設定する関数
*@param   double linear 並進速度[m/s]
*@details 取得するときはgetLinearSpeed()を使用
*/
void Robot::setLinearSpeed(double linear)
{
    linear_speed_ = linear;
}

/**
*@brief   角速度をangular_speed_に設定する関数
*@param   double angular 角速度[rad/s]
*@details 取得するときはgetAngularSpeed()を使用
*/
void Robot::setAngularSpeed(double angular)
{
    angular_speed_ = angular;
}

/**
*@brief   並進速度[m/s]と角速度[rad/s]をパブリッシュする
*@param   double linear 並進速度
*@param   double angular 角速度
*/
void Robot::move(double linear, double angular)
{
    geometry_msgs::Twist cmd;
    cmd.linear.x  = linear;
    cmd.angular.z = angular;
    cmd_vel_pub.publish(cmd);
}

/**
*@brief   opencvのAPIを使うためにLIDARデータを画像に変換する関数
*@param   int dataCount 走査線数
*@param   double laser_angle_min , double laser_angle_max LIDARの走査線の一番右（最小）と一番左（最大）
*@details lidar_image(500x500)の範囲に反応があったら255、なければ0をlidar_gray_imageの各要素に入力する
*/
void Robot::changeToPicture(int dataCount, double laser_angle_min,
                            double laser_angle_max, double laser_angle_increment)
{

    lidar_gray_image = cv::Scalar::all(0); // 画像データを黒で初期化

    int center = dataCount/2;  // レーザ中央の走査線番号
    int search_lines = 1080 *  (kFollowAngle/270.0); // kFollowAngleの走査線数

	Pose local;
	Pose world;

    for (int j = center - search_lines/2; j <= center + search_lines/2; j++) {
        int x=0, y=0, tmp=0;
		// 走査線の角度
        double angle = (laser_angle_max - laser_angle_min) * j/ (double) dataCount + laser_angle_min;
        // 画像座標系に変換,画像は左上が(0,0)
        x = kMToPixel * laser_distance_[j]*cos(angle) + 
			(int) (0.5 * kImageWidth); // x座標の中心
        y = kMToPixel * laser_distance_[j]*sin(angle) +
			(int) (0.5 * kImageHeight); // y座標の中心
        tmp = kMToPixel * laser_distance_[j]*cos(angle);
        // ROSは進行方向がｘ座標,横がy座標
        x   = kImageWidth/2 - kMToPixel * laser_distance_[j]*sin(angle); // x軸は左右反転
        y   = kImageHeight/2 - tmp; // y軸は上下反転

        // ローカル座標系はROSに合わせて進行方向がx, 左方向がy,yamada
        local.y    = (x - kImageWidth/2) / kMToPixel;
        local.x    = (y - kImageHeight/2) / kMToPixel;
		// ワールド座標系に変換,yamada
        localToWorld(local, &world);

		// cols:横画素数500,rows:縦画素数500
        if ((0 <= x) && (x < lidar_image.cols) && (0 <= y) && (y < lidar_image.rows)) {
            int value = (int) (laser_intensities_[j] * 255.0/6000.0); // 6000は反射強度の最大値
            if (value > 255)  value = 255;
            if (value <   0)  value =   0;

            // グレースケール画像に正規化した反射強度valueを代入
            lidar_gray_image.data[y*lidar_gray_image.step+x*lidar_gray_image.elemSize()] = value;
			// ワールド座標系で画像に描写,yamada
            world_pose_image.at<unsigned char>(kMagnificationWorldImagePos * world.x+250,kMagnificationWorldImagePos * world.y+250) = 0;
        }
    }
	// 動体を検出するための処理,yamada
    // 差分をとるワールド座標系の画像をg_last_image_countごとに更新
	if (g_last_image_count == kUpdateLastImageCount){
	    last_world_pose_image = world_pose_image.clone();
		// 画像を収縮して膨張,ノイズ除去
		cv::erode(last_world_pose_image,erode_last_world_pose_image,cv::Mat(),cv::Point(-1,-1),1);
        cv::dilate(erode_last_world_pose_image,dilate_last_world_pose_image,cv::Mat(),cv::Point(-1,-1), 1);
		g_last_image_count = 0;
		// 画像を初期化
        world_pose_image = cv::Scalar::all(255);
		world_pose_candidate_leg_image = cv::Scalar::all(255);
	}
	g_last_image_count++;
}

/**
*@brief   LIDARデータを受け取るコールバック関数
*@param   sensor_msgs::LaserScan laser_scan トピック/scanをサブスクライブしている
*@details LIDARの走査線数や距離、反射強度の値を変数に代入。LIDARデータをchangeToPictureに渡す。
*/
void Robot::laserCallback(const sensor_msgs::LaserScan laser_scan)
{

    int dataNum = 0;

    // 走査線数
    dataCount_ = laser_scan.ranges.size();
    // LIDARの一番右の走査線-2.356194[rad] -135度
    laser_angle_min_ = laser_scan.angle_min;
    // LIDARの一番左の走査線 2.356194[rad] 135度
	laser_angle_max_ = laser_scan.angle_max;

    for(int i = 0; i < dataCount_; i++) {
        double value = laser_scan.ranges[i]; // 走査線の距離
        if ((value >= laser_scan.range_min) && (value <= laser_scan.range_max))//最大、60[m] 最小 0.023[m]
        {
            laser_distance_[i] = value;
            laser_intensities_[i] = laser_scan.intensities[i]; // 反射強度
        }
        else {
            laser_distance_[i]    =  999; // 無効なデータ
            laser_intensities_[i] = -999;
        }
    }

    // LIDARのデータを画像に変換
    changeToPicture(dataCount_, laser_scan.angle_min,
                    laser_scan.angle_max,laser_scan.angle_increment);

    for(int i = 0; i < dataCount_; i++) {
        laser_last_distance_[i] = laser_distance_[i];
        laser_last_intensities_[i] = laser_intensities_[i];
    }
}

/**
*@brief ローカル座標系をワールド座標系に変換
*@param Pose local_pose Poseは構造体,ローカル座標
*@param Pose *world_pose ワールド座標
*@details ワールド座標系での物体の位置＝回転行列×ローカル座標の物体の位置-ローカル座標の原点(ワールド座標でのロボットの位置)
*/
void Robot::localToWorld(Pose local_pose, Pose *world_pose )
{
    world_pose->x = local_pose.x * cos(robot_theta_) - local_pose.y * sin(robot_theta_) - robot_x_;
    world_pose->y = local_pose.x * sin(robot_theta_) + local_pose.y * cos(robot_theta_) - robot_y_;
    world_pose->theta = robot_theta_;
}

/*
*@brief   ワールド座標系をローカル座標に変換
*@param obj_x ワールド座標のｘ
*@param obj_y ワールド座標のｙ
*@details     ワールド座標をローカル座標に変換してlocal_x,yに代入,yamada
*/
void Robot::worldToLocal(int obj_x, int obj_y, Pose *local_pose)
{
	local_pose->x = (obj_x - robot_x_ ) * cos(robot_theta_) + (obj_y - robot_y_) * sin(robot_theta_) ;
	local_pose->y = -(obj_x - robot_x_) * sin(robot_theta_) + (obj_y - robot_y_) * cos(robot_theta_) ;



}

/**
*@brief   入力画像から脚候補の数を探索する関数
*@param   Mat input_image   入力画像
*@param   Object *object    脚候補のオブジェクト
*@param   Mat display_image 出力画像
*@param   Scalar color      色
*@param   int contour_min ,contour_max 輪郭長:最小、最大[px]
*@paramau int width_min , width_max    外接矩形:横幅最小、最大[px]
*@param   double ratio_min , ratio_max 矩形縦横:比率:最小、最大
*@param   double m00_min , m00_max     0次モーメント(面積(要素数))：最小、最大
*@param   double m10_min , m10_max     1次モーメント(m10:x軸方向の平均値,m01:y軸方向の平均値)：最小、最大
*@param   double diff_x  , diff_y      外接円と重心の差
*@return  int 脚候補の数
*@details 各パラメータを調節して脚候補となるオブジェクトを調整できる
*脚候補は輪郭で判断する,条件に合致した輪郭を脚候補として数え合計の値を返す
*/
int Robot::findLegs(cv::Mat input_image, Object *object,
                    cv::Mat display_image, cv::Scalar color,
                    int contour_min, int contour_max, int width_min, int width_max,
                    double ratio_min, double  ratio_max,double m00_min, double m00_max,
                    double m10_min, double m10_max, double diff_x,  double diff_y)
{
    static int epoch = 0;

    std::vector<std::vector<cv::Point> > contours; // 輪郭
    std::vector<cv::Vec4i> hierarchy;

    // 輪郭の探索
    findContours(input_image, contours, hierarchy,
                 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0 , 0) );

    int object_num = 0; // 脚候補の数
    for(unsigned int cn=0; cn<contours.size(); cn++)
    {

        cv::Point2f center;   // 最小外接円の中心
        float radius;         // 最小外接円の半径
        double tmp, count = 0, intensity = 0;

        // 輪郭の最小外接円
        cv::minEnclosingCircle(contours[cn],center,radius); // radiusは参照

        // ロボットより後ろは除外
        if (center.y -kImageHeight/2 > 0) continue;

        // 輪郭の長さにより除外
        if (!((contours[cn].size() >= contour_min) && (contours[cn].size() <= contour_max))) continue;
		
		//  kFollowMaxDistance * kMToPixelより遠い物体は検出しない
        if (kFollowMaxDistance * kMToPixel < kImageHeight/2 - center.y) continue;
        
		// 外接する長方形を求める
        cv::Rect rect = cv::boundingRect(cv::Mat(contours[cn]));

        // 長方形の底辺による除外
        if (!((rect.width >= width_min) && (rect.width <= width_max))) continue;

		// 長方形の高さによる除外,いらない？調整中yamada
		int height_min = 3;
		int height_max = 20;
		if (!((rect.height >= height_min) && (rect.height <= height_max))) continue;

        // 縦横比による除外
        double ratio;
        if (rect.width != 0) {
            ratio = (double) rect.height/rect.width;
            if (!((ratio >= ratio_min) && (ratio <= ratio_max))) continue;
        }

        // 面積による除外(m00)
        cv::Moments mom = cv::moments(contours[cn]);
        if (!((mom.m00 > m00_min) && (mom.m00 < m00_max))) continue;
        if (!((mom.m10 > m10_min) && (mom.m10 < m10_max))) continue;

        // 重心による判定
        // 脚（円柱）の断面はU字型なので重心のy座標が円より下になる
        // x座標は中心近辺。中心からずれている脚は追従しない
        Point2f point;
        point.x = mom.m10/mom.m00;
        point.y = mom.m01/mom.m00;

        if(center.y - point.y > diff_y)  continue; // 外接円と重心の差

        // 反射強度による除外 tl:左上頂点の座標
        for (int i=rect.tl().y; i < rect.tl().y + rect.height; i++) {
            for (int j=rect.tl().x; j < rect.tl().x + rect.width; j++) {
                tmp = lidar_gray_image.at<uchar>(i, j); // (i,j)での反射強度
                if (tmp != 0) { // 輪郭領域の輝度の合計
                    count++;
                    intensity += tmp;
                }
            }
        }
        // 輪郭領域の平均輝度
        if (count != 0) intensity /= count;
        else intensity = 0;

        // 反射強度は距離の関数なので変更の必要あり
        //double intensity_min = 120, intensity_max = 125;  // チノパン
        //double intensity_min = 80, intensity_max = 160;   // 黒室内
        //double intensity_min = 140, intensity_max = 300;   // 茶色、家
        //human.intensity_min_ = 90; human.intensity_max_ = 159; // コーデロイ

        // 反射強度による除外
        if (!((intensity > human.intensity_min_)
		   && (intensity < human.intensity_max_))) continue;


//データ取得用
#ifdef MOMENT_EXEL
        double contour_size_array[1081], intensity_array[1081], rect_width_array[1081];
        double ratio_array[1081], m00_array[1081], m01_array[1081];
        double m10_array[1081], m11_array[1081];
        double diff_x_array[1081], diff_y_array[1081];
        double contour_size_sum = 0, intensity_sum = 0, rect_width_sum = 0;
        double contour_size_min = 1000, contour_size_max = 0;
        double ratio_sum = 0, m00_sum = 0,  m01_sum = 0, m10_sum = 0, m11_sum = 0;
        double diff_x_sum =0, diff_y_sum = 0;
        double ratio_min = 100, ratio_max = 0, m00_min = 1000, m00_max = 0;
        double m01_min = 100000, m01_max = 0, m10_min = 100000, m10_max = 0;
        double m11_min = 1000000, m11_max = 0;
        double diff_x_min =1000, diff_x_max = -1000;
        double diff_y_min = 1000, diff_y_max =-1000;
        double rect_width_min = 500, rect_width_max = 0;
        double intensity_min = 255, intensity_max = 0;

        if ((center.y > 50)   && (center.y < 250)) {
            if ((center.x > 230) && (center.x < 270)) {
                printf("Epochs=,%d,", epoch);
                printf("Contour[%d],(, %.0f, %.0f,),size=,%d,intensity=,%.1f, ",
                       cn, center.x, center.y, (int) contours[cn].size(),intensity);
                printf(" rect.width=,%d, ratio=,%.2f,",rect.width, ratio);
                printf("  m00=,%.1f, m01=,%.1f, m10=,%.1f, m11=,%.1f, ",mom.m00, mom.m01, mom.m10, mom.m11);
                printf("  center.x-point.x=,%f, center.y-point.y=,%f\n", center.x-point.x, rect.tl().y+rect.height/2-point.y);

                contour_size_array[epoch] = contours[cn].size();
                contour_size_sum         += contours[cn].size();
                if (contours[cn].size() < contour_size_min) contour_size_min = contours[cn].size();
                if (contours[cn].size() > contour_size_max) contour_size_max =contours[cn].size();

                intensity_array[epoch]    = intensity;
                intensity_sum            += intensity;
                if (intensity < intensity_min) intensity_min = intensity;
                if (intensity > intensity_max) intensity_max = intensity;

                rect_width_array[epoch]   = rect.width;
                rect_width_sum           += rect.width;
                if (rect.width < rect_width_min) rect_width_min = rect.width;
                if (rect.width > rect_width_max) rect_width_max = rect.width;

                ratio_array[epoch]        = ratio;
                ratio_sum                += ratio;
                if (ratio < ratio_min) ratio_min = ratio;
                if (ratio > ratio_max) ratio_max = ratio;

                m00_array[epoch]   = mom.m00;
                m00_sum           += mom.m00;
                if (mom.m00 < m00_min) m00_min = mom.m00;
                if (mom.m00 > m00_max) m00_max = mom.m00;

                m01_array[epoch]   = mom.m01;
                m01_sum           += mom.m01;
                if (mom.m01 < m01_min) m01_min = mom.m01;
                if (mom.m01 > m01_max) m01_max = mom.m01;

                m10_array[epoch]   = mom.m10;
                m10_sum           += mom.m10;
                if (mom.m10 < m10_min) m10_min = mom.m10;
                if (mom.m10 > m10_max) m10_max = mom.m10;

                m11_array[epoch]   = mom.m11;
                m11_sum           += mom.m11;
                if (mom.m11 < m11_min) m11_min = mom.m11;

                if (mom.m11 > m11_max) m10_max = mom.m11;

                diff_x_array[epoch]  = center.x - point.x;
                diff_x_sum          += center.x - point.x;
                if (center.x - point.x < diff_x_min) diff_x_min = center.x - point.x;
                if (center.x - point.x > diff_x_max) diff_x_max = center.x - point.x;

                diff_y_array[epoch] = center.y - point.y;
                diff_y_sum        += center.y - point.y;
                if (center.y - point.y < diff_y_min) diff_y_min = center.y - point.y;
                if (center.y - point.y > diff_y_max) diff_y_max = center.y - point.y;
                epoch++;
            }

            double epoch_max = 1000;

            if (epoch >= epoch_max-1) {
                double contour_size_ave = 0, intensity_ave = 0, rect_width_ave = 0;
                double ratio_ave = 0, m00_ave = 0, m01_ave = 0, m10_ave = 0, m11_ave = 0;
                double diff_x_ave = 0, diff_y_ave = 0;

                contour_size_ave = contour_size_sum/epoch_max;
                intensity_ave    = intensity_sum/epoch_max;
                rect_width_ave   = rect_width_sum/epoch_max;
                ratio_ave        = ratio_sum/epoch_max;
                m00_ave          = m00_sum/epoch_max;
                m01_ave          = m01_sum/epoch_max;
                m10_ave          = m10_sum/epoch_max;
                m11_ave          = m11_sum/epoch_max;
                diff_x_ave       = diff_x_sum/epoch_max;
                diff_y_ave       = diff_y_sum/epoch_max;

                double contour_size_sum2 = 0, intensity_sum2 = 0, rect_width_sum2 = 0;
                double ratio_sum2 = 0, m00_sum2 = 0, m01_sum2 = 0, m10_sum2 = 0, m11_sum2 = 0;
                double diff_x_sum2 = 0, diff_y_sum2 = 0;

                for (int i=0; i < epoch_max; i++) {
                    contour_size_sum2 += pow(contour_size_array[i]-contour_size_ave,2);
                    intensity_sum2    += pow(intensity_array[i]-intensity_ave,2);
                    rect_width_sum2   += pow(rect_width_array[i]-rect_width_ave,2);
                    ratio_sum2        += pow(ratio_array[i]-ratio_ave,2);
                    m00_sum2          += pow(m00_array[i]-m00_ave,2);
                    m01_sum2          += pow(m01_array[i]-m01_ave,2);
                    m10_sum2          += pow(m10_array[i]-m10_ave,2);
                    m11_sum2          += pow(m11_array[i]-m11_ave,2);
                    diff_x_sum2       += pow(diff_x_array[i]-diff_x_ave,2);
                    diff_y_sum2       += pow(diff_y_array[i]-diff_y_ave,2);
                }

                double contour_size_sigma, intensity_sigma, rect_width_sigma, ratio_sigma;
                double m00_sigma, m01_sigma, m10_sigma, m11_sigma, diff_x_sigma, diff_y_sigma;
                contour_size_sigma = sqrt(contour_size_sum2)/epoch_max;
                intensity_sigma    = sqrt(intensity_sum2)/epoch_max;
                rect_width_sigma   = sqrt(rect_width_sum2)/epoch_max;
                ratio_sigma        = sqrt(ratio_sum2)/epoch_max;
                m00_sigma          = sqrt(m00_sum2)/epoch_max;
                m01_sigma          = sqrt(m01_sum2)/epoch_max;
                m10_sigma          = sqrt(m10_sum2)/epoch_max;
                m11_sigma          = sqrt(m11_sum2)/epoch_max;
                diff_x_sigma       = sqrt(diff_x_sum2)/epoch_max;
                diff_y_sigma       = sqrt(diff_y_sum2)/epoch_max;

                printf("*****************************************************\n");
                printf("************ Statistical information  ***************\n");
                printf("*****************************************************\n\n");
                printf("contour size: min=%.0f max=%.0f ave=%.2f sigma=%.3f \n",
                       contour_size_min,contour_size_max,contour_size_ave,contour_size_sigma);
                printf("intensity: min=%.0f max=%.0f ave=%.2f sigma=%.3f \n",
                       intensity_min,intensity_max, intensity_ave, intensity_sigma);
                printf("rect width: min=%.0f max=%.0f ave=%.2f sigma=%.3f \n",
                       rect_width_min,rect_width_max, rect_width_ave, rect_width_sigma);
                printf("ratio: min=%.0f max=%.0f ave=%.2f sigma=%.3f \n",
                       ratio_min,ratio_max, ratio_ave, ratio_sigma);
                printf("m00: min=%.0f max=%.0f ave=%.2f sigma=%.3f \n",
                       m00_min,m00_max, m00_ave, m00_sigma);
                printf("m01: min=%.0f max=%.0f ave=%.2f sigma=%.3f \n",
                       m01_min,m01_max, m01_ave, m01_sigma);
                printf("m10: min=%.0f max=%.0f ave=%.2f sigma=%.3f \n",
                       m10_min,m10_max, m10_ave, m10_sigma);
                printf("m11: min=%.0f max=%.0f ave=%.2f sigma=%.3f \n",
                       m11_min,m11_max, m11_ave, m11_sigma);
                printf("diff x: min=%.2f max=%.2f ave=%.2f sigma=%.3f \n",
                       diff_x_min,diff_x_max, diff_x_ave, diff_x_sigma);
                printf("diff y: min=%.2f max=%.2f ave=%.2f sigma=%.3f \n",
                       diff_y_min,diff_y_max, diff_y_ave, diff_y_sigma);

                exit(1);
            }

        }
#endif
        //脚の可能性が高い領域に対して矩形で表示
        cv::rectangle(display_image,rect,color,1);

        object[object_num].radius_    = radius;
        object[object_num].image_pos_ = center;

        // ローカル座標系はROSに合わせて進行方向がx, 左方向がy
        object[object_num].local.y    = (center.x - kImageWidth/2) / kMToPixel;
        object[object_num].local.x    = (center.y - kImageHeight/2) / kMToPixel;
		// 脚候補のオブジェクトをワールド座標系に変換
        localToWorld(object[object_num].local, &object[object_num].world);
        object[object_num].setX(object[object_num].world.x);
        object[object_num].setY(object[object_num].world.y);
        object[object_num].setTheta(object[object_num].world.theta);
        // ワールド座標系にした脚候補のオブジェクトの位置の画素に0,yamada
		world_pose_candidate_leg_image.at<unsigned char>(kMagnificationWorldImagePos * object[object_num].world.x+250,
				                                         kMagnificationWorldImagePos * object[object_num].world.y+250) = 0;
        object_num++;
    }
    // 動体を検出するための処理,yamada
	// 1時刻前の画像と現在の画像の差分
	substraction_world_pose_image = ~world_pose_candidate_leg_image - ~dilate_last_world_pose_image;
    
	// 動体の位置を検出,yamada
	for(int i=0; i<object_num; i++)
	 {
		 // 脚候補のオブジェクトが動体かどうか
		 object[i].judge_dynamic_ = false;
		 // この座標の画素値が0でなければ動体
         int pixel_value = static_cast<int>(substraction_world_pose_image.at<unsigned char>(
					                        kMagnificationWorldImagePos * object[i].world.x + 250,
					                        kMagnificationWorldImagePos * object[i].world.y + 250));

         if(0 < pixel_value)
		 {
			 // 動体であればlidar_imageに描写する
             cv::circle(lidar_image, cv::Point(object[i].image_pos_.x, object[i].image_pos_.y), 3, cv::Scalar(200,200,0), -1, CV_AA);
             // 動体の座標
			 object[i].dynamic_image_pos_ = cv::Point(object[i].image_pos_.x, object[i].image_pos_.y);
			 object[i].judge_dynamic_ = true;
         }
     }

    return object_num;
}

/**
*@brief   人の位置を推定する関数（ローカル座標系）
*@param   int object_num    脚候補の数
*@param   Object *object    脚候補のオブジェクト
*@param   Object *human_obj 人の位置
*@details 脚候補のオブジェクトから人の角度や距離を推定する。
*脚の条件は２個の脚候補同士の距離が0.6mより離れていないこと、検出範囲の中に脚の重心が存在すること。
*移動量から脚の位置を予測
*/
void Robot::calcHumanPose(int object_num,  Object *object, Object *human_obj)
{
    // 人の位置推定アルゴリズム
    // 物体数０：ロスト
    // 物体数１：ロスト
    // 物体数２以上：2個の脚の中心,検出範囲から外れているまたは、２つの重心が60cm以上離れていると除外する。

    double leg1_distance = 999999999, leg1_num=999, image1_distance;
    double leg2_distance = 999999999, image2_distance;
    Point2f leg1_point, leg2_point;
	// yamada
	double obj_radius;
	double diff_distance;
	double last_diff_distance = 999;
	bool   dynamic_obj_flg = false;
    const  int length = 3;
	static double dynamic_leg_point_x[length]; // 移動平均をとるときに動体の脚の位置を入れておく
	static double dynamic_leg_point_y[length];
	

    switch (object_num) {
    case 0:
    case 1: {
        human_obj->distance_ = 999;
        human_obj->angle_    = 999;
        human_obj->local.x  = 999;
        human_obj->local.y  = 999;
        human_obj->setX(999);
        human_obj->setY(999);
        human_obj->setTheta(999);
        human_obj->image_pos_.x = 999;
        human_obj->image_pos_.y = 999;

        return;
    }
    default: {// 2個以上

        // 1個目の脚を探索
        for (int i=0; i < object_num ; i++) {
			// オブジェクトが動体かどうか,yamada
			if (object[i].judge_dynamic_ == true){
				// 画像の中心位置からオブジェクトまでの距離
				image1_distance = (object[i].dynamic_image_pos_.x - kImageWidth/2) * (object[i].dynamic_image_pos_.x -kImageWidth/2) +
							      (object[i].dynamic_image_pos_.y - kImageHeight/2) * (object[i].dynamic_image_pos_.y -kImageHeight/2);
				// 予測した座標からオブジェクトまでの距離(半径) 
				obj_radius = sqrt(pow(object[i].dynamic_image_pos_.x - human_obj->image_expect_human_x_, 2) + 
								  pow(object[i].dynamic_image_pos_.y - human_obj->image_expect_human_y_, 2));
			}
		/*	else {
				image1_distance = (object[i].image_pos_.x - kImageWidth/2) * (object[i].image_pos_.x -kImageWidth/2) +
							      (object[i].image_pos_.y - kImageHeight/2) * (object[i].image_pos_.y -kImageHeight/2);

				obj_radius = sqrt(pow(object[i].image_pos_.x - human_obj->image_expect_human_x_, 2) + 
								  pow(object[i].image_pos_.y - human_obj->image_expect_human_y_, 2));
			}
		*/
			// 座標が検出範囲(円)の中にあるか,yamada
			if(obj_radius < g_find_leg_radius){
                leg1_num  = i;
                leg1_distance = image1_distance;
				// 動体かどうか
				if (object[i].judge_dynamic_ == true){ 
                    leg1_point.x = object[i].dynamic_image_pos_.x;
                    leg1_point.y = object[i].dynamic_image_pos_.y;
					// 出村先生が書かれた移動平均フィルタを過去の人追従プログラムから移植
					// 脚と判断した動体の座標の移動平均
					leg1_point.x = SMAfilter(leg1_point.x, dynamic_leg_point_x, length);
					leg1_point.y = SMAfilter(leg1_point.y, dynamic_leg_point_y, length);
					printf("leg1_point.x = %f, leg1_point.y = %f\n", leg1_point.x,leg1_point.y);
					dynamic_obj_flg = true;
				} 
			/*	else {
				    leg1_point.x = object[i].image_pos_.x;
                    leg1_point.y = object[i].image_pos_.y;
				}
			*/
            }
        }


/*
        // 2個目の脚を探索
        for (int i=0; i < object_num ; i++) {
            //1個目の脚はとばす
            if (i == leg1_num) continue;
			// オブジェクトが動体かどうか,yamada
			if (object[i].judge_dynamic_ == true){
                image2_distance = pow(object[i].dynamic_image_pos_.x - kImageWidth / 2, 2) + 
				                  pow(object[i].dynamic_image_pos_.y - kImageHeight / 2, 2);
                obj_radius = sqrt(pow(object[i].dynamic_image_pos_.x - human_obj->image_expect_human_x_, 2) + 
				                  pow(object[i].dynamic_image_pos_.y - human_obj->image_expect_human_y_, 2));
			} else {
				image2_distance = pow(object[i].image_pos_.x - kImageWidth/ 2, 2) + pow(object[i].image_pos_.y - kImageHeight/2,2);
                obj_radius = sqrt(pow(object[i].image_pos_.x - human_obj->image_expect_human_x_, 2) + 
				                  pow(object[i].image_pos_.y - human_obj->image_expect_human_y_, 2));
			    // leg1までの距離
				diff_distance = sqrt( pow(object[i].image_pos_.x - leg1_point.x, 2) + 
					                  pow(object[i].image_pos_.y - leg1_point.y, 2));

			}

            //脚候補が予測した範囲の中にあるときのみ脚と判断する,yamada
			if(obj_radius < g_find_leg_radius){
                leg2_distance = image2_distance;
				// 動体かどうか
				if(object[i].judge_dynamic_ == true){
					leg2_point.x = object[i].dynamic_image_pos_.x;
                    leg2_point.y = object[i].dynamic_image_pos_.y;
					dynamic_obj_flg = true;
				} else {
				    // leg2から一番近い脚候補のオブジェクトを追いかけるように
				    if(diff_distance < last_diff_distance)
					   leg2_point.x = object[i].image_pos_.x;
                       leg2_point.y = object[i].image_pos_.y;
					   last_diff_distance = diff_distance;
				}
            }
        }
*/
        // 脚の可能性が高いもの中心の座標,yamada
        double tmp_ave_x = leg1_point.x;// + leg2_point.x)/2;
        double tmp_ave_y = leg1_point.y;// + leg2_point.y)/2;

        // 脚の中心座標を予測,yamada
        // １時刻前の脚との座標の差分を求める,yamada
        double image_diff_human_x = fabs(tmp_ave_x - human_obj->last_human_x_);
        double image_diff_human_y = fabs(tmp_ave_y - human_obj->last_human_y_);
        // 求めた差分から人の位置を予測し、次の検出範囲の中心とする,yamada
        human_obj->image_expect_human_x_ = (tmp_ave_x + image_diff_human_x);
        human_obj->image_expect_human_y_ = (tmp_ave_y + image_diff_human_y);

		// 半径,yamada
		double tmp_radius = sqrt(pow(tmp_ave_x - human_obj->image_expect_human_x_, 2) + 
				                 pow(tmp_ave_y - human_obj->image_expect_human_y_, 2));

        // 脚候補の中心が検出範囲の中にあるか(円),yamada
		bool human_pos_judge = tmp_radius < g_find_leg_radius;

        // 片足間の重心の距離[m]
        //double d = sqrt(pow(leg1_point.x - leg2_point.x, 2) + pow(leg1_point.y - leg2_point.y, 2)) /kMToPixel;

        // 脚と判断したときの処理
        if ((/*d < kLegBetweenDistance &&*/ human_pos_judge) && dynamic_obj_flg == true) {

            // 画像中心から脚までの距離[m]
            human_obj->distance_ = sqrt(leg1_distance) / kMToPixel ;//+ sqrt(leg2_distance))/(kMToPixel * 2);
			// 画像中心から脚の位置の角度[rad/s]
            human_obj->angle_    = atan2(leg1_point.x - kImageWidth/2, kImageHeight/2 - leg1_point.y) ;// + (atan2(leg2_point.x - kImageWidth/2, kImageHeight/2 - leg2_point.y)))/2;

            // 現在の脚の座標の中心位置
            double ave_x = leg1_point.x; //+ leg2_point.x)/2;
            double ave_y = leg1_point.y; //+ leg2_point.y)/2;
            
            // 脚を見失っていない時は１時刻前の座標に現在の座標を代入,yamada
            if((ave_x!=0) && (ave_y != 0)) {
                human_obj->last_human_x_   = ave_x;
                human_obj->last_human_y_   = ave_y;
            }

             // 脚の中心位置からの円を描写,yamada
			cv::circle(lidar_image, cv::Point(human_obj->image_expect_human_x_, human_obj->image_expect_human_y_), g_find_leg_radius, 
					   cv::Scalar(0,255,0),2);

            // 人の位置のx座標,見失った場合は１時刻前の座標を使用
            if (human_obj->image_expect_human_x_==0) {
                human_obj->image_pos_.x = human_obj->last_human_x_;
            } else {
                human_obj->image_pos_.x = human_obj->image_expect_human_x_;
            }
            //人の位置のy座標,見失った場合は１時刻前の座標を使用
            if (human_obj->image_expect_human_y_==0) {
                human_obj->image_pos_.y = human_obj->last_human_y_;
            } else {
                human_obj->image_pos_.y = human_obj->image_expect_human_y_;
            }
            return;

        } else {
            human_obj->distance_ = 999;
            human_obj->angle_  = 999;
            human_obj->local.x = 999;
            human_obj->local.y = 999;
            human_obj->setX(999);
            human_obj->setY(999);
            human_obj->setTheta(999);
            human_obj->image_pos_.x = 999;
            human_obj->image_pos_.y = 999;
		}
        return;
    }
    }
}

/**
*@brief   パラメータを表示する関数
*@details 最大、最小追従距離と角度
*/
void Robot::welcomeMessage()
{
    std::cout << "Follow me program by demura lab.,KIT " << std::endl;
    std::cout << "kFollowMaxDistance =" << kFollowMaxDistance << std::endl;
    std::cout << "kFollowMinDistance =" << kFollowMinDistance << std::endl;
    std::cout << "kFollowAngle        =" << kFollowAngle << std::endl;
}

/**
*@brief   録画する際のエラーチェック
*@details 録画する際にエラーだったら終了する
*/
void Robot::prepRecord()
{
    if(!writer1.isOpened()) {
        cout << "video file open error" << endl;
        exit(1);
    }
}

/**
*@brief   imageを録画する関数
*@param   VideoWriter writer imageを書き込む
*@parame  Mat image 録画する画像
*/
void Robot::record(cv::VideoWriter writer, cv::Mat image)
{
    writer << image;
}

/**
*@brief   ロボットの状態をチェックする関数
*@param   double theta 角度
*@return  DANGER 障害物に衝突する
*@return  SAFE   障害物なし
*@details 危険領域に障害物があるかないかをチェックする。
*危険領域はstop_width,stop_distanceの値で決めている
*/
int Robot::checkCondition(double theta)
{
    int center = dataCount_/2; // 走査線の半分540
    int search_lines = 1080 *  (180.0/270.0); // 正面のみの走査線数720
    double angle = 0, x, y;
    double stop_width     = 0.25; //止まるときの範囲,lidarの中心位置から
    double stop_distance   = 0.30; //

    int j = 0;

    for (int i = center - search_lines/2; i <= center + search_lines/2; i++) {

        // 1.5は180度を270度に対応させるため,hokuyoLIDARの最大測定角度は270度
        angle = j * (1.5 * M_PI)/1080;
		j++;
        // 角度theta [rad]に相当する捜査線の数だけ足す（マイナスがあるから)
        int k = i + theta * 1080/(1.5*M_PI); // 走査線
        if (k < center - search_lines/2) continue; // 正面の走査線以外は飛ばす
        if (k > center + search_lines/2) continue;

        x = laser_distance_[k] * cos(angle); // 検出した時の座標
        y = laser_distance_[k] * sin(angle);

        // 障害物が停止範囲に入った場合
        if ((fabs(x) <= stop_width) && (fabs(y) <= stop_distance)) {
            // 衝突する可能性が極めて大。緊急事態
            return DANGER;
        }
    }
    return SAFE;
}

/**
*@brief  衝突を検出する関数
*@return  int SAFE    安全
*@return  int DANGER  障害物あり衝突の危険大
*@details 左右に５度ずつの角度をcheckConditionに渡し、障害物がないかチェックしてする。
*/
int Robot::checkCollision()
{
    for (int i = 0; i < 90; i+= 5) {
        for (int j = -1; j < 1; j+=1) {
            int condition;
			double angle = i * j;
			double avoid_angle = 0;
            condition = checkCondition(DEG2RAD(angle));
            if (condition == SAFE) {
				//LIDARのスキャン方向は反時計回りなのでマイナスをかける
                 avoid_angle = -angle; // この値を使えば障害物回避が可能
				 return SAFE;
            } else if (condition == DANGER) {
                return DANGER;
            }
        }
    }
}

/**
*@brief   脚の位置から人に追従する関数
*@param   input_image lidar画像
*@details calcHumanPoseから人の位置を推定し、並進や回転の制御を行う
*人を見失いかけた時は検出範囲を拡大する。
*/
void Robot::followHuman(cv::Mat input_image)
{

    Object obj[100]; // 脚候補
    int obj_num = 0; // オブジェクトの数

#ifdef MOMENT_EXEL
    // データ収集用
    obj_num = findLegs(input_image, obj, lidar_image, red,
                       0,1000, 0, 1000, 0, 10, 5, 1000, 5, 2500000, 1.5, 2.0);
#else
    // 脚候補を見つけるために原画像で連結領域を探索
    // 脚候補を見つけてその数を返す
    // 長ズボンの際は0次モーメント(輪郭面積)を40,半ズボンの際は34
    obj_num = findLegs(input_image, obj, lidar_image,  red,
                       10,28, 10, 18, 0.2, 1.5, 42/*最小輪郭面積*/, 110, 3400, 25000, 1.5, 0.5);
#endif

    // 人間の位置と方向を計算(ローカル座標系）
    calcHumanPose(obj_num,obj, &human);

    cv::waitKey(1); // 画像表示のために1ms待つ

    // 見つけた人の位置に丸を描写
    cv::circle(lidar_image,cv::Point(human.image_pos_.x, human.image_pos_.y),5,green,2);

    static int lost_count = 0;    // 人を見失った回数
    const int lost_count_max = 1; // この回数だけ連続して失敗すると検出範囲を拡大

    if (human.distance_ == 999) { //人を見失ったとき999
        lost_count++;
    }  else {                   // 人を見失っていない
        human_lost_ = false;
        lost_count = 0;
        g_find_leg_radius = kOriginalRadius; // 検出範囲を元の大きさにする 20[px],yamada
    }
    // 連続lost_count_max回発見できなかったら検出範囲を拡大
    if (lost_count >= lost_count_max) {
        human_lost_ = true;
        g_find_leg_radius = kExtendRadius; // 見失った時に脚検出範囲を拡大する25[px],yamada
    }

    if (human_lost_ ) { // 見失ったときは１時刻前の値を使う
        human.distance_ = human.last_distance_;
        human.angle_    = human.last_angle_;
    }

    // 追跡中の並進の制御,yamada
    if (human.distance_ == 999) { // 人を完全に見失ったとき
        reduceSpeed(&human); // 減速して停止
    } else if (((human.distance_ >=  kFollowMinDistance) &&
                (human.distance_ <=  kFollowMaxDistance))) { // 人が追跡距離内にいる場合
        //printf("人が追跡距離内にいる\n");

        if(lost_count < 15) { // 0.45秒以上人を見失わなければ追跡する
            double tmp_speed = followLinearSpeed(human);
            human.last_linear_speed_ = tmp_speed; // 1時刻前の速度
            setLinearSpeed(tmp_speed);
        } else {
			// ふらついて見失う場合が多いので見失いそうなときは角速度を０[rad/s]に設定する
            setAngularSpeed(0);
            if (lost_count>=kLostTime) { // 人を完全に見失ったら範囲を初期化
                defaultPos(&human);
                human.distance_ = 999;
                human.angle_    = 999;
            }
        }
    } else { // 人が追跡距離外に出た場合
        //printf("人が追跡距離外にいる\n");
        reduceSpeed(&human); // 減速して停止
    }

    // 追跡中の回転の制御
    // 5度以内のときは回転しない
    if ((human.angle_ == 999) || (human.last_angle_ == 999)) {
        setAngularSpeed(0);
    }
    else if (fabs(human.angle_) > DEG2RAD(5.0)) {
        double tmp_speed = getAngularSpeed();
        tmp_speed += -kKp * human.angle_ - kKd * (human.angle_ - human.last_angle_); // PD制御
        setAngularSpeed(tmp_speed);
    }

    // 速度の上限と下限を設定
    if (getAngularSpeed() >  kTurnMaxSpeed)
        setAngularSpeed(kTurnMaxSpeed);
    if (getAngularSpeed() < -kTurnMaxSpeed)
        setAngularSpeed(-kTurnMaxSpeed);

    // 衝突検出
    int collision = checkCollision();

    if (collision == DANGER) { // 衝突する可能性が高いので停止
		move(0,0);
        printf("DANGER\n");
    } else { // 衝突する可能性は低いので進む
        move(getLinearSpeed(),getAngularSpeed());
        printf("SAFE\n");
    }

    human.last_distance_ = human.distance_;
    human.last_angle_    = human.angle_;

}

/**
*@brief   LIDAR画像の処理をする関数
*@details LIDAR画像に縮小処理をし、ノイズ除去を行う
*/
void Robot::prepWindow()
{
    cv::Mat lidar_bin_image;

    // グレースケールに変換する
    cv::threshold(lidar_gray_image, lidar_bin_image, 0, 255,cv::THRESH_BINARY| cv::THRESH_OTSU);

    lidar_bin_image = ~lidar_bin_image; // 反転
    cv::erode(lidar_bin_image, lidar_erode_image, cv::Mat(), cv::Point(-1,-1), 1); // 縮小処理,ノイズ除去
    cv::Mat lidar_color_image;
    cv::cvtColor(lidar_gray_image, lidar_color_image, CV_GRAY2BGR); // カラー画像に変換

    lidar_image = lidar_color_image;
	
}

/**
*@brief   画像を表示する関数
*@details legs_image:脚周辺を切り出した画像,連番で画像を保存する
*/
void Robot::showWindow()
{
    cv::namedWindow( "Map", CV_WINDOW_AUTOSIZE );
	//cv::namedWindow("world", CV_WINDOW_AUTOSIZE);
    cv::Mat dst_img = ~lidar_image;

    cv::imshow("Map",dst_img); // 画像表示
	//cv::imshow("world",substraction_world_pose_image);

    // 動画,画像を取るときは以下をコメントアウトする
    // writer << 取り込みたイメージ名
#ifdef RECORD
    record(writer1, ~lidar_image);
#endif
}

/*
*@brief   角度の符号が逆になってしまうのを直す関数?
*@details 180度?
*/
double wrap_angle(const double &angle)
{
    double wrapped;

	if ((angle <= M_PI) && (angle >= -M_PI)) {
		wrapped = angle;
	} else if (angle < 0.0) {
		wrapped = fmodf(angle - M_PI, 2.0 * M_PI) + M_PI;
	} else {
		wrapped = fmodf(angle + M_PI, 2.0 * M_PI) - M_PI;
	}

    return wrapped;
}


/*
*@brief   オドメトリのコールバック関数
*@details 位置や角度をx,y,thetaに代入
*/
void Robot::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	robot_x_ = msg->pose.pose.position.x;
	robot_y_ = msg->pose.pose.position.y;
	robot_theta_ = tf::getYaw(msg->pose.pose.orientation);
	robot_theta_ = wrap_angle(robot_theta_);
}


/**
*@brief   パブリッシャーとサブスクライバーの宣言をする関数
*@details LIDARのトピックをサブスクライブ、並進速度[m/s]と角速度[rad/]をパブリッシュしている
*/
void Robot::init()
{
    laser_sub        = nh.subscribe("/scan", 100, &Robot::laserCallback,this);
	odom_sub         = nh.subscribe("/odom", 100, &Robot::odomCallback, this);
    cmd_vel_pub      = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100);
}

/**
*@brief main関数
*@param int argc     rosのinitで使用
*@param char* argv[] rosのinitで使用
*/
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "chaser"); // ROSの初期化

    Robot robot;

    //robot.makeDir();

    robot.welcomeMessage(); // パラメータを表示する

    robot.init(); // パブリッシャーとサブスクライバーの宣言

    ros::Rate loop_rate(33); // 33Hz(約30.3ms)のタイマー

#ifdef RECORD
    robot.prepRecord(); // 録画する際のエラーチェック
#endif

    int loop = 0;

    while(ros::ok()) {
        robot.prepWindow(); // LIDAR画像の処理

        if (loop++ < 10) { // LIDARからのデータを読み込むまで時間を稼ぐ
            loop_rate.sleep(); // タイマーからループの処理時間を引いた時間だけ待つ
            ros::spinOnce(); // コールバック関数の呼び出しを許可
            continue;
        }

        robot.followHuman(lidar_erode_image); // LIDAR画像から人に追従する

        robot.showWindow(); // LIDAR画像を表示
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}


