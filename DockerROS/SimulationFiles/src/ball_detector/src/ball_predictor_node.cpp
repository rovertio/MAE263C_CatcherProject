/* ========================================================================
 *  ball_predictor_node.cpp – stereo CV → projectile-landing predictor
 *  • publish only while ball is RISING  &  h > 5 cm
 *  • no log throttling (prints every processed frame)
 *  • otherwise identical behaviour
 *  • Implementation for simulation with two image cameras in: simset.sdf
 * ===================================================================== */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <deque>
#include <cmath>
#include <algorithm>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>   

using std::placeholders::_1;
using geometry_msgs::msg::Point;

/* ── camera / blob params ──────────────────────────────────────────── */
constexpr int    IMG_W = 640, IMG_H = 240;
constexpr int    HALF_W = IMG_W / 2;
constexpr double IMG_CX = HALF_W / 2.0;
constexpr double IMG_CY = IMG_H  / 2.0;
constexpr double MIN_AREA = 15.0 * 15.0;
constexpr int    THRESH = 225;
constexpr int    ERODE_SZ = 3, DILATE_SZ = 7;

/* ── height estimation ─────────────────────────────────────────────── */
constexpr double AREA_REF = 340.0;
constexpr double H_SCALE  = 120.0;
constexpr double BASELINE = 5;
constexpr double HFOV_DEG = 45.0;
constexpr double CAMERA_H = 85.0;
constexpr double FOCAL_PX = (HALF_W/2.0) /
        std::tan((HFOV_DEG*M_PI/180.0)/2.0);

/* ── predictor & buffers ───────────────────────────────────────────── */
constexpr double LPF_ALPHA = 0.7;
constexpr double VEL_LPF_ALPHA = 0.7;
constexpr double PRED_LPF_ALPHA = 0.6; // For predicted landing position
constexpr double G_CM      = 981.0;
constexpr size_t BUF_MAX   = 4;
constexpr size_t BUF_MIN   = 2;
constexpr double RESET_GAP_S = 0.30;

/* =================================================================== */
class BallPredictorNode : public rclcpp::Node
{
public:
  BallPredictorNode():Node("ball_predictor_node")
  {
    // imgL_sub_ = create_subscription<sensor_msgs::msg::Image>(
    //   "/camera/left_image_raw",10,std::bind(&BallPredictorNode::cb_img,this,_1));
    // imgR_sub_ = create_subscription<sensor_msgs::msg::Image>(
    //   "/camera/right_image_raw",10,std::bind(&BallPredictorNode::cb_img,this,_1));
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    subscriber_imgL_.subscribe(this, "/camera/left_image_raw", rmw_qos_profile);
    subscriber_imgR_.subscribe(this, "/camera/right_image_raw", rmw_qos_profile);


    pub_ = create_publisher<Point>("/ball_centroid",10);

    img_sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(subscriber_imgL_, subscriber_imgR_, 10);
    img_sync_->registerCallback(std::bind(&BallPredictorNode::cb_img, this, std::placeholders::_1, std::placeholders::_2));

    erodeK_  = cv::getStructuringElement(cv::MORPH_ELLIPSE,{ERODE_SZ,ERODE_SZ});
    dilateK_ = cv::getStructuringElement(cv::MORPH_ELLIPSE,{DILATE_SZ,DILATE_SZ});
    RCLCPP_INFO(get_logger(),"Predictor ready (g=%.1f cm/s²)",G_CM);
  }

private:
/* ---- centroid of bright blob in ROI -------------------------------- */
  bool centroid(const cv::Mat&roi,double&cx,double&cy,double&area)
  {
    cv::Mat g,m; cv::cvtColor(roi,g,cv::COLOR_BGR2GRAY);
    cv::threshold(g,m,THRESH,255,cv::THRESH_BINARY);
    cv::erode(m,m,erodeK_); cv::dilate(m,m,dilateK_);
    std::vector<std::vector<cv::Point>> cont;
    cv::findContours(m,cont,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

    double bestA=0; cv::Point2d bestC;
    for(const auto& c:cont){
      double a=cv::contourArea(c); if(a<MIN_AREA) continue;
      auto mo=cv::moments(c); if(mo.m00==0) continue;
      double x=mo.m10/mo.m00, y=mo.m01/mo.m00;
      if(a>bestA){ bestA=a; bestC={x,y}; }
    }
    if(bestA==0) return false;
    cx=bestC.x; cy=bestC.y; area=bestA; return true;
  }

/* ---- main callback -------------------------------------------------- */
  void cb_img(const sensor_msgs::msg::Image::ConstSharedPtr& msg_L, 
              const sensor_msgs::msg::Image::ConstSharedPtr& msg_R)
  {

    // Converting the Image formatted messages for CV processing
    auto cv_ptrL=cv_bridge::toCvCopy(msg_L,sensor_msgs::image_encodings::BGR8);
    auto cv_ptrR=cv_bridge::toCvCopy(msg_R,sensor_msgs::image_encodings::BGR8);
    const cv::Mat& imgL=cv_ptrL->image;
    const cv::Mat& imgR=cv_ptrR->image;

    // Extracting centroid from images from camera feed within simulation
    // if(img.cols!=IMG_W||img.rows!=IMG_H) return;
    double lx,ly,rx,ry,aL=0,aR=0;
    bool fL=centroid(imgL, lx,ly,aL);
    bool fR=centroid(imgR, rx,ry,aR);

    // RCLCPP_INFO(get_logger(),"Image processed");

    if(!fL&&!fR){
      if(last_seen_.nanoseconds()!=0 &&
         (get_clock()->now()-last_seen_).seconds()>RESET_GAP_S)
        {
          buf_.clear();
          has_prev_h_ = false;
          has_prev_v_ = false;
          has_prev_pred_ = false; // Reset prediction LPF state
        }
      return;
    }
    last_seen_=get_clock()->now();

    double px,py,area;
    if(fL&&fR){
      px=(lx+rx)/2; py=(ly+ry)/2; area=(aL+aR)/2;
    }else if(fL){ px=lx; py=ly; area=aL; }
    else         { px=rx; py=ry; area=aR; }

    // Print raw detected values
    // RCLCPP_INFO(get_logger(),
    //   "px=%.1f py=%.1f area=%.1f",
    //   px,py,area);
    // End of image processing for centroid

    double h_area = std::max(0.0, H_SCALE*(std::sqrt(AREA_REF/area)-1.0));
    double disp = lx - rx;
    double dist=(BASELINE*FOCAL_PX)/disp;
    double h_st = CAMERA_H - dist;
    double h_raw = h_st; // Raw height before LPF
    
    if(has_prev_h_) h_raw = LPF_ALPHA*prev_h_ + (1.0-LPF_ALPHA)*h_raw;
    prev_h_=h_raw; has_prev_h_=true;

    double corr=(CAMERA_H-h_raw)/CAMERA_H; // Use filtered height for correction
    double cx=IMG_CX+(px-IMG_CX)*corr;
    double cy=IMG_CY+(py-IMG_CY)*corr;

    double t = msg_L->header.stamp.sec + msg_L->header.stamp.nanosec*1e-9;
    buf_.push_back({t,cx,cy,h_raw}); // Store filtered height in buffer
    if(buf_.size()>BUF_MAX) buf_.pop_front();
    if(buf_.size()<BUF_MIN) return;

    const auto& q0=buf_[buf_.size()-2];
    const auto& q1=buf_.back();
    double dt=q1.t-q0.t; 
    if(dt<1e-4) {
        has_prev_pred_ = false; // Not enough time change for velocity, invalidate prediction
        return;
    }

    double vx_raw=(q1.x-q0.x)/dt;
    double vy_raw=(q1.y-q0.y)/dt;
    double vz_raw=(q1.h-q0.h)/dt;

    double vx, vy, vz;
    if(has_prev_v_){
      vx = VEL_LPF_ALPHA*prev_vx_ + (1.0-VEL_LPF_ALPHA)*vx_raw;
      vy = VEL_LPF_ALPHA*prev_vy_ + (1.0-VEL_LPF_ALPHA)*vy_raw;
      vz = LPF_ALPHA*prev_vz_ + (1.0-VEL_LPF_ALPHA)*vz_raw;
    } else {
      vx = vx_raw; vy = vy_raw; vz = vz_raw;
      has_prev_v_ = true;
    }
    prev_vx_ = vx; prev_vy_ = vy; prev_vz_ = vz;

    if(q1.h <= -80.0 || q1.h >= 80.0 || vz <= -80.0){
      RCLCPP_INFO(get_logger(),"skip  h=%.1f vz=%.1f  (gate)",q1.h,vz);
      has_prev_pred_ = false; // Prediction not made, reset filter state
      return;
    }

    double a_grav=-0.5*G_CM, b_vel=vz, c_height=q1.h;
    double disc=b_vel*b_vel-4*a_grav*c_height; 
    if(disc<0) {
        has_prev_pred_ = false; // Prediction fails, reset filter
        return;
    }
    double t_land=(-b_vel - std::sqrt(disc))/(2*a_grav);
    if(t_land<=0) {
        has_prev_pred_ = false; // Prediction fails, reset filter
        return;
    }

    double raw_pred_x=q1.x+vx*t_land;
    double raw_pred_y=q1.y+vy*t_land;

    double filtered_pred_x, filtered_pred_y;
    if (has_prev_pred_) {
        filtered_pred_x = PRED_LPF_ALPHA * prev_pred_x_ + (1.0 - PRED_LPF_ALPHA) * raw_pred_x;
        filtered_pred_y = PRED_LPF_ALPHA * prev_pred_y_ + (1.0 - PRED_LPF_ALPHA) * raw_pred_y;
    } else {
        filtered_pred_x = raw_pred_x;
        filtered_pred_y = raw_pred_y;
        has_prev_pred_ = true;
    }
    prev_pred_x_ = filtered_pred_x;
    prev_pred_y_ = filtered_pred_y;

    filtered_pred_x = std::clamp(filtered_pred_x, 0.0, double(HALF_W-1));
    filtered_pred_y = std::clamp(filtered_pred_y, 0.0, double(IMG_H-1));

    Point out; 
    out.x = filtered_pred_x;
    out.y = filtered_pred_y;
    out.z = 0.0;
    pub_->publish(out);

    RCLCPP_INFO(get_logger(),
      "h0=%.1f vx=%.1f vy=%.1f vz=%.1f tL=%.3f | Raw(%.1f,%.1f) Filt(%.1f,%.1f)",
      q1.h,vx,vy,vz,t_land, raw_pred_x, raw_pred_y, out.x,out.y);
  }

/* ---- data ----------------------------------------------------------- */
  struct Sample{ double t,x,y,h; };
  std::deque<Sample> buf_;
  rclcpp::Time last_seen_{0,0,RCL_ROS_TIME}; // Initialize to avoid nanoseconds() on zero
  double prev_h_{0}; bool has_prev_h_{false};
  double prev_vx_{0}, prev_vy_{0}, prev_vz_{0}; bool has_prev_v_{false};
  double prev_pred_x_{0.0}, prev_pred_y_{0.0}; bool has_prev_pred_{false};

  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgR_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgL_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_imgL_;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_imgR_;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> img_sync_;

  rclcpp::Publisher<Point>::SharedPtr pub_;
  cv::Mat erodeK_, dilateK_;
};

/* -------------------------------------------------------------------- */
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<BallPredictorNode>());
  rclcpp::shutdown();
  return 0;
}
