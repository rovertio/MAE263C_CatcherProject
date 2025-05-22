/* ========================================================================
 *  ball_predictor_node.cpp – stereo CV → projectile-landing predictor
 *  • publish only while ball is RISING  &  h > 5 cm
 *  • no log throttling (prints every processed frame)
 *  • otherwise identical behaviour
 * ===================================================================== */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <deque>
#include <cmath>
#include <algorithm>

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
constexpr double BASELINE = 5.5;
constexpr double HFOV_DEG = 45.0;
constexpr double CAMERA_H = 85.0;
constexpr double FOCAL_PX = (HALF_W/2.0) /
        std::tan((HFOV_DEG*M_PI/180.0)/2.0);

/* ── predictor & buffers ───────────────────────────────────────────── */
constexpr double LPF_ALPHA = 0.3;        // 1-pole height LPF
constexpr double VEL_LPF_ALPHA = 0.3;    // 1-pole velocity LPF
constexpr double G_CM      = 981.0;      // gravity in cm/s²
constexpr size_t BUF_MAX   = 4;
constexpr size_t BUF_MIN   = 2;
constexpr double RESET_GAP_S = 0.30;

/* =================================================================== */
class BallPredictorNode : public rclcpp::Node
{
public:
  BallPredictorNode():Node("ball_predictor_node")
  {
    img_sub_ = create_subscription<sensor_msgs::msg::Image>( // CORRECTED THIS LINE
      "/camera/image_raw",10,std::bind(&BallPredictorNode::cb_img,this,_1));
    pub_ = create_publisher<Point>("/ball_centroid",10);

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
  void cb_img(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    auto cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    const cv::Mat& img=cv_ptr->image;
    if(img.cols!=IMG_W||img.rows!=IMG_H) return;

    /* detect left / right */
    double lx,ly,rx,ry,aL=0,aR=0;
    bool fL=centroid(img(cv::Rect(0,0,HALF_W,IMG_H)), lx,ly,aL);
    bool fR=centroid(img(cv::Rect(HALF_W,0,HALF_W,IMG_H)), rx,ry,aR);

    /* reset buffer after gap ---------------------------------------- */
    if(!fL&&!fR){
      if(last_seen_.nanoseconds()!=0 &&
         (get_clock()->now()-last_seen_).seconds()>RESET_GAP_S)
        {
          buf_.clear();
          has_prev_v_ = false; // Reset velocity LPF state
        }
      return;
    }
    last_seen_=get_clock()->now();

    /* merge detections */
    double px,py,area;
    if(fL&&fR){
      px=(lx+rx)/2; py=(ly+ry)/2; area=(aL+aR)/2;
    }else if(fL){ px=lx; py=ly; area=aL; }
    else         { px=rx; py=ry; area=aR; }

    /* ---- height ---------------------------------------------------- */
    double h_area = std::max(0.0, H_SCALE*(std::sqrt(AREA_REF/area)-1.0));
    double disp = lx - rx;
    double dist=(BASELINE*FOCAL_PX)/disp;
    double h_st = CAMERA_H - dist;
    double h = h_st;
    double h_filt;
    
    /* LPF */
    if(has_prev_h_) h_filt = LPF_ALPHA*prev_h_ + (1.0-LPF_ALPHA)*h;
    prev_h_=h; has_prev_h_=true;

    /* ground-plane pixel */
    double corr=(CAMERA_H-h_filt)/CAMERA_H;
    double cx=IMG_CX+(px-IMG_CX)*corr;
    double cy=IMG_CY+(py-IMG_CY)*corr;

    /* store sample --------------------------------------------------- */
    double t = msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9;
    buf_.push_back({t,cx,cy,h_filt});
    if(buf_.size()>BUF_MAX) buf_.pop_front();
    if(buf_.size()<BUF_MIN) return;

    /* velocities ----------------------------------------------------- */
    const auto& q0=buf_[buf_.size()-2];
    const auto& q1=buf_.back();
    double dt=q1.t-q0.t; if(dt<1e-4) return;

    double vx_raw=(q1.x-q0.x)/dt;
    double vy_raw=(q1.y-q0.y)/dt;
    double vz_raw=(q1.h_filt-q0.h_filt)/dt;

    double vx, vy, vz;
    double vx_filt, vy_filt, vz_filt;
    
    if(has_prev_v_){
      vx_filt = VEL_LPF_ALPHA*prev_vx_ + (1.0-VEL_LPF_ALPHA)*vx_raw;
      vy_filt = VEL_LPF_ALPHA*prev_vy_ + (1.0-VEL_LPF_ALPHA)*vy_raw;
      vz_filt = VEL_LPF_ALPHA*prev_vz_ + (1.0-VEL_LPF_ALPHA)*vz_raw;
    } else {
      vx_filt = vx_raw; vy_filt = vy_raw; vz_filt = vz_raw;
      has_prev_v_ = true;
    }
    prev_vx_ = vx_raw; prev_vy_ = vy_raw; prev_vz_ = vz_raw;

    /* ---- gating: only when h>5 cm and STILL GOING UP -------------- */
    if(q1.h_filt <= 0.0 || q1.h_filt >= 30.0 || vz_filt <= -5.0){ // Note: vz is now filtered
      RCLCPP_INFO(get_logger(),
        "skip  h=%.1f vz=%.1f  (below gate)",q1.h_filt,vz_filt);
      return;
    }

    /* projectile solve  h_filt + vz·t -½g t² = 0 ------------------------- */
    double a=-0.5*G_CM, b=vz, c=q1.h_filt; // Note: vz is now filtered
    double disc=b*b-4*a*c; if(disc<0) return;
    double t_land=(-b - std::sqrt(disc))/(2*a);  // positive root
    if(t_land<=0) return;

    double pred_x=q1.x+vx*t_land, pred_y=q1.y+vy*t_land; // Note: vx, vy are filtered
    pred_x=std::clamp(pred_x,0.0,double(HALF_W-1));
    pred_y=std::clamp(pred_y,0.0,double(IMG_H-1));

    Point out; out.x=pred_x; out.y=pred_y; out.z=0.0;
    pub_->publish(out);

    /* continuous log (no throttle) ---------------------------------- */
    RCLCPP_INFO(get_logger(),
      "h0=%.1f  vx=%.1f vy=%.1f vz=%.1f  t=%.3f  \n (%.1f,%.1f) → (%.1f,%.1f)",
      q1.h_filt,vx_filt,vy_filt,vz_filt,t_land,q1.x,q1.y,out.x,out.y); // vx,vy,vz are filtered
  }

/* ---- data ----------------------------------------------------------- */
  struct Sample{ double t,x,y,h_filt; };
  std::deque<Sample> buf_;
  rclcpp::Time last_seen_;
  double prev_h_{0}; bool has_prev_h_{false};
  double prev_vx_{0}, prev_vy_{0}, prev_vz_{0};
  bool has_prev_v_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
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
