/* ====================================================================
 * ball_predictor_node.cpp – stereo blob tracker + parabolic landing
 * --------------------------------------------------------------------
 *  • subscribes   /camera/image_raw   (640×240 stereo image)
 *  • publishes    /ball_centroid      (geometry_msgs/Point, px coords)
 * ==================================================================== */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <deque>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

/* ── shared vision constants (duplicate of detector.cpp – factor out later) */
namespace vp {
constexpr int    IMG_W = 640, IMG_H = 240;
constexpr int    HALF_W = IMG_W / 2;
constexpr double IMG_CX = HALF_W / 2.0;
constexpr double IMG_CY = IMG_H  / 2.0;

constexpr int    THRESH    = 225;
constexpr int    ERODE_SZ  = 3, DILATE_SZ = 7;
constexpr double MIN_AREA  = 15.0 * 15.0;

constexpr double HFOV_DEG  = 45.0;
constexpr double CAMERA_H  = 85.0;          // camera height above table [cm]
constexpr double BASELINE  = 5.5;           // stereo baseline [cm]
constexpr double FOCAL_PX  = (HALF_W/2.0) /
        std::tan((HFOV_DEG*M_PI/180.0)/2.0);

/* Physics & filter */
constexpr double G_CM      = 981.0;         // gravity [cm·s⁻²]
constexpr double LPF_ALPHA = 0.8;           // pos low-pass
constexpr double VEL_ALPHA = 0.8;           // vel low-pass
constexpr double PRED_ALPHA= 0.8;           // landing-point LPF
constexpr size_t  BUF_MAX  = 4;
constexpr size_t  BUF_MIN  = 2;
constexpr double  RESET_GAP_S = 0.30;
} // namespace vp

/* ---- centroid helper (identical to detector.cpp) ------------------ */
static bool centroid(const cv::Mat& roi,
                     double& cx,double& cy,double& area,
                     const cv::Mat& erodeK,const cv::Mat& dilateK)
{
  cv::Mat g,m; cv::cvtColor(roi,g,cv::COLOR_BGR2GRAY);
  cv::threshold(g,m,vp::THRESH,255,cv::THRESH_BINARY);
  cv::erode(m,m,erodeK); cv::dilate(m,m,dilateK);

  std::vector<std::vector<cv::Point>> cont;
  cv::findContours(m,cont,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  if(cont.empty()) return false;
  auto it = std::max_element(cont.begin(),cont.end(),
        [](auto&a,auto&b){ return cv::contourArea(a)<cv::contourArea(b); });
  area = cv::contourArea(*it);
  if(area < vp::MIN_AREA) return false;
  cv::Moments mom = cv::moments(*it);
  cx = mom.m10 / mom.m00;
  cy = mom.m01 / mom.m00;
  return true;
}

/* =================================================================== */
class BallPredictorNode : public rclcpp::Node
{
public:
  BallPredictorNode():Node("ball_predictor_node")
  {
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",10,std::bind(&BallPredictorNode::cb_img,this,_1));
    pub_=create_publisher<geometry_msgs::msg::Point>("/ball_centroid",10);

    erodeK_=cv::getStructuringElement(cv::MORPH_ELLIPSE,{vp::ERODE_SZ,vp::ERODE_SZ});
    dilateK_=cv::getStructuringElement(cv::MORPH_ELLIPSE,{vp::DILATE_SZ,vp::DILATE_SZ});
    RCLCPP_INFO(get_logger(),"Predictor ready (g=%.1f cm/s²)",vp::G_CM);
  }

private:
/* ---- helper: compute height from area & disparity ----------------- */
  double height_cm(double area,double disparity) const
  {
    const double AREA_REF = 340.0, H_SCALE = 120.0;
    double h_area = std::max(0.0, H_SCALE*(std::sqrt(AREA_REF/area)-1.0));
    if(std::abs(disparity) < 1e-2) return h_area; // avoid div-by-zero
    double dist = (vp::BASELINE*vp::FOCAL_PX)/disparity;
    double h_st = vp::CAMERA_H - dist;
    return 0.6*h_st + 0.4*h_area;                // cheap fuse
  }

/* ---- main callback ------------------------------------------------ */
  void cb_img(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    /* ··· basic checks ··· */
    auto cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    const cv::Mat& img = cv_ptr->image;
    if(img.empty()) return;
    if(img.cols!=vp::IMG_W || img.rows!=vp::IMG_H){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
                           "Unexpected image size %dx%d",img.cols,img.rows);
      return;
    }

    /* ··· extract centroids from left/right eye ··· */
    cv::Rect roiL(0,0,vp::HALF_W,vp::IMG_H), roiR(vp::HALF_W,0,vp::HALF_W,vp::IMG_H);
    double lx,ly,aL=0; bool fL = centroid(img(roiL),lx,ly,aL,erodeK_,dilateK_);
    double rx,ry,aR=0; bool fR = centroid(img(roiR),rx,ry,aR,erodeK_,dilateK_);

    if(!(fL||fR)){   // nothing
      if((get_clock()->now()-last_seen_).seconds()>vp::RESET_GAP_S){
        buf_.clear(); has_prev_h_=has_prev_v_=has_pred_=false;
      }
      return;
    }
    last_seen_ = get_clock()->now();

    /* choose centroid */
    double px,py,area;
    if(fL&&fR){ px=(lx+rx)/2; py=(ly+ry)/2; area=(aL+aR)/2; }
    else if(fL){ px=lx; py=ly; area=aL; }
    else        { px=rx; py=ry; area=aR; }

    double disp = (fL&&fR)? lx-rx : 0.0;

    /* height LPF */
    double h_raw = height_cm(area,disp);
    if(has_prev_h_) h_raw = vp::LPF_ALPHA*prev_h_ + (1.0-vp::LPF_ALPHA)*h_raw;
    prev_h_=h_raw; has_prev_h_=true;

    /* perspective correct */
    double corr = (vp::CAMERA_H-h_raw)/vp::CAMERA_H;
    px = vp::IMG_CX + (px-vp::IMG_CX)*corr;
    py = vp::IMG_CY + (py-vp::IMG_CY)*corr;

    /* buffer sample */
    if(buf_.size()==vp::BUF_MAX) buf_.pop_front();
    buf_.push_back({px,py,h_raw,get_clock()->now().seconds()});

    if(buf_.size()<vp::BUF_MIN) return;

    /* planar velocity */
    const auto& q0=buf_[buf_.size()-2];
    const auto& q1=buf_.back();
    double dt = q1.t - q0.t;
    if(dt<1e-3) return;      // avoid div0
    double vx=(q1.x - q0.x)/dt, vy=(q1.y - q0.y)/dt;
    if(has_prev_v_){
      vx = vp::VEL_ALPHA*prev_vx_ + (1.0-vp::VEL_ALPHA)*vx;
      vy = vp::VEL_ALPHA*prev_vy_ + (1.0-vp::VEL_ALPHA)*vy;
    }
    prev_vx_=vx; prev_vy_=vy; has_prev_v_=true;

    /* time until ground (vertical parabola) */
    double vz   = (q1.h - q0.h) / dt;
    double h_cl = std::max(0.0, q1.h);              // ← 1. clamp height
    double disc = vz*vz + 2*vp::G_CM*h_cl;          // ← 2. safe radicand
    if (disc <= 0.0) return;                        // ← 3. give up this frame

    double t_land = (-vz - std::sqrt(disc)) / -vp::G_CM;

    double px_land = px + vx*t_land;
    double py_land = py + vy*t_land;

    if(has_pred_){
      px_land = vp::PRED_ALPHA*prev_pred_x_ + (1.0-vp::PRED_ALPHA)*px_land;
      py_land = vp::PRED_ALPHA*prev_pred_y_ + (1.0-vp::PRED_ALPHA)*py_land;
    }
    prev_pred_x_=px_land; prev_pred_y_=py_land; has_pred_=true;
    
    px_land = std::clamp(px_land, 0.0, static_cast<double>(vp::HALF_W  - 1)); // 0 … 319
    py_land = std::clamp(py_land, 0.0, static_cast<double>(vp::IMG_H   - 1)); // 0 … 239


    /* publish */
    geometry_msgs::msg::Point out;
    out.x=px_land; out.y=py_land; out.z=h_raw;
    pub_->publish(out);
  }

/* ---- sample struct & state ---------------------------------------- */
  struct Sample{ double x,y,h,t; };
  std::deque<Sample> buf_;

  rclcpp::Time last_seen_{0,0,RCL_ROS_TIME};
  double prev_h_{0}; bool has_prev_h_{false};
  double prev_vx_{0}, prev_vy_{0}; bool has_prev_v_{false};
  double prev_pred_x_{0}, prev_pred_y_{0}; bool has_pred_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
  cv::Mat erodeK_, dilateK_;
};

/* ---- main --------------------------------------------------------- */
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<BallPredictorNode>());
  rclcpp::shutdown();
  return 0;
}

