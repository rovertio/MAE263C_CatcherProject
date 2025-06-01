/* ====================================================================
 * detector.cpp – simple bright-blob centroid detector (legacy)
 * --------------------------------------------------------------------
 *  • subscribes   /camera/image_raw   (640×240 stereo image)
 *  • publishes    /ball_centroid      (geometry_msgs/Point, px coords)
 *  • keeps no physics model – just centroiding + optional 1-step velocity
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

/* ── shared vision constants (move to a header later) ──────────────── */
namespace vp {
constexpr int    IMG_W = 640, IMG_H = 240;
constexpr int    HALF_W = IMG_W / 2;                 // width of each eye
constexpr double IMG_CX = HALF_W / 2.0;
constexpr double IMG_CY = IMG_H  / 2.0;

constexpr int    THRESH    = 225;                    // binarisation threshold
constexpr int    ERODE_SZ  = 3, DILATE_SZ = 7;       // morphology kernel sizes
constexpr double MIN_AREA  = 15.0 * 15.0;            // ignore tiny blobs

/* Camera geometry */
constexpr double HFOV_DEG  = 45.0;
constexpr double CAMERA_H  = 85.0;                   // cam height [cm]
constexpr double BASELINE  = 5.5;                    // stereo baseline [cm]
constexpr double FOCAL_PX  = (HALF_W/2.0) /
        std::tan((HFOV_DEG * M_PI / 180.0)/2.0);

/* Simple 1-step motion prediction */
constexpr size_t HISTORY   = 2;      // frames kept for velocity estimate
constexpr double LOOKAHEAD = 0.0;    // how many frames ahead to predict
} // namespace vp

/* Helper – compute centroid (cx,cy,area) of the largest white blob */
static bool centroid(const cv::Mat& roi,
                     double& cx, double& cy, double& area,
                     const cv::Mat& erodeK, const cv::Mat& dilateK)
{
  cv::Mat g,m; cv::cvtColor(roi,g,cv::COLOR_BGR2GRAY);
  cv::threshold(g,m,vp::THRESH,255,cv::THRESH_BINARY);
  cv::erode(m,m,erodeK); cv::dilate(m,m,dilateK);

  std::vector<std::vector<cv::Point>> cont;
  cv::findContours(m,cont,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  if(cont.empty()) return false;

  auto it = std::max_element(cont.begin(),cont.end(),
        [](auto&a,auto&b){ return cv::contourArea(a) < cv::contourArea(b); });
  area = cv::contourArea(*it);
  if(area < vp::MIN_AREA) return false;

  cv::Moments mom = cv::moments(*it);
  cx = mom.m10 / mom.m00;
  cy = mom.m01 / mom.m00;
  return true;
}

/* =================================================================== */
class BallCentroidDetector : public rclcpp::Node
{
public:
  BallCentroidDetector():Node("ball_centroid_detector")
  {
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",10,std::bind(&BallCentroidDetector::cb_img,this,_1));
    pub_ = create_publisher<geometry_msgs::msg::Point>("/ball_centroid",10);

    erodeK_  = cv::getStructuringElement(cv::MORPH_ELLIPSE,{vp::ERODE_SZ,vp::ERODE_SZ});
    dilateK_ = cv::getStructuringElement(cv::MORPH_ELLIPSE,{vp::DILATE_SZ,vp::DILATE_SZ});

    RCLCPP_INFO(get_logger(),
      "Centroid detector ready. IMG %dx%d – thresh=%d",vp::IMG_W,vp::IMG_H,vp::THRESH);
  }

private:
/* ---- image callback ------------------------------------------------ */
  void cb_img(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    auto cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    const cv::Mat& img = cv_ptr->image;
    if(img.empty()) return;
    if(img.cols!=vp::IMG_W || img.rows!=vp::IMG_H){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
                           "Unexpected image size %dx%d",img.cols,img.rows);
      return;
    }

    /* -- split stereo image ----------------------------------------- */
    cv::Rect roiL(0,0,vp::HALF_W,vp::IMG_H), roiR(vp::HALF_W,0,vp::HALF_W,vp::IMG_H);
    double lx,ly,rx,ry,aL=0,aR=0;
    bool fL = centroid(img(roiL),lx,ly,aL,erodeK_,dilateK_);
    bool fR = centroid(img(roiR),rx,ry,aR,erodeK_,dilateK_);

    /* -- choose best observation ------------------------------------ */
    double px=0,py=0,area=0;
    if(fL&&fR){ px=(lx+rx)/2; py=(ly+ry)/2; area=(aL+aR)/2; }
    else if(fL){ px=lx; py=ly; area=aL; }
    else if(fR){ px=rx; py=ry; area=aR; }
    else       { return; }                  // nothing found

    /* -- naive velocity & lookahead --------------------------------- */
    if(buf_.size()==vp::HISTORY) buf_.pop_front();
    buf_.emplace_back(px,py);

    double px_pred=px, py_pred=py;
    if(buf_.size()==vp::HISTORY){
      const auto& p0=buf_[0], &p1=buf_[1];
      px_pred = p1.x + (p1.x - p0.x) * vp::LOOKAHEAD;
      py_pred = p1.y + (p1.y - p0.y) * vp::LOOKAHEAD;
    }

    /* -- publish ----------------------------------------------------- */
    geometry_msgs::msg::Point out;
    out.x = px_pred;
    out.y = py_pred;
    out.z = area;               // encode area if downstream needs it
    pub_->publish(out);
  }

/* ---- members ------------------------------------------------------- */
  std::deque<cv::Point2d> buf_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
  cv::Mat erodeK_, dilateK_;
};

/* ---- main ---------------------------------------------------------- */
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<BallCentroidDetector>());
  rclcpp::shutdown();
  return 0;
}

