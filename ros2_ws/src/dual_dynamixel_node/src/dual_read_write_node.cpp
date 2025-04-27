#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "dual_dynamixel_node/msg/set_positions.hpp"
#include "dual_dynamixel_node/srv/get_positions.hpp"

using SetPositions = dual_dynamixel_node::msg::SetPositions;
using GetPositions = dual_dynamixel_node::srv::GetPositions;

/* ------------------------------------------------------------------ */
/*  Control-table addresses & named constants                         */
/* ------------------------------------------------------------------ */
constexpr uint8_t ADDR_OPERATING_MODE   = 11;
constexpr uint8_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint8_t ADDR_GOAL_POSITION    = 116;
constexpr uint8_t ADDR_PRESENT_POSITION = 132;

constexpr uint8_t OPERATING_MODE_POSITION = 3;   // value 3 = Position mode
constexpr uint8_t TORQUE_ON               = 1;
constexpr uint8_t TORQUE_OFF              = 0;

constexpr uint32_t INVALID_POSITION = 0x7fffffff;  // outside valid 0-4095

/* ------------------------------------------------------------------ */
class DualDriver : public rclcpp::Node
{
public:
  DualDriver() : Node("dual_read_write_node")
  {
    /* ---- parameters ------------------------------------------------ */
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 57600);
    declare_parameter<int>("id1", 1);
    declare_parameter<int>("id2", 2);

    port_name_ = get_parameter("port").as_string();
    baud_      = get_parameter("baudrate").as_int();
    ids_[0]    = static_cast<uint8_t>(get_parameter("id1").as_int());
    ids_[1]    = static_cast<uint8_t>(get_parameter("id2").as_int());

    RCLCPP_INFO(get_logger(),
                "Opening %s @ %d bps for IDs %u & %u",
                port_name_.c_str(), baud_, ids_[0], ids_[1]);

    open_and_configure_port();
    setup_servos();

    /* ---- ROS interface --------------------------------------------- */
    setpos_sub_ = create_subscription<SetPositions>(
        "set_positions", 10,
        [this](const SetPositions::SharedPtr msg){ on_set(msg); });

    getpos_srv_ = create_service<GetPositions>(
        "get_positions",
        [this](const std::shared_ptr<GetPositions::Request> req,
               std::shared_ptr<GetPositions::Response>       res)
        { on_get(req, res); });

    RCLCPP_INFO(get_logger(), "Dual Dynamixel node ready.");
  }

  ~DualDriver() override
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    int comm = packet_->write1ByteTxRx(port_, 0xFE,
                                       ADDR_TORQUE_ENABLE,
                                       TORQUE_OFF, &dxl_err_);
    if(comm != COMM_SUCCESS || dxl_err_ != 0)
      RCLCPP_ERROR(get_logger(),
                   "Failed to disable torque during shutdown: %s",
                   packet_->getTxRxResult(comm));

    port_->closePort();
  }

private:
  /* =======================  HW init helpers  ====================== */
  void open_and_configure_port()
  {
    port_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if(!port_->openPort())
      throw std::runtime_error("openPort() failed");
    if(!port_->setBaudRate(baud_))
      throw std::runtime_error("setBaudRate() failed");
  }

  void setup_servos()
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    for(auto id : ids_) {
      write_register(id, ADDR_OPERATING_MODE,
                     OPERATING_MODE_POSITION,
                     "operating-mode");
      write_register(id, ADDR_TORQUE_ENABLE, TORQUE_ON, "torque-enable");
    }
  }

  void write_register(uint8_t id, uint8_t addr,
                      uint8_t value, const char *label)
  {
    int comm = packet_->write1ByteTxRx(port_, id, addr, value, &dxl_err_);
    if(comm != COMM_SUCCESS || dxl_err_ != 0)
      RCLCPP_ERROR(get_logger(), "Set %s failed for ID %u: %s",
                   label, id, packet_->getTxRxResult(comm));
  }

  /* =======================  Callbacks  ============================ */
  void on_set(const SetPositions::SharedPtr msg)
  {
    if(msg->id.size()!=2 || msg->position.size()!=2){
      RCLCPP_ERROR(get_logger(), "Need exactly 2 ids & 2 positions");
      return;
    }

    std::lock_guard<std::mutex> lock(sdk_mutex_);
    dynamixel::GroupSyncWrite sync(port_, packet_,
                                   ADDR_GOAL_POSITION, 4);

    auto append_param = [&](uint8_t id, int32_t pos){
      uint32_t p = static_cast<uint32_t>(pos);
      uint8_t buf[4] = {uint8_t(p),
                        uint8_t(p>>8),
                        uint8_t(p>>16),
                        uint8_t(p>>24)};
      sync.addParam(id, buf);
    };

    append_param(msg->id[0], msg->position[0]);
    append_param(msg->id[1], msg->position[1]);

    int comm = sync.txPacket();
    if(comm != COMM_SUCCESS)
      RCLCPP_ERROR(get_logger(), "SyncWrite failed: %s",
                   packet_->getTxRxResult(comm));
  }

  void on_get(const std::shared_ptr<GetPositions::Request> req,
              std::shared_ptr<GetPositions::Response>      res)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    dynamixel::GroupSyncRead sync(port_, packet_,
                                  ADDR_PRESENT_POSITION, 4);

    for(auto id : req->id) sync.addParam(id);

    int comm = sync.txRxPacket();
    if(comm != COMM_SUCCESS){
      RCLCPP_ERROR(get_logger(), "SyncRead comm error: %s",
                   packet_->getTxRxResult(comm));
      res->position.assign(req->id.size(), INVALID_POSITION);
      return;
    }

    res->position.resize(req->id.size());
    for(size_t i=0;i<req->id.size();++i){
      auto id = req->id[i];
      if(sync.isAvailable(id, ADDR_PRESENT_POSITION, 4))
        res->position[i] =
          int32_t(sync.getData(id, ADDR_PRESENT_POSITION, 4));
      else
        res->position[i] = INVALID_POSITION;
    }
  }

  /* =======================  members  ============================== */
  std::string port_name_;
  int         baud_;
  uint8_t     ids_[2];
  uint8_t     dxl_err_{0};

  dynamixel::PortHandler  *port_{nullptr};
  dynamixel::PacketHandler*packet_{nullptr};

  std::mutex sdk_mutex_;   // guarantee single SDK access

  rclcpp::Subscription<SetPositions>::SharedPtr setpos_sub_;
  rclcpp::Service<GetPositions>::SharedPtr      getpos_srv_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try{
    rclcpp::spin(std::make_shared<DualDriver>());
  }catch(const std::exception& e){
    RCLCPP_FATAL(rclcpp::get_logger("dual_read_write_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}


