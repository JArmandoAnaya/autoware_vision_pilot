#include <vehicle_state_subscriber/ros2_to_can.hpp>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <stdexcept>

namespace vp_middleware {

// ── helpers ───────────────────────────────────────────────────────────────────

static uint64_t now_ns() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}

// ── constructor ───────────────────────────────────────────────────────────────

VehicleStateSubscriber::VehicleStateSubscriber(
    const std::string& speed_topic,
    const std::string& steering_topic,
    const std::string& node_name)
{
    // Init ROS2 once (shared with camera_subscriber if present)
    if (!rclcpp::ok()) {
        static int    argc   = 1;
        static const char* fake_argv[] = {"vehicle_state_subscriber", nullptr};
        rclcpp::init(argc, const_cast<char**>(fake_argv));
    }

    node_ = std::make_shared<rclcpp::Node>(node_name);
    RCLCPP_INFO(node_->get_logger(), "VehicleStateSubscriber: speed=%s  steering=%s",
                speed_topic.c_str(), steering_topic.c_str());

    // Best-effort, depth=1 — we only care about the latest value
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    speed_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        speed_topic, qos,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { speed_callback(msg); });

    steering_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        steering_topic, qos,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { steering_callback(msg); });

    open_shm();
    RCLCPP_INFO(node_->get_logger(), "VehicleStateSubscriber ready  (shmem: %s)", VP_STATE_SHM_NAME);

    spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
}

// ── POSIX shmem ───────────────────────────────────────────────────────────────

void VehicleStateSubscriber::open_shm() {
    shm_fd_ = shm_open(VP_STATE_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) {
        RCLCPP_WARN(node_->get_logger(), "shm_open(%s) failed — shmem disabled", VP_STATE_SHM_NAME);
        return;
    }
    if (ftruncate(shm_fd_, sizeof(VehicleStateShmLayout)) < 0) {
        RCLCPP_WARN(node_->get_logger(), "ftruncate failed — shmem disabled");
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
    shm_ptr_ = mmap(nullptr, sizeof(VehicleStateShmLayout),
                    PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (shm_ptr_ == MAP_FAILED) {
        RCLCPP_WARN(node_->get_logger(), "mmap failed — shmem disabled");
        close(shm_fd_);
        shm_fd_  = -1;
        shm_ptr_ = nullptr;
    }
}

void VehicleStateSubscriber::write_to_shm(const VehicleState& s) {
    if (!shm_ptr_) return;
    auto* layout = reinterpret_cast<VehicleStateShmLayout*>(shm_ptr_);

    // Seqlock write: odd epoch → data → even epoch
    __atomic_fetch_add(&layout->epoch, 1u, __ATOMIC_RELEASE);
    layout->speed_ms    = s.speed_ms;
    layout->steering_rad = s.steering_rad;
    layout->timestamp_ns = now_ns();
    __atomic_fetch_add(&layout->epoch, 1u, __ATOMIC_RELEASE);
}

// ── callbacks ─────────────────────────────────────────────────────────────────

void VehicleStateSubscriber::speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    VehicleState updated;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_.speed_ms = msg->data;
        got_speed_      = true;
        updated         = state_;
    }
    write_to_shm(updated);
    RCLCPP_DEBUG(node_->get_logger(), "speed=%.3f m/s", msg->data);
}

void VehicleStateSubscriber::steering_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    VehicleState updated;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_.steering_rad = msg->data;
        got_steering_       = true;
        updated             = state_;
    }
    write_to_shm(updated);
    RCLCPP_DEBUG(node_->get_logger(), "steering=%.4f rad", msg->data);
}

// ── public API ────────────────────────────────────────────────────────────────

VehicleState VehicleStateSubscriber::get_state() const {
    std::lock_guard<std::mutex> lk(state_mutex_);
    return state_;
}

bool VehicleStateSubscriber::is_valid() const {
    std::lock_guard<std::mutex> lk(state_mutex_);
    return got_speed_ && got_steering_;
}

// ── destructor ────────────────────────────────────────────────────────────────

VehicleStateSubscriber::~VehicleStateSubscriber() {
    if (shm_ptr_) munmap(shm_ptr_, sizeof(VehicleStateShmLayout));
    if (shm_fd_ >= 0) close(shm_fd_);
    if (spin_thread_.joinable()) spin_thread_.join();
}

}  // namespace vp_middleware
