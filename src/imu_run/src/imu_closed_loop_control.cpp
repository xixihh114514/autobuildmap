#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <math.h>
#include <string>
#include <vector>
#include <atomic>
#include <errno.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#ifdef __linux__
#include <linux/serial.h>
#endif

#define MOTOR_PULSES_PER_REVOLUTION 65536.0
#define MOTOR_SECONDS_PER_MINUTE 60.0
#define DEGREES_PER_REVOLUTION 360.0

#define ROLL_MOTOR_ID  0x16
#define PITCH_MOTOR_ID 0x15

#define SERVO_CMD_WRITE         0x01
#define SERVO_ADDR_TARGET_SPEED 0x09
#define SERVO_ADDR_WORK_MODE    0x0F
#define SERVO_ADDR_ENABLE       0x10

struct Controller {
    std::atomic<double> roll;
    std::atomic<double> pitch;
    std::atomic<double> imu_timestamp_sec;
    std::atomic<int> imu_packets;
    std::atomic<int> imu_parse_errors;
    std::atomic<bool> running;
    std::atomic<bool> imu_ready;
    int can_sock;

    Controller()
        : roll(0.0),
          pitch(0.0),
          imu_timestamp_sec(0.0),
          imu_packets(0),
          imu_parse_errors(0),
          running(true),
          imu_ready(false),
          can_sock(-1) {}
};

struct RuntimeOptions {
    std::string can_interface;
    std::string imu_device;
    int imu_baud;

    double control_hz;
    double print_hz;
    double imu_timeout_sec;
    double imu_warmup_sec;

    int main_thread_priority;
    int imu_thread_priority;
    int main_thread_cpu;
    int imu_thread_cpu;

    bool try_realtime;
    bool try_mlockall;
    bool try_serial_low_latency;

    bool switch_to_speed_mode_on_startup;
    int speed_mode_code;
    bool enable_on_startup;
    bool disable_on_shutdown;
    bool zero_speed_on_shutdown;

    bool always_send;
    double min_send_speed_delta_rpm;
};

struct AxisConfig {
    // 平台角度低通
    double imu_angle_lpf_alpha;

    // 平台角速度低通
    double rate_lpf_hz;

    // 差分求角速度时的尖峰限幅（平台角速度）
    double max_platform_rate_deg_s;

    // 纯速度补偿增益：rpm / (deg/s)
    double rate_ff_rpm_per_deg_s;

    // 方向修正
    double motor_dir;

    // 平台静止零偏
    double level_offset_deg;

    // 最小 / 最大速度
    double min_speed_rpm;
    double max_speed_rpm;

    // 速度斜率限制
    double speed_slew_rpm_per_s;

    // 虚拟角度软限位（靠积分估计，防 runaway）
    double max_virtual_angle_deg;

    // 当平台角速度很小，允许小回中（防慢漂）
    double quiet_rate_deg_s;
    double center_return_k_rpm_per_deg;
};

struct AxisRuntime {
    bool angle_initialized;
    bool rate_initialized;

    double filtered_angle_deg;
    double prev_filtered_angle_deg;
    double filtered_rate_deg_s;

    double speed_cmd_rpm;
    double last_sent_speed_cmd_rpm;

    // 由“已发送速度命令”积分出的虚拟云台角度估计
    double virtual_gimbal_angle_deg;
};

struct ImuThreadArgs {
    Controller *ctrl;
    RuntimeOptions *opts;
};

static inline double monotonic_time_sec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

static inline uint64_t monotonic_time_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

static double clamp_value(double value, double min_value, double max_value) {
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static double approach_value(double current, double target, double max_step) {
    if (max_step <= 0.0) return target;
    double delta = target - current;
    if (delta > max_step) return current + max_step;
    if (delta < -max_step) return current - max_step;
    return target;
}

static double cutoff_to_alpha(double cutoff_hz, double dt) {
    if (cutoff_hz <= 0.0) return 1.0;
    const double pi = 3.14159265358979323846;
    double tau = 1.0 / (2.0 * pi * cutoff_hz);
    return dt / (tau + dt);
}

static int try_set_realtime_for_current_thread(int policy, int priority, int cpu_core) {
    if (cpu_core >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_core, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            return -1;
        }
    }

    struct sched_param sch;
    memset(&sch, 0, sizeof(sch));
    sch.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), policy, &sch) != 0) {
        return -2;
    }
    return 0;
}

static void try_enable_process_realtime(const RuntimeOptions &opts) {
    if (opts.try_mlockall) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            ROS_WARN("mlockall failed: %s", strerror(errno));
        }
    }

    if (opts.try_realtime) {
        int ret = try_set_realtime_for_current_thread(SCHED_FIFO,
                                                      opts.main_thread_priority,
                                                      opts.main_thread_cpu);
        if (ret != 0) {
            ROS_WARN("Failed to set main thread realtime scheduling (ret=%d, errno=%d: %s)",
                     ret, errno, strerror(errno));
        } else {
            ROS_INFO("Main thread set to SCHED_FIFO priority=%d cpu=%d",
                     opts.main_thread_priority, opts.main_thread_cpu);
        }
    }
}

static bool try_set_serial_low_latency(int fd) {
#ifdef __linux__
    struct serial_struct ser;
    if (ioctl(fd, TIOCGSERIAL, &ser) != 0) {
        return false;
    }
    ser.flags |= ASYNC_LOW_LATENCY;
    if (ioctl(fd, TIOCSSERIAL, &ser) != 0) {
        return false;
    }
    return true;
#else
    (void)fd;
    return false;
#endif
}

static void send_can_frame_fast(int sock, uint32_t can_id, uint8_t cmd, uint8_t addr, int32_t value) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = 8;
    frame.data[0] = cmd;
    frame.data[1] = addr;
    frame.data[2] = static_cast<uint8_t>((value >> 24) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((value >> 16) & 0xFF);
    frame.data[4] = static_cast<uint8_t>((value >> 8) & 0xFF);
    frame.data[5] = static_cast<uint8_t>(value & 0xFF);

    ssize_t ret = write(sock, &frame, sizeof(frame));
    (void)ret;
}

static int32_t rpm_to_pulses_per_second(double rpm) {
    return static_cast<int32_t>(rpm * MOTOR_PULSES_PER_REVOLUTION / MOTOR_SECONDS_PER_MINUTE);
}

static void set_motor_mode(Controller *c, int id, uint32_t mode) {
    send_can_frame_fast(c->can_sock, id, SERVO_CMD_WRITE, SERVO_ADDR_WORK_MODE, static_cast<int32_t>(mode));
}

static void set_motor_enable(Controller *c, int id, int enabled) {
    send_can_frame_fast(c->can_sock, id, SERVO_CMD_WRITE, SERVO_ADDR_ENABLE, enabled ? 1 : 0);
}

static void set_motor_speed(Controller *c, int id, double rpm) {
    send_can_frame_fast(c->can_sock, id, SERVO_CMD_WRITE, SERVO_ADDR_TARGET_SPEED, rpm_to_pulses_per_second(rpm));
}

static void stop_all_motors(Controller *c) {
    set_motor_speed(c, ROLL_MOTOR_ID, 0.0);
    usleep(1000);
    set_motor_speed(c, PITCH_MOTOR_ID, 0.0);
    usleep(1000);
}

static void disable_all_motors(Controller *c) {
    set_motor_enable(c, ROLL_MOTOR_ID, 0);
    usleep(2000);
    set_motor_enable(c, PITCH_MOTOR_ID, 0);
    usleep(2000);
}

static void enable_all_motors(Controller *c) {
    set_motor_enable(c, ROLL_MOTOR_ID, 1);
    usleep(2000);
    set_motor_enable(c, PITCH_MOTOR_ID, 1);
    usleep(2000);
}

static void switch_all_motors_to_speed_mode(Controller *c, int speed_mode_code) {
    set_motor_mode(c, ROLL_MOTOR_ID, static_cast<uint32_t>(speed_mode_code));
    usleep(5000);
    set_motor_mode(c, PITCH_MOTOR_ID, static_cast<uint32_t>(speed_mode_code));
    usleep(5000);
}

static bool extract_imu_packet(const std::vector<uint8_t> &buf, size_t start_idx, float &roll, float &pitch) {
    if (start_idx + 6 > buf.size()) return false;

    uint16_t payload_len = static_cast<uint16_t>(buf[start_idx + 2]) |
                           (static_cast<uint16_t>(buf[start_idx + 3]) << 8);
    size_t total_len = 6 + payload_len;
    if (start_idx + total_len > buf.size()) return false;

    // 保持和你原始程序一致
    const size_t roll_offset = start_idx + 6 + 48;
    const size_t pitch_offset = roll_offset + sizeof(float);
    if (pitch_offset + sizeof(float) > start_idx + total_len) return false;

    memcpy(&roll, &buf[roll_offset], sizeof(float));
    memcpy(&pitch, &buf[pitch_offset], sizeof(float));
    return true;
}

static void *imu_thread_func(void *arg) {
    ImuThreadArgs *thread_args = reinterpret_cast<ImuThreadArgs*>(arg);
    Controller *c = thread_args->ctrl;
    RuntimeOptions *opts = thread_args->opts;

    if (opts->try_realtime) {
        int ret = try_set_realtime_for_current_thread(SCHED_FIFO,
                                                      opts->imu_thread_priority,
                                                      opts->imu_thread_cpu);
        if (ret != 0) {
            ROS_WARN("Failed to set IMU thread realtime scheduling (ret=%d, errno=%d: %s)",
                     ret, errno, strerror(errno));
        } else {
            ROS_INFO("IMU thread set to SCHED_FIFO priority=%d cpu=%d",
                     opts->imu_thread_priority, opts->imu_thread_cpu);
        }
    }

    int fd = open(opts->imu_device.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        ROS_ERROR("IMU open failed: %s", strerror(errno));
        return NULL;
    }

    struct termios opt;
    memset(&opt, 0, sizeof(opt));
    if (tcgetattr(fd, &opt) != 0) {
        ROS_ERROR("tcgetattr failed: %s", strerror(errno));
        close(fd);
        return NULL;
    }

    speed_t baud = B115200;
    switch (opts->imu_baud) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
#ifdef B230400
        case 230400: baud = B230400; break;
#endif
#ifdef B460800
        case 460800: baud = B460800; break;
#endif
        default: baud = B115200; break;
    }

    cfsetispeed(&opt, baud);
    cfsetospeed(&opt, baud);
    cfmakeraw(&opt);
    opt.c_cflag |= (CLOCAL | CREAD);
    opt.c_cflag &= ~CRTSCTS;
    opt.c_cc[VMIN] = 1;
    opt.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &opt) != 0) {
        ROS_ERROR("tcsetattr failed: %s", strerror(errno));
        close(fd);
        return NULL;
    }

    if (opts->try_serial_low_latency) {
        bool ok = try_set_serial_low_latency(fd);
        ROS_INFO("Serial low latency request: %s", ok ? "success" : "failed / unsupported");
    }

    std::vector<uint8_t> stream_buf;
    stream_buf.reserve(8192);
    uint8_t temp[1024];

    while (c->running.load(std::memory_order_relaxed)) {
        ssize_t n = read(fd, temp, sizeof(temp));
        if (n < 0) {
            if (errno == EINTR) continue;
            ROS_ERROR_THROTTLE(1.0, "IMU read failed: %s", strerror(errno));
            continue;
        }
        if (n == 0) continue;

        stream_buf.insert(stream_buf.end(), temp, temp + n);

        size_t scan = 0;
        while (scan + 6 <= stream_buf.size()) {
            if (stream_buf[scan] != 0x5A || stream_buf[scan + 1] != 0xA5) {
                ++scan;
                continue;
            }

            uint16_t payload_len = static_cast<uint16_t>(stream_buf[scan + 2]) |
                                   (static_cast<uint16_t>(stream_buf[scan + 3]) << 8);
            size_t total_len = 6 + payload_len;
            if (total_len < 6 || total_len > 2048) {
                c->imu_parse_errors.fetch_add(1, std::memory_order_relaxed);
                ++scan;
                continue;
            }

            if (scan + total_len > stream_buf.size()) {
                break;
            }

            float roll = 0.0f;
            float pitch = 0.0f;
            if (extract_imu_packet(stream_buf, scan, roll, pitch)) {
                c->roll.store(static_cast<double>(roll), std::memory_order_relaxed);
                c->pitch.store(static_cast<double>(pitch), std::memory_order_relaxed);
                c->imu_timestamp_sec.store(monotonic_time_sec(), std::memory_order_relaxed);
                c->imu_packets.fetch_add(1, std::memory_order_relaxed);
                c->imu_ready.store(true, std::memory_order_relaxed);
            } else {
                c->imu_parse_errors.fetch_add(1, std::memory_order_relaxed);
            }
            scan += total_len;
        }

        if (scan > 0) {
            stream_buf.erase(stream_buf.begin(), stream_buf.begin() + static_cast<long>(scan));
        }
        if (stream_buf.size() > 8192) {
            stream_buf.erase(stream_buf.begin(), stream_buf.end() - 2048);
        }
    }

    close(fd);
    return NULL;
}

static void reset_axis_runtime(AxisRuntime *rt) {
    rt->angle_initialized = false;
    rt->rate_initialized = false;
    rt->filtered_angle_deg = 0.0;
    rt->prev_filtered_angle_deg = 0.0;
    rt->filtered_rate_deg_s = 0.0;
    rt->speed_cmd_rpm = 0.0;
    rt->last_sent_speed_cmd_rpm = 0.0;
    rt->virtual_gimbal_angle_deg = 0.0;
}

static void update_axis_runtime_feedforward(const AxisConfig *cfg,
                                            AxisRuntime *rt,
                                            double platform_angle_deg,
                                            double dt,
                                            double *platform_angle_used_deg,
                                            double *platform_rate_used_deg_s,
                                            double *desired_speed_before_limit_rpm)
{
    if (dt <= 1e-6) {
        dt = 1.0 / 200.0;
    }

    double corrected_angle = platform_angle_deg - cfg->level_offset_deg;

    if (!rt->angle_initialized) {
        rt->filtered_angle_deg = corrected_angle;
        rt->prev_filtered_angle_deg = corrected_angle;
        rt->angle_initialized = true;
    } else {
        rt->filtered_angle_deg += cfg->imu_angle_lpf_alpha * (corrected_angle - rt->filtered_angle_deg);
    }

    double raw_rate_deg_s = (rt->filtered_angle_deg - rt->prev_filtered_angle_deg) / dt;
    raw_rate_deg_s = clamp_value(raw_rate_deg_s,
                                 -cfg->max_platform_rate_deg_s,
                                 cfg->max_platform_rate_deg_s);

    if (!rt->rate_initialized) {
        rt->filtered_rate_deg_s = raw_rate_deg_s;
        rt->rate_initialized = true;
    } else {
        double alpha = cutoff_to_alpha(cfg->rate_lpf_hz, dt);
        rt->filtered_rate_deg_s += alpha * (raw_rate_deg_s - rt->filtered_rate_deg_s);
    }

    rt->prev_filtered_angle_deg = rt->filtered_angle_deg;

    // 核心：平台角速度前馈 -> 云台反向速度补偿
    double desired_speed_rpm = -cfg->motor_dir * cfg->rate_ff_rpm_per_deg_s * rt->filtered_rate_deg_s;

    // 平台安静时，允许很弱的回中，防止长时间累计漂移
    if (fabs(rt->filtered_rate_deg_s) < cfg->quiet_rate_deg_s && cfg->center_return_k_rpm_per_deg > 0.0) {
        desired_speed_rpm += -cfg->motor_dir * cfg->center_return_k_rpm_per_deg * rt->virtual_gimbal_angle_deg;
    }

    *desired_speed_before_limit_rpm = desired_speed_rpm;

    // 软限位：如果虚拟角度已经接近边界，就只允许往回走
    if (rt->virtual_gimbal_angle_deg >= cfg->max_virtual_angle_deg && desired_speed_rpm > 0.0) {
        desired_speed_rpm = 0.0;
    }
    if (rt->virtual_gimbal_angle_deg <= -cfg->max_virtual_angle_deg && desired_speed_rpm < 0.0) {
        desired_speed_rpm = 0.0;
    }

    // 小速度死区 / 静摩擦补偿
    if (fabs(desired_speed_rpm) > 1e-6 && fabs(desired_speed_rpm) < cfg->min_speed_rpm) {
        desired_speed_rpm = (desired_speed_rpm > 0.0 ? 1.0 : -1.0) * cfg->min_speed_rpm;
    }

    desired_speed_rpm = clamp_value(desired_speed_rpm, -cfg->max_speed_rpm, cfg->max_speed_rpm);

    double max_step = cfg->speed_slew_rpm_per_s * dt;
    rt->speed_cmd_rpm = approach_value(rt->speed_cmd_rpm, desired_speed_rpm, max_step);

    // 由已发/将发速度估计虚拟角度，1 rpm = 6 deg/s
    rt->virtual_gimbal_angle_deg += rt->speed_cmd_rpm * 6.0 * dt;
    rt->virtual_gimbal_angle_deg = clamp_value(rt->virtual_gimbal_angle_deg,
                                               -cfg->max_virtual_angle_deg,
                                               cfg->max_virtual_angle_deg);

    *platform_angle_used_deg = rt->filtered_angle_deg;
    *platform_rate_used_deg_s = rt->filtered_rate_deg_s;
}

static bool should_send_axis(const AxisRuntime &rt, const RuntimeOptions &opts) {
    if (opts.always_send) return true;
    return fabs(rt.speed_cmd_rpm - rt.last_sent_speed_cmd_rpm) >= opts.min_send_speed_delta_rpm;
}

static void mark_axis_sent(AxisRuntime *rt) {
    rt->last_sent_speed_cmd_rpm = rt->speed_cmd_rpm;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_platform_rate_speed_comp");
    ros::NodeHandle pnh("~");

    Controller ctrl;

    RuntimeOptions opts;
    opts.can_interface = "gimbalcan";
    opts.imu_device = "/dev/imu";
    opts.imu_baud = 115200;

    opts.control_hz = 500.0;
    opts.print_hz = 10.0;
    opts.imu_timeout_sec = 0.08;
    opts.imu_warmup_sec = 1.0;

    opts.main_thread_priority = 80;
    opts.imu_thread_priority = 85;
    opts.main_thread_cpu = -1;
    opts.imu_thread_cpu = -1;

    opts.try_realtime = true;
    opts.try_mlockall = true;
    opts.try_serial_low_latency = true;

    opts.switch_to_speed_mode_on_startup = true;
    opts.speed_mode_code = 3;
    opts.enable_on_startup = true;
    opts.disable_on_shutdown = false;
    opts.zero_speed_on_shutdown = true;

    opts.always_send = true;
    opts.min_send_speed_delta_rpm = 0.02;

    AxisConfig roll_axis;
    roll_axis.imu_angle_lpf_alpha = 0.20;
    roll_axis.rate_lpf_hz = 8.0;
    roll_axis.max_platform_rate_deg_s = 180.0;
    roll_axis.rate_ff_rpm_per_deg_s = 0.18;   // 理论上约 1/6 比较接近
    roll_axis.motor_dir = 1.0;
    roll_axis.level_offset_deg = 0.0;
    roll_axis.min_speed_rpm = 0.0;
    roll_axis.max_speed_rpm = 25.0;
    roll_axis.speed_slew_rpm_per_s = 120.0;
    roll_axis.max_virtual_angle_deg = 30.0;
    roll_axis.quiet_rate_deg_s = 1.0;
    roll_axis.center_return_k_rpm_per_deg = 0.05;

    AxisConfig pitch_axis = roll_axis;
    pitch_axis.max_speed_rpm = 20.0;
    pitch_axis.max_virtual_angle_deg = 25.0;

    pnh.param("can_interface", opts.can_interface, opts.can_interface);
    pnh.param("imu_device", opts.imu_device, opts.imu_device);
    pnh.param("imu_baud", opts.imu_baud, opts.imu_baud);

    pnh.param("control_hz", opts.control_hz, opts.control_hz);
    pnh.param("print_hz", opts.print_hz, opts.print_hz);
    pnh.param("imu_timeout_sec", opts.imu_timeout_sec, opts.imu_timeout_sec);
    pnh.param("imu_warmup_sec", opts.imu_warmup_sec, opts.imu_warmup_sec);

    pnh.param("main_thread_priority", opts.main_thread_priority, opts.main_thread_priority);
    pnh.param("imu_thread_priority", opts.imu_thread_priority, opts.imu_thread_priority);
    pnh.param("main_thread_cpu", opts.main_thread_cpu, opts.main_thread_cpu);
    pnh.param("imu_thread_cpu", opts.imu_thread_cpu, opts.imu_thread_cpu);

    pnh.param("try_realtime", opts.try_realtime, opts.try_realtime);
    pnh.param("try_mlockall", opts.try_mlockall, opts.try_mlockall);
    pnh.param("try_serial_low_latency", opts.try_serial_low_latency, opts.try_serial_low_latency);

    pnh.param("switch_to_speed_mode_on_startup",
              opts.switch_to_speed_mode_on_startup,
              opts.switch_to_speed_mode_on_startup);
    pnh.param("speed_mode_code", opts.speed_mode_code, opts.speed_mode_code);
    pnh.param("enable_on_startup", opts.enable_on_startup, opts.enable_on_startup);
    pnh.param("disable_on_shutdown", opts.disable_on_shutdown, opts.disable_on_shutdown);
    pnh.param("zero_speed_on_shutdown", opts.zero_speed_on_shutdown, opts.zero_speed_on_shutdown);

    pnh.param("always_send", opts.always_send, opts.always_send);
    pnh.param("min_send_speed_delta_rpm", opts.min_send_speed_delta_rpm, opts.min_send_speed_delta_rpm);

    pnh.param("roll_imu_angle_lpf_alpha", roll_axis.imu_angle_lpf_alpha, roll_axis.imu_angle_lpf_alpha);
    pnh.param("roll_rate_lpf_hz", roll_axis.rate_lpf_hz, roll_axis.rate_lpf_hz);
    pnh.param("roll_max_platform_rate_deg_s", roll_axis.max_platform_rate_deg_s, roll_axis.max_platform_rate_deg_s);
    pnh.param("roll_rate_ff_rpm_per_deg_s", roll_axis.rate_ff_rpm_per_deg_s, roll_axis.rate_ff_rpm_per_deg_s);
    pnh.param("roll_motor_dir", roll_axis.motor_dir, roll_axis.motor_dir);
    pnh.param("roll_level_offset_deg", roll_axis.level_offset_deg, roll_axis.level_offset_deg);
    pnh.param("roll_min_speed_rpm", roll_axis.min_speed_rpm, roll_axis.min_speed_rpm);
    pnh.param("roll_max_speed_rpm", roll_axis.max_speed_rpm, roll_axis.max_speed_rpm);
    pnh.param("roll_speed_slew_rpm_per_s", roll_axis.speed_slew_rpm_per_s, roll_axis.speed_slew_rpm_per_s);
    pnh.param("roll_max_virtual_angle_deg", roll_axis.max_virtual_angle_deg, roll_axis.max_virtual_angle_deg);
    pnh.param("roll_quiet_rate_deg_s", roll_axis.quiet_rate_deg_s, roll_axis.quiet_rate_deg_s);
    pnh.param("roll_center_return_k_rpm_per_deg", roll_axis.center_return_k_rpm_per_deg, roll_axis.center_return_k_rpm_per_deg);

    pnh.param("pitch_imu_angle_lpf_alpha", pitch_axis.imu_angle_lpf_alpha, pitch_axis.imu_angle_lpf_alpha);
    pnh.param("pitch_rate_lpf_hz", pitch_axis.rate_lpf_hz, pitch_axis.rate_lpf_hz);
    pnh.param("pitch_max_platform_rate_deg_s", pitch_axis.max_platform_rate_deg_s, pitch_axis.max_platform_rate_deg_s);
    pnh.param("pitch_rate_ff_rpm_per_deg_s", pitch_axis.rate_ff_rpm_per_deg_s, pitch_axis.rate_ff_rpm_per_deg_s);
    pnh.param("pitch_motor_dir", pitch_axis.motor_dir, pitch_axis.motor_dir);
    pnh.param("pitch_level_offset_deg", pitch_axis.level_offset_deg, pitch_axis.level_offset_deg);
    pnh.param("pitch_min_speed_rpm", pitch_axis.min_speed_rpm, pitch_axis.min_speed_rpm);
    pnh.param("pitch_max_speed_rpm", pitch_axis.max_speed_rpm, pitch_axis.max_speed_rpm);
    pnh.param("pitch_speed_slew_rpm_per_s", pitch_axis.speed_slew_rpm_per_s, pitch_axis.speed_slew_rpm_per_s);
    pnh.param("pitch_max_virtual_angle_deg", pitch_axis.max_virtual_angle_deg, pitch_axis.max_virtual_angle_deg);
    pnh.param("pitch_quiet_rate_deg_s", pitch_axis.quiet_rate_deg_s, pitch_axis.quiet_rate_deg_s);
    pnh.param("pitch_center_return_k_rpm_per_deg", pitch_axis.center_return_k_rpm_per_deg, pitch_axis.center_return_k_rpm_per_deg);

    roll_axis.imu_angle_lpf_alpha = clamp_value(roll_axis.imu_angle_lpf_alpha, 0.0, 1.0);
    pitch_axis.imu_angle_lpf_alpha = clamp_value(pitch_axis.imu_angle_lpf_alpha, 0.0, 1.0);

    try_enable_process_realtime(opts);

    ctrl.can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (ctrl.can_sock < 0) {
        ROS_ERROR("Failed to create CAN socket: %s", strerror(errno));
        return 1;
    }

    int sndbuf = 256 * 1024;
    int prio = 6;
    int loopback = 0;
    setsockopt(ctrl.can_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    setsockopt(ctrl.can_sock, SOL_SOCKET, SO_PRIORITY, &prio, sizeof(prio));
    setsockopt(ctrl.can_sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, opts.can_interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(ctrl.can_sock, SIOCGIFINDEX, &ifr) < 0) {
        ROS_ERROR("Failed to get CAN interface index for [%s]: %s",
                  opts.can_interface.c_str(), strerror(errno));
        close(ctrl.can_sock);
        return 1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(ctrl.can_sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        ROS_ERROR("Failed to bind CAN socket: %s", strerror(errno));
        close(ctrl.can_sock);
        return 1;
    }

    ROS_INFO("CAN bind OK on [%s]", opts.can_interface.c_str());

    // 安全起始：先 0 速
    stop_all_motors(&ctrl);
    usleep(5000);

    if (opts.switch_to_speed_mode_on_startup) {
        ROS_WARN("Trying to switch motors to SPEED mode, speed_mode_code=%d", opts.speed_mode_code);
        disable_all_motors(&ctrl);
        usleep(10000);
        switch_all_motors_to_speed_mode(&ctrl, opts.speed_mode_code);
        usleep(10000);
    } else {
        ROS_WARN("switch_to_speed_mode_on_startup=false, make sure motors are already in SPEED mode.");
    }

    if (opts.enable_on_startup) {
        enable_all_motors(&ctrl);
        usleep(10000);
    }

    stop_all_motors(&ctrl);

    pthread_t imu_thread;
    ImuThreadArgs imu_thread_args;
    imu_thread_args.ctrl = &ctrl;
    imu_thread_args.opts = &opts;

    if (pthread_create(&imu_thread, NULL, imu_thread_func, &imu_thread_args) != 0) {
        ROS_ERROR("Failed to create IMU thread");
        close(ctrl.can_sock);
        return 1;
    }

    AxisRuntime roll_rt;
    AxisRuntime pitch_rt;
    reset_axis_runtime(&roll_rt);
    reset_axis_runtime(&pitch_rt);

    ROS_INFO("Platform-rate feedforward speed compensation started");
    ROS_INFO("  control_hz=%.1f  imu_timeout=%.3f  imu_warmup=%.3f  print_hz=%.1f",
             opts.control_hz, opts.imu_timeout_sec, opts.imu_warmup_sec, opts.print_hz);
    ROS_INFO("  roll : ff=%.4f rpm/(deg/s), max=%.1f rpm, max_virtual=%.1f deg",
             roll_axis.rate_ff_rpm_per_deg_s, roll_axis.max_speed_rpm, roll_axis.max_virtual_angle_deg);
    ROS_INFO("  pitch: ff=%.4f rpm/(deg/s), max=%.1f rpm, max_virtual=%.1f deg",
             pitch_axis.rate_ff_rpm_per_deg_s, pitch_axis.max_speed_rpm, pitch_axis.max_virtual_angle_deg);

    const uint64_t period_ns = static_cast<uint64_t>(1e9 / opts.control_hz);
    uint64_t next_tick_ns = monotonic_time_ns();
    uint64_t last_loop_ns = next_tick_ns;
    uint64_t last_print_ns = next_tick_ns;
    double print_period_ns = (opts.print_hz > 0.0) ? (1e9 / opts.print_hz) : 1e9;

    double first_fresh_imu_time = -1.0;

    while (ros::ok()) {
        next_tick_ns += period_ns;

        struct timespec ts;
        ts.tv_sec = static_cast<time_t>(next_tick_ns / 1000000000ULL);
        ts.tv_nsec = static_cast<long>(next_tick_ns % 1000000000ULL);
        int sleep_ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        if (sleep_ret != 0 && sleep_ret != EINTR) {
            ROS_WARN_THROTTLE(1.0, "clock_nanosleep returned %d", sleep_ret);
        }

        uint64_t now_ns = monotonic_time_ns();
        double now_sec = monotonic_time_sec();
        double dt = static_cast<double>(now_ns - last_loop_ns) * 1e-9;
        last_loop_ns = now_ns;
        if (dt <= 1e-6) dt = 1.0 / opts.control_hz;

        double imu_age = now_sec - ctrl.imu_timestamp_sec.load(std::memory_order_relaxed);
        bool imu_fresh = ctrl.imu_ready.load(std::memory_order_relaxed) && (imu_age <= opts.imu_timeout_sec);

        if (imu_fresh && first_fresh_imu_time < 0.0) {
            first_fresh_imu_time = now_sec;
        }

        bool imu_warmed_up = imu_fresh &&
                             first_fresh_imu_time > 0.0 &&
                             ((now_sec - first_fresh_imu_time) >= opts.imu_warmup_sec);

        double roll_angle_raw = ctrl.roll.load(std::memory_order_relaxed);
        double pitch_angle_raw = ctrl.pitch.load(std::memory_order_relaxed);

        double roll_angle_used = 0.0, pitch_angle_used = 0.0;
        double roll_rate_used = 0.0, pitch_rate_used = 0.0;
        double roll_desired_raw_rpm = 0.0, pitch_desired_raw_rpm = 0.0;

        if (imu_warmed_up) {
            update_axis_runtime_feedforward(&roll_axis, &roll_rt, roll_angle_raw, dt,
                                            &roll_angle_used, &roll_rate_used, &roll_desired_raw_rpm);

            update_axis_runtime_feedforward(&pitch_axis, &pitch_rt, pitch_angle_raw, dt,
                                            &pitch_angle_used, &pitch_rate_used, &pitch_desired_raw_rpm);
        } else {
            // 预热期 / IMU 失效期：平滑刹到 0
            double roll_step = roll_axis.speed_slew_rpm_per_s * dt;
            double pitch_step = pitch_axis.speed_slew_rpm_per_s * dt;
            roll_rt.speed_cmd_rpm = approach_value(roll_rt.speed_cmd_rpm, 0.0, roll_step);
            pitch_rt.speed_cmd_rpm = approach_value(pitch_rt.speed_cmd_rpm, 0.0, pitch_step);

            if (!imu_fresh) {
                reset_axis_runtime(&roll_rt);
                reset_axis_runtime(&pitch_rt);
                first_fresh_imu_time = -1.0;
            }
        }

        bool send_roll = should_send_axis(roll_rt, opts);
        bool send_pitch = should_send_axis(pitch_rt, opts);

        if (send_roll) {
            set_motor_speed(&ctrl, ROLL_MOTOR_ID, roll_rt.speed_cmd_rpm);
            mark_axis_sent(&roll_rt);
        }
        if (send_pitch) {
            set_motor_speed(&ctrl, PITCH_MOTOR_ID, pitch_rt.speed_cmd_rpm);
            mark_axis_sent(&pitch_rt);
        }

        if (static_cast<double>(now_ns - last_print_ns) >= print_period_ns) {
            last_print_ns = now_ns;
            printf("IMU:%s warm:%s age=%6.3fms pkt=%d err=%d | "
                   "PlatAng R=%7.3f P=%7.3f | "
                   "PlatRate R=%7.3f P=%7.3f | "
                   "RawRPM R=%7.3f P=%7.3f | "
                   "CmdRPM R=%7.3f P=%7.3f | "
                   "VirtPos R=%7.3f P=%7.3f\n",
                   imu_fresh ? "OK" : "STALE",
                   imu_warmed_up ? "YES" : "NO ",
                   imu_age * 1000.0,
                   ctrl.imu_packets.load(std::memory_order_relaxed),
                   ctrl.imu_parse_errors.load(std::memory_order_relaxed),
                   roll_angle_used, pitch_angle_used,
                   roll_rate_used, pitch_rate_used,
                   roll_desired_raw_rpm, pitch_desired_raw_rpm,
                   roll_rt.speed_cmd_rpm, pitch_rt.speed_cmd_rpm,
                   roll_rt.virtual_gimbal_angle_deg, pitch_rt.virtual_gimbal_angle_deg);
            fflush(stdout);
        }
    }

    ctrl.running.store(false, std::memory_order_relaxed);
    pthread_join(imu_thread, NULL);

    if (opts.zero_speed_on_shutdown) {
        stop_all_motors(&ctrl);
        usleep(5000);
    }
    if (opts.disable_on_shutdown) {
        disable_all_motors(&ctrl);
    }

    close(ctrl.can_sock);
    return 0;
}