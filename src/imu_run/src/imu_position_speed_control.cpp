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
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <algorithm>

#ifdef __linux__
#include <linux/serial.h>
#endif

#define MOTOR_PULSES_PER_REVOLUTION 65536.0
#define DEGREES_PER_REVOLUTION 360.0
#define MOTOR_SECONDS_PER_MINUTE 60.0
#define ROLL_MOTOR_ID 0x16
#define PITCH_MOTOR_ID 0x15
#define SERVO_CMD_WRITE 0x01
#define SERVO_ADDR_TARGET_SPEED 0x09
#define SERVO_ADDR_TARGET_POSITION 0x0A
#define SERVO_ADDR_TARGET_ACCEL 0x0B
#define SERVO_ADDR_TARGET_DECEL 0x0C
#define SERVO_ADDR_WORK_MODE 0x0F
#define SERVO_ADDR_ENABLE 0x10
#define SERVO_MODE_PROFILE_POSITION_SPEED 0x00000001

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
        : roll(0.0), pitch(0.0), imu_timestamp_sec(0.0), imu_packets(0),
          imu_parse_errors(0), running(true), imu_ready(false), can_sock(-1) {}
};

struct PidConfig {
    double kp;
    double ki;
    double kd;
    double output_limit;
    double integral_limit;
    double integral_separation_deg;
    double anti_windup_gain;
    double d_lpf_hz;
};

struct PidState {
    double integral_term;
    double d_term_lpf;
    double prev_measurement;
    bool initialized;
};

struct AxisConfig {
    double error_deadband_deg;
    double error_deadband_exit_deg;
    double imu_lpf_alpha;
    double speed_scale_rpm_per_deg;
    double min_speed_rpm;
    double max_speed_rpm;
    double target_slew_deg_per_s;
    double speed_slew_rpm_per_s;
    double motor_dir;
    double level_offset_deg;
    double position_offset_deg;
};

struct AxisRuntime {
    bool active;
    bool filtered_initialized;
    double filtered_measurement;
    double target_cmd_deg;
    double speed_cmd_rpm;
    double last_sent_target_cmd_deg;
    double last_sent_speed_cmd_rpm;
    bool last_sent_active;

    // 基于 IMU 变化率的动态发送频率
    bool send_rate_initialized;
    double prev_measurement_for_rate_deg;
    double imu_rate_ema_deg_s;
    double current_send_hz;
    double send_time_accumulator_sec;
};

struct RuntimeOptions {
    std::string can_interface;
    std::string imu_device;
    int imu_baud;
    double control_hz;
    double print_hz;
    double imu_timeout_sec;
    bool home_on_startup;
    double home_speed_rpm;
    double home_accel_rpm_s;
    double home_decel_rpm_s;
    double home_settle_time_sec;
    int main_thread_priority;
    int imu_thread_priority;
    int main_thread_cpu;
    int imu_thread_cpu;
    bool try_realtime;
    bool try_mlockall;
    bool try_serial_low_latency;
    double min_send_position_delta_deg;
    double min_send_speed_delta_rpm;
    bool always_send;

    // 新增：按 IMU 角速度动态调节发送频率
    bool dynamic_imu_rate_send_rate_enable;
    double dynamic_imu_rate_ema_alpha;
    double dynamic_imu_rate_send_hz_level_0;  // 最小角速度 -> 最高频
    double dynamic_imu_rate_send_hz_level_1;
    double dynamic_imu_rate_send_hz_level_2;
    double dynamic_imu_rate_send_hz_level_3;  // 最大角速度 -> 最低频
    double dynamic_imu_rate_threshold_1_deg_s;
    double dynamic_imu_rate_threshold_2_deg_s;
    double dynamic_imu_rate_threshold_3_deg_s;
    double dynamic_imu_rate_force_resend_sec;
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
    if(value > max_value) return max_value;
    if(value < min_value) return min_value;
    return value;
}

static double approach_value(double current, double target, double max_step) {
    if(max_step <= 0.0) {
        return target;
    }
    double delta = target - current;
    if(delta > max_step) return current + max_step;
    if(delta < -max_step) return current - max_step;
    return target;
}

static double cutoff_to_alpha(double cutoff_hz, double dt) {
    if(cutoff_hz <= 0.0) {
        return 1.0;
    }
    const double pi = 3.14159265358979323846;
    double tau = 1.0 / (2.0 * pi * cutoff_hz);
    return dt / (tau + dt);
}

static double smoothstep01(double x) {
    x = clamp_value(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

static void pid_reset(PidState *state) {
    state->integral_term = 0.0;
    state->d_term_lpf = 0.0;
    state->prev_measurement = 0.0;
    state->initialized = false;
}

static double pid_update(const PidConfig *cfg, PidState *state, double error, double measurement, double dt) {
    if(dt <= 1e-6) {
        dt = 1.0 / 200.0;
    }

    if(!state->initialized) {
        state->prev_measurement = measurement;
        state->d_term_lpf = 0.0;
        state->initialized = true;
    }

    double p_term = cfg->kp * error;

    bool integral_enabled = (cfg->integral_separation_deg <= 0.0) ||
                            (fabs(error) <= cfg->integral_separation_deg);

    if(integral_enabled && fabs(cfg->ki) > 1e-12) {
        state->integral_term += cfg->ki * error * dt;
    }

    if(cfg->integral_limit > 0.0) {
        state->integral_term = clamp_value(state->integral_term, -cfg->integral_limit, cfg->integral_limit);
    }

    double d_input = (measurement - state->prev_measurement) / dt;
    double raw_d_term = -cfg->kd * d_input;
    double d_alpha = cutoff_to_alpha(cfg->d_lpf_hz, dt);
    state->d_term_lpf += d_alpha * (raw_d_term - state->d_term_lpf);

    double unsat_output = p_term + state->integral_term + state->d_term_lpf;
    double sat_output = unsat_output;
    if(cfg->output_limit > 0.0) {
        sat_output = clamp_value(unsat_output, -cfg->output_limit, cfg->output_limit);
    }

    if(cfg->anti_windup_gain > 0.0 && fabs(cfg->ki) > 1e-12) {
        double aw_term = cfg->anti_windup_gain * (sat_output - unsat_output);
        state->integral_term += aw_term * dt;
        if(cfg->integral_limit > 0.0) {
            state->integral_term = clamp_value(state->integral_term, -cfg->integral_limit, cfg->integral_limit);
        }
    }

    state->prev_measurement = measurement;
    return sat_output;
}

static int try_set_realtime_for_current_thread(int policy, int priority, int cpu_core) {
    if(cpu_core >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_core, &cpuset);
        if(pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            return -1;
        }
    }

    struct sched_param sch;
    memset(&sch, 0, sizeof(sch));
    sch.sched_priority = priority;
    if(pthread_setschedparam(pthread_self(), policy, &sch) != 0) {
        return -2;
    }
    return 0;
}

static void try_enable_process_realtime(const RuntimeOptions &opts) {
    if(opts.try_mlockall) {
        if(mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            ROS_WARN("mlockall failed: %s", strerror(errno));
        }
    }

    if(opts.try_realtime) {
        int ret = try_set_realtime_for_current_thread(SCHED_FIFO, opts.main_thread_priority, opts.main_thread_cpu);
        if(ret != 0) {
            ROS_WARN("Failed to set main thread realtime scheduling (ret=%d, errno=%d: %s)", ret, errno, strerror(errno));
        } else {
            ROS_INFO("Main thread set to SCHED_FIFO priority=%d cpu=%d", opts.main_thread_priority, opts.main_thread_cpu);
        }
    }
}

static bool try_set_serial_low_latency(int fd) {
#ifdef __linux__
    struct serial_struct ser;
    if(ioctl(fd, TIOCGSERIAL, &ser) != 0) {
        return false;
    }
    ser.flags |= ASYNC_LOW_LATENCY;
    if(ioctl(fd, TIOCSSERIAL, &ser) != 0) {
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

static int32_t degrees_to_pulses(double degrees) {
    return static_cast<int32_t>(degrees * MOTOR_PULSES_PER_REVOLUTION / DEGREES_PER_REVOLUTION);
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

static void set_motor_position(Controller *c, int id, int32_t pulses) {
    send_can_frame_fast(c->can_sock, id, SERVO_CMD_WRITE, SERVO_ADDR_TARGET_POSITION, pulses);
}

static void set_motor_accel(Controller *c, int id, double accel_rpm_s) {
    send_can_frame_fast(c->can_sock, id, SERVO_CMD_WRITE, SERVO_ADDR_TARGET_ACCEL, rpm_to_pulses_per_second(accel_rpm_s));
}

static void set_motor_decel(Controller *c, int id, double decel_rpm_s) {
    send_can_frame_fast(c->can_sock, id, SERVO_CMD_WRITE, SERVO_ADDR_TARGET_DECEL, rpm_to_pulses_per_second(decel_rpm_s));
}

static void configure_position_mode(Controller *c, int id, double accel_rpm_s, double decel_rpm_s) {
    set_motor_mode(c, id, SERVO_MODE_PROFILE_POSITION_SPEED);
    usleep(5000);
    set_motor_accel(c, id, accel_rpm_s);
    usleep(2000);
    set_motor_decel(c, id, decel_rpm_s);
    usleep(2000);
}

static void home_motor_to_zero(Controller *c, int id, double home_speed_rpm, double home_accel_rpm_s, double home_decel_rpm_s) {
    configure_position_mode(c, id, home_accel_rpm_s, home_decel_rpm_s);
    set_motor_speed(c, id, home_speed_rpm);
    usleep(2000);
    set_motor_position(c, id, 0);
}

static void home_motors_to_zero(Controller *c, double home_speed_rpm, double home_accel_rpm_s,
                                double home_decel_rpm_s, double settle_time_sec) {
    ROS_INFO("Homing motors to reference zero...");
    home_motor_to_zero(c, ROLL_MOTOR_ID, home_speed_rpm, home_accel_rpm_s, home_decel_rpm_s);
    usleep(10000);
    home_motor_to_zero(c, PITCH_MOTOR_ID, home_speed_rpm, home_accel_rpm_s, home_decel_rpm_s);
    usleep(static_cast<useconds_t>(settle_time_sec * 1000000.0));
}

static bool extract_imu_packet(const std::vector<uint8_t> &buf, size_t start_idx, float &roll, float &pitch) {
    if(start_idx + 6 > buf.size()) return false;
    uint16_t payload_len = static_cast<uint16_t>(buf[start_idx + 2]) |
                           (static_cast<uint16_t>(buf[start_idx + 3]) << 8);
    size_t total_len = 6 + payload_len;
    if(start_idx + total_len > buf.size()) return false;

    const size_t roll_offset = start_idx + 6 + 48;
    const size_t pitch_offset = roll_offset + sizeof(float);
    if(pitch_offset + sizeof(float) > start_idx + total_len) return false;

    memcpy(&roll, &buf[roll_offset], sizeof(float));
    memcpy(&pitch, &buf[pitch_offset], sizeof(float));
    return true;
}

static void *imu_thread_func(void *arg) {
    ImuThreadArgs *thread_args = reinterpret_cast<ImuThreadArgs*>(arg);
    Controller *c = thread_args->ctrl;
    RuntimeOptions *opts = thread_args->opts;

    if(opts->try_realtime) {
        int ret = try_set_realtime_for_current_thread(SCHED_FIFO, opts->imu_thread_priority, opts->imu_thread_cpu);
        if(ret != 0) {
            ROS_WARN("Failed to set IMU thread realtime scheduling (ret=%d, errno=%d: %s)", ret, errno, strerror(errno));
        } else {
            ROS_INFO("IMU thread set to SCHED_FIFO priority=%d cpu=%d", opts->imu_thread_priority, opts->imu_thread_cpu);
        }
    }

    int fd = open(opts->imu_device.c_str(), O_RDONLY | O_NOCTTY);
    if(fd < 0) {
        ROS_ERROR("IMU open failed: %s", strerror(errno));
        return NULL;
    }

    struct termios opt;
    memset(&opt, 0, sizeof(opt));
    if(tcgetattr(fd, &opt) != 0) {
        ROS_ERROR("tcgetattr failed: %s", strerror(errno));
        close(fd);
        return NULL;
    }

    speed_t baud = B115200;
    switch(opts->imu_baud) {
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

    if(tcsetattr(fd, TCSANOW, &opt) != 0) {
        ROS_ERROR("tcsetattr failed: %s", strerror(errno));
        close(fd);
        return NULL;
    }

    if(opts->try_serial_low_latency) {
        bool ok = try_set_serial_low_latency(fd);
        ROS_INFO("Serial low latency request: %s", ok ? "success" : "failed / unsupported");
    }

    std::vector<uint8_t> stream_buf;
    stream_buf.reserve(8192);
    uint8_t temp[1024];

    while(c->running.load(std::memory_order_relaxed)) {
        ssize_t n = read(fd, temp, sizeof(temp));
        if(n < 0) {
            if(errno == EINTR) continue;
            ROS_ERROR_THROTTLE(1.0, "IMU read failed: %s", strerror(errno));
            continue;
        }
        if(n == 0) {
            continue;
        }

        stream_buf.insert(stream_buf.end(), temp, temp + n);

        size_t scan = 0;
        while(scan + 6 <= stream_buf.size()) {
            if(stream_buf[scan] != 0x5A || stream_buf[scan + 1] != 0xA5) {
                ++scan;
                continue;
            }

            uint16_t payload_len = static_cast<uint16_t>(stream_buf[scan + 2]) |
                                   (static_cast<uint16_t>(stream_buf[scan + 3]) << 8);
            size_t total_len = 6 + payload_len;
            if(total_len < 6 || total_len > 2048) {
                c->imu_parse_errors.fetch_add(1, std::memory_order_relaxed);
                ++scan;
                continue;
            }

            if(scan + total_len > stream_buf.size()) {
                break;
            }

            float roll = 0.0f;
            float pitch = 0.0f;
            if(extract_imu_packet(stream_buf, scan, roll, pitch)) {
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

        if(scan > 0) {
            stream_buf.erase(stream_buf.begin(), stream_buf.begin() + static_cast<long>(scan));
        }
        if(stream_buf.size() > 8192) {
            stream_buf.erase(stream_buf.begin(), stream_buf.end() - 2048);
        }
    }

    close(fd);
    return NULL;
}

static double compute_smooth_min_speed_rpm(const AxisConfig *axis_cfg, double abs_error_deg) {
    if(axis_cfg->min_speed_rpm <= 0.0) {
        return 0.0;
    }

    const double smooth_zone_start = std::max(0.0, axis_cfg->error_deadband_deg);
    const double auto_extra_zone = std::max(0.5, 2.0 * std::max(0.0, axis_cfg->error_deadband_deg));
    const double smooth_zone_end = std::max(axis_cfg->error_deadband_exit_deg,
                                            smooth_zone_start + auto_extra_zone);

    if(abs_error_deg <= smooth_zone_start) {
        return 0.0;
    }
    if(abs_error_deg >= smooth_zone_end) {
        return axis_cfg->min_speed_rpm;
    }

    double t = (abs_error_deg - smooth_zone_start) / (smooth_zone_end - smooth_zone_start);
    t = smoothstep01(t);
    return axis_cfg->min_speed_rpm * t;
}

static bool update_axis_runtime(const AxisConfig *axis_cfg, const PidConfig *pid_cfg, PidState *pid_state,
                                AxisRuntime *rt, double measured_angle_deg, double dt,
                                double *pid_output_deg, double *error_deg) {
    if(!rt->filtered_initialized) {
        rt->filtered_measurement = measured_angle_deg;
        rt->filtered_initialized = true;
    } else {
        rt->filtered_measurement += axis_cfg->imu_lpf_alpha * (measured_angle_deg - rt->filtered_measurement);
    }

    *error_deg = axis_cfg->level_offset_deg - rt->filtered_measurement;
    double abs_error = fabs(*error_deg);

    if(rt->active) {
        rt->active = abs_error >= axis_cfg->error_deadband_deg;
    } else {
        rt->active = abs_error >= axis_cfg->error_deadband_exit_deg;
    }

    if(rt->active) {
        *pid_output_deg = pid_update(pid_cfg, pid_state, *error_deg, rt->filtered_measurement, dt);
    } else {
        *pid_output_deg = 0.0;
        pid_reset(pid_state);
    }

    double desired_target = axis_cfg->position_offset_deg + axis_cfg->motor_dir * (*pid_output_deg);
    double desired_speed = 0.0;

    if(rt->active) {
        double base_speed = fabs(*pid_output_deg) * axis_cfg->speed_scale_rpm_per_deg;
        base_speed = clamp_value(base_speed, 0.0, axis_cfg->max_speed_rpm);

        double smooth_min_speed = compute_smooth_min_speed_rpm(axis_cfg, abs_error);

        desired_speed = std::max(base_speed, smooth_min_speed);
        desired_speed = clamp_value(desired_speed, 0.0, axis_cfg->max_speed_rpm);
    }

    double target_step = axis_cfg->target_slew_deg_per_s * dt;
    double speed_step = axis_cfg->speed_slew_rpm_per_s * dt;
    rt->target_cmd_deg = approach_value(rt->target_cmd_deg, desired_target, target_step);
    rt->speed_cmd_rpm = approach_value(rt->speed_cmd_rpm, desired_speed, speed_step);

    return rt->active;
}

static double compute_imu_rate_based_send_hz(const RuntimeOptions *opts, double imu_rate_ema_deg_s) {
    double hz = opts->dynamic_imu_rate_send_hz_level_0;

    if(imu_rate_ema_deg_s <= opts->dynamic_imu_rate_threshold_1_deg_s) {
        hz = opts->dynamic_imu_rate_send_hz_level_0;
    } else if(imu_rate_ema_deg_s <= opts->dynamic_imu_rate_threshold_2_deg_s) {
        hz = opts->dynamic_imu_rate_send_hz_level_1;
    } else if(imu_rate_ema_deg_s <= opts->dynamic_imu_rate_threshold_3_deg_s) {
        hz = opts->dynamic_imu_rate_send_hz_level_2;
    } else {
        hz = opts->dynamic_imu_rate_send_hz_level_3;
    }

    hz = clamp_value(hz, 0.1, std::max(0.1, opts->control_hz));
    return hz;
}

static void update_axis_send_scheduler_by_imu_rate(AxisRuntime *rt,
                                                   const RuntimeOptions *opts,
                                                   double current_measurement_deg,
                                                   double dt) {
    if(!opts->dynamic_imu_rate_send_rate_enable) {
        return;
    }

    double alpha = clamp_value(opts->dynamic_imu_rate_ema_alpha, 0.0, 1.0);

    if(!rt->send_rate_initialized) {
        rt->prev_measurement_for_rate_deg = current_measurement_deg;
        rt->imu_rate_ema_deg_s = 0.0;
        rt->current_send_hz = compute_imu_rate_based_send_hz(opts, 0.0);
        rt->send_time_accumulator_sec = 1.0; // 首次允许立即发送
        rt->send_rate_initialized = true;
        return;
    }

    double safe_dt = std::max(dt, 1e-6);
    double inst_rate_deg_s = fabs(current_measurement_deg - rt->prev_measurement_for_rate_deg) / safe_dt;
    rt->prev_measurement_for_rate_deg = current_measurement_deg;

    rt->imu_rate_ema_deg_s += alpha * (inst_rate_deg_s - rt->imu_rate_ema_deg_s);
    rt->current_send_hz = compute_imu_rate_based_send_hz(opts, rt->imu_rate_ema_deg_s);
    rt->send_time_accumulator_sec += dt;
}

static bool should_send_axis_basic(const AxisRuntime &rt, const RuntimeOptions &opts) {
    if(opts.always_send) return true;
    return (fabs(rt.target_cmd_deg - rt.last_sent_target_cmd_deg) >= opts.min_send_position_delta_deg) ||
           (fabs(rt.speed_cmd_rpm - rt.last_sent_speed_cmd_rpm) >= opts.min_send_speed_delta_rpm);
}

static bool should_send_axis_adaptive(const AxisRuntime &rt, const RuntimeOptions &opts) {
    if(!opts.dynamic_imu_rate_send_rate_enable) {
        return should_send_axis_basic(rt, opts);
    }

    bool state_changed = (rt.active != rt.last_sent_active);
    if(state_changed) {
        return true;
    }

    bool changed = (fabs(rt.target_cmd_deg - rt.last_sent_target_cmd_deg) >= opts.min_send_position_delta_deg) ||
                   (fabs(rt.speed_cmd_rpm - rt.last_sent_speed_cmd_rpm) >= opts.min_send_speed_delta_rpm);

    if(!opts.always_send && !changed) {
        if(opts.dynamic_imu_rate_force_resend_sec > 0.0 &&
           rt.send_time_accumulator_sec >= opts.dynamic_imu_rate_force_resend_sec) {
            return true;
        }
        return false;
    }

    double interval_sec = 1.0 / std::max(0.1, rt.current_send_hz);
    bool interval_ready = (rt.send_time_accumulator_sec >= interval_sec);

    if(interval_ready) {
        return true;
    }

    if(opts.dynamic_imu_rate_force_resend_sec > 0.0 &&
       rt.send_time_accumulator_sec >= opts.dynamic_imu_rate_force_resend_sec) {
        return true;
    }

    return false;
}

static void mark_axis_sent(AxisRuntime *rt, const RuntimeOptions *opts) {
    rt->last_sent_target_cmd_deg = rt->target_cmd_deg;
    rt->last_sent_speed_cmd_rpm = rt->speed_cmd_rpm;
    rt->last_sent_active = rt->active;

    if(opts->dynamic_imu_rate_send_rate_enable) {
        rt->send_time_accumulator_sec = 0.0;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_position_speed_control_low_latency");
    ros::NodeHandle pnh("~");

    Controller ctrl;

    RuntimeOptions opts;
    opts.can_interface = "can0";
    opts.imu_device = "/dev/ttyUSB0";
    opts.imu_baud = 115200;
    opts.control_hz = 300.0;
    opts.print_hz = 10.0;
    opts.imu_timeout_sec = 0.08;
    opts.home_on_startup = true;
    opts.home_speed_rpm = 10.0;
    opts.home_accel_rpm_s = 800.0;
    opts.home_decel_rpm_s = 800.0;
    opts.home_settle_time_sec = 3.0;
    opts.main_thread_priority = 80;
    opts.imu_thread_priority = 85;
    opts.main_thread_cpu = -1;
    opts.imu_thread_cpu = -1;
    opts.try_realtime = true;
    opts.try_mlockall = true;
    opts.try_serial_low_latency = true;
    opts.min_send_position_delta_deg = 0.002;
    opts.min_send_speed_delta_rpm = 0.02;
    opts.always_send = true;

    // 默认关闭，兼容旧 launch
    opts.dynamic_imu_rate_send_rate_enable = false;
    opts.dynamic_imu_rate_ema_alpha = 0.25;
    opts.dynamic_imu_rate_send_hz_level_0 = 30.0; // 最小角速度
    opts.dynamic_imu_rate_send_hz_level_1 = 20.0;
    opts.dynamic_imu_rate_send_hz_level_2 = 10.0;
    opts.dynamic_imu_rate_send_hz_level_3 = 5.0;  // 最大角速度
    opts.dynamic_imu_rate_threshold_1_deg_s = 2.0;
    opts.dynamic_imu_rate_threshold_2_deg_s = 8.0;
    opts.dynamic_imu_rate_threshold_3_deg_s = 20.0;
    opts.dynamic_imu_rate_force_resend_sec = 0.20;

    AxisConfig roll_axis;
    roll_axis.error_deadband_deg = 0.25;
    roll_axis.error_deadband_exit_deg = 0.50;
    roll_axis.imu_lpf_alpha = 0.40;
    roll_axis.speed_scale_rpm_per_deg = 8.0;
    roll_axis.min_speed_rpm = 6.0;
    roll_axis.max_speed_rpm = 120.0;
    roll_axis.target_slew_deg_per_s = 260.0;
    roll_axis.speed_slew_rpm_per_s = 520.0;
    roll_axis.motor_dir = 1.0;
    roll_axis.level_offset_deg = 0.0;
    roll_axis.position_offset_deg = 0.0;

    AxisConfig pitch_axis = roll_axis;

    PidConfig roll_pid_cfg;
    roll_pid_cfg.kp = 1.20;
    roll_pid_cfg.ki = 0.0;
    roll_pid_cfg.kd = 0.045;
    roll_pid_cfg.output_limit = 60.0;
    roll_pid_cfg.integral_limit = 20.0;
    roll_pid_cfg.integral_separation_deg = 8.0;
    roll_pid_cfg.anti_windup_gain = 6.0;
    roll_pid_cfg.d_lpf_hz = 6.0;

    PidConfig pitch_pid_cfg = roll_pid_cfg;

    pnh.param("can_interface", opts.can_interface, opts.can_interface);
    pnh.param("imu_device", opts.imu_device, opts.imu_device);
    pnh.param("imu_baud", opts.imu_baud, opts.imu_baud);
    pnh.param("control_hz", opts.control_hz, opts.control_hz);
    pnh.param("print_hz", opts.print_hz, opts.print_hz);
    pnh.param("imu_timeout_sec", opts.imu_timeout_sec, opts.imu_timeout_sec);
    pnh.param("home_on_startup", opts.home_on_startup, opts.home_on_startup);
    pnh.param("home_speed_rpm", opts.home_speed_rpm, opts.home_speed_rpm);
    pnh.param("home_accel_rpm_s", opts.home_accel_rpm_s, opts.home_accel_rpm_s);
    pnh.param("home_decel_rpm_s", opts.home_decel_rpm_s, opts.home_decel_rpm_s);
    pnh.param("home_settle_time_sec", opts.home_settle_time_sec, opts.home_settle_time_sec);
    pnh.param("main_thread_priority", opts.main_thread_priority, opts.main_thread_priority);
    pnh.param("imu_thread_priority", opts.imu_thread_priority, opts.imu_thread_priority);
    pnh.param("main_thread_cpu", opts.main_thread_cpu, opts.main_thread_cpu);
    pnh.param("imu_thread_cpu", opts.imu_thread_cpu, opts.imu_thread_cpu);
    pnh.param("try_realtime", opts.try_realtime, opts.try_realtime);
    pnh.param("try_mlockall", opts.try_mlockall, opts.try_mlockall);
    pnh.param("try_serial_low_latency", opts.try_serial_low_latency, opts.try_serial_low_latency);
    pnh.param("min_send_position_delta_deg", opts.min_send_position_delta_deg, opts.min_send_position_delta_deg);
    pnh.param("min_send_speed_delta_rpm", opts.min_send_speed_delta_rpm, opts.min_send_speed_delta_rpm);
    pnh.param("always_send", opts.always_send, opts.always_send);

    // 新增：按 IMU 变化率动态发送频率
    pnh.param("dynamic_imu_rate_send_rate_enable", opts.dynamic_imu_rate_send_rate_enable, opts.dynamic_imu_rate_send_rate_enable);
    pnh.param("dynamic_imu_rate_ema_alpha", opts.dynamic_imu_rate_ema_alpha, opts.dynamic_imu_rate_ema_alpha);
    pnh.param("dynamic_imu_rate_send_hz_level_0", opts.dynamic_imu_rate_send_hz_level_0, opts.dynamic_imu_rate_send_hz_level_0);
    pnh.param("dynamic_imu_rate_send_hz_level_1", opts.dynamic_imu_rate_send_hz_level_1, opts.dynamic_imu_rate_send_hz_level_1);
    pnh.param("dynamic_imu_rate_send_hz_level_2", opts.dynamic_imu_rate_send_hz_level_2, opts.dynamic_imu_rate_send_hz_level_2);
    pnh.param("dynamic_imu_rate_send_hz_level_3", opts.dynamic_imu_rate_send_hz_level_3, opts.dynamic_imu_rate_send_hz_level_3);
    pnh.param("dynamic_imu_rate_threshold_1_deg_s", opts.dynamic_imu_rate_threshold_1_deg_s, opts.dynamic_imu_rate_threshold_1_deg_s);
    pnh.param("dynamic_imu_rate_threshold_2_deg_s", opts.dynamic_imu_rate_threshold_2_deg_s, opts.dynamic_imu_rate_threshold_2_deg_s);
    pnh.param("dynamic_imu_rate_threshold_3_deg_s", opts.dynamic_imu_rate_threshold_3_deg_s, opts.dynamic_imu_rate_threshold_3_deg_s);
    pnh.param("dynamic_imu_rate_force_resend_sec", opts.dynamic_imu_rate_force_resend_sec, opts.dynamic_imu_rate_force_resend_sec);

    pnh.param("roll_deadband_deg", roll_axis.error_deadband_deg, roll_axis.error_deadband_deg);
    pnh.param("roll_deadband_exit_deg", roll_axis.error_deadband_exit_deg, roll_axis.error_deadband_exit_deg);
    pnh.param("roll_imu_lpf_alpha", roll_axis.imu_lpf_alpha, roll_axis.imu_lpf_alpha);
    pnh.param("roll_speed_scale_rpm_per_deg", roll_axis.speed_scale_rpm_per_deg, roll_axis.speed_scale_rpm_per_deg);
    pnh.param("roll_min_speed_rpm", roll_axis.min_speed_rpm, roll_axis.min_speed_rpm);
    pnh.param("roll_max_speed_rpm", roll_axis.max_speed_rpm, roll_axis.max_speed_rpm);
    pnh.param("roll_target_slew_deg_per_s", roll_axis.target_slew_deg_per_s, roll_axis.target_slew_deg_per_s);
    pnh.param("roll_speed_slew_rpm_per_s", roll_axis.speed_slew_rpm_per_s, roll_axis.speed_slew_rpm_per_s);
    pnh.param("roll_motor_dir", roll_axis.motor_dir, roll_axis.motor_dir);
    pnh.param("roll_level_offset_deg", roll_axis.level_offset_deg, roll_axis.level_offset_deg);
    pnh.param("roll_position_offset_deg", roll_axis.position_offset_deg, roll_axis.position_offset_deg);

    pnh.param("pitch_deadband_deg", pitch_axis.error_deadband_deg, pitch_axis.error_deadband_deg);
    pnh.param("pitch_deadband_exit_deg", pitch_axis.error_deadband_exit_deg, pitch_axis.error_deadband_exit_deg);
    pnh.param("pitch_imu_lpf_alpha", pitch_axis.imu_lpf_alpha, pitch_axis.imu_lpf_alpha);
    pnh.param("pitch_speed_scale_rpm_per_deg", pitch_axis.speed_scale_rpm_per_deg, pitch_axis.speed_scale_rpm_per_deg);
    pnh.param("pitch_min_speed_rpm", pitch_axis.min_speed_rpm, pitch_axis.min_speed_rpm);
    pnh.param("pitch_max_speed_rpm", pitch_axis.max_speed_rpm, pitch_axis.max_speed_rpm);
    pnh.param("pitch_target_slew_deg_per_s", pitch_axis.target_slew_deg_per_s, pitch_axis.target_slew_deg_per_s);
    pnh.param("pitch_speed_slew_rpm_per_s", pitch_axis.speed_slew_rpm_per_s, pitch_axis.speed_slew_rpm_per_s);
    pnh.param("pitch_motor_dir", pitch_axis.motor_dir, pitch_axis.motor_dir);
    pnh.param("pitch_level_offset_deg", pitch_axis.level_offset_deg, pitch_axis.level_offset_deg);
    pnh.param("pitch_position_offset_deg", pitch_axis.position_offset_deg, pitch_axis.position_offset_deg);

    pnh.param("roll_pid_kp", roll_pid_cfg.kp, roll_pid_cfg.kp);
    pnh.param("roll_pid_ki", roll_pid_cfg.ki, roll_pid_cfg.ki);
    pnh.param("roll_pid_kd", roll_pid_cfg.kd, roll_pid_cfg.kd);
    pnh.param("roll_pid_output_limit_deg", roll_pid_cfg.output_limit, roll_pid_cfg.output_limit);
    pnh.param("roll_pid_integral_limit", roll_pid_cfg.integral_limit, roll_pid_cfg.integral_limit);
    pnh.param("roll_pid_integral_separation_deg", roll_pid_cfg.integral_separation_deg, roll_pid_cfg.integral_separation_deg);
    pnh.param("roll_pid_anti_windup_gain", roll_pid_cfg.anti_windup_gain, roll_pid_cfg.anti_windup_gain);
    pnh.param("roll_pid_d_lpf_hz", roll_pid_cfg.d_lpf_hz, roll_pid_cfg.d_lpf_hz);

    pnh.param("pitch_pid_kp", pitch_pid_cfg.kp, pitch_pid_cfg.kp);
    pnh.param("pitch_pid_ki", pitch_pid_cfg.ki, pitch_pid_cfg.ki);
    pnh.param("pitch_pid_kd", pitch_pid_cfg.kd, pitch_pid_cfg.kd);
    pnh.param("pitch_pid_output_limit_deg", pitch_pid_cfg.output_limit, pitch_pid_cfg.output_limit);
    pnh.param("pitch_pid_integral_limit", pitch_pid_cfg.integral_limit, pitch_pid_cfg.integral_limit);
    pnh.param("pitch_pid_integral_separation_deg", pitch_pid_cfg.integral_separation_deg, pitch_pid_cfg.integral_separation_deg);
    pnh.param("pitch_pid_anti_windup_gain", pitch_pid_cfg.anti_windup_gain, pitch_pid_cfg.anti_windup_gain);
    pnh.param("pitch_pid_d_lpf_hz", pitch_pid_cfg.d_lpf_hz, pitch_pid_cfg.d_lpf_hz);

    roll_axis.imu_lpf_alpha = clamp_value(roll_axis.imu_lpf_alpha, 0.0, 1.0);
    pitch_axis.imu_lpf_alpha = clamp_value(pitch_axis.imu_lpf_alpha, 0.0, 1.0);

    if(roll_axis.error_deadband_deg < 0.0) roll_axis.error_deadband_deg = 0.0;
    if(pitch_axis.error_deadband_deg < 0.0) pitch_axis.error_deadband_deg = 0.0;

    if(roll_axis.error_deadband_exit_deg < roll_axis.error_deadband_deg) {
        roll_axis.error_deadband_exit_deg = roll_axis.error_deadband_deg;
    }
    if(pitch_axis.error_deadband_exit_deg < pitch_axis.error_deadband_deg) {
        pitch_axis.error_deadband_exit_deg = pitch_axis.error_deadband_deg;
    }

    opts.dynamic_imu_rate_ema_alpha = clamp_value(opts.dynamic_imu_rate_ema_alpha, 0.0, 1.0);
    opts.dynamic_imu_rate_send_hz_level_0 = clamp_value(opts.dynamic_imu_rate_send_hz_level_0, 0.1, std::max(0.1, opts.control_hz));
    opts.dynamic_imu_rate_send_hz_level_1 = clamp_value(opts.dynamic_imu_rate_send_hz_level_1, 0.1, std::max(0.1, opts.control_hz));
    opts.dynamic_imu_rate_send_hz_level_2 = clamp_value(opts.dynamic_imu_rate_send_hz_level_2, 0.1, std::max(0.1, opts.control_hz));
    opts.dynamic_imu_rate_send_hz_level_3 = clamp_value(opts.dynamic_imu_rate_send_hz_level_3, 0.1, std::max(0.1, opts.control_hz));

    if(opts.dynamic_imu_rate_threshold_1_deg_s < 0.0) opts.dynamic_imu_rate_threshold_1_deg_s = 0.0;
    if(opts.dynamic_imu_rate_threshold_2_deg_s < opts.dynamic_imu_rate_threshold_1_deg_s) {
        opts.dynamic_imu_rate_threshold_2_deg_s = opts.dynamic_imu_rate_threshold_1_deg_s;
    }
    if(opts.dynamic_imu_rate_threshold_3_deg_s < opts.dynamic_imu_rate_threshold_2_deg_s) {
        opts.dynamic_imu_rate_threshold_3_deg_s = opts.dynamic_imu_rate_threshold_2_deg_s;
    }
    if(opts.dynamic_imu_rate_force_resend_sec < 0.0) {
        opts.dynamic_imu_rate_force_resend_sec = 0.0;
    }

    try_enable_process_realtime(opts);

    struct ifreq ifr;
    struct sockaddr_can addr;
    ctrl.can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(ctrl.can_sock < 0) {
        ROS_ERROR("Failed to create CAN socket: %s", strerror(errno));
        return 1;
    }

    int sndbuf = 256 * 1024;
    int prio = 6;
    int loopback = 0;
    setsockopt(ctrl.can_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    setsockopt(ctrl.can_sock, SOL_SOCKET, SO_PRIORITY, &prio, sizeof(prio));
    setsockopt(ctrl.can_sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    strncpy(ifr.ifr_name, opts.can_interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if(ioctl(ctrl.can_sock, SIOCGIFINDEX, &ifr) < 0) {
        ROS_ERROR("Failed to get CAN interface index: %s", strerror(errno));
        close(ctrl.can_sock);
        return 1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(ctrl.can_sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        ROS_ERROR("Failed to bind CAN socket: %s", strerror(errno));
        close(ctrl.can_sock);
        return 1;
    }

    ROS_INFO("Initializing motors for profile position mode...");
    set_motor_enable(&ctrl, ROLL_MOTOR_ID, 1);
    set_motor_enable(&ctrl, PITCH_MOTOR_ID, 1);
    usleep(20000);

    if(opts.home_on_startup) {
        home_motors_to_zero(&ctrl, opts.home_speed_rpm, opts.home_accel_rpm_s,
                            opts.home_decel_rpm_s, opts.home_settle_time_sec);
    }

    configure_position_mode(&ctrl, ROLL_MOTOR_ID, opts.home_accel_rpm_s, opts.home_decel_rpm_s);
    configure_position_mode(&ctrl, PITCH_MOTOR_ID, opts.home_accel_rpm_s, opts.home_decel_rpm_s);

    pthread_t imu_thread;
    ImuThreadArgs imu_thread_args;
    imu_thread_args.ctrl = &ctrl;
    imu_thread_args.opts = &opts;

    if(pthread_create(&imu_thread, NULL, imu_thread_func, &imu_thread_args) != 0) {
        ROS_ERROR("Failed to create IMU thread");
        close(ctrl.can_sock);
        return 1;
    }

    ROS_INFO("Low-latency leveling started");
    ROS_INFO("  control_hz=%.1f  imu_timeout=%.3f  print_hz=%.1f",
             opts.control_hz, opts.imu_timeout_sec, opts.print_hz);
    ROS_INFO("  roll PID: kp=%.3f ki=%.3f kd=%.3f out_lim=%.2f I_lim=%.2f",
             roll_pid_cfg.kp, roll_pid_cfg.ki, roll_pid_cfg.kd, roll_pid_cfg.output_limit, roll_pid_cfg.integral_limit);
    ROS_INFO("  pitch PID: kp=%.3f ki=%.3f kd=%.3f out_lim=%.2f I_lim=%.2f",
             pitch_pid_cfg.kp, pitch_pid_cfg.ki, pitch_pid_cfg.kd, pitch_pid_cfg.output_limit, pitch_pid_cfg.integral_limit);

    if(opts.dynamic_imu_rate_send_rate_enable) {
        ROS_INFO("Dynamic IMU-rate-based send rate ENABLED");
        ROS_INFO("  EMA alpha=%.3f", opts.dynamic_imu_rate_ema_alpha);
        ROS_INFO("  thresholds(deg/s)=%.3f / %.3f / %.3f",
                 opts.dynamic_imu_rate_threshold_1_deg_s,
                 opts.dynamic_imu_rate_threshold_2_deg_s,
                 opts.dynamic_imu_rate_threshold_3_deg_s);
        ROS_INFO("  hz levels=%.1f / %.1f / %.1f / %.1f",
                 opts.dynamic_imu_rate_send_hz_level_0,
                 opts.dynamic_imu_rate_send_hz_level_1,
                 opts.dynamic_imu_rate_send_hz_level_2,
                 opts.dynamic_imu_rate_send_hz_level_3);
        ROS_INFO("  force resend sec=%.3f", opts.dynamic_imu_rate_force_resend_sec);
    }

    PidState roll_pid_state;
    PidState pitch_pid_state;
    pid_reset(&roll_pid_state);
    pid_reset(&pitch_pid_state);

    AxisRuntime roll_rt;
    AxisRuntime pitch_rt;
    memset(&roll_rt, 0, sizeof(roll_rt));
    memset(&pitch_rt, 0, sizeof(pitch_rt));

    roll_rt.target_cmd_deg = roll_axis.position_offset_deg;
    pitch_rt.target_cmd_deg = pitch_axis.position_offset_deg;
    roll_rt.last_sent_target_cmd_deg = roll_rt.target_cmd_deg;
    pitch_rt.last_sent_target_cmd_deg = pitch_rt.target_cmd_deg;
    roll_rt.last_sent_active = false;
    pitch_rt.last_sent_active = false;

    const uint64_t period_ns = static_cast<uint64_t>(1e9 / opts.control_hz);
    uint64_t next_tick_ns = monotonic_time_ns();
    uint64_t last_loop_ns = next_tick_ns;
    uint64_t last_print_ns = next_tick_ns;
    double print_period_ns = (opts.print_hz > 0.0) ? (1e9 / opts.print_hz) : 1e9;

    while(ros::ok()) {
        next_tick_ns += period_ns;

        struct timespec ts;
        ts.tv_sec = static_cast<time_t>(next_tick_ns / 1000000000ULL);
        ts.tv_nsec = static_cast<long>(next_tick_ns % 1000000000ULL);
        int sleep_ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        if(sleep_ret != 0 && sleep_ret != EINTR) {
            ROS_WARN_THROTTLE(1.0, "clock_nanosleep returned %d", sleep_ret);
        }

        uint64_t now_ns = monotonic_time_ns();
        double dt = static_cast<double>(now_ns - last_loop_ns) * 1e-9;
        last_loop_ns = now_ns;
        if(dt <= 1e-6) dt = 1.0 / opts.control_hz;

        double imu_age = monotonic_time_sec() - ctrl.imu_timestamp_sec.load(std::memory_order_relaxed);
        bool imu_fresh = ctrl.imu_ready.load(std::memory_order_relaxed) && (imu_age <= opts.imu_timeout_sec);

        double roll_meas = ctrl.roll.load(std::memory_order_relaxed);
        double pitch_meas = ctrl.pitch.load(std::memory_order_relaxed);

        double roll_pid_output = 0.0;
        double pitch_pid_output = 0.0;
        double roll_error = 0.0;
        double pitch_error = 0.0;

        if(imu_fresh) {
            update_axis_runtime(&roll_axis, &roll_pid_cfg, &roll_pid_state, &roll_rt, roll_meas, dt, &roll_pid_output, &roll_error);
            update_axis_runtime(&pitch_axis, &pitch_pid_cfg, &pitch_pid_state, &pitch_rt, pitch_meas, dt, &pitch_pid_output, &pitch_error);
        } else {
            pid_reset(&roll_pid_state);
            pid_reset(&pitch_pid_state);
            roll_rt.active = false;
            pitch_rt.active = false;
            roll_rt.speed_cmd_rpm = 0.0;
            pitch_rt.speed_cmd_rpm = 0.0;
        }

        // 用滤波后的 IMU 角度差分估计角速度，再做 EMA
        update_axis_send_scheduler_by_imu_rate(&roll_rt, &opts, roll_rt.filtered_measurement, dt);
        update_axis_send_scheduler_by_imu_rate(&pitch_rt, &opts, pitch_rt.filtered_measurement, dt);

        bool send_roll = should_send_axis_adaptive(roll_rt, opts);
        bool send_pitch = should_send_axis_adaptive(pitch_rt, opts);

        if(send_roll) {
            set_motor_speed(&ctrl, ROLL_MOTOR_ID, roll_rt.speed_cmd_rpm);
            set_motor_position(&ctrl, ROLL_MOTOR_ID, degrees_to_pulses(roll_rt.target_cmd_deg));
            mark_axis_sent(&roll_rt, &opts);
        }
        if(send_pitch) {
            set_motor_speed(&ctrl, PITCH_MOTOR_ID, pitch_rt.speed_cmd_rpm);
            set_motor_position(&ctrl, PITCH_MOTOR_ID, degrees_to_pulses(pitch_rt.target_cmd_deg));
            mark_axis_sent(&pitch_rt, &opts);
        }

        if(static_cast<double>(now_ns - last_print_ns) >= print_period_ns) {
            last_print_ns = now_ns;

            if(opts.dynamic_imu_rate_send_rate_enable) {
                printf("IMU:%s age=%6.3fms pkt=%d err=%d | "
                       "Meas R=%7.3f P=%7.3f | "
                       "Err R=%7.3f P=%7.3f | "
                       "PID R=%7.3f P=%7.3f | "
                       "CmdPos R=%7.3f P=%7.3f | "
                       "CmdSpd R=%7.3f P=%7.3f | "
                       "RateEMA R=%7.3f P=%7.3f deg/s | "
                       "Hz R=%5.1f P=%5.1f\n",
                       imu_fresh ? "OK" : "STALE",
                       imu_age * 1000.0,
                       ctrl.imu_packets.load(std::memory_order_relaxed),
                       ctrl.imu_parse_errors.load(std::memory_order_relaxed),
                       roll_rt.filtered_measurement, pitch_rt.filtered_measurement,
                       roll_error, pitch_error,
                       roll_pid_output, pitch_pid_output,
                       roll_rt.target_cmd_deg, pitch_rt.target_cmd_deg,
                       roll_rt.speed_cmd_rpm, pitch_rt.speed_cmd_rpm,
                       roll_rt.imu_rate_ema_deg_s, pitch_rt.imu_rate_ema_deg_s,
                       roll_rt.current_send_hz, pitch_rt.current_send_hz);
            } else {
                printf("IMU:%s age=%6.3fms pkt=%d err=%d | "
                       "Meas R=%7.3f P=%7.3f | "
                       "Err R=%7.3f P=%7.3f | "
                       "PID R=%7.3f P=%7.3f | "
                       "CmdPos R=%7.3f P=%7.3f | "
                       "CmdSpd R=%7.3f P=%7.3f\n",
                       imu_fresh ? "OK" : "STALE",
                       imu_age * 1000.0,
                       ctrl.imu_packets.load(std::memory_order_relaxed),
                       ctrl.imu_parse_errors.load(std::memory_order_relaxed),
                       roll_rt.filtered_measurement, pitch_rt.filtered_measurement,
                       roll_error, pitch_error,
                       roll_pid_output, pitch_pid_output,
                       roll_rt.target_cmd_deg, pitch_rt.target_cmd_deg,
                       roll_rt.speed_cmd_rpm, pitch_rt.speed_cmd_rpm);
            }
            fflush(stdout);
        }
    }

    ctrl.running.store(false, std::memory_order_relaxed);
    pthread_join(imu_thread, NULL);
    close(ctrl.can_sock);
    return 0;
}