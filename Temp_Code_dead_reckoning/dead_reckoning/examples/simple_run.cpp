#include <iostream>
#include <memory>
#include <thread>
#include <fstream>
#include <cmath>
#include "src/dead_reckoning_core.h"

using namespace Magna::dead_reckoning;

int main() {
    std::cout << "初始化 DeadReckoningCore..." << std::endl;
    DeadReckoningCore core;
    core.Init();

    // CSV 输出文件
    const std::string csv_path = "examples/run_output.csv";
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        std::cerr << "无法打开输出 CSV: " << csv_path << std::endl;
        return 1;
    }
    // CSV header
    csv << "time,x,y,heading (rad),speed" << std::endl;

    // 仿真参数
    double t = 0.0;
    const double dt = 0.05;          // 50 ms 步长
    const int steps = 400;           // 总时长 = steps * dt

    // 预先分配 localization 输出结构并在每步传入
    auto localization = std::make_shared<LocalizationOutput>();

    double x = 0.0, y = 0.0; // 初始化位移
    for (int i = 0; i < steps; ++i) {
        auto chassis = std::make_shared<ChassisData>();
        chassis->timestamp = t;

        // 线速度剖面
        double target_speed = 0.0;
        if (i < 100) {
            target_speed = 2.0 * (double(i) / 100.0); // 0 -> 2.0
        } else if (i < 300) {
            target_speed = 2.0;
        } else {
            target_speed = 2.0 * (1.0 - double(i - 300) / 100.0); // 2.0 -> 0
            if (target_speed < 0.0) target_speed = 0.0;
        }

        // 设置轮速为近似的车速（简化：四轮相同）
        chassis->has_wheel_speed = true;
        chassis->wheel_speed.fl_wheel_speed = target_speed;
        chassis->wheel_speed.fr_wheel_speed = target_speed;
        chassis->wheel_speed.rl_wheel_speed = target_speed;
        chassis->wheel_speed.rr_wheel_speed = target_speed;

        // 角速度剖面（rad/s）: 在 150..250 步期间转弯，angle_rate 为 0.5 rad/s
        double yaw_rate = 0.0;
        if (i >= 150 && i < 250) yaw_rate = 0.5; // 约 28.6 deg/s
        chassis->has_yaw_rate = true;
        chassis->yaw_rate = yaw_rate;

        // 调用核心处理；RunOnce 接受 (chassis, localization)
        core.RunOnce(chassis, localization);

        // 使用速度和航向角进行积分生成位移
        double dx = target_speed * std::cos(localization->heading) * dt;
        double dy = target_speed * std::sin(localization->heading) * dt;
        x += dx;
        y += dy;

        // 写入 CSV
        csv << t << "," << x << "," << y << "," << localization->heading << "," << target_speed << std::endl;

        // 少量终端输出以便实时观察（每 10 步）
        if (i % 10 == 0) {
            printf("t=%.3f pos=(%.3f,%.3f) heading=%.3f speed=%.3f\n", t, x, y, localization->heading, target_speed);
        }

        t += dt;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    csv.close();
    std::cout << "CSV 已写入: " << csv_path << std::endl;
    return 0;
}
