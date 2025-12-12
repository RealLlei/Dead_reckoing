# Dead Reckoning 项目使用与代码逻辑手册（中文）

本手册基于仓库当前代码生成，覆盖环境准备、构建、运行、代码结构、核心算法与调试建议，面向 0 基础读者。

说明
- 目标：此仓库实现了车辆的死算（Dead Reckoning）模块核心代码，使用车轮速度、方向盘角度、IMU 等传感器数据估计车辆位置、速度与朝向。
- 适用人群：对 C/C++、基础线性代数、卡尔曼滤波没有预备知识的读者也能通过本手册逐步理解并运行代码。
- 当前仓库状态：为便于在不依赖 Apollo 等外部库的环境下编译，代码里包含了若干“兼容 stub（替代头）”与 CMake 的最小构建模式。最终产物在 minimal 模式下会把 `src` 编译为对象文件集合（未做外部库链接）。

目录（本手册结构）
1. 环境准备（必备软件与库）
2. 构建与运行（最小构建、完整构建的说明与示例命令）
3. 目录与主要文件简介（逐文件功能）
4. 核心数据结构与类型（状态向量、传感器数据结构）
5. DeadReckoningCore 类详解（成员、生命周期、关键函数）
6. 算法与数据流（预测、校正、EKF 工作流程）
7. 日志、配置与参数（如何调整、重要标志说明）
8. 编译/运行常见问题与排查
9. 扩展、替换真实依赖与学习资源
10. 快速上手示例（从无到会：构建、运行单次、观察输出）

---

## 1. 环境准备（面向 0 基础）
必备软件（最低列表）
- 操作系统：Linux（说明中使用 bash）
- 编译器：g++ 支持 C++17（例如 g++ 7+）
- 构建工具：CMake（3.x 系列）
- 头文件依赖：Eigen（线性代数库），通常系统路径为 `/usr/include/eigen3`
	- 在本仓库中，CMake 已将 `/usr/include/eigen3` 加入 include path（如系统缺失，请安装：ubuntu apt install libeigen3-dev）
- 其他：make / ninja（由 CMake 生成后端）

说明
- 原始项目依赖 Apollo、protobuf、gflags、glog 等库。当前仓库包含最小兼容性替代（stub），可以在不安装 Apollo 系列的机器上编译 `src`。
- 如果需要最终链接成完整可运行的库或二进制，需要安装或替换对应外部库；这超出“最小编译”的范围（手册第9节给出替换建议）。

---

## 2. 构建与运行

构建类型说明
- BUILD_MINIMAL=ON（默认）：仅编译项目源码为对象集合（OBJECT library），避免链接到外部 Apollo/gflags/glog/protobuf 等。适合快速编译与代码阅读、单元测试开发。
- BUILD_MINIMAL=OFF：尝试完整构建并链接（会尝试寻找并链接外部库）；若机器上未安装这些库会失败。

快速命令（在仓库根目录）
- 创建构建目录并进行最小构建（推荐初次使用）：
	```bash
	mkdir -p build_dir
	cmake -S . -B build_dir -DBUILD_MINIMAL=ON
	cmake --build build_dir -j$(nproc)
	```
	说明：成功后你会看到构建 target `europa_dead_reckoning_objects`，这表示 `src` 的源文件已被编译为对象文件集合。

- 尝试完整构建（如果你已安装所有外部依赖）：
	```bash
	mkdir -p build_full
	cmake -S . -B build_full -DBUILD_MINIMAL=OFF
	cmake --build build_full -j$(nproc)
	```
	如果缺失库，cmake configure 或 build 阶段会报错，提示无法找到包或库；参考第9节替换或安装库。

运行
- 仓库当前未提供直接可执行的“主程序”将所有 runtime 依赖串联运行（因为原始环境依赖大量 runtime 框架）。但你可以：
	- 读取 `build_dir` 下的对象文件进行单元测试。
	- 在本仓库内编写小的驱动程序（main），创建一个 DeadReckoningCore 实例，注入模拟传感器数据并调用 `RunOnce` 或相似接口来验证行为（第10节给出示例代码片段）。

---

## 3. 目录与主要文件简介
（按子目录或文件分组说明关键文件的目的与作用；此处列出并逐一说明）

仓库顶层
- `CMakeLists.txt`
	- 项目构建配置。已加入 `BUILD_MINIMAL` 选项；默认会把源码编译为 OBJECT 库以避免链接外部库。
- `main.cpp`
	- 项目入口（如果存在），但可能依赖外部消息系统或 Apollo 框架；在 minimal 模式下未必可直接运行。
- `run_dead_reckoning.sh`
	- 运行脚本（若存在运行时依赖会在执行时报错）。可作为参考如何组合命令行参数与配置。

src/
- `src/dead_reckoning_core.cc`
	- 核心实现文件，包含 DeadReckoningCore 类的成员函数实现：初始化（Init）、单步处理（RunOnce / StateEstimateEkf / LocalPredict / LocalCarSpeedCorrect / PublishLocalization）等。
- `src/dead_reckoning_core.h`
	- 核心头文件，定义主要数据结构（配置、车辆状态向量、传感器数据类型）与 DeadReckoningCore 类接口。这个头被清理并简化以便本地编译，包含必要的成员和前置类型。
- `src/dead_reckoning_gflags.h` / `src/dead_reckoning_gflags.cc`
	- 配置与标志定义。原本依赖 gflags；仓库内使用了兼容性替代实现（将 DECLARE_* 替换为 extern 声明，并用简单 DEFINE_* 宏在 `.cc` 中定义 FLAGS_ 变量）。
- `src/compat.h`
	- 兼容层，提供最小 logging 宏（AINFO/ADEBUG/AERROR）和 DEFINE_* 宏等，目的在于避免真实 glog/gflags 的依赖。对阅读代码的人来说，理解这里是关键：这些只是占位实现，真实系统中会被真正的日志库替换。

common/
- `common/kalmanfilter.h` / `common/kalmanfilter.cc`
	- 卡尔曼滤波器实现与封装，包含 KalmanFilterCorrector（或类似命名）用于状态估计更新步骤。
- `common/util.h` / `common/util.cc`
	- 工具函数（数学/时间/数值稳定性处理等）。
- `common/structs.h`
	- 一些共享的结构类型定义（例如消息结构、传输格式），在本仓库中某些路径存在 wrapper 以满足包括路径如 `dead_reckoning/common/*`。

conf/
- `conf/dead_reckoning/*`
	- 配置示例文件（protobuf 文本、yaml、conf），用于切换参数。可直接编辑用于运行时参数调整（例如卡尔曼滤波参数、是否使用 IMU 等）。

launch/
- `dead_reckoning.launch`
	- 如果在 ROS 或 Apollo 环境中，这个文件帮助启动节点；在最小环境下通常不使用。

dag/
- `dead_reckoning.dag`
	- 如果项目使用 dag 任务或容器编排，这里有任务图；一般阅读参考即可。

---

## 4. 核心数据结构与类型（面向零基础）
为简单起见，把复杂概念分解为基本要素。

4.1 状态向量与常量
- STATE_NUMBER：表示状态向量的维度（例如位置、速度、偏航角、偏置等）。
- 一般状态包含：
	- x, y：平面位置坐标（米）
	- v：车速（m/s）
	- yaw（psi）：航向（弧度）
	- bias/gyro_bias：陀螺偏置（用于 IMU 零漂校正）
	- 其他扩展状态（轮速比例、横摆角速偏差等）

4.2 传感器数据结构（在 `src/dead_reckoning_core.h`）
- WheelSpeedData：车轮传感器速度信息（前后左右轮速度、方向位标志）。字段示例：
	- fl_wheel_speed, fr_wheel_speed, rl_wheel_speed, rr_wheel_speed（原始读数 / 编码器值或已换算为速度）
	- fl_dir, fr_dir, rl_dir, rr_dir（轮子方向：正转/反转）
- ImuData：IMU 数据（角速度、线加速度、时间戳）
	- 通常包含：gyro_z（偏航角速）、acc_x、acc_y、acc_z、timestamp
- SteerData：方向盘角度与角速（steer_angle, steer_angle_rate）
- ChassisData：车辆底盘总体状态（含速度、档位、里程计、是否静止）
- LocalizationOutput：模块最终输出（位置信息、朝向、速度、协方差/置信度）

4.3 DeadReckoningConfig（配置）
- 包含若干参数，例如：
	- use_imu（是否启用 IMU 修正）
	- wheel_radius（车轮半径）
	- neta_forward_coef / neta_backward_coef（轮胎/驱动相关比例系数）
	- rear_wheel_rot_arm（后轮旋转臂长）
	- enable_auto_angle_rate_bias（是否自动估计角速偏置）
	- 以及若干用于滤波器与置信度计算的增益参数
- 这些配置可以在 `conf/` 下的 yaml 或 conf 文件中调整。调整配置对估计的稳定性与响应有显著影响。

---

## 5. DeadReckoningCore 类详解（生命周期与关键函数）
5.1 目的
- DeadReckoningCore 负责接收传感器输入（轮速、IMU、转向角等），维护状态向量及协方差（P），并在每次数据到来时执行预测与更新步骤，输出本地化估计（LocalizationOutput）。

5.2 重要成员（部分、简化）
- double last_msg_time_：上次处理消息的时间戳
- Eigen::VectorXd x_：状态向量（长度 STATE_NUMBER）
- Eigen::MatrixXd P_：状态协方差矩阵
- Eigen::Vector3d pos_：当前估计位置（x,y,z）
- Eigen::Quaterniond q_：当前估计姿态（四元数）
- KalmanFilterCorrector car_speed_corrector_ / yaw_cov_corrector_：用于速度与偏航协方差的辅助滤波器/校正器
- gyro_bias_, acc_bias_：IMU 偏置估计
- DeadReckoningConfig config_：模块参数

5.3 重要方法（调用顺序示例）
- Init(const DeadReckoningConfig& conf)
	- 初始化状态、协方差与滤波器参数。读取 conf 并设置初始 P_、x_ 初值与一些阈值。
- RunOnce(const WheelSpeedData& ws, const ImuData& imu, const SteerData& steer, double timestamp)
	- 单次处理入口。一条完整的数据流可能包含轮速、IMU、转向传感器的联合数据。
	- 内部一般做：
		1. 计算时间差 dt = timestamp - last_msg_time_
		2. 如果 dt 非常大或零，做边界处理（例如只初始化或丢弃）
		3. 调用 LocalPredict(dt, ws, steer) 进行状态预测（基于车速、转向角推导运动学模型）
		4. 若可用，调用 StateEstimateEkf(imu, ws, steer) 进行 EKF 校正（如果 IMU/里程可用，则更新）
		5. 调用 LocalCarSpeedCorrect(...) 对车速进行非线性校正（轮速与车速关系）
		6. 生成 LocalizationOutput 并 PublishLocalization(...)
- LocalPredict(double dt, ...)
	- 基于运动学模型（例如单摆或自行车模型）对状态做时间推进：
		- x_{k+1} = f(x_k, u_k, dt)
		- P = F*P*F^T + Q（Q：过程噪声）
	- 该函数也可估计协方差的传播（F：雅可比矩阵）
- StateEstimateEkf(...)
	- 当传感器测量可用（例如 IMU 或 GPS），利用观测模型进行卡尔曼增益计算并更新状态与协方差：
		- y = z - h(x)
		- S = H*P*H^T + R
		- K = P*H^T*S^{-1}
		- x = x + K*y
		- P = (I - K*H)*P
- LocalCarSpeedCorrect(...)
	- 对来自轮速的速度估计做非线性修正（考虑打滑、轮胎半径、方向差等）
- PublishLocalization(...)
	- 封装并输出 LocalizationOutput，可能写入日志、发送消息或存文件（取决于运行环境）

备注
- 各函数内部常见步骤包括：时间同步、有效性检查（例如轮速是否异常）、限幅（例如最大转向角速度）、异常处理（例如检测静止并重置一部分状态）等。
- 误差协方差 P 的正确初始化对卡尔曼滤波稳定性至关重要；如果初值过小，滤波器会对观测不敏感；过大则会过度抖动。

---

## 6. 算法与数据流（深入理解 EKF 和模块流程）
为 0 基础读者把 EKF 分解为“易懂”的步骤。

6.1 基础概念（非常简短）
- 目标：在噪声传感器输入下估计未知变量（位置、速度、朝向）。
- EKF（扩展卡尔曼滤波）用于非线性系统的状态估计。它通过在当前线性化点计算雅可比矩阵将非线性问题转化为线性卡尔曼滤波步骤。

6.2 预测步骤（Predict）
- 输入：上次状态 x_k，控制/运动输入 u_k（例如由轮速与转向角计算的车速与转向率），时间差 dt
- 输出：预测状态 x_{k+1|k}，预测协方差 P_{k+1|k}
- 数学：
	- x_pred = f(x, u, dt)  （运动学或动力学模型）
	- F = ?f/?x（在 x 点的雅可比）
	- P_pred = F * P * F^T + Q（过程噪声 Q 表示模型不确定性）
- 代码实现要点：
	- 使用小角度近似或完整三角函数，取决于角度范围
	- 对 dt 做限幅，避免过大步长导致数值不稳定

6.3 更新步骤（Update）
- 当有观测 z（例如 IMU、编码器修正或外部位置定位）时进行：
	- 计算观测预测 h(x_pred)
	- 计算观测雅可比 H = ?h/?x
	- 计算创新 y = z - h(x_pred)
	- S = H * P_pred * H^T + R（R：观测噪声协方差）
	- K = P_pred * H^T * S^{-1}（卡尔曼增益）
	- x_upd = x_pred + K * y
	- P_upd = (I - K*H) * P_pred
- 代码实现要点：
	- 要谨慎处理角度差（例如航向差要做 wrap-around，保持在 [-π, π]）
	- 使用稳定的矩阵求逆或分解（例如 Cholesky）

6.4 参数调试要点
- Q 与 R 的选择：Q 越大，意味着对模型不信任，滤波器更依赖观测；R 越大，则滤波器更信任模型预测。通过实验调整以获得平稳与响应之间的平衡。
- 初始 P_：影响收敛速度与稳定性。通常位置初值不确定性较大，速度/偏置初值可以小一些。
- 数据失真与异常：要增加检测逻辑在发现传感器异常时暂时减小更新或忽略异常观测。

6.5 与车辆运动模型的关系
- 单车模型或自行车模型通常用于预测：
	- 基于车速 v、转向角 δ 计算角速度 ψ_dot = v * tan(δ) / L（L 为轴距）
	- x_dot = v * cos(ψ), y_dot = v * sin(ψ)
- 轮速与车速映射可能受轮胎半径、滑移、传动比影响，这些通过 `config_` 中的系数做修正（neta_forward_coef、neta_backward_coef 等）。

---

## 7. 日志、配置与参数
7.1 日志
- 仓库使用了一个最小替代的 `compat.h`，提供 AINFO/ADEBUG/AERROR 等宏用于打印。
- 在真实部署中应替换为 glog 或您组织的日志库。

7.2 配置文件
- `conf/dead_reckoning` 中提供了示例参数文件（yaml / conf / pb.txt）。关键参数：
	- 是否启用 imu 修正（use_imu）
	- 轮半径（wheel_radius）
	- 初始协方差大小（可在源码或 conf 中指定）
	- 角速偏置估计开关（enable_auto_angle_rate_bias）
- 使用方法：把需要的配置复制到运行环境，并在运行时加载或在 main 中传入。

7.3 如何调整参数以快速试验
- 小步迭代：调整一个参数，运行一个短时间序列的合成数据或记录，观察输出变化。
- 建议实验项：Q、R、初始 P、是否启用 IMU、轮速转速系数 neta_forward/backward。
- 记录：每次实验修改前保存配置副本便于对比。

---

## 8. 编译/运行常见问题与排查（实用排错清单）
8.1 构建失败（找不到头文件或外部库）
- 错误示例：找不到 `gflags/gflags.h` 或 `google/protobuf` 等
	- 原因：未安装外部依赖或 BUILD_MINIMAL=ON 被设为 OFF。
	- 解决：使用最小模式编译（DBUILD_MINIMAL=ON），或安装必要库（例如 apt install libgflags-dev libgoogle-glog-dev libprotobuf-dev）。
- 错误示例：找不到 Eigen
	- 解决：apt install libeigen3-dev，或从源码安装；或修改 CMakeLists.txt 指向 Eigen 安装路径。

8.2 链接失败（undefined reference）
- 原因：调用了外部库函数但未链接对应库（例如 glog、protobuf、europa_common 等）。
- 解决：安装库并在 CMake 中找到它们，或继续使用 minimal 模式；若需要完整链接，需将这些依赖逐个满足或提供本地替代实现。

8.3 运行时报 NaN 或 发散的估计值
- 原因：Q/R 配置不合适、协方差初始化不当、时间戳不连续或 dt 过大、数值不稳定（矩阵不可逆）。
- 解决：
	- 检查输入数据时间戳，确保单调递增且 dt 合理。
	- 检查初始 P_ 是否太小或太大。
	- 打印中间矩阵（S、K、P）查看是否异常。加入更多日志。
	- 使用小 dt 单步调试找到什么时候发生 NaN。

8.4 角度差处理错误（跳变）
- 问题：航向角变化跨越 π 导致观测创新很大。
- 解决：对角度差进行 wrap-around，确保差值落在 [-π, π]。代码中检查并修正角度差的函数通常位于 util 或 core 中。

8.5 传感器数据时序与不同频率
- 说明：IMU、轮速编码器、转向角等传感器频率不同，需要在 RunOnce 或上游做时间同步/插值。
- 建议：在进入 EKF 之前对异频数据做插值或使用缓冲队列，确保每次预测使用的控制输入是对齐的。

---

## 9. 后续改进与如何替换 stub 为真实依赖
如果你要把项目移回真实环境或在生产中使用，建议以下步骤：

9.1 替换日志与 gflags
- 把 `src/compat.h` 中的宏替换为真实 `glog` 或 `spdlog` 调用。
- 用原始 gflags 库替换简易 DEFINE_* 宏，并在 `CMakeLists.txt` 中寻找 gflags（find_package(GFlags REQUIRED)）。

9.2 替换消息/接口
- 原始项目可能依赖 Apollo 的消息类型（例如 `LocalizationEstimate`），请把这些替换回项目中使用的消息类型或定义合适的序列化/反序列化方案（protobuf/flatbuffers/ROS msg）。
- 确保 `dead_reckoning_gflags.h` 中 extern 的 FLAGS_* 与实际 gflags 声明匹配。

9.3 链接与依赖管理
- 更新 `CMakeLists.txt` 添加 find_package() 调用并链接真实库（glog, gflags, protobuf, euopa_common 等）。
- 如果使用 ROS/Apollo 框架，则按对应框架推荐的构建系统（catkin, bazel）调整。

9.4 增加测试套件
- 增加单元测试（例如使用 GoogleTest）：
	- 1）构建 KalmanFilter 的单元测试（已知输入→验证输出）
	- 2）构建 DeadReckoningCore 的仿真测试：合成真值轨迹，添加噪声传感器模拟，检查估计误差随时间变化
- 自动化：在 CI（GitHub Action）中加入编译与单元测试步骤。

---

## 10. 快速上手示例（小驱动程序）
下面给出一个非常简化的思路：写一个小 main，构造 DeadReckoningCore，传入模拟的数据并调用 RunOnce 来观察输出。伪代码说明（你可以把它实现为 `examples/simple_run.cpp`）：

伪代码（思路）
- 创建 DeadReckoningConfig conf，设置 wheel_radius=0.3, use_imu=true 等
- DeadReckoningCore core; core.Init(conf)
- 循环 100 次：
	- 构造 WheelSpeedData ws（根据设定速度填写四轮速度）
	- 构造 ImuData imu（角速、加速度带少量噪声）
	- 构造 SteerData steer（小角度）
	- double t = start_time + i*dt
	- core.RunOnce(ws, imu, steer, t)
	- 读取 core 输出（或调用 PublishLocalization，或打印 core.pos_ / core.yaw 等）

运行示例
- 把文件加入 CMakeLists.txt，构建并执行。如果 minimal 模式下编译通过但链接失败（外部依赖），尽量只编译并在同一进程中链接本地对象（以 object 库或手动 g++ 链接方式测试）。

---

## 附：常见代码阅读点（快速定位文件中的关键函数）
- 想看 EKF 如何写：打开 `src/dead_reckoning_core.cc`，查找 `StateEstimateEkf`、`LocalPredict`、`LocalCarSpeedCorrect`。
- 想看卡尔曼滤波具体实现：打开 `common/kalmanfilter.cc`。
- 配置参数点：`src/dead_reckoning_gflags.cc` 与 `conf/dead_reckoning/*`。

---

## 结语与后续建议
- 已完成：一份面向 0 基础读者的完整手册，覆盖环境、构建、代码结构、核心算法与调试建议。
- 建议下一步：
	- 如果你想，我可以：
		1. 为你生成一个 `examples/simple_run.cpp` 的完整实现并加入 CMake，用最小依赖（只用 Eigen）做本地运行示例；
		2. 或者把 `compat.h` 更接近真实 glog/gflags 的接口（逐步替换），并配置 CMake 以支持切换回真实依赖。
- 如果你要继续：告诉我你偏好（生成示例运行程序 / 改进 stub / 尝试完整链接），我将基于当前仓库立刻动手实现下一步。

完成情况说明（与仓库当前状态对照）
- 本手册基于仓库内的当前文件与之前的构建尝试撰写。说明中提及的“最小兼容 stub”与 BUILD_MINIMAL 模式是当前仓库为了脱离 Apollo 依赖所做的改动；若你在另一台机器上复现，请确保 `CMakeLists.txt` 的 BUILD_MINIMAL 默认为 ON，或在 cmake 调用时指定 -DBUILD_MINIMAL=ON。

如果需要，我可以把本手册另存为 Word（.docx）格式并放到仓库（注意：生成 docx 需额外库，若你接受我可以生成并保存为 markdown 与同时提供转换步骤）。
