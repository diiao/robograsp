#include <math.h>
#include "aubo_sdk/rpc.h"
#include <fstream>
#ifdef _WIN32
#include <Windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define EMBEDDED
std::ofstream file("force.csv", std::ios::app);

// 实现阻塞功能: 当机械臂运动到目标路点时，程序再往下执行
int waitArrival(RobotInterfacePtr impl)
{
    int cnt = 0;
    while (impl->getMotionControl()->getExecId() == -1) {
        if (cnt++ > 5) {
            std::cout << "Motion fail!" << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    auto id = impl->getMotionControl()->getExecId();
    while (1) {
        auto id1 = impl->getMotionControl()->getExecId();
        if (id != id1) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const std::vector<T> &list)
{
    for (size_t i = 0; i < list.size(); i++) {
        os << list.at(i);
        if (i != (list.size() - 1)) {
            os << ",";
        }
    }
    return os;
}

// 计算两点之间的欧氏距离，默认第二点为原点
inline double calculateDistance(
    const std::vector<double> &p1,
    const std::vector<double> &p2 = std::vector<double>(6, 0))
{
    if ((6 == p1.size()) && (6 == p2.size())) {
        double sum = 0.;
        for (int i = 0; i < 3; i++) {
            sum += pow(p1[i] - p2[i], 2);
        }
        return std::sqrt(sum);
    }
    return 0;
}

// 读取力传感器数据并保存到force.csv文件中
void tcpSensorTest(RpcClientPtr cli)
{
    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();
#ifdef EMBEDDED
    // 设置力传感器类型为内置传感器
    std::vector<double> sensor_pose = { 0, 0, 0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("embedded");
#else
    // 设置力传感器类型为外置坤维传感器
    std::vector<double> sensor_pose = { 0, 0, 0.047, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("kw_ftsensor");
#endif
    while (1) {
        auto sensor_data = cli->getRobotInterface(robot_name)
                               ->getRobotState()
                               ->getTcpForceSensors();

        std::cout << "--------------------------------------" << std::endl;
        for (int i = 0; i < sensor_data.size(); i++) {
            std::cout << "获取TCP力传感器读数: " << i + 1 << ": "
                      << sensor_data[i] << std::endl;
        }
        std::cout << "--------------------------------------" << std::endl;
        file << sensor_data << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// 力传感器标定
ForceSensorCalibResult tcpSensorcalibration(
    RpcClientPtr cli, std::vector<std::vector<double>> joints)
{
    auto robot_name = cli->getRobotNames().front();

    printf("goto p0\n");
    // 关节运动到负载辨识的第一个参考点
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(joints[0], 10 * (M_PI / 180), 5 * (M_PI / 180), 0, 0);
    // 阻塞
    waitArrival(cli->getRobotInterface(robot_name));
    // 等待机器人停稳
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 记录第一个参考点的关节位置和TCP力传感器数据
    auto q1 = cli->getRobotInterface(robot_name)
                  ->getRobotState()
                  ->getJointPositions();
    auto tcp_force1 = cli->getRobotInterface(robot_name)
                          ->getRobotState()
                          ->getTcpForceSensors();

    printf("goto p1\n");
    // 关节运动到负载辨识的第二个参考点
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(joints[1], 20 * (M_PI / 180), 10 * (M_PI / 180), 0, 0);
    // 阻塞
    waitArrival(cli->getRobotInterface(robot_name));
    // 等待机器人停稳
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 记录第二个参考点的关节位置和TCP力传感器数据
    auto q2 = cli->getRobotInterface(robot_name)
                  ->getRobotState()
                  ->getJointPositions();
    auto tcp_force2 = cli->getRobotInterface(robot_name)
                          ->getRobotState()
                          ->getTcpForceSensors();

    printf("goto p2\n");
    // 关节运动到负载辨识的第三个参考点
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(joints[2], 20 * (M_PI / 180), 10 * (M_PI / 180), 0, 0);
    // 阻塞
    waitArrival(cli->getRobotInterface(robot_name));
    // 等待机器人停稳
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 记录第三个参考点的关节位置和TCP力传感器数据
    auto q3 = cli->getRobotInterface(robot_name)
                  ->getRobotState()
                  ->getJointPositions();
    auto tcp_force3 = cli->getRobotInterface(robot_name)
                          ->getRobotState()
                          ->getTcpForceSensors();

    // 正解获得三个参考点的位姿
    auto pose1 = cli->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->forwardKinematics(q1);

    auto pose2 = cli->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->forwardKinematics(q2);

    auto pose3 = cli->getRobotInterface(robot_name)
                     ->getRobotAlgorithm()
                     ->forwardKinematics(q3);

    std::vector<std::vector<double>> calib_forces{ tcp_force1, tcp_force2,
                                                   tcp_force3 };
    std::vector<std::vector<double>> calib_poses{ std::get<0>(pose1),
                                                  std::get<0>(pose2),
                                                  std::get<0>(pose3) };
    // 力传感器标定算法(三点标定法)
    // 将三个参考点的传感器数据与位姿传入到此接口中做负载辨识
    auto result = cli->getRobotInterface(robot_name)
                      ->getRobotAlgorithm()
                      ->calibrateTcpForceSensor(calib_forces, calib_poses);
    return result;
}

void exampleForceControl(RpcClientPtr cli)
{
    auto robot_name = cli->getRobotNames().front();
#ifdef EMBEDDED
    // 设置力传感器类型为内置传感器
    std::vector<double> sensor_pose = { 0, 0, 0, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("embedded");
#else
    // 设置力传感器类型为外置坤维传感器
    std::vector<double> sensor_pose = { 0, 0, 0.047, 0, 0, 0 };
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->selectTcpForceSensor("kw_ftsensor");
#endif

    // 设置传感器安装位姿
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceSensorPose(sensor_pose);

    // 设置TCP偏移
    std::vector<double> tcp_pose = sensor_pose;
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpOffset(tcp_pose);

    // 负载辨识参考点
    std::vector<double> joint1 = { -0.261799, 0.261799, 1.309,
                                   1.0472,    1.39626,  0.0 };
    std::vector<double> joint2 = { -0.628319, 0.471239, 1.65806,
                                   -0.471239, 0.0,      0.0 };
    std::vector<double> joint3 = { -0.628319, 0.366519, 1.74533,
                                   -0.10472,  1.5708,   0.0 };
    // 力传感器标定
    auto calib_result = tcpSensorcalibration(cli, { joint1, joint2, joint3 });
    std::cout << "force_offset: " << std::get<0>(calib_result) << std::endl;
    std::cout << "com: " << std::get<1>(calib_result) << std::endl;
    std::cout << "mass: " << std::get<2>(calib_result) << std::endl;

    if (calculateDistance(std::get<0>(calib_result)) < 0.0001) {
        std::cout << "标定错误，请检查传感器数据!" << std::endl;
        exit(-1);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "go to start_joint" << std::endl;
    std::vector<double> start_joint = { 0 / 180 * M_PI,     16.41 / 180 * M_PI,
                                        76.36 / 180 * M_PI, 7.87 / 180 * M_PI,
                                        90.21 / 180 * M_PI, 0 / 180 * M_PI };
    // 关节运动到起始位置
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->moveJoint(start_joint, 10 * (M_PI / 180), 5 * (M_PI / 180), 0, 0);
    waitArrival(cli->getRobotInterface(robot_name));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // (通过负载辨识的结果来)设置负载
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setPayload(std::get<2>(calib_result), std::get<1>(calib_result),
                     { 0. }, { 0. });
    // 设置力传感器偏移
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setTcpForceOffset(std::get<0>(calib_result));

    // 导纳质量
    std::vector<double> admittance_m = { 10.0, 10.0, 10.0, 2.0, 2.0, 2.0 };
    // 导纳阻尼
    // 用来调节力控运动过程中的速度大小，在相同受力情况下，阻尼越大，速度越小
    std::vector<double> admittance_d = {
        200.0, 200.0, 200.0, 20.0, 20.0, 20.0
    };
    // 导纳刚度
    // 用来调节机械臂弹性，刚度越大回弹越快
    std::vector<double> admittance_k = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    // 设置力控动力学模型
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setDynamicModel(admittance_m, admittance_d, admittance_k);

    // 柔性轴（方向）选择
    std::vector<bool> compliance = { true, true, true, true, true, true };
    // 目标力/力矩
    std::vector<double> target_wrench(6, 0.);
    // 速度限制
    std::vector<double> speed_limits(6, 2.0);
    // 设置力控参考(目标)值。
    cli->getRobotInterface(robot_name)
        ->getForceControl()
        ->setTargetForce(std::vector<double>(6, 0.), compliance, target_wrench,
                         speed_limits, TaskFrameType::NONE);

    std::cout << "Press the key 's'/'q' to enable/disable force control mode."
              << std::endl;
    while (1) {
        std::cout << "Please input your choose: " << std::endl;
        char key;
        std::cin >> key;
        switch (key) {
        case 's':
            if (cli->getRobotInterface(robot_name)
                    ->getForceControl()
                    ->isFcEnabled()) {
                std::cout << "The robot has already been force control mode. "
                             "Can't enable force control mode"
                          << std::endl;
                break;
            } else {
                // 使能力控
                cli->getRobotInterface(robot_name)
                    ->getForceControl()
                    ->fcEnable();
                std::cout << "Enter force control mode" << std::endl;
            }
            break;
        case 'q':
            if (!cli->getRobotInterface(robot_name)
                     ->getForceControl()
                     ->isFcEnabled()) {
                std::cout << "The robot has already quit force control mode. "
                             "Can't disable force control mode"
                          << std::endl;
                break;
            } else {
                // 退出力控
                cli->getRobotInterface(robot_name)
                    ->getForceControl()
                    ->fcDisable();
                std::cout << "Quit force control mode" << std::endl;
            }
            break;
        default:
            std::cout << "Please input 's' or 'q'." << std::endl;
            break;
        }
    }
}

#define LOCAL_IP "127.0.0.1"
int main(int argc, char **argv)
{
#ifdef _WIN32
    // 将Windows控制台输出代码页设置为 UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
    auto rpc_cli = std::make_shared<RpcClient>();
    rpc_cli->setRequestTimeout(1000);
    // 接口调用: 连接到 RPC 服务
    rpc_cli->connect(LOCAL_IP, 30004);
    // 接口调用: 登录
    rpc_cli->login("aubo", "123456");

    // 使能和退出力控
    exampleForceControl(rpc_cli);

    // 读取力传感器数据并保存到force.csv文件中
    //    tcpSensorTest(rpc_cli);
}