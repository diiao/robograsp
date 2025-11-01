#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 实现阻塞功能: 当机械臂运动到目标路点时，程序再往下执行
int waitArrival(RobotInterfacePtr impl)
{
    const int max_retry_count = 20;
    int cnt = 0;

    // 接口调用: 获取当前的运动指令 ID
    int exec_id = impl->getMotionControl()->getExecId();

    // 等待机械臂开始运动
    while (exec_id == -1) {
        if (cnt++ > max_retry_count) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        exec_id = impl->getMotionControl()->getExecId();
    }

    // 等待机械臂动作完成
    while (impl->getMotionControl()->getExecId() != -1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}

void examplePathOffsetWeave(RpcClientPtr cli)
{
    // 接口调用: 开启运行时。
    // 注意: 摆弧运动需要将运行时打开,否则摆弧不生效。
    cli->getRuntimeMachine()->start();

    // 关节角，单位: 弧度
    std::vector<double> joint_angle = {
        0.0 * (M_PI / 180),  -15.0 * (M_PI / 180), 100.0 * (M_PI / 180),
        25.0 * (M_PI / 180), 90.0 * (M_PI / 180),  0.0 * (M_PI / 180)
    };
    // 位姿
    std::vector<double> pose1 = { 0.551, -0.295, 0.261, -3.135, 0.0, 1.569 };
    std::vector<double> pose2 = { 0.551, 0.295, 0.261, -3.135, 0.0, 1.569 };

    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();

    auto robot_interface = cli->getRobotInterface(robot_name);

    // 接口调用: 设置机械臂的速度比率
    robot_interface->getMotionControl()->setSpeedFraction(0.75);

    // 接口调用: 设置工具中心点（TCP相对于法兰盘中心的偏移）
    std::vector<double> tcp_offset(6, 0.0);
    robot_interface->getRobotConfig()->setTcpOffset(tcp_offset);

    // 接口调用: 关节运动到起始位置
    robot_interface->getMotionControl()->moveJoint(
        joint_angle, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);

    // 阻塞
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "关节运动到起始位置成功！" << std::endl;
    } else {
        std::cout << "关节运动到起始位置失败！" << std::endl;
    }

    // 接口调用: 直线运动到位置1
    robot_interface->getMotionControl()->moveLine(pose1, 1.2, 0.25, 0.0, 0);

    // 阻塞
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "直线运动到位置1成功！" << std::endl;
    } else {
        std::cout << "直线运动到位置1失败！" << std::endl;
    }

    std::string params = "{\"type\":\"SINE\",\"step\":0.01,\"amplitude\": "
                         "[0.01,0.01],\"hold_distance\": "
                         "[0,0],\"hold_time\":[0,0], "
                         "\"angle\":0, \"direction\":0}";

    // 接口调用: 开始摆动
    robot_interface->getMotionControl()->weaveStart(params);

    // 接口调用: 直线运动到位置2
    robot_interface->getMotionControl()->moveLine(pose2, 0.05, 0.01, 0.0, 0.0);

    // 接口调用: 路径偏移使能
    // 注意: pathOffsetEnable必须与pathOffsetDisable搭配使用
    robot_interface->getMotionControl()->pathOffsetEnable();

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    // 接口调用: 设置路径偏移
    for (int i = 0; i < 10; i++) {
        robot_interface->getMotionControl()->pathOffsetSet(
            { 0, 0, 0.15, 0, 0, 0 }, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        robot_interface->getMotionControl()->pathOffsetSet(
            { 0, 0, -0.15, 0, 0, 0 }, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 阻塞
    ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "摆弧运动到位置2成功！" << std::endl;
    } else {
        std::cout << "摆弧运动到位置2失败！" << std::endl;
    }

    // 接口调用: 结束摆动
    robot_interface->getMotionControl()->weaveEnd();

    // 接口调用: 路径偏移失能
    robot_interface->getMotionControl()->pathOffsetDisable();
    // 此处增加延时，是为了保证pathOffsetDisable被执行完
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 摆弧运动结束后将运行时关闭
    cli->getRuntimeMachine()->stop();
}

/**
 * 功能: 机械臂摆弧中心线偏移运动
 * 步骤:
 * 第一步: 设置 RPC 超时、连接 RPC 服务、机械臂登录
 * 第二步: 设置运动速度比率和工具中心点
 * 第三步: 开启运行时，
 *       先关节运动到起始位置，
 *       然后直线运动到位置1，
 *       开启摆动，向位置2做摆弧运动，使能并设置路径偏移，
 *       到达位置2后，结束摆动，路径偏移失能，关闭运行时
 * 第四步: RPC 退出登录、断开连接
 */

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
#ifdef WIN32
    // 将Windows控制台输出代码页设置为 UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    auto rpc_cli = std::make_shared<RpcClient>();
    // 接口调用: 设置 RPC 超时
    rpc_cli->setRequestTimeout(1000);
    // 接口调用: 连接到 RPC 服务
    rpc_cli->connect(LOCAL_IP, 30004);
    // 接口调用: 登录
    rpc_cli->login("aubo", "123456");

    // 摆弧中心线偏移运动
    examplePathOffsetWeave(rpc_cli);

    // 接口调用: 退出登录
    rpc_cli->logout();
    // 接口调用: 断开连接
    rpc_cli->disconnect();

    return 0;
}
