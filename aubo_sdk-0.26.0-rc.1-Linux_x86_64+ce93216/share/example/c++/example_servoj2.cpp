#include <cstring>
#include <fstream>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif

#include "aubo_sdk/rpc.h"
#include "trajectory_io.h"

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 实现阻塞功能: 当机械臂运动到目标路点时，程序再往下执行
int waitArrival(RobotInterfacePtr impl)
{
    const int max_retry_count = 5;
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

/**
 * 测试1: 让1关节旋转120度，规划时间为10秒，
 * 在第5秒时下发第二个点，让第三关节旋转90度
 * 结论: 在机器人没到达目标点前，下发新目标点，
 * 机器人会放弃原目标点，直接向新的目标点运动
 */
int exampleServoj1(RpcClientPtr cli)
{
    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);

    // 接口调用: 设置机械臂的速度比率
    robot_interface->getMotionControl()->setSpeedFraction(0.3);

    // 关节角，单位: 弧度
    std::vector<double> joint_angle = {
        0.0 * (M_PI / 180),  -15.0 * (M_PI / 180), 100.0 * (M_PI / 180),
        25.0 * (M_PI / 180), 90.0 * (M_PI / 180),  0.0 * (M_PI / 180)
    };

    // 接口调用: 关节运动
    robot_interface->getMotionControl()->moveJoint(
        joint_angle, 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);
    // 阻塞
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "关节运动到初始位置成功" << std::endl;
    } else {
        std::cout << "关节运动到初始位置失败" << std::endl;
    }

    // 接口调用: 开启servo模式
    robot_interface->getMotionControl()->setServoMode(true);

    // 等待进入 servo 模式
    int i = 0;
    while (!robot_interface->getMotionControl()->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Servo 模式使能失败! 当前servo状态为 "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::vector<double> q1 = { -120.0 * (M_PI / 180), -15.0 * (M_PI / 180),
                               100.0 * (M_PI / 180),  25.0 * (M_PI / 180),
                               90.0 * (M_PI / 180),   0.0 * (M_PI / 180) };
    std::cout << "向第一个目标点运动" << std::endl;
    // 接口调用: 关节伺服运动
    robot_interface->getMotionControl()->servoJoint(q1, 0.1, 0.2, 10, 0.1, 200);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::vector<double> q2 = { 0, 0, 90.0 / 180 * M_PI, 0, 0, 0 };
    std::cout << "向第二个目标点运动" << std::endl;
    // 接口调用: 关节伺服运动
    robot_interface->getMotionControl()->servoJoint(q2, 0.1, 0.2, 10, 0.1, 200);

    // 等待运动结束
    while (!cli->getRobotInterface(robot_name)->getRobotState()->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Servoj 运动结束" << std::endl;

    // 关闭servo模式
    robot_interface->getMotionControl()->setServoMode(false);

    // 等待结束 servo 模式
    i = 0;
    while (robot_interface->getMotionControl()->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout
                << "Servo 模式失能失败! 当前的 Servo 模式是 "
                << robot_interface->getMotionControl()->isServoModeEnabled()
                << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}

/**
 * 测试2:
 * 采用servoj跟踪一个轨迹,目标点下发时间间隔5ms,截断式,每次更新路点后不保证经过上个路点
 */
int exampleServoj2(RpcClientPtr cli)
{
    // 读取轨迹文件
    auto filename = "../trajs/record6.offt";
    TrajectoryIo input(filename);

    // 尝试打开轨迹文件，如果无法打开，直接返回
    if (!input.open()) {
        return 0;
    }

    // 解析轨迹数据
    auto traj = input.parse();

    // 检查轨迹文件中是否有路点，
    // 如果数量为 0，输出错误消息并返回
    auto traj_sz = traj.size();
    if (traj_sz == 0) {
        std::cerr << "轨迹文件中的路点数量为0." << std::endl;
        return 0;
    }

    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);

    // 接口调用: 设置机械臂的速度比率
    robot_interface->getMotionControl()->setSpeedFraction(1);

    // 接口调用: 关节运动到轨迹中的第一个点，否则容易引起较大超调
    robot_interface->getMotionControl()->moveJoint(traj[0], 80 * (M_PI / 180),
                                                   60 * (M_PI / 180), 0, 0);

    // 阻塞
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "关节运动到轨迹文件中的第一个路点成功" << std::endl;
    } else {
        std::cout << "关节运动到轨迹文件中的第一个路点失败" << std::endl;
    }

    // 接口调用: 开启 servo 模式
    cli->getRobotInterface(robot_name)->getMotionControl()->setServoMode(true);

    // 等待进入 servo 模式
    int i = 0;
    while (!cli->getRobotInterface(robot_name)
                ->getMotionControl()
                ->isServoModeEnabled()) {
        if (i++ > 20) {
            std::cout << "Servo 模式使能失败! 当前的 Servo 模式是 "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    for (size_t i = 1; i < traj.size(); i++) {
        // 接口调用: 关节伺服运动

        while (cli->getRobotInterface(robot_name)
                   ->getMotionControl()
                   ->servoJoint(traj[i], 0.1, 0.2, 0.01, 0.1, 200) == 2) {
            // 缓存满了，需要重新下发
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            std::cout << "Buffer is full，i: " << i << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 等待运动结束
    while (!cli->getRobotInterface(robot_name)->getRobotState()->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Servoj 运动结束" << std::endl;

    // 接口调用: 关闭 servo 模式
    cli->getRobotInterface(robot_name)->getMotionControl()->setServoMode(false);

    // 等待结束 servo 模式
    while (cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->isServoModeEnabled()) {
        if (i++ > 20) {
            std::cout << "Servo 模式失能失败! 当前的 Servo 模式是 "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}

/**
 * 测试3: 采用servoj mode 2 跟踪一个轨迹,目标点下发时间间隔5ms
 */
int exampleServoj3(RpcClientPtr cli)
{
    // 读取轨迹文件
    auto filename = "../trajs/record6.offt";
    TrajectoryIo input(filename);

    // 尝试打开轨迹文件，如果无法打开，直接返回
    if (!input.open()) {
        return 0;
    }

    // 解析轨迹数据
    auto traj = input.parse();

    // 检查轨迹文件中是否有路点，
    // 如果数量为 0，输出错误消息并返回
    auto traj_sz = traj.size();
    if (traj_sz == 0) {
        std::cerr << "轨迹文件中的路点数量为0." << std::endl;
        return 0;
    }

    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);

    // 接口调用: 设置机械臂的速度比率
    robot_interface->getMotionControl()->setSpeedFraction(1);

    // 接口调用: 关节运动到轨迹中的第一个点，否则容易引起较大超调
    robot_interface->getMotionControl()->moveJoint(traj[0], 80 * (M_PI / 180),
                                                   60 * (M_PI / 180), 0, 0);

    // 阻塞
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "关节运动到轨迹文件中的第一个路点成功" << std::endl;
    } else {
        std::cout << "关节运动到轨迹文件中的第一个路点失败" << std::endl;
    }

    // 接口调用: 开启 servo 模式 2
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoModeSelect(2);

    // 等待进入 servo 模式 2
    int i = 0;
    while (cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->getServoModeSelect() != 2) {
        if (i++ > 50) {
            std::cout << "Servo 模式 2 使能失败! 当前的 Servo 模式是 "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->getServoModeSelect()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    for (size_t i = 1; i < traj.size(); i++) {
        // 接口调用: 关节伺服运动
        cli->getRobotInterface(robot_name)
            ->getMotionControl()
            ->servoJoint(traj[i], 0.1, 0.2, 0.1, 0.1, 200);

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 等待运动结束
    while (!cli->getRobotInterface(robot_name)->getRobotState()->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Servoj 运动结束" << std::endl;

    // 接口调用: 关闭 servo 模式 2
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoModeSelect(0);

    // 等待结束 servo 模式 2
    while (cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->getServoModeSelect() != 0) {
        if (i++ > 50) {
            std::cout << "Servo 模式 2 失能失败! 当前的 Servo 模式是 "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->getServoModeSelect()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}

/**
 * 测试4:
 * 采用servoj跟踪一个轨迹,目标点下发时间间隔5ms, 路点必达式
 */
int exampleServoj4(RpcClientPtr cli)
{
    // 读取轨迹文件
    auto filename = "../trajs/record6.offt";
    TrajectoryIo input(filename);

    // 尝试打开轨迹文件，如果无法打开，直接返回
    if (!input.open()) {
        return 0;
    }

    // 解析轨迹数据
    auto traj = input.parse();

    // 检查轨迹文件中是否有路点，
    // 如果数量为 0，输出错误消息并返回
    auto traj_sz = traj.size();
    if (traj_sz == 0) {
        std::cerr << "轨迹文件中的路点数量为0." << std::endl;
        return 0;
    }

    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();
    auto robot_interface = cli->getRobotInterface(robot_name);

    // 接口调用: 设置机械臂的速度比率
    robot_interface->getMotionControl()->setSpeedFraction(1);

    // 接口调用: 关节运动到轨迹中的第一个点，否则容易引起较大超调
    robot_interface->getMotionControl()->moveJoint(traj[0], 80 * (M_PI / 180),
                                                   60 * (M_PI / 180), 0, 0);

    // 阻塞
    int ret = waitArrival(robot_interface);
    if (ret == 0) {
        std::cout << "关节运动到轨迹文件中的第一个路点成功" << std::endl;
    } else {
        std::cout << "关节运动到轨迹文件中的第一个路点失败" << std::endl;
    }

    // 接口调用: 开启 servo 模式 5
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoModeSelect(5);

    // 等待进入 servo 模式
    int i = 0;
    while (cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->getServoModeSelect() != 5) {
        if (i++ > 20) {
            std::cout << "Servo 模式使能失败! 当前的 Servo 模式是 "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->getServoModeSelect()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    for (size_t i = 1; i < traj.size(); i++) {
        // 接口调用: 关节伺服运动

        while (cli->getRobotInterface(robot_name)
                   ->getMotionControl()
                   ->servoJoint(traj[i], 0.1, 0.2, 0.01, 0.1, 200) == 2) {
            // 缓存满了，需要重新下发
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            std::cout << "Buffer is full，i: " << i << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 等待运动结束
    while (!cli->getRobotInterface(robot_name)->getRobotState()->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Servoj 运动结束" << std::endl;

    // 接口调用: 关闭 servo 模式 5
    cli->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoModeSelect(0);

    // 等待结束 servo 模式5
    while (cli->getRobotInterface(robot_name)
               ->getMotionControl()
               ->getServoModeSelect() != 0) {
        if (i++ > 20) {
            std::cout << "Servo 模式失能失败! 当前的 Servo 模式是 "
                      << cli->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->getServoModeSelect()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}

#define LOCAL_IP "127.0.0.1"
/**
 * servoj功能使用步骤:
 * 1、采用实时系统测试servoj功能
 * 2、孤立一个CPU，保证该CPU没有其他任务
 * 3、将该进程设置为实时进程，优先级设置为最大
 * 4、绑定CPU，将该实时进程绑定到第二步孤立的CPU上
 */
int main(void)
{
#ifdef WIN32
    // 将Windows控制台输出代码页设置为 UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
#ifndef _WIN32
    // 实时优先级最大值、最小值
    int sched_max = sched_get_priority_max(SCHED_FIFO);

    // 设置实时调度策略及优先级
    struct sched_param sParam;
    sParam.sched_priority = sched_max;
    sched_setscheduler(0, SCHED_FIFO, &sParam);
    auto i_schedFlag = sched_getscheduler(0);
    printf("设置调度策略 = [%d]\n", i_schedFlag);

    // 绑定CPU
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);

    // bind process to processor 0
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) < 0) {
        perror("Sched_setaffinity fail!");
    }
#endif

    auto rpc_cli = std::make_shared<RpcClient>();
    // 接口调用: 设置 RPC 超时
    rpc_cli->setRequestTimeout(1000);
    // 接口调用: 连接到 RPC 服务
    rpc_cli->connect(LOCAL_IP, 30004);
    // 接口调用: 登录
    rpc_cli->login("aubo", "123456");

    exampleServoj1(rpc_cli);
    // exampleServoj2(rpc_cli);
    // exampleServoj3(rpc_cli);
    // exampleServoj4(rpc_cli);

    // 接口调用: RPC 退出登录
    rpc_cli->logout();
    // 接口调用: RPC 断开连接
    rpc_cli->disconnect();

    return 0;
}
