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
 * 测试: 采用 TrackJoint 跟踪一个轨迹,目标点下发时间间隔5ms
 */
int exampleTrackJoint(RpcClientPtr cli)
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
    robot_interface->getMotionControl()->setSpeedFraction(0.8);

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

    for (size_t i = 1; i < traj.size(); i++) {
        int traj_queue_size =
            robot_interface->getMotionControl()->getTrajectoryQueueSize();
        std::cout << "traj_queue_size: " << traj_queue_size << std::endl;
        while (traj_queue_size > 8) {
            traj_queue_size =
                robot_interface->getMotionControl()->getTrajectoryQueueSize();

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        robot_interface->getMotionControl()->trackJoint(traj[i], 0.01, 0.5, 1);
    }

    // 等待运动结束
    while (!robot_interface->getRobotState()->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    robot_interface->getMotionControl()->stopJoint(1);

    std::cout << "trackJoint 运动结束" << std::endl;

    return 0;
}

#define LOCAL_IP "127.0.0.1"

/**
 * trackJoint 功能使用步骤:
 * 1、采用实时系统测试 trackJoint 功能
 * 2、孤立一个 CPU，保证该 CPU 没有其他任务
 * 3、将该进程设置为实时进程，优先级设置为最大
 * 4、绑定 CPU，将该实时进程绑定到第二步孤立的 CPU 上
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

    exampleTrackJoint(rpc_cli);

    // 接口调用: RPC 退出登录
    rpc_cli->logout();
    // 接口调用: RPC 断开连接
    rpc_cli->disconnect();

    return 0;
}
