#include "aubo_sdk/rpc.h"
#include "math.h"
#ifdef WIN32
#include <Windows.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

bool isJointNaN(const std::vector<double> &q, int dof)
{
    for (int i = 0; i < dof; i++) {
        if (std::isnan(q[i])) {
            return true;
        }
    }
    return false;
}

bool isJointInRange(const std::vector<double> &q,
                    const std::vector<double> &upper,
                    const ::std::vector<double> &lower, int dof)
{
    for (int i = 0; i < dof; i++) {
        if (q[i] < (lower[i] - 1e-6) || q[i] > (upper[i] + 1e-6)) {
            return false;
        }
    }
    return true;
}

bool checkJoint(RpcClientPtr impl, const std::vector<double> &q)
{
    auto robot_name = impl->getRobotNames().front();
    auto interface = impl->getRobotInterface(robot_name);
    auto dof = interface->getRobotConfig()->getDof();
    auto upper = interface->getRobotConfig()->getJointMaxPositions();
    auto lower = interface->getRobotConfig()->getJointMinPositions();
    if ((int)q.size() != dof || isJointNaN(q, dof)) {
        return false;
    }

    // 检查关节是否超限
    if (!isJointInRange(q, upper, lower, dof)) {
        return false;
    }
}

// 检查给定的轨迹是否有效，进行插值和逆解验证
bool exampleTrajectoryValid(RpcClientPtr impl, const std::vector<double> &p1,
                            const std::vector<double> &p2, int num_points)
{
    // 如果num_points小于2，则无法进行插值
    if (num_points < 2) {
        throw std::invalid_argument("num_points must be at least 2");
    }

    // 接口调用: 获取机器人的名字
    auto robot_name = impl->getRobotNames().front();

    // 接口调用: 设置 tcp 偏移
    std::vector<double> offset = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    impl->getRobotInterface(robot_name)->getRobotConfig()->setTcpOffset(offset);

    // 计算每个插值点的alpha值，并调用interpolatePose
    for (int i = 0; i < num_points; ++i) {
        double alpha =
            static_cast<double>(i) / (num_points - 1); // alpha从0到1均匀变化

        // 接口调用: 计算线性差值
        auto pose = impl->getMath()->interpolatePose(p1, p2, alpha);

        // 接口调用: 根据计算出的差值位姿，检查是否能找到有效的逆解
        auto result = impl->getRobotInterface(robot_name)
                          ->getRobotAlgorithm()
                          ->inverseKinematicsAll(pose);

        if (std::get<1>(result) != 0) {
            std::cout << "逆解失败, inverseKinematicsAll返回值:"
                      << std::get<1>(result) << std::endl;
            std::cout << "轨迹规划失败" << std::endl;
            return false;
        }
        auto qs = std::get<0>(result);
        for (size_t i = 0; i < qs.size(); ++i) {
            if (!checkJoint(impl, qs[i])) {
                std::cout << "轨迹规划失败" << std::endl;
                return false;
            }
        }
    }

    // 如果所有插值点均有效，表示轨迹规划成功
    std::cout << "轨迹规划成功" << std::endl;
    return true;
}

/**
 * 输入两个关节角，检测规划路径是否有效
 * @brief exampleTrajectoryValid3
 * @param impl
 * @param p1 位姿1
 * @param blend1 交融半径1
 * @param p2 位姿2
 * @param blend2 交融半径2
 * @param qnear
 * @return
 */
bool exampleTrajectoryValid2(RpcClientPtr impl, const std::vector<double> &q1,
                             double blend1, const std::vector<double> &q2,
                             double blend2)
{ // 接口调用: 获取机器人的名字
    auto robot_name = impl->getRobotNames().front();
    auto interface = impl->getRobotInterface(robot_name);
    // 接口调用: 设置 tcp 偏移
    std::vector<double> offset = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    interface->getRobotConfig()->setTcpOffset(offset);

    auto traj =
        interface->getRobotAlgorithm()->pathMovej(q1, blend1, q2, blend2, 0.05);

    for (size_t i = 0; i < traj.size(); ++i) {
        if (!checkJoint(impl, traj[i])) {
            std::cout << "轨迹规划失败" << std::endl;
            return false;
        }
    }
    std::cout << "轨迹规划成功" << std::endl;
    return true;
}

#define LOCAL_IP "127.0.0.1"

int main(int argc, char **argv)
{
#ifdef WIN32
    // 将Windows控制台输出代码页设置为 UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif
    auto rpc_cli = std::make_shared<RpcClient>();
    // 接口调用: 设置 RPC 超时, 单位: ms
    rpc_cli->setRequestTimeout(1000);
    // 接口调用: 连接到 RPC 服务
    rpc_cli->connect(LOCAL_IP, 30004);
    // 接口调用: 登录
    rpc_cli->login("aubo", "123456");

    // 起始位姿和目标位姿
    std::vector<double> pose1 = { 0.551, -0.295, 0.261, -3.135, 0.0, 1.569 };
    std::vector<double> pose2 = { 0.551, 0.295, 0.261, -3.135, 0.0, 0 };

    // 插值点数量
    int num_points = 30;

    // 检查给定的轨迹是否有效，进行插值和逆解验证
    exampleTrajectoryValid(rpc_cli, pose1, pose2, num_points);

    // exampleTrajectoryValid2
    // 关节角，单位: 弧度
    //    std::vector<double> q1 = {
    //        0.0 * (M_PI / 180),  -15.0 * (M_PI / 180), 100.0 * (M_PI / 180),
    //        25.0 * (M_PI / 180), 90.0 * (M_PI / 180),  0.0 * (M_PI / 180)
    //    };

    //    std::vector<double> q2 = {
    //        35.92 * (M_PI / 180),  -11.28 * (M_PI / 180), 59.96 * (M_PI /
    //        180), -18.76 * (M_PI / 180), 90.0 * (M_PI / 180),   35.92 * (M_PI
    //        / 180)
    //    };
    // exampleTrajectoryValid2(rpc_cli, q1, 0, q2, 0);
    // 接口调用: 退出登录
    rpc_cli->logout();
    // 接口调用: 断开连接
    rpc_cli->disconnect();

    return 0;
}
