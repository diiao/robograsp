#include <math.h>
#include <iostream>

#include <stdlib.h>
#ifdef WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <chrono>

#include "aubo_sdk/rpc.h"

#include "research_interface/robot.h"
#include "trajectory_io.h"

template <typename T, std::size_t N>
inline std::ostream &operator<<(std::ostream &os, const std::array<T, N> &list)
{
    for (size_t i = 0; i < list.size(); i++) {
        os << list.at(i);
        if (i != (list.size() - 1)) {
            os << ",";
        }
    }
    return os;
}

template <typename T, std::size_t N>
inline auto operator*(const std::array<T, N> &l, T x)
{
    std::array<T, N> temp;
    for (size_t i = 0; i < l.size(); i++) {
        temp[i] = l[i] * x;
    }
    return temp;
}

using namespace arcs::research_interface;
int main(int argc, char *argv[])
{
    auto robot = Robot("127.0.0.1");

    // servo是否循环下发离线轨迹
    bool loop = true;

    // 离线轨迹是否完成或者机器人状态是否出错
    bool finish = false;

    // 如果机器人没有上电，对机器人进行上电操作

    if (robot.startup() < 0) {
        std::cout << "robot start up failed" << std::endl;
        return -1;
    }

    TrajectoryIo traj_io("../trajs/record7.offt");
    auto trajs = traj_io.parse();
    int index = 0;
    // 移动到第一个点
    JointPositions jnt_pos;
    for (int i = 0; i < 6; i++) {
        jnt_pos.q[i] = trajs[index][i];
    }

    std::cout << "movej: " << jnt_pos.q * (180. / M_PI) << std::endl;
    if (robot.movej(jnt_pos) < 0) {
        return -1;
    }

    while (!finish) {
        robot.read([](const RobotState &robot_state) {
            std::cout << "read ===> q: " << robot_state.q * (180. / M_PI)
                      << std::endl;

            return true;
        });
        // 执行离线轨迹
        robot.control(
            [&](const RobotState &robot_state, double duration) {
                JointPositions jnt_pos;
                for (int i = 0; i < 6; i++) {
                    jnt_pos.q[i] = trajs[index][i];
                }
                index++;
                if (index >= trajs.size() && loop) {
                    // 无限循环跑离线轨迹
                    index = 0;
                } else if (index >= trajs.size() && !loop) {
                    // 只跑一次离线轨迹
                    jnt_pos.finished = true;
                    finish = true;
                } else if (robot_state.error_code) {
                    // 机械臂状态出现了问题
                    jnt_pos.finished = true;
                    finish = true;
                }
                return jnt_pos;
            },
            false, 0, loop);
    }

    return 0;
}
