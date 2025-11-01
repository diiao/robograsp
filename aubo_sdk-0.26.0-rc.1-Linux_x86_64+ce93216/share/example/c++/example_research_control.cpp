#include <math.h>
#include <iostream>

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
    TrajectoryIo traj_io("../trajs/record6.offt");
    auto trajs = traj_io.parse();
    int index = 0;

    // 移动到第一个点
    JointPositions jnt_pos;
    for (int i = 0; i < 6; i++) {
        jnt_pos.q[i] = trajs[index][i];
    }
    std::cout << "movej: " << jnt_pos.q * (180. / M_PI) << std::endl;
    robot.movej(jnt_pos);

    robot.read([](const RobotState &robot_state) {
        std::cout << "q: " << robot_state.q * (180. / M_PI) << std::endl;
        return true;
    });

    robot.control([&](const RobotState &robot_state, double duration) {
        JointPositions jnt_pos;
        for (int i = 0; i < 6; i++) {
            jnt_pos.q[i] = trajs[index][i];
        }
        index++;
        jnt_pos.finished = (index >= trajs.size());
        return jnt_pos;
    });

    return 0;
}
