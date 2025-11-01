#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <chrono>
#include <aubo_sdk/rpc.h>
#include <aubo_sdk/rtde.h>
#include <skill_interface/force_control.h>

using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

class example
{
public:
    example(const RpcClientPtr rpc, const RtdeClientPtr rtde)
        : rpc_(rpc), rtde_(rtde)
    {
        subscribeRealtimeState();
        fc_ = new arcs::aubo_sdk::ForceControl(rpc_, rtde_);
        state_ = 0;

        fc_->registerStateCallBack(
            std::bind(&example::stateCallback, this, std::placeholders::_1));
    }

    int init()
    {
        auto name = rpc_->getRobotNames().front();
        auto robot_interface = rpc_->getRobotInterface(name);

        robot_interface->getRobotConfig()->setTcpForceOffset(
            actual_tcp_force_sensor_);
    }

    // 接近示例
    int example_touch()
    {
        auto name = rpc_->getRobotNames().front();
        auto robot_interface = rpc_->getRobotInterface(name);

        // 设置接近参数
        arcs::aubo_sdk::ForceControl::TouchParams param;
        param.frame_type = TOOL_FORCE;
        // 接触力为10N
        param.wrench = { 0.0, 0.0, -10.0, 0.0, 0.0, 0.0 };
        // 接近过程中的速度为10mm/s
        param.speed = { 0.0, 0.0, 0.01, 0.0, 0.0, 0.0 };
        // 探寻距离为20mm
        param.distance = 0.020;
        // 软硬程度为0
        param.stiff_scale = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
        // 1s超时
        param.touch_time = 50000;
        fc_->fcTouch(param);

        // 等待接近流程结束
        int ret = fc_->fcWaitCondition();
        if (ret == arcs::aubo_sdk::ForceControl::StateCode::Touch_Succeed) {
            std::cout << "达到目标力" << std::endl;
        } else if (ret ==
                   arcs::aubo_sdk::ForceControl::StateCode::Touch_Distance) {
            std::cout << "超过探寻距离" << std::endl;
        } else if (ret == arcs::aubo_sdk::ForceControl::StateCode::TimeOut) {
            std::cout << "探寻超时" << std::endl;
        }

        fc_->fcExit();
    }

    // 恒力示例
    int example_constant()
    {
        auto name = rpc_->getRobotNames().front();
        auto robot_interface = rpc_->getRobotInterface(name);

        // 设置恒力参数
        arcs::aubo_sdk::ForceControl::ConstantParams param;
        param.frame_type = TOOL_FORCE;
        param.wrench = { 0.0, 0.0, -10.0, 0.0, 0.0, 0.0 };
        // 设置阻尼 避免震荡
        param.wrench = { 0.0, 0.0, -10.0, 0.0, 0.0, 0.0 };
        // 根据实际测试环境设置环境刚度
        param.env_stiff = { 0.0, 0.0, 0.2, 0.0, 0.0, 0.0 };
        fc_->fcConstant(param);

        // 设置参考轨迹 从当前位姿沿X正方向移动20mm
        std::vector<double> pose = actual_tcp_pose_;
        {
            std::unique_lock<std::mutex> lock(rtde_mutex_);
            pose = actual_tcp_pose_;
        }
        pose[0] += 0.05;
        robot_interface->getMotionControl()->moveLine(pose, 0.01, 0.01, 0.0,
                                                      -1);
        // 等待参考轨迹到位
        waitArrival(robot_interface);
        if (state_ == 0) {
            std::cout << "恒力轨迹到位" << std::endl;
        } else if (state_ == -100) {
            std::cout << "恒力控制超时" << std::endl;
        }

        fc_->fcExit();
        return 0;
    }

    void example_insert()
    {
        // 设置插孔参数
        arcs::aubo_sdk::ForceControl::InsertParams param;

        // 无参考轨迹
        param.guide_traj_type = arcs::aubo_sdk::ForceControl::NONE;

        // 螺旋线
        //        param.guide_traj_type = arcs::aubo_sdk::ForceControl::SPIRAL;
        //        param.spiral_param.step = 3;
        //        param.spiral_param.direction = 1;
        //        param.spiral_param.plane =
        //            arcs::aubo_sdk::ForceControl::SpiralPlane::xy;
        //        param.spiral_param.spiral = 0.005;
        //        param.spiral_param.helix = 0.005;
        //        param.spiral_param.radius = 0.06;

        // 摆线
        //        param.guide_traj_type = arcs::aubo_sdk::ForceControl::WEAVE;
        //        param.weave_param.step = 30;
        //        param.weave_param.direction =
        //            arcs::aubo_sdk::ForceControl::WeaveSelect::rx;
        //        param.weave_param.amplitude = 0.005;
        //        param.weave_param.hold_distance = 0.2;
        //        param.weave_param.angle = 0.1;

        fc_->fcInsert(param);

        // 等待接近流程结束
        while (state_.load() == 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(10)); //   延时10ms
        }

        if (state_.load() ==
            arcs::aubo_sdk::ForceControl::StateCode::Insert_Succeed) {
            std::cout << "插孔成功" << std::endl;
        } else if (state_.load() ==
                   arcs::aubo_sdk::ForceControl::StateCode::Insert_MaxForce) {
            std::cout << "插孔力过大" << std::endl;
        }

        fc_->fcExit();
    }

    void example_search()
    {
        // 设置搜孔参数
        arcs::aubo_sdk::ForceControl::SearchParams param;

        param.search_time = 10 * 1000;
        param.search_guide_force = 5;
        param.search_max_force = 20;

        // 螺旋线
        param.guide_traj_type = arcs::aubo_sdk::ForceControl::SPIRAL;
        param.spiral_param.step = 3;
        param.spiral_param.direction = 1;
        param.spiral_param.plane =
            arcs::aubo_sdk::ForceControl::SpiralPlane::xy;
        param.spiral_param.spiral = 0.005;
        param.spiral_param.helix = 0.0;
        param.spiral_param.radius = 0.005;

        // 摆线
        //        param.guide_traj_type = arcs::aubo_sdk::ForceControl::SPIRAL;
        //        param.weave_param.step = 30;
        //        param.weave_param.direction =
        //            arcs::aubo_sdk::ForceControl::WeaveSelect::rx;
        //        param.weave_param.amplitude = 0.005;
        //        param.weave_param.hold_distance = 0.2;
        //        param.weave_param.angle = 0.1;

        fc_->fcSearch(param);

        // 等待接近流程结束
        int ret = fc_->fcWaitCondition();

        if (ret == arcs::aubo_sdk::ForceControl::StateCode::Search_Succeed) {
            std::cout << "搜孔成功" << std::endl;
        } else if (ret ==
                   arcs::aubo_sdk::ForceControl::StateCode::Search_MaxForce) {
            std::cout << "搜孔力过大" << std::endl;
        } else if (ret == arcs::aubo_sdk::ForceControl::StateCode::TimeOut) {
            std::cout << "搜孔超时" << std::endl;
        } else {
            std::cout << "Error state code:" << state_.load() << std::endl;
        }

        fc_->fcExit();
    }

    void getTcpTaskForce()
    {
        TaskFrameType type = TaskFrameType::MOTION_FORCE;
        std::vector<double> feature = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        while (1) {
            auto task_force = fc_->getTaskForce(type, feature);
            std::cout << task_force[0] << ", " << task_force[1] << ", "
                      << task_force[2] << ", " << task_force[3] << ", "
                      << task_force[4] << ", " << task_force[5] << ", "
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        return;
    }

    void exit() { fc_->fcExit(); }

    void stateCallback(int state) { state_.store(state); }

private:
    void subscribeRealtimeState()
    {
        if (rtde_ == nullptr) {
            return;
        }

        int topic1 =
            rtde_->setTopic(false,
                            { "R1_actual_TCP_pose", "R1_actual_TCP_force",
                              "R1_actual_TCP_force_sensor" },
                            40, 1);
        rtde_->subscribe(topic1, [this](arcs::aubo_sdk::InputParser &parser) {
            std::unique_lock<std::mutex> lck(rtde_mutex_);
            actual_tcp_pose_ = parser.popVectorDouble();
            actual_tcp_force_ = parser.popVectorDouble();
            actual_tcp_force_sensor_ = parser.popVectorDouble();
        });
    }

    // 实现阻塞功能: 当机械臂运动到目标路点时，程序再往下执行
    int waitArrival(RobotInterfacePtr impl)
    {
        //接口调用: 获取当前的运动指令 ID
        int exec_id = impl->getMotionControl()->getExecId();

        int cnt = 0;
        // 在等待机械臂开始运动时，获取exec_id最大的重试次数
        int max_retry_count = 50;

        // 等待机械臂开始运动
        while (exec_id == -1 && state_.load() == 0) {
            if (cnt++ > max_retry_count) {
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            exec_id = impl->getMotionControl()->getExecId();
        }

        // 等待机械臂动作完成
        while (exec_id != -1 && state_.load() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            exec_id = impl->getMotionControl()->getExecId();
        }

        return 0;
    }

private:
    RpcClientPtr rpc_ = nullptr;
    RtdeClientPtr rtde_ = nullptr;

    arcs::aubo_sdk::ForceControl *fc_ = nullptr;

    // RTDE相关
    std::mutex rtde_mutex_;
    std::vector<double> actual_tcp_pose_;
    std::vector<double> actual_tcp_force_;
    std::vector<double> actual_tcp_force_sensor_;

    std::atomic_int state_;
};

#define LOCAL_IP "192.168.1.20"

int main(int argc, char **argv)
{
#ifdef WIN32
    // 将Windows控制台输出代码页设置为 UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    auto rpc = std::make_shared<RpcClient>();
    auto rtde = std::make_shared<RtdeClient>();
    rpc->setRequestTimeout(1000);
    auto ret = rpc->connect(LOCAL_IP, 30004);
    rpc->login("aubo", "123456");

    rtde->connect(LOCAL_IP, 30010);
    rtde->login("aubo", "123456");

    example ex(rpc, rtde);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 等待rtde数据

    //    ex.init();
    //    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 接近示例
    //    ex.example_touch();

    // 恒力示例
    //    ex.example_constant();

    // 插入示例
    //    ex.example_insert();

    // 搜孔示例
    //    ex.example_search();

    // 获取实时力控参考系下的力数据
    ex.getTcpTaskForce();

    auto robot_name = rpc->getRobotNames().front();
    auto robot_interface = rpc->getRobotInterface(robot_name);

    rpc->logout();
    rtde->logout();

    rpc->disconnect();
    rtde->disconnect();

    return 0;
}
