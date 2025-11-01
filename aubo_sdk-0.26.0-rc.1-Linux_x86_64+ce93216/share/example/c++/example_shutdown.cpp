#include "aubo_sdk/rpc.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace arcs::aubo_sdk;

#define ip_local "127.0.0.1"

void exampleShutdown1(RpcClientPtr cli)
{
    // 接口调用: 关机
    int ret = cli->shutdown();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "ret = " << ret << std::endl;
}

void exampleShutdown2(RpcClientPtr cli)
{
    // 接口调用: 获取机械臂的名称
    auto robot_name = cli->getRobotNames().front();

    // 接口调用: 关机
    cli->getRobotInterface(robot_name)
        ->getRobotConfig()
        ->setHardwareCustomParameters("tp_shutdown = true");
}

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
    rpc_cli->connect(ip_local, 30004);
    // 接口调用: 登录
    rpc_cli->login("aubo", "123456");

    // 示例1：通过shutdown接口来关机
    exampleShutdown1(rpc_cli);

    // 示例2：通过setHardwareCustomParameters接口来关机
    //    exampleShutdown2(rpc_cli);

    // 接口调用: 退出登录
    rpc_cli->logout();
    // 接口调用: 断开连接
    rpc_cli->disconnect();

    return 0;
}
