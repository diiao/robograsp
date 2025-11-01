using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using System.Threading;

namespace csharp_example
{
    class ExampleMoveJ
    {
        const int RSERR_SUCC = 0;

        static UInt16 rshd = 0xffff;
        //机械臂IP地址
        const string robotIP = "192.168.204.151";
        //机械臂端口号
        const int serverPort = 30004;
        //M_PI
        const double M_PI = 3.14159265358979323846;


        // 实现阻塞功能: 当机械臂运动到目标路点时，程序再往下执行
        static int waitArrival(IntPtr robot_interface)
        {
            const int max_retry_count = 5;
            int cnt = 0;

            // 接口调用: 获取当前的运动指令 ID
            IntPtr motion_control = cSharpBinging_RobotInterface.robot_getMotionControl(robot_interface);
            int exec_id =  cSharpBinging_MotionControl.getExecId(motion_control);
 
            // 等待机械臂开始运动
            while (exec_id == -1)
            {
                if (cnt++ > max_retry_count)
                {
                    return -1;
                }
                Thread.Sleep(50);
                exec_id = cSharpBinging_MotionControl.getExecId(motion_control);
            }

            // 等待机械臂动作完成
            while (cSharpBinging_MotionControl.getExecId(motion_control) != -1)
            {
                Thread.Sleep(50);
            }

            return 0;
        }

        static int exampleMoveJ(IntPtr cli)
        {
            
 
                List<double> joint_angle1 = new List<double>()
            {
                0.0 * (M_PI / 180), -15.0 * (M_PI / 180), 100.0 * (M_PI / 180),
                25.0 * (M_PI / 180), 90.0 * (M_PI / 180), 0.0 * (M_PI / 180)
            };

                List<double> joint_angle2 = new List<double>()
            {
                35.92 * (M_PI / 180), -11.28 * (M_PI / 180), 59.96 * (M_PI / 180),
                -18.76 * (M_PI / 180), 90.0 * (M_PI / 180), 35.92 * (M_PI / 180)
            };

                List<double> joint_angle3 = new List<double>()
            {
                41.04 * (M_PI / 180), -7.65 * (M_PI / 180), 98.80 * (M_PI / 180),
                16.44 * (M_PI / 180), 90.0 * (M_PI / 180), 11.64 * (M_PI / 180)
            };

                // 接口调用: 获取机器人的名字
                IntPtr[] robot_names = new IntPtr[10];
                for (int i = 0; i < 10; i++)
                {
                    robot_names[i] = Marshal.AllocHGlobal(100); // 分配100字节内存用于存放字符串（考虑 '\0' 结尾）
                }

                int num = cSharpBinding_RPC.rpc_getRobotNames(cli, robot_names);
                if (num <= 0)
                {
                    for (int i = 0; i < 10; i++)
                    {
                        Marshal.FreeHGlobal(robot_names[i]); // 释放分配的内存
                    }
                    return -1;
                }
                string robot_name = Marshal.PtrToStringAnsi(robot_names[0]);
                for (int i = 0; i < 10; i++)
                {
                    Marshal.FreeHGlobal(robot_names[i]); // 释放分配的内存
                }
                if (robot_name == "")
                {
                    return -1;
                }

                IntPtr robot_interface = cSharpBinding_RPC.rpc_getRobotInterface(cli, robot_name);

                // 接口调用: 设置机械臂的速度比率，
            IntPtr motion_control = cSharpBinging_RobotInterface.robot_getMotionControl(robot_interface);
            cSharpBinging_MotionControl.setSpeedFraction(motion_control, 0.3);

                // 接口调用: 关节运动
            cSharpBinging_MotionControl.moveJoint(motion_control, joint_angle1.ToArray(), 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);
                // 阻塞
                int ret = waitArrival(robot_interface);
                if (ret == 0)
                {
                    Console.WriteLine("关节运动到路点1成功");
                }
                else
                {
                    Console.WriteLine("关节运动到路点1失败");
                }

            // 接口调用: 关节运动
            cSharpBinging_MotionControl.moveJoint(motion_control, joint_angle2.ToArray(), 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);

            // 阻塞
            ret = waitArrival(robot_interface);
                if (ret == 0)
                {
                    Console.WriteLine("关节运动到路点2成功");
                }
                else
                {
                    Console.WriteLine("关节运动到路点2失败");
                }

            // 接口调用: 关节运动
            cSharpBinging_MotionControl.moveJoint(motion_control, joint_angle3.ToArray(), 80 * (M_PI / 180), 60 * (M_PI / 180), 0, 0);

            // 阻塞
            ret = waitArrival(robot_interface);
                if (ret == 0)
                {
                    Console.WriteLine("关节运动到路点3成功");
                }
                else
                {
                    Console.WriteLine("关节运动到路点3失败");
                }
                return 0;

            
        }
        static void Main_MoveJ(string[] args)
        {
            cSharpBinding_RPC csharpbingding_rpc = new cSharpBinding_RPC();
            //初始化机械臂控制库
            IntPtr rpc_client = cSharpBinding_RPC.rpc_create_client(0);
            Console.Out.WriteLine("rpc_create_client ret={0}", rpc_client);
            if (rpc_client == IntPtr.Zero)
            {
                Console.Error.WriteLine("rpc_create_client failed!");
            }

            cSharpBinding_RPC.rpc_connect(rpc_client, robotIP, serverPort);
            cSharpBinding_RPC.rpc_setRequestTimeout(rpc_client, 1000);
            cSharpBinding_RPC.rpc_login(rpc_client, "aubo", "123456");

            exampleMoveJ(rpc_client);

            cSharpBinding_RPC.rpc_logout(rpc_client);
            cSharpBinding_RPC.rpc_disconnect(rpc_client);
        }
    }
}
