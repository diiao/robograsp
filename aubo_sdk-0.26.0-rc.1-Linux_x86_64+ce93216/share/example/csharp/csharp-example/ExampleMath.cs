using System;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;

namespace csharp_example
{
    class ExampleMath
    {
        const int RSERR_SUCC = 0;

        static UInt16 rshd = 0xffff;
        //机械臂IP地址
        const string robot_ip = "127.0.0.1";
        //机械臂端口号
        const int server_port = 30004;

        // 展平二维数组为一维数组（行优先）
        public static double[] flattenArray(double[][] nestedArray)
        {
            int totalSize = nestedArray.Sum(subArray => subArray.Length);
            double[] flatArray = new double[totalSize];
            int index = 0;
            foreach (var subArray in nestedArray)
            {
                foreach (var value in subArray)
                {
                    flatArray[index++] = value;
                }
            }
            return flatArray;
        }

        static int waitArrival(IntPtr robot_interface)
        {
            const int max_retry_count = 5;
            int cnt = 0;

            // 接口调用: 获取当前的运动指令 ID
            IntPtr motion_control = cSharpBinging_RobotInterface.robot_getMotionControl(robot_interface);
            int exec_id = cSharpBinging_MotionControl.getExecId(motion_control);

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


        // 打印单个变量的值和名称
        public static void printSingle<T>(T param, string name)
        {
            Console.WriteLine($"{name}: {param}");
        }
        // 打印 double[] 类型的数组
        public static void printArray(double[] param, string name)
        {
            Console.Write($"{name}: ");
            for (int i = 0; i < param.Length; i++)
            {
                Console.Write(param[i]);
                if (i != param.Length - 1)
                {
                    Console.Write(", ");
                }
            }
            Console.WriteLine();
        }
        // 打印 double[][] 类型的二维数组
        public static void print2DArray(double[][] param, string name)
        {
            Console.WriteLine($"{name}: ");
            for (int i = 0; i < param.Length; i++)
            {
                Console.Write($"  第{i + 1}组: ");
                for (int j = 0; j < param[i].Length; j++)
                {
                    Console.Write(param[i][j]);
                    if (j != param[i].Length - 1)
                    {
                        Console.Write(", ");
                    }
                    else
                    {
                        Console.WriteLine();
                    }
                }
                if (i == param.Length - 1)
                {
                    Console.WriteLine();
                }
            }
        }

        public static int exampleTcpOffsetIdentify(IntPtr cli)
        {
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


            // 调用函数
            double[][] nestedPoses = new double[][]
            {
                new double[] { 1.0, 2.0, 3.0, 4, 5, 6 },
                new double[] { 4.0, 5.0, 6.0, 4, 5, 6 }
            };

            int rows = nestedPoses.Length;

            double[] flatPoses = flattenArray(nestedPoses);

            double[] result = new double[10];
            IntPtr robot_state = cSharpBinging_RobotInterface.robot_getRobotState(robot_interface);
            cSharpBinding_Math.tcpOffsetIdentify(robot_state, flatPoses, rows, result);

            printArray(result, "tcpOffsetIdentify result: ");

            return 0;
        }

        public static void examplePoseAdd(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            double[] p1 = { 0.2, 0.5, 0.1, 1.57, 0, 0 };
            double[] p2 = { 0.2, 0.5, 0.6, 1.57, 0, 0 };

            // 接口调用: 位姿加法
            double[] result = new double[6];
            cSharpBinding_Math.poseAdd(mathPtr, p1, p2, result);

            printArray(result, "poseAdd");
        }

        public static void examplePoseSub(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            double[] p1 = { 0.2, 0.5, 0.1, 1.57, 0, 0 };
            double[] p2 = { 0.2, 0.5, 0.6, 1.57, 0, 0 };

            // 接口调用: 位姿减法
            double[] result = new double[6];
            cSharpBinding_Math.poseSub(mathPtr, p1, p2, result);

            printArray(result, "poseSub");
        }

        // 线性插值
        public static void exampleInterpolatePose(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            double[] p1 = { 0.2, 0.2, 0.4, 0, 0, 0 };
            double[] p2 = { 0.2, 0.2, 0.6, 0, 0, 0 };
            double alpha = 0.5;

            // 接口调用: 计算线性插值
            double[] result = new double[6];
            cSharpBinding_Math.interpolatePose(mathPtr, p1, p2, alpha, result);

            printArray(result, "interpolatePose");
        }

        // 位姿变换
        public static void examplePoseTrans(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            // B相对于A的位姿
            double[] F_B_A = { 0.2, 0.5, 0.1, 1.57, 0, 0 };
            // C相对于B的位姿
            double[] F_C_B = { 0.2, 0.5, 0.6, 1.57, 0, 0 };

            // 接口调用: 位姿变换获得C相对于A的位姿
            double[] F_C_A = new double[6];
            cSharpBinding_Math.poseTrans(mathPtr, F_B_A, F_C_B, F_C_A);

            printArray(F_C_A, "poseTrans");
        }

        // 位姿逆变换
        public static void examplePoseTransInv(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            // C相对于A的位姿
            double[] F_C_A = { 0.4, -0.0996016, 0.600478, 3.14, 0, 0 };
            // C相对于B的位姿
            double[] F_C_B = { 0.2, 0.5, 0.6, 1.57, 0, 0 };

            // 接口调用: 位姿逆变换获得B相对于A的位姿
            double[] F_B_A = new double[6];
            cSharpBinding_Math.poseTransInv(mathPtr, F_C_A, F_C_B, F_B_A);

            printArray(F_B_A, "poseTransInv");
        }

        // 位姿逆
        public static void examplePoseInverse(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            double[] p = { 0.2, 0.5, 0.1, 1.57, 0, 3.14 };

            // 接口调用: 获取位姿的逆
            double[] result = new double[6];
            cSharpBinding_Math.poseInverse(mathPtr, p, result);

            printArray(result, "poseInverse");
        }

        // 位姿距离
        public static void examplePoseDistance(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            double[] p1 = { 0.1, 0.3, 0.1, 0.3142, 0.0, 1.571 };
            double[] p2 = { 0.2, 0.5, 0.6, 0, -0.172, 0.0 };

            // 接口调用: 获得两个位姿的位置距离
            double result = cSharpBinding_Math.poseDistance(mathPtr, p1, p2);

            printSingle(result, "poseDistance");
        }

        // 位姿相等判断
        public static void examplePoseEqual(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            double[] p1 = { 0.1, 0.3, 0.1, 0.3142, 0.0, 1.571 };
            double[] p2 = { 0.1, 0.3, 0.1, 0.3142, 0.0, 1.5711 };

            // 接口调用: 判断两个位姿是否相等
            double eps = 0.00005;
            bool result = cSharpBinding_Math.poseEqual(mathPtr, p1, p2, eps);

            printSingle(result, "poseEqual");
        }

        // 欧拉角转四元数
        public static void exampleRpyToQuat(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            double[] rpy = { 0.611, 0.785, 0.960 };

            // 接口调用: 欧拉角转四元数
            double[] quat = new double[4];
            cSharpBinding_Math.rpyToQuaternion(mathPtr, rpy, quat);

            printArray(quat, "欧拉角 -> 四元数");
        }

        // 四元数转欧拉角
        public static void exampleQuatToRpy(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            double[] quat = { 0.834722, 0.0780426, 0.451893, 0.304864 };

            // 接口调用: 四元数转欧拉角
            double[] rpy = new double[3];
            cSharpBinding_Math.quaternionToRpy(mathPtr, quat, rpy);

            printArray(rpy, "四元数 -> 欧拉角");
        }

        // 坐标系标定
        public static void exampleCalibrateCoordinate(IntPtr cli)
        {
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);

            // 获取机器人的名字
            IntPtr[] robot_names = new IntPtr[10];
            for (int i = 0; i < 10; i++)
            {
                robot_names[i] = Marshal.AllocHGlobal(100); // 分配100字节内存用于存放字符串
            }
            int num = cSharpBinding_RPC.rpc_getRobotNames(cli, robot_names);
            if (num <= 0)
            {
                for (int i = 0; i < 10; i++)
                {
                    Marshal.FreeHGlobal(robot_names[i]); // 释放分配的内存
                }
                return;
            }
            string robot_name = Marshal.PtrToStringAnsi(robot_names[0]);
            for (int i = 0; i < 10; i++)
            {
                Marshal.FreeHGlobal(robot_names[i]); // 释放分配的内存
            }

            // 设置 TCP 偏移值
            double[] tcp_offset = { 0.17734, 0.00233, 0.14682, 0.0, 0.0, 0.0 };
            IntPtr robot_interface = cSharpBinding_RPC.rpc_getRobotInterface(cli, robot_name);
            IntPtr robot_config = cSharpBinging_RobotInterface.robot_getRobotConfig(robot_interface);
            cSharpBinging_RobotConfig.setTcpOffset(robot_config, tcp_offset);

            // TCP 在基坐标系下的位姿
            double[][] nested_coords = new double[][]
           {
                new double[]{ 0.55462, 0.06219, 0.37175, -3.142, 0.0, 1.580 },
                new double[]{ 0.63746, 0.11805, 0.37175, -3.142, 0.0, 1.580 },
                new double[]{ 0.40441, 0.28489, 0.37174, -3.142, 0.0, 1.580 },
            };
            int rows = nested_coords.Length;

            double[] flat_coords = flattenArray(nested_coords);
            // 用户坐标系标定
            double[] user_on_base = new double[6];
            cSharpBinding_Math.calibrateCoordinate(mathPtr, flat_coords, rows, 0, user_on_base);

            // 获取 TCP 在基坐标系下的当前位姿
            double[] tcp_on_base = new double[6];
            IntPtr robot_state = cSharpBinging_RobotInterface.robot_getRobotState(robot_interface);
            cSharpBinging_RobotState.getTcpPose(robot_state, tcp_on_base);

            // 获取用户坐标系相对于基坐标系的位姿的逆
            double[] tcp_on_base_inv = new double[6];
            cSharpBinding_Math.poseInverse(mathPtr, user_on_base, tcp_on_base_inv);

            // 获取 TCP 在用户坐标系下的当前位姿
            double[] tcp_on_user = new double[6];
            cSharpBinding_Math.poseTrans(mathPtr, tcp_on_base_inv, tcp_on_base, tcp_on_user);

            printArray(tcp_on_base, "TCP在基坐标系下的位姿");
            printArray(tcp_offset, "TCP偏移");
            printArray(user_on_base, "用户坐标系相对于基坐标系的位姿");
            printArray(tcp_on_user, "TCP在用户坐标系下的位姿");
        }

        // 计算另一半圆弧的中间点
        public static void exampleCalculateCircleFourthPoint(IntPtr cli)
        {
            // 获取机器人的名字
            IntPtr[] robot_names = new IntPtr[10];
            for (int i = 0; i < 10; i++)
            {
                robot_names[i] = Marshal.AllocHGlobal(100); // 分配100字节内存用于存放字符串
            }
            int num = cSharpBinding_RPC.rpc_getRobotNames(cli, robot_names);
            if (num <= 0)
            {
                for (int i = 0; i < 10; i++)
                {
                    Marshal.FreeHGlobal(robot_names[i]); // 释放分配的内存
                }
                return;
            }
            string robot_name = Marshal.PtrToStringAnsi(robot_names[0]);
            for (int i = 0; i < 10; i++)
            {
                Marshal.FreeHGlobal(robot_names[i]); // 释放分配的内存
            }

            // 设置 TCP 偏移值
            double[] tcp_offset = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            IntPtr robot_interface = cSharpBinding_RPC.rpc_getRobotInterface(cli, robot_name);
            IntPtr robot_config = cSharpBinging_RobotInterface.robot_getRobotConfig(robot_interface);
            cSharpBinging_RobotConfig.setTcpOffset(robot_config, tcp_offset);

            // 设置机械臂运动的速度比率
            IntPtr motion_control = cSharpBinging_RobotInterface.robot_getMotionControl(robot_interface);
            cSharpBinging_MotionControl.setSpeedFraction(motion_control, 0.3);

            // 关节运动到起始位置
            double[] q = { 0.00,
                          -10.43 / 180 * Math.PI,
                           87.39 / 180 * Math.PI,
                           7.82 / 180 * Math.PI,
                           90.0 / 180 * Math.PI,
                           0.0 / 180 * Math.PI };
            cSharpBinging_MotionControl.moveJoint(motion_control, q, 1.2, 1.0, 0, 0);

            // 阻塞，等待机械臂运动完成
            int ret = waitArrival(robot_interface);
            if (ret == 0)
            {
                Console.WriteLine("关节运动到初始位置成功");
            }
            else
            {
                Console.WriteLine("关节运动到初始位置失败");
            }

            // 定义三个点的位姿
            double[] p1 = { 0.5488696249770836, -0.1214996547187204, 0.2631931199112321, -3.14159198038469, -3.673205103150083e-06, 1.570796326792424 };
            double[] p2 = { 0.5488696249770835, -0.1214996547187207, 0.3599720701808493, -3.14159198038469, -3.6732051029273e-06, 1.570796326792423 };
            double[] p3 = { 0.5488696249770836, -0.0389996547187214, 0.3599720701808496, -3.141591980384691, -3.673205102557476e-06, 1.570796326792422 };

            // 直线运动到圆的第一个点
            cSharpBinging_MotionControl.moveLine(motion_control, p1, 1.2, 0.25, 0, 0);

            // 阻塞，等待机械臂运动完成
            ret = waitArrival(robot_interface);
            if (ret == 0)
            {
                Console.WriteLine("直线运动到圆的起始点成功");
            }
            else
            {
                Console.WriteLine("直线运动到圆的起始点失败");
            }

            // 计算另一半圆弧的中间点位置
            IntPtr mathPtr = cSharpBinding_RPC.rpc_getMath(cli);
            double[] p4 = new double[6];
            int retval = cSharpBinding_Math.calculateCircleFourthPoint(mathPtr, p1, p2, p3, 1, p4);

            if (retval == 0)
            {
                Console.Error.WriteLine("计算另一半圆弧的中间点失败，无法完成圆运动");
            }
            else
            {
                // 设置为固定模式
                cSharpBinging_MotionControl.setCirclePathMode(motion_control, 0);

                // 做圆弧运动
                cSharpBinging_MotionControl.moveCircle(motion_control, p2, p3, 1.2, 0.25, 0, 0);

                // 阻塞，等待机械臂运动完成
                ret = waitArrival(robot_interface);
                if (ret == -1)
                {
                    Console.WriteLine("圆运动失败");
                }

                // 做组成圆运动的另一半圆弧运动
                cSharpBinging_MotionControl.moveCircle(motion_control, p4, p1, 1.2, 0.25, 0, 0);

                // 阻塞，等待机械臂运动完成
                ret = waitArrival(robot_interface);
                if (ret == 0)
                {
                    Console.WriteLine("圆运动成功");
                }
                else
                {
                    Console.WriteLine("圆运动失败");
                }
            }
        }

        static void Main(string[] args)
        {
            cSharpBinding_RPC csharpbingding_rpc = new cSharpBinding_RPC();
            //初始化机械臂控制库
            IntPtr rpc_client = cSharpBinding_RPC.rpc_create_client(0);
            Console.Out.WriteLine("rpc_create_client ret={0}", rpc_client);
            if (rpc_client == IntPtr.Zero)
            {
                Console.Error.WriteLine("rpc_create_client failed!");
            }
            cSharpBinding_RPC.rpc_connect(rpc_client, robot_ip, server_port);
            cSharpBinding_RPC.rpc_setRequestTimeout(rpc_client, 1000);
            cSharpBinding_RPC.rpc_login(rpc_client, "aubo", "123456");

            exampleTcpOffsetIdentify(rpc_client);
            examplePoseAdd(rpc_client);
            examplePoseSub(rpc_client);
            exampleInterpolatePose(rpc_client);
            examplePoseTrans(rpc_client);
            examplePoseTransInv(rpc_client);
            examplePoseInverse(rpc_client);
            examplePoseDistance(rpc_client);
            examplePoseEqual(rpc_client);
            exampleRpyToQuat(rpc_client);
            exampleQuatToRpy(rpc_client);
            exampleCalibrateCoordinate(rpc_client);
            // exampleCalculateCircleFourthPoint(rpc_client);

            cSharpBinding_RPC.rpc_logout(rpc_client);
            cSharpBinding_RPC.rpc_disconnect(rpc_client);
        }
    }
}
