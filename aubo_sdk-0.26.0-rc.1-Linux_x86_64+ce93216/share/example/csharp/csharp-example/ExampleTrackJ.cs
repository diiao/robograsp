using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.IO;
using System.Threading;

namespace csharp_example
{

    class ExampleTrackJ
    {
        const int RSERR_SUCC = 0;

        static UInt16 rshd = 0xffff;
        //机械臂IP地址
        const string robot_ip = "192.168.204.151";
        //机械臂端口号
        const int server_port = 30004;
        //M_PI
        const double M_PI = 3.14159265358979323846;

        // 实现阻塞功能: 当机械臂运动到目标路点时，程序再往下执行
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

        // 轨迹数据读取类
        class TrajectoryIo
        {
            private StreamReader input_file;

            // 构造函数，接受要打开的文件名作为参数
            public TrajectoryIo(string filename)
            {
                try
                {
                    input_file = new StreamReader(filename);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"无法打开轨迹文件. 请检查输入的文件路径是否正确. 异常信息: {ex.Message}");
                }
            }

            // 检查文件是否成功打开
            public bool Open()
            {
                return input_file != null;
            }

            // 解析文件中的轨迹数据，
            public List<List<double>> Parse()
            {
                List<List<double>> res = new List<List<double>>();
                string tmp;
                int linenum = 1;
                while ((tmp = input_file.ReadLine()) != null)
                {
                    try
                    {
                        List<double> q = Split(tmp, ",");
                        res.Add(q);
                    }
                    catch (Exception p)
                    {
                        Console.WriteLine($"Line: {linenum} \"{p.Message}\" is not a number of double");
                        break;
                    }
                    linenum++;
                }
                return res;
            }

            // 切割字符串并转换为double类型
            private List<double> Split(string str, string delim)
            {
                List<double> res = new List<double>();
                if (string.IsNullOrEmpty(str))
                {
                    return res;
                }

                string[] parts = str.Split(delim.ToCharArray(), StringSplitOptions.RemoveEmptyEntries);
                foreach (string part in parts)
                {
                    if (double.TryParse(part, out double v))
                    {
                        res.Add(v);
                    }
                    else
                    {
                        throw new Exception(part);
                    }
                }
                return res;
            }
        }

        static int exampleTrackJoint(IntPtr cli)
        {
            // 读取轨迹文件
            string filename = "../trajs/record6.offt";
            TrajectoryIo input = new TrajectoryIo(filename);

            // 尝试打开轨迹文件，如果无法打开，直接返回
            if (!input.Open())
            {
                return 0;
            }

            // 解析轨迹数据
            List<List<double>> traj = input.Parse();

            // 检查轨迹文件中是否有路点，
            if (traj.Count == 0)
            {
                Console.WriteLine("轨迹文件中的路点数量为0.");
                return 0;
            }

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

            // 接口调用: 关节运动到轨迹中的第一个点，否则容易引起较大超调
            cSharpBinging_MotionControl.moveJoint(motion_control, traj[0].ToArray(), 80 * (M_PI / 180),
                                                          60 * (M_PI / 180), 0, 0);

            // 阻塞
            int ret = waitArrival(robot_interface);
            if (ret == 0)
            {
                Console.WriteLine("关节运动到轨迹文件中的第一个路点成功");
            }
            else
            {
                Console.WriteLine("关节运动到轨迹文件中的第一个路点失败");
            }

            for (int i = 1; i < traj.Count; i++)
            {
                int traj_queue_size = cSharpBinging_MotionControl.getTrajectoryQueueSize(motion_control);
                Console.WriteLine($"traj_queue_size: {traj_queue_size}");
                while (traj_queue_size > 8)
                {
                    traj_queue_size = cSharpBinging_MotionControl.getTrajectoryQueueSize(motion_control);
                    Thread.Sleep(1);
                }

                cSharpBinging_MotionControl.moveJoint(motion_control, traj[i].ToArray(), 0.01, 0.5, 1, 0);

            }

            // 等待运动结束
            IntPtr robot_state = cSharpBinging_RobotInterface.robot_getRobotState(robot_interface);
            while (!cSharpBinging_RobotState.isSteady(robot_state))
            {
                Thread.Sleep(5);
            }

            cSharpBinging_MotionControl.stopJoint(motion_control, 1);

            Console.WriteLine("trackJoint 运动结束");

            return 0;
        }


        static void Main_TrackJ()
        {
#if NETCOREAPP
            // 获取实时优先级最大值
            int sched_max = GetMaxPriority(); 

            // 设置实时调度策略及优先级
            SchedParam sParam = new SchedParam();
            sParam.Priority = sched_max;
            SetScheduler(0, SchedType.FIFO, sParam);
            int i_schedFlag = GetScheduler(0);
            Console.WriteLine($"设置调度策略 = [{i_schedFlag}]");

            // 绑定CPU
            CpuSet cpuset = new CpuSet();
            CpuSet.Clear(cpuset);
            CpuSet.Set(1, cpuset);
            SetAffinity(0, cpuset);
#endif

            IntPtr rpc_client = cSharpBinding_RPC.rpc_create_client(0);
            Console.Out.WriteLine("rpc_create_client ret={0}", rpc_client);
            if (rpc_client == IntPtr.Zero)
            {
                Console.Error.WriteLine("rpc_create_client failed!");
            }

            cSharpBinding_RPC.rpc_connect(rpc_client, robot_ip, server_port);
            cSharpBinding_RPC.rpc_setRequestTimeout(rpc_client, 1000);
            cSharpBinding_RPC.rpc_login(rpc_client, "aubo", "123456");

            exampleTrackJoint(rpc_client);

            cSharpBinding_RPC.rpc_logout(rpc_client);
            cSharpBinding_RPC.rpc_disconnect(rpc_client);
        }



    }
}
