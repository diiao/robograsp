using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Text;

namespace csharp_example
{
    using ROBOT_HANDLER = IntPtr;
    using SYNC_MOVE_HANDLER = IntPtr;
    using TRACE_HANDLER = IntPtr;
    using FORCE_CONTROL_HANDLER = IntPtr;
    using IO_CONTROL_HANDLER = IntPtr;
    using MOTION_CONTROL_HANDLER = IntPtr;
    using ROBOT_ALGORITHM_HANDLER = IntPtr;
    using ROBOT_MANAGE_HANDLER = IntPtr;
    using ROBOT_CONFIG_HANDLER = IntPtr;
    using ROBOT_STATE_HANDLER = IntPtr;

    public static class GlobalConstants
    {
        // 需要根据使用的dll库更改
        public const string service_interface_dll = "aubo_sdkd.dll";
    }

    public class cSharpBinding
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;
        //关节个数
        const int ARM_DOF = 6;
        //M_PI
        const double M_PI = 3.14159265358979323846;
        //路点位置信息的表示方法
        [StructLayout(LayoutKind.Sequential)]
        public struct Pos
        {
            public double x;
            public double y;
            public double z;
        }

        //路点位置信息的表示方法
        [StructLayout(LayoutKind.Sequential)]
        public struct cartesianPos_U
        {
            // 指定数组尺寸
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public double[] positionVector;
        };

        //姿态的四元素表示方法
        [StructLayout(LayoutKind.Sequential)]
        public struct Ori
        {
            public double w;
            public double x;
            public double y;
            public double z;
        };

        //姿态的欧拉角表示方法
        [StructLayout(LayoutKind.Sequential)]
        public struct Rpy
        {
            public double rx;
            public double ry;
            public double rz;
        };

        //描述机械臂的路点信息
        [StructLayout(LayoutKind.Sequential)]
        public struct wayPoint_S
        {
            //机械臂的位置信息　X,Y,Z
            public Pos cartPos;
            //机械臂姿态信息
            public Ori orientation;
            //机械臂关节角信息
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = ARM_DOF)]
            public double[] jointpos;
        };

        [StructLayout(LayoutKind.Sequential)]
        public struct ik_solutions
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public wayPoint_S[] waypoint;
            public int solution_count;
        };


        //机械臂关节速度加速度信息
        [StructLayout(LayoutKind.Sequential)]
        public struct JointVelcAccParam
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = ARM_DOF)]
            public double[] jointPara;
        };

        //机械臂关节角度
        [StructLayout(LayoutKind.Sequential)]
        public struct JointRadian
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = ARM_DOF)]
            public double[] jointRadian;
        };

        //机械臂工具端参数
        [StructLayout(LayoutKind.Sequential)]
        public struct ToolInEndDesc
        {
            //工具相对于末端坐标系的位置
            public Pos cartPos;
            //工具相对于末端坐标系的姿态
            public Ori orientation;
        };

        //坐标系结构体
        [StructLayout(LayoutKind.Sequential)]
        public struct CoordCalibrate
        {
            //坐标系类型：当coordType==BaseCoordinate或者coordType==EndCoordinate是，下面3个参数不做处理
            public int coordType;
            //坐标系标定方法
            public int methods;
            //用于标定坐标系的３个点（关节角），对应于机械臂法兰盘中心点基于基座标系
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public JointRadian[] jointPara;
            //标定的时候使用的工具描述
            public ToolInEndDesc toolDesc;
        };


        //工具标定结构体
        [StructLayout(LayoutKind.Sequential)]
        public struct ToolCalibrate
        {
            //用于位置标定点的数量
            public int posCalibrateNum;
            //位置标定点
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public wayPoint_S[] posCalibrateWaypoint;
            //用于姿态标定点的数量
            public int oriCalibrateNum;
            //姿态标定点
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public wayPoint_S[] oriCalibrateWaypoint;
            public int CalibMethod;
        };

        //转轴定义
        [StructLayout(LayoutKind.Sequential)]
        public struct MoveRotateAxis
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public double[] rotateAxis;
        };

        //描述运动属性中的偏移属性
        [StructLayout(LayoutKind.Sequential)]
        public struct MoveRelative
        {
            //是否使能偏移
            public byte enable;
            //偏移量 x,y,z
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public float[] pos;
            //public Pos pos;
            //相对姿态偏移量
            public Ori orientation;
        };

        //该结构体描述工具惯量
        [StructLayout(LayoutKind.Sequential)]
        public struct ToolInertia
        {
            public double xx;
            public double xy;
            public double xz;
            public double yy;
            public double yz;
            public double zz;
        };

        //动力学参数
        [StructLayout(LayoutKind.Sequential)]
        public struct ToolDynamicsParam
        {
            public double positionX; //工具重心的X坐标
            public double positionY; //工具重心的Y坐标
            public double positionZ; //工具重心的Z坐标
            public double payload; //工具重量
            public ToolInertia toolInertia; //工具惯量
        };

        //机械臂事件
        [StructLayout(LayoutKind.Sequential)]
        public struct RobotEventInfo
        {
            public int eventType; //事件类型号
            public int eventCode; //事件代码
            public IntPtr eventContent; //事件内容(std::string)
        };

        //关节状态信息
        [StructLayout(LayoutKind.Sequential)]
        public struct JointStatus
        {
            public int jointCurrentI;       // 关节电流    Current of driver
            public int jointSpeedMoto;      // 关节速度    Speed of driver
            public float jointPosJ;           // 关节角      Current position in radian
            public float jointCurVol;         // 关节电压    Rated voltage of motor. Unit: mV
            public float jointCurTemp;        // 当前温度    Current temprature of joint
            public int jointTagCurrentI;    // 电机目标电流 Target current of motor
            public float jointTagSpeedMoto;   // 电机目标速度 Target speed of motor
            public float jointTagPosJ;        // 目标关节角　 Target position of joint in radian
            public short jointErrorNum;       // 关节错误码   Joint error of joint num
        };
        //机械臂诊断信息
        [StructLayout(LayoutKind.Sequential)]
        public struct RobotDiagnosis
        {
            public Byte armCanbusStatus;                // CAN通信状态:0x01~0x80：关节CAN通信错误（每个关节占用1bit） 0x00：无错误
            public float armPowerCurrent;                // 机械臂48V电源当前电流
            public float armPowerVoltage;                // 机械臂48V电源当前电压
            public Byte armPowerStatus;                 // 机械臂48V电源状态（开、关）
            public Byte contorllerTemp;                 // 控制箱温度
            public Byte contorllerHumidity;             // 控制箱湿度
            public Byte remoteHalt;                     // 远程关机信号
            public Byte softEmergency;                  // 机械臂软急停
            public Byte remoteEmergency;                // 远程急停信号
            public Byte robotCollision;                 // 碰撞检测位
            public Byte forceControlMode;               // 机械臂进入力控模式标志位
            public Byte brakeStuats;                    // 刹车状态
            public float robotEndSpeed;                  // 末端速度
            public int robotMaxAcc;                    // 最大加速度
            public Byte orpeStatus;                     // 上位机软件状态位
            public Byte enableReadPose;                 // 位姿读取使能位
            public Byte robotMountingPoseChanged;       // 安装位置状态
            public Byte encoderErrorStatus;             // 磁编码器错误状态
            public Byte staticCollisionDetect;          // 静止碰撞检测开关
            public Byte jointCollisionDetect;           // 关节碰撞检测 每个关节占用1bit 0-无碰撞 1-存在碰撞
            public Byte encoderLinesError;              // 光电编码器不一致错误 0-无错误 1-有错误
            public Byte jointErrorStatus;               // joint error status
            public Byte singularityOverSpeedAlarm;      // 机械臂奇异点过速警告
            public Byte robotCurrentAlarm;              // 机械臂电流错误警告
            public Byte toolIoError;                    // tool error
            public Byte robotMountingPoseWarning;       // 机械臂安装位置错位（只在力控模式下起作用）
            public ushort macTargetPosBufferSize;         // mac缓冲器长度          预留
            public ushort macTargetPosDataSize;           // mac缓冲器有效数据长度   预留
            public Byte macDataInterruptWarning;        // mac数据中断           预留
            public Byte controlBoardAbnormalStateFlag;  //主控板(接口板)异常状态标志
        };

        //关节版本信息
        [StructLayout(LayoutKind.Sequential)]
        public struct JointVersion
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            char[] hw_version;  //硬件版本信息
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            char[] sw_version; //固件版本信息

        };

        //机械臂ID信息
        [StructLayout(LayoutKind.Sequential)]
        public struct JointProductID
        {

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            char[] productID;

        };

        //设备信息
        [StructLayout(LayoutKind.Sequential)]
        public struct RobotDevInfo
        {
            Byte type;                       // 设备型号、芯片型号：上位机主站：0x01  接口板0x02
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            char[] revision;                // 设备版本号，eg:V1.0
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            char[] manu_id;                 // 厂家ID，"OUR "的ASCII码0x4F 55 52 00
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            char[] joint_type;              // 机械臂类型
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            JointVersion[] joint_ver;        // 机械臂关节及工具端信息
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)]
            char[] desc;                    // 设备描述字符串以0x00结束
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            JointProductID[] jointProductID; // 关节ID信息
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            char[] slave_version;           // 从设备版本号 - 字符串表示，如“V1.0.0
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            char[] extio_version;           // IO扩展板版本号 -字符串标志，如“V1.0.0

        };

        //初始化机械臂控制库
        [DllImport(service_interface_dll, EntryPoint = "rs_initialize", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_initialize();

        //反初始化机械臂控制库
        [DllImport(service_interface_dll, EntryPoint = "rs_uninitialize", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_uninitialize();

        //创建机械臂控制上下文句柄
        [DllImport(service_interface_dll, EntryPoint = "rs_create_context", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_create_context(ref UInt16 rshd);

        //注销机械臂控制上下文句柄
        [DllImport(service_interface_dll, EntryPoint = "rs_destory_context", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_destory_context(UInt16 rshd);

        //链接机械臂服务器
        [DllImport(service_interface_dll, EntryPoint = "rs_login", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_login(UInt16 rshd, [MarshalAs(UnmanagedType.LPStr)] string addr, int port);

        //断开机械臂服务器链接
        [DllImport(service_interface_dll, EntryPoint = "rs_logout", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_logout(UInt16 rshd);

        //初始化全局的运动属性
        [DllImport(service_interface_dll, EntryPoint = "rs_init_global_move_profile", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_init_global_move_profile(UInt16 rshd);

        //设置六个关节轴动的最大速度（最大为180度/秒），注意如果没有特殊需求，6个关节尽量配置成一样！
        [DllImport(service_interface_dll, EntryPoint = "rs_set_global_joint_maxvelc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_global_joint_maxvelc(UInt16 rshd, double[] max_velc);

        //获取六个关节轴动的最大速度
        [DllImport(service_interface_dll, EntryPoint = "rs_get_global_joint_maxvelc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_global_joint_maxvelc(UInt16 rshd, ref JointVelcAccParam max_velc);

        //设置六个关节轴动的最大加速度 （十倍的最大速度），注意如果没有特殊需求，6个关节尽量配置成一样！
        [DllImport(service_interface_dll, EntryPoint = "rs_set_global_joint_maxacc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_global_joint_maxacc(UInt16 rshd, double[] max_acc);

        //获取六个关节轴动的最大加速度
        [DllImport(service_interface_dll, EntryPoint = "rs_get_global_joint_maxacc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_global_joint_maxacc(UInt16 rshd, ref JointVelcAccParam max_acc);

        //设置机械臂末端最大线加速度
        [DllImport(service_interface_dll, EntryPoint = "rs_set_global_end_max_line_acc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_global_end_max_line_acc(UInt16 rshd, double max_acc);

        //设置机械臂末端最大线速度
        [DllImport(service_interface_dll, EntryPoint = "rs_set_global_end_max_line_velc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_global_end_max_line_velc(UInt16 rshd, double max_velc);

        //获取机械臂末端最大线加速度
        [DllImport(service_interface_dll, EntryPoint = "rs_get_global_end_max_line_acc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_global_end_max_line_acc(UInt16 rshd, ref double max_acc);

        //获取机械臂末端最大线速度
        [DllImport(service_interface_dll, EntryPoint = "rs_get_global_end_max_line_velc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_global_end_max_line_velc(UInt16 rshd, ref double max_velc);

        //设置机械臂末端最大角加速度
        [DllImport(service_interface_dll, EntryPoint = "rs_set_global_end_max_angle_acc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_global_end_max_angle_acc(UInt16 rshd, double max_acc);

        //设置机械臂末端最大角速度
        [DllImport(service_interface_dll, EntryPoint = "rs_set_global_end_max_angle_velc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_global_end_max_angle_velc(UInt16 rshd, double max_velc);

        //获取机械臂末端最大角加速度
        [DllImport(service_interface_dll, EntryPoint = "rs_get_global_end_max_angle_acc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_global_end_max_angle_acc(UInt16 rshd, ref double max_acc);

        //获取机械臂末端最大角加速度
        [DllImport(service_interface_dll, EntryPoint = "rs_get_global_end_max_angle_velc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_global_end_max_angle_velc(UInt16 rshd, ref double max_velc);

        //设置用户坐标系
        [DllImport(service_interface_dll, EntryPoint = "rs_set_user_coord", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_user_coord(UInt16 rshd, ref CoordCalibrate user_coord);

        //设置基座坐标系
        [DllImport(service_interface_dll, EntryPoint = "rs_set_base_coord", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_base_coord(UInt16 rshd);

        //机械臂轴动
        [DllImport(service_interface_dll, EntryPoint = "rs_move_joint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_joint(UInt16 rshd, double[] joint_radia, bool isblock);

        //机械臂直线运动
        [DllImport(service_interface_dll, EntryPoint = "rs_move_line", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_line(UInt16 rshd, double[] joint_radia, bool isblock);

        //机械臂直线运动
        [DllImport(service_interface_dll, EntryPoint = "rs_move_rotate_to_waypoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_rotate_to_waypoint(UInt16 rshd, ref wayPoint_S target_waypoint, bool isblock);

        //保持当前位置变换姿态做旋转运动
        [DllImport(service_interface_dll, EntryPoint = "rs_move_rotate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_rotate(UInt16 rshd, ref CoordCalibrate user_coord, ref MoveRotateAxis rotate_axis, double rotate_angle, bool isblock);

        //根据当前路点信息获取姿态旋转变换目标路点
        [DllImport(service_interface_dll, EntryPoint = "rs_get_rotate_target_waypiont", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_rotate_target_waypiont(UInt16 rshd, ref wayPoint_S source_waypoint, double[] rotate_axis_on_basecoord, double rotate_angle, ref wayPoint_S target_waypoint);

        //将用户坐标系下描述的坐标轴变换到基坐标系下描述
        [DllImport(service_interface_dll, EntryPoint = "rs_get_rotateaxis_user_to_Base", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_rotateaxis_user_to_Base(UInt16 rshd, ref Ori ori_usercoord, double[] rotate_axis_on_usercoord, double[] rotate_axis_on_basecoord);

        //根据位置获取目标路点信息(获取基于基座标下的目标路点通过基于用户坐标系的位置，目标路点保持起点姿态)
        [DllImport(service_interface_dll, EntryPoint = "rs_get_target_waypoint_by_position", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_target_waypoint_by_position(UInt16 rshd, ref wayPoint_S source_waypoint_on_basecoord, ref CoordCalibrate usercoord, ref Pos tool_End_Position, ref ToolInEndDesc toolInEndDesc, ref wayPoint_S target_waypoint_on_basecoord);

        //清除所有已经设置的全局路点
        [DllImport(service_interface_dll, EntryPoint = "rs_remove_all_waypoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_remove_all_waypoint(UInt16 rshd);

        //添加全局路点用于轨迹运动
        [DllImport(service_interface_dll, EntryPoint = "rs_add_waypoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_add_waypoint(UInt16 rshd, double[] joint_radia);

        //设置交融半径
        [DllImport(service_interface_dll, EntryPoint = "rs_set_blend_radius", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_blend_radius(UInt16 rshd, double radius);

        //设置圆运动圈数
        [DllImport(service_interface_dll, EntryPoint = "rs_set_circular_loop_times", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_circular_loop_times(UInt16 rshd, int times);

        //检查用户坐标系参数设置是否合理
        [DllImport(service_interface_dll, EntryPoint = "rs_check_user_coord", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_check_user_coord(UInt16 rshd, ref CoordCalibrate user_coord);

        //用户坐标系标定
        [DllImport(service_interface_dll, EntryPoint = "rs_user_coord_calibrate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_user_coord_calibrate(UInt16 rshd, ref CoordCalibrate user_coord, double[] bInWPos, double[] bInWOri, double[] wInBPos);

        //工具标定
        [DllImport(service_interface_dll, EntryPoint = "rs_tool_calibration", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_tool_calibration(UInt16 rshd, ref ToolCalibrate toolCalibrate, ref ToolInEndDesc toolInEndDesc);

        //设置基于基座标系运动偏移量
        [DllImport(service_interface_dll, EntryPoint = "rs_set_relative_offset_on_base", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_relative_offset_on_base(UInt16 rshd, ref MoveRelative relative);

        //设置基于用户标系运动偏移量
        [DllImport(service_interface_dll, EntryPoint = "rs_set_relative_offset_on_user", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_relative_offset_on_user(UInt16 rshd, ref MoveRelative relative, ref CoordCalibrate user_coord);

        //取消提前到位设置
        [DllImport(service_interface_dll, EntryPoint = "rs_set_no_arrival_ahead", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_no_arrival_ahead(UInt16 rshd);

        //设置距离模式下的提前到位距离
        [DllImport(service_interface_dll, EntryPoint = "rs_set_arrival_ahead_distance", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_arrival_ahead_distance(UInt16 rshd, double distance);

        //设置时间模式下的提前到位时间
        [DllImport(service_interface_dll, EntryPoint = "rs_set_arrival_ahead_time", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_arrival_ahead_time(UInt16 rshd, double sec);

        //轨迹运动
        [DllImport(service_interface_dll, EntryPoint = "rs_move_track", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_track(UInt16 rshd, int sub_move_mode, bool isblock);

        //保持当前位姿通过直线运动的方式运动到目标位置
        [DllImport(service_interface_dll, EntryPoint = "rs_move_line_to", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_line_to(UInt16 rshd, ref Pos target, ref ToolInEndDesc tool, bool isblock);

        //保持当前位姿通过关节运动的方式运动到目标位置
        [DllImport(service_interface_dll, EntryPoint = "rs_move_joint_to", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_joint_to(UInt16 rshd, ref Pos target, ref ToolInEndDesc tool, bool isblock);

        //获取机械臂当前位置信息
        [DllImport(service_interface_dll, EntryPoint = "rs_get_current_waypoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_current_waypoint(UInt16 rshd, ref wayPoint_S waypoint);

        //正解
        [DllImport(service_interface_dll, EntryPoint = "rs_forward_kin", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_forward_kin(UInt16 rshd, double[] joint_radia, ref wayPoint_S waypoint);

        //逆解
        [DllImport(service_interface_dll, EntryPoint = "rs_inverse_kin", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_inverse_kin(UInt16 rshd, double[] joint_radia, ref Pos pos, ref Ori ori, ref wayPoint_S waypoint);

        //逆解 （最多八组解）
        [DllImport(service_interface_dll, EntryPoint = "rs_inverse_kin_closed_form", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_inverse_kin_closed_form(UInt16 rshd, ref Pos pos, ref Ori ori, ref ik_solutions waypoint);

        //四元素转欧拉角
        [DllImport(service_interface_dll, EntryPoint = "rs_rpy_to_quaternion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_rpy_to_quaternion(UInt16 rshd, ref Rpy rpy, ref Ori ori);

        //欧拉角转四元素
        [DllImport(service_interface_dll, EntryPoint = "rs_quaternion_to_rpy", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_quaternion_to_rpy(UInt16 rshd, ref Ori ori, ref Rpy rpy);

        //基座坐标系转用户坐标系
        [DllImport(service_interface_dll, EntryPoint = "rs_base_to_user", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_base_to_user(UInt16 rshd, ref Pos pos_onbase, ref Ori ori_onbase, ref CoordCalibrate user_coord, ref ToolInEndDesc tool_pos, ref Pos pos_onuser, ref Ori ori_onuser);

        //用户坐标系转基座坐标系
        [DllImport(service_interface_dll, EntryPoint = "rs_user_to_base", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_user_to_base(UInt16 rshd, ref Pos pos_onuser, ref Ori ori_onuser, ref CoordCalibrate user_coord, ref ToolInEndDesc tool_pos, ref Pos pos_onbase, ref Ori ori_onbase);

        //基坐标系转基座标得到工具末端点的位置和姿态
        [DllImport(service_interface_dll, EntryPoint = "rs_base_to_base_additional_tool", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_base_to_base_additional_tool(UInt16 rshd, ref Pos flange_center_pos_onbase, ref Ori flange_center_ori_onbase, ref ToolInEndDesc tool_pos, ref Pos tool_end_pos_onbase, ref Ori tool_end_ori_onbase);

        //设置工具的运动学参数
        [DllImport(service_interface_dll, EntryPoint = "rs_set_tool_end_param", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_tool_end_param(UInt16 rshd, ref ToolInEndDesc tool);

        //设置无工具的动力学参数
        [DllImport(service_interface_dll, EntryPoint = "rs_set_none_tool_dynamics_param", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_none_tool_dynamics_param(UInt16 rshd);

        //根据接口板IO类型和地址设置IO状态
        [DllImport(service_interface_dll, EntryPoint = "rs_set_board_io_status_by_addr", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_board_io_status_by_addr(UInt16 rshd, int io_type, int addr, double val);

        //根据接口板IO类型和地址获取IO状态
        [DllImport(service_interface_dll, EntryPoint = "rs_get_board_io_status_by_addr", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_board_io_status_by_addr(UInt16 rshd, int io_type, int addr, ref double val);

        //设置工具端IO状态
        [DllImport(service_interface_dll, EntryPoint = "rs_set_tool_do_status", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_tool_do_status(UInt16 rshd, string name, int val);

        //获取工具端IO状态
        [DllImport(service_interface_dll, EntryPoint = "rs_get_tool_io_status", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_tool_io_status(UInt16 rshd, string name, ref double val);

        //设置工具端电源电压类型
        [DllImport(service_interface_dll, EntryPoint = "rs_set_tool_power_type", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_tool_power_type(UInt16 rshd, int type);

        //获取工具端电源电压类型
        [DllImport(service_interface_dll, EntryPoint = "rs_get_tool_power_type", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_tool_power_type(UInt16 rshd, ref int type);

        //设置工具端数字量IO的类型
        [DllImport(service_interface_dll, EntryPoint = "rs_set_tool_io_type", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_tool_io_type(UInt16 rshd, int addr, int type);

        //设置工具的动力学参数
        [DllImport(service_interface_dll, EntryPoint = "rs_set_tool_dynamics_param", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_tool_dynamics_param(UInt16 rshd, ref ToolDynamicsParam tool);

        //获取工具的动力学参数
        [DllImport(service_interface_dll, EntryPoint = "rs_get_tool_dynamics_param", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_tool_dynamics_param(UInt16 rshd, ref ToolDynamicsParam tool);

        //设置无工具运动学参数
        [DllImport(service_interface_dll, EntryPoint = "rs_set_none_tool_kinematics_param", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_none_tool_kinematics_param(UInt16 rshd);

        //设置工具的运动学参数
        [DllImport(service_interface_dll, EntryPoint = "rs_set_tool_kinematics_param", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_tool_kinematics_param(UInt16 rshd, ref ToolInEndDesc tool);

        //获取工具的运动学参数
        [DllImport(service_interface_dll, EntryPoint = "rs_get_tool_kinematics_param", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_tool_kinematics_param(UInt16 rshd, ref ToolInEndDesc tool);

        //启动机械臂
        [DllImport(service_interface_dll, EntryPoint = "rs_robot_startup", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_robot_startup(UInt16 rshd, ref ToolDynamicsParam tool, byte colli_class, bool read_pos, bool static_colli_detect, int board_maxacc, ref int state);

        //关闭机械臂
        [DllImport(service_interface_dll, EntryPoint = "rs_robot_shutdown", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_robot_shutdown(UInt16 rshd);

        //关闭机械臂
        [DllImport(service_interface_dll, EntryPoint = "rs_robot_control", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_robot_control(UInt16 rshd, int robotControlCommand);

        //通知机械臂工程启动，服务器同时开始检测安全IO
        [DllImport(service_interface_dll, EntryPoint = "rs_project_startup", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_project_startup(UInt16 rshd);

        //通知机械臂工程停止，服务器停止检测安全IO
        [DllImport(service_interface_dll, EntryPoint = "rs_project_stop", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_project_stop(UInt16 rshd);

        //停止机械臂运动
        [DllImport(service_interface_dll, EntryPoint = "rs_move_stop", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_stop(UInt16 rshd);

        //停止机械臂运动
        [DllImport(service_interface_dll, EntryPoint = "rs_move_fast_stop", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_fast_stop(UInt16 rshd);

        //暂停机械臂运动
        [DllImport(service_interface_dll, EntryPoint = "rs_move_pause", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_pause(UInt16 rshd);

        //暂停后回复机械臂运动
        [DllImport(service_interface_dll, EntryPoint = "rs_move_continue", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_move_continue(UInt16 rshd);

        //机械臂碰撞后恢复
        [DllImport(service_interface_dll, EntryPoint = "rs_collision_recover", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_collision_recover(UInt16 rshd);

        //获取机械臂当前状态
        [DllImport(service_interface_dll, EntryPoint = "rs_get_robot_state", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_robot_state(UInt16 rshd, ref int state);

        //获取关节状态信息
        [DllImport(service_interface_dll, EntryPoint = "rs_get_joint_status", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_joint_status(UInt16 rshd, IntPtr pBuff);

        //获取机械臂诊断信息
        [DllImport(service_interface_dll, EntryPoint = "rs_get_diagnosis_info", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_diagnosis_info(UInt16 rshd, ref RobotDiagnosis robotDiagnosis);

        //获取机械臂诊断信息
        [DllImport(service_interface_dll, EntryPoint = "rs_get_device_info", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_device_info(UInt16 rshd, ref RobotDevInfo dev);

        //设置机械臂服务器工作模式
        [DllImport(service_interface_dll, EntryPoint = "rs_set_work_mode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_work_mode(UInt16 rshd, int state);

        //获取机械臂服务器当前工作模式
        [DllImport(service_interface_dll, EntryPoint = "rs_get_work_mode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_work_mode(UInt16 rshd, ref int state);

        //设置机械臂碰撞等级
        [DllImport(service_interface_dll, EntryPoint = "rs_set_collision_class", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_set_collision_class(UInt16 rshd, int grade);

        //获取当前碰撞等级
        [DllImport(service_interface_dll, EntryPoint = "rs_get_collision_class", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_collision_class(UInt16 rshd, ref int grade);

        //根据错误号返回错误信息
        [DllImport(service_interface_dll, EntryPoint = "rs_get_error_information_by_errcode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rs_get_error_information_by_errcode(UInt16 rshd, int err_code);

        //获取socket链接状态
        [DllImport(service_interface_dll, EntryPoint = "rs_get_socket_status", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_get_socket_status(UInt16 rshd, ref byte connected);

        //设置是否允许实时路点信息推送
        [DllImport(service_interface_dll, EntryPoint = "rs_enable_push_realtime_roadpoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rs_enable_push_realtime_roadpoint(UInt16 rshd, bool enable);

        //实时路点回调函数
        [System.Runtime.InteropServices.UnmanagedFunctionPointerAttribute(System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public delegate void REALTIME_ROADPOINT_CALLBACK(ref wayPoint_S waypoint, IntPtr arg);

        [DllImport(service_interface_dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void rs_setcallback_realtime_roadpoint(UInt16 rshd, [MarshalAs(UnmanagedType.FunctionPtr)] REALTIME_ROADPOINT_CALLBACK CurrentPositionCallback, IntPtr arg);

        //实时末端速度回调函数
        [System.Runtime.InteropServices.UnmanagedFunctionPointerAttribute(System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public delegate void REALTIME_ENDSPEED_CALLBACK(double speed, IntPtr arg);

        [DllImport(service_interface_dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void rs_setcallback_realtime_end_speed(UInt16 rshd, [MarshalAs(UnmanagedType.FunctionPtr)] REALTIME_ENDSPEED_CALLBACK CurrentEndSpeedCallback, IntPtr arg);


        //机械臂事件回调
        [System.Runtime.InteropServices.UnmanagedFunctionPointerAttribute(System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public delegate void ROBOT_EVENT_CALLBACK(ref RobotEventInfo rs_event, IntPtr arg);

        [DllImport(service_interface_dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void rs_setcallback_robot_event(UInt16 rshd, [MarshalAs(UnmanagedType.FunctionPtr)] ROBOT_EVENT_CALLBACK RobotEventCallback, IntPtr arg);

        //机械臂关节状态回调
        [System.Runtime.InteropServices.UnmanagedFunctionPointerAttribute(System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public delegate void ROBOT_JOINT_STATUS_CALLBACK(IntPtr pBuff, int size, IntPtr arg);

        [DllImport(service_interface_dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void rs_setcallback_realtime_joint_status(UInt16 rshd, [MarshalAs(UnmanagedType.FunctionPtr)] ROBOT_JOINT_STATUS_CALLBACK RobotJointStatusCallback, IntPtr arg);


        //位置回调
        public static void CurrentPositionCallback(ref wayPoint_S waypoint, IntPtr arg)
        {
            PrintWaypoint(waypoint);
        }
        //速度回调
        public static void CurrentEndSpeedCallback(double speed, IntPtr arg)
        {
            Console.Out.WriteLine("CurrentSpeed:{0}\n", speed);
        }


        //关节状态回调
        public static void CurrentJointStatusCallback(IntPtr pBuff, int size, IntPtr arg)
        {
            cSharpBinding.JointStatus[] jointStatus = new cSharpBinding.JointStatus[6];
            for (int i = 0; i < 6; i++)
            {
                IntPtr pPonitor = new IntPtr(pBuff.ToInt64() + Marshal.SizeOf(typeof(cSharpBinding.JointStatus)) * i);
                jointStatus[i] = (cSharpBinding.JointStatus)Marshal.PtrToStructure(pPonitor, typeof(cSharpBinding.JointStatus));
            }
            for (int i = 0; i < 6; i++)
            {
                Console.Out.WriteLine("jointStatus {0}", i + 1);
                Console.Out.WriteLine("joint I: {0}\njoint Speed: {1}\njoint Angel: {2}", jointStatus[i].jointCurrentI, jointStatus[i].jointSpeedMoto, jointStatus[i].jointPosJ);
                Console.Out.WriteLine("joint vol: {0}\njoint temp: {1}\njoint target I: {2}", jointStatus[i].jointCurVol, jointStatus[i].jointCurTemp, jointStatus[i].jointTagCurrentI);
                Console.Out.WriteLine("joint target Speed: {0}\njoint Target Angel: {1}\njoint Errnum: {2}", jointStatus[i].jointTagSpeedMoto, jointStatus[i].jointTagPosJ, jointStatus[i].jointErrorNum);
                Console.Out.WriteLine("---------------------------------------------------------------------------------------");

            }

        }


        //打印路点信息
        public static void PrintWaypoint(wayPoint_S point)
        {
            Console.Out.WriteLine("---------------------------------------------------------------------------------------");
            Console.Out.WriteLine("pos.x={0} y={1} z={2}", point.cartPos.x, point.cartPos.y, point.cartPos.z);
            Console.Out.WriteLine("ori.w={0} x={1} y={2} z={3}", point.orientation.w, point.orientation.x, point.orientation.y, point.orientation.z);
            Console.Out.WriteLine("joint1={0} joint2={1} joint3={2}", point.jointpos[0] * 180 / M_PI, point.jointpos[1] * 180 / M_PI, point.jointpos[2] * 180 / M_PI);
            Console.Out.WriteLine("joint4={0} joint5={1} joint6={2}", point.jointpos[3] * 180 / M_PI, point.jointpos[4] * 180 / M_PI, point.jointpos[5] * 180 / M_PI);
            Console.Out.WriteLine("---------------------------------------------------------------------------------------");
        }

        public static void RobotEventCallback(ref RobotEventInfo rs_event, IntPtr arg)
        {
            Console.Out.WriteLine("---------------------------------------------------------------------------------------");
            Console.Out.WriteLine("robot event.type={0}", rs_event.eventType);
            Console.Out.WriteLine("robot event.eventCode={0}", rs_event.eventCode);
            Console.Out.WriteLine("robot event.eventContent={0}", Marshal.PtrToStringAnsi(rs_event.eventContent));
            Console.Out.WriteLine("---------------------------------------------------------------------------------------");
        }

    }

    public class cSharpBinding_RPC
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;
        //初始化机械臂控制库
        [DllImport(service_interface_dll, EntryPoint = "rpc_create_client", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_create_client(int mode);

        //反初始化机械臂控制库
        [DllImport(service_interface_dll, EntryPoint = "rpc_destroy_client", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern void rpc_destroy_client(IntPtr cli);

        //设置日志处理器
        [DllImport(service_interface_dll, EntryPoint = "rpc_setLogHandler", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern void rpc_setLogHandler(IntPtr cli, LogHandlerDelegate handler);
        // 定义日志处理函数委托类型，需和C++中函数原型对应
        public delegate void LogHandlerDelegate(int level, string filename, int line, string message);

        //连接到RPC服务
        [DllImport(service_interface_dll, EntryPoint = "rpc_connect", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpc_connect(IntPtr cli, [MarshalAs(UnmanagedType.LPStr)] string ip, int port);

        //断开RPC连接
        [DllImport(service_interface_dll, EntryPoint = "rpc_disconnect", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpc_disconnect(IntPtr cli);

        //判断是否连接RPC
        [DllImport(service_interface_dll, EntryPoint = "rpc_hasConnected", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool rpc_hasConnected(IntPtr cli);

        //登录
        [DllImport(service_interface_dll, EntryPoint = "rpc_login", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpc_login(IntPtr cli, [MarshalAs(UnmanagedType.LPStr)] string usrname, [MarshalAs(UnmanagedType.LPStr)] string passwd);

        //登出
        [DllImport(service_interface_dll, EntryPoint = "rpc_logout", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpc_logout(IntPtr cli);

        //判断是否登录
        [DllImport(service_interface_dll, EntryPoint = "rpc_hasLogined", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool rpc_hasLogined(IntPtr cli);

        //设置RPC请求超时时间
        [DllImport(service_interface_dll, EntryPoint = "rpc_setRequestTimeout", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpc_setRequestTimeout(IntPtr cli, int timeout);

        //设置事件处理
        [DllImport(service_interface_dll, EntryPoint = "rpc_setEventHandler", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpc_setEventHandler(IntPtr cli, EventCallbackDelegate cb);
        // 定义事件回调委托类型，需根据实际C++中回调函数原型补充准确参数等定义
        public delegate void EventCallbackDelegate();

        //返回错误代码
        [DllImport(service_interface_dll, EntryPoint = "rpc_errorCode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpc_errorCode(IntPtr cli);

        //设备关机
        [DllImport(service_interface_dll, EntryPoint = "rpc_shutdown", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpc_shutdown(IntPtr cli);

        //获取纯数学相关接口
        [DllImport(service_interface_dll, EntryPoint = "rpc_getMath", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_getMath(IntPtr cli);

        //获取系统信息
        [DllImport(service_interface_dll, EntryPoint = "rpc_getSystemInfo", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_getSystemInfo(IntPtr cli);

        //获取运行时接口
        [DllImport(service_interface_dll, EntryPoint = "rpc_getRuntimeMachine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_getRuntimeMachine(IntPtr cli);

        //对外寄存器接口
        [DllImport(service_interface_dll, EntryPoint = "rpc_getRegisterControl", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_getRegisterControl(IntPtr cli);

        //获取机器人列表
        [DllImport(service_interface_dll, EntryPoint = "rpc_getRobotNames", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpc_getRobotNames(IntPtr cli, IntPtr[] names);

        //根据名字获取RobotInterfacePtr接口
        [DllImport(service_interface_dll, EntryPoint = "rpc_getRobotInterface", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_getRobotInterface(IntPtr cli, [MarshalAs(UnmanagedType.LPStr)] string name);

        //获取外部轴列表
        [DllImport(service_interface_dll, EntryPoint = "rpc_getAxisNames", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpc_getAxisNames(IntPtr cli, [MarshalAs(UnmanagedType.LPArray)] string[] names);

        //获取外部轴接口
        [DllImport(service_interface_dll, EntryPoint = "rpc_getAxisInterface", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_getAxisInterface(IntPtr cli, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 获取socket
        [DllImport(service_interface_dll, EntryPoint = "rpc_getSocket", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_getSocket(IntPtr cli);

        // 获取Serial
        [DllImport(service_interface_dll, EntryPoint = "rpc_getSerial", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_getSerial(IntPtr cli);

        //获取同步运动接口
        [DllImport(service_interface_dll, EntryPoint = "rpc_getSyncMove", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_getSyncMove(IntPtr cli, [MarshalAs(UnmanagedType.LPStr)] string name);

        //获取告警信息接口
        [DllImport(service_interface_dll, EntryPoint = "rpc_getTrace", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rpc_getTrace(IntPtr cli, [MarshalAs(UnmanagedType.LPStr)] string name);

    }


    public class cSharpBinding_RTDE
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // OutputBuilder
        [DllImport(service_interface_dll, EntryPoint = "rtde_pushInt", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_pushInt(IntPtr builder, int val);

        [DllImport(service_interface_dll, EntryPoint = "rtde_pushDouble", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_pushDouble(IntPtr builder, double val);

        [DllImport(service_interface_dll, EntryPoint = "rtde_pushVectorDouble", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_pushVectorDouble(IntPtr builder, double[] val, int size);

        [DllImport(service_interface_dll, EntryPoint = "rtde_pushTupleIntBool", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_pushTupleIntBool(IntPtr builder, int first, bool second);

        [DllImport(service_interface_dll, EntryPoint = "rtde_pushInt16", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_pushInt16(IntPtr builder, short val);

        [DllImport(service_interface_dll, EntryPoint = "rtde_pushVectorInt16", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_pushVectorInt16(IntPtr builder, short[] val, int size);

        [DllImport(service_interface_dll, EntryPoint = "rtde_pushVectorInt", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_pushVectorInt(IntPtr builder, int[] val, int size);

        [DllImport(service_interface_dll, EntryPoint = "rtde_pushString", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_pushString(IntPtr builder, string val);

        [DllImport(service_interface_dll, EntryPoint = "rtde_pushChar", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_pushChar(IntPtr builder, char val);

        [DllImport(service_interface_dll, EntryPoint = "rtde_pushRtdeRecipe", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_pushRtdeRecipe(IntPtr builder, ref cSharpBinging_TypeDef.RtdeRecipe val);
    

        // InputParser
        [DllImport(service_interface_dll, EntryPoint = "rtde_popBool", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool rtde_popBool(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popInt32", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_popInt32(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popInt64", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern long rtde_popInt64(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popInt16", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern short rtde_popInt16(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popDouble", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double rtde_popDouble(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popChar", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern char rtde_popChar(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popVectorInt", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_popVectorInt(IntPtr parser, [In, Out] int[] data, int size);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popVectorInt16", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_popVectorInt16(IntPtr parser, [In, Out] short[] data, int size);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popVectorDouble", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_popVectorDouble(IntPtr parser, [In, Out] double[] data, int size);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popVectorVectorDouble", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_popVectorVectorDouble(IntPtr parser, [In, Out] IntPtr[] data, int rows, int cols);

        // 枚举类型
        [DllImport(service_interface_dll, EntryPoint = "rtde_popVectorJointStateType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.JointStateType rtde_popVectorJointStateType(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popRobotModeType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.RobotModeType rtde_popRobotModeType(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popOperationalModeType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.OperationalModeType rtde_popOperationalModeType(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popSafetyModeType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.SafetyModeType rtde_popSafetyModeType(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popRuntimeState", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.RuntimeState rtde_popRuntimeState(IntPtr parser);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popRobotMsgVector", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_popRobotMsgVector(IntPtr parser, ref cSharpBinging_TypeDef.RobotMsgVector robot_msg_vec);

        [DllImport(service_interface_dll, EntryPoint = "rtde_popPayload", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_popPayload(IntPtr parser, ref cSharpBinging_TypeDef.Payload payload);


        // RtdeClient
        [DllImport(service_interface_dll, EntryPoint = "rtde_create_client", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rtde_create_client(int mode);

        [DllImport(service_interface_dll, EntryPoint = "rtde_destroy_client", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern void rtde_destroy_client(IntPtr cli);

        [DllImport(service_interface_dll, EntryPoint = "rtde_setLogHandler", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern void rtde_setLogHandler(IntPtr cli, LogHandlerDelegate handler);
        public delegate void LogHandlerDelegate(int level, string filename, int line, string message);

        [DllImport(service_interface_dll, EntryPoint = "rtde_connect", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_connect(IntPtr cli, [MarshalAs(UnmanagedType.LPStr)] string ip, int port);

        [DllImport(service_interface_dll, EntryPoint = "rtde_hasConnected", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool rtde_hasConnected(IntPtr cli);

        [DllImport(service_interface_dll, EntryPoint = "rtde_login", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_login(IntPtr cli, [MarshalAs(UnmanagedType.LPStr)] string username, [MarshalAs(UnmanagedType.LPStr)] string password);

        [DllImport(service_interface_dll, EntryPoint = "rtde_hasLogined", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool rtde_hasLogined(IntPtr cli);

        [DllImport(service_interface_dll, EntryPoint = "rtde_logout", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_logout(IntPtr cli);

        [DllImport(service_interface_dll, EntryPoint = "rtde_disconnect", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_disconnect(IntPtr cli);

        [DllImport(service_interface_dll, EntryPoint = "rtde_getProtocolVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_getProtocolVersion(IntPtr cli);

        [DllImport(service_interface_dll, EntryPoint = "rtde_setTopic", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_setTopic(IntPtr cli, [MarshalAs(UnmanagedType.I1)] bool to_server, [MarshalAs(UnmanagedType.LPStr)] string names, double freq, int expected_chanel);

        [DllImport(service_interface_dll, EntryPoint = "rtde_removeTopic", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_removeTopic(IntPtr cli, [MarshalAs(UnmanagedType.I1)] bool to_server, int chanel);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void SubscribeCallbackDelegate(IntPtr parser);
        [DllImport(service_interface_dll, EntryPoint = "rtde_subscribe", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_subscribe(IntPtr cli, int chanel, SubscribeCallbackDelegate callback);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void PublishCallbackDelegate(IntPtr builder);
        [DllImport(service_interface_dll, EntryPoint = "rtde_publish", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtde_publish(IntPtr cli, int chanel, PublishCallbackDelegate callback);

    }


    public class cSharpBinding_Math
    {

        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 位姿相加函数封装
        [DllImport(service_interface_dll, EntryPoint = "poseAdd", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int poseAdd(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] p1, [MarshalAs(UnmanagedType.LPArray)] double[] p2, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 位姿相减函数封装
        [DllImport(service_interface_dll, EntryPoint = "poseSub", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int poseSub(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] p1, [MarshalAs(UnmanagedType.LPArray)] double[] p2, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 位姿插值函数封装
        [DllImport(service_interface_dll, EntryPoint = "interpolatePose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int interpolatePose(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] p1, [MarshalAs(UnmanagedType.LPArray)] double[] p2, double alpha, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 位姿变换函数封装
        [DllImport(service_interface_dll, EntryPoint = "poseTrans", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int poseTrans(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose_from, [MarshalAs(UnmanagedType.LPArray)] double[] pose_from_to, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 位姿逆变换函数封装
        [DllImport(service_interface_dll, EntryPoint = "poseTransInv", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int poseTransInv(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose_from, [MarshalAs(UnmanagedType.LPArray)] double[] pose_to_from, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 位姿求逆函数封装
        [DllImport(service_interface_dll, EntryPoint = "poseInverse", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int poseInverse(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 位姿距离计算函数封装
        [DllImport(service_interface_dll, EntryPoint = "poseDistance", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double poseDistance(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] p1, [MarshalAs(UnmanagedType.LPArray)] double[] p2);

        // 位姿角度距离计算函数封装
        [DllImport(service_interface_dll, EntryPoint = "poseAngleDistance", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double poseAngleDistance(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] p1, [MarshalAs(UnmanagedType.LPArray)] double[] p2);

        // 位姿相等判断函数封装
        [DllImport(service_interface_dll, EntryPoint = "poseEqual", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool poseEqual(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] p1, [MarshalAs(UnmanagedType.LPArray)] double[] p2, double eps);

        // 参考系转换函数封装
        [DllImport(service_interface_dll, EntryPoint = "transferRefFrame", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int transferRefFrame(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] F_b_a_old, [MarshalAs(UnmanagedType.LPArray)] double[] V_in_a, int type, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 位姿旋转函数封装
        [DllImport(service_interface_dll, EntryPoint = "poseRotation", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int poseRotation(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, [MarshalAs(UnmanagedType.LPArray)] double[] rotv, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 欧拉角转四元数函数封装
        [DllImport(service_interface_dll, EntryPoint = "rpyToQuaternion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rpyToQuaternion(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] rpy, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 四元数转欧拉角函数封装
        [DllImport(service_interface_dll, EntryPoint = "quaternionToRpy", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int quaternionToRpy(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] quant, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // TCP偏移识别函数封装
        [DllImport(service_interface_dll, EntryPoint = "tcpOffsetIdentify", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int tcpOffsetIdentify(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] poses, int rows, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 坐标校准函数封装
        [DllImport(service_interface_dll, EntryPoint = "calibrateCoordinate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int calibrateCoordinate(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] poses, int rows, int type, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 计算圆上第四点函数封装
        [DllImport(service_interface_dll, EntryPoint = "calculateCircleFourthPoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int calculateCircleFourthPoint(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] p1, [MarshalAs(UnmanagedType.LPArray)] double[] p2, [MarshalAs(UnmanagedType.LPArray)] double[] p3, int mode, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 力的变换函数封装
        [DllImport(service_interface_dll, EntryPoint = "forceTrans", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int forceTrans(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose_a_in_b, [MarshalAs(UnmanagedType.LPArray)] double[] force_in_a, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 根据传感器距离获取位姿增量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDeltaPoseBySensorDistance", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getDeltaPoseBySensorDistance(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] distances, double position, double radius, double track_scale, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 位姿增量变换函数封装
        [DllImport(service_interface_dll, EntryPoint = "deltaPoseTrans", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int deltaPoseTrans(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose_a_in_b, [MarshalAs(UnmanagedType.LPArray)] double[] ft_in_a, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 位姿增量相加函数封装
        [DllImport(service_interface_dll, EntryPoint = "deltaPoseAdd", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int deltaPoseAdd(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose_a_in_b, [MarshalAs(UnmanagedType.LPArray)] double[] v_in_b, [MarshalAs(UnmanagedType.LPArray)] double[] result);
    }


    public class cSharpBinding_RegisterControl
    {

        const string service_interface_dll = GlobalConstants.service_interface_dll;


        // 获取布尔型输入值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getBoolInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool getBoolInput(IntPtr h, int address);

        // 设置布尔型输入值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setBoolInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setBoolInput(IntPtr h, int address, bool value);

        // 获取Int32型输入值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getInt32Input", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getInt32Input(IntPtr h, int address);

        // 设置Int32型输入值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setInt32Input", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setInt32Input(IntPtr h, int address, int value);

        // 获取Float型输入值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getFloatInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern float getFloatInput(IntPtr h, int address);

        // 设置Float型输入值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setFloatInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setFloatInput(IntPtr h, int address, float value);

        // 获取Double型输入值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDoubleInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getDoubleInput(IntPtr h, int address);

        // 设置Double型输入值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setDoubleInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setDoubleInput(IntPtr h, int address, double value);

        // 获取布尔型输出值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getBoolOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool getBoolOutput(IntPtr h, int address);

        // 设置布尔型输出值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setBoolOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setBoolOutput(IntPtr h, int address, bool value);

        // 获取Int32型输出值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getInt32Output", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getInt32Output(IntPtr h, int address);

        // 设置Int32型输出值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setInt32Output", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setInt32Output(IntPtr h, int address, int value);

        // 获取Float型输出值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getFloatOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern float getFloatOutput(IntPtr h, int address);

        // 设置Float型输出值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setFloatOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setFloatOutput(IntPtr h, int address, float value);

        // 获取Double型输出值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDoubleOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getDoubleOutput(IntPtr h, int address);

        // 设置Double型输出值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setDoubleOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setDoubleOutput(IntPtr h, int address, double value);

        // 获取Int16型寄存器值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getInt16Register", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern Int16 getInt16Register(IntPtr h, int address);

        // 设置Int16型寄存器值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setInt16Register", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setInt16Register(IntPtr h, int address, Int16 value);

        // 检查变量是否更新函数封装
        [DllImport(service_interface_dll, EntryPoint = "variableUpdated", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool variableUpdated(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, UInt64 since);

        // 检查是否存在指定名称变量函数封装
        [DllImport(service_interface_dll, EntryPoint = "hasNamedVariable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool hasNamedVariable(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key);

        // 获取指定名称变量类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "getNamedVariableType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getNamedVariableType(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPStr)] string result);

        // 获取指定名称布尔型变量值（带默认值）函数封装
        [DllImport(service_interface_dll, EntryPoint = "getBool", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool getBool(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, bool default_value);

        // 设置指定名称布尔型变量值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setBool", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setBool(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, bool value);

        // 获取指定名称字符向量变量值（带默认值）函数封装
        [DllImport(service_interface_dll, EntryPoint = "getVecChar", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getVecChar(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPStr)] string default_value, [MarshalAs(UnmanagedType.LPStr)] string result);

        // 设置指定名称字符向量变量值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setVecChar", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setVecChar(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPStr)] string value);

        // 获取指定名称Int32型变量值（带默认值）函数封装
        [DllImport(service_interface_dll, EntryPoint = "getInt32", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getInt32(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, int default_value);

        // 设置指定名称Int32型变量值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setInt32", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setInt32(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, int value);


        // 获取指定名称Int32向量型变量值（带默认值）函数封装
        [DllImport(service_interface_dll, EntryPoint = "getVecInt32", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getVecInt32(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPArray)] Int32[] default_value, [MarshalAs(UnmanagedType.LPArray)] int[] result);

        // 设置指定名称Int32向量型变量值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setVecInt32", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setVecInt32(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPArray)] Int32[] value);

        // 获取指定名称Float型变量值（带默认值）函数封装
        [DllImport(service_interface_dll, EntryPoint = "getFloat", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern float getFloat(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, float default_value);

        // 设置指定名称Float型变量值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setFloat", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setFloat(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, float value);

        // 获取指定名称Float向量型变量值（带默认值）函数封装
        [DllImport(service_interface_dll, EntryPoint = "getVecFloat", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getVecFloat(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPArray)] float[] default_value, [MarshalAs(UnmanagedType.LPArray)] float[] result);

        // 设置指定名称Float向量型变量值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setVecFloat", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setVecFloat(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPArray)] float[] value);

        // 获取指定名称Double型变量值（带默认值）函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDouble", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getDouble(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, double default_value);

        // 设置指定名称Double型变量值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setDouble", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setDouble(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, double value);

        // 获取指定名称Double向量型变量值（带默认值）函数封装
        [DllImport(service_interface_dll, EntryPoint = "getVecDouble", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getVecDouble(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPArray)] double[] default_value, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 设置指定名称Double向量型变量值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setVecDouble", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setVecDouble(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPArray)] double[] value);

        // 获取指定名称字符串变量值（带默认值）函数封装
        [DllImport(service_interface_dll, EntryPoint = "getString", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getString(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPStr)] string default_value, [Out] StringBuilder result);

        // 设置指定名称字符串变量值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setString", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setString(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, [MarshalAs(UnmanagedType.LPStr)] string value);

        // 清除指定名称变量函数封装
        [DllImport(service_interface_dll, EntryPoint = "clearNamedVariable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int clearNamedVariable(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key);

        // 设置看门狗相关参数函数封装
        [DllImport(service_interface_dll, EntryPoint = "setWatchDog", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setWatchDog(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, double timeout, int action);

        // 获取看门狗动作函数封装
        [DllImport(service_interface_dll, EntryPoint = "getWatchDogAction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getWatchDogAction(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key);

        // 获取看门狗超时时间函数封装
        [DllImport(service_interface_dll, EntryPoint = "getWatchDogTimeout", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getWatchDogTimeout(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key);

        // 添加Modbus信号函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusAddSignal", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusAddSignal(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string device_info, int slave_number, int signal_address, int signal_type, [MarshalAs(UnmanagedType.LPStr)] string signal_name, bool sequential_mode);

        // 删除Modbus信号函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusDeleteSignal", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusDeleteSignal(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string signal_name);

        // 删除所有Modbus信号函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusDeleteAllSignals", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusDeleteAllSignals(IntPtr h);

        // 获取Modbus信号状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusGetSignalStatus", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusGetSignalStatus(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string signal_name);

        // 获取Modbus信号名称函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusGetSignalNames", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusGetSignalNames(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] string[] result);

        // 获取Modbus信号类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusGetSignalTypes", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusGetSignalTypes(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] int[] result);

        // 获取Modbus信号值函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusGetSignalValues", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusGetSignalValues(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] int[] result);

        // 获取Modbus信号错误信息函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusGetSignalErrors", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusGetSignalErrors(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] int[] result);

        // 发送Modbus自定义命令函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusSendCustomCommand", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusSendCustomCommand(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string IP, int slave_number, int function_code, [MarshalAs(UnmanagedType.LPArray)] byte[] data);

        // 设置Modbus数字输入动作函数封装
        //  [DllImport(service_interface_dll, EntryPoint = "modbusSetDigitalInputAction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        //  public static extern int modbusSetDigitalInputAction(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string robot_name, [MarshalAs(UnmanagedType.LPStr)] string signal_name, StandardInputAction_C action);

        // 设置Modbus输出运行状态函数封装
        //  [DllImport(service_interface_dll, EntryPoint = "modbusSetOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        // public static extern int modbusSetOutputRunstate(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string robot_name, [MarshalAs(UnmanagedType.LPStr)] string signal_name, StandardOutputRunState_C runstate);

        // 设置Modbus输出信号值函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusSetOutputSignal", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusSetOutputSignal(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string signal_name, ushort value);

        // 设置Modbus输出信号脉冲函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusSetOutputSignalPulse", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusSetOutputSignalPulse(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string signal_name, ushort value, double duration);

        // 设置Modbus信号更新频率函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusSetSignalUpdateFrequency", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusSetSignalUpdateFrequency(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string signal_name, int update_frequency);

        // 获取Modbus信号索引函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusGetSignalIndex", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusGetSignalIndex(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string signal_name);

        // 获取Modbus信号错误函数封装
        [DllImport(service_interface_dll, EntryPoint = "modbusGetSignalError", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int modbusGetSignalError(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string signal_name);

        // 获取Modbus设备状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getModbusDeviceStatus", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getModbusDeviceStatus(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string device_name);
    }



    public class cSharpBinging_RuntimeMachine
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 创建新任务函数封装
        [DllImport(service_interface_dll, EntryPoint = "newTask", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int newTask(IntPtr h, bool daemon);

        // 删除任务函数封装
        [DllImport(service_interface_dll, EntryPoint = "deleteTask", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int deleteTask(IntPtr h, int tid);

        // 分离任务函数封装
        [DllImport(service_interface_dll, EntryPoint = "detachTask", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int detachTask(IntPtr h, int tid);

        // 检查任务是否存活函数封装
        [DllImport(service_interface_dll, EntryPoint = "isTaskAlive", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isTaskAlive(IntPtr h, int tid);

        // 空操作函数封装
        [DllImport(service_interface_dll, EntryPoint = "nop", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int nop(IntPtr h);

        // 切换任务函数封装
        [DllImport(service_interface_dll, EntryPoint = "switchTask", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int switchTask(IntPtr h, int tid);

        // 设置任务标签函数封装
        [DllImport(service_interface_dll, EntryPoint = "setLabel", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setLabel(IntPtr h, int tid, [MarshalAs(UnmanagedType.LPStr)] string lineno);

        // 设置计划上下文函数封装
        [DllImport(service_interface_dll, EntryPoint = "setPlanContext", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setPlanContext(IntPtr h, int tid, int lineno, [MarshalAs(UnmanagedType.LPStr)] string comment);

        // 跳转到指定行函数封装
        [DllImport(service_interface_dll, EntryPoint = "gotoLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int gotoLine(IntPtr h, int lineno);

        // 获取高级计划上下文函数封装
        [DllImport(service_interface_dll, EntryPoint = "getAdvancePlanContext", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getAdvancePlanContext(IntPtr h, int tid, ref cSharpBinging_TypeDef.PlanContextResult result);

        // 获取高级指针函数封装
        [DllImport(service_interface_dll, EntryPoint = "getAdvancePtr", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getAdvancePtr(IntPtr h, int tid);

        // 获取主指针函数封装
        [DllImport(service_interface_dll, EntryPoint = "getMainPtr", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getMainPtr(IntPtr h, int tid);

        // 获取解释器指针函数封装
        [DllImport(service_interface_dll, EntryPoint = "getInterpPtr", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getInterpPtr(IntPtr h, int tid);

        // 获取计划上下文函数封装
        [DllImport(service_interface_dll, EntryPoint = "getPlanContext", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getPlanContext(IntPtr h, int tid, ref cSharpBinging_TypeDef.PlanContextResult result);

        // 获取执行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getExecutionStatus", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getExecutionStatus(IntPtr h, ref cSharpBinging_TypeDef.ExecutionStatusResult result);

        // 获取执行状态1函数封装
        [DllImport(service_interface_dll, EntryPoint = "getExecutionStatus1", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getExecutionStatus1(IntPtr h, ref cSharpBinging_TypeDef.ExecutionStatus1Result result);

        // 加载程序函数封装
        [DllImport(service_interface_dll, EntryPoint = "loadProgram", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int loadProgram(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string program);

        // 运行程序函数封装
        [DllImport(service_interface_dll, EntryPoint = "runProgram", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int runProgram(IntPtr h);

        // 启动函数封装
        [DllImport(service_interface_dll, EntryPoint = "start", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int start(IntPtr h);

        // 停止函数封装
        [DllImport(service_interface_dll, EntryPoint = "stop", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int stop(IntPtr h);

        // 终止函数封装
        [DllImport(service_interface_dll, EntryPoint = "abort1", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int abort(IntPtr h);

        // 暂停函数封装
        [DllImport(service_interface_dll, EntryPoint = "pause", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pause(IntPtr h);

        // 单步执行函数封装
        [DllImport(service_interface_dll, EntryPoint = "step", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int step(IntPtr h);

        // 设置恢复等待状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "setResumeWait", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setResumeWait(IntPtr h, bool wait);

        // 恢复执行函数封装
        [DllImport(service_interface_dll, EntryPoint = "resume", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int resume(IntPtr h);

        // 获取运行时状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStatus", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.RuntimeState getStatus(IntPtr h);

        // 获取运行时状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getRuntimeState", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.RuntimeState getRuntimeState(IntPtr h);

        // 设置断点函数封装
        [DllImport(service_interface_dll, EntryPoint = "setBreakPoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setBreakPoint(IntPtr h, int lineno);

        // 移除断点函数封装
        [DllImport(service_interface_dll, EntryPoint = "removeBreakPoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int removeBreakPoint(IntPtr h, int lineno);

        // 清除所有断点函数封装
        [DllImport(service_interface_dll, EntryPoint = "clearBreakPoints", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int clearBreakPoints(IntPtr h);

        // 定时器启动函数封装
        [DllImport(service_interface_dll, EntryPoint = "timerStart", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int timerStart(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 定时器停止函数封装
        [DllImport(service_interface_dll, EntryPoint = "timerStop", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int timerStop(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 定时器重置函数封装
        [DllImport(service_interface_dll, EntryPoint = "timerReset", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int timerReset(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 定时器删除函数封装
        [DllImport(service_interface_dll, EntryPoint = "timerDelete", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int timerDelete(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 获取定时器值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTimer", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getTimer(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 触发开始函数封装
        [DllImport(service_interface_dll, EntryPoint = "triggBegin", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int triggBegin(IntPtr h, double distance, double delay);

        // 触发结束函数封装
        [DllImport(service_interface_dll, EntryPoint = "triggEnd", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int triggEnd(IntPtr h);

        // 触发中断函数封装
        [DllImport(service_interface_dll, EntryPoint = "triggInterrupt", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int triggInterrupt(IntPtr h, double distance, double delay);

        // 获取触发中断信息函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTriggInterrupts", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTriggInterrupts(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] int[] result);
    }

    public class cSharpBinging_Serial
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 打开串口函数封装
        [DllImport(service_interface_dll, EntryPoint = "serialOpen", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int serialOpen(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string device, int baud, float stop_bits, int even, [MarshalAs(UnmanagedType.LPStr)] string serial_name);

        // 关闭串口函数封装
        [DllImport(service_interface_dll, EntryPoint = "serialClose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int serialClose(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string serial_name);

        // 读取单个字节函数封装
        [DllImport(service_interface_dll, EntryPoint = "serialReadByte", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int serialReadByte(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string variable, [MarshalAs(UnmanagedType.LPStr)] string serial_name);

        // 读取字节列表函数封装
        [DllImport(service_interface_dll, EntryPoint = "serialReadByteList", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int serialReadByteList(IntPtr h, int number, [MarshalAs(UnmanagedType.LPStr)] string variable, [MarshalAs(UnmanagedType.LPStr)] string serial_name);

        // 读取字符串函数封装
        [DllImport(service_interface_dll, EntryPoint = "serialReadString", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int serialReadString(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string variable, [MarshalAs(UnmanagedType.LPStr)] string serial_name, [MarshalAs(UnmanagedType.LPStr)] string prefix, [MarshalAs(UnmanagedType.LPStr)] string suffix, bool interpret_escape);

        // 发送单个字节函数封装
        [DllImport(service_interface_dll, EntryPoint = "serialSendByte", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int serialSendByte(IntPtr h, char value, [MarshalAs(UnmanagedType.LPStr)] string serial_name);

        // 发送整数函数封装
        [DllImport(service_interface_dll, EntryPoint = "serialSendInt", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int serialSendInt(IntPtr h, int value, [MarshalAs(UnmanagedType.LPStr)] string serial_name);

        // 发送一行数据函数封装
        [DllImport(service_interface_dll, EntryPoint = "serialSendLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int serialSendLine(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string str, [MarshalAs(UnmanagedType.LPStr)] string serial_name);

        // 发送字符串函数封装
        [DllImport(service_interface_dll, EntryPoint = "serialSendString", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int serialSendString(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string str, [MarshalAs(UnmanagedType.LPStr)] string serial_name);

        // 发送全部字符串函数封装
        [DllImport(service_interface_dll, EntryPoint = "serialSendAllString", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int serialSendAllString(IntPtr h, bool is_check, [MarshalAs(UnmanagedType.LPStr)] string str, [MarshalAs(UnmanagedType.LPStr)] string serial_name);
    }


    public class cSharpBinging_Socket
    {

        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 打开Socket函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketOpen", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketOpen(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string address, int port, [MarshalAs(UnmanagedType.LPStr)] string socket_name);

        // 关闭Socket函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketClose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketClose(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string socket_name);

        // 从Socket读取ASCII格式浮点数函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketReadAsciiFloat", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketReadAsciiFloat(IntPtr h, int number, [MarshalAs(UnmanagedType.LPStr)] string variable, [MarshalAs(UnmanagedType.LPStr)] string socket_name);

        // 从Socket读取二进制整数函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketReadBinaryInteger", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketReadBinaryInteger(IntPtr h, int number, [MarshalAs(UnmanagedType.LPStr)] string variable, [MarshalAs(UnmanagedType.LPStr)] string socket_name);

        // 从Socket读取字节列表函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketReadByteList", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketReadByteList(IntPtr h, int number, [MarshalAs(UnmanagedType.LPStr)] string variable, [MarshalAs(UnmanagedType.LPStr)] string socket_name);

        // 从Socket读取字符串函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketReadString", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketReadString(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string variable, [MarshalAs(UnmanagedType.LPStr)] string socket_name, [MarshalAs(UnmanagedType.LPStr)] string prefix, [MarshalAs(UnmanagedType.LPStr)] string suffix, bool interpret_escape);

        // 从Socket读取全部字符串函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketReadAllString", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketReadAllString(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string variable, [MarshalAs(UnmanagedType.LPStr)] string socket_name);

        // 向Socket发送单个字节函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketSendByte", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketSendByte(IntPtr h, char value, [MarshalAs(UnmanagedType.LPStr)] string socket_name);

        // 向Socket发送整数函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketSendInt", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketSendInt(IntPtr h, int value, [MarshalAs(UnmanagedType.LPStr)] string socket_name);

        // 向Socket发送一行数据函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketSendLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketSendLine(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string str, [MarshalAs(UnmanagedType.LPStr)] string socket_name);

        // 向Socket发送字符串函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketSendString", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketSendString(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string str, [MarshalAs(UnmanagedType.LPStr)] string socket_name);

        // 向Socket发送全部字符串函数封装
        [DllImport(service_interface_dll, EntryPoint = "socketSendAllString", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int socketSendAllString(IntPtr h, bool is_check, [MarshalAs(UnmanagedType.LPStr)] string str, [MarshalAs(UnmanagedType.LPStr)] string socket_name);
    }
    public class cSharpBinging_SyncMove
    {

        const string service_interface_dll = GlobalConstants.service_interface_dll;


        // 执行同步操作函数封装
        [DllImport(service_interface_dll, EntryPoint = "sync", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int sync(IntPtr h);

        // 开启同步移动函数封装
        [DllImport(service_interface_dll, EntryPoint = "syncMoveOn", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int syncMoveOn(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string syncident, [MarshalAs(UnmanagedType.LPArray)] string[] taskset);

        // 判断同步移动段是否有效函数封装
        [DllImport(service_interface_dll, EntryPoint = "syncMoveSegment", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool syncMoveSegment(IntPtr h, int id);

        // 关闭同步移动函数封装
        [DllImport(service_interface_dll, EntryPoint = "syncMoveOff", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int syncMoveOff(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string syncident);

        // 撤销同步移动操作函数封装
        [DllImport(service_interface_dll, EntryPoint = "syncMoveUndo", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int syncMoveUndo(IntPtr h);

        // 等待同步任务函数封装
        [DllImport(service_interface_dll, EntryPoint = "waitSyncTasks", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int waitSyncTasks(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string syncident, [MarshalAs(UnmanagedType.LPArray)] string[] taskset);

        // 判断同步移动是否开启函数封装
        [DllImport(service_interface_dll, EntryPoint = "isSyncMoveOn", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isSyncMoveOn(IntPtr h);

        // 暂停同步移动函数封装
        [DllImport(service_interface_dll, EntryPoint = "syncMoveSuspend", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int syncMoveSuspend(IntPtr h);

        // 恢复同步移动函数封装
        [DllImport(service_interface_dll, EntryPoint = "syncMoveResume", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int syncMoveResume(IntPtr h);
    }

    public class cSharpBinging_SystemInfo
    {

        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 获取控制软件版本代码函数封装
        [DllImport(service_interface_dll, EntryPoint = "getControlSoftwareVersionCode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getControlSoftwareVersionCode(IntPtr h);

        // 获取控制软件完整版本信息函数封装
        [DllImport(service_interface_dll, EntryPoint = "getControlSoftwareFullVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getControlSoftwareFullVersion(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string result);

        // 获取接口版本代码函数封装
        [DllImport(service_interface_dll, EntryPoint = "getInterfaceVersionCode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getInterfaceVersionCode(IntPtr h);

        // 获取控制软件构建日期函数封装
        [DllImport(service_interface_dll, EntryPoint = "getControlSoftwareBuildDate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getControlSoftwareBuildDate(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string result);

        // 获取控制软件版本哈希值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getControlSoftwareVersionHash", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getControlSoftwareVersionHash(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string result);

        // 获取控制系统时间函数封装
        [DllImport(service_interface_dll, EntryPoint = "getControlSystemTime", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern ulong getControlSystemTime(IntPtr h);
    }

    public class cSharpBinging_Trace
    {

        const string service_interface_dll = GlobalConstants.service_interface_dll;
        public enum TraceLevel_C
        {
            // 根据实际C语言中TraceLevel_C的枚举值添加对应项
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct RobotMsg_C
        {
            // 根据实际RobotMsg_C结构体的成员添加对应字段
        }

        // 触发告警函数封装
        [DllImport(service_interface_dll, EntryPoint = "alarm", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int alarm(IntPtr h, TraceLevel_C level, int code, [MarshalAs(UnmanagedType.LPArray)] string[] args);

        // 弹出提示框函数封装
        [DllImport(service_interface_dll, EntryPoint = "popup", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int popup(IntPtr h, TraceLevel_C level, [MarshalAs(UnmanagedType.LPStr)] string title, [MarshalAs(UnmanagedType.LPStr)] string msg, int mode);

        // 显示文本消息函数封装
        [DllImport(service_interface_dll, EntryPoint = "textmsg", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int textmsg(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string msg);

        // 发送通知函数封装
        [DllImport(service_interface_dll, EntryPoint = "notify", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int notify(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string msg);

        // 查看消息函数封装
        [DllImport(service_interface_dll, EntryPoint = "peek", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int peek(IntPtr h, ulong num, ulong last_time, [MarshalAs(UnmanagedType.LPStruct)] RobotMsg_C result);
    }



    public class cSharpBinging_TypeDef
    {

        // 定义常量
        public const int CARTESIAN_DOF = 6;
        public const int SAFETY_PARAM_SELECT_NUM = 2; // normal + reduced
        public const int SAFETY_PLANES_NUM = 8; // 安全平面的数量
        public const int SAFETY_CUBIC_NUM = 10; // 安全立方体的数量
        public const int TOOL_CONFIGURATION_NUM = 3; // 工具配置数量
        public const int MAX_DOF = 7; // 工具配置数量（或最大自由度）


        // 定义向量类型
        public struct Vector3d
        {
            public double X, Y, Z;

            public Vector3d(double x, double y, double z)
            {
                X = x;
                Y = y;
                Z = z;
            }
        }

        public struct Vector4d
        {
            public double W, X, Y, Z;

            public Vector4d(double w, double x, double y, double z)
            {
                W = w;
                X = x;
                Y = y;
                Z = z;
            }
        }

        public struct Vector3f
        {
            public float X, Y, Z;

            public Vector3f(float x, float y, float z)
            {
                X = x;
                Y = y;
                Z = z;
            }
        }

        public struct Vector4f
        {
            public float W, X, Y, Z;

            public Vector4f(float w, float x, float y, float z)
            {
                W = w;
                X = x;
                Y = y;
                Z = z;
            }
        }

        public struct Vector6f
        {
            public float X, Y, Z, Rx, Ry, Rz;

            public Vector6f(float x, float y, float z, float rx, float ry, float rz)
            {
                X = x;
                Y = y;
                Z = z;
                Rx = rx;
                Ry = ry;
                Rz = rz;
            }
        }


        // 定义机器人安全参数范围结构体
        [StructLayout(LayoutKind.Sequential)]
        public struct RobotSafetyParameterRange_C
        {
            public uint crc32;

            // 定义参数结构体
            [StructLayout(LayoutKind.Sequential)]
            public struct SafetyParameter
            {
                public float power;
                public float momentum;
                public float stop_time;
                public float stop_distance;
                public float reduced_entry_time;
                public float reduced_entry_distance;
                public float tcp_speed;
                public float elbow_speed;
                public float tcp_force;
                public float elbow_force;

                [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_DOF)]
                public float[] qmin;

                [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_DOF)]
                public float[] qmax;

                [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_DOF)]
                public float[] qdmax;

                [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_DOF)]
                public float[] joint_torque;

                public Vector3f tool_orientation;
                public float tool_deviation;

                [MarshalAs(UnmanagedType.ByValArray, SizeConst = SAFETY_PLANES_NUM)]
                public Vector4f[] planes;

                [MarshalAs(UnmanagedType.ByValArray, SizeConst = SAFETY_PLANES_NUM)]
                public int[] restrict_elbow;
            }

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = SAFETY_PARAM_SELECT_NUM)]
            public SafetyParameter[] params_; // 注意：在C#中，'params' 是关键字，因此使用 'params_' 作为字段名

            // 定义触发平面结构体
            [StructLayout(LayoutKind.Sequential)]
            public struct TriggerPlane
            {
                public Vector4f plane;
                public int restrict_elbow;
            }

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = SAFETY_PLANES_NUM)]
            public TriggerPlane[] trigger_planes;

            // 定义立方块结构体
            [StructLayout(LayoutKind.Sequential)]
            public struct Cubic
            {
                public Vector6f orig;
                public Vector3f size;
                public int restrict_elbow;
            }

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = SAFETY_CUBIC_NUM)]
            public Cubic[] cubic;

            // 工具数组
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = TOOL_CONFIGURATION_NUM)]
            public Vector4f[] tools;

            public float tool_inclination;
            public float tool_azimuth;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_DOF)]
            public float[] safety_home;

            // 可配置IO的输入输出安全功能配置
            public uint safety_input_emergency_stop;
            public uint safety_input_safeguard_stop;
            public uint safety_input_safeguard_reset;
            public uint safety_input_auto_safeguard_stop;
            public uint safety_input_auto_safeguard_reset;
            public uint safety_input_three_position_switch;
            public uint safety_input_operational_mode;
            public uint safety_input_reduced_mode;
            public uint safety_input_handguide;

            public uint safety_output_emergency_stop;
            public uint safety_output_not_emergency_stop;
            public uint safety_output_robot_moving;
            public uint safety_output_robot_steady;
            public uint safety_output_reduced_mode;
            public uint safety_output_not_reduced_mode;
            public uint safety_output_safe_home;
            public uint safety_output_robot_not_stopping;
            public uint safety_output_safetyguard_stop;

            public int tp_3pe_for_handguide;
            public int allow_manual_high_speed;
        }

        public enum AuboErrorCodes
        {
            AUBO_OK = 0,                                  // Success
            AUBO_BAD_STATE = 1,                           // State error
            AUBO_QUEUE_FULL = 2,                          // Planning queue full
            AUBO_BUSY = 3,                                // The previous command is executing
            AUBO_TIMEOUT = 4,                             // Timeout
            AUBO_INVL_ARGUMENT = 5,                       // Invalid parameters
            AUBO_NOT_IMPLEMENTED = 6,                     // Interface not implemented (fixed typo)
            AUBO_NO_ACCESS = 7,                           // Cannot access
            AUBO_CONN_REFUSED = 8,                        // Connection refused
            AUBO_CONN_RESET = 9,                          // Connection is reset
            AUBO_INPROGRESS = 10,                         // Execution in progress
            AUBO_EIO = 11,                                // Input/Output error
            AUBO_NOBUFFS = 12,                            // No buffers (assuming an appropriate description)
            AUBO_REQUEST_IGNORE = 13,                     // Request was ignored
            AUBO_ALGORITHM_PLAN_FAILED = 14,              // Motion planning algorithm error
            AUBO_VERSION_INCOMPAT = 15,                   // Interface version unmatch
            AUBO_DIMENSION_ERR = 16,                      // Input parameter dimension is incorrect
            AUBO_SINGULAR_ERR = 17,                       // Input configuration may be singular
            AUBO_POS_BOUND_ERR = 18,                      // Input position boundary exceeds the limit range
            AUBO_INIT_POS_ERR = 19,                       // Initial position input is unreasonable
            AUBO_ELP_SETTING_ERR = 20,                    // Envelope body setting error
            AUBO_TRAJ_GEN_FAIL = 21,                      // Trajectory generation failed
            AUBO_TRAJ_SELF_COLLISION = 22,                // Trajectory self collision
            AUBO_IK_NO_CONVERGE = 23,                     // Inverse kinematics computation did not converge; computation failed
            AUBO_IK_OUT_OF_RANGE = 24,                    // Inverse kinematics result out of robot range
            AUBO_IK_CONFIG_DISMATCH = 25,                 // Inverse kinematics input configuration contains errors
            AUBO_IK_JACOBIAN_FAILED = 26,                 // The calculation of the inverse Jacobian matrix failed
            AUBO_IK_NO_SOLU = 27,                         // The target point has solutions, but it has exceeded the joint limit conditions
            AUBO_IK_UNKNOWN_ERROR = 28,                   // Inverse kinematics unknown error (fixed typo)
            AUBO_ERR_UNKNOWN = 99999                      // Unknown error occurred.
        }


        public enum RuntimeState
        {
            [Description("正在运行中")]
            Running = 0,

            [Description("倒退")]
            Retracting = 1,

            [Description("暂停中")]
            Pausing = 2,

            [Description("暂停状态")]
            Paused = 3,

            [Description("单步执行中")]
            Stepping = 4,

            [Description("受控停止中(保持原有轨迹)")]
            Stopping = 5,

            [Description("已停止")]
            Stopped = 6,

            [Description("停止(最大速度关节运动停机)")]
            Aborting = 7
        }

        public enum RobotModeType
        {
            [Description("提供给示教器使用的, 如果aubo_control进程崩溃则会显示为NoController")]
            NoController = -1,

            [Description("没有连接到机械臂本体(控制器与接口板断开连接或是 EtherCAT 等总线断开)")]
            Disconnected = 0,

            [Description("正在进行安全配置, 断电状态下进行")]
            ConfirmSafety = 1,

            [Description("机械臂本体正在上电初始化")]
            Booting = 2,

            [Description("机械臂本体处于断电状态")]
            PowerOff = 3,

            [Description("机械臂本体上电成功, 刹车暂未松开(抱死), 关节初始状态未获取")]
            PowerOn = 4,

            [Description("机械臂上电成功, 刹车暂未松开(抱死), 电机不通电, 关节初始状态获取完成")]
            Idle = 5,

            [Description("机械臂上电成功, 刹车正在松开")]
            BrakeReleasing = 6,

            [Description("反向驱动：刹车松开, 电机不通电")]
            BackDrive = 7,

            [Description("机械臂刹车松开, 运行模式, 控制权由硬件移交给软件")]
            Running = 8,

            [Description("维护模式: 包括固件升级、参数写入等")]
            Maintaince = 9,

            [Description("")]
            Error = 10,

            [Description("机械臂本体处于断电过程中")]
            PowerOffing = 11
        }

        public enum SafetyModeType
        {
            [Description("安全状态待定")]
            Undefined = 0,

            [Description("正常运行模式")]
            Normal = 1,

            [Description("缩减运行模式")]
            ReducedMode = 2,

            [Description("启动时如果在安全限制之外, 机器人将进入recovery模式")]
            Recovery = 3,

            [Description("超出安全限制（根据安全配置, 例如速度超限等）")]
            Violation = 4,

            [Description("软件触发的停机（保持轨迹, 不抱闸, 不断电）")]
            ProtectiveStop = 5,

            [Description("IO触发的防护停机（不保持轨迹, 抱闸, 不断电）")]
            SafeguardStop = 6,

            [Description("系统急停：急停信号由外部输入(可配置输入), 不对外输出急停信号")]
            SystemEmergencyStop = 7,

            [Description("机器人急停：控制柜急停输入或者示教器急停按键触发, 对外输出急停信号")]
            RobotEmergencyStop = 8,

            [Description("机械臂硬件故障或者系统故障")]
            Fault = 9
        }

        public enum OperationalModeType
        {
            [Description("禁用模式: 不使用Operational Mode")]
            Disabled = 0,

            [Description("自动模式: 机器人正常工作模式, 运行速度不会被限制")]
            Automatic = 1,

            [Description("手动模式: 机器人编程示教模式(T1), 机器人运行速度将会被限制或者机器人程序校验模式(T2)")]
            Manual = 2
        }

        public enum RobotControlModeType
        {
            [Description("未知的控制模式")]
            Unknown = 0,

            [Description("位置控制  movej")]
            Position = 1,

            [Description("速度控制  speedj/speedl")]
            Speed = 2,

            [Description("位置控制  servoj")]
            Servo = 3,

            [Description("拖动示教  freedrive_mode")]
            Freedrive = 4,

            [Description("末端力控  force_mode")]
            Force = 5,

            [Description("关节力矩控制")]
            Torque = 6,

            [Description("碰撞模式")]
            Collision = 7
        }

        public enum JointServoModeType
        {
            [Description("未知")]
            Unknown = -1,

            [Description("开环模式")]
            Open = 0,

            [Description("电流伺服模式")]
            Current = 1,

            [Description("速度伺服模式")]
            Velocity = 2,

            [Description("位置伺服模式")]
            Position = 3,

            [Description("力矩伺服模式")]
            Torque = 4
        }

        public enum JointStateType
        {
            [Description("节点未连接到接口板或者已经断电")]
            Poweroff = 0,

            [Description("节点空闲")]
            Idle = 2,

            [Description("节点错误, 节点停止伺服运动, 刹车抱死")]
            Fault = 3,

            [Description("节点伺服")]
            Running = 4,

            [Description("节点bootloader状态, 暂停一切通讯")]
            Bootload = 5
        }

        public enum StandardInputAction
        {
            [Description("无触发")]
            Default = 0,

            [Description("拖动示教，高电平触发")]
            Handguide = 1,

            [Description("运动到工程初始位姿，高电平触发")]
            GoHome = 2,

            [Description("开始工程，上升沿触发")]
            StartProgram = 3,

            [Description("停止工程，上升沿触发")]
            StopProgram = 4,

            [Description("暂停工程，上升沿触发")]
            PauseProgram = 5,

            [Description("消除弹窗，上升沿触发")]
            PopupDismiss = 6,

            [Description("机器人上电/松刹车，上升沿触发")]
            PowerOn = 7,

            [Description("机器人抱死刹车/断电，上升沿触发")]
            PowerOff = 8,

            [Description("恢复工程，上升沿触发")]
            ResumeProgram = 9,

            [Description("机器人减速触发1，高电平触发")]
            SlowDown1 = 10,

            [Description("机器人减速触发2，高电平触发")]
            SlowDown2 = 11,

            [Description("安全停止，高电平触发")]
            SafeStop = 12,

            [Description("信号，高电平有效")]
            RunningGuard = 13,

            [Description("运动到工程初始位姿，高电平触发")]
            MoveToFirstPoint = 14,

            [Description("机器人减速触发1，低电平触发")]
            xSlowDown1 = 15,

            [Description("机器人减速触发2，低电平触发")]
            xSlowDown2 = 16
        }

        public enum StandardOutputRunState
        {
            [Description("标准输出状态未定义")]
            None = 0,
            [Description("低电平指示工程停止")]
            StopLow = 1,
            [Description("高电平指示机器人停止")]
            StopHigh = 2,
            [Description("指示工程正在运行")]
            RunningHigh = 3,
            [Description("指示工程已经暂停")]
            PausedHigh = 4,
            [Description("高电平指示机器人正在拖动")]
            AtHome = 5,
            [Description("高电平指示机器人正在拖动（注意：与AtHome重复描述，可能需要修正）")]
            Handguiding = 6,
            [Description("高电平指示机器人已经上电")]
            PowerOn = 7,
            [Description("高电平指示机器人急停按下")]
            RobotEmergencyStop = 8,
            [Description("高电平指示外部输入系统急停按下")]
            SystemEmergencyStop = 9,
            [Description("系统错误，包括故障、超限、急停、安全停止、防护停止")]
            SystemError = 10,
            [Description("无系统错误，包括普通模式、缩减模式和恢复模式")]
            NotSystemError = 11,
            [Description("机器人可操作，机器人上电且松刹车了")]
            RobotOperable = 12
        }

        public enum SafetyInputAction
        {
            [Description("安全输入未分配动作")]
            Unassigned = 0,
            [Description("安全输入触发急停")]
            EmergencyStop = 1,
            [Description("安全输入触发防护停止, 边沿触发")]
            SafeguardStop = 2,
            [Description("安全输入触发防护重置, 边沿触发")]
            SafeguardReset = 3,
            [Description("3档位使能开关")]
            ThreePositionSwitch = 4,
            [Description("切换自动模式和手动模式")]
            OperationalMode = 5,
            [Description("拖动示教")]
            HandGuide = 6,
            [Description("安全参数切换1(缩减模式)，序号越低优先级越高，三路输出都无效时，选用第0组安全参数")]
            ReducedMode = 7,
            [Description("自动模式下防护停机输入(需要配置三档位使能设备)")]
            AutomaticModeSafeguardStop = 8,
            [Description("自动模式下上升沿触发防护重置(需要配置三档位使能设备)")]
            AutomaticModeSafeguardReset = 9
        }

        public enum SafetyOutputRunState
        {
            [Description("安全输出未定义")]
            Unassigned = 0,
            [Description("输出高当有机器人急停输入或者急停按键被按下")]
            SystemEmergencyStop = 1,
            [Description("输出低当有机器人急停输入或者急停按键被按下")]
            NotSystemEmergencyStop = 2,
            [Description("输出高当有关节运动速度超过 0.1rad/s")]
            RobotMoving = 3,
            [Description("输出高当所有的关节运动速度不超过 0.1rad/s")]
            RobotNotMoving = 4,
            [Description("输出高当机器人处于缩减模式")]
            ReducedMode = 5,
            [Description("输出高当机器人不处于缩减模式")]
            NotReducedMode = 6,
            [Description("输出高当机器人已经处于安全Home位姿")]
            SafeHome = 7,
            [Description("输出低当机器人正在急停或者安全停止中")]
            RobotNotStopping = 8
        }

        public enum PayloadIdentifyMoveAxis
        {
            [Description("第2和6关节运动")]
            Joint_2_6 = 0,
            [Description("第3和6关节运动")]
            Joint_3_6 = 1,
            [Description("第4和6关节运动")]
            Joint_4_6 = 2,
            [Description("第4、5、6关节运动")]
            Joint_4_5_6 = 3
        }

        public enum EnvelopingShape
        {
            [Description("立方体")]
            Cube = 1,

            [Description("柱状体")]
            Column = 2,

            [Description("以STL文件的形式描述负载碰撞集合体")]
            Stl = 3
        }

        public enum TaskFrameType
        {
            [Description("")]
            NONE = 0,

            [Description("力控坐标系发生变换, 使得力控参考坐标系的y轴沿着机器人TCP指向力控所选特征的原点, x和z轴取决于所选特征的原始方向\n力控坐标系发生变换, 使得力控参考坐标系的y轴沿着机器人TCP指向力控所选特征的原点, x和z轴取决于所选特征的原始方向\n机器人TCP与所选特征的起点之间的距离至少为10mm\n优先选择X轴, 为所选特征的X轴在力控坐标系Y轴垂直平面上的投影, 如果所选特征的X轴与力控坐标系的Y轴平行, \n通过类似方法确定力控坐标系Z轴, Y-X或者Y-Z轴确定之后, 通过右手法则确定剩下的轴")]
            POINT_FORCE = 1,

            [Description("力控坐标系不发生变换 SIMPLE_FORC")]
            FRAME_FORCE = 2,

            [Description("力控坐标系发生变换, 使得力控参考坐标系的x轴为机器人TCP速度在所选特征x-y平面上的投影y轴将垂直于机械臂运动, 并在所选特征的x-y平面内")]
            MOTION_FORCE = 3,

            [Description("以工具末端坐标系作为力控参考坐标系")]
            TOOL_FORCE = 4
        }

        public enum TraceLevel
        {
            [Description("")]
            FATAL = 0,

            [Description("")]
            ERROR = 1,

            [Description("")]
            WARNING = 2,

            [Description("")]
            INFO = 3,

            [Description("")]
            DEBUG = 4
        }

        public enum AxisModeType
        {
            [Description("提供给示教器使用的, 如果aubo_control进程崩溃则会显示为NoController")]
            NoController = -1,

            [Description("未连接")]
            Disconnected = 0,

            [Description("断电")]
            PowerOff = 1,

            [Description("刹车松开中")]
            BrakeReleasing = 2,

            [Description("空闲")]
            Idle = 3,

            [Description("运行中")]
            Running = 4,

            [Description("错误状态")]
            Fault = 5
        }

        public enum SafeguedStopType
        {
            [Description("无安全停止")]
            None = 0,

            [Description("安全停止(IO输入)")]
            SafeguedStopIOInput = 1,

            [Description("安全停止(三态开关)")]
            SafeguedStop3PE = 2,

            [Description("安全停止(操作模式)")]
            SafeguedStopOperational = 3
        }


        public enum ForceControlState
        {
            [Description("力控状态：已停止")]
            ForceControlState_Stopped,

            [Description("力控状态：正在启动")]
            ForceControlState_Starting,

            // 注意：原枚举值 "Stropping" 可能是 "Stopping" 的拼写错误，这里按照原样保留，但建议检查是否为拼写错误
            [Description("力控状态：正在停止（注意：可能是'Stopping'的拼写错误）")]
            ForceControlState_Stropping,

            [Description("力控状态：正在运行")]
            ForceControlState_Running
        }

        public enum RefFrameType
        {
            [Description("无参考坐标系")]
            RefFrameType_None,

            [Description("工具坐标系")]
            RefFrameType_Tool,

            [Description("轨迹坐标系")]
            RefFrameType_Path,

            [Description("基坐标系")]
            RefFrameType_Base
        }

        // 圆周运动参数定义
        [StructLayout(LayoutKind.Sequential)]
        public struct CircleParameters
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] PoseVia;  // 圆周运动途中点的位姿

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] PoseTo;   // 圆周运动结束点的位姿

            public double Acceleration; // 加速度, 单位: m/s^2
            public double Speed;        // 速度，单位: m/s
            public double BlendRadius;  // 交融半径, 单位: m
            public double Duration;     // 运行时间，单位: s
            public double Helix;
            public double Spiral;
            public double Direction;
            public int LoopTimes; // 暂不支持
        }

        // 螺旋线参数定义
        [StructLayout(LayoutKind.Sequential)]
        public struct SpiralParameters
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] Frame; // 参考点，螺旋线的中心点和参考坐标系
            public int Plane;      // 参考平面选择 0-XY 1-YZ 2-ZX
            public double Angle;   // 转动的角度，如果为正数，机器人逆时针旋转
            public double Spiral;  // 正数外扩
            public double Helix;   // 正数上升
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct Enveloping
        {
            public EnvelopingShape Shape; // 包络体形状

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] EpArgs; // 包络体组合，根据shape的不同，数组元素含义不同

            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            private string stlPath; // STL文件路径（在C#中，我们通常使用string而不是char数组）

            // 提供STL路径的公共属性，带有getter和setter
            public string StlPath
            {
                get { return stlPath; }
                set { stlPath = value; }
            }
        }

        // 用于负载辨识的轨迹配置
        public class TrajConfig
        {
            public Enveloping[] Envelopings;           // 包络体组合（使用数组而不是指针）
            public int MoveAxis;                       // 运动的轴(ID), 下标从0开始（假设只有一个轴在运动）
                                                       // 如果需要多个轴，可以考虑使用数组或集合

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_DOF)] // 注意：MAX_DOF需要在某处定义
            public double[] InitJoint;          // 关节初始位置

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_DOF)]
            public double[] UpperJointBound;    // 运动轴上限

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_DOF)]
            public double[] LowerJointBound;    // 运动轴下限

            public double MaxVelocity;     // 关节运动的最大速度，默认值为3.0
            public double MaxAcceleration; // 关节运动的最大加速度，默认值为5.0
        }


        // 对应C语言中的DHParam_C结构体
        [StructLayout(LayoutKind.Sequential)]
        public struct DHParam
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] theta;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] beta;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] d;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] a;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] alpha;
        }

        // 对应C语言中的DHComp_C结构体
        [StructLayout(LayoutKind.Sequential)]
        public struct DHComp
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] theta_comp;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] beta_comp;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] d_comp;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] a_comp;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] alpha_comp;
        }

        // 对应C语言中的Payload_C结构体
        [StructLayout(LayoutKind.Sequential)]
        public struct Payload
        {
            public double mass;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public double[] cog;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public double[] aom;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] inertia;
        }

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
        public struct PlanContextResult
        {
            public int tid;
            public int lineno;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string comment;
        }

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
        public struct ExecutionStatusResult
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string name;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string status;
        }

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
        public struct ExecutionStatus1Result
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string name;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string status;
            public int retval;
        }

        // 对应C语言中的ForceSensorCalibResult_C结构体
        [StructLayout(LayoutKind.Sequential)]
        public struct ForceSensorCalibResult
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] force_offset;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public double[] com;
            public double mass;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] angle;
        }

        // 对应C语言中的DynamicsModel结构体
        [StructLayout(LayoutKind.Sequential)]
        public struct DynamicsModel
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] m;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] d;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] k;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct RobotMsg
        {
            public ulong timestamp;
            public TraceLevel level;
            public int code;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 100)]
            public string source;
            public IntPtr args;
            public int args_count;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct RobotMsgVector
        {
            public IntPtr data; // RobotMsg*
            public int size;    // RobotMsg个数
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct RtdeRecipe
        {
            public bool to_server;
            public int chanel;
            public double frequency;
            public int trigger;
            public IntPtr segments;
            public int segments_count;
        }

        public enum ErrorType
        {
            [Description("解析错误")]
            ParseError = -32700,
            [Description("无效请求")]
            InvalidRequest = -32600,
            [Description("方法未找到")]
            MethodNotFound = -32601,
            [Description("无效参数")]
            InvalidParams = -32602,
            [Description("内部错误")]
            InternalError = -32603,
            [Description("服务器错误")]
            ServerError, // 在C#中，如果不指定值，则默认为前一个值加1（但这里由于前面有显式值，所以默认是0，需要手动设置）
            [Description("无效")]
            Invalid = 1
        }

        public enum ExceptionCode
        {
            [Description("断开连接")]
            Disconnected = -1,
            [Description("未登录")]
            NotLogined = -2,
            [Description("无效套接字")]
            InvalidSocket = -3,
            [Description("请求繁忙")]
            RequestBusy = -4,
            [Description("发送失败")]
            SendFailed = -5,
            [Description("接收超时")]
            RecvTimeout = -6,
            [Description("接收错误")]
            RecvError = -7,
            [Description("解析错误")]
            ParseError = -8,
            [Description("无效请求")]
            InvalidRequest = -9,
            [Description("方法未找到")]
            MethodNotFound = -10,
            [Description("无效参数")]
            InvalidParams = -11,
            [Description("内部错误")]
            InternalError = -12,
            [Description("服务器错误")]
            ServerError = -13,
            [Description("无效")]
            Invalid = -14
        }

        // 定义一个帮助类来获取枚举值的描述
        public static class EnumHelper
        {
            public static string GetDescription(Enum value)
            {
                var field = value.GetType().GetField(value.ToString());
                var descriptionAttribute = Attribute.GetCustomAttribute(field, typeof(DescriptionAttribute)) as DescriptionAttribute;
                return descriptionAttribute == null ? value.ToString() : descriptionAttribute.Description;
            }
        }
    }

    public class cSharpBinging_RobotInterface
    {
        const string dllName = "aubo_sdkd.dll";

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern ROBOT_CONFIG_HANDLER robot_getRobotConfig(ROBOT_HANDLER robot);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern MOTION_CONTROL_HANDLER robot_getMotionControl(ROBOT_HANDLER robot);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern FORCE_CONTROL_HANDLER robot_getForceControl(ROBOT_HANDLER robot);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IO_CONTROL_HANDLER robot_getIoControl(ROBOT_HANDLER robot);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern SYNC_MOVE_HANDLER robot_getSyncMove(ROBOT_HANDLER robot);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern ROBOT_ALGORITHM_HANDLER robot_getRobotAlgorithm(ROBOT_HANDLER robot);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern ROBOT_MANAGE_HANDLER robot_getRobotManage(ROBOT_HANDLER robot);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern ROBOT_STATE_HANDLER robot_getRobotState(ROBOT_HANDLER robot);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern TRACE_HANDLER robot_getTrace(ROBOT_HANDLER robot);
    }
    // aubo_sdk/robot
    public class cSharpBinging_ForceControl
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 启用力控功能函数封装
        [DllImport(service_interface_dll, EntryPoint = "fcEnable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int fcEnable(IntPtr h);

        // 禁用力控功能函数封装
        [DllImport(service_interface_dll, EntryPoint = "fcDisable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int fcDisable(IntPtr h);

        // 判断力控是否启用函数封装
        [DllImport(service_interface_dll, EntryPoint = "isFcEnabled", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isFcEnabled(IntPtr h);

        // 设置目标力函数封装
        [DllImport(service_interface_dll, EntryPoint = "setTargetForce", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setTargetForce(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] feature, [MarshalAs(UnmanagedType.LPArray)] byte[] compliance, [MarshalAs(UnmanagedType.LPArray)] double[] wrench, [MarshalAs(UnmanagedType.LPArray)] double[] limits, cSharpBinging_TypeDef.TaskFrameType type);

        // 设置动力学模型函数封装
        [DllImport(service_interface_dll, EntryPoint = "setDynamicModel", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setDynamicModel(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] m, [MarshalAs(UnmanagedType.LPArray)] double[] d, [MarshalAs(UnmanagedType.LPArray)] double[] k);

        // 获取动力学模型函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDynamicModel", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.DynamicsModel getDynamicModel(IntPtr h);

        // 设置条件力函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCondForce", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCondForce(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] min, [MarshalAs(UnmanagedType.LPArray)] double[] max, bool outside, double timeout);

        // 设置条件方向函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCondOrient", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCondOrient(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] frame, double max_angle, double max_rot, bool outside, double timeout);

        // 设置条件平面函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCondPlane", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCondPlane(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] plane, double timeout);

        // 设置条件圆柱函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCondCylinder", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCondCylinder(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] axis, double radius, bool outside, double timeout);

        // 设置条件球体函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCondSphere", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCondSphere(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] center, double radius, bool outside, double timeout);

        // 设置条件TCP速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCondTcpSpeed", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCondTcpSpeed(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] min, [MarshalAs(UnmanagedType.LPArray)] double[] max, bool outside, double timeout);

        // 设置条件激活函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCondActive", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCondActive(IntPtr h);

        // 设置条件距离函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCondDistance", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCondDistance(IntPtr h, double distance, double timeout);

        // 设置高级条件函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCondAdvanced", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCondAdvanced(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string type, [MarshalAs(UnmanagedType.LPArray)] double[] args, double timeout);

        // 判断条件是否满足函数封装
        [DllImport(service_interface_dll, EntryPoint = "isCondFullfiled", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isCondFullfiled(IntPtr h);

        // 设置监督力函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSupvForce", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSupvForce(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] min, [MarshalAs(UnmanagedType.LPArray)] double[] max);

        // 设置监督方向函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSupvOrient", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSupvOrient(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] frame, double max_angle, double max_rot, bool outside);

        // 设置监督位置盒子函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSupvPosBox", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSupvPosBox(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] frame, [MarshalAs(UnmanagedType.LPArray)] double[] box);

        // 设置监督位置圆柱函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSupvPosCylinder", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSupvPosCylinder(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] frame, [MarshalAs(UnmanagedType.LPArray)] double[] cylinder);

        // 设置监督位置球体函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSupvPosSphere", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSupvPosSphere(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] frame, [MarshalAs(UnmanagedType.LPArray)] double[] sphere);

        // 设置监督重定向速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSupvReoriSpeed", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSupvReoriSpeed(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] speed_limit, bool outside, double timeout);

        // 设置监督TCP速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSupvTcpSpeed", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSupvTcpSpeed(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] speed_limit, bool outside, double timeout);

        // 设置低通滤波器函数封装
        [DllImport(service_interface_dll, EntryPoint = "setLpFilter", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setLpFilter(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] cutoff_freq);

        // 重置低通滤波器函数封装
        [DllImport(service_interface_dll, EntryPoint = "resetLpFilter", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int resetLpFilter(IntPtr h);

        // 速度变化调谐函数封装
        [DllImport(service_interface_dll, EntryPoint = "speedChangeTune", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int speedChangeTune(IntPtr h, int speed_levels, double speed_ratio_min);

        // 启用速度变化函数封装
        [DllImport(service_interface_dll, EntryPoint = "speedChangeEnable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int speedChangeEnable(IntPtr h, double ref_force);

        // 禁用速度变化函数封装
        [DllImport(service_interface_dll, EntryPoint = "speedChangeDisable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int speedChangeDisable(IntPtr h);

        // 设置阻尼函数封装
        [DllImport(service_interface_dll, EntryPoint = "setDamping", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setDamping(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] damping, double ramp_time);

        // 重置阻尼函数封装
        [DllImport(service_interface_dll, EntryPoint = "resetDamping", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int resetDamping(IntPtr h);

        // 启用软浮动函数封装
        [DllImport(service_interface_dll, EntryPoint = "softFloatEnable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int softFloatEnable(IntPtr h);

        // 禁用软浮动函数封装
        [DllImport(service_interface_dll, EntryPoint = "softFloatDisable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int softFloatDisable(IntPtr h);

        // 判断软浮动是否启用函数封装
        [DllImport(service_interface_dll, EntryPoint = "isSoftFloatEnabled", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isSoftFloatEnabled(IntPtr h);

        // 设置软浮动参数函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSoftFloatParams", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSoftFloatParams(IntPtr h, bool joint_space, [MarshalAs(UnmanagedType.LPArray)] byte[] select, [MarshalAs(UnmanagedType.LPArray)] double[] stiff_percent, [MarshalAs(UnmanagedType.LPArray)] double[] stiff_damp_ratio, [MarshalAs(UnmanagedType.LPArray)] double[] force_threshold, [MarshalAs(UnmanagedType.LPArray)] double[] force_limit);

        // 工具接触函数封装
        [DllImport(service_interface_dll, EntryPoint = "toolContact", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int toolContact(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] byte[] direction);
    }
    public class cSharpBinging_IoControl
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 获取标准数字输入数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardDigitalInputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getStandardDigitalInputNum(IntPtr h);

        // 获取工具数字输入数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolDigitalInputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolDigitalInputNum(IntPtr h);

        // 获取可配置数字输入数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getConfigurableDigitalInputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getConfigurableDigitalInputNum(IntPtr h);

        // 获取标准数字输出数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardDigitalOutputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getStandardDigitalOutputNum(IntPtr h);

        // 获取工具数字输出数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolDigitalOutputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolDigitalOutputNum(IntPtr h);

        // 设置工具IO输入函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolIoInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolIoInput(IntPtr h, int index, bool input);

        // 判断工具IO输入状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "isToolIoInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isToolIoInput(IntPtr h, int index);

        // 获取可配置数字输出数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getConfigurableDigitalOutputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getConfigurableDigitalOutputNum(IntPtr h);

        // 获取标准模拟输入数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardAnalogInputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getStandardAnalogInputNum(IntPtr h);

        // 获取工具模拟输入数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolAnalogInputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolAnalogInputNum(IntPtr h);

        // 获取标准模拟输出数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardAnalogOutputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getStandardAnalogOutputNum(IntPtr h);

        // 获取工具模拟输出数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolAnalogOutputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolAnalogOutputNum(IntPtr h);

        // 设置数字输入动作默认值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setDigitalInputActionDefault", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setDigitalInputActionDefault(IntPtr h);

        // 设置标准数字输入动作函数封装
        [DllImport(service_interface_dll, EntryPoint = "setStandardDigitalInputAction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setStandardDigitalInputAction(IntPtr h, int index, cSharpBinging_TypeDef.StandardInputAction action);

        // 设置工具数字输入动作函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolDigitalInputAction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolDigitalInputAction(IntPtr h, int index, cSharpBinging_TypeDef.StandardInputAction action);

        // 设置可配置数字输入动作函数封装
        [DllImport(service_interface_dll, EntryPoint = "setConfigurableDigitalInputAction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setConfigurableDigitalInputAction(IntPtr h, int index, cSharpBinging_TypeDef.StandardInputAction action);

        // 获取标准数字输入动作函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardDigitalInputAction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.StandardInputAction getStandardDigitalInputAction(IntPtr h, int index);

        // 获取工具数字输入动作函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolDigitalInputAction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.StandardInputAction getToolDigitalInputAction(IntPtr h, int index);

        // 获取可配置数字输入动作函数封装
        [DllImport(service_interface_dll, EntryPoint = "getConfigurableDigitalInputAction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.StandardInputAction getConfigurableDigitalInputAction(IntPtr h, int index);

        // 设置数字输出运行状态默认值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setDigitalOutputRunstateDefault", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setDigitalOutputRunstateDefault(IntPtr h);

        // 设置标准数字输出运行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "setStandardDigitalOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setStandardDigitalOutputRunstate(IntPtr h, int index, cSharpBinging_TypeDef.StandardOutputRunState runstate);

        // 设置工具数字输出运行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolDigitalOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolDigitalOutputRunstate(IntPtr h, int index, cSharpBinging_TypeDef.StandardOutputRunState runstate);

        // 设置可配置数字输出运行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "setConfigurableDigitalOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setConfigurableDigitalOutputRunstate(IntPtr h, int index, cSharpBinging_TypeDef.StandardOutputRunState runstate);

        // 获取标准数字输出运行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardDigitalOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.StandardOutputRunState getStandardDigitalOutputRunstate(IntPtr h, int index);

        // 获取工具数字输出运行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolDigitalOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.StandardOutputRunState getToolDigitalOutputRunstate(IntPtr h, int index);

        // 获取可配置数字输出运行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getConfigurableDigitalOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.StandardOutputRunState getConfigurableDigitalOutputRunstate(IntPtr h, int index);

        // 设置标准模拟输出运行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "setStandardAnalogOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setStandardAnalogOutputRunstate(IntPtr h, int index, cSharpBinging_TypeDef.StandardOutputRunState runstate);

        // 设置工具模拟输出运行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolAnalogOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolAnalogOutputRunstate(IntPtr h, int index, cSharpBinging_TypeDef.StandardOutputRunState runstate);

        // 获取标准模拟输出运行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardAnalogOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.StandardOutputRunState getStandardAnalogOutputRunstate(IntPtr h, int index);

        // 获取工具模拟输出运行状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolAnalogOutputRunstate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.StandardOutputRunState getToolAnalogOutputRunstate(IntPtr h, int index);

        // 设置标准模拟输入域函数封装
        [DllImport(service_interface_dll, EntryPoint = "setStandardAnalogInputDomain", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setStandardAnalogInputDomain(IntPtr h, int index, int domain);

        // 设置工具模拟输入域函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolAnalogInputDomain", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolAnalogInputDomain(IntPtr h, int index, int domain);

        // 获取标准模拟输入域函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardAnalogInputDomain", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getStandardAnalogInputDomain(IntPtr h, int index);

        // 获取工具模拟输入域函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolAnalogInputDomain", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolAnalogInputDomain(IntPtr h, int index);

        // 设置标准模拟输出域函数封装
        [DllImport(service_interface_dll, EntryPoint = "setStandardAnalogOutputDomain", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setStandardAnalogOutputDomain(IntPtr h, int index, int domain);

        // 设置工具模拟输出域函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolAnalogOutputDomain", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolAnalogOutputDomain(IntPtr h, int index, int domain);

        // 设置工具电压输出域函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolVoltageOutputDomain", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolVoltageOutputDomain(IntPtr h, int domain);

        // 获取工具电压输出域函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolVoltageOutputDomain", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolVoltageOutputDomain(IntPtr h);

        // 获取标准模拟输出域函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardAnalogOutputDomain", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getStandardAnalogOutputDomain(IntPtr h, int index);

        // 获取工具模拟输出域函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolAnalogOutputDomain", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolAnalogOutputDomain(IntPtr h, int index);

        // 设置标准数字输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "setStandardDigitalOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setStandardDigitalOutput(IntPtr h, int index, bool value);

        // 设置标准数字输出脉冲函数封装
        [DllImport(service_interface_dll, EntryPoint = "setStandardDigitalOutputPulse", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setStandardDigitalOutputPulse(IntPtr h, int index, bool value, double duration);

        // 设置工具数字输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolDigitalOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolDigitalOutput(IntPtr h, int index, bool value);

        // 设置工具数字输出脉冲函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolDigitalOutputPulse", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolDigitalOutputPulse(IntPtr h, int index, bool value, double duration);

        // 设置可配置数字输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "setConfigurableDigitalOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setConfigurableDigitalOutput(IntPtr h, int index, bool value);

        // 设置可配置数字输出脉冲函数封装
        [DllImport(service_interface_dll, EntryPoint = "setConfigurableDigitalOutputPulse", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setConfigurableDigitalOutputPulse(IntPtr h, int index, bool value, double duration);

        // 设置标准模拟输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "setStandardAnalogOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setStandardAnalogOutput(IntPtr h, int index, double value);

        // 设置工具模拟输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolAnalogOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolAnalogOutput(IntPtr h, int index, double value);

        // 获取标准数字输入函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardDigitalInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool getStandardDigitalInput(IntPtr h, int index);

        // 获取所有标准数字输入函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardDigitalInputs", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 getStandardDigitalInputs(IntPtr h);

        // 获取工具数字输入函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolDigitalInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool getToolDigitalInput(IntPtr h, int index);

        // 获取所有工具数字输入函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolDigitalInputs", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 getToolDigitalInputs(IntPtr h);

        // 获取可配置数字输入函数封装
        [DllImport(service_interface_dll, EntryPoint = "getConfigurableDigitalInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool getConfigurableDigitalInput(IntPtr h, int index);

        // 获取所有可配置数字输入函数封装
        [DllImport(service_interface_dll, EntryPoint = "getConfigurableDigitalInputs", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 getConfigurableDigitalInputs(IntPtr h);

        // 获取标准模拟输入函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardAnalogInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getStandardAnalogInput(IntPtr h, int index);

        // 获取工具模拟输入函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolAnalogInput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getToolAnalogInput(IntPtr h, int index);

        // 获取标准数字输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardDigitalOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool getStandardDigitalOutput(IntPtr h, int index);

        // 获取所有标准数字输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardDigitalOutputs", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 getStandardDigitalOutputs(IntPtr h);

        // 获取工具数字输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolDigitalOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool getToolDigitalOutput(IntPtr h, int index);

        // 获取所有工具数字输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolDigitalOutputs", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 getToolDigitalOutputs(IntPtr h);

        // 获取可配置数字输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "getConfigurableDigitalOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool getConfigurableDigitalOutput(IntPtr h, int index);

        // 获取所有可配置数字输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "getConfigurableDigitalOutputs", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 getConfigurableDigitalOutputs(IntPtr h);

        // 获取标准模拟输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStandardAnalogOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getStandardAnalogOutput(IntPtr h, int index);

        // 获取工具模拟输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolAnalogOutput", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getToolAnalogOutput(IntPtr h, int index);

        // 获取静态链路输入数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStaticLinkInputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getStaticLinkInputNum(IntPtr h);

        // 获取静态链路输出数量函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStaticLinkOutputNum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getStaticLinkOutputNum(IntPtr h);

        // 获取所有静态链路输入函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStaticLinkInputs", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 getStaticLinkInputs(IntPtr h);

        // 获取所有静态链路输出函数封装
        [DllImport(service_interface_dll, EntryPoint = "getStaticLinkOutputs", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 getStaticLinkOutputs(IntPtr h);

        // 判断是否有编码器传感器函数封装
        [DllImport(service_interface_dll, EntryPoint = "hasEncoderSensor", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool hasEncoderSensor(IntPtr h);

        // 设置编码器解码器类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "setEncDecoderType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setEncDecoderType(IntPtr h, int type, int range_id);

        // 设置编码器计数值函数封装
        [DllImport(service_interface_dll, EntryPoint = "setEncTickCount", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setEncTickCount(IntPtr h, int tick);

        // 获取编码器解码器类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "getEncDecoderType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getEncDecoderType(IntPtr h);

        // 获取编码器计数值函数封装
        [DllImport(service_interface_dll, EntryPoint = "getEncTickCount", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getEncTickCount(IntPtr h);

        // 展开编码器增量计数值函数封装
        [DllImport(service_interface_dll, EntryPoint = "unwindEncDeltaTickCount", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int unwindEncDeltaTickCount(IntPtr h, int delta_count);

        // 获取工具按钮状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolButtonStatus", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool getToolButtonStatus(IntPtr h);

    }
    public class cSharpBinging_MotionControl
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 获取等效半径函数封装
        [DllImport(service_interface_dll, EntryPoint = "getEqradius", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getEqradius(IntPtr h);

        // 设置等效半径函数封装
        [DllImport(service_interface_dll, EntryPoint = "setEqradius", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setEqradius(IntPtr h, double eqradius);

        // 获取速度分数函数封装
        [DllImport(service_interface_dll, EntryPoint = "getSpeedFraction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getSpeedFraction(IntPtr h);

        // 设置速度分数函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSpeedFraction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSpeedFraction(IntPtr h, double fraction);

        // 启用/禁用速度分数临界功能函数封装
        [DllImport(service_interface_dll, EntryPoint = "speedFractionCritical", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int speedFractionCritical(IntPtr h, bool enable);

        // 判断速度分数是否处于临界状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "isSpeedFractionCritical", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isSpeedFractionCritical(IntPtr h);

        // 判断是否处于混合状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "isBlending", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isBlending(IntPtr h);

        // 启用路径偏移功能函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathOffsetEnable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathOffsetEnable(IntPtr h);

        // 设置路径偏移函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathOffsetSet", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathOffsetSet(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] offset, int type);

        // 禁用路径偏移功能函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathOffsetDisable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathOffsetDisable(IntPtr h);

        // 启用关节偏移功能函数封装
        [DllImport(service_interface_dll, EntryPoint = "jointOffsetEnable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int jointOffsetEnable(IntPtr h);

        // 设置关节偏移函数封装
        [DllImport(service_interface_dll, EntryPoint = "jointOffsetSet", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int jointOffsetSet(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] offset, int type);

        // 禁用关节偏移功能函数封装
        [DllImport(service_interface_dll, EntryPoint = "jointOffsetDisable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int jointOffsetDisable(IntPtr h);

        // 获取轨迹队列大小函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTrajectoryQueueSize", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTrajectoryQueueSize(IntPtr h);

        // 获取队列大小函数封装
        [DllImport(service_interface_dll, EntryPoint = "getQueueSize", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getQueueSize(IntPtr h);

        // 获取执行ID函数封装
        [DllImport(service_interface_dll, EntryPoint = "getExecId", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getExecId(IntPtr h);

        // 获取指定ID的持续时间函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDuration", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getDuration(IntPtr h, int id);

        // 获取指定ID的运动剩余时间函数封装
        [DllImport(service_interface_dll, EntryPoint = "getMotionLeftTime", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getMotionLeftTime(IntPtr h, int id);

        // 获取进度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getProgress", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getProgress(IntPtr h);

        // 设置工作对象保持相关信息函数封装
        [DllImport(service_interface_dll, EntryPoint = "setWorkObjectHold", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setWorkObjectHold(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string module_name, [MarshalAs(UnmanagedType.LPArray)] double[] mounting_pose);

        // 获取工作对象保持相关信息函数封装
        [DllImport(service_interface_dll, EntryPoint = "getWorkObjectHold", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr getWorkObjectHold(IntPtr h);

        // 获取暂停关节位置函数封装
        [DllImport(service_interface_dll, EntryPoint = "getPauseJointPositions", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getPauseJointPositions(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 设置伺服模式启用状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "setServoMode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setServoMode(IntPtr h, bool enable);

        // 判断伺服模式是否启用函数封装
        [DllImport(service_interface_dll, EntryPoint = "isServoModeEnabled", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isServoModeEnabled(IntPtr h);

        // 设置伺服模式选择函数封装
        [DllImport(service_interface_dll, EntryPoint = "setServoModeSelect", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setServoModeSelect(IntPtr h, int mode);

        // 获取伺服模式选择函数封装
        [DllImport(service_interface_dll, EntryPoint = "getServoModeSelect", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getServoModeSelect(IntPtr h);

        // 关节伺服控制函数封装
        [DllImport(service_interface_dll, EntryPoint = "servoJoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int servoJoint(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, double a, double v, double t, double lookahead_time, double gain);

        // 笛卡尔坐标伺服控制函数封装
        [DllImport(service_interface_dll, EntryPoint = "servoCartesian", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int servoCartesian(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, double a, double v, double t, double lookahead_time, double gain);

        // 带外部坐标的关节伺服控制函数封装
        [DllImport(service_interface_dll, EntryPoint = "servoJointWithAxes", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int servoJointWithAxes(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, [MarshalAs(UnmanagedType.LPArray)] double[] extq, double a, double v, double t, double lookahead_time, double gain);

        // 带外部坐标的笛卡尔坐标伺服控制函数封装
        [DllImport(service_interface_dll, EntryPoint = "servoCartesianWithAxes", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int servoCartesianWithAxes(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, [MarshalAs(UnmanagedType.LPArray)] double[] extq, double a, double v, double t, double lookahead_time, double gain);

        // 关节轨迹跟踪函数封装
        [DllImport(service_interface_dll, EntryPoint = "trackJoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int trackJoint(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, double t, double smooth_scale, double delay_sacle);

        // 笛卡尔坐标轨迹跟踪函数封装
        [DllImport(service_interface_dll, EntryPoint = "trackCartesian", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int trackCartesian(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, double t, double smooth_scale, double delay_sacle);

        // 关节跟随控制函数封装
        [DllImport(service_interface_dll, EntryPoint = "followJoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int followJoint(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q);

        // 直线跟随控制函数封装
        [DllImport(service_interface_dll, EntryPoint = "followLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int followLine(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose);

        // 关节速度控制函数封装
        [DllImport(service_interface_dll, EntryPoint = "speedJoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int speedJoint(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] qd, double a, double t);

        // 恢复关节速度控制函数封装
        [DllImport(service_interface_dll, EntryPoint = "resumeSpeedJoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int resumeSpeedJoint(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] qd, double a, double t);

        // 直线速度控制函数封装
        [DllImport(service_interface_dll, EntryPoint = "speedLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int speedLine(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] xd, double a, double t);

        // 恢复直线速度控制函数封装
        [DllImport(service_interface_dll, EntryPoint = "resumeSpeedLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int resumeSpeedLine(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] xd, double a, double t);

        // 样条曲线运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "moveSpline", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int moveSpline(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, double a, double v, double duration);

        // 关节运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "moveJoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int moveJoint(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, double a, double v, double blend_radius, double duration);

        // 恢复关节运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "resumeMoveJoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int resumeMoveJoint(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, double a, double v, double duration);

        // 直线运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "moveLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int moveLine(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, double a, double v, double blend_radius, double duration);

        // 过程运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "moveProcess", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int moveProcess(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, double a, double v, double blend_radius);

        // 恢复直线运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "resumeMoveLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int resumeMoveLine(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, double a, double v, double duration);

        // 圆形运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "moveCircle", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int moveCircle(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] via_pose, [MarshalAs(UnmanagedType.LPArray)] double[] end_pose, double a, double v, double blend_radius, double duration);

        // 设置圆形路径模式函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCirclePathMode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCirclePathMode(IntPtr h, int mode);

        // 圆形运动（使用参数结构体）函数封装
        [DllImport(service_interface_dll, EntryPoint = "moveCircle2", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int moveCircle2(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] cSharpBinging_TypeDef.CircleParameters param);

        // 路径缓冲区分配函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathBufferAlloc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathBufferAlloc(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name, int type, int size);

        // 路径缓冲区添加路径点函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathBufferAppend", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathBufferAppend(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name, [MarshalAs(UnmanagedType.LPArray)] double[] waypoints, int rows);

        // 路径缓冲区评估函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathBufferEval", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathBufferEval(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name, [MarshalAs(UnmanagedType.LPArray)] double[] a, [MarshalAs(UnmanagedType.LPArray)] double[] v, double t);

        // 判断路径缓冲区是否有效函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathBufferValid", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool pathBufferValid(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 释放路径缓冲区函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathBufferFree", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathBufferFree(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 获取路径缓冲区列表函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathBufferList", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathBufferList(IntPtr h, out IntPtr result);

        // 按路径缓冲区移动函数封装
        [DllImport(service_interface_dll, EntryPoint = "movePathBuffer", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int movePathBuffer(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 移动到交点函数封装
        [DllImport(service_interface_dll, EntryPoint = "moveIntersection", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int moveIntersection(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] poses, int rows, double a, double v, double main_pipe_radius, double sub_pipe_radius, double normal_distance, double normal_alpha);

        // 停止关节运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "stopJoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int stopJoint(IntPtr h, double acc);

        // 恢复停止关节运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "resumeStopJoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int resumeStopJoint(IntPtr h, double acc);

        // 停止直线运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "stopLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int stopLine(IntPtr h, double acc, double acc_rot);

        // 恢复停止直线运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "resumeStopLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int resumeStopLine(IntPtr h, double acc, double acc_rot);

        // 编织运动开始函数封装
        [DllImport(service_interface_dll, EntryPoint = "weaveStart", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int weaveStart(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string param);

        // 编织运动结束函数封装
        [DllImport(service_interface_dll, EntryPoint = "weaveEnd", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int weaveEnd(IntPtr h);

        // 存储路径函数封装
        [DllImport(service_interface_dll, EntryPoint = "storePath", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int storePath(IntPtr h, bool keep_sync);

        // 停止运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "stopMove", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int stopMove(IntPtr h, bool quick, bool all_tasks);

        // 开始运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "startMove", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int startMove(IntPtr h);

        // 清除路径函数封装
        [DllImport(service_interface_dll, EntryPoint = "clearPath", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int clearPath(IntPtr h);

        // 恢复路径（可能是拼写错误，推测正确应为restorePath）函数封装
        [DllImport(service_interface_dll, EntryPoint = "restoPath", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int restoPath(IntPtr h);

        // 设置未来点采样周期函数封装
        [DllImport(service_interface_dll, EntryPoint = "setFuturePointSamplePeriod", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setFuturePointSamplePeriod(IntPtr h, double sample_time);

        // 获取未来关节路径点函数封装
        [DllImport(service_interface_dll, EntryPoint = "getFuturePathPointsJoint", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getFuturePathPointsJoint(IntPtr h, out IntPtr result);

        // 圆形传送带跟踪函数封装
        [DllImport(service_interface_dll, EntryPoint = "conveyorTrackCircle", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int conveyorTrackCircle(IntPtr h, int encoder_id, [MarshalAs(UnmanagedType.LPArray)] double[] center, int tick_per_revo, bool rotate_tool);

        // 直线传送带跟踪函数封装
        [DllImport(service_interface_dll, EntryPoint = "conveyorTrackLine", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int conveyorTrackLine(IntPtr h, int encoder_id, [MarshalAs(UnmanagedType.LPArray)] double[] direction, int tick_per_meter);

        // 停止传送带跟踪函数封装
        [DllImport(service_interface_dll, EntryPoint = "conveyorTrackStop", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int conveyorTrackStop(IntPtr h, double a);

        // 螺旋运动函数封装
        [DllImport(service_interface_dll, EntryPoint = "moveSpiral", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int moveSpiral(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] cSharpBinging_TypeDef.SpiralParameters param, double blend_radius, double v, double a, double t);

        // 设置路径偏移限制函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathOffsetLimits", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathOffsetLimits(IntPtr h, double v, double a);

        // 设置路径偏移坐标参考函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathOffsetCoordinate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathOffsetCoordinate(IntPtr h, int ref_coord);

        // 获取前瞻尺寸函数封装
        [DllImport(service_interface_dll, EntryPoint = "getLookAheadSize", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getLookAheadSize(IntPtr h);

        // 设置前瞻尺寸函数封装
        [DllImport(service_interface_dll, EntryPoint = "setLookAheadSize", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setLookAheadSize(IntPtr h, int size);

    }
    public class cSharpBinging_RobotAlgorithm
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 校准TCP力传感器（第一个版本）函数封装
        [DllImport(service_interface_dll, EntryPoint = "calibrateTcpForceSensor", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.ForceSensorCalibResult calibrateTcpForceSensor(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] forces, int forcesRows, [MarshalAs(UnmanagedType.LPArray)] double[] poses, int posesRows);
        // 校准TCP力传感器（第二个版本）函数封装
        [DllImport(service_interface_dll, EntryPoint = "calibrateTcpForceSensor2", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.ForceSensorCalibResult calibrateTcpForceSensor2(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] forces, int forcesRows, [MarshalAs(UnmanagedType.LPArray)] double[] poses, int posesRows);

        // 负载识别（版本1，使用两个字符串参数）函数封装
        [DllImport(service_interface_dll, EntryPoint = "payloadIdentify", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int payloadIdentify(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string data_no_payload, [MarshalAs(UnmanagedType.LPStr)] string data_with_payload);

        // 负载识别（版本2，使用文件名参数）函数封装
        [DllImport(service_interface_dll, EntryPoint = "payloadIdentify1", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int payloadIdentify1(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string file_name);

        // 判断负载计算是否完成函数封装
        [DllImport(service_interface_dll, EntryPoint = "payloadCalculateFinished", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int payloadCalculateFinished(IntPtr h);

        // 获取负载识别结果函数封装
        [DllImport(service_interface_dll, EntryPoint = "getPayloadIdentifyResult", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.Payload getPayloadIdentifyResult(IntPtr h);

        // 生成负载识别轨迹函数封装
        [DllImport(service_interface_dll, EntryPoint = "generatePayloadIdentifyTraj", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int generatePayloadIdentifyTraj(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name, [MarshalAs(UnmanagedType.LPArray)] cSharpBinging_TypeDef.TrajConfig TrajConfig);

        // 判断负载识别轨迹生成是否完成函数封装
        [DllImport(service_interface_dll, EntryPoint = "payloadIdentifyTrajGenFinished", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int payloadIdentifyTrajGenFinished(IntPtr h);

        // 摩擦模型识别函数封装
        [DllImport(service_interface_dll, EntryPoint = "frictionModelIdentify", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool frictionModelIdentify(IntPtr h,
            [MarshalAs(UnmanagedType.LPArray)] double[] q, int qRows,
            [MarshalAs(UnmanagedType.LPArray)] double[] qd, int qdRows,
            [MarshalAs(UnmanagedType.LPArray)] double[] qdd, int qddRows,
            [MarshalAs(UnmanagedType.LPArray)] double[] temp, int tempRows);

        // 校准工件坐标参数函数封装
        [DllImport(service_interface_dll, EntryPoint = "calibWorkpieceCoordinatePara", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int calibWorkpieceCoordinatePara(IntPtr h,
            [MarshalAs(UnmanagedType.LPArray)] double[] q, int qRows, int type,
            [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 正向动力学（版本1）函数封装
        [DllImport(service_interface_dll, EntryPoint = "forwardDynamics", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int forwardDynamics(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, [MarshalAs(UnmanagedType.LPArray)] double[] torqs, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 正向运动学（版本1）函数封装
        [DllImport(service_interface_dll, EntryPoint = "forwardKinematics", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int forwardKinematics(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 正向工具运动学（版本1）函数封装
        [DllImport(service_interface_dll, EntryPoint = "forwardToolKinematics", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int forwardToolKinematics(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 正向动力学（版本2）函数封装
        [DllImport(service_interface_dll, EntryPoint = "forwardDynamics1", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int forwardDynamics1(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, [MarshalAs(UnmanagedType.LPArray)] double[] torqs, [MarshalAs(UnmanagedType.LPArray)] double[] tcp_offset, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 正向运动学（版本2）函数封装
        [DllImport(service_interface_dll, EntryPoint = "forwardKinematics1", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int forwardKinematics1(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, [MarshalAs(UnmanagedType.LPArray)] double[] tcp_offset, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 逆运动学（版本1）函数封装
        [DllImport(service_interface_dll, EntryPoint = "inverseKinematics", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int inverseKinematics(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] qnear, [MarshalAs(UnmanagedType.LPArray)] double[] pose, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 逆运动学（获取所有解，版本1）函数封装
        [DllImport(service_interface_dll, EntryPoint = "inverseKinematicsAll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int inverseKinematicsAll(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, out IntPtr result);

        // 逆运动学（版本2）函数封装
        [DllImport(service_interface_dll, EntryPoint = "inverseKinematics1", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int inverseKinematics1(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] qnear, [MarshalAs(UnmanagedType.LPArray)] double[] pose, [MarshalAs(UnmanagedType.LPArray)] double[] tcp_offset, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 逆运动学（获取所有解，版本2）函数封装
        [DllImport(service_interface_dll, EntryPoint = "inverseKinematicsAll1", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int inverseKinematicsAll1(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, [MarshalAs(UnmanagedType.LPArray)] double[] tcp_offset, out IntPtr result);

        // 逆工具运动学（版本1）函数封装
        [DllImport(service_interface_dll, EntryPoint = "inverseToolKinematics", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int inverseToolKinematics(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] qnear, [MarshalAs(UnmanagedType.LPArray)] double[] pose, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 逆工具运动学（获取所有解，版本1）函数封装
        [DllImport(service_interface_dll, EntryPoint = "inverseToolKinematicsAll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int inverseToolKinematicsAll(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose, out IntPtr result);

        // 路径关节移动函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathMovej", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathMovej(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q1, double r1, [MarshalAs(UnmanagedType.LPArray)] double[] q2, double r2, double d, out IntPtr result);

        // 三点路径混合函数封装
        [DllImport(service_interface_dll, EntryPoint = "pathBlend3Points", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pathBlend3Points(IntPtr h, int type, [MarshalAs(UnmanagedType.LPArray)] double[] q_start, [MarshalAs(UnmanagedType.LPArray)] double[] q_via, [MarshalAs(UnmanagedType.LPArray)] double[] q_to, double r, double d, out IntPtr result);

        // 计算雅可比矩阵函数封装
        [DllImport(service_interface_dll, EntryPoint = "calcJacobian", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int calcJacobian(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] q, bool base_or_end, [MarshalAs(UnmanagedType.LPArray)] double[] result);

    }
    public class cSharpBinging_RobotConfig
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 获取自由度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDof", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getDof(IntPtr h);

        // 获取机器人名称函数封装
        [DllImport(service_interface_dll, EntryPoint = "getName", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getName(IntPtr h, [Out] StringBuilder result);

        // 获取循环时间函数封装
        [DllImport(service_interface_dll, EntryPoint = "getCycletime", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getCycletime(IntPtr h);

        // 设置减速分数函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSlowDownFraction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSlowDownFraction(IntPtr h, int level, double fraction);

        // 获取减速分数函数封装
        [DllImport(service_interface_dll, EntryPoint = "getSlowDownFraction", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getSlowDownFraction(IntPtr h, int level);

        // 获取机器人类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "getRobotType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getRobotType(IntPtr h, [Out] StringBuilder result);

        // 获取机器人子类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "getRobotSubType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getRobotSubType(IntPtr h, [Out] StringBuilder result);

        // 获取控制箱类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "getControlBoxType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getControlBoxType(IntPtr h, [Out] StringBuilder result);

        // 获取默认工具加速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDefaultToolAcc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getDefaultToolAcc(IntPtr h);

        // 获取默认工具速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDefaultToolSpeed", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getDefaultToolSpeed(IntPtr h);

        // 获取默认关节加速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDefaultJointAcc", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getDefaultJointAcc(IntPtr h);

        // 获取默认关节速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getDefaultJointSpeed", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getDefaultJointSpeed(IntPtr h);

        // 设置安装姿态函数封装
        [DllImport(service_interface_dll, EntryPoint = "setMountingPose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setMountingPose(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose);

        // 获取安装姿态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getMountingPose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getMountingPose(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 设置碰撞级别函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCollisionLevel", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCollisionLevel(IntPtr h, int level);

        // 获取碰撞级别函数封装
        [DllImport(service_interface_dll, EntryPoint = "getCollisionLevel", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getCollisionLevel(IntPtr h);

        // 设置碰撞停止类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "setCollisionStopType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setCollisionStopType(IntPtr h, int type);

        // 获取碰撞停止类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "getCollisionStopType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getCollisionStopType(IntPtr h);

        // 设置回零位置函数封装
        [DllImport(service_interface_dll, EntryPoint = "setHomePosition", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setHomePosition(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] positions);

        // 获取回零位置函数封装
        [DllImport(service_interface_dll, EntryPoint = "getHomePosition", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getHomePosition(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 设置自由驱动阻尼函数封装
        [DllImport(service_interface_dll, EntryPoint = "setFreedriveDamp", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setFreedriveDamp(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] damp);

        // 获取自由驱动阻尼函数封装
        [DllImport(service_interface_dll, EntryPoint = "getFreedriveDamp", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getFreedriveDamp(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取TCP力传感器名称列表函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpForceSensorNames", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpForceSensorNames(IntPtr h, out IntPtr result);

        // 选择TCP力传感器函数封装
        [DllImport(service_interface_dll, EntryPoint = "selectTcpForceSensor", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int selectTcpForceSensor(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 判断是否有TCP力传感器函数封装
        [DllImport(service_interface_dll, EntryPoint = "hasTcpForceSensor", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool hasTcpForceSensor(IntPtr h);

        // 设置TCP力偏移函数封装
        [DllImport(service_interface_dll, EntryPoint = "setTcpForceOffset", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setTcpForceOffset(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] force_offset);

        // 获取TCP力偏移函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpForceOffset", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpForceOffset(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取基座力传感器名称列表函数封装
        [DllImport(service_interface_dll, EntryPoint = "getBaseForceSensorNames", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getBaseForceSensorNames(IntPtr h, out IntPtr result);

        // 选择基座力传感器函数封装
        [DllImport(service_interface_dll, EntryPoint = "selectBaseForceSensor", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int selectBaseForceSensor(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

        // 判断是否有基座力传感器函数封装
        [DllImport(service_interface_dll, EntryPoint = "hasBaseForceSensor", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool hasBaseForceSensor(IntPtr h);

        // 设置基座力偏移函数封装
        [DllImport(service_interface_dll, EntryPoint = "setBaseForceOffset", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setBaseForceOffset(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] force_offset);

        // 获取基座力偏移函数封装
        [DllImport(service_interface_dll, EntryPoint = "getBaseForceOffset", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getBaseForceOffset(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 设置持久化参数函数封装
        [DllImport(service_interface_dll, EntryPoint = "setPersistentParameters", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setPersistentParameters(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string param);

        // 设置运动学补偿参数函数封装
        [DllImport(service_interface_dll, EntryPoint = "setKinematicsCompensate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setKinematicsCompensate(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] param);

        // 设置硬件自定义参数函数封装
        [DllImport(service_interface_dll, EntryPoint = "setHardwareCustomParameters", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setHardwareCustomParameters(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string param);

        // 获取硬件自定义参数函数封装
        [DllImport(service_interface_dll, EntryPoint = "getHardwareCustomParameters", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getHardwareCustomParameters(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string param, [Out] StringBuilder result);

        // 设置机器人零点函数封装
        [DllImport(service_interface_dll, EntryPoint = "setRobotZero", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setRobotZero(IntPtr h);

        // 获取运动学参数函数封装
        [DllImport(service_interface_dll, EntryPoint = "getKinematicsParam", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr getKinematicsParam(IntPtr h, bool real);

        // 获取运动学补偿参数（指定参考温度）函数封装
        [DllImport(service_interface_dll, EntryPoint = "getKinematicsCompensate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr getKinematicsCompensate(IntPtr h, double ref_temperature);

        // 获取安全参数校验和函数封装
        [DllImport(service_interface_dll, EntryPoint = "getSafetyParametersCheckSum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 getSafetyParametersCheckSum(IntPtr h);

        // 确认安全参数函数封装
        [DllImport(service_interface_dll, EntryPoint = "confirmSafetyParameters", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int confirmSafetyParameters(IntPtr h, cSharpBinging_TypeDef.RobotSafetyParameterRange_C parameters);

        // 计算安全参数校验和函数封装
        [DllImport(service_interface_dll, EntryPoint = "calcSafetyParametersCheckSum", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 calcSafetyParametersCheckSum(IntPtr h, cSharpBinging_TypeDef.RobotSafetyParameterRange_C parameters);

        // 获取关节最大位置函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointMaxPositions", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointMaxPositions(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节最小位置函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointMinPositions", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointMinPositions(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节最大速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointMaxSpeeds", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointMaxSpeeds(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节最大加速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointMaxAccelerations", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointMaxAccelerations(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取TCP最大速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpMaxSpeeds", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpMaxSpeeds(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取TCP最大加速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpMaxAccelerations", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpMaxAccelerations(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 判断工具空间是否在范围内函数封装
        [DllImport(service_interface_dll, EntryPoint = "toolSpaceInRange", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool toolSpaceInRange(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] pose);

        // 设置负载函数封装
        [DllImport(service_interface_dll, EntryPoint = "setPayload", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setPayload(IntPtr h, double m, [MarshalAs(UnmanagedType.LPArray)] double[] cog, [MarshalAs(UnmanagedType.LPArray)] double[] aom, [MarshalAs(UnmanagedType.LPArray)] double[] inertia);

        // 获取负载函数封装
        [DllImport(service_interface_dll, EntryPoint = "getPayload", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.Payload getPayload(IntPtr h);

        // 获取TCP偏移函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpOffset", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpOffset(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取重力函数封装
        [DllImport(service_interface_dll, EntryPoint = "getGravity", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getGravity(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 设置重力函数封装
        [DllImport(service_interface_dll, EntryPoint = "setGravity", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setGravity(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] gravity);

        // 设置TCP偏移函数封装
        [DllImport(service_interface_dll, EntryPoint = "setTcpOffset", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setTcpOffset(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] offset);

        // 设置工具惯性参数函数封装
        [DllImport(service_interface_dll, EntryPoint = "setToolInertial", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setToolInertial(IntPtr h, double m, [MarshalAs(UnmanagedType.LPArray)] double[] com, [MarshalAs(UnmanagedType.LPArray)] double[] inertial);

        // 固件更新函数封装
        [DllImport(service_interface_dll, EntryPoint = "firmwareUpdate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int firmwareUpdate(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string fw);

        // 获取固件更新进度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getFirmwareUpdateProcess", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getFirmwareUpdateProcess(IntPtr h);

        // 设置TCP力传感器姿态函数封装
        [DllImport(service_interface_dll, EntryPoint = "setTcpForceSensorPose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setTcpForceSensorPose(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] sensor_pose);

        // 获取TCP力传感器姿态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpForceSensorPose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpForceSensorPose(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节极限最大位置函数封装
        [DllImport(service_interface_dll, EntryPoint = "getLimitJointMaxPositions", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getLimitJointMaxPositions(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节极限最小位置函数封装
        [DllImport(service_interface_dll, EntryPoint = "getLimitJointMinPositions", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getLimitJointMinPositions(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节极限最大速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getLimitJointMaxSpeeds", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getLimitJointMaxSpeeds(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节极限最大加速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getLimitJointMaxAccelerations", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getLimitJointMaxAccelerations(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取TCP极限最大速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getLimitTcpMaxSpeed", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getLimitTcpMaxSpeed(IntPtr h);

        // 获取安全保护停止类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "getSafeguardStopType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.SafeguedStopType getSafeguardStopType(IntPtr h);

        // 获取安全保护停止源函数封装
        [DllImport(service_interface_dll, EntryPoint = "getSafeguardStopSource", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getSafeguardStopSource(IntPtr h);

    }
    public class cSharpBinging_RobotManage
    {
        const string service_interface_dll = GlobalConstants.service_interface_dll;


        // 机器人上电函数封装
        [DllImport(service_interface_dll, EntryPoint = "poweron", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int poweron(IntPtr h);

        // 机器人启动函数封装
        [DllImport(service_interface_dll, EntryPoint = "startup", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int startup(IntPtr h);

        // 机器人下电函数封装
        [DllImport(service_interface_dll, EntryPoint = "poweroff", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int poweroff(IntPtr h);

        // 机器人反向驱动使能函数封装
        [DllImport(service_interface_dll, EntryPoint = "backdrive", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int backdrive(IntPtr h, bool enable);

        // 机器人自由驱动使能函数封装
        [DllImport(service_interface_dll, EntryPoint = "freedrive", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int freedrive(IntPtr h, bool enable);

        // 手动引导模式函数封装
        [DllImport(service_interface_dll, EntryPoint = "handguideMode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int handguideMode(IntPtr h, out Int32 freeAxes, [MarshalAs(UnmanagedType.LPArray)] double[] feature);

        // 退出手动引导模式函数封装
        [DllImport(service_interface_dll, EntryPoint = "exitHandguideMode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int exitHandguideMode(IntPtr h);

        // 获取手动引导状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getHandguideStatus", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getHandguideStatus(IntPtr h);

        // 获取手动引导触发信号函数封装
        [DllImport(service_interface_dll, EntryPoint = "getHandguideTrigger", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getHandguideTrigger(IntPtr h);

        // 判断手动引导是否使能函数封装
        [DllImport(service_interface_dll, EntryPoint = "isHandguideEnabled", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isHandguideEnabled(IntPtr h);

        // 设置模拟模式使能函数封装
        [DllImport(service_interface_dll, EntryPoint = "setSim", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setSim(IntPtr h, bool enable);

        // 设置操作模式函数封装
        [DllImport(service_interface_dll, EntryPoint = "setOperationalMode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setOperationalMode(IntPtr h, cSharpBinging_TypeDef.OperationalModeType mode);

        // 获取操作模式函数封装
        [DllImport(service_interface_dll, EntryPoint = "getOperationalMode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.OperationalModeType getOperationalMode(IntPtr h);

        // 获取机器人控制模式函数封装
        [DllImport(service_interface_dll, EntryPoint = "getRobotControlMode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.RobotControlModeType getRobotControlMode(IntPtr h);

        // 判断模拟模式是否使能函数封装
        [DllImport(service_interface_dll, EntryPoint = "isSimulationEnabled", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isSimulationEnabled(IntPtr h);

        // 判断自由驱动是否使能函数封装
        [DllImport(service_interface_dll, EntryPoint = "isFreedriveEnabled", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isFreedriveEnabled(IntPtr h);

        // 判断反向驱动是否使能函数封装
        [DllImport(service_interface_dll, EntryPoint = "isBackdriveEnabled", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isBackdriveEnabled(IntPtr h);

        // 设置解锁保护停止函数封装
        [DllImport(service_interface_dll, EntryPoint = "setUnlockProtectiveStop", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setUnlockProtectiveStop(IntPtr h);

        // 开始记录函数封装
        [DllImport(service_interface_dll, EntryPoint = "startRecord", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int startRecord(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string file_name);

        // 停止记录函数封装
        [DllImport(service_interface_dll, EntryPoint = "stopRecord", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int stopRecord(IntPtr h);

        // 暂停/恢复记录函数封装
        [DllImport(service_interface_dll, EntryPoint = "pauseRecord", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int pauseRecord(IntPtr h, bool pause);

        // 重启接口板函数封装
        [DllImport(service_interface_dll, EntryPoint = "restartInterfaceBoard", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int restartInterfaceBoard(IntPtr h);

        // 设置链路模式使能函数封装
        [DllImport(service_interface_dll, EntryPoint = "setLinkModeEnable", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int setLinkModeEnable(IntPtr h, bool enable);

        // 判断链路模式是否使能函数封装
        [DllImport(service_interface_dll, EntryPoint = "isLinkModeEnabled", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isLinkModeEnabled(IntPtr h);

    }
    public class cSharpBinging_RobotState
    {

        const string service_interface_dll = GlobalConstants.service_interface_dll;

        // 获取机器人模式类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "getRobotModeType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.RobotModeType getRobotModeType(IntPtr h);

        // 获取安全模式类型函数封装
        [DllImport(service_interface_dll, EntryPoint = "getSafetyModeType", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern cSharpBinging_TypeDef.SafetyModeType getSafetyModeType(IntPtr h);

        // 判断是否上电函数封装
        [DllImport(service_interface_dll, EntryPoint = "isPowerOn", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isPowerOn(IntPtr h);

        // 判断是否稳定函数封装
        [DllImport(service_interface_dll, EntryPoint = "isSteady", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isSteady(IntPtr h);

        // 判断是否发生碰撞函数封装
        [DllImport(service_interface_dll, EntryPoint = "isCollisionOccurred", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isCollisionOccurred(IntPtr h);

        // 判断是否在安全限制内函数封装
        [DllImport(service_interface_dll, EntryPoint = "isWithinSafetyLimits", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool isWithinSafetyLimits(IntPtr h);

        // 获取TCP位姿函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpPose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpPose(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取实际TCP偏移函数封装
        [DllImport(service_interface_dll, EntryPoint = "getActualTcpOffset", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getActualTcpOffset(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取目标TCP位姿函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTargetTcpPose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTargetTcpPose(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取工具位姿函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolPose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolPose(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取TCP速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpSpeed", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpSpeed(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取TCP力函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpForce", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpForce(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取肘部位置函数封装
        [DllImport(service_interface_dll, EntryPoint = "getElbowPosistion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getElbowPosistion(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取肘部速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getElbowVelocity", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getElbowVelocity(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取基座力函数封装
        [DllImport(service_interface_dll, EntryPoint = "getBaseForce", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getBaseForce(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取TCP目标位姿函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpTargetPose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpTargetPose(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取TCP目标速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpTargetSpeed", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpTargetSpeed(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取TCP目标力函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpTargetForce", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpTargetForce(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节状态函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointState", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointState(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] cSharpBinging_TypeDef.JointStateType result);

        // 获取关节伺服模式函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointServoMode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointServoMode(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] cSharpBinging_TypeDef.JointServoModeType result);

        // 获取关节位置函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointPositions", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointPositions(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节位置历史函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointPositionsHistory", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointPositionsHistory(IntPtr h, int steps, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointSpeeds", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointSpeeds(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节加速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointAccelerations", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointAccelerations(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节扭矩传感器数据函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointTorqueSensors", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointTorqueSensors(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节接触扭矩函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointContactTorques", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointContactTorques(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取基座力传感器数据函数封装
        [DllImport(service_interface_dll, EntryPoint = "getBaseForceSensor", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getBaseForceSensor(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取TCP力传感器数据函数封装
        [DllImport(service_interface_dll, EntryPoint = "getTcpForceSensors", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getTcpForceSensors(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节电流函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointCurrents", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointCurrents(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节电压函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointVoltages", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointVoltages(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节温度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointTemperatures", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointTemperatures(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节唯一ID函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointUniqueIds", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointUniqueIds(IntPtr h, out IntPtr result);

        // 获取关节固件版本函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointFirmwareVersions", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointFirmwareVersions(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] int[] result);

        // 获取关节硬件版本函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointHardwareVersions", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointHardwareVersions(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] int[] result);

        // 获取主控板唯一ID函数封装
        [DllImport(service_interface_dll, EntryPoint = "getMasterBoardUniqueId", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getMasterBoardUniqueId(IntPtr h, [Out] StringBuilder result);

        // 获取主控板固件版本函数封装
        [DllImport(service_interface_dll, EntryPoint = "getMasterBoardFirmwareVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getMasterBoardFirmwareVersion(IntPtr h);

        // 获取主控板硬件版本函数封装
        [DllImport(service_interface_dll, EntryPoint = "getMasterBoardHardwareVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getMasterBoardHardwareVersion(IntPtr h);

        // 获取从控板唯一ID函数封装
        [DllImport(service_interface_dll, EntryPoint = "getSlaveBoardUniqueId", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getSlaveBoardUniqueId(IntPtr h, [Out] StringBuilder result);

        // 获取从控板固件版本函数封装
        [DllImport(service_interface_dll, EntryPoint = "getSlaveBoardFirmwareVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getSlaveBoardFirmwareVersion(IntPtr h);

        // 获取从控板硬件版本函数封装
        [DllImport(service_interface_dll, EntryPoint = "getSlaveBoardHardwareVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getSlaveBoardHardwareVersion(IntPtr h);

        // 获取工具唯一ID函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolUniqueId", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolUniqueId(IntPtr h, [Out] StringBuilder result);

        // 获取工具固件版本函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolFirmwareVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolFirmwareVersion(IntPtr h);

        // 获取工具硬件版本函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolHardwareVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolHardwareVersion(IntPtr h);

        // 获取工具通信模式函数封装
        [DllImport(service_interface_dll, EntryPoint = "getToolCommMode", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getToolCommMode(IntPtr h);

        // 获取底座唯一ID函数封装
        [DllImport(service_interface_dll, EntryPoint = "getPedestalUniqueId", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getPedestalUniqueId(IntPtr h, [Out] StringBuilder result);

        // 获取底座固件版本函数封装
        [DllImport(service_interface_dll, EntryPoint = "getPedestalFirmwareVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getPedestalFirmwareVersion(IntPtr h);

        // 获取底座硬件版本函数封装
        [DllImport(service_interface_dll, EntryPoint = "getPedestalHardwareVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getPedestalHardwareVersion(IntPtr h);

        // 获取关节目标位置函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointTargetPositions", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointTargetPositions(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节目标速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointTargetSpeeds", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointTargetSpeeds(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节目标加速度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointTargetAccelerations", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointTargetAccelerations(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节目标扭矩函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointTargetTorques", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointTargetTorques(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取关节目标电流函数封装
        [DllImport(service_interface_dll, EntryPoint = "getJointTargetCurrents", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getJointTargetCurrents(IntPtr h, [MarshalAs(UnmanagedType.LPArray)] double[] result);

        // 获取控制箱温度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getControlBoxTemperature", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getControlBoxTemperature(IntPtr h);

        // 获取控制箱湿度函数封装
        [DllImport(service_interface_dll, EntryPoint = "getControlBoxHumidity", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getControlBoxHumidity(IntPtr h);

        // 获取主电压函数封装
        [DllImport(service_interface_dll, EntryPoint = "getMainVoltage", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getMainVoltage(IntPtr h);

        // 获取主电流函数封装
        [DllImport(service_interface_dll, EntryPoint = "getMainCurrent", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getMainCurrent(IntPtr h);

        // 获取机器人电压函数封装
        [DllImport(service_interface_dll, EntryPoint = "getRobotVoltage", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getRobotVoltage(IntPtr h);

        // 获取机器人电流函数封装
        [DllImport(service_interface_dll, EntryPoint = "getRobotCurrent", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern double getRobotCurrent(IntPtr h);

        // 获取减速等级函数封装
        [DllImport(service_interface_dll, EntryPoint = "getSlowDownLevel", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        public static extern int getSlowDownLevel(IntPtr h);
    }


}
