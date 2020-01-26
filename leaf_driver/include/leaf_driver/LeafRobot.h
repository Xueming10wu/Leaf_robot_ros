#ifndef LEAFROBOT_H
#define LEAFROBOT_H

#include "leaf_driver/ManipulatorProtocol.h"


//相当于MyRbot
class LeafRobot
{
public:
    LeafRobot( string name );       //传入action的名称
    virtual ~LeafRobot();
    void home();        //收起来
    void zero();        //直立起来
    bool read();         //0为正常读取到数据，-1为没有读取到数据或数据出错
    int write();       //数据写入

    //timer回调函数
    void timerCallback(const ros::TimerEvent& e);
    
    //goal回调函数
    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

    //添加路点到容器中 并重新排序
    void reorder_traj_joints(trajectory_msgs::JointTrajectory trajectory_);

    //执行轨迹上的所有路点
    void trackMove();

    //某个路点已经到达
    bool isArrived();

    //机械臂当前还处于运动状态
    bool isMoving();

    //关节状态更新
    void jointStateUpdate();

private:
    //句柄实例
    ros::NodeHandle nh_;

    //ros系统时间
	ros::Time time_current, time_prev;

    //ros定时器
    ros::Timer timer;
    double clock;
	
    string action_name;
    //定义action服务端
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>  as_;    
    //actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>  as_;

    //反馈
    control_msgs::FollowJointTrajectoryFeedback feedback_;
	
    //用来反馈action目标的执行情况，客户端由此可以得知服务端是否执行成功了
    control_msgs::FollowJointTrajectoryResult result_; 


    //关节状态发布
    ros::Publisher joint_pub_;
    sensor_msgs::JointState msg;
    sensor_msgs::JointState msg_pre;
    
    //串口
    string serialport_name;
    int baudrate;

    //通讯协议对象
    ManipulatorProtocol * manipulatorProtocolPtr;

    //通讯质量
    int errortimes;
    long reads;
    long writes;

    //自由度
    const int joint_count;
    string * joint_name;


    //路径点容器
    vector<trajectory_msgs::JointTrajectoryPoint> wayPoints;

    //下一个路径点对应的数据
    vector<StepMotor> nextPoint;

    //各个关节转PI/2需要的节拍
    int * plu2angel;
    int * zeroPlu;
};

#endif //LEAFROBOT
