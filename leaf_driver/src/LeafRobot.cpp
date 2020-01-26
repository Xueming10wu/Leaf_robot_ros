#include "leaf_driver/LeafRobot.h"


LeafRobot::LeafRobot(string name) : as_(nh_, name, boost::bind(&LeafRobot::executeCB, this,  _1), false) ,joint_count(6)
{   
    action_name = name;
    //私有参数 获取
    ros::NodeHandle nh_private("~");
    nh_private.param<string>("serialport_name", serialport_name, "/dev/ttyACM0");
    nh_private.param<int>("baudrate", baudrate, 230400);
    clock = 0.025;

    cout << "串口名 : " << serialport_name << "  , 波特率 : " << baudrate << endl;

    //初始化一些变量
    joint_name = new string[joint_count];
    plu2angel = new int[joint_count];
    zeroPlu = new int[joint_count];
    msg.name.resize(joint_count);
    msg.position.resize(joint_count);
    msg.header.frame_id = "/leaf";

    //下一个路径点
    StepMotor stepmotor = { 0, 0, 0 };
    for (size_t i = 0; i < joint_count; i ++)
    {
        nextPoint.push_back(stepmotor);
    }

    //重要参数
    //旋转+90°(+1.5707963)，需要的节拍
    plu2angel[0] = -7800;
    plu2angel[1] = -6350;
    plu2angel[2] = -8000;
    plu2angel[3] = -4550;
    plu2angel[4] = -3400;
    plu2angel[5] = -1600;

    //零点参数
    zeroPlu[0] = 0;
    zeroPlu[1] = -5600;
    zeroPlu[2] = 6600;
    zeroPlu[3] = 0;
    zeroPlu[4] = -3400;
    zeroPlu[5] = 0;

    for (size_t i = 0; i < joint_count; i++)
    {
        msg.position[i] = 0;
    }
    errortimes = 0;
    reads = 0;
    writes = 0;
    

    //关节命名
    stringstream ss;
    ss.clear();ss.str("");
    for (size_t i = 0; i < joint_count; i++)
    {
        ss << "joint" << i + 1;
        joint_name[i] = ss.str();
        msg.name[i] = joint_name[i];
        ss.clear();ss.str("");
    }

    //初始化协议
    manipulatorProtocolPtr = new ManipulatorProtocol();
    manipulatorProtocolPtr->manipulatorInit(joint_count);
    manipulatorProtocolPtr->setSerialPort(serialport_name, baudrate);

    //关节发布
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/leaf/joint_states", 1);

    timer = nh_.createTimer(ros::Duration(clock), &LeafRobot::timerCallback, this);
    cout << "定时器启动完毕" <<endl;
    cout << "机械臂初始化中..." << endl;
    usleep(100000);
    manipulatorProtocolPtr->udpHandle->serialPort->flush ();
    //到达0点
    //zero();
    //write();
    cout << "机械臂到达零点位置..." <<endl;
    usleep(1000000);
    cout << "机械臂到达零点位置完毕，服务端启动中" <<endl;
    as_.start();    //服务端启动
    cout << "服务端启动完毕" <<endl;
}

LeafRobot::~LeafRobot()
{   
    delete manipulatorProtocolPtr;
    delete []joint_name;
    delete []plu2angel;
    
    manipulatorProtocolPtr = NULL;
    joint_name = NULL;
    plu2angel = NULL;
    cout << "机械臂程序已退出，请等待5s左右，并请关掉电源，以免电机过热。"<< endl;
}


void LeafRobot::home()
{
    for (size_t i = 0; i < joint_count; i ++)
    {
        manipulatorProtocolPtr->writeManipulator.stepMotorList[i].position = 0;
    }
    cout << "home..." << endl << endl;
}

void LeafRobot::zero()
{
    for (size_t i = 0; i < joint_count; i ++)
    {
        manipulatorProtocolPtr->writeManipulator.stepMotorList[i].position = zeroPlu[i];
    }
    cout << "zero..." << endl << endl;
}


bool LeafRobot::read()
{   
    if (manipulatorProtocolPtr->udpHandle->serialPort->available()  >= 57)
    {
        if (manipulatorProtocolPtr->read())
        {
            reads++;
            //获取时间
            /*
            time_current = ros::Time::now();
            double elapsed = (time_current - time_prev).toSec();
            time_prev = time_current;

            cout << "\nErrorTimes : " << errortimes
                    << "  , R : " << reads
                    << "  , W : " << writes 
                    << "  , Rate: " << setprecision(8) << (double)(reads-errortimes) * 100 / (double)reads << "%" 
                    << "    duration : " << setprecision(8) << elapsed << endl;
            */
            return true;
        }
        else
        {
            errortimes++;
            //cout << "\n\033[31mGetError\033[0m";
            cout << "\033[31mErrorTimes\033[0m : " << errortimes
                    << "  , R : " << reads
                    << "  , W : " << writes 
                    << "  , Rate: " << setprecision(8) << (double)(reads-errortimes) * 100 / (double)reads << "%\n";
            timer.stop();
            manipulatorProtocolPtr->udpHandle->serialPort->flush();
            usleep(30000);//停止0.5s
            timer.start();

        }
    }
    //cout << "没有读到数据" << endl;
    return false;
}

int LeafRobot::write()
{
    writes++;
    //从cmd中获取数据
    
    /*
    for (size_t i = 0; i < joint_count; i++)
    {
        manipulatorProtocolPtr->writeManipulator.stepMotorList[i].position = round(cmd[i] * 2 * plu2angel[i] / PI) + zeroPlu[i];
        //cout << cmd[i] << " ";
    }
    time_current = ros::Time::now();
    double elapsed = (time_current - time_prev).toSec();
    time_prev = time_current;
    cout << "writing duration : " << elapsed << endl;*/
    return manipulatorProtocolPtr->write();
    //cout << endl ;
}


void LeafRobot::timerCallback(const ros::TimerEvent& e)
{
    //cout << "TimerCallback\n";
    //write();
    if(read())
    {
        return;
    }
    usleep(1000);
    if(read())
    {
        return;
    }
}


void LeafRobot::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    cout << "executeCB\n";
    reorder_traj_joints(goal->trajectory);
    trackMove();

    if (isArrived())
    {
        result_.error_code = result_.SUCCESSFUL;
        as_.setSucceeded(result_);
    }
    else
    {
        result_.error_code = result_.GOAL_TOLERANCE_VIOLATED;
        as_.setAborted(result_);
    }
    
    
}

void LeafRobot::reorder_traj_joints(trajectory_msgs::JointTrajectory trajectory_)
{
    feedback_.header.frame_id = trajectory_.header.frame_id;
    //添加路点到容器中
    cout << "Points count : " << trajectory_.points.size() << endl;
    for (size_t seq = 0; seq < trajectory_.points.size(); seq ++)
    {
        wayPoints.push_back(trajectory_.points[seq]);
    }

    //Feedback也要进行设计
    feedback_.joint_names.clear();

    //根据名称进行排序
    for (size_t index = 0; index < joint_count; index ++)
    {//joint_names中，逐个索引
        const char *p = trajectory_.joint_names[index].c_str();
        int degree_id = 0;
        for (size_t i = 0; i < trajectory_.joint_names[index].length(); i ++)
        {//string -> int。 将编号提取出来
            degree_id = degree_id * 10 + (int)(p[i + 5] - '0');
        }
        feedback_.joint_names.push_back(trajectory_.joint_names[index]);
    }
}


void LeafRobot::trackMove()
{
    //从cmd中获取数据
    cout << "TrackMoving \n";
    feedback_.desired.positions.clear();

    //将路点的终点写入
    for (size_t i = 0; i < joint_count; i++)
    {
        feedback_.desired.positions.push_back(wayPoints[wayPoints.size() - 1].positions[i]);
    }
    
    for (size_t seq = 0; seq < wayPoints.size(); seq ++)
    {
        //cout << "Push " << seq << "th point : \n";
        for (size_t i = 0; i < joint_count; i++)
        {//需要到达的点
            //feedback_.desired.positions.push_back(wayPoints[seq].positions[i]);
            nextPoint[i].position = round(wayPoints[seq].positions[i] * 2 * plu2angel[i] / PI) + zeroPlu[i];
            manipulatorProtocolPtr->writeManipulator.stepMotorList[i].position = nextPoint[i].position;
            //cout << manipulatorProtocolPtr->writeManipulator.stepMotorList[i].position << " ";
        }
        write();
        usleep(80000);         //点写入频率最高为14hz，默认10hz
    
    }
    ROS_INFO("LeafRobot: Now , Let's go !");
    usleep(1000000);
    while (isMoving())
    {//等待机械臂运动停止
        feedback_.actual.positions.clear();
        for (size_t i = 0; i < joint_count; i++)
        {//当前点
            feedback_.actual.positions.push_back(msg.position[i]);
        }
        feedback_.header.stamp = msg.header.stamp;
        as_.publishFeedback(feedback_);
        usleep(100000);     //10hz速率进行反馈
    }

    ROS_INFO("stop Moving");
    for( size_t tolerance = 0 ; !isArrived() && tolerance < 20 ; tolerance ++)
    {
        if (!isMoving() && tolerance % 5 == 0)
        { //机械臂停了，如果没有到达最终点，那就再写一遍
            ROS_WARN("LeafRbot : write finish goal, it means data loss happend. ");
            write();
        }
        feedback_.actual.positions.clear();
        for (size_t i = 0; i < joint_count; i++)
        {//当前点
            feedback_.actual.positions.push_back(msg.position[i]);
        }
        feedback_.header.stamp = msg.header.stamp;
        as_.publishFeedback(feedback_);
        usleep(100000);     //10hz速率进行反馈
    }
    ROS_INFO("LeafRobot: Done !");
    wayPoints.clear();
}


bool LeafRobot::isArrived()
{
    for (size_t i = 0; i < manipulatorProtocolPtr->writeManipulator.degrees; i ++)
    {
        if ( manipulatorProtocolPtr->writeManipulator.stepMotorList[i].position != manipulatorProtocolPtr->readManipulator.stepMotorList[i].position)
        {   
            //cout << "第"<< i + 1 << "号关节没有到位\n";
            return false;
        }
    }
    return true;
}

bool LeafRobot::isMoving()
{
    if ( msg_pre.position == msg.position )
    {
        return false;
    }
    return true;
}


void LeafRobot::jointStateUpdate()
{
    msg_pre = msg;
    //cout << "jointStateUpdate\n";
    for (size_t i = 0; i < manipulatorProtocolPtr->readManipulator.degrees ; i ++)
    {//获取到数据 写入到pos中
        msg.position[i] = (manipulatorProtocolPtr->readManipulator.stepMotorList[i].position - zeroPlu[i]) * PI / (2 * plu2angel[i]);
        //cout << manipulatorProtocolPtr->readManipulator.stepMotorList[i].position << " ";
    }
    msg.header.stamp = ros::Time::now();
    joint_pub_.publish(msg);
}