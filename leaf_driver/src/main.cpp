#include "leaf_driver/LeafRobot.h"



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "leaf_node");
    LeafRobot robot("leaf_controller/follow_joint_trajectory");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time time_current = ros::Time::now();
    ros::Time  time_prev = time_current;
    double elapsed = 0;
    while (ros::ok())
    {
        time_current = ros::Time::now();
        elapsed = (time_current - time_prev).toSec();
        if(elapsed >= 0.1)
        {
            time_prev = time_current;
            robot.jointStateUpdate();
        }
        usleep(5000);
    }
    cout << "机械臂程序正在关闭..." << endl;
    robot.home();
    robot.write();
    usleep(2000000);//等待2s
    return 0;
}
