# Leaf_robot_ros
ros机械臂代码上位机部分
使用smartarm作为机械臂载体
ros_moviet控制，成本较低。

This code is used for maniputor in ROS. And it's my graduation design.It will took a long time(1~20s) for load trajectory in arduino(or other MCU) before maniputor moves. Because Serial is slowly. I took 0.08s to load one point to arduino, it means that arduino coulde get 12.5 points in a second. But generally, movite would give me about 30 to 200 points in a trajectory, it would take a long time. If you use use Raspberry PI's GPIO to control steppers with controllers(eg. TB6600), and communicat Raspberry PI & PC by data cable, it should be better , because cable has faster communication speeds and more reliable communication quality. But will talk me more time and more money(circuit design, udp/tcp config .etc), and I have many other things to do. So just use it. I just bought a maniputor(about 160$) from other.And remap1.6(4$),TB6600 x 3(4$ * 3) ,A4988 x 3(1$ x 3), 24V&12V DC-power(12$), a cheaper arduino2560(10$) and others. In total, it cost me about 220$.
