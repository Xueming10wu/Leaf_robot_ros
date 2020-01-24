#ifndef MANIPULATOR_PROTOCOL_H
#define MANIPULATOR_PROTOCOL_H

#include "leaf_driver/UDP.h"

//具体应用，结构应和具体应用场景一样
struct StepMotor
{
	//步进电机
	int position; 			//位置，基本控制(节拍)
	int velocity;  			//速度，中等控制，(节拍/秒)
	int effort;				//电流，高级控制，(ps 计算机专业 知识有限) 目前不会控制 (mA)
};

struct Manipulator
{
	//机械臂
	int degrees;						//自由度，确定关节以及电机数量
	vector<StepMotor> stepMotorList;	//关节列表
	uint8_t * stream;					//数据流，用于嵌入下层通讯协议字段中
};

class ManipulatorProtocol
{
public:
	ManipulatorProtocol();
	virtual ~ManipulatorProtocol();

	void setSerialPort( string serialport_name, int baudrate );

	void manipulatorInit(int degrees);

	bool read();		//调用完毕后，从manipulator对象中读取数据
	int write();	//调用前，将数据放入manipulator对象中

	//机械臂对象
	Manipulator readManipulator;
	Manipulator writeManipulator;

	//下层通讯句柄
	UDP *udpHandle;

private:
	
	//消息
	message msg;
};

#endif // MANIPULATOR_PROTOCOL