#include "leaf_driver/ManipulatorProtocol.h"

ManipulatorProtocol::ManipulatorProtocol()
{
    udpHandle = new UDP();
}

ManipulatorProtocol::~ManipulatorProtocol()
{
    delete []msg.value;
    msg.value = NULL;
    delete []readManipulator.stream;
    readManipulator.stream = NULL;
    delete []writeManipulator.stream;
    writeManipulator.stream = NULL;
}

void ManipulatorProtocol::setSerialPort( string serialport_name, int baudrate )
{
    udpHandle->setSerialPort( serialport_name, baudrate );
}

void ManipulatorProtocol::manipulatorInit(int degrees)
{
    readManipulator.degrees = degrees;
    writeManipulator.degrees = degrees;
    StepMotor stepmotor = { 0, 0, 0 };
    for (size_t i = 0; i < degrees; i ++)
    {
        readManipulator.stepMotorList.push_back(stepmotor);
        writeManipulator.stepMotorList.push_back(stepmotor);
    }
    readManipulator.stream = new uint8_t[readManipulator.degrees*3*3];
    writeManipulator.stream = new uint8_t[readManipulator.degrees*3*3];
    udpHandle->setMessageValueSize(msg, degrees*3*3);

    //下层设置
    msg.start = (uint8_t)'$';
    msg.end = (uint8_t)'\n';
}

bool ManipulatorProtocol::read()
{
    msg.start = (uint8_t)'$';
    msg.end = (uint8_t)'\n';
    if (udpHandle->read(msg))
    {
        memcpy(readManipulator.stream, msg.value, msg.valueSize);
        for (size_t i = 0; i < readManipulator.degrees; i ++)
        {
            readManipulator.stepMotorList[i].position = ((readManipulator.stream[i*3*3+1] << 8 ) | readManipulator.stream[i*3*3+2]) & 0xffff;
            readManipulator.stepMotorList[i].velocity = ((readManipulator.stream[i*3*3+4] << 8 ) | readManipulator.stream[i*3*3+5]) & 0xffff;
            readManipulator.stepMotorList[i].effort = ((readManipulator.stream[i*3*3+7] << 8 ) | readManipulator.stream[i*3*3+8]) & 0xffff;
            
            readManipulator.stream[i*3*3+0] == 0xf0 ? readManipulator.stepMotorList[i].position *= -1 : readManipulator.stepMotorList[i].position *= 1;
            readManipulator.stream[i*3*3+3] == 0xf0 ? readManipulator.stepMotorList[i].velocity *= -1 : readManipulator.stepMotorList[i].velocity *= 1;
            readManipulator.stream[i*3*3+6] == 0xf0 ? readManipulator.stepMotorList[i].effort *= -1 : readManipulator.stepMotorList[i].effort *= 1;
        }
        return true;
    }
    return false;
}


int ManipulatorProtocol::write()
{
    msg.start = (uint8_t)'$';
    msg.end = (uint8_t)'\n';
    
    for (size_t i = 0; i < writeManipulator.degrees; i ++)
    {
        writeManipulator.stream[i*3*3+1] = (abs(writeManipulator.stepMotorList[i].position) >> 8) & 0xff;
        writeManipulator.stream[i*3*3+2] = abs(writeManipulator.stepMotorList[i].position) & 0xff;
        writeManipulator.stream[i*3*3+4] = (abs(writeManipulator.stepMotorList[i].velocity) >> 8) & 0xff;
        writeManipulator.stream[i*3*3+5] = abs(writeManipulator.stepMotorList[i].velocity) & 0xff;
        writeManipulator.stream[i*3*3+7] = (abs(writeManipulator.stepMotorList[i].effort) >> 8) & 0xff;
        writeManipulator.stream[i*3*3+8] = abs(writeManipulator.stepMotorList[i].effort) & 0xff;

        writeManipulator.stepMotorList[i].position < 0 ? writeManipulator.stream[i*3*3+0] = 0xf0 : writeManipulator.stream[i*3*3+0] = 0x0f;
        writeManipulator.stepMotorList[i].velocity < 0 ? writeManipulator.stream[i*3*3+3] = 0xf0 : writeManipulator.stream[i*3*3+3] = 0x0f;
        writeManipulator.stepMotorList[i].effort < 0 ? writeManipulator.stream[i*3*3+6] = 0xf0 : writeManipulator.stream[i*3*3+6] = 0x0f;
    }
    memcpy(msg.value, writeManipulator.stream, msg.valueSize);
    /*
    for(int i = 0 ; i < msg.valueSize ; i ++)
    {
        if(writeManipulator.stream[i] >= 32 && writeManipulator.stream[i] <= 126)
        {
            cout << (char)writeManipulator.stream[i] << " ";
        }
        else
        {
            cout << "0x" << hex <<(int)writeManipulator.stream[i] << " ";
        }
    }
    cout << "\n" << dec;
    */
    return udpHandle->write(msg);
}
