#include "leaf_driver/UDP.h"

UDP::UDP()
{
    setBufferSize(8192);
    udata = new uint8_t [1];
    recieveStatus = 1;
}

UDP::~UDP()
{
    serialPort->close();
    //delete []serialPort;
    serialPort = NULL;
    delete []udata;
    udata = NULL;
    delete []txBuffer;
    txBuffer = NULL;
    delete []rxBuffer;
    rxBuffer = NULL;
}

//设置串口
void UDP::setSerialPort( string serialport_name, int baudrate )
{
    //串口初始化
    Timeout timeout = Timeout::simpleTimeout(2000);
    try
    {
        serialPort = new Serial(serialport_name, baudrate, timeout);
        //serialPort->open();
        
    }
    catch (exception e)
    {
        cout << "SerialPort Opened failed" << endl;
        exit(0);
    }
}


void UDP::setMessageValueSize(message& msg, int size)
{
    msg.valueSize = size;
    msg.value = new uint8_t[msg.valueSize];
}

void UDP::setStart(message& msg, uint8_t signal_)
{
    msg.start = signal_;
}

void UDP::setCrc(message& msg)
{
    int sum = 0;
    sum += msg.start;
    sum += msg.end;
    for(int i = 0 ; i < msg.valueSize ; i ++)
    {
        sum += msg.value[i];
    }
    msg.crc = (uint8_t)(sum& 0xff);
}

void UDP::setEnd(message& msg, uint8_t signal_)
{
    msg.end = signal_;
}

int UDP::getMessageValueSize(message& msg) const
{
    return msg.valueSize;
}

uint8_t UDP::getStart(message& msg) const
{
    return msg.start;
}

uint8_t UDP::getCrc(message& msg) const
{
    return msg.crc;
}

uint8_t UDP::getEnd(message& msg) const
{
    return msg.end;
}

void UDP::setBufferSize(int size)
{
    bufferSize = size;
    txBuffer = new uint8_t[bufferSize];
    rxBuffer = new uint8_t[bufferSize];
}

int UDP::getBufferSize() const
{
    return bufferSize;
}

int UDP::write(message& msg)
{
    //转换到缓存中
    msg2udp(msg);
    //写入操作
    return serialPort->write(txBuffer, 3 + msg.valueSize);;
}

bool UDP::read(message& msg)
{   
    if(simpleRecieve(msg.start, msg.end, 3 + msg.valueSize))
    {
        udp2msg(msg);
        int tempcrc = msg.start + msg.end;
        uint8_t crc = 0;
        for (size_t i = 0; i < msg.valueSize; i++)
        {
            tempcrc += msg.value[i];
        }
        crc = (uint8_t)(tempcrc & 0xff);
        if (crc == msg.crc)
        {
            //cout << "read success\n";
            return true;
        }
    }
    //printBuffer(rxBuffer, 3 + msg.valueSize);
    
    //校验位出错
    return false;
}

void UDP::flushTxBuffer()
{
    txIndex = 0;
    for(int i = 0 ; i < bufferSize ; i ++)
    {
        txBuffer[i] = 0x00;
    }
}

void UDP::flushRxBuffer()
{
    rxIndex = 0;
    for(int i = 0 ; i < bufferSize ; i ++)
    {
        rxBuffer[i] = 0x00;
    }
}

void UDP::flush()
{
    txIndex = 0;
    rxIndex = 0;
    for(int i = 0 ; i < bufferSize ; i ++)
    {
        txBuffer[i] = 0x00;
        rxBuffer[i] = 0x00;
    }
}

void UDP::printBuffer(uint8_t* buf, int size)
{
    for(int i = 0 ; i < size ; i ++)
    {
        if(buf[i] >= 32 && buf[i] <= 126)
        {
            cout << (char)buf[i] << " ";
        }
        else
        {
            cout << "0x" << hex <<(int)buf[i] << " ";
        }
    }
    cout << "\n" << dec;
}

void UDP::msg2udp(message& msg)
{
    flushTxBuffer();
    txBuffer[0] = msg.start;
    memcpy(txBuffer + 1, msg.value, msg.valueSize);
    txBuffer[ 2 + msg.valueSize] = msg.end;
    setCrc(msg);
    txBuffer[ 1 + msg.valueSize] = msg.crc;
    //printBuffer(txBuffer, 3 + msg.valueSize);
}

void UDP::udp2msg(message& msg)
{
    msg.start = rxBuffer[0];
    memcpy(msg.value, rxBuffer + 1, msg.valueSize);
    msg.crc = rxBuffer[ 1 + msg.valueSize];
    msg.end = rxBuffer[ 2 + msg.valueSize];
    flushRxBuffer();
}

int UDP::recieve(uint8_t startSignal, uint8_t endSignal, int size)
{
    flushRxBuffer();
    //cout << "size " << size << endl;
    //cout << "startSignal " << (char)startSignal << ",  endSignal " << endSignal << endl;
    if(recieveStatus == 0)
    {
        //通讯状态良好，整体接收
        serialPort->read(rxBuffer, size);
        if(rxBuffer[0] == startSignal && rxBuffer[size - 1] == endSignal)
        {
            recieveStatus = 0;
        }
        else
        {
            recieveStatus = 1;
        }
    }
    else
    {
        //通讯状态不好，逐个接收
        while( rxIndex < size)
        {
            if(serialPort->available() <= 0)
            {
                return -1;
            }
            serialPort->read(udata, 1);
            rxBuffer[rxIndex] = udata[0];
            cout << rxIndex << "  " << (char)udata[0] << endl;
            rxIndex += 1;
            if (rxBuffer[0] != startSignal)
            {
                cout << "no right start :" <<(char)startSignal <<  endl;
                flushRxBuffer();
            }
            if(rxIndex > size - 1)
            {
                if( rxBuffer[size - 1] != endSignal)
                {
                    cout << "no right end :"  <<(char)endSignal << endl;
                    flushRxBuffer();
                }
                else
                {
                    recieveStatus = 0;
                    break;
                }
            }
        }
    }
    return 0;
}

bool UDP::simpleRecieve(uint8_t startSignal, uint8_t endSignal, int size)
{   
    flushRxBuffer();
    if(serialPort->available() >= size)
    {
        serialPort->read(rxBuffer, serialPort->available());
        if(rxBuffer[0] == startSignal && rxBuffer[size - 1] == endSignal)
        {   
            return true;
        }
    }
    //cout << "\nsimpleRecieve()  error\n";
    //serialPort->flushInput();
    
    return false;
}