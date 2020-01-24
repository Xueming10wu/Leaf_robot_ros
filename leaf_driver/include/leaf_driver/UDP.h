#ifndef UDP_H
#define UDP_H

#include "leaf_driver/common.h"
struct message
{
    int valueSize;
    uint8_t start;
    uint8_t * value;
    uint8_t crc;
    uint8_t end;
};

class UDP
{
public:
    //构造函数
	UDP();

	virtual ~UDP();

    //设置串口
    void setSerialPort( string serialport_name, int baudrate );

    //message
    void setMessageValueSize(message & msg, int size);
    void setStart(message & msg, uint8_t signal_);
    void setCrc(message & msg);
    void setEnd(message & msg, uint8_t signal_);
    int getMessageValueSize(message & msg) const;
    uint8_t getStart(message & msg) const;
    uint8_t getCrc(message & msg) const;
    uint8_t getEnd(message & msg) const;

	//缓存数组最大长度
    void setBufferSize(int size);
    int getBufferSize() const;

    int write( message & msg );    
    bool read( message & msg );    

	//串口
    Serial * serialPort;

private:

    //清理UDP缓存
	void flushTxBuffer();
	void flushRxBuffer();
	void flush();

	void printBuffer(uint8_t * buf, int size);

    //接收报文数据，包括接收数据和接收反馈,并写入rxBuffer中。
    //需要 起始界符 末尾界符 总长度
    int recieve(uint8_t startSignal, uint8_t endSignal, int size);
    bool simpleRecieve(uint8_t startSignal, uint8_t endSignal, int size);

    void msg2udp( message & msg );//msg -> txBuffer  发送数据时使用
    void udp2msg( message & msg );//rxBuffer -> msg  接收数据时使用

	//缓存数组最大长度
	int bufferSize;

	//UDP发送缓存
	uint8_t * txBuffer;

	//UDP接收缓存
	uint8_t * rxBuffer;

	//记录UDP缓存区中已用的位数
	int rxIndex;
	int txIndex;

	//记录数据传输过程中通讯状态,为recieve函数提供服务
	// 0 良好  1 不好
    int recieveStatus;
    uint8_t * udata;

};

#endif // UDP