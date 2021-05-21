
#ifndef SERIAL_INTERFACE_HPP_
#define SERIAL_INTERFACE_HPP_

#include <string>
#include <cstring>
#include <stdint.h>

union TargetPosition {
    float f;
    uint8_t b[4]; 
};

class SerialInterface {
public:
	SerialInterface(const std::string& portName);
	virtual ~SerialInterface();

	virtual bool isOpen() const = 0;
    
	static SerialInterface* createSerialInterface(const std::string& portName, unsigned int baudRate); 

	bool sendTargetPositions(TargetPosition pos_0, TargetPosition pos_1, TargetPosition pos_2);

private:

	virtual bool writeBytes(const unsigned char* data, unsigned int dataSizeInBytes) = 0;
	virtual bool readBytes(unsigned char* data, unsigned int dataSizeInBytes) = 0;
};

#endif // SERIAL_INTERFACE_HPP_