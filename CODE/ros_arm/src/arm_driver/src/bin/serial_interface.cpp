
#include "serial_interface.h"
#include "serial_posix.h"

SerialInterface* SerialInterface::createSerialInterface(const std::string& portName, unsigned int baudRate) {
	SerialInterface* serialInterface = NULL;

	serialInterface = new SerialInterfacePOSIX(portName);

	return serialInterface;
}

bool SerialInterface::sendTargetPositions(TargetPosition pos_0, TargetPosition pos_1, TargetPosition pos_2) {
	if (!isOpen())
		return false;

	uint8_t target_postion_0[4];
	uint8_t target_postion_1[4];
	uint8_t target_postion_2[4];

	uint8_t command[12];

	memmove(command + 0,  pos_0.b, sizeof(TargetPosition));
	memmove(command + 4,  pos_1.b, sizeof(TargetPosition));
	memmove(command + 8,  pos_2.b, sizeof(TargetPosition));
	
	if (!writeBytes(command, sizeof(command)))
		return false;

	return true;
}