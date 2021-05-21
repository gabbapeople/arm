
#ifndef SERIAL_POSIX_HPP_
#define SERIAL_POSIX_HPP_

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <assert.h>
#include <errno.h>

#include "serial_interface.h"

class SerialInterfacePOSIX: public SerialInterface {
public:
	SerialInterfacePOSIX(const std::string& portName);
	virtual ~SerialInterfacePOSIX();

	virtual bool isOpen() const;

private:
	int openPort(const std::string& portName);

	virtual bool writeBytes(const unsigned char* data,unsigned int dataSizeInBytes);
	virtual bool readBytes(unsigned char* data, unsigned int dataSizeInBytes);

	int mFileDescriptor;
};

#endif // SERIAL_POSIX_HPP_