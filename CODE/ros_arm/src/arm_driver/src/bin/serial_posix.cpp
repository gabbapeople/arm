

#include "serial_posix.h"

SerialInterfacePOSIX::SerialInterfacePOSIX(const std::string& portName) :
    SerialInterface(portName),
    mFileDescriptor(-1) {
	mFileDescriptor = openPort(portName);
}

bool SerialInterfacePOSIX::isOpen() const {
	return mFileDescriptor != -1;
}

int SerialInterfacePOSIX::openPort(const std::string& portName) {
	int fd = open(portName.c_str(), O_RDWR | O_NOCTTY);
	if (fd == -1) {
		perror(portName.c_str());
		return -1;
	}

	return fd;
}

bool SerialInterfacePOSIX::writeBytes(const unsigned char* data, unsigned int numBytesToWrite) {
	// See http://linux.die.net/man/2/write
	assert( isOpen());
	ssize_t ret = write(mFileDescriptor, data, numBytesToWrite);
	if (ret == -1) {
		printf("Error writing. errno=%d\n", errno);
		return false;
	}
	assert(ret == numBytesToWrite);
	return true;
}

bool SerialInterfacePOSIX::readBytes(unsigned char* data, unsigned int numBytesToRead) {
	// See http://linux.die.net/man/2/read
	assert( isOpen());
	ssize_t ret = read(mFileDescriptor, data, numBytesToRead);
	if (ret == -1) {
		printf("Error reading. errno=%d\n", errno);
		return false;
	}
	assert(ret == numBytesToRead);
	return true;
}
