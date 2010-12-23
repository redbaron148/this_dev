#ifndef SP_
#define SP_

#include <stdio.h>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>


/**
 * @class SerialPort
 *
 * @author Nate Roney
 *
 * Enables simple serial port access in Linux
 *
 * Example usage:
 *
	SerialPort lsc("/dev/ttyUSB1", B38400);
	lsc.readData(buffer, 200);
	lsc.closePort();
 *
 * --OR--
 *
	SerialPort lsc();
	lsc.setPort("/dev/ttyUSB1");
	lsc.setBaudRate(B38400);
	lsc.openPort();
	lsc.readData(buffer, 200);
	lsc.closePort();
 *
 */
class SerialPort
{
public:
	
	/**
	 * Default Constructor
	 *
	 * If using this constructor, you must use setPort, setBaudRate and openPort to initiate
	 *  serial port communications.
	 */
	SerialPort();
	
	
	/**
	 * Using this constructor will open the specified port at the specified baud rate
	 * 
	 * Baud rates MUST be entered with a leading 'B' for internal conversion to speed_t
	 *  e.g. B9600 or B38400
	 *
	 * @param port : the port to open
	 * @param baudRate : the baud rate of the port
	 */
	SerialPort(char *port, int baudRate);
	
	
	/**
	 * Destructor
	 *  performs a clean disconnect
	 */
	~SerialPort();


	/**
	 * Reads data from the serial port
	 *
	 * @param readBuffer : A character array to store the data
	 * @param bytesToRead : The number of bytes to read. This must be >= the size of readBuffer
	 *
	 * @return bytesRead : -1 on error, >= 0 indicating number of bytes read on success
	 */
	int readData(char *readBuffer, int bytesToRead);
	
	
	/**
	 * Writes data to the serial port
	 *
	 * @param writeBuffer : A character array storing data to be written
	 * @param bytesTowrite : The number of bytes to write. This must be >= the size of readBuffer
	 *
	 * @return bytesRead : -1 on error, >= 0 indicating number of bytes written on success
	 */
	int writeData(const char *writeBuffer, int bytesToWrite);
	
	
	/**
	 * Opens the specified port 
	 *
	 * @return boolean
	 */
	bool openPort();
	
	
	/**
	 * Closes the currently specified port
	 */
	void closePort();
	
	
	/**
	 * Sets the baud rate. 
	 *
	 * @param baudRate : the speed at which to open the port
	 * 		Baud rates MUST be entered with a leading 'B' for internal conversion to speed_t
	 *  		e.g. B9600 or B38400
	 */
	 void setBaudRate(int baudRate);
	 
	 /**
	  * Sets the port. 
	  *
	  * @param port : the port 
	  */
	  void setPort(char *port);
	  
	  /**
	   * Queries the port status; open or closed
	   */
	  bool isOpen();

protected:

	/**
	 * Internal flag indicating port status
	 */
	bool portIsOpen;
	
	
	/**
	 * File descriptor representing the opened serial port
	 */
	int fileDescriptor;
	
	
	/**
	 * The name of the serial port
	 */
	char *portName;
	
	
	/**
	 * The speed at which to open the port
	 *
	 * Baud rates MUST be entered with a leading 'B' for internal conversion to speed_t
	 * e.g. B9600 or B38400
	 */
	int portBaudRate;
	
	
	/**
	 * Configures the port. 
	 *
	 * @param fd : a file descriptor
	 * @param baudRate : the speed at which to open the port
	 * 		Baud rates MUST be entered with a leading 'B' for internal conversion to speed_t
	 *  		e.g. B9600 or B38400
	 */
	int configurePort(int fd, int baudRate);

};

SerialPort::SerialPort()
{
	portName = NULL;
	portBaudRate = B0;
	portIsOpen = false;
}

SerialPort::SerialPort(char *port, int baudRate)
{
	portName = port;
	portBaudRate = baudRate;
	portIsOpen = openPort();	
}

SerialPort::~SerialPort()
{
	close(fileDescriptor);
}

bool SerialPort::openPort()
{	
	if(!portIsOpen && (portName != NULL) )
	{	
		fileDescriptor = open(portName, O_RDWR | O_NOCTTY);	
	}

	if(fileDescriptor == -1)
	{
		perror("openPort(): Unable to open serial port");
		return false;
	}
	else
	{
		fcntl(fileDescriptor, F_SETFL, 0);
		portIsOpen = true;
		configurePort(fileDescriptor, portBaudRate);
		return true;
	}
		
}

void SerialPort::closePort()
{
	if(portIsOpen) 
	{
		close(fileDescriptor);
		portName = NULL;
		fileDescriptor = -1;
	}
}

int SerialPort::readData(char *readBuffer, int bytesToRead)
{
	int bytesRead = -1;

	if(portIsOpen)
	{
		bytesRead = read(fileDescriptor, readBuffer, bytesToRead);
	}

	return bytesRead;
}

int SerialPort::writeData(const char *writeBuffer, int bytesToWrite)
{
	int bytesWritten = -1;

	if(portIsOpen)
	{
		bytesWritten = write(fileDescriptor, writeBuffer, bytesToWrite);
	}
	
	return bytesWritten;
}
void SerialPort::setBaudRate(int baudRate)
{
	portBaudRate = baudRate;
}

void SerialPort::setPort(char *port)
{
	portName = port;
}

int SerialPort::configurePort(int fd, int baudRate)      // configure the port
{
	struct termios port_settings;      // structure to store the port settings in

	cfsetispeed(&port_settings, baudRate);    // set baud rates
	cfsetospeed(&port_settings, baudRate);

	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;

	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
	return(fd);

}	

bool SerialPort::isOpen()
{
	return portIsOpen;
}

#endif
