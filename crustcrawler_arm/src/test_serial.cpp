#include "SerialPort.cpp"

int main()
{
  SerialPort lsc("/dev/ttyUSB0",B115200);
  
  lsc.writeData("I 5 180;",10);
  
  lsc.closePort();
}
