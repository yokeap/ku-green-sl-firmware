/*
 * HF and CO gas sensors modbus
 * MODBUS must read at addr 0 with length 6
 * All of values are stored with Float - Big Endian (ABCD) format but avr is little endian
 * HF is stored at addr 0 and 1
 * CO is stored at addr 2 and 3
 * Temperature is stored at addr 4 and 5
 * 
 * MODBUS Query command is 0X 04 00 00 00 06 70 80
 */


#include <ModbusSlave.h>
#include <LMP91000.h>
#include <Wire.h>
#include <ZE07CO_Sensor.h>
#include <Adafruit_AM2315.h>

#define SLAVE_ID 1           // The Modbus slave ID, change to the ID you want to use.
#define SERIAL_BAUDRATE 9600 // Change to the baudrate you want to use for Modbus communication.
#define SERIAL_PORT Serial1   // Serial port to use for RS485 communication, change to the port you're using.
#define COAnalogPin A3    //this pin read the analog voltage from the CO sensor
#define VREF  3.3       //voltage on AREF pin

LMP91000 pStat = LMP91000();
ZE07CO_Sensor CO_Sensor(&Serial3);
Modbus slave(SERIAL_PORT, SLAVE_ID);
Adafruit_AM2315 am2315;

uint8_t analog_pins[] = {A0, A3}; 
uint8_t analog_pins_size = sizeof(analog_pins) / sizeof(analog_pins[0]); // Get the size of the analog_pins array

union fsend_t{
  float f;
  uint16_t u[4];
};

void setup() {
  pStat.standby();
  gasInitialization();
  am2315.begin();

  // Register functions to call when a certain function code is received.
  slave.cbVector[CB_READ_INPUT_REGISTERS] = readInputRegister;

  // Set the serial port and slave to the given baudrate.
    SERIAL_PORT.begin(SERIAL_BAUDRATE);
    slave.begin(SERIAL_BAUDRATE);

  // Serial2 is PC RS232 for debuging
    Serial2.begin(9600);
    Serial3.begin(9600);  //the baudrate of CO sensor is 9600
}

void loop() {

 slave.poll();
// int HFVal = (int)(pStat.getCurrent(pStat.getOutput(A0), 3.3, 10) / 0.4);
// int TempVal = (int)pStat.getTemp(pStat.getOutput(A0), 3.3, 10) * 10;
// Serial2.println(TempVal);
// delay(1000);
}

void gasInitialization(){
  pStat.disableFET();
  pStat.setGain(1);
  pStat.setRLoad(10);
  pStat.setIntRefSource();
  pStat.setIntZ(1);
  pStat.setThreeLead();
  pStat.setBias(3300);
  pStat.setPosBias();
//  pStat.setTempSensor(0);
}

// Handle the function code Read Input Registers (FC=04) and write back the values from analog input pins (input registers).
uint8_t readInputRegister(uint8_t fc, uint16_t address, uint16_t length)
{
    // Check if the requested addresses exist in the array
    if (address != 0 || length < 3)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

//    union fsend_t HFVal, COVal, TempVal;

    int HFVal = (int)(pStat.getCurrent(pStat.getOutput(A0), 3.3, 10) / 0.4);
    int COVal = (int)CO_Sensor.uartReadPPM();
//    int TempVal = (int)pStat.getTemp(pStat.getOutput(A0), 3.3, 10) * 10;
    int TempVal = (int)am2315.readTemperature() * 100;

    slave.writeRegisterToBuffer(address, HFVal);
    slave.writeRegisterToBuffer(address + 1, COVal);
    slave.writeRegisterToBuffer(address + 2, TempVal);
    
//    slave.writeRegisterToBuffer(address, HFVal.u[1]);
//    slave.writeRegisterToBuffer(address + 1, HFVal.u[0]);
//
//    slave.writeRegisterToBuffer(address + 2, COVal.u[1]);
//    slave.writeRegisterToBuffer(address + 3, COVal.u[0]);
//
//    slave.writeRegisterToBuffer(address + 4, TempVal.u[1]);
//    slave.writeRegisterToBuffer(address + 5, TempVal.u[0]);

    return STATUS_OK;
}

//void writeFloatValToRegister(uint16_t address, float fltVal){
//      union fsend_t floatConvert;
//      floatConvert.f = fltVal;
//      slave.writeRegisterToBuffer(address, floatConvert.u[1]);
//      slave.writeRegisterToBuffer(address + 1, floatConvert.u[0]);
//}
