/**
 * @file arduino.ino
 *
 * @brief Arduino with PXR Temperature Controller sample code
 *
 * This program configures a Arduino as a RS485/Modubus Single Master and
 * communicate with Temperature Controller PXR for temp control. The labview in
 * PC acquire the signal data from Arduino using a simple protocol via UART
 * communication.
 * Arduino Due is required for its dual UART interfaces.
 *
 * Target:         Arduino Due
 * Tool chain:     Arduino IDE 1.8.2
 * Command Line:   None
 *
 * @author  shuwen (shuwen.ou@thermofisher.com)
 * @version 1.0
 * @date    2017-09-20
 * @bug     None
 *
 * Revision history:
 * $Log:$
 *
 */
#include <DueTimer.h>      ///< Timer Library to work with Arduino DUE

unsigned char sensorValue = 0;
unsigned char crcHigh;
unsigned char crcLow;
unsigned char pxr_cmd[8] = {0x04,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
float value;
float total_value;
unsigned int n;

float SV_upper = 30;
float SV_lower = 28;
float pxr_sv_data;
float pxr_pv_data;

unsigned char changeTime_low = 1;
unsigned char changeTime_up = 1;
unsigned int completeTime;
unsigned char initData;
unsigned char changeFlag,waitFlag,initFlag;

unsigned int pxr_read_reg(unsigned char id, unsigned char fun, unsigned int startAdress, unsigned char registerCounter);
unsigned char pxr_write_reg(unsigned char id, unsigned char fun, unsigned int starAdress, unsigned int registerState);
void pxr_init(float tem);
void pxr_temp_ctrl(void);
void read_cmd(void);
void handler2(void);

/**
 * @brief setup
 *
 * put your setup code here, to run once.
 *
 * @return void
 */
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  while (!Serial1)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pxr_init(30);

  Timer4.start();

  /*
   * To call a function handler2 one times a second.
   */
  Timer4.attachInterrupt(handler2).setFrequency(1).start();
}

/**
 * @brief loop
 *
 * put your main code here, to run repeatedly
 *
 * @return void
 */
void loop() {
  static unsigned int m;
  int data = analogRead(sensorValue);
  float vol = data*3.3*0.977;
  float pressure = (vol/101-4)/16*500;
  total_value += pressure;
  if (++n>=300) {
    n=0;
    pressure=total_value/300;
    value=pressure;
    total_value=0;
  }

  if (++m>60000) {
    m = 0;
    pxr_temp_ctrl();
  }

  if (initFlag) {
    initFlag=0;
    pxr_init(initData);
  }
}

/**
 * @brief read_cmd
 *
 * read command from PC serial port.
 *
 * @return void
 */
void read_cmd() {
  unsigned char cmd;
  unsigned char data;

  cmd = Serial.read();
  data = Serial.read();
  switch (cmd) {
    case 0x01:             //Set SV
      initFlag=1;
      initData=data;
      break;
    case 0x02:             //Set SV_upper
      SV_upper=data;
      break;
    case 0x03:              //Set SV_lower
      SV_lower=data;
      break;
    case 0x04:       //Set the time on upper temperature
      changeTime_up=data;
      break;
    case 0x05:            //Set the time on lower temperature
      changeTime_low=data;
      break;
    default:break;
  }
}

/**
 * @brief pxr_temp_ctrl
 *
 * read and process PXR data for temperature control.
 *
 * @return void
 */
void pxr_temp_ctrl() {
  unsigned int Temp = 0;
  Temp = pxr_read_reg(0x02, 0x04, 0x0000, 0x01);
  pxr_pv_data = (float)(Temp)/100*5;
  Temp = pxr_read_reg(0x02, 0x04, 0x0001, 0x01);
  pxr_sv_data = (float)(Temp)/100*5;

  if (pxr_sv_data<=pxr_pv_data&&pxr_sv_data==SV_upper&&waitFlag==0) {
    changeFlag=1;
    waitFlag=1;
  }
  if (pxr_sv_data>=pxr_pv_data&&pxr_sv_data==SV_lower&&waitFlag==0) {
    changeFlag=2;
    waitFlag=1;
  }
  if (changeFlag==3) {
    while (!(pxr_write_reg(0x02, 0x06, 0x0002, 20*SV_lower)))
      ;
    changeFlag=0;
    waitFlag=0;
  }
  if (changeFlag==4) {
    while (!(pxr_write_reg(0x02, 0x06, 0x0002, 20*SV_upper)))
      ;
    changeFlag=0;
    waitFlag=0;
  }
}

/**
 * @brief handler2
 *
 * ISR for timer4 interrupt.
 *
 * @return void
 */
void handler2() {
  Serial.println(value);
  if(Serial.available()>0) {
    read_cmd();
  }
  if (changeFlag==1&&waitFlag==1) {
    if(++completeTime >= changeTime_up*60) {
      completeTime = 0;
      changeFlag = 3;
    }
  }
  if (changeFlag==2 && waitFlag==1) {
    if (++completeTime>=changeTime_low*60) {
      completeTime=0;
      changeFlag=4;
    }
  }
}

/**
 * @brief crc16
 *
 * Implements the 16-bit cyclic redundancy check.
 *
 * @return void
 */
void crc16(unsigned char *ptr, unsigned int len) {
  unsigned int wcrc = 0xFFFF;
  unsigned char temp;
  int i = 0,j = 0;

  for (i=0; i<len; i++) {
    temp = *ptr&0xFF;
    ptr++;
    wcrc ^= temp;
    for (j=0; j<8; j++) {
      if (wcrc&0x0001) {
        wcrc >>= 1;
        wcrc ^= 0xA001;
      }
      else {
        wcrc >>= 1;
      }
    }
  }

  temp = wcrc;
  crcHigh = wcrc;
  crcLow = wcrc>>8;
}

/**
 * @brief pxr_write_reg
 *
 * write register of PXR temperature controller via Modubus protocol.
 *
 * @return unsigned char
 */
unsigned char pxr_write_reg(unsigned char id, unsigned char func, unsigned int starAdress, unsigned int registerState)
{
  unsigned char ii;
  unsigned char pxr_get_data[8];

  pxr_cmd[0] = id;  // station no.
  pxr_cmd[1] = func; // function code
  pxr_cmd[2] = (unsigned char)(starAdress>>8); // start address high byte to 0
  pxr_cmd[3] = (unsigned char)(starAdress&0x00ff); // start address low byte
  pxr_cmd[4] = (unsigned char)(registerState>>8); // end address high byte to 0
  pxr_cmd[5] = (unsigned char)(registerState&0x00ff); // end address low byte
  crc16(pxr_cmd,6);
  pxr_cmd[6] = crcHigh; // crc16 high byte
  pxr_cmd[7] = crcLow; // crc16 low byte
  for (ii=0; ii<8; ii++) {
    Serial1.write(pxr_cmd[ii]);
  }
  delay(100);

  /*
   * pxr command reasponse check.
   */
  ii=0;
  if (Serial1.available()) {
    while (Serial1.available()) {
      pxr_get_data[ii]=Serial1.read();
      ii++;
      if (ii>=8) {
        Serial1.flush();
        break;
      }
    }
    crc16(pxr_get_data,6);
    if ((pxr_get_data[6]==crcHigh) && (pxr_get_data[7]==crcLow)) {
      return 1;
    }
  }
  else {
    return 0;
  }
}

/**
 * @brief pxr_read_reg
 *
 * Read register of PXR temperature controller via Modubus.
 *
 * @return unsigned int register value
 */
unsigned int pxr_read_reg(unsigned char id, unsigned char func, unsigned int startAdress, unsigned char registerCounter)
{
  unsigned char ii=0;
  unsigned char dataLength = (registerCounter<<1)+5;
  unsigned char pxr_get_data[dataLength];

  pxr_cmd[0] = id;  // station no.
  pxr_cmd[1] = func; // func code
  pxr_cmd[2] = startAdress>>8; // start address high byte to be 0
  pxr_cmd[3] = startAdress&0x00ff; // start address low byte
  pxr_cmd[4] = 0x00; // end address high byte to be 0
  pxr_cmd[5] = registerCounter; // end address low byte
  crc16(pxr_cmd, 6);
  pxr_cmd[6] = crcHigh; // crc16 high byte
  pxr_cmd[7] = crcLow; // crc16 low byte
  for (ii=0;ii<8;ii++) {
    Serial1.write(pxr_cmd[ii]);
  }
  delay(100);

  ii = 0;
  if (Serial1.available()) {
    while (Serial1.available()) {
      pxr_get_data[ii]=Serial1.read();
      ii++;
      if (ii>=dataLength) {
        Serial1.flush();
        break;
      }
    }
    crc16(pxr_get_data,dataLength-2);
    if ((pxr_get_data[dataLength-2]==crcHigh) && (pxr_get_data[dataLength-1]==crcLow)) {
      return ((pxr_get_data[3]<<8)+pxr_get_data[4]);
    }
  }
  else {
    return 0;
  }
}

/**
 * @brief pxr_init
 *
 * PXR temperature controller init
 *
 * @return void
 */
void pxr_init(float tem) {
  pxr_write_reg(0x02, 0x06, 0x0002, tem*20);
}
