
/*********************************************************************
Based of nrf24l01 code from http://www.elecfreaks.com
And Se8r01 code supplied by supplier,
And the final piece from this post:http://bbs.ai-thinker.com/forum.php?mod=viewthread&tid=407 
Cant read the language but if you can say thank you from me!!
All the pieces put together by me, Stefan (f2k) 151215                          
*********************************************************************/
 //This code uses simple software spi so you can change these
//to whatever suits you best. 
#include "API.h"
#define CEq       8
// CE_BIT:   Digital Input     Chip Enable Activates RX or TX mode
#define CSNq      9
// CSN BIT:  Digital Input     SPI Chip Select
#define SCKq      10
// SCK BIT:  Digital Input     SPI Clock
#define MOSIq     11
// MOSI BIT: Digital Input     SPI Slave Data Input
#define MISOq     12
// MISO BIT: Digital Output    SPI Slave Data Output, with tri-state option
#define IRQq      13
// IRQ BIT:  Digital Output    Maskable interrupt pin

byte SE8R01_DR_2M=0;  //choose 1 of these to set the speed
byte SE8R01_DR_1M=0;
byte SE8R01_DR_500K=1;


#define ADR_WIDTH    4   // 4 unsigned chars TX(RX) address width
#define PLOAD_WIDTH  32  // 32 unsigned chars TX payload

byte pload_width_now=0;
byte newdata=0;
byte gtemp[5];
char signal_lv=0;
byte pip=0;
unsigned char status =0;



unsigned char ADDRESS2[1]= {0xb1};	
unsigned char ADDRESS3[1]= {0xb2};	
unsigned char ADDRESS4[1]= {0xb3};		
unsigned char ADDRESS5[1]= {0xb4};	


unsigned char ADDRESS1[ADR_WIDTH]  = 
{
  0xb0,0x43,0x10,0x10
}; // Define a static TX address

//***************************************************


unsigned char ADDRESS0[ADR_WIDTH]  = 
{
  0x34,0x43,0x10,0x10
}; // Define a static TX address

unsigned char rx_buf[PLOAD_WIDTH]= {0};
unsigned char tx_buf[PLOAD_WIDTH]= {0};
//***************************************************
 
struct dataStruct{
  unsigned long counter;
  byte rt;
}myData_pip5;

struct dataStruct1{
  unsigned long counter;
  byte rt;
}myData_pip4;







//**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void)
{
  digitalWrite(IRQq, 0);
  digitalWrite(CEq, 0);			// chip enable
  digitalWrite(CSNq, 1);                 // Spi disable	
}

/**************************************************
 * Function: SPI_RW();
 * 
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 **************************************************/
unsigned char SPI_RW(unsigned char Byte)
{
  unsigned char i;
  for(i=0;i<8;i++)                      // output 8-bit
  {
    if(Byte&0x80)
    {
      digitalWrite(MOSIq, 1);    // output 'unsigned char', MSB to MOSI
    }
    else
    {
      digitalWrite(MOSIq, 0);
    }
    digitalWrite(SCKq, 1);                      // Set SCK high..
    Byte <<= 1;                         // shift next bit into MSB..
    if(digitalRead(MISOq) == 1)
    {
      Byte |= 1;       	                // capture current MISO bit
    }
    digitalWrite(SCKq, 0);         	// ..then set SCK low again
  }
  return(Byte);           	        // return read unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
/**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
  unsigned char status;

  digitalWrite(CSNq, 0);                   // CSN low, init SPI transaction
  status = SPI_RW(reg);                   // select register
  SPI_RW(value);                          // ..and write value to it..
  digitalWrite(CSNq, 1);                   // CSN high again

  return(status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 * 
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
/**************************************************/
unsigned char SPI_Read(unsigned char reg)
{
  unsigned char reg_val;

  digitalWrite(CSNq, 0);           // CSN low, initialize SPI communication...
  SPI_RW(reg);                   // Select register to read from..
  reg_val = SPI_RW(0);           // ..then read register value
  digitalWrite(CSNq, 1);          // CSN high, terminate SPI communication

  return(reg_val);               // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 * 
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  digitalWrite(CSNq, 0);                  // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);       	    // Select register to write to and read status unsigned char

  for(i=0;i<bytes;i++)
  {
    pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read unsigned char from nRF24L01
  }

  digitalWrite(CSNq, 1);                   // Set CSN high again

  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  digitalWrite(CSNq, 0);                   // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
  {
    SPI_RW(*pBuf++);
  }
  digitalWrite(CSNq, 1);                  // Set CSN high again
  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/


void rf_switch_bank(unsigned char bankindex)
{
    unsigned char temp0,temp1;
    temp1 = bankindex;

    temp0 = SPI_RW(iRF_BANK0_STATUS);

    if((temp0&0x80)!=temp1)
    {
        SPI_RW_Reg(iRF_CMD_ACTIVATE,0x53);
    }
}




void SE8R01_Calibration()
{
        unsigned char temp[5];
        rf_switch_bank(iBANK0);
        temp[0]=0x03;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG,temp, 1);

        temp[0]=0x32;

        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_CH, temp,1);



if (SE8R01_DR_2M==1)
  {temp[0]=0x48;}
else if (SE8R01_DR_1M==1)
  {temp[0]=0x40;}
else  
  {temp[0]=0x68;}   
  
  SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,temp,1);
  temp[0]=0x77;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_PRE_GURD, temp,1);
  
        rf_switch_bank(iBANK1);
        temp[0]=0x40;
        temp[1]=0x00;
        temp[2]=0x10;
if (SE8R01_DR_2M==1)
          {temp[3]=0xE6;}
else
    {temp[3]=0xE4;}

        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, temp, 4);

        temp[0]=0x20;
        temp[1]=0x08;
        temp[2]=0x50;
        temp[3]=0x40;
        temp[4]=0x50;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, temp, 5);

        temp[0]=0x00;
        temp[1]=0x00;
if (SE8R01_DR_2M==1)
       { temp[2]=0x1E;}
else
   { temp[2]=0x1F;}

        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_IF_FREQ, temp, 3);
       
if (SE8R01_DR_2M==1)
       { temp[0]=0x29;}
else
 { temp[0]=0x14;}

        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, temp, 1);

        temp[0]=0x00;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_LOW,temp,1);

        temp[0]=0x7F;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_HI,temp,1);

        temp[0]=0x02;
        temp[1]=0xC1;
        temp[2]=0xEB;            
        temp[3]=0x1C;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, temp,4);
//--
        temp[0]=0x97;
        temp[1]=0x64;
        temp[2]=0x00;
        temp[3]=0x81;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, temp, 4);
        rf_switch_bank(iBANK0);
        
        digitalWrite(CEq, 1); 
        delayMicroseconds(30);
        digitalWrite(CEq, 0);  

        delay(50);                            // delay 50ms waitting for calibaration.

        digitalWrite(CEq, 1); 
        delayMicroseconds(30);
        digitalWrite(CEq, 0); 

        delay(50);                            // delay 50ms waitting for calibaration.
}


void SE8R01_Analog_Init()           //SE8R01 初始化
{    


        gtemp[0]=0x28;
        gtemp[1]=0x32;
        gtemp[2]=0x80;
        gtemp[3]=0x90;
        gtemp[4]=0x00;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_SETUP_VALUE, gtemp, 5);
        delay(2);
  
  unsigned char temp[5];   
 
  rf_switch_bank(iBANK1);
   
        temp[0]=0x40;
        temp[1]=0x01;               
        temp[2]=0x30;               
if (SE8R01_DR_2M==1)
       { temp[3]=0xE2; }              
else
 { temp[3]=0xE0;}
   
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, temp,4);

        temp[0]=0x29;
        temp[1]=0x89;
        temp[2]=0x55;                     
        temp[3]=0x40;
        temp[4]=0x50;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, temp,5);

if (SE8R01_DR_2M==1)
       { temp[0]=0x29;}
else
 { temp[0]=0x14;}
         
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, temp,1);

    temp[0]=0x55;
    temp[1]=0xC2;
    temp[2]=0x09;
    temp[3]=0xAC;  
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RX_CTRL,temp,4);

    temp[0]=0x00;
    temp[1]=0x14;
    temp[2]=0x08;   
    temp[3]=0x29;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FAGC_CTRL_1, temp,4);

    temp[0]=0x02;
    temp[1]=0xC1;
    temp[2]=0xCB;  
    temp[3]=0x1C;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, temp,4);

    temp[0]=0x97;
    temp[1]=0x64;
    temp[2]=0x00;
    temp[3]=0x01;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, temp,4);
    
        gtemp[0]=0x2A;
        gtemp[1]=0x04;
        gtemp[2]=0x00;
        gtemp[3]=0x7D;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_TEST_PKDET, gtemp, 4);

         rf_switch_bank(iBANK0);
}

void SE8R01_Init()  
{
  unsigned char temp[5];
        SE8R01_Calibration();   
        SE8R01_Analog_Init();   
         

  
if (SE8R01_DR_2M==1)
{  temp[0]=B01001111; }     //2MHz,+5dbm
else if  (SE8R01_DR_1M==1)
{  temp[0]=B01000111;  }     //1MHz,+5dbm
else
  {temp[0]=B01101111;  }     //500K,+5dbm

SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,temp,1);



        SPI_RW_Reg(WRITE_REG|iRF_BANK0_EN_AA, B00111111);          //enable auto acc on pip 1
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_EN_RXADDR, B00111111);      //enable pip 1
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_SETUP_AW, 0x02);  
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_RF_CH, 40);

  SPI_Write_Buf(WRITE_REG + TX_ADDR, ADDRESS0, ADR_WIDTH);    	
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, ADDRESS0, ADR_WIDTH); 
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P1, ADDRESS1, ADR_WIDTH); 
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P2, ADDRESS2, 1); 
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P3, ADDRESS3, 1); 
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P4, ADDRESS4, 1); 
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P5, ADDRESS5, 1); 
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, PLOAD_WIDTH); 
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_CONFIG, 0x3f); 
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_DYNPD, B00111111);          // enable dynamic payload length data
  SPI_RW_Reg(WRITE_REG|iRF_BANK0_FEATURE, 0x07);        // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack
digitalWrite(CEq, 1); 


}




void setup() 
{
  
   pinMode(CEq,  OUTPUT);
  pinMode(SCKq, OUTPUT);
  pinMode(CSNq, OUTPUT);
  pinMode(MOSIq,  OUTPUT);
  pinMode(MISOq, INPUT);
  pinMode(IRQq, INPUT);

Serial.begin(115200);
  init_io();                        // Initialize IO port
  SPI_RW_Reg(FLUSH_RX,0); 
  unsigned char status=SPI_Read(STATUS);
  Serial.print("status = ");    
  Serial.println(status,HEX);     
  Serial.println("*******************Radio starting*****************");

 SE8R01_Init();

}
void loop() 
{
  for(;;)
  {
 
  if(digitalRead(IRQq)==LOW)
  {
    delayMicroseconds(240);
    signal_lv=SPI_Read(iRF_BANK0_RPD);
    status = SPI_Read(STATUS);
    
    if(status&STA_MARK_RX)                                                 // if receive data ready (TX_DS) interrupt
    {   
      
      pip= (status&B00001110)>>1;
      pload_width_now=SPI_Read(iRF_CMD_R_RX_PL_WID);
      SPI_Read_Buf(RD_RX_PLOAD, rx_buf,pload_width_now);             // read playload to rx_buf
      SPI_RW_Reg(FLUSH_RX,0); 
  //    print_pip();
      newdata=1;
      }
   
    SPI_RW_Reg(WRITE_REG+STATUS,status);       // clear RX_DR or TX_DS or MAX_RT interrupt flag
  
  }
   
    
  
  if(newdata==1)
  {
  newdata=0;
  if(pip==0)
  {
     unsigned long T_counter=0;
T_counter = (unsigned long)rx_buf[3] << 24;
T_counter += (unsigned long)rx_buf[2] << 16;
T_counter += (unsigned long)rx_buf[1] << 8;
T_counter += (unsigned long)rx_buf[0];

 Serial.print(" transmission counter: ");
 Serial.print(T_counter);
 
     Serial.print(" rt packets: ");
   Serial.print(rx_buf[4]&B00001111);
  }
  
  
  if(pip==1)
  {
  
  
  }
  
  
  else if(pip==2)
  {
  
  }
  
  else if(pip==3)
  {
   
  } 
   
   
  
 else if(pip==4)
  {
 memcpy(&myData_pip4, rx_buf, sizeof(myData_pip4));
 Serial.print(" transmission counter: ");
 Serial.print(myData_pip4.counter);
 Serial.print(" rt packets: ");
 Serial.print(myData_pip4.rt&B00001111);

    }
    
  
    else if(pip==5)
  {

 memcpy(&myData_pip5, rx_buf, sizeof(myData_pip5));
 Serial.print(" transmission counter: ");
 Serial.print(myData_pip5.counter);
 Serial.print(" rt packets: ");
 Serial.print(myData_pip5.rt&B00001111);

  }
  
  
  Serial.println("");
  
  }
  }
  
}


 void print_pip()
 {
  if(pip>5)
   {
   Serial.println("NO Data"); 
   }
   else{
   Serial.print(" SS= ");
   Serial.print(signal_lv,DEC);
   Serial.print("  pip: ");    
   Serial.print(pip);
   Serial.print(" ");
   Serial.print("rx buff with= ");
   Serial.print(pload_width_now);
   Serial.print(" data= ");
      for(byte i=0; i<pload_width_now; i++)
      {
          Serial.print(" ");
          Serial.print(rx_buf[i]);                              // print rx_buf
      }
     Serial.print("  ");
 }}
 
 
