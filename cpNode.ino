/*==================================================================================

  cpNode - Control Point CMRI Node
    
------------------------------------------------------------------------------------
  cpNode concept and design committed on 5/31/2013 by Chuck Catania and Seth Neumann
  
  This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License. 
  To view a copy of the license, visit http://creativecommons.org/licenses/by-sa/3.0/deed.en_US
------------------------------------------------------------------------------------

  Author: Chuck Catania - Aspect Control Systems 
  System Board:  Modern Devices BBLeo (Arduino Style Leonardo)
  
  Revision History: 
    0.0d  08/24/2013  Initial template definition
          09/06/2013  Updates to reflect Proto zero testing
          09/08/2013  Successful test with USB connection to JMRI
          09/09/2013  Redefined IO maps
                      Added #ifdef for pin assignments
                      Updated Pack_Node_Inputs to reflect pin assignments
          09/10/2013  Tested with RS485 ChubbNet
          09/11/2013  ChubbNet protocol validation
          09/14/2013  Added serial read flush until ETX seen in protocol parser to deal with
                      garbled messages and noisy line.  
                      Added output byte unpacker
                      Adjusted delay timers for better performance
          09/15/2013  Added input port latching for all reads.   Bit status OR'ed into IB()
                      in outer loop.  Cleared after poll response sent.
                      Adjusted delay timers.
          09/24/2013  Fixed Ax references to PAx in output unpacking routines
          09/28/2013  Add a work around to deal with the SYN (FF) character for output bits not
                      having a DLE escape sequence.  Chubb specification does not call SYN as
                      a character which should be DLE'd.
                      Receive (input bits sent) also has DLE removed for SYN pattern.
          10/06/2013  Added a flush to ETX for respNone to outside protocol parser.  Provides better
                      processing for messages not addressed to this node.
          10/07/2013  Started IOX code. Changed cNode references to cpNode.
          10/08/2013  Added Seth's panel code processing.
          10/19/2013  Added APortMap to handle bug with the IDE pre-processor for Analog pins
          10/20/2013  Fixed I2C pin flip. 
                      IOX base configuration code tested and works with pin changes.
          10/21/2013  Assigned PA0-PA5 references to A0-A5 per Paul Badger from Modern Devices
                      Changed the node configuration variables/code blocks to #ifdef's instead of case statements.  Allows for
                      the easy selection of all related and required I/O configuration variables and code blocks.
          10/23/2013  Removed byte timeout from Read_CMRI_Byte.
          10/26/2013  Removed all delay timeouts. 
          11/03/2013  Completed the IOX implementation.
          11/07/2013  Added input latching to IOX.
          11/09/2013  Added Creative Commons license declaration.
          11/14/2013  Fixed RSMC pin assignments
                      Tested IOX32/IOX16 mix with various IO byte configs - Worked correctly.
          
  This sketch is the code template for a CMRI serial protocol node implemented in an Arduino style system board.  
  The Modern Devices BBLeo, Bare Bones Leonardo (ATMega32u4) is the target Arduino style system board. 
  
  Implements the CMRI Serial Protocol designed by Dr. Bruce Chubb and published publicly in various books, magazines, and articles.
  The physical link is RS485, 4-wire, half duplex, serial.  
  Each node has a one byte address in the range of 0-127.  
  The Host polls the CMRI nodes for data.   
  Port assignments represent the defined capability of a cpNode.
  The base node data configuration is two bytes in, two bytes out.
  All needed signal pins are connected via pin headers to the node board.  
  
  Reserved I/O pins are:
   D0 - RX  CMRI RS485 Receive
   D1 - TX  CMRI RS485 Transmit
   D2 - SDA I2C Data
   D3 - SCL I2C Clock
   
                   SERIAL PORT ASSIGNMENTS
           
  Serial        Bootloader/Debug monitor (USB Built in)
  Serial       CMRI Port (RS-485) RX(D0) and TX(D1) pins
  
/**************************************************************/
/**********        CMRI Protocol Message Format      **********/
/**************************************************************
- Initialization Message (I)  HOST to NODE
SYN SYN STX <UA> <I><NDP><dH><dL><NS><CT(1)><CT(NS)> ETX

- Poll for Data (P)  HOST to NODE
SYN SYN STX <UA> <P> ETX

- Read Data (R)  NODE To HOST  (Response to Poll)
SYN SYN STX <UA> <R><IB(1)><IB(NS)> ETX

- Transmit Data (T)  HOST to NODE
SYN SYN STX <UA> <T><OB(1)><OB(NS)> ETX
*/

/*************************************************************/
/*********            Library Header Files           *********/
/*************************************************************/
#include <SoftwareSerial.h>
// 9510 Bytes BASE_NODE
// 11096 Bytes with IOX

//#include <Servo.h>
// 11798 Bytes

/*==============================================*/
/*====    NODE CONFIGURATION PARAMETERS     ====*/
/*==============================================*/
                                                //
char nodeSN[8] = "AA0000A";                     //
                                                //
int nodeID = 20;                                //
                                                //
const int CMRINET_SPEED =  9600;                //         
                                                //      
boolean debugMode  = false,                     //
        debugMode1 = false;                     //
                                                //
/*==============================================*/
/*====    NODE CONFIGURATION PARAMETERS     ====*/
/*==============================================*/

//--------------------------------------------------------
//  Uncomment only one of the standard node configurations
//  #ifdef selects the higher order bit definitions
//-------------------------------------------------------
//#define BASE_NODE
//#define BASE_NODE_8IN8OUT
//#define BASE_NODE_16IN
#define BASE_NODE_16OUT
//#define BASE_NODE_RSMC
//#define BASE_NODE_SERVO

//----------------------
// Custom configurations
//----------------------
//#define DIRIDON_PANEL

//----------------------
// I/O Expander boards
//----------------------
//#define USE_IOX

/************************************************/
/*****        STANDARD DATA PORT MAP        *****/          
/************************************************/
const int 
// RS-485 Port
//------------
           RX      =  0,  // D0  CMRI RS485 Receive
           TX      =  1,  // D1  CMRI RS485 Transmit
           
// I2C Port
//---------
           i2C_SDA =  2,  // D2  SDA (data) 
           i2c_SCL =  3,  // D3  SCL (clock)         
                               
// Digital Pins
//-------------
           PD4     =  4,  // D4  SG1A-R  
           PD5     =  5,  // D5  SG1A-Y  
           PD6     =  6,  // D6  SG1A-G  
           PD7     =  7,  // D7  SG1B-R
           PD8     =  8,  // D8  SG1B-Y
           PD9     =  9,  // D9  SG2-R 
           PD10    = 10,  // D10 SG2-Y  
           PD11    = 11,  // D11 SG2-G
           PD12    = 12,  // D12 SG3-R
           PD13    = 13,  // D13 SG3-Y

// Analog Pins
//------------  
           PA0     = A0,  // A0 
           PA1     = A1,  // A1             
           PA2     = A2,  // A2   
           PA3     = A3,  // A3 
           PA4     = A4,  // A4  
           PA5     = A5;  // A5

// Address the bug in the Arduino IDE which does not handle re-defined analog pins.
// Redefined analog pin names do not work in digitalWrite calls and for loop indexes.
// This has been confirmed by extensive testing.
// D12 and D13 are included in the table for ease of coding bit extraction of the high byte
//-----------------------------------------------------------------------------------------           
const byte  PD12i    = 0,  // Index to actual internal pin value to keep the compiler happy
            PD13i    = 1,
            PA0i     = 2,
            PA1i     = 3,
            PA2i     = 4,
            PA3i     = 5,
            PA4i     = 6,
            PA5i     = 7;
          
// Pin name values which work for analog pins.   PDxx included for coding convenience
//-----------------------------------------------------------------------------------
const byte numAPorts = 8;
int APortMap[numAPorts] = {PD12, PD13, A0, A1, A2, A3, A4, A5};

/**********************************************************************/
/******     cpNode onboard I/O bit mapping for 16 bits           ******/
/******     Only one ioMap is active per node configuration      ******/
/**********************************************************************/
/* 
 [ BASE_NODE ]
    ----- 6 Bits IN ----- 
    IB0 6 bits       IB1 All Zero    -> HOST
    xxIIIIII         xxxxxx 
    CSnn001-CSnn006  n/a 
    ----- 10 Bits OUT ----- 
    OB1 8 bits       OB2 2 bits      -> NODE
    // OOOOOOOO         xxxxxxOO 
    // CTnn001-CTnn008  CTnn009-CTnn010

 [ BASE_NODE_8IN8OUT ]
    ----- 8 Bits IN -----  
    IB0 8 bits       IB1 All Zero    -> HOST
    IIIIIIII         xxxxxxxx 
    CSnn001-CSnn008  n/a 
    ----- 8 Bits OUT ----- 
    OB1 8 bits       OB2 All Zero    -> NODE
    OOOOOOOO         xxxxxxxx 
    CTnn001-CTnn008  n/a 
                          
 [ BASE_NODE_16IN ]
    ----- 16 Bits IN  -----  
    IB0 8 bits       IB1 8 bits      -> HOST
    IIIIIIII         IIIIIIII     
    CSnn001-CSnn008  CSnn009-CSnn016                          
    OB1 All Zero     OB2 All Zero    -> NODE
    xxxxxxxx         xxxxxxxx 
    n/a              n/a 
                          
 [ BASE_NODE_16OUT ]
    ----- 16 Bits OUT ----- 
    IB0 All Zero     IB1 All Zero    -> HOST
    xxxxxxxx         xxxxxxxx     
    n/a              n/a                           
    OB1 8 bits       OB2 8 bits      -> NODE
    OOOOOOOO         OOOOOOOO 
    CTnn001-CTnn008  CTnn009-CTnn016

 [ BASE_NODE_RSMC ]
    ----- 5 Bits IN -----   //  RSMC control on A0 as output
    IB0 5 bits       IB1 All Zero    -> HOST
    xxxIIIII         xxxxxxxx     
    CSnn001-CSnn005  n/a
    ----- 11 Bits OUT ----- 
    OB1 8 bits       OB2 3 bits      -> NODE
    OOOOOOOO         xxxxxOOO 
    CTnn001-CTnn008  CTnn009-CTnn011 
  
 [ BASE_NODE_SERVO ]          
    ----- 5 Bits IN -----   //  RSMC control on D13 as PWM output
    IB0 5 bits       IB1 All Zero    -> HOST
    xxxIIIII         xxxxxxxx     
    CSnn001-CSnn005  n/a
    ----- 11 Bits OUT -----
    OB1 8 bits       OB2 3 bits      -> NODE
    OOOOOOOO         xxxxxOOO 
    CTnn001-CTnn008  CTnn009-CTnn011
                          
 [ DIRIDON_PANEL ]          
    ----- 2 Bits IN -----  //  Seth's Diridon Control Panel
    IB0 2 bits       IB1 All Zero    -> HOST
    xxxxxxII         xxxxxxxx     
    CSnn001-CSnn002  n/a
    ----- 14 Bits OUT ----- 
    OB1 8 bits       OB2 6 bits      -> NODE
    OOOOOOOO         xxOOOOOO 
    CTnn001-CTnn008  CTnn009-CTnn014 
*/

// Protocol characters 
//--------------------
const char STX    = 0x02,
           ETX    = 0x03,
           DLE    = 0x10,
           SYN    = 0xFF,

// Read responses
//---------------
           respNone    = 0,  //  Not for this node address
           respInit    = 1,  //  "I" Message
           respPoll    = 2,  //  "P" Message
           respRead    = 3,  //  "R" Message
           respTransmit= 4,  //  "T" Message
           respErr     = 5,  //  Error occured
           respIgnore  = 6;  //  Not for this node, flush to ETX

// Debug monitor speed  
//--------------------
const int MON_SPEED      = 1200;

const int DEBOUNCE_DELAY = 10;

// Timer variables
//----------------
long previousMS = 0;
unsigned long currentMS = 0;

//---------------
// Node variables
//---------------
const char cpNODE_NDP = 'C';    // Node Definition Parameter for a cpNode - Control Point Node ** FUTURE **
boolean    CMRIX      = false;  // Set true if node will use extended protocol ** FUTURE **
int        CMRIX_flags= 0;      // Extended CMRI protocol flags ** FUTURE **

byte  UA,           // Node address, stored as UA + UA_Offset
      matchID;      // Incomming address in message to match to node UA
int   c;            // Handy character variable

const int UA_Offset = 65;   // 0x41 Per CMRI protocol spec

// Data buffers
//-------------
const byte cpNode_bufSize= 20,   //  cpNode (2) + IOXx8 (16) + 4 pad
           CMRI_bufSize  = 72,   //  Max SUSIC + 8 pad
           IO_bufsize    = 20,   //  cpNode (2) + IOXx8 (16) + 4 pad
           maxIB         = 2,    //  Max number of onboard input bytes
           maxOB         = 2;    //  Max number of onboard output bytes 
          
byte       CMRI_Buf[CMRI_bufSize],   // CMRI message buffer

           OB[IO_bufsize],  // Output bits  HOST to NODE
           IB[IO_bufsize];  // Input bits   NODE to HOST

byte       nIB,             // Total configured input bytes
           nOB;             // Total configured output bytes

/**********************************************************************/
/**********                      IOX VARIABLES               **********/
/**********************************************************************/
#ifdef USE_IOX

// If using the I/O expander, include the i2c library
//---------------------------------------------------
#include <Wire.h>

const byte num_IOX_Devices= 16,  // device = MCP28017 chip
           num_IOX_Bytes  = num_IOX_Devices,
           max_IOX        = 16,

// I2C command registers
//----------------------
           ACTIVELOW_REG_BASE  = 0x02,  // Port A, Port B = 0x02
           PULLUP_REG_BASE     = 0x0C,  // Port A, Port B = 0x0D
           GPIO_REG_BASE       = 0x12,  // Port A, Port B = 0x13
           
           SET_PORT_OUTPUT     = 0x00,
           SET_PORT_INPUT      = 0xFF,
           SET_PORT_PULLUPS    = 0xFF,
           SET_PORT_ACTIVELOW  = 0xFF;
           
                               // Board Addresses (duplicated for ease of indexing)   
const int IOX_devAddr[max_IOX] = {0x20,0x20, 0x21,0x21, 0x22,0x22, 0x23,0x23, 
                                  0x24,0x24, 0x25,0x25, 0x26,0x26, 0x27,0x27  },

// I2C port direction bytes 0 = OUTPUT  1 = INPUT  -1 = Not Assigned
//------------------------------------------------------------------
                              // Addr  0x20     0x21     0x22     0x23     0x24     0x25     0x26     0x27
                              // Port  A  B     A  B     A  B     A  B     A  B     A  B     A  B     A  B
           IOX_ioMap[max_IOX]   =   { -1,-1,   -1,-1,   -1,-1,   -1,-1,   -1,-1,   -1,-1,   -1,-1,   -1,-1 };

// Data byte mapping table created from devAddr/ioMap configuration
// Created by Configure_IOX_Ports

// [port index][0] is I2C chip address
// [port index][1] is I2C port 0=A, 1=B
//-----------------------------------------------------------------
const byte I2C_ADDR = 0,
           I2C_PORT = 1;
          
int IOX_Output_Map[max_IOX][2],
    IOX_Input_Map[max_IOX][2];
    
// Actual count of defined I2C input and output ports
//---------------------------------------------------
byte numIOX_OUT= 0,
     numIOX_IN = 0;
    
// I2C input and output buffers
//-----------------------------
int IOX_inBuf[max_IOX+2],
    IOX_outBuf[max_IOX+2];


/************************************************************************* 
***********                IOX SUPPORT ROUTINES                 ********** 
**************************************************************************
  For each defined IOX board, setup the two ports for direction.
  Input ports will have the weak pull up enabled.
  
  Packing and Unpacking routines will place data bytes sequentially in
  the IOX buffers in the order of device address.    
**************************************************************************/

//------------------------------------------------------------------------
// Collect the input/output byte assignments for the number of IOX's
// Build the tables used for processing the data bytes
//------------------------------------------------------------------------
void Configure_IOX_Ports()
{
  byte i=0,
       j,k;

// Count the number of defined ports by direction.  
// Setup the input and output IOX byte map for the CMRI buffers
//-------------------------------------------------------------
  while (i < max_IOX)    
  {
    for (j=0; j<2; j++)
     {
       k = i+j;
       switch (IOX_ioMap[k])
       {
         case  0:   // Outputs
          IOX_Output_Map[numIOX_OUT][I2C_ADDR] = IOX_devAddr[k];
          IOX_Output_Map[numIOX_OUT++][I2C_PORT] = j;
         break;
         case  1:   // Inputs
          IOX_Input_Map[numIOX_IN][I2C_ADDR] = IOX_devAddr[k];
          IOX_Input_Map[numIOX_IN++][I2C_PORT] = j;
         break;
         default: ; // No Port in slot
       } 
     }
    i=i+2;      
  } 
  
}  // CONFIGURE IOX PORTS


//-------------------------------------------------------------------------------
// For each of the defined OUTPUT bytes, set the register parameters for the chip
//-------------------------------------------------------------------------------
void Setup_IOX_Outputs()
{
  byte i;
  
  for (i=0; i<numIOX_OUT; i++)
  {      
    Wire.beginTransmission(IOX_Output_Map[i][I2C_ADDR]);  // Board Address
    Wire.write(IOX_Output_Map[i][I2C_PORT]);              // A or B Port
    Wire.write(SET_PORT_OUTPUT);                          // Command to set to Output
    Wire.endTransmission();   
  }
  
}  // SET UP IOX OUTPUTS


//--------------------------------------------------------------------------
// Unpack the bytes sent in the transmit message and output to the I2C ports
//--------------------------------------------------------------------------
void Unpack_IOX_Outputs()
{
  byte i,j,n;
      
// Set up values to assign CMRI output bytes to I2C bytes
//-------------------------------------------------------
  if (numIOX_OUT > 0)
  {
    i = nOB;      // Last onboard output byte index           
    j = 0;        // Index into stored I2C output byte
    n = 0;        // Byte transfer count           

// Transfer CMRI output bytes to the I2C buffer
//---------------------------------------------    
   while (n < numIOX_OUT) 
   {
    IOX_outBuf[j++] = CMRI_Buf[i++]; 
    n++;
   } 
    
// Write the bytes to the I2C chips
//---------------------------------    
   for (i=0; i<numIOX_OUT; i++)
    {  
      Wire.beginTransmission(IOX_Output_Map[i][I2C_ADDR]);        // Board Address
      Wire.write(GPIO_REG_BASE + IOX_Output_Map[i][I2C_PORT]);    // A or B Port
      Wire.write(IOX_outBuf[i]);                                  // Data to send
      Wire.endTransmission();  
    } 
  }
  
}  // UNPACK IOX OUTPUTS


//-------------------------------------------------------------------------------
// For each of the defined INPUT bytes, set the register parameters for the chip
//-------------------------------------------------------------------------------
void Setup_IOX_Inputs()
{
  byte i,
       addr,port;
  
  
  for (i=0; i<numIOX_IN; i++)
  {     
    addr =  IOX_Input_Map[i][I2C_ADDR];
    port =  IOX_Input_Map[i][I2C_PORT]; 
    
// Set port to Inputs
//-------------------
    Wire.beginTransmission(addr); 
    Wire.write(port);              
    Wire.write(SET_PORT_INPUT);                   
    Wire.endTransmission();    

// Set port to use weak pullups
//-----------------------------
    Wire.beginTransmission(addr);         
    Wire.write(PULLUP_REG_BASE + port);  
    Wire.write(SET_PORT_PULLUPS);                              
    Wire.endTransmission();    
    
// Set port to active LOW polarity
//--------------------------------
    Wire.beginTransmission(addr);            
    Wire.write(ACTIVELOW_REG_BASE + port); 
    Wire.write(SET_PORT_ACTIVELOW);                               
    Wire.endTransmission();   
  }
    
}  // SET UP IOX INPUTS
  

//-----------------------------------------------------------------------------------
// Read the input bytes from the I2C ports and collect them into the I2C input buffer
// The bits will be latched and cleared when sent in the poll response.
//-----------------------------------------------------------------------------------
void Pack_IOX_Inputs()
{
  byte i;
      
// Transfer the input bytes to the I2C buffer
//-------------------------------------------    
  if (numIOX_IN > 0)
  {
   for (i=0; i<numIOX_IN; i++)
    {  
      Wire.beginTransmission(IOX_Input_Map[i][I2C_ADDR]);        // Board Address
      Wire.write(GPIO_REG_BASE + IOX_Input_Map[i][I2C_PORT]);    // A or B Port
      Wire.endTransmission();  

      Wire.requestFrom(IOX_Input_Map[i][I2C_ADDR],1);            // Data to read
      IOX_inBuf[i] = IOX_inBuf[i] | Wire.read();                 // Latch the inputs
      delay(DEBOUNCE_DELAY);
    } 
  }
  
}  // PACK IOX INPUTS

#endif


/*************************************************************************/
/**********            GENERAL SUPPORT ROUTINES                 **********/
/*************************************************************************/
//-----------------------
//  Get the available ram 
//-----------------------
int freeRam()
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}  // FREE RAM


/*---------------------------------------------------------------------
  Perform the base cpNode setup at bootup
-----------------------------------------------------------------------*/
void SetUp_Node()
{

// Set the number of onboard I/O bytes
//-------------------------------------
  nIB = maxIB;    
  nOB = maxOB;      

}  // SET UP NODE


/*----------------------------------------------
  Set the Node ID 
------------------------------------------------*/
void SetNodeAddr(byte nodeAddr)
{
    
// Set node address to 127 if an invalid decimal address is passed
//----------------------------------------------------------------
  if ((nodeAddr < 0) || (nodeAddr > 127)) 
   nodeAddr = 127; 
   
  UA = nodeAddr + UA_Offset;
 
}  // SET NODE ADDR


/*************************************************************************/
/**********              DATA BIT PACK/UNPACK ROUTINES          **********/
/*************************************************************************/

/*-------------------------------------------------------------------------
  The input routines collect the bits from the digitalRead() calls and
  puts the them into the correct bytes for transmission.

  The ports are read twice with a delay between for input debounce.
  
  Two bytes are always stored.
---------------------------------------------------------------------------*/
void Pack_Node_Inputs()
{
  byte i,k,bits,
       n;
  
  for (i=0; i<2; i++)
  {
#ifdef BASE_NODE
    // IB0 6 bits     IB1 All Zero
    // 00IIIIII       00000000
    //----------------------------
        bits = 0;
        for (n=0,k=PA5i; n<6; n++) 
         bits = bits | (!digitalRead(APortMap[k--]) << (5-n));
        IB[0] = bits;
        IB[1] = 0;
#endif

#ifdef BASE_NODE_8IN8OUT
    // IB0 8 bits     IB1 All Zero
    // IIIIIIII       00000000
    //----------------------------
        bits = 0;
        for (n=0,k=PA5i; n<8; n++) 
         bits = bits | (!digitalRead(APortMap[k--]) << (7-n));
        IB[0] = bits;
        IB[1] = 0;
#endif

#ifdef BASE_NODE_16IN    
    // IB0 8 bits     IB1 8 bits
    // IIIIIIII       IIIIIIII
    //----------------------------
        bits = 0;
        for (n=0,k=PD11; n<8; n++) 
         bits = bits | (!digitalRead(k--) << (7-n));
        IB[0] = bits;
        
        bits = 0;
        for (n=0,k=PA5i; n<8; n++) 
         bits = bits | (!digitalRead(APortMap[k--]) << (7-n));
        IB[1] = bits;
#endif

#ifdef BASE_NODE_16OUT
    // IB0 All Zero   IB1 All Zero
    // 00000000       00000000
    //----------------------------
        IB[0] = 0;
        IB[1] = 0;
#endif

#ifdef BASE_NODE_RSMC
    // IB0 5 bits     IB1 All Zero
    // 000IIIII       00000000
    //----------------------------
        bits = 0;
        for (n=0,k=PA5i; n<5; n++) 
         bits = bits | (!digitalRead(APortMap[k--]) << (4-n));
        IB[0] = bits;
        IB[1] = 0;
#endif

#ifdef BASE_NODE_SERVO
    // IB0 5 bits     IB1 All Zero
    // 000IIIII       00000000
    //----------------------------
        bits = 0;
        bits = (!digitalRead(APortMap[PD12i]) << 5);
        for (n=0,k=PA4i; n<5; n++) 
         bits = bits | (!digitalRead(APortMap[k--]) << (4-n));
        IB[0] = bits;
        IB[1] = 0;
#endif

#ifdef DIRIDON_PANEL     
    // IB0 2 bits     IB1 All Zero
    // 000000II       00000000
    //----------------------------
        IB[0] = (!digitalRead(A5) << 1) | (!digitalRead(A4)); 
        IB[1] = 0;
#endif

// Wait for debounce time
//-----------------------
   delay(DEBOUNCE_DELAY);
  }

}  // PACK NODE INPUTS


/*-------------------------------------------------------------------------
  Unpack the first byte of output bit stream.  Called first by most of the
  unpacking modes.
---------------------------------------------------------------------------*/
void Unpack_Lower8()
{
  byte n,j,k,
       bits;
  
  bits = OB[0];
  for (j=0,k=PD4; k<PD12; k++) 
  {  
    n = ((bits >> j++) &  0x01);
    digitalWrite(k,n);
  };  
        
}  // UNPACK LOWER 8


/*-------------------------------------------------------------------------
  The output routines takes received bits from the ouput buffer and 
  writes them to the correct output port using digitalWrite().
  
  The correct formatting routine is based upon the value of cpNode_ioMap
---------------------------------------------------------------------------*/
void Unpack_Node_Outputs()
{
  byte i,j,k,
       n,bits;
      
// Move the received bytes to the output buffer
//---------------------------------------------
    for (i=0; i<maxOB; i++)
     OB[i] = CMRI_Buf[i];

#ifdef BASE_NODE
     // OB1 8 bits     OB2 2 bits      -> NODE
     // OOOOOOOO       xxxxxxOO 
     //----------------------------
        Unpack_Lower8(); 
        bits = OB[1]; 
        for (j=0,k=PD12i; k<PA0i; k++) 
        {  
          n = ((bits >> j++) &  0x01);
          digitalWrite(APortMap[k],n);
        }; 
#endif

#ifdef BASE_NODE_8IN8OUT
     // OB1 8 bits     OB2 All Zero    -> NODE
     // OOOOOOOO       xxxxxxxx 
     //----------------------------
        Unpack_Lower8();  
#endif

#ifdef BASE_NODE_16IN
     // OB1 All Zero   OB2 All Zero    -> NODE
     // xxxxxxxx       xxxxxxxx 
     //----------------------------
#endif

#ifdef BASE_NODE_16OUT      
     // OB1 8 bits     OB2 8 bits      -> NODE
     // OOOOOOOO       OOOOOOOO 
     //----------------------------
        Unpack_Lower8();  
        bits = OB[1]; 
        for (j=0,k=PD12i; k<PA5i; k++) 
        {  
          n = ((bits >> j++) &  0x01);
          digitalWrite(APortMap[k],n);
        };  
#endif

#ifdef BASE_NODE_RSMC     
    // OB1 8 bits       OB2 3 bits      -> NODE
    // OOOOOOOO         xxxxxOOO 
    //----------------------------
        Unpack_Lower8();
        bits = OB[1];  
        for (j=0,k=PD12i; k<PA1i; k++) 
        {  
          n = ((bits >> j++) &  0x01);
          digitalWrite(APortMap[k],n);
        };  
#endif

#ifdef BASE_NODE_SERVO
    // OB1 8 bits       OB2 3 bits      -> NODE
    // OOOOOOOO         xxxxxOOO 
    //----------------------------
        Unpack_Lower8();
        bits = OB[1];  
        for (j=0,k=PD13i; k<PA2i; k++) 
        {  
          n = ((bits >> j++) &  0x01);
          digitalWrite(APortMap[k],n);
        };  
#endif

#ifdef DIRIDON_PANEL
    // OB1 8 bits     OB2 6 bits      -> NODE
    // OOOOOOOO       xxOOOOOO 
    //----------------------------
        Unpack_Lower8();  
        bits = OB[1];  
        for (j=0,k=PD12i; k<PA4i; k++) 
        {  
          n = ((bits >> j++) &  0x01);
          digitalWrite(APortMap[k],n);
        };  
#endif
    
}  // UNPACK NODE OUTPUTS


/*************************************************************************/
/**********             CMRI SERIAL SUPPORT ROUTINES            **********/
/*************************************************************************/
/*-----------------------------------------------------------------------------------------
  Perform any initialization and setup using the initialization message
  NDP must be a "C" (cpNode) for initialization to be done
  
- Initialization Message (I)
SYN SYN STX <UA> <I><NDP><dH><dL><NS><CT(1)><CT(NS)> ETX  
-------------------------------------------------------------------------------------------*/
void Initialize_cpNode()
{

// Check if initialize message is for a cpNode
// if so, process any options
//--------------------------------------------
  if (cpNODE_NDP == CMRI_Buf[0])
   {
   } 
   
}  // INITIALIZE CP NODE


/*-----------------------------------------------------
  This function flushes the serial read until an ETX
  is seen.  This is used to ignore any inbound messages
  not addressed to the node or to re-sync the protocol
  parser for a garbled message.
-------------------------------------------------------*/
void Flush_CMRI_To_ETX()
{
  do
  {
   if (Serial.available())
    c = Serial.read();
  } while (c != ETX); 
  
}  // FLUSH CMRI TO ETX


/*--------------------------------------------------------
  Read a byte from the specified serial port.
  Return the character read 
----------------------------------------------------------*/
char Read_CMRI_Byte()
{
  while (true)
  { 
    if (Serial.available() > 0)
     return char(Serial.read());
  };
}  // READ CMRI BYTE


/*----------------------------------------------------------------------
  Read the message from the Host and determine if the message is 
  for this node.  The header portion of the message is read, 
  and if the node address does not match, the remainder of the message is
  flushed to the ETX.
  
  The data message body is processed by an appropriate message handler. 

- Initialization Packet (I)
SYN SYN STX <UA> <I> <NDP> <dH><dL> <NS> <CT(1)><CT(NS)> ETX

- Poll for Data (P)
SYN SYN STX <UA> <P> ETX

- Read Data (R)
SYN SYN STX <UA> <R> <IB(1)><IB(NS)> ETX
----------------------------------------------------------------------*/
int CMRI_Read()
{
  byte i,
       resp;
  int  inCnt;
      
  boolean reading = true,
          inData = false;

//-----------------------------------
// Check input buffer for a character
//-----------------------------------
  if (Serial.available() <= 0)
   return respNone;
  
//--------------------
// Process the message
//--------------------
  matchID = 0;
  inCnt = 0;
  
  do
  {
     c = Read_CMRI_Byte();  // read the byte
     switch( int(c) )
     {
      case STX:         // Start of message header, start parsing protocol message
//        if (debugMode) Serial.print("STX ");

// Read node address and message type
//-----------------------------------       
        matchID = Read_CMRI_Byte();  // Node Address         
//        if (debugMode) {Serial.print(matchID-UA_Offset); Serial.print(" ");}
        
      // If node ID does not match, exit and flush to ETX in outer loop
      //---------------------------------------------------------------
        if (matchID != UA) 
         {
//          if (debugMode) {Serial.print("No Match "); Serial.println(matchID); }
          resp = respIgnore;
          reading=false;
         }
       else
      {  
// Set response code based upon message type
//------------------------------------------
       c = Read_CMRI_Byte();        // Message Type
//       if (debugMode) { Serial.print(char(c)); Serial.print(" ");}

       switch( c )
       {
         case 'I':          // Initialization
           resp = respInit;
         break;
         case 'P':          // Poll
           resp = respPoll;
         break;
         case 'R':          // Read
           resp = respRead;
         break;
         case 'T':          // Write (Transmit)
           resp = respTransmit;
//           if (debugMode) { Serial.print(":"); i=Serial.available(); Serial.print(i,HEX); }
         break;
         default:           // Unknown - Error
           resp = respErr;
           reading = false;
         break;
        }

// Completed the header, go into message data mode
//------------------------------------------------
        inData = true;     
       }
      break;

      case ETX:         // End of message, read complete
//        if (debugMode) Serial.print(" ETX ");         
        reading = false;
      break;

      case DLE:         // Read the next byte regardless of value and store it
//        if (debugMode) Serial.print("DLE ");
        CMRI_Buf[inCnt++] = Read_CMRI_Byte();
//        if (debugMode) {Serial.print(byte(CMRI_Buf[inCnt-1]),HEX); Serial.print(" ");}
      break;
      
      case SYN:         // Sync character Ignore it if not reading data
        if (inData) CMRI_Buf[inCnt++] = c; 
//        if (debugMode) Serial.print("SYN ");
      break;
      
      default:  // Stuff the data character into the receive buffer
//       if (debugMode) {Serial.print("{"); Serial.print(byte(c),HEX); Serial.print("}");}
       CMRI_Buf[inCnt++] = c;
      break;       
    }    

// Check for buffer overrun and terminate the read if true
//--------------------------------------------------------     
    if (inCnt > CMRI_bufSize) 
     {
        reading = false;
        resp = respErr;
//        if (debugMode) { Serial.print("\nBuffer Overrun inCnt = "); Serial.println(inCnt); };
     }
     
  } while (reading);
  
// Null terminate the input buffer
//--------------------------------
   CMRI_Buf[inCnt] = 0;

//---------------------------------------------------------
// Match the node address in the message to the UA+65 value
// if no match, ignore message, not addessed to this node
//--------------------------------------------------------- 
//  if (debugMode) { Serial.print("\n ->inCnt = "); Serial.println(inCnt); };

  return resp;
    
}  // CMRI READ


/*----------------------------------------------------------
 This function sends the input bytes to the host in response
 to a poll message.  Bytes are moved from the IB(n) buffer
 to the transmit buffer.
 DLE characters are inserted for data values which are
 protocol characters.
 
- Read Data (R) Message
SYN SYN STX <UA> <R><IB(1)><IB(NS)> ETX

NB: Change to int if longer buffers are implemented
------------------------------------------------------------*/
void CMRI_Poll_Resp()
{
  byte i=0,
       j=0;
  
// Packet Header
//--------------
  CMRI_Buf[i++] = SYN;
  CMRI_Buf[i++] = SYN;
  CMRI_Buf[i++] = STX;

// Message Header
//---------------
  CMRI_Buf[i++] = UA;
  CMRI_Buf[i++] = 'R';

// Load the onboard input bytes into the output buffer
//----------------------------------------------------
  if (nIB > 0)
   {
     for (j=0; j < nIB; j++)
      {
       c = IB[j];  // Insert a DLE if the output byte value is a protocol character
       switch(c)  
       {
         case STX:
         case ETX:
         case DLE:
//       case SYN:   Syncs are ignored to conform to the published protocol
         CMRI_Buf[i++] = DLE;
         break;
       }
       CMRI_Buf[i++] = c;
       IB[j] = 0;   // Clear the latched inputs
      }
   }

#ifdef USE_IOX
// Load the IOX input bytes into the output buffer
//------------------------------------------------
  if (numIOX_IN > 0)
   {
     for (j=0; j < numIOX_IN; j++)
      {
       c = IOX_inBuf[j];  // Insert a DLE if the output byte value is a protocol character
       switch(c)  
       {
         case STX:
         case ETX:
         case DLE:
//       case SYN:    Syncs are ignored to conform to the published protocol
         CMRI_Buf[i++] = DLE;
         break;
       }
       CMRI_Buf[i++] = c;
       IOX_inBuf[j] = 0;   // Clear the latched inputs
      }
   }
#endif

// Add the ETX and send the complete buffer
//-----------------------------------------
  CMRI_Buf[i++] = ETX;
  for (j=0; j<i; j++)
   Serial.write(CMRI_Buf[j]);
/*
  if (debugMode)
   {
     Serial.print("Poll Response "); Serial.print("nIDB= "); Serial.print(nIB,HEX); Serial.print(" "); 
     for (j=0; j<i; j++) {Serial.print(CMRI_Buf[j],HEX); Serial.print(" "); }
     Serial.println();
   }
*/ 
}  // CMRI POLL RESP


/************************************************
*******            Setup Code          **********
*************************************************/
void setup()
{
  byte i,j;
  
  // RS-485 Port
//------------
  pinMode(RX, INPUT);          // D0  CMRI RS485 Receive    
  pinMode(TX, OUTPUT);         // D1  CMRI RS485 Transmit

// I2C Ports, actual setup done by the Wire library
//-------------------------------------------------
  pinMode(i2C_SDA, OUTPUT);    // D2  SDA (data)    
  pinMode(i2c_SCL, OUTPUT);    // D3  SCL (clock)    
  
#ifdef BASE_NODE    //----------  Base Node assignments ----------
  pinMode(PD4,  OUTPUT);       // D4  SG1A-R   
  pinMode(PD5,  OUTPUT);       // D5  SG1A-Y   
  pinMode(PD6,  OUTPUT);       // D6  SG1A-G   
  pinMode(PD7,  OUTPUT);       // D7  SG1B-R  
  pinMode(PD8,  OUTPUT);       // D8  SG1B-Y   
  pinMode(PD9,  OUTPUT);       // D9  SG2-R   
  pinMode(PD10, OUTPUT);       // D10 SG2-Y   
  pinMode(PD11, OUTPUT);       // D11 SG2-G
  pinMode(PD12, OUTPUT);       // D12 SG3-R  
  pinMode(PD13, OUTPUT);       // D13 SG3-Y   

  pinMode(A0,  INPUT_PULLUP);  // A0  TC1  Track Circuit 1   
  pinMode(A1,  INPUT_PULLUP);  // A1  TC2  Track Circuit 2   
  pinMode(A2,  INPUT_PULLUP);  // A2  OS1  OS Circuit 1   
  pinMode(A3,  INPUT_PULLUP);  // A3  SW1  Turnout throw input   
  pinMode(A4,  INPUT_PULLUP);  // A4  AUX1   
  pinMode(A5,  INPUT_PULLUP);  // A5  AUX2  
#endif

#ifdef BASE_NODE_8IN8OUT //----------  Base Node 8 IN 8 OUT ----------
  pinMode(PD4,  OUTPUT);        // D4   
  pinMode(PD5,  OUTPUT);        // D5   
  pinMode(PD6,  OUTPUT);        // D6   
  pinMode(PD7,  OUTPUT);        // D7  
  pinMode(PD8,  OUTPUT);        // D8   
  pinMode(PD9,  OUTPUT);        // D9   
  pinMode(PD10, OUTPUT);        // D10   
  pinMode(PD11, OUTPUT);        // D11

  pinMode(PD12, INPUT_PULLUP);  // D12  
  pinMode(PD13, INPUT_PULLUP);  // D13   
  pinMode(A0,   INPUT_PULLUP);  // A0 
  pinMode(A1,   INPUT_PULLUP);  // A1   
  pinMode(A2,   INPUT_PULLUP);  // A2   
  pinMode(A3,   INPUT_PULLUP);  // A3   
  pinMode(A4,   INPUT_PULLUP);  // A4   
  pinMode(A5,   INPUT_PULLUP);  // A5  
#endif

#ifdef BASE_NODE_16IN //----------  Base Node 16 IN ----------
  pinMode(PD4,  INPUT_PULLUP);  // D4   
  pinMode(PD5,  INPUT_PULLUP);  // D5   
  pinMode(PD6,  INPUT_PULLUP);  // D6   
  pinMode(PD7,  INPUT_PULLUP);  // D7  
  pinMode(PD8,  INPUT_PULLUP);  // D8   
  pinMode(PD9,  INPUT_PULLUP);  // D9   
  pinMode(PD10, INPUT_PULLUP);  // D10   
  pinMode(PD11, INPUT_PULLUP);  // D11

  pinMode(PD12, INPUT_PULLUP);  // D12  
  pinMode(PD13, INPUT_PULLUP);  // D13   
  pinMode(A0,   INPUT_PULLUP);  // A0 
  pinMode(A1,   INPUT_PULLUP);  // A1   
  pinMode(A2,   INPUT_PULLUP);  // A2   
  pinMode(A3,   INPUT_PULLUP);  // A3   
  pinMode(A4,   INPUT_PULLUP);  // A4   
  pinMode(A5,   INPUT_PULLUP);  // A5  
#endif

#ifdef BASE_NODE_16OUT //----------  Base Node 16 OUT ----------
  pinMode(PD4,  OUTPUT);  // D4   
  pinMode(PD5,  OUTPUT);  // D5   
  pinMode(PD6,  OUTPUT);  // D6   
  pinMode(PD7,  OUTPUT);  // D7  
  pinMode(PD8,  OUTPUT);  // D8   
  pinMode(PD9,  OUTPUT);  // D9   
  pinMode(PD10, OUTPUT);  // D10   
  pinMode(PD11, OUTPUT);  // D11

  pinMode(PD12, OUTPUT);  // D12  
  pinMode(PD13, OUTPUT);  // D13   
  pinMode(A0,   OUTPUT);  // A0 
  pinMode(A1,   OUTPUT);  // A1   
  pinMode(A2,   OUTPUT);  // A2   
  pinMode(A3,   OUTPUT);  // A3   
  pinMode(A4,   OUTPUT);  // A4   
  pinMode(A5,   OUTPUT);  // A5  
#endif

#ifdef BASE_NODE_RSMC    //----------  Base Node with remote stall motor ---------- 
  pinMode(PD4,  OUTPUT);       // D4  SG1A-R   
  pinMode(PD5,  OUTPUT);       // D5  SG1A-Y   
  pinMode(PD6,  OUTPUT);       // D6  SG1A-G   
  pinMode(PD7,  OUTPUT);       // D7  SG1B-R  
  pinMode(PD8,  OUTPUT);       // D8  SG1B-Y   
  pinMode(PD9,  OUTPUT);       // D9  SG2-R   
  pinMode(PD10, OUTPUT);       // D10 SG2-Y   
  pinMode(PD11, OUTPUT);       // D11 SG2-G
  pinMode(PD12, OUTPUT);       // D12 SG3-R  
  pinMode(PD13, OUTPUT);       // D13 SG3-Y   
  pinMode(A0,   OUTPUT);       // A0  SW1  Turnout throw input to RSMC
  
  pinMode(A1,  INPUT_PULLUP);  // A1  TC1  Track Circuit 1   
  pinMode(A2,  INPUT_PULLUP);  // A2  TC2  Track Circuit 2   
  pinMode(A3,  INPUT_PULLUP);  // A3  OS1  OS Circuit 1   
  pinMode(A4,  INPUT_PULLUP);  // A4  SW1  Turnout throw input to RSMC   
  pinMode(A5,  INPUT_PULLUP);  // A5  AUX1 
#endif

#ifdef BASE_NODE_SERVO    //----------  Base Node with servo ---------- 
  pinMode(PD4,  OUTPUT);       // D4  SG1A-R   
  pinMode(PD5,  OUTPUT);       // D5  SG1A-Y   
  pinMode(PD6,  OUTPUT);       // D6  SG1A-G   
  pinMode(PD7,  OUTPUT);       // D7  SG1B-R  
  pinMode(PD8,  OUTPUT);       // D8  SG1B-Y   
  pinMode(PD9,  OUTPUT);       // D9  SG2-R   
  pinMode(PD10, OUTPUT);       // D10 SG2-Y   
  pinMode(PD11, OUTPUT);       // D11 SG2-G

  pinMode(PD12, INPUT_PULLUP); // D12 SW1  Turnout throw input   

  pinMode(PD13, OUTPUT);       // D13 Servo PWM   
  pinMode(A0,   OUTPUT);       // A0  SG3-R
  pinMode(A1,   OUTPUT);       // A1  SG3-Y   

  pinMode(A2,  INPUT_PULLUP);  // A2  TC1  Track Circuit 1   
  pinMode(A3,  INPUT_PULLUP);  // A3  TC2  Track Circuit 2   
  pinMode(A4,  INPUT_PULLUP);  // A4  OS1  OS Circuit 1  
  pinMode(A5,  INPUT_PULLUP);  // A5  AUX1   
#endif

#ifdef DIRIDON_PANEL //----------  Diridon Control Panel ----------
  pinMode(PD4,  OUTPUT);       // D4  Milpitas Main OD  
  pinMode(PD5,  OUTPUT);       // D5  CP M019 OS   
  pinMode(PD6,  OUTPUT);       // D6  Berryessa OD   
  pinMode(PD7,  OUTPUT);       // D7  SW.CPM019 N 
  pinMode(PD8,  OUTPUT);       // D8  SW.CPM019 R   
  pinMode(PD9,  OUTPUT);       // D9  SG.M019 GREEN Milpitas   
  pinMode(PD10, OUTPUT);       // D10 SG.M019 YELLOW   
  pinMode(PD11, OUTPUT);       // D11 SG.M019 RED
  pinMode(PD12, OUTPUT);       // D12 SG.M019 RED lower     
  pinMode(PD13, OUTPUT);       // D13 SG.M019 GREEN Berryessa     
  pinMode(A0,   OUTPUT);       // A0  SG.M019 YELLOW 
  pinMode(A1,   OUTPUT);       // A1  SG.M019 RED       
  pinMode(A2,   OUTPUT);       // A2  Dwarf RED    
  pinMode(A3,   OUTPUT);       // A3  End Of Track Indicator

  pinMode(A4,  INPUT_PULLUP);  // A4  SW throw input   
  pinMode(A5,  INPUT_PULLUP);  // A5  Spare
#endif

// Open the monitor port if in debug mode
//---------------------------------------
  if ((debugMode) || (debugMode1))
    {
     Serial.begin(MON_SPEED); 
     while(!Serial);
    }
  else
    Serial.end();
  delay(100);
   
// Open the CMRInet port
//----------------------
  Serial.begin(CMRINET_SPEED); 
  while(!Serial);
  delay(100);
  
// Call the default initialization routine
//----------------------------------------
  SetUp_Node();
  
// Set up I/O Expander (IOX) access if present
//--------------------------------------------
#ifdef USE_IOX
  Wire.begin();
  Configure_IOX_Ports();
  Setup_IOX_Inputs();
  Setup_IOX_Outputs();
#endif

// Set the node address
//---------------------
  SetNodeAddr( nodeID );
  if ((debugMode) || (debugMode1))
   { 
    Serial.print("\nCMRI Node "); Serial.print(UA - UA_Offset); 
    Serial.print(" Serial Number "); Serial.print(nodeSN);
    Serial.print("  Memory Available ");  Serial.println(freeRam());
   } 
   
// Let the node settle down
//-------------------------
  delay(100);

}  // SETUP


/**************************************************
*******          Main Loop Code          **********
***************************************************/
void loop()
{
  
  currentMS = millis();

//----------------------------------------------
//  Check for any messages from the host
//----------------------------------------------
    switch( CMRI_Read() )
    {      

// No data recveived, ignore
//--------------------------
      case respNone:
      break;
      
// "I" Initialize Message  HOST -> NODE, set configuration parameters
//-------------------------------------------------------------------
      case respInit:
       Initialize_cpNode();
      break;
      
// "P" Poll Message     HOST -> NODE request for input data
// "R" Receive Message  NODE -> HOST send input port data to host
//---------------------------------------------------------------
      case respPoll:   
        CMRI_Poll_Resp();
      break;
      
// "T" Transmit (Write) Message  HOST -> NODE, set output bits
//------------------------------------------------------------
      case respTransmit:
        Unpack_Node_Outputs();
#ifdef USE_IOX
        Unpack_IOX_Outputs();
#endif      
      break;

// Flush input buffer to ETX for various reasons
//----------------------------------------------
      case respErr:
//        if (debugMode) Serial.println("Error - Flush to ETX");
      case respIgnore:
      default:
        Flush_CMRI_To_ETX();
      break;      
     }  
      
// Read the input bits and latch for poll response
//-----------------------------------------------
   Pack_Node_Inputs();   // Pack the onboard inputs
#ifdef USE_IOX
   Pack_IOX_Inputs();    // Pack any inputs off the I2C bus
#endif      

}  // LOOP

