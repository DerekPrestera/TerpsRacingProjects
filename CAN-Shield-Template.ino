/*
 * CAN Shield Sender v1.0
 * 
 * This is the program that runs on an Arduino with the Terps Racing
 * CAN shield. It communicates with the MCP2515 and MCP3208 to
 * communicate over CAN and read the ADC values respectively. It sends
 * The data it receives from the ADC over the CAN bus.
 * 
 *  Derek Prestera, 5/27/2021
 */
#include <SPI.h>

// Define all the pin numbers for the MCP2515 CAN controller
#define MCP2515_RESET_PIN   5  // reset input pin
#define MCP2515_CS_PIN      6  // chip select pin
#define MCP2515_INT_PIN     9  // interrupt flag - for this shield specifically
#define MCP2515_RX0BF_PIN   8  // receive buffer 0 full flag
#define MCP2515_RX1BF_PIN   7  // receive buffer 1 full flag
#define MCP2515_TX0RTS_PIN  4  // transmit buffer 0 request to send
#define MCP2515_TX1RTS_PIN  3  // transmit buffer 1 request to send
#define MCP2515_TX2RTS_PIN  2  // transmit buffer 2 request to send

// Define the MCP2515 instruction bytes
#define MCP2515_RESET_INSTR       0xc0
#define MCP2515_READ_INSTR        0x03
#define MCP2515_READ_RX_BUF_INSTR(n,m) ((0x90) + (0B ## (n ## (m ## 0))))
#define MCP2515_WRITE_INSTR       0x02
#define MCP2515_LOAD_TX_BUF_INSTR(a,b,c) ((0x40) + (0B ## (a ## (b ## c))))
#define MCP2515_RTS_INSTR(n)      ((0x80) + (1 << n))
#define MCP2515_READ_STATUS_INSTR 0xa0
#define MCP2515_RX_STATUS_INSTR   0xb0
#define MCP2515_BIT_MODIFY_INSTR  0x05

// mcp2515 registers - I just went down each column of the chart in the data sheet
#define MCP2515_RXF0SIDH_REG    0x00    // Receive filter 0 short ID high
#define MCP2515_RXF0SIDL_REG    0x01    // Receive filter 0 short ID low
#define MCP2515_RXF0EID8_REG    0x02    // Receive filter 0 extended ID 8
#define MCP2515_RXF0EID0_REG    0x03    // Receive filter 0 extended ID 0

#define MCP2515_RXF1SIDH_REG    0x04    // Receive filter 1 short ID high
#define MCP2515_RXF1SIDL_REG    0x05    // Receive filter 1 short ID low
#define MCP2515_RXF1EID8_REG    0x06    // Receive filter 1 extended ID 8
#define MCP2515_RXF1EID0_REG    0x07    // Receive filter 1 extended ID 0

#define MCP2515_RXF2SIDH_REG    0x08    // Receive filter 2 short ID high
#define MCP2515_RXF2SIDL_REG    0x09    // Receive filter 2 short ID low
#define MCP2515_RXF2EID8_REG    0x0a    // Receive filter 2 extended ID 8
#define MCP2515_RXF2EID0_REG    0x0b    // Receive filter 2 extended ID 0

#define MCP2515_BFPCTRL_REG     0x0c    // Buffer Flag Pin Control
#define MCP2515_TXRTSCTRL_REG   0x0d    // TX Request To Send Control

#define MCP2515_CANSTAT_REG     0x0e    // CAN status
#define MCP2515_CANCTRL_REG     0x0f    // CAN control

#define MCP2515_RXF3SIDH_REG    0x10    // Receive filter 3 short ID high
#define MCP2515_RXF3SIDL_REG    0x11    // Receive filter 3 short ID low
#define MCP2515_RXF3EID8_REG    0x12    // Receive filter 3 extended ID 8
#define MCP2515_RXF3EID0_REG    0x13    // Receive filter 3 extended ID 0

#define MCP2515_RXF4SIDH_REG    0x14    // Receive filter 4 short ID high
#define MCP2515_RXF4SIDL_REG    0x15    // Receive filter 4 short ID low
#define MCP2515_RXF4EID8_REG    0x16    // Receive filter 4 extended ID 8
#define MCP2515_RXF4EID0_REG    0x17    // Receive filter 4 extended ID 0

#define MCP2515_RXF5SIDH_REG    0x18    // Receive filter 5 short ID high
#define MPC2515_RXF5SIDL_REG    0x19    // Receive filter 5 short ID low
#define MCP2515_RXF5EID8_REG    0x1a    // Receive filter 5 extended ID 8
#define MCP2515_RXF5EID0_REG    0x1b    // Receive filter 5 extended ID 0

#define MCP2515_TEC_REG         0x1c    // Transmit error counter
#define MCP2515_REC_REG         0x1d    // Receive error counter

// CANSTAT and CANCTRL occupy all addresses with 0xnE or 0xnF

#define MCP2515_RXM0SIDH_REG    0x20    // Receive mask 0 short ID high
#define MCP2515_RXM0SIDL_REG    0x21    // Receive mask 0 short ID low
#define MCP2515_RXM0EID8_REG    0x22    // Receive mask 0 extended ID 8
#define MCP2515_RXM0EID0_REG    0x23    // Receive mask 0 extended ID 0

#define MCP2515_RXM1SIDH_REG    0x24    // Receive mask 1 short ID high
#define MCP2515_RXM1SIDL_REG    0x25    // Receive mask 1 short ID low
#define MCP2515_RXM1EID8_REG    0x26    // Receive mask 1 extended ID 8
#define MCP2515_RXM1EID0_REG    0x27    // Receive mask 1 extended ID 0

// Bit timing configuration registers
#define MCP2515_CNF3_REG        0x28    // Bit timing configuration 3
#define MCP2515_CNF2_REG        0x29    // Bit timing configuration 2
#define MCP2515_CNF1_REG        0x2a    // Bit timing configuration 1

// CAN interrupt enable and flag registers
#define MCP2515_CANINTE_REG     0x2b    // CAN interrupt enable
#define MCP2515_CANINTF_REG     0x2c    // CAN interrupt flag

#define MCP2515_EFLG_REG        0x2d    // Error flag register

#define MCP2515_TXB0CTRL_REG    0x30    // Transmit buffer 0 control

#define MCP2515_TXB0SIDH_REG    0x31    // Transmit buffer 0 short ID high
#define MCP2515_TXB0SIDL_REG    0x32    // Transmit buffer 0 short ID low

#define MCP2515_TXB0EID8_REG    0x33    // Transmit buffer 0 extended ID 8
#define MCP2515_TXB0EID0_REG    0x34    // Transmit buffer 0 extended ID 0

#define MCP2515_TXB0DLC_REG     0x35    // Transmit buffer 0 data length code

#define MCP2515_TXB0D0_REG      0x36    // Transmit buffer 0 data registers...
#define MCP2515_TXB0D1_REG      0x37
#define MCP2515_TXB0D2_REG      0x38
#define MCP2515_TXB0D3_REG      0x39
#define MCP2515_TXB0D4_REG      0x3a
#define MCP2515_TXB0D5_REG      0x3b
#define MCP2515_TXB0D6_REG      0x3c
#define MCP2515_TXB0D7_REG      0x3d

#define MCP2515_TXB1CTRL_REG    0x40    // Transmit buffer 1 control

#define MCP2515_TXB1SIDH_REG    0x41    // Transmit buffer 1 short ID high
#define MCP2515_TXB1SIDL_REG    0x42    // Transmit buffer 1 short ID low

#define MCP2515_TXB1EID8_REG    0x43    // Transmit buffer 1 short ID 8
#define MCP2515_TXB1EID0_REG    0x44    // Transmit buffer 1 short ID 0

#define MCP2515_TXB1DLC_REG     0x45    // Transmit buffer 1 data length code

#define MCP2515_TXB1D0_REG      0x46    // Transmit buffer 1 data registers...
#define MCP2515_TXB1D1_REG      0x47
#define MCP2515_TXB1D2_REG      0x48
#define MCP2515_TXB1D3_REG      0x49
#define MCP2515_TXB1D4_REG      0x4a
#define MCP2515_TXB1D5_REG      0x4b
#define MCP2515_TXB1D6_REG      0x4c
#define MCP2515_TXB1D7_REG      0x4d

#define MCP2515_TXB2CTRL_REG    0x50    // Transmit buffer 2 control

#define MCP2515_TXB2SIDH_REG    0x51    // Transmit buffer 2 short ID high
#define MCP2515_TXB2SIDL_REG    0x52    // Transmit buffer 2 short ID low

#define MCP2515_TXB2EID8_REG    0x53    // Transmit buffer 2 extended ID 8
#define MCP2515_TXB2EID0_REG    0x54    // Transmit buffer 2 extended ID 0

#define MCP2515_TXB2DLC_REG     0x55    // Transmit buffer 2 data length code

#define MCP2515_TXB2D0_REG      0x56    // Transmit buffer 2 data registers...
#define MCP2515_TXB2D1_REG      0x57
#define MCP2515_TXB2D2_REG      0x58
#define MCP2515_TXB2D3_REG      0x59
#define MCP2515_TXB2D4_REG      0x5a
#define MCP2515_TXB2D5_REG      0x5b
#define MCP2515_TXB2D6_REG      0x5c
#define MCP2515_TXB2D7_REG      0x5d

// Receive buffer addresses
#define MCP2515_RXB0CTRL_REG    0x60    // Receive buffer 0 control

#define MCP2515_RXB0SIDH_REG    0x61    // Receive buffer 0 short ID high
#define MCP2515_RXB0SIDL_REG    0x62    // Receive buffer 0 short ID low

#define MCP2515_RXB0EID8_REG    0x63    // Receive buffer 0 extended ID 8
#define MCP2515_RXB0EID0_REG    0x64    // Receive buffer 0 extended ID 0

#define MCP2515_RXB0DLC_REG     0x65    // Receive buffer 0 data length code

#define MCP2515_RXB0D0_REG      0x66    // Receive buffer 0 data registers...
#define MCP2515_RXB0D1_REG      0x67
#define MCP2515_RXB0D2_REG      0x68
#define MCP2515_RXB0D3_REG      0x69
#define MCP2515_RXB0D4_REG      0x6a
#define MCP2515_RXB0D5_REG      0x6b
#define MCP2515_RXB0D6_REG      0x6c
#define MCP2515_RXB0D7_REG      0x6d

#define MCP2515_RXB1CTRL_REG    0x70    // Receive buffer 1 control

#define MCP2515_RXB1SIDH_REG    0x71    // Receive buffer 1 short ID high
#define MCP2515_RXB1SIDL_REG    0x72    // Receive buffer 1 short ID low

#define MCP2515_RXB1EID8_REG    0x73    // Receive buffer 1 extended ID 8
#define MCP2515_RXB1EID0_REG    0x74    // Receive buffer 1 extended ID 0

#define MCP2515_RXB1DLC_REG     0x75    // Receive buffer 1 data length code

#define MCP2515_RXB1D0_REG      0x76    // Receive buffer 1 data registers...
#define MCP2515_RXB1D1_REG      0x77
#define MCP2515_RXB1D2_REG      0x78
#define MCP2515_RXB1D3_REG      0x79
#define MCP2515_RXB1D4_REG      0x7a
#define MCP2515_RXB1D5_REG      0x7b
#define MCP2515_RXB1D6_REG      0x7c
#define MCP2515_RXB1D7_REG      0x7d

// Define the chip select pin for the MCP3208 12-bit ADC chip
#define MCP3208_CS_PIN 10

byte received_data[8] = {0}; // Array to store what will be received - init to 0
byte sent_data[8] = {0}; // Array to store what will be sent - init to 0

// tx register array
byte mcp2515_tx_regs[3][8] = {
  {MCP2515_TXB0D0_REG, MCP2515_TXB0D1_REG, MCP2515_TXB0D2_REG, MCP2515_TXB0D3_REG, MCP2515_TXB0D4_REG, MCP2515_TXB0D5_REG, MCP2515_TXB0D6_REG, MCP2515_TXB0D7_REG},
  {MCP2515_TXB1D0_REG, MCP2515_TXB1D1_REG, MCP2515_TXB1D2_REG, MCP2515_TXB1D3_REG, MCP2515_TXB1D4_REG, MCP2515_TXB1D5_REG, MCP2515_TXB1D6_REG, MCP2515_TXB1D7_REG},
  {MCP2515_TXB2D0_REG, MCP2515_TXB2D1_REG, MCP2515_TXB2D2_REG, MCP2515_TXB2D3_REG, MCP2515_TXB2D4_REG, MCP2515_TXB2D5_REG, MCP2515_TXB2D6_REG, MCP2515_TXB2D7_REG}
};

// rx registers
byte mcp2515_rx_regs[2][8] = {
  {MCP2515_RXB0D0_REG, MCP2515_RXB0D1_REG, MCP2515_RXB0D2_REG, MCP2515_RXB0D3_REG, MCP2515_RXB0D4_REG, MCP2515_RXB0D5_REG, MCP2515_RXB0D6_REG, MCP2515_RXB0D7_REG},
  {MCP2515_RXB1D0_REG, MCP2515_RXB1D1_REG, MCP2515_RXB1D2_REG, MCP2515_RXB1D3_REG, MCP2515_RXB1D4_REG, MCP2515_RXB1D5_REG, MCP2515_RXB1D6_REG, MCP2515_RXB1D7_REG}
};

void setup() {
  // put your setup code here, to run once:

  // Initialize the MCP2515
  can_shield_init();
}

void loop() {
  int adc_data = 0;

  /*
   * Un-comment to receive
   */
//  can_receive(0);  // This function is blocking and will wait for a new message
//  print_data();   // print the received_data[] array

  /*
   * Un-comment to transmit adc channel 1
   */
//  adc_data = adc_read(0); // Read the value from the sensor
//  sent_data[0] = adc_data >> 8; // Transfer the data to the sent_data[] array
//  sent_data[1] = (byte)(adc_data & 0x00ff);
//  can_send(0); // Send the data
}





/*
 * Lowers then raises the mcp2515 RESET pin and sends the RESET command 
 * to reset the chip then performs the necessary configurations.
 */
void can_shield_init() {
  // Set pinmode for mcp2515 pins
  pinMode(MCP2515_RESET_PIN,  OUTPUT);
  pinMode(MCP2515_CS_PIN, OUTPUT);
  pinMode(MCP2515_INT_PIN, INPUT);
  pinMode(MCP2515_RX0BF_PIN, INPUT);
  pinMode(MCP2515_RX1BF_PIN, INPUT);
  pinMode(MCP2515_TX0RTS_PIN, OUTPUT);
  pinMode(MCP2515_TX1RTS_PIN, OUTPUT);
  pinMode(MCP2515_TX2RTS_PIN, OUTPUT);

  // Set pinmode for mcp3208 pins
  pinMode(MCP3208_CS_PIN, OUTPUT);

  // Deselect both chips for now
  digitalWrite(MCP2515_CS_PIN, HIGH);
  digitalWrite(MCP3208_CS_PIN, HIGH);

  // Start Serial for debugging
  Serial.begin(9600);

  // Start the SPI bus. Do I need the SPI.beginTransaction() function?
  SPI.begin();
  Serial.println("Resetting...");
  // First, write the reset pin to low (it's active low)
  digitalWrite(MCP2515_RESET_PIN, LOW);
  delay(1); // wait. RESET must be low for 2 microseconds but this works
  digitalWrite(MCP2515_RESET_PIN, HIGH);

  /*
   * Do any other configuration here:
   * Message reception:
   * - enable the receive interrupt
   * - configure the RXB0F pin to be a buffer flag
   * - configure the chip to accept all messages
   * - change the chip from configuration mode to normal mode
   * Message transmission:
   * - configure TXBnCTRL with the CAN ID (short) and DLC
   */
  // -------------------- Configure Receive Buffer -------------------- //
  Serial.println("---------- initializing RXB0 -----------");
  // enable the interrupt for receive buffer 0
  // RX1IE is bit 0 of CANINTE
  Serial.println("\nEnabling RX0BF interrupt...");
  mcp2515_register_write(MCP2515_CANINTE_REG, mcp2515_register_read(MCP2515_CANINTE_REG) | 0b00000001);
  Serial.print("CAN Interrupt Enable Register:\t\t");
  print_byte(mcp2515_register_read(MCP2515_CANINTE_REG));
  Serial.print("CAN Interrupt Flag Register:\t\t");
  print_byte(mcp2515_register_read(MCP2515_CANINTF_REG));

  // configure the RX0BF pin to be a receive buffer interrupt
  // set the B0BFE bit in the BFPCTRL register
  Serial.println("\nConfiguring RX0BF pin as interrupt pin...");
  mcp2515_register_write(MCP2515_BFPCTRL_REG, mcp2515_register_read(MCP2515_BFPCTRL_REG) | 0b00000100);
  Serial.print("Buffer Flag Pin Control Register:\t");
  print_byte(mcp2515_register_read(MCP2515_BFPCTRL_REG));

  // Configure RX0 to receive all messages
  // set RXM[0:1] bits to '11'
  // RXM0 and RXM1 are RXB0CTRL bits 5 and 6
  Serial.println("\nConfiguring RXB0 to receive all messages...");
  mcp2515_register_write(MCP2515_RXB0CTRL_REG, mcp2515_register_read(MCP2515_RXB0CTRL_REG) | 0b01100000);
  Serial.print("RXB0 Control Register:\t\t\t");
  print_byte(mcp2515_register_read(MCP2515_RXB0CTRL_REG));

  // Disable rollover for RXB0
  Serial.println("\nDisabling rollover for RXB0...");
  mcp2515_register_write(MCP2515_RXB0CTRL_REG, mcp2515_register_read(MCP2515_RXB0CTRL_REG) & 0b11111011);

  Serial.println("-------- Completed initilizing RXB0 --------");

  // ----------------- Configure Transmission Buffer ----------------- //
  Serial.println("---------- initializing TXB0 ----------");
  // Set the CAN ID to 15 (temporary arbitrary value for now)
  // TXB0SIDH - Short ID High = 00000000
  // TXB0SIDL - Short ID Low =  00001111
  Serial.println("Configuring TXB0 with CAN ID 15...");
  mcp2515_register_write(MCP2515_TXB0SIDH_REG, 0b00000000);
  mcp2515_register_write(MCP2515_TXB0SIDL_REG, 0b00001111);
  Serial.print("TXB0 Short ID High: ");
  print_byte(mcp2515_register_read(MCP2515_TXB0SIDH_REG));
  Serial.print("TXB0 Short ID Low:  ");
  print_byte(mcp2515_register_read(MCP2515_TXB0SIDL_REG));
  
  // Set the Data Lenght Code (DLC) to 8 (2 bytes for each of 4 sensor inputs)
  // Bit 6 of DLC reg is "Remote Transmission Request" (RTR) bit -> set to 0 for data frame
  Serial.println("Set the Data Length Code (DLC) to 8...");
  mcp2515_register_write(MCP2515_TXB0DLC_REG, 0b00001000);
  Serial.print("TXB0 Data Length Code: ");
  print_byte(mcp2515_register_read(MCP2515_TXB0DLC_REG));

  // Set TXB0 to have the highest buffer priority (11)
  Serial.println("Configuring TXB0 to have highest buffer priority...");
  mcp2515_register_write(MCP2515_TXB0CTRL_REG, mcp2515_register_read(MCP2515_TXB0CTRL_REG) | 0b00000011);
  Serial.print("TXB0 Control Register: ");
  print_byte(mcp2515_register_read(MCP2515_TXB0CTRL_REG));
  Serial.println("-------- Completed initializing TXB0 --------");


  // -------------------- Configure Bit Timing -------------------- //
  Serial.println("---------- initializing bit timings ----------");
  // testing is conducted at 500kbps

  // 125 kbps -> t_bit = 1/125000  =  8 us  =  8000 ns
  // 500 kbps -> t_bit = 1/500000  =  2 us  =  2000 ns  <----
  // 1 Mbps   -> t_bit = 1/1000000 =  1 us  =  1000 ns

  // Time Quantum = 125 ns  -->  need 16 of them for 500 kbps
  
  // | ----- | --------- | ------------------ | ------------- |
  //  SyncSeg   PropSeg      PhaseSeg1 (PS1)   PhaseSeg2 (PS2)
  //
  //   1 T_Q     2 T_Q            7 T_Q             6 T_Q

  // set the Baud Rate Prescaler (BRP) bits to determin Time Quantum (T_Q)
  // T_OSC = 1/F_OSC = 1/16MHZ = 62.5 ns
  // 2 * T_OSC = 125 ns -> min Time Quantum
  // T_Q = 2 * BRP * T_OSC
  // set BRP in the CNF1 register -> set to 0 for T_Q = 2 * T_OSC
  // Bits CNF1[5:0]
  // ALSO Set the Synchronization Jump Width Length bits
  // make it 2 Time Quanta -> CNF1[7:6] = 01 
  Serial.println("\nConfiguring Time Quantum to 2*T_OSC...");
  Serial.println("Configuring Synchronization Jump Width...");
  mcp2515_register_write(MCP2515_CNF1_REG, mcp2515_register_read(MCP2515_CNF1_REG) & 0b00000000);
  mcp2515_register_write(MCP2515_CNF1_REG, mcp2515_register_read(MCP2515_CNF1_REG) | 0b01000000);
  Serial.print("Configuration Register 1:\t\t");
  print_byte(mcp2515_register_read(MCP2515_CNF1_REG));

  // Set the sample point to sample once
  Serial.println("\nConfiguring sample point...");
  mcp2515_register_write(MCP2515_CNF2_REG, mcp2515_register_read(MCP2515_CNF2_REG) & 0b10111111);
  Serial.print("Configuration Register 2:\t\t");
  print_byte(mcp2515_register_read(MCP2515_CNF2_REG));

  // Set the PS2 length with CNF3 bits
  // CNF2[7] = 1
  Serial.println("\nConfiguring PS2 Bit Time Selection Method...");
  mcp2515_register_write(MCP2515_CNF2_REG, mcp2515_register_read(MCP2515_CNF2_REG) | 0b10000000);
  Serial.print("Configuration Register 2:\t\t");
  print_byte(mcp2515_register_read(MCP2515_CNF2_REG));

  // ----- Propagation Segment (PropSeg) length ----- //
  // set to 2 -> CNF2[2:0] = 001
  Serial.println("\nConfiguring Propagation Segment Length...");
  mcp2515_register_write(MCP2515_CNF2_REG, mcp2515_register_read(MCP2515_CNF2_REG) | 0b00000001);
  Serial.print("Configuration Register 2:\t\t");
  print_byte(mcp2515_register_read(MCP2515_CNF2_REG));

  // ----- Phase Segment 1 (PS1) length ----- //
  // set to 7 -> CNF2[5:3] = 7 - 1 = 6 = 110
  Serial.println("\nConfiguring Phase Segment 1 Length...");
  mcp2515_register_write(MCP2515_CNF2_REG, mcp2515_register_read(MCP2515_CNF2_REG) & 0b11000111);
  mcp2515_register_write(MCP2515_CNF2_REG, mcp2515_register_read(MCP2515_CNF2_REG) | 0b00110000);
  Serial.print("Configuration Register 2:\t\t");
  print_byte(mcp2515_register_read(MCP2515_CNF2_REG));

  // ----- Phase Segment 2 (PS2) length ----- //
  // set to 6 -> CNF3[2:0] = 6 - 1 = 5 = 101
  Serial.println("\nConfiguring Phase Segment 2 Length...");
  mcp2515_register_write(MCP2515_CNF3_REG, mcp2515_register_read(MCP2515_CNF3_REG) & 0b11111000);
  mcp2515_register_write(MCP2515_CNF3_REG, mcp2515_register_read(MCP2515_CNF3_REG) | 0b00000101);
  Serial.print("Configuration Register 3:\t\t");
  print_byte(mcp2515_register_read(MCP2515_CNF3_REG));


  // -------------------- Disable Clock out pin -------------------- //
  // CANCTRL[2] = 0
  Serial.println("\nDisabling Clock out pin...");
  mcp2515_register_write(MCP2515_CANCTRL_REG, mcp2515_register_read(MCP2515_CANCTRL_REG) & 0b11111011);
  Serial.print("CAN Control Register: ");
  print_byte(mcp2515_register_read(MCP2515_CANCTRL_REG));
  
  // -------------------- Set to Normal Mode -------------------- //
  // REQOP[2:0] = 000 to operating in normal mode
  // REQOP[2:0] are bits CANCTRL[7:5]
  Serial.println("\nRequesting 'Normal Mode' Operation Mode");
  mcp2515_register_write(MCP2515_CANCTRL_REG, mcp2515_register_read(MCP2515_CANCTRL_REG) & 0b00011111);
  //mcp2515_register_write(MCP2515_CANCTRL_REG, mcp2515_register_read(MCP2515_CANCTRL_REG) | 0b01100000);
  Serial.print("CAN Control Register:\t\t\t");
  print_byte(mcp2515_register_read(MCP2515_CANCTRL_REG));

  Serial.println("------ initialization complete ------");
  delay(1000);
}

/*
 * Takes the address of a register and a byte to be written
 * to the register and send the instruction to write to that
 * register address.
 */
void mcp2515_register_write(byte addr, byte data) {
  // select the chip
  digitalWrite(MCP2515_CS_PIN, LOW);

  // Write the sequence of bytes
  SPI.transfer(MCP2515_WRITE_INSTR);
  SPI.transfer(addr);
  SPI.transfer(data);

  // deselect the chips
  digitalWrite(MCP2515_CS_PIN, HIGH);
}

/*
 * Takes the address of the register to be read and returns the
 * byte stored in that register.
 */
byte mcp2515_register_read(byte addr) {
  byte received_value;
  
  //select the chip
  digitalWrite(MCP2515_CS_PIN, LOW);

  // send the instruction
  SPI.transfer(MCP2515_READ_INSTR);
  SPI.transfer(addr);
  received_value = SPI.transfer(0x0); // transfer to get the value

  // deselect the chip
  digitalWrite(MCP2515_CS_PIN, HIGH);

  return received_value;
}

/*
 * Sends the data stored in the global variable "data" using the
 * transmit buffer value stored in the parameter "tx_buffer".
 */
void can_send(byte tx_buffer) {
  if (tx_buffer != 0 && tx_buffer != 1 && tx_buffer != 2) {
    Serial.println("TX BUFFER CAN BE 0, 1, 0R 2");
    return;
  }

  // Wait for the TXBnRTS bit to be cleared
  while(1) {
    if (tx_buffer == 0 && (mcp2515_register_read(MCP2515_TXB0CTRL_REG) & 0b00001000) == 0) break;
    if (tx_buffer == 1 && (mcp2515_register_read(MCP2515_TXB1CTRL_REG) & 0b00001000) == 0) break;
    if (tx_buffer == 2 && (mcp2515_register_read(MCP2515_TXB2CTRL_REG) & 0b00001000) == 0) break;
    Serial.println("Waiting to send...");
  }

  // Write the send_data[] array to the tranmission buffer specified
  for (int i = 0; i < 8; i++) {
    mcp2515_register_write(mcp2515_tx_regs[tx_buffer][i], sent_data[i]);
  }

  // Initiate transmission by setting the TXREQ bit in the TXB0CTRL register
  // I'll do this by sending the SPI Request-to-Send Instruction (there are
  //    other ways - see MCP2515 datasheet)
  digitalWrite(MCP2515_CS_PIN, LOW);
  SPI.transfer(MCP2515_RTS_INSTR(0));
  digitalWrite(MCP2515_CS_PIN, HIGH);
}

/*
 * Populates the global variable "data" with the value stored
 * in the receive buffer specified with the parameter "rx_buffer".
 */
void can_receive(byte rx_buffer) {
  if (rx_buffer != 0 && rx_buffer != 1) {
    Serial.println("RECEIVE BUFFER CAN BE 0 OR 1");
    return;
  }

  // Wait for a new message in RXB0
  while (digitalRead(MCP2515_INT_PIN));

  // Read the data from the registers into the data[] array
  for (int i = 0; i < 8; i++) {
    received_data[i] = mcp2515_register_read(mcp2515_rx_regs[rx_buffer][i]);
  }

  // reset the received message flag bit
  mcp2515_register_write(MCP2515_CANINTF_REG, mcp2515_register_read(MCP2515_CANINTF_REG) & 0b111111110); // read the value and clear bit 0
}

/*
 * Interaction is 2-bytes long but the first 3 bits sent are the channel selection.
 * The address is sent out and the last 4 bits of the received byte from that
 * "transfer()" are the first 4 bits of the reading. The last 8 bits are shifted out
 * during the subsquent "shiftIn()" call.
 */
int adc_read(byte channel) {
  int adc_value = 0; // some random thing to show if its working

  byte adc_hi = 0;
  byte adc_lo = 0;

  // Begin the SPI interaction
  digitalWrite(MCP3208_CS_PIN, LOW);
  // send out the bits to indicate single-ended Channel 0: 1000000
  // Bit timing for sampling a channel - see page 21 of the MCP3208 datasheet
  // send leading zeros to allow for 3 groups of 8 bits.
  SPI.transfer(0b00000110);
  adc_hi = SPI.transfer(0b00000000); // Fisrt 4 bits of the data
  adc_lo = SPI.transfer(0b00000000); // Last 8 bits of the data

  // Cut out any crazy don't care bits
  adc_hi = adc_hi & 0b00001111;

  // Construct the final value
  adc_value = (int)adc_hi;
  adc_value = adc_value << 8;
  adc_value = adc_value | (int)adc_lo;

  digitalWrite(MCP3208_CS_PIN, HIGH);

  return adc_value;
}




/*
 * Debugging Functions
 */

void byte_to_string(byte b, char result[9]) {
  result[8] = '\0';
  for (int i = 0; i < 8; i++) {
    result[i] = 'x';
  }

  for (int i = 7; i >= 0; i--) {
    if ((b & 0b00000001) == 1) {
      result[i] = '1';
    } else {
      result[i] = '0';
    }
    b = b >> 1;
  }
}

void print_byte(byte b) {
  char byte_string[9];
  
  byte_to_string(b, byte_string);
  Serial.print("0b");
  Serial.println(byte_string);
}

void print_data() {
  for (int i = 0; i < 8; i++) {
    Serial.print("; data[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(received_data[i]);
  }
  Serial.println("");
}
