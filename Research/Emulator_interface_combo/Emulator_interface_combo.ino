/*************************************************************************
 *
 * GAMEBOY PRINTER EMULATION PROJECT V3.2.1 (Arduino)
 * Copyright (C) 2022 Brian Khuu
 *
 * PURPOSE: To capture gameboy printer images without a gameboy printer
 *          via the arduino platform. (Tested on the arduino nano)
 *          This version is to investigate gameboy behaviour.
 *          This was originally started on 2017-4-6 but updated on 2020-08-16
 * LICENCE:
 *   This file is part of Arduino Gameboy Printer Emulator.
 *
 *   Arduino Gameboy Printer Emulator is free software:
 *   you can redistribute it and/or modify it under the terms of the
 *   GNU General Public License as published by the Free Software Foundation,
 *   either version 3 of the License, or (at your option) any later version.
 *
 *   Arduino Gameboy Printer Emulator is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Arduino Gameboy Printer Emulator.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
 ///////////////////////////////////super printer intercace stuff//////////////////////////////////////////////////
byte palette            = 0xE4;  // 0x00 is treated as default (= 0xE4)
byte intensity          = 0x40;  //default intensity is 0x40, min is 0x00, max is 0x7F, values between 0x80 and 0xFF are treated as default
byte margin             = 0x03;  //high nibble, upper margin, low nibble, lower margin, that simple
byte sheets             = 0x01;  //Number of sheets to print (0-255). 0 means line feed only.
bool state_printer_busy = 0;
bool bit_sent, bit_read;
bool state_printer_connected = 0;
byte byte_sent;
// if you modify a command, the checksum bytes must be modified accordingly if it's not a const
const byte INIT[] = { 0x88, 0x33, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };                                      //Init command, will never change
byte PRNT[]       = { 0x88, 0x33, 0x02, 0x00, 0x04, 0x00, sheets, margin, palette, intensity, 0x00, 0x00, 0x00, 0x00 };  //Print command without feed lines
const byte EMPT[] = { 0x88, 0x33, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00 };                                      //Empty data packet, mandatory for preparing printing, will never change
const byte ABOR[] = { 0x88, 0x33, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00 };                                      //Abort sequence, rarely used by games, will never change
const byte INQU[] = { 0x88, 0x33, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00 };                                      //Inquiry command, will never change

#define PRINT_PAYLOAD_SIZE  3
#define DATA_PAYLOAD_SIZE   640
#define DATA_TOTAL_SIZE     650
#define DATA_PAYLOAD_OFFSET 6
#define BUFFER_WAIT_TIME    100
uint8_t printBuffer[PRINT_PAYLOAD_SIZE];
///////////////////////////////////super printer intercace stuff//////////////////////////////////////////////////

// See /WEBUSB.md for details
#if USB_VERSION == 0x210
#include <WebUSB.h>
WebUSB WebUSBSerial(1, "herrzatacke.github.io/gb-printer-web/#/webusb");
#define Serial WebUSBSerial
#endif

#define GAME_BOY_PRINTER_MODE      true   // to use with https://github.com/Mraulio/GBCamera-Android-Manager and https://github.com/Raphael-Boichot/PC-to-Game-Boy-Printer-interface
#define GBP_OUTPUT_RAW_PACKETS     true   // by default, packets are parsed. if enabled, output will change to raw data packets for parsing and decompressing later
#define GBP_USE_PARSE_DECOMPRESSOR false  // embedded decompressor can be enabled for use with parse mode but it requires fast hardware (SAMD21, SAMD51, ESP8266, ESP32)

#include <stdint.h>  // uint8_t
#include <stddef.h>  // size_t

#include "gameboy_printer_protocol.h"
#include "gbp_serial_io.h"

#if GBP_OUTPUT_RAW_PACKETS
#define GBP_FEATURE_PACKET_CAPTURE_MODE
#else
#define GBP_FEATURE_PARSE_PACKET_MODE
#if GBP_USE_PARSE_DECOMPRESSOR
#define GBP_FEATURE_PARSE_PACKET_USE_DECOMPRESSOR
#endif
#endif

#ifdef GBP_FEATURE_PARSE_PACKET_MODE
#include "gbp_pkt.h"
#endif




/* Gameboy Link Cable Mapping to Arduino Pin */
// Note: Serial Clock Pin must be attached to an interrupt pin of the arduino
//  ___________
// |  6  4  2  |
//  \_5__3__1_/   (at cable)
//

// clang-format off
#ifdef ESP8266
// Pin Setup for ESP8266 Devices
//                  | Arduino Pin | Gameboy Link Pin  |
#define GBP_VCC_PIN               // Pin 1            : 5.0V (Unused)
#define GBP_SO_PIN       13       // Pin 2            : ESP-pin 7 MOSI (Serial OUTPUT) -> Arduino 13
#define GBP_SI_PIN       12       // Pin 3            : ESP-pin 6 MISO (Serial INPUT)  -> Arduino 12
#define GBP_SD_PIN                // Pin 4            : Serial Data  (Unused)
#define GBP_SC_PIN       14       // Pin 5            : ESP-pin 5 CLK  (Serial Clock)  -> Arduino 14
#define GBP_GND_PIN               // Pin 6            : GND (Attach to GND Pin)
#define LED_STATUS_PIN    2       // Internal LED blink on packet reception
#else
// Pin Setup for Arduinos
//                  | Arduino Pin | Gameboy Link Pin  |
#define GBP_VCC_PIN               // Pin 1            : 5.0V (Unused)
#define GBP_SO_PIN        4       // Pin 2            : Serial OUTPUT
#define GBP_SI_PIN        3       // Pin 3            : Serial INPUT
#define GBP_SD_PIN                // Pin 4            : Serial Data  (Unused)
#define GBP_SC_PIN        2       // Pin 5            : Serial Clock (Interrupt)
#define GBP_GND_PIN               // Pin 6            : GND (Attach to GND Pin)
#define LED_STATUS_PIN   13       // Internal LED blink on packet reception
#endif
// clang-format on

/*******************************************************************************
*******************************************************************************/

// Dev Note: Gamboy camera sends data payload of 640 bytes usually

//#ifdef GBP_FEATURE_PARSE_PACKET_MODE
//#define GBP_BUFFER_SIZE 400
//#else
///////////////////////////////////super printer intercace stuff//////////////////////////////////////////////////
#define GBP_BUFFER_SIZE 650 //I need 650 bytes buffer !
///////////////////////////////////super printer intercace stuff//////////////////////////////////////////////////
//#endif

/* Serial IO */
// This circular buffer contains a stream of raw packets from the gameboy
uint8_t gbp_serialIO_raw_buffer[GBP_BUFFER_SIZE] = { 0 };

#ifdef GBP_FEATURE_PARSE_PACKET_MODE
/* Packet Buffer */
gbp_pkt_t gbp_pktState                                 = { GBP_REC_NONE, 0 };
uint8_t gbp_pktbuff[GBP_PKT_PAYLOAD_BUFF_SIZE_IN_BYTE] = { 0 };
uint8_t gbp_pktbuffSize                                = 0;
#ifdef GBP_FEATURE_PARSE_PACKET_USE_DECOMPRESSOR
gbp_pkt_tileAcc_t tileBuff = { 0 };
#endif
#endif

#ifdef GBP_FEATURE_PACKET_CAPTURE_MODE
inline void gbp_packet_capture_loop();
#endif
#ifdef GBP_FEATURE_PARSE_PACKET_MODE
inline void gbp_parse_packet_loop();
#endif

/*******************************************************************************
  Utility Functions
*******************************************************************************/

const char* gbpCommand_toStr(int val)
{
  switch (val)
  {
    case GBP_COMMAND_INIT: return "INIT";
    case GBP_COMMAND_PRINT: return "PRNT";
    case GBP_COMMAND_DATA: return "DATA";
    case GBP_COMMAND_BREAK: return "BREK";
    case GBP_COMMAND_INQUIRY: return "INQY";
    default: return "?";
  }
}

/*******************************************************************************
  Interrupt Service Routine
*******************************************************************************/

#ifdef ESP8266
void ICACHE_RAM_ATTR serialClock_ISR(void)
#else
void serialClock_ISR(void)
#endif
{
  // Serial Clock (1 = Rising Edge) (0 = Falling Edge); Master Output Slave Input (This device is slave)
#ifdef GBP_FEATURE_USING_RISING_CLOCK_ONLY_ISR
  const bool txBit = gpb_serial_io_OnRising_ISR(digitalRead(GBP_SO_PIN));
#else
  const bool txBit = gpb_serial_io_OnChange_ISR(digitalRead(GBP_SC_PIN), digitalRead(GBP_SO_PIN));
#endif
  digitalWrite(GBP_SI_PIN, txBit ? HIGH : LOW);
}


/*******************************************************************************
  Main Setup and Loop
*******************************************************************************/

void setup(void)
{
  // Config Serial
  // Has to be fast or it will not transfer the image fast enough to the computer
  Serial.begin(115200);

  // Wait for Serial to be ready
  while (!Serial) { ; }
#if GAME_BOY_PRINTER_MODE  //Printer mode
  Connect_to_printer();    //makes an attempt to switch in printer mode
#endif
  /* Pins from gameboy link cable */
  pinMode(GBP_SC_PIN, INPUT);
  pinMode(GBP_SO_PIN, INPUT);
  pinMode(GBP_SI_PIN, OUTPUT);

  /* Default link serial out pin state */
  digitalWrite(GBP_SI_PIN, LOW);

  /* LED Indicator */
  pinMode(LED_STATUS_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);

  /* Setup */
  gpb_serial_io_init(sizeof(gbp_serialIO_raw_buffer), gbp_serialIO_raw_buffer);

  /* Attach ISR */
#ifdef GBP_FEATURE_USING_RISING_CLOCK_ONLY_ISR
  attachInterrupt(digitalPinToInterrupt(GBP_SC_PIN), serialClock_ISR, RISING);  // attach interrupt handler
#else
  attachInterrupt(digitalPinToInterrupt(GBP_SC_PIN), serialClock_ISR, CHANGE);  // attach interrupt handler
#endif

  /* Packet Parser */
#ifdef GBP_FEATURE_PARSE_PACKET_MODE
  gbp_pkt_init(&gbp_pktState);
#endif

#define VERSION_STRING "V3.2.1 (Copyright (C) 2022 Brian Khuu)"

  /* Welcome Message */
#ifdef GBP_FEATURE_PACKET_CAPTURE_MODE
  Serial.println(F("// GAMEBOY PRINTER Packet Capture " VERSION_STRING));
  Serial.println(F("// Note: Each byte is from each GBP packet is from the gameboy"));
  Serial.println(F("//       except for the last two bytes which is from the printer"));
  Serial.println(F("// JS Raw Packet Decoder: https://mofosyne.github.io/arduino-gameboy-printer-emulator/GameBoyPrinterDecoderJS/gameboy_printer_js_raw_decoder.html"));
#endif
#ifdef GBP_FEATURE_PARSE_PACKET_MODE
  Serial.println(F("// GAMEBOY PRINTER Emulator " VERSION_STRING));
  Serial.println(F("// Note: Each hex encoded line is a gameboy tile"));
  Serial.println(F("// JS Decoder: https://mofosyne.github.io/arduino-gameboy-printer-emulator/GameBoyPrinterDecoderJS/gameboy_printer_js_decoder.html"));
#endif
  Serial.println(F("// --- GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007 ---"));
  Serial.println(F("// This program comes with ABSOLUTELY NO WARRANTY;"));
  Serial.println(F("// This is free software, and you are welcome to redistribute it"));
  Serial.println(F("// under certain conditions. Refer to LICENSE file for detail."));
  Serial.println(F("// ---"));

  Serial.flush();
}  // setup()

void loop()
{
  static uint16_t sioWaterline = 0;

#ifdef GBP_FEATURE_PACKET_CAPTURE_MODE
  gbp_packet_capture_loop();
#endif
#ifdef GBP_FEATURE_PARSE_PACKET_MODE
  gbp_parse_packet_loop();
#endif

  // Trigger Timeout and reset the printer if byte stopped being received.
  static uint32_t last_millis = 0;
  uint32_t curr_millis        = millis();
  if (curr_millis > last_millis)
  {
    uint32_t elapsed_ms = curr_millis - last_millis;
    if (gbp_serial_io_timeout_handler(elapsed_ms))
    {
      Serial.println("");
      Serial.print("// Completed ");
      Serial.print("(Memory Waterline: ");
      Serial.print(gbp_serial_io_dataBuff_waterline(false));
      Serial.print("B out of ");
      Serial.print(gbp_serial_io_dataBuff_max());
      Serial.println("B)");
      Serial.flush();
      digitalWrite(LED_STATUS_PIN, LOW);

#ifdef GBP_FEATURE_PARSE_PACKET_MODE
      gbp_pkt_reset(&gbp_pktState);
#ifdef GBP_FEATURE_PARSE_PACKET_USE_DECOMPRESSOR
      tileBuff.count = 0;
#endif
#endif
    }
  }
  last_millis = curr_millis;

  // Diagnostics Console
  while (Serial.available() > 0)
  {
    switch (Serial.read())
    {
      case '?':
        Serial.println("d=debug, ?=help");
        break;

      case 'd':
        Serial.print("waterline: ");
        Serial.print(gbp_serial_io_dataBuff_waterline(false));
        Serial.print("B out of ");
        Serial.print(gbp_serial_io_dataBuff_max());
        Serial.println("B");
        break;
    }
  };
}  // loop()

/******************************************************************************/

#ifdef GBP_FEATURE_PARSE_PACKET_MODE
inline void gbp_parse_packet_loop(void)
{
  const char nibbleToCharLUT[] = "0123456789ABCDEF";
  for (int i = 0; i < gbp_serial_io_dataBuff_getByteCount(); i++)
  {
    if (gbp_pkt_processByte(&gbp_pktState, (const uint8_t)gbp_serial_io_dataBuff_getByte(), gbp_pktbuff, &gbp_pktbuffSize, sizeof(gbp_pktbuff)))
    {
      if (gbp_pktState.received == GBP_REC_GOT_PACKET)
      {
        digitalWrite(LED_STATUS_PIN, HIGH);
        Serial.print((char)'{');
        Serial.print("\"command\":\"");
        Serial.print(gbpCommand_toStr(gbp_pktState.command));
        Serial.print("\"");
        if (gbp_pktState.command == GBP_COMMAND_INQUIRY)
        {
          // !{"command":"INQY","status":{"lowbatt":0,"jam":0,"err":0,"pkterr":0,"unproc":1,"full":0,"bsy":0,"chk_err":0}}
          Serial.print(", \"status\":{");
          Serial.print("\"LowBat\":");
          Serial.print(gpb_status_bit_getbit_low_battery(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"ER2\":");
          Serial.print(gpb_status_bit_getbit_other_error(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"ER1\":");
          Serial.print(gpb_status_bit_getbit_paper_jam(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"ER0\":");
          Serial.print(gpb_status_bit_getbit_packet_error(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"Untran\":");
          Serial.print(gpb_status_bit_getbit_unprocessed_data(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"Full\":");
          Serial.print(gpb_status_bit_getbit_print_buffer_full(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"Busy\":");
          Serial.print(gpb_status_bit_getbit_printer_busy(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"Sum\":");
          Serial.print(gpb_status_bit_getbit_checksum_error(gbp_pktState.status) ? '1' : '0');
          Serial.print((char)'}');
        }
        if (gbp_pktState.command == GBP_COMMAND_PRINT)
        {
          //!{"command":"PRNT","sheets":1,"margin_upper":1,"margin_lower":3,"pallet":228,"density":64 }
          Serial.print(", \"sheets\":");
          Serial.print(gbp_pkt_printInstruction_num_of_sheets(gbp_pktbuff));
          Serial.print(", \"margin_upper\":");
          Serial.print(gbp_pkt_printInstruction_num_of_linefeed_before_print(gbp_pktbuff));
          Serial.print(", \"margin_lower\":");
          Serial.print(gbp_pkt_printInstruction_num_of_linefeed_after_print(gbp_pktbuff));
          Serial.print(", \"pallet\":");
          Serial.print(gbp_pkt_printInstruction_palette_value(gbp_pktbuff));
          Serial.print(", \"density\":");
          Serial.print(gbp_pkt_printInstruction_print_density(gbp_pktbuff));
        }
        if (gbp_pktState.command == GBP_COMMAND_DATA)
        {
          //!{"command":"DATA", "compressed":0, "more":0}
#ifdef GBP_FEATURE_PARSE_PACKET_USE_DECOMPRESSOR
          Serial.print(", \"compressed\":0");  // Already decompressed by us, so no need to do so
#else
          Serial.print(", \"compressed\":");
          Serial.print(gbp_pktState.compression);
#endif
          Serial.print(", \"more\":");
          Serial.print((gbp_pktState.dataLength != 0) ? '1' : '0');
        }
        Serial.println((char)'}');
        Serial.flush();
      }
      else
      {
#ifdef GBP_FEATURE_PARSE_PACKET_USE_DECOMPRESSOR
        // Required for more complex games with compression support
        while (gbp_pkt_decompressor(&gbp_pktState, gbp_pktbuff, gbp_pktbuffSize, &tileBuff))
        {
          if (gbp_pkt_tileAccu_tileReadyCheck(&tileBuff))
          {
            // Got Tile
            for (int i = 0; i < GBP_TILE_SIZE_IN_BYTE; i++)
            {
              const uint8_t data_8bit = tileBuff.tile[i];
              if (i == GBP_TILE_SIZE_IN_BYTE - 1)
              {
                Serial.print((char)nibbleToCharLUT[(data_8bit >> 4) & 0xF]);
                Serial.println((char)nibbleToCharLUT[(data_8bit >> 0) & 0xF]);  // use println on last byte to reduce serial calls
              }
              else
              {
                Serial.print((char)nibbleToCharLUT[(data_8bit >> 4) & 0xF]);
                Serial.print((char)nibbleToCharLUT[(data_8bit >> 0) & 0xF]);
                Serial.print((char)' ');
              }
            }
            Serial.flush();
          }
        }
#else
        // Simplified support for gameboy camera only application
        // Dev Note: Good for checking if everything above decompressor is working
        if (gbp_pktbuffSize > 0)
        {
          // Got Tile
          for (int i = 0; i < gbp_pktbuffSize; i++)
          {
            const uint8_t data_8bit = gbp_pktbuff[i];
            if (i == gbp_pktbuffSize - 1)
            {
              Serial.print((char)nibbleToCharLUT[(data_8bit >> 4) & 0xF]);
              Serial.println((char)nibbleToCharLUT[(data_8bit >> 0) & 0xF]);  // use println on last byte to reduce serial calls
            }
            else
            {
              Serial.print((char)nibbleToCharLUT[(data_8bit >> 4) & 0xF]);
              Serial.print((char)nibbleToCharLUT[(data_8bit >> 0) & 0xF]);
              Serial.print((char)' ');
            }
          }
          Serial.flush();
        }
#endif
      }
    }
  }
}
#endif

#ifdef GBP_FEATURE_PACKET_CAPTURE_MODE
inline void gbp_packet_capture_loop()
{
  /* tiles received */
  static uint32_t byteTotal     = 0;
  static uint32_t pktTotalCount = 0;
  static uint32_t pktByteIndex  = 0;
  static uint16_t pktDataLength = 0;
  const size_t dataBuffCount    = gbp_serial_io_dataBuff_getByteCount();
  if (
    ((pktByteIndex != 0) && (dataBuffCount > 0)) || ((pktByteIndex == 0) && (dataBuffCount >= 6)))
  {
    const char nibbleToCharLUT[] = "0123456789ABCDEF";
    uint8_t data_8bit            = 0;
    for (int i = 0; i < dataBuffCount; i++)
    {  // Display the data payload encoded in hex
      // Start of a new packet
      if (pktByteIndex == 0)
      {
        pktDataLength = gbp_serial_io_dataBuff_getByte_Peek(4);
        pktDataLength |= (gbp_serial_io_dataBuff_getByte_Peek(5) << 8) & 0xFF00;
#if 0
        Serial.print("// ");
        Serial.print(pktTotalCount);
        Serial.print(" : ");
        Serial.println(gbpCommand_toStr(gbp_serial_io_dataBuff_getByte_Peek(2)));
#endif
        digitalWrite(LED_STATUS_PIN, HIGH);
      }
      // Print Hex Byte
      data_8bit = gbp_serial_io_dataBuff_getByte();
      Serial.print((char)nibbleToCharLUT[(data_8bit >> 4) & 0xF]);
      Serial.print((char)nibbleToCharLUT[(data_8bit >> 0) & 0xF]);
      // Splitting packets for convenience
      if ((pktByteIndex > 5) && (pktByteIndex >= (9 + pktDataLength)))
      {
        digitalWrite(LED_STATUS_PIN, LOW);
        Serial.println("");
        pktByteIndex = 0;
        pktTotalCount++;
      }
      else
      {
        Serial.print((char)' ');
        pktByteIndex++;  // Byte hex split counter
        byteTotal++;     // Byte total counter
      }
    }
    Serial.flush();
  }
}
#endif

//////////////////////////////////////Printer stuff//////////////////////////////////////////////
void Connect_to_printer()
{
  pinMode(GBP_SC_PIN, OUTPUT);
  pinMode(GBP_SO_PIN, INPUT_PULLUP);
  pinMode(GBP_SI_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  send_printer_packet(INIT, 10);  // here we send the INIT command until we get 0x81 at byte 9, printer connected
  if (state_printer_connected)  //Printer connected !
  {// format the general data buffer, same as the printer emulator
    gbp_serialIO_raw_buffer[0]=0x88;
    gbp_serialIO_raw_buffer[1]=0x33;
    gbp_serialIO_raw_buffer[2]=0x04;
    digitalWrite(GBP_SC_PIN, HIGH);  //acts like a real Game Boy
    digitalWrite(GBP_SI_PIN, LOW);   //acts like a real Game Boy
    delay(100);                      // Give host time to connect
    Serial.println();
    Serial.println(F("// --- Super Printer Interface by Raphaël BOICHOT, 6 June 2025 ---"));  //welcome message for GNU Octave
    Serial.println(F("// ----- GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007 ------"));
    Serial.flush();  // Ensure it's fully transmitted

    while (true)
    {
      if (Serial.available())
      {
        char packetType = Serial.read();

        if (packetType == 'P')
        {
          if (receiveAndStorePayload(printBuffer, PRINT_PAYLOAD_SIZE))
          {
            PRNT[7] = printBuffer[0];
            PRNT[8] = printBuffer[1];
            PRNT[9] = printBuffer[2];
            echoPacket('P', printBuffer, PRINT_PAYLOAD_SIZE);
            finalize_and_print();  // stuff the printer with commands
            Serial.println(F("Printer ready"));
            Serial.flush();
          }
          else
          {
            flushSerialInput();
          }
        }
        else if (packetType == 'D')
        {
          if (receiveAndStorePayload(gbp_serialIO_raw_buffer + DATA_PAYLOAD_OFFSET, DATA_PAYLOAD_SIZE))
          {
            echoPacket('D', gbp_serialIO_raw_buffer + DATA_PAYLOAD_OFFSET, DATA_PAYLOAD_SIZE);
            transmit_data_packet(gbp_serialIO_raw_buffer, 640);  //packet formatting
            Serial.println(F("Printer ready"));
            Serial.flush();
          }
          else
          {
            flushSerialInput();
          }
        }
      }
    }
  }
}

void printing_loop()
{
  state_printer_busy = 1;  //to enter the loop
  delay(200);              //printer is not immediately busy
  while (state_printer_busy)
  {  //call iquiry until not busy
    Serial.println();
    Serial.print(F("INQU packet sent -->"));
    state_printer_busy = 0;
    send_printer_packet(INQU, 10);
    delay(200);
  }
  Serial.println();
  Serial.print(F("Printer not busy !"));
}

void send_printer_packet(byte packet[], int sequence_length)
{
  for (int i = 0; i <= sequence_length - 1; i++)
  {
    int error_check      = (i == sequence_length - 1) ? 1 : -1;
    int connection_check = (i == sequence_length - 2) ? 1 : -1;
    int mode             = ((i == sequence_length - 1) || (i == sequence_length - 2)) ? 2 : 1;
    digitalWrite(LED_STATUS_PIN, HIGH);
    printing(packet[i], mode, error_check, connection_check);
    digitalWrite(LED_STATUS_PIN, LOW);
  }
}

void printing(int byte_sent, int mode, int error_check, int connection_check)
{  // this function prints bytes to the serial, all the meat is here
  byte byte_read;
  for (int j = 0; j <= 7; j++)
  {
    bit_sent = bitRead(byte_sent, 7 - j);
    digitalWrite(GBP_SC_PIN, LOW);
    digitalWrite(GBP_SI_PIN, bit_sent);
    delayMicroseconds(30);  //double speed mode
    digitalWrite(GBP_SC_PIN, HIGH);
    bit_read = (digitalRead(GBP_SO_PIN));
    bitWrite(byte_read, 7 - j, bit_read);
    delayMicroseconds(30);  //double speed mode
  }
  delayMicroseconds(0);  //optional delay between bytes, may me less than 1490 µs

  //// this part is just for debugging, it shows all the data sent////////
  // if (mode == 1) {
  //   if (byte_sent <= 0x0F) {
  //     Serial.print('0');
  //   }
  //   Serial.print(byte_sent, HEX);
  //   Serial.print(' ');
  // }
  // if (mode == 2) {
  //   if (byte_read <= 0x0F) {
  //     Serial.print('0');
  //   }
  //   Serial.print(byte_read, HEX);
  //   Serial.print(' ');
  // }
  /////////////////////////////////////////////////////////////////////////

  if (connection_check == 1)
  {
    state_printer_connected = 0;
    if (byte_read == 0x81)
    {
      state_printer_connected = 1;
    };
  }

  if (error_check == 1)
  {
    //Serial.print("--> ");
    for (int m = 0; m <= 7; m++) { Serial.print(bitRead(byte_read, 7 - m)); }
    if (bitRead(byte_read, 0)) { Serial.print(F(" / Checksum error")); }
    state_printer_busy = 0;
    if (bitRead(byte_read, 1))
    {
      state_printer_busy = 1;
      Serial.print(F(" / Printer busy"));
    }
    if (bitRead(byte_read, 2)) { Serial.print(F(" / Image data full")); }
    if (bitRead(byte_read, 3)) { Serial.print(F(" / Unprocessed data")); }
    if (bitRead(byte_read, 4)) { Serial.print(F(" / Packet error")); }
    if (bitRead(byte_read, 5)) { Serial.print(F(" / Paper jam")); }
    if (bitRead(byte_read, 6)) { Serial.print(F(" / Other error")); }
    if (bitRead(byte_read, 7)) { Serial.print(F(" / Low battery")); }
  }
}

//checksum is always from the third to the last-4 bytes
void update_checksum(byte* packet, int start_index, int end_index, int checksum_pos)
{
  word checksum = 0;
  for (int i = start_index; i <= end_index; i++)
  {
    checksum += packet[i];
  }
  packet[checksum_pos]     = checksum & 0x00FF;  // low byte
  packet[checksum_pos + 1] = checksum >> 8;      // high byte
}

//checksum is always from the third to the last-4 bytes
void update_size(byte* packet, int size_pos, word data_size)
{
  packet[size_pos]     = data_size & 0x00FF;  // low byte
  packet[size_pos + 1] = data_size >> 8;      // high byte
}

void transmit_data_packet(byte* packet, word data_size)
{
  int size_start        = 4;
  int checksum_start    = 2;
  int checksum_end      = data_size + 5;   // Reserve 2 bytes for checksum
  int total_packet_size = data_size + 10;  // Safety margin for checksum + header/trailer if needed
  Serial.println();
  Serial.print(F("Updating DATA packet checksum for size "));
  Serial.print(data_size);
  update_size(packet, size_start, data_size);                               //size first as it is into the checksum
  update_checksum(packet, checksum_start, checksum_end, checksum_end + 1);  // update checksum at last so
  Serial.println();
  Serial.print(F("DATA packet sent --> "));
  send_printer_packet(packet, total_packet_size);  // Send complete packet
  Serial.println();
  Serial.flush();
}

void finalize_and_print()
{
  Serial.println();
  Serial.print(F("EMPT packet sent -->"));
  send_printer_packet(EMPT, 10);  // here we send a mandatory empty packet with 0 payload
  Serial.println();
  Serial.print(F("PRNT packet sent -->"));
  update_checksum(PRNT, 2, 9, 10);
  send_printer_packet(PRNT, 14);  // here we send the last printing command
  printing_loop();                // flux control
  Serial.println();
  Serial.flush();
}

//////////////////////////////////////IO stuff///////////////////////////////////////////////////
bool receiveAndStorePayload(uint8_t* buffer, size_t length)
{
  size_t received       = 0;
  unsigned long timeout = millis() + BUFFER_WAIT_TIME;

  while (received < length && millis() < timeout)
  {
    if (Serial.available())
    {
      buffer[received++] = Serial.read();
    }
  }

  // Wait for CR terminator
  while (millis() < timeout)
  {
    if (Serial.available())
    {
      char terminator = Serial.read();
      return terminator == '\r';
    }
  }
  return false;
}

void echoPacket(char type, uint8_t* buffer, size_t length)
{
  Serial.write(type);
  Serial.write(buffer, length);
  Serial.write('\r');  // Echo CR
}

void flushSerialInput()
{
  while (Serial.available()) Serial.read();
}
//////////////////////////////////////end of Printer stuff///////////////////////////////////////////
