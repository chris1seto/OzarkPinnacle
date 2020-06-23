#!/usr/bin/env python3

import os
import sys
import tty
import argparse
import cstruct
import serial

PINNACLE_HEADER = '$'
PINNACLE_COMMAND_APRS_MESSAGE = 0x02

class AprsMessage(cstruct.CStruct):
  __byte_order__ = cstruct.LITTLE_ENDIAN
  __struct__ = """
  uint8_t destination_callsign[9];
  uint8_t message_size;
  uint8_t message[67];
  uint32_t message_number;
  """

def _string_to_byte_array(s):
  return [ord(c) for c in s]

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('-p', help='Port', action='store', dest='port', default='/dev/ttyACM0')
  parser.add_argument('-d', help='Destination Callsign', action='store', dest='destination_callsign', required=True)
  parser.add_argument('-m', help='APRS Message', action='store', dest='message', default='Ping!')
  parser.add_argument('-n', help='Message Number', action='store', dest='message_number', type=int, default=0)
  args = parser.parse_args()
  
  if (

  new_message = AprsMessage()
  new_message.destination_callsign = _string_to_byte_array(args.destination_callsign)
  new_message.message = _string_to_byte_array(args.message)
  new_message.message_size = len(new_message.message)
  new_message.message_number = args.message_number

  vcp_message = bytearray()

  # Header
  vcp_message.append(ord(PINNACLE_HEADER))
  vcp_message.append(PINNACLE_COMMAND_APRS_MESSAGE)

  # Message
  vcp_message.extend(new_message.pack())

  # Fake CRC
  vcp_message.append(0x55)
  vcp_message.append(0x55)

  print(vcp_message)

  # Write to serial
  board_vcp = serial.Serial(args.port, 115200)
  board_vcp.write(vcp_message)
  board_vcp.close()