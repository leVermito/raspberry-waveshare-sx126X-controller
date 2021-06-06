#!/usr/bin/python
# -*- coding: UTF-8 -*-

from waveshareSX126 import sx1268

import traceback

if __name__ == '__main__':
  
  # Listen for All messages on network 5 and print them to console
  # initialize hat with default parameters using ttyAMA0 serial
  # by default hat will be set to:  
  # address   : 0x0
  # networkID : 0x0
  # channel   : 0x0 
  # mode      : configuration
  controller = sx1268.Controller(serialPipe = "/dev/ttyAMA0")

  try:

    # set HAT address to be broadcast & monitor 0xFFFF
    controller.address = 0xFFFF

    # set HAT to operate on networkID 0x5
    controller.networkId = 0x5

    # set controller mode to Transmission
    controller.mode = sx1268.OperatingMode.Transmission

    # listen and print any messages with will come
    for message in controller.listen():
      print(message.decode())
    
  except:
    print(traceback.format_exc())
  finally:
    controller.cleanup()
