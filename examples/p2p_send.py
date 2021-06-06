#!/usr/bin/python
# -*- coding: UTF-8 -*-

import time
import traceback

from waveshareSX126 import sx1268

if __name__ == '__main__':
  
  # Send P2P message on address 0xB8, network 0x5 every 5 seconds
  # initialize hat with default parameters using ttyAMA0 serial
  # by default hat will be set to:  
  # address   : 0x0
  # networkID : 0x0
  # channel   : 0x0 
  # mode      : configuration
  controller = sx1268.Controller(serialPipe = "/dev/ttyAMA0")

  try:

    # set HAT address to 0xB8
    controller.address = 0xB8

    # set HAT to operate on networkID 0x5
    controller.networkId = 0x5

    # set controller mode to Transmission
    controller.mode = sx1268.OperatingMode.Transmission

    start_time = time.time()
    i = 0

    while True:

      if time.time() - start_time > 10:
        controller.sendMessage(f"This is message number: {i}")
        start_time = time.time()
        i += 1

  except:
    print(traceback.format_exc())
  finally:
    controller.cleanup()