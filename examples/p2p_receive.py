#!/usr/bin/python
# -*- coding: UTF-8 -*-

import traceback
import time

from wssx126 import sx1268

if __name__ == '__main__':
  controller = sx1268.Controller()

  try:

    controller.initialize(serialPipe = "/dev/ttyAMA0")

    controller.channel = 0x1
    controller.address = 0xB8
    controller.networkId = 0x10

    controller.mode = sx1268.OperatingMode.Transmission

    start_time = time.time()
    numberOfBytes = 0


    while True:
      if controller.messageAvailable():
        for message in controller.readMessages():
          numberOfBytes += len(message)

      if time.time() - start_time > 1:
        print("\rB/s:", str(numberOfBytes).rjust(10), end='', flush=True)
        numberOfBytes = 0
        start_time = time.time()

  except:
    print(traceback.format_exc())
  finally:
    controller.cleanup()
