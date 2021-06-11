#!/usr/bin/python
# -*- coding: UTF-8 -*-

from wssx126 import sx1268

import time
import traceback

# Send P2P message every 5 seconds, listen on incoming P2P messages

if __name__ == '__main__':

  controller = sx1268.Controller()

  try:
    controller.initialize()

    # set HAT address to 0xB8
    controller.address = 0xB8

    # set HAT to operate on networkID 0x5
    controller.networkId = 0x5

    # set WOR mode to Sender (Send and Receive), and start operate in WOR mode
    controller.worMode = sx1268.WORMode.Sender
    controller.mode = sx1268.OperatingMode.Watch

    start_time = time.time()
    i = 0
    while True:

      if controller.messageAvailable():
        for message in controller.readMessages():
          print(message.decode())

      if time.time() - start_time > 5:
        controller.sendMessage(f"This is message number: {i}")
        start_time = time.time()
        i += 1

      time.sleep(0.1)

  except:
    print(traceback.format_exc())
  finally:
    controller.cleanup()
