#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sx1268

import traceback

if __name__ == '__main__':

  hexPrint = lambda payload : print(' '.join(format(x, '02X') for x in payload))

  controller = sx1268.Controller()

  try:
    controller.channel = 0x1
    controller.networkId = 0x5

    ## run on RPI 0 - adress 0xFFFF is indicating broadcast mode - will catch all messages on network 5 on channel 1
    controller.address = 0xFFFF
    controller.mode = sx1268.OperatingMode.Transmission
    for message in controller.listen():
      print(message.decode())
    
    ## run on RPI 1 - simply send message on address 1 on network 5
    controller.address = 0x1
    controller.mode = sx1268.OperatingMode.Transmission
    controller.sendMessage("I'm hunting for nazis behind The Moon!")
    ##

  except Exception:
    print(traceback.format_exc())
  finally:
    controller.cleanup()
