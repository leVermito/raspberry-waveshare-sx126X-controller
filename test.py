#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sx1268

import traceback

if __name__ == '__main__':

  hexPrint = lambda payload : print(' '.join(format(x, '02X') for x in payload))

  controller = sx1268.Controller()

  try:

    controller.address = 0xFFFF
    controller.channel = 0x1
    controller.networkId = 0x5
    controller.mode = sx1268.OperatingMode.Transmission

    ## run on RPI 0 
    for message in controller.listen():
      print(message)
    
    ## run on RPI 1
    controller.mode = sx1268.OperatingMode.Configuration
    controller.address = 0x0
    controller.mode = sx1268.OperatingMode.Transmission

    controller.sendMessage("I'm hunting for nazis behind The Moon!")
    ##

  except Exception:
    print(traceback.format_exc())
  finally:
    controller.cleanup()
