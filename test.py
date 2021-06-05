#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sx1268

import traceback

if __name__ == '__main__':

  hexPrint = lambda payload : print(' '.join(format(x, '02X') for x in payload))

  controller = sx1268.Controller()

  try:
    controller.mode = sx1268.OperatingMode.Configuration
    controller.writeRegister(register = sx1268.Registers.REG2)

    output = controller.readRegister(register = sx1268.Registers.REG2)
    hexPrint(output)

  except Exception:
    print(traceback.format_exc())
  finally:
    controller.cleanup()
