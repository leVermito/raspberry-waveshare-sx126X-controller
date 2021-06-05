#!/usr/bin/python
# -*- coding: UTF-8 -*-

import RPi.GPIO as GPIO
import serial

import sys
import time
from enum import Enum


class Registers(Enum):
  ADDH = 0x0
  ADDL = 0x1
  NETID = 0x2
  REG0 = 0x3
  REG1 = 0x4
  REG2 = 0x5
  REG3 = 0x6
  CRYPT_H = 0x7
  CRYPT_L = 0x8
  PID = 0x9


class OperatingMode(Enum):
  Transmission = 0
  Watch = 1
  Configuration = 2
  DeepSleep = 3


class BaudRate(Enum):
  _1200    = 0
  _2400    = 1
  _4800    = 2
  _9600    = 3
  _19200   = 4
  _38400   = 5
  _57600   = 6
  _115200  = 7


class ParityBit(Enum):
  _8N1 = 0
  _8O1 = 1
  _8E1 = 2


class AirSpeed(Enum):
  _300    = 0
  _1200   = 1
  _2400   = 2
  _4800   = 3
  _9600   = 4
  _19200  = 5
  _38400  = 6
  _62500  = 7


class PacketSize(Enum):
  _240B = 0
  _128B = 1
  _64B  = 2
  _32B  = 3


class AmbientNoise(Enum):
  Disabled  = 0
  Enabled   = 1


class TransmitPower(Enum):
  _22dBm = 0
  _17dBm = 1
  _12dBm = 2
  _10dBm = 3


class RSSI(Enum):
  Disabled = 0
  Enabled = 1


class TransmittingMode(Enum):
  Transparent = 0
  PointToPoint = 1


class Relay(Enum):
  Disabled = 0
  Enabled = 1


class ListenBeforeTransmit(Enum):
  Disabled = 0
  Enabled = 1


class WORMode(Enum):
  Transmit = 0
  Sender = 1


class WORPeriod(Enum):
  _500ms = 0
  _1000ms = 1
  _1500ms = 2
  _2000ms = 3
  _2500ms = 4
  _3000ms = 5
  _3500ms = 6
  _4000ms = 7


class Exceptions:


  class CommunicationSerialPipeClosed(Exception):
    pass

  class AnswerMissMatch(Exception):
    pass

  class RegisterNotFound(Exception):
    pass


class Controller():
  def __init__(self, M0=22, M1=27, serialPipe = "/dev/ttyAMA0"):
    self.M0 = M0
    self.M1 = M1

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(True)
    GPIO.setup(self.M0,GPIO.OUT)
    GPIO.setup(self.M1,GPIO.OUT)

    self.serialPipe = serial.Serial(serialPipe, 9600)
    self.serialPipe.flushInput()
    self.serialPipe.flushOutput()

    ### Properties / Setters set

    ## device mode 
    self._mode = None

    ## Used for parameters tracking
    self._address = 0x0
    self._networkId = 0x0
    self._baudRate = BaudRate._9600
    self._parityBit = ParityBit._8N1
    self._airSpeed = AirSpeed._4800
    self._packetSize = PacketSize._240B
    self._ambientNoise = AmbientNoise.Disabled
    self._transmitPower = TransmitPower._22dBm
    self._channel = 0x0
    self._rssi = RSSI.Disabled
    self._transmittingMode = TransmittingMode.Transparent
    self._relay = Relay.Disabled
    self._listenBeforeTransmit = ListenBeforeTransmit.Disabled
    self._worMode = WORMode.Transmit
    self._worPeriod = WORPeriod._500ms
    self._encryptionKey = 0x0

    ## Used for register tracking
    # Read/Write
    self._ADDH   = 0x0
    self._ADDL   = 0x0
    self._NETID  = 0x0
    self._REG0   = 0x0
    self._REG1   = 0x0
    self._REG2   = 0x0
    self._REG3   = 0x0
    # Write
    self._CRYPT_H = 0x0
    self._CRYPT_L = 0x0

    ###

    # Read
    self._PID = 0x0

  @property
  def mode(self):
    return self._mode

  @mode.setter
  def mode(self, value : OperatingMode):
    if value == OperatingMode.Transmission:
      GPIO.output(self.M0,GPIO.HIGH)
      GPIO.output(self.M1,GPIO.HIGH)
    elif value == OperatingMode.Watch:
      GPIO.output(self.M0, GPIO.HIGH)
      GPIO.output(self.M1, GPIO.LOW)
    elif value == OperatingMode.Configuration:
      GPIO.output(self.M0,GPIO.LOW)
      GPIO.output(self.M1,GPIO.HIGH)
    elif value == OperatingMode.DeepSleep:
      GPIO.output(self.M0, GPIO.LOW)
      GPIO.output(self.M1, GPIO.LOW)
    time.sleep(1)
    self._mode = value

  @property
  def address(self):
    return self._address

  @address.setter
  def address(self, value):
    print('address setter')
    if value < 0x0 or value > 0xFFFF:
      raise ValueError("Address out of range 0x0 - 0xFFFF")
    
    self._address = value
    self.setRegisterFromParameter(Registers.ADDH)
    self.setRegisterFromParameter(Registers.ADDL)

  @property
  def networkId(self):
    return self._networkId
  
  @networkId.setter
  def networkId(self, value):
    if value < 0x0 or value > 0xFF:
      raise ValueError('NetworkID out of range 0x0 - 0xFF')

    self._networkId = value
    self.setRegisterFromParameter(Registers.NETID)
  
  @property
  def baudRate(self):
    return self._baudRate

  @baudRate.setter
  def baudRate(self, value):
    self._baudRate = value
    self.setRegisterFromParameter(Registers.REG0)

  @property
  def parityBit(self):
    return self._parityBit

  @parityBit.setter
  def parityBit(self, value : ParityBit):
    self._parityBit = value
    self.setRegisterFromParameter(Registers.REG0)

  @property
  def airSpeed(self):
    return self._airSpeed

  @airSpeed.setter
  def airSpeed(self, value : AirSpeed):
    self._airSpeed = value
    self.setRegisterFromParameter(Registers.REG0)

  @property
  def packetSize(self):
    return self._packetSize

  @packetSize.setter
  def packetSize(self, value : PacketSize):
    self._packetSize = value
    self.setRegisterFromParameter(Registers.REG1)

  @property
  def ambientNoise(self):
    return self._ambientNoise

  @ambientNoise.setter
  def ambientNoise(self, value : AmbientNoise):
    self._ambientNoise = value
    self.setRegisterFromParameter(Registers.REG1)

  @property
  def channel(self):
    return self._channel

  @channel.setter
  def channel(self, value : int):
    self._channel = value
    self.setRegisterFromParameter(Registers.REG2)

  @property
  def rssi(self):
    return self._rssi

  @rssi.setter
  def rssi(self, value : rssi):
    self._rssi = value
    self.setRegisterFromParameter(Registers.REG3)

  @property
  def transmittingMode(self):
    return self._transmittingMode

  @transmittingMode.setter
  def transmittingMode(self, value : TransmittingMode):
    self._transmittingMode = value
    self.setRegisterFromParameter(Registers.REG3)

  @property
  def relay(self):
    return self._relay

  @relay.setter
  def relay(self, value : Relay):
    self._relay = value
    self.setRegisterFromParameter(Registers.REG3)

  @property
  def listenBeforeTransmit(self):
    return self._listenBeforeTransmit

  @listenBeforeTransmit.setter
  def listenBeforeTransmit(self, value : ListenBeforeTransmit):
    self._listenBeforeTransmit = value
    self.setRegisterFromParameter(Registers.REG3)

  @property
  def worMode(self):
    return self._worMode

  @worMode.setter
  def worMode(self, value : worMode):
    self._worMode = value
    self.setRegisterFromParameter(Registers.REG3)
    
  @property
  def worPeriod(self):
    return self._worPeriod

  @worPeriod.setter
  def worPeriod(self, value : worPeriod):
    self._worPeriod = value
    self.setRegisterFromParameter(Registers.REG3)
    
  @property
  def encryptionKey(self):
    return self._encryptionKey

  @encryptionKey.setter
  def encryptionKey(self, value : int):
    if value < 0x0 or value > 0xFFFF:
      raise ValueError('Encryption key not in range 0x0 - 0xFFFF')
    self._encryptionKey = value
    self.setRegisterFromParameter(Registers.CRYPT_H)
    self.setRegisterFromParameter(Registers.CRYPT_L)

  def getRegisterValueByRegisterNumber(self, registerNumber : int):
    if registerNumber == 0x0:
      return self._ADDH
    elif registerNumber == 0x1:
      return self._ADDL
    elif registerNumber == 0x2:
      return self._NETID
    elif registerNumber == 0x3:
      return self._REG0
    elif registerNumber == 0x4:
      return self._REG1
    elif registerNumber == 0x5:
      return self._REG2
    elif registerNumber == 0x6:
      return self._REG3
    elif registerNumber == 0x7:
      return self._CRYPT_H
    elif registerNumber == 0x8:
      return self._CRYPT_L
    elif registerNumber == 0x9:
      return self._PID
    else:
      raise Exceptions.RegisterNotFound(f'Register {registerNumber} not found.')

  # set register value from corresponding parameters
  def setRegisterFromParameter(self, register : Registers):
    if register == Registers.ADDH:
      self._ADDH   = (self._address & 0xFF00) >> 8
    elif register == Registers.ADDL:
      self._ADDL   = self._address & 0xFF
    elif register == Registers.NETID:
      self._NETID  = self._networkId & 0xFF
    elif register == Registers.REG0:
      self._REG0   = 0xFF & (
        self._baudRate.value << 5 |
        self._parityBit.value << 3 |
        self._airSpeed.value
      )
    elif register == Registers.REG1:
      self._REG1   = 0xFF & (
        self._packetSize.value << 6 |
        self._ambientNoise.value << 5 |
        self._transmitPower.value
      )
    elif register == Registers.REG2:
      self._REG2   = 0x0
    elif register == Registers.REG3:
      self._REG3   = 0xFF & (
        self._rssi << 7 |
        self._transmittingMode.value << 6 |
        self._relay << 5 |
        self._listenBeforeTransmit.value << 4 |
        self._worMode.value << 3 |
        self._worPeriod.value
      )
    elif register == Registers.CRYPT_H:
      self._CRYPT_H = 0xFF00 & self._encryptionKey
    elif register == Registers.CRYPT_L:
      self._CRYPT_L = 0xFF & self._encryptionKey

  # # write configuration to physical register
  def writeRegister(self, register : Registers, writeWaitTime : float = 0.1):
    if self.serialPipe.isOpen():
      payload = bytearray([0xC0, register.value, 0x01, self.getRegisterValueByRegisterNumber(register.value)])
      self.serialPipe.write(payload)
      time.sleep(writeWaitTime)
    else:
      raise Exceptions.CommunicationSerialPipeClosed

    answer = bytearray(self.serialPipe.read(self.serialPipe.in_waiting))

    if len(answer) == 0 or answer[0] != 0xC1 or answer[1:] != payload[1:]:
      raise Exceptions.AnswerMissMatch(f'Payload: {payload}, Answer: {answer}')

  # read configuration from physical register
  def readRegister(self, register : Registers, writeWaitTime : float = 0.1):
    if self.serialPipe.isOpen():
      payload = bytearray([0xC1, register.value, 0x01])
      self.serialPipe.write(payload)
      time.sleep(writeWaitTime)
      output = bytearray(self.serialPipe.read(self.serialPipe.in_waiting))
      return output
    else:
      raise Exceptions.CommunicationSerialPipeClosed

  def cleanup(self):
    GPIO.cleanup()  
