## Raspberry waveshare sx1268 433Mhz controller + small tutorial

This guide is for people which are struggling to get WaveShare Raspberry hat with SX1268 (SX126X series) radio transceiver.  

`I will be using "HAT" as sx1268 waveshare rpi hat in the rest of this README`

## Instructions

### Step 0: Prepare raspberry for waveshare hat 

HAT is using GPIO 14 and GPIO 15 for UART communication.  
With that in mind we have to free up this line from other devices.  

#### Raspbian

*/boot/config.txt should contain:*

`enable_uart=1`

#### ArchlinuxARM
*/boot/config.txt should contain:*
`dtoverlay=uart0` 

`config.txt` can contain multiple dtoverlay definitions

```bash
# If you're struggling to install RPi.GPIO and you have GCC version > 10 use
export CFLAGS=-fcommon

pip install RPi.GPIO 
# or 
pip install waveshareSX126
```

### Step 1: Installation

```bash
pip install waveshareSX126
```


### Step 2: Examples!

#### Set HAT to 'monitor mode'
*Listen for All messages on network 5 and print them to console*

```python
from waveshareSX126 import sx1268

# initialize hat with default parameters using ttyAMA0 serial
# by default hat will be set to:  
# address   : 0x0
# networkID : 0x0
# channel   : 0x0 
# mode      : configuration
controller = sx1268.Controller(serialPipe = "/dev/ttyAMA0")

# set HAT address to be broadcast & monitor 0xFFFF
controller.address = 0xFFFF

# set HAT to operate on networkID 0x5
controller.networkId = 0x5

# set controller mode to Transmission
controller.mode = sx1268.OperatingMode.Transmission

# listen and print any messages with will come
for message in controller.listen():
  print(message.decode())
```

#### Send P2P message
*Send P2P message on address 0xB8, network 0x5 every 5 seconds*
```python
from waveshareSX126 import sx1268

import time

# initialize hat with default parameters using ttyAMA0 serial
# by default hat will be set to:  
# address   : 0x0
# networkID : 0x0
# channel   : 0x0 
# mode      : configuration
controller = sx1268.Controller(serialPipe = "/dev/ttyAMA0")

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

```
#### Operate in Watch / WOR mode.
*Send P2P message every 5 seconds, listen on incoming P2P messages*
```python
from waveshareSX126 import sx1268

import time

controller = sx1268.Controller()

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

```

