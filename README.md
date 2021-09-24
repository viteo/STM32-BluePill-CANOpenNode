CANOpenNode on STM32F103 (BluePill)
==========

CANopen stack running on STM32F103 microcontroller.

It is based on [CANopenNode](https://github.com/CANopenNode/CANopenNode), which is free and open source CANopen Stack and is included as a git submodule.

CANopen is the internationally standardized (EN 50325-4) ([CiA301](http://can-cia.org/standardization/technical-documents)) CAN-based higher-layer protocol for embedded control system. For more information on CANopen see http://www.can-cia.org/.


Using on STM32 BluePill
------------------------------------------
Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

STM32 programmer is required.

File -> Open Projects from File System... -> Directory

Program is tested on STM32 BluePill board with STM32F103c8 chip. CAN transceiver chip must be connected to the board.

By default device uses Object Dictionary based on `CANopenNode/example`, which contains only communication parameters.


### STM32 BluePill
- Add CAN transciever (MCP2551, SN65HVD23x or similar) to the board. See example schema below.

            +-------------+                +----------------+
          5-| NC      RXD |-4-----------32-| PA11           |
          6-| CAN_L   VCC |-3-------+-3.3V-| 3.3V out       |
          7-| CAN_H   GND |-2--+    |      |                |
          8-| NC      TXD |-1--|----|---33-| PA12           |
            +-------------+    |    |      |                |
             VP232             |  100nF    |                |
                               |    |      |                |
                               +----+--GND-| GND            |
                                           +----------------+
                                            BluePill

- EEprom chip is not used.
- Default CAN bitrate is 500kbps and CANopen NodeId is 0x0A. See main_blank.c file.
- CANOpenNode driver implementation is in ./device/co_driver folder
