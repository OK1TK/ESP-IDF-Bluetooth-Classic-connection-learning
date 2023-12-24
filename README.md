Based on the sample codes BT_SPP_INITIATOR and BT_SPP_ACCEPTOR, I have created a code for connecting two ESP32 boards without the need to enter a pairing PIN code, and overall, I tried to simplify the code for easier use in my future project. Currently, the code for connecting both ESP32 boards should send a message from MASTER to SLAVE and from SLAVE to MASTER. These messages should be displayed in the terminal.

Problems: 
1. After receiving messages, the boards go into power-saving mode, and the connection is not maintained.
2. Strange behavior of MASTER during attempts to reconnect.
3. In the CMakeList.txt file in the main directory, for both MASTER and SLAVE, it is necessary to add the following line under INCLUDE_DIRS "."): target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format).
