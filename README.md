BLEtest

/**
 * This an example application that demonstrates Bluetooth Smart peripheral
 * connectivity using BGLib C function definitions. It resets the device, then
 * reads the MAC address and executes a test command. The functionality can be used for
 * RF testing and manufacturing purposes.
 *
 * Most of the functionality in BGAPI uses a request-response-event pattern
 * where the module responds to each command with a response, indicating that
 * it has processed the command. Events which occur asynchonously or with non-
 * predictable timing may be sent from the module to the host at any time. For
 * more information, please see the WSTK BGAPI GPIO Demo Application Note.
 *
 * This is targeted to run on a Linux/Cygwin host. The BLE NCP can be connected to any /dev/ttyx
 * including hardware UARTs as well as USB virtual COM ports. This includes the WSTK development board
 * connected over USB (virtual COM) with an applicable NCP image programmed into the EFR32 module.
 */

Arguments:
 -v Print version number defined in application.
 -t <duration in ms>
 -m <payload/modulation type, 0:PBRS9, 1:11110000 packet payload, 2:10101010 packet payload, 3:unmodulated carrier>
 -p <power level in 0.1dBm steps>
 -u <UART port name>
 -c <channel, 2402 MHz + 2*channel>
 -l <packet length, ignored for unmodulated carrier>
 -r DTM receive test. Prints number of received DTM packets.
 -x <16-bit crystal tuning value, e.g. 0x0136>
 -a <48-bit MAC address, e.g. 01:02:03:04:05:06>
 -z Read 16-bit crystal tuning value
 -f Read FW revision string from Device Information (GATT)
 -e Enter a fast advertisement mode with scan response for TIS/TRP chamber testing
 -d Run as a daemon process.
 Example - transmit PRBS9 payload of length=25 for 10 seconds on 2402 MHz at 5.5dBm output power level on device connected to serial port /dev/ttyAMA0 :
    BLEtest -t 10000 -m 0 -p 55 -u /dev/ttyAMA0 -c 0 -l 25


Examples:

1. Transmit an unmodulated carrier for 10 seconds on 2402 at 5.0 dBm output power level on the device connected to serial port /dev/ttyAMA0:

	$ ./exe-host/BLEtest -t 10000 -m 3 -p 50 -u /dev/ttyAMA0 -c 0

	------------------------
	Waiting for boot pkt...
	boot pkt rcvd: gecko_evt_system_boot(1, 0, 2, 755, 0, 1)
	MAC address: 00:0B:57:07:6C:04
	Outputting modulation type 3 for 10000 ms at 2402 MHz at 5.0 dBm
	Test completed!
	$

2. Transmit DTM (direct test mode) packets containing a pseudorandom PRBS9 payload of length=25 for 10 seconds on 2404 MHz at 8dBm output power level on device connected to serial port /dev/ttyAMA0
	$ ./exe-host/BLEtest -t 10000 -m 0 -p 80 -u /dev/ttyAMA0 -c 1 -l 25

	------------------------
	Waiting for boot pkt...
	boot pkt rcvd: gecko_evt_system_boot(1, 0, 2, 755, 0, 1)
	MAC address: 00:0B:57:07:6C:04
	Outputting modulation type 0 for 10000 ms at 2404 MHz at 8.0 dBm
	Test completed!
	$

3. Receive DTM (direct test mode) packets for 10 seconds on 2404 MHz on device connected to serial port /dev/ttyAMA0. Note that the printout below shows an example of 100% packet reception rate when running example #2 (PRBS9 DTM length=25) on a separate device.
	$ ./exe-host/BLEtest -t 20000 -r -u /dev/ttyAMA0 -c 1

	------------------------
	Waiting for boot pkt...
	boot pkt rcvd: gecko_evt_system_boot(1, 0, 2, 755, 0, 1)
	MAC address: 00:0B:57:07:6C:04
	DTM receive enabled, freq=2404 MHz
	DTM receive completed. Number of packets received: 16214
	$

4. Program custom MAC address 01:02:03:04:05:06 on device connected to virtual COM port /dev/ttyACM0:
	$ ./exe-host/BLEtest -u /dev/ttyACM0 -a 01:02:03:04:05:06

	------------------------
	Waiting for boot pkt...
	boot pkt rcvd: gecko_evt_system_boot(1, 0, 2, 755, 0, 1)
	MAC address: 00:0B:57:00:E5:01
	Writing MAC address: 01:02:03:04:05:06
	Rebooting with new MAC address...
	boot pkt rcvd: gecko_evt_system_boot(1, 0, 2, 755, 0, 1)
	MAC address: 01:02:03:04:05:06
	Outputting modulation type 3 for 1000 ms at 2402 MHz at 5.0 dBm
	Test completed!
	$

5. Write crystal tuning value of 0x0155 to device connected to virtual COM port /dev/ttyACM0 and output test tone of 2402 MHz at 5.0 dBm for 10 seconds to verify frequency using a spectrum analyzer (set the instrument to 500 kHz bandwidth).
	$ ./exe-host/BLEtest -u /dev/ttyACM0 -x 0x0155 -m 3 -t 10000 -c 0 -p 50

	------------------------
	Waiting for boot pkt...
	boot pkt rcvd: gecko_evt_system_boot(1, 0, 2, 755, 0, 1)
	MAC address: 00:0B:57:00:E5:01
	Writing ctune value to 0x0155
	Rebooting with new ctune value...
	boot pkt rcvd: gecko_evt_system_boot(1, 0, 2, 755, 0, 1)
	MAC address: 00:0B:57:00:E5:01
	Outputting modulation type 3 for 10000 ms at 2402 MHz at 5.0 dBm
	Test completed!
	$

  BUILD INSTRUCTIONS
  To build:
  * Install Simplicity Studio v4 from SiLabs.com
  * From within Studio v4, install the latest BLE stack (latest tested / compatible version v2.3)
  * Copy BLEtest folder to "sdks/gecko_sdk_suite/v1.0/app/bluetooth_2.3/examples_ncp_host/BLEtest" (sdk directory within Simplicity Studio install directory tree by default)

  For cross compilation, modify the makefile at "sdks/gecko_sdk_suite/v1.0/app/bluetooth_2.3/examples_ncp_host/BLEtest/makefile" for your target host and execute make.

  To make on a target (tested on the Raspberry Pi):
  *Start at the root directory of the stack (sdks/gecko_sdk_suite/v1.0/) and make a tarball as follows:
  *tar -cvf ble_2_3_0.tgz "protocol/bluetooth_2.3/ble_stack/inc" "protocol/bluetooth_2.3/ble_stack/src" "app/bluetooth_2.3/examples_ncp_host/"
  *Extract on target
  *cd to "sdks/gecko_sdk_suite/v1.0/app/bluetooth_2.0/examples_ncp_host/"
  *make
