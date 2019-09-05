
 /***********************************************************************************************//**
  * \file   main.c
  * \brief  BLEtest NCP host project for RF testing and manufacturing
 */
 /*******************************************************************************
 * @section License
 * <b>Copyright 2017 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.@n
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

/**
 * This an example application that demonstrates Bluetooth Smart peripheral
 * connectivity using BGLib C function definitions. It resets the device, then
 * reads the MAC address and executes a test command. The functionality can be used for
 * RF testing and manufacturing purposes. Please refer to the accompanying
 * README.txt for details.
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

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "gecko_bglib.h"
#include "uart.h"

#include "app.h"

#define TRUE   1u
#define FALSE  0u

BGLIB_DEFINE();

#define MAX_UART_PORT_STRING 30u

/* Use HW flow control? (compile time option) */
#define USE_RTS_CTS 0

/** The default serial port to use for BGAPI communication if port not specified on command line. */
static char *default_uart_port = "/dev/ttyAMA0";	//RPi target

/** The default baud rate to use if not specified on command line. */
uint32_t default_baud_rate = 115200;

/** The serial port to use for BGAPI communication. */
//static char *uart_port = NULL;
static char uart_port[MAX_UART_PORT_STRING] = "";

/** The baud rate to use. */
static uint32_t baud_rate = 0;


/**
 * Function called when a message needs to be written to the serial port.
 * @param msg_len Length of the message.
 * @param msg_data Message data, including the header.
 * @param data_len Optional variable data length.
 * @param data Optional variable data.
 */
static void on_message_send(uint32_t msg_len, uint8* msg_data)
{
    /** Variable for storing function return values. */
    int ret;

#ifdef _DEBUG
	printf("on_message_send()\n");
#endif /* DEBUG */

    ret = uartTx(msg_len, msg_data);
    if (ret < 0)
    {
        printf("on_message_send() - failed to write to serial port %s, ret: %d, errno: %d\n", uart_port, ret, errno);
        exit(EXIT_FAILURE);
    }
}

int hw_init(int argc, char* argv[])
{

	/**
	* Handle the command-line arguments.
	*/
	baud_rate = default_baud_rate;
	strcpy(uart_port, default_uart_port);

	if (appParseArguments(argc, argv, uart_port, sizeof(uart_port)))
	{
		appPrintUsage();
		exit(EXIT_FAILURE);
	}

    /**
    * Initialise the serial port.
    */
    return uartOpen((int8_t*)uart_port, baud_rate,USE_RTS_CTS,100);
}

/**
 * The main program.
 */
int main(int argc, char* argv[])
{
	struct gecko_cmd_packet *evt;


	/* register the signal interrupt handler */
	if (signal(SIGINT, sig_handler) == SIG_ERR)
	{
		printf("\ncError registering SIGINT\n");
	}

	/**
    * Initialize BGLIB with our output function for sending messages.
		* Using non-block for timeout functionality - need to use
		* gecko_peek_event().
  **/

	BGLIB_INITIALIZE_NONBLOCK(on_message_send, uartRx, uartRxPeek);

	if (hw_init(argc, argv) < 0)
  {
      printf("Hardware initialization failure, check serial port and baud rate values\n");
      exit(EXIT_FAILURE);
  }

	/* Init application */
	appInit();

	while (1)
  {
		/* Check for stack event. */
		evt = gecko_peek_event();

		/* Run application and event handler. */
		appHandleEvents(evt);

		/* Check timeout if active */
		if (appCheckTimeout())
		{
			/* timeout */
			printf("Timed out - exiting!\n");
			exit(EXIT_FAILURE);
		}
  }

    return -1;
}
