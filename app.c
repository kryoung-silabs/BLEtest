/***********************************************************************************************//**
 * \file   app.c
 * \brief  Event handling and application code for BLEtest example
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* standard library headers */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>

#include "uart.h"

/* BG stack headers */
#include "bg_types.h"
#include "gecko_bglib.h"

/* Own header */
#include "app.h"

#define VERSION_MAJ	1u
#define VERSION_MIN	6u

#define TRUE   1u
#define FALSE  0u

#define PHY test_phy_1m	//always use 1Mbit PHY for now

/* Program defaults - will use these if not specified on command line */
#define DEFAULT_DURATION		1000*1000u	//1 second
#define DEFAULT_MODULATION		test_pkt_carrier	//unmodulated/carrier
#define DEFAULT_CHANNEL			0u	//2.402 GHz
#define DEFAULT_POWER_LEVEL		50u	//5 dBm
#define DEFAULT_PACKET_LENGTH	25u
#define CMP_LENGTH	2u

/**
 * Configurable parameters that can be modified to match the test setup.
 */

/* The modulation type */
static enum test_packet_type mod_type=DEFAULT_MODULATION;

/* Modulation types - arguments based on "old" API but keeping to avoid Changing
 procedures */
#define ARG_MOD_TYPE_PRBS9_PKTS		0
#define ARG_MOD_TYPE_11110000			1
#define ARG_MOD_TYPE_10101010			2
#define ARG_MOD_TYPE_CW						3
#define ARG_MOD_TYPE_11111111			4
#define ARG_MOD_TYPE_00000000			5
#define ARG_MOD_TYPE_00001111			6
#define ARG_MOD_TYPE_01010101			7
#define ARG_MOD_TYPE_PN9					8 /* continuous modulation */

#define EXIT_TIMEOUT_SEC 5u
#define BOOT_TIMEOUT_SEC 5u

#define USLEEP_MAX	1000000u

/* The power level */
static uint16_t power_level=DEFAULT_POWER_LEVEL;

/* The duration in us */
static uint32_t duration_usec=DEFAULT_DURATION;

/* channel */
static uint8_t channel=DEFAULT_CHANNEL;

/* packet length */
static uint8_t packet_length=DEFAULT_PACKET_LENGTH;

/* run as daemon? */
static uint8_t daemon_flag = 0;

/* Wifi coex / PTA enabled? */
static uint8_t pta_enable = FALSE;

/* Infinite mode */
static uint8_t infinite_mode = FALSE;

/* Timeout variables */
static uint32_t timeout_sec;
static uint8_t timeout_active_flag = FALSE;
static time_t timeout_start_val;
static time_t timeout_last_sec;

/* advertising test mode */
#define TEST_ADV_INTERVAL_MS 50
#define ADV_INTERVAL_UNIT 0.625 //per API guide
#define CHANNEL_MAP_ALL 0x07 //all channels

/* ADV data for TIS/TRP test advertisement */
/* LE general discoverable, FIND ME service (0x1802) */
static uint8_t adv_data[] = {0x02, 0x01, 0x06, 0x03, 0x03, 0x02, 0x18};

/* 20 byte length, 0x09 (full name), "Blue Gecko Test App" */
static uint8_t scan_rsp_data[] = {20,0x09,0x42,0x6c,0x75,0x65,0x20,0x47,0x65,0x63,0x6b,0x6f,0x20,0x54,0x65,0x73,0x74,0x20,0x41,0x70,0x70};

/* flash write state machine */
static enum ps_states {
	ps_none,
	ps_write_mac,
	ps_write_ctune,
	ps_read_ctune,
	ps_read_gatt_fwversion //note that this isn't really a PS command, but we'll leave it here
} ps_state;

/* application state machine */
static enum app_states {
	adv_test,
	dtm_begin,
	dtm_started,
	default_state
} app_state;

/* dtm modes */
static enum dtm_modes {
	dtm_mode_rx,
	dtm_mode_pkt_tx,
	dtm_mode_cont_tx
} dtm_mode;

#define MAC_PSKEY_LENGTH	6u
#define CTUNE_PSKEY_LENGTH	2u
#define MAX_CTUNE_VALUE 511u

#define GATT_FWREV_LO 0x26
#define GATT_FWREV_HI 0x2A
#define FWREV_TYPE_LEN 2
static uint8_t fwrev_type_data[FWREV_TYPE_LEN] = {GATT_FWREV_LO,GATT_FWREV_HI};

static uint16_t ctune_value=0; //unsigned int representation of ctune
static uint8_t ctune_array[CTUNE_PSKEY_LENGTH]; //ctune value to be stored (little endian)
static uint8_t mac_address[MAC_PSKEY_LENGTH]; //mac address to be stored

/* Store NCP version information */
static uint8_t version_major;
static uint8_t version_minor;

// App booted flag
static uint8_t appBooted = FALSE;

/***********************************************************************************************//**
 *  \brief  Event handler function.
 *  \param[in] evt Event pointer.
 **************************************************************************************************/
void appHandleEvents(struct gecko_cmd_packet *evt)
{
  void *rsp;

  if (NULL == evt) {
    return;
  }

  // Do not handle any events until system is booted up properly.
  if ((BGLIB_MSG_ID(evt->header) != gecko_evt_system_boot_id)
      && !appBooted) {
#if defined(DEBUG)
    printf("Event: 0x%04x\n", BGLIB_MSG_ID(evt->header));
#endif
    usleep(50000);
    return;
  }

  /* Handle events */
  switch (BGLIB_MSG_ID(evt->header)) {
    // SYSTEM BOOT (power-on/reset)
          case gecko_evt_system_boot_id:
          appBooted = TRUE;
  				timeout_active_flag = FALSE; //disable timeout

  				/* Store version info */
  				version_major = evt->data.evt_system_boot.major;
  				version_minor = evt->data.evt_system_boot.minor;

  				printf("\nboot pkt rcvd: gecko_evt_system_boot(%d, %d, %d, %d, 0x%8x, %d)\n",
  					version_major,
  					version_minor,
  					evt->data.evt_system_boot.patch,
  					evt->data.evt_system_boot.build,
  					evt->data.evt_system_boot.bootloader,
  					evt->data.evt_system_boot.hw
  				);

  			/* Read and print MAC address */
  			rsp=gecko_cmd_system_get_bt_address();
  			printf("MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
  				((struct gecko_msg_system_get_bt_address_rsp_t *)rsp)->address.addr[5], // <-- address is little-endian
  				((struct gecko_msg_system_get_bt_address_rsp_t *)rsp)->address.addr[4],
  				((struct gecko_msg_system_get_bt_address_rsp_t *)rsp)->address.addr[3],
  				((struct gecko_msg_system_get_bt_address_rsp_t *)rsp)->address.addr[2],
  				((struct gecko_msg_system_get_bt_address_rsp_t *)rsp)->address.addr[1],
  				((struct gecko_msg_system_get_bt_address_rsp_t *)rsp)->address.addr[0]
  				);

  				/* Go ahead and manage PTA/coexistence */
  				if (pta_enable == TRUE)
  	      {
  	        rsp = gecko_cmd_coex_set_options(coex_option_enable, coex_option_enable);
  	        printf("Enabling PTA, status 0x%2x ", ((struct gecko_msg_coex_set_options_rsp_t *)rsp)->result);
  					if (((struct gecko_msg_coex_set_options_rsp_t *)rsp)->result == bg_err_success)
  					{
  						 printf("(success)\n");
  					} else
  					{
  						 printf("(error)\n");
  					}
  	      }
  	      else
  	      {
  					/* This is the default - don't print any status messages. If not
							supported by the NCP, it's OK */

					 /* Version 1.x NCPs don't respond to this command at all, which hangs up
					 the app waiting for the response. Also need to try early 2.x versions and
					 possibly also check the minor */
						if (version_major >= 2) //TODO: May need to check version minor too
						{
							gecko_cmd_coex_set_options(coex_option_enable, FALSE);
						}

  	      }


  			/* Run test commands */
  			/* deal with flash writes first, since reboot is needed to take effect */
  			if(ps_state == ps_write_ctune) {
  				/* Write ctune value and reboot */
  				printf("Writing ctune value to 0x%04x\n", ctune_value);
          gecko_cmd_flash_ps_save(FLASH_PS_KEY_CTUNE, CTUNE_PSKEY_LENGTH, ctune_array); //write out key
  				ps_state = ps_none; /* reset state machine */
  				rsp = gecko_cmd_system_reset(0); /* reset to take effect */
  				printf("Rebooting with new ctune value...\n");
  			} else if (ps_state == ps_read_ctune)
  			{
  				/* read the PS key for CTUNE */
  				printf("Reading flash PS value for CTUNE (%d)\n", FLASH_PS_KEY_CTUNE);
          rsp = gecko_cmd_flash_ps_load(FLASH_PS_KEY_CTUNE);
  				errorcode_t err;
  				err = ((struct gecko_msg_flash_ps_load_rsp_t *)rsp)->result;
  				if (err == bg_err_hardware_ps_key_not_found)
  				{
  					printf("CTUNE value not loaded in PS flash!\n");
  				} else {

  					/* CTUNE is stored in ps key as little endian with LSB first. Need to print correctly
  					as uint16 depending on endianness of host processor. NOTE: Not tested on big endian host */
  					/* Test for a little-endian machine */
  					#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  						/* ctune_value is little endian - LSB goes in first (low) byte */
  						ctune_array[0] = ((struct gecko_msg_flash_ps_load_rsp_t *)rsp)->value.data[0];
  						ctune_array[1] = ((struct gecko_msg_flash_ps_load_rsp_t *)rsp)->value.data[1];
  					#else
  						/* ctune_value is big endian - LSB goes in second (high) byte */
  						ctune_array[1] = ((struct gecko_msg_flash_ps_load_rsp_t *)rsp)->value.data[0];
  						ctune_array[0] = ((struct gecko_msg_flash_ps_load_rsp_t *)rsp)->value.data[1];
  					#endif
  					printf("Stored CTUNE = 0x%04x\nRebooting...\n", (uint16_t)(ctune_array[0] | ctune_array[1] << 8));
  				}
  					/* Reset again and proceed with other commands */
  					ps_state = ps_none; /* reset state machine */
  					rsp = gecko_cmd_system_reset(0); /* reset to take effect */

  			} else if (ps_state == ps_read_gatt_fwversion)
  			{
  				/* Find the firmware revision string (UUID 0x2a26) in the local GATT and print it out */
  				rsp = gecko_cmd_gatt_server_find_attribute(0,FWREV_TYPE_LEN,fwrev_type_data);
  				errorcode_t err;
  				err = ((struct gecko_msg_gatt_server_find_attribute_rsp_t *)rsp)->result;
  				if (err == bg_err_att_att_not_found)
  				{
  						printf("Firmware revision string not found in the local GATT.\nRebooting...\n");
  				} else
  				{
  					uint16_t attribute = ((struct gecko_msg_gatt_server_find_attribute_rsp_t *)rsp)->attribute;
  					printf("Found firmware revision string at handle %d\n",attribute);

  					/* This is the FW revision string from the GATT - load it and print it as an ASCII string */
  					rsp = gecko_cmd_gatt_server_read_attribute_value(attribute,0); //read from the beginning (offset 0)
  					uint8_t value_len = ((struct gecko_msg_gatt_server_read_attribute_value_rsp_t *)rsp)->value.len;
  					uint8 i = 0;
  					printf("FW Revision string (length = %d): ",value_len);
  					for (i = 0; i < value_len; i++)
  					{
  						printf("%c",((struct gecko_msg_gatt_server_read_attribute_value_rsp_t *)rsp)->value.data[i]);
  					}
  					printf("\n");
  				}

  				/* Reset again and proceed with other commands (not flash commands)*/
  				ps_state = ps_none; /* reset state machine */
  				rsp = gecko_cmd_system_reset(0); /* reset to take effect */
  			}
  			else if (ps_state == ps_write_mac) {
  				/* write MAC value and reboot */
  				printf("Writing MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
  				mac_address[5], // <-- address is little-endian
  				mac_address[4],
  				mac_address[3],
  				mac_address[2],
  				mac_address[1],
  				mac_address[0]
  				);
  				gecko_cmd_flash_ps_save(FLASH_PS_KEY_LOCAL_BD_ADDR, MAC_PSKEY_LENGTH, (uint8 *)&mac_address); //write out key
  				ps_state = ps_none; /* reset state machine */
  				rsp = gecko_cmd_system_reset(0); /* reset to take effect */
  				printf("Rebooting with new MAC address...\n");
  			}
  			else if ((app_state == dtm_begin) && (dtm_mode == dtm_mode_rx))
  			{
          printf("DTM receive enabled, freq=%d MHz\n",2402+(2*channel));
  				if (infinite_mode == TRUE)
  				{
  					printf("Infinite mode - ctrl-C to exit\n");
  				}
          gecko_cmd_test_dtm_rx(channel,PHY);
          if (infinite_mode == FALSE)
  				{
						/* sleep during test */
						while (duration_usec > USLEEP_MAX)
						{
							usleep(USLEEP_MAX);
							duration_usec -= USLEEP_MAX;
						}

  					usleep(duration_usec);  /* sleep remainder */
  					gecko_cmd_test_dtm_end();
  				}

        }
  			else if (app_state == adv_test)
  			{
  				/* begin advertisement test */
  				printf("\n--> Starting fast advertisements for antenna pattern testing\n");
  				rsp = gecko_cmd_system_set_tx_power(power_level);
  				uint8_t ret_power_level = ((struct gecko_msg_system_set_tx_power_rsp_t *)rsp)->set_power;
  				printf("Attempted power setting of %.1f dBm, actual setting %.1f dBm\n",(float)power_level/10,(float)ret_power_level/10);
  				printf("Press 'control-c' to end...\n");
          rsp = gecko_cmd_le_gap_set_adv_data(0, sizeof(adv_data),adv_data); //advertising data
          rsp = gecko_cmd_le_gap_set_adv_data(1, sizeof(scan_rsp_data), scan_rsp_data); //scan response data
          rsp = gecko_cmd_le_gap_set_adv_parameters(TEST_ADV_INTERVAL_MS/ADV_INTERVAL_UNIT,TEST_ADV_INTERVAL_MS/ADV_INTERVAL_UNIT,CHANNEL_MAP_ALL); //advertise on all channels at specified interval
          rsp = gecko_cmd_le_gap_set_mode(le_gap_user_data,le_gap_undirected_connectable );

  			}
  			else if (ps_state == ps_none) {
					/* Handle printing of status */
  				printf("Outputting modulation type %d ",mod_type);
					if (infinite_mode == FALSE)
					{
						/* there is a time duration - print it */
						printf("for %d ms ",duration_usec/1000);
					}

					/* Print frequency and power level */
					printf("at %d MHz at %.1f dBm ",2402+(2*channel),(float)power_level/10);

					if (dtm_mode == dtm_mode_pkt_tx)
					{
						/* packet mode - length has meaning */
						printf("with packet length %d\n",packet_length);
					} else
					{
						printf("\n");
					}

					if (infinite_mode == TRUE)
					{
						printf("\nInfinite mode - ctrl-C to exit\n");
					}

  				/*Run test commands */
  				gecko_cmd_system_set_tx_power(power_level);
  				gecko_cmd_test_dtm_tx(mod_type,packet_length,channel,PHY);
  				if (infinite_mode == FALSE)
  				{
						/* sleep during test */
						while (duration_usec > USLEEP_MAX)
						{
							usleep(USLEEP_MAX);
							duration_usec -= USLEEP_MAX;
						}

						usleep(duration_usec);  /* sleep remainder */
  					gecko_cmd_test_dtm_end();
  				}
  			}
  		break;

  		case gecko_evt_test_dtm_completed_id:
  				if (app_state == dtm_begin)
  				{
  						//This is just an acknowledgement of the DTM start - set a flag for the next event which is the end
  						app_state = dtm_started;
  				} else if ((dtm_mode == dtm_mode_rx) && (app_state == dtm_started))
  				{
  					//This is the event received at the end of the test
  					printf("DTM receive completed. Number of packets received: %d\n",evt->data.evt_test_dtm_completed.number_of_packets);
  					exit(1);	//test done - terminate
  				} else if ((dtm_mode == dtm_mode_pkt_tx) && (app_state == dtm_started))
  				{
  					//This is the event received at the end of the test
  					printf("DTM transmit completed. Number of packets transmitted: %d\n",evt->data.evt_test_dtm_completed.number_of_packets);
  					exit(1);	//test done - terminate
  				}
  			  else { 		//continuous dtm tx (stream, CW)
  					/* Exit after DTM TX is completed */
  					printf("Test completed!\n");
  					exit(1); //test done - terminate
  			}

  			break;
  			case gecko_evt_dfu_boot_id:
  					/* DFU boot detected  */
  					printf("DFU Boot, Version 0x%x\n",evt->data.evt_dfu_boot.version);
  					break;
  			case gecko_evt_le_connection_closed_id:
  					/* Connection has closed - resume advertising if in if app_state == adv_test */
  					printf("Disconnected from central\n");
  					if (app_state == adv_test)
  					{
  						rsp = gecko_cmd_le_gap_set_mode(le_gap_user_data,le_gap_undirected_connectable );
  					}
  					break;
  			case gecko_evt_dfu_boot_failure_id:
  					/* DFU boot failure detected */
  					printf("DFU boot failure detected, reason 0x%2x: ",evt->data.evt_dfu_boot_failure.reason);
  					/* Print a more useful reason */
  					switch (evt->data.evt_dfu_boot_failure.reason)
  					{
  						case bg_err_security_image_signature_verification_failed:
  							printf("device firmware signature verification failed.\n");
  							break;
  						case bg_err_security_file_signature_verification_failed:
  							printf("bootloading file signature verification failed.\n");
  							break;
  						case bg_err_security_image_checksum_error:
  							printf("device firmware checksum is not valid.\n");
  							break;
  						default:
  							printf("\n"); //add a lf when no detailed reason provided.
  							break;
  					}
  					printf("Run the uart_dfu application to update the firmware.\n");
  					exit(1); //terminate
  			default:
  		       printf("Unhandled event received, event ID: 0x%2x\n", BGLIB_MSG_ID(evt -> header));
  		      break;
  }
}

void appPrintUsage(void)
{
	printf("\n Arguments: \n");
	printf("-v Print version number defined in application.\n");
	printf("-t <duration in ms>. Use 0 for infinite mode, terminated by control-C.\n");
	printf("-m <payload/modulation type, 0:PRBS9 packets, 1:0xF0 packets, 2:0xAA packets, 3:unmodulated carrier, 4:0xFF packets, 5:0x00 packets, 6:0x0F packets, 7:0x55 packets, 8:Continuous PN9 stream>\n");
	printf("-p <power level in 0.1dBm steps>\n");
	printf("-u <UART port name>\n");
	printf("-c <channel, 2402 MHz + 2*channel>\n");
	printf("-l <packet length, ignored for unmodulated carrier>\n");
	printf("-r DTM receive test. Prints number of received DTM packets.\n");
	printf("-x <16-bit crystal tuning value, e.g. 0x0136>\n");
	printf("-a <48-bit MAC address, e.g. 01:02:03:04:05:06>\n\n");
	printf("-z Read 16-bit crystal tuning value\n");
	printf("-f Read FW revision string from Device Information (GATT)\n");
	printf("-e Enter a fast advertisement mode with scan response for TIS/TRP chamber antenna pattern testing. Overrides -t setting (runs until process is killed).\n");
	printf("-d Run as a daemon process.\n");
	printf("-w Enable WiFi coexistence / PTA.\n");
	printf("\nExample - transmit PRBS9 payload of length=25 for 10 seconds on 2402 MHz at 5.5dBm output power level on device connected to serial port /dev/ttyAMA0 :\n\tBLEtest -t 10000 -m 0 -p 55 -u /dev/ttyAMA0 -c 0 -l 25\n\n\n");
}

uint8_t appParseArguments(int argc, char* argv[], char uart_port_arg[], size_t uart_port_str_len)
{
  int argCount = 1;  /* Used for parsing command line arguments */
	char *temp;
	int values[8];
	uint8_t i;

  /* Init state machines */
  ps_state = ps_none;

	/* Default for dtm tx unless otherwise specified */
  app_state = dtm_begin;
	dtm_mode = dtm_mode_cont_tx;

  /* Let's go ahead and start parsing the command line arguments */
  if (argc > 1)
  {
	  if ((!strncasecmp(argv[1],"-h",CMP_LENGTH)) || (!strncasecmp(argv[1],"-?",CMP_LENGTH)))
	  {
		return TRUE;
	  }
  }

  while (argCount < argc)
  {
		if(!strncasecmp(argv[argCount],"-t",CMP_LENGTH))
		{
			duration_usec = atoi(argv[argCount+1])*1000;
			if (duration_usec == 0)
			{
				/* infinite mode */
				infinite_mode = TRUE;
			}
		} else if(!strncasecmp(argv[argCount],"-m",CMP_LENGTH))
		{
			/* modulation type */
				switch (atoi(argv[argCount+1])) {
					case ARG_MOD_TYPE_PRBS9_PKTS:
						mod_type = test_pkt_prbs9;
						dtm_mode = dtm_mode_pkt_tx;
						break;
					case ARG_MOD_TYPE_11110000:
						mod_type = test_pkt_11110000;
						dtm_mode = dtm_mode_pkt_tx;
						break;
					case ARG_MOD_TYPE_10101010:
						mod_type = test_pkt_10101010;
						dtm_mode = dtm_mode_pkt_tx;
						break;
					case ARG_MOD_TYPE_CW:
						mod_type = test_pkt_carrier;
						dtm_mode = dtm_mode_cont_tx;
						break;
					case ARG_MOD_TYPE_11111111:
						mod_type = test_pkt_11111111;
						dtm_mode = dtm_mode_pkt_tx;
						break;
					case ARG_MOD_TYPE_00000000:
						mod_type = test_pkt_00000000;
						dtm_mode = dtm_mode_pkt_tx;
						break;
					case ARG_MOD_TYPE_00001111:
						mod_type = test_pkt_00001111;
						dtm_mode = dtm_mode_pkt_tx;
						break;
					case ARG_MOD_TYPE_01010101:
						mod_type = test_pkt_01010101;
						dtm_mode = dtm_mode_pkt_tx;
						break;
					case ARG_MOD_TYPE_PN9:
						mod_type = test_pkt_pn9;
						dtm_mode = dtm_mode_cont_tx;
						break;
					default:
						printf("%d is not a valid modulation type.\n\n", atoi(argv[argCount+1]));
				 		return(TRUE);
						break;
			}

		} else if(!strncasecmp(argv[argCount],"-p",CMP_LENGTH))
		{
			power_level = atoi(argv[argCount+1]);
			if (power_level>200)
			{
			  printf("Error in power level: max value 200 (20.0 dBm)\n\n");
			  return(TRUE);
			}
		} else if(!strncasecmp(argv[argCount],"-u",CMP_LENGTH))
		{
      size_t arg_len = sizeof(argv[argCount+1]);
      if (arg_len <= uart_port_str_len)
      {
        strcpy(uart_port_arg,argv[argCount+1]);
      } else
      {
        printf("Error in uart port parameter!\n\n");
        return TRUE;
      }
      //uart_port_arg = (char *)argv[argCount+1];
		}
		else if(!strncasecmp(argv[argCount],"-c",CMP_LENGTH))
		{
			channel = atoi(argv[argCount+1]);
			if (channel>39)
			{
				printf("Error in channel: max value 39\n\n");
				return(TRUE);
			}
		}
		else if(!strncasecmp(argv[argCount],"-l",CMP_LENGTH))
		{
			packet_length = atoi(argv[argCount+1]);
			if (packet_length>255)
			{
			  printf("Error in packet length: max value 46\n\n");
			  return(TRUE);
			}
		}
		else if(!strncasecmp(argv[argCount],"-r",CMP_LENGTH))
		{
			/* DTM receive */
			app_state = dtm_begin;
			dtm_mode = dtm_mode_rx;

			/* Receive infinitely */
			infinite_mode = TRUE;
		}
		else if(!strncasecmp(argv[argCount],"-e",CMP_LENGTH))
		{
			/* advertise test for TIS / TRP */
			app_state = adv_test;
		}
		else if(!strncasecmp(argv[argCount],"-x",CMP_LENGTH))
		{
			/* xtal ctune */
			ctune_value = (uint16_t) strtoul(argv[argCount+1],&temp,0);
			/* check range */
			if (ctune_value > MAX_CTUNE_VALUE)
			{
					printf("Ctune value entered (0x%2x) is larger than the maximum allowed value of 0x%2x. Ignoring argument.\n",
							ctune_value, MAX_CTUNE_VALUE);
			} else
			{
				  ps_state = ps_write_ctune; //init ps state machine
					/* store in ctune_array as little endian */
					/* Test for a little-endian machine */
					#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
						/* ctune_value is little endian - LSB in low byte */
						ctune_array[0] = (uint8_t) (ctune_value & 0xff);
						ctune_array[1] = (uint8_t) ((ctune_value >> 8) & 0xff);
					#else
						/* ctune_value is big endian - LSB in high byte */
						ctune_array[1] = (uint8_t) ((ctune_value & 0xff);
						ctune_array[0] = (uint8_t) ((ctune_value >> 8) & 0xff);
					#endif
			}
		}
		else if(!strncasecmp(argv[argCount],"-v",CMP_LENGTH))
		{
				/* Print #defined version number */
				printf("%s version %d.%d\n",argv[0],VERSION_MAJ,VERSION_MIN);
		}
		else if(!strncasecmp(argv[argCount],"-z",CMP_LENGTH))
		{
				ps_state = ps_read_ctune; //init ps state machine
		}
		else if(!strncasecmp(argv[argCount],"-f",CMP_LENGTH))
		{
				ps_state = ps_read_gatt_fwversion; //init ps state machine (note - this will overwrite states from other arguments)
		}
	  else if(!strncasecmp(argv[argCount],"-d",CMP_LENGTH))
		{
				daemon_flag = 1; //run as daemon (fork the process, etc.)
				printf("Running %s as daemon...\n",argv[0]);
		}
		else if(!strncasecmp(argv[argCount],"-a",CMP_LENGTH))
		{
			/* mac address */
			ps_state = ps_write_mac;
			if( 6 == sscanf( argv[argCount+1], "%x:%x:%x:%x:%x:%x", &values[5], &values[4], &values[3], &values[2], &values[1], &values[0]) )
			{
				/* convert to uint8_t */
				for( i = 0; i < 6; ++i )
					mac_address[i] = (uint8_t) values[i];
			}
			else
			{
				/* invalid mac */
				printf("Error in mac address - enter 6 ascii hex bytes separated by ':'\n\n");
				return(TRUE);
			}
		} else if(!strncasecmp(argv[argCount],"-w",CMP_LENGTH))
		{
			/* Enable WiFi coexistence / PTA if enabled in image */
			pta_enable = 1;
		}

	  /* Parse next argument */
		argCount=argCount+1;
  }

  return FALSE; //ok if we got here

}

/* Handle posix signals and clean up if possible */
void sig_handler(int signo)
{
	struct gecko_cmd_packet *evt;
	void *rsp;
	uint8_t exit_loop = FALSE;
	appEnableTimeout(EXIT_TIMEOUT_SEC);

	printf("sig handler %d received\n", signo);
  if (signo == SIGINT || signo == SIGTERM)
	{
		printf("Cleaning up and exiting\n");

		if ((app_state == dtm_begin) || (app_state == dtm_started))
		{
			printf("Attempting to complete DTM in progress...");

			/* We are in the middle of DTM - let's stop and print the status */
			rsp = gecko_cmd_test_dtm_end(); //NOTE: Will hang here if NCP is unresponsive
			if (((struct gecko_msg_test_dtm_end_rsp_t *)rsp)->result == bg_err_success)
			{

				/* Wait for event */
				while (exit_loop == FALSE)
		    {
					// non-blocking wait for event API packet
					evt = gecko_peek_event();

					if (NULL != evt)
					{
		        switch (BGLIB_MSG_ID(evt -> header))
		        {
							case gecko_evt_test_dtm_completed_id:
								if (app_state == dtm_begin)
								{
										//This is just an acknowledgement of the DTM start - set a flag for the next event which is the end
										app_state = dtm_started;
								} else if ((dtm_mode == dtm_mode_rx) && (app_state == dtm_started))
								{
									//This is the event received at the end of the test
									printf("DTM receive completed. Number of packets received: %d\n",evt->data.evt_test_dtm_completed.number_of_packets);
									exit_loop = TRUE;
								} else if ((dtm_mode == dtm_mode_pkt_tx) && (app_state == dtm_started))
								{
									//This is the event received at the end of the test
									printf("DTM transmit completed. Number of packets transmitted: %d\n",evt->data.evt_test_dtm_completed.number_of_packets);
									exit_loop = TRUE;
								} else if ((dtm_mode == dtm_mode_cont_tx) && (app_state == dtm_started)){
									//This is the event received at the end of the test
									printf("Continuous DTM transmit completed.\n");
									exit_loop = TRUE;
								}
								break;
							default:
								break;
						}
					}

					if (appCheckTimeout())
          {
            exit_loop = TRUE;
          }


				} //end while

			} else
			{
				printf("Error ending DTM - exiting\n");
			}
		}//end if app_state = dtm_started
		gecko_cmd_system_reset(0); /* reset NCP */
		sleep(1);
		uartClose();
		exit(1);
	}
}

void appHandleDaemon(void){

  /* handle the daemon processing if requested */
	if (daemon_flag == 1)
	{
		pid_t process_id = 0;
		pid_t sid = 0;

		// Create child process
		process_id = fork();

		// Indication of fork() failure
		if (process_id < 0)
		{
			printf("daemon fork failed! Exiting.\n");
			// Return failure in exit status
			exit(1);
		}
		// PARENT PROCESS. Need to kill it.
		if (process_id > 0)
		{
			printf("Child process %d created\n", process_id);
			// return success in exit status
			exit(0);
		}

		//unmask the file mode
		umask(0);

		//set new session
		sid = setsid();
		if(sid < 0)
		{
			// Return failure
			exit(1);
		}

		// Change the current working directory to root.
		chdir("/");

		// Close stdin. stdout and stderr
		close(STDIN_FILENO);
		close(STDOUT_FILENO);
		close(STDERR_FILENO);

	}

}

void appEnableTimeout(uint8_t timeout_sec_val)
{
  /* Capture current time and enable timeout flag */
  timeout_active_flag = TRUE;
  time(&timeout_start_val);
  timeout_last_sec = timeout_start_val;
  timeout_sec = timeout_sec_val;

}
void appDisableTimeout()
{
  timeout_active_flag = FALSE;
}
uint8_t appCheckTimeout(){
  /* If timeout is enabled, check current time against timeout
  * and return TRUE if timeout has occurred */
  time_t curtime;
  time(&curtime);
  if (timeout_active_flag == TRUE)
  {
    if (curtime > (timeout_start_val + timeout_sec))
    {
      return TRUE;
    } else if (curtime > timeout_last_sec)
    {
      /* Print activity once a second while waiting for timeout */
      timeout_last_sec = curtime;
      printf(".");
      fflush(stdout);
    }

  }
  return FALSE;
}
void appInit(void)
{

  printf("\n------------------------\n");

	// trigger reset manually with an API command
	printf("Waiting for boot pkt...");
	gecko_cmd_system_reset(0);

	/* Enable timeout waiting for system boot message. */
	appEnableTimeout(BOOT_TIMEOUT_SEC);
}
