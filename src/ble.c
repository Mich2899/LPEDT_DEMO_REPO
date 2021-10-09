/*
 * ble.c
 *  Handles all the bluetooth related events.
 *  Checks for indication flags and sends the required information to the EFR connect app.
 *  Created on: Sep 30, 2021
 *      Author: mich1576
 */

#include <stdio.h>
#include "stdbool.h"
#include "sl_bt_api.h"
#include "gatt_db.h"
#include "app_assert.h"
#include "ble.h"
#include "i2c.h"
#include "lcd.h"
// Include logging for this file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


sl_bt_msg_t *evt;                                                               //evt struct that stores data and header.
float temp;                                                                     //stores the value of temperature recorded from Si7021
                                                                                //which is later converted to bitstream to send to the app.

// Connection handle.
//static uint8_t app_connection = 0;                                              //stores the handles

// BLE private data
ble_data_struct_t ble_data;

// function that returns a pointer to the
// BLE private data
ble_data_struct_t* getBleDataPtr() {
  return (&ble_data);
} // getBleDataPtr()

/**************************************************************************//**
 * Temperature Measurement characteristic indication confirmed.
 *****************************************************************************/
SL_WEAK void sl_bt_ht_temperature_measurement_indication_confirmed_cb(uint8_t connection)
{
  (void)connection;
}

void handle_ble_event(sl_bt_msg_t *evt) {

  sl_status_t retstat;                                                          //stores the return status of different bluetooth API functions

  switch (SL_BT_MSG_ID(evt->header)) {
  // ******************************************************
  // Events common to both Servers and Clients
  // ******************************************************
  // --------------------------------------------------------
  // This event indicates the device has started and the radio is ready.
  // Do not call any stack API commands before receiving this boot event!
  // Including starting BT stack soft timers!
  // --------------------------------------------------------
  // ATTRIBUTION NOTE:Some cases are handled in a manner the "SOC THERMOMETER" example handles them
      case sl_bt_evt_system_boot_id:
        // handle boot event

        displayInit();

        displayPrintf(DISPLAY_ROW_NAME, "%s", BLE_DEVICE_TYPE_STRING);                                  //SERVER
        displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A6");                                //ASSIGNMENT NUMBER

        //ID extracted from address
        retstat = sl_bt_system_get_identity_address(&ble_data.myAddress, &ble_data.myAddressType);
        app_assert_status(retstat);

        // display address on LCD
        displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                                         ble_data.myAddress.addr[5],
                                         ble_data.myAddress.addr[4],
                                         ble_data.myAddress.addr[3],
                                         ble_data.myAddress.addr[2],
                                         ble_data.myAddress.addr[1],
                                         ble_data.myAddress.addr[0]);//SERVER ADDRESS

        //store ID in advertisingSetHandle
        ble_data.advertisingSetHandle = 0xff;

        // Create an advertising set.
        retstat = sl_bt_advertiser_create_set(&ble_data.advertisingSetHandle);
        app_assert_status(retstat);

        // Set advertising interval to 250ms.
        retstat = sl_bt_advertiser_set_timing(
            ble_data.advertisingSetHandle, // advertising set handle
            0x190,                                                              // min. adv. interval (milliseconds * 1.6) 250*1.6 = 400 hex 190
            0x190,                                                              // max. adv. interval (milliseconds * 1.6) 250*1.6 = 400 hex 190
            0,                                                                  // adv. duration
            0);                                                                 // max. num. adv. events
        app_assert_status(retstat);

        //Once the device is booted up start advertising
        retstat = sl_bt_advertiser_start(
            ble_data.advertisingSetHandle,
            sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
        app_assert_status(retstat);

        ble_data.soft_timer_handle = evt->data.evt_system_soft_timer.handle;

        displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");


        break;

      case sl_bt_evt_connection_opened_id:
        // handle open event

        //Once the device is connected to the app stop advertising
        retstat = sl_bt_advertiser_stop(ble_data.advertisingSetHandle);
        app_assert_status(retstat);

        //store the connection handle to send an indication for temperature
        ble_data.connection_handle = evt->data.evt_connection_opened.connection;

        //set the connection parameters
        retstat = sl_bt_connection_set_parameters  (
            ble_data.connection_handle,                                         // connection handle
            60,                                                                 // min. connection. interval 75/ 1.25 = 60 hex 3C
            60,                                                                 // max. connection. interval 75/ 1.25 = 60 hex 3C
            0x03,                                                               // latency
            0x50,                                                               // timeout
            250,                                                                // min ce length
            250);                                                               // max ce length

        app_assert_status(retstat);
        LOG_INFO("connection set parameters return status: %ld\n\r", retstat);

        displayPrintf(DISPLAY_ROW_CONNECTION,"Connected");

        break;
      case sl_bt_evt_connection_closed_id:
        // handle close event
        // Restart advertising after client has disconnected.
        ble_data.i_am_a_bool_for_temp = false;
        retstat = sl_bt_advertiser_start(
            ble_data.advertisingSetHandle,
            sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
        app_assert_status(retstat);

        displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");
        displayPrintf(DISPLAY_ROW_TEMPVALUE, "");

        break;

      case  sl_bt_evt_connection_parameters_id:
        //log all the connection parameters if any of them changes
       /* LOG_INFO("Connection params: connection=%d\n\r, interval=%d\n\r, latency=%d\n\r, timeout=%d\n\r, securitymode=%d\n\r",
                (int) (evt->data.evt_connection_parameters.connection),
                (int) (evt->data.evt_connection_parameters.interval*1.25),
                (int) (evt->data.evt_connection_parameters.latency),
                (int) (evt->data.evt_connection_parameters.timeout*10),
                (int) (evt->data.evt_connection_parameters.security_mode) );
        */
        break;

        //Events for Slave/Server
      case sl_bt_evt_gatt_server_characteristic_status_id:
        //This flag is raised every time the characteristics of the connection change
        if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement) {
            //Indicates either that a local Client Characteristic Configuration descriptor (CCCD) was changed by the remote GATT client

            //check if the Characteristic client configuration has changed
            if(sl_bt_gatt_server_client_config == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags) {
                //check if the indication is enabled
                if (sl_bt_gatt_disable != (sl_bt_gatt_client_config_flag_t) evt->data.evt_gatt_server_characteristic_status.client_config_flags){
                    ble_data.characteristic = evt->data.evt_gatt_server_characteristic_status.characteristic;
                    ble_data.i_am_a_bool_for_temp = true;
                }
                // confirmation of indication received from remove GATT client
                else if (sl_bt_gatt_server_confirmation == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags) {
                  sl_bt_ht_temperature_measurement_indication_confirmed_cb(
                    evt->data.evt_gatt_server_characteristic_status.connection);
                }
                else{
                    displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
                    ble_data.i_am_a_bool_for_temp = false;
                    //LOG_INFO("Indication disabled!!\n\r");
                }
            }
        }
        break;

      case sl_bt_evt_gatt_server_indication_timeout_id:
        //LOG_INFO("The connection timed out!!!\n\r");
        //Possible event from calling sl_bt_gatt_server_send_indication() -
        //i.e. we never received a confirmation for a previously transmitted indication.
        break;

      case sl_bt_evt_system_soft_timer_id:
        displayUpdate();
  } // end - switch

} // handle_ble_event()


void sl_bt_ht_temperature_measurement_indication_changed_cb(uint8_t connection, uint16_t characteristic)
{

    // -------------------------------------------------------------------
    // Update our local GATT DB and send indication if enabled for the characteristic
    // -------------------------------------------------------------------

      sl_status_t sc;                                                           //to store and check the return status

      uint8_t htm_temperature_buffer[5];                                        // Stores the temperature data in the Health Thermometer (HTM) format.
                                                                                // format of the buffer is: flags_byte + 4-bytes of IEEE-11073 32-bit float
      uint8_t *p = htm_temperature_buffer;                                      // Pointer to HTM temperature buffer needed for converting values to bitstream.
      uint32_t htm_temperature_flt;                                             // Stores the temperature data read from the sensor in the IEEE-11073 32-bit float format

      temp = store();

      uint8_t flags = 0x00;                                                     // HTM flags set as 0 for Celsius, no time stamp and no temperature type.

      // "bitstream" refers to the order of bytes and bits sent. byte[0] is sent first, followed by byte[1]...
      UINT8_TO_BITSTREAM(p, flags); // put the flags byte in first, "convert" is a strong word, it places the byte into the buffer

      // Convert sensor data to IEEE-11073 32-bit floating point format.
      htm_temperature_flt = UINT32_TO_FLOAT(temp*1000, -3);

      // Convert temperature to bitstream and place it in the htm_temperature_buffer
      UINT32_TO_BITSTREAM(p, htm_temperature_flt);

      sc = sl_bt_gatt_server_write_attribute_value( gattdb_temperature_measurement, // handle from gatt_db.h
                                                    0,                              // offset
                                                    5,                              // length
                                                    &htm_temperature_buffer[0]);    // pointer to buffer where data is


      if (sc != SL_STATUS_OK) {
      LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
      }

      //convert and store the temperature data for logging
      sc = sl_bt_gatt_server_send_indication(connection,                        //connection handle
                                             characteristic,                    //for what characteristic we are sending an indication
                                             5,                                 // length
                                             &htm_temperature_buffer[0]);       // pointer to buffer where data is);
      if (sc != SL_STATUS_OK) {
      LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x", (unsigned int) sc);
      }

      displayPrintf(DISPLAY_ROW_TEMPVALUE, "%f", temp);
}


