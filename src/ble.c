/*
 * ble.c
 *
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
// Include logging for this file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


sl_bt_msg_t *evt;
int temp;

// Connection handle.
static uint8_t app_connection = 0;

// BLE private data
ble_data_struct_t ble_data;

// function that returns a pointer to the
// BLE private data
ble_data_struct_t* getBleDataPtr() {
  return (&ble_data);
} // getBleDataPtr()


uint8_t htm_temperature_buffer[5];
uint8_t *p = htm_temperature_buffer;
uint32_t htm_temperature_flt;


SL_WEAK void sl_bt_ht_temperature_measurement_indication_confirmed_cb(uint8_t connection)
{
  (void)connection;
}

void handle_ble_event(sl_bt_msg_t *evt) {

  sl_status_t retstat;

  switch (SL_BT_MSG_ID(evt->header)) {
  // ******************************************************
  // Events common to both Servers and Clients
  // ******************************************************
  // --------------------------------------------------------
  // This event indicates the device has started and the radio is ready.
  // Do not call any stack API commands before receiving this boot event!
  // Including starting BT stack soft timers!
  // --------------------------------------------------------
  //Some cases are handled in a manner the soc thermometer example handles them
      case sl_bt_evt_system_boot_id:
        // handle boot event

        //ID extracted from address
        retstat = sl_bt_system_get_identity_address(&ble_data.myAddress, &ble_data.myAddressType);
        app_assert_status(retstat);

        //store ID in advertisingSetHandle
        ble_data.advertisingSetHandle = 0xff;

        // Create an advertising set.
        retstat = sl_bt_advertiser_create_set(&ble_data.advertisingSetHandle);
        app_assert_status(retstat);

        // Set advertising interval to 250ms.
        retstat = sl_bt_advertiser_set_timing(
            ble_data.advertisingSetHandle, // advertising set handle
            400, // min. adv. interval (milliseconds * 1.6) 250
            400, // max. adv. interval (milliseconds * 1.6) 250
            0,   // adv. duration
            0);  // max. num. adv. events
        app_assert_status(retstat);

        retstat = sl_bt_advertiser_start(
            ble_data.advertisingSetHandle,
            sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
        app_assert_status(retstat);

        break;

      case sl_bt_evt_connection_opened_id:
        // handle open event

        retstat = sl_bt_advertiser_stop(ble_data.advertisingSetHandle);
        app_assert_status(retstat);

        app_connection = evt->data.evt_connection_opened.connection;
        retstat = sl_bt_connection_set_parameters  (
            app_connection,             // connection handle
            60,                         // min. connection. interval (milliseconds * 1.25)
            60,                         // max. connection. interval (milliseconds * 1.25)
            4,                          // latency
            750,                        // timeout
            0,                          // min ce length
            0xffff);                    // max ce length

        app_assert_status(retstat);

        break;
      case sl_bt_evt_connection_closed_id:
        // handle close event
        // Restart advertising after client has disconnected.
        retstat = sl_bt_advertiser_start(
            ble_data.advertisingSetHandle,
            sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
        app_assert_status(retstat);

        break;

      case  sl_bt_evt_connection_parameters_id:
        LOG_INFO("Connection params: connection=%d, interval=%d, latency=%d, timeout=%d, securitymode=%d",
                (int) (evt->data.evt_connection_parameters.connection),
                (int) (evt->data.evt_connection_parameters.interval*1.25),
                (int) (evt->data.evt_connection_parameters.latency),
                (int) (evt->data.evt_connection_parameters.timeout*10),
                (int) (evt->data.evt_connection_parameters.security_mode) );

        break;

        //Events for Slave/Server
        // more case statements to handle other BT event
        /*  PACKSTRUCT( struct sl_bt_evt_gatt_server_characteristic_status_s
        {
        uint8_t  connection;          // Connection handle
        uint16_t characteristic;      // GATT characteristic handle. This value is normally received from the
                                             gatt_characteristic event.
        uint8_t  status_flags;        // Enum @ref
                                             sl_bt_gatt_server_characteristic_status_flag_t.
                                             Describes whether Client Characteristic
                                             Configuration was changed or if a
                                             confirmation was received. Values:
                                             - <b>sl_bt_gatt_server_client_config
                                               (0x1):</b> Characteristic client
                                               configuration has been changed.
                                             - <b>sl_bt_gatt_server_confirmation
                                               (0x2):</b> Characteristic confirmation
                                               has been received.
        uint16_t client_config_flags;   // Enum @ref
                                             sl_bt_gatt_server_client_configuration_t.
                                             This field carries the new value of the
                                             Client Characteristic Configuration. If the
                                             status_flags is 0x2 (confirmation
                                             received), the value of this field can be
                                             ignored.
        uint16_t client_config;         // The handle of client-config descriptor.
      });
      */

      case sl_bt_evt_gatt_server_characteristic_status_id:
        if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement) {
            //Indicates either that a local Client Characteristic Configuration descriptor (CCCD) was changed by the remote GATT client

            if(sl_bt_gatt_server_client_config == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags) {
                sl_bt_ht_temperature_measurement_indication_changed_cb(evt->data.evt_gatt_server_characteristic_status.connection,
                                                                      (sl_bt_gatt_client_config_flag_t) evt->data.evt_gatt_server_characteristic_status.client_config_flags);
            }

            //or that a confirmation from the remote GATT client was received upon a successful reception of the indication
            else if(sl_bt_gatt_server_confirmation == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags){
                sl_bt_ht_temperature_measurement_indication_confirmed_cb(evt->data.evt_gatt_server_characteristic_status.connection);
            }
            else {
                app_assert(false, "[E: 0x%04x] Unexpected status flag in evt_gatt_server_characteristic_status\n", (int)evt->data.evt_gatt_server_characteristic_status.status_flags);
            }
        }
        break;

      case sl_bt_evt_gatt_server_indication_timeout_id:
        LOG_INFO("timeout\n\r");
        //Possible event from calling sl_bt_gatt_server_send_indication() -
        //i.e. we never received a confirmation for a previously transmitted indication.
        break;
  } // end - switch

} // handle_ble_event()


void sl_bt_ht_temperature_measurement_indication_changed_cb(uint8_t connection,
                                                            sl_bt_gatt_client_config_flag_t client_config)
{
  sl_status_t sc;
  app_connection = connection;

  temp = store();

  htm_temperature_flt = UINT32_TO_FLOAT(temp*1000, -3);
  // Convert temperature to bitstream and place it in the htm_temperature_buffer
  UINT32_TO_BITSTREAM(p, htm_temperature_flt);

  sc = sl_bt_gatt_server_write_attribute_value( gattdb_temperature_measurement, // handle from gatt_db.h
                                                0, // offset
                                                5, // length
                                                &htm_temperature_buffer[0] // pointer to buffer where data is
                                                                        );

  if (sc != SL_STATUS_OK) {
  LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
  }
  // Indication or notification enabled.
  if (sl_bt_gatt_disable != client_config){
      //convert and store the temperature data for logging
      sc = sl_bt_gatt_server_send_indication(app_connection,
                                             gattdb_temperature_measurement,
                                             sizeof(p),
                                             p);
      app_assert_status(sc);
      ble_data.i_am_a_bool = true;

  }
  else{
      ble_data.i_am_a_bool = false;
  }
}

