/*
 * ble.h
 *  Handles all the bluetooth related events.
 *  Created on: Sep 30, 2021
 *      Author: mich1576
 */

#ifndef SRC_BLE_H_
#define SRC_BLE_H_

#define UINT8_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); }                     // use this for the flags byte, which you set = 0

#define UINT32_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
                                  *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }

#define UINT32_TO_FLOAT(m, e) (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

// BLE Data Structure, save all of our private BT data in here.
// Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct {
      // values that are common to servers and clients
      bd_addr myAddress;

      uint8_t myAddressType;

      // values unique for server
      uint8_t advertisingSetHandle;

      //store connection handle for sending the indication
      uint8_t connection_handle;

      //store the characteristic we are sending the indication for
      uint16_t characteristic;

      //soft timer handle
      uint8_t soft_timer_handle;

      //Rollover count
      uint8_t rollover_count;

      //milliseconds
      uint32_t milliseconds;

      //bool indication for temperature measurement characteristic
      bool i_am_a_bool_for_temp;
      // values unique for client

} ble_data_struct_t;

// function prototypes
/* function     : getBleDataPtr
 * params       : void
 * brief        : function to get the ble data struct rather than keeping it global and corrupting the data inside
 * return type  : ble_data_struct_t*
 */
ble_data_struct_t* getBleDataPtr(void);

/* function     : handle_ble_event
 * params       : sl_bt_msg_t *evt
 * brief        : takes different bluetooth events as input and and handles events based on different flags
 * return type  : void
 */
void handle_ble_event(sl_bt_msg_t *evt);

/* function     : sl_bt_ht_temperature_measurement_indication_confirmed_cb
 * params       : uint8_t connection
 * brief        : function that checks confirmatin of indication from the EFR connect
 * return type  : void
 */
void sl_bt_ht_temperature_measurement_indication_confirmed_cb(uint8_t connection);

/* function     : sl_bt_ht_temperature_measurement_indication_changed_cb
 * params       : uint8_t connection, uint16_t characteristic
 * brief        : function called if indication is enabled from the user
 * return type  : void
 */
void sl_bt_ht_temperature_measurement_indication_changed_cb(uint8_t connection, uint16_t characteristic);

#endif /* SRC_BLE_H_ */
