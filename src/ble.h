/*
 * ble.h
 *
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

      bool i_am_a_bool;
      // values unique for client

} ble_data_struct_t;

// function prototypes
ble_data_struct_t* getBleDataPtr(void);

void handle_ble_event(sl_bt_msg_t *evt);

void sl_bt_ht_temperature_measurement_indication_confirmed_cb(uint8_t connection);

void sl_bt_ht_temperature_measurement_indication_changed_cb(uint8_t connection, sl_bt_gatt_client_config_flag_t client_config);

#endif /* SRC_BLE_H_ */
