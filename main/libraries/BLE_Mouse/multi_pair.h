#include <cstdint>
#include "esp_err.h"
#include "esp_bt_defs.h"

typedef uint8_t UINT8;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
typedef uint64_t UINT64;
typedef bool BOOLEAN;

typedef UINT32 tBTA_SERVICE_MASK;
typedef UINT8 tBTA_IO_CAP;

#define LINK_KEY_LEN    16
typedef UINT8 LINK_KEY[LINK_KEY_LEN];       /* Link Key */

#define DEV_CLASS_LEN   3
typedef UINT8 DEV_CLASS[DEV_CLASS_LEN];     /* Device class */
typedef UINT8 *DEV_CLASS_PTR;               /* Pointer to Device class */

struct BondedDevice {
    esp_err_t save();
    esp_err_t load();
    void apply();
    int device_num;
    esp_bd_addr_t bd_addr;
    DEV_CLASS dev_class;
    LINK_KEY link_key;
    tBTA_SERVICE_MASK trusted_mask;
    BOOLEAN is_trusted;
    UINT8 key_type;
    tBTA_IO_CAP io_cap;
    UINT8 pin_length;
    UINT8 sc_support;
};

const BondedDevice EmptyBondedDevice = BondedDevice{0, {0}, {0}, {0}, 0, false, 0, 0, 0, 0};