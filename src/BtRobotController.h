#ifndef __BTROBOTCONTROLLER_H__
#define __BTROBOTCONTROLLER_H__

#include <stdint.h>
#include <string.h>


#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_bt.h"

#define BTROBOT_ROBOTNAME_MAXLEN 25
#define BTROBOT_CONFIG_MAX_CHARS 30

#define BTROBOT_CONFIG_NAME_MAXLEN 15

enum BtRobotConfigType{
    BTROBOT_CONFIG_INT,
    BTROBOT_CONFIG_FLOAT,
    BTROBOT_CONFIG_DOUBLE,
    BTROBOT_CONFIG_EVENT,
    BTROBOT_CONFIG_LATCH
};

typedef uint32_t (*robotUserCallbackFn)(void * data, uint32_t len);

struct BtRobotConfiguration
{
    char paramName[BTROBOT_CONFIG_NAME_MAXLEN];
    enum BtRobotConfigType dataType;
    robotUserCallbackFn callback;
};

class BtRobotController
{

public:
    BtRobotController();

    void Init(char * robotName, struct BtRobotConfiguration btServicesConfig[], uint32_t lenServicesConfig);

private:

    /**
     * Initialize the basic nimBLE features of ESP32
    */
    void internalBtInit();

    ble_uuid128_t generateUUID();

    uint8_t lastCharacteristic;

    static int commonCallback(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

    ble_uuid128_t CHARACTERISTIC_UUID[BTROBOT_CONFIG_MAX_CHARS];
    char internalRobotName[BTROBOT_ROBOTNAME_MAXLEN];

};

#endif //__BTROBOTCONTROLLER_H__