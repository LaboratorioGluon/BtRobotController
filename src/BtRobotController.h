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

enum BtRobotOperationType{
    BTROBOT_OP_READ = 0,
    BTROBOT_OP_WRITE,
};

typedef uint32_t (*robotUserCallbackFn)(void * data, uint32_t len, BtRobotOperationType operation);

struct BtRobotConfiguration
{
    char paramName[BTROBOT_CONFIG_NAME_MAXLEN];
    enum BtRobotConfigType dataType;
    robotUserCallbackFn callback;
};

class BtRobotController
{

public:
    static BtRobotController& getBtRobotController();   

    void Init(char * robotName, struct BtRobotConfiguration btServicesConfig[], uint32_t lenServicesConfig);

    uint32_t runCallback(uint32_t id, void * data, uint32_t len, BtRobotOperationType operation);

private:
    BtRobotController();
    BtRobotController(const BtRobotController&) = delete;
    BtRobotController(BtRobotController&&) = delete;
    BtRobotController& operator=(const BtRobotController&) = delete;
    BtRobotController& operator=(BtRobotController&&) = delete;


    /**
     * Initialize the basic nimBLE features of ESP32
    */
    void internalBtInit();


    char internalRobotName[BTROBOT_ROBOTNAME_MAXLEN];

    /***** BLE Items *****/

    uint8_t lastCharacteristic;

    // Only one service, the second one is the {0}
    struct ble_gatt_svc_def gatt_svcs[2];
    struct ble_gatt_chr_def characteristics[BTROBOT_CONFIG_MAX_CHARS] = {0};

    ble_uuid128_t generateUUID();

    static int commonCallback(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);



    robotUserCallbackFn callbackMap[BTROBOT_CONFIG_MAX_CHARS] = {nullptr};

    ble_uuid128_t CHARACTERISTIC_UUID[BTROBOT_CONFIG_MAX_CHARS];

};




#endif //__BTROBOTCONTROLLER_H__