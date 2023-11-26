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


#ifndef BTROBOT_ROBOTNAME_MAXLEN
    #define BTROBOT_ROBOTNAME_MAXLEN 25
#endif 

#define BTROBOT_CONFIG_MAX_CHARS 10
#define BTROBOT_CONFIG_MAX_DESCRIPTORS 5

#define BTROBOT_MAX_DATA_LEN 100

#define BTROBOT_CONFIG_NAME_MAXLEN 15


/*********** Public Types **************/

enum BtRobotConfigType{
    BTROBOT_CONFIG_INT = 0,
    BTROBOT_CONFIG_FLOAT,
    BTROBOT_CONFIG_EVENT,
    BTROBOT_CONFIG_LATCH,
    BTROBOT_CONFIG_INT_SLIDE,
    BTROBOT_CONFIG_FLOAT_SLIDE,
};

enum BtRobotOperationType{
    BTROBOT_OP_READ = 0,
    BTROBOT_OP_WRITE,
};

typedef uint32_t (*robotUserCallbackFn)(void * data, uint32_t len, BtRobotOperationType operation);

struct dataType{
    enum BtRobotConfigType dataType;
    union{
        
        struct{
            uint32_t min;
            uint32_t max;
            int32_t step;
        }intSlide;

        struct{
            float min;
            float max;
            float step;
        }floatSlide;

    }config;
};


struct BtRobotConfiguration
{
    char paramName[BTROBOT_CONFIG_NAME_MAXLEN];
    robotUserCallbackFn callback;
    struct dataType dataConfig;
};

/*********** Main Class **************/
class BtRobotController
{

public:
    static BtRobotController& getBtRobotController();

    void Init(char * robotName, struct BtRobotConfiguration btServicesConfig[], uint32_t lenServicesConfig);

    uint32_t runCallback(uint32_t id, void * data, uint32_t len, BtRobotOperationType operation);

    static void data_op_read(void * data, uint32_t len);


    struct BtRobotConfiguration userConfiguration[BTROBOT_CONFIG_MAX_CHARS] = {0};

    uint8_t numUserCharacteristics;
    char characteristicNames[BTROBOT_CONFIG_MAX_CHARS][BTROBOT_CONFIG_NAME_MAXLEN];

private:
    // Make private so there is only one controller created in getBtRobotController()
    BtRobotController();

    // Disable non-basic constructors.
    BtRobotController(const BtRobotController&) = delete;
    BtRobotController(BtRobotController&&) = delete;
    BtRobotController& operator=(const BtRobotController&) = delete;
    BtRobotController& operator=(BtRobotController&&) = delete;



    // Interchange data
    static uint8_t ReadData[BTROBOT_MAX_DATA_LEN];
    static uint32_t ReadDataLen;

    static uint8_t WriteData[BTROBOT_MAX_DATA_LEN];
    static uint32_t WriteDataLen;

    /**
     * Initialize the basic nimBLE features of ESP32
    */
    void internalBtInit();

    char internalRobotName[BTROBOT_ROBOTNAME_MAXLEN];

    /***** BLE Items *****/
    uint8_t lastCharacteristic;

    // last svc is {0}
    struct ble_gatt_svc_def gatt_svcs[3];
    struct ble_gatt_chr_def commonCharacteristics[BTROBOT_CONFIG_MAX_CHARS] = {0};
    struct ble_gatt_chr_def userCharacteristics[BTROBOT_CONFIG_MAX_CHARS] = {0};
    struct ble_gatt_dsc_def userDescriptors[BTROBOT_CONFIG_MAX_CHARS][BTROBOT_CONFIG_MAX_DESCRIPTORS] = {0};

    ble_uuid128_t generateUUID();

    static int commonCallback(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

    static int configCallback(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

    static int typeCallback(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

    robotUserCallbackFn callbackMap[BTROBOT_CONFIG_MAX_CHARS] = {nullptr};
    

    ble_uuid128_t CHARACTERISTIC_UUID[BTROBOT_CONFIG_MAX_CHARS];
    ble_uuid128_t DESCRIPTORS_UUID[BTROBOT_CONFIG_MAX_CHARS][BTROBOT_CONFIG_MAX_DESCRIPTORS];

    struct BtRobotConfiguration userConfigurations[BTROBOT_CONFIG_MAX_CHARS];

};




#endif //__BTROBOTCONTROLLER_H__