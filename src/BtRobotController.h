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

enum BtRobotConfigType
{
    BTROBOT_CONFIG_INT = 0,
    BTROBOT_CONFIG_FLOAT,
    BTROBOT_CONFIG_EVENT,
    BTROBOT_CONFIG_LATCH,
    BTROBOT_CONFIG_INT_SLIDE,
    BTROBOT_CONFIG_FLOAT_SLIDE,
};

enum BtRobotOperationType
{
    BTROBOT_OP_READ = 0,
    BTROBOT_OP_WRITE,
};

typedef uint32_t (*robotUserCallbackFn)(void *data, uint32_t len, BtRobotOperationType operation);

struct dataType
{
    enum BtRobotConfigType dataType;
    union
    {

        struct
        {
            uint32_t min;
            uint32_t max;
            int32_t step;
        } intSlide;

        struct
        {
            float min;
            float max;
            float step;
        } floatSlide;

    } config;
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
    static BtRobotController &getBtRobotController();

    /**
     * @brief Initialize the BLE/services/configuration. Shall be called at startup.
     * @param robotName A '\0' finished string with the name of the robot, that will appear in the smartphone
     * @param btServicesConfig A list of the required configuration/actions.
     * @param lenServicesConfig Number of configurations in 'btServicesConfig'
     */
    void Init(char *robotName, struct BtRobotConfiguration btServicesConfig[], uint32_t lenServicesConfig);

    /**
     * @brief A helper function to run a callback. This function is used by a static one and *should not be* used by
     *  the user of the library.
     * @param id Id number of the characteristic.
     * @param data Data to be read/write
     * @param len Len of Data
     * @param operation type of operation.
     * @return
     */
    uint32_t runCallback(uint32_t id, void *data, uint32_t len, BtRobotOperationType operation);

    static void data_op_read(void *data, uint32_t len);

    struct BtRobotConfiguration userConfiguration[BTROBOT_CONFIG_MAX_CHARS] = {};

    uint8_t numUserCharacteristics;
    char characteristicNames[BTROBOT_CONFIG_MAX_CHARS][BTROBOT_CONFIG_NAME_MAXLEN];

private:
    // Make private so there is only one controller created in getBtRobotController()
    BtRobotController();

    // Disable non-basic constructors.
    BtRobotController(const BtRobotController &) = delete;
    BtRobotController(BtRobotController &&) = delete;
    BtRobotController &operator=(const BtRobotController &) = delete;
    BtRobotController &operator=(BtRobotController &&) = delete;

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
    struct ble_gatt_chr_def commonCharacteristics[BTROBOT_CONFIG_MAX_CHARS] = {};
    struct ble_gatt_chr_def userCharacteristics[BTROBOT_CONFIG_MAX_CHARS] = {};
    struct ble_gatt_dsc_def userDescriptors[BTROBOT_CONFIG_MAX_CHARS][BTROBOT_CONFIG_MAX_DESCRIPTORS] = {};

    ble_uuid128_t generateUUID();

    /*******************************/
    /* Callbacks                  */
    /*******************************/

    static int commonCallback(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

    static int configCallback(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

    static int typeCallback(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);

    robotUserCallbackFn callbackMap[BTROBOT_CONFIG_MAX_CHARS] = {nullptr};

    /*******************************/
    /* Characteristics UUIDs       */
    /*******************************/

    ble_uuid128_t CHARACTERISTIC_UUID[BTROBOT_CONFIG_MAX_CHARS];
    ble_uuid128_t DESCRIPTORS_UUID[BTROBOT_CONFIG_MAX_CHARS][BTROBOT_CONFIG_MAX_DESCRIPTORS];

    struct BtRobotConfiguration userConfigurations[BTROBOT_CONFIG_MAX_CHARS];

    /*******************************/
    /* BLE extra functions         */
    /*******************************/

    /**
     * @brief Callback function called on BLE synchronization event.
     *
     * This function is called when the BLE stack has synchronized with the host.
     * It configures the BLE address type and starts advertising the services.
     *
     * @note Make sure to include the necessary header files and dependencies in your project.
     *
     * @note The 'ble_app_advertise()' function should be defined in your project to start advertising the services.
     */
    static void ble_app_on_sync(void);

    /**
     * @brief Callback function to handle BLE GAP events.
     *
     * This function handles various BLE GAP events such as connection, disconnection, advertising completion, and subscription.
     *
     * @note Make sure to include the necessary header files and dependencies in your project.
     *
     * @param event Pointer to the BLE GAP event structure.
     * @param arg Argument provided during event registration.
     * @return int Returns 0 to indicate success.
     */
    static int ble_gap_event(struct ble_gap_event *event, void *arg);

    /**
     * @brief Function to start advertising the BLE services.
     *
     * This function configures the advertisement fields and parameters, then starts advertising the BLE services.
     *
     * @note Make sure to include the necessary header files and dependencies in your project.
     *
     * @note The 'ble_svc_gap_device_name()' function should be defined in your project to retrieve the BLE device name.
     *
     * @param ble_addr_type The BLE address type to be used for advertising.
     */
    static void ble_app_advertise(void);
    /**
     * @brief NimBLE host task function.
     *
     * This function runs the NimBLE host task using the NimBLE port.
     *
     * @param param Pointer to task parameters (unused in this context).
     */
    static void host_task(void *param);

    static void configure_ble_max_power();
};

#endif //__BTROBOTCONTROLLER_H__