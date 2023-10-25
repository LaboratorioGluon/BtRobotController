#include "BtRobotController.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "esp_assert.h"
#include <functional>

#include <string.h>

static const char *TAG = "BTROBOTCONTROLLER";

BtRobotController& BtRobotController::getBtRobotController()
{
    static BtRobotController instance;
    ESP_LOGI(TAG, "Returning: %p", &instance);
    return instance;
}

BtRobotController::BtRobotController()
{
    lastCharacteristic = 0x00;

    static const ble_uuid128_t SERVICE = BLE_UUID128_INIT(0x0f, 0x4b, 0xe0, 0x8b, 0x89, 0x3c, 0x40, 0x97, 0xa3, 0xc5, 0x5e, 0x7c, 0xfc, 0xd2, 0x73, 0x70); //"0f4be08b-893c-4097-a3c5-5e7cfcd27370"

    gatt_svcs[0] = {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SERVICE.u,
        .characteristics = characteristics
    };

    gatt_svcs[1] = {0};
}

void BtRobotController::Init(char *robotName, struct BtRobotConfiguration btServicesConfig[], uint32_t lenServicesConfig)
{
    ESP_LOGI(TAG, "Robot: %p", this);
    // >= due to the last item being the {0}
    if (lenServicesConfig >= BTROBOT_CONFIG_MAX_CHARS)
    {
        ESP_LOGE(TAG, "Error Maximum characterics are %d, provided: %lu", BTROBOT_CONFIG_MAX_CHARS, lenServicesConfig);
        return;
    }

    // Handle robot Name
    if (strlen(robotName) > BTROBOT_ROBOTNAME_MAXLEN - 1)
    {
        ESP_LOGE(TAG, "Error RobotName longer than %d", BTROBOT_ROBOTNAME_MAXLEN - 1);
        return;
    }
    strcpy(internalRobotName, robotName);

    ESP_LOGI(TAG,"Starting Filling chars..");

    // Generate Bluetooth's Structure.
    for (uint32_t i = 0; i < lenServicesConfig; i++)
    {
        ESP_LOGE(TAG,"Adding characteristic %lu, cb: %p", i, btServicesConfig[i].callback);
        generateUUID();
        characteristics[i] =
            {
                .uuid = &(CHARACTERISTIC_UUID[i].u),
                .access_cb = &BtRobotController::commonCallback, // Ahora se espera una funcion super chunga de gatt, tenemos que
                // idear una forma de que haya un callback mas sencillo para el usuario.
                .arg = (void *)i,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .min_key_size = 16U
            };
        
        callbackMap[i] = btServicesConfig[i].callback;
    }
    characteristics[lenServicesConfig] = {0};

    gatt_svcs[0].characteristics = characteristics;


    internalBtInit();
    ble_gatts_count_cfg(gatt_svcs);                         // config all the gatt services that wanted to be used.
    ble_gatts_add_svcs(gatt_svcs);                          // queues all services.
}

ble_uuid128_t BtRobotController::generateUUID()
{
    CHARACTERISTIC_UUID[lastCharacteristic] =
        BLE_UUID128_INIT(0x3f, 0xd3, 0x2b, 0xe3, 0xad, 0x57, 0x4f, 0x3a, 0xad, 0xca, 0xb9, 0x3f, 0x14, 0x79, 0x86, lastCharacteristic);

    lastCharacteristic++;
    return CHARACTERISTIC_UUID[lastCharacteristic];
}

void BtRobotController::internalBtInit()
{
    nimble_port_init();                                              // nimble library initialization.
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(internalRobotName)); // set BLE name.
    ble_svc_gap_init();                                              // initialize the gap service.
    ble_svc_gatt_init();                                             // initailize the gatt service.
}

uint32_t BtRobotController::runCallback(uint32_t id, void * data, uint32_t len, BtRobotOperationType operation )
{
    ESP_LOGI(TAG, "Robot: %p", this);
    ESP_LOGI(TAG, "CallbackMap (%lu): %p", id, callbackMap[id]);

    if (callbackMap[id] == nullptr)
    {
        ESP_LOGE(TAG, "Error callback not defined! \n");
        return -1;
    }

    return callbackMap[id](data,len,operation);
}

int BtRobotController::commonCallback(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    BtRobotController& controller = BtRobotController::getBtRobotController();
    uint32_t id = (int)arg;

    ESP_LOGI(TAG, "Callback arg: %d\n", (int)arg);

    switch(ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        controller.runCallback(id, nullptr, 0, BTROBOT_OP_READ);
        break;
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        break;
    case BLE_GATT_ACCESS_OP_READ_DSC:
        break;
    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        break;
    }
    return 0;
}