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

uint8_t BtRobotController::ReadData[BTROBOT_MAX_DATA_LEN] = {0};
uint32_t BtRobotController::ReadDataLen = 0;

uint8_t BtRobotController::WriteData[BTROBOT_MAX_DATA_LEN] = {0};
uint32_t BtRobotController::WriteDataLen = 0;

BtRobotController &BtRobotController::getBtRobotController()
{
    static BtRobotController instance;
    return instance;
}

BtRobotController::BtRobotController()
{
    lastCharacteristic = 0x00;
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

    ESP_LOGI(TAG, "Starting Filling chars..");

    static const ble_uuid128_t userService = BLE_UUID128_INIT(0x0f, 0x4b, 0xe0, 0x8b, 0x89, 0x3c, 0x40, 0x97, 0xa3, 0xc5, 0x5e, 0x7c, 0xfc, 0xd2, 0x73, 0x70);   //"0f4be08b-893c-4097-a3c5-5e7cfcd27370"
    static const ble_uuid128_t configService = BLE_UUID128_INIT(0x0a, 0x4b, 0xe0, 0x8b, 0x89, 0x3c, 0x40, 0x97, 0xa3, 0xc5, 0x5e, 0x7c, 0xfc, 0xd2, 0x73, 0x70); //"0a4be08b-893c-4097-a3c5-5e7cfcd27370"

    static const ble_uuid128_t configChrNames BLE_UUID128_INIT(0x3f, 0xd3, 0x2b, 0xe3, 0xad, 0x57, 0x4f, 0x3a, 0xad, 0xca, 0xb9, 0x3f, 0x14, 0x79, 0x08, lastCharacteristic);

    // Configuration service
    memset(commonCharacteristics, 0, BTROBOT_CONFIG_MAX_CHARS * sizeof(ble_gatt_chr_def));
    commonCharacteristics[0] = {
        .uuid = &(configChrNames.u),
        .access_cb = &BtRobotController::configCallback,
        .arg = (void *)1,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 16};

    gatt_svcs[0] = {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &configService.u,
        .characteristics = commonCharacteristics};

    // User service

    gatt_svcs[1] = {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &userService.u,
        .characteristics = userCharacteristics};

    gatt_svcs[2] = {0};

    numUserCharacteristics = lenServicesConfig;

    // Generate Bluetooth's Structure for user characteristics
    for (uint32_t i = 0; i < lenServicesConfig; i++)
    {
        // Store locally
        // userConfigurations[i] = btServicesConfig[i];
        if (strlen(btServicesConfig[i].paramName) < BTROBOT_CONFIG_NAME_MAXLEN)
        {
            strcpy(characteristicNames[i], btServicesConfig[i].paramName);
        }

        ESP_LOGE(TAG, "Adding characteristic %lu, cb: %p", i, btServicesConfig[i].callback);
        generateUUID();
        userCharacteristics[i] =
            {
                .uuid = &(CHARACTERISTIC_UUID[i].u),
                .access_cb = &BtRobotController::commonCallback,
                .arg = (void *)i,
                .descriptors = userDescriptors[i],
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .min_key_size = 16U,
            };

        // Type of characteristic.
        userDescriptors[i][0] = {
            .uuid = &DESCRIPTORS_UUID[i][0].u,
            .att_flags = BLE_ATT_F_READ,
            .min_key_size = 16U,
            .access_cb = &BtRobotController::typeCallback,
            .arg = (void *)i,
        };
        userConfiguration[i].dataConfig = btServicesConfig[i].dataConfig;

        callbackMap[i] = btServicesConfig[i].callback;
    }

    userCharacteristics[lenServicesConfig] = {0};

    gatt_svcs[1].characteristics = userCharacteristics;

    internalBtInit();
    ble_gatts_count_cfg(gatt_svcs); // config all the gatt services that wanted to be used.
    ble_gatts_add_svcs(gatt_svcs);  // queues all services.
}

ble_uuid128_t BtRobotController::generateUUID()
{
    CHARACTERISTIC_UUID[lastCharacteristic] =
        BLE_UUID128_INIT(0x3f, 0xd3, 0x2b, 0xe3, 0xad, 0x57, 0x4f, 0x3a, 0xad, 0xca, 0xb9, 0x3f, 0x14, 0x79, 0x86, lastCharacteristic);

    for (uint8_t i = 0; i < BTROBOT_CONFIG_MAX_DESCRIPTORS; i++)
    {
        DESCRIPTORS_UUID[lastCharacteristic][i] =
            BLE_UUID128_INIT(0x3f, 0xd3, 0x2b, 0xe3, 0xad, 0x57, 0x4f, 0x3a, 0xad, 0xca, 0xb9, 0x3f, 0x14, 0x79, i, lastCharacteristic);
    }

    lastCharacteristic++;
    return CHARACTERISTIC_UUID[lastCharacteristic];
}

void BtRobotController::internalBtInit()
{
    nimble_port_init();
    ble_hs_cfg.sm_bonding = 1; // nimble library initialization.
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_DISP_ONLY;
    ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(internalRobotName)); // set BLE name.
    ble_svc_gap_init();                                              // initialize the gap service.
    ble_svc_gatt_init();                                             // initailize the gatt service.
}

uint32_t BtRobotController::runCallback(uint32_t id, void *data, uint32_t len, BtRobotOperationType operation)
// uint32_t BtRobotController::runCallback(uint32_t id, struct ble_gatt_access_ctxt *ctxt)
{
    if (callbackMap[id] == nullptr)
    {
        ESP_LOGE(TAG, "Error callback not defined! \n");
        return -1;
    }

    return callbackMap[id](data, len, operation);
}

int BtRobotController::commonCallback(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    BtRobotController &controller = BtRobotController::getBtRobotController();
    uint32_t id = (int)arg;

    ESP_LOGI(TAG, "Callback arg: %d\n", (int)arg);
    uint8_t om_len;
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        controller.runCallback(id, nullptr, 0, BTROBOT_OP_READ);
        os_mbuf_append(ctxt->om, ReadData, ReadDataLen);
        break;
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        om_len = OS_MBUF_PKTLEN(ctxt->om);
        memset(WriteData, 0, sizeof(WriteData));
        memcpy(WriteData, (char *)ctxt->om->om_data, om_len);
        WriteDataLen = om_len;
        ESP_LOGI(TAG, "Valor entrante: %d\n", WriteData[0]);
        controller.runCallback(id, WriteData, WriteDataLen, BTROBOT_OP_WRITE);

        break;
    case BLE_GATT_ACCESS_OP_READ_DSC:
        break;
    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        break;
    }
    return 0;
}

int BtRobotController::configCallback(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    BtRobotController &controller = BtRobotController::getBtRobotController();
    uint32_t id = (int)arg;

    ESP_LOGI(TAG, "ConfigCallback arg: %d\n", (int)arg);
    uint8_t om_len;

    char configData[BTROBOT_CONFIG_MAX_CHARS * BTROBOT_CONFIG_NAME_MAXLEN];
    char *configDataPointer = configData;

    for (uint32_t i = 0; i < controller.numUserCharacteristics; i++)
    {
        // configDataLen += strlen(controller.characteristicNames[i]) + 1;
        strcpy(configDataPointer, controller.characteristicNames[i]);
        configDataPointer += strlen(controller.characteristicNames[i]);
        *configDataPointer = ';';
        configDataPointer++;
    }
    //  "NOMBRE_1§Nombre_2§Nombre_3§": (len(nombre) + 1 )*numChar

    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        os_mbuf_append(ctxt->om, configData, strlen(configData));
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


int BtRobotController::typeCallback(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    BtRobotController &controller = BtRobotController::getBtRobotController();
    uint32_t id = (int)arg;

    ESP_LOGI(TAG, "typeCallback arg: %d\n", (int)arg);
    uint8_t om_len;
    
    uint8_t *p = (uint8_t*)&controller.userConfiguration[id].dataConfig;
    ESP_LOGI(TAG,"Data: \n");
    for(int i = 0; i < sizeof(struct dataType); i++)
    {
        ESP_LOGI(TAG,"%X ", *p++);
    }
    ESP_LOGI(TAG,"End Data: \n");
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_DSC:
        os_mbuf_append(ctxt->om, &controller.userConfiguration[id].dataConfig, sizeof(controller.userConfiguration[id].dataConfig));
        break;
    }
    return 0;
}

void BtRobotController::data_op_read(void *data, uint32_t len)
{
    if (len < BTROBOT_MAX_DATA_LEN)
    {
        memcpy(&ReadData, data, len);
        ReadDataLen = len;
    }
    else
    {
        ESP_LOGE(TAG, "Received data len bigger than %d", BTROBOT_MAX_DATA_LEN);
    }
}