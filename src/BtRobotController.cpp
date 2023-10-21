
#include "BtRobotController.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "esp_assert.h"

#include <string.h>

static const char * TAG = "BTROBOTCONTROLLER";

BtRobotController::BtRobotController()
{
    
}


void BtRobotController::Init(char * robotName, struct BtRobotConfiguration btServicesConfig)
{
    if(strlen(robotName) > BTROBOT_ROBOTNAME_MAXLEN-1)
    {
        ESP_LOGE(TAG, "Error RobotName longer than %d", BTROBOT_ROBOTNAME_MAXLEN-1);
        return;
    }

    strcpy(internalRobotName, robotName);

    internalBtInit();
}


void BtRobotController::internalBtInit()
{
    nimble_port_init();                                     // nimble library initialization.
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(internalRobotName)); // set BLE name.
    ble_svc_gap_init();                                     // initialize the gap service.
    ble_svc_gatt_init();                                    // initailize the gatt service.
}