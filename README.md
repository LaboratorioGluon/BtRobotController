# ESP32 Bluetooth Robot Controller

An ESP32 library to automatically create the services and characteristics to work easily with Bluetooth. This library allows one to initialize and work with bluetooth with just simple configuration.

The user shall only define the type of data that the app will exchange and create the corresponding callbacks.

## Requirements

One shall add the following configuration to the esp-idf configuration
```
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_CONTROLLER_ENABLED=y
```

## Example

```c

// Create our controller
BtRobotController& robotCtrl = BtRobotController::getBtRobotController();

uint32_t callBack1(void * data, uint32_t len, BtRobotOperationType operation)
{
	// Read means the BT App wants to read a value, so we send it to the BT Stack using 
    // 'BtRobotController::data_op_read(void* data, sizeof data)
    if(operation == BTROBOT_OP_READ)
    {
        BtRobotController::data_op_read(&gpioValue, sizeof(gpioValue));
    }
    // Write means the BT App wants to update a value in this ESP32
    // The data came in the input parameter data/len
    else if(operation == BTROBOT_OP_WRITE)
    {
        if(len == sizeof(uint8_t))
        {
            memcpy(&gpioValue, data, len);
            gpio_set_level(GPIO_NUM_5, gpioValue);
        }
    }

    return 0;
}

uint32_t CallBack2(void * data, uint32_t len, BtRobotOperationType operation)
{
	//....
}

// Creates the configuration for the Bluetooth.
struct BtRobotConfiguration robotConfig[]={
    {
        .paramName="Toggle LED", // The name that will appear in the BT app
        .callback = callBack1,	 // The callback function to read/write this characteristic.
        .dataConfig= {			 // Defines the type of data that it is exchanged in this characteristic.
            .dataType = BTROBOT_CONFIG_LATCH,
            .config = {}
        }
    },
    {
        .paramName="Push",
        .callback = CallBack2,
        .dataConfig= {
            .dataType = BTROBOT_CONFIG_FLOAT,
            .config{
                .floatSlide = {
                    .min = 0.0f,
                    .max = 10.0f,
                    .step = 0.1f
                }
            }
        }
    },



extern "C" void app_main()
{
    nvs_flash_init();
    
    // Initialize the bluetooth with name "MY_BT_DEVICE" and the configuration in robotConfig.
    robotCtrl.Init("MY_BT_DEVICE", robotConfig, 2);
    ...
}
```

### Video tutorial

21-oct-2023, 1h20