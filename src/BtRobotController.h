#ifndef __BTROBOTCONTROLLER_H__
#define __BTROBOTCONTROLLER_H__

#include <stdint.h>
#include <string.h>


#define BTROBOT_ROBOTNAME_MAXLEN 25

struct BtRobotConfiguration{

};


class BtRobotController
{

public:
    BtRobotController();

    void Init(char * robotName, struct BtRobotConfiguration btServicesConfig);

private:

    /**
     * Initialize the basic nimBLE features of ESP32
    */
    void internalBtInit();

    char internalRobotName[BTROBOT_ROBOTNAME_MAXLEN];

};

#endif //__BTROBOTCONTROLLER_H__