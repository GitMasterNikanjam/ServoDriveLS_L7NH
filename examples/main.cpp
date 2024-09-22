// For complie:
// g++ -o main main.cpp ../*.cpp ../../simpleEthercat/simpleEthercat.cpp -lsoem
// ###############################################
// Header Includes:
#include <iostream>                                         // standard I/O operations
#include <thread>                                           // For multi thread programming
#include <chrono>                                           // system clock functions
#include "../../simpleEthercat/simpleEthercat.h"        // EtherCAT functionality 
#include "../L7NH.h"                                           // Motor driver library
#include <cmath>

using namespace std;
// ###############################################
// Global Variables

// Ethercat object for general ethercat managements.
SIMPLE_ETHERCAT ETHERCAT;

//  Port name for ethercat slave connections.
const char port_name[] = "enp2s0";

// Motor Driver object.
L7NH motor1;

// ################################################
// Declare functions

void loop(void);
void motorSetup_PRE_OP(void);

int expectedWKC;
// #################################################
int main(void)
{
    if(ETHERCAT.init(port_name))
    {
        printf("Ethercat on %s succeeded.\n",port_name);
        if(ETHERCAT.configSlaves())
        {
            printf("Slaves Configured!\n");
            printf("Slave state are in PRE_OP state.\n");
            printf("%d slaves found and configured.\n",ETHERCAT.getSlaveCount());
            // motor1.autoSetup(2);
            motor1.setSlaveID(1);
            if(motor1.setModesOfOperationSDO(OPERATION_MODE_PT) == FALSE)
            {
                printf("motor operation can not set!\n");
            }

            motor1.assignRxPDO_rank(1);
            motor1.assignTxPDO_rank(1);

            uint32_t map_rx[2] = {MapValue_ControlWord, MapValue_TargetTorque};
            
            if(motor1.setRxPDO(2, map_rx) == FALSE)
                printf("motor setRxPDO can not set!\n");

            uint32_t map_tx[5] = {MapValue_StatusWord, MapValue_PositionActual, MapValue_VelocityActual, MapValue_OperationModeDisplay, MapValue_DigitalInput};
            
            if(motor1.setTxPDO(5, map_tx) == FALSE)
                printf("motor setTxPDO can not set!\n");

            ETHERCAT.configMap();
            printf("IOmap Configured.\n");
            ETHERCAT.configDc();
            printf("Distribution clock configured.\n");
            if(ETHERCAT.setSafeOperationalState())
            {
                printf("Slave state are in SafeOperation state.\n");
            }
            ETHERCAT.setOperationalState();
            ETHERCAT.listSlaves();
        }
        else
        {
            printf("Failed to config slaves\n");
            printf("Error: No slaves detected!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExecute as root maybe solve problem \n",port_name);
    }

    if(ETHERCAT.getState() == EC_STATE_OPERATIONAL)
    {
        printf("Operational state reached for all slaves.\n");
        loop();
        printf("\nRequest init state for all slaves\n");
        ETHERCAT.setInitState();
    }
    else
    {
        /*
        If not all slaves reach the operational state within the specified timeout, 
        the code prints a message indicating which slaves failed to reach the operational state.
        */
        printf("Not all slaves reached operational state.\n");
        ETHERCAT.showStates();
    }

    printf("close ethercat socket\n");
    ETHERCAT.close();

    return 0;
}

void loop(void)
{   
    motor1.servoOnSDO();

    // Using time point and system_clock
    std::chrono::time_point<std::chrono::system_clock> start, end;
    float t = 0;
    while(1)
    {
        t++;
        motor1.setControlWordPDO(0x000f);
        // motor1.setTargetPositionSDO(100000*sin(t/10000));
        motor1.setTargetTorquePDO(50);
        start = std::chrono::system_clock::now();
        bool flag_proccess = ETHERCAT.updateProccess();
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        if(flag_proccess)
        {
            // int32_t pos = motor1.getPositionActualPDO();
            int16_t pos = motor1.getTargetTorqueSDO();
            printf("rpm: %d\n", (int)t);
        }
        else
        {
            printf("Ethercat update proccess failed!\n");
        }        
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}