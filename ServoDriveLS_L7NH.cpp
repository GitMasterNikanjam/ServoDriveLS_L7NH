#include "ServoDriveLS_L7NH.h"

// use namespace for prevent confilict to other sources.
namespace _L7NH
{
    // union structure for save variable in any format.
    union UNION_DATA
    {
        uint8_t ui8a[100];
        uint8_t ui8;
        uint16_t ui16;
        uint32_t ui32;
        int8_t i8;
        int16_t i16;
        int32_t i32;
        float f;
        char s[100];
    }udata;
}

using namespace _L7NH;

void L7NH::statesClear(void)
{
    states.run = 0;
    states.ctlmode = OPERATION_MODE_NO;
    states.ethMode = EC_STATE_ERROR;
    states.posAct = 0 ;
    states.posActRaw = 0;
    states.statusword = 0;
    states.trqAct = 0;
    states.trqActRaw = 0;
    states.trqActRaw_cmd = 0;
    states.velAct = 0;
    states.velActRaw = 0;
    states.velActRaw_cmd = 0;
}

bool L7NH::setSlaveID(int ID_num)
{
    slaveID = ID_num;

    if(std::string(ec_slave[ID_num].name) == "")
    {
        errorMessage = "Error L7NH: Motor drive can not detected.";
        return false;
    }

    PulsePerRevolution = getEncoderPulsePerRevolution();

    if(PulsePerRevolution == 0)
    {
        errorMessage = "Error L7NH: Motor drive communication problem.";
        return false;
    }

    return true;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set/Get TX/RX PDO configurations:

bool L7NH::assignRxPDO_rank(int pdo_rank)
{
    // Read state of all slaves in ethercat.
    ec_readstate();

    if(ec_slave[slaveID].state != EC_STATE_PRE_OP)
        return FALSE;

    int wkc;                // Working counter     
    uint16_t index;         // Certain index for assign in syncManager
    uint8_t data;            

    switch(pdo_rank)
    {
        case 1:
            index = Index_ReceivePDOMapping_1st;
        break;
        case 2:
            index = Index_ReceivePDOMapping_2st;
        break;
        case 3:
            index = Index_ReceivePDOMapping_3st;
        break;
        case 4:
            index = Index_ReceivePDOMapping_4st;
        break;
        default:
            return FALSE;
    }
    // Set subindex 0 to 0 for syncManagerAssignedRxPDO
    data = 0;
    ec_SDOwrite(slaveID, Index_syncManagerAssignedRxPDO, 0, FALSE, 1, &data, EC_TIMEOUTRXM);

    // Assign RxPDO index.
    wkc = ec_SDOwrite(slaveID, Index_syncManagerAssignedRxPDO, 1, FALSE, 2, &index, EC_TIMEOUTRXM);

    // Set subindex 0 to 1 for syncManagerAssignedRxPDO
    data = 1;
    ec_SDOwrite(slaveID, Index_syncManagerAssignedRxPDO, 0, FALSE, 1, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        return FALSE;
    }
        
    RxPDO_rank = pdo_rank;
    return TRUE;
}

bool L7NH::assignTxPDO_rank(int pdo_rank)
{
    // Read state of all slaves in ethercat.
    ec_readstate();

    if(ec_slave[slaveID].state != EC_STATE_PRE_OP)
        return FALSE;

    int wkc;                // Working counter     
    uint16_t index;         // Certain index for assign in syncManager
    uint8_t data;      

    switch(pdo_rank)
    {
        case 1:
            index = Index_TransmitPDOMapping_1st;
        break;
        case 2:
            index = Index_TransmitPDOMapping_2st;
        break;
        case 3:
            index = Index_TransmitPDOMapping_3st;
        break;
        case 4:
            index = Index_TransmitPDOMapping_4st;
        break;
        default:
            return FALSE;
    }
    // Set subindex 0 to 0 for syncManagerAssignedRxPDO
    data = 0;
    ec_SDOwrite(slaveID, Index_syncManagerAssignedTxPDO, 0, FALSE, 1, &data, EC_TIMEOUTRXM);

    // Assign RxPDO index.
    wkc = ec_SDOwrite(slaveID, Index_syncManagerAssignedTxPDO, 1, FALSE, 2, &index, EC_TIMEOUTRXM);

    // Set subindex 0 to 1 for syncManagerAssignedRxPDO
    data = 1;
    ec_SDOwrite(slaveID, Index_syncManagerAssignedTxPDO, 0, FALSE, 1, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
    {
        return FALSE;
    }
        
    TxPDO_rank = pdo_rank;
    return TRUE;
}

uint16_t L7NH::getRxPDO_rank(void)
{
    int wkc;           
    int size = 2;
    uint16_t data;
    wkc = ec_SDOread(slaveID, Index_syncManagerAssignedRxPDO, 1, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

uint16_t L7NH::getTxPDO_rank(void)
{
    int wkc;
    int size = 2;
    uint16_t data;
    wkc = ec_SDOread(slaveID, Index_syncManagerAssignedTxPDO, 1, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

bool L7NH::setRxPDO(uint8_t num_enteries, uint32_t* mapping_entry)
{
    _RxMapFlag[0] = 0;
    _RxMapFlag[1] = 0;
    _RxMapFlag[2] = 0;
    _RxMapFlag[3] = 0;
    _RxMapFlag[4] = 0;
    _RxMapFlag[5] = 0;

    int wkc;
    uint16_t index;

    switch(RxPDO_rank)
    {
        case 1:
            index = Index_ReceivePDOMapping_1st;
        break;
        case 2:
            index = Index_ReceivePDOMapping_2st;
        break;
        case 3:
            index = Index_ReceivePDOMapping_3st;
        break;
        case 4:
            index = Index_ReceivePDOMapping_4st;
        break;
        default:
            return FALSE;
    }

    wkc = ec_SDOwrite(slaveID, index, 0, FALSE, 1, &num_enteries, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    uint8_t offset = 0;

    for(int subindex=1; subindex <= num_enteries; subindex++)
    {
        wkc = ec_SDOwrite(slaveID, index, subindex, FALSE, 4, &mapping_entry[subindex - 1], EC_TIMEOUTRXM);
        osal_usleep(10000);     // delay 10ms

        if(wkc <= 0)
            return FALSE;

        switch(mapping_entry[subindex - 1])
        {
            case MapValue_ControlWord:
                RxMapOffset_ControlWord = offset;
                offset += 2;
                _RxMapFlag[0] = 1;
            break;
            case MapValue_DigitalOutput_PhysicalOutputs:
                RxMapOffset_DigitalOutput_PhysicalOutputs = offset;
                offset += 4;
                _RxMapFlag[4] = 1;
            break;
            case MapValue_ModesOfOperation:
                RxMapOffset_ModesOfOperation = offset;
                offset += 1;
                _RxMapFlag[5] = 1;
            break;
            case MapValue_TargetPosition:
                RxMapOffset_TargetPosition = offset;
                offset += 4;
                _RxMapFlag[1] = 1;
            break;
            case MapValue_TargetTorque:
                RxMapOffset_TargetTorque = offset;
                offset += 2;
                _RxMapFlag[3] = 1;
            break;
            case MapValue_TargetVelocity:
                RxMapOffset_TargetVelocity = offset;
                offset += 4;
                _RxMapFlag[2] = 1;
            break;
            default:
                return FALSE;
        }
    }

    return TRUE;
}

bool L7NH::setTxPDO(uint8_t num_enteries, uint32_t* mapping_entry)
{
    _TxMapFlag[0] = 0;
    _TxMapFlag[1] = 0;
    _TxMapFlag[2] = 0;
    _TxMapFlag[3] = 0;
    _TxMapFlag[4] = 0;
    _TxMapFlag[5] = 0;
    _TxMapFlag[6] = 0;
    _TxMapFlag[7] = 0;
    _TxMapFlag[8] = 0;
    _TxMapFlag[9] = 0;
    _TxMapFlag[10] = 0;
    _TxMapFlag[11] = 0;

    int wkc;
    uint16_t index;

    switch(TxPDO_rank)
    {
        case 1:
            index = Index_TransmitPDOMapping_1st;
        break;
        case 2:
            index = Index_TransmitPDOMapping_2st;
        break;
        case 3:
            index = Index_TransmitPDOMapping_3st;
        break;
        case 4:
            index = Index_TransmitPDOMapping_4st;
        break;
        default:
            return FALSE;
    }

    wkc = ec_SDOwrite(slaveID, index, 0, FALSE, 1, &num_enteries, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    uint8_t offset = 0;

    for(int subindex=1; subindex <= num_enteries; subindex++)
    {
        wkc = ec_SDOwrite(slaveID, index, subindex, FALSE, 4, &mapping_entry[subindex - 1], EC_TIMEOUTRXM);
        osal_usleep(10000);   // delay 10ms

        if(wkc <= 0)
        {  
            return FALSE;
        }

        switch(mapping_entry[subindex - 1])
        {
            case MapValue_StatusWord:
                TxMapOffset_StatusWord = offset;
                offset += 2;
                _TxMapFlag[0] = 1;
            break;
            case MapValue_PositionActualInternal:
                TxMapOffset_PositionActualInternal = offset;
                offset += 4;
                _TxMapFlag[1] = 1;
            break;
            case MapValue_PositionActual:
                TxMapOffset_PositionActual = offset;
                offset += 4;
                _TxMapFlag[2] = 1;
            break;
            case MapValue_VelocityActual:
                TxMapOffset_VelocityActual = offset;
                offset += 4;
                _TxMapFlag[3] = 1;
            break;
            case MapValue_TorqueActual:
                TxMapOffset_TorqueActual = offset;
                offset += 2;
                _TxMapFlag[4] = 1;
            break;
            case MapValue_PositionDemandInternal:
                TxMapOffset_PositionDemandInternal = offset;
                offset += 4;
                _TxMapFlag[5] = 1;
            break;
            case MapValue_PositionDemand:
                TxMapOffset_PositionDemand = offset;
                offset += 4;
                _TxMapFlag[6] = 1;
            break;
            case MapValue_VelocityDemand:
                TxMapOffset_VelocityDemand = offset;
                offset += 4;
                _TxMapFlag[7] = 1;
            break;
            case MapValue_FeedbackSpeed:
                TxMapOffset_FeedbackSpeed = offset;
                offset += 2;
                _TxMapFlag[8] = 1;
            break;
            case MapValue_TorqueDemand:
                TxMapOffset_TorqueDemand = offset;
                offset += 2;
                _TxMapFlag[9] = 1;
            break;
            case MapValue_DigitalInput:
                TxMapOffset_DigitalInput = offset;
                offset += 4;
                _TxMapFlag[10] = 1;
            break;
            case MapValue_OperationModeDisplay:
                TxMapOffset_OperationModeDisplay = offset;
                offset += 1;
                _TxMapFlag[11] = 1;
            break;
            default:
                return FALSE;
        }
    }

    return TRUE;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++
// Save/Restore:

bool L7NH::saveParamsAll(void)
{
    int wkc;
    uint32_t data = SAVE;
    wkc = ec_SDOwrite(slaveID, Index_StoreParameters, SubIndex_StoreParametersAll, FALSE, 4, &data, EC_TIMEOUTRXM);
    
    osal_usleep(1500000);
    
    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool L7NH::saveParamsCommunication(void)
{
    int wkc;
    uint32_t data = SAVE;
    wkc = ec_SDOwrite(slaveID, Index_StoreParameters, SubIndex_StoreParametersCommunication, FALSE, 4, &data, EC_TIMEOUTRXM);
    
    osal_usleep(1500000);

    if(wkc <= 0)
        return FALSE;

    return TRUE;

}

bool L7NH::saveParamsCiA402(void)
{
    int wkc;
    uint32_t data = SAVE;
    wkc = ec_SDOwrite(slaveID, Index_StoreParameters, SubIndex_StoreParametersCiA402, FALSE, 4, &data, EC_TIMEOUTRXM);
    
    osal_usleep(1500000);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool L7NH::saveParamsSpecific(void)
{
    int wkc;
    uint32_t data = SAVE;
    wkc = ec_SDOwrite(slaveID, Index_StoreParameters, SubIndex_StoreParametersSpecific, FALSE, 4, &data, EC_TIMEOUTRXM);
    
    osal_usleep(1500000);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool L7NH::loadParamsAll(void)
{
    int wkc;
    uint32_t data = LOAD;
    wkc = ec_SDOwrite(slaveID, Index_RestoreDefaultParameters, SubIndex_RestoreDefaultParametersAll, FALSE, 4, &data, EC_TIMEOUTRXM);
    
    osal_usleep(1500000);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

 bool L7NH::loadParamsCommunication(void)
 {
    int wkc;
    uint32_t data = LOAD;
    wkc = ec_SDOwrite(slaveID, Index_RestoreDefaultParameters, SubIndex_RestoreDefaultParametersCommunication, FALSE, 4, &data, EC_TIMEOUTRXM);
    
    osal_usleep(1500000);   

    if(wkc <= 0)
        return FALSE;

    return TRUE;
 }

bool L7NH::loadParamsCiA402(void)
{
    int wkc;
    uint32_t data = LOAD;
    wkc = ec_SDOwrite(slaveID, Index_RestoreDefaultParameters, SubIndex_RestoreDefaultParametersCiA402, FALSE, 4, &data, EC_TIMEOUTRXM);
    
    osal_usleep(1500000);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool L7NH::loadParamsSpecific(void)
{
    int wkc;
    uint32_t data = LOAD;
    wkc = ec_SDOwrite(slaveID, Index_RestoreDefaultParameters, SubIndex_RestoreDefaultParametersSpecific, FALSE, 4, &data, EC_TIMEOUTRXM);
    
    osal_usleep(1500000);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}
    
// +++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set/Get Driver ID:

bool L7NH::setMotorID(uint16_t ID)
{
    int wkc;

    wkc = ec_SDOwrite(slaveID, Index_MotorID, 0, FALSE, 2, &ID, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

int16_t L7NH::getMotorID(void)
{
    int wkc;
    int size = 2;
    uint16_t ID;
    wkc = ec_SDOread(slaveID, Index_MotorID, 0, FALSE, &size, &ID, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
        return -1;

    return (int16_t)ID;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++
// Get Driver Node ID:

int16_t L7NH::getNodeID(void)
{
    int wkc;
    int size = 2;
    uint16_t ID;
    wkc = ec_SDOread(slaveID, Index_NodeID, 0, FALSE, &size, &ID, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
    {
        errorMessage = "Error L7NH: There is a problem for ethercat connection.";
        std::cout << errorMessage << std::endl;
        return -1;
    } 

    return (int16_t)ID;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set/Get encoder configuration:

bool L7NH::setEncoderType(uint16_t type)
{
    int wkc;
    
    wkc = ec_SDOwrite(slaveID, Index_EncoderType, 0, FALSE, 2, &type, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

int16_t L7NH::getEncoderType(void)
{
    int wkc;
    int size = 2;
    uint16_t type;
    wkc = ec_SDOread(slaveID, Index_EncoderType, 0, FALSE, &size, &type, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
        return -1;

    return (int16_t)type;
}

uint32_t L7NH::getEncoderPulsePerRevolution(void)
{
    int wkc;
    int size = 4;
    uint32_t data;
    wkc = ec_SDOread(slaveID, Index_EncoderPulsePerRevolution, 0, FALSE, &size, &data, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
        return 0;

    return data;
}

uint8_t L7NH::getRotationDirectionSelect(void)
{
    int wkc;
    int size = 1;
    uint8_t dir;
    wkc = ec_SDOread(slaveID, Index_RotationDirectionSelect, 0, FALSE, &size, &dir, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
        return 2;

    return dir;
}

bool L7NH::setRotationDirectionSelect(uint16_t dir)
{
    int wkc;
    wkc = ec_SDOwrite(slaveID, Index_RotationDirectionSelect, 0, FALSE, 2, &dir, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
        return FALSE;

    return  TRUE;
}

uint16_t L7NH::getEncoderConfiguration(void)
{
    int wkc;
    int size = 2;
    uint16_t data;
    wkc = ec_SDOread(slaveID, Index_EncoderConfiguration, 0, FALSE, &size, &data, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
        return 2;

    return data;
}

bool L7NH::setEncoderConfiguration(uint16_t config)
{
    int wkc;
    wkc = ec_SDOwrite(slaveID, Index_EncoderConfiguration, 0, FALSE, 2, &config, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++
// Servo ON/OFF:

void L7NH::servoOnSDO(void)
{
    setControlWordSDO(0x0006);
    osal_usleep(10000);

    setControlWordSDO(0x0007);
    osal_usleep(10000);

    setControlWordSDO(0x000F);
    osal_usleep(10000);
}

void L7NH::servoOnPDO(void)
{
    setControlWordPDO(0x0006);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(10000);

    setControlWordPDO(0x0007);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(10000);

    setControlWordPDO(0x000F);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(10000);
}

void L7NH::servoOffSDO(void)
{
    setControlWordSDO(0x0006);
    osal_usleep(10000);

    setControlWordSDO(0x0000);
    osal_usleep(10000);
}

void L7NH::servoOffPDO(void)
{
    setControlWordPDO(0x0006);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(10000);

    setControlWordPDO(0x0000);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(10000);
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++
// Controlword, statusword, Operation mode and machin state

void L7NH::printStateMachine(void)
{
    uint16_t statusWord = getStatuseWordSDO();
    stateUpdate(statusWord);

    switch(stateMachine)
    {
        case NotReadyToSwitchOn:
            printf("StateMachine: NotReadyToSwitchOn\n");
        break;
        case SwitchOnDisabled:
            printf("StateMachine: SwitchOnDisabled\n");
        break;
        case ReadyToSwitchOn:
            printf("StateMachine: ReadyToSwitchOn\n");
        break;
        case SwitchedOn:
            printf("StateMachine: SwitchedOn\n");
        break;
        case OperationEnabled:
            printf("StateMachine: OperationEnabled\n");
        break;
        case QuickStopActive:
            printf("StateMachine: QuickStopActive\n");
        break;
        case FaultReactionActive:
            printf("StateMachine: FaultReactionActive\n");
        break;
        case Fault:
            printf("StateMachine: Fault\n");
        break;
        default:
            printf("StateMachine: None\n");
    }
}

bool L7NH::setModesOfOperationSDO(int8_t mode)
{
    int wkc;
    wkc = ec_SDOwrite(slaveID, Index_ModesOfOperation, 0, FALSE, 1, &mode, EC_TIMEOUTRXM);
    osal_usleep(1000);

    if(wkc <= 0)
    {
        return FALSE;
    }
    if(getModeOfOperationSDO() != mode)
    {
        return FALSE;
    }

    // Update control mode for states.
    states.ctlmode = mode;

    return TRUE;
}

int8_t L7NH::getModeOfOperationSDO(void)
{
    int wkc;
    int size = 1;
    int8_t mode;
    wkc = ec_SDOread(slaveID, Index_ModesOfOperation, 0, FALSE, &size, &mode, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return mode;
}

bool L7NH::setModesOfOperationPDO(int8_t mode)
{
    if(_RxMapFlag[5] == 0)
        return false;

        // Access the process data outputs for the specified slave
    uint8 *outputs = ec_slave[slaveID].outputs;
    
    // Write the control word to the specified offset
    int8_t *ptr = (int8_t *)(outputs + RxMapOffset_ModesOfOperation);
    *ptr = mode;

    return true;
}

bool L7NH::setControlWordPDO(uint16 control_word) 
{
    if(_RxMapFlag[0] == 0)
        return false;

    // Access the process data outputs for the specified slave
    uint8 *outputs = ec_slave[slaveID].outputs;
    
    // Write the control word to the specified offset
    uint16 *control_word_ptr = (uint16 *)(outputs + RxMapOffset_ControlWord);
    *control_word_ptr = control_word;

    return true;
}

uint16 L7NH::getStatuseWordPDO(void) 
{
    if(_TxMapFlag[0] == 0)
        return 0;

    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[slaveID].inputs;
    
    // Read the status word from the specified offset
    uint16 *status_word_ptr = (uint16 *)(inputs + TxMapOffset_StatusWord);
    return *status_word_ptr;
}

int8_t L7NH::getOperationModeDisplayPDO(void)
{
    if(_TxMapFlag[11] == 0)
        return 0;

    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[slaveID].inputs;
    
    // Read the status word from the specified offset
    int8 *ptr = (int8 *)(inputs + TxMapOffset_OperationModeDisplay);
    return *ptr;
}

bool L7NH::setControlWordSDO(uint16 control_word)
{
    int wkc = ec_SDOwrite(slaveID, Index_Controlword, 0, FALSE, 1, &control_word, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

uint16_t L7NH::getStatuseWordSDO(void)
{
    int wkc;
    int size = 2;
    uint16_t data;
    wkc = ec_SDOread(slaveID, Index_Statusword, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

void L7NH::stateUpdate(uint16_t statusWord) {

    if(statusWord & StatusWord_MainPowerIsOn)
        state_MainPowerOn = TRUE;
    else
        state_MainPowerOn = FALSE;

    if(statusWord & StatusWord_WarningIsOccurred)
        state_WarningIsOccurred = TRUE;
    else
        state_WarningIsOccurred = FALSE;

    if ((statusWord & 0x004F) == StatusWord_NotReadyToSwitchOn) 
    {
        stateMachine = NotReadyToSwitchOn;
    } 
    else if ((statusWord & 0x004F) == StatusWord_SwitchOnDisabled) 
    {
        stateMachine = SwitchOnDisabled;
    } 
    else if ((statusWord & 0x006F) == StatusWord_ReadyToSwitchOn) 
    {
        stateMachine = ReadyToSwitchOn;
    } 
    else if ((statusWord & 0x006F) == StatusWord_SwitchedOn) 
    {
        stateMachine = SwitchedOn;
    } 
    else if ((statusWord & 0x006F) == StatusWord_OperationEnabled) 
    {
        stateMachine = OperationEnabled;
    } 
    else if ((statusWord & 0x006F) == StatusWord_QuickStopActive) 
    {
        stateMachine = QuickStopActive;
    } 
    else if ((statusWord & 0x004F) == StatusWord_FaultReactionActive) 
    {
        stateMachine = FaultReactionActive;
    } 
    else if ((statusWord & 0x004F) == StatusWord_Fault) 
    {
        stateMachine = Fault;
    }
    else
    {
        stateMachine = None;        // Default state
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++
// Get/Set Position:

bool L7NH::setTargetPositionSDO(int32_t position)
{
    int wkc = ec_SDOwrite(slaveID, Index_TargetPosition, 0, FALSE, 4, &position, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool L7NH::setTargetPositionPDO(int32_t position)
{
    if(_RxMapFlag[1] == 0)
        return false;

    // Access the process data outputs for the specified slave
    uint8 *outputs = ec_slave[slaveID].outputs;
    
    // Write the position to the specified offset
    int32_t *position_ptr = (int32_t *)(outputs + RxMapOffset_TargetPosition);
    *position_ptr = position;

    return true;
}

int32_t L7NH::getPositionActualSDO(void)
{
    int wkc;
    int size = 4;
    int32_t data;
    wkc = ec_SDOread(slaveID, Index_PositionActualValue, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

int32_t L7NH::getPositionActualPDO(void)
{
    if(_TxMapFlag[2] == 0)
        return 0;

    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[slaveID].inputs;
    
    // Read from the specified offset
    int32_t data = *(int32_t *)(inputs + TxMapOffset_PositionActual);

    return data;
}

int32_t L7NH::getPositionDemandInternalSDO(void)
{
    int wkc;
    int size = 4;
    int32_t data;
    wkc = ec_SDOread(slaveID, Index_PositionDemandInternalValue, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++
// Set/Get Torque:

void L7NH::setRatedTorque(double value)
{
    ratedTorque = value;
}

bool L7NH::setMaximumTorqueSDO(uint16_t torque)
{
    int wkc = ec_SDOwrite(slaveID, Index_MaximumTorque, 0, FALSE, 2, &torque, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool L7NH::setTorqueSlopeSDO(uint32_t slope)
{
    int wkc = ec_SDOwrite(slaveID, Index_TorqueSlope, 0, FALSE, 4, &slope, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool L7NH::setTorqueLimitFunctionSelectSDO(uint16_t value)
{
    int wkc = ec_SDOwrite(slaveID, Index_TorqueLimitFunctionSelect, 0, FALSE, 2, &value, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool L7NH::setTargetTorqueSDO(int16_t torque)
{
    int wkc = ec_SDOwrite(slaveID, Index_TargetTorque, 0, FALSE, 2, &torque, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool L7NH::setTargetTorquePDO(int16_t torque)
{
    if(_RxMapFlag[3] == 0)
        return false;

    // Access the process data outputs for the specified slave
    uint8 *outputs = ec_slave[slaveID].outputs;
    
    // Write the torque to the specified offset
    int16_t *torque_ptr = (int16_t *)(outputs + RxMapOffset_TargetTorque);
    *torque_ptr = torque;

    return true;
}

int16_t L7NH::getTorqueActualPDO(void)
{
    if(_TxMapFlag[4] == 0)
        return 0;

    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[slaveID].inputs;
    
    // Read from the specified offset
    int32_t data = *(int32_t *)(inputs + TxMapOffset_TorqueActual);

    return data;
}

int16_t L7NH::getTorqueDemandPDO(void)
{
    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[slaveID].inputs;
    
    // Read from the specified offset
    int32_t data = *(int32_t *)(inputs + TxMapOffset_TorqueDemand);

    return data;
}

int16_t L7NH::getTargetTorqueSDO(void)
{
    int wkc;
    int size = 2;
    int16_t data;
    wkc = ec_SDOread(slaveID, Index_TargetTorque, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <=0)
        return 0;

    return data;
}

int16_t L7NH::getTorqueActualSDO(void)
{
    int wkc;
    int size = 2;
    int16_t data;
    wkc = ec_SDOread(slaveID, Index_TorqueActualValue, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <=0)
        return 0;

    return data; 
}

int16_t L7NH::getTorqueDemandSDO(void)
{
    int wkc;
    int size = 2;
    int16_t data;
    wkc = ec_SDOread(slaveID, Index_TorqueDemandValue, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <=0)
        return 0;

    return data; 
}

// ++++++++++++++++++++++++++++++++++++++++++++++++
// Get supported modes:

uint32_t L7NH::getSupportedDriveModes(bool show_op)
{
    int wkc;
    int size = 4;
    uint32_t data;
    wkc = ec_SDOread(slaveID, Index_SupportedDriveModes, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    if(show_op)
    {
        if(data & (1 << 0))
        {
            printf("Profile position mode supported.\n");
        }

        if(data & (1 << 1))
        {
            printf("Velocity mode supported.\n");
        }

        if(data & (1 << 2))
        {
            printf("Profile velocity mode supported.\n");
        }

        if(data & (1 << 3))
        {
            printf("Profile torque mode supported.\n");
        }

        if(data & (1 << 5))
        {
            printf("Homing mode supported.\n");
        }

        if(data & (1 << 6))
        {
            printf("Interpolated Position mode supported.\n");
        }

        if(data & (1 << 7))
        {
            printf("Cyclic synchronous position mode supported.\n");
        }

        if(data & (1 << 8))
        {
            printf("Cyclic synchronous velocity mode supported.\n");
        }

        if(data & (1 << 9))
        {
            printf("Cyclic synchronous torque mode supported.\n");
        }
    }
    
    return data;
}

int32_t L7NH::getPositionActualInternalSDO(void)
{
    int wkc;
    int size = 4;
    int32_t data;
    wkc = ec_SDOread(slaveID, Index_PositionActualInternalValue, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

int32_t L7NH::getPositionActualInternalPDO(void)
{
    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[slaveID].inputs;
    
    // Write the torque to the specified offset
    int32_t data = *(int32_t *)(inputs + TxMapOffset_PositionActualInternal);

    return data;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++
// Set/Get Velocity:

int32_t L7NH::getVelocityActualSDO(void)
{
    int wkc;
    int size = 4;
    int32_t data;
    wkc = ec_SDOread(slaveID, Index_VelocityActualValue, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

int32_t L7NH::getVelocityDemandSDO(void)
{
    int wkc;
    int size = 4;
    int32_t data;
    wkc = ec_SDOread(slaveID, Index_VelocityDemandValue, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

int32_t L7NH::getVelocityActualPDO(void)
{
    if(_TxMapFlag[3] == 0)
        return 0;

    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[slaveID].inputs;
    
    // Write the torque to the specified offset
    int32_t data = *(int32_t *)(inputs + TxMapOffset_VelocityActual);

    return data;
}

int32_t L7NH::getVelocityDemandPDO(void)
{
    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[slaveID].inputs;
    
    // Write the torque to the specified offset
    int32_t data = *(int32_t *)(inputs + TxMapOffset_VelocityDemand);

    return data;
}

bool L7NH::setTargetVelocityPDO(int32_t velocity)
{
    if(_RxMapFlag[2] == 0)
        return false;

    // Access the process data outputs for the specified slave
    uint8 *outputs = ec_slave[slaveID].outputs;
    
    // Write the torque to the specified offset
    int32_t *velocity_ptr = (int32_t *)(outputs + RxMapOffset_TargetVelocity);
    *velocity_ptr = velocity;

    return true;
}

bool L7NH::setTargetVelocitySDO(int32_t velocity)
{
    int wkc = ec_SDOwrite(slaveID, Index_TargetVelocity, 0, FALSE, 4, &velocity, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

bool L7NH::setMaxProfileVelocitySDO(uint32_t velocity)
{
    int wkc = ec_SDOwrite(slaveID, Index_MaxProfileVelocity, 0, FALSE, 4, &velocity, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;    
}

bool L7NH::setSpeedLimitFunctionSelect(bool state)
{
    uint16_t data;
    if(state == true)
    {
        data = 1;
    }
    else
    {
        data = 0;
    }

    int wkc = ec_SDOwrite(slaveID, Index_SpeedLimitFunctionSelect, 0, FALSE, 2, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;    
}

bool L7NH::setSpeedLimitValueAtTorqueControlMode(uint16_t value)
{
    int wkc = ec_SDOwrite(slaveID, Index_SpeedLimitValueAtTorqueControlMode, 0, FALSE, 2, &value, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE; 
}

int16_t L7NH::getFeedbackSpeedSDO(void)
{
    int wkc;
    int size = 2;
    int16_t data;
    wkc = ec_SDOread(slaveID, Index_FeedbackSpeed, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;    
}

int16_t L7NH::getFeedbackSpeedPDO(void)
{
    if(_TxMapFlag[8] == 0)
    return false;

    // Access the process data inputs for the specified slave
    uint8 *inputs = ec_slave[slaveID].inputs;
    
    // Write the torque to the specified offset
    int32_t data = *(int32_t *)(inputs + TxMapOffset_FeedbackSpeed);

    return data;   
}

uint16_t L7NH::getMotorRatedSpeed(void)
{
    int wkc;
    int size = 2;
    uint16_t data;
    wkc = ec_SDOread(slaveID, Index_MotorRatedSpeed, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;    
}

bool L7NH::setProfileAccelerationSDO(int32_t acc)
{
    int wkc = ec_SDOwrite(slaveID, Index_ProfileAcceleration, 0, FALSE, 4, &acc, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;    
}

bool L7NH::setProfileDecelerationSDO(int32 acc)
{
    int wkc = ec_SDOwrite(slaveID, Index_ProfileDeceleration, 0, FALSE, 4, &acc, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;      
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++
// Homing:

int L7NH::startHoming(void) {

    servoOnSDO();
    // Shutdown the drive first to clear any errors
    if (setControlWordSDO(0x0006) != 0) {
        return -1;
    }

    // Switch on the drive and enable operation
    if (setControlWordSDO(0x000F) != 0) {
        return -1;
    }

    // Start the homing process
    if (setControlWordSDO(0x001F) != 0) { // 0x001F is the command to start homing in CiA 402
        return -1;
    }

    return 0;
}

bool L7NH::setHomeOffset(int32_t offset)
{
    int wkc;
    wkc = ec_SDOwrite(slaveID, Index_HomeOffset, 0, FALSE, 4, &offset, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

int32_t L7NH::getHomeoffset(void)
{
    int wkc;
    int32_t data;
    int size = 4;
    wkc = ec_SDOread(slaveID, Index_HomeOffset, 0, FALSE, &size, &data, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return 0;

    return data;
}

bool L7NH::setHomingMethod(int8_t method)
{
    int wkc;
    wkc = ec_SDOwrite(slaveID, Index_HomingMethod, 0, FALSE, 1, &method, EC_TIMEOUTRXM);

    if(wkc <= 0)
        return FALSE;

    return TRUE;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++
// Auto configuration options:

bool L7NH::autoSetup(int ID, int setup_num)
{
    setSlaveID(ID);

    if(setup_num == 1)
    {
        if(setModesOfOperationSDO(OPERATION_MODE_CST) == FALSE)
        {
            return FALSE;
        }
        
        assignRxPDO_rank(1);
        assignTxPDO_rank(1);

        uint32_t map_rx[2] = {MapValue_ControlWord, MapValue_TargetTorque};
        uint32_t map_tx[5] = {MapValue_StatusWord, MapValue_PositionActual, MapValue_VelocityActual, MapValue_OperationModeDisplay, MapValue_DigitalInput};
            
        if(setRxPDO(sizeof(map_rx)/4, map_rx) == FALSE)
            return FALSE;

        if(setTxPDO(sizeof(map_tx)/4, map_tx) == FALSE)
            return FALSE;
    } 
    else
    {
        return false;
    }

    PulsePerRevolution = 0;
    PulsePerRevolution = getEncoderPulsePerRevolution();

    if(PulsePerRevolution == 0)
        return false;

    return TRUE;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++

bool L7NH::updateStatesPDO(void)
{
    states.posActRaw = getPositionActualPDO();
    states.velActRaw = getVelocityActualPDO();
    states.trqActRaw = getTorqueActualPDO();
    states.statusword = getStatuseWordPDO();
    states.ctlmode = getOperationModeDisplayPDO();

    states.posAct = ((double)states.posActRaw / (double)PulsePerRevolution) * 360.0;
    states.velAct = ((double)states.velActRaw / (double)PulsePerRevolution) * 60.0;
    states.trqAct = (double)states.trqActRaw * 0.1;

    /*
    Servo On Conditions for statusword:
    Bit 0 (Ready to switch on): Should be set (1).
    Bit 1 (Switched on): Should be set (1).
    Bit 2 (Operation enabled): Should be set (1).
    Bit 3 (Fault): Should not be set (0).

    Servo Off Conditions for statusword:
    Bit 0 (Ready to switch on): Could be set (1) but not necessarily.
    Bit 1 (Switched on): Should not be set (0).
    Bit 2 (Operation enabled): Should not be set (0).
    Bit 3 (Fault): Should be set (1).
    */

    if(states.statusword & 0b110)
    {
        states.run = true;
    }
    else
    {
        states.run = false;
    }

    // Refresh states of all slaves state and read them.
    // ec_readstate();

    states.ethMode = ec_slave[slaveID].state; 

    return true;
}

bool L7NH::updateStatesSDO(void)
{
    states.posActRaw = getPositionActualSDO();
    states.velActRaw = getVelocityActualSDO();
    states.trqActRaw = getTorqueActualSDO();
    states.statusword = getStatuseWordSDO();
    states.ctlmode = getModeOfOperationSDO();

    states.posAct = ((double)states.posActRaw / (double)PulsePerRevolution) * 360.0;
    states.velAct = ((double)states.velActRaw / (double)PulsePerRevolution) * 60.0;
    states.trqAct = (double)states.trqActRaw * 0.1;

    /*
    Servo On Conditions for statusword:
    Bit 0 (Ready to switch on): Should be set (1).
    Bit 1 (Switched on): Should be set (1).
    Bit 2 (Operation enabled): Should be set (1).
    Bit 3 (Fault): Should not be set (0).

    Servo Off Conditions for statusword:
    Bit 0 (Ready to switch on): Could be set (1) but not necessarily.
    Bit 1 (Switched on): Should not be set (0).
    Bit 2 (Operation enabled): Should not be set (0).
    Bit 3 (Fault): Should be set (1).
    */

    if(states.statusword & 0b110)
    {
        states.run = true;
    }
    else
    {
        states.run = false;
    }

    // Refresh states of all slaves state and read them.
    // ec_readstate();

    // states.ethMode = ec_slave[slaveID].state; 

    return true;
}