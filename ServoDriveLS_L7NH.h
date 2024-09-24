#ifndef _L7NH_H
#define _L7NH_H

// Header Includes:
#include <iostream>                 // standard I/O operations
#include <chrono>                   // For time managements
#include <thread>                   // For thread programming
#include "ethercat.h"               // SOEM EtherCAT functionality 
#include "ServoDriveLS_L7NH_objDict.h"           // Object dictionary for L7NH drivers

// ####################################################

class L7NH
{
public:
    
    std::string errorMessage;
    
    struct MOTOR_STATE
    {
        int32_t posActRaw;                      // Raw Actual position. [pulses]
        int32_t velActRaw;                      // Raw Actual  velocity. [pulses/sec]
        int16_t trqActRaw;                      // Raw Actual torque. [0.1% of nominal torque]
        double posActRaw_cmd;                  // posActRaw command from out source. [deg]
        double velActRaw_cmd;                  // velActRaw command from out source. [RPM]
        double trqActRaw_cmd;                  // trqActRaw command from out source. [%]
        double posAct;                          // Actual position. [deg]
        double velAct;                          // Actual  velocity. [RPM]
        double trqAct;                          // Actual torque. [%]
        uint8_t ethMode = EC_STATE_NONE;        // Ethercat mode: OPT/PRE_OPT/SAFE_OPT/INIT
        uint8_t ctlmode = OPERATION_MODE_NO;    // Control mode: NO/PP/PV/PT/HM/CSP/CSV/CST
        uint8_t run = 0;                        // 1/0 -> On/Off state of driver.
        uint16_t statusword;                    // Statusword register value.
    }states;

    uint32_t PulsePerRevolution;
    
    void statesClear(void);
    
    /**
     * Set slave num/ID in ethercat slaves detected.
     * @return true if successed.  
     * @return flase if there is a problem for ethercat slave connection.
     *  */ 
    bool setSlaveID(int ID_num);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get TX/RX PDO configurations:

    // Use Sync Manager for assign certain RXPDO
    // Note: slave must in PRE_OP. 
    // Use this function before ethercat configMap().
    // There are 4 value for pdo_rank. 1 to 4. other value return false
    bool assignRxPDO_rank(int pdo_rank);

    // Use Sync Manager for assign certain TXPDO 
    // Note: slave must in PRE_OP.
    // Use this function before ethercat configMap().
    // There are 4 value for pdo_rank. 1 to 4. other value return false.
    bool assignTxPDO_rank(int pdo_rank);

    // Return current rank of RxPDO
    // Rank is the which PDO mapping selected. Range: 1, 2, 3, 4
    // If not success to read return 0.
    uint16_t getRxPDO_rank(void);

    // Return current rank of TxPDO
    // Rank is the which PDO mapping selected. Range: 1, 2, 3, 4
    // If not success to read return 0.
    uint16_t getTxPDO_rank(void);

    /* 
    Set RxPDO object vector.
    num_enteries: number of object that want to set in PDO mapping.
    mapping_entry: array of objects for set PDO mapping
    Note: set assignRxPDO_rank(int pdo_rank) before use this function.
    Note: slave must in PRE_OP.
    Use this function before ethercat configMap().
    return: true if successed. 
    */
    bool setRxPDO(uint8_t num_enteries, uint32_t* mapping_entry);

    /* 
    Set RxPDO object vector.
    num_enteries: number of object that want to set in PDO mapping.
    mapping_entry: array of objects for set PDO mapping
    Note: set assignTxPDO_rank(int pdo_rank) before use this function.
    Note: slave must in PRE_OP.
    Use this function before ethercat configMap().
    return: true if successed. 
    */
    bool setTxPDO(uint8_t num_enteries, uint32_t* mapping_entry);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get Driver ID:

    /*
    Set  Motor ID. it is specific for each model of motor. 
    Hint: dont set incorrectly. in most cases it is not require set. driver automaticlly identify 
    and set it after connect encoder cable and reset of power.
    return: true if successed.
    */
    bool setMotorID(uint16_t ID);

    // Get  Motor ID. it is specific for each model of motor. 
    // return: -1 if not successed.
    int16_t getMotorID(void);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Get Driver Node ID:

    // Return NodeId value. NodeID can set physically on the driver fuselage.
    // return: -1 if not successed.
    int16_t getNodeID(void);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get encoder configuration:

    /*
    Set Encoder type. it is specific for each encoder of motor. 
    Hint: dont set incorrectly. in most cases it is not require set. driver automaticlly identify 
    and set it after connect encoder cable and reset of power.
    Return values           Encoder Type
    0                       Quadrature (incremental, A lead B)
    1                       Quadrature (incremental, B lead A)
    2                       BiSS Serial (single-turn only)
    3                       Reserved
    4                       BiSS Serial Absolute (multi-turn 16-bit)
    5 to 6                  Reserved
    7                       Sinusoidal(1Vpp)
    8                       Analog Hall
    9 to 10                 Reserved
    11                      Tamagawa Serial (single-turn only)
    12                      Tamagawa Serial Absolute (multi-turn 16-bit)
    13                      EnDat 2.2 Serial

    return: true if successed.
    */
    bool setEncoderType(uint16_t type);

    /* 
    Get Encoder type. it is specific for each encoder of motor. 
    Return values           Encoder Type
    0                       Quadrature (incremental, A lead B)
    1                       Quadrature (incremental, B lead A)
    2                       BiSS Serial (single-turn only)
    3                       Reserved
    4                       BiSS Serial Absolute (multi-turn 16-bit)
    5 to 6                  Reserved
    7                       Sinusoidal(1Vpp)
    8                       Analog Hall
    9 to 10                 Reserved
    11                      Tamagawa Serial (single-turn only)
    12                      Tamagawa Serial Absolute (multi-turn 16-bit)
    13                      EnDat 2.2 Serial

    return: -1 if not successed.
    */
    int16_t getEncoderType(void);

    // Return number of pulses or value for one revolution of motor.
    // return: 0 if not successed.
    uint32_t getEncoderPulsePerRevolution(void);

    /* Set direction of rotation relative to the command value. 
    dir value:
    0    With a positive command, the motor rotates counterclockwise. Then, the position feedback value increases.
    1    With a positive command, the motor rotates clockwise. Then, the position feedbackvalue increases.

    return: true if successed.
    */
    bool setRotationDirectionSelect(uint8_t dir);

    /* Get direction of rotation relative to the command value. 
    return value:
    0    With a positive command, the motor rotates counterclockwise. Then, the position feedback value increases.
    1    With a positive command, the motor rotates clockwise. Then, the position feedbackvalue increases.
    2    if not successed.
    */
    uint8_t getRotationDirectionSelect(void);
    
    /* 
    Get the usage of the absolute encoder.
    Setting values          Description
    0                       Uses the absolute encoder as the absolute encoder. Uses the multi-turn data.
    1                       Uses the absolute encoder as the incremental encoder. Does not use the multi-turn
                            data. Does not display any battery-related alarm/warning.
    2                       if not succsessed.
    */
    uint16_t getEncoderConfiguration(void);

    /* 
    Set the usage of the absolute encoder.
    Setting values          Description
    0                       Uses the absolute encoder as the absolute encoder. Uses the multi-turn data.
    1                       Uses the absolute encoder as the incremental encoder. Does not use the multi-turn
                        data. Does not display any battery-related alarm/warning.
    return: true if successed.
    */
    bool setEncoderConfiguration(uint16_t config);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Save/Restore:

    // Save all parameters in EEPROM memory.
    // retutn: true if successed.
    bool saveParamsAll(void);

    // Save Communication parameters in EEPROM memory.
    // retutn: true if successed.
    bool saveParamsCommunication(void);

    // Save CiA402 parameters in EEPROM memory.
    // retutn: true if successed.
    bool saveParamsCiA402(void);

    // Save Specific parameters in EEPROM memory.
    // retutn: true if successed.
    bool saveParamsSpecific(void);

    // Restore and load all default parameters.
    // retutn: true if successed.
    bool loadParamsAll(void);

    // Restore and load Communication default parameters.
    // retutn: true if successed.
    bool loadParamsCommunication(void);

    // Restore and load CiA402 default parameters.
    // retutn: true if successed.
    bool loadParamsCiA402(void);

    // Restore and load Specific default parameters.
    // retutn: true if successed.
    bool loadParamsSpecific(void);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Servo ON/OFF:

    // servo ON command.
    // Driver must be in ethercat operational state.
    // After servo On, the proccess data must send cyclic frequently, otherwise driver automaticly set servo Off.
    void servoOnSDO(void);

    // servo ON command.
    // Driver must be in ethercat operational state.
    // After servo On, the proccess data must send cyclic frequently, otherwise driver automaticly set servo Off.
    // Note: just use it if controlword exist in RxPDO mapping.
    void servoOnPDO(void);

    // Servo Off command.
    // Driver must be in ethercat operational state.
    void servoOffSDO(void);

    // Servo Off command.   
    // Driver must be in ethercat operational state.
    void servoOffPDO(void);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Controlword, statusword, Operation mode and machin state
    
    // Get statusword and update sate machine, then Print stateMachine string mode.  
    void printStateMachine(void);

    // Set operational mode. eg: PP:1, PV:3, PT:4, CSP:8, CSV:9, CST:10, HOME:6
    // return: true if successed.
    bool setModesOfOperationSDO(int8_t mode);

    // Set operational mode. eg: PP:1, PV:3, PT:4, CSP:8, CSV:9, CST:10, HOME:6
    // Hint: Use it when ModesOfOperation exist in PDO mapping.
    bool setModesOfOperationPDO(int8_t mode);

    // Get Operation Mode Display.
    // Hint: Use it when OperationModeDisplay exist in PDO mapping.
    int8_t getOperationModeDisplayPDO(void);

    // Get current operational mode. eg: PP:1, PV:3, PT:4, CSP:8, CSV:9, CST:10, HOME:6
    // return operation mode code number.
    int8_t getModeOfOperationSDO(void);

    // Set the control word in the PDO mode.
    // Hint: Use it when ControlWord exist in PDO mapping, otherwise set incorrect value.
    bool setControlWordPDO(uint16 control_word);

    // Set the control word in the SDO mode.
    bool setControlWordSDO(uint16 control_word);

    // Get the status word in PDO mode.
    // Hint: Use it when StatuseWord exist in PDO mapping, otherwise get incorrect value.
    uint16 getStatuseWordPDO(void);

    // Get the status word in SDO mode.
    uint16 getStatuseWordSDO(void);

    // Update state machine of motor driver.
    void stateUpdate(uint16_t statusWord);

    // Get and displays the mode(s) supported by the drive.
    // if show_op be TRUE, print and display modes.
    uint32_t getSupportedDriveModes(bool show_op);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Homing:

    int startHoming(void);

    bool setHomeOffset(int32_t offset);

    int32_t getHomeoffset(void);

    bool setHomingMethod(int8_t method);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get Torque:

    // Set the design rate torque value [N.m] for Servo motor. Its depend on manufacture designing.
    void setRatedTorque(double value);

    // Set Target Torque in SDO mode. [0.1%]
    // return: true if successed.
    bool setTargetTorqueSDO(int16_t torque);

    // Set Target Torque in PDO mode. [0.1%]
    // Hint: Use it when TargetTorque exist in PDO mapping.
    bool setTargetTorquePDO(int16_t torque);

    // Get Target Torque in SDO mode. [0.1%]
    int16_t getTargetTorqueSDO(void);

    // Get Torque Actual in SDO mode.[0.1%]
    int16_t getTorqueActualSDO(void);

    // Get Torque Actual in PDO mode.[0.1%]
    // Hint: Use it when TorqueActual exist in PDO mapping.
    int16_t getTorqueActualPDO(void);

    // Get Torque Demand in SDO mode.[0.1%]
    int16_t getTorqueDemandSDO(void);

    // Get Torque Demand in PDO mode.[0.1%]
    // Hint: Use it when TorqueDemand exist in PDO mapping.
    int16_t getTorqueDemandPDO(void);

    // Set Maximum Torque in SDO mode.[0.1%]
    bool setMaximumTorqueSDO(uint16_t torque);

    // Set Torque Slope in SDO mode.[0.1%/s].
    // This specifies the torque slope for the PT mode operation.
    bool setTorqueSlopeSDO(uint32_t slope);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get Velocity:

    // Get Velocity Actual Value in SDO mode. [pulses/s]
    int32_t getVelocityActualSDO(void);

    // Get Velocity Demand Value in SDO mode. [pulses/s]
    int32_t getVelocityDemandSDO(void);

    // Get Velocity Demand Value in PDO mode. [pulses/s]
    // Hint: Use it when VelocityDemandValue exist in PDO mapping, otherwise return incorrect value.
    int32_t getVelocityDemandPDO(void);

    // Get Velocity Actual Value in PDO mode. [pulses/s]
    // Hint: Use it when VelocityActualValue exist in PDO mapping, otherwise return incorrect value.
    int32_t getVelocityActualPDO(void);

    // Set Target Velocity in PDO mode. [pulses/s]
    // Hint: Use it when TargetVelocity exist in PDO mapping, otherwise set incorrect value.
    bool setTargetVelocityPDO(int32_t velocity);

    // Set Target Velocity in SDO mode. [pulses/s]
    // return: true if successed.
    bool setTargetVelocitySDO(int32_t velocity);

    // Set Max Profile Velocity in SDO mode. [pulses/s]
    // return: true if successed.
    bool setMaxProfileVelocitySDO(uint32_t velocity);

    /**
     * This specifies the speed limit function for torque control.
     * @return true if successed.
     */
    bool setSpeedLimitFunctionSelect(bool state);

    /**
     * This specifies the speed limit value for torque control. This setting is applied only when the Speed Limit Function Setting (0x230D) is set to 0. 
     * @return true if successed.
     */
    bool setSpeedLimitValueAtTorqueControlMode(uint16_t value);

    /**
     * This represents the current rotation speed of the motor. [rpm]
     */
    int16_t getFeedbackSpeedSDO(void);

    /**
     * This represents the current rotation speed of the motor. [rpm]
     */
    int16_t getFeedbackSpeedPDO(void);

    /**
     * This represents the rated speed of the driving motor. [RPM]
     */
    uint16_t getMotorRatedSpeed(void);
    
    // +++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get Acceleration:

    // Set Profile Acceleration in SDO mode. [pulses/s^2]
    bool setProfileAccelerationSDO(int32_t acc);

    // Set Profile Deceleration in SDO mode. [pulses/s^2]
    bool setProfileDecelerationSDO(int32 acc);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Get/Set Position:

    // Set Target Position in SDO mode. [pulses]
    // return: true if successed.
    bool setTargetPositionSDO(int32_t position);

    // Set Target Position in PDO mode. [pulses]
    // Hint: Use it when TargetPosition exist in PDO mapping, otherwise set incorrect value.
    bool setTargetPositionPDO(int32_t position);

    // This represents the value entered as the command during the position control in SDO mode. [pulses]
    int32_t getPositionDemandInternalSDO(void);

    // Get Position Actual Internal Value in SDO mode. [pulses]
    int32_t getPositionActualInternalSDO(void);

    // Get Position Actual Internal Value in PDO mode. [pulses]
    // Hint: Use it when PositionActualInternalValue exist in PDO mapping, otherwise return incorrect value.
    int32_t getPositionActualInternalPDO(void);

    // Get Position Actual Value in SDO mode. [pulses]
    int32_t getPositionActualSDO(void);

    // Get Position Actual Value in PDO mode. [pulses]
    // Hint: Use it when PositionActualValue exist in PDO mapping, otherwise return incorrect value.
    int32_t getPositionActualPDO(void);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Digital input/output:

    /**
     * Get DigitalInputs port value.
     */
    uint32_t getDigitalInputs(void);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Auto configuration options:

    // Automatic init and setup of driver. 
    // setup_num=1 : assigne 1st rank. set CST mode. RXPDO: {ControlWord, TargetTorque}. TXPDO: {StatusWord, PositionActual, VelocityActual, OperationModeDisplay, DigitalInput}
    bool autoSetup(int ID, int setup_num);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Auto update driver states:

    bool updateStatesPDO(void);

    bool updateStatesSDO(void);

private:

    // ID for salve number that used in simple_ethercat library.
    int slaveID;

    // Access the process data inputs.
    uint8 *inputs;

    // Access the process data outputs.
    uint8 *outputs;

    uint8_t RxPDO_rank;     // rank range: 1, 2, 3, 4
    uint8_t TxPDO_rank;     // rank range: 1, 2, 3, 4

    // Offset value for RX mapping
    uint8_t RxMapOffset_ControlWord;
    uint8_t RxMapOffset_TargetPosition;
    uint8_t RxMapOffset_TargetVelocity;
    uint8_t RxMapOffset_TargetTorque;
    uint8_t RxMapOffset_DigitalOutput_PhysicalOutputs;
    uint8_t RxMapOffset_ModesOfOperation;

    /*
    _RxMapFlag indexes:
    0: ControlWord
    1: TargetPosition
    2: TargetVelocity
    3: TargetTorque
    4: DigitalOutput_PhysicalOutputs
    5: ModesOfOperation
    */
    uint8_t _RxMapFlag[6] = {0, 0, 0, 0, 0, 0};

    // Offset value for TX mapping
    uint8_t TxMapOffset_StatusWord;
    uint8_t TxMapOffset_PositionActualInternal;
    uint8_t TxMapOffset_PositionActual;
    uint8_t TxMapOffset_VelocityActual;
    uint8_t TxMapOffset_TorqueActual;
    uint8_t TxMapOffset_PositionDemandInternal;
    uint8_t TxMapOffset_PositionDemand;
    uint8_t TxMapOffset_VelocityDemand;
    uint8_t TxMapOffset_FeedbackSpeed;
    uint8_t TxMapOffset_TorqueDemand;
    uint8_t TxMapOffset_DigitalInput;
    uint8_t TxMapOffset_OperationModeDisplay;

    /*
    _TxMapFlag indexes :
     0: StatusWord
     1: PositionActualInternal
     2: PositionActual
     3: VelocityActual
     4: TorqueActual
     5: PositionDemandInternal
     6: PositionDemand
     7: VelocityDemand
     8: TorqueDemand
     9: DigitalInput
     10: OperationModeDisplay
    */
    uint8_t _TxMapFlag[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // State machine of motor driver
    enum stateMachine_enum
    {
        None,
        NotReadyToSwitchOn,          
        SwitchOnDisabled,             
        ReadyToSwitchOn,          
        SwitchedOn,                
        OperationEnabled,  
        QuickStopActive,        
        FaultReactionActive,     
        Fault                    
    }stateMachine = None;

    // State of main power off/on. On: TRUE, Off: FALSE.
    bool state_MainPowerOn;      

    // state of warning is accurred. Warrning: TRUE, No warrning: FALSE.              
    bool state_WarningIsOccurred;  

    // Rated torque. [N.m]
    double ratedTorque;

};

#endif