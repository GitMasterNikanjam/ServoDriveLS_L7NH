#ifndef L7NH_H
#define L7NH_H

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
    
    /// @brief Last error message accured for object.
    std::string errorMessage;
    
    /// @brief Parameters structure. 
    struct ParameterStructure
    {
        /**
         * @brief Speed measurements unit. 
         * 
         * Configure the speed calculation which affects the speed value of the encoder
         * @note
         * Config value Range:
         * 
         * 0: revolutions per Minute (rpm)
         * 
         * 1: deg/sec
         */
        uint8_t SPD_UNIT;

        /**
         * @brief Encoder scaled gain value for scaled position output calcultation.  
         * @note
         * - Scaled_position = GEAR_RATIO * NonScaled_position    
         * 
         * - If GEAR_RATIO be zero value, it means the gear ratio is inactive.
         */
        float GEAR_RATIO;

        /**
         * @brief Ethercat slave id number. 
         * @note 
         * - The default value is -1, it means not assigned any id.  
         * 
         * - The value more than 0 is acceptable. 
         */
        int ETHERCAT_ID;

        /**
         * @brief TXPDO/RXPDO map configuration type.
         *  
         * @note #### Vlaue:1
         * 
         * - RXPDO = {MapValue_ControlWord, MapValue_TargetTorque}
         * 
         * - TXPDO = {MapValue_StatusWord, MapValue_PositionActual, MapValue_VelocityActual, MapValue_OperationModeDisplay, MapValue_DigitalInput}
         * 
         * @note #### Other values  
         * 
         * - Not acceptabled. 
         */
        uint8_t PDOMAP_CONFIG_TYPE;

        /**
         * @brief Direction behavior. 0: CCW, 1:CW     
         * @note - dir: 0 -> With a positive command, the motor rotates counterclockwise. Then, the position feedback value increases.(looking at the shaft)
         * @note - dir: 1 -> With a positive command, the motor rotates clockwise. Then, the position feedbackvalue increases.(looking at the shaft)
         */
        uint8_t ROTATION_DIR;

        /**
         * @brief Rated torque. [N.m]. Its depend on manufacture designing.
         *  */ 
        float TORQUE_RATED;
    }parameters;

    /// @brief Values structure.
    struct ValuesStructure
    {
        int32_t posActStep;                 ///< Raw Actual position. [pulses]
        float posActDeg;                   ///< Actual position. [deg]
        float posActCmdDeg;                ///< posActRaw command from out source. [deg]
        float posActTargetDeg;

        int32_t velActStep;                 ///< Raw Actual  velocity. [pulses/sec]
        float velAct;                      ///< Raw Actual  velocity. [deg/sec]                                  
        float velActCmd;                   ///< velActRaw command from out source. [RPM]
        float velActTarget;

        int16_t trqActStep;                 ///< Raw Actual torque. [0.1% of nominal torque]
        float trqActNm;                    ///< Actual torque. [%]  
        float trqActCmdStep;               ///< trqActRaw command from out source. [%]
        
        uint8_t ethercatState;              ///< Ethercat mode: OPT/PRE_OPT/SAFE_OPT/INIT/ERROR/NONE
        uint8_t controlMode;                ///< Control mode: NO/PP/PV/PT/HM/CSP/CSV/CST
        bool runState;                      ///< 1/0 -> On/Off state of driver.
        bool powerState;
        bool faultState;
        bool warningState;
        bool limitState;
        bool digitalInputs[8];               ///< Statusword register value.
    }value;
    
    /// @brief  Default constructor. Init parameters and values.
    L7NH();

    /**
     * @brief Init object. Check parameters. Set and config parameters.
     * @return true if successed.
     */
    bool init(void);

    /**
     * @brief Check parameters validation.
     * @return true if successed.
     */
    bool checkParameters(void);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get TX/RX PDO configurations:

    /**
     * @brief Use Sync Manager for assign certain RXPDO.
     * @return true if successed.
     * @note - slave must in PRE_OP.
     * @note - Use this function before ethercat configMap().
     * @note - There are 4 value for pdo_rank. 1 to 4. other value return false.
     */
    bool assignRxPDO_rank(int pdo_rank);

    /**
     * @brief Use Sync Manager for assign certain TXPDO.
     * @return true if successed.
     * @note - slave must in PRE_OP.
     * @note - Use this function before ethercat configMap().
     * @note - There are 4 value for pdo_rank. 1 to 4. other value return false.
     * @note - rank 1st index address: 0x1A00
     * @note - rank 2st index address: 0x1A01
     * @note - rank 3st index address: 0x1A02
     * @note - rank 4st index address: 0x1A03
     */
    bool assignTxPDO_rank(int pdo_rank);

    /**
     * @brief Get current index address of RxPDO.
     * @return 0 if not successed.
     */
    uint16_t getRxPDOIndex(void);

    /**
     * @brief Get current index address of TxPDO.
     * @return 0 if not successed.
     */
    uint16_t getTxPDOIndex(void);

    /**
     * @brief Set RxPDO object vector.
     * @param num_enteries number of object that want to set in PDO mapping.
     * @param mapping_entry array of objects for set PDO mapping.
     * @return true if successed.
     * @note - set assignRxPDO_rank(int pdo_rank) before use this function.
     * @note - slave must in PRE_OP.
     * @note - Use this function before ethercat configMap().
     */
    bool setRxPDO(uint8_t num_enteries, uint32_t* mapping_entry);

    /**
     * @brief Set RxPDO object vector.
     * @param num_enteries number of object that want to set in PDO mapping.
     * @param mapping_entry array of objects for set PDO mapping
     * @return true if successed. 
     * @note - set assignTxPDO_rank(int pdo_rank) before use this function.
     * @note - slave must in PRE_OP.
     * @note - Use this function before ethercat configMap().
     */
    bool setTxPDO(uint8_t num_enteries, uint32_t* mapping_entry);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get Driver ID:

    /**
     * @brief Set  Motor ID. it is specific for each model of motor. 
     * @return true if successed.
     * @warning dont set incorrectly. in most cases it is not require set. driver automaticlly identify 
     * and set it after connect encoder cable and reset of power.
     */
    bool setMotorID(uint16_t ID);

    /**
     * @brief Get  Motor ID. it is specific for each model of motor.
     * @return -1 if not successed.
     */
    int16_t getMotorID(void);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Get Driver Node ID:

    /**
     * @brief Get NodeId value. NodeID can set physically on the driver fuselage.
     * @return -1 if not successed.
     */
    int16_t getNodeID(void);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get encoder configuration:

    /**
     * @brief Set Encoder type. it is specific for each encoder of motor. 
     * @return true if successed.
     * @note - type: 0 -> Quadrature (incremental, A lead B)        
     * @note - type: 1 -> Quadrature (incremental, B lead A)
     * @note - type: 2 -> BiSS Serial (single-turn only)
     * @note - type: 3 -> Reserved
     * @note - type: 4 -> BiSS Serial Absolute (multi-turn 16-bit)
     * @note - type: 5 to 6 -> Reserved 
     * @note - type: 7 -> Sinusoidal(1Vpp) 
     * @note - type: 8 -> Analog Hall
     * @note - type: 9 to 10 -> Reserved
     * @note - type: 11 -> Tamagawa Serial (single-turn only)
     * @note - type: 12 -> Tamagawa Serial Absolute (multi-turn 16-bit)
     * @note - type: 13 -> EnDat 2.2 Serial
     * @warning dont set incorrectly. in most cases it is not require set. driver automaticlly identify 
     * and set it after connect encoder cable and reset of power.
     */
    bool setEncoderType(uint16_t type);

    /**
     * @brief Get Encoder type. it is specific for each encoder of motor.
     * @return -1 if not successed.
     * @note - type: 0 -> Quadrature (incremental, A lead B)        
     * @note - type: 1 -> Quadrature (incremental, B lead A)
     * @note - type: 2 -> BiSS Serial (single-turn only)
     * @note - type: 3 -> Reserved
     * @note - type: 4 -> BiSS Serial Absolute (multi-turn 16-bit)
     * @note - type: 5 to 6 -> Reserved 
     * @note - type: 7 -> Sinusoidal(1Vpp) 
     * @note - type: 8 -> Analog Hall
     * @note - type: 9 to 10 -> Reserved
     * @note - type: 11 -> Tamagawa Serial (single-turn only)
     * @note - type: 12 -> Tamagawa Serial Absolute (multi-turn 16-bit)
     * @note - type: 13 -> EnDat 2.2 Serial
     */
    int16_t getEncoderType(void);

    /**
     * @brief Get number of pulses or value for one revolution of motor.
     * Shows the encoder resolution in the unit of pulse (count) based on a multiple of 4.
     * @return 0 if not successed.
     */
    uint32_t getEncoderPulsePerRevolution(void);

    /**
     * @brief Set direction of rotation relative to the command value.
     * @param dir is direction value. 0:CCW, 1:CW
     * @return true if successed.
     * @note - dir: 0 -> With a positive command, the motor rotates counterclockwise. Then, the position feedback value increases.
     * @note - dir: 1 -> With a positive command, the motor rotates clockwise. Then, the position feedbackvalue increases.
     * @note - Change attribute: Servo off
     */
    bool setRotationDirectionSelect(uint16_t dir);

    /**
     * @brief Get direction of rotation relative to the command value.
     * @return - 0:    With a positive command, the motor rotates counterclockwise. Then, the position feedback value increases.   
     * @return - 1:    With a positive command, the motor rotates clockwise. Then, the position feedbackvalue increases.   
     * @return - 2:    if not successed.   
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

    /**
     * @brief Reset software reset of driver by procedure commands.
     * @return true if successed.
     */
    bool softwareReset(void);

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
    bool servoOffSDO(void);

    // Servo Off command.   
    // Driver must be in ethercat operational state.
    bool servoOffPDO(void);

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

    /**
     * @brief This specifies the function to limit the output torque of the drive.
     *  
     * | Setting values  |                                     Description                                      |
     * |:---------------:|--------------------------------------------------------------------------------------|
     * |                 | Limits the torque using positive/negative torque limit value according to the        |
     * |        0        | driving direction; the maximum value is limited by the maximum torque                |
     * |                 | (0x6072).                                                                            |
     * |                 | Forward: 0x60E0, Reverse: 0x60E1                                                     |
     * |-----------------|--------------------------------------------------------------------------------------|
     * |        1        | Limits the torque only by the maximum torque (0x6072) regardless of the              |
     * |                 | driving direction.                                                                   |
     * |-----------------|--------------------------------------------------------------------------------------|
     * |                 | Limits the torque using external positive/negative torque limit value                |
     * |        2        | according to the driving direction.                                                  |
     * |                 | - Forward: 0x2111, Reverse: 0x2112                                                   |
     * |-----------------|--------------------------------------------------------------------------------------|
     * |                 | Limits the torque using internal and external torque limit value according to        |
     * |                 | the driving direction and the torque limit signal.                                   |
     * |        3        | - Forward: 0x60E0(if P_CL signal is not input), 0x2111(if P_CL signal is             |
     * |                 | input)                                                                               |
     * |                 | - Reverse: 0x60E1(if N_CL signal is not input), 0x2112(if N_CL signal is             |
     * |                 | input)                                                                               |
     * |-----------------|--------------------------------------------------------------------------------------|
     * |        4        | Limited by the analog input torque limit.                                            |
     * |                 | - Refer to analog torque limit scale (0x221C) and offset (0x221D)                    |
     * |-----------------|--------------------------------------------------------------------------------------|
     */
    bool setTorqueLimitFunctionSelectSDO(uint16_t value);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get Velocity:

    /**
     * @brief Get Velocity Actual Value in SDO mode. [pulses/s]
     */
    int32_t getVelocityActualSDO(void);

    /**
     * @brief Get Velocity Demand Value in SDO mode. [pulses/s]
     */
    int32_t getVelocityDemandSDO(void);

    /**
     * @brief Get Velocity Demand Value in PDO mode. [pulses/s]
     * @warning  Use it when VelocityDemandValue exist in PDO mapping, otherwise return incorrect value.
     */
    int32_t getVelocityDemandPDO(void);

    /**
     * @brief Get Velocity Actual Value in PDO mode. [pulses/s]
     * @warning Use it when VelocityActualValue exist in PDO mapping, otherwise return incorrect value.
     */
    int32_t getVelocityActualPDO(void);

    /**
     * @brief Set Target Velocity in PDO mode. [pulses/s]
     * @warning Use it when TargetVelocity exist in PDO mapping, otherwise set incorrect value.
     */
    bool setTargetVelocityPDO(int32_t velocity);

    /**
     * @brief Set Target Velocity in SDO mode. [pulses/s]
     * @return true if successed.
     */
    bool setTargetVelocitySDO(int32_t velocity);

    /**
     * @brief Set Max Profile Velocity in SDO mode. [pulses/s]
     * @return true if successed.
     */
    bool setMaxProfileVelocitySDO(uint32_t velocity);

    /**
     * This specifies the speed limit function for torque control.
     * @param state 0 Limited by speed limit value (0x230E). 1 Limited by the maximum motor speed.  
     * @return true if successed.
     */
    bool setSpeedLimitFunctionSelect(bool state);

    /**
     * This specifies the speed limit value for torque control. This setting is applied only when the Speed Limit Function Setting (0x230D) is set to 0. 
     * @param value: Max speed [RPM] at torque control mode.
     * @return true if successed.
     */
    bool setSpeedLimitValueAtTorqueControlMode(uint16_t value);

    /**
     * @brief This represents the current rotation speed of the motor. [rpm]
     */
    int16_t getFeedbackSpeedSDO(void);

    /**
     * @brief This represents the current rotation speed of the motor. [rpm]
     */
    int16_t getFeedbackSpeedPDO(void);

    /**
     * @brief This represents the rated speed of the driving motor. [RPM]
     */
    uint16_t getMotorRatedSpeed(void);
    
    /**
     * @brief Set JogOperationSpeed. 
     * @param value is jog operation speed. [RPM].
     * @return true if successed.
     */
    bool setJogOperationSpeed(int16_t value);

    /**
     * @brief Set SpeedCommandAccelerationTime. 
     * @param value [ms]
     * @return true if successed.
     */
    bool setSpeedCommandAccelerationTime(uint16_t value);

    /**
     * @brief Set SpeedCommandDecelerationTime. 
     * @param value [ms]
     * @return true if successed.
     */
    bool setSpeedCommandDecelerationTime(uint16_t value);

    /**
     * @brief Set JogOperationSpeed. 
     * @param value [ms]
     * @return true if successed.
     */
    bool setSpeedCommandScurveTime(uint16_t value);

    /**
     * @brief Set ServoLockFunction enable/ disable. 
     * @param value 0 means Servo-lock function disabled. 1 means Servo-lock function enabled. 
     * @return true if successed.
     */
    bool setServoLockFunctionSetting(uint16_t value);

    // +++++++++++++++++++++++++++++++++++++++++++++++++
    // Set/Get Acceleration:

    /**
     * @brief Set Profile Acceleration in SDO mode. [pulses/s^2]
     */
    bool setProfileAccelerationSDO(int32_t acc);

    /**
     * @brief Set Profile Deceleration in SDO mode. [pulses/s^2]
     */
    bool setProfileDecelerationSDO(int32 acc);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Get/Set Position:

    /**
     * @brief Set Target Position in SDO mode. [pulses]
     * @return true if successed.
     */
    bool setTargetPositionSDO(int32_t position);

    /**
     * @brief Set Target Position in PDO mode. [pulses]
     * @warning Use it when TargetPosition exist in PDO mapping, otherwise set incorrect value.
     */
    bool setTargetPositionPDO(int32_t position);

    /**
     * @brief This represents the value entered as the command during the position control in SDO mode. [pulses]
     */
    int32_t getPositionDemandInternalSDO(void);

    /**
     * @brief Get Position Actual Internal Value in SDO mode. [pulses]
     */
    int32_t getPositionActualInternalSDO(void);

    /**
     * @brief Get Position Actual Internal Value in PDO mode. [pulses]
     * @warning Use it when PositionActualInternalValue exist in PDO mapping, otherwise return incorrect value.
     */
    int32_t getPositionActualInternalPDO(void);

    /**
     * @brief Get Position Actual Value in SDO mode. [pulses]
     */
    int32_t getPositionActualSDO(void);

    /**
     * @brief Get Position Actual Value in PDO mode. [pulses]
     * @warning Use it when PositionActualValue exist in PDO mapping, otherwise return incorrect value.
     */
    int32_t getPositionActualPDO(void);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Digital input/output:

    /**
     * @brief Set digital input setting for certain channel. 
     * @param inputChannel is channel number of input IO. It can be at range 1 to 8.  
     * @param assignedValue is input assigned function value. It can be at range of 0x00 to 0x0C.
     * @param activeMode is input signal activation mode. 0 value means active high. 1 value means active low.  
     * @return true if successed.
     * @note You can use from The macros **AssignInputValue_** and **InputMode_** for **assignedValue** and **activeMode**   
     * 
     */
    bool setDigitalInput(uint8_t inputChannel, uint8_t assignedValue, uint8_t activeMode);

    /**
     * @brief Read and get digital inputs values for all channels. In SDO mode. 
     * @return 0 for each bit value means input signal is disactive.
     * @return 1 for each bit value means input signal is active.  
     * @note bit 0 is for channel 1, bit 1 is for channel 2, and so on for other channels.
     */
    uint8_t getDigitalInputValueSDO(void);

    /**
     * @brief Read and get digital inputs values for all channels. In PDO mode. 
     * @return 0 for each bit value means input signal is disactive.
     * @return 1 for each bit value means input signal is active.  
     * @note bit 0 is for channel 1, bit 1 is for channel 2, and so on for other channels.
     */
    uint8_t getDigitalInputValuePDO(void);

    /**
     * @brief Read and get digital input assigned value for certain channel.
     * @param inputChannel is channel number of input IO. It can be at range 1 to 8. 
     * @return -1 if not successed to read.
     */
    int8_t getDigitalInputAssignedValue(uint8_t inputChannel);

    /**
     * @brief Read and get digital input active mode for certain channel.
     * @param inputChannel is channel number of input IO. It can be at range 1 to 8. 
     * @return 0 is active high mode.
     * @return 1 is active low mode.
     * @return -1 if not successed to read.
     */
    int8_t getDigitalInputActiveMode(uint8_t inputChannel);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Procedure Command Code & Procedure Command Argument:

    /**
     * @brief Set ProcedureCommandCode.
     * @param value is procedure command code value.  
     * @note You can use **ProcedureCommandCode_** macros for value parameter.
     * @return true if successed.
     */
    bool setProcedureCommandCode(uint16_t value);

    /**
     * @brief Set ProcedureCommandArgument. 
     * @param value is procedure command argument.
     * @return true if successed.
     */
    bool setProcedureCommandArgument(uint16_t value);

    /**
     * Do procedure ManualJOG_ServoOn. 
     * @return true if successed.
     */
    bool ManualJOG_ServoOn(void);

    /**
     * Do procedure ManualJOG_ServoOff. 
     * @return true if successed.
     */
    bool ManualJOG_ServoOff(void);

    /**
     * Do procedure ManualJOG_Positive. 
     * @return true if successed.
     */
    bool ManualJOG_Positive(void);

    /**
     * Do procedure ManualJOG_Negative. 
     * @return true if successed.
     */
    bool ManualJOG_Negative(void);

    /**
     * Do procedure ManualJOG_Stop. 
     * @return true if successed.
     */
    bool ManualJOG_Stop(void);

    // ++++++++++++++++++++++++++++++++++++++++++++++++
    // Auto update driver states:

    /**
     * @brief Update driver values in PDO mode.
     */
    bool updateValuesPDO(void);

    /**
     * @brief Update driver values in SDO mode.
     */
    bool updateValuesSDO(void);

private:

    // speed conversion gain for convert step unit to user unit.
    float _velConStep2Uu;

    uint32_t _PulsePerRevolution;

    /// Access the process data inputs.
    uint8 *inputs;

    /// Access the process data outputs.
    uint8 *outputs;

    uint8_t RxPDO_rank;     ///< rank range: 1, 2, 3, 4
    uint8_t TxPDO_rank;     ///< rank range: 1, 2, 3, 4

    // Offset value for RX mapping
    uint8_t RxMapOffset_ControlWord;
    uint8_t RxMapOffset_TargetPosition;
    uint8_t RxMapOffset_TargetVelocity;
    uint8_t RxMapOffset_TargetTorque;
    uint8_t RxMapOffset_DigitalOutput_PhysicalOutputs;
    uint8_t RxMapOffset_ModesOfOperation;

    /**
     * @brief _RxMapFlag indexes
     * @note Array cells:
     * @note - 0: ControlWord
     * @note - 1: TargetPosition
     * @note - 2: TargetVelocity
     * @note - 3: TargetTorque
     * @note - 4: DigitalOutput_PhysicalOutputs
     * @note - 5: ModesOfOperation
     */
    uint8_t _RxMapFlag[6];

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

    /**
     * @brief _TxMapFlag indexes
     * @note Array cells:
     * @note - 0: StatusWord
     * @note - 1: PositionActualInternal
     * @note - 2: PositionActual
     * @note - 3: VelocityActual
     * @note - 4: TorqueActual
     * @note - 5: PositionDemandInternal
     * @note - 6: PositionDemand
     * @note - 7: VelocityDemand
     * @note - 8: FeedbackSpeed
     * @note - 9: TorqueDemand
     * @note - 10: DigitalInput
     * @note - 11: OperationModeDisplay
     */
    uint8_t _TxMapFlag[12];
};

#endif