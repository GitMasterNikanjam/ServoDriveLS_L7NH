// L7NH Driver Object Dictionary Header File:

#ifndef _L7NH_OBJDICT_H
#define _L7NH_OBJDICT_H

// Usefull  declarations:

namespace _L7NH
{
    // Basic commands order
    #define SAVE        0x65766173  // 0:'S', 1:'A', 2:'V', 3:'E'
    #define LOAD        0x64616f6c  // 0:'L', 1:'O', 2:'A', 3:'D'
    #define RSET        0x74657372  // 0:'R', 1:'S', 2:'E', 3:'T'
    #define READ        0x64616572  // 0:'R', 1:'E', 2:'A', 3:'D'  

    // DataType:
    #define SINT        1       // Signed 8bit -128 ~127                 
    #define USINT       2       // Unsigned 8bit 0 ~ 255
    #define INT         3       // Signed 16bit -32768 ~ 32767
    #define UINT        4       // Unsigned 16bit 0 ~ 65535
    #define DINT        5       // Signed 32bit -2147483648 ~ 2147483647
    #define UDINT       6       // Unsigned 32bit 0 ~ 4294967295
    #define FP32        7       // Float 32bit Single precision floating point
    #define STRING      8       // String Value

    // StatusWord value for States machine
    #define StatusWord_NotReadyToSwitchOn               0x0000
    #define StatusWord_SwitchOnDisabled                 0x0040
    #define StatusWord_ReadyToSwitchOn                  0x0021
    #define StatusWord_SwitchedOn                       0x0023
    #define StatusWord_OperationEnabled                 0x0027
    #define StatusWord_QuickStopActive                  0x0007
    #define StatusWord_FaultReactionActive              0x000F
    #define StatusWord_Fault                            0x0008
    #define StatusWord_MainPowerIsOn                    0x0010
    #define StatusWord_WarningIsOccurred                0x0080

    // ControlWord value for command states machine
    #define Controlword_Shutdown                        0b0110  
    #define Controlword_SwitchOn                        0b0111
    #define Controlword_SwitchOnEnableOperation         0b1111  
    #define Controlword_DisableVoltage                  0b0000
    #define Controlword_QuickStop                       0b0010
    #define Controlword_DisableOperation                0b0111
    #define Controlword_EnableOperation                 0b1111

    // Operation modes enum
    #define OPERATION_MODE_NO                           0
    #define OPERATION_MODE_PP                           1
    #define OPERATION_MODE_PV                           3
    #define OPERATION_MODE_PT                           4
    #define OPERATION_MODE_HM                           6
    #define OPERATION_MODE_CSP                          8
    #define OPERATION_MODE_CSV                          9
    #define OPERATION_MODE_CST                          10

    // MapValue for RX PDO
    #define MapValue_ControlWord                        0x60400010
    #define MapValue_TargetTorque                       0x60710010
    #define MapValue_TargetPosition                     0x607A0020  
    #define MapValue_TargetVelocity                     0x60FF0020   
    #define MapValue_DigitalOutput_PhysicalOutputs      0x60FE0120
    #define MapValue_ModesOfOperation                   0x60600008

    //MapValue for TX PDO
    #define MapValue_StatusWord                         0x60410010
    #define MapValue_TorqueActual                       0x60770010
    #define MapValue_TorqueDemand                       0x60740010
    #define MapValue_PositionActual                     0x60640020
    #define MapValue_PositionActualInternal             0x60630020  
    #define MapValue_PositionDemand                     0x60620020
    #define MapValue_PositionDemandInternal             0x60FC0020
    #define MapValue_VelocityActual                     0x606C0020  
    #define MapValue_VelocityDemand                     0x606B0020
    #define MapValue_DigitalInput                       0x60FD0020 
    #define MapValue_OperationModeDisplay               0x60610008

}

// ################################################################
// General Objects (from 0x1000)

/* Error Register
The following table shows the error register values for each device. This value is stored in the
emergency message.
*/
#define Index_ErrorRegister                             0x1001

// Manufacturer Device Name
#define Index_ManufacturerDeviceName                    0x1008    

// Store Parameters
// Note: For save all parameters, write 0x65766173("save").
#define Index_StoreParameters                           0x1010
#define SubIndex_StoreParametersAll                     1
#define SubIndex_StoreParametersCommunication           2
#define SubIndex_StoreParametersCiA402                  3
#define SubIndex_StoreParametersSpecific                4


// Restore Default Parameters
// Note: For load default parameters, write 0x64616f6c("load")
// Turn the power off and then back on to restore the default values.
#define Index_RestoreDefaultParameters                      0x1011    
#define SubIndex_RestoreDefaultParametersAll                1
#define SubIndex_RestoreDefaultParametersCommunication      2
#define SubIndex_RestoreDefaultParametersCiA402             3
#define SubIndex_RestoreDefaultParametersSpecific           4

// ReceivePDOMapping and TransmitPDOMapping
#define Index_ReceivePDOMapping_1st                     0x1600
#define Index_ReceivePDOMapping_2st                     0x1601
#define Index_ReceivePDOMapping_3st                     0x1602
#define Index_ReceivePDOMapping_4st                     0x1603

#define Index_TransmitPDOMapping_1st                    0x1A00
#define Index_TransmitPDOMapping_2st                    0x1A01
#define Index_TransmitPDOMapping_3st                    0x1A02
#define Index_TransmitPDOMapping_4st                    0x1A03

// Sync Manager 2 PDO Assignment -> assigned RxPDO
/* Only change in Pre-Operational state and before IOmap configuration*/
#define Index_syncManagerAssignedRxPDO                  0x1C12

// Sync Manager 3 PDO Assignment -> assigned TxPDO
/* Only change in Pre-Operational state and before IOmap configuration*/
#define Index_syncManagerAssignedTxPDO                  0x1C13

// ##############################################################
// Manufacturer Specific Objects (from 0x2000)

// Motor ID
/* This specifies the motor ID.*/
// Range: 0 to 999
#define Index_MotorID                               0x2000

// Encoder Type
/*
Set the encoder type. You have to set it correctly by referencing the table below. However, the serial
encoder provided by LSIS (4 in the table below) is automatically recognized and configured regardless
of these settings. Then, you can check the type of the encoder automatically recognized.
*/
/*
Setting values          Encoder Type
0                       Quadrature (incremental, A lead B)
1                       Quadrature (incremental, B lead A)
2                       BiSS Serial (single-turn only)
3                       Reserved
4                       BiSS Serial Absolute (multi-turn 16-bit)
5~6                     Reserved
7                       Sinusoidal(1Vpp)
8                       Analog Hall
9~10                    Reserved
11                      Tamagawa Serial (single-turn only)
12                      Tamagawa Serial Absolute (multi-turn 16-bit)
13                      EnDat 2.2 Serial
*/
#define Index_EncoderType                               0x2001

// Encoder Pulse per Revolution
/*
This specifies the encoder bits read for automatic display
Setting range: 0 to 21
*/
#define Index_EncoderPulsePerRevolution                 0x2002

// Node ID
/*
Display the node ID configured for the node setting switch of the drive. The value of the node setting
switch is read just once when the power is turned on. Any set value modified subsequently will be in
effect only when the power is turned on again.
*/
#define Index_NodeID                                    0x2003

// Rotation Direction Select
/*
Set the rotation direction of the motor. You can change the rotation direction with this setting when the
direction is changed between positive and negative relative to the user at the final apparatus section.
*/
/*
Setting values          Description
0                       With a positive command, the motor rotates counterclockwise. Then, the position feedback value increases.
1                       With a positive command, the motor rotates clockwise. Then, the position feedbackvalue increases.
*/
#define Index_RotationDirectionSelect                   0x2004

/* Absolute Encoder Configuration
Setting values          Description
0                       Uses the absolute encoder as the absolute encoder. Uses the multi-turn data.
1                       Uses the absolute encoder as the incremental encoder. Does not use the multi-turn
                        data. Does not display any battery-related alarm/warning.
*/
#define Index_EncoderConfiguration                      0x2005

/* Phase Current Offset
Manually set the phase current offset. The configured offset value is subtracted from the measured
current value, and then applied as an actual current value. Do not manually set the offset if you do not
know the exact setting value. You can check the automatically-tuned value if you tune the current offset
with the procedure function (refer to the description of 0x2700). 
*/
#define Index_UPhaseCurrentOffset                       0x2015
#define Index_VPhaseCurrentOffset                       0x2016
#define Index_WPhaseCurrentOffset                       0x2017

// Inertia Ratio Setting
/*
This sets the inertia ratio by calculating the load inertia from the machine system and rotor inertia 
listed on the motor specification table.
The inertia/load ratio is an important control parameter for the operation of the servo. 
It is crucial to set the correct inertia ratio for optimal servo operation.
*/
#define Index_InertiaRatio                              0x2100

/*
Torque Limit Function Select
This specifies the function to limit the output torque of the drive.
Setting values          Description
-------------------------------------------------------------------------------
                        Limits the torque using positive/negative torque limit value according to the
0                       driving direction; the maximum value is limited by the maximum torque
                        (0x6072).
                        - Forward: 0x60E0, Reverse: 0x60E1
--------------------------------------------------------------
1                       Limits the torque only by the maximum torque (0x6072) regardless of the
                        driving direction.
--------------------------------------------------------------
                        Limits the torque using external positive/negative torque limit value
2                       according to the driving direction.
                        - Forward: 0x2111, Reverse: 0x2112
--------------------------------------------------------------
                        Limits the torque using internal and external torque limit value according to
3                       the driving direction and the torque limit signal.
                        - Forward: 0x60E0(if P_CL signal is not input), 0x2111(if P_CL signal is
                        input)
                        - Reverse: 0x60E1(if N_CL signal is not input), 0x2112(if N_CL signal is
                        input)
--------------------------------------------------------------
4                       Limited by the analog input torque limit.
                        - Refer to analog torque limit scale (0x221C) and offset (0x221D)
*/
#define Index_TorqueLimitFunctionSelect                 0x2110

/* 
External Positive Torque Limit Value
This specifies the external positive torque limit value according to the torque limit function setting
(0x2110).
*/
#define Index_ExternalPositiveTorqueLimitValue          0x2111

/*
External Negative Torque Limit Value
This specifies the external negative torque limit value according to the torque limit function setting
(0x2110).
*/
#define Index_ExternalNegativeTorqueLimitValue          0x2112

/*
Digital Input Signal Selection
This specifies the functions of digital input signal 1 of the I/O and the input signal level.
Setting values          Assigned signal
0x00                    Not assigned
0x01                    POT
0x02                    NOT
0x03                    HOME
0x04                    STOP
0x05                    PCON
0x06                    GAIN2
0x07                    P_CL
0x08                    N_CL
0x09                    PROBE1
0x0A                    PROBE2
0x0B                    EMG
0x0C                    A_RST
*/
#define Index_InputSignalSelection_1                    0x2200
#define Index_InputSignalSelection_2                    0x2201
#define Index_InputSignalSelection_3                    0x2202
#define Index_InputSignalSelection_4                    0x2203
#define Index_InputSignalSelection_5                    0x2204
#define Index_InputSignalSelection_6                    0x2205
#define Index_InputSignalSelection_7                    0x2206
#define Index_InputSignalSelection_8                    0x2207

/*
Digital Output Signal Selection
Assign the functions of digital output signal 1 of I/O and set the output signal level.
Setting values          Assigned signal
0x8000                  Not assigned
0x8001                  BRAKE
0x8002                  ALARM
0x8003                  READY
0x8004                  ZSPD
0x8005                  INPOS1
0x8006                  TLMT
0x8007                  VLMT
0x8008                  INSPD
0x8009                  WARN
0x800A                  TGON
0x800B                  INPOS2
*/
#define Index_DigitalOutputSignalSelection_1                0x2210
#define Index_DigitalOutputSignalSelection_2                0x2211
#define Index_DigitalOutputSignalSelection_3                0x2212
#define Index_DigitalOutputSignalSelection_4                0x2213

/*
Speed Limit Function Select
This specifies the speed limit function for torque control.
Setting values      Setting details
0                   Limited by speed limit value (0x230E)
1                   Limited by the maximum motor speed
*/
#define Index_SpeedLimitFunctionSelect                      0x230D

/*
Speed Limit Value at Torque Control Mode
This specifies the speed limit value for torque control. This setting is applied only when the Speed Limit
Function Setting (0x230D) is set to 0.
*/
#define Index_SpeedLimitValueAtTorqueControlMode            0x230E

/*
User Drive Name
The user can customize the drive name. Up to 16 characters can be used to define the name.
*/
#define Index_UserDriveName                                 0x240D

/*
Individual Parameter Storage
This specifies whether to save parameters individually. This parameter is not saved and initialized to 0
during power ON.
Setting values      Setting details
0:                  Parameters are not saved individually. For details on storing a
                    parameter, refer to Storing Parameters (0x1010).
1:                  Save the parameters individually. When a parameter is written, it is
                    immediately stored in the memory.
*/
#define Index_IndividualParameterStorage                    0x240E

/*
Feedback Speed
This represents the current rotation speed of the motor.
*/
#define Index_FeedbackSpeed                                 0x2600

/*
Command Speed
This represents the speed command input to the speed control loop of the drive.
*/
#define Index_CommandSpeed                                  0x2601

/*
Mechanical Angle
This represents the single-turn data of the motor, ranging from 0.0 to 359.9.
*/
#define Index_MechanicalAngle                               0x2608

/*
Electrical Angle
This represents the electrical angle of the motor, ranging from -180.0 to 180.0.
*/
#define Index_ElectricalAngle                               0x2609

/*
Drive Temperature 1
It is the temperature measured by the temperature sensor integrated onto the drive power board. If the
measurement is higher than 95􀭅, the drive overheat alarm 1 (AL-22) will be generated.
*/
#define Index_DriveTemperature1                             0x260B

/*
Drive Temperature 2
This represents the temperature measured by the temperature sensor integrated onto the drive control
board. If the measured temperature is higher than 90􀭅, the drive overheat alarm 2 (AL-25) will be
generated.
*/
#define Index_DriveTemperature2                             0x260C

/*
Encoder Temperature
This represents the temperature measured by the temperature sensor integrated into serial encoder
provided by LSIS (if the setting values of the encoder type (0x2001) are 3, 4, 5, and 6). If the measured
temperature is higher than 90􀭅, the encoder overheat alarm (AL-26) will be generated.
*/
#define Index_EncoderTemperature                            0x260D

/*
Motor Rated Speed
This represents the rated speed of the driving motor.
*/
#define Index_MotorRatedSpeed                               0x260E

/*
Motor Maximum Speed
This represents the maximum speed of the driving motor.*/
#define Index_MotorMaximumSpeed                             0x260F

/*
Warning Code
This represents a warning code which has occurred in the drive.
*/
#define Index_WarningCode                                   0x2614

// Procedure Command Code & Procedure Command Argument
/*
You can run various procedures with the following procedure command codes and command
arguments. Make sure to enter correct value of command argument prior to entering command code
because the drive refers to the command argument at the moment of entering the command code.
*/
/*
Command code            Command argument    Run procedure
-----------------------------------------------------------------------------
                        1                   Servo on
                        2                   Servo off
Manual Jog              3                   Positive (+) driving (0x2300)
(0x0001)                4                   Negative (-) driving (0x2300)
                        5                   Stop to zero speed
-------------------------------------------------------------------------------           
                        1                   Servo on
Programmed Jog          2                   Servo off
(0x0002)                3                   Operation start
                        4                   Stop to zero speed (server on maintained)
-------------------------------------------------------------------------------                  
Servo Alarm History     1
Initialization(0x0003)  
-------------------------------------------------------------------------------
Off-line Auto Tuning    1                   Start auto tuning
(0x0004)
-------------------------------------------------------------------------------                       
                        1                   Servo on
                        2                   Servo off
Index Pulse Search      3                   Positive (+) search (0x230C)
(0x0005)                4                   Negative (-) search (0x230C)
                        5                   Stop to zero speed
--------------------------------------------------------------------------------                  
Absolute encoder reset  1                   Absolute encoder reset
(0x0006)
--------------------------------------------------------------------------------                      
Instantaneous Maximum
Operation Overload      1                   Resets instantaneous maximum operation overload (0x2604) value
Reset (0x0007)
--------------------------------------------------------------------------------                     
                                            Phase current offset tuning
Phase current offset    1                   (The U-/V-/W-phase offsets are stored in 0x2015 -
tuning                                      7, respectively. If the offset is abnormally large,
(0x0008)                                    AL-15 will be generated.)
---------------------------------------------------------------------------------                                      
Software reset          1                   Software reset
(0x0009)
---------------------------------------------------------------------------------                    
Commutation             1                   Commutation is performed
(0x000A)    
---------------------------------------------------------------------------------              
*/
#define Index_ProcedureCommandCode          0x2700
#define Index_ProcedureCommandArgument      0x2701

/*
Servo Alarm History
This represents the history of servo alarm generated from the drive. Up to 16 servo alarms recently
generated are stored. The SubIndex 1 is the latest alarm while the SubIndex 16 is the oldest one out of
the recently generated alarms. The servo alarm history can be reset by procedure command.*/
#define Index_ServoAlarmHistory             0x2702

// #####################################################
// CiA402 Objects (from 0x6000)

// Error Code
/*This displays the most recent alarm/warning code generated by the servo drive*/
#define Index_ErrorCode                     0x603F

// Controlword
/*
This is composed of bits which control the drive state, the operation mode, and manufacturer-specific
options.
*/
#define Index_Controlword                   0x6040

// Statusword
/*
The Statusword indicates the current state of the drive. It consists of bits that indicate the state
according to the drive and operation mode.
*/
#define Index_Statusword                    0x6041

// Modes of Operation
/*
This sets the servo drive operation mode. The master sets the operation mode when the power is
turned on.
This drive provides the following operation modes:
Setting values      name        Details
0                   -           Mode not assigned
1                   PP          Profile Position mode
2                   -           Reserved
3                   PV          Profile Velocity mode
4                   PT          Profile Torque mode
6                   HM          Homing mode
7                   -           Reserved
8                   CSP         Cyclic Synchronous Position mode
9                   CSV         Cyclic Synchronous Velocity mode
10                  CST         Cyclic Synchronous Torque mode
Other               -           Reserved
*/
#define Index_ModesOfOperation                  0x6060

// Target Position
/*
This specifies the target position in Profile Position (PP) mode and Cyclic Synchronous Position (CSP) mode.
It is used as absolute coordinate or relative coordinate depending on the Bit 4 (0x6040.4) setting of the
Controlword in the PP mode, and is always used as absolute value in the CSP mode.
*/
#define Index_TargetPosition                    0X607A

// Position Demand Internal Value
// This represents the value entered as the command during the position control.
#define Index_PositionDemandInternalValue       0x60FC

// Position Demand Value
/*
This displays the position demand value in the position units (UU) specified by the user.
*/
#define Index_PositionDemandValue               0x6062

// Position Actual Internal Value
/* This displays the actual internal position value in encoder pulses.*/
#define Index_PositionActualInternalValue       0x6063

// Position Actual Value
/* This displays the actual position value in user-defined position units (UU).*/
#define Index_PositionActualValue               0x6064

/* Home Offset
This sets the offset value for the origin of the absolute encoder or absolute external scale and the zero
position of the actual position value (0x6064).
• Incremental Encoder
If it finds the home position or it is at the home position, then the position moved by the home offset
value becomes the zero position.
• Absolute Encoder
If the absolute encoder is connected, then the home offset value is added to the absolute position (the
actual position value).
*/
#define Index_HomeOffset                        0x607C

// Homing Method
/*
This sets the homing method. For more information, refer to 4.6 Homing.
Setting values      Details
0                   Disabled
1                   Homing using the index pulse and reverse limit contact
2                   Homing using the index pulse and forward limit contact
7 to 14             Homing using the index pulse and home contact
24                  Same as method 8 (does not use the index pulse)
28                  Same as method 12 (does not use the index pulse)
33, 34              Homing to the index pulse
35                  Homing to the current position
-1                  Homing using the negative stopper and index pulse
-2                  Homing using the positive stopper and index pulse
-3                  Homing using the negative stopper only
-4                  Homing using the positive stopper only
*/
#define Index_HomingMethod                      0x6098

// Homing Speeds
/*This specifies the operation speed for homing.*/
#define Index_HomingSpeeds                      0x6099

// Software Position Limit
/*
This specifies the software position limit value. It limits the range of the position demand value (0x6062)
and actual position value (0x6064) and checks the new target positions for the setting value at every
cycle.
The minimum software limit value is the reverse rotation limit. The maximum software limit value is the
forward rotation limit.
*/
#define Index_SoftwarePositionLimit             0x607D
#define SubIndex_SoftwarePositionLimit_Min      1
#define SubIndex_SoftwarePositionLimit_Max      2

// Velocity Demand Value
/*
This displays the output speed of the position controller or the command speed input to the speed
controller.*/
#define Index_VelocityDemandValue               0x606B

// Velocity Actual Value
/*This displays the actual velocity value in user-defined position unit.*/
#define Index_VelocityActualValue               0x606C

// Target Velocity
// This specifies the target velocity in the PV mode and the CSV mode.
#define Index_TargetVelocity                    0x60FF

// Max Profile Velocity
/*This specifies the maximum profile speed for the PP mode operation.*/
#define Index_MaxProfileVelocity                0x607F

// Profile Velocity
/*This specifies the profile speed for the PP mode operation.*/
#define Index_ProfileVelocity                   0x6081

// Profile Acceleration
/*This specifies the profile acceleration for the PP/PV mode operation.*/
#define Index_ProfileAcceleration               0x6083

// Profile Deceleration
/*This specifies the profile deceleration for the PP/PV mode operation.*/
#define Index_ProfileDeceleration               0x6084

// Target Torque
/*
This specifies the target torque for the motor in 0.1% increments of the rated torque during torque
control.
*/
#define Index_TargetTorque                  0x6071

// Maximum Torque
/*
This sets the maximum torque that the motor can output in 0.1% increments of the rated torque.
Used in all mode.
*/
#define Index_MaximumTorque                 0x6072

// Torque Demand Value
/*This displays the current torque demand value in 0.1% increments of the rated torque.*/
#define Index_TorqueDemandValue             0x6074

// Torque Actual Value
/*This displays the actual torque value generated by the drive in 0.1% increments of the rated torque.*/
#define Index_TorqueActualValue             0x6077

// Torque Slope
/*This specifies the torque slope for the PT mode operation.*/
#define Index_TorqueSlope                   0x6087

// Positive Torque Limit Value
/*This sets the limit of positive torque values.*/
#define Index_PositiveTorqueLimitValue      0x60E0

// Negative Torque Limit Value
/*This sets the limit of negative torque values.*/
#define Index_NegativeTorqueLimitValue      0x60E1

// Supported Drive Modes
// This displays the mode(s) supported by the drive.
/*
Bit         Supported modes                     Details
0           PP (Profile Position)               1: Supported
1           Vl (Velocity)                       0: Not supported
2           PV (Profile Velocity)               1: Supported
3           PT (Torque Profile)                 1: Supported
4           Reserved                            0
5           HM (Homing)                         1: Supported
6           IP (Interpolated Position)          0: Not Supported
7           CSP (Cyclic Synchronous Position)   1: Supported
8           CSV (Cyclic Synchronous Velocity)   1: Supported
9           CST (Cyclic Synchronous Torque)     1: Supported
10 to 31    Reserved                           0
*/
#define Index_SupportedDriveModes           0x6502

// Digital Inputs
/*They indicate the status of digital inputs.*/
/*
Bit                 Description
0                   NOT (negative limit switch)
1                   POT (positive limit switch)
2                   HOME (origin sensor input)
3 to 15             Reserved
16                  DI #1(I/O pin 11), 0:Open, 1:Close
17                  DI #2(I/O pin 12), 0:Open, 1:Close
18                  DI #3(I/O pin 7), 0:Open, 1:Close
19                  DI #4(I/O pin 8), 0:Open, 1:Close
20                  DI #5(I/O pin 13), 0:Open, 1:Close
21                  DI #6(I/O pin 14), 0:Open, 1:Close
22                  DI #7(I/O pin 9), 0:Open, 1:Close
23                  DI #8(I/O pin 10), 0:Open, 1:Close
24~30               Reserved
31                  STO(Safe Torque Off), 0:Close, 1:Open
*/
#define Index_DigitalInputs                 0x60FD

// Digital Outputs
/*They indicate the status of digital outputs.*/
/*
Description of physical outputs
Bit         Description
0 to 15     Reserved
16          Forced output (0: OFF, 1: ON) of DO #1 (I/O pins 3 and 4)
            Provided that the relevant bit mask (0x60FE:02.16) is set to 1.
17          Forced output (0: OFF, 1: ON) of DO #2 (I/O pins 23 and 24)
            Provided that the relevant bit mask (0x60FE:02.17) is set to 1.
18          Forced output (0: OFF, 1: ON) of DO #3 (I/O pins 25 and 26)
            Provided that the relevant bit mask (0x60FE:02.18) is set to 1.
19          Forced output (0: OFF, 1: ON) of DO #4 (I/O pins 1 and 2)
            Provided that the relevant bit mask (0x60FE:02.19) is set to 1.
20 to 23    Reserved
24          Output status of DO #1 (0: OFF, 1: ON)
25          Output status of DO #2 (0: OFF, 1: ON)
26          Output status of DO #3 (0: OFF, 1: ON)
27          Output status of DO #4 (0: OFF, 1: ON)
28 to 31    Reserved
*/
#define Index_DigitalOutputs                        0x60FE
#define SubIndex_DigitalOutputs_Physicaloutputs     1


#endif