// This file is autogenerated by VESC Tool

#ifndef MCCONF_DEFAULT_H_
#define MCCONF_DEFAULT_H_

// PWM Mode
#define MCCONF_PWM_MODE 1

// Commutation Mode
#define MCCONF_COMM_MODE 0

// Motor Type
#define MCCONF_DEFAULT_MOTOR_TYPE 2

// Sensor Mode
#define MCCONF_SENSOR_MODE 0

// Motor Current Max
#define MCCONF_L_CURRENT_MAX 60

// Motor Current Max Brake
#define MCCONF_L_CURRENT_MIN -60

// Battery Current Max
#define MCCONF_L_IN_CURRENT_MAX 133.33

// Battery Current Max Regen
#define MCCONF_L_IN_CURRENT_MIN -133.33

// Input Current Limit Map Start
#define MCCONF_L_IN_CURRENT_MAP_START 1

// Input Current Map Filter
#define MCCONF_L_IN_CURRENT_MAP_FILTER 0.005

// Absolute Maximum Current
#define MCCONF_L_MAX_ABS_CURRENT 120

// Max ERPM Reverse
#define MCCONF_L_RPM_MIN -150000

// Max ERPM
#define MCCONF_L_RPM_MAX 150000

// ERPM Limit Start
#define MCCONF_L_RPM_START 0.8

// Max ERPM Full Brake
#define MCCONF_L_CURR_MAX_RPM_FBRAKE 300

// Max ERPM Full Brake Current Control
#define MCCONF_L_CURR_MAX_RPM_FBRAKE_CC 1500

// Minimum Input Voltage
#define MCCONF_L_MIN_VOLTAGE 12

// Maximum Input Voltage
#define MCCONF_L_MAX_VOLTAGE 84

// Battery Voltage Cutoff Start
#define MCCONF_L_BATTERY_CUT_START 0

// Battery Voltage Cutoff End
#define MCCONF_L_BATTERY_CUT_END 0

// Battery Voltage Regen Cutoff Start
#define MCCONF_L_BATTERY_REGEN_CUT_START 1000

// Battery Voltage Regen Cutoff End
#define MCCONF_L_BATTERY_REGEN_CUT_END 1100

// Slow ABS Current Limit
#define MCCONF_L_SLOW_ABS_OVERCURRENT 0

// MOSFET Temp Cutoff Start
#define MCCONF_L_LIM_TEMP_FET_START 110

// MOSFET Temp Cutoff End
#define MCCONF_L_LIM_TEMP_FET_END 110

// Motor Temp Cutoff Start
#define MCCONF_L_LIM_TEMP_MOTOR_START 85

// Motor Temp Cutoff End
#define MCCONF_L_LIM_TEMP_MOTOR_END 100

// Acceleration Temperature Decrease
#define MCCONF_L_LIM_TEMP_ACCEL_DEC 0.15

// Minimum Duty Cycle
#define MCCONF_L_MIN_DUTY 0.005

// Maximum Duty Cycle
#define MCCONF_L_MAX_DUTY 0.99

// Maximum Wattage
#define MCCONF_L_WATT_MAX 1.5e+06

// Maximum Braking Wattage
#define MCCONF_L_WATT_MIN -1.5e+06

// Max Current Scale
#define MCCONF_L_CURRENT_MAX_SCALE 1

// Min Current Scale
#define MCCONF_L_CURRENT_MIN_SCALE 1

// Duty Cycle Current Limit Start
#define MCCONF_L_DUTY_START 1

// Minimum ERPM
#define MCCONF_SL_MIN_RPM 150

// Minimum ERPM Integrator
#define MCCONF_SL_MIN_ERPM_CYCLE_INT_LIMIT 1100

// Max Brake Current at Direction Change
#define MCCONF_SL_MAX_FB_CURR_DIR_CHANGE 10

// Cycle Integrator Limit
#define MCCONF_SL_CYCLE_INT_LIMIT 62

// Phase Advance at BR ERPM
#define MCCONF_SL_PHASE_ADVANCE_AT_BR 0.8

// BR ERPM
#define MCCONF_SL_CYCLE_INT_BR 80000

// BEMF Coupling
#define MCCONF_SL_BEMF_COUPLING_K 600

// Hall Table [0]
#define MCCONF_HALL_TAB_0 -1

// Hall Table [1]
#define MCCONF_HALL_TAB_1 1

// Hall Table [2]
#define MCCONF_HALL_TAB_2 3

// Hall Table [3]
#define MCCONF_HALL_TAB_3 2

// Hall Table [4]
#define MCCONF_HALL_TAB_4 5

// Hall Table [5]
#define MCCONF_HALL_TAB_5 6

// Hall Table [6]
#define MCCONF_HALL_TAB_6 4

// Hall Table [7]
#define MCCONF_HALL_TAB_7 -1

// Sensorless ERPM Hybrid
#define MCCONF_HALL_ERPM 2000

// Current KP
#define MCCONF_FOC_CURRENT_KP 0.15

// Current KI
#define MCCONF_FOC_CURRENT_KI 15

// Zero Vector Frequency
#define MCCONF_FOC_F_ZV 30000

// Dead Time Compensation
#define MCCONF_FOC_DT_US 0.12

// Encoder Inverted
#define MCCONF_FOC_ENCODER_INVERTED 0

// Encoder Offset
#define MCCONF_FOC_ENCODER_OFFSET 48.35

// Encoder Ratio
#define MCCONF_FOC_ENCODER_RATIO 21

// Sensor Mode
#define MCCONF_FOC_SENSOR_MODE 1

// Speed Tracker Kp
#define MCCONF_FOC_PLL_KP 2000

// Speed Tracker Ki
#define MCCONF_FOC_PLL_KI 30000

// Motor Inductance (L)
#define MCCONF_FOC_MOTOR_L 3.564e-05

// Motor Inductance Difference (Ld - Lq)
#define MCCONF_FOC_MOTOR_LD_LQ_DIFF 1.498e-05

// Motor Resistance (R)
#define MCCONF_FOC_MOTOR_R 0.0747

// Motor Flux Linkage (λ)
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE 0.00282

// Observer Gain (x1M)
#define MCCONF_FOC_OBSERVER_GAIN 7.592e+07

// Observer Gain At Minimum Duty
#define MCCONF_FOC_OBSERVER_GAIN_SLOW 0.05

// Observer Offset
#define MCCONF_FOC_OBSERVER_OFFSET 0

// Duty Downramp Kp
#define MCCONF_FOC_DUTY_DOWNRAMP_KP 1

// Duty Downramp Ki
#define MCCONF_FOC_DUTY_DOWNRAMP_KI 1000

// Start Current Decrease
#define MCCONF_FOC_START_CURR_DEC 1

// Start Current Decrease ERPM
#define MCCONF_FOC_START_CURR_DEC_RPM 2500

// Openloop ERPM
#define MCCONF_FOC_OPENLOOP_RPM 1400

// Openloop ERPM at Min Current
#define MCCONF_FOC_OPENLOOP_RPM_LOW 0

// D Axis Gain Scaling Start
#define MCCONF_FOC_D_GAIN_SCALE_START 0.9

// D Axis Gain Scaling at Max Mod
#define MCCONF_FOC_D_GAIN_SCALE_MAX_MOD 1

// Openloop Hysteresis
#define MCCONF_FOC_SL_OPENLOOP_HYST 0.1

// Openloop Lock Time
#define MCCONF_FOC_SL_OPENLOOP_T_LOCK 0

// Openloop Ramp Time
#define MCCONF_FOC_SL_OPENLOOP_T_RAMP 0.1

// Openloop Time
#define MCCONF_FOC_SL_OPENLOOP_TIME 0.05

// Openloop Current Boost
#define MCCONF_FOC_SL_OPENLOOP_BOOST_Q 0

// Openloop Current Max
#define MCCONF_FOC_SL_OPENLOOP_MAX_Q -1

// Hall Table [0]
#define MCCONF_FOC_HALL_TAB_0 255

// Hall Table [1]
#define MCCONF_FOC_HALL_TAB_1 255

// Hall Table [2]
#define MCCONF_FOC_HALL_TAB_2 255

// Hall Table [3]
#define MCCONF_FOC_HALL_TAB_3 255

// Hall Table [4]
#define MCCONF_FOC_HALL_TAB_4 255

// Hall Table [5]
#define MCCONF_FOC_HALL_TAB_5 255

// Hall Table [6]
#define MCCONF_FOC_HALL_TAB_6 255

// Hall Table [7]
#define MCCONF_FOC_HALL_TAB_7 255

// Hall Interpolation ERPM
#define MCCONF_FOC_HALL_INTERP_ERPM 500

// Sensored ERPM Start
#define MCCONF_FOC_SL_ERPM_START 2500

// Sensorless ERPM
#define MCCONF_FOC_SL_ERPM 12000

// Sample in V0 and V7
#define MCCONF_FOC_SAMPLE_V0_V7 1

// High Current Sampling Mode
#define MCCONF_FOC_SAMPLE_HIGH_CURRENT 0

// Saturation Compensation Mode
#define MCCONF_FOC_SAT_COMP_MODE 2

// Saturation Compensation Factor
#define MCCONF_FOC_SAT_COMP 0

// Temp Comp
#define MCCONF_FOC_TEMP_COMP 0

// Temp Comp Base Temp
#define MCCONF_FOC_TEMP_COMP_BASE_TEMP 27.5

// Current Filter Constant
#define MCCONF_FOC_CURRENT_FILTER_CONST 1

// Current Controller Decoupling
#define MCCONF_FOC_CC_DECOUPLING 0

// Observer Type
#define MCCONF_FOC_OBSERVER_TYPE 3

// HFI Start Voltage
#define MCCONF_FOC_HFI_VOLTAGE_START 20

// HFI Run Voltage
#define MCCONF_FOC_HFI_VOLTAGE_RUN 4

// HFI Max Voltage
#define MCCONF_FOC_HFI_VOLTAGE_MAX 6

// HFI Gain
#define MCCONF_FOC_HFI_GAIN 0.3

// HFI Current Hysteresis
#define MCCONF_FOC_HFI_HYST 0

// Sensorless ERPM HFI
#define MCCONF_FOC_SL_ERPM_HFI 3000

// HFI Start Samples
#define MCCONF_FOC_HFI_START_SAMPLES 5

// HFI Observer Override Time
#define MCCONF_FOC_HFI_OBS_OVR_SEC 0.001

// HFI Samples
#define MCCONF_FOC_HFI_SAMPLES 1

// Run calibration at boot
#define MCCONF_FOC_OFFSETS_CAL_ON_BOOT 1

// Current Offset 0
#define MCCONF_FOC_OFFSETS_CURRENT_0 2040.24

// Current Offset 1
#define MCCONF_FOC_OFFSETS_CURRENT_1 2039.84

// Current Offset 2
#define MCCONF_FOC_OFFSETS_CURRENT_2 2042.68

// Voltage Offset 0
#define MCCONF_FOC_OFFSETS_VOLTAGE_0 0.006

// Voltage Offset 1
#define MCCONF_FOC_OFFSETS_VOLTAGE_1 -0.0082

// Voltage Offset 2
#define MCCONF_FOC_OFFSETS_VOLTAGE_2 0.0021

// Voltage Offset Undriven 0
#define MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_0 0

// Voltage Offset Undriven 1
#define MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_1 0

// Voltage Offset Undriven 2
#define MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_2 0

// Enable Phase Filters
#define MCCONF_FOC_PHASE_FILTER_ENABLE 0

// Disable Phase Filter Fault Code
#define MCCONF_FOC_PHASE_FILTER_DISABLE_FAULT 1

// Maximum ERPM for phase filters
#define MCCONF_FOC_PHASE_FILTER_MAX_ERPM 4000

// MTPA Algorithm Mode
#define MCCONF_FOC_MTPA_MODE 0

// Field Weakening Current Max
#define MCCONF_FOC_FW_CURRENT_MAX 0

// Field Weakening Duty Start
#define MCCONF_FOC_FW_DUTY_START 0.9

// Field Weakening Ramp Time
#define MCCONF_FOC_FW_RAMP_TIME 0.2

// Q Axis Current Factor
#define MCCONF_FOC_FW_Q_CURRENT_FACTOR 0.02

// Speed Tracker Position Source
#define MCCONF_FOC_SPEED_SOURCE 0

// Buffer Notification Length
#define MCCONF_GPD_BUFFER_NOTIFY_LEFT 200

// Buffer Sampling Interpolation
#define MCCONF_GPD_BUFFER_INTERPOL 0

// Current Filter Constant
#define MCCONF_GPD_CURRENT_FILTER_CONST 0.1

// Current KP
#define MCCONF_GPD_CURRENT_KP 0.03

// Current KI
#define MCCONF_GPD_CURRENT_KI 50

// PID Loop Rate
#define MCCONF_SP_PID_LOOP_RATE 8

// Speed PID Kp
#define MCCONF_S_PID_KP 0.002

// Speed PID Ki
#define MCCONF_S_PID_KI 0.1

// Speed PID Kd
#define MCCONF_S_PID_KD 6e-06

// Speed PID Kd Filter
#define MCCONF_S_PID_KD_FILTER 0.2

// Minimum ERPM
#define MCCONF_S_PID_MIN_RPM 0

// Allow Braking
#define MCCONF_S_PID_ALLOW_BRAKING 1

// Ramp eRPMs per second
#define MCCONF_S_PID_RAMP_ERPMS_S 25000

// Speed Source
#define MCCONF_S_PID_SPEED_SOURCE 0

// Position PID Kp
#define MCCONF_P_PID_KP 0.01

// Position PID Ki
#define MCCONF_P_PID_KI 0

// Position PID Kd
#define MCCONF_P_PID_KD 0

// Position PID Kd Process
#define MCCONF_P_PID_KD_PROC 0.00035

// Position PID Kd Filter
#define MCCONF_P_PID_KD_FILTER 0.002

// Position Angle Division
#define MCCONF_P_PID_ANG_DIV 1

// Gain Decrease Angle
#define MCCONF_P_PID_GAIN_DEC_ANGLE 0

// Position PID Offset Angle
#define MCCONF_P_PID_OFFSET 207.861

// Startup boost
#define MCCONF_CC_STARTUP_BOOST_DUTY 0.01

// Minimum Current
#define MCCONF_CC_MIN_CURRENT 0.05

// Current Controller Gain
#define MCCONF_CC_GAIN 0.0046

// Current Control Ramp Step Max
#define MCCONF_CC_RAMP_STEP 0.04

// Fault Stop Time
#define MCCONF_M_FAULT_STOP_TIME 500

// Duty Ramp Step Max
#define MCCONF_M_RAMP_STEP 0.02

// Current Backoff Gain
#define MCCONF_M_CURRENT_BACKOFF_GAIN 0.5

// Encoder counts
#define MCCONF_M_ENCODER_COUNTS 8192

// Sine Amplitude
#define MCCONF_M_ENCODER_SIN_AMP 1

// Cosine Amplitude
#define MCCONF_M_ENCODER_COS_AMP 1

// Sine Offset
#define MCCONF_M_ENCODER_SIN_OFFSET 1.65

// Cosine Offset
#define MCCONF_M_ENCODER_COS_OFFSET 1.65

// Sin/Cos Filter Constant
#define MCCONF_M_ENCODER_SINCOS_FILTER 0.5

// Sin/Cos Phase Correction
#define MCCONF_M_ENCODER_SINCOS_PHASE 0

// Sensor Port Mode
#define MCCONF_M_SENSOR_PORT_MODE 2

// Invert Motor Direction
#define MCCONF_M_INVERT_DIRECTION 1

// DRV8301 OC Mode
#define MCCONF_M_DRV8301_OC_MODE 0

// DRV8301 OC Adjustment
#define MCCONF_M_DRV8301_OC_ADJ 16

// Minimum Switching Frequency
#define MCCONF_M_BLDC_F_SW_MIN 3000

// Maximum Switching Frequency
#define MCCONF_M_BLDC_F_SW_MAX 35000

// Switching Frequency
#define MCCONF_M_DC_F_SW 25000

// Beta Value for Motor Thermistor
#define MCCONF_M_NTC_MOTOR_BETA 3478

// Auxiliary Output Mode
#define MCCONF_M_OUT_AUX_MODE 0

// Motor Temperature Sensor Type
#define MCCONF_M_MOTOR_TEMP_SENS_TYPE 0

// Coefficient for PTC Motor Thermistor
#define MCCONF_M_PTC_MOTOR_COEFF 0.61

// Custom NTC/PTC Resistance
#define MCCONF_M_NTCX_PTCX_RES 10000

// Custom NTC/PTC Base Temperature
#define MCCONF_M_NTCX_PTCX_BASE_TEMP 25

// Hall Sensor Extra Samples
#define MCCONF_M_HALL_EXTRA_SAMPLES 2

// Battery Filter Constant
#define MCCONF_M_BATT_FILTER_CONST 45

// Motor Poles
#define MCCONF_SI_MOTOR_POLES 42

// Gear Ratio
#define MCCONF_SI_GEAR_RATIO 1

// Wheel Diameter
#define MCCONF_SI_WHEEL_DIAMETER 0.23368

// Battery Type
#define MCCONF_SI_BATTERY_TYPE 0

// Battery Cells Series
#define MCCONF_SI_BATTERY_CELLS 17

// Battery Capacity
#define MCCONF_SI_BATTERY_AH 2.8

// Motor No Load Current
#define MCCONF_SI_MOTOR_NL_CURRENT 1

// BMS Type
#define MCCONF_BMS_TYPE 1

// BMS Limit Mode
#define MCCONF_BMS_LIMIT_MODE 3

// Temperature Limit Start
#define MCCONF_BMS_T_LIMIT_START 45

// Temperature Limit End
#define MCCONF_BMS_T_LIMIT_END 65

// SOC Limit Start
#define MCCONF_BMS_SOC_LIMIT_START 0.05

// SOC Limit End
#define MCCONF_BMS_SOC_LIMIT_END 0

// Forward CAN to Local
#define MCCONF_BMS_FWD_CAN_MODE 0

// MCCONF_DEFAULT_H_
#endif

