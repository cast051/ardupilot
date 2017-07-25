// BUILDROOT=/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/Build.ArduCopter HAL_BOARD=HAL_BOARD_PX4 HAL_BOARD_SUBTYPE= TOOLCHAIN=NATIVE EXTRAFLAGS=-DGIT_VERSION="b0a25989" -I/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/libraries/AP_Common/missing -DMAVLINK_PROTOCOL_VERSION=2 -DNUTTX_GIT_VERSION="c626db4d" -DPX4_GIT_VERSION="f84ee497" -DUAVCAN=1 -D__STDC_FORMAT_MACROS -DHAVE_STD_NULLPTR_T=0 -DHAVE_ENDIAN_H=0 -DHAVE_BYTESWAP_H=0 -DHAVE_OCLOEXEC=0 -I/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/Build.ArduCopter/libraries/GCS_MAVLink/include/mavlink -I/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/uavcan/libuavcan/include -I/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/uavcan/libuavcan/include/dsdlc_generated
Checking modules
Building /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/module.mk
%% module_mk
make[1]: Entering directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware'
Skipping submodules. NUTTX_SRC is set to /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/NuttX/nuttx/
Generating uORB topic headers
vehicle_local_position_setpoint.h: unchanged
hil_sensor.h: unchanged
vision_position_estimate.h: unchanged
distance_sensor.h: unchanged
actuator_controls.h: unchanged
esc_report.h: unchanged
vehicle_status.h: unchanged
actuator_controls_virtual_mc.h: unchanged
actuator_controls_virtual_fw.h: unchanged
home_position.h: unchanged
vtol_vehicle_status.h: unchanged
sensor_combined.h: unchanged
navigation_capabilities.h: unchanged
actuator_controls_3.h: unchanged
mc_virtual_attitude_setpoint.h: unchanged
vehicle_command.h: unchanged
airspeed.h: unchanged
actuator_controls_2.h: unchanged
actuator_armed.h: unchanged
time_offset.h: unchanged
filtered_bottom_flow.h: unchanged
vehicle_force_setpoint.h: unchanged
vehicle_rates_setpoint.h: unchanged
vehicle_gps_position.h: unchanged
ekf2_replay.h: unchanged
vehicle_local_position.h: unchanged
actuator_direct.h: unchanged
rc_parameter_map.h: unchanged
sensor_mag.h: unchanged
fence_vertex.h: unchanged
tecs_status.h: unchanged
encoders.h: unchanged
camera_trigger.h: unchanged
esc_status.h: unchanged
fence.h: unchanged
sensor_accel.h: unchanged
sensor_gyro.h: unchanged
output_pwm.h: unchanged
mavlink_log.h: unchanged
qshell_req.h: unchanged
att_pos_mocap.h: unchanged
optical_flow.h: unchanged
subsystem_info.h: unchanged
geofence_result.h: unchanged
follow_target.h: unchanged
manual_control_setpoint.h: unchanged
vehicle_command_ack.h: unchanged
vehicle_global_velocity_setpoint.h: unchanged
rc_channels.h: unchanged
control_state.h: unchanged
vehicle_attitude_setpoint.h: unchanged
actuator_controls_1.h: unchanged
sensor_baro.h: unchanged
differential_pressure.h: unchanged
safety.h: unchanged
mc_att_ctrl_status.h: unchanged
ekf2_innovations.h: unchanged
uavcan_parameter_value.h: unchanged
pwm_input.h: unchanged
telemetry_status.h: unchanged
mission.h: unchanged
vehicle_attitude.h: unchanged
uavcan_parameter_request.h: unchanged
actuator_controls_0.h: unchanged
debug_key_value.h: unchanged
fw_virtual_rates_setpoint.h: unchanged
actuator_outputs.h: unchanged
satellite_info.h: unchanged
estimator_status.h: unchanged
servorail_status.h: unchanged
input_rc.h: unchanged
vehicle_control_mode.h: unchanged
fw_virtual_attitude_setpoint.h: unchanged
position_setpoint_triplet.h: unchanged
mc_virtual_rates_setpoint.h: unchanged
test_motor.h: unchanged
wind_estimate.h: unchanged
parameter_update.h: unchanged
battery_status.h: unchanged
vehicle_land_detected.h: unchanged
multirotor_motor_limits.h: unchanged
system_power.h: unchanged
offboard_control_mode.h: unchanged
mission_result.h: unchanged
position_setpoint.h: unchanged
vehicle_global_position.h: unchanged
Generating multiplatform uORB topic wrapper headers
vehicle_local_position_setpoint.h: unchanged
hil_sensor.h: unchanged
vision_position_estimate.h: unchanged
distance_sensor.h: unchanged
actuator_controls.h: unchanged
esc_report.h: unchanged
vehicle_status.h: unchanged
actuator_controls_virtual_mc.h: unchanged
actuator_controls_virtual_fw.h: unchanged
home_position.h: unchanged
vtol_vehicle_status.h: unchanged
sensor_combined.h: unchanged
navigation_capabilities.h: unchanged
actuator_controls_3.h: unchanged
mc_virtual_attitude_setpoint.h: unchanged
vehicle_command.h: unchanged
airspeed.h: unchanged
actuator_controls_2.h: unchanged
actuator_armed.h: unchanged
time_offset.h: unchanged
filtered_bottom_flow.h: unchanged
vehicle_force_setpoint.h: unchanged
vehicle_rates_setpoint.h: unchanged
vehicle_gps_position.h: unchanged
ekf2_replay.h: unchanged
vehicle_local_position.h: unchanged
actuator_direct.h: unchanged
rc_parameter_map.h: unchanged
sensor_mag.h: unchanged
fence_vertex.h: unchanged
tecs_status.h: unchanged
encoders.h: unchanged
camera_trigger.h: unchanged
esc_status.h: unchanged
fence.h: unchanged
sensor_accel.h: unchanged
sensor_gyro.h: unchanged
output_pwm.h: unchanged
mavlink_log.h: unchanged
qshell_req.h: unchanged
att_pos_mocap.h: unchanged
optical_flow.h: unchanged
subsystem_info.h: unchanged
geofence_result.h: unchanged
follow_target.h: unchanged
manual_control_setpoint.h: unchanged
vehicle_command_ack.h: unchanged
vehicle_global_velocity_setpoint.h: unchanged
rc_channels.h: unchanged
control_state.h: unchanged
vehicle_attitude_setpoint.h: unchanged
actuator_controls_1.h: unchanged
sensor_baro.h: unchanged
differential_pressure.h: unchanged
safety.h: unchanged
mc_att_ctrl_status.h: unchanged
ekf2_innovations.h: unchanged
uavcan_parameter_value.h: unchanged
pwm_input.h: unchanged
telemetry_status.h: unchanged
mission.h: unchanged
vehicle_attitude.h: unchanged
uavcan_parameter_request.h: unchanged
actuator_controls_0.h: unchanged
debug_key_value.h: unchanged
fw_virtual_rates_setpoint.h: unchanged
actuator_outputs.h: unchanged
satellite_info.h: unchanged
estimator_status.h: unchanged
servorail_status.h: unchanged
input_rc.h: unchanged
vehicle_control_mode.h: unchanged
fw_virtual_attitude_setpoint.h: unchanged
position_setpoint_triplet.h: unchanged
mc_virtual_rates_setpoint.h: unchanged
test_motor.h: unchanged
wind_estimate.h: unchanged
parameter_update.h: unchanged
battery_status.h: unchanged
vehicle_land_detected.h: unchanged
multirotor_motor_limits.h: unchanged
system_power.h: unchanged
offboard_control_mode.h: unchanged
mission_result.h: unchanged
position_setpoint.h: unchanged
vehicle_global_position.h: unchanged
%%%%
%%%% Building px4io-v2_default in /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/
%%%%
make[2]: Entering directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build'
%  PX4_BASE            = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/
%  GIT_DESC            = 
%  CONFIG              = px4io-v2_default
%  BOARD               = px4io-v2
%  WORK_DIR            = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/
%  NUTTX_EXPORT_DIR    = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/nuttx-export/
%  NUTTX_CONFIG_HEADER = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/nuttx-export/include/nuttx/config.h
make[3]: Entering directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/boards/px4io-v2'
%% MODULE_MK           = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/boards/px4io-v2/module.mk
%  MODULE_NAME         = px4io-v2
%  MODULE_SRC          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/boards/px4io-v2/
%  MODULE_OBJ          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/boards/px4io-v2/module.pre.o
%  MODULE_WORK_DIR     = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/boards/px4io-v2
make[3]: Nothing to be done for 'module'.
make[3]: Leaving directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/boards/px4io-v2'
make[3]: Entering directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/stm32'
%% MODULE_MK           = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/stm32/module.mk
%  MODULE_NAME         = stm32
%  MODULE_SRC          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/stm32/
%  MODULE_OBJ          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/stm32/module.pre.o
%  MODULE_WORK_DIR     = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/stm32
make[3]: Nothing to be done for 'module'.
make[3]: Leaving directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/drivers/stm32'
make[3]: Entering directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/modules/px4iofirmware'
%% MODULE_MK           = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/modules/px4iofirmware/module.mk
%  MODULE_NAME         = px4iofirmware
%  MODULE_SRC          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/modules/px4iofirmware/
%  MODULE_OBJ          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/modules/px4iofirmware/module.pre.o
%  MODULE_WORK_DIR     = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/modules/px4iofirmware
make[3]: Nothing to be done for 'module'.
make[3]: Leaving directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/modules/px4iofirmware'
make[3]: Entering directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/common'
%% MODULE_MK           = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/common/module.mk
%  MODULE_NAME         = common
%  MODULE_SRC          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/common/
%  MODULE_OBJ          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/common/module.pre.o
%  MODULE_WORK_DIR     = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/common
make[3]: Nothing to be done for 'module'.
make[3]: Leaving directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/common'
make[3]: Entering directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/nuttx/px4_layer'
%% MODULE_MK           = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/nuttx/px4_layer/module.mk
%  MODULE_NAME         = px4_layer
%  MODULE_SRC          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/nuttx/px4_layer/
%  MODULE_OBJ          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/nuttx/px4_layer/module.pre.o
%  MODULE_WORK_DIR     = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/nuttx/px4_layer
make[3]: Nothing to be done for 'module'.
make[3]: Leaving directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/src/platforms/nuttx/px4_layer'
LINK:    /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/firmware.elf
BIN:     /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/firmware.bin
%% Generating /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build/firmware.px4
make[2]: Leaving directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4io-v2_default.build'
%% Copying /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Images/px4io-v2_default.px4
make[1]: Leaving directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware'
PX4IOv2 Firmware is in px4io-v2.bin
Building px4-v2
%% px4-v2
make[1]: Entering directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot'
Skipping submodules. NUTTX_SRC is set to /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4NuttX/nuttx/
Generating uORB topic headers
vehicle_local_position_setpoint.h: unchanged
hil_sensor.h: unchanged
vision_position_estimate.h: unchanged
distance_sensor.h: unchanged
actuator_controls.h: unchanged
esc_report.h: unchanged
vehicle_status.h: unchanged
actuator_controls_virtual_mc.h: unchanged
actuator_controls_virtual_fw.h: unchanged
home_position.h: unchanged
vtol_vehicle_status.h: unchanged
sensor_combined.h: unchanged
navigation_capabilities.h: unchanged
actuator_controls_3.h: unchanged
mc_virtual_attitude_setpoint.h: unchanged
vehicle_command.h: unchanged
airspeed.h: unchanged
actuator_controls_2.h: unchanged
actuator_armed.h: unchanged
time_offset.h: unchanged
filtered_bottom_flow.h: unchanged
vehicle_force_setpoint.h: unchanged
vehicle_rates_setpoint.h: unchanged
vehicle_gps_position.h: unchanged
ekf2_replay.h: unchanged
vehicle_local_position.h: unchanged
actuator_direct.h: unchanged
rc_parameter_map.h: unchanged
sensor_mag.h: unchanged
fence_vertex.h: unchanged
tecs_status.h: unchanged
encoders.h: unchanged
camera_trigger.h: unchanged
esc_status.h: unchanged
fence.h: unchanged
sensor_accel.h: unchanged
sensor_gyro.h: unchanged
output_pwm.h: unchanged
mavlink_log.h: unchanged
qshell_req.h: unchanged
att_pos_mocap.h: unchanged
optical_flow.h: unchanged
subsystem_info.h: unchanged
geofence_result.h: unchanged
follow_target.h: unchanged
manual_control_setpoint.h: unchanged
vehicle_command_ack.h: unchanged
vehicle_global_velocity_setpoint.h: unchanged
rc_channels.h: unchanged
control_state.h: unchanged
vehicle_attitude_setpoint.h: unchanged
actuator_controls_1.h: unchanged
sensor_baro.h: unchanged
differential_pressure.h: unchanged
safety.h: unchanged
mc_att_ctrl_status.h: unchanged
ekf2_innovations.h: unchanged
uavcan_parameter_value.h: unchanged
pwm_input.h: unchanged
telemetry_status.h: unchanged
mission.h: unchanged
vehicle_attitude.h: unchanged
uavcan_parameter_request.h: unchanged
actuator_controls_0.h: unchanged
debug_key_value.h: unchanged
fw_virtual_rates_setpoint.h: unchanged
actuator_outputs.h: unchanged
satellite_info.h: unchanged
estimator_status.h: unchanged
servorail_status.h: unchanged
input_rc.h: unchanged
vehicle_control_mode.h: unchanged
fw_virtual_attitude_setpoint.h: unchanged
position_setpoint_triplet.h: unchanged
mc_virtual_rates_setpoint.h: unchanged
test_motor.h: unchanged
wind_estimate.h: unchanged
parameter_update.h: unchanged
battery_status.h: unchanged
vehicle_land_detected.h: unchanged
multirotor_motor_limits.h: unchanged
system_power.h: unchanged
offboard_control_mode.h: unchanged
mission_result.h: unchanged
position_setpoint.h: unchanged
vehicle_global_position.h: unchanged
Generating multiplatform uORB topic wrapper headers
vehicle_local_position_setpoint.h: unchanged
hil_sensor.h: unchanged
vision_position_estimate.h: unchanged
distance_sensor.h: unchanged
actuator_controls.h: unchanged
esc_report.h: unchanged
vehicle_status.h: unchanged
actuator_controls_virtual_mc.h: unchanged
actuator_controls_virtual_fw.h: unchanged
home_position.h: unchanged
vtol_vehicle_status.h: unchanged
sensor_combined.h: unchanged
navigation_capabilities.h: unchanged
actuator_controls_3.h: unchanged
mc_virtual_attitude_setpoint.h: unchanged
vehicle_command.h: unchanged
airspeed.h: unchanged
actuator_controls_2.h: unchanged
actuator_armed.h: unchanged
time_offset.h: unchanged
filtered_bottom_flow.h: unchanged
vehicle_force_setpoint.h: unchanged
vehicle_rates_setpoint.h: unchanged
vehicle_gps_position.h: unchanged
ekf2_replay.h: unchanged
vehicle_local_position.h: unchanged
actuator_direct.h: unchanged
rc_parameter_map.h: unchanged
sensor_mag.h: unchanged
fence_vertex.h: unchanged
tecs_status.h: unchanged
encoders.h: unchanged
camera_trigger.h: unchanged
esc_status.h: unchanged
fence.h: unchanged
sensor_accel.h: unchanged
sensor_gyro.h: unchanged
output_pwm.h: unchanged
mavlink_log.h: unchanged
qshell_req.h: unchanged
att_pos_mocap.h: unchanged
optical_flow.h: unchanged
subsystem_info.h: unchanged
geofence_result.h: unchanged
follow_target.h: unchanged
manual_control_setpoint.h: unchanged
vehicle_command_ack.h: unchanged
vehicle_global_velocity_setpoint.h: unchanged
rc_channels.h: unchanged
control_state.h: unchanged
vehicle_attitude_setpoint.h: unchanged
actuator_controls_1.h: unchanged
sensor_baro.h: unchanged
differential_pressure.h: unchanged
safety.h: unchanged
mc_att_ctrl_status.h: unchanged
ekf2_innovations.h: unchanged
uavcan_parameter_value.h: unchanged
pwm_input.h: unchanged
telemetry_status.h: unchanged
mission.h: unchanged
vehicle_attitude.h: unchanged
uavcan_parameter_request.h: unchanged
actuator_controls_0.h: unchanged
debug_key_value.h: unchanged
fw_virtual_rates_setpoint.h: unchanged
actuator_outputs.h: unchanged
satellite_info.h: unchanged
estimator_status.h: unchanged
servorail_status.h: unchanged
input_rc.h: unchanged
vehicle_control_mode.h: unchanged
fw_virtual_attitude_setpoint.h: unchanged
position_setpoint_triplet.h: unchanged
mc_virtual_rates_setpoint.h: unchanged
test_motor.h: unchanged
wind_estimate.h: unchanged
parameter_update.h: unchanged
battery_status.h: unchanged
vehicle_land_detected.h: unchanged
multirotor_motor_limits.h: unchanged
system_power.h: unchanged
offboard_control_mode.h: unchanged
mission_result.h: unchanged
position_setpoint.h: unchanged
vehicle_global_position.h: unchanged
%%%%
%%%% Building px4fmu-v2_APM in /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build/
%%%%
make[2]: Entering directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build'
%  PX4_BASE            = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/
%  GIT_DESC            = 
%  CONFIG              = px4fmu-v2_APM
%  BOARD               = px4fmu-v2
%  WORK_DIR            = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build/
%  NUTTX_EXPORT_DIR    = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build/nuttx-export/
%  NUTTX_CONFIG_HEADER = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build/nuttx-export/include/nuttx/config.h
make[3]: Entering directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot'
%% MODULE_MK           = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/module.mk
%  MODULE_NAME         = ardupilot
%  MODULE_SRC          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/
%  MODULE_OBJ          = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/module.pre.o
%  MODULE_WORK_DIR     = /home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot
make[3]: Leaving directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build/home/liujinwei/Work/APM/APM_Code/demo/ardupilot'
/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/makefiles/firmware.mk:233: recipe for target '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build//home/liujinwei/Work/APM/APM_Code/demo/ardupilot/module.pre.o' failed
make[2]: Leaving directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build'
/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Makefile.make:143: recipe for target '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot/modules/PX4Firmware/Build/px4fmu-v2_APM.build/firmware.px4' failed
make[1]: Leaving directory '/home/liujinwei/Work/APM/APM_Code/demo/ardupilot'
../mk/px4_targets.mk:99: recipe for target 'px4-v2' failed
