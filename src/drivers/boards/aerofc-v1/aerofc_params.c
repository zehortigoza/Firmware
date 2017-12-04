/**
* Set the baudrate of the MAVLink stream in the telemetry connector
*
* If baudrate is not handled in script files no MAVLink stream
* will be started in the telemetry port.
*
* @reboot_required true
*
* @group AeroFC
*/
PARAM_DEFINE_INT32(AEROFC_TELE_BPS, 57600);
