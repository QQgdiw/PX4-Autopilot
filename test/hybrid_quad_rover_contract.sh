#!/usr/bin/env bash
set -uo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
document="$root/docs/hybrid/quad-rover-mavlink-dds-contract.md"
source_file="$root/src/modules/hybrid_vehicle_control/hybrid_vehicle_control.cpp"
dialect="$root/src/modules/mavlink/mavlink/message_definitions/v1.0/hybrid_vehicle.xml"
status_msg="$root/msg/HybridVehicleStatus.msg"
dds_topics="$root/src/modules/uxrce_dds_client/dds_topics.yaml"
failures=0

if command -v rg >/dev/null 2>&1; then
	search_quiet() { rg -q -- "$1" "$2"; }
	search_count() { rg -c -- "$1" "$2"; }
else
	search_quiet() { grep -Eq -- "$1" "$2"; }
	search_count() { grep -Ec -- "$1" "$2"; }
fi

check_file() {
	local path="$1"
	local description="$2"
	if [[ ! -f "$path" ]]; then
		printf 'FAIL: %s (%s)\n' "$description" "$path" >&2
		failures=$((failures + 1))
	fi
}

check_pattern() {
	local pattern="$1"
	local path="$2"
	local description="$3"
	if [[ ! -f "$path" ]] || ! search_quiet "$pattern" "$path"; then
		printf 'FAIL: %s\n' "$description" >&2
		failures=$((failures + 1))
	fi
}

check_file "$document" 'hybrid integration contract exists'

# Wire/source invariants: these fail if the implementation drifts away from the contract.
check_pattern '<entry value="200" name="MAV_TYPE_QUAD_ROVER">' "$dialect" 'private vehicle type remains 200'
check_pattern '<entry value="50000" name="MAV_CMD_DO_HYBRID_TRANSITION"' "$dialect" 'private transition command remains 50000'
check_pattern '<message id="60000" name="HYBRID_VEHICLE_STATUS">' "$dialect" 'private status message remains 60000'
check_pattern 'uint64 command_timestamp' "$status_msg" 'uORB status retains command correlation timestamp'
check_pattern 'uint32 transition_sequence' "$status_msg" 'uORB status retains transition sequence'
check_pattern 'topic: /fmu/in/rover_velocity_setpoint' "$dds_topics" 'DDS rover input topic remains exported'
check_pattern 'ack.result_param2 = _transition_sequence' "$source_file" 'hybrid ACK carries transition sequence'

ack_publishers="$(search_count '_vehicle_command_ack_pub\.publish\(ack\)' "$source_file" || true)"
if [[ "$ack_publishers" != "1" ]]; then
	printf 'FAIL: hybrid controller must have exactly one ACK publication site (found %s)\n' "$ack_publishers" >&2
	failures=$((failures + 1))
fi

# External documentation invariants: deliberately explicit so clients can rely on them.
check_pattern 'Dialect:.*hybrid_vehicle' "$document" 'document names the private dialect'
check_pattern 'MAV_TYPE_QUAD_ROVER.*200' "$document" 'document states the private vehicle type and value'
check_pattern 'MAV_CMD_DO_HYBRID_TRANSITION.*50000' "$document" 'document states the command and value'
check_pattern 'param1.*1.*Quad.*2.*Rover' "$document" 'document defines transition param1 targets'
check_pattern 'param2.*param7.*reserved' "$document" 'document defines reserved command parameters'
check_pattern 'MAVLink 2' "$document" 'document requires MAVLink 2'
check_pattern 'result_param2.*transition_sequence' "$document" 'document defines ACK sequence correlation'
check_pattern 'command_timestamp.*vehicle_command\.timestamp' "$document" 'document defines status command correlation'
check_pattern 'transition_elapsed_ms.*floor.*UINT32_MAX' "$document" 'document defines elapsed conversion and saturation'
check_pattern '/fmu/in/rover_velocity_setpoint' "$document" 'document names the Rover DDS input topic'
check_pattern 'rover_velocity.*true.*RoverVelocitySetpoint' "$document" 'document defines Rover Offboard pairing'
check_pattern 'speed_body_x.*linear\.x' "$document" 'document defines ROS forward-speed mapping'
check_pattern 'yaw_rate.*-.*angular\.z' "$document" 'document defines ROS/PX4 yaw sign mapping'
check_pattern 'EXTENDED_SYS_STATE.*MAV_VTOL_STATE_UNDEFINED' "$document" 'document defines the VTOL-state boundary'
check_pattern 'physical validation' "$document" 'document identifies remaining physical validation'

if (( failures != 0 )); then
	printf 'hybrid contract: %d check(s) failed\n' "$failures" >&2
	exit 1
fi

printf 'hybrid contract: all checks passed\n'
