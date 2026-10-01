/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "UwbPositioningModule.hpp"

using namespace matrix;

namespace uwb_positioning_module
{

ModuleBase::Descriptor UwbPositioningModule::desc{task_spawn, custom_command, print_usage};

UwbPositioningModule::UwbPositioningModule() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_visual_odometry_pub.advertise();
}

UwbPositioningModule::~UwbPositioningModule()
{
	perf_free(_cycle_perf);
}

int UwbPositioningModule::task_spawn(int argc, char *argv[])
{
	UwbPositioningModule *obj = new UwbPositioningModule();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	desc.object.store(obj);
	desc.task_id = task_id_is_work_queue;

	if (!obj->init()) {
		delete obj;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

bool UwbPositioningModule::init()
{
	// Run once per UWB measurement rather than polling on a timer.
	if (!_sensor_uwb_sub.registerCallback()) {
		PX4_ERR("sensor_uwb callback registration failed");
		return false;
	}

	return true;
}

void UwbPositioningModule::parameters_update()
{
	updateParams();
}

bool UwbPositioningModule::computePosition(const sensor_uwb_s &uwb, const Quatf &q_nb,
		Vector3f &pos_ned)
{
	const float d  = uwb.distance;
	const float az = math::radians(uwb.aoa_azimuth_dev);
	const float el = math::radians(uwb.aoa_elevation_dev);

	// Spherical to Cartesian in the sensor frame.
	//   N = -d * cos(elevation)
	//   E =  d * sin(azimuth)
	//   Z = -d * cos(azimuth) * sin(elevation)
	const Vector3f v_sensor(-d * cosf(az) * sinf(el),
				 d * sinf(az),
				-d * cosf(el));

	// Add the lever arm from the vehicle centre of gravity to the UWB antenna.
	// Both are expressed in body axes, so this is a plain vector sum.
	const Vector3f offset_body(uwb.offset_x, uwb.offset_y, uwb.offset_z);
	const Vector3f v_body = v_sensor + offset_body;

	// Rotate body FRD into NED. This is the coordinate transformation that
	// separates vehicle tilt from translation: when the vehicle pitches or
	// rolls in place the measured angles change, but the attitude changes by
	// the same amount, so the resulting NED vector is unchanged.
	const Dcmf R_nb(q_nb);
	const Vector3f v_ned = R_nb * v_body;

	// The anchor is the NED origin, so the vector from the anchor to the
	// vehicle is the measured vector itself.
	pos_ned = v_ned;

	return pos_ned.isAllFinite();
}

void UwbPositioningModule::publishOdometry(const sensor_uwb_s &uwb, const Vector3f &pos_ned)
{
	vehicle_odometry_s odom{};

	odom.timestamp_sample = uwb.timestamp;

	// The anchor defines the origin of a North-East-Down frame aligned with
	// the one the estimator uses.
	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;

	pos_ned.copyTo(odom.position);

	const float pos_var = _param_uwb_pos_noise.get() * _param_uwb_pos_noise.get();
	odom.position_variance[0] = pos_var;
	odom.position_variance[1] = pos_var;
	odom.position_variance[2] = pos_var;

	// This sensor measures position only. Leaving the other fields at NaN
	// makes EKF2 skip them, since it gates each one on finiteness.
	odom.q[0] = NAN;
	odom.q[1] = NAN;
	odom.q[2] = NAN;
	odom.q[3] = NAN;
	odom.orientation_variance[0] = NAN;
	odom.orientation_variance[1] = NAN;
	odom.orientation_variance[2] = NAN;

	odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_UNKNOWN;
	odom.velocity[0] = NAN;
	odom.velocity[1] = NAN;
	odom.velocity[2] = NAN;
	odom.velocity_variance[0] = NAN;
	odom.velocity_variance[1] = NAN;
	odom.velocity_variance[2] = NAN;
	odom.angular_velocity[0] = NAN;
	odom.angular_velocity[1] = NAN;
	odom.angular_velocity[2] = NAN;

	odom.reset_counter = 0;
	odom.quality = 0;

	odom.timestamp = hrt_absolute_time();

	_visual_odometry_pub.publish(odom);

	_publish_count++;
	_last_pos_ned = pos_ned;
}

void UwbPositioningModule::Run()
{
	if (should_exit()) {
		_sensor_uwb_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		parameters_update();
	}

	sensor_uwb_s uwb;

	while (_sensor_uwb_sub.update(&uwb)) {
		_sample_count++;

		if (_param_uwb_pos_en.get() == 0) {
			_reject_disabled++;
			continue;
		}

		if (!PX4_ISFINITE(uwb.distance)
		    || !PX4_ISFINITE(uwb.aoa_azimuth_dev)
		    || !PX4_ISFINITE(uwb.aoa_elevation_dev)) {
			_reject_nonfinite++;
			continue;
		}

		// Ranging status reported by the UWB module; 0 means success.
		if (uwb.status != 0) {
			_reject_status++;
			continue;
		}

		// Non-line-of-sight: the range is likely a reflection.
		if (uwb.nlos != 0) {
			_reject_nlos++;
			continue;
		}

		if (uwb.distance < _param_uwb_pos_r_min.get()
		    || uwb.distance > _param_uwb_pos_r_max.get()) {
			_reject_range++;
			continue;
		}

		// The angles are meaningless without knowing how the vehicle is
		// oriented, so a recent attitude estimate is required.
		vehicle_attitude_s att;

		if (!_vehicle_attitude_sub.copy(&att)) {
			_reject_attitude++;
			continue;
		}

		if (hrt_elapsed_time(&att.timestamp) > ATTITUDE_TIMEOUT_US) {
			_reject_attitude++;
			continue;
		}

		const Quatf q_nb(att.q);

		if (!q_nb.isAllFinite()) {
			_reject_attitude++;
			continue;
		}

		Vector3f pos_ned{};

		if (!computePosition(uwb, q_nb, pos_ned)) {
			_reject_nonfinite++;
			continue;
		}

		publishOdometry(uwb, pos_ned);
	}

	perf_end(_cycle_perf);
}

int UwbPositioningModule::print_status()
{
	PX4_INFO("enabled: %s", _param_uwb_pos_en.get() ? "yes" : "no");
	PX4_INFO("samples: %u  published: %u", (unsigned)_sample_count, (unsigned)_publish_count);
	PX4_INFO("rejected: disabled %u  nonfinite %u  status %u  nlos %u  range %u  attitude %u",
		 (unsigned)_reject_disabled, (unsigned)_reject_nonfinite, (unsigned)_reject_status,
		 (unsigned)_reject_nlos, (unsigned)_reject_range, (unsigned)_reject_attitude);

	if (_publish_count > 0) {
		PX4_INFO("last position NED: [%.3f %.3f %.3f] m",
			 (double)_last_pos_ned(0), (double)_last_pos_ned(1), (double)_last_pos_ned(2));
	}

	perf_print_counter(_cycle_perf);

	return 0;
}

int UwbPositioningModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Single-anchor UWB positioning module.

Takes the distance and angle-of-arrival measurements from `sensor_uwb`, converts
them into a vehicle position in the NED frame, and publishes the result on
`vehicle_visual_odometry` so that EKF2 can fuse it as an external vision
position. This allows GPS to be disabled for indoor flight.

The UWB anchor is the origin of the NED frame, with its antenna facing up from
the ground and the vehicle antenna facing down.

The module also subscribes to `vehicle_attitude` and rotates each measurement
from the body frame into NED. This coordinate transformation is what allows a
vehicle tilt to be distinguished from an actual translation, since tilting in
place changes the reported angles without changing the vehicle position.

To use it, enable external vision position fusion and disable GPS:
	param set EKF2_EV_CTRL 3
	param set EKF2_HGT_REF 3
	param set EKF2_GPS_CTRL 0
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uwb_positioning_module", "estimator");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

} // namespace uwb_positioning_module

extern "C" __EXPORT int uwb_positioning_module_main(int argc, char *argv[])
{
	return ModuleBase::main(uwb_positioning_module::UwbPositioningModule::desc, argc, argv);
}
