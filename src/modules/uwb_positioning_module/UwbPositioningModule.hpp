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

/**
 * @file UwbPositioningModule.hpp
 *
 * Computes a vehicle position from single-anchor UWB range and angle-of-arrival
 * measurements and publishes it as external vision odometry for EKF2, allowing
 * GPS to be turned off indoors.
 *
 * The UWB anchor is the origin of the NED frame: it sits on the ground with its
 * antenna facing up, and the vehicle carries the other antenna facing down.
 *
 * The angles reported by the sensor are expressed in the sensor frame, which is
 * rigidly attached to the airframe. A pure vehicle rotation therefore changes
 * the reported angles even though the vehicle has not moved, so the measurement
 * is rotated by the current vehicle attitude before it is used as a position.
 * That coordinate transformation is what separates tilt from translation.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_uwb.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_odometry.h>

using namespace time_literals;

namespace uwb_positioning_module
{

class UwbPositioningModule : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	UwbPositioningModule();
	~UwbPositioningModule() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;

	void parameters_update();

	/**
	 * Convert one UWB sample into a vehicle position in the NED frame.
	 *
	 * @param uwb      raw UWB sample (distance and angles of arrival)
	 * @param q_nb     attitude quaternion, body FRD to NED
	 * @param pos_ned  [out] vehicle position in NED, anchor at the origin
	 * @return true if the sample produced a finite position
	 */
	bool computePosition(const sensor_uwb_s &uwb, const matrix::Quatf &q_nb,
			     matrix::Vector3f &pos_ned);

	void publishOdometry(const sensor_uwb_s &uwb, const matrix::Vector3f &pos_ned);

	/** Attitude older than this cannot be used to interpret the angles. */
	static constexpr hrt_abstime ATTITUDE_TIMEOUT_US = 100_ms;

	uORB::SubscriptionCallbackWorkItem _sensor_uwb_sub{this, ORB_ID(sensor_uwb)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<vehicle_odometry_s> _visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};

	// Counters reported by print_status()
	uint32_t _sample_count{0};
	uint32_t _publish_count{0};
	uint32_t _reject_disabled{0};
	uint32_t _reject_nonfinite{0};
	uint32_t _reject_status{0};
	uint32_t _reject_nlos{0};
	uint32_t _reject_range{0};
	uint32_t _reject_attitude{0};

	matrix::Vector3f _last_pos_ned{};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UWB_POS_EN>)      _param_uwb_pos_en,
		(ParamFloat<px4::params::UWB_POS_NOISE>) _param_uwb_pos_noise,
		(ParamFloat<px4::params::UWB_POS_R_MIN>) _param_uwb_pos_r_min,
		(ParamFloat<px4::params::UWB_POS_R_MAX>) _param_uwb_pos_r_max
	)
};

} // namespace uwb_positioning_module
