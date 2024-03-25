/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "new_pub_cpp.hpp"

using namespace time_literals;

new_pub_cpp::new_pub_cpp() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)

{

}

new_pub_cpp::~new_pub_cpp()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool new_pub_cpp::init()   // run only one time
{
	// execute Run() on every sensor_accel publication
	// if (!_sensor_accel_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate
	ScheduleOnInterval(1000);
	return true;
}

void new_pub_cpp::parameters_update()
{
	// Check if parameters have changed / update
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}
}

void new_pub_cpp::printintfromqgc()
{

	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;



		if (_sensor_accel_sub.copy(&accel)) {
			// DO WORK

			// access parameter value (SYS_AUTOSTART)
			if (_param_new_scale_en.get() == 1) {
				// do something if new scale en is 1
				//int test_print = _param_new_scale_en.get();
				int new_int1 = _param_new_num_int.get();


				if (_time_reset){ //true and start if
					_previous_time = hrt_absolute_time();
					_time_reset = false;
				}

				// if (hrt_elapsed_time(& _previous_time)> 2_s){  //set every n second
				// 	_time_reset = true;
				// 	//PX4_INFO("Hi scale en is %d. IT's a meeee MAAAARIOOOOOO", test_print);
				// 	PX4_INFO("new num int set to %d. ", new_int1);
				// 	PX4_INFO("so the result isssss %d. ", new_int1*10);

				// }
				if (hrt_elapsed_time(& _previous_time)> 2_s){  //set every n second
					_time_reset = true;
					int mode_int = _param_new_sw_mode.get();
					servo_info_s servo_info{};

					switch(mode_int){
					case 0:
						break;
					case 1:
						PX4_INFO("new num int set to %d. ", new_int1);
						PX4_INFO("so the result is %d and send to counter. ", new_int1*10);
						servo_info.count = new_int1*10;
						break;
					case 2:
						PX4_INFO("new num int set to %d. ", new_int1);
						PX4_INFO("so the result is %d and send to count. ", new_int1*200);
						servo_info.counter = new_int1*200;
						break;
					default:
						PX4_WARN("Error no case");
						break;
					}
					servo_info.timestamp = hrt_absolute_time();
					_servo_info_pub.publish(servo_info);

					//PX4_INFO("Hi scale en is %d. IT's a meeee MAAAARIOOOOOO", test_print);


				}
			}

		}
	}


}

void new_pub_cpp::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);
	parameters_update(); //
	// // Check if parameters have changed / update
	// if (_parameter_update_sub.updated()) {
	// 	// clear update
	// 	parameter_update_s param_update;
	// 	_parameter_update_sub.copy(&param_update);
	// 	updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	// }


	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) { // current arm compare with previou arm status
				//PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);
				int arm_status = vehicle_status.latest_arming_reason;
				switch(arm_status){
					case 1:
						PX4_WARN("vehicle armed due to rc strick");
						break;
					case 5:
						PX4_WARN("vehicle armed due to mission");
						break;
					default:
						PX4_WARN("Error no case");
						break;
				}

				// if (arm_status == 1){
				// 	PX4_WARN("vehicle armed due to rc strick");
				// }
			} else if (!armed && _armed) {
				//PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
				int disarm_status = vehicle_status.latest_disarming_reason;
				switch(disarm_status){
					case 7:
						PX4_WARN("vehicle disarmed due to auto land");
						break;
					case 1:
						PX4_WARN("vehicle disarmed due to rc strick");
						break;
					default:
						PX4_WARN("Error no case");
						break;
				}


				// if (disarm_status == 7){
				// 	PX4_WARN("vehicle disarmed due to auto land");
				// }
			}

			_armed = armed;
		}
	}

	printintfromqgc();
	// Example
	//  grab latest accelerometer data



	// Example
	//  publish some data
	// servo_info_s servo{};
	// servo.temperature[0] = 314;
	// servo.timestamp = hrt_absolute_time();
	// _servo_info_pub.publish(servo);


	perf_end(_loop_perf);

}


int new_pub_cpp::task_spawn(int argc, char *argv[])
{
	new_pub_cpp *instance = new new_pub_cpp();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int new_pub_cpp::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int new_pub_cpp::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int new_pub_cpp::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("new_pub_cpp", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int new_pub_cpp_main(int argc, char *argv[])
{
	return new_pub_cpp::main(argc, argv);
}
