/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/test_new.h>
#include <uORB/topics/servo_info.h>
#include <drivers/drv_hrt.h>

__EXPORT int new_subpub_main(int argc, char *argv[]);  //give function (main) appear inside help in px4

int new_subpub_main(int argc, char *argv[])
{
	PX4_INFO("Hellooooooo It's a meeeeeeeee MAAAAARIOOOO");

	/* subscribe to vehicle_acceleration topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_baro));
	int sensor_sub_fd2 = orb_subscribe(ORB_ID(sensor_gyro));
	int sensor_sub_fd3 = orb_subscribe(ORB_ID(vehicle_local_position));

	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);  //ms
	orb_set_interval(sensor_sub_fd2, 200);
	orb_set_interval(sensor_sub_fd3, 200);

	/* advertise attitude topic */
	struct debug_vect_s vec;
	memset(&vec, 0, sizeof(vec)); //clear memory
	orb_advert_t vect_pub = orb_advertise(ORB_ID(debug_vect), &vec);

	struct test_new_s alpha;
	memset(&alpha, 0, sizeof(alpha)); //clear memory
	orb_advert_t alpha_pub = orb_advertise(ORB_ID(test_new), &alpha);

	struct servo_info_s servo;
	memset(&servo, 0, sizeof(servo)); //clear memory
	orb_advert_t servo_pub = orb_advertise(ORB_ID(servo_info), &servo);

	// struct debug_value_s alpha;
	// memset(&alpha, 0, sizeof(alpha)); //clear memory
	// orb_advert_t alpha_pub = orb_advertise(ORB_ID(debug_value), &alpha);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = sensor_sub_fd2,   .events = POLLIN },
		{ .fd = sensor_sub_fd3,   .events = POLLIN }
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;
	//uint64_t timestamp_us = hrt_absolute_time()

	for (int i = 0; i < 100; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000); //monitor data every one second

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {  //poll_ret > 0

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_baro_s baro;
				struct sensor_gyro_s gyro;
				struct vehicle_local_position_s vlp;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_baro), sensor_sub_fd, &baro);
				orb_copy(ORB_ID(sensor_gyro), sensor_sub_fd2, &gyro);
				orb_copy(ORB_ID(vehicle_local_position), sensor_sub_fd3, &vlp);
				PX4_INFO("Baro: P:\t%4.4f T:\t%4.4f\nGyro:\t%8.4f\t%8.4f\t%8.4f",
					 (double)baro.pressure,
					 (double)baro.temperature,
					 (double)gyro.x,
					 (double)gyro.y,
					 (double)gyro.z);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
				vec.timestamp = hrt_absolute_time();
				strncpy(vec.name, "dbg_lopo", 10);
				// vec.x = vlp.x;
				// vec.y = vlp.y;
				// vec.z = vlp.z;
				vec.x = 1000+i;
				vec.y = 100+i;
				vec.z = 200+i;

				strncpy(alpha.name, "dbg_gyro", 10);
				alpha.a = gyro.x;
				alpha.b = gyro.y;
				alpha.c = gyro.z;

				servo.timestamp = hrt_absolute_time();
				servo.count = 1+i;
				servo.counter = 2+i;
				servo.connectiontype = SERVO_INFO_ESC_CONNECTION_TYPE_CAN;
				servo.error_count[0] = 2000+i;
				servo.temperature[0] = 1000+i;



				orb_publish(ORB_ID(debug_vect), vect_pub, &vec);
				orb_publish(ORB_ID(test_new), alpha_pub, &alpha);
				orb_publish(ORB_ID(servo_info), servo_pub, &servo);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("Bye have a beautiful time");

	return 0;
}
