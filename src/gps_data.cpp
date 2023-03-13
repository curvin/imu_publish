#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

//-----------------------------------
#include "ros/ros.h"

#include <sstream>

#include <sensor_msgs/Imu.h>
#include <iostream>

//------------------------------------

using namespace std;

#define PI (3.1415926535897932346f)
#define RADIANS_TO_DEGREES (180.0 / M_PI)

int an_packet_transmit(an_packet_t *an_packet)
{
	an_packet_encode(an_packet);
	return SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

float Latitude;
float Longitude;
float Height;

/*
 * This is an example of sending a configuration packet to Spatial.
 *
 * 1. First declare the structure for the packet, in this case sensor_ranges_packet_t.
 * 2. Set all the fields of the packet structure
 * 3. Encode the packet structure into an an_packet_t using the appropriate helper function
 * 4. Send the packet
 * 5. Free the packet
 */

void set_sensor_ranges()
{
	an_packet_t *an_packet;
	sensor_ranges_packet_t sensor_ranges_packet;

	sensor_ranges_packet.permanent = TRUE;
	sensor_ranges_packet.accelerometers_range = accelerometer_range_4g;
	sensor_ranges_packet.gyroscopes_range = gyroscope_range_500dps;
	sensor_ranges_packet.magnetometers_range = magnetometer_range_2g;

	an_packet = encode_sensor_ranges_packet(&sensor_ranges_packet);

	an_packet_transmit(an_packet);

	an_packet_free(&an_packet);
}

int main(int argc, char **argv)
{
	//*******************************************************************************************//
	an_decoder_t an_decoder;
	an_packet_t *an_packet;

	system_state_packet_t system_state_packet;
	raw_sensors_packet_t raw_sensors_packet;

	int bytes_received;

	/* open the com port */
	if (OpenComport("/dev/ttyUSB0", 115200))
	{
		printf("Could not open serial port\n");
		exit(EXIT_FAILURE);
	}
	//-----------------------------------------------------------------------------
	an_decoder_initialise(&an_decoder);

	//*********************************************************************************************************//
	ros::init(argc, argv, "gps");

	ros::NodeHandle n;
	ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("gps_data", 20);
	ros::Rate loop_rate(10);
	sensor_msgs::Imu imu_data;
	while (ros::ok())
	{

		//------------------------------------------------------------------------------------------------
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{

			imu_data.header.stamp = ros::Time::now();
			imu_data.header.frame_id = "base_link";

			an_decoder_increment(&an_decoder, bytes_received);

			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				if (an_packet->id == packet_id_system_state) /* system state packet */
				{
					if (decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{

						Latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
						Longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
						Height = system_state_packet.height * RADIANS_TO_DEGREES;

						// IMU 三轴角度
						imu_data.orientation.x = system_state_packet.orientation[0] * RADIANS_TO_DEGREES;
						imu_data.orientation.y = system_state_packet.orientation[1] * RADIANS_TO_DEGREES;
						imu_data.orientation.z = system_state_packet.orientation[2] * RADIANS_TO_DEGREES;
					}
				}
				else if (an_packet->id == packet_id_raw_sensors) /* raw sensors packet */
				{
					if (decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
					{
						// IMU 三轴加速度
						imu_data.linear_acceleration.x = raw_sensors_packet.accelerometers[0];
						imu_data.linear_acceleration.y = raw_sensors_packet.accelerometers[1];
						imu_data.linear_acceleration.z = raw_sensors_packet.accelerometers[2];

						// IMU 三轴角速度
						imu_data.angular_velocity.x = raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES;
						imu_data.angular_velocity.y = raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES;
						imu_data.angular_velocity.z = raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES;
					}
				}
				else
				{
					ROS_INFO("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
				}

				ROS_INFO("Latitude=%f", Latitude);
				ROS_INFO("Longitude=%f", Longitude);
				ROS_INFO("Height=%f", Height);
				IMU_pub.publish(imu_data);

				/* Ensure that you free the an_packet when your done with it or you will leak memory */
				an_packet_free(&an_packet);
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
