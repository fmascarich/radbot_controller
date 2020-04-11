#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <radbot_serial_interface/motor_command.h>
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <random>
#include <jsk_rviz_plugins/OverlayText.h>
#include <boost/circular_buffer.hpp>
#include "radbot_serial.hpp"


#define M_NAN	std::numeric_limits<double>::quiet_NaN()

class radbot_pid
{
	public:
		radbot_pid()
		{

		}
		void set_params(double kp, double ki, double kd, double output_limit, double sampling_time,
						double not_moving_offset, double not_moving_gain, double not_moving_sum_flag, double not_moving_limit)
		{
			m_kp = kp;
			m_ki = ki;
			m_kd = kd;
			m_output_limit = output_limit;
			m_sampling_time = sampling_time;
			m_not_moving_offset = not_moving_offset;
			m_not_moving_gain = not_moving_gain;
			m_not_moving_sum_flag = not_moving_sum_flag;
			m_not_moving_limit = not_moving_limit;
		}
		void reset()
		{
			m_integral = 0;
			m_derivative = 0;
			m_previous_error = 0;
			m_previous_derivative = 0.0;
		}
		double update(double error, bool is_moving)
		{
			// kill the integral if the error is small
			if(abs(error) < 0.05)
			{
				m_integral = 0.0;
			}
			// calculate the integral component
			m_integral += (error * m_sampling_time);
			// calculate integral windup limit as 1/3 of the output limit
			double integral_limit = m_output_limit/3.0;
			// check for integral saturation
			if(abs(m_integral * m_ki) > integral_limit)
			{
				// if positive saturation
				if((m_integral * m_ki) > integral_limit)
				{
					m_integral = integral_limit / m_ki;
				}
				// if negative saturation
				else
				{
					m_integral = integral_limit * -1.0 / m_ki;
				}
			}
			// calculate derivative
			m_derivative = (error - m_previous_error) / m_sampling_time;
			// low filter derivative
			m_derivative = (0.25 * m_derivative) + (0.75 * m_previous_derivative);
			// save current for next iter
			m_previous_derivative = m_derivative;
			m_previous_error = error;
			// calculate output
			double output = (m_kp * error) + (m_ki * m_integral) + (m_kd * m_derivative);
			// clamp output to limits
			if(output > m_output_limit)
			{
				output = m_output_limit;
			}
			else if(output < (m_output_limit * -1.0))
			{
				output = -1.0 * m_output_limit;
			}
			if(!is_moving)
			{
				output *= m_not_moving_gain;
				output += m_not_moving_offset;
				if(m_not_moving_sum_flag)
				{
					output += m_previous_output;
				}
				if(output < 0 && output < m_not_moving_limit * -1.0)
				{
					output = m_not_moving_limit;
				}
				else if(output > 0 && output > m_not_moving_limit)
				{
					output = m_not_moving_limit;
				}
			}
			m_previous_output = output;
			return output;
		}

	private:
		double m_kp, m_ki, m_kd;
		double m_sampling_time;
		double m_output_limit;
		double m_integral = 0.0;
		double m_derivative = 0.0;
		double m_previous_error = 0.0;
		double m_previous_derivative = 0.0;
		double m_not_moving_offset = 0.0;
		double m_not_moving_gain = 1.0;
		double m_not_moving_limit = 1.0;
		bool m_not_moving_sum_flag = false;
		double m_previous_output = 0;
};

class radbot_controller
{

	public:

		radbot_controller(ros::NodeHandlePtr nh_ptr)
		{
			m_controller_update_freq = 50.0;
			nh_ptr->getParam("controller_update_freq", m_controller_update_freq);
			m_steering_angle_threshold = 0.2;
			nh_ptr->getParam("steering_angle_threshold", m_steering_angle_threshold);

			double controller_update_period = 1.0/m_controller_update_freq;

			double steering_kp = 1.0;
			nh_ptr->getParam("steering_kp", steering_kp);
			double steering_ki = 1.0;
			nh_ptr->getParam("steering_ki", steering_ki);
			double steering_kd = 1.0;
			nh_ptr->getParam("steering_kd", steering_kd);
			double steering_out_limit = 1.0;
			nh_ptr->getParam("steering_out_limit", steering_out_limit);
			double steering_not_moving_offset = 0.0;
			nh_ptr->getParam("steering_not_moving_offset", steering_not_moving_offset);
			double steering_not_moving_gain = 1.0;
			nh_ptr->getParam("steering_not_moving_gain", steering_not_moving_gain);
			bool steering_not_moving_sum_flag = false;
			nh_ptr->getParam("steering_not_moving_sum_flag", steering_not_moving_sum_flag);
			double steering_not_moving_limit = 1.0;
			nh_ptr->getParam("steering_not_moving_limit", steering_not_moving_limit);
			m_is_turning_thresh = 0.15;
			nh_ptr->getParam("is_turning_thresh", m_is_turning_thresh);

			double position_kp = 1.0;
			nh_ptr->getParam("position_kp", position_kp);
			double position_ki = 1.0;
			nh_ptr->getParam("position_ki", position_ki);
			double position_kd = 1.0;
			nh_ptr->getParam("position_kd", position_kd);
			double position_out_limit = 1.0;
			nh_ptr->getParam("position_out_limit", position_out_limit);
			double position_not_moving_offset = 0.0;
			nh_ptr->getParam("position_not_moving_offset", position_not_moving_offset);
			double position_not_moving_gain = 1.0;
			nh_ptr->getParam("position_not_moving_gain", position_not_moving_gain);
			bool position_not_moving_sum_flag = false;
			nh_ptr->getParam("position_not_moving_sum_flag", position_not_moving_sum_flag);
			double position_not_moving_limit = 1.0;
			nh_ptr->getParam("position_not_moving_limit", position_not_moving_limit);
			m_is_moving_thresh = 0.02;
			nh_ptr->getParam("is_moving_thresh", m_is_moving_thresh);

			std::string serial_port_name = "/dev/ttyUSB0";
			nh_ptr->getParam("serial_port_name", serial_port_name);


			loam_frame_id = "rover_body";
			nh_ptr->getParam("loam_frame_id", loam_frame_id);


			m_radbot_serial = new radbot_serial(serial_port_name);

			m_steering_pid.set_params(steering_kp, steering_ki, steering_kd, steering_out_limit, controller_update_period, 
						steering_not_moving_offset, steering_not_moving_gain, steering_not_moving_sum_flag, steering_not_moving_limit);
			m_steering_pid.reset();

			m_position_pid.set_params(position_kp, position_ki, position_kd, position_out_limit, controller_update_period,
						position_not_moving_offset, position_not_moving_gain, position_not_moving_sum_flag, position_not_moving_limit);
			m_position_pid.reset();

			imu_rotational_vels.set_capacity(10);
			imu_linear_vels.set_capacity(10);

			m_imu_sub 				= nh_ptr->subscribe("/vn100/comp_imu", 10, &radbot_controller::imu_cb, this);
			m_imu_twist_sub 		= nh_ptr->subscribe("/vn100/deltaThetaVel", 10, &radbot_controller::imu_twist_cb, this);
			target_position_sub 	= nh_ptr->subscribe("/rover_controller/move_target", 10, &radbot_controller::target_pos_cb, this);
			current_pose_sub 		= nh_ptr->subscribe("/aft_mapped_to_init_CORRECTED", 10, &radbot_controller::loam_odom_cb, this);
			
			m_command_overlay_text_pub	= nh_ptr->advertise<jsk_rviz_plugins::OverlayText>("/radbot/wheel_commands_overlay", 10, true);
			m_error_overlay_text_pub	= nh_ptr->advertise<jsk_rviz_plugins::OverlayText>("/radbot/error_overlay", 10, true);

			m_controller_update_timer 	= nh_ptr->createTimer(ros::Duration(controller_update_period), &radbot_controller::controller_update_timer_cb, this);
			m_radbot_serial_read_timer 	= nh_ptr->createTimer(ros::Duration(0.1), &radbot_controller::check_serial_port, this);
			set_wheel_speeds(0.0,0.0,0.0,0.0);
		}
		void imu_twist_cb(const geometry_msgs::TwistStamped::ConstPtr msg)
		{
			double current_angular_z_velocity = msg->twist.angular.z;
			double current_linear_velocity = msg->twist.linear.x;
			imu_rotational_vels.push_back(current_angular_z_velocity);
			imu_linear_vels.push_back(current_linear_velocity);
			double imu_rotational_vels_sum = 0.0;
			for(auto it : imu_rotational_vels)
			{
				imu_rotational_vels_sum += it;
			}
			double imu_linear_vels_sum = 0.0;
			for(auto it : imu_linear_vels)
			{
				imu_linear_vels_sum += it;
			}
			m_is_moving_flag = imu_linear_vels_sum > m_is_moving_thresh*imu_linear_vels.size();
			m_is_turning_flag = imu_rotational_vels_sum > m_is_turning_thresh*imu_linear_vels.size();
		}
		void imu_cb(const sensor_msgs::Imu::ConstPtr msg)
		{
			tf::Quaternion previous_imu_quat = current_imu_quat;
			current_imu_quat.setX(msg->orientation.x);
			current_imu_quat.setY(msg->orientation.y);
			current_imu_quat.setZ(msg->orientation.z);
			current_imu_quat.setW(msg->orientation.w);
			if(imu_offset_valid)
			{
				tf::Quaternion estimated_quaternion = loam_to_imu_offset * current_imu_quat;
				estimated_quaternion.normalize();
				double current_heading = tf::getYaw(estimated_quaternion);
				if(target_valid)
				{
					tf::Vector3 vector_to_target = target_pose.getOrigin() -  current_loam_pose.getOrigin();
					double yaw_to_target = atan2(vector_to_target[1],vector_to_target[0]);
					if(!std::isnan(yaw_to_target))
					{
						tf::Quaternion quat_from_world_to_target;
						quat_from_world_to_target.setRPY(0.0,0.0,yaw_to_target);
						tf::Quaternion rot_from_current_to_target = quat_from_world_to_target * estimated_quaternion.inverse();
						m_current_heading_error = tf::getYaw(rot_from_current_to_target);
						m_current_position_error = vector_to_target.length();
					}
					else
					{
						m_current_heading_error = 0.0;
						m_current_position_error = 0.0;
					}
				}
			}
		}
		void controller_update_timer_cb(const ros::TimerEvent&)
		{
			if(std::isnan(m_current_heading_error) || std::isnan(m_current_position_error))
			{
				ROS_WARN_THROTTLE(1.0, "CONTROLLER HAS NaNs, stopping motors");
				set_wheel_speeds(0.0,0.0,0.0,0.0);
				return;
			}
			double steering_output;
			double position_output;
			jsk_rviz_plugins::OverlayText msg;
			msg.text = "Heading E : " + std::to_string(m_current_heading_error);
			msg.text += "\nPos E: " + std::to_string(m_current_position_error);

			double left_speed;
			double right_speed;
			if(m_current_heading_error > m_steering_angle_threshold || m_current_heading_error < (-1.0 * m_steering_angle_threshold))
			{
				steering_output = m_steering_pid.update(m_current_heading_error, m_is_turning_flag);
				position_output = m_position_pid.update(0.0, m_is_moving_flag);
				left_speed = steering_output * -1.0;
				right_speed = steering_output;
				msg.text += "\n Turning";
			}
			else
			{
				steering_output = m_steering_pid.update(0.0, m_is_turning_flag);
				position_output = m_position_pid.update(m_current_position_error, m_is_moving_flag);
				left_speed = position_output;
				right_speed = position_output;
				msg.text += "\n Straight";
			}
			set_wheel_speeds(right_speed, left_speed, right_speed, left_speed);
			ROS_INFO("LEFT : %f, Right : %f", left_speed, right_speed);
			m_error_overlay_text_pub.publish(msg);
		}
		void set_wheel_speeds(double FR, double FL, double RR, double RL)
		{
			
			std::string command = "<" + std::to_string(int(FR)) + "," + 
										std::to_string(int(RR)) + "," + 
										std::to_string(int(FL)) + "," + 
										std::to_string(int(RL)) + ">";
			m_radbot_serial->write_port(command);
			//ROS_INFO("Wrote to serial port : %s", command.c_str());
			jsk_rviz_plugins::OverlayText msg;
			msg.text = command;
			m_command_overlay_text_pub.publish(msg);
		}
		void check_serial_port(const ros::TimerEvent&)
		{
			std::string cmd = m_radbot_serial->get_command();
			if(cmd != "")
			{
				std::size_t start_char_pos = cmd.find("$");
				std::size_t end_char_pos = cmd.find("#");
				if(start_char_pos != std::string::npos && end_char_pos != std::string::npos)
				{
					std::vector<int> result;
					cmd = cmd.substr(start_char_pos+1, end_char_pos-start_char_pos);
					std::stringstream s_stream(cmd);
					while(s_stream.good()) 
					{
						std::string substr;
						std::getline(s_stream, substr, ',');
						result.push_back(std::stoi(substr));
					}
					if(result.size() == 5)
					{
						if(result[0] == 1)
						{
							ROS_ERROR("Radbot E-Stop Button has been pressed. Motors are disabled. Restart microcontroller to clear condition.");
						}
						if(result[1] == 1)
						{
							ROS_ERROR("Radbot R/C E-Stop has been detected, check R/C F-Switch");
						}
						if(result[2] == 1)
						{
							ROS_ERROR("Radbot XBee E-Stop has been detected, consult DARPA Officials");
						}
						if(result[3] == 0)
						{
							ROS_WARN("Manual R/C Mode Detected, check R/C C-Switch");
						}
						if(result[4] == 0)
						{
							ROS_WARN("No R/C Detected");
						}
					}
					//ROS_INFO("GOT FROM PORT PARAMS : %i,%i,%i,%i,%i",result[0],result[1],result[2],result[3],result[4]);
				}

			}
		}
		void loam_odom_cb(const nav_msgs::Odometry::ConstPtr msg)
		{
			tf::StampedTransform transform;
			try
			{
				listener.lookupTransform("world", loam_frame_id,ros::Time(0), transform);
			}
			catch(...)
			{
				return;
			}
			// set the loam pose from the tranform
			current_loam_pose.setOrigin(transform.getOrigin());
			current_loam_pose.setRotation(transform.getRotation());
					
			// calculate the loam to IMU rotation
			loam_to_imu_offset = transform.getRotation() * current_imu_quat.inverse();
			// set the imu offset flag to true
			imu_offset_valid = true;
		}
		void target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr msg)
		{
			
			tf::Vector3 pos;
			pos.setX(msg->pose.position.x);
			pos.setY(msg->pose.position.y);
			pos.setZ(msg->pose.position.z);
			target_pose.setOrigin(pos);
			tf::Quaternion target_quat;
			target_quat.setX(msg->pose.orientation.x);
			target_quat.setY(msg->pose.orientation.y);
			target_quat.setZ(msg->pose.orientation.z);
			target_quat.setW(msg->pose.orientation.w);
			target_pose.setRotation(target_quat);
			target_valid = true;
		}
	private:
		bool target_valid;
		bool imu_offset_valid;
		std::string loam_frame_id;

		tf::Quaternion current_imu_quat;
		tf::Quaternion loam_to_imu_offset;

		tf::Pose target_pose;
		tf::Pose current_loam_pose;
		double m_controller_update_freq;
		double m_steering_angle_threshold;
		double m_current_position_error = M_NAN;
		double m_current_heading_error = M_NAN;
		double m_is_turning_thresh = 0.15;
		double m_is_moving_thresh = 0.02;
		bool m_is_turning_flag = false;
		bool m_is_moving_flag = false;

		ros::Timer m_controller_update_timer;
		ros::Timer m_radbot_serial_read_timer;

		ros::Publisher m_command_overlay_text_pub;
		ros::Publisher m_error_overlay_text_pub;

		ros::Subscriber m_imu_sub;
		ros::Subscriber m_imu_twist_sub;
		ros::Subscriber target_position_sub;
		ros::Subscriber current_pose_sub;
		tf::TransformListener listener;

		radbot_pid m_steering_pid;
		radbot_pid m_position_pid;

		boost::circular_buffer<double> imu_rotational_vels;
		boost::circular_buffer<double> imu_linear_vels;

		radbot_serial *m_radbot_serial;

};