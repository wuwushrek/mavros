/**
 * @brief mpc_full_state plugin
 * @file mpc_full_state.cpp
 * @author Franck Djeumou <frmbouwe@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2022 Franck Djeumou
 *
 */

#include <mavros/mavros_plugin.h>
#include <boost/algorithm/string.hpp>

#include <mavros_msgs/MPCFullState.h>

namespace mavros {
namespace extra_plugins {
using mavlink::common::MAV_FRAME;
using mavlink::common::MAV_ESTIMATOR_TYPE;

// Create a function to multiply quaternions
void multiply_quaternions(const float q1[4], const float q2[4], float q3[4])
{
    q3[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q3[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q3[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q3[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

// Create a function to inverse a quaternion
void inverse_quaternion(const float q1[4], float q2[4])
{
    q2[0] = q1[0];
    q2[1] = -q1[1];
    q2[2] = -q1[2];
    q2[3] = -q1[3];
}

// Create a function to rotate a vector by a quaternion
void rotate_vector(const float q[4], const float v[3], float v_rotated[3])
{
    float q_conjugate[4];
    inverse_quaternion(q, q_conjugate);
    float q_vector[4] = {0, v[0], v[1], v[2]};
    float q_result[4];
    multiply_quaternions(q, q_vector, q_result);
    multiply_quaternions(q_result, q_conjugate, q_vector);
    v_rotated[0] = q_vector[1];
    v_rotated[1] = q_vector[2];
    v_rotated[2] = q_vector[3];
}

// ENU to NED Orientation
void enu_to_ned_quaternion(const float q_enu[4], float q_ned[4])
{
    float q_enu_to_ned[4] = {0, sqrt(0.5), sqrt(0.5), 0};
    float q_flu_to_frd[4] = {0., 1., 0., 0.};
    float q_flu_to_ned[4];
    multiply_quaternions(q_enu_to_ned, q_enu, q_flu_to_ned);
    // COnjugate flu_to_frd quaternion
    float q_flu_to_frd_conjugate[4];
    inverse_quaternion(q_flu_to_frd, q_flu_to_frd_conjugate);
    multiply_quaternions(q_flu_to_ned, q_flu_to_frd_conjugate, q_ned);
}

// ENU to NED Position using quaternion
void enu_to_ned_position(const float p_enu[3], float p_ned[3])
{
    float q_enu_to_ned[4] = {0, sqrt(0.5), sqrt(0.5), 0};
    rotate_vector(q_enu_to_ned, p_enu, p_ned);
}

// FLU to FRD angular velocity
void flu_to_frd_vel(const float w_flu[3], float w_frd[3])
{
    float q_flu_to_frd[4] = {0., 1., 0., 0.};
    rotate_vector(q_flu_to_frd, w_flu, w_frd);
}

/**
 * @brief MPCFullState plugin
 *
 * Publishes mpc_full_state data that comes from FCU.
 *
 * Pose is expressed in parent frame (NED frame). (Quaternion rotates from child (BODY FRAME) to parent)
 * The twist is expressed in the child frame (BODY FRAME)
 *
 * @see handle_mpc_state()	receiving and transforming mpc_state from fcu
 */
class MPCFullStatePlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW		// XXX(vooon): added to try to fix #1223. Not sure that it is needed because class do not have Eigen:: fields.

	MPCFullStatePlugin() : PluginBase(),
		mpc_full_state_nh("~mpc_full_state")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		// publishers
		mpc_full_state_pub = mpc_full_state_nh.advertise<mavros_msgs::MPCFullState>("state", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&MPCFullStatePlugin::handle_odom)
		};
	}

private:
	ros::NodeHandle mpc_full_state_nh;			//!< node handler
	ros::Publisher mpc_full_state_pub;			//!< mavros_msgs/MPCFullState publisher

	/**
	 * @brief Handle MPC_FULL_STATE MAVlink message.

	 * @param msg	Received Mavlink msg
	 * @param mpc_msg	ODOMETRY msg
	 */
	void handle_odom(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MPC_FULL_STATE &mpc_msg)
	{
		auto mpc = boost::make_shared<mavros_msgs::MPCFullState>();

        mpc->time_usec = mpc_msg.time_usec;

		/**
		 * Position parsing to ENU
		 */
        float curr_pos[3] = {mpc_msg.x, mpc_msg.y, mpc_msg.z};
        float enu_pos[3];
        enu_to_ned_position(curr_pos, enu_pos);
        mpc->x = enu_pos[0];
        mpc->y = enu_pos[1];
        mpc->z = enu_pos[2];

        /**
		 * Velocity parsing to ENU
         */
        float curr_vel[3] = {mpc_msg.vx, mpc_msg.vy, mpc_msg.vz};
        float enu_vel[3];
        enu_to_ned_position(curr_vel, enu_vel);
        mpc->vx = enu_vel[0];
        mpc->vy = enu_vel[1];
        mpc->vz = enu_vel[2];

        /**
		 * Orientation parsing. Quaternion has to be the rotation from body ENU to parent ENU
         */
		float curr_rot[4] = {mpc_msg.qw, mpc_msg.qx, mpc_msg.qy, mpc_msg.qz};
        float enu_rot[4];
        enu_to_ned_quaternion(curr_rot, enu_rot);
        mpc->qw = enu_rot[0];
        mpc->qx = enu_rot[1];
        mpc->qy = enu_rot[2];
        mpc->qz = enu_rot[3];

		/**
		 * Angular velocities parsing
		 * angular velocities are transforned to the desired child_frame.
		 */
        float curr_ang_vel[3] = {mpc_msg.wx, mpc_msg.wy, mpc_msg.wz};
        float flu_ang_vel[3];
        flu_to_frd_vel(curr_ang_vel, flu_ang_vel);
        mpc->wx = flu_ang_vel[0];
        mpc->wy = flu_ang_vel[1];
        mpc->wz = flu_ang_vel[2];

        /**
         * Store motor values
         */
        mpc->m1 = mpc_msg.m1;
        mpc->m2 = mpc_msg.m2;
        mpc->m3 = mpc_msg.m3;
        mpc->m4 = mpc_msg.m4;
        mpc->m5 = mpc_msg.m5;
        mpc->m6 = mpc_msg.m6;

		//! Publish the data
		mpc_full_state_pub.publish(mpc);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MPCFullStatePlugin, mavros::plugin::PluginBase)
