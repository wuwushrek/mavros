/**
 * @brief mpc_motors_cmd plugin
 * @file mpc_motors_cmd.cpp
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

#include <mavros_msgs/MPCMotorsCMD.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief MPCMotorsCMD plugin
 *
 * Publishes motors command to the FCU
 */
class MPCMotorsCMDPlugin : public plugin::PluginBase {
public:
	MPCMotorsCMDPlugin() : PluginBase(),
		mpc_motors_cmd_nh("~mpc_motors_cmd")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		// Subscribers
        mpc_motors_cmd_sub = mpc_motors_cmd_nh.subscribe("cmd", 2, &MPCMotorsCMDPlugin::mpc_motors_cmd_cb, this);
	}

	Subscriptions get_subscriptions() override
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mpc_motors_cmd_nh;			//!< node handler
    ros::Subscriber mpc_motors_cmd_sub;        //!< mavros_msgs/MPCMotorsCMD subscriber

    void mpc_motors_cmd_cb(const mavros_msgs::MPCMotorsCMD::ConstPtr &req)
    {
       mavlink::common::msg::MPC_MOTORS_CMD msg{};
       msg.time_usec = req->time_usec; 
       msg.time_init = req->time_init;
       msg.dt = req->dt;
       msg.mpc_on = req->mpc_on;
       msg.tn = req->tn;
       for (int i = 0; i < mavros_msgs::MPCMotorsCMD::HORIZON_MPC; i++) {
           msg.tb[i] = req->tb[i];
           msg.mx[i] = req->mx[i];
           msg.my[i] = req->my[i];
           msg.mz[i] = req->mz[i];
       }
       UAS_FCU(m_uas)->send_message_ignore_drop(msg);
       // Warn the message was sent
    //    ROS_INFO("MPC_MOTORS_CMD message sent");
    }
};

}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MPCMotorsCMDPlugin, mavros::plugin::PluginBase)
