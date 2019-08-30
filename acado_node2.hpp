#ifndef ACADO_NODE2_HPP
#define ACADO_NODE2_HPP

#include "sensor_msgs/MultiDOFJointState.h"
#include "std_msgs/Int32MultiArray.h"
#include "openkite/aircraft_controls.h"
#include "geometry_msgs/PoseStamped.h"
#include "openkite/mpc_diagnostic.h"

#include "boost/thread/mutex.hpp"
#include "kiteNMPF.h"
#include "integrator.h"

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "ros/ros.h"

#include <stdio.h>
#include <iostream>
#include <fstream>

struct ACADO_STATE
{
    double theta{0};
    double phi{0};
    double gamma{0};
    double th{0};
    double dth{0};
    double I{0};
};

class ACADO_Node
{
public:
    ACADO_Node(const ros::NodeHandle &_nh);
    virtual ~ACADO_Node(){}

    ros::Time last_computed_control;
    void filterCallback(const sensor_msgs::MultiDOFJointState::ConstPtr &msg);

    //void compute_control(const geometry_msgs::PoseStamped &_pose);
    void compute_control();
    void publish();
    void publish_trajectory();
    void publish_mpc_diagnostic();

    void initialize(){m_initialized = true;}
    bool is_initialized(){return m_initialized;}

    boost::mutex m_mutex;
    double comp_time_ms;

    ACADO_STATE acado_state;

private:
    real_t * control;
    real_t * kite_state;

    ros::Publisher  control_pub;
    ros::Publisher  traj_pub;
    ros::Publisher  diagnostic_pub;
    ros::Subscriber state_sub;

    /** handle instance to access node params */
    std::shared_ptr<ros::NodeHandle> nh;

    bool m_initialized;
    double transport_delay;

    //casadi::DM convertToDM(const sensor_msgs::MultiDOFJointState &_value);
};


#endif // ACADO_NODE2_HPP
