#include "acado_node2.hpp"

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NU          ACADO_NU  /* Number of control inputs. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   8        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

void ACADO_Node::filterCallback(const sensor_msgs::MultiDOFJointState::ConstPtr &msg)
{
    acado_state.theta = msg->twist.back().angular.x;
    acado_state.phi   = msg->twist.back().angular.y;
    acado_state.gamma = msg->twist.back().angular.z;
    if(!is_initialized())
        initialize();
}

ACADO_Node::ACADO_Node(const ros::NodeHandle &_nh )
{
    /** initialize subscribers and publishers */
    nh = std::make_shared<ros::NodeHandle>(_nh);

    control_pub    = nh->advertise<openkite::aircraft_controls>("kite_controls", 100);
    traj_pub       = nh->advertise<sensor_msgs::MultiDOFJointState>("/opt_traj", 10);
    diagnostic_pub = nh->advertise<openkite::mpc_diagnostic>("/mpc_diagnostic", 10);

    std::string state_topic = "/kite_state";
    state_sub = nh->subscribe(state_topic, 100, &ACADO_Node::filterCallback, this);

    m_initialized = false;
    comp_time_ms = 0.0;
}


void ACADO_Node::publish()
{
    openkite::aircraft_controls control_msg;
    control = acado_getVariablesU();

    control_msg.header.stamp = ros::Time::now();
    control_msg.thrust   = 0;
    control_msg.elevator = 0;
    control_msg.rudder   = control[0];

    /** publish current control */
    control_pub.publish(control_msg);

}

/** publish diagnostic info */
void ACADO_Node::publish_mpc_diagnostic()
{
    openkite::mpc_diagnostic diag_msg;
    diag_msg.header.stamp = ros::Time::now();

    diag_msg.pos_error    = acado_getObjective();
    diag_msg.comp_time_ms = comp_time_ms * 1000;
    diag_msg.virt_state   = 0.0;
    diag_msg.vel_error    = 0.0;

    /** dummy output */
    diag_msg.cost         = 0;
    diagnostic_pub.publish(diag_msg);
}

void ACADO_Node::compute_control()
{
    /** update intial state */

    real_t *x = acado_getVariablesX();

    acadoVariables.x0[ 0 ] = acado_state.theta;
    acadoVariables.x0[ 1 ] = acado_state.phi;
    acadoVariables.x0[ 2 ] = acado_state.gamma;
    acadoVariables.x0[ 3 ] = x[3];
    acadoVariables.x0[ 4 ] = x[4];
    acadoVariables.x0[ 5 ] = 0;

    for(int iter = 0; iter < NUM_STEPS; ++iter)
    {
        /* Perform the feedback step. */
        acado_feedbackStep( );

        if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );

        /* Prepare for the next step. */
        acado_preparationStep();
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "acado_node2");
    ros::NodeHandle nh;

    /* Some temporary variables. */
    int    i, iter;
    acado_timer timer;

    //ros::Publisher pb = n.advertise<openkite::aircraft_controls>("kite_controls", 100);

    /* Initialize the solver. */
    acado_initializeSolver();

    /* Initialize the states and controls. */
    for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 1.0;
    for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 1.0;

    ACADO_Node tracker(nh);
    ros::Rate loop_rate(100); /** 18 Hz */
    bool broadcast_trajectory = true;

    while (ros::ok())
    {
        ros::spinOnce();

        if(tracker.is_initialized())
        {
            double start = ros::Time::now().toSec();
            tracker.compute_control();
            double finish = ros::Time::now().toSec();
            tracker.publish();
            tracker.publish_mpc_diagnostic();
            tracker.comp_time_ms = finish - start;
            std::cout << "Control computational delay: " << finish - start << "\n";

            loop_rate.sleep();
        }
        else
        {
            loop_rate.sleep();
        }

    }

    return 0;
}	
