#include "acado/acado_code_generation.hpp"

USING_NAMESPACE_ACADO

int main(void)
{
    // Differential states
    DifferentialState theta, phi, gamma, th, dth, I;

    // Controls
    Control u_gamma, v;

    // setup the differential equation
    IntermediateState a11 = 0.2 * cos(gamma) * cos(gamma) * sin(theta) * cos(phi);
    IntermediateState a12 = 0.2 * cos(gamma) * sin(gamma) * sin(theta);
    IntermediateState a13 = -cos(gamma) * cos(theta) * cos(phi);

    IntermediateState a21 = 0.2 * cos(theta) * sin(gamma) * cos(gamma) * sin(theta) * cos(phi);
    IntermediateState a22 = 0.2 * cos(theta) * sin(gamma) * sin(theta);
    IntermediateState a23 = -cos(theta) * sin(gamma) * cos(theta) * cos(phi);

    double h = M_PI / 6.0;
    double a = 0.2;
    IntermediateState e_theta = h + a * sin(2 * th) - theta;
    IntermediateState e_phi   = 4 * a * cos(th) - phi;
    IntermediateState final_cost =  5 * 1e2 * ((e_theta * e_theta) + (e_phi * e_phi));

    IntermediateState ctl_cost = 1e-3 * (u_gamma * u_gamma) + 0.1 * (v * v);
    IntermediateState state_cost = 5 * 1e2 * ((e_theta * e_theta) + (e_phi * e_phi));
    IntermediateState virt_cost = (dth - 1.0) * (dth - 1.0);
    IntermediateState integ_cost = state_cost + ctl_cost + virt_cost;

    DifferentialEquation f;
    f << dot(theta) == -1.1 * (a11 + a12 + a13);
    f << dot(phi)   == -1.1 * (a21 + a22 + a23);
    f << dot(gamma) == u_gamma;
    f << dot(th)    == dth;
    f << dot(dth)   == v;
    f << dot(I)     == integ_cost;

    // set the expressions
    Expression states;
    states << theta;
    states << phi;
    states << gamma;
    states << th;
    states << dth;
    states << I;

    Expression controls;
    controls << u_gamma;
    controls << v;

    Function rf, rfN;
    rf << states;
    rf << controls;

    rfN << states;

    BMatrix Wmat(rf.getDim(), rf.getDim()); Wmat.setAll( true );
    BMatrix WNmat(rfN.getDim(), rfN.getDim()); WNmat.setAll( true );

    // SET UP THE MPC - OPTIMAL CONTROL PROBLEM
    const double t_start =  0.0;
    const double t_end   =  1.5;
    const uint   N 		 =  12;

    OCP ocp( t_start, t_end, N );
    ocp.minimizeMayerTerm( final_cost + I );

    ocp.setModel( f );

    ocp.subjectTo( 0       <= theta <= M_PI_2   );
    ocp.subjectTo( -M_PI_2 <= phi <= M_PI_2   );
    ocp.subjectTo( -M_PI   <= gamma <= M_PI   );
    ocp.subjectTo( -100    <= th    <= 100   );
    ocp.subjectTo( -100   <= dth   <= 100   );

    ocp.subjectTo( -4   <= u_gamma  <= 4   );
    ocp.subjectTo( -5   <= v  <= 5  );

    //ocp.subjectTo(AT_START, 0 <= I <= 0);

    // export the problem
    OCPexport mpc( ocp );

    mpc.set( HESSIAN_APPROXIMATION,       EXACT_HESSIAN  		);
    mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING 	);
    mpc.set( INTEGRATOR_TYPE,             INT_RK4   			);
    mpc.set( NUM_INTEGRATOR_STEPS,        12            		);
    mpc.set( QP_SOLVER,                   QP_QPOASES    		);
    mpc.set( HOTSTART_QP,                 YES             		);
    mpc.set( GENERATE_TEST_FILE,          YES            		);
    mpc.set( GENERATE_MAKE_FILE,          YES            		);
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO            		);
    mpc.set( SPARSE_QP_SOLUTION, 		  FULL_CONDENSING_N2	);
    mpc.set( DYNAMIC_SENSITIVITY, 		  SYMMETRIC				);
    mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO 					);
    mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES 				);

    mpc.exportCode( "kite_mpc_export" );
    mpc.printDimensionsQP( );
    return 0;
}	
