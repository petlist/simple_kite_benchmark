#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <stdio.h>
#include <iostream>
#include <fstream>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   8        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

int main(void)
{
    /* Some temporary variables. */
    int    i, iter;
    acado_timer timer;

    /* Initialize the solver. */
    acado_initializeSolver();

    /* Initialize the states and controls. */
    for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 1.0;
    for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 1.0;

    //if( VERBOSE )
    //    printf("\n\n Average time of one real-time iteration:   %.3g milliseconds\n\n", 1e3 * te);

    /** read the data */
    std::ifstream data_file("psopt_data.txt", std::ios::in);
    std::ofstream acado_sol;
    acado_sol.open("acado_solution.txt", std::ios::out);

    /* Prepare first step */
    acado_preparationStep();

    double vv = 0;
    for(uint i = 0; i < 1904; ++i)
    {
        /** update the initial condition */
        double t, theta, phi, gamma, vs;
        double I0 = 0;
        data_file >> t;
        data_file >> theta;
        data_file >> phi;
        data_file >> gamma;
        data_file >> vs;

        acadoVariables.x0[ 0 ] = theta;
        acadoVariables.x0[ 1 ] = phi;
        acadoVariables.x0[ 2 ] = gamma;
        acadoVariables.x0[ 3 ] = vs;
        acadoVariables.x0[ 4 ] = vv;
        acadoVariables.x0[ 5 ] = I0;

        acado_tic(&timer);
        /* The "real-time iterations" loop. */
        for(iter = 0; iter < NUM_STEPS; ++iter)
        {
            /* Perform the feedback step. */
            acado_feedbackStep( );

            /* Apply the new control immediately to the process, first NU components. */
            if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );

            /* Prepare for the next step. */
            acado_preparationStep();
        }

        /* Read the elapsed time. */
        real_t te = acado_toc( &timer );
        real_t *U = acado_getVariablesU();


        std::cout << "X0 : " << theta << " " << phi << " " << gamma << " " << vs << " " << vv << "\n";
        std::cout << "CPU time: " << (te * 1000) << " Return code: " << acado_getKKT()
                      << " Control: " << U[0] << "\n";

         /** write solution to a file */
         acado_sol << t << " " << U[0] << " " << (te * 1000) << " " << acado_getObjective() << "\n";
    }

    acado_sol.close();

    return 0;
}	
