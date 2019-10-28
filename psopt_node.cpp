#include "psopt.h"
#include <fstream>


adouble endpoint_cost(adouble* initial_states, adouble* final_states,
                      adouble* parameters,adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
    double h = M_PI / 6.0;
    double a = 0.2;
    adouble e_theta = h + a * sin(2 * final_states[CINDEX(4)]) - final_states[CINDEX(1)];
    adouble e_phi   = 4 * a * cos(final_states[CINDEX(4)]) - final_states[CINDEX(2)];

    return 5 * 1e2 * ((e_theta * e_theta) + (e_phi * e_phi));
}

adouble integrand_cost(adouble* states, adouble* controls, adouble* parameters,
                       adouble& time, adouble* xad, int iphase, Workspace* workspace)
{
    double h = M_PI / 6.0;
    double a = 0.2;
    adouble e_theta = h + a * sin(2 * states[CINDEX(4)]) - states[CINDEX(1)];
    adouble e_phi   = 4 * a * cos(states[CINDEX(4)]) - states[CINDEX(2)];

    adouble ctl_cost = 1e-3 * (controls[ CINDEX(1) ] * controls[ CINDEX(1) ]) + 0.1 * ((controls[ CINDEX(2) ] * controls[ CINDEX(2) ]));
    adouble state_cost = 5 * 1e2 * ((e_theta * e_theta) + (e_phi * e_phi));
    adouble virt_cost = (states[CINDEX(5)] - 1.0) * (states[CINDEX(5)] - 1.0);

    return state_cost + ctl_cost + virt_cost;
}


void dae(adouble* derivatives, adouble* path, adouble* states,
         adouble* controls, adouble* parameters, adouble& time,
         adouble* xad, int iphase, Workspace* workspace)
{

    adouble u_gamma = controls[ CINDEX(1) ];
    adouble v       = controls[ CINDEX(2) ];

    adouble theta 		= states[ CINDEX(1) ];
    adouble phi 	    = states[ CINDEX(2) ];
    adouble gamma 		= states[ CINDEX(3) ];
    adouble th1         = states[ CINDEX(4) ];
    adouble th2         = states[ CINDEX(5) ];

    adouble a11 = 0.2 * cos(gamma) * cos(gamma) * sin(theta) * cos(phi);
    adouble a12 = 0.2 * cos(gamma) * sin(gamma) * sin(theta);
    adouble a13 = -cos(gamma) * cos(theta) * cos(phi);

    adouble a21 = 0.2 * cos(theta) * sin(gamma) * cos(gamma) * sin(theta) * cos(phi);
    adouble a22 = 0.2 * cos(theta) * sin(gamma) * sin(theta);
    adouble a23 = -cos(theta) * sin(gamma) * cos(theta) * cos(phi);

    derivatives[ CINDEX(1) ] =   -1.1 * (a11 + a12 + a13);
    derivatives[ CINDEX(2) ] =   -1.1 * (a21 + a22 + a23);
    derivatives[ CINDEX(3) ] =   u_gamma;
    derivatives[ CINDEX(4) ] =   th2;
    derivatives[ CINDEX(5) ] =   v;
}


void events(adouble* e, adouble* initial_states, adouble* final_states,
            adouble* parameters,adouble& t0, adouble& tf, adouble* xad,
            int iphase, Workspace* workspace)

{

    adouble x_i 	= initial_states[ CINDEX(1) ];
    adouble y_i 	= initial_states[ CINDEX(2) ];
    adouble vx_i 	= initial_states[ CINDEX(3) ];
    adouble vy_i 	= initial_states[ CINDEX(4) ];

    e[ CINDEX(1) ] 	 = initial_states[ CINDEX(1) ];
    e[ CINDEX(2) ]    = initial_states[ CINDEX(2) ];
    e[ CINDEX(3) ]    = initial_states[ CINDEX(3) ];
    e[ CINDEX(4) ]    = initial_states[ CINDEX(4) ];
    e[ CINDEX(5) ]    = initial_states[ CINDEX(5) ];

}


void linkages( adouble* linkages, adouble* xad, Workspace* workspace)
{
    // Single phase problem
}



int main(void)
{
    Alg  algorithm;
    Sol  solution;
    Prob problem;

    problem.name        		= "Kite path following problem";
    problem.outfilename         = "lox.txt";
    problem.nphases   			= 1;
    problem.nlinkages           = 0;

    psopt_level1_setup(problem);

    problem.phases(1).nstates   		 = 5;
    problem.phases(1).ncontrols 		 = 2;
    problem.phases(1).nevents   		 = 7;
    problem.phases(1).npath     		 = 0;
    problem.phases(1).nodes           = "[12]";

    psopt_level2_setup(problem, algorithm);

    problem.phases(1).bounds.lower.states = "[-1.57 -1.5708 -3.14 -100 -100]";
    problem.phases(1).bounds.upper.states = "[ 1.58  1.5708 3.14 100 100]";
    problem.phases(1).bounds.lower.controls = "[-4 -5]";
    problem.phases(1).bounds.upper.controls = "[4 5]";
    problem.phases(1).bounds.lower.events="[0.78 -0.78 1.57 -3.14 -5.0]";
    problem.phases(1).bounds.upper.events="[0.78 -0.78 1.57  3.14  5.0]";
    problem.phases(1).bounds.lower.StartTime    = 0.0;
    problem.phases(1).bounds.upper.StartTime    = 0.0;
    problem.phases(1).bounds.lower.EndTime      = 1.5;
    problem.phases(1).bounds.upper.EndTime      = 1.5;

    problem.integrand_cost 	= &integrand_cost;
    problem.endpoint_cost 	= &endpoint_cost;
    problem.dae             = &dae;
    problem.events 		    = &events;
    problem.linkages		= &linkages;

    algorithm.nlp_iter_max                = 500;
    algorithm.nlp_tolerance               = 1.e-4;
    algorithm.nlp_method                  = "IPOPT";
    algorithm.scaling                     = "automatic";
    algorithm.derivatives                 = "automatic";
    algorithm.collocation_method          = "Legendre";
    algorithm.print_level                 = 1;
    algorithm.hessian                     = "exact";
    algorithm.ode_tolerance               = 1e-2;
    algorithm.mr_max_iterations           = 2;

    //psopt(solution, problem, algorithm);

    //std::cout << "CPU time: " << solution.cpu_time << "\n";
    //std::cout << "Return code: " <<  solution.nlp_return_code << "\n";

    DMatrix states, control;

    //states      = solution.get_states_in_phase(1);
    //control     = solution.get_controls_in_phase(1);

    /** read the data */
    std::ifstream data_file("psopt_data.txt", std::ios::in);
    std::ofstream psopt_sol;
    psopt_sol.open("psopt_solution.txt", std::ios::out);

    double vv = 0;
    for(uint i = 0; i < 1904; ++i)
    {
        /** update the initial condition */
        double t, theta, phi, gamma, vs;
        data_file >> t;
        data_file >> theta;
        data_file >> phi;
        data_file >> gamma;
        data_file >> vs;

        if(i >= 0) {
            problem.phases(1).bounds.lower.events(1) = theta;
            problem.phases(1).bounds.upper.events(1) = theta;
            problem.phases(1).bounds.lower.events(2) = phi;
            problem.phases(1).bounds.upper.events(2) = phi;
            problem.phases(1).bounds.lower.events(3) = gamma;
            problem.phases(1).bounds.upper.events(3) = gamma;
            problem.phases(1).bounds.lower.events(4) = vs;
            problem.phases(1).bounds.upper.events(4) = vs;
            problem.phases(1).bounds.lower.events(5) = -5;
            problem.phases(1).bounds.upper.events(5) = 5;

            psopt(solution, problem, algorithm);

            states = solution.get_states_in_phase(1);
            control = solution.get_controls_in_phase(1);

            vv = states(5, 1);
            double u_gamma = control(1, 1);

            problem.phases(1).guess.states = states;
            problem.phases(1).guess.controls = control;

            std::cout << "X0 : " << theta << " " << phi << " " << gamma << " " << vs << " " << vv << "\n";
            std::cout << "CPU time: " << solution.cpu_time << " Return code: " << solution.nlp_return_code
                      << " Control: " << u_gamma << "\n";

            /** write solution to a file */
            psopt_sol << t << " " << u_gamma << " " << solution.cpu_time << " " << solution.get_cost() << "\n";
        }
    }
    psopt_sol.close();

    return 0;
}
