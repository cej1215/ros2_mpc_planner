#include <mpc_planner_solver/mpc_planner_parameters.h>

#include <mpc_planner_solver/solver_interface.h>
namespace MPCPlanner{

void setSolverParameterAcceleration(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 53 + 0] = value;
}
void setSolverParameterAngularVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 53 + 1] = value;
}
void setSolverParameterVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 53 + 2] = value;
}
void setSolverParameterReferenceVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 53 + 3] = value;
}
void setSolverParameterContour(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 53 + 4] = value;
}
void setSolverParameterLag(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 53 + 5] = value;
}
void setSolverParameterTerminalAngle(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 53 + 6] = value;
}
void setSolverParameterTerminalContouring(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 53 + 7] = value;
}
void setSolverParameterSplineXA(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 53 + 8] = value;
	else if(index == 1)
		params.all_parameters[k * 53 + 17] = value;
	else if(index == 2)
		params.all_parameters[k * 53 + 26] = value;
	else if(index == 3)
		params.all_parameters[k * 53 + 35] = value;
	else if(index == 4)
		params.all_parameters[k * 53 + 44] = value;
}
void setSolverParameterSplineXB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 53 + 9] = value;
	else if(index == 1)
		params.all_parameters[k * 53 + 18] = value;
	else if(index == 2)
		params.all_parameters[k * 53 + 27] = value;
	else if(index == 3)
		params.all_parameters[k * 53 + 36] = value;
	else if(index == 4)
		params.all_parameters[k * 53 + 45] = value;
}
void setSolverParameterSplineXC(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 53 + 10] = value;
	else if(index == 1)
		params.all_parameters[k * 53 + 19] = value;
	else if(index == 2)
		params.all_parameters[k * 53 + 28] = value;
	else if(index == 3)
		params.all_parameters[k * 53 + 37] = value;
	else if(index == 4)
		params.all_parameters[k * 53 + 46] = value;
}
void setSolverParameterSplineXD(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 53 + 11] = value;
	else if(index == 1)
		params.all_parameters[k * 53 + 20] = value;
	else if(index == 2)
		params.all_parameters[k * 53 + 29] = value;
	else if(index == 3)
		params.all_parameters[k * 53 + 38] = value;
	else if(index == 4)
		params.all_parameters[k * 53 + 47] = value;
}
void setSolverParameterSplineYA(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 53 + 12] = value;
	else if(index == 1)
		params.all_parameters[k * 53 + 21] = value;
	else if(index == 2)
		params.all_parameters[k * 53 + 30] = value;
	else if(index == 3)
		params.all_parameters[k * 53 + 39] = value;
	else if(index == 4)
		params.all_parameters[k * 53 + 48] = value;
}
void setSolverParameterSplineYB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 53 + 13] = value;
	else if(index == 1)
		params.all_parameters[k * 53 + 22] = value;
	else if(index == 2)
		params.all_parameters[k * 53 + 31] = value;
	else if(index == 3)
		params.all_parameters[k * 53 + 40] = value;
	else if(index == 4)
		params.all_parameters[k * 53 + 49] = value;
}
void setSolverParameterSplineYC(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 53 + 14] = value;
	else if(index == 1)
		params.all_parameters[k * 53 + 23] = value;
	else if(index == 2)
		params.all_parameters[k * 53 + 32] = value;
	else if(index == 3)
		params.all_parameters[k * 53 + 41] = value;
	else if(index == 4)
		params.all_parameters[k * 53 + 50] = value;
}
void setSolverParameterSplineYD(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 53 + 15] = value;
	else if(index == 1)
		params.all_parameters[k * 53 + 24] = value;
	else if(index == 2)
		params.all_parameters[k * 53 + 33] = value;
	else if(index == 3)
		params.all_parameters[k * 53 + 42] = value;
	else if(index == 4)
		params.all_parameters[k * 53 + 51] = value;
}
void setSolverParameterSplineStart(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 53 + 16] = value;
	else if(index == 1)
		params.all_parameters[k * 53 + 25] = value;
	else if(index == 2)
		params.all_parameters[k * 53 + 34] = value;
	else if(index == 3)
		params.all_parameters[k * 53 + 43] = value;
	else if(index == 4)
		params.all_parameters[k * 53 + 52] = value;
}
}
