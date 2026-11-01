#include <mpc_planner_solver/mpc_planner_parameters.h>

#include <mpc_planner_solver/solver_interface.h>
namespace MPCPlanner{

void setSolverParameterAcceleration(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 0] = value;
}
void setSolverParameterAngularVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 1] = value;
}
void setSolverParameterVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 2] = value;
}
void setSolverParameterReferenceVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 3] = value;
}
void setSolverParameterContour(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 4] = value;
}
void setSolverParameterLag(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 5] = value;
}
void setSolverParameterTerminalAngle(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 6] = value;
}
void setSolverParameterTerminalContouring(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 7] = value;
}
void setSolverParameterSplineXA(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 56 + 8] = value;
	else if(index == 1)
		params.all_parameters[k * 56 + 17] = value;
	else if(index == 2)
		params.all_parameters[k * 56 + 26] = value;
	else if(index == 3)
		params.all_parameters[k * 56 + 35] = value;
	else if(index == 4)
		params.all_parameters[k * 56 + 44] = value;
}
void setSolverParameterSplineXB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 56 + 9] = value;
	else if(index == 1)
		params.all_parameters[k * 56 + 18] = value;
	else if(index == 2)
		params.all_parameters[k * 56 + 27] = value;
	else if(index == 3)
		params.all_parameters[k * 56 + 36] = value;
	else if(index == 4)
		params.all_parameters[k * 56 + 45] = value;
}
void setSolverParameterSplineXC(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 56 + 10] = value;
	else if(index == 1)
		params.all_parameters[k * 56 + 19] = value;
	else if(index == 2)
		params.all_parameters[k * 56 + 28] = value;
	else if(index == 3)
		params.all_parameters[k * 56 + 37] = value;
	else if(index == 4)
		params.all_parameters[k * 56 + 46] = value;
}
void setSolverParameterSplineXD(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 56 + 11] = value;
	else if(index == 1)
		params.all_parameters[k * 56 + 20] = value;
	else if(index == 2)
		params.all_parameters[k * 56 + 29] = value;
	else if(index == 3)
		params.all_parameters[k * 56 + 38] = value;
	else if(index == 4)
		params.all_parameters[k * 56 + 47] = value;
}
void setSolverParameterSplineYA(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 56 + 12] = value;
	else if(index == 1)
		params.all_parameters[k * 56 + 21] = value;
	else if(index == 2)
		params.all_parameters[k * 56 + 30] = value;
	else if(index == 3)
		params.all_parameters[k * 56 + 39] = value;
	else if(index == 4)
		params.all_parameters[k * 56 + 48] = value;
}
void setSolverParameterSplineYB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 56 + 13] = value;
	else if(index == 1)
		params.all_parameters[k * 56 + 22] = value;
	else if(index == 2)
		params.all_parameters[k * 56 + 31] = value;
	else if(index == 3)
		params.all_parameters[k * 56 + 40] = value;
	else if(index == 4)
		params.all_parameters[k * 56 + 49] = value;
}
void setSolverParameterSplineYC(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 56 + 14] = value;
	else if(index == 1)
		params.all_parameters[k * 56 + 23] = value;
	else if(index == 2)
		params.all_parameters[k * 56 + 32] = value;
	else if(index == 3)
		params.all_parameters[k * 56 + 41] = value;
	else if(index == 4)
		params.all_parameters[k * 56 + 50] = value;
}
void setSolverParameterSplineYD(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 56 + 15] = value;
	else if(index == 1)
		params.all_parameters[k * 56 + 24] = value;
	else if(index == 2)
		params.all_parameters[k * 56 + 33] = value;
	else if(index == 3)
		params.all_parameters[k * 56 + 42] = value;
	else if(index == 4)
		params.all_parameters[k * 56 + 51] = value;
}
void setSolverParameterSplineStart(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 56 + 16] = value;
	else if(index == 1)
		params.all_parameters[k * 56 + 25] = value;
	else if(index == 2)
		params.all_parameters[k * 56 + 34] = value;
	else if(index == 3)
		params.all_parameters[k * 56 + 43] = value;
	else if(index == 4)
		params.all_parameters[k * 56 + 52] = value;
}
void setSolverParameterGoalWeight(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 53] = value;
}
void setSolverParameterGoalX(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 54] = value;
}
void setSolverParameterGoalY(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 56 + 55] = value;
}
}
