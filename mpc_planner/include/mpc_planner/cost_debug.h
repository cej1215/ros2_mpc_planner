#ifndef MPC_PLANNER_COST_BREAKDOWN_H
#define MPC_PLANNER_COST_BREAKDOWN_H

#include <string>

namespace MPCPlanner
{
    class Solver;

    struct StageCostTerms
    {
        double acceleration{0.0};
        double angular_velocity{0.0};
        double velocity{0.0};
        double lag{0.0};
        double contour{0.0};

        double total() const
        {
            return acceleration
                 + angular_velocity
                 + velocity
                 + lag
                 + contour;
        }
    };

    struct TerminalCostTerms
    {
        double heading{0.0};
        double lag{0.0};
        double contour{0.0};

        double total() const
        {
            return heading + lag + contour;
        }
    };

    struct CostDebug
    {
        bool valid{false};
        std::string error;

        int stage_count{0};
        double stage_cost_scale{0.0};

        StageCostTerms raw;
        StageCostTerms scaled;

        TerminalCostTerms terminal;
    };

    CostDebug evaluateStageCosts(
        Solver &solver,
        int num_segments,
        double stage_cost_scale);
}

#endif