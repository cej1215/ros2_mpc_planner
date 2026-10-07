#include <mpc_planner/cost_debug.h>
#include <mpc_planner_solver/solver_interface.h>
#include <ros_tools/logging.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace MPCPlanner
{
    namespace
    {
        struct SplineValue
        {
            double value;
            double derivative;
        };

        double readParameter(
            Solver &solver,
            int stage,
            const std::string &name)
        {
            if (!solver.hasParameter(std::string(name)))
            {
                throw std::runtime_error(
                    "Missing solver parameter: " + name);
            }

            const double value =
                solver.getParameter(stage, std::string(name));

            if (!std::isfinite(value))
            {
                throw std::runtime_error(
                    "Non-finite solver parameter: " + name);
            }

            return value;
        }

        // 1 / (1 + exp(z))를 overflow를 피하는 형태로 계산한다.
        double sigmoidWeight(double z)
        {
            if (z >= 0.0)
            {
                const double e = std::exp(-z);
                return e / (1.0 + e);
            }

            return 1.0 / (1.0 + std::exp(z));
        }

        // solver_generator/spline.py의 at()와 deriv()를 재현
        // name에는 "spline_x" 또는 "spline_y"가 들어간다.
        SplineValue evaluateSpline(
            Solver &solver,
            int stage,
            const std::string &name,
            int num_segments,
            double s)
        {
            const auto evaluateSegment = [&](int index) -> SplineValue
            {
                const std::string number = std::to_string(index);
                const std::string prefix = name + number + "_";

                const double a =
                    readParameter(solver, stage, prefix + "a");
                const double b =
                    readParameter(solver, stage, prefix + "b");
                const double c =
                    readParameter(solver, stage, prefix + "c");
                const double d =
                    readParameter(solver, stage, prefix + "d");

                const double start = readParameter(
                    solver, stage, "spline" + number + "_start");

                const double ds = s - start;

                return {
                    a * ds * ds * ds + b * ds * ds + c * ds + d,
                    3.0 * a * ds * ds + 2.0 * b * ds + c
                };
            };

            SplineValue result = evaluateSegment(num_segments - 1);

            for (int i = num_segments - 1; i > 0; --i)
            {
                const double start = readParameter(
                    solver,
                    stage,
                    "spline" + std::to_string(i) + "_start");

                const double lambda =
                    sigmoidWeight((s - start + 0.02) / 0.1);

                const SplineValue previous = evaluateSegment(i - 1);

                result.value =
                    lambda * previous.value
                    + (1.0 - lambda) * result.value;

                // Python deriv()와 동일하게 구간별 미분값을 혼합한다.
                result.derivative =
                    lambda * previous.derivative
                    + (1.0 - lambda) * result.derivative;
            }

            return result;
        }
    }

    CostDebug evaluateStageCosts(
        Solver &solver,
        int num_segments,
        double stage_cost_scale)
    {
        CostDebug result;
        result.stage_count = solver.N;
        result.stage_cost_scale = stage_cost_scale;

        try
        {
            if (solver.N <= 0 || num_segments <= 0)
            {
                throw std::runtime_error(
                    "Invalid horizon or spline segment count");
            }

            if (!std::isfinite(stage_cost_scale)
                || stage_cost_scale < 0.0)
            {
                throw std::runtime_error("Invalid stage cost scale");
            }

            // 매 호출마다 새 result에서 이번 주기의 비용만 합산한다.
            for (int k = 0; k < solver.N; ++k)
            {
                // 이번 최적화에서 반환된 상태와 입력
                const double a = solver.getOutput(k, "a");
                const double w = solver.getOutput(k, "w");
                const double v = solver.getOutput(k, "v");

                const double x = solver.getOutput(k, "x");
                const double y = solver.getOutput(k, "y");
                const double s = solver.getOutput(k, "spline");

                // 이번 최적화에 실제로 전달한 가중치와 목표 속도
                const double W_acc =
                    readParameter(solver, k, "acceleration");
                const double W_w =
                    readParameter(solver, k, "angular_velocity");
                const double W_v =
                    readParameter(solver, k, "velocity");
                const double v_ref =
                    readParameter(solver, k, "reference_velocity");
                const double W_l =
                    readParameter(solver, k, "lag");
                const double W_c =
                    readParameter(solver, k, "contour");

                // 해당 stage의 최적화된 s에서 참조 경로를 평가한다.
                const SplineValue path_x = evaluateSpline(
                    solver, k, "spline_x", num_segments, s);

                const SplineValue path_y = evaluateSpline(
                    solver, k, "spline_y", num_segments, s);

                const double tangent_norm = std::sqrt(
                    path_x.derivative * path_x.derivative
                    + path_y.derivative * path_y.derivative);

                // 경로 접선 크기가 유효한 값인지 검사
                if (!std::isfinite(tangent_norm)
                    || tangent_norm <= 0.0)
                {
                    throw std::runtime_error(
                        "Invalid spline tangent at stage "
                        + std::to_string(k));
                }

                // contouring의 stage cost와 동일
                const double tangent_x =
                    path_x.derivative / tangent_norm;
                const double tangent_y =
                    path_y.derivative / tangent_norm;

                const double dx = x - path_x.value;
                const double dy = y - path_y.value;

                const double lag_error =
                    tangent_x * dx + tangent_y * dy;

                const double contour_error =
                    tangent_y * dx - tangent_x * dy;

                const double velocity_error = v - v_ref;

                result.raw.acceleration += W_acc * a * a;
                result.raw.angular_velocity += W_w * w * w;
                result.raw.velocity +=
                    W_v * velocity_error * velocity_error;
                result.raw.lag +=
                    W_l * lag_error * lag_error;
                result.raw.contour +=
                    W_c * contour_error * contour_error;
            }

            // acaods solver 배율 적용
            result.scaled.acceleration =
                stage_cost_scale * result.raw.acceleration;
            result.scaled.angular_velocity =
                stage_cost_scale * result.raw.angular_velocity;
            result.scaled.velocity =
                stage_cost_scale * result.raw.velocity;
            result.scaled.lag =
                stage_cost_scale * result.raw.lag;
            result.scaled.contour =
                stage_cost_scale * result.raw.contour;

            // Terminal 상태는 N, 파라미터는 현재 인터페이스와 동일하게 N-1 사용
            {
                const int state_stage = solver.N;
                const int parameter_stage = solver.N - 1;

                const double x_N = solver.getOutput(state_stage, "x");
                const double y_N = solver.getOutput(state_stage, "y");
                const double psi_N = solver.getOutput(state_stage, "psi");
                const double s_N = solver.getOutput(state_stage, "spline");

                const double W_psi =
                    readParameter(solver, parameter_stage, "terminal_angle");

                const double m_term =
                    readParameter(solver, parameter_stage, "terminal_contouring");

                const double W_l =
                    readParameter(solver, parameter_stage, "lag");

                const double W_c =
                    readParameter(solver, parameter_stage, "contour");

                // 최종 예측 s에서 참조 경로 위치와 접선을 계산
                const SplineValue path_x = evaluateSpline(
                    solver, parameter_stage, "spline_x", num_segments, s_N);

                const SplineValue path_y = evaluateSpline(
                    solver, parameter_stage, "spline_y", num_segments, s_N);

                const double tangent_norm = std::sqrt(
                    path_x.derivative * path_x.derivative
                    + path_y.derivative * path_y.derivative);

                if (!std::isfinite(tangent_norm) || tangent_norm <= 0.0)
                {
                    throw std::runtime_error(
                        "Invalid spline tangent at terminal stage");
                }

                const double tangent_x = path_x.derivative / tangent_norm;
                const double tangent_y = path_y.derivative / tangent_norm;

                const double dx = x_N - path_x.value;
                const double dy = y_N - path_y.value;

                const double lag_error =
                    tangent_x * dx + tangent_y * dy;

                const double contour_error =
                    tangent_y * dx - tangent_x * dy;

                const double path_angle = std::atan2(tangent_y, tangent_x);

                // Python haar_difference_without_abs()와 동일한 식
                const double pi = std::acos(-1.0);
                const double angle_error =
                    std::fmod(psi_N - path_angle + pi, 2.0 * pi) - pi;

                LOG_INFO(
                    "Terminal cost debug: "
                    << ", heading_error=" << angle_error * 180.0 / pi
                );

                // 현재 생성된 solver의 terminal 비용 배율은 1.0
                result.terminal.heading =
                    W_psi * angle_error * angle_error;

                result.terminal.lag =
                    m_term * W_l * lag_error * lag_error;

                result.terminal.contour =
                    m_term * W_c * contour_error * contour_error;
            }

            // raw cost 합계값과 배율이 적용된 cost total값 중 하나라도 오류가 있는지 검사
            if (!std::isfinite(result.raw.total())
                || !std::isfinite(result.scaled.total()))
            {
                throw std::runtime_error(
                    "Non-finite stage cost result");
            }

            result.valid = true;
        }
        catch (const std::exception &e)
        {
            // 진단 계산 실패를 MPC 성공/실패 판정으로 전달하지 않는다.
            result.valid = false;
            result.error = e.what();
        }

        return result;
    }
}