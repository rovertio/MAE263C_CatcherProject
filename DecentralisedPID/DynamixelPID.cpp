#include <array>
#include <cmath>
#include <algorithm>

// Define constants
const int PWM_SCALE = 884;
const double MAX_PWM = 1.0;

// Define types for clarity
using Vector2 = std::array<double, 2>;
using Vector4 = std::array<double, 4>;
//
//  State vectors shoudl be of form
//      [ Theta1Target]
//      [ Theta2Target]
//      [  Theta1Arm  ]
//      [  Theta2Arm  ]
//
//    MATLAB CODE HAS BETTER COMMENTING PLEASE REFER TO THAT    
//
// Function implementing the PID controller
void DynamixelPID(const Vector4& CurrentState,
                  Vector4& PastState,
                  double TimeSinceLast,
                  Vector2& RunningErrorSums,
                  Vector2& PWMValues) {
    // PID gain matrices (identity matrices in this case)
    const double K_p[2][2] = {{1.0, 0.0}, {0.0, 1.0}};
    const double K_I[2][2] = {{1.0, 0.0}, {0.0, 1.0}};
    const double K_D[2][2] = {{1.0, 0.0}, {0.0, 1.0}};

    // Calculate current error
    Vector2 CurrentError = {
        CurrentState[0] - CurrentState[2],
        CurrentState[1] - CurrentState[3]
    };

    // Calculate error derivative
    Vector2 ErrorDeriv = {
        ((CurrentState[0] - CurrentState[2]) - (PastState[0] - PastState[2])) / TimeSinceLast,
        ((CurrentState[1] - CurrentState[3]) - (PastState[1] - PastState[3])) / TimeSinceLast
    };

    // Update running error sums
    RunningErrorSums[0] += CurrentError[0] * TimeSinceLast;
    RunningErrorSums[1] += CurrentError[1] * TimeSinceLast;

    // Compute PID output
    Vector2 PWMOutput = {
        K_p[0][0] * CurrentError[0] + K_D[0][0] * ErrorDeriv[0] + K_I[0][0] * RunningErrorSums[0],
        K_p[1][1] * CurrentError[1] + K_D[1][1] * ErrorDeriv[1] + K_I[1][1] * RunningErrorSums[1]
    };

    // Clamp PWM values to [-1, 1]
    for (int i = 0; i < 2; ++i) {
        if (std::abs(PWMOutput[i]) > MAX_PWM) {
            PWMOutput[i] = (PWMOutput[i] > 0 ? 1 : -1);
        }
    }

    // Scale to Dynamixel range and round
    PWMValues[0] = std::round(PWMOutput[0] * PWM_SCALE);
    PWMValues[1] = std::round(PWMOutput[1] * PWM_SCALE);

    // Update past state
    PastState = CurrentState;
}
