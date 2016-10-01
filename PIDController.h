#pragma once
//
// Lucky Resistor's Fan Controller
// ---------------------------------------------------------------------------
// (c)2016 by Lucky Resistor. See LICENSE for details.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//


#include <Arduino.h>


namespace lr {
    
    
/// A simple PID controller.
///
class PIDController
{
public:
    /// Create a new controller instance.
    ///
    PIDController();
    
public:
    /// Initialize the controller with the kp, ki and kd values.
    ///
    /// @param kp The proportional correction (>=0). Use 2.0f if you have no clue.
    /// @param ki The integral correction (>=0). Use 5.0f if you have no clue.
    /// @param kd The derivative correction (>=0). Use 0.5f if you have no clue.
    ///
    void begin(float kp, float ki, float kd);
    
    /// Set output limits.
    ///
    /// Make sure minimum < maximum.
    ///
    void setOutputLimits(float minimum, float maximum);
    
    /// Set the output direction.
    ///
    void setOutputReverse(bool isReverse);
    
    /// Set a drift for the output.
    ///
    /// A drift for the output keeps the algorithm challenging into one direction.
    /// E.g. if it's the goal to keep the output as low as possible. Simple set
    /// a drift of e.g. -0.1. This drift will always be added after the final
    /// calculation and if the set-point is reached, it will try to keep the
    /// set-point while decreasing the output over time.
    ///
    void setOutputDrift(float outputDrift);
    
    /// Set the (initial) output value.
    ///
    void setOutputValue(float value);
    
    /// Get the current output value.
    ///
    float getOutputValue() const;
    
    /// Set the target (set-point) value.
    /// If this changes a lot, set it once before the calculate() call.
    ///
    void setTargetValue(float value);
    
    /// Calculate a new outout value for the given input.
    ///
    /// This funtion automatically measures the sample time you use between two
    /// function calls. Make sure you call this method in a regular and precise
    /// interval like every 100ms for example.
    ///
    /// @return The new output value.
    ///
    float calculateOutput(float input);
    
private:
    float _kp; ///< Proportional
    float _ki; ///< Integral
    float _kd; ///< Derivative
    bool _reverseDirection; ///< true if the output works in reverse.
    float _outputDrift; ///< A constant drift to the output.
    float _output; ///< The current output value.
    float _outputMinimum; ///< The minimum value for the output.
    float _outputMaximum; ///< The maximum value for the output.
    float _errorSumAndKi; ///< The error sum of the output multiplied by Ki
    float _target; ///< The target value.
    float _lastInput; ///< The last input value.
    float _sampleTime; ///< The average sample time in seconds. Measured using the calculate() calls.
    uint32_t _lastCalculation; ///< The millis() value from the last calculation.
};
    
    
}




