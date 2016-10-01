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
#include "PIDController.h"


namespace lr {
    
    
PIDController::PIDController()
:
    _kp(0.0f),
    _ki(0.0f),
    _kd(0.0f),
    _reverseDirection(false),
    _outputDrift(0.0f),
    _output(0.0f),
    _outputMinimum(0.0f),
    _outputMaximum(255.0f),
    _errorSumAndKi(0.0f),
    _target(0.0f),
    _lastInput(0.0f),
    _sampleTime(-2.0f),
    _lastCalculation(0)
{
}


void PIDController::begin(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}


void PIDController::setOutputLimits(float minimum, float maximum)
{
    _outputMinimum = minimum;
    _outputMaximum = maximum;
}


void PIDController::setOutputReverse(bool isReverse)
{
    if (_reverseDirection != isReverse) {
        _kp = -_kp;
        _ki = -_ki;
        _kd = -_kd;
        _reverseDirection = isReverse;
    }
}

    
void PIDController::setOutputDrift(float outputDrift)
{
    _outputDrift = outputDrift;
}

    
void PIDController::setOutputValue(float value)
{
    _output = value;
}


float PIDController::getOutputValue() const
{
    return _output;
}


void PIDController::setTargetValue(float value)
{
    _target = value;
}


float PIDController::calculateOutput(float input)
{
    // Check if we already have a sample time measurement.
    if (_sampleTime < -1.0f) {
        // No, just store the time point, input value and keep the current output value.
        _lastCalculation = millis();
        _lastInput = input;
        _errorSumAndKi = _output;
        _sampleTime = -1.0f;
        return _output;
    }
    
    // Update the sample time.
    const float newSampleTime = static_cast<float>(millis() - _lastCalculation)/1000.0f;
    if (_sampleTime < 0.0f) {
        _sampleTime = (_sampleTime + newSampleTime)/2.0f;
    } else {
        _sampleTime = newSampleTime;
    }
    
    // Calculate the new output value.
    const float kp = _kp;
    const float ki = _ki * _sampleTime; // Make sure our tuning values change on timing.
    const float kd = _kd / _sampleTime;
    const float error = _target - input; // Get the current error
    _errorSumAndKi += (ki * error); // Add to the error sum and integrate the Ki value into it.
    _errorSumAndKi += _outputDrift; // Add the output drift to the error sum and ki value.
    // Limit the error sum (*Ki) to values in the given limits of the output.
    if (_errorSumAndKi > _outputMaximum) {
      _errorSumAndKi = _outputMaximum;
    } else if (_errorSumAndKi < _outputMinimum) {
      _errorSumAndKi = _outputMinimum;
    }
    const float inputDelta = input - _lastInput; // Get the delat of the input.
    const float newOutput = (kp*error) + _errorSumAndKi - (kd*inputDelta);
    if (newOutput > _outputMaximum) {
        _output = _outputMaximum;
    } else if (newOutput < _outputMinimum) {
        _output = _outputMinimum;
    } else {
        _output = newOutput;
    }
    
    _lastCalculation = millis();
    _lastInput = input;
    return _output;
}
    
    
}



