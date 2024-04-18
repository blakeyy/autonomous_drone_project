/*
 * PidController.cpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#include <arp/PidController.hpp>
#include <stdexcept>

namespace arp {

// Set the controller parameters
void PidController::setParameters(const Parameters & parameters)
{
  parameters_ = parameters;
}

// Implements the controller as u(t)=c(e(t))
double PidController::control(uint64_t timestampMicroseconds, double e,
                              double e_dot)
{
  // Compute the output of the controller.
  double output = parameters_.k_p * e + parameters_.k_i * integratedError_ + parameters_.k_d * e_dot;

  // Compute time delta and convert from micro-seconds to seconds.
  double time_delta_in_sec = (timestampMicroseconds - lastTimestampMicroseconds_)*1.0e-6;

  if (output < minOutput_) {
    output = minOutput_;   // clamp & DO NOT INTEGRATE ERROR (anti-reset windup)
  } else if (output > maxOutput_) {
    output = maxOutput_;   // clamp & DO NOT INTEGRATE ERROR (anti-reset windup)
  } else if (time_delta_in_sec < 0.1) {
    // When the automatic controller is off for a while and called again after, 
    // the time delta might be huge and integrating the error signal with that 
    // huge delta would be catastrophic. So limit the maximum time delta allowed 
    // for integrating to something like 0.1s.
    integratedError_ += e * time_delta_in_sec; // Integrate the error.
  }

  // Store timestamp for next control cycle.
  lastTimestampMicroseconds_ = timestampMicroseconds;
  
  return output;
}

// Set output limits
void PidController::setOutputLimits(double minOutput, double maxOutput)
{
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

// Reset the integrator to zero again.
void PidController::resetIntegrator()
{
  integratedError_ = 0.0;
}

}  // namespace arp
