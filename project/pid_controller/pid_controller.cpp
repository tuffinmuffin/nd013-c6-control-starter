/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

#include <bits/stdc++.h>
#include <boost/algorithm/clamp.hpp>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   kp_ = Kpi;
   ki_ = Kii;
   kd_ = Kdi;
   output_lim_max_ = output_lim_maxi;
   output_lim_min_ = output_lim_mini;

   //reset internal state
   culm_err_ = prev_err_ = diff_err_ = 0.0;
   dt_ = 0.0;
}


void PID::UpdateError(double cte) {
   if(dt_ < 1e-8) {
      //drop d term if dt too small.
      diff_err_ = 0.0;
   } else {
      diff_err_ = (cte - prev_err_) / dt_;
   }
   
   culm_err_ += cte * dt_;
   prev_err_ = cte;
}

double PID::TotalError() {
   double control = kp_ * prev_err_ + ki_ * culm_err_ + kd_ * diff_err_;

   return boost::algorithm::clamp(control, output_lim_min_, output_lim_max_);
}

double PID::UpdateDeltaTime(double new_delta_time) {
   double dt_prev = dt_;
   dt_ = new_delta_time;
   //unclear what this wants returned. Assuming it was previous delta time. Or it could be "new_delta_time" is the new time and we are supposed to calc the delta...
   return dt_prev;
}