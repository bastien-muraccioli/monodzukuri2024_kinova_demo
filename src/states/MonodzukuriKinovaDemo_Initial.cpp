#include "MonodzukuriKinovaDemo_Initial.h"

#include "../MonodzukuriKinovaDemo.h"

void MonodzukuriKinovaDemo_Initial::configure(const mc_rtc::Configuration & config) {}

void MonodzukuriKinovaDemo_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
}

bool MonodzukuriKinovaDemo_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  output("OK");
  return true;
}

void MonodzukuriKinovaDemo_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_Initial", MonodzukuriKinovaDemo_Initial)
