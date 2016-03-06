/*! \file
    \brief Oven controller implementation.
    This file implements the class methods for the oven controller class.

    This file is free software; you can redistribute it and/or modify
    it under the terms of GNU Lesser General Public License version 3.0,
    as published by the Free Software Foundation.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <arduino.h>
#include <TextConsole.h>
#include <math.h>
#include "utils.h"
#include "VLOvenController.h"



VLOvenController::VLOvenController( VLOvenShield& shield, TextConsole& Console ) :
  m_Shield( shield ),
  m_Console( Console ),
  m_lpPhases( NULL ), m_CurrentPhase( 0 ), m_PhasesCount( 0 ),
  m_Running( false ),
  m_PID( &m_PID_Input, &m_PID_Output, &m_PID_Setpoint, 0, 0, 0, DIRECT )
{}


void VLOvenController::setPhases( const VLOvenControllerPhase_t* lpPhases, int Count )
{
  Stop();
  m_lpPhases = lpPhases;
  m_CurrentPhase = 0;
  m_Shield.setHeaterDuty( 0.0 );
  m_PhasesCount = Count;
  m_Running = false;
}


void VLOvenController::begin()
{
  m_Shield.getLCD().begin( 20, 4 );
  m_Shield.getLCD().noAutoscroll();
  m_Shield.getLCD().clear();
  m_Shield.getLCD().setCursor( 0, 0 );
  m_Shield.getLCD().print( F("--------------------") );
  m_Shield.getLCD().setCursor( 0, 1 );
  m_Shield.getLCD().print( F("Temperature Cntrollr") );
  m_Shield.getLCD().setCursor( 0, 2 );
  m_Shield.getLCD().print( F("--------------------") );
  m_Shield.getLCD().setCursor( 0, 3 );
  m_Shield.getLCD().print( F("V1.0 - VictorL 2015") );

  delay( 1000 );
  m_Shield.getLCD().clear();
}


const VLOvenControllerPhase_t* VLOvenController::getCurrentPhase()
{
  return ((m_CurrentPhase >= 0) && (m_CurrentPhase < m_PhasesCount)) ? &m_lpPhases[ m_CurrentPhase ] : (const VLOvenControllerPhase_t*)NULL;
}


void VLOvenController::SetPIDTunings( double kp, double ki, double kd )
{
  m_PIDTunings.kp = kp;
  m_PIDTunings.kd = kd;
  m_PIDTunings.ki = ki;
}


void VLOvenController::startPhase( int PhaseIndex )
{
  const VLOvenControllerPhase_t* lpCurrentPhase;

  if ((PhaseIndex < 0) || (PhaseIndex >= m_PhasesCount))
  {
    // End of process;
    m_Running = false;
    m_CurrentPhase = -1;
    SendOvenState();
    return;
  }

  m_CurrentPhase = PhaseIndex;
  lpCurrentPhase = &m_lpPhases[ m_CurrentPhase ];
  m_StartTemp = m_Shield.readTC();

  // Configure profile envelope generation parameters.
  if (lpCurrentPhase->Slope > 0.0)
    m_Slope = lpCurrentPhase->Slope;
  else if (lpCurrentPhase->Duration > 0)
    m_Slope = (lpCurrentPhase->EndTemp - m_StartTemp) / ((double)lpCurrentPhase->Duration);
  else
    m_Slope = lpCurrentPhase->EndTemp > m_StartTemp ? MAXIMUM_TEMPERATURE_SLOPE : -MAXIMUM_TEMPERATURE_SLOPE;

  // The objective is to follow the profile envelope,
  // it should not be a problem if current temperature is above the initial temperature
  m_PID_Setpoint = m_StartTemp;

  // Configure the PID controller.
  m_PID.SetOutputLimits( PID_OUTPUT_LIMIT_MIN, PID_OUTPUT_LIMIT_MAX );
  m_PID.SetSampleTime( PID_SAMPLE_TIME );
  m_PID.SetTunings( m_PIDTunings.kp, m_PIDTunings.ki, m_PIDTunings.kd );

  // Turn the PID on
  m_PID.SetMode( AUTOMATIC );

  m_PhaseStartTime = millis();
  m_ProfileSampleTime = m_PhaseStartTime;
  
  m_Console.beginEvent();
  SendPhaseInfo( lpCurrentPhase );
  m_Console.endEvent();
}


bool VLOvenController::Start()
{
  if (!m_Running && (m_lpPhases != NULL))
  {
    m_ProcessStartTime = millis();
    startPhase( 0 );

    m_Running = true;
    SendOvenState();
  }
  return m_Running;
}


unsigned long VLOvenController::getProcessDuration()
{
  if (m_Running)
    return (millis() - m_ProcessStartTime);
  else
    return 0;
}


unsigned long VLOvenController::getPhaseDuration()
{
  if (m_Running)
    return (millis() - m_PhaseStartTime);
  else
    return 0;
}


void VLOvenController::SendTemperatureSensorState()
{
  m_Console.beginEvent();
  m_Console.send( F("temp[st=") );
  m_Console.send( millis() );
  m_Console.send( F(",lpt=") );
  m_Console.send( m_ProcessStartTime );
  m_Console.send( F(",tmp=") );
  m_Console.send( m_Shield.readTC() );
  m_Console.send( F("]") );
  m_Console.endEvent();
}


void VLOvenController::SendOvenState()
{
  m_Console.beginEvent();
  if (m_Running)
    m_Console.send( F("oven[on=1]") );
  else
    m_Console.send( F("oven[on=0]") );
  m_Console.endEvent();
}


void VLOvenController::SendPhaseInfo( const VLOvenControllerPhase_t* lpPhase )
{
  m_Console.send( F("phase[nam=\"") );
  if (lpPhase) {
    m_Console.send( lpPhase->Name );
    m_Console.send( F("\",end=") );
    m_Console.send( lpPhase->EndTemp );
    m_Console.send( F(",m=") );
    m_Console.send( lpPhase->Slope );
    m_Console.send( F(",t=") );
    m_Console.send( lpPhase->Duration );
    m_Console.send( F("]") );
  }
  else
    m_Console.send( F("\"]") );
}


void VLOvenController::Stop()
{
  // Turn the PID off.
  m_PID.SetMode( MANUAL );
  m_Shield.setHeaterDuty( 0.0 );
  m_Running = false;
  SendOvenState();
}


void VLOvenController::doCycle()
{
  unsigned long Now;
  unsigned long ElapsedPhaseTime;
  PressedKeyCode_t Key;
  const VLOvenControllerPhase_t* lpCurrentPhase;

  m_Shield.doCycle();

  if (m_Running)
  {
    Now = millis();
    ElapsedPhaseTime = Now - m_PhaseStartTime;

    /* Read current temperature value */
    m_PID_Input = m_Shield.readTC();

    //if (0 == (ElapsedPhaseTime % PROFILE_SAMPLING_TIME))
    if (PROFILE_SAMPLING_TIME <= (Now - m_ProfileSampleTime))
    {
      lpCurrentPhase = &m_lpPhases[ m_CurrentPhase ];
      m_ProfileSampleTime = Now;
      if (m_Slope != 0.0)
      {
        /* Adjust the setpoint for following the profile envelope */
        m_PID_Setpoint = m_StartTemp + m_Slope * ((double)ElapsedPhaseTime / 1000.0);
        if (m_StartTemp < lpCurrentPhase->EndTemp)
        {
          if (m_PID_Setpoint > lpCurrentPhase->EndTemp)
          {
            m_PID_Setpoint = lpCurrentPhase->EndTemp;
            m_Slope = 0.0;
          }
        }
        else if (m_StartTemp > lpCurrentPhase->EndTemp)
        {
          if (m_PID_Setpoint < lpCurrentPhase->EndTemp)
          {
            m_PID_Setpoint = lpCurrentPhase->EndTemp;
            m_Slope = 0.0;
          }
        }
      }

      if (m_Slope == 0.0)
      {
        if (
          /* Phase duration reached */
          (
            (lpCurrentPhase->Duration > 0) &&
            (getPhaseDuration() / 1000 >= lpCurrentPhase->Duration)
          ) ||
          (
            (lpCurrentPhase->Duration == 0) &&
            (
              /* Phase end temperature reached */
              ((m_StartTemp <= lpCurrentPhase->EndTemp) && (m_PID_Input >= lpCurrentPhase->EndTemp)) ||
              ((m_StartTemp >= lpCurrentPhase->EndTemp) && (m_PID_Input <= lpCurrentPhase->EndTemp))
            )
          )
        ) {
          startPhase( ++m_CurrentPhase );
        }
      }
    }

    /* Let the PID controller do its job */
    if (m_PID.Compute())
    {
      /* Handle the SSR */
      m_Shield.setHeaterDuty( m_PID_Output );

      m_Console.beginEvent();
      m_Console.send( F("pid[pdt=") );
      m_Console.send( getProcessDuration() );
      m_Console.send( F(",tmp=") );
      m_Console.send( m_PID_Input );
      m_Console.send( F(",slp=") );
      m_Console.send( m_Slope );
      m_Console.send( F(",spt=") );
      m_Console.send( m_PID_Setpoint );
      m_Console.send( F(",out=") );
      m_Console.send( m_PID_Output );
      m_Console.send( F("]") );

      m_Console.endEvent();
    }
  }
  else
  {
    if (TEMPLOGSAMPLING_TIME <= (millis() - m_TemperatureSampleTime))
    {
      m_TemperatureSampleTime = millis();
      //SendTemperatureSensorState();
    }
  }
}
