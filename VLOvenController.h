/*! \file 
 *  \brief Oven controller implementation class declarations.
 *  This file declares the classes and types implementing the oven controller.
 *
 *  This file is free software; you can redistribute it and/or modify
 *  it under the terms of GNU Lesser General Public License version 3.0,
 *  as published by the Free Software Foundation.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#ifndef  _VLOvenController_h_
#define  _VLOvenController_h_

#include <arduino.h>
#include <PID_v1.h>
#include "VLOvenShield.h"


#define PID_OUTPUT_LIMIT_MAX      (100.0)       /*!< \brief Upper limit for the PID output. */
#define PID_OUTPUT_LIMIT_MIN      (0.0)         /*!< \brief Lower limit for the PID output. */
#define PID_SAMPLE_TIME           (250)         /*!< \brief Sampling time for the PID in <b>ms</b>. */
#define PROFILE_SAMPLING_TIME     (50)          /*!< \brief Sampling time for temperature profile generator in <b>ms</b>. */
#define TEMPLOGSAMPLING_TIME      (500)         /*!< \brief Temperature reporting time while the oven controller is idle. */

#define MAX_PHASENAME_LEN         (10+1)        /*!< \brief Maximum number of chars for storing profile phase names. */
#define MAXIMUM_TEMPERATURE_SLOPE 100.0         /*!< \brief Absolute maximum value for temperature slope specification. */


/*!
 * \brief PID tunning parameters set.
 * This structure stores the PID controller tunning parameter values.
*/
typedef struct {
  double kp;              /*!< \brief PID tunning parameter <b>KP</b>. */
  double ki;              /*!< \brief PID tunning parameter <b>KI</b>. */
  double kd;              /*!< \brief PID tunning parameter <b>KI</b>. */
} PIDTunings_t;


/*!
 * \brief Oven control phase parameters definition.
 * Fields in this structure control how the oven operates during a temperature control phase.
*/

typedef struct {
  /*! \brief User readable name for the phase used for status visualization. */
  char Name[MAX_PHASENAME_LEN];
  
  /*! \brief Final phase temperature in degrees C. */
  double EndTemp;

  /*! \brief Maximum temperature variation slope in degreesC/second. 
      \remarks The slope sign must be in accordance with the initial and final temperature values. A value of \c 0.0 instructs
      the controller to calculate the slope according to specified \b Maximum and \b Minimum temperatures. */
  double Slope;

  /*! \brief Minimum phase duration in seconds.
      \remarks When specified as \c 0 seconds, the temperature controller changes to next phase when the final temperature is reached.
      The value \c -1 instructs the controller to stay in current phase \c indefinitely.*/
  int Duration;
} VLOvenControllerPhase_t;


/*!
 * \brief Oven controller implementation class.
 * This class implements functionalities required for controlling the oven.
*/
class VLOvenController
{
  public :
    /*!
     * \brief Constructor
     * \param shield Reference to the hardware abstraction layer implementation.
     * \param Console Reference to the communications console.
    */
    VLOvenController( VLOvenShield& shield, TextConsole& Console );
    
    /*!
     * \brief Instance initialization method. Should be called once at startup from function #setup().
    */
    void begin();
    
    /*!
     * \brief Cycle per cycle operations implementation method.
     * \remark This method should be called on every call to the #loop() function.
    */
    void doCycle();
    
    
    /*!
     * \brief Get the current process execution state.
     * \return \c true when the oven controller is active, \c false otherwize.
    */
    bool getRuning() { return m_Running; };
    
    /*!
     * \brief Get the current setpoint (requested temperature for the tempearture controller).
     * \return A value indicating the requested oven temperature for the temperature controller.
     * \remarks This value changes over time at a rate defined by #PROFILE_SAMPLING_TIME to follow the temperatuure 
     * envelope established in the phase configuration structure.
    */
    double getSetpoint() { return m_PID_Setpoint; }
    
    
    /*!
     * \brief Get the phase control parameters for the currently active phase, if any.
     * \return the phase control parameters for the currently active phase if the controller is running, \c NULL otherwize.
    */
    const VLOvenControllerPhase_t* getCurrentPhase();
    
    /*!
     * \brief Get the first phase control parameters list for the currently active process, if any.
     * \return the phase control parameters list for the currently active process if the controller is running, \c NULL otherwize.
    */
    const VLOvenControllerPhase_t* getPhases() { return m_lpPhases; };

    
    /*!
     * \brief Sets the phase control parameters list for the current process.
     * \param lpPhases Pointer to the first entry in the list of phase control parameters. Can be \c NULL to force the 
     * oven controller to stop operation.
     * \param Count Number of phases defined in the phases list.
    */
    void setPhases( const VLOvenControllerPhase_t* lpPhases, int Count );
    
    /*!
     * \brief Enables the oven controller for operation.
     * \remarks Prior to enabling operation, the phase control parameters list must be established using the function #setPhases().
     * \return Returns \c true on successful process start, \c false otherwise.
    */
    bool Start();
    
    /*!
     * \brief Stops the current process.
     * This function stops any process started using the function #Start().
    */
    void Stop();

    
    /*!
     * \brief Get the elapsed time interval from process start in \b ms.
     * \return This function returns a value indicating the elapsed time from process start.
     * \remarks This function returns value \c 0 when the oven controller is disabled.
    */
    unsigned long getProcessDuration();
    
    /*!
     * \brief Get the elapsed time interval from current process phase start in \b ms.
     * \return This function returns a value indicating the elapsed time from current process phase start.
     * \remarks This function returns value \c 0 when the oven controller is disabled.
    */
    unsigned long getPhaseDuration();

    /*!
     * \brief Send an asych event with a value indicating the current oven state.
     * \remarks This function must NOT be called when already started sending a console command response.
    */
    void SendOvenState();

    /*!
     * \brief Send a text message containing information describing the oven control phase parameters.
    */
    void SendPhaseInfo( const VLOvenControllerPhase_t* m_lpPhase );

    /*!
     * \brief Send an asych event with a value indicating the current temperature value.
     * \remarks This function must NOT be called when already started sending a console command response.
    */
    void SendTemperatureSensorState();

    /*!
     * \brief Set control paramters for the PID controller.
     * \param kp Proportional parameter.
     * \param ki Integral parameter.
     * \param kd Differential parameter.
    */
    void SetPIDTunings( double kp, double ki, double kd );

  private :
    bool m_Running;                                           /*!< General status flag, indicates whether the controller is running or not. */
    TextConsole& m_Console;                                   /*!< Reference to remote PC console interface */
    VLOvenShield&  m_Shield;                                /*!< Reference to the hardware abstraction layer implementation. */
    PID m_PID;                                                /*!< PID controller implementation instance. */
    const VLOvenControllerPhase_t* m_lpPhases;              /*!< Pointer to the first entry in the list of phase control parameters. */
    int m_PhasesCount;                                        /*!< Configured phases count */
    int m_CurrentPhase;                                       /*!< Index to current phase control parameters into the phases list. */
    double m_PID_Setpoint;                                    /*!< PID setpoint, requested oven temperature.*/
    double m_PID_Input;                                       /*!< Input value for the PID controller, read from the shield temperature sensor using function #VLOvenShield::readTC(). */
    double m_PID_Output;                                      /*!< Output value from the PID controller, used for commanding the SSR duty cycle. */
    double m_Slope;                                           /*!< Current temperature profile envelope slope. */
    unsigned long m_PhaseStartTime;                           /*!< Time of current phase start, undefined if #m_Running is \c false. */
    unsigned long m_ProcessStartTime;                         /*!< Time of process start, undefined if #m_Running is \c false. */
    unsigned long m_ProfileSampleTime;                        /*!< Time of previous profile sampling. */
    unsigned long m_TemperatureSampleTime;                    /*!< Time of previous temperature log sampling. */
    PIDTunings_t m_PIDTunings;                                /*!< Control parameters for the PID controller. */
    double m_StartTemp;                                       /*!< Buffer for storing the temperature value at which current phase started. */

    /*!
     * \brief Setup controller parameters for executing a process phase.
     * This function configures the oven controller with parameters defining a process control phase.
     * \param lpPhase Pointer to phase control parameters. Can be \c NULL to force the
     * oven controller to stop operation.
    */
    void startPhase( int PhaseIndex );
};

#endif  /* _VLOvenController_h_ */

