/*! \file 
 *  \brief Oven controller shield abstraction layer.
 *  This file declares the class methods for the oven controller shield hardware abstraction layer.
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

#ifndef  _VLOvenShield_h_
#define _VLOvenShield_h_

#include <arduino.h>
#include <inttypes.h>
#include "utils.h"
#include <LiquidCrystal.h>
#include <PinChangeInt.h>
#include <GPIOKey.h>
#include <GPIOLed.h>
#include <GPIOToggler.h>
#include "RunningAverage.h"



#define ADC_FULLSCALE           (1023)    /*!< \brief ADC full scale value (decimal). */
#define ADC_REFERENCE           INTERNAL  /**/

#if (ADC_REFERENCE == INTERNAL)
# define ADC_REFVOLTAGE         (1.1)     /*!< \brief ADC reference voltaje. */
#else
# define ADC_REFVOLTAGE         (5.0)     /*!< \brief ADC reference voltaje. */
#endif

#define AD_READINGMASK          0         /*!< \brief Number of bits to mask from the resulting digital ADC reading. */

#define LINE_FREQUENCY          50
#define TEMP_SAMPLING_TIME      10
#define TEMP_AVERAGING_SAMPLES  100       /*!< \brief Number of temperature sensor reading samples to average. */

#define PORT_TEMP_SONDE         A0        /*!< \brief Pin connected to the temperature sonde amplifier's output. */
#define PORT_LCD_PIN_DB7        A2        /*!< \brief LCD data bus bit 7. */
#define PORT_LCD_PIN_DB6        A3        /*!< \brief LCD data bus bit 6. */
#define PORT_LCD_PIN_DB5        A4        /*!< \brief LCD data bus bit 5. */
#define PORT_LCD_PIN_DB4        A5        /*!< \brief LCD data bus bit 4. */

#define PORT_LCD_PIN_RS         2         /*!< \brief LCD control signal RS. */
#define PORT_LCD_PIN_RW         3         /*!< \brief LCD control signal RW. */
#define PORT_LCD_PIN_EN         4         /*!< \brief LCD control signal EN. */

#define PIN_KEY_OK              5         /*!< \brief Input pin for the OK key switch. */
#define PIN_KEY_CANCEL          8         /*!< \brief Input pin for the CANCEL key switch. */
#define PIN_KEY_UP              7         /*!< \brief Input pin for the UP key switch. */
#define PIN_KEY_DOWN            6         /*!< \brief Input pin for the DOWN key switch. */

#define PIN_LED1                9         /*!< \brief Output pin for the status indicator LED (1) */
//#define PIN_LED2              10        /*!< \brief Output pin for the status indicator LED (2). */

#define PIN_SSR                 10        /*!< \brief Output pin for the SSR control input. */

#define HEATER_PERIODE          250       /*!< \brief Periode for SSR duty cicle control. */


/*! 
 * \brief Unsigned integer alias.
 * Alias for the unsigned integral type.
*/
typedef unsigned int uint_t;


/*!
 * \brief Codes assigned internally to detected keystrokes.
 * This codes represent the equivalent of keystrokes in the keyboard.
*/
typedef enum {
  NO_KEY,               /*!< \brief No key has been pressed. */
  KEYPRESS_CANCEL,      /*!< \brief CANCEL key keypress detected. */
  KEYPRESS_UP,          /*!< \brief UP key keypress detected. */
  KEYPRESS_DOWN,        /*!< \brief DOWN key keypress detected. */
  KEYPRESS_OK           /*!< \brief OK key keypress detected. */
} PressedKeyCode_t;


/*!
 * \brief Oven controller shield hardware abstraction.
 * This class creates the abstraction layer for accessing the oven controller shield from the application.
*/
class VLOvenShield
{
  public:
    /*!
     * \brief Constructor.
    */
    VLOvenShield();
    
    /*!
     * \brief Method for accessing the LCD control instance.
     * \return Returns a reference to the instance of the class that controls the LCD.
    */
    LiquidCrystal& getLCD() { return m_Lcd; }

    /*!
     * \brief Cycle per cycle operations implementation method.
     * \remark This method should be called on every call to the #loop() function.
    */
    void doCycle();

    /*!
     * \brief Keys checking function.
     * \return Returns a code representing keypress events.
    */
    PressedKeyCode_t checkKeys();

    /*!
     * \brief Temperature sensor reading function.
     * \return Returns the temperature measurement result. Results are expected in Degree Celsius.
    */
    float readTC();

    /*!
     * \brief Heater SSR duty cycle control.
     * This functions controls the activation, deactivation and duty cycle of the SSR controlling the heater.
     * \b Minimum value \c 0.0 disables the heater. \b Maximum value \c 100.0 puts the heater in \b ON mode. 
     * Any value above \c 0.0 and below \c 100.0 activates the heater with the corresponding duty cycle.
    */
    void setHeaterDuty( double Duty );

    /*!
     * \brief Method for accessing the Led (1) indicator control instance.
     * \return Returns a reference to the instance of the class that controls the Led indicator (1).
    */
    GPIOLed& getLed1() { return m_Led1; }

    /* !
     * \brief Method for accessing the Led (2) indicator control instance.
     * \return Returns a reference to the instance of the class that controls the Led indicator (2).
    */
    //GPIOLed& getLed2() { return m_Led2; }
    
  private:
    GPIOKey m_Keys[4];              /*!< \brief List of key managing instances. */
    GPIOLed m_Led1;                 /*!< \brief Led (1) managing instance. */
    //GPIOLed m_Led2;                 /*!< \brief Led (2) managing instance. */
    LiquidCrystal m_Lcd;            /*!< \brief LCD managing instance. */
    GPIOToggler m_SSR;              /*!< \brief SSR managing instance. */
    unsigned long m_TempSampleTime;
    long m_TempAccumulator;
    int m_TempSamplesCount;
    float m_TempSample;
    RunningAverage m_Average;
};


#endif  /* _VLOvenShield_h_ */


