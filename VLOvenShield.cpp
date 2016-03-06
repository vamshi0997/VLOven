/*! \file 
 *  \brief Oven controller shield abstraction layer.
 *  This file implements the class methods for the oven controller shield hardware abstraction layer classes.
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

#include "VLOvenShield.h"


VLOvenShield::VLOvenShield() : 
  m_Led1( PIN_LED1 ), //m_Led2( PIN_LED2 ),
  m_Lcd( PORT_LCD_PIN_RS, PORT_LCD_PIN_RW, PORT_LCD_PIN_EN, PORT_LCD_PIN_DB4, PORT_LCD_PIN_DB5, PORT_LCD_PIN_DB6, PORT_LCD_PIN_DB7 ),
  m_Keys( { GPIOKey(PIN_KEY_OK, (uint8_t)KEYPRESS_OK ), GPIOKey( PIN_KEY_CANCEL, (uint8_t)KEYPRESS_CANCEL ), GPIOKey( PIN_KEY_UP, (uint8_t)KEYPRESS_UP ), GPIOKey( PIN_KEY_DOWN, (uint8_t)KEYPRESS_DOWN ) } ),
  m_SSR( PIN_SSR, HEATER_PERIODE ),
  m_Average( TEMP_AVERAGING_SAMPLES )
{
  m_Lcd.begin( 20, 4 );
  m_Led1.off();
  //m_Led2.off();
  m_TempSampleTime = millis();
  m_Average.clear();
  analogReference( ADC_REFERENCE );
}


PressedKeyCode_t VLOvenShield::checkKeys()
{
  for (int Index = 0; Index < sizeof(m_Keys) / sizeof(m_Keys[0]); Index++)
  {
    if (m_Keys[ Index ].Check() == GPIOKEYRELEASED)
    {
      if (m_Keys[ Index ].keyPressDuration() >= 50)
        return (PressedKeyCode_t)m_Keys[ Index ].keyCode();
      else
        return NO_KEY;
    }
  }
  
  return NO_KEY;
}


void VLOvenShield::setHeaterDuty( double Duty )
{
  m_SSR.setDutyCycle( Duty );
}


void VLOvenShield::doCycle()
{
  m_Led1.update();
  m_SSR.update();
  //m_Led2.update();

  if (TEMP_SAMPLING_TIME <= (millis() - m_TempSampleTime))
  {
    m_Average.addValue( (float)(analogRead( PORT_TEMP_SONDE ) & (~AD_READINGMASK)) * (float)ADC_REFVOLTAGE / (float)(ADC_FULLSCALE) / 5e-3 );
    m_TempSampleTime = millis();
  }

  m_TempAccumulator += analogRead( PORT_TEMP_SONDE );
  m_TempSamplesCount++;
}


float VLOvenShield::readTC()
{
  //return m_TempSample;
  
  float Result = 0.0;
  int Count;

  return m_Average.getAverage();
}

