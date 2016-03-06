/*! \file 
 *  \brief Utility functions.
 *  This file implements some utility functions.
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

#include <stdio.h>
#include <string.h>
#include "utils.h"


static const char* UNITNAMES[] = {
  "",
  "m",
  "u",
  "n"
};


/*!
 * \brief Float value to string conversion.
 * This functions implements conversion from float to string.
 * 
 * \param lpBuffer Address for the destination buffer.
 * \param MaxLength Destination buffer size in bytes.
 * \param Value Value to convert to string.
 * \param Length Minimum length of the resulting string.
 * \param Decimals Number of digits after the decimal point.
 * \param MagnitudeName Commodity string containing the magnitude name for the value to convert to string.
*/
void formatFloat( char* lpBuffer, int MaxLength, float Value, int Length, int Decimals, char* MagnitudeName )
{
  char* Sign;
  int  Count;
  char* lpFormat = "%s%d.%03d%s%s";

  if (Value >= 0)
    Sign = "+";
  else
  {
    Sign = "-";
    Value = -Value;
  }

  for (Count = 0; (Count < 3) && (Value < 1.0); Count++)
  {
    Value *= 1000.0;
  }

  int Dec = 1;

  lpFormat[7] = '0' + Decimals;

  while (Decimals-- > 0)
  {
    Dec *= 10;
  }

  int Remainder = ((int)(Value * (float)Dec)) % Dec;
  if (Remainder < 0)
    Remainder = 0;

  snprintf( lpBuffer, MaxLength, lpFormat, Sign, (int)Value, Remainder, UNITNAMES[Count], MagnitudeName );
  Count = strlen( lpBuffer );
  while ((Count < Length) && (Count < MaxLength))
  {
    lpBuffer[Count++] = ' ';
  }
  lpBuffer[Count] = '\0';
}


