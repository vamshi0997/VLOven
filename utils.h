/*! \file 
 *  \brief Utility functions.
 *  This file declares some utility functions.
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
#ifndef  _utils_h_
#define  _utils_h_


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
extern void formatFloat( char* lpBuffer, int MaxLength, float Value, int Length, int Decimals, char* MagnitudeName );

#endif  /* _utils_h_ */

