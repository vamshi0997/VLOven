/*! \file 
 *  \brief Main project file.
 *  This file implements the application glue logic and the user interface.
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
#include <arduino.h>
#include <TextConsole.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <SoftReset.h>
#include "utils.h"
#include "VLOvenShield.h"
#include "VLOvenController.h"

/*!
 * \mainpage VLOven Oven Controller Documentation.
 * \section pageTOC Content
 * -# @ref Description
 * -# @ref Disclaimer
 * -# @ref Credits
 * -# @ref Dependencies
 * -# @ref Other_tools
 * -# @ref Companion_software
 * -# @ref Hardware
 * -# @ref License
 * 
 * \section Description
 * This Arduino application implements an oven controller capable of working with
 * several temperature control profiles. By default, two temperature control profiles are 
 * registered on first program run: <b> Pb-Free Reflow Oven</b>
 * and <b>Temperature Controlled Oven</b>.
 * 
 * For modifying the default temperature control profiles, please take a look at function #EEPROMRegisterDefaultProfiles().
 * 
 * \section Disclaimer
 * 
 * Dealing with high voltage is a very dangerous act! Please make sure you know
 * what you are dealing with and have proper knowledge before hand. Your use of 
 * any information or materials on this reflow oven controller is entirely at 
 * your own risk, for which we shall not be liable. 
 * 
 * \section Credits
 * This project was inspired in some ways by the <b>[Reflow Oven Controller]
 * (https://github.com/rocketscream/Reflow-Oven-Controller.git)</b> project
 * by <b>Lim Phang Moh</b> from [<b>Rocket Scream Electronics</b>](www.rocketscream.com).
 * 
 * Main temperature control component is implemented using the <b>[Arduino PID  Library]
 * (https://github.com/br3ttb/Arduino-PID-Library.git)</b>
 * by <b>[Brett Beauregard](www.brettbeauregard.com)</b>.
 * 
 * \section Dependencies
 * This software makes use of following independent libraries, some of which are not part of Arduino:
 * -# [Arduino PID  Library] (https://github.com/br3ttb/Arduino-PID-Library.git)
 * by <b>[Brett Beauregard](www.brettbeauregard.com)</b>.
 * -# [GPIOKey Library] (https://github.com/VLorz/GPIOKey.git) by Victor Lorenzo (EDesignsForge).
 * -# [GPIOToggler Library] (https://github.com/VLorz/GPIOToggler.git) by Victor Lorenzo (EDesignsForge).
 * -# [TextConsole Library] (https://github.com/VLorz/TextConsole.git) by Victor Lorenzo (EDesignsForge).
 * -# [GPIOLed Library] (https://github.com/VLorz/GPIOLed.git) by Victor Lorenzo (EDesignsForge). This library
 * was used just for convenience as its functionality is only a subset from the GPIOToggler's library functionality.
 * -# [RunningAverage library for Arduino] (https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningAverage) by Rob Tillaart.
 * -# [SoftReset software reset library for Arduino] (https://github.com/WickedDevice/SoftReset.git) by WickedDevice.
 * 
 * \section Other_tools Other required tools
 * The source code has been documented using following free/open source tools:
 * -# [doxygen](http://www.doxygen.org/) version 1.8.10. The doxygen configuration file is found under project relative path <b>./dox/doxy.dox</b>.
 * -# [Graphviz] (http://www.graphviz.org/) version 2.38.
 * -# [Mscgen] (http://www.mcternan.me.uk/mscgen/) version 0.20.
 * 
 * \section Companion_software Companion software VLOven.Net
 * As a companion software, VLOven.Net project implements a C# Windows application for displaying current oven status and profiles
 * design, modification and planning.
 * 
 * \section Hardware
 * The oven control shield makes room for the following hardware:
 * -# 1 x 4 lines by 20 characters LCD module working in 4-bit mode.
 * -# 4 x keys keyboard.
 * -# 2 x red color indicator leds.
 * -# 1 x 70A, 230V SSR with input control voltage ranging from 5V to 30V.
 *  
 *  All definitions for the hardware abstraction layer (<b>HAL</b>) can be found 
 *  in file VLOvenShield.h
 *  
 * \section License
 * This application and its associated files (referred together 
 * as the SOFTWARE) are free software; you can redistribute it and/or 
 * modify it under the terms of GNU Lesser General Public License version 3.0, 
 * as published by the Free Software Foundation.
 * 
 * This SOFTWARE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 * 
 * \verbinclude lgpl-3.0.txt
*/


#define PROFILE_NAME_LENGTH       (20)            /*!< \brief Number of chars for storing profile names. */
#define EEPROM_SIGNATURE_LENGTH   (9)             /*!< \brief Number of chars for storing the EEPROM signature. */

#define EEPROM_SIGNATURE_OFFSET   0               /*!< \brief EEPROM location of the EEPROM signature. */
#define EEPROM_APPDATA_OFFSET     (EEPROM_SIGNATURE_OFFSET + sizeof(EEPROMSignature_t)) /*!< \brief EEPROM location for the application non-volatile data. */


/*!
 * \brief Signature for the EEPROM.
 * This structure holds the "magic" mark that serves for validating that EEPROM contains valid application data.
 */
typedef struct 
{
  char Signature[ EEPROM_SIGNATURE_LENGTH ];      /*!< \brief EEPROM data signature. */
} EEPROMSignature_t;


/*!
 * \brief Header containing basic information for the temperature control profile.
 * This structure holds the basic information required for a temperature control profile.
 */

typedef struct
{
  char Name[ PROFILE_NAME_LENGTH ];               /*!< \brief Meaningful name for the temperature control profile. */
  int PhasesCount;                                /*!< \brief Number of phases conforming the temperature control profile. */
} ProfileHeader_t;


/*!
 * \brief Temperature control profile definition.
 * This structure holds basic información required for defining a temperature
 * control profile.
 */
typedef struct
{
  /*!
   * \brief Header containing basic information for the temperature control profile.
  */
  ProfileHeader_t Header;
  
  /*! 
   * \brief List of phases conforming the temperature control profile. 
   * The list of phase definitions given here will define the temperature control profile. 
   */
  VLOvenControllerPhase_t*  lpPhases;

  /*!
   * \brief EEPROM location where the profile resides.
  */
  short EEPromOffset;
} ProfileInfo_t;


/*!
 * \brief Default EEPROM signature used for this application.
 */
static const EEPROMSignature_t DefaultSignature =
{
  Signature : { 'V', 'L', 'R', 'e', 'f', 'l', 'o', 'w', '\0' }
};


void CmdHelp( TextConsole* lpSilly );           /*!< Forward Declaration: Handler for '?' interpreter command. */
void CmdReadInput( TextConsole* lpSilly );      /*!< Forward Declaration: Handler for 'i' interpreter command. */
void CmdProfiles( TextConsole* lpSilly );       /*!< Forward Declaration: Handler for 'p' interpreter command. */
void CmdEEPROM( TextConsole* lpSilly );         /*!< Forward Declaration: Handler for 'e' interpreter command. */
void CmdReset( TextConsole* lpSilly );          /*!< Forward Declaration: Handler for 'rst' interpreter command. */


/*! 
 * \brief List of supported commands.
*/
static const struct ConsoleCommandEntry  CommandEntries[] =
{
  { "?",        CmdHelp },
  { "i",        CmdReadInput },
  { "p",        CmdProfiles },
  { "e",        CmdEEPROM },
  { "rst",      CmdReset },
  { NULL,       NULL }
};


/*! \brief Text string reported by command '?' (Help command) when invoked at the text console prompt. */
#define HELP \
  TEXTCONSOLE_EOLN \
  "Available commands:" TEXTCONSOLE_EOLN \
  "  i <input>" TEXTCONSOLE_EOLN \
  "    read current status for input <input>" TEXTCONSOLE_EOLN \
  "  ?" TEXTCONSOLE_EOLN \
  "    this help" TEXTCONSOLE_EOLN
  

/*! \brief Default KP parameter (proportional gain) for the PID controller */
#define PID_KP  300
/*! \brief Default KI parameter (integral gain) for the PID controller */
#define PID_KI  0.05
/*! \brief Default KD parameter (derivative gain) for the PID controller */
#define PID_KD  250

/*! \brief Phases list definition for acting as a reflow oven. 
 * Values provided here configure the oven controller for
 * going through the different phases required for reflow soldering.
*/
static const VLOvenControllerPhase_t  PBFREEREFLOWCONTROLLER_PHASES[] PROGMEM =
{
  {
    Name :          { 'P', 'r', 'e', 'h', 'e', 'a', 't', '-', '1', '\0' }, //"Preheat-1",
    EndTemp :       50.0,
    Slope :         2.0,       /* 2.0ºC/s */
    Duration :      0
  },
  {
    Name :          { 'P', 'r', 'e', 'h', 'e', 'a', 't', '-', '2', '\0' }, //"Preheat-2",
    EndTemp :       150.0,
    Slope :         2.0,       /* 2.0ºC/s */
    Duration :      0
  },
  {
    Name :          { 'S', 'o', 'a', 'k', '-', '1', '\0' }, //"Soak-1",
    EndTemp :       200.0,
    Slope :         0.0,
    Duration :      100
  },
  {
    Name :          { 'S', 'o', 'a', 'k', '-', '2', '\0' }, //"Soak-2",
    EndTemp :       217.0,
    Slope :         2.0,       /* 2.0ºC/s */
    Duration :      0
  },
  {
    Name :          { 'R', 'e', 'f', 'l', 'o', 'w', '-', '1', '\0' }, //"Reflow-1",
    EndTemp :       245.0,
    Slope :         0.0,
    Duration :      20
  },
  {
    Name :          { 'R', 'e', 'f', 'l', 'o', 'w', '-', '1', '\0' }, //"Reflow-2",
    EndTemp :       217.0,
    Slope :         0.0,
    Duration :      20
  },
  {
    Name :          { 'C', 'o', 'o', 'l', 'i', 'n', 'g', '\0' }, //"Cooling",
    EndTemp :       100.0,
    Slope :         -3.0,       /* -3.0ºC/s */
    Duration :      0
  },
  {
    Name :          { 'D', 'o', 'n', 'e', '(', 'H', 'O', 'T', ')', '\0' }, //"Done(HOT)",
    EndTemp :       50.0,
    Slope :         -10.0,       /* -10.0ºC/s */
    Duration :      0
  }
};


/*! \brief Phases list definition for acting as a semistandard oven. 
 * Values provided here, in conjunction with values provided by #OVENCONTROLLER_PROFILEHEADER,
 * configure the oven controller for going through two basic phases: "heating" and "hot", 
 * just like an ordinary temperature controlled oven would.
*/
#if !defined(__DOXYGEN__)
static const VLOvenControllerPhase_t  OVENCONTROLLER_PHASES[] PROGMEM =
#else
static const VLOvenControllerPhase_t  OVENCONTROLLER_PHASES[]=
#endif
{
  {
    Name :          { 'H', 'e', 'a', 't', 'i', 'n', 'g', '\0' },//{"Heating"},
    EndTemp :       50.0,
    Slope :         2.0,       /* 2.0ºC/s */
    Duration :      0
  },
  {
    Name :          { 'H', 'o', 't', '\0' },//{"Hot"},
    EndTemp :       50.0,
    Slope :         0.0,
    Duration :      -1
  }
};


/*! \brief Profile parameters definition for acting as a semistandard oven. 
 * Values provided here, in conjunction with values provided by #OVENCONTROLLER_PHASES,
 * configure the oven controller for going through two basic phases: "heating" and "hot", 
 * just like an ordinary temperature controlled oven would.
*/
#if !defined(__DOXYGEN__)
static const ProfileHeader_t OVENCONTROLLER_PROFILEHEADER PROGMEM = 
#else
static const ProfileHeader_t OVENCONTROLLER_PROFILEHEADER = 
#endif
{
  Name :      { 'O', 'v', 'e', 'n', ' ', 'C', 'o', 'n', 't', 'r', 'o', 'l', 'l', 'e', 'r', '\0' },
  PhasesCount :   sizeof(OVENCONTROLLER_PHASES) / sizeof(OVENCONTROLLER_PHASES[0])
};


static const ProfileHeader_t PBFREEREFLOWCONTROLLER_PROFILEHEADER PROGMEM = {
  Name :          { 'P', 'b', 'F', 'r', 'e', 'e', ' ', '-', ' ', 'R', 'e', 'f', 'l', 'o', 'w', '\0' },
  PhasesCount :   sizeof(PBFREEREFLOWCONTROLLER_PHASES) / sizeof(PBFREEREFLOWCONTROLLER_PHASES[0])
};


char m_ConsoleBuffer[ 64 ];       /*!< \brief Communications buffer. This buffer is used by the commands interpreter for PC command/response interface. */
char m_TextsBuffer[ 64 ];         /*!< \brief Multi-purpose memory buffer. This buffer is used a temporary storage for strings formatting. */

/*! \brief Commands interpreter for PC communication. 
 * This instance allows command/response interchange with a PC user application or serial console. */
TextConsole         m_Console( Serial, m_ConsoleBuffer, sizeof(m_ConsoleBuffer), &CommandEntries[ 0 ] );

/*! \brief Instance of the oven controller Arduino shield abstraction.
 * This instance of the shield abstraction gives access to all oven controlling hardware functionalities.*/
VLOvenShield      m_Shield;

/*!* \brief The Controller instance. 
 * This instance is the oven controller, it implements all the magic done by this application. */
VLOvenController  m_Controller( m_Shield, m_Console );

/*! \brief Current temperature control profile selector. 
 * This variable holds an index into the temperature control profiles list, it points to the currently selected temperature control profile. */
unsigned int        m_CurrentProfileIndex;

/*! \brief Current temperature profile control parameters.
 * This variable holds currently active profile definition parameters. */
ProfileInfo_t       m_ActiveProfile;


/*!
 * \brief Function used when requiring user confirmation.
 * This function is used in the code for asking user confirmation.
 * 
 * \param ifshQuestion  Text to display to the user. This text must be short enough to fit in one single line.
 * \param lpResult  Value returned by the function. The parameter must be initialized with the default return value.
 * \return          The function returns \c true when the user presses the \b OK button and \c false when the user presses the \b CANCEL button.
*/
bool Ask( const __FlashStringHelper *ifshQuestion, bool* lpResult )
{
  bool Result;
  bool printResult;

  m_Shield.getLCD().clear();
  m_Shield.getLCD().setCursor( 0, 1 );
  m_Shield.getLCD().print( ifshQuestion );

  Result = *lpResult;
  printResult = true;
  while (true)
  {
    if (printResult)
    {
      printResult = false;
      m_Shield.getLCD().setCursor( 5, 2 );
      if (Result)
        m_Shield.getLCD().print( F("[YES]  no ") );
      else
        m_Shield.getLCD().print( F(" yes  [NO]") );
    }

    switch (m_Shield.checkKeys())
    {
      case KEYPRESS_UP :
      case KEYPRESS_DOWN :
      {
        printResult = true;
        Result = !Result;
        break;
      }

      case KEYPRESS_CANCEL :
      {
        m_Shield.getLCD().clear();
        return false;
      }

      case KEYPRESS_OK :
      {
        m_Shield.getLCD().clear();
        *lpResult = Result;
        return true;
      }
    }
    
    // Not using any RTOS, so should make sure background operations
    // are given time to run.
    m_Controller.doCycle();
    m_Console.handleInput();
  }
}


/*! 
 * \brief Function used for counting the number of profiles stored in the EEPROM.
 * \return Returns the number of profiles stored in the EEPROM.
*/
int GetProfilesCount()
{
  int Count = 0;
  int Offset = EEPROM_APPDATA_OFFSET;
  ProfileHeader_t Header;

  while (Offset < (EEPROM.length() - sizeof(Header))) {
    EEPROM.get( Offset, Header );

    if (Header.Name[0] == 0)
      break;

    Offset += (sizeof(Header) + Header.PhasesCount * sizeof(VLOvenControllerPhase_t));
    Count++;
  };

  return Count;
}


/*! 
 * \brief Function used for releasing the dynamic memory used for loading one profile.
 * \param Profile Reference to a #ProfileInfo_t variable previously initialized with the 
 * function #AllocProfilePhases().
*/
void FreeProfile( ProfileInfo_t& Profile )
{
  if (Profile.lpPhases) {
    free( Profile.lpPhases );
    Profile.lpPhases = NULL;
    Profile.Header.Name[0] = 0;
  }
}


/*! 
 * \brief Function used for reserving memory for storing the profile temperature control phases information.
 * \param ProfileInfo Reference to a #ProfileInfo_t variable.
 * \return Returns \c TRUE on successful completion, \c FALSE otherwise.
 * \remarks The allocated memory must be released using the function #FreeProfile().
*/
bool AllocProfilePhases( ProfileInfo_t& ProfileInfo )
{
    ProfileInfo.lpPhases = (VLOvenControllerPhase_t*)malloc( ProfileInfo.Header.PhasesCount * sizeof(VLOvenControllerPhase_t) );

    return (ProfileInfo.lpPhases != NULL);
}


/*! 
 * \brief Function used for checking the EEPROM memory signature.
 * \return Returns \c TRUE on successful completion, \c FALSE otherwise.
 * \remarks The EEPROM signature is stored by function #EEPROMFormat() on first program run or
 * by remote text console commands.
*/
bool EEPROMCheckSignature()
{
  EEPROMSignature_t Signature;

  EEPROM.get( EEPROM_SIGNATURE_OFFSET, Signature );

  return (strcmp( Signature.Signature, DefaultSignature.Signature ) == 0);
}


/*! 
 * \brief Function used for formatting the EEPROM memory.
 * \remarks This function MUST be used with CAUTION. This function ERASES ALL THE EEPROM memory.
 * 
*/
void EEPROMFormat()
{
  EEPROM.put( EEPROM_SIGNATURE_OFFSET, DefaultSignature );

  for (int i = EEPROM_APPDATA_OFFSET ; i < EEPROM.length() ; i++) {
    EEPROM.write( i, 0 );
  }
}


/*! 
 * \brief Function used for copying data from SRAM to EEPROM memory.
 * \param lpSrc Buffer address of source data.
 * \param EEOffset EEPROM location (offset) for target data location in EEPROM.
 * \param Count Number of bytes to copy.
 * \remarks This function MUST be used with CAUTION, any prior data at target location WILL BE LOST.
 * 
  */
void CopyToEEPROM( uint8_t* lpSrc, int& EEOffset, int Count )
{
  while (Count)
  {
    EEPROM.write( EEOffset, *lpSrc );
    EEOffset++;
    lpSrc++;
    Count--;
  }
}


/*! 
 * \brief Function used for locating the first free location in EEPROM for storing temperature profiles.
 * \return Returns a positive number containing the offset of the first free byte in EEPROM, or \c -1
 * in case there is no more empty space in EEPROM.
  */
int FindFreeEEPROMStart()
{
  ProfileHeader_t Header;
  int Offset = EEPROM_APPDATA_OFFSET;

  while (Offset < (EEPROM.length() - sizeof(Header))) {
    EEPROM.get( Offset, Header );

    if (Header.Name[0] == 0)
      return Offset;

    Offset += (sizeof(Header) + Header.PhasesCount * sizeof(VLOvenControllerPhase_t));
  };

  // Should never get here, it would mean memory is corrupted.
  return -1;
}


/*! 
 * \brief Function used for appending a new temperature control profile at the end of the 
 * application data EEPROM.
 * \param lpProfile Reference to a variable containting a valid profile data structure.
 * \return Resturns \c TRUE on success, \c FALSE otherwise.
 */
bool EEPROMAppendProfile( ProfileInfo_t* lpProfile )
{
  int Offset;

  Offset = FindFreeEEPROMStart();
  if (Offset <= 0)
    return false;
  
  // >HEADER:
  EEPROM.put( Offset, lpProfile->Header );
  Offset += sizeof(lpProfile->Header);

  // >PHASES:
  CopyToEEPROM( (uint8_t*)lpProfile->lpPhases, Offset, lpProfile->Header.PhasesCount * sizeof(lpProfile->lpPhases[0]) );
}


/*! 
 * \brief Function used for copying data from program FLASH to SRAM.
 * \param dest Target buffer address in SRAM.
 * \param src Source buffer address in program FLASH.
 * \param count Number of bytes to copy.
 * 
  */
void copyPS( char* dest, const char* src, int count )
{
  for (int Index = 0; (Index < count); Index++)
  {
    *dest = pgm_read_byte_near( src + Index );
    dest++;
  }
}


/*! 
 * \brief Function used for registering the default profiles in application data EEPROM.
 * 
  */
void EEPROMRegisterDefaultProfiles()
{
  ProfileInfo_t ProfileInfo;
  
  // STD Oven controller profile.
  copyPS( (char*)&ProfileInfo, (const char*)&OVENCONTROLLER_PROFILEHEADER, sizeof(ProfileInfo) );
  if (AllocProfilePhases( ProfileInfo )) {
    copyPS( (char*)ProfileInfo.lpPhases, (const char*)&OVENCONTROLLER_PHASES, ProfileInfo.Header.PhasesCount * sizeof(VLOvenControllerPhase_t) );

    EEPROMAppendProfile( &ProfileInfo );
    FreeProfile( ProfileInfo );
  }

  // Pb-Free reflow oven controller profile.
  copyPS( (char*)&ProfileInfo, (const char*)&PBFREEREFLOWCONTROLLER_PROFILEHEADER, sizeof(ProfileInfo) );
  if (AllocProfilePhases( ProfileInfo )) {
    copyPS( (char*)ProfileInfo.lpPhases, (const char*)&PBFREEREFLOWCONTROLLER_PHASES, ProfileInfo.Header.PhasesCount * sizeof(VLOvenControllerPhase_t) );

    EEPROMAppendProfile( &ProfileInfo );
    FreeProfile( ProfileInfo );
  }
}


/*! 
 * \brief Function used for loading a profile header from the application data EEPROM.
 * \param Header Variable reference to target profile header buffer.
 * \param ProfileIndex Profile index into EEPROM.
 * \return Returns a positive integer value corresponding the an EEPROM offset where the header was found, \c -1 otherwise.
  */
int LoadProfileHeader( ProfileHeader_t& Header, int ProfileIndex )
{
  if (ProfileIndex >= 0) {
    int Offset = EEPROM_APPDATA_OFFSET;

    while (Offset < (EEPROM.length() - sizeof(Header))) {
      EEPROM.get( Offset, Header );

      if (Header.Name[0] == 0)
        return -1;
      else if (ProfileIndex == 0)
        return Offset;

      Offset += (sizeof(Header) + Header.PhasesCount * sizeof(VLOvenControllerPhase_t));
      ProfileIndex--;
    };
  }

  return -1;
}


bool LoadProfile( ProfileInfo_t& ProfileInfo, int ProfileIndex )
{
  int Offset;
  bool Result = false;
  
  if (ProfileIndex < 0)
    return false;
  
  Offset = LoadProfileHeader( ProfileInfo.Header, ProfileIndex );
      
  if (Offset > 0) 
  {
    if (AllocProfilePhases( ProfileInfo ))
    {
      int Count;
      VLOvenControllerPhase_t* lpPhase;

      lpPhase = ProfileInfo.lpPhases;
      Offset += sizeof(ProfileInfo.Header);
      Count = ProfileInfo.Header.PhasesCount;

      while (Count--)
      {
        EEPROM.get( Offset, *lpPhase );
        Offset += sizeof(*lpPhase);
        lpPhase++;
      }

      Result = true;
    }
    else {
      ProfileInfo.Header.Name[0] = 0;
    }
  }
  
  return Result;
}


/*!
 * \brief Standard Arduino system configuration and setup function.
 * This function is called once by the Arduino startup code during system initialization.
*/
void setup()
{
  bool Result = false;
  m_CurrentProfileIndex = -1;
  m_ActiveProfile.lpPhases = NULL;
  m_ActiveProfile.Header.Name[ 0 ] = 0;
  
  Serial.begin( 115200 );
  m_Console.begin( F("%Reflow oven controller!" TEXTCONSOLE_EOLN) );

  if (!EEPROMCheckSignature() && Ask( F("Wrong EEPROM, init?"), &Result ))
  {
    EEPROMFormat();
    EEPROMRegisterDefaultProfiles();
  }

  if (LoadProfile( m_ActiveProfile, 0 )) {
    m_CurrentProfileIndex = 0;
  }
  
  m_Controller.SetPIDTunings( PID_KP, PID_KI, PID_KD );
  m_Controller.begin();

  // Now we're on control!
  m_Shield.getLed1().on();
}


/*!
 * \brief Utility function for displaying informmation about current temperature control profile.
 * This function is called periodically from the #loop() function.
*/
void updateProfileInfo()
{
  // Temperature control profile info.
  m_Shield.getLCD().setCursor( 0, 0 );
  if (m_ActiveProfile.lpPhases != NULL)
    sprintf( m_TextsBuffer, "%10s", m_ActiveProfile.Header.Name );
  else
    sprintf( m_TextsBuffer, "%10s", "" );
  m_Shield.getLCD().print( m_TextsBuffer );
}


void SendProfileInfo()
{
  m_Console.beginEvent();
  m_Console.send( F("profile[idx=") );
  m_Console.send( m_CurrentProfileIndex );
  m_Console.send( F("]") );
  m_Console.endEvent();
}


void ActivateProfile( ProfileInfo_t Profile )
{
  m_Controller.setPhases( NULL, 0 );
  FreeProfile( m_ActiveProfile );
  memcpy( &m_ActiveProfile, &Profile, sizeof(m_ActiveProfile) );
}


bool ActivateProfile( int ProfileIndex )
{
  if (ProfileIndex >= 0) {
    ProfileInfo_t Profile;
    
    if (LoadProfile( Profile, ProfileIndex )) {
      ActivateProfile( Profile );
      m_CurrentProfileIndex = ProfileIndex;

      return true;
    }
  }

  return false;
}


/*!
 * \brief Utility function for alternating temperature control profiles.
 * This function disables the oven controller and selects the next avaiable temperature control profile.
*/
void setNextProfile()
{
  m_Controller.setPhases( NULL, 0 );
  ActivateProfile( m_CurrentProfileIndex + 1 );
  SendProfileInfo();
}


/*!
 * \brief Utility function for alternating temperature control profile.
 * This function disables the oven controller and selects the previous avaiable temperature control profile.
*/
void setPrevProfile()
{
  m_Controller.setPhases( NULL, 0 );
  ActivateProfile( m_CurrentProfileIndex - 1 );
  SendProfileInfo();
}


/*!
 * \brief Standard Arduino system application loop function.
 * This function is called continuously by the Arduino startup code during system initialization.
*/
void loop()
{
  m_Controller.doCycle();
  
  if (!m_Console.handleInput())
  {
    PressedKeyCode_t  Key;
    const VLOvenControllerPhase_t*  lpPhase;
    const VLOvenControllerPhase_t*  lpPhases;

    lpPhase = m_Controller.getCurrentPhase();
    lpPhases = m_Controller.getPhases();

    bool Disabled = (lpPhase == NULL) || (lpPhase->Name[0] == '\0');
    bool Running = m_Controller.getRuning();
    
    // Handle keyboard events.
    Key = m_Shield.checkKeys();
    if (Key != NO_KEY)
    {
      bool Result = false;
      
      if (Running)
      {
        if ((Key == KEYPRESS_CANCEL) && Ask( F("Disable controller?"), &Result ))
        {
          if (Result)
            m_Controller.setPhases( NULL, 0 );
        }
      }
      else if ((Key == KEYPRESS_OK) && (m_ActiveProfile.lpPhases != NULL) && Ask( F("Enable controller?"), &Result ))
      {
        if (Result)
        {
          m_Controller.setPhases( m_ActiveProfile.lpPhases, m_ActiveProfile.Header.PhasesCount );
          m_Controller.Start();
        }
      }
      else if (Key == KEYPRESS_UP)
      {
        setNextProfile();
      }
      else if (Key == KEYPRESS_DOWN)
      {
        setPrevProfile();
      }
    }

    // Update the LCD on a regular basis, but not too often.
    if (0 == (millis() % 250))
    {

      // Update general information.
      updateProfileInfo();

      // Running or not.
      m_Shield.getLCD().setCursor( 17, 0 );
      if (m_Controller.getRuning())
        m_Shield.getLCD().print( F("ON ") );
      else
        m_Shield.getLCD().print( F("OFF") );

      // Phase info.
      if (Disabled)
        sprintf( m_TextsBuffer, "Phase: %-13s", "DISABLED" );
      else
        sprintf( m_TextsBuffer, "Phase: %-13s", lpPhase->Name );
      m_Shield.getLCD().setCursor( 0, 1 );
      m_Shield.getLCD().print( m_TextsBuffer );

      // Elapsed times
      sprintf( m_TextsBuffer, "PT: %4ds", m_Controller.getPhaseDuration() / 1000 );
      m_Shield.getLCD().setCursor( 0, 2 );
      m_Shield.getLCD().print( m_TextsBuffer );

      sprintf( m_TextsBuffer, "TT: %4ds", m_Controller.getProcessDuration() / 1000 );
      m_Shield.getLCD().setCursor( 10, 2 );
      m_Shield.getLCD().print( m_TextsBuffer );
      
      // Current temperature.
      formatFloat( m_TextsBuffer, sizeof(m_TextsBuffer), m_Shield.readTC(), 10, 1, "C" );
      m_Shield.getLCD().setCursor( 0, 3 );
      m_Shield.getLCD().print( m_TextsBuffer );

      // Setpoint.
      formatFloat( m_TextsBuffer, sizeof(m_TextsBuffer), m_Controller.getSetpoint(), 10, 1, "C" );
      m_Shield.getLCD().setCursor( 10, 3 );
      m_Shield.getLCD().print( m_TextsBuffer );
    }
  }
}


/*!
 * \brief Interpreter command handler: HELP command.
 * This function is called when the commands interpreter receives a request for the HELP command.
*/
void CmdHelp( TextConsole* lpSilly )
{
  lpSilly->sendResponse( CONSOLESUCCESS, F(HELP) );
}


/*!
 * \brief Interpreter command handler: READ INPUT command.
 * This function is called when the commands interpreter receives a request for the READ INPUT command.
*/
void CmdReadInput( TextConsole* lpSilly )
{
  int Index;
  
  if (lpSilly->argsCount() != 1)
  {
    lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGSCOUNT) );
    return;
  }
  
  Index = atoi( lpSilly->getArg( 0 ) );
  int pval = -1;
  int val;

  lpSilly->beginResponse( CONSOLESUCCESS );
  while (!lpSilly->hasNewInput())
  {
    val = digitalRead( Index );
    if (val != pval)
    {
      snprintf( m_ConsoleBuffer, sizeof(m_ConsoleBuffer), "in[%d]=%i;\r\n", Index, val );
      lpSilly->send( m_ConsoleBuffer );
    }

    pval = val;
  }
  
  lpSilly->endResponse();
}


void CmdEEPROM( TextConsole* lpSilly )
{
  if (lpSilly->argsCount() == 0)
  {
   lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGSCOUNT) );
  }
  else if (!strcmp( lpSilly->getArg( 0 ), "inf" ))
  {
    bool SignatureOK;

    SignatureOK = EEPROMCheckSignature();
    
    lpSilly->beginResponse();
    m_Console.send( F("eeprom[sigOk=") );
    m_Console.send( SignatureOK );
    m_Console.send( F(", len=") );
    m_Console.send( EEPROM.length() );
    m_Console.send( F(", freestart=") );
    m_Console.send( FindFreeEEPROMStart() );
    m_Console.send( F("]" ) );
    lpSilly->endResponse( CONSOLESUCCESS );
  }
  else if (!strcmp( lpSilly->getArg( 0 ), "fmt" ))
  {
    EEPROMFormat();
    EEPROMRegisterDefaultProfiles();
    lpSilly->sendResponse( CONSOLESUCCESS );
  }
  else if (!strcmp( lpSilly->getArg( 0 ), "d" ))
  {
    if (lpSilly->argsCount() != 2)  {
     lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGSCOUNT) );
    }
    else {
      int Offset = atoi( lpSilly->getArg( 1 ) );

      EEPROM.get( Offset, m_TextsBuffer );

      lpSilly->beginResponse();
      for (int Index = 0; Index < sizeof(m_TextsBuffer); Index++) {
        char Txt[16];

        if (0 == (Index % 16))
          m_Console.send( F(TEXTCONSOLE_EOLN) );
          
        sprintf( Txt, "%02X ", (unsigned char)m_TextsBuffer[Index] );
        lpSilly->send( Txt );
      }
      lpSilly->endResponse( CONSOLESUCCESS );
    }
  }
  else {
    lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGINVALIDOPT) );
  }
}


void CmdReset( TextConsole* lpSilly )
{
  soft_restart();
}


/*!
 * \brief Interpreter command handler: PROFILES handling command.
 * This function is called when the commands interpreter receives a request for the PROFILES handling command.
*/
void CmdProfiles( TextConsole* lpSilly )
{
  if (lpSilly->argsCount() == 0)
  {
   lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGSCOUNT) );
  }
  else if (!strcmp( lpSilly->getArg( 0 ), "cur" ))
  {
    lpSilly->beginResponse();
    lpSilly->send( m_CurrentProfileIndex );
    lpSilly->endResponse( CONSOLESUCCESS );
  }
  else if (!strcmp( lpSilly->getArg( 0 ), "ls" ))
  {
    int Index;
    ProfileHeader_t Header;

    lpSilly->beginResponse();
    Index = 0;

    while (LoadProfileHeader( Header, Index ) > 0)
    {
      if (Index)
        lpSilly->send( TEXTCONSOLE_EOLN );
        
      lpSilly->send( Header.Name );
      Index++;
    }

    lpSilly->endResponse( CONSOLESUCCESS );
  }
  else if (!strcmp( lpSilly->getArg( 0 ), "sel" ))
  {
    if (lpSilly->argsCount() != 2) {
      lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGSCOUNT) );
    }
    else {
      int ProfileIndex = atoi( lpSilly->getArg( 1 ) );

      /* Disable the controller */
      if (m_Controller.getRuning())
        m_Controller.setPhases( NULL, 0 );

      if (ActivateProfile( ProfileIndex )) {
        lpSilly->sendResponse( CONSOLESUCCESS );
        SendProfileInfo();
      }
      else
        lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGOUTOFRANGE) );
    }
  }
  else if (!strcmp( lpSilly->getArg( 0 ), "on" )) {
    if (lpSilly->argsCount() != 1) {
      lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGSCOUNT) );
    }
    else if (m_ActiveProfile.lpPhases == NULL) {
      lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGINVALIDOPT) );
    }
    else {
      m_Controller.setPhases( m_ActiveProfile.lpPhases, m_ActiveProfile.Header.PhasesCount );
      m_Controller.Start();
      lpSilly->sendResponse( CONSOLESUCCESS );
    }
  }
  else if (!strcmp( lpSilly->getArg( 0 ), "off" )) {
    if (lpSilly->argsCount() != 1) {
      lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGSCOUNT) );
    }
    else {
      m_Controller.setPhases( NULL, 0 );
      lpSilly->sendResponse( CONSOLESUCCESS );
    }
  }
  else if (!strcmp( lpSilly->getArg( 0 ), "get" )) {
    if (lpSilly->argsCount() != 2) {
      lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGSCOUNT) );
    }
    else {
      ProfileInfo_t Profile;
      int ProfileIndex = atoi( lpSilly->getArg( 1 ) );

      if (!LoadProfile( Profile, ProfileIndex)) {
        lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGOUTOFRANGE) );
      }
      else {
        const VLOvenControllerPhase_t*  lpPhase;

        lpSilly->beginResponse();
        m_Console.send( F("profile[idx=") );
        m_Console.send( ProfileIndex );
        m_Console.send( F(",Name=\"") );
        m_Console.send( Profile.Header.Name );
        m_Console.send( F("\",pnct=") );
        m_Console.send( Profile.Header.PhasesCount );
        m_Console.send( F("]" ) );

        lpPhase = Profile.lpPhases;

        for (int Count = 0; Count < Profile.Header.PhasesCount; Count++) {
          m_Console.send( F(TEXTCONSOLE_EOLN) );
          m_Controller.SendPhaseInfo( lpPhase );
          lpPhase++;
        }
        lpSilly->endResponse( CONSOLESUCCESS );

        FreeProfile( Profile );
      }
    }
  }
  else if (!strcmp( lpSilly->getArg( 0 ), "nw" )) {
    const char* Name;
    ProfileInfo_t Profile;
    int NameLen;
    int PhasesCount;
    
    if (lpSilly->argsCount() != 3) {
      lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGSCOUNT) );
    }
    else {
      Name = lpSilly->getArg( 1 );
      NameLen = strlen( Name );
      PhasesCount = atoi( lpSilly->getArg( 2 ) );
      
      if ((NameLen >= (sizeof( Profile.Header.Name ) - 1)) || (PhasesCount < 1)) {
        lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGOUTOFRANGE) );
      }
      else {
        memcpy( &Profile.Header.Name[0], Name, NameLen );
        Profile.Header.PhasesCount = PhasesCount;

        if (AllocProfilePhases( Profile )) {
          memset( Profile.lpPhases, 0, Profile.Header.PhasesCount * sizeof(Profile.lpPhases[0]) );
          ActivateProfile( Profile );
          m_CurrentProfileIndex = GetProfilesCount();
          lpSilly->endResponse( CONSOLESUCCESS );

          SendProfileInfo();
        }
        else {
          lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDNOMEMORY) );
        }
      }
    }
  }
  else {
    lpSilly->sendResponse( CONSOLEERROR, F(TEXTCONSOLE_CMDARGINVALIDOPT) );
  }
}


