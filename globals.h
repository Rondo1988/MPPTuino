/*
 * gloabals.h
 *
 * Created: 2/26/2012 9:33:46 PM
 * Modified: 11/27/2015
 *  Author: Wedge
 */ 


#ifndef GLOABALS_H_
#define GLOABALS_H_

#define globleVERSION                 "v0.1"

 /************************************************************************/
/* Hardware Pin Assignments		                                         */
/*************************************************************************/

#define globalHEARTBEAT_PIN				13 	//3.1 on board LED
#define globalH0_PIN					23	// Buck Side high fet gate signal
#define globalL0_PIN					22  // Buck Side low fet gate signal
#define globalH1_PIN					10  // Boost Side high fet gate signal
#define globalL1_PIN					9  // Boost Side low fet gate signal
#define globalSTATUS1_LED_PIN			5	// Status LED on MPPT Board, 
#define globalSTATUS2_LED_PIN			6	// Status LED on MPPT Board, 
#define globalCANCTRL_PIN				2	// Control of the CAN transciever
 // Analog
#define globalSOLAR_I_PIN				14  // Input Current
#define globalSOLAR_V_PIN				15  // Input Voltage
#define globalOUT_I_PIN					16  // Output Current
#define globalOUT_V_PIN					17  // Output Voltage
#define globalBOARD_T_PIM				18	// Onboard Tempreature Sensor
#define globalSOLAR_T_PIN				19	// Solar Panel Tempreature Sensor

/************************************************************************/
/* MPPT Defaults/Constants/Defines                                      */
/************************************************************************/

/** TODO: MAKE MACROS FOR MAX VALUES **/
// Default Settings
#define globalALGORITHM					0 // The first and simplist
#define globalVMAXOUT					1293   // TESTING 30V // 109.2 / 2^12 in 12bit counts
#define globalIMAXOUT					2612	// 8 / globalAMPBITS 
#define globalMPPT_MS					10000   // 10 ms (in ns)
#define globalHEARTBEAT_MS				1000000 // 1 second	
#define globalHEARTBEAT_FLASH_MS		500  // 1/2 SECOND	
#define globalTELEMETRY_MS				5000000 // 5 SECONDS	
#define globalCAN_TELEMETRY_DEFAULT_ID	0x200
#define globalCAN_RX_CMD_ID				0x400
#define globalPWM_FREQ					100000 // 100 kHz
#define globalSOLAR_DIV					19.3617f // 91K / 4.7K RESISTOR DEVISION
#define globalOUT_DIV					38.7917f // 93.1K / 2.4K RESISTOR DIVISION
#define globalSOLAR_VBITS				0.0155990258789f
#define globalOUT_VBITS					0.0312530786133f
#define globalVOLTPERAMPS				0.264F // V/A
#define globalAMPBITS					0.0030517578125f // A/BIT
#define globalSS_TIME					1000000// in ns
#define globalCurrOffset				410 // 330mV offset

// KC130 Values @ STC (1000 W/m^2, 25C)
#define KC130_VOC               		21.9f
#define KC130_ISC               		8.02f
#define KC130_TEMPCOEFFVOC      		-.0821f // V/C
#define KC130_TEMPCOEFFISC      		.00318f // A/C
#define KC130_VMPP              		17.6f   // V
#define KC130_IMPP              		7.39f   // A
#define DQUALITY                		1.2 //Diode quality of the solar cells

// Solar Panel Properties @ STC
#define globalVOC                     KC130_VOC
#define globalVMPP                    KC130_VMPP
#define globalISC                     KC130_ISC
#define globleIMPP                    KC130_IMPP
#define globleTEMPCOEFFVOC            KC130_TEMPCOEFFVOC
#define globleTEMPCOEFFISC            KC130_TEMPCOEFFISC


// Cosmological Konstants 
#define globleKBOLTZ                  0.000000000000000000000013806488f
#define globleQCHARGE                 0.0000000000000000001602176565f

// Other Constants

// The area around 0 to be considered zero
#define ZEROCROSSINGD           .01f

// For MPPT Method (1) Fast Tracking
#define CNOTT                   QCHARGE / (KBOLTZ * DQUALITY)
#define PVIPRECISION            1000000  // 6 deimal places (maybe increase)
#define DUTYMAX                 90 //TODO: CHANGE to represent 90% of max pwm
#define DUTYMIN                 10 //TODO: respresnet 10%
#define HVDEFAULT               120 // TODO; convert to what ever i end up with using
#define DATABASENUM             9

// this is the amount the duty will change +-
// The value will have to be picked, 5 is just a placeholder.
#define PO_delta                5 
      
// Duty/Volts
#define DUTYVOLTS               10
 
#define MAXMPPTVARS             10 // Arbitrary large number

// For MPPT Method (3) vmpptMethodTREND
#define CURRENTHALFRATED        KC130_IMPP/2 // Half the rated current, MPI=5
#define FIELD2ERR               1           // tweak



/************************************************************************/
/* Handy Defines                                                        */
/************************************************************************/
#define CR                      13       //!< Carriage return
#define DEL                     127       //!< Delete Key
#define BS                      8       //!< Backspace Key
#define CUB1                    "\x1B[1D"   //!< Move Curser to the Left 1 Col
#define SAVC                    "\x1B7"     //!< Save Cursor Location
#define RESC                    "\x1B8"     //!< Restore Cursor Location
#define HOME                    "\x1B[;H"   //!< Move cursor to Top Left Corner
#define CUD1                    "\x1B[1B"   //!< Move cursor down 1 line
#define ESC                     "\x1B["      //!< vt100 Esc sequence

//#define serialLED                     LED1    // LED Used when there's USART Activity
#define serialDELIMITER         CR   // We'll use the carriage Return

// Turn on RX Echo
#define serialSERIAL_ECHO       1    

#define menuQUEUESIZE           (unsigned portBASE_TYPE)20 //10 Chars worth
// #define menuCMDSIZE     4
// // For Command Array 
// #define menuCMD         0
// #define menuACT         1
// #define menuDEV         2
// #define menuVAL         3
// 10ms Block time for ISR to exit after waking menu
#define menuBLOCKTIME           ( ( ( portTickType ) 10 ) / portTICK_RATE_MS )

#define menuPROMPT              "> "
#define menuHELP                "?"/*63*/      //!< (?) Question Mark
#define menuADC                 "A"/*65*/      //!< A
#define menuPWM                 "P"/*80*/      //!< P 
#define menuDATA                "D"/*68*/      //!< D
#define menuINFO                "I"/*73*/      //!< I
#define menuSET                 "="/*61*/      //!< (=) Equals - Chane a value
#define menuDISABLE             "-"/*45*/      //!< (-) Hyphen, or Minus - Disables Something
#define menuENABLE              "+"/*43*/      //!< (+) Plus - Enables something
#define menuTOGGLE              "!"/*33*/      //!< (!) Exclamation - Toggles a device
#define menuDISPLAY             "$"/*36*/      //!< ($) Dolar Sign - Displays Value
#define menuLWROFFSET          0x20      //!< OFFSET for Uppercase to Lowercase
#define menuCANCEL              "."/*46*/      //!< (.) Period - Cancels current menu
//#define menuRANGEDELI           58      //!< (:) Colon - Sets a range of devices
#define menuCMDDELI             " "/*32*/      //!< ( ) Space - Value Delimiter

/************************************************************************/
/* Menu Switch Defines                                                  */
/************************************************************************/
#define switchHELP                '?'/*63*/      //!< (?) Question Mark
#define switchADC                 'A'/*65*/      //!< A
#define switchPWM                 'P'/*80*/      //!< P 
#define switchDATA                'D'/*68*/      //!< D
#define switchINFO                'I'/*73*/      //!< I
#define switchSET                 '='/*61*/      //!< (=) Equals - Chane a value
#define switchDISABLE             '-'/*45*/      //!< (-) Hyphen, or Minus - Disables Something
#define switchENABLE              '+'/*43*/      //!< (+) Plus - Enables something
#define switchTOGGLE              '!'/*33*/      //!< (!) Exclamation - Toggles a device
#define switchDISPLAY             '$'/*36*/      //!< ($) Dolar Sign - Displays Value
#define switchLWROFFSET          0x20      //!< OFFSET for Uppercase to Lowercase
#define switchCANCEL              '.'/*46*/      //!< (.) Period - Cancels current menu
#define switchCMDDELI             ' '/*32*/      //!< ( ) Space - Value Delimiter



/************************************************************************/
/* Return Values                                                        */
/************************************************************************/
#define errNONE                 0
#define errFAILURE             -1
#define errINVALID             -2                       


/************************************************************************/
/* Menu Formating                                                       */
/************************************************************************/
#define MOVCRTOXY(X,Y)          ("\x1B[" X ";" Y "H")
// Moves curser forward X (# in size char)
#define CUF(X)                  ( ESC X "C" )

#define COL1                    45


/************************************************************************/
/* Software Reset                                                       */
/************************************************************************/
#define RESET_AVR               wdt_reset_mcu();


/************************************************************************/
/* Helpful Functions                                                    */
/************************************************************************/
char *itoa(int i);

// Takes a char and determines the number of col to move right in order
// to justify the text to a specific col
char *cpRightJust(char * data, long from, long to);
#endif /* GLOABALS_H_ */
