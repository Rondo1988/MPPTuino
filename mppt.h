/****************************************************************
* Author: Dustin Sanders
* Version: 1 10/16/2015
* File: MPPT_H
* Descrioption: Tracks max power using one of the algorithms present in
** this file.  Sensor data is provided as a public function. 
****************************************************************/

/*
Requirements: 
- At least 1 algorithm using simple hill climbing and P&O
- Sensor values are provided via public function
- Public function to excecute telemetry 
- Private function to update PWM output values
- Document via Doxygen format
*/
#include "globals.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
//#include <ADC.h>

// SHOULD MOVE TO GLOBALS
#define mpptDEFAULT_P_GAIN		1.0f
#define mpptDEFAULT_I_GAIN		1.0f
#define mpptDEFAULT_D_GAIN		1.0f
#define mpptDEFAULT_WINDUP		1.0f

#define mpptSTARTED_MASK		0x10
#define mpptBOOST_MASK			0x01
#define mpptBUCK_MASK			0x02
#define mpptDISABLED			0xFF
#define mpptMIN_DUTY			50		// TODO: FIX ONCE DEADTIME IS KNOWN
#define mpptMAX_DUTY			324		// TODO: FIX ONCE DEADTIME IS KNOWN
#define mpptSS_STEP				5
#define mpptQAD_CTRL_STEP		1
#define mpptUV_MASK				0x20
#define mpptOV_MASK				0x40
#define mpptRUNNING_MASSK		0x80
#define mpptOC_MASK				0x04
#define mpptMIN_SS_VOLTAGE		10
#define mpptMAX_PWM_BITS		511
#define mpptSMPS_DISABLE		-1
#define mpptINOUT_RATIO			( globalOUT_DIV / globalSOLAR_DIV )


// Structure to pass sensor data out of the class
typedef struct
{	
    // PV Current 
	uint16_t PVI;
	// PV Voltage 
	uint16_t PVV; 
	// PV Temperature 
	uint16_t PVT;
	// HV Current
	uint16_t HVI;
	// HV Voltage
	uint16_t HVV;
	// Board Temperature
	uint16_t BrdT;
	// SMPS Mode bit mask (0x01 = Boost, 0x02 = Buck, 0x04 = Buck/Boost, 0x10 = Started, 0xFF = disabled)
	uint8_t Mode;
	// Errors (todo)
	uint8_t Errors;

} power_data_t; 

/* TODO: Add safety compare on output to adjust duty when output voltage
* gets to high, this can happen at kHz.  When the threashold has been 
* reached it goes to ISR where it reduces the duty cycle and wakes 
* MPPT task to handle further.  */


// called everytime a new value is converted. The DMA isr is called first
/* Could be used to wake the MPPT task if a value has made a large enough 
* change so that the algorithm can rerun and get max power. 
*/
// void adc0_isr(void) {
//     //int t = micros();
//     Serial.println("ADC0_ISR"); //Serial.println(t);
//     ADC0_RA; // clear interrupt
// }
// void adc1_isr(void) {
//     //int t = micros();
//     Serial.println("ADC1_ISR"); //Serial.println(t);
//     ADC1_RA; // clear interrupt
// }


 /** Store and Retrieve Settings for the controller */
class Settings
{
public:
  // Default constructor
  Settings():
    AlorithmType( globalALGORITHM ),
    VMax( globalVMAXOUT ),
    IMin( globalIMAXOUT ),
    MPPTRate( globalMPPT_MS ), 
    MPPTId( 0 )
    {}

  // Get Setting
  void* read ( uint8_t Setting );

  // Store Setting
  void write (uint8_t Setting, void* fVal);

  // Var Seettings

  // Type of algorithm to use (0 - ??)
  uint8_t AlorithmType;
  // Max Output Voltage in counts
  uint16_t VMax;
  // Max Output Current in counts
  uint16_t IMax;
  // Miniumu Charge Current (somehow used to determine full battery,
  // might be better to monitor the CAN bus for this) in counts
  uint16_t IMin;
  // Maximum Board Tempreature in counts
  uint16_t MinBrdT;
  // Max Power Point Tracker Refresh Rate in Miliseconds (default = 10 ms, max 1 ms)
  uint16_t MPPTRate;
  // Telemetry Rate sent by CAN (Samples per Minute)
  uint8_t telemetryRate;
  // Theoretical (or measured) Open Circuit Voltage
  float PVVoc;
  // Theoretical (or measured) Short Circuit Current
  float PVPsc;
  // Theoretical (or measured) Maximum Power Point Voltage
  float PVVmp;
  // Theoretical (or measured) Maximum Power Point Current
  float PVPmp;
  // MPPT ID
  uint8_t MPPTId;
  
};

/** PID Controller
* Link to Example: https://nicisdigital.wordpress.com/2011/06/27/proportional-integral-derivative-pid-controller/ 
* Question: Should this control input or output voltage, obviously
* output voltage needs to be watched but not controlled since it
* will be attached to a battery load which will hold it and a specific
* voltage.  
*/

class PID 
{
	public: 
	  float fAntiWindup;
	  float fPgain;
	  float fIgain;
	  float fDgain;
	  int16_t fCtrl;

	  PID():
	    fAntiWindup(mpptDEFAULT_WINDUP), 
	    fPgain(mpptDEFAULT_P_GAIN),
	    fIgain(mpptDEFAULT_I_GAIN),
	    fDgain(mpptDEFAULT_D_GAIN),
	    fPrevErr(0),
	    fIntErr(0)
	    {}

	  /** Updates PID controller 
	  * Input: Current Error, Change in Time
	  * Output: Control Value 
	  */
	  int16_t updatePID( int16_t fcurrErr, uint32_t u32Time );


	private:
	  float fPrevErr;
	  float fIntErr;  
	  uint32_t u32PrevTime;

};


/** Class MPPT: Handles all MPPT activity
*
**/
class MPPT
{  
	private:
		// Vars
		// PV Power
		uint16_t PVP;	
		// PV dPower
		int32_t PVdP;
	  // PV Current 
		uint16_t PVI;
		// PV current delta
		uint16_t PVdI;
		// PV Voltage 
		uint16_t PVV;
		// PV Voltage delta
		uint16_t PVdV; 
		// PV Temperature 
		uint16_t PVT;
		// HV Power
		uint16_t HVP;
		// HV Current
		uint16_t HVI;
		// HV Current delta
		uint16_t HVdI;
		// HV Voltage
		uint16_t HVV;
		// HV Voltage Delta
		uint16_t HVdV;
	    // Duty Cycle 
		uint16_t Duty; // only uses 9 bits
		// Board Temperature
		uint16_t BrdT;
		// Output Voltage Setpoint
		uint16_t VoutSet;
		// SMPS Mode bit mask (0x01 = Boost, 0x02 = Buck, 0x04 = Buck/Boost, 0x10 = Started, 0xFF = disabled)
		uint8_t Mode;
		// Settings
		Settings conf;
		// PID Controller
		PID controller;


		// Update Power Stage
		//! Updates H[1:0] & L[1:0] outputs based on VoutSet
		// Pins 9, 10, 22, 23 are on FTM0
		void vUpdatePS ( uint16_t );

		/** Soft Start Function */
		void vSoftStart ( void );


		////// Algorithsms ///////

		// 0: Basic SMPS
		//! Keeps output voltage at Settings.VMax
		void vMethodSMPS ( void );


		// TODO: FINISH ALGORITHMS

		// 1: Basic Hill Climber/Perturbe and Observe
		//! .... 
		// (7) Hillclimb/Perturb and Observe
		// psudo:
		//     Input: 
		//         PV_V, PV_I, HV_V, HV_I
		//     returns:
		//         Duty Cycle

		void vMethodHCPO( void );

		// 2: Improved HC
		//! ....
		// Modified HC/PO
		// quickly blasts up the hill at a course increment
		// Finds the highest course (this helps with multipule maxima)
		// Goes from C-1 -> C+1 with a fine hill run. Max is found
		// A finer control tracks the change in current to adjust for changes.  
		// a large enough current change causes the a backward regretion though
		// course and fine tuning.  such as if a large enough current change has happend
		// it can go from very fine to fine to search for the new point then ripple
		// until max is found.  If an even greater change has occured do course
		// over again over whole range.  This should happen fast, with in a few
		// cycles (like the course should be done say every 2 pulse widths or 10 depending
		// on how quickly the power will change due to the caps.  Less input cap might
		// increase convergence speed.
		//void vMethodCustom( );

		// 3: Incremental Conductance
		// -I/V = dI/dV @ MPP
		// -I/V < dI/dV (left of MPP)
		// -I/V > dI/dV (right of MPP)
		// 
		// Again modification can be done with varying stepped response to the
		// derivatives. 
	 	//void vMethodINDCON( void );

	 	// (1) Rapid Tracking Hill Climbing 
		//  calculates a beta parameter and sets a min and max
		//  calculates a duty cycle based an error in the beta. 
		//  
		//void vMethodRTHC( void );

		// (2) DeltaP (Incremental Conductances derivative)
		// Calcuates actual power, the recorded max power is subtracted by actual
		// this is delta P, if Delta P > UpperLimit then increase pmax
		// delta p is sent to pwm controller to adjust the duty cycle.
		// PMax is held for several samples before saving and used in algorithm
		//void vMethodDELTAP( void );	

		// (3) Trend (or Power Prediction Line)
		// Generates a trend line based on temperature and a konstant value for 
		// optimal current (max power current)
		//void vMethodTREND( void );
			
		// (4) Quadradic Interpolation
		// Gets 3 samples at various locations around a point
		// Interplates the Vmax
		// adjusts voltage step based on difference in Vi and Vmax
		// deltaP > 0 -> Y = +DeltaV else -DeltaV? 
		// to get different points
		//void vMethodQI( void );
			
		// (8) Variable Step (P&O ish)
		// Does by computing the next voltage value using a newton iteritive model
		// 3 separate samples are taken along the power curve.  These are used
		// to determine whether to increase voltage or decrease to achieve max power
		// Best to build in a tolerance, or to check for current change, cause if 
		// your at max power and current hasn't changed then your still @ max power
		// and you don't need to move to various locations to check.  
		// to simplify the calculations, could the dP/dV and d2P/dV2 calulations
		// just use the previous dP/dV samples to calculate f'(V)
		// those values are used in eq (10). although if you look at the equations for
		// f(V) and f'(V), all of the Isc and Voc get filtered out and your left with
		// simply the dP and dV.. OOH LOL.. that's exactly what its doing LOL.. 
		// just inversed,  dV^2*dP1/(dV2dP1 + dV2dP2)
		// 
		//void vMethodVS( void );


		// (13) Fuzzy Logic 
		// do some fuzzies
		// Seems similar to (8) but differnt... high hopes for both
		//void vMethodsFL( void );


		// (12) Inhomogeneous Insolation
		// Just uses ratios of Power and Current to find a(T)
		// P(t) - a(T)*Imeas(t)
		// Pmeas(t) > P(t)
		//void vMethodINHOMO( void );

		// Full Sweep
		// This does a full sweep and determines the max power from the sweep. 
		// The sweep happens again once delta I is above a certain value.
		// Could have it sweep in the direction of current change? 
		// So if current change is negative then the sweep brings voltage down
		// until power is max again.  vise versa for positive delta.  Have
		// an programmed overshoot so confirm that it is max power. 
		// Problem could be multipule maxima.  when the max power its at 
		// a point and a MM shows up it might be right under where previous MP was
		// and the real MP is further down in voltage.  I suppose if an input of
		// series cells then MM detection can happen where it sweeps back to where
		// a known MM would occur if one was possible? 
		//void vMethodFS( void );
	public:
	  /** Default Constructor */
	  MPPT():
	    PVP(0),
	    PVdP(0),
	    PVV(0), 
	    HVI(0),   
	    HVV(0),  
	    Duty(0),
	    Mode(0)
	    {}

	// Get power data
	//! Returns power_data_t 
	/*!
	*  */
	power_data_t xGetData ( void );

	// Run Algorithm
	//! Runs the configured mppt algorithm 
	/*!
	* list of algorithms: 
	* 0: Stndard SMPS, keeps the set max output voltage
	* 1: Basic Hill climber/Perturbe and Observe algorithm
	* 2: Improved Hill climb/P&O
	* 3: (7) Incremental Conductance
	* 4: ...
	*/
	void vRunAlgorithm ( void );

		// Get Sensors
		//! take data from ADC buffer and stores in class
		// May not need if using DMA and a ring buffer. 
	void vGetSensors ( void );


	uint8_t getMPPTid ( void )
	{   
		return conf.MPPTId; 
	}

	uint8_t getMode( void )
	{
		return Mode;
	}

	void setMode( uint8_t mode )
	{
		Mode = mode;
	}
};


	
