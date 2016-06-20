#include <ADC.h>
#include <ADC_Module.h>
#include "core_pins.h"

/****************************************************************
* Author: Dustin Sanders
* Version: 1 10/16/2015
* File: MPPT_C
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

#include "mppt.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
#include "globals.h"
/* TODO: Update algorithms with new datastructures */

/** ADC Object created in main **/
extern ADC* adc;

void MPPT::vRunAlgorithm ( void )
{
		/** Run the configured algorithm **/
	switch(conf.AlorithmType)
	{
		case 0:		/** Simple SMPS **/
		{
			vMethodSMPS();
			break; 
		}
		case 1:  	/** Hill Climb / Perturbe Observe **/
		{
			vMethodHCPO();
			break;
		}
		default:	/** nothing yet **/
			break;

			
	}
}

/** Get Sensors
* Gets sensor data from ADC
* All sensor values in 12 bit Counts! 
*/
void MPPT::vGetSensors( void )
{

	/** use analogSynchronizedRead for adc pins 0|2 and 1|3
	* for voltage and current  

	* for using compare: 
	* Set the compare range for the ADC
	* Start conversion of desired pin. 
	*/

	/* Quick and dirty method just to get it working
	* Replace with a non-blocking method */
	static ADC::Sync_result xResult;

	PVdI = PVP;
	PVdV = PVV;
	HVdI = HVI;
	HVdV = HVV;
	// Get current
	xResult = adc->analogSynchronizedRead( globalSOLAR_I_PIN, globalOUT_I_PIN );
	PVP = (uint16_t) xResult.result_adc0 - globalCurrOffset;
	HVI = (uint16_t) xResult.result_adc1 - globalCurrOffset;
	// Get Voltage
	xResult = adc->analogSynchronizedRead( globalSOLAR_V_PIN, globalOUT_V_PIN );
	PVV = (uint16_t) xResult.result_adc0;
	HVV = (uint16_t) xResult.result_adc1;

	// TODO: USE AN ARRAY TO CALCULATE TREND
	PVdP = PVP;  // replace dPdV with last PVP
	// Calculate power
	PVP = PVV * PVP;
	// 
	HVP = HVV * HVI;
	// Calculate differentials
	PVdP = PVP - PVdP;
 	HVdV = HVV - HVdV;
 	HVdI = HVI - HVdI;
 	PVdV = PVV - PVdV;
 	PVdI = PVP - PVdI;

	// TODO: GET TEMPREATURE

}

/** Simple SMPS
* Looks at the output and regulates to set Vmax
*/
void MPPT::vMethodSMPS( void )
{
	//static float Err; // Error between currnet voltage and desired voltage
	//static int16_t ctrl; // control value passed by 
	//static uint16_t DutyVal;


	/** mppt not started and not disabled **/
	if ( ( Mode & mpptSTARTED_MASK ) == 0 && Mode != mpptDISABLED )
	{
  		vSoftStart( );
	}
	else
	{
		vUpdatePS( 0 );
	}

	/** Recovery from soft start failure **/
	


	
}

/** Return power_data_t data from the MPPT class
* example: ... 
*
**/

power_data_t MPPT::xGetData( void )
{
	power_data_t data;

	data.PVV = PVV;
	data.PVI = PVI; 
	data.HVV = HVV; 
	data.HVI = HVI;
	data.PVT = PVT;
	data.BrdT = BrdT;
	data.Mode = Mode;
	// TODO: fix error reporting
	data.Errors = 0;

	return data;
}

/** SMPW Soft Start 
* Used with vMethod SMPS to start the controller
*/
void MPPT::vSoftStart ( void )
{
	static uint16_t targetDuty;
	static uint16_t DutyVal;
	static uint8_t counter = 0;
	static uint16_t pastVout = 0;
	uint16_t vmaxNorm = conf.VMax * mpptINOUT_RATIO;

 	DutyVal = mpptMIN_DUTY;

	/** Scale voltages **
	* scale input to output **/

	// input div 19.36
	// output div 38.79


	/** Scaled TargetDuty cycle for softstart **/
	/* Boost Mode *
	* Target duty cycle: D = (Vout - Vin) / Vout *
	*/
	if ( ( Mode & mpptBOOST_MASK ) == 1 )
	{
		targetDuty = (uint16_t)( ( 1 - ((float)PVV / (float)vmaxNorm) ) * mpptMAX_PWM_BITS);

		#ifdef USBDEBUG
	   	Serial.println("boost mode");
	   	#endif
	}

	/* Buck Mode *
	* Target Duty Cycle: D = Vout / Vin *
	*/
	else if ( ( Mode & mpptBUCK_MASK ) == 1 )
	{
		targetDuty = (uint16_t)(( (float)vmaxNorm / (float)PVV ) * mpptMAX_PWM_BITS); 
		#ifdef USBDEBUG
    	Serial.println("buck mode");
    	#endif
	}
  else 
  {
    targetDuty = mpptMIN_DUTY;
    #ifdef USBDEBUG
    Serial.println("no mode");
    #endif
  }

	// /** Sanity check */
	// else if ( targetDuty >= DutyVal )
	// {
	// 	Mode |= mpptSTARTED_MASK;
	// }
  #ifdef USBDEBUG
  Serial.print("TargetDuty = ");
  Serial.print( targetDuty, DEC);
  Serial.print(", VMAX = ");
  Serial.print(vmaxNorm, DEC);
  Serial.print(", VIN = ");
  Serial.println(PVV, DEC);
  #endif /* USBDEBUG */

  //Turn on output
  vUpdatePS( DutyVal ); // set min value
  FTM0_OUTMASK = 0x0;            // Turns on PWM output
  pastVout = HVV; 
	// with target duty set 
	while (DutyVal < targetDuty)
	{
		DutyVal += mpptSS_STEP;
    Serial.println("SS");
		vUpdatePS( DutyVal );
    delay(10); 
		//vTaskSuspend( NULL ); // suspend until next MPPT wake
		if (counter++ > 5)
		{
			counter = 0;
			vGetSensors();
			if ( HVV - pastVout < mpptMIN_SS_VOLTAGE ) // something wrong its not increasing in voltage 
			{
				DutyVal = mpptMIN_DUTY;
				Serial.println("failed SS exiting/disabling");
				Mode = 0xFF; // disable SMPS
				vUpdatePS( mpptSMPS_DISABLE );
				break;
			}
		}
	}

	/** Soft start complete */
	Mode |= mpptSTARTED_MASK; 

}

/** Update Power Stage **
* Input: pwm duty cycle
* TODO: MAKE INPUT A VOLTAGE AND IT CALCULATES DUTY CYCLE 
*/
void MPPT::vUpdatePS( uint16_t DutyVal )
{
	static int16_t Verror;
	/** Check if outside bounds **/

	#ifdef USBDEBUG
	Serial.print("Duty: ");
	Serial.print(Duty, DEC);
	Serial.print(" CND: ");
	Serial.print(FTM0_CNT, DEC);
	Serial.print(" MODE: ");
	Serial.print(FTM0_MODE, DEC);
	Serial.print(" FMS: ");
	Serial.println(FTM0_FMS, DEC);
	#endif /* USBDEBUG */

	// Get error
	// use PID controller if no input duty is provided.
	if ( DutyVal == 0 ){
		Verror = globalVMAXOUT - HVV;
		DutyVal = Duty + controller.updatePID(Verror, micros());
	}
	else if ( DutyVal == mpptSMPS_DISABLE || Mode == 0xFF ){
		/* code */
		FTM0_OUTMASK = 0xFF;            // Turns off PWM output
    Serial.println("off with PWM");
	}
	else if ( DutyVal > mpptMAX_DUTY ){
		Duty = mpptMAX_DUTY; 
	}
	else if ( DutyVal < mpptMIN_DUTY ){
		Duty = mpptMIN_DUTY;
	}
	else {
		Duty = DutyVal;
	}


	
	// Update PID controller

	analogWrite(globalH1_PIN, Duty );
  if(Mode == 0xFF){
  	digitalWrite(globalSTATUS2_LED_PIN, HIGH);
  }			
  else{
   	digitalWrite(globalSTATUS2_LED_PIN, !digitalReadFast(globalSTATUS2_LED_PIN));
  }
	if ( ( ( FTM0_MODE & FTM_MODE_FTMEN ) == 0 ) && ( Mode != 0xFF ) ){
		// Enable output
		FTM0_OUTMASK = 0x0;            // Turns on PWM output
    Serial.println("turning on pwm");
	}

	FTM0_SYNC |= 0x80;  

	// setup voltage compare to interrupt 

}


// (7) Hillclimb/Perturb and Observe
 // psudo:
 //     Input:
 //         These values are in their float voltage 
 //         converted/conditioned form
 //         PV_V, PV_I
 //     returns:
 //         Duty Cycle
 //         
 
 void MPPT::vMethodHCPO()
{
	/************************************************************************/
	/* Variables
	 * 
	 * Polarity = Tells the tracker which way it went last time. 
	 *  Will get screwed up when not actively tracking, but whateves. 
	 * 
	 * This is as standard as P&O as it gets no modifications.
	 * 
	 * */
	/************************************************************************/
    // Setup Tracker Stuff
	 uint8_t u8Polarity = 0;
														 
	
	/************************************************************************/
	/*  Truth Table
	 *  DPV > 0 | Polarity  |   output
	 *      1   |     1     |   +PO_delta   //Positive Change
	 *      1   |     0     |   -PO_delta   //Negative Change
	 *      0   |     1     |   -PO_delta
	 *      0   |     0     |   +PO_delta
	 *      
	 */                                                                      
	/************************************************************************/
	// If dP/dV > 0 then we're on the left side (low side) of max power
	// add Positive change.  According to truth table, an XOR will give us
	// A positive change when polarity and dPdV > 0 are equal
 
	if ( ( PVdP/PVdV > 0 ) ^ u8Polarity ) //Positive change
	{
		//might want to make this a variable so it can be modified
	  Duty += PO_delta;
		u8Polarity = 1;
	}
	else  // Negative Change
	{
	 	Duty -= PO_delta;
		u8Polarity = 0;
	}
	vUpdatePS ( Duty );
}

// // Custom (everything I've found good about every type of algorithm
// //  will be incorporated into custom to try to have a well rounded method
// // --kind of a poor man fuzzy logic controller.. 
// //      actually it is a fuzzy logic controller
// // quickly blasts up the hill at a course increment
// // Finds the highest course (this helps with multipule maxima)
// // Goes from C-1 -> C+1 with a fine hill run. Max is found
// // A finer control tracks the change in current to adjust for changes.  
// // a large enough current change causes the a backward regretion though
// // course and fine tuning.  such as if a large enough current change has happend
// // it can go from very fine to fine to search for the new point then ripple
// // until max is found.  If an even greater change has occured do course
// // over again over whole range.  This should happen fast, with in a few
// // cycles (like the course should be done say every 2 pulse widths or 10 depending
// // on how quickly the power will change due to the caps.  Less input cap might
// // increase convergence speed.
// void vmpptMethodCustom()
// {
// 	// Add some custom code here.
// }

// // Notes:
// // Only real difference between P&O is that there's a don't change option.  
// // Which keeps it from perturbing. 

// void vmpptMethodINDCON( void )
// {
// 	// Setup Tracker Stuff

// 	// Supplemental Calculations
	
// 	/************************************************************************/
// 	/* Truth Table
// 	 * 0Delta > dV < 0Delta  |   dI < 0delta  |  dP/dV < -I/V |  output
// 	 *          1                    x                1        |   Vref-- 
// 	 *          1                    x                0        |  Vref++
// 	 *         0                    1                x        |  Vref--
// 	 *         0                    0                x        |  Vref++
// 	 *                                                                            
// 	************************************************************************/
	
// 	                     /* not zero */
// 	if ( ((d->PVV->D > ZEROCROSSINGD && d->PVV->D < -ZEROCROSSINGD ) &&
// 	    d->fdPdV > 0) || d->PVP->D > 0)
// 	{  
// 		// Increment Vref (which controls the duty)
//         (U16)xVref->I++;
// 	}
// 	else
//     {
// 	    // Decrement Vref	
// 		(U16)xVref->I--;
// 	}		
	
// 	// Convert Vref to Duty Cycle (probably best found experimentally)
//     my->pxMyDuty->I = DUTYVOLTS * (U16)xVref->I;
// }

// // (1) Rapid Tracking Hill Climbing 
// //  calculates a beta parameter and sets a min and max
// //  calculates a duty cycle based an error in the beta.
// //  
// //************************************
// // Method:    mpptMethodRTHC
// // FullName:  mpptMethodRTHC
// // Access:    public 
// // Returns:   U16
// // Qualifier:
// // Parameter: U16 usPVV - Fixed point PV Voltage     (10 decimal?)
// // Parameter: U16 usPVI - Fixed point PV Current     (10 decimal?)
// // Parameter: U16 usPVT - Fixed Point PV Temperature (2 decimal percision)
// // 
// // I DONT LIKE THIS ONE, ITS KINDA LAME
// //************************************
// void vmpptMethodRTHC( void )
// {
// 	// Setup Tracker Stuff

		
// 	float fC = CNOTT/((float) d->PVT->I);
// 	float k = 1; // Find slope
	
// 	// probably for debugging be easy to split this process up.
// 	fBetaA->I = log(g_xOptions.xData.PVP->I / g_xOptions.xData.PVV->I) - 
// 					(fC * g_xOptions.xData.PVV->I);
					
// 	// Need to generate the beta slope 
	

// 	// A thought would be to find what the max power is then set the 
// 	// duty max a percentage past that.  that way it won't ever go too far
// 	// past MPP?  
// 	// 
	
// 	// Steady state? 
// 	// if D = 0? or check for several D's? Theres no way its going to be
// 	// Steady.  Maybe just check 1 or 2 back if the change is within
// 	// a delta accept it as steady state.
// 	// 
	
// 	// Is B between Min and Max? 
// 	if ( fBetaA < fbetaMax || fBetaA > fBetaMin )
// 	{
// 		// do some method to get an exact MPP	
// 	}
// 	else      // do the beta thing
// 	{
// 		// BetaG this is a mystery and they barely describe it, sounds like a 
// 	    // fudge factor to me. Maybe its for when you don't know temperature 
// 	    // OOH its a lookup table of temperature values that coincide with
// 	    // the beta that should give max power. 
// 	    // Only problem is it won't really work well for low incidence.  
	
// 	    // error = BetaG-BetaA;
// 		// BetaG might not be a factor if there's temperature data. 
	
	
// 	    // dnew = dold+error*k(the slope of beta)
// 	    d->usDuty->I = pvOuroborosPOP( d->usDuty->xIHistory ) + fError * k; 
// 	}
	

// 	// sudo:
// 	// configure bmax and min automatically.
// 	// if power max < bmin set new bmin till max power is found
// 	// same with bmax.  Eventually they'll reach their maxes throughout
// 	// the day.    Don't really need to save each one though.  
   
	
// 	 // Save duty max bounds for everyone? yes.  -> goto main process
	
		
    
// }

// // (2) DeltaP (Incremental Conductances derivative)
// // Calcuates actual power, the recorded max power is subtracted by actual
// // this is delta P, if Delta P > UpperLimit then increase pmax
// // delta p is sent to pwm controller to adjust the duty cycle.
// // PMax is held for several samples before saving and used in algorithm
// // 
// // The more i look at this one the more i don't like it either.
// // PMAX its "max power" and only decrements max power if its above UL?
// // PMAX should change when MP is lower than 
// // Ehh fix it if you can if not toast it. 
// // 
// void vmpptMethodDELTAP( void )
//  {

	
// 	// Calc new Delta P (cause it a bit different then standard dp.
// 	// calculte duty from delta
// 	// changes max if max is below the upper limit.  
// 	// doesn't really work though, since yeah it hits that upper limit
// 	// but with if its cloudy, its going to just try dumping more power
// 	// till it gets its to upper limit.  
// 	// How does it handle changes in irradiation?  
// 	// 
	 
	    
// }

// // This one is dumb too.  Uses hill climbing to estimate MP...WTF? 
// // Of course that's going to get MP cause your climbing up.. stupid.
// // DUMP THIS ONE FOR 12
// void vmpptMethodTREND( void )
// {

// 	float fIpr = 0;
// 	// a(T) = VMP - TEMPCOEFFVOC*(25C-PVT) 
// 	// 25C is STC, and we know VOC from datasheet
// 	fAT->I = VMPP - TEMPCOEFFVOC*(25-d->PVT->I);
	
// 	// how to handle a0? just use VMPP? Probably have to modofy this later
	
// 	// Pm = a(T) * PVI
// 	fPm->I = fAT->I * d->PVP->I;
	
// 	// if PVP > Pm?
	
// 	if (d->PVP->I > fPm->I)
// 	{
// 	    // Field II
// 	    // Ipr(n+1) = PVP / a(T)
// 	    fIpr =  d->PVP->I / fAT;
// 	    // pvp - pm / pmax < err
// 		if (fIpr > FIELD2ERR)
// 		{
// 			// change Ipr to dutycycle
// 		}
// 	}
// 	else
// 	{
// 		// else Field I
// 	    // Ipr(n+1) = ksc * PVI
// 	    fIpr = IMPP * d->PVP;
// 	    // pn > pm+1
// 		if ( d->PVP->I < (fIpr * fAT) )
// 		{
// 			// change Ipr to dutycycle
// 		}
// 	}
	
	
	
	
// 	// Vary duty to match Ipr (predicted) 
	
// 	// if out of bounds restart procedure
	
						 
						 
// }

// // (4) Quadratic Interpolation
// // Gets 3 samples at various locations around a point
// // Interpolates the Vmax
// // adjusts voltage step based on difference in Vi and Vmax
// // deltaP > 0 -> Y = +DeltaV else -DeltaV? 
// // to get different points
// void vmpptMethodQI( void )
// {

						 
// 	// The system should start @ MPP based on datasheet
	
// 	// do we need a counter or should it just always look at the last
// 	// 3 samples? 
	
	
// 	    // if dP < 0 then Step = -dV
		
// 	// Now we have 3 samples
// 	// Calculated K[1-3]
// 	// y[0-2] = PVP[0-2]
// 	// x[0-2] = PVV[0-2]
// 	// k1 = Y0/((x0-x1)(x0-x2))
// 	// k2 = Y1/((x1-x0)(x1-x2))
// 	// k3 = Y2/((x2-x0)(x2-x1))
// 	// Vmax = (k1(x1+x2)+k2(x0+x2)+k3(x0+x1))/(2(k1+k2+k3))
// 	// step = Vmax - Vi; Vset = Vmax
    						 
	
// }

// // (8) Variable Step (P&O ish)
// // Does by computing the next voltage value using a newton iteritive model
// // 3 separate samples are taken along the power curve.  These are used
// // to determine whether to increase voltage or decrease to achieve max power
// // Best to build in a tolerance, or to check for current change, cause if 
// // your at max power and current hasn't changed then your still @ max power
// // and you don't need to move to various locations to check.  
// // to simplify the calculations, could the dP/dV and d2P/dV2 calulations
// // just use the previous dP/dV samples to calculate f'(V)
// // those values are used in eq (10). although if you look at the equations for
// // f(V) and f'(V), all of the Isc and Voc get filtered out and your left with
// // simply the dP and dV.. OOH LOL.. that's exactly what its doing LOL.. 
// // just inversed,  dV^2*dP1/(dV2dP1 + dV2dP2)
// // 
// void vmpptMethodVS( void )
// {

						 
// 	// Vk+1 = Vk - (Vk-Vk-2)(Vk-1-Vk-2)(P(Vk)-P(Vk-1))/
// 	//     ((Vk-1-Vk-2)(P(Vk)-(PVk-1))+((Vk-Vk-1)(P(Vk-1)-P(Vk-2)))
// 	//     
	
						 
// }

// // (13) Fuzzy Logic 
// // do some fuzzies
// // Seems similar to (8) but differnt... high hopes for both

// void vmpptMethodsFL( void )
// {

						 
// 	// CE = fdPdV?
// 	// 
// 	// dAn+1 = sum(Wi * Fi(An))/sum(Wi) = ...
// 	//      What are the weights?!
// 	//  W1 = Ups
// 	//  W2 = UPB ^ Uh1
// 	//  W3 = UPB ^ Uh2
// 	//  W4 = Uns
// 	//  W5 = Unb ^ Uh1
// 	//  W6 = Unb ^ Uh2
// 	// 
// 	// An+1 = w1*(1 - An)*<something> +..+..-..-..-;
	
						 
// }

// // (12) Inhomogeneous Insolation
// // Just uses ratios of Power and Current to find a(T)
// // P(t) = a(T)*Imeas(t)
// // Pmeas(t) > P(t)
// // 
// // Same as (3) but better written
// // 
// void vmpptMethodINHOMO( void )
// {

						 
						 
	
// }

/** PID Controller
* Input: Current Error, current time (in ms?)
* output: PID Control Value to be used by pwm update function
*/

int16_t PID::updatePID( int16_t fcurrErr, uint32_t nowtime )
{
	// Static Values
	static float diff;
    static float p_term;
    static float i_term;
    static float d_term;
    static uint32_t fdT;

    // Calc time dT
    fdT = nowtime - u32PrevTime;
    u32PrevTime = nowtime;
 
    // integration with windup guarding
    fIntErr += ( fcurrErr * fdT );
    if ( fIntErr < -( fAntiWindup ) )
    	fIntErr = -( fAntiWindup );
    else if (fIntErr > fAntiWindup )
        fIntErr = fAntiWindup;
 
    // differentiation
    diff = ((fcurrErr - fPrevErr) / (uint32_t) fdT);
 
    // scaling
    p_term = (fPgain * fcurrErr);
    i_term = (fIgain * fIntErr);
    d_term = (fDgain * diff);
 
    // summation of terms
    fCtrl = (int16_t)(p_term + i_term + d_term);
 
    // save current error as previous error for next iteration
    fPrevErr = fcurrErr;

    return fCtrl;
}

/** Settings::Get 
* how to return the value if there's multiple types of variables?  
* Could use same for all, but some don't need tha t much memory
*/
