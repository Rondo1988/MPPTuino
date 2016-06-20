/** 
* Teensy 3.1 Arduino MPPT
* Dustin Sanders
* 2016/6/18
**/

#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <ADC.h>
// Global Defines, including default settings, please check settings! 
#include "globals.h"
#include "mppt.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define USBDEBUG

// Main Global Variables

MPPT xTracker;
FlexCAN xCANbus(500000);
IntervalTimer mpptTimer;
/**
Create ADC object
*/
ADC *adc = new ADC(); // adc object

//uint32_t mpptLastTime = 0;
uint32_t telLastTime = 0;
uint32_t heartLastTime = 0;





// ISR(TIMER1_COMPA_vect){
	
// 	#ifdef USBDEBUG
// 	Serial.println( "MPPT Task" );
// 	#endif
// 	  /** get sensor values **/
// 	xTracker.vGetSensors();

// 	/** TODO: SET MODE FUNCTION **/


// 	/** Run Algorithm **/
// 	xTracker.vRunAlgorithm();
// }

void prvMPPTTask( void  )
{
  /* The parameters are not used.  Prevent compiler warnings. */
  //( void ) pvParameters;
 
 	#ifdef USBDEBUG
	Serial.println( "MPPT Task" );
	#endif
    /** get sensor values **/
  xTracker.vGetSensors();

  /** TODO: SET MODE FUNCTION **/


  /** Run Algorithm **/
  xTracker.vRunAlgorithm();

  
}

void prvTelemetryTask( void )
{
    /** The parameters are not used.  Prevent compiler warnings. **/
    //( void ) pvParameters;



    /** Power Data Struct Creation **/
    static power_data_t data;

    /** CAN Message Variable **/
    static CAN_message_t msg;

    //static float fconversion = 3.3/adc->getMaxValue(ADC_0);

    // for( ;; )
    // {

        /** Get Power Data **/
        data = xTracker.xGetData();

        /** Extract and send data in 2 chunks 0x2?[0|1] where ? is 
        * the set MPPT ID **/

        /** TODO: FORMAT CAN DATA FOR VOLTS **/

        /** MSG 0 Packet
        *  [5:4]        |   [3:2]           |   [1]         |   [0]
        *  MPPTN_TEMP   |   MPPTN_P_TEMP    |   MPPTN_MODE  |   MPPTN_ERR 
        **/

        msg.len = 6;
        msg.id = 0x200 | ( (uint8_t)(xTracker.vGetMPPTId() ) << 4 );
        msg.buf[0] = (uint16_t)data.Errors;
        msg.buf[1] = (uint16_t)data.Mode;
        msg.buf[2] = (uint16_t)data.PVT;        // LSBs
        msg.buf[3] = (uint16_t)(data.PVT) >> 8;   // MSBs
        msg.buf[4] = (uint16_t)data.BrdT;       // LSBs
        msg.buf[5] = (uint16_t)(data.BrdT) >> 8;  // MSBs

        /** Send CAN msg 0 **/
        xCANbus.write(msg);

        /** MSG 1 Packet         
        *  [7:6]        |   [5:4]       |   [3:2]       |   [1:0]   
        *   MPPTN_OUT_I |   MPPTN_IN_I  |   MPPTN_OUT_V |   MPPTN_IN_V
        **/
        msg.len = 8;
        msg.id = 0x200 | ( (uint8_t)(xTracker.vGetMPPTId() ) << 4 ) | 0x001;
        msg.buf[0] = (uint16_t)data.PVV;        // LSBs
        msg.buf[1] = (uint16_t)(data.PVV) >> 8;   // MSBs
        msg.buf[2] = (uint16_t)data.HVV;        // LSBs
        msg.buf[3] = (uint16_t)(data.HVV) >> 8;   // MSBs
        msg.buf[4] = (uint16_t)data.PVI;        // LSBs
        msg.buf[5] = (uint16_t)(data.PVI) >> 8;   // MSBs
        msg.buf[6] = (uint16_t)data.HVI;        // LSBs
        msg.buf[7] = (uint16_t)(data.HVI) >> 8;   // MSBs

        /** Send CAN msg 1 **/
        xCANbus.write(msg);

        /** Send data out serial port
        * Serial data is formated to volts for ease of reading **/

        Serial.print( micros(), DEC );
        Serial.print( " ms, Mode: ");
        Serial.println(data.Mode, HEX);
        Serial.print( "PV I: " );
        Serial.println( data.PVI * globalAMPBITS , DEC );
        Serial.print( "PV V: ");
        Serial.println( data.PVV * globalSOLAR_VBITS , DEC );     
        Serial.print( "HV I: ");
        Serial.println( data.HVI * globalAMPBITS , DEC );
        Serial.print( "HV V: ");
        Serial.println( data.HVV * globalOUT_VBITS , DEC );  

        //vTaskDelay( mainTELEMETRY_MS );
    // }

}

//! Flashes an LED then waits configured time to flash again

// void prvHeartBeatLEDTask( void )
// {
    /* The parameters are not used.  Prevent compiler warnings. */
    //( void ) pvParameters;
  	// for(;;)
  	// {
    	// digitalWriteFast( globalHEARTBEAT_PIN, !digitalReadFast(globalHEARTBEAT_PIN) );
    	//vTaskDelay( mainHEARTBEAT_LED_FLASH_MS );
	    //digitalWriteFast( globalHEARTBEAT_PIN, LOW );
  		//vTaskDelay( mainHEARTBEAT_LED_TIMER_PERIOD_MS - mainHEARTBEAT_LED_FLASH_MS );
 	  //}
// }

/**
*   Timer which wakes up the MPPT task at the set time interval
*
*/
// void prvMPPTTimerCallback( TimerHandle_t xMPPTTimer )
// {
//     vTaskResume( gxMPPTTask );
// }


void setup() {

    // PIN setup

    // Diginal Outputs
    pinMode( globalHEARTBEAT_PIN, OUTPUT );
    pinMode( globalSTATUS1_LED_PIN, OUTPUT );
    pinMode( globalSTATUS2_LED_PIN, OUTPUT );
    pinMode( globalH0_PIN, OUTPUT );
    pinMode( globalH1_PIN, OUTPUT );   
    pinMode( globalL0_PIN, OUTPUT );
    pinMode( globalL1_PIN, OUTPUT );
    pinMode( globalCANCTRL_PIN, OUTPUT );

    CORE_PIN9_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;    //config teensy output port pins
    CORE_PIN10_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;    //config teensy output port pins
    CORE_PIN23_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;    //config teensy output port pins
    CORE_PIN22_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;    //config teensy output port pins

    // Analog Inputs
    pinMode( globalSOLAR_I_PIN, INPUT );
    pinMode( globalSOLAR_V_PIN, INPUT );
    pinMode( globalOUT_I_PIN, INPUT );
    pinMode( globalOUT_V_PIN, INPUT );
    pinMode( globalBOARD_T_PIM, INPUT );
    pinMode( globalSOLAR_T_PIN, INPUT );

    digitalWriteFast( globalHEARTBEAT_PIN, HIGH );
    Serial.begin(9600);
    
    #ifdef USBDEBUG
  
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    
    #endif

    Serial.println("Starting Setup");
    digitalWriteFast( globalHEARTBEAT_PIN, LOW );

    digitalWrite(globalL0_PIN, LOW);
    //digitalWrite(globalL1_PIN, LOW);
    digitalWrite(globalH0_PIN, LOW);
    //digitalWrite(globalH1_PIN, LOW);
    digitalWrite(globalSTATUS1_LED_PIN, LOW);

    /** TODO: Calibration Function **/
    /** START ADC
    * Current sensor has BW of 120KHz, so get samples at at least 240KHz
    * MPPT uses the analog data @100Hz, thats 150 samples with avging of 16.
    * is that too big for a buffer? 
    * Solar[I/V], Temperature ADC0
    *  Batt[I/V] ADC1
    * Use compare to to signal int for large jumps in current/voltage
    */
    /** Configure ADC0 */
    adc->setAveraging(8);
    adc->setResolution(12);
    adc->setConversionSpeed(ADC_MED_SPEED);
    adc->setSamplingSpeed(ADC_MED_SPEED);


    /** Configure ADC1 */
    adc->setAveraging(8, ADC_1);
    adc->setResolution(12, ADC_1);
    adc->setConversionSpeed(ADC_MED_SPEED, ADC_1);
    adc->setSamplingSpeed(ADC_MED_SPEED, ADC_1);


    /** START PWM 
    * Combined Mode
    * Page 837 in TMR
    * FTMEN = 1 • QUADEN = 0 • DECAPEN = 0 • COMBINE = 1, and • CPWMS = 0
    * DEADTIME sets the dead time between complimentary pulses
    * Need >150ns of deadtime. 
    * 8 counts = 160nS @ 93750 Hz frequency. 
    * FTM0_CH5_PIN 20 (changing to FTM0_CH2_PIN  9 ) = L1
    * FTM0_CH6_PIN 21 (changing to FTM0_CH3_PIN 10) = H1 
    * FTM0_CH0_PIN 22 = L0
    * FTM0_CH1_PIN 23 = H0
    */
    /** Setting Combine for N=0,1; COMBINE, COMP, DTEN AND SYNCEN = 1
    * per 36.3.14 FTMx_COMBINE
    * Set to 9 bits per table on http://www.pjrc.com/teensy/td_pulse.html **/
    
    analogWriteRes(9);
    analogWriteFrequency(globalH1_PIN, globalPWM_FREQ); /** sets all of FTM0 **/
    //digitalWrite(globalSTATUS1_LED_PIN, HIGH);
    FTM0_POL = 0;                  // Positive Polarity 
    FTM0_OUTMASK = 0xFF;           // Use mask to disable outputs
    FTM0_COMBINE = FTM_COMBINE_COMBINE0 | FTM_COMBINE_COMBINE1 | FTM_COMBINE_COMP0 | FTM_COMBINE_COMP1 | FTM_COMBINE_DTEN0 | FTM_COMBINE_DTEN1 | FTM_COMBINE_SYNCEN0 | FTM_COMBINE_SYNCEN1;
    FTM0_DEADTIME = FTM_DEADTIME_DTVAL(20) | FTM_DEADTIME_DTPS(0); // 0 PRESCALE, MAX DEADTIME (~1.2 uS)
    FTM0_SYNC = FTM_SYNC_REINIT | FTM_SYNC_SWSYNC; // SOFTWARE SYNC, REINITIALIZE COUNTER = 1
    // Set ELSB to 1 per table 36-67 in TRM
    // Use these not POL in order to change polarity of the H and L signals
    FTM0_C0SC = FTM_CSC_ELSB | FTM_CSC_MSB;
    FTM0_C1SC = FTM_CSC_ELSB | FTM_CSC_MSB;
    FTM0_C2SC = FTM_CSC_ELSA | FTM_CSC_MSB;
    FTM0_C3SC = FTM_CSC_ELSA | FTM_CSC_MSB;       
    // Clear the even channel values
    FTM0_C0V = 0;
    //FTM0_C1V = 0;
    FTM0_C2V = 0;
    //FTM0_C3V = 0;  
    // TODO: TEST THE OUTPUT OF THE PWM WITH INIT, OUTMASK AND FTMEN
    digitalWrite(globalSTATUS1_LED_PIN, HIGH);
    FTM0_OUTINIT = FTM_OUTINIT_CH3OI;
  
    digitalWrite(globalSTATUS1_LED_PIN, HIGH);
    FTM0_MODE = FTM_MODE_INIT;
    digitalWrite(globalSTATUS1_LED_PIN, LOW);
    FTM0_MODE |= FTM_MODE_FTMEN;

    digitalWrite(globalSTATUS1_LED_PIN, HIGH);
  
    /** FreeRTOS Task Creation */
    // xTaskCreate( prvMPPTTask , "MPPT", configMINIMAL_STACK_SIZE, NULL,  
    //     mainMPPT_TASK_PRIORITY, &gxMPPTTask );
    // Check for bad task creation
//    if ( gxMPPTTask == NULL ){
//        Serial.print("MPPT Task wasn't created! :-(\n");
//    }
//    else{
//        Serial.print("MPPT Task Created!\n");
//    }
    
    // xTaskCreate( prvHeartBeatLEDTask , "HRTB", configMINIMAL_STACK_SIZE, NULL, 
    //     mainHEARTBEAT_TASK_PRIORITY, &gxHeartBeatTask );
    // Check for bad task creation
//    if ( gxHeartBeatTask == NULL ){
//        Serial.print("Heartbeat Task wasn't created! :-(\n");
//    }
//    else{
//        Serial.print("Heartbeat Task Created!\n");
//    }
    
    // xTaskCreate( prvTelemetryTask , "TLM", configMINIMAL_STACK_SIZE, NULL,      
    //     mainTELEMETRY_TASK_PRIORITY, &gxTelemetryTask );    
    // Check for bad task creation
//    if ( gxTelemetryTask == NULL ){
//        Serial.print("Telemetry Task wasn't created! :-(\n");
//    }
//    else{
//        Serial.print("Telemetry Task Created!\n");
//    }
    
    // gxMPPTTimer = xTimerCreate( "MPPT", mainMPPT_DEFAULT_TIMER_PERIOD_MS, pdTRUE, 0,       
    //     prvMPPTTimerCallback );
    // Check for bad timer creation
//    if ( gxMPPTTimer == NULL ){
//        Serial.print("MPPT Timer wasn't created! :-(\n");
//    }
//    else{
//        Serial.print("MPPT Timer Created!\n");
//    }
//    
//
 

    /** Set to BOOST MODE for testing **/
    xTracker.setMode(mpptBOOST_MASK);


    /* Setup CAN */
    digitalWrite(globalCANCTRL_PIN, LOW);
    xCANbus.begin();
    // TODO: ADD VALUE FROM SETTINGS CLASS
    mpptTimer.begin(prvMPPTTask, 100000 ); // 100Hz 
    Serial.print("End of Setup\n");

}



void loop() {
    
    /** Poor Man (Open Loop) Scheduler **/
    // TODO: Use timers and Interrupt callbacks 
    // if ( micros() - mpptLastTime >= globalMPPT_MS )
    // {
    //     mpptLastTime = micros();
    //     prvMPPTTask();
    // }
    //else if ( mpptLastTime > micros() ) // handle overflow
    // {
    //     mpptLastTime = ((uint32_t)4294967295 - mpptLastTime) + micros();
    // }
    
    if ( micros() - telLastTime >= globalTELEMETRY_MS )
    {
        telLastTime = micros();
        prvTelemetryTask();
    }
    else if ( telLastTime > micros() ) // handle overflow
    {
        telLastTime = ((uint32_t)4294967295 - telLastTime) + micros();
    }
    
    if ( micros() - heartLastTime >= globalHEARTBEAT_MS )
    {
        heartLastTime = micros();
        digitalWriteFast( globalHEARTBEAT_PIN, !digitalReadFast(globalHEARTBEAT_PIN) );
    }
    else if ( heartLastTime > micros() ) // handle overflow
    {
        heartLastTime = ((uint32_t)4294967295 - heartLastTime) + micros();
    }

    // TODO: Keyboard input
    /** Handle Console Commands **/


  // put your main code here, to run repeatedly:
   //Serial.println( "MainLoop Start" );
  // xTimerStart( gxMPPTTimer, mainDONT_BLOCK );
  // vTaskStartScheduler( );

    //Serial.println( "uh oh, yhou shouldn't be here!" );
}
