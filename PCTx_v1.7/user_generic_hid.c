/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include <usart.h>
#include <delays.h>
#include "typedefs.h"
#include "usb.h"
#include "io_cfg.h"             // I/O pin mapping
#include "user_generic_hid.h"

/** V A R I A B L E S ********************************************************/
#pragma udata

byte number_of_bytes_read;
unsigned char transmit_buffer[EP0_BUFF_SIZE];
unsigned char receive_buffer[EP0_BUFF_SIZE];

int count;
int i;
unsigned int pulse1, pulse2, pulse3, pulse4, pulse5, pulse6, pulse7, pulse8, pulse9;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
void BlinkUSBStatus(void);
void ReportLoopback(void);
void HandleControlOutReport(void);

/** D E C L A R A T I O N S **************************************************/
#pragma code
void UserInit(void) {
	TRISCbits.TRISC0 = 0; //pin c0 output

	//pre/post scale registers from right to left in increasing order
	// Timer2 prescale = 16
	T2CONbits.T2CKPS0 = 0;
	T2CONbits.T2CKPS1 = 1;
	
	// Timer2 postscale = 16
	T2CONbits.T2OUTPS0 = 1;
	T2CONbits.T2OUTPS1 = 1;
	T2CONbits.T2OUTPS2 = 1;
	T2CONbits.T2OUTPS3 = 1;

	// Timer period
	PR2 = 0xE9;  //233
	//83.3*16*16*(233+1)= 5ms

	// Timer2 is on
	T2CONbits.TMR2ON = 1;

    // Enable Timer2 interrupt
    PIE1bits.TMR2IE = 1;

	//count = 2;
	//120 = 1ms
	//240 = 2ms
	//36 = .3ms

	//vista needs this to be small otherwise err code 10 in OS
	pulse1 = -1; //a check for vista
	pulse2 = 1;
	pulse3 = 1;
	pulse4 = 1;
	pulse5 = 1;
	pulse6 = 1;
	pulse7 = 1;
	pulse8 = 1;
	pulse8 = 1;
	pulse9 = 1;
}


void ProcessIO(void) {   
    if((usb_device_state < CONFIGURED_STATE)||(UCONbits.SUSPND==1)) { 
		return;
	}
    
	ReportLoopback();

	if (pulse1 != -1){ //fixed the error code 10 message in vista
	    if (PIR1bits.TMR2IF) {			// if the timer interrupt bit is set
				T2CONbits.TMR2ON = 0; 	//turn off the timer
	
				LATCbits.LATC0 = 0;
				Delay100TCYx(36);
	
				//C0
	    		LATCbits.LATC0 = 1; 	
				Delay100TCYx(pulse1); 	
	    		LATCbits.LATC0 = 0; 
	
				Delay100TCYx(36);
	
	    		LATCbits.LATC0 = 1; 	
				Delay100TCYx(pulse2); 	
	    		LATCbits.LATC0 = 0; 
	
				Delay100TCYx(36);
	
	    		LATCbits.LATC0 = 1; 	
				Delay100TCYx(pulse3); 	
	    		LATCbits.LATC0 = 0;
	
				Delay100TCYx(36);
	
	    		LATCbits.LATC0 = 1; 	
				Delay100TCYx(pulse4); 	
	    		LATCbits.LATC0 = 0;
	
				Delay100TCYx(36);
	
	    		LATCbits.LATC0 = 1; 	
				Delay100TCYx(pulse5); 	
	    		LATCbits.LATC0 = 0; 
	
				Delay100TCYx(36);
	
	    		LATCbits.LATC0 = 1; 	
				Delay100TCYx(pulse6); 	
	    		LATCbits.LATC0 = 0; 
	
				Delay100TCYx(36);
	
	    		LATCbits.LATC0 = 1; 	
				Delay100TCYx(pulse7); 	
	    		LATCbits.LATC0 = 0;
	
				Delay100TCYx(36);
	
	    		LATCbits.LATC0 = 1; 	
				Delay100TCYx(pulse8); 	
	    		LATCbits.LATC0 = 0;
	
				Delay100TCYx(36);
	
	    		LATCbits.LATC0 = 1; 	
				Delay100TCYx(pulse9); 	
	    		LATCbits.LATC0 = 0;
	
				Delay100TCYx(36);
				LATCbits.LATC0 = 1;		
	
		        PIR1bits.TMR2IF = 0; 	//reset the timer
				T2CONbits.TMR2ON = 1; 	//turn the timer on 
		} 
	}
}


void ReportLoopback(void) {
	byte count;

	// Find out if an Output report has been received from the host.
	number_of_bytes_read = HIDRxReport(receive_buffer, HID_OUTPUT_REPORT_BYTES);

	if (number_of_bytes_read > 0) {
		// An Output report was received.
		pulse1 = receive_buffer[0]; //servo1
		pulse2 = receive_buffer[1]; //servo2
		pulse3 = receive_buffer[2]; //servo3
		pulse4 = receive_buffer[3]; //servo4
		pulse5 = receive_buffer[4]; //servo5
		pulse6 = receive_buffer[5]; //servo6
		pulse7 = receive_buffer[6]; //servo7
		pulse8 = receive_buffer[7]; //servo8
		pulse9 = receive_buffer[8]; //servo9

		for (count = 1; count <= HID_OUTPUT_REPORT_BYTES; count = count + 1) {				
//			transmit_buffer[count-1] = receive_buffer[count-1];			
			transmit_buffer[count-1] = 0;			
		}

		// If necessary, wait until the interrupt IN endpoint isn't busy.
		while(mHIDTxIsBusy()) {
			// Service USB interrupts.
			USBDriverService(); 
		}
		// The report will be sent in the next interrupt IN transfer.
   	    HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
	}
}


void HandleControlOutReport()
{
	byte count;

	// Find out if an Output or Feature report has arrived on the control pipe.
	// Get the report type from the Setup packet.

	switch (MSB(SetupPkt.W_Value))
    {
		case 0x02: // Output report 

			// Get the report ID from the Setup packet.

    		switch(LSB(SetupPkt.W_Value))
		    {
				case 0: // Report ID 0

					// This example application copies the Output report data 
					// to hid_report_in. 
					// (Assumes Input and Output reports are the same length.)
					// A "real" application would do something more useful with the data.

				    // wCount holds the number of bytes read in the Data stage.
					// This example assumes the report fits in one transaction.	
				
					for (count = 1; count <= HID_OUTPUT_REPORT_BYTES; count = count + 1)
					{
						hid_report_in[count-1] = hid_report_out[count-1];
					}				
					// The number of bytes in the report (from usbcfg.h).
		
					break;		
			} // end switch(LSB(SetupPkt.W_Value))

		case 0x03: // Feature report 

			// Get the report ID from the Setup packet.

    		switch(LSB(SetupPkt.W_Value))
		    {
				case 0: // Report ID 0

				// The Feature report data is in hid_report_feature.
				// This example application just sends the data back in the next
				// Get_Report request for a Feature report.			
			
			    // wCount holds the number of bytes read in the Data stage.
				// This example assumes the report fits in one transaction.	
		
				break;
			} // end switch(LSB(SetupPkt.W_Value))		

	} // end switch(MSB(SetupPkt.W_Value))

} // end HandleControlOutReport


void BlinkUSBStatus(void)
{
    static word led_count=0;
    
    if(led_count == 0)led_count = 10000U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(UCONbits.SUSPND == 1)
    {
        if(led_count==0)
        {
            mLED_1_Toggle();
            mLED_2 = mLED_1;        // Both blink at the same time
        }//end if
    }
    else
    {
        if(usb_device_state == DETACHED_STATE)
        {
            mLED_Both_Off();
        }
        else if(usb_device_state == ATTACHED_STATE)
        {
            mLED_Both_On();
        }
        else if(usb_device_state == POWERED_STATE)
        {
            mLED_Only_1_On();
        }
        else if(usb_device_state == DEFAULT_STATE)
        {
            mLED_Only_2_On();
        }
        else if(usb_device_state == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        }
        else if(usb_device_state == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
                mLED_1_Toggle();
                mLED_2 = !mLED_1;       // Alternate blink                
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus


void GetInputReport0(void)
{
	byte count;

	// Set pSrc.bRam to point to the report.
	// 

	pSrc.bRam = (byte*)&hid_report_in;

	// The number of bytes in the report (from usbcfg.h).

	LSB(wCount) = HID_INPUT_REPORT_BYTES;

} // end GetInputReport0


void GetFeatureReport0(void)
{
	byte count;

	// Set pSrc.bRam to point to the report.

	pSrc.bRam = (byte*)&hid_report_feature;

	// The number of bytes in the report (from usbcfg.h).

    LSB(wCount) = HID_FEATURE_REPORT_BYTES;

} // end GetFeatureReport0

/** EOF user_generic_hid.c *********************************************************/
