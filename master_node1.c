/*************************************************************************
 *  Timestamp free synchronization
 *  Master node code
 *  DRB Apr 10, 2015
 *  Uses time reversal ideas developed on Apr 9, 2015
 *  States:
 *  0 : searching
 *  1 : recording
 *  2 : waiting for clock tick
 *  3 : waiting to play back response
 *  4 : playing back response
 *
 *************************************************************************/

#define CHIP_6713 1

// length of searching window in samples
#define M 40

// threshold value for searching window
#define T1 100000

// sinc pulse normalized bandwidth
#define BW 0.0125

// 2*N+1 is the number of samples in the sinc function
#define N 512

// virtual clock counter
#define L 4096

// set this to one to pass through the left channel
// (useful for debugging master node functionality)
// or zero to not pass through (should normally be zero)
#define PASSTHROUGH 0

// define PI and INVPI
#define PI 3.14159265358979323846
#define INVPI 0.318309886183791

#include <stdio.h>
#include <c6x.h>
#include <csl.h>
#include <csl_mcbsp.h>
#include <csl_irq.h>
#include <math.h>

#include "dsk6713.h"
#include "dsk6713_aic23.h"
#include "dsk6713_led.h"

// ------------------------------------------
// start of variables
// ------------------------------------------
float buf[M];       // search buffer
float mfc[M];		// in-phase correlation buffer
float mfs[M];       // quadrature correlation buffer
float corr_max, corr_max_s, corr_max_c; // correlation variables
float corr_c[2*M];
float corr_s[2*M];
float s[2*M];
short corr_max_lag;
short bufindex = 0;
short i,j;
float zc, zs, z;
double t,x,y;
short wait_count = 0;
short clockbuf[L];  // clock buffer (right channel)
short recbuf[2*N+2*M]; // recording buffer
short recbufindex = 0;
short max_recbuf = 0;
short playback_scale = 1;
int state = 0;
short vclock_counter = 0; // virtual clock counter
short recbuf_start_clock = 0; // virtual clock counter for first sample in recording buffer
char r = 0;
short max_samp = 0;

short ledTriggered = 0; //Debug  variable, check if led has been triggered before in this cycle.
DSK6713_AIC23_CodecHandle hCodec;							// Codec handle
DSK6713_AIC23_Config config = DSK6713_AIC23_DEFAULTCONFIG;  // Codec configuration with default settings
// ------------------------------------------
// end of variables
// ------------------------------------------

double sin(double);
double cos(double);
interrupt void serialPortRcvISR(void);

void main()
{

	// set up the cosine and sin matched filters for searching
	// also initialize searching buffer
	for (i=0;i<M;i++){
		t = i*0.25;				// time
		y = cos(2*PI*t);		// cosine matched filter (double)
		mfc[i] = (float) y;		// cast and store
		y = sin(2*PI*t);		// sine matched filter (double)
		mfs[i] = (float) y;     // cast and store
		buf[i] = 0;             // clear searching buffer
	}

	// initialize clock buffer
	for (i=0;i<L;i++)
		clockbuf[i] = 0;

	// set up clock buffer to play modulated sinc centered at zero
	for (i=-N;i<=N;i++){
		x = i*BW;
		if (i!=0) {
			t = i*0.25;
			y = 32767.0*cos(2*PI*t)*sin(PI*x)/(PI*x); // double
		}
		else {
			y = 32767.0;
		}
		j = i;
		if (j<0) {
			j += L; // wrap
		}
		clockbuf[j] = (short) y;
	}



	DSK6713_init();		// Initialize the board support library, must be called first
	DSK6713_LED_init(); // initialize LEDs
    hCodec = DSK6713_AIC23_openCodec(0, &config);	// open codec and get handle

	// Configure buffered serial ports for 32 bit operation
	// This allows transfer of both right and left channels in one read/write
	MCBSP_FSETS(SPCR1, RINTM, FRM);
	MCBSP_FSETS(SPCR1, XINTM, FRM);
	MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);

	// set codec sampling frequency
	DSK6713_AIC23_setFreq(hCodec, DSK6713_AIC23_FREQ_8KHZ);

	// interrupt setup
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt
	IRQ_map(IRQ_EVT_RINT1,15);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts

	while(1)						// main loop
	{
	}
}

interrupt void serialPortRcvISR()
{
	union {Uint32 combo; short channel[2];} temp;

	temp.combo = MCBSP_read(DSK6713_AIC23_DATAHANDLE);
	// Note that right channel is in temp.channel[0]
	// Note that left channel is in temp.channel[1]

	// keep track of largest sample for diagnostics
	if (temp.channel[0]>max_samp)
		max_samp = temp.channel[0];

	if (state==0) {  // SEARCHING STATE

        // put sample in searching buffer
		buf[bufindex] = (float) temp.channel[0];  // right channel
		if (PASSTHROUGH!=1) {
			temp.channel[0] = 0;  // can comment this out for debugging at home
		}

	    // compute incoherent correlation
	    zc = 0;
	    zs = 0;
		for(i=0;i<M;i++) {
			zc += mfc[i]*buf[i];
			zs += mfs[i]*buf[i];
		}
		z = zc*zc+zs*zs;

		if (z>T1) {  				// threshold exceeded?
			max_recbuf = 0;
			state = 1; 				// enter "recording" state (takes effect in next interrupt)
			recbufindex = M;		// start recording new samples at position M
			j = bufindex;			//
			for (i=0;i<M;i++){  	// copy samples from buf to first M elements of recbuf
				j++;   				// the first time through, this puts us at the oldest sample
				if (j>=M)
					j=0;
				recbuf[i] = (short) buf[j];
				buf[j] = 0;  		// clear out searching buffer to avoid false trigger
			}
		}
		else {
			// increment and wrap pointer
		    bufindex++;
		    if (bufindex>=M)
		    	bufindex = 0;
		}

	}
	else if (state==1) { // RECORDING STATE
		if(!ledTriggered)
				{
					DSK6713_LED_toggle(0);
					ledTriggered = 1;
				}
		// put sample in recording buffer
		recbuf[recbufindex] = temp.channel[0];  // right channel
		if (PASSTHROUGH!=1) {
			temp.channel[0] = 0;  // can comment this out for debugging at home
		}
		if (abs(recbuf[recbufindex])>max_recbuf) {
			max_recbuf = abs(recbuf[recbufindex]); // keep track of largest sample
		}
		recbufindex++;
		if (recbufindex>=(2*N+2*M)) {
			state = 2;  		// buffer is full (stop recording and start counting samples to next clock tick)
			recbufindex = 0; 	// shouldn't be necessary
			wait_count = 0;     // reset the wait counter
			if (max_recbuf<2048)
				playback_scale = 0;  // don't send response (signal was too weak)
			else if (max_recbuf<4096)
				playback_scale = 8;  // reply and scale by 8
			else if (max_recbuf<8192)
				playback_scale = 4;  // reply and scale by 4
			else if (max_recbuf<16384)
				playback_scale = 2;  // reply and scale by 2
			else
				playback_scale = 1;  // no scaling

		}
	}
	else if (state==2) { // WAITING STATE (counting up samples until clock tick)
		if(ledTriggered)
				{
					DSK6713_LED_toggle(0);
					ledTriggered = 0;
				}
		if (PASSTHROUGH!=1) {
			temp.channel[0] = 0;  // can comment this out for debugging at home
		}
		wait_count++;
		// only way out of this state is when the virtual clock rolls over
	}
	else if (state==3) { // WAITING STATE (counting down samples until reply)
		if (PASSTHROUGH!=1) {
			temp.channel[0] = 0;  // can comment this out for debugging at home
		}
		wait_count--;
		if (wait_count<0) {
			state = 4;  // done waiting, time to play back rec_buffer in reverse
			recbufindex = 2*N+2*M;
		}
	}
	else if (state==4) { // RESPONSE STATE (play back recording buffer in reverse)
		recbufindex--;
		if (recbufindex>=0) {
			temp.channel[0] = playback_scale*recbuf[recbufindex];
		}
		else
		{
			state = 0;  // go back to searching
		}
	}

	temp.channel[1] = clockbuf[vclock_counter];  // master clock signal (always played)

	MCBSP_write(DSK6713_AIC23_DATAHANDLE, temp.combo); // output L/R channels

	// update virtual clock (cycles from 0 to L-1)
	vclock_counter++;
	if (vclock_counter>=L) {
		vclock_counter = 0; // clock tick occurred, wrap
		if (state==2) {
			state = 3;
		}
	}

}

