// 4-ch PPM generator

// PPM0 and PPM1 on OC1A and OC1B pin, high resolution. Resolution = 0.5us (2000 degree)
// The Timer1 of AVR could preform 2 dedicated PMM generator

// PPM2 and PPM3 on OC0A and OC0B pin, semi-high resolution. Resolution = 16us (63 degree)
// Raise on T1 overflow, fall on T0 compare match

#define PPM_ZONE 500 //Recommended value is 500, 
// PPM signal could be high for 1500 +/- PPM_ZONE micro seconds. For example, PPM_ZONE = 500 gives 1ms to 2ms PPM signal, PPM_ZONE = 1000 gives 0.5ms to 2.5ms PPM signal
// Some ESCs or servos accept PPM_ZONE 1000, this vary from manufactures. Test it first.

#define PPM_MAXDIFF_MAIN	PPM_ZONE/0.5
#define PPM_MAXDIFF_AUX		PPM_ZONE/16.0

// PPM generator init
void initPPM() {
	//Setup PPM0 and PPM1
	TCCR1A = (2<<COM1A0) | (2<<COM1B0) | (2<<WGM10); //Mode 14: Fast PWM, set at zero, clear on compare match
	TCCR1B = (3<<WGM12) | (2<<CS10); //Clk scaler = 8 => PWM resolution = 0.5us, range = 32.8ms
	ICR1 = 39999; //OF every 20ms
	OCR1A = 3000; //Allow range PPM_MAXDIFF_MAIN
	OCR1B = 3000; //Default position at center
	
	//Setup PPM2 and PPM3
	TIMSK1 = (1<<TOIE1); //When T1 overflow (reaches ICR1), software-ly set the pin
	TCCR0A = (2<<COM0A0) | (2<<COM0B0); //Mode 0: Free running timer. When T0 reaches the compare value, software-ly clear the pin
	TCCR0B = (4<<CS00); //Clk scaler = 256 => PWM resolution = 16us, range = 4.1ms. A 128 scaler will be better, but the highest scaler under 256 is 64.
	OCR0A = 94; //Allow range 94 +/- PPM_MAXDIFF_AUX
	OCR0B = 94;
	
	/*
	Notice:
	According to servo PPM regulation, the period is 20ms, and the signal should be high for 1 to 2ms.
	Because the CPU speed is 16MHz, it is clear to say that, the PPM signal should be high for 16000 to 32000 cycles.
	T1 is the main PPM generator, it controls the raising and falling of the PPM output pins
	Therefore, T1 needs to cover the full 20ms PPM cycle. Since T1 is 16-bit wide, this is not a issue.
	T0 is the aux PPM generator. Since T0 is 8-bit wide, some trick is required.
	If T0 needs to cover the full 20ms period, the resolution will be so poor. (ideal resolution = 256 / 20 * (2.5-1.5) = 25.6)
	The trick is that, T0 only covers the period which the signal may be high. In another word, the first 2ms of the 20ms period.
	The main PPM generator (T1) will raise pins for aux PPM generator (T0).
	The aux PPM generator is in normal free running mode, and the compare match only clears the PPM output pin.
	Doing so increase the resolution of aux PPM generator.
	It is possible that the T0 count for more than one full revolution in the 20ms PPM period, and this will cause the T0 compare match for more than one times.
	However, since the T0 compare match only clear the output pin, and the pin should be remain low after the first match in the 20ms PPM period.
	It is OK to re-clear the pin. There is no any effect due to the extra T0 match.
	*/
}

// Reset AUX PPM generator (PPM2 and PPM3)
ISR (TIMER1_OVF_vect) {
	TCCR0B = (0<<CS00); //Pause T0
	TCNT0 = 0x00; //Reset aux PPM generator (T0), set PPM2 and PPM3 pin
	TCCR0A = (3<<COM0A0) | (3<<COM0B0); //Force compare to raise the pins (GPIO is disconnected, modify the wave gen to access the output pin)
	TCCR0B |= (1<<FOC0A) | (1<<FOC0B);
	TCCR0A = (2<<COM0A0) | (2<<COM0B0); //Restore the wave gen behavior and resume T0
	TCCR0B = (4<<CS00);
}

// Set PPM value to PPM0 to PPM3 pin.
// Since the controller (e.g. PID) gives result in float, these setter will accept float. Float has poor performance, but, better controllability is more important.
// Accept range +/- 1.0
void setPPM0(float position) {
	if (position > 1.0)
		OCR1A = 3000 + PPM_MAXDIFF_MAIN;
	else if (position < -1.0)
		OCR1A = 3000 - PPM_MAXDIFF_MAIN;
	else
		OCR1A = 3000 + PPM_MAXDIFF_MAIN * position;
}

// Set PPM value to PPM1
void setPPM1(float position) {
	if (position > 1.0)
		OCR1B = 3000 + PPM_MAXDIFF_MAIN;
	else if (position < -1.0)
		OCR1B = 3000 - PPM_MAXDIFF_MAIN;
	else
		OCR1B = 3000 + PPM_MAXDIFF_MAIN * position;
}

// Set PPM value to PPM2
void setPPM2(float position) {
	if (position > 1.0)
		OCR0A = 94 + PPM_MAXDIFF_AUX;
	else if (position < -1.0)
		OCR0A = 94 - PPM_MAXDIFF_AUX;
	else
		OCR0A = 94 + PPM_MAXDIFF_AUX * position;
}

// Set PPM value to PPM3
void setPPM3(float position) {
	if (position > 1.0)
		OCR0B = 94 + PPM_MAXDIFF_AUX;
	else if (position < -1.0)
		OCR0B = 94 - PPM_MAXDIFF_AUX;
	else
		OCR0B = 94 + PPM_MAXDIFF_AUX * position;
}