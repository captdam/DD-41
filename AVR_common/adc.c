// ADC util
// Polling method <-- ADC is not slow, using polling method simples the software compre to interrupt

#define ADC_ADMUX_BASE (1<<REFS0) //Vref = AVcc
#define ADC_ADCSRA_BASE ( (1<<ADEN) | (7<<ADPS0) ) //Enable ADC, , ADC clk = Sys clk / 128 = 125kHz (best range 50kHz-200kHz)

// ADC hardware init
void initADC() {
	ADMUX = ADC_ADMUX_BASE;
	ADCSRA = ADC_ADCSRA_BASE;
	DIDR0 = 0b00001111; //ADC channel 0 to 3 is for ADC only, disable the digital input buffer could reduce power consumption
}

// Polling ADC value
uint16_t getADC(uint8_t channel) {
	ADMUX = ADC_ADMUX_BASE | ( (channel<<MUX0)&0x0F ); //Select ADC channel
	ADCSRA = ADC_ADCSRA_BASE | (1<<ADSC); //Start ADC
	while ( ADCSRA & (1<<ADSC) ); //Wait (by polling) until ADC finished
	return ADC; //Get ADC result (MSB 8 bits)
}