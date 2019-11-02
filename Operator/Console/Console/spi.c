// SPI util - Master
// Polling method <-- SPI is not slow, using polling method simples the software compre to interrupt
// SPI frequency = 4MHz

// Init SPI
void initSPIMaster() {
	DDRB |= (1<<5) | (1<<3); //PORTB5=SCK, B4=MISO, B3=MOSI
	DDRB &= ~(1<<4);
	SPCR = (1<<SPE) | (1<<MSTR) | (0<<SPR0); //Enable SPI, no interrupt, MSB first, as master, SCK low on idle, f_SPI = CPU_FREQ / 4 = 4MHz @ 16MHz CPU
}

// Write on SPI bus
uint8_t shiftSPIMaster(uint8_t data) {
	SPDR = data;
	while ( !(SPSR & (1<<SPIF)) ); //Wait until data send
	return SPDR;
}
