// UART-based data exchange util

// 9600 BAUD, 1 stop bit, 8 data bits. Full-duplex independent design
// 32 bytes packet, plus 1 byte checksum

// It takes 34ms to send 32 byte data and 1 byte checksum
// The module will send packet every 40ms (25 packet exchange exchange per 1 second)
// To identify packet starting: if the new word from UART is more than 4 ms later than the last word, it is the starting of new packet
// It is possible that: 4 consecutively word lost resulting in "new packet"
// In this case, because the length is not enough, packet will be dropped

/*
Notice:
Timer2 will be used to start packet sending process and measure interval between UART words, because:
- T2 is running in SLEEP mode.
- T0 and T1 is used on ROV side for PPM generator.
*/

#define BAUDRATE	9600
#define PACKETSIZE	32
#define EXCHANGETIME	40
#define BLANKTIME	4

volatile uint8_t txAppBuffer[PACKETSIZE], txBuffer[PACKETSIZE], txPointer;
volatile uint8_t rxAppBuffer[PACKETSIZE], rxBuffer[PACKETSIZE], rxPointer, rxChecksum;

volatile uint8_t exchangeEventTimer, uartBlankTimer; //Exchange packet every 40ms.
volatile uint8_t txBufferSwapRequest, rxBufferSwapRequest;

void initUART() {
	UBRR0 = RC_CLOCK_FREQ/16/BAUDRATE - 1;
	UCSR0B = (1<<RXCIE0) | (1<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0); //Enable Tx, Rx, Tx interrupt and Rx interrupt
	UCSR0C = (0<<USBS0) | (3<<UCSZ00); //Async USART (UART), no parity, 1 stop bit, 8 bits data
	txPointer = 0;
	rxPointer = 0;
}

void initSysTimer() {
	TCCR2A = (2<<WGM20); //CTC mode
	TCCR2B = (5<<CS20); //Prescaler = 128, @ 16MHz = 125 cycle per 1ms
	OCR2A = 125;
	TIMSK2 = (1<<OCIE2A); //Enable ISR
	exchangeEventTimer = 0; //Reset timer (high byte)
	uartBlankTimer = 0;
}

// Application call this procedure to place a 32-byte packet to tx buffer
// The packet will later be copied into tx working buffer and then send
void placePacket(uint8_t data[]) {
	cli();
	for (uint8_t i = 0; i < PACKETSIZE; i++) txAppBuffer[i] = data[i];
	sei();
	
	/*
	Notice:
	The communication module is working independently; hence it is possible to see read-while-write.
	If the data exchange module read the txAppBuffer while application is writing txAppBuffer, a packet combined with old and new data
	will be send to other side and cause some issues.
	To prevent this, disable interrupts to prevent read-while-write.
	*/
}

// Application call this procedure to fetch a 32-byte packet from rx buffer
// This procedure is PASS BY REFERENCE
void fetchPacket(uint8_t data[]) {
	cli();
	for (uint8_t i = 0; i < PACKETSIZE; i++) data[i] = rxAppBuffer[i];
	sei();
	
	/*
	Notice:
	The communication module is working independently; hence it is possible to see write-while-read.
	If the data exchange module write the rxAppBuffer while application is reading rxAppBuffer, a packet combined with old and new data
	will be fetch to application side and cause some issues.
	To prevent this, disable interrupts to prevent write-while-read.
	*/
}


ISR (TIMER2_COMPA_vect) {
	if (exchangeEventTimer++ >= EXCHANGETIME) { //Exchange packet every X interval
		exchangeEventTimer = 0;
		
		uint8_t checksum = 0; //Copy packet from application-side buffer into uart-side buffer and get checksum
		for (uint8_t i = 0; i < PACKETSIZE; i++) {
			txBuffer[i] = txAppBuffer[i];
			checksum -= txAppBuffer[i];
		}
		
		UDR0 = checksum; //Send first byte (checksum, not in the buffer) and start Tx
		txPointer = 0;
	}
	
	if (uartBlankTimer < 0xFF) uartBlankTimer++;
}

ISR (USART_TX_vect) {
	if (txPointer < PACKETSIZE) UDR0 = txBuffer[txPointer++]; //Send data is packet is not fully send; otherwise, halt Tx
}

ISR (USART_RX_vect) {
	uint8_t data = UDR0; //Clear RxC flag and reset blank timer

	if (uartBlankTimer > BLANKTIME) { //Starting of packet detected
		rxChecksum = data;
		rxPointer = 0;
	}
	else {
		if (rxPointer < PACKETSIZE) {
			rxBuffer[rxPointer++] = data;
			rxChecksum += data;
			
			if (rxPointer == PACKETSIZE && rxChecksum == 0) { //Last word of the packet detected and packet verified
				for (uint8_t i = 0; i < PACKETSIZE; i++) rxAppBuffer[i] = rxBuffer[i];
			}
		}
		//If more words come, ignore them
	}
	
	uartBlankTimer = 0;
}

