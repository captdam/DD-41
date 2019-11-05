// String util

// Unsigned int to BCD (range 0 to 9999)
uint16_t uint2bcd(uint16_t binary) {
	if (binary >= 9999)
		return 0x9999;
		
	uint8_t i3 = binary / 1000;
	uint8_t i2 = binary / 100 % 10;
	uint8_t i1 = binary / 10 % 10;
	uint8_t i0 = binary % 10;

	return (i3<<12) | (i2<<8) | (i1<<4) | i0;
}

// BCD to unsigned int (range 0 to 999)
uint16_t bcd2uint(uint16_t bcd) {
	uint16_t ans = bcd & 0x000F;
	
	bcd = bcd >> 4;
	ans += ( bcd & 0x000F ) * 10;
	
	bcd = bcd >> 4;
	ans += ( bcd & 0x000F ) * 100;
	
	bcd = bcd >> 4;
	ans += ( bcd & 0x000F ) * 1000;
	
	return ans;
}

// BCD to OSD (On Screen Display module encoding)
uint16_t bcd2osd(uint8_t bcd) {
	uint8_t h, l;
	
	h = bcd >> 4;
	if (!h) h = 0x0A; //Char code for 0 is 0x0A

	l = bcd & 0x0F;
	if (!l) l = 0x0A;

	return (h<<8) | l;
}