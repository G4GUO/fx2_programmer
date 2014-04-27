#define ALLOCATE_EXTERN
#include <fx2regs.h>
//
// Read TRM p.15-115 for an explanation on this. 
// A single nop is sufficient for default setup but like that we're on 
// the safe side. 
//
#define	NOP		_asm nop _endasm
#define	SYNCDELAY	NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP

#define SET_RESET_BIT (IOC |= 0x80)
#define CLR_RESET_BIT (IOC &= 0x7F)
#define SET_nCONFIG_BIT (IOC |= 0x08)
#define CLR_nCONFIG_BIT (IOC &= 0xF7)
#define SET_LED_BIT (IOC |= 0x01)
#define CLR_LED_BIT (IOC &= 0xFE)
#define SET_TX_BIT (IOC |= 0x10)
#define CLR_TX_BIT (IOC &= 0xEF)

// Commands to FX2 via USB
#define RST_CMD      0x00
#define BK_CMD       0x01
#define TX_CMD       0x02
#define RX_CMD       0x03

// I2C devices
#define ADRF6755_ADD 0x80
#define AD7992_ADD   0x40
#define FLASH_ADD    0xAE
#define FX2_ADD      0x42
#define SI570_ADD    0xAA

//////////////////////////////////////////////////////////////////////////
//
// Important connections between FX2 and FPGA
//
// PC7 Reset
// PC6 nSTATUS
// PC5 FX2_ERROR
// PC4 TX Line
// PC3 nCONFIG
// PC1 CONF_DONE
// PC0 LED
//
// PA0 DATA0
// PA1 DCLK
//
//////////////////////////////////////////////////////////////////////////

static void test(void)
{
   int i = 0;
}
/*
 *
 * Code used to poll the VBUS line.
 * Code derived from Cypress .asm library.
 * C.H Brain G4GUO 24-03-2013
 *
 */
void EZUSB_Susp(void)
{
    // Clear the Wake Source bit(s) in
    // the WAKEUPCS register
    // clear PA2 and WPIN
    WAKEUPCS |= 0xC0;
    // Write any walue to SUSPEND register
    SUSPEND   = 0x00;
    // Place the processor in idle
    PCON     |= 0x01;
    // Do nothing for a while
    SYNCDELAY;
}
void EZUSB_Delay( int len )
{
    int i;
    for( i = 0; i < len; i++ )
    {
        NOP;
    }
}
//
// Cypress AN15813 for an explanation.
//
static void poll_vbus(void)
{
/*
    WAKEUPCS = bmWU | bmDPEN | bmWUEN;
    WAKEUPCS = bmWU | bmDPEN | bmWUEN;

    if(WAKEUPCS & bmWU )
    {
        USBCS |= bmDISCON;
        USBIE = 0;
        EZUSB_Delay(30);

        WAKEUPCS = bmWU | bmWUPOL | bmWUEN;
        EZUSB_Susp();

        WAKEUPCS = bmWU | bmDPEN | bmWUEN;
        USBCS &= ~bmDISCON;

        USBIE = 1;
        EZUSB_Delay(30);      
    }
*/
}
static void InitializeFX2(void)
{
    CPUCS = 0x12;SYNCDELAY;   // 48 MHz, CLKOUT output enabled. 
    IFCONFIG = 0xE3;SYNCDELAY;  // Internal 48MHz IFCLK; IFCLK pin output enabled
		                // slave FIFO in synchronous mode	
    REVCTL = 0x03;SYNCDELAY;    // See TRM...
    USBIE = 0x00;SYNCDELAY; // Disable interupts   
    EPIE  = 0x00;SYNCDELAY;
    
    IOA = 0x00;
    OEA = 0x03;	// set port A bits 0 and 1 for output

    IOC = 0x00;
    SET_RESET_BIT;

    // set port C bits 3 for output nCONFIG, bit 0 set for output (gp flag), bit 7 for RESET
    // bit 4 for DAC/PLL disable
    OEC = 0x99;	

    PINFLAGSAB = 0x98;  // FLAGA = EP2 EF (empty flag); FLAGB = EP4 EF
    SYNCDELAY;
 
    PINFLAGSCD = 0xfe;  // FLAGC = EP6 FF (full flag); FLAGD = EP8 FF
    SYNCDELAY;

    EP1INCFG =0xa0;     // EP1 IN enbled sends I2C responses from I2C to host
    SYNCDELAY;
    EP1OUTCFG=0xa0;	// EP1 OUT receives FPGA .rbf bitfile and I2C from the host
    SYNCDELAY;
	
    EP4CFG=0x60; SYNCDELAY;  // 0110 0000 (bulk IN, 512 bytes, double-buffered) (not used)
    EP6CFG=0x62; SYNCDELAY;  // 0110 0010 (bulk IN, 512 bytes, double-buffered)  (not used)
    EP8CFG=0x62; SYNCDELAY;  // 0110 0010 (bulk IN, 512 bytes, double-buffered)  (not used)
 
    EP2CFG=0xa8; SYNCDELAY; // 1010 1000 (bulk OUT, 1024 bytes, quad-buffered)

    FIFORESET = 0x80;  SYNCDELAY;  // NAK all requests from host. 

    FIFORESET = 0x81;  SYNCDELAY;  // Reset individual EP (2,4,6,8)
    FIFORESET = 0x82;  SYNCDELAY;  // NAK all requests from host. 
    FIFORESET = 0x84;  SYNCDELAY;
    FIFORESET = 0x88;  SYNCDELAY;
    FIFORESET = 0x00;  SYNCDELAY;  // Resume normal operation. 

    // Be sure to clear the 2 buffers (quad-buffered) (required!). 
    // Has to be done with FIFO config register 0 for some reason
    EP2FIFOCFG = 0x00;  SYNCDELAY;
    OUTPKTEND  = 0x82;  SYNCDELAY;
    OUTPKTEND  = 0x82;  SYNCDELAY;
    OUTPKTEND  = 0x82;  SYNCDELAY;
    OUTPKTEND  = 0x82;  SYNCDELAY;

    // Be sure to clear the 2 buffers (double-buffered) (required!). 
    // Has to be done with FIFO config register 0 for some reason
    //EP4FIFOCFG = 0x00;  SYNCDELAY;
    //OUTPKTEND  = 0x84;  SYNCDELAY;
    //OUTPKTEND  = 0x84;  SYNCDELAY;

#ifdef USB8
    // 8 bit operation
    EP2FIFOCFG = 0x10; SYNCDELAY; //  AUTOOUT=1; byte-wide operation
#endif

#ifdef USB16
    // 16 bit operation
    EP2FIFOCFG = 0x11; SYNCDELAY; //  AUTOOUT=1; word-wide operation
#endif

    EP4FIFOCFG = 0x00; SYNCDELAY; //  AUTOIN=1; byte-wide operation
    EP6FIFOCFG = 0x0c; SYNCDELAY; //  AUTOIN=1; byte-wide operation
    EP8FIFOCFG = 0x0c; SYNCDELAY; //  AUTOIN=1; byte-wide operation
    EP1OUTBC   = 0xff; SYNCDELAY;// arm endpoint 1 for OUT (host->device) transfers
}
//
// Send len bytes to device a
//
static int i2_send( BYTE a, BYTE *b, int len )
{
    BYTE c;
    int  e;
    int  i;
    // Set errors to 0
    e = 0;
    // Wait for stop
    while(I2CS & bmSTOP);
    // Send address
    I2CS  = bmSTART;
    I2DAT = a;// Write Device
    // Wait for Done
    while(!((c=I2CS)&bmDONE));
    //Check ACK
    if((c&bmBERR)||(!(c&bmACK))) e++;
    for( i = 0; i < len; i++ )
    {
        // Next byte
        I2DAT = b[i];
        // Wait for Done
        while(!((c=I2CS)&bmDONE));
        if((c&bmBERR)||(!(c&bmACK))) e++;
    }
    I2CS = bmSTOP;
    return e;
} 
//
// Receive len bytes from device a
//
static int i2c_receive( BYTE i2c_a, BYTE *b, int len )
{
    BYTE c;
    int i;
    // Wait for stop
    while(I2CS & bmSTOP);
    //
    // Set read address field of device
    //
    I2CS  = bmSTART;
    I2DAT = i2c_a|0x01;// LSB = 1
    // Wait for ACK
    while(!((c=I2CS)&bmDONE));//Wait done
    if((c&bmBERR)||(!(c&bmACK))) goto leave;
    if( len == 1 )
    {
        // We are only receiving 1 byte so signal last
        I2CS |= bmLASTRD;
        // Read and throw away contents
        b[0] = I2DAT;
        while(!((c=I2CS)&bmDONE));//Wait done
        // Send a stop
        I2CS |= bmSTOP;
        // Read the data
        b[0] = I2DAT;
    }
    else
    {
        // We are receiving 2 bytes or more 
        // Read and throw away contents
        b[0] = I2DAT;// Throw away
        while(!((c=I2CS)&bmDONE));//Wait done
        for( i = 0; i < len - 2; i++ )
        {
            b[i] = I2DAT;//Save
            while(!((c=I2CS)&bmDONE));//Wait done
        }
        I2CS |= bmLASTRD;//2nd to last
        b[i++] = I2DAT;//Save
        while(!((c=I2CS)&bmDONE));//Wait done
        // Send a stop
        I2CS |= bmSTOP;
        b[i] = I2DAT;//Save
    }   
    return 0;
leave:
    I2CS |= bmSTOP;
    return -1;
}
//
// Receive from a slave I2C device
// Assumes first byte is the address.
// Length is the address, register to read
// and the value to be read
//
static int i2c_receive_flash( BYTE i2c_a, int mem_a, BYTE *b, BYTE len )
{
    BYTE c;
    // Wait for stop
    while(I2CS & bmSTOP);

    // Send address field
    I2CS  = bmSTART;
    I2DAT = i2c_a&0xFE;// LSB = 0

    // Wait for ACK
    while(!((c=I2CS)&bmDONE));//Wait done
    //check ACK
    if((c&bmBERR)||(!(c&bmACK))) goto leave;
    // MSB of Address
    I2DAT = mem_a>>8;
    // Wait for Done
    while(!((c=I2CS)&1));
    // LSB of Address
    I2DAT = mem_a&0xFF;
    // Wait for Done
    while(!((c=I2CS)&bmDONE));
    //
    // The address is set up so we now need 
    // to read its contents
    //
    i2c_receive( i2c_a, b, len );
    return 0;
leave:
    I2CS |= bmSTOP;
    return -1;
}

//
// If a breakout command is received 
// We must go back to loading the FPGA 
//
static int I2cEP1Data(void)
{
    xdata const unsigned char *src = EP1OUTBUF;
    xdata unsigned char *dest = EP1INBUF;
    unsigned int len = EP1OUTBC;
    unsigned int i;
    unsigned int add;
    int resends;
    BYTE rl;
 
    BYTE buffer[40];

    if( len > 40 )
    {
        // Fault
        EP1OUTBC=0xff; // re-arm endpoint 1
        return 1;
    }

    for(i=0; i<len; i++,src++)
    {
	buffer[i] = *src;
    }

    //
    // Send or receive it depending on the LSBit of the Address field.
    //
    if(buffer[0]&0x01)
    {
	// Receive 
        // ADD - LEN - PAYLOAD
        // Calculate the response message length
        rl = buffer[1] + len;
        if( rl > 40 )
        {
            // Fault
            EP1OUTBC=0xff; // re-arm EP1
            return 1;
        }
        // Receive operation
        add = buffer[2];
        add <<= 8;
        add |= buffer[3];
        i2c_receive_flash( buffer[0], add, &buffer[4], buffer[1] );
        // Send message back to host via EP1
        // We assume the endpoint is ready and don't 
        // test the status.
        for(i=0; i<rl; i++,dest++)
        {
	   *dest = buffer[i];// Load data
        }
        EP1INBC = rl;// Trigger send
    }
    else
    {
        // transmit operation
        //re-transmit up to 3 times with delay        
        resends = 3;
	rl = buffer[1]+2;//2 is the address field
        while( resends > 0 )
        {
            resends--;
            if( i2_send( buffer[0], &buffer[2], rl ) == 0 )
                resends = 0;
            else
                EZUSB_Delay(4000);
        }
    }
    EP1OUTBC=0xff; // re-arm EP1
    return 0;
}
//
// After program has loaded, this is called forever.
//
void i2c_handler(void)
{
    while(1)
    {
	// EP1 input
	if (!(EP1OUTCS & bmEPBUSY)) 
        {
           SET_LED_BIT;	// Signal usb LED on
           // Check for breakout
           I2cEP1Data();
        }
        CLR_LED_BIT;	// Signal usb LED off
        poll_vbus();
    }
}

void main(void)
{
    // Configure everything
    InitializeFX2();
    // Wait for I2C messages
    while(1)
    {
    	// Spin waiting for I2C data 
    	i2c_handler();
    }
}
