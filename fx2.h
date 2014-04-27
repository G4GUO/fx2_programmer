#ifndef FX2_H
#define FX2_H

#define USB_VENDOR     0x4B4
#define USB_PROD       0x8613
// Both are out for in add 0x80
#define EP1OUT         0x01
#define EP1IN          0x81
#define EP2OUT         0x02

#define USB_TIMEOUT    1000
//
// Addresses of various I2C devices on the DATVExpress board
// Everything has an address including the FPGA
// If the LSB is zero it is a write operation if it is a one
// it is a read operation.
//
#define FLASH_ADD0    0xA0
#define FLASH_ADD1    0xA2
#define FLASH_ADD2    0xAE
#define I2C_WR        0x00
#define I2C_RD        0x01

#define EXP_OK    1
#define EXP_FAIL -1
#define EXP_MISS -2
#define EXP_IHX  -3
#define EXP_RBF  -4
#define EXP_CONF -5

typedef unsigned char uchar;

//
void fx2_deinit(void);
int  fx2_init( unsigned int cvid, unsigned int cpid, unsigned int nvid, unsigned int npid );
// Start the FX2 code running
void fx2_run(void);
int  fx2_i2c_bulk_transfer(int ep, unsigned char *b, int l );
const char *fx2_result_message(void);

#endif // FX2USB_H
