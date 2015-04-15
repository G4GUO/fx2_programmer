#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <sys/types.h>
#include <semaphore.h>
#include <memory.h>
#include "fx2.h"

static struct usb_device    *m_usbdev;
static libusb_device_handle *m_handle;
static libusb_context       *m_usbctx;
static int m_status;
static char m_error_text[256];

#define EXPRESS_SAMPLES 1
#define EXPRESS_I2C     2

//
// Transfer buffer handling
//

void fx2_wait( void )
{
    struct timespec tim;

    tim.tv_sec  = 0;
    tim.tv_nsec = 100000000; // 100 ms
    nanosleep( &tim, NULL);
}
void fx2_wait_long( void )
{
    struct timespec tim;

    tim.tv_sec  = 1; // 1 sec
    tim.tv_nsec = 0;
    nanosleep( &tim, NULL);
}
//
// Used to send I2C messages
//
int fx2_i2c_bulk_transfer(int ep, unsigned char *b, int l )
{
    int alen=0;
    if(libusb_bulk_transfer( m_handle, ep, b, l, &alen, USB_TIMEOUT )<0)
    {
        sprintf(m_error_text,"I2C transfer failed");
        return -1;
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////
//
// Only called start start up
//
///////////////////////////////////////////////////////////////////////////////////////

int fx2_find_and_open( int vid, int pid )
{
    libusb_device **list;
    libusb_device_handle *handle;

    int res = -1;

    int n = libusb_get_device_list 	( m_usbctx, &list );
    if(n < 1 ) return res;

    for( int i = 0; i < n;  i++ )
    {
        libusb_device_descriptor desc;
        libusb_device *device = list[i];
        if(libusb_get_device_descriptor( device, &desc )==0)
        {
            if((vid == desc.idVendor)&&(pid == desc.idProduct))
            {
                if(libusb_open( device, &handle ) == 0)
                {
                    // We have successfully opened a device.
                    m_handle = handle;
                    res      = 0;
                }
            }
        }
        if (res==0) break;
    }
    libusb_free_device_list ( list, 1 );
    return res;
}

//
// Initialise the fx2 device
//
int fx2_find( unsigned int vid, unsigned int pid  )
{
    // discover devices
    m_usbdev = NULL;
    int ret;

    if( libusb_init( &m_usbctx ) < 0 )
    {
        sprintf(m_error_text,"Cannot init USB\n");
        return EXP_MISS;
    }
    if(fx2_find_and_open( vid, pid ))
    {
        return EXP_MISS;
    }
    //libusb_set_debug( m_usbctx,3 );
    // Bulk transfers on EP2 of samples
    if(libusb_kernel_driver_active(m_handle,0)) libusb_detach_kernel_driver(m_handle,0);

    if((ret=libusb_claim_interface(m_handle,0))<0)
    {
        sprintf(m_error_text,"Cannot claim USB interface %d",ret);
        libusb_close(m_handle);
        libusb_exit(m_usbctx);
        return EXP_MISS;
    }
    /*

    libusb_device_descriptor  desc;
    unsigned char data[255];

    if(libusb_get_descriptor(m_handle, LIBUSB_DT_DEVICE, 0, (unsigned char*)&desc, sizeof(libusb_device_descriptor) )>=0)
    {
        printf("D length %d\n",desc.bLength);
        printf("D Product id %.4x\n",desc.idProduct);
        printf("D Vendor id %.4x\n",desc.idVendor);
        printf("D Class %.1x\n",desc.bDeviceClass);
        printf("D Sub Class %.1x\n",desc.bDeviceSubClass);
        printf("D Max Packet size %d\n",desc.bMaxPacketSize0);
    }
*/
    if((ret=libusb_set_interface_alt_setting(m_handle,0,1))<0)
    {
        sprintf(m_error_text,"Express cannot set interface alt %d",ret);
        libusb_release_interface(m_handle,0);
        libusb_attach_kernel_driver(m_handle,0);
        libusb_close(m_handle);
        libusb_exit(m_usbctx);
        return EXP_MISS;
    }
    //unsigned char text[256];
    //libusb_get_string_descriptor_ascii( m_handle, 0,text,256);
    //loggerf("Desc: %s\n",text);
    //libusb_device *dev = libusb_get_device(m_handle);
    //int speed =  libusb_get_device_speed(dev);
    //loggerf("Speed %d\n",speed);
    //    loggerf("Size %d\n",libusb_get_max_iso_packet_size(dev, EP2));
    //libusb_set_debug( m_usbctx,0 );
    return EXP_OK;
}

//
// Write a 1 to reset the FX2 device
//
void fx2_reset(void)
{
    uchar data = 1;
    if( m_status != EXP_OK ) return;
    int res = libusb_control_transfer(m_handle,0x40,0xa0,0xE600,0,&data,1,USB_TIMEOUT);
    if( res < 0) sprintf(m_error_text,"Express HW reset failed");
}
//
// Write a 0 to start the FX2 running
//
void fx2_run(void)
{
    uchar data = 0;
    if( m_status != EXP_OK ) return;
    int res = libusb_control_transfer(m_handle,0x40,0xa0,0xE600,0,&data,1,USB_TIMEOUT);
    if( res < 0) sprintf(m_error_text,"Express HW run failed");
}
//
// Load the FX2 Firmware
//
int fx2_firmware_load(const char *fx2_filename)
{
    FILE *fp;
    if((fp=fopen(fx2_filename,"r"))!=NULL)
    {
        int line = 0;
        const size_t buflen = 1024;
        char buffer[buflen];
        char *b;

        while(fgets(buffer,buflen,fp))
        {
            if(feof(fp)) break;
            line++;
            b = buffer;
            unsigned int nbytes=0,addr=0,type=0;
            if(b[0] == ':')
            {
                b++;
                sscanf(b,"%02x%04x%02x",&nbytes,&addr,&type);
                b += 8;
                unsigned int d;
                unsigned char data[nbytes];
                unsigned char chksum = nbytes+addr+(addr>>8)+type;
                for( unsigned int i = 0; i < nbytes; i++ )
                {
                    sscanf(b,"%02x",&d);
                    b += 2;
                    data[i] = d;
                    chksum += d;
                }
                unsigned int fchksum  = 0;
                sscanf(b,"%02x",&fchksum);
                if((chksum+fchksum)&0xFF)
                {
                    sprintf(m_error_text,"Express Firmware file CRC error");
                }
                else
                {
                   // Write to RAM
                   if(libusb_control_transfer(m_handle,0x40,0xa0,addr,0,data,nbytes,USB_TIMEOUT)<0)
                   {
                       sprintf(m_error_text,"Express HW firmware control transfer load failed");
                       break;
                   }
                }
            }
        }
        fclose(fp);
    }
    else
    {
        sprintf(m_error_text,"Express cannot find firmware file %s",fx2_filename);
        return EXP_IHX;
    }
    return EXP_OK;
}
//
// Read and write to the flash memory
//
// Write
// Add is the address to write to
// b is the bytes to be written
// len id the number of bytes to write (max32)
//
void fx2_write_flash( int add, unsigned char *b, uchar len )
{
    if( m_status != EXP_OK ) return;
    // Format up into an I2C message for the flash
    uchar msg[40];
    msg[0]  = FLASH_ADD1 | I2C_WR;//address
    msg[1]  = len;
    msg[2]  = add>>8;//msb of address
    msg[3]  = add&0xFF;//lsb of address
    // Attach data
    if( len > 32) len = 32;
    for( int i = 0; i < len; i++)
    {
        msg[i+4] = b[i];//data
    }
    // Send
    fx2_i2c_bulk_transfer( EP1OUT, msg, len + 4 );
}
// Read
// Add is the address to read from
// b is the bytes to be read
// len id the number of bytes to write (max32 self imposed)
//
void fx2_read_flash( int add, unsigned char *b, uchar len )
{
    if( m_status != EXP_OK ) return;
    uchar msg[50];
    if(len > 32) len = 32;
    msg[0]  = FLASH_ADD1 | I2C_RD;//address
    msg[1]  = len;//bytes to receive
    msg[2]  = add>>8;//msb of address
    msg[3]  = add&0xFF;//lsb of address
    // Set read address
    fx2_i2c_bulk_transfer( EP1OUT, msg, 4 );
    // Read
    msg[0]  = FLASH_ADD1 | I2C_RD;//address
    fx2_i2c_bulk_transfer( EP1IN, msg, len+4);
    memcpy( b, &msg[4], len);
}
//
// Save the FX2 Firmware to flash
//
int fx2_firmware_save_to_flash( const char *fx2_filename )
{
    FILE *fp;
    if((fp=fopen(fx2_filename,"r"))!=NULL)
    {
        int line = 0;
        const size_t buflen = 1024;
        char buffer[buflen];
        char *b;

        while(fgets(buffer,buflen,fp))
        {
            if(feof(fp)) break;
            line++;
            b = buffer;
            unsigned int nbytes=0,addr=0,type=0;
            if(b[0] == ':')
            {
                b++;
                sscanf(b,"%02x%04x%02x",&nbytes,&addr,&type);
                b += 8;
                unsigned int d;
                unsigned char data[nbytes];
                unsigned char chksum = nbytes+addr+(addr>>8)+type;
                for( unsigned int i = 0; i < nbytes; i++ )
                {
                    sscanf(b,"%02x",&d);
                    b += 2;
                    data[i] = d;
                    chksum += d;
                }
                unsigned int fchksum  = 0;
                sscanf(b,"%02x",&fchksum);
                if((chksum+fchksum)&0xFF)
                {
                    sprintf(m_error_text,"Express Firmware file CRC error");
                }
                else
                {
                   // Write to RAM
                   fx2_write_flash( addr, data, nbytes );
                   fx2_wait();
                }
            }
        }
        fclose(fp);
    }
    else
    {
        sprintf(m_error_text,"Express cannot find firmware file %s",fx2_filename);
        return EXP_IHX;
    }
    return EXP_OK;
}

//////////////////////////////////////////////////////////////////////////
//
// Called when system is running
//
//////////////////////////////////////////////////////////////////////////

const char *fx2_result_message(void)
{
    return m_error_text;
}

void fx2_deinit(void)
{
    if( m_status != EXP_OK ) return;
    libusb_release_interface(m_handle,0);
    libusb_attach_kernel_driver(m_handle,0);
    libusb_close(m_handle);
    libusb_exit(m_usbctx);
    m_status = EXP_CONF;
}

int fx2_init( const char *flash_file, unsigned int cvid, unsigned int cpid, unsigned int nvid, unsigned int npid )
{
    m_status = EXP_CONF;
    sprintf(m_error_text,"Success!");

    // Find the board
    if(( m_status = fx2_find( cvid, cpid ))<0)
    {
        sprintf(m_error_text,"FX2 Hardware not found");
        return EXP_FAIL;
    }
    // Reset it, this gets it into a state ready for programming
    fx2_reset();
    fx2_wait();

    // Load the FX2 firmware
    if(( m_status = fx2_firmware_load( "flash_loader.ihx"))<0)
    {
        sprintf(m_error_text,"FX2 firmware file not found");
        return EXP_IHX;
    }
    // Start the FX2 code running
    // to accept a new program
    fx2_run();
    fx2_wait();

    m_status = EXP_OK;

   unsigned char b[32];
   // Change the VID and PID by writing to flash memory

   b[0] = 0xC0;
   b[1] = nvid&0xFF;
   b[2] = nvid>>8;
   b[3] = npid&0xFF;
   b[4] = npid>>8;
   b[5] = 0x00;
   b[6] = 0x01;
   b[7] = 0x00;

   fx2_write_flash( 0x00, b, 8 );
   fx2_wait_long();

   memset(b,0,12);

   if( strlen(flash_file) != 0 ) fx2_firmware_save_to_flash( flash_file );

   fx2_read_flash( 0x00, b, 8 );
   sprintf(m_error_text,"%.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x",b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7]);


   fx2_deinit();

    return m_status;
}
