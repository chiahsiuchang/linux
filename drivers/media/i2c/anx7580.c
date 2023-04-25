// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright(c) 2023, Valve Software. All rights reserved.
 *
 */
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include "anx7580_regs.h"

#define LOG_TAG "ANX"

typedef struct {
	struct i2c_client *client;
	u8 panel_read_data[512];
	int panel_read_reg_addr, panel_read_num_bytes;
	unsigned int read_sid, read_offset, read_len;
	uint16_t dp_vtotal;
	struct delayed_work work;
} anx7580_t;

/* i2c functions ported from analogix chicago android driver */

static int anx7580_i2c_write_byte( anx7580_t * anx, u8 slave_id, u16 offset_addr, u8 data )
{
	int ret = 0;

	if ((((slave_id & 0x0F)!=0)&&((offset_addr&0xFF00)!=0))||((offset_addr&0xF000)!=0)) {
		pr_err( "%s %s: I2C slave_id or offset_addr ERROR!! %02x %04x\n",LOG_TAG,__func__,slave_id,offset_addr );
		return -EINVAL;
	}

	anx->client->addr = ( ANX7580_SLAVE_ID_ADDR >> 1 );
	ret = i2c_smbus_write_byte_data( anx->client, 0x00, (slave_id | (u8)((offset_addr&0x0F00)>>8)) );
	if ( ret < 0 ) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr);
	}else{
		anx->client->addr = ( ANX7580_OFFSET_ADDR >> 1 );
		ret = i2c_smbus_write_byte_data( anx->client, (u8)(offset_addr&0x00FF), data );
		if ( ret < 0 ) {
			pr_err( "%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr );
		}
	}
	return ret;
}

static int anx7580_i2c_write_block( anx7580_t * anx, u8 slave_id, u16 offset_addr, u8 length, u8 *p_data )
{
	int ret = 0;

	if ((((slave_id & 0x0F)!=0)&&((offset_addr&0xFF00)!=0))||((offset_addr&0xF000)!=0)) {
		pr_info( "%s %s: I2C slave_id or offset_addr ERROR!! %02x %04x\n",LOG_TAG,__func__,slave_id,offset_addr );
		return -EINVAL;
	}

	anx->client->addr = ( ANX7580_SLAVE_ID_ADDR >> 1 );
	ret = i2c_smbus_write_byte_data( anx->client, 0x00, (slave_id | (u8)((offset_addr&0x0F00)>>8)) );
	if (ret < 0) {
		pr_err( "%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr );
	}else{
		anx->client->addr = ( ANX7580_OFFSET_ADDR >> 1 );
		ret = i2c_smbus_write_i2c_block_data( anx->client, (u8)(offset_addr&0x00FF), length, p_data );
		if (ret < 0) {
			pr_err( "%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr );
		}
	}
	return ret;
}

static int anx7580_i2c_write_byte4( anx7580_t * anx, u8 slave_id, u16 offset_addr, u32 data )
{
	int ret = 0;
	u8	buf[4],i;
	for ( i=0; i<4; i++ ) {
		buf[i] = (u8)(data&0x000000FF);
		data = data >> 8;
	}

	ret = anx7580_i2c_write_block( anx, slave_id, offset_addr, 4, &buf[0] );

	return ret;
}

static int anx7580_i2c_read_byte( anx7580_t * anx, u8 slave_id, u16 offset_addr, u8 *p_data )
{
	int ret = 0;

	if ((((slave_id & 0x0F)!=0)&&((offset_addr&0xFF00)!=0))||((offset_addr&0xF000)!=0)) {
		pr_info( "%s %s: I2C slave_id or offset_addr ERROR!! %02x %04x\n",LOG_TAG,__func__,slave_id,offset_addr );
		return -EINVAL;
	}

	anx->client->addr = ( ANX7580_SLAVE_ID_ADDR >> 1 );
	ret = i2c_smbus_write_byte_data( anx->client, 0x00, (slave_id | (u8)((offset_addr&0x0F00)>>8)) );
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr);
	}else{
		anx->client->addr = ( ANX7580_OFFSET_ADDR >> 1 );
		ret = i2c_smbus_read_byte_data( anx->client, (u8)(offset_addr&0x00FF) );
		if (ret < 0) {
			pr_err(" %s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr );
			return ret;
		}
		*p_data = (u8)ret;
	}
	return ret;
}

static int anx7580_i2c_read_block( anx7580_t * anx, u8 slave_id, u16 offset_addr, u8 length, u8 *p_data )
{
	int ret = 0;

	if ((((slave_id & 0x0F)!=0)&&((offset_addr&0xFF00)!=0))||((offset_addr&0xF000)!=0)) {
		pr_err( "%s %s: I2C slave_id or offset_addr ERROR!! %02x %04x\n",LOG_TAG,__func__,slave_id,offset_addr );
		return -EINVAL;
	}

	anx->client->addr = (ANX7580_SLAVE_ID_ADDR >> 1);
	ret = i2c_smbus_write_byte_data( anx->client, 0x00, (slave_id | (u8)((offset_addr&0x0F00)>>8)) );
	if (ret < 0) {
		pr_err( "%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr );
	} else {
		anx->client->addr = ( ANX7580_OFFSET_ADDR >> 1 );
		ret = i2c_smbus_read_i2c_block_data( anx->client, (u8)(offset_addr&0x00FF), length, p_data );
		if (ret < 0) {
			pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr);
		}
	}
	return ret;
}

static int anx7580_i2c_read_byte4( anx7580_t * anx, u8 slave_id, u16 offset_addr, u32 *p_data )
{
	int ret = 0;
	u8	buf[4],i;
	
	ret = anx7580_i2c_read_block( anx, slave_id, offset_addr, 4, &buf[0] );

	if ( ret>=0 ) {
		*p_data = 0;
		for ( i=0; i<3; i++ ){
			*p_data += (u32)(buf[3-i]);
			*p_data = *p_data<<8;
		}
		*p_data += (u32)(buf[3-i]);
	}
	
	return ret;
}

static void anx7580_panel_dcs_cmd( anx7580_t * anx, uint8_t cmd )
{
	uint32_t reg_val = DCS_WRITE_NO_PARAM | cmd << 8;
	anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_HDR, reg_val );
}

static void anx7580_panel_write_immediate( anx7580_t * anx, uint8_t reg, uint8_t val )
{
	uint32_t reg_val = DCS_WRITE_ONE_PARAM | reg << 8 | val << 16;
	anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_HDR, reg_val );
}

static void anx7580_panel_write_long( anx7580_t * anx, void * _buf, uint8_t len )
{
    const uint8_t *buf = _buf;
    for ( uint32_t v = 0, l = len; l; ) {
      int i;

      // swap byte order in string to meet anx mipi port interface requirements
      for ( i = 0; i < 4 && l; i++, l-- ) {
        v = ( v >> 8 ) | ( *( buf++ ) << 24 );
      }
      anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_PLD_DATA, v >> ( 8 * ( 4 - i ) ) );
    }
    anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_HDR, ( ( len ) << 8 ) | DCS_WRITE_LONG );
}

//-----------------------------------------------------------------------------
// Reset mipi output block
// Must do this after panel read to clear buffer offset
static int anx7580_mipi_reset( anx7580_t * anx )
{
  anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, PWR_UP, 0 );
  return anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, PWR_UP, 1 );
};

//Minimum time between DCS transfers.
//Right now, we are going to indiscriminately wait, since the methodology for
//timing out and verifiying transaction seems to be fraught with peril.
//
//This seems to be pretty reliable in my tests.
#define DCS_INTERWRITE_DELAY_MS 34   

static uint32_t anx7580_dcs_read( anx7580_t * anx, uint8_t reg, uint8_t len, uint8_t *pay )
{
	uint32_t data, bytes_copied = 0, returned_bytes = 0;
	uint16_t dcs_timeout_msec = 0;
	uint32_t fifo_status;

	// request len number of bytes from reg
	anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_HDR, ( len << 8 ) | DCS_SET_RETURN_SIZE );
	anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_HDR, ( uint32_t )DCS_READ | ( reg << 8 ) );

	do
	{
		anx7580_i2c_read_byte4( anx, SLAVEID_MIPI_PORT0, CMD_PKT_STATUS, &fifo_status );
		msleep(1);
		dcs_timeout_msec++;
	}
	while (( fifo_status & 0x10 ) && ( dcs_timeout_msec < DCS_INTERWRITE_DELAY_MS ) );

	if ( dcs_timeout_msec >= DCS_INTERWRITE_DELAY_MS ) {
		pr_err( "anx7580: MIPI Read timeout\n" );
		return 0;
	}

	// get data from anx7580 fifo
	do
	{
		anx7580_i2c_read_byte4(anx, SLAVEID_MIPI_PORT0, GEN_PLD_DATA, &data );
		for ( uint8_t i = 0; i < 4 && ( i + returned_bytes ) < len; i++ )
		{
			pay[bytes_copied++] = ( data >> ( i * 8 ) ) & 0xFF;
		}
		returned_bytes += 4;
		anx7580_i2c_read_byte4( anx, SLAVEID_MIPI_PORT0, CMD_PKT_STATUS, &fifo_status );
	} while ( !( fifo_status & 0x10 ) );

	// must reset mipi buffers after read to correct offset
	anx7580_mipi_reset(anx);

	return bytes_copied;
}

static void anx7580_read_chip_id(anx7580_t * anx)
{
	u8 reg_temp;
	u16 reg_int;
	
	anx7580_i2c_read_byte( anx,SLAVEID_SPI, CHIP_ID_HIGH, &reg_temp );
	reg_int = ((((u16)reg_temp)<<8)&0xFF00);
	anx7580_i2c_read_byte( anx,SLAVEID_SPI, CHIP_ID_LOW, &reg_temp );
	reg_int |= (((u16)reg_temp)&0x00FF);
	pr_info( "anx7580: Chip ID = %04X\n", reg_int );
}

static u8 anx7580_check_ocm_status(anx7580_t * anx)
{
	u8 reg_temp;
	// Check OCM status
	anx7580_i2c_read_byte( anx, SLAVEID_SERDES,SERDES_POWER_CONTROL, &reg_temp );
	if ( OCM_LOAD_DONE == ( OCM_LOAD_DONE & reg_temp ) ) {
		return 1;
    }
    return 0;
}

static void anx7580_get_dp_vtotal(anx7580_t * anx, uint16_t * dp_vtotal)
{
	uint8_t high, low;
	anx7580_i2c_read_byte( anx, SLAVEID_MAIN_LINK, ADDR_VTOTAL_DBG, &high );
	anx7580_i2c_read_byte( anx, SLAVEID_MAIN_LINK, ADDR_VTOTAL_DBG+4, &low );
	*dp_vtotal = high << 8 | low;
}

static void anx7580_clear_interrupts( anx7580_t * anx )
{
	// clear the audio int mask so that we can service and clear interrupts
	// due to the audio pll resetting.  this interrupt will mask future interrupts if not handled
	anx7580_i2c_write_byte( anx, SLAVEID_DP_TOP, ADDR_INTR_MASK, ~AUDIO_INTR );
	// from anx7530 android driver: clear OCM based interrupt interface?
	// clear 01:90 and 01:91 by i2c write block funciton
	anx7580_i2c_write_byte( anx, SLAVEID_SPI, INT_NOTIFY_MCU0, 0 );
	anx7580_i2c_write_byte( anx, SLAVEID_SPI, INT_NOTIFY_MCU1, 0 );
	// Clear SW and PLL interrupts, and unmask PLL interrupt
	// otherwise PLL interrupt blocks future SW interrupts
	anx7580_i2c_write_byte( anx, SLAVEID_DP_TOP, ADDR_SW_INTR_CTRL, 0 );
}

static irqreturn_t anx7580_irq_handler_top(int irq, void *dev_id)
{
	// handle irq in threaded handler for i2c functions
    return IRQ_WAKE_THREAD;
}

// anx7580 interrupt handler
// main purpose is to handle refresh rate changes
static irqreturn_t anx7580_irq_handler( int irq, void *dev_id )
{
	uint16_t dp_vtotal_new;
	uint8_t notify_reg[2];
	int32_t vfp;
	struct i2c_client *client = dev_id;
    anx7580_t *anx = dev_get_drvdata( &client->dev );
	anx7580_i2c_read_block( anx, SLAVEID_SPI, INT_NOTIFY_MCU0, 2, notify_reg );

	//pr_info( "anx7580: got int %x %x\n", notify_reg[0], notify_reg[1]);

	// video stream change, check msa for a change in video mode
	if ( notify_reg[0] & VIDEO_INPUT_EMPTY  ) { 
		// Do nothing here, we get this interrupt before the new mode is stable
	} else if ( notify_reg[0] & VIDEO_STABLE ) {
		anx7580_get_dp_vtotal( anx, &dp_vtotal_new );
	    pr_info( "anx7580: vtotal old %d vtotal new %d\n", anx->dp_vtotal, dp_vtotal_new );
		if( anx->dp_vtotal != dp_vtotal_new && dp_vtotal_new > 0 )
		{
			#define PANEL_VACT 1280
			#define PANEL_VBP  22 /* for EV1 Panels TODO make this configurable */
			vfp = dp_vtotal_new - PANEL_VACT - PANEL_VBP;
			if (vfp)
			{
				pr_info( "anx7580: mode change detected, setting new VFP %d\n", vfp );
				anx->dp_vtotal = dp_vtotal_new;
				
				// video mode vfp
				anx7580_i2c_write_byte( anx, SLAVEID_PPS, REG_ADDR_V_F_PORCH_CFG_L, vfp & 0xff ); // [51]  Vfp <7:0>
				anx7580_i2c_write_byte( anx, SLAVEID_PPS, REG_ADDR_V_F_PORCH_CFG_H, vfp >> 8 ); // [51]  Vfp <13:8>
				// mipi vfp
				anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, VID_VFP_LINES, vfp ); // [186] Vfp (lines)       
				anx7580_mipi_reset( anx );
			}
		}
	}
	anx7580_clear_interrupts( anx );
	return IRQ_HANDLED;
}

// on initial setup we sometimes miss the interrupt from the analogix
// so we run a delayed work handler to clean up the interrupt status
static void anx7580_delayed_work_handler(struct work_struct *work)
{
	// check to see if backlight is still in the correct mode
	uint8_t int_reg;
	uint8_t notify_reg[2];

	anx7580_t *anx = container_of((struct delayed_work *)work, anx7580_t, work);
	
	anx7580_i2c_read_byte( anx, SLAVEID_DP_TOP, ADDR_SW_INTR_CTRL, &int_reg );
	anx7580_i2c_read_block( anx, SLAVEID_SPI, INT_NOTIFY_MCU0, 2, notify_reg );
	// if interrupt was set and we missed it, clear it and schedule another check back
	// otherwise this would be handled in the interrupt
	if ( int_reg || notify_reg[0] || notify_reg[1] ) {
		anx7580_clear_interrupts( anx );
		//pr_info("anx7580: schedule another check on in status %x\n", int_reg);
		schedule_delayed_work(&anx->work, HZ/4);
	}
}

static void anx7580_work_init(anx7580_t *anx)
{
	INIT_DELAYED_WORK(&anx->work, anx7580_delayed_work_handler);
	schedule_delayed_work(&anx->work, HZ/4);
}

// sysfs interface 
static ssize_t bmode_store(struct device *device,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int result;
	unsigned int mode;
	anx7580_t *anx = dev_get_drvdata(device);
	if ( !anx ) {
		printk( "anx7580: failed to get anx driver data\n" );
		return 0;
	}

    result = sscanf( buf, "%d", &mode );
	if( result == 1 ) {
		if ( mode == 1 ){
			pr_info( "anx7580: setting high brightness mode\n" );
			anx7580_panel_write_immediate( anx, 0x53, 0xE0 ); // high brightness mode
		} else if (mode == 2){
			pr_info( "anx7580: setting normal brightness mode in 32 frames\n" );
			anx7580_panel_write_immediate( anx, 0x53, 0x28 ); // normal mode, update brightness within 32 frame
		} else if (mode == 3){
			pr_info( "anx7580: setting normal brightness mode in 1 frame\n" );
			anx7580_panel_write_immediate( anx, 0x53, 0x20 ); // normal mode, update brightness within 32 frame
		} else {
			pr_info( "anx7580: brightness mode %d\n", mode );
		}
	}
	return count;
}

static ssize_t bmode_show(struct device *device,
			   struct device_attribute *attr, char *buf)
{
	uint8_t brightness_mode_reg, count, bytes_copied = 0;
	anx7580_t *anx = dev_get_drvdata( device );
	// TODO return current brightness mode
	bytes_copied = anx7580_dcs_read( anx, 0x53, 1, &brightness_mode_reg );
	printk("anx7580 read %d bytes\n", bytes_copied);
	if ( brightness_mode_reg == 0xE0 ) {
		count = sprintf( buf, "High Brightness\n");
	} else if ( brightness_mode_reg == 0x28 ) {
		count = sprintf( buf, "Normal Brightness 32 frames\n" );
	} else if ( brightness_mode_reg == 0x20 ) {
		count = sprintf( buf, "Normal Brightness 1 frames\n" );
	} else {
		count = sprintf( buf, "Unknown Brightness mode %x\n", brightness_mode_reg );
	}
	return count;
}

static ssize_t brightness_store(struct device *device,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int result;
	unsigned int brightness;
	uint8_t cmd[3];
	anx7580_t *anx = dev_get_drvdata( device );

	result = sscanf( buf, "%d", &brightness );

    if ( result == 1 ) {
		// limit brightness to 16 bits
		brightness &= 0xFFFF;
		cmd[0] = 0x51;
		cmd[1] = ( brightness & 0xFF00 ) >> 8;
		cmd[2] = brightness & 0xFF;

		pr_info("anx7580: setting %d brightness\n", brightness);
		anx7580_panel_write_long( anx, cmd, 3 );
	}
	return count;
}

static ssize_t brightness_show( struct device *device, struct device_attribute *attr, char *buf )
{
	uint8_t brightness_reg[2];
	uint8_t count, bytes_copied = 0;
	anx7580_t *anx = dev_get_drvdata( device );
	// TODO return current brightness mode
	bytes_copied = anx7580_dcs_read( anx, 0x51, 2, brightness_reg );
	printk( "anx7580 read %d bytes\n", bytes_copied );
	if (bytes_copied) {
		count = sprintf( buf, "Brightness %d\n", (int)(brightness_reg[0] << 8 | brightness_reg[1]) );
	} else {
		count = sprintf( buf, "Brightness Unknwn\n" );
	}
	return count;
}

static ssize_t panel_read_show( struct device *device, struct device_attribute *attr, char *buf )
{
	int ret;
	ssize_t len = 0;
	unsigned int i;
	anx7580_t *anx = dev_get_drvdata( device );
	ret = anx7580_dcs_read( anx, anx->panel_read_reg_addr, anx->panel_read_num_bytes, anx->panel_read_data );
	if ( ret < 0 ) {
		pr_err( "anx7580: failed to read panel reg %u\n", anx->panel_read_reg_addr );
		return ret;
	} else {
		for (i = 0; i < anx->panel_read_num_bytes; i++) {
				ret = scnprintf( buf + len, PAGE_SIZE - len, "%x ",
								anx->panel_read_data[i] );
				if ( ret < 0 )
						return ret;
				len += ret;
		}
		buf[len - 1] = '\n';
		return len;
	}
}

static ssize_t panel_read_store( struct device *device, struct device_attribute *attr,const char *buf, size_t count )
{
	int ret;
	anx7580_t *anx = dev_get_drvdata( device );
	ret = sscanf( buf, "%2x %d", &anx->panel_read_reg_addr, &anx->panel_read_num_bytes );
	if(ret != 2) {
		pr_err( "anx7580: panel read got wrong number of input parameters from sscanf" );
		return -EINVAL;
	}
	return count;
}

static ssize_t panel_write_store( struct device *device, struct device_attribute *attr,const char *buf, size_t count )
{
	int ret;
	int addr, len, i, v;
	char char_data[256];
	u8 payload[256];
	anx7580_t *anx = dev_get_drvdata(device);
	// get the base parameters
	ret = sscanf( buf, "%2x %d %s", &addr, &len, char_data );
	if(ret != 3) {
		pr_err( "anx7580: panel write got wrong number of input parameters (%d) from sscanf", ret );
		return -EINVAL;
	}
	// TODO: for now we limit writes to a total of 256 bytes
	if(len >256) {
		pr_err( "anx7580: panel write exceed max length %d\n", len );
		return -EINVAL;
	}
	payload[0] = addr;
	// get the data payload
	for ( i=0; i<len; i++ ) {
	    if ( sscanf( char_data + i * 2, "%2x", &v ) != 1 ) break;
        payload[i+1] = (unsigned char)v;
	}

    if ( len == 0 )
	    anx7580_panel_dcs_cmd( anx, payload[0] );
	if ( len == 1 )
		anx7580_panel_write_immediate( anx, payload[0], payload[1] );
	else
		anx7580_panel_write_long( anx, payload, len );

	pr_info( "anx7580: wrote %s to reg %x\n", char_data, addr );

	return count;
}

static ssize_t panel_write_show( struct device *device, struct device_attribute *attr, char *buf )
{
	return 0;
}

static ssize_t anx7580_read_reg_store( struct device *device, struct device_attribute *attr,const char *buf, size_t count )
{
    int ret;
	anx7580_t *anx = dev_get_drvdata(device);
	ret = sscanf( buf, "%x %x %u", &anx->read_sid, &anx->read_offset, &anx->read_len);
	if ( ret != 3 ) {
		pr_err( "anx7580: invalid reg read params\n" );
		return -EINVAL;
	}
	return count;
}

static ssize_t anx7580_read_reg_show( struct device *device, struct device_attribute *attr, char *buf )
{
	ssize_t len = 0;
	unsigned int i, ret;
	uint8_t data[256];
	anx7580_t *anx = dev_get_drvdata(device);
	anx7580_i2c_read_block(anx, anx->read_sid, anx->read_offset, anx->read_len, data);
	for (i = 0; i < anx->read_len; i++) {
		ret = scnprintf( buf + len, PAGE_SIZE - len, "%x ", data[i] );
		if ( ret < 0 )
				return ret;
		len += ret;
	}
	buf[len - 1] = '\n';
	return len;
}

static ssize_t anx7580_write_reg_store( struct device *device, struct device_attribute *attr,const char *buf, size_t count )
{
    int ret;
	unsigned int write_byte_sid, write_byte_offset, write_byte_data;
	anx7580_t *anx = dev_get_drvdata(device);
	ret = sscanf( buf, "%x %x %x", &write_byte_sid, &write_byte_offset, &write_byte_data);
	if ( ret != 3 ) {
		pr_err( "anx7580: invalid write byte params\n" );
		return -EINVAL;
	}
	anx7580_i2c_write_byte(anx, write_byte_sid, write_byte_offset, write_byte_data);
	return count;
}

static ssize_t anx7580_write_reg_show( struct device *device, struct device_attribute *attr, char *buf )
{
	return 0;
}

static ssize_t anx7580_write_reg4_store( struct device *device, struct device_attribute *attr,const char *buf, size_t count )
{
    int ret, i, v;
	unsigned int sid, offset;
	char char_data[4*2];
	unsigned int reg_value = 0;
	anx7580_t *anx = dev_get_drvdata(device);

	// get the base parameters
	ret = sscanf( buf, "%x %x %s", &sid, &offset, char_data );
	if(ret != 3) {
		pr_err( "anx7580: panel write got wrong number of input parameters (%d) from sscanf", ret );
		return -EINVAL;
	}

	// get the data payload
	for ( i=0; i<4; i++ ) {
	    if ( sscanf( char_data + i * 2, "%2x", &v ) != 1 ) break;
        reg_value = ( reg_value << (8) ) | (unsigned char)v;
	}
	// pr_info("anx7580: write4 %x %x = %x\n", sid, offset, reg_value);
	anx7580_i2c_write_byte4(anx, sid, offset, reg_value);
	return count;
}

static ssize_t anx7580_write_reg4_show( struct device *device, struct device_attribute *attr, char *buf )
{
	return 0;
}

static struct device_attribute anx7580_attrs[] = {
	__ATTR(bmode, 0664, bmode_show, bmode_store),
	__ATTR(brightness, 0664, brightness_show, brightness_store),
	__ATTR(panel_read, 0664, panel_read_show, panel_read_store),
	__ATTR(panel_write, 0664, panel_write_show, panel_write_store),
	__ATTR(anx7580_read_reg, 0664, anx7580_read_reg_show, anx7580_read_reg_store),
	__ATTR(anx7580_write_reg, 0664, anx7580_write_reg_show, anx7580_write_reg_store),
	__ATTR(anx7580_write_reg4, 0664, anx7580_write_reg4_show, anx7580_write_reg4_store),
};

static int anx7580_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	anx7580_t *anx;
	unsigned int i, irq;
	int ret;

	if ( !i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK) ) {
		pr_err("%s: i2c bus does not support the anx7580\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	anx = kzalloc(sizeof(anx7580_t), GFP_KERNEL);
	if ( !anx ) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	dev_set_drvdata( &client->dev, anx );
	anx->client = client;

	// anx7580 is powered on by bios, so we assume ocm wait period is complete
	if ( anx7580_check_ocm_status(anx) ) {
		// OCM load done
		// Read Chip ID
		anx7580_read_chip_id(anx);
		// store current dp vtotal for future mode changes
		anx7580_get_dp_vtotal(anx, &anx->dp_vtotal);
		pr_info("anx7580: initial dp vtotal %d\n", anx->dp_vtotal);
	}

	// setup irq
	irq = acpi_dev_gpio_irq_get( ACPI_COMPANION(&client->dev), 0 );
	
	if ( irq <= 0 ) {
		pr_err( "anx7580: could not get acpi irq %d\n", irq );
	} else {
		pr_info( "anx7580: got irq %d\n", irq );
		ret = devm_request_threaded_irq( &client->dev, irq, anx7580_irq_handler_top, anx7580_irq_handler,
									IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ANX7580_IRQ", client );
		if ( ret )
			pr_err( "anx7580: request interrupt failed with %d\n", ret );
		anx7580_clear_interrupts( anx );
		anx7580_work_init( anx );
	}

	for ( i=0; i < ARRAY_SIZE(anx7580_attrs); i++ )
	{
		if ( device_create_file(&client->dev, &anx7580_attrs[i]) )
			pr_err( "anx7580: could not create sysfs index %d\n", i );
	}

exit:
    return 0;
}

static void anx7580_remove(struct i2c_client *client)
{
	anx7580_t *data;
	unsigned int i;
	data = i2c_get_clientdata(client);
	for (i=0; i < ARRAY_SIZE(anx7580_attrs); i++)
	{
		device_remove_file(&client->dev, &anx7580_attrs[i]);
	}
	kfree(data);
}

static int anx7580_suspend_late(struct device *dev)
{
	pr_info("anx7580: %s called\n", __func__);
	return 0;
}

static void sequence_1st(struct device *dev)
{
	pr_info("anx7580: %s enter\n", __func__);

	//panel powanx7580_write_reg_store4 on sequence
	
	//1.8V anx7580_write_reg_store4ovide to ANX7580
	//wait >= 1ms
	//1.0V anx7580_write_reg_store4ovide to ANX7580
	//wait >= 2ms
	//CHIP_PWR_UP set to high
	//wait >= 20ms
	//RESET set to high
	//wait >= 10ms
	
	//read 01:05, if 01:05 = 0xF0, OCM CRC passed.
	
	//power down the panel and analogix mipi while we attempt reinit
	panel_write_store(dev, NULL, "28 0 0", 0);
	panel_write_store(dev, NULL, "10 0 0", 0);
	
	//set HPD to low 
	anx7580_write_reg4_store(dev, NULL, "30 028 00", 0);
	
	//write EDID without audio block
	anx7580_write_reg_store(dev, NULL, "03 000 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 001 FF", 0);
	anx7580_write_reg_store(dev, NULL, "03 002 FF", 0);
	anx7580_write_reg_store(dev, NULL, "03 003 FF", 0);
	anx7580_write_reg_store(dev, NULL, "03 004 FF", 0);
	anx7580_write_reg_store(dev, NULL, "03 005 FF", 0);
	anx7580_write_reg_store(dev, NULL, "03 006 FF", 0);
	anx7580_write_reg_store(dev, NULL, "03 007 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 008 59", 0);
	anx7580_write_reg_store(dev, NULL, "03 009 96", 0);
	anx7580_write_reg_store(dev, NULL, "03 00A 03", 0);
	anx7580_write_reg_store(dev, NULL, "03 00B 30", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 00C 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 00D 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 00E 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 00F 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 010 ff", 0);
	anx7580_write_reg_store(dev, NULL, "03 011 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 012 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 013 04", 0);
	anx7580_write_reg_store(dev, NULL, "03 014 A5", 0);
	anx7580_write_reg_store(dev, NULL, "03 015 06", 0);
	anx7580_write_reg_store(dev, NULL, "03 016 05", 0);
	anx7580_write_reg_store(dev, NULL, "03 017 78", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 018 17", 0);
	anx7580_write_reg_store(dev, NULL, "03 019 3C", 0);
	anx7580_write_reg_store(dev, NULL, "03 01A 71", 0);
	anx7580_write_reg_store(dev, NULL, "03 01B AE", 0);
	anx7580_write_reg_store(dev, NULL, "03 01C 51", 0);
	anx7580_write_reg_store(dev, NULL, "03 01D 3C", 0);
	anx7580_write_reg_store(dev, NULL, "03 01E B9", 0);
	anx7580_write_reg_store(dev, NULL, "03 01F 23", 0);
	anx7580_write_reg_store(dev, NULL, "03 020 0C", 0);
	anx7580_write_reg_store(dev, NULL, "03 021 50", 0);
	anx7580_write_reg_store(dev, NULL, "03 022 54", 0);
	anx7580_write_reg_store(dev, NULL, "03 023 00", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 024 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 025 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 026 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 027 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 028 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 029 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 02A 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 02B 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 02C 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 02D 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 02E 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 02F 01", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 030 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 031 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 032 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 033 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 034 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 035 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 036 08", 0);
	anx7580_write_reg_store(dev, NULL, "03 037 34", 0);
	anx7580_write_reg_store(dev, NULL, "03 038 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 039 48", 0);
	anx7580_write_reg_store(dev, NULL, "03 03A 31", 0);
	anx7580_write_reg_store(dev, NULL, "03 03B 00", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 03C 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 03D 50", 0);
	anx7580_write_reg_store(dev, NULL, "03 03E 14", 0);
	anx7580_write_reg_store(dev, NULL, "03 03F 08", 0);
	anx7580_write_reg_store(dev, NULL, "03 040 91", 0);
	anx7580_write_reg_store(dev, NULL, "03 041 40", 0);
	anx7580_write_reg_store(dev, NULL, "03 042 64", 0);
	anx7580_write_reg_store(dev, NULL, "03 043 96", 0);
	anx7580_write_reg_store(dev, NULL, "03 044 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 045 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 046 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 047 1E", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 048 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 049 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 04A 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 04B FC", 0);
	anx7580_write_reg_store(dev, NULL, "03 04C 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 04D 41", 0);
	anx7580_write_reg_store(dev, NULL, "03 04E 4E", 0);
	anx7580_write_reg_store(dev, NULL, "03 04F 58", 0);
	anx7580_write_reg_store(dev, NULL, "03 050 37", 0);
	anx7580_write_reg_store(dev, NULL, "03 051 35", 0);
	anx7580_write_reg_store(dev, NULL, "03 052 33", 0);
	anx7580_write_reg_store(dev, NULL, "03 053 30", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 054 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 055 55", 0);
	anx7580_write_reg_store(dev, NULL, "03 056 0A", 0);
	anx7580_write_reg_store(dev, NULL, "03 057 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 058 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 059 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 05A 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 05B 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 05C 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 05D FD", 0);
	anx7580_write_reg_store(dev, NULL, "03 05E 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 05F 2D", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 060 5A", 0);
	anx7580_write_reg_store(dev, NULL, "03 061 76", 0);
	anx7580_write_reg_store(dev, NULL, "03 062 77", 0);
	anx7580_write_reg_store(dev, NULL, "03 063 0E", 0);
	anx7580_write_reg_store(dev, NULL, "03 064 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 065 0A", 0);
	anx7580_write_reg_store(dev, NULL, "03 066 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 067 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 068 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 069 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 06A 20", 0);
	anx7580_write_reg_store(dev, NULL, "03 06B 20", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 06C 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 06D 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 06E 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 06F 10", 0);
	anx7580_write_reg_store(dev, NULL, "03 070 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 071 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 072 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 073 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 074 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 075 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 076 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 077 00", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 078 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 079 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 07A 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 07B 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 07C 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 07D 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 07E 01", 0);
	anx7580_write_reg_store(dev, NULL, "03 07F C7", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 080 02", 0);
	anx7580_write_reg_store(dev, NULL, "03 081 03", 0);
	anx7580_write_reg_store(dev, NULL, "03 082 0B", 0);
	anx7580_write_reg_store(dev, NULL, "03 083 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 084 E6", 0);
	anx7580_write_reg_store(dev, NULL, "03 085 06", 0);
	anx7580_write_reg_store(dev, NULL, "03 086 03", 0);
	anx7580_write_reg_store(dev, NULL, "03 087 01", 0);
	
	anx7580_write_reg_store(dev, NULL, "03 088 6A", 0);
	anx7580_write_reg_store(dev, NULL, "03 089 6A", 0);
	anx7580_write_reg_store(dev, NULL, "03 08A 03", 0);
	anx7580_write_reg_store(dev, NULL, "03 08B 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 08C 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 08D 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 08E 00", 0);
	anx7580_write_reg_store(dev, NULL, "03 08F 00", 0);

	anx7580_write_reg_store(dev, NULL, "03 0FF 29", 0);


	//panel parameterss setting
	
	//SW_H_ACTIVE 800
	anx7580_write_reg_store(dev, NULL, "01 0A0 20", 0);
	anx7580_write_reg_store(dev, NULL, "01 0A1 03", 0);
	
	//SW_HFP 276
	anx7580_write_reg_store(dev, NULL, "01 0A2 14", 0);
	anx7580_write_reg_store(dev, NULL, "01 0A3 01", 0);
	
	//SW_HSYNC 8
	anx7580_write_reg_store(dev, NULL, "01 0A4 08", 0);
	anx7580_write_reg_store(dev, NULL, "01 0A5 00", 0);
	
	//SW_HBP 44
	anx7580_write_reg_store(dev, NULL, "01 0A6 2C", 0);
	anx7580_write_reg_store(dev, NULL, "01 0A7 00", 0);
	
	//SW_V_ACTIVE 1280
	anx7580_write_reg_store(dev, NULL, "01 0A8 00", 0);
	anx7580_write_reg_store(dev, NULL, "01 0A9 05", 0);
	
	//SW_VFP 9
	anx7580_write_reg_store(dev, NULL, "01 0AA 09", 0);
	anx7580_write_reg_store(dev, NULL, "01 0AB 00", 0);
	
	//SW_VSYNC 1
	anx7580_write_reg_store(dev, NULL, "01 0AC 01", 0);
	anx7580_write_reg_store(dev, NULL, "01 0AD 00", 0);
	
	//SW_VBP 22
	anx7580_write_reg_store(dev, NULL, "01 0AE 16", 0);
	anx7580_write_reg_store(dev, NULL, "01 0AF 00", 0);
	
	//SW_PANEL_FRAME_RATE, 90Hz
	anx7580_write_reg_store(dev, NULL, "01 09D 5A", 0);
	
	//SW_PANEL_INFO_0, 1 panel, 1 MIPI port, 4 MIPI lane per port, Video one in one out mode
	anx7580_write_reg_store(dev, NULL, "01 0B0 0C", 0);
	
	//SW_PANEL_INFO_1, Set DPHY timing, Burst mode, no DSC, Display left
	anx7580_write_reg_store(dev, NULL, "01 0B1 48", 0);
	
	//for OLED panel, set 500 nits to OCM first, 500 nits = 0x7FF
	anx7580_write_reg_store(dev, NULL, "01 0B9 FF", 0);
	anx7580_write_reg_store(dev, NULL, "01 0BA 07", 0);
	
	//M_VALUE_MULTIPLY = 123 (*1.23)
	anx7580_write_reg_store(dev, NULL, "01 09F 7B", 0);
	
	//AMD_OLED_BACKLIGHT_SUPPORT (bit6), SET_ACCURATE_M_VALUE (bit3)
	anx7580_write_reg_store(dev, NULL, "01 092 48", 0);
	
	//MISC_NOTIFY_OCM0, MCU load done, panel info setting done
	anx7580_write_reg_store(dev, NULL, "01 09E C0", 0);
	//sleep 0.200
	msleep(200);

	pr_info("anx7580: %s exit\n", __func__);
}

static void sequence_2nd(struct device *dev)
{
	pr_info("anx7580: %s enter\n", __func__);

	msleep(1000);

	//# if 01:90 bit1 = 1 = video stable
	//# set panel DCS command set
	//#################################### ANX7580 interrupt handling
	//get interrupt event
	//anx7580_read_reg_show(dev, NULL, "01 90 1");
	//anx7580_read_reg_show(dev, NULL, "01 91 1");
	//clear interrupt
	anx7580_write_reg_store(dev, NULL, "A0 044 40", 0);
	anx7580_write_reg_store(dev, NULL, "01 090 00", 0);
	anx7580_write_reg_store(dev, NULL, "01 091 00", 0);
	
	//#################################### interrupt event = "video stable"
	//#
	//# DATASHORT_DCS_WRITE_1 		  0x15
	//# DATASHORT_DCS_WRITE_LONG		  0x39
	//#
	//####################################
	//Panel power up ###
	//power on panel
	
	//LP mode ###
	anx7580_write_reg_store(dev, NULL, "a0 10 0", 0);
	anx7580_write_reg_store(dev, NULL, "a0 0 3F", 0);
	anx7580_write_reg_store(dev, NULL, "a0 14 0", 0);
	
	anx7580_write_reg_store(dev, NULL, "c0 34 01", 0);
	
	//select port first (0xC0)
	anx7580_write_reg_store(dev, NULL, "08 19 10", 0);
	
	//panel power on sequence, please refer to Samsung datasheet
	//Panel HW reset ###
	
	//send DCS "panel initial"
	
	//Bulldog initial code
	
	//Unlock
	anx7580_write_reg4_store(dev, NULL, "C0 70 005A5AF0", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 005A5AF1", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	
	//anx7580_write_reg_store4equency Setting
	anx7580_write_reg4_store(dev, NULL, "C0 70 004700D0", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 0022B015", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 0097BD15", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 002CB015", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 AA2AAAB7", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 0000000A", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000539", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 0007F715", 0);
	
	//Lock
	anx7580_write_reg4_store(dev, NULL, "C0 70 00A5A5F0", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 00A5A5F1", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	
	//Unlock
	anx7580_write_reg4_store(dev, NULL, "C0 70 005A5AF0", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 005A5AF1", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	
	//Sleep Out Setting
	anx7580_write_reg4_store(dev, NULL, "C0 6C 0029B015", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 0000BD15", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 0007EF15", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 0004B015", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 0004EF15", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 0008B015", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 0049F315", 0);
	
	//Lock
	anx7580_write_reg4_store(dev, NULL, "C0 70 00A5A5F0", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 00A5A5F1", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	
	//SLP OUT
	//SSD_Number(0x01);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00001105", 0);
	
	//Delay 180 ms;
	msleep(180);
	//sleep 0.180
	
	//Unlock
	anx7580_write_reg4_store(dev, NULL, "C0 70 005A5AF0", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 005A5AF1", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	
	//High Brightness Mode Setting
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00BBB015", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 0000FBB1", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000239", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 CC0003B5", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000439", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 0CD30CBA", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 00FA0D7E", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000739", 0);
	
	//Lock
	anx7580_write_reg4_store(dev, NULL, "C0 70 00A5A5F0", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 70 00A5A5F1", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	
	//Turn on Normal Brightness Mode
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00285315", 0);
	//500 NITS
	anx7580_write_reg4_store(dev, NULL, "C0 70 00FF0751", 0);
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00000339", 0);
	
	//Delay 20 ms;
	msleep(20);
	//sleep 0.020
	
	//DISP ON
	anx7580_write_reg4_store(dev, NULL, "C0 6C 00002905", 0);
	
	//Delay 5 ms;
	msleep(5);
	//sleep 0.005
	
	//HS mode ###
	
	anx7580_write_reg_store(dev, NULL, "a0 10 7F", 0);
	anx7580_write_reg_store(dev, NULL, "a0 0 3F", 0);
	anx7580_write_reg_store(dev, NULL, "a0 14 1F", 0);
	
	anx7580_write_reg_store(dev, NULL, "c0 34 00000000", 0);
	
	//set HPD to unforce high
	anx7580_write_reg_store(dev, NULL, "30 028 08", 0);

	pr_info("anx7580: %s exit\n", __func__);
}

static int anx7580_resume_early(struct device *dev)
{
	sequence_1st(dev);
	return 0;
}

static void anx7580_complete(struct device *dev)
{
	sequence_2nd(dev);
}

//static DEFINE_RUNTIME_DEV_PM_OPS(anx7580_pm_ops, anx7580_runtime_suspend,
//				 anx7580_runtime_resume, NULL);
static const struct dev_pm_ops anx7580_pm_ops = {
	.complete = anx7580_complete,
	.resume_early = anx7580_resume_early,
	.suspend_late = anx7580_suspend_late,
};

static const struct acpi_device_id anx7580_acpi_match[] = {
        { "ANX7580A", 0 },
        { }
};
MODULE_DEVICE_TABLE(acpi, anx7580_acpi_match);

static const struct i2c_device_id anx7580_i2c_ids[] = {
	{ "anx7580", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, anx7580_i2c_ids);

static const struct of_device_id anx7580_of_ids[] = {
	{ .compatible = "analogix,anx7580", },
	{ }
};
MODULE_DEVICE_TABLE(of, anx7580_of_ids);

static struct i2c_driver anx7580_i2c_driver = {
        .driver = {
                .name   = "anx7580",
                .pm     = &anx7580_pm_ops,
                .of_match_table = anx7580_of_ids,
                .acpi_match_table = anx7580_acpi_match,
        },
        .probe          = anx7580_probe,
        .remove         = anx7580_remove,
        .id_table       = anx7580_i2c_ids,
};
module_i2c_driver(anx7580_i2c_driver);

MODULE_AUTHOR("Keith Mikoleit <keithm@valvesoftware.com>");
MODULE_DESCRIPTION("Analogix ANX7580 DP to MIPI Bridge Interface");
MODULE_LICENSE("GPL");
