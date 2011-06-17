/* $Id$
 * $URL$
 *
 * TeakLCM lcd4linux driver
 *
 * Copyright (C) 2005 Michael Reinelt <michael@reinelt.co.at>
 * Copyright (C) 2005, 2006, 2007 The LCD4Linux Team <lcd4linux-devel@users.sourceforge.net>
 * Copyright (C) 2011 ixs
 *
 * This file is part of LCD4Linux.
 *
 * LCD4Linux is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * LCD4Linux is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/* 
 *
 * exported fuctions:
 *
 * struct DRIVER drv_TeakLCM
 *
 */

#include "config.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <assert.h>

#include "debug.h"
#include "cfg.h"
#include "qprintf.h"
#include "udelay.h"
#include "plugin.h"
#include "widget.h"
#include "widget_text.h"
#include "widget_icon.h"
#include "widget_bar.h"
#include "drv.h"

#include "drv_generic_text.h"
#include "drv_generic_serial.h"


static char Name[] = "TeakLCM";


#define HI8(value) ((unsigned char)(((value)>>8) & 0xff))
#define LO8(value) ((unsigned char)((value) & 0xff))


static u_int16_t CRC16(u_int8_t value, u_int16_t crcin)
{
    u_int16_t k = (((crcin >> 8) ^ value) & 255) << 8;
    u_int16_t crc = 0;
    int bits;
    for (bits=8; bits; --bits) {
	if ((crc ^ k) & 0x8000)
	    crc = (crc << 1) ^ 0x1021;
	else
	    crc <<= 1;
	k <<= 1;
    }
    return ((crcin << 8) ^ crc);
}


#if 0
static u_int16_t CRC16Buf(unsigned int count, unsigned char *buffer)
{
    u_int16_t crc = 0;
    do {
	u_int8_t value = *buffer++;
	crc = CRC16(value, crc);
    } while (--count);
    return crc;
}
#endif


/** Return a printable character */
static char printable(const char ch)
{
    if ((32 <= ch) && (ch < 127)) {
	return ch;
    } else {
	return '.';
    }
}


static void debug_data(const char *prefix, const void *data, const size_t size)
{
    const u_int8_t *b = (const u_int8_t *)data;
    size_t y;
    for (y=0; y<size; y+=16) {
	char buf[80];
	size_t x;
	ssize_t idx = 0;
	idx += sprintf(&(buf[idx]), "%04x ", y);
	for (x=0; x<16; x++) {
	    const size_t i = x+y;
	    if (i<size) {
		idx += sprintf(&(buf[idx]), " %02x", b[i]);
	    } else {
		idx += sprintf(&(buf[idx]), "   ");
	    }
	}
	idx += sprintf(&buf[idx], "  ");
	for (x=0; x<16; x++) {
	    const size_t i = x+y;
	    if (i<size) {
		idx += sprintf(&buf[idx], "%c", printable(b[i]));
	    } else {
		idx += sprintf(&buf[idx], " ");
	    }
	}
	debug("%s%s", prefix, buf);
    }
}


typedef enum {
    CMD_CONNECT         = 0x05,
    CMD_DISCONNECT      = 0x06,
    CMD_ALARM           = 0x07,
    CMD_WRITE           = 0x08,
    CMD_PRINT1          = 0x09,
    CMD_PRINT2          = 0x0A,
    CMD_ACK             = 0x0B,
    CMD_NACK            = 0x0C,
    CMD_CONFIRM         = 0x0D,
    CMD_RESET           = 0x0E,

    LCM_CLEAR           = 0x21,
    LCM_HOME            = 0x22,
    LCM_CURSOR_SHIFT_R  = 0x23,
    LCM_CURSOR_SHIFT_L  = 0x24,
    LCM_BACKLIGHT_ON    = 0x25,
    LCM_BACKLIGHT_OFF   = 0x26,
    LCM_LINE2           = 0x27,
    LCM_DISPLAY_SHIFT_R = 0x28,
    LCM_DISPLAY_SHIFT_L = 0x29,
    LCM_CURSOR_ON       = 0x2A,
    LCM_CURSOR_OFF      = 0x2B,
    LCM_CURSOR_BLINK    = 0x2C,
    LCM_DISPLAY_ON      = 0x2D,
    LCM_DISPLAY_OFF     = 0x2E
} lcm_cmd_t;


const char *cmdstr(const lcm_cmd_t cmd) {
   switch (cmd) {
#define D(CMD) case CMD_ ## CMD: return "CMD_" # CMD; break;
	D(CONNECT);
	D(DISCONNECT);
	D(ACK);
	D(NACK);
	D(CONFIRM);
	D(RESET);
	D(ALARM);
	D(WRITE);
	D(PRINT1);
	D(PRINT2);
#undef D
#define D(CMD) case LCM_ ## CMD: return "LCM_" # CMD; break;
	D(CLEAR);
	D(HOME);
	D(CURSOR_SHIFT_R);
	D(CURSOR_SHIFT_L);
	D(BACKLIGHT_ON);
	D(BACKLIGHT_OFF);
	D(LINE2);
	D(DISPLAY_SHIFT_R);
	D(DISPLAY_SHIFT_L);
	D(CURSOR_ON);
	D(CURSOR_OFF);
	D(CURSOR_BLINK);
	D(DISPLAY_ON);
	D(DISPLAY_OFF);
#undef D
    }
    return "CMD_UNKNOWN";
}


/*
 * Magic defines
 */

#define LCM_FRAME_MASK      0xFF
#define LCM_TIMEOUT         2
#define LCM_ESC             0x1B

#define LCM_KEY1            0x31
#define LCM_KEY2            0x32
#define LCM_KEY3            0x33
#define LCM_KEY4            0x34
#define LCM_KEY12           0x35
#define LCM_KEY13           0x36
#define LCM_KEY14           0x37
#define LCM_KEY23           0x38
#define LCM_KEY24           0x39
#define LCM_KEY34           0x3A


/****************************************/
/***  hardware dependant functions    ***/
/****************************************/

/* global LCM state machine */


typedef enum { MODE_0, MODE_1, MODE_IDLE } lcm_mode_t;


const char *modestr(const lcm_mode_t mode) {
    switch (mode) {
    case MODE_0: return "MODE 0"; break;
    case MODE_1: return "MODE 1"; break;
    case MODE_IDLE: return "IDLE"; break;
    }
    return "MODE_UNKNOWN";
}


static lcm_mode_t lcm_mode = MODE_0;


static void lcm_receive_check(void);
static void lcm_handle_cmd_frame(lcm_cmd_t cmd);
static void lcm_handle_data_frame(const lcm_cmd_t cmd,
				  const u_int8_t *payload,
				  unsigned int payload_len);


static void lcm_receive_check(void)
{
    static u_int8_t rxframe[32];
    const int readlen = drv_generic_serial_poll((void *)rxframe, sizeof(rxframe));
    unsigned int framelen = 0;
    if (framelen <= 0) {
	debug("%s Received no data", __FUNCTION__);
	return;
    }
    framelen = readlen;
    debug("%s received %d bytes", __FUNCTION__, framelen);
    debug_data(" RX ", rxframe, framelen);
    if (framelen == 3 &&
	       (rxframe[0] == LCM_FRAME_MASK) &&
	       (rxframe[2] == LCM_FRAME_MASK)) {
	const lcm_cmd_t cmd = rxframe[1];
	debug("%s Received cmd frame (cmd=%d=%s)", __FUNCTION__, cmd, cmdstr(cmd));
	lcm_handle_cmd_frame(cmd);
	return;
    } else if (rxframe[0] == LCM_FRAME_MASK) {
	unsigned int ri; /* raw indexed */
	unsigned int ci; /* cooked indexed */

	debug("%s Received possible data frame", __FUNCTION__);

	/* unescape rxframe data in place */
	unsigned int truelen = framelen;
	for (ri=1, ci=1; ri < framelen; ri++) {
	    if (rxframe[ri] == LCM_ESC) {
		ri++;
		truelen--;
	    }
	    rxframe[ci++] = rxframe[ri];
	}

	/* calculate CRC for unescaped data */
	u_int16_t crc=0;
	for (ci=1; ci<truelen-3; ci++) {
	    crc = CRC16(crc, ci);
	}
	if ((rxframe[ci+0]==HI8(crc)) &&
	    (rxframe[ci+1]==LO8(crc)) &&
	    (rxframe[ci+2]==LCM_FRAME_MASK)) {
	    lcm_cmd_t cmd = rxframe[1];
	    u_int16_t len = (rxframe[3]<<8) + rxframe[2];
	    lcm_handle_data_frame(cmd, &rxframe[5], len);
	} else {
	    debug("%s checksum/framemask error", __FUNCTION__);
	}
	return;
    } else {
	debug("%s Received garbage data", __FUNCTION__);
	return;
    }
}


/* Send a command frame to the board */
static void lcm_send_cmd_frame(lcm_cmd_t cmd)
{
    lcm_receive_check();
    char cmd_buf[3];
    cmd_buf[0] = LCM_FRAME_MASK;
    cmd_buf[1] = cmd;
    cmd_buf[2] = LCM_FRAME_MASK;
    debug("%s sending cmd frame cmd=%d=%s", __FUNCTION__, cmd, cmdstr(cmd));
    debug_data(" TX ", cmd_buf, 3);
    drv_generic_serial_write(cmd_buf, 3);
    usleep(100000);
}


static void lcm_handle_cmd_frame(lcm_cmd_t cmd)
{
    switch (lcm_mode) {
    case MODE_0:
    case MODE_1:
	switch (cmd) {
	case CMD_CONNECT: lcm_send_cmd_frame(CMD_ACK); break;
	case CMD_NACK:    lcm_send_cmd_frame(CMD_CONFIRM); break;
	case CMD_CONFIRM: lcm_mode = MODE_IDLE; break;
	default:
	    error("%s: Unhandled cmd %s in state %s", Name, cmdstr(cmd), modestr(lcm_mode));
	    break;
	}
	break;
    case MODE_IDLE:
	switch (cmd) {
	case CMD_ACK: lcm_send_cmd_frame(CMD_CONFIRM); break;
	case CMD_DISCONNECT: lcm_send_cmd_frame(CMD_ACK); break;
	default:
	    error("%s: Unhandled cmd %s in state %s", Name, cmdstr(cmd), modestr(lcm_mode));
	    break;
	}
	break;
    }
}



static void lcm_handle_data_frame(const lcm_cmd_t cmd,
				  const u_int8_t *payload,
				  unsigned int payload_len)
{
    switch (lcm_mode) {
    case MODE_IDLE:
	switch (cmd) {
	case CMD_WRITE:
	    assert(payload_len == 1);
	    debug("Got a key %c=0x%x", *payload, *payload);
	    lcm_send_cmd_frame(CMD_ACK);
	    break;
	default:
	    debug("Got an unknown data frame: %d=%s", cmd, cmdstr(cmd));
	    lcm_send_cmd_frame(CMD_NACK);
	    break;
	}
	break;
    case MODE_0:
    case MODE_1:
	lcm_send_cmd_frame(CMD_NACK);
	break;
    }
}


#if 0
static int lcm_expect_cmd(lcm_cmd_t cmd)
{
    /* give LCM 100ms time to reply */
    usleep(100000);
    static unsigned char buf[3];

    int retchar = drv_generic_serial_read((void *)buf, 3);

    if (retchar != 3) {
	debug("%s: No %s cmd received", Name, cmdstr(cmd));
	return 0;
    } else if ((buf[0] != LCM_FRAME_MASK) ||
	       (buf[2] != LCM_FRAME_MASK)) {
	debug("%s: Invalid cmd received waiting for %d", Name, cmd);
	return 0;
    }
    lcm_handle_cmd_frame(buf[1]);
    if (buf[1] == cmd) {
	return 1;
    } else {
	return 0;
    }
}
#endif


/* Initialize the LCM by completing the handshake */
static void drv_TeakLCM_connect()
{
    lcm_mode = MODE_0;
    lcm_send_cmd_frame(CMD_RESET);
    usleep(100000);

    lcm_send_cmd_frame(CMD_ACK);
}

static int drv_TeakLCM_open(const char *section)
{
    /* open serial port */
    /* don't mind about device, speed and stuff, this function will take care of */

    if (drv_generic_serial_open(section, Name, 0) < 0)
	return -1;

    return 0;
}



static int drv_TeakLCM_close(void)
{
    /* close whatever port you've opened */
    drv_generic_serial_close();

    return 0;
}


// drv_generic_serial_write(data, len);


static void lcm_send_data_frame(lcm_cmd_t cmd, const char *data, const unsigned int len)
{
    unsigned int di; /* data index */
    unsigned int fi; /* frame index */
    static char frame[32];
    u_int16_t crc = 0;

    lcm_receive_check();

    frame[0] = LCM_FRAME_MASK;

    frame[1] = cmd;
    crc = CRC16(frame[1], crc);

    frame[2] = HI8(len);
    crc = CRC16(frame[2], crc);

    frame[3] = LO8(len);
    crc = CRC16(frame[3], crc);

#define APPEND(value)				       \
    do {					       \
	const unsigned char v = (value);	       \
	if ((v == LCM_FRAME_MASK) || (v == LCM_ESC)) { \
	    frame[fi++] = LCM_ESC;		       \
	}					       \
	frame[fi++] = v;			       \
	crc = CRC16(v, crc);			       \
    } while (0)

#define APPEND_NOCRC(value)			       \
    do {					       \
	const unsigned char v = (value);	       \
	if ((v == LCM_FRAME_MASK) || (v == LCM_ESC)) { \
	    frame[fi++] = LCM_ESC;		       \
	}					       \
	frame[fi++] = v;			       \
    } while (0)

    for (fi=4, di=0; di<len; di++) {
	APPEND(data[di]);
    }

    APPEND_NOCRC(HI8(crc));
    APPEND_NOCRC(LO8(crc));

    frame[fi++] = LCM_FRAME_MASK;

    debug_data("snd ", frame, fi);
    drv_generic_serial_write(frame, fi);

    usleep(100000);

#undef LO8
#undef HI8
#undef APPEND
}


/* text mode displays only */
static void drv_TeakLCM_clear(void)
{
    /* do whatever is necessary to clear the display */
    lcm_send_cmd_frame(LCM_CLEAR);
}


/* shadow buffer */
char *shadow;


/* text mode displays only */
static void drv_TeakLCM_write(const int row, const int col, const char *data, int len)
{
    debug("%s row=%d col=%d len=%d data=\"%s\"", __FUNCTION__,
	  row, col, len, data);

    memcpy(&shadow[DCOLS*row+col], data, len);

    debug_data("shadow ", shadow, DCOLS*DROWS);

    lcm_send_data_frame((row == 0)?CMD_PRINT1:CMD_PRINT2,
			&shadow[DCOLS*row], DCOLS);
}


/* text mode displays only */
#if 0
static void drv_TeakLCM_defchar(const int ascii, const unsigned char *matrix)
{
    char cmd[10];
    int i;

    /* call the 'define character' function */
    /* assume 0x03 to be the 'defchar' command */
    cmd[0] = 0x03;
    cmd[1] = ascii;

    /* send bitmap to the display */
    for (i = 0; i < 8; i++) {
	cmd[i + 2] = *matrix++;
    }
    drv_TeakLCM_send(cmd, 10);
}
#endif

/* example function used in a plugin */
#if 0
static int drv_TeakLCM_contrast(int contrast)
{
    char cmd[2];

    /* adjust limits according to the display */
    if (contrast < 0)
	contrast = 0;
    if (contrast > 255)
	contrast = 255;

    /* call a 'contrast' function */
    /* assume 0x04 to be the 'set contrast' command */
    cmd[0] = 0x04;
    cmd[1] = contrast;
    drv_TeakLCM_send(cmd, 2);

    return contrast;
}
#endif

/* start text mode display */
static int drv_TeakLCM_start(const char *section)
{
#if 0
    int contrast;
#endif
    int rows = -1, cols = -1;
    char *s;

    s = cfg_get(section, "Size", NULL);
    if (s == NULL || *s == '\0') {
	error("%s: no '%s.Size' entry from %s", Name, section, cfg_source());
	return -1;
    }
    if (sscanf(s, "%dx%d", &cols, &rows) != 2 || rows < 1 || cols < 1) {
	error("%s: bad %s.Size '%s' from %s", Name, section, s, cfg_source());
	free(s);
	return -1;
    }
 
    DROWS = rows;
    DCOLS = cols;
    shadow = malloc(DROWS*DCOLS);
    memset(shadow, 32, DROWS*DCOLS);

    /* open communication with the display */
    if (drv_TeakLCM_open(section) < 0) {
	return -1;
    }
    debug("%s: %s opened", Name, __FUNCTION__);

    /* reset & initialize display */
    drv_TeakLCM_connect();

    debug("%s: %s connected", Name, __FUNCTION__);

#if 0
    if (cfg_number(section, "Contrast", 0, 0, 255, &contrast) > 0) {
	drv_TeakLCM_contrast(contrast);
    }
#endif

    drv_TeakLCM_clear();	/* clear display */
    lcm_send_cmd_frame(LCM_BACKLIGHT_ON);
    lcm_send_cmd_frame(LCM_DISPLAY_ON);

    debug("%s: %s done", Name, __FUNCTION__);
    return 0;
}


/****************************************/
/***            plugins               ***/
/****************************************/

#if 0
static void plugin_contrast(RESULT * result, RESULT * arg1)
{
    double contrast;

    contrast = drv_TeakLCM_contrast(R2N(arg1));
    SetResult(&result, R_NUMBER, &contrast);
}
#endif


/****************************************/
/***        widget callbacks          ***/
/****************************************/


/* using drv_generic_text_draw(W) */
/* using drv_generic_text_icon_draw(W) */
/* using drv_generic_text_bar_draw(W) */


/****************************************/
/***        exported functions        ***/
/****************************************/


/* list models */
int drv_TeakLCM_list(void)
{
    printf("TeakLCM driver");
    return 0;
}


/* initialize driver & display */
/* use this function for a text display */
int drv_TeakLCM_init(const char *section, const int quiet)
{
    WIDGET_CLASS wc;
    int ret;

    info("%s: %s", Name, "$Rev$");

    /* display preferences */
    XRES = 5;			/* pixel width of one char  */
    YRES = 8;			/* pixel height of one char  */
    CHARS = 0;			/* number of user-defineable characters */
    CHAR0 = 0;			/* ASCII of first user-defineable char */
    GOTO_COST = -1;		/* number of bytes a goto command requires */

    /* real worker functions */
    drv_generic_text_real_write = drv_TeakLCM_write;
#if 0
    drv_generic_text_real_defchar = drv_TeakLCM_defchar;
#endif


    /* start display */
    if ((ret = drv_TeakLCM_start(section)) != 0)
	return ret;

    if (!quiet) {
	char buffer[40];
	qprintf(buffer, sizeof(buffer), "%s %dx%d", Name, DCOLS, DROWS);
	if (drv_generic_text_greet(buffer, "Moo!")) {
	    sleep(3);
	    drv_TeakLCM_clear();
	}
    }

    /* initialize generic text driver */
    if ((ret = drv_generic_text_init(section, Name)) != 0)
	return ret;

#if 0
    /* initialize generic icon driver */
    if ((ret = drv_generic_text_icon_init()) != 0)
	return ret;

    /* initialize generic bar driver */
    if ((ret = drv_generic_text_bar_init(0)) != 0)
	return ret;

    /* add fixed chars to the bar driver */
    drv_generic_text_bar_add_segment(0, 0, 255, 32);	/* ASCII  32 = blank */
#endif

    /* register text widget */
    wc = Widget_Text;
    wc.draw = drv_generic_text_draw;
    widget_register(&wc);

#if 0
    /* register icon widget */
    wc = Widget_Icon;
    wc.draw = drv_generic_text_icon_draw;
    widget_register(&wc);

    /* register bar widget */
    wc = Widget_Bar;
    wc.draw = drv_generic_text_bar_draw;
    widget_register(&wc);

    /* register plugins */
    AddFunction("LCD::contrast", 1, plugin_contrast);
#endif

    return 0;
}



/* close driver & display */
/* use this function for a text display */
int drv_TeakLCM_quit(const int quiet)
{

    info("%s: shutting down.", Name);

    drv_generic_text_quit();

    /* clear display */
    drv_TeakLCM_clear();

    /* say goodbye... */
    if (!quiet) {
	drv_generic_text_greet("goodbye!", NULL);
    }

    lcm_send_cmd_frame(LCM_DISPLAY_OFF);
    // lcm_send_cmd_frame(LCM_BACKLIGHT_OFF);

    debug("closing connection");
    drv_TeakLCM_close();

    return (0);
}

/* use this one for a text display */
DRIVER drv_TeakLCM = {
    .name = Name,
    .list = drv_TeakLCM_list,
    .init = drv_TeakLCM_init,
    .quit = drv_TeakLCM_quit,
};

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
