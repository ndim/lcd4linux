/* $Id$
 * $URL$
 *
 * TeakLCM lcd4linux driver
 *
 * Copyright (C) 2005 Michael Reinelt <michael@reinelt.co.at>
 * Copyright (C) 2005, 2006, 2007 The LCD4Linux Team <lcd4linux-devel@users.sourceforge.net>
 * Copyright (C) 2011 ixs
 * Copyright (C) 2011 Hans Ulrich Niedermann
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


static void debug_data_int(const char *prefix, const void *data, const size_t size,
			   const unsigned int delta)
{
    const u_int8_t *b = (const u_int8_t *)data;
    size_t y;
    assert(delta <= 24);
    for (y=0; y<size; y+=delta) {
	char buf[100];
	size_t x;
	ssize_t idx = 0;
	idx += sprintf(&(buf[idx]), "%04x ", y);
	for (x=0; x<delta; x++) {
	    const size_t i = x+y;
	    if (i<size) {
		idx += sprintf(&(buf[idx]), " %02x", b[i]);
	    } else {
		idx += sprintf(&(buf[idx]), "   ");
	    }
	}
	idx += sprintf(&buf[idx], "  ");
	for (x=0; x<delta; x++) {
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


static void debug_data(const char *prefix, const void *data, const size_t size)
{
    debug_data_int(prefix, data, size, 16);
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


struct _lcm_fsm_t;
typedef struct _lcm_fsm_t lcm_fsm_t;


typedef enum {
    ST_IDLE,     /* mode == 0, IDLE */
    ST_COMMAND,  /* mode == 1, COMMAND */
    ST_CONNECTED /* mode == 2, CONNECTED */
} lcm_state_t;


const char *state2str(const lcm_state_t state) {
    switch (state) {
    case ST_IDLE:      return "ST_IDLE (0)"; break;
    case ST_COMMAND:   return "ST_COMMAND (1)"; break;
    case ST_CONNECTED: return "ST_CONNECTED (2)"; break;
    }
    return "ST_UNKNOWN";
}


static
void raw_send_cmd_frame(lcm_cmd_t cmd);

static
void raw_send_data_frame(lcm_cmd_t cmd,
			 const char *data, const unsigned int len);


static
int lcm_receive_check(void);


static
lcm_state_t fsm_get_state(lcm_fsm_t *fsm);

static
void fsm_handle_cmd(lcm_fsm_t *fsm,
		    const lcm_cmd_t cmd);

static
void fsm_handle_datacmd(lcm_fsm_t *fsm,
			const lcm_cmd_t cmd,
			const u_int8_t *payload,
			const unsigned int payload_len);

static
void fsm_step(lcm_fsm_t *fsm);

static
void fsm_trans_noop(lcm_fsm_t *fsm,
		    const lcm_state_t next_state);

static
void fsm_trans_cmd(lcm_fsm_t *fsm,
		   const lcm_state_t next_state,
		   const lcm_cmd_t cmd);

static
void fsm_trans_data(lcm_fsm_t *fsm,
		    const lcm_state_t next_state,
		    const lcm_cmd_t cmd,
		    const char *data, const unsigned int len);


static
int fsm_handle_bytes(lcm_fsm_t *fsm,
		     u_int8_t *rxbuf, const unsigned int buflen)
{
    if ((buflen >= 3) &&
	(rxbuf[0] == LCM_FRAME_MASK) &&
	(rxbuf[2] == LCM_FRAME_MASK)) {
	const lcm_cmd_t cmd = rxbuf[1];
	debug("%s Received cmd frame (cmd=%d=%s)", __FUNCTION__, cmd, cmdstr(cmd));
	fsm_handle_cmd(fsm, cmd);
	if (buflen > 3) {
	    fsm_handle_bytes(fsm, &rxbuf[3], buflen-3);
	}
	return 1;
    } else if ((buflen > 3) &&
	       (rxbuf[0] == LCM_FRAME_MASK)) {
	unsigned int ri; /* raw indexed */
	unsigned int ci; /* cooked indexed, i.e. after unescaping */

	debug("%s Received possible data frame", __FUNCTION__);

	/* unescape rxframe data in place */
	unsigned int clen = buflen;
	for (ri=1, ci=1; ri < buflen; ri++) {
	    if ((ri < (buflen-1)) && (rxbuf[ri] == LCM_FRAME_MASK)) {
		/* Unescaped LCM_FRAME_MASK. Should not happen - means
		   broken frame. */
		fsm_trans_cmd(fsm, fsm_get_state(fsm), /* TODO: Is this a good next_state value? */
			     CMD_NACK);
		debug("%s framemask error", __FUNCTION__);
		return 1;
	    } else if (rxbuf[ri] == LCM_ESC) {
		ri++;
		clen--;
	    }
	    rxbuf[ci++] = rxbuf[ri];
	}

	/* calculate CRC for unescaped data */
	u_int16_t crc=0;
	for (ci=1; ci<clen-3; ci++) {
	    crc = CRC16(crc, ci);
	}
	if ((rxbuf[ci+0]==HI8(crc)) &&
	    (rxbuf[ci+1]==LO8(crc)) &&
	    (rxbuf[ci+2]==LCM_FRAME_MASK)) {
	    lcm_cmd_t cmd = rxbuf[1];
	    u_int16_t len = (rxbuf[3]<<8) + rxbuf[2];
	    fsm_handle_datacmd(fsm, cmd, &rxbuf[5], len);
	    return 1;
	} else {
	    fsm_trans_cmd(fsm, fsm_get_state(fsm), /* TODO: Is this a good next_state value? */
			 CMD_NACK);
	    debug("%s checksum/framemask error", __FUNCTION__);
	    return 1;
	}
    } else {
	debug("%s Received garbage data:", __FUNCTION__);
	debug_data(" RXD ", rxbuf, buflen);
	return 1;
    }
}


static int lcm_receive_check(void);


static void fsm_handle_cmd(lcm_fsm_t *fsm, lcm_cmd_t cmd)
{
    // debug("fsm_handle_cmd: old state 0x%02x %s", lcm_mode, modestr(lcm_mode));
    const lcm_state_t old_state = fsm_get_state(fsm);
    switch (old_state) {
    case ST_IDLE:
    case ST_COMMAND:
	switch (cmd) {
	case CMD_CONNECT:
	    fsm_trans_cmd(fsm, ST_COMMAND, CMD_ACK);
	    break;
	case CMD_ACK:
	    fsm_trans_cmd(fsm, ST_CONNECTED, CMD_CONFIRM);
	    break;
	case CMD_NACK:
	    fsm_trans_cmd(fsm, ST_IDLE, CMD_CONFIRM);
	    break;
	case CMD_CONFIRM:
	    fsm_trans_noop(fsm, ST_CONNECTED);
	    break;
	case CMD_RESET:
	    fsm_trans_cmd(fsm, ST_COMMAND, CMD_CONNECT);
	    break;
	default:
	    error("%s: Unhandled cmd %s in state %s", Name,
		  cmdstr(cmd), state2str(old_state));
	    fsm_trans_cmd(fsm, ST_IDLE, CMD_NACK);
	    break;
	}
	break;
    case ST_CONNECTED: /* "if (mode == 2)" */
	switch (cmd) {
	case CMD_ACK:
	    fsm_trans_cmd(fsm, ST_CONNECTED, CMD_CONFIRM);
	    break;
	case CMD_CONNECT:
	    fsm_trans_cmd(fsm, ST_CONNECTED, CMD_NACK);
	    break;
	case CMD_DISCONNECT:
	    fsm_trans_cmd(fsm, ST_CONNECTED, CMD_ACK);
	    break;
	case CMD_RESET:
	    fsm_trans_cmd(fsm, ST_IDLE, CMD_CONNECT);
	    break;
	default:
	    error("%s: Unhandled cmd %s in state %s", Name,
		  cmdstr(cmd), state2str(old_state));
	    break;
	}
	break;
    }
    fsm_step(fsm);
    usleep(1000000);
    lcm_receive_check();
    usleep(1000000);
    lcm_receive_check();
}


static
void fsm_handle_datacmd(lcm_fsm_t *fsm,
			const lcm_cmd_t cmd,
			const u_int8_t *payload,
			unsigned int payload_len)
{
    const lcm_state_t old_state = fsm_get_state(fsm);
    debug("fsm_handle_datacmd: old state 0x%02x %s", old_state, state2str(old_state));
    switch (old_state) {
    case ST_CONNECTED:
	switch (cmd) {
	case CMD_WRITE:
	    assert(payload_len == 1);
	    debug("Got a key %c=0x%x", *payload, *payload);
	    fsm_trans_noop(fsm, ST_CONNECTED);
	    // lcm_send_cmd_frame(CMD_ACK);
	    break;
	default:
	    debug("Got an unknown data frame: %d=%s", cmd, cmdstr(cmd));
	    fsm_trans_noop(fsm, ST_CONNECTED);
	    // lcm_send_cmd_frame(CMD_NACK);
	    break;
	}
	break;
    case ST_IDLE:
    case ST_COMMAND:
	fsm_trans_cmd(fsm, old_state, CMD_NACK);
	break;
    }
    fsm_step(fsm);
    usleep(1000000);
    lcm_receive_check();
}


struct _lcm_fsm_t {
    lcm_state_t state;
    lcm_state_t next_state;
    enum {
	ACTION_UNINITIALIZED,
	ACTION_NOOP,
	ACTION_CMD,
	ACTION_DATA
    } action_type;
    union {
	struct {
	    lcm_cmd_t cmd;
	} cmd_frame;
	struct {
	    lcm_cmd_t cmd;
	    const char *data;
	    unsigned int len;
	} data_frame;
    } action;
};


static
lcm_state_t fsm_get_state(lcm_fsm_t *fsm)
{
    return fsm->state;
}


static
void fsm_step(lcm_fsm_t *fsm)
{
    debug("fsm: old_state=%s new_state=%s",
	  state2str(fsm->state), state2str(fsm->next_state));
    switch (fsm->action_type) {
    case ACTION_UNINITIALIZED:
	error("Uninitialized LCM FSM action");
	break;
    case ACTION_NOOP:
	break;
    case ACTION_CMD:
	raw_send_cmd_frame(fsm->action.cmd_frame.cmd);
	break;
    case ACTION_DATA:
	raw_send_data_frame(fsm->action.data_frame.cmd,
			    fsm->action.data_frame.data,
			    fsm->action.data_frame.len);
	break;
    }
    fsm->action_type = ACTION_UNINITIALIZED;
    switch (fsm->next_state) {
    case ST_IDLE:
    case ST_COMMAND:
    case ST_CONNECTED:
	fsm->state = fsm->next_state;
	fsm->next_state = -1;
	return;
	break;
    }
    error("LCM FSM: Illegal next_state");
}


static
void fsm_trans_noop(lcm_fsm_t *fsm,
		    const lcm_state_t next_state)
{
    fsm->next_state = next_state;
    fsm->action_type = ACTION_NOOP;
}


static
void fsm_trans_cmd(lcm_fsm_t *fsm,
		  const lcm_state_t next_state,
		  const lcm_cmd_t cmd)
{
    fsm->next_state = next_state;
    fsm->action_type = ACTION_CMD;
    fsm->action.cmd_frame.cmd = cmd;
}


static
void fsm_trans_data(lcm_fsm_t *fsm,
		   const lcm_state_t next_state,
		   const lcm_cmd_t cmd,
		   const char *data, const unsigned int len)
{
    fsm->next_state = next_state;
    fsm->action_type = ACTION_DATA;
    fsm->action.data_frame.cmd = cmd;
    fsm->action.data_frame.data = data;
    fsm->action.data_frame.len = len;
}


static
void fsm_send(lcm_fsm_t *fsm, const lcm_cmd_t cmd)
{
    const lcm_state_t old_state = fsm_get_state(fsm);
    switch (old_state) {
    case ST_IDLE:
    case ST_COMMAND:
	/* Silently ignore the command to send. */
	/* TODO: Would it be better to queue it and send it later? */
	break;
    case ST_CONNECTED:
	fsm_trans_cmd(fsm, ST_CONNECTED, cmd);
	break;
    }
    fsm_step(fsm);
}


static
void fsm_send_data(lcm_fsm_t *fsm,
		   const lcm_cmd_t cmd,
		   const void *data,
		   const unsigned int len)
{
    const lcm_state_t old_state = fsm_get_state(fsm);
    switch (old_state) {
    case ST_IDLE:
    case ST_COMMAND:
	/* Silently ignore the command to send. */
	/* TODO: Would it be better to queue it and send it later? */
	break;
    case ST_CONNECTED:
	fsm_trans_data(fsm, ST_CONNECTED, cmd, data, len);
	break;
    }
    fsm_step(fsm);
}


static lcm_fsm_t lcm_fsm;


static
void fsm_init(void)
{
    lcm_fsm.state       = ST_IDLE;
    lcm_fsm.next_state  = -1;
    lcm_fsm.action_type = ACTION_UNINITIALIZED;
}


static int lcm_receive_check(void)
{
    static u_int8_t rxbuf[32];
    const int readlen = drv_generic_serial_poll((void *)rxbuf, sizeof(rxbuf));
    if (readlen <= 0) {
	debug("%s Received no data", __FUNCTION__);
	return 0;
    }
    debug("%s RECEIVED %d bytes", __FUNCTION__, readlen);
    debug_data(" RX ", rxbuf, readlen);
    return fsm_handle_bytes(&lcm_fsm, rxbuf, readlen);
}


/* Send a command frame to the TCM board */
static
void raw_send_cmd_frame(lcm_cmd_t cmd)
{
    // lcm_receive_check();
    char cmd_buf[3];
    cmd_buf[0] = LCM_FRAME_MASK;
    cmd_buf[1] = cmd;
    cmd_buf[2] = LCM_FRAME_MASK;
    debug("%s sending cmd frame cmd=0x%02x=%s", __FUNCTION__, cmd, cmdstr(cmd));
    debug_data(" TX ", cmd_buf, 3);
    drv_generic_serial_write(cmd_buf, 3);
#if 0
    usleep(100000);
    switch (cmd) {
    case CMD_ACK:
	//case CMD_CONFIRM:
    case CMD_NACK:
	lcm_receive_check();
	break;
    default:
	if (1) {
	    int i;
	    for (i=0; i<20; i++) {
		usleep(100000);
		if (lcm_receive_check()) {
		    break;
		}
	    }
	}
	break;
    }
#endif
}


/* Send a data frame to the TCM board */
static
void raw_send_data_frame(lcm_cmd_t cmd,
			 const char *data, const unsigned int len)
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

    debug_data(" TXD ", frame, fi);
    drv_generic_serial_write(frame, fi);

    usleep(100000);
    lcm_receive_check();

#undef APPEND
}


static
void lcm_send_cmd(lcm_cmd_t cmd)
{
    fsm_send(&lcm_fsm, cmd);
}


/* Initialize the LCM by completing the handshake */
static void drv_TeakLCM_connect()
{
    static u_int8_t rxbuf[32];
    const int readlen = drv_generic_serial_poll((void *)rxbuf, sizeof(rxbuf));
    if (readlen >= 0) {
	debug_data(" initial RX garbage ", rxbuf, readlen);
    }
    fsm_init();
    raw_send_cmd_frame(CMD_RESET);

    while (fsm_get_state(&lcm_fsm) != ST_CONNECTED) {
	usleep(100000);
	lcm_receive_check();
    }
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


/* text mode displays only */
static void drv_TeakLCM_clear(void)
{
    /* do whatever is necessary to clear the display */
    lcm_send_cmd(LCM_CLEAR);
}


/* shadow buffer */
char *shadow;


static void debug_shadow(const char *prefix)
{
    debug_data_int(prefix, shadow, DCOLS*DROWS, 20);
}


/* text mode displays only */
static void drv_TeakLCM_write(const int row, const int col, const char *data, int len)
{
    debug("%s row=%d col=%d len=%d data=\"%s\"", __FUNCTION__,
	  row, col, len, data);

    memcpy(&shadow[DCOLS*row+col], data, len);

    debug_shadow(" shadow ");

    fsm_send_data(&lcm_fsm,
		  (row == 0)?CMD_PRINT1:CMD_PRINT2,
		  &shadow[DCOLS*row], DCOLS);
}


/* start text mode display */
static int drv_TeakLCM_start(const char *section)
{
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
    memset(shadow, ' ', DROWS*DCOLS);

    /* open communication with the display */
    if (drv_TeakLCM_open(section) < 0) {
	return -1;
    }
    debug("%s: %s opened", Name, __FUNCTION__);

    /* reset & initialize display */
    drv_TeakLCM_connect();

    debug("%s: %s connected", Name, __FUNCTION__);

    drv_TeakLCM_clear();	/* clear display */
    lcm_send_cmd(LCM_BACKLIGHT_ON);
    lcm_send_cmd(LCM_DISPLAY_ON);

    debug("%s: %s done", Name, __FUNCTION__);
    return 0;
}


/****************************************/
/***            plugins               ***/
/****************************************/


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

    /* register text widget */
    wc = Widget_Text;
    wc.draw = drv_generic_text_draw;
    widget_register(&wc);

    /* register plugins */

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

    lcm_send_cmd(LCM_DISPLAY_OFF);
    // lcm_send_cmd_frame(LCM_BACKLIGHT_OFF);
    lcm_send_cmd(CMD_DISCONNECT);

    /* consume final ack frame */
    usleep(100000);
    lcm_receive_check();

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
