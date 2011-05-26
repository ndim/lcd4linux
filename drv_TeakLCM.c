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

static int Mode = 0;


/*
 * Magic defines
 */

#define CMD_CONNECT         0x05
#define CMD_DISCONNECT      0x06
#define CMD_ALARM           0x07
#define CMD_WRITE           0x08
#define CMD_PRINT1          0x09
#define CMD_PRINT2          0x0A
#define CMD_ACK             0x0B
#define CMD_NACK            0x0C
#define CMD_CONFIRM         0x0D
#define CMD_RESET           0x0E

#define LCM_CLEAR           0x21
#define LCM_HOME            0x22
#define LCM_CURSOR_SHIFT_R  0x23
#define LCM_CURSOR_SHIFT_L  0x24
#define LCM_BACKLIGHT_ON    0x25
#define LCM_BACKLIGHT_OFF   0x26
#define LCM_LINE2           0x27
#define LCM_DISPLAY_SHIFT_R 0x28
#define LCM_DISPLAY_SHIFT_L 0x29
#define LCM_CURSOR_ON       0x2A
#define LCM_CURSOR_OFF      0x2B
#define LCM_CURSOR_BLINK    0x2C
#define LCM_DISPLAY_ON      0x2D
#define LCM_DISPLAY_OFF     0x2E

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

/* Send a command frame to the board */
static void drv_TeakLCM_send_cmd_frame(unsigned char cmd)
{
    char cmd_buf[3];
    cmd_buf[0] = LCM_FRAME_MASK;
    cmd_buf[1] = cmd;
    cmd_buf[2] = LCM_FRAME_MASK;
    drv_generic_serial_write(cmd_buf, 3);
}


/* Initialize the LCM by completing the handshake */
static void drv_TeakLCM_connect()
{
    Mode = 0;
    char buffer[3];
    drv_TeakLCM_send_cmd_frame(CMD_RESET);

    if ((drv_generic_serial_read(buffer, 3) != 3)
	|| (buffer[0] != LCM_FRAME_MASK)
	|| (buffer[2] != LCM_FRAME_MASK)
	) {
	error("%s: Error during handshake", Name);
    }
    if (buffer[1] == CMD_CONNECT) {
	Mode = 1;
	drv_TeakLCM_send_cmd_frame(CMD_ACK);
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

/* dummy function that sends something to the display */
static void drv_TeakLCM_send(const char *data, const unsigned int len)
{
    /* send data to the serial port is easy... */
    drv_generic_serial_write(data, len);

}


/* text mode displays only */
static void drv_TeakLCM_clear(void)
{
    char cmd[1];

    /* do whatever is necessary to clear the display */
    /* assume 0x01 to be a 'clear display' command */
    cmd[0] = 0x01;
    drv_TeakLCM_send(cmd, 1);
}


/* text mode displays only */
static void drv_TeakLCM_write(const int row, const int col, const char *data, int len)
{
    char cmd[3];

    /* do the cursor positioning here */
    /* assume 0x02 to be a 'Goto' command */
    cmd[0] = 0x02;
    cmd[1] = row;
    cmd[2] = col;
    drv_TeakLCM_send(cmd, 3);

    /* send string to the display */
    drv_TeakLCM_send(data, len);

}

/* text mode displays only */
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


/* example function used in a plugin */
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


/* start text mode display */
static int drv_TeakLCM_start(const char *section)
{
    int contrast;
    int rows = -1, cols = -1;
    char *s;
    char cmd[1];

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

    /* open communication with the display */
    if (drv_TeakLCM_open(section) < 0) {
	return -1;
    }

    /* reset & initialize display */
    /* assume 0x00 to be a 'reset' command */
    cmd[0] = 0x00;
    drv_TeakLCM_send(cmd, 0);

    if (cfg_number(section, "Contrast", 0, 0, 255, &contrast) > 0) {
	drv_TeakLCM_contrast(contrast);
    }

    drv_TeakLCM_clear();	/* clear display */

    return 0;
}


/****************************************/
/***            plugins               ***/
/****************************************/

static void plugin_contrast(RESULT * result, RESULT * arg1)
{
    double contrast;

    contrast = drv_TeakLCM_contrast(R2N(arg1));
    SetResult(&result, R_NUMBER, &contrast);
}


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
    CHARS = 8;			/* number of user-defineable characters */
    CHAR0 = 0;			/* ASCII of first user-defineable char */
    GOTO_COST = 2;		/* number of bytes a goto command requires */

    /* real worker functions */
    drv_generic_text_real_write = drv_TeakLCM_write;
    drv_generic_text_real_defchar = drv_TeakLCM_defchar;


    /* start display */
    if ((ret = drv_TeakLCM_start(section)) != 0)
	return ret;

    if (!quiet) {
	char buffer[40];
	qprintf(buffer, sizeof(buffer), "%s %dx%d", Name, DCOLS, DROWS);
	if (drv_generic_text_greet(buffer, "www.bwct.de")) {
	    sleep(3);
	    drv_TeakLCM_clear();
	}
    }

    /* initialize generic text driver */
    if ((ret = drv_generic_text_init(section, Name)) != 0)
	return ret;

    /* initialize generic icon driver */
    if ((ret = drv_generic_text_icon_init()) != 0)
	return ret;

    /* initialize generic bar driver */
    if ((ret = drv_generic_text_bar_init(0)) != 0)
	return ret;

    /* add fixed chars to the bar driver */
    drv_generic_text_bar_add_segment(0, 0, 255, 32);	/* ASCII  32 = blank */

    /* register text widget */
    wc = Widget_Text;
    wc.draw = drv_generic_text_draw;
    widget_register(&wc);

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
