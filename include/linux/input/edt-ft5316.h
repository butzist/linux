#ifndef _EDT_FT5316_H
#define _EDT_FT5316_H

/*
 * Copyright (c) 2014 bbv Software Services, Zurich, Switzerland, <adam.szalkowski@bbv.ch>
 *
 * based on
 * Copyright (c) 2012 Simon Budig, <simon.budig@kernelconcepts.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

struct edt_ft5316_platform_data {
	int irq_pin;
	int reset_pin;
	u32 max_x;
	u32 max_y;
};

#endif /* _EDT_FT5316_H */
