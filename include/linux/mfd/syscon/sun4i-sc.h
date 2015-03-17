/**
 * sun4i-sc.h - Allwinner sun4i system control register bit definitions
 *
 * Copyright (C) 2014 Chen-Yu Tsai
 *
 * Chen-Yu Tsai  <wens@xxxxxxxx>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_SUN4I_SC_H
#define __LINUX_SUN4I_SC_H

#include <linux/bitops.h>

/* Registers */
#define SUN4I_SC0	0x00
#define SUN4I_SC1	0x04

/* SRAM control register 0 bits */
#define SUN4I_SC0_SRAM_C1_MAP_VE	0x7fffffff

/* SRAM control register 1 bits */
#define SUN4I_SC1_BIST_NDMA_CTRL_SEL	BIT(31)
#define SUN4I_SC1_SRAM_C3_MAP_ISP	BIT(12)
#define SUN4I_SC1_SRAM_C2_MAP_MASK	0x0300
#define SUN4I_SC1_SRAM_C2_MAP_AE	0x0100
#define SUN4I_SC1_SRAM_C2_MAP_CE	0x0200
#define SUN4I_SC1_SRAM_C2_MAP_ACE	0x0300
#define SUN4I_SC1_SRAM_A3_A4_MAP_MASK	0x0030
#define SUN4I_SC1_SRAM_A3_A4_MAP_EMAC	0x0010
#define SUN4I_SC1_SRAM_D_MAP_USB0	BIT(0)

#endif /* __LINUX_SUN4I_SC_H */
