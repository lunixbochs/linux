/*
 * Allwinner sun4i USB phy header
 *
 * Copyright (C) 2015 Hans de Goede <hdego...@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __PHY_SUN4I_USB_H
#define __PHY_SUN4I_USB_H

#include <linux/phy/phy.h>

/**
 * sun4i_usb_phy_update_iscr() - Update sun4i usb phy0 Interface Status and
 * Control bits
 * @phy: Reference to sun4i usb phy0
 * @clr: bits to clear in the ISCR register
 * @set: bits to set in the ISCR register
 */
void sun4i_usb_phy_update_iscr(struct phy *phy, u32 clr, u32 set);

#endif
