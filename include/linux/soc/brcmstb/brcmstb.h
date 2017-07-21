#ifndef __BRCMSTB_SOC_H
#define __BRCMSTB_SOC_H

#define BRCM_ID(reg)	((u32)reg >> 28 ? (u32)reg >> 16 : (u32)reg >> 8)
#define BRCM_REV(reg)	((u32)reg & 0xff)

/*
 * Bus Interface Unit control register setup, must happen early during boot,
 * before SMP is brought up, called by machine entry point.
 */
void brcmstb_biuctrl_init(void);

/*
 * Helper functions for getting family or product id from the
 * SoC driver.
 */
u32 brcmstb_get_family_id(void);
u32 brcmstb_get_product_id(void);

#endif /* __BRCMSTB_SOC_H */
