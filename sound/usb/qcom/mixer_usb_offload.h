/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __USB_OFFLOAD_MIXER_H
#define __USB_OFFLOAD_MIXER_H

#if IS_ENABLED(CONFIG_SND_USB_QC_OFFLOAD_MIXER)
int snd_usb_offload_create_ctl(struct snd_usb_audio *chip);
#else
static inline int snd_usb_offload_create_ctl(struct snd_usb_audio *chip)
{
	return 0;
}
#endif
#endif /* __USB_OFFLOAD_MIXER_H */
