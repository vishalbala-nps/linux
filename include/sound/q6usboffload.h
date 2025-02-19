/* SPDX-License-Identifier: GPL-2.0
 *
 * sound/q6usboffload.h -- QDSP6 USB offload
 *
 * Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/**
 * struct q6usb_offload - USB backend DAI link offload parameters
 * @dev: dev handle to usb be
 * @domain: allocated iommu domain
 * @sid: streamID for iommu
 * @intr_num: usb interrupter number
 **/
struct q6usb_offload {
	struct device *dev;
	struct iommu_domain *domain;
	long long sid;
	u16 intr_num;
};
