/*
 * Internal types shared between pci-fixed-bar-validate.c and
 * pci-fixed-bar-blob.c.  Not part of the public API.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_PCI_FIXED_BAR_PRIV_H
#define HW_PCI_FIXED_BAR_PRIV_H

#include "hw/pci/pci.h"
#include "pci-fixed-bar.h"

/* Validated address, size and type flags for one BAR. */
typedef struct {
    uint8_t  bar;
    uint32_t flags;
    uint64_t addr;
    uint64_t size;
} BarInfo;

/* One PCI function entry to be written to the blob. */
typedef struct {
    uint8_t  rp_bus;     /* primary bus of the root port this device is under */
    uint16_t vendor_id;
    uint16_t device_id;
    bool     is_fixed;
    GArray  *bars;  /* array of BarInfo */
} DeviceInfo;

/*
 * Threaded through the PCI tree walk.
 * Carries BAR validation bounds and accumulates DeviceInfo entries.
 */
typedef struct {
    GArray   *devices;
    PCIBus   *root_bus;
    bool      is_fixed_subtree;
    uint64_t  mmio32_base;
    uint64_t  mmio32_limit;
    uint64_t  mmio64_base;
    uint64_t  mmio64_limit;
} FixedBarsInfo;

bool fixed_bars_validate(FixedBarsInfo *info);

#endif /* HW_PCI_FIXED_BAR_PRIV_H */
