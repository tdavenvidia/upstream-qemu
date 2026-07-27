/*
 * Fixed-BAR GPA=HPA support — QEMU side declarations.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_PCI_FIXED_BAR_H
#define HW_PCI_FIXED_BAR_H

#include "hw/nvram/fw_cfg.h"
#include "hw/pci/pci_bus.h"

#define FW_CFG_FIXED_BARS        "etc/fixed-bars"
#define QEMU_FIXED_BARS_VERSION  2
#define QEMU_FIXED_BARS_MAX_PATH 8

/*
 * On-wire layout (little-endian, packed):
 *   QemuFixedBarsHdr
 *   QemuFixedBarsEntry[num_entries]   -- one entry per (device, BAR)
 */

typedef struct {
    uint32_t version;       /* QEMU_FIXED_BARS_VERSION */
    uint32_t num_entries;
    uint64_t window_base;   /* GPA start of the reserved fixed-bar window */
    uint64_t window_size;   /* bytes */
    uint8_t  reserved[8];
} QEMU_PACKED QemuFixedBarsHdr;

/* One hop in the PCI topology path (device + function). */
typedef struct {
    uint8_t dev;
    uint8_t fn;
} QEMU_PACKED QemuFixedBarsPathEntry;

/*
 * One fixed-BAR entry — describes a single BAR of a single device.
 *
 * vendor_id / device_id: PCI IDs read from device config space; used by
 *   EDK2's EFI_INCOMPATIBLE_PCI_DEVICE_SUPPORT_PROTOCOL.CheckDevice() to
 *   match this entry to the correct device during enumeration.
 *
 * path[0..path_len-1]: devfn chain from the PXB root bridge down to the
 *   device (root-first order).  Bus numbers are NOT stored because they are
 *   assigned by EDK2 and are unknown at blob-write time.  Carried for
 *   future location-based matching.
 *
 * address == 0 means the BAR is emulated and may be placed anywhere.
 */
typedef struct {
    uint16_t               segment;
    uint8_t                path_len;
    uint8_t                bar;        /* BAR index 0-5 */
    uint32_t               flags;      /* QEMU_FIXED_BAR_F_* */
    uint64_t               address;    /* HPA (== GPA); 0 = emulated */
    uint64_t               size;       /* bytes */
    uint16_t               vendor_id;  /* PCI vendor ID */
    uint16_t               device_id;  /* PCI device ID */
    uint8_t                reserved[4];
    QemuFixedBarsPathEntry path[QEMU_FIXED_BARS_MAX_PATH];
} QEMU_PACKED QemuFixedBarsEntry;    /* 48 bytes */

/* BAR flag bits */
#define QEMU_FIXED_BAR_F_MEM64  (1u << 0)
#define QEMU_FIXED_BAR_F_PREF   (1u << 1)

bool fixed_bars_collect_and_write(PCIBus     *root_bus,
                                   FWCfgState *fw_cfg,
                                   uint64_t    mmio64_base,
                                   uint64_t    mmio64_size,
                                   Error     **errp);

#endif /* HW_PCI_FIXED_BAR_H */
