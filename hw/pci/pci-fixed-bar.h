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
#define QEMU_FIXED_BARS_VERSION  1
#define QEMU_FIXED_BARS_MAX_PATH 8

/*
 * On-wire layout (little-endian, packed).  The blob preserves the PCI
 * device discovery order used by EDK2 (host bridges in ascending bus-number
 * order, then root ports, then depth-first traversal of each subordinate
 * bus).  QemuFixedBarsDxe consumes the blob sequentially during PCI
 * enumeration and relies on this ordering to associate blob records with
 * discovered PCI functions.
 *
 *   QemuFixedBarsHdr
 *
 *   For each device (num_devices total):
 *     QemuFixedBarsDevice               -- fixed-size device header
 *     QemuFixedBarsPathEntry[path_len]  -- devfn topology path, root-first
 *     QemuFixedBarsBar[num_bars]        -- one record per BAR
 */

typedef struct {
    uint32_t version;       /* QEMU_FIXED_BARS_VERSION */
    uint32_t num_devices;   /* number of PCI function entries */
    uint32_t num_bars;      /* total BAR records across all devices */
    uint8_t  reserved[4];
    uint64_t window_base;   /* GPA start of the reserved fixed-bar window */
    uint64_t window_size;   /* bytes */
} QEMU_PACKED QemuFixedBarsHdr;     /* 32 bytes */

/* One hop in the PCI topology path (device + function). */
typedef struct {
    uint8_t dev;
    uint8_t fn;
} QEMU_PACKED QemuFixedBarsPathEntry;

/*
 * Per-device header.  Followed immediately by path[path_len] then
 * bar[num_bars] records.
 *
 * vendor_id / device_id: matched against CheckDevice() arguments to detect
 *   enumeration-order mismatches early.
 *
 * path[]: devfn chain from the PXB root bus to the device (root-first).
 *   Bus numbers are not stored because they are assigned by EDK2.
 */
typedef struct {
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t segment;
    uint8_t  path_len;
    uint8_t  num_bars;
} QEMU_PACKED QemuFixedBarsDevice;  /* 8 bytes */

/*
 * Per-BAR record.  address == 0 means the BAR has no host-physical address
 * and firmware must place it within the fixed-bar window.
 */
typedef struct {
    uint8_t  bar;           /* BAR index 0-5 */
    uint8_t  reserved[3];
    uint32_t flags;         /* QEMU_FIXED_BAR_F_* */
    uint64_t address;       /* HPA (== GPA); 0 = to be assigned by firmware */
    uint64_t size;          /* bytes */
} QEMU_PACKED QemuFixedBarsBar;     /* 24 bytes */

/* BAR flag bits */
#define QEMU_FIXED_BAR_F_MEM64  (1u << 0)
#define QEMU_FIXED_BAR_F_PREF   (1u << 1)

bool fixed_bars_collect_and_write(PCIBus     *root_bus,
                                   FWCfgState *fw_cfg,
                                   uint64_t    mmio64_base,
                                   uint64_t    mmio64_size,
                                   Error     **errp);

#endif /* HW_PCI_FIXED_BAR_H */
