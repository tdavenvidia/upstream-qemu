/*
 * Copyright (C) 2025 NVIDIA
 * Written by Tushar Dave
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_VIRT_PCI_RESOURCE_H
#define HW_ARM_VIRT_PCI_RESOURCE_H

#include "hw/arm/virt.h"
#include "exec/hwaddr.h"
#include "hw/pci/pci.h"
#include <glib.h>

#define IORESOURCE_PREFETCH     0x00002000    /* No side effects */

typedef struct {
    uint64_t addr;
    uint64_t end;
    uint64_t flags;
} PhysBAR;

typedef struct {
    uint64_t wbase;
    uint64_t wlimit;
    uint64_t wbase64;
    uint64_t wlimit64;
    uint64_t rbase;
    uint64_t rlimit;
    uint64_t rsize;
    uint64_t piobase;
    bool     available;
    bool     search_mmio64;
    PCIDevice *dev;
    PCIBus *bus;
    /* Allocator window (filled once from vms->memmap) */
    hwaddr   mmio32_base;
    hwaddr   mmio32_size;
    hwaddr   mmio64_base;
    hwaddr   mmio64_size;
} VirtPciAllocCfg;

typedef struct FixedClaim {
    uint64_t start;
    uint64_t end;
    PCIDevice *owner;
    int bar;
} FixedClaim;

typedef struct {
    uint64_t start;
    uint64_t end;
} AddressInterval;

typedef struct {
    PCIDevice *dev;
    int bar_idx;
    uint64_t size;
} BarEntry;

typedef struct {
    int leftmost_hole;      /* Index of hole before first anchor, or -1 */
    GArray *middle_holes;   /* Array of hole indices between anchors */
    int rightmost_hole;     /* Index of hole after last anchor, or -1 */
} CategorizedHoles;

typedef struct {
    hwaddr mmio64_base;
    hwaddr mmio64_size;
    GHashTable *had_fixed; /* set of PCIDevice* that had at least one fixed BAR */
} VirtPciProgramCtx;

void pci_fixed_bar_allocator(VirtMachineState *vms);

#endif
