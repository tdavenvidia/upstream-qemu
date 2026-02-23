/*
 * Copyright (C) 2026 NVIDIA
 * Written by Tushar Dave
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_bus.h"
#include "hw/pci/pci-enumerate.h"

/* Forward declaration */
static uint8_t pci_program_bus_numbers(PCIBus *bus, uint8_t current_bus_num,
                                       uint8_t *next_bus_num);

/* Bridge device and child bus for programming primary/secondary/subordinate */
typedef struct {
    PCIDevice *dev;
    PCIBus *child_bus;
} PciBridgePair;

static int cmp_bridge_pairs_by_devfn(gconstpointer a, gconstpointer b)
{
    const PciBridgePair *pa = (const PciBridgePair *)a;
    const PciBridgePair *pb = (const PciBridgePair *)b;
    return (int)pa->dev->devfn - (int)pb->dev->devfn;
}

static int cmp_bus_by_num(gconstpointer a, gconstpointer b)
{
    PCIBus *ba = *(PCIBus * const *)a;
    PCIBus *bb = *(PCIBus * const *)b;
    return pci_bus_num(ba) - pci_bus_num(bb);
}

/*
 * Program one bridge's primary, secondary and subordinate bus numbers
 * and recurse. Return the max subordinate bus number.
 */
static uint8_t pci_program_bridge(PCIBus *bus, uint8_t current_bus_num,
                                  PCIDevice *bridge, PCIBus *child_bus,
                                  uint8_t *next_bus_num)
{
    uint8_t secondary, max_child, max_sub = current_bus_num;

    if (*next_bus_num == 0) {
        return max_sub;
    }
    secondary = *next_bus_num;
    (*next_bus_num)++;

    pci_default_write_config(bridge, PCI_PRIMARY_BUS, current_bus_num, 1);
    pci_default_write_config(bridge, PCI_SECONDARY_BUS, secondary, 1);
    pci_default_write_config(bridge, PCI_SUBORDINATE_BUS, secondary, 1);

    max_child = pci_program_bus_numbers(child_bus, secondary, next_bus_num);
    pci_default_write_config(bridge, PCI_SUBORDINATE_BUS, max_child, 1);
    if (max_child > max_sub) {
        max_sub = max_child;
    }
    return max_sub;
}

/*
 * Program bus numbers for this bus and all subordinates.
 * - current_bus_num: this bus' number (0 for root, or already set for PXB).
 * - next_bus_num: next free bus number to assign to a bridge.
 *
 * Children come from bus->child only. Two kinds:
 * 1) PXB (extra root): child has PCI_BUS_IS_ROOT. Has bus number
 *    already set, recurse only.
 * 2) Normal bridge: parent is IS_PCI_BRIDGE. Assign secondary = *next_bus_num,
 *    program primary, secondary and subordinate bus numbers, and recurse.
 *
 * Order (match EDK2): process PXB children first (sorted by bus number), then
 * bridges (sorted by devfn).
 */
static uint8_t pci_program_bus_numbers(PCIBus *bus, uint8_t current_bus_num,
                                       uint8_t *next_bus_num)
{
    uint8_t max_subordinate = current_bus_num;
    uint8_t child_num;
    uint8_t one_max;
    PCIDevice *parent;
    PCIBus *child_bus;
    GArray *pxb_buses = g_array_new(FALSE, FALSE, sizeof(PCIBus *));
    GArray *bridge_pairs = g_array_new(FALSE, FALSE, sizeof(PciBridgePair));
    PciBridgePair pair;
    guint i;

    /* Single pass over bus->child: split into PXB vs bridge */
    QLIST_FOREACH(child_bus, &bus->child, sibling) {
        parent = child_bus->parent_dev;
        if (!parent) {
            continue;
        }
        if (pci_bus_is_root(child_bus)) {
            /* PXB or similar: bus number already set (e.g. bus_nr=1, 9) */
            g_array_append_val(pxb_buses, child_bus);
        } else if (IS_PCI_BRIDGE(parent)) {
            pair.dev = parent;
            pair.child_bus = child_bus;
            g_array_append_val(bridge_pairs, pair);
        }
    }

    /* PXB first, sorted by bus number (e.g. 1 before 9) */
    if (pxb_buses->len > 1) {
        g_array_sort(pxb_buses, cmp_bus_by_num);
    }
    for (i = 0; i < pxb_buses->len; i++) {
        child_bus = g_array_index(pxb_buses, PCIBus *, i);
        child_num = (uint8_t)pci_bus_num(child_bus);
        if (child_num + 1 > *next_bus_num) {
            *next_bus_num = child_num + 1;
        }
        one_max = pci_program_bus_numbers(child_bus, child_num, next_bus_num);
        if (one_max > max_subordinate) {
            max_subordinate = one_max;
        }
    }
    g_array_free(pxb_buses, TRUE);

    /* Bridges second, sorted by devfn */
    if (bridge_pairs->len > 1) {
        g_array_sort(bridge_pairs, cmp_bridge_pairs_by_devfn);
    }
    for (i = 0; i < bridge_pairs->len; i++) {
        pair = g_array_index(bridge_pairs, PciBridgePair, i);
        one_max = pci_program_bridge(bus, current_bus_num,
                                                  pair.dev, pair.child_bus,
                                                  next_bus_num);
        if (one_max > max_subordinate) {
            max_subordinate = one_max;
        }
    }
    g_array_free(bridge_pairs, TRUE);

    return max_subordinate;
}

void pci_enumerate_bus(PCIBus *root_bus)
{
    uint8_t next_bus_num;

    if (!root_bus) {
        return;
    }
    next_bus_num = 1;
    pci_program_bus_numbers(root_bus, 0, &next_bus_num);
}
