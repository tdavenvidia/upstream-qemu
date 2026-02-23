/*
 * Copyright (C) 2026 NVIDIA
 * Written by Tushar Dave
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_VIRT_PCI_ENUMERATE_H
#define HW_ARM_VIRT_PCI_ENUMERATE_H

#include "hw/pci/pci_bus.h"

void virt_pci_enumerate_bus(PCIBus *root_bus);

#endif
