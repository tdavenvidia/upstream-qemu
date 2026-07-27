/*
 * Fixed-BAR GPA=HPA support — QEMU side implementation.
 *
 * Walk the PCI topology at machine_done time, find every pcie-root-port
 * with fixed-bar=on, capture host-physical BAR addresses from vfio config
 * space (stored in VFIOPCIDevice.bar_hpa[]), and write the etc/fixed-bars
 * fw_cfg blob that OVMF will consume.
 *
 * Each BAR of each fixed device produces one QemuFixedBarsEntry carrying
 * the topology path (devfn chain, root-first) instead of a bus number.
 * Bus numbers are not stored because they are assigned by EDK2.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pcie_port.h"
#include "hw/pci/pci_host.h"
#include "hw/vfio/pci.h"
#include "hw/nvram/fw_cfg.h"
#include "qemu/error-report.h"
#include "pci-internal.h"
#include "pci-fixed-bar.h"

/* One fixed-BAR entry collected during tree walk. */
typedef struct {
    QemuFixedBarsPathEntry path[QEMU_FIXED_BARS_MAX_PATH];
    int      path_len;
    uint16_t segment;
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t  bar;
    uint32_t flags;
    uint64_t hpa;
    uint64_t size;
} EntryInfo;

/* Accumulated state across a fixed-bar sub-tree. */
typedef struct {
    GArray  *entries;     /* array of EntryInfo */
    PCIBus  *root_bus;    /* PXB root bus — path is relative to this */
    uint64_t real_min;    /* lowest real HPA seen (hpa != 0) */
    uint64_t real_max;    /* highest (real HPA + size) seen */
    uint64_t emul_sum;    /* sum of emulated BAR sizes */
} SubtreeInfo;

/*
 * Build the devfn topology path from @pdev up to (but not crossing)
 * @root_bus.  The path is stored root-first in @path[].
 * Returns the path length, or -1 if the topology is deeper than
 * QEMU_FIXED_BARS_MAX_PATH hops.
 */
static int build_devfn_path(PCIDevice *pdev, PCIBus *root_bus,
                             QemuFixedBarsPathEntry *path)
{
    QemuFixedBarsPathEntry rev[QEMU_FIXED_BARS_MAX_PATH];
    PCIDevice *cur = pdev;
    int depth = 0;

    while (cur != NULL) {
        if (depth >= QEMU_FIXED_BARS_MAX_PATH) {
            return -1;
        }
        rev[depth].dev = PCI_SLOT(cur->devfn);
        rev[depth].fn  = PCI_FUNC(cur->devfn);
        depth++;

        if (pci_get_bus(cur) == root_bus) {
            break;
        }
        cur = pci_get_bus(cur)->parent_dev;
    }

    /* Reverse: root-first order. */
    for (int i = 0; i < depth; i++) {
        path[i] = rev[depth - 1 - i];
    }
    return depth;
}

static void collect_device(PCIBus *bus, PCIDevice *pdev, void *opaque)
{
    SubtreeInfo *info = opaque;
    VFIOPCIDevice *vdev;
    EntryInfo e;
    int i, path_len;

    if (!object_dynamic_cast(OBJECT(pdev), TYPE_VFIO_PCI_DEVICE)) {
        return;
    }
    vdev = VFIO_PCI_DEVICE(pdev);

    /* Compute path once to check depth before processing BARs. */
    path_len = build_devfn_path(pdev, info->root_bus, e.path);
    if (path_len < 0) {
        error_report("fixed-bars: topology too deep for device %s, skipping",
                     object_get_typename(OBJECT(pdev)));
        return;
    }

    for (i = 0; i < PCI_NUM_REGIONS - 1; i++) {
        VFIOBAR *bar = &vdev->bars[i];

        if (!bar->size) {
            continue;
        }

        memset(&e, 0, sizeof(e));
        /* Rebuild path into the fresh entry struct. */
        path_len = build_devfn_path(pdev, info->root_bus, e.path);
        e.path_len = path_len;
        e.segment  = 0;
        e.bar      = i;
        e.size     = bar->size;
        e.hpa      = vdev->bar_hpa[i];
        e.vendor_id = pci_get_word(pdev->config + PCI_VENDOR_ID);
        e.device_id = pci_get_word(pdev->config + PCI_DEVICE_ID);
        e.flags    = 0;
        if (bar->mem64) {
            e.flags |= QEMU_FIXED_BAR_F_MEM64;
        }
        if (bar->type & PCI_BASE_ADDRESS_MEM_PREFETCH) {
            e.flags |= QEMU_FIXED_BAR_F_PREF;
        }

        if (e.hpa != 0) {
            uint64_t end;

            end = e.hpa + bar->size;
            if (e.hpa < info->real_min) {
                info->real_min = e.hpa;
            }
            if (end > info->real_max) {
                info->real_max = end;
            }
        } else {
            info->emul_sum += bar->size;
        }

        error_report("fixed-bars:   BAR%d path_len=%d hpa=0x%"PRIx64
                     " size=0x%"PRIx64" flags=0x%x",
                     i, path_len, e.hpa, e.size, e.flags);
        g_array_append_val(info->entries, e);
    }
}

static void collect_fixed_subtree(PCIBus *bus, SubtreeInfo *info)
{
    PCIBus *child;

    pci_for_each_device_under_bus(bus, collect_device, info);

    QLIST_FOREACH(child, &bus->child, sibling) {
        collect_fixed_subtree(child, info);
    }
}

bool fixed_bars_collect_and_write(PCIBus     *root_bus G_GNUC_UNUSED,
                                   FWCfgState *fw_cfg,
                                   uint64_t    mmio64_base,
                                   uint64_t    mmio64_size,
                                   Error     **errp)
{
    SubtreeInfo  info;
    bool         found = false;
    uint64_t     gran, wbase, wsize;
    guint        nentries;
    size_t       blob_size;
    uint8_t     *blob;
    QemuFixedBarsHdr   *hdr;
    QemuFixedBarsEntry *eout;

    info.entries  = g_array_new(false, true, sizeof(EntryInfo));
    info.root_bus = NULL;
    info.real_min = UINT64_MAX;
    info.real_max = 0;
    info.emul_sum = 0;

    {
        PCIHostState *hb;

        QLIST_FOREACH(hb, &pci_host_bridges, next) {
            PCIBus *rbus = hb->bus;
            int devfn;

            if (!rbus) {
                continue;
            }

            for (devfn = 0; devfn < PCI_DEVFN_MAX; devfn++) {
                PCIDevice *pdev = rbus->devices[devfn];
                PCIESlot  *slot;
                PCIBus    *sec;

                if (!pdev) {
                    continue;
                }
                if (!object_dynamic_cast(OBJECT(pdev), TYPE_PCIE_ROOT_PORT)) {
                    continue;
                }
                slot = PCIE_SLOT(pdev);
                if (!slot->fixed_bar) {
                    continue;
                }
                sec = pci_bridge_get_sec_bus(PCI_BRIDGE(pdev));
                if (!sec) {
                    continue;
                }

                error_report("fixed-bars: collecting sub-tree under root port "
                             "%02x.%x on bus %s",
                             PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn),
                             BUS(rbus)->name);

                info.root_bus = rbus;
                collect_fixed_subtree(sec, &info);
                found = true;
            }
        }
    }

    if (!found || info.entries->len == 0) {
        error_report("fixed-bars: no fixed-bar devices found, "
                     "not writing etc/fixed-bars");
        g_array_free(info.entries, true);
        return true;
    }

    error_report("fixed-bars: found %u BAR entries, "
                 "real_min=0x%"PRIx64" real_max=0x%"PRIx64
                 " emul_sum=0x%"PRIx64,
                 info.entries->len, info.real_min,
                 info.real_max, info.emul_sum);

    if (info.real_min == UINT64_MAX) {
        info.real_min = mmio64_base;
        info.real_max = mmio64_base;
    }

    gran  = 1ULL << 20;
    wbase = info.real_min & ~(gran - 1);
    wsize = ROUND_UP(info.real_max - wbase + info.emul_sum, gran);

    if (wbase < mmio64_base || wbase + wsize > mmio64_base + mmio64_size) {
        error_setg(errp,
                   "fixed-bar window [0x%"PRIx64"+0x%"PRIx64"] "
                   "outside mmio64 [0x%"PRIx64"+0x%"PRIx64"]; "
                   "set pcie-mmio-window to cover the GPU HPAs",
                   wbase, wsize, mmio64_base, mmio64_size);
        g_array_free(info.entries, true);
        return false;
    }

    nentries  = info.entries->len;
    blob_size = sizeof(QemuFixedBarsHdr) +
                nentries * sizeof(QemuFixedBarsEntry);
    blob      = g_malloc0(blob_size);
    hdr       = (QemuFixedBarsHdr *)blob;
    eout      = (QemuFixedBarsEntry *)(blob + sizeof(*hdr));

    hdr->version     = cpu_to_le32(QEMU_FIXED_BARS_VERSION);
    hdr->num_entries = cpu_to_le32(nentries);
    hdr->window_base = cpu_to_le64(wbase);
    hdr->window_size = cpu_to_le64(wsize);

    for (guint i = 0; i < nentries; i++) {
        EntryInfo *ei = &g_array_index(info.entries, EntryInfo, i);

        eout[i].segment   = cpu_to_le16(ei->segment);
        eout[i].path_len  = ei->path_len;
        eout[i].bar       = ei->bar;
        eout[i].flags     = cpu_to_le32(ei->flags);
        eout[i].address   = cpu_to_le64(ei->hpa);
        eout[i].size      = cpu_to_le64(ei->size);
        eout[i].vendor_id = cpu_to_le16(ei->vendor_id);
        eout[i].device_id = cpu_to_le16(ei->device_id);
        memcpy(eout[i].path, ei->path,
               ei->path_len * sizeof(QemuFixedBarsPathEntry));
    }

    error_report("fixed-bars: writing etc/fixed-bars blob: "
                 "window=0x%"PRIx64"+0x%"PRIx64" %u BAR entries",
                 wbase, wsize, nentries);
    fw_cfg_add_file(fw_cfg, FW_CFG_FIXED_BARS, blob, blob_size);

    g_array_free(info.entries, TRUE);
    return true;
}
