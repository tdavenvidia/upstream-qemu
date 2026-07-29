/*
 * Produce the etc/fixed-bars fw_cfg blob for GPA=HPA BAR placement.
 *
 * The blob contains one device entry per PCI function, grouped by root port.
 * Host bridges are processed in ascending bus-number order so that the
 * entry sequence matches the PCIe enumeration order used by EDK2.  This
 * ordering allows firmware to identify each device by its position in the
 * blob rather than by location information not available through
 * EFI_INCOMPATIBLE_PCI_DEVICE_SUPPORT_PROTOCOL.
 *
 * Each entry carries the devfn topology path (root-first) instead of a
 * bus number because bus numbers are assigned by EDK2, not by QEMU.
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

/* Internal per-BAR data collected during tree walk. */
typedef struct {
    uint8_t  bar;
    uint32_t flags;
    uint64_t hpa;
    uint64_t size;
} BarInfo;

/* Internal per-device data collected during tree walk. */
typedef struct {
    QemuFixedBarsPathEntry path[QEMU_FIXED_BARS_MAX_PATH];
    int      path_len;
    uint16_t segment;
    uint16_t vendor_id;
    uint16_t device_id;
    GArray  *bars;   /* array of BarInfo */
} DeviceInfo;

/* Accumulated state across a fixed-bar sub-tree. */
typedef struct {
    GArray  *devices;  /* array of DeviceInfo */
    PCIBus  *root_bus; /* PXB root bus — path is relative to this */
    uint64_t real_min; /* lowest real HPA seen (hpa != 0) */
    uint64_t real_max; /* highest (real HPA + size) seen */
    uint64_t emul_sum; /* sum of BAR sizes with hpa == 0 */
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

static void collect_fixed_device(PCIBus *bus, PCIDevice *pdev, void *opaque)
{
    SubtreeInfo *info = opaque;
    VFIOPCIDevice *vdev;
    DeviceInfo dev;
    BarInfo bar_info;
    int i, path_len;

    if (!object_dynamic_cast(OBJECT(pdev), TYPE_VFIO_PCI_DEVICE)) {
        return;
    }
    vdev = VFIO_PCI_DEVICE(pdev);

    memset(&dev, 0, sizeof(dev));
    path_len = build_devfn_path(pdev, info->root_bus, dev.path);
    if (path_len < 0) {
        error_report("fixed-bars: topology too deep for device %s, skipping",
                     object_get_typename(OBJECT(pdev)));
        return;
    }

    dev.path_len  = path_len;
    dev.segment   = 0;
    dev.vendor_id = pci_get_word(pdev->config + PCI_VENDOR_ID);
    dev.device_id = pci_get_word(pdev->config + PCI_DEVICE_ID);
    dev.bars      = g_array_new(false, true, sizeof(BarInfo));

    for (i = 0; i < PCI_NUM_REGIONS - 1; i++) {
        VFIOBAR *bar = &vdev->bars[i];

        if (!bar->size) {
            continue;
        }

        memset(&bar_info, 0, sizeof(bar_info));
        bar_info.bar   = i;
        bar_info.hpa   = vdev->bar_hpa[i];
        bar_info.size  = bar->size;
        bar_info.flags = 0;
        if (bar->mem64) {
            bar_info.flags |= QEMU_FIXED_BAR_F_MEM64;
        }
        if (bar->type & PCI_BASE_ADDRESS_MEM_PREFETCH) {
            bar_info.flags |= QEMU_FIXED_BAR_F_PREF;
        }

        if (bar_info.hpa != 0) {
            uint64_t end = bar_info.hpa + bar->size;
            if (bar_info.hpa < info->real_min) {
                info->real_min = bar_info.hpa;
            }
            if (end > info->real_max) {
                info->real_max = end;
            }
        } else {
            info->emul_sum += bar->size;
        }

        error_report("fixed-bars:   BAR%d hpa=0x%"PRIx64
                     " size=0x%"PRIx64" flags=0x%x",
                     i, bar_info.hpa, bar_info.size, bar_info.flags);
        g_array_append_val(dev.bars, bar_info);
    }

    if (dev.bars->len > 0) {
        g_array_append_val(info->devices, dev);
    } else {
        g_array_free(dev.bars, TRUE);
    }
}

static void collect_fixed_subtree(PCIBus *bus, SubtreeInfo *info)
{
    PCIBus *child;

    pci_for_each_device_under_bus(bus, collect_fixed_device, info);

    QLIST_FOREACH(child, &bus->child, sibling) {
        collect_fixed_subtree(child, info);
    }
}

static gint
pci_host_compare_bus_num(gconstpointer a, gconstpointer b)
{
    PCIHostState *ha = *(PCIHostState **)a;
    PCIHostState *hb = *(PCIHostState **)b;
    return (gint)pci_bus_num(ha->bus) - (gint)pci_bus_num(hb->bus);
}

bool fixed_bars_collect_and_write(PCIBus     *root_bus G_GNUC_UNUSED,
                                   FWCfgState *fw_cfg,
                                   uint64_t    mmio64_base,
                                   uint64_t    mmio64_size,
                                   Error     **errp)
{
    SubtreeInfo  info;
    uint64_t     gran, wbase, wsize;
    size_t       blob_size;
    guint        total_bars;
    uint8_t     *blob;
    uint8_t     *ptr;
    QemuFixedBarsHdr *hdr;
    guint        d, b;
    bool         ret = true;

    info.devices  = g_array_new(false, true, sizeof(DeviceInfo));
    info.root_bus = NULL;
    info.real_min = UINT64_MAX;
    info.real_max = 0;
    info.emul_sum = 0;

    {
        PCIHostState *hb;
        GPtrArray    *hb_sorted;
        guint         i;

        /* Prepare blob entries in the same order as EDK2 PCI enumeration. */
        hb_sorted = g_ptr_array_new();
        QLIST_FOREACH(hb, &pci_host_bridges, next) {
            if (hb->bus) {
                g_ptr_array_add(hb_sorted, hb);
            }
        }
        g_ptr_array_sort(hb_sorted, pci_host_compare_bus_num);

        for (i = 0; i < hb_sorted->len; i++) {
            PCIBus *rbus;
            int devfn;

            hb   = g_ptr_array_index(hb_sorted, i);
            rbus = hb->bus;

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
            }
        }
        g_ptr_array_free(hb_sorted, TRUE);
    }

    if (info.devices->len == 0) {
        error_report("fixed-bars: no fixed-bar devices found, "
                     "not writing etc/fixed-bars");
        g_array_free(info.devices, TRUE);
        return true;
    }

    /* Count total BARs across all devices. */
    total_bars = 0;
    for (d = 0; d < info.devices->len; d++) {
        DeviceInfo *dev = &g_array_index(info.devices, DeviceInfo, d);
        total_bars += dev->bars->len;
    }

    error_report("fixed-bars: found %u device(s) %u BAR(s), "
                 "real_min=0x%"PRIx64" real_max=0x%"PRIx64
                 " emul_sum=0x%"PRIx64,
                 info.devices->len, total_bars,
                 info.real_min, info.real_max, info.emul_sum);

    if (info.real_min == UINT64_MAX) {
        info.real_min = mmio64_base;
        info.real_max = mmio64_base;
    }

    gran  = 1ULL << 20;
    wbase = info.real_min & ~(gran - 1);
    wsize = ROUND_UP(info.real_max - wbase + info.emul_sum, gran);

    if (wbase < mmio64_base ||
        wsize > mmio64_size ||
        wbase - mmio64_base > mmio64_size - wsize) {
        error_setg(errp,
                   "fixed-bar window [0x%"PRIx64"+0x%"PRIx64"] "
                   "outside mmio64 [0x%"PRIx64"+0x%"PRIx64"]; "
                   "set pcie-mmio-window to cover the device HPAs",
                   wbase, wsize, mmio64_base, mmio64_size);
        ret = false;
        goto out;
    }

    /* Compute blob size: header + per-device headers + paths + BAR records. */
    blob_size = sizeof(QemuFixedBarsHdr);
    for (d = 0; d < info.devices->len; d++) {
        DeviceInfo *dev = &g_array_index(info.devices, DeviceInfo, d);
        blob_size += sizeof(QemuFixedBarsDevice);
        blob_size += dev->path_len * sizeof(QemuFixedBarsPathEntry);
        blob_size += dev->bars->len * sizeof(QemuFixedBarsBar);
    }

    blob = g_malloc0(blob_size);
    hdr  = (QemuFixedBarsHdr *)blob;

    hdr->version     = cpu_to_le32(QEMU_FIXED_BARS_VERSION);
    hdr->num_devices = cpu_to_le32(info.devices->len);
    hdr->num_bars    = cpu_to_le32(total_bars);
    hdr->window_base = cpu_to_le64(wbase);
    hdr->window_size = cpu_to_le64(wsize);

    ptr = blob + sizeof(*hdr);

    for (d = 0; d < info.devices->len; d++) {
        DeviceInfo *dev = &g_array_index(info.devices, DeviceInfo, d);
        QemuFixedBarsDevice *dev_out = (QemuFixedBarsDevice *)ptr;

        dev_out->vendor_id = cpu_to_le16(dev->vendor_id);
        dev_out->device_id = cpu_to_le16(dev->device_id);
        dev_out->segment   = cpu_to_le16(dev->segment);
        dev_out->path_len  = dev->path_len;
        dev_out->num_bars  = dev->bars->len;
        ptr += sizeof(QemuFixedBarsDevice);

        memcpy(ptr, dev->path,
               dev->path_len * sizeof(QemuFixedBarsPathEntry));
        ptr += dev->path_len * sizeof(QemuFixedBarsPathEntry);

        for (b = 0; b < dev->bars->len; b++) {
            BarInfo *bi = &g_array_index(dev->bars, BarInfo, b);
            QemuFixedBarsBar *bar_out = (QemuFixedBarsBar *)ptr;

            bar_out->bar     = bi->bar;
            bar_out->flags   = cpu_to_le32(bi->flags);
            bar_out->address = cpu_to_le64(bi->hpa);
            bar_out->size    = cpu_to_le64(bi->size);
            ptr += sizeof(QemuFixedBarsBar);
        }
    }

    error_report("fixed-bars: writing etc/fixed-bars blob: "
                 "window=0x%"PRIx64"+0x%"PRIx64
                 " %u device(s) %u BAR(s)",
                 wbase, wsize, info.devices->len, total_bars);
    fw_cfg_add_file(fw_cfg, FW_CFG_FIXED_BARS, blob, blob_size);

out:
    for (d = 0; d < info.devices->len; d++) {
        DeviceInfo *dev = &g_array_index(info.devices, DeviceInfo, d);
        g_array_free(dev->bars, TRUE);
    }
    g_array_free(info.devices, TRUE);
    return ret;
}
