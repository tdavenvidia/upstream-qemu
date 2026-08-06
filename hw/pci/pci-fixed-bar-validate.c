/*
 * BAR address validation for fixed-BAR placement.
 *
 * Walks the PCI tree at machine_done and enforces:
 *   - pci-bars= mandatory under a fixed-bar root port
 *   - every memory BAR covered (completeness)
 *   - address aligned to BAR size
 *   - address within machine MMIO window
 *   - no overlap between any two fixed BAR address ranges
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pcie_port.h"
#include "hw/pci/pci_host.h"
#include "qemu/error-report.h"
#include "qemu/range.h"
#include "pci-internal.h"
#include "pci-fixed-bar-priv.h"

/*
 * Claimed BAR ranges — detect inter-device and inter-hierarchy overlaps
 * across all fixed BARs system-wide.
 */

typedef struct {
    uint64_t    start;
    uint64_t    end;
    const char *owner;  /* device name, for error messages */
    int         bar;
} FixedClaim;

static GArray *fixed_claims;

static void fixed_claims_init(void)
{
    if (fixed_claims) {
        g_array_free(fixed_claims, true);
    }
    fixed_claims = g_array_new(false, true, sizeof(FixedClaim));
}

static void fixed_claims_free(void)
{
    g_array_free(fixed_claims, true);
    fixed_claims = NULL;
}

static bool fixed_claims_overlap(uint64_t start, uint64_t end,
                                  const char **owner_out, int *bar_out)
{
    FixedClaim *c;
    guint i;

    for (i = 0; i < fixed_claims->len; i++) {
        c = &g_array_index(fixed_claims, FixedClaim, i);
        if (ranges_overlap(start, end - start + 1,
                           c->start, c->end - c->start + 1)) {
            *owner_out = c->owner;
            *bar_out   = c->bar;
            return true;
        }
    }
    return false;
}

static void fixed_claims_add(uint64_t start, uint64_t end,
                              const char *owner, int bar)
{
    FixedClaim cl;

    cl.start = start;
    cl.end   = end;
    cl.owner = owner;
    cl.bar   = bar;
    g_array_append_val(fixed_claims, cl);
}

static bool validate_bars(PCIDevice *pdev, FixedBarsInfo *info,
                           GArray *bars_out)
{
    bool provided = (pdev->fixed_bar_addrs != NULL);
    const char *overlap_owner;
    uint64_t addr, end, wbase, wlim;
    bool is_64bit, is_pref;
    const char *wname;
    PCIIORegion *r;
    int overlap_bar;
    BarInfo bi;
    int i;

    if (info->is_fixed_subtree && !provided) {
        error_report("pci-bars: %s [%02x:%02x.%x] under fixed-bar root port "
                     "has memory BARs but no pci-bars= specified",
                     pdev->name, pci_dev_bus_num(pdev),
                     PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
        exit(1);
    }

    if (!provided) {
        return false;
    }

    /* Completeness: every memory BAR must have an address. */
    for (i = 0; i < PCI_NUM_REGIONS - 1; i++) {
        r = &pdev->io_regions[i];
        if (!r->size || (r->type & PCI_BASE_ADDRESS_SPACE_IO)) {
            continue;
        }
        if (pdev->fixed_bar_addrs[i] == PCI_BAR_UNMAPPED) {
            error_report("pci-bars: %s [%02x:%02x.%x] BAR%d "
                         "missing from pci-bars=",
                         pdev->name, pci_dev_bus_num(pdev),
                         PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn), i);
            exit(1);
        }
    }

    /* Per-BAR checks: alignment, MMIO window, overlap. */
    for (i = 0; i < PCI_NUM_REGIONS - 1; i++) {
        r = &pdev->io_regions[i];
        if (!r->size || (r->type & PCI_BASE_ADDRESS_SPACE_IO)) {
            continue;
        }

        is_64bit = !!(r->type & PCI_BASE_ADDRESS_MEM_TYPE_64);
        is_pref  = !!(r->type & PCI_BASE_ADDRESS_MEM_PREFETCH);
        addr     = (uint64_t)pdev->fixed_bar_addrs[i];
        end      = addr + r->size - 1;

        if (is_64bit) {
            wbase = info->mmio64_base;
            wlim  = info->mmio64_limit;
            wname = "64-bit MMIO";
        } else {
            wbase = info->mmio32_base;
            wlim  = info->mmio32_limit;
            wname = "32-bit MMIO";
        }

        if (addr & (r->size - 1)) {
            error_report("pci-bars: %s [%02x:%02x.%x] BAR%d "
                         "addr=0x%"PRIx64" not aligned to size=0x%"PRIx64,
                         pdev->name, pci_dev_bus_num(pdev),
                         PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn),
                         i, addr, r->size);
            exit(1);
        }

        if (addr < wbase || end > wlim) {
            error_report("pci-bars: %s [%02x:%02x.%x] BAR%d "
                         "[0x%"PRIx64"..0x%"PRIx64"] outside %s window "
                         "[0x%"PRIx64"..0x%"PRIx64"]",
                         pdev->name, pci_dev_bus_num(pdev),
                         PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn),
                         i, addr, end, wname, wbase, wlim);
            exit(1);
        }

        if (fixed_claims_overlap(addr, end, &overlap_owner, &overlap_bar)) {
            error_report("pci-bars: %s [%02x:%02x.%x] BAR%d "
                         "[0x%"PRIx64"..0x%"PRIx64"] overlaps %s BAR%d",
                         pdev->name, pci_dev_bus_num(pdev),
                         PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn),
                         i, addr, end, overlap_owner, overlap_bar);
            exit(1);
        }

        fixed_claims_add(addr, end, pdev->name, i);

        memset(&bi, 0, sizeof(bi));
        bi.bar   = i;
        bi.addr  = addr;
        bi.size  = r->size;
        bi.flags = (is_64bit ? QEMU_FIXED_BAR_F_MEM64 : 0) |
                   (is_pref  ? QEMU_FIXED_BAR_F_PREF  : 0);

        error_report("pci-bars:   BAR%d addr=0x%"PRIx64" size=0x%"PRIx64,
                     i, addr, r->size);
        g_array_append_val(bars_out, bi);
    }

    return true;
}

static void scan_bus(PCIBus *bus, FixedBarsInfo *info);

static void scan_device(PCIBus *bus, PCIDevice *pdev, FixedBarsInfo *info)
{
    bool is_bridge = !!object_dynamic_cast(OBJECT(pdev), TYPE_PCI_BRIDGE);
    bool is_root_port = !!object_dynamic_cast(OBJECT(pdev), TYPE_PCIE_ROOT_PORT);
    bool has_mem_bar = false;
    bool saved_fixed;
    DeviceInfo dev;
    PCIIORegion *r;
    PCIBus *sec;
    int i;

    for (i = 0; i < PCI_NUM_REGIONS - 1; i++) {
        r = &pdev->io_regions[i];
        if (r->size && !(r->type & PCI_BASE_ADDRESS_SPACE_IO)) {
            has_mem_bar = true;
            break;
        }
    }

    if (has_mem_bar) {
        memset(&dev, 0, sizeof(dev));
        dev.rp_bus    = pci_bus_num(info->root_bus);
        dev.vendor_id = pci_get_word(pdev->config + PCI_VENDOR_ID);
        dev.device_id = pci_get_word(pdev->config + PCI_DEVICE_ID);
        dev.bars      = g_array_new(false, true, sizeof(BarInfo));
        dev.is_fixed  = validate_bars(pdev, info, dev.bars);
        g_array_append_val(info->devices, dev);
    }

    if (!is_bridge) {
        return;
    }
    sec = pci_bridge_get_sec_bus(PCI_BRIDGE(pdev));
    if (!sec) {
        return;
    }
    saved_fixed = info->is_fixed_subtree;
    if (is_root_port) {
        info->is_fixed_subtree = PCIE_SLOT(pdev)->fixed_bar;
    }
    scan_bus(sec, info);
    info->is_fixed_subtree = saved_fixed;
}

static void scan_bus(PCIBus *bus, FixedBarsInfo *info)
{
    int devfn;

    for (devfn = 0; devfn < PCI_DEVFN_MAX; devfn++) {
        if (bus->devices[devfn]) {
            scan_device(bus, bus->devices[devfn], info);
        }
    }
}

static gint cmp_host_bus_num(gconstpointer a, gconstpointer b)
{
    PCIHostState *ha = *(PCIHostState **)a;
    PCIHostState *hb = *(PCIHostState **)b;
    return (gint)pci_bus_num(ha->bus) - (gint)pci_bus_num(hb->bus);
}

/*
 * Scan all host bridges to validate the fixed BAR configuration. During
 * the scan, collect DeviceInfo records for the fw_cfg metadata. Host
 * bridges are processed in ascending bus-number order so that the
 * resulting DeviceInfo records match the PCI function discovery order
 * used by EDK2.
 *
 * TODO: This ordering is only required when constructing the fw_cfg
 * metadata and can be moved into the blob creation path.
 */
static void scan_all_host_bridges(FixedBarsInfo *info)
{
    PCIHostState *hb;
    GPtrArray *sorted;
    guint i;

    sorted = g_ptr_array_new();
    QLIST_FOREACH(hb, &pci_host_bridges, next) {
        if (hb->bus) {
            g_ptr_array_add(sorted, hb);
        }
    }
    g_ptr_array_sort(sorted, cmp_host_bus_num);

    fixed_claims_init();
    for (i = 0; i < sorted->len; i++) {
        hb = g_ptr_array_index(sorted, i);
        info->root_bus = hb->bus;
        info->is_fixed_subtree = false;
        scan_bus(hb->bus, info);
    }
    fixed_claims_free();

    g_ptr_array_free(sorted, true);
}

static bool any_fixed(FixedBarsInfo *info)
{
    guint d;

    for (d = 0; d < info->devices->len; d++) {
        if (g_array_index(info->devices, DeviceInfo, d).is_fixed) {
            return true;
        }
    }
    return false;
}

/*
 * fixed_bars_validate - scan all PCI devices, validate fixed BAR addresses.
 *
 * @info: carries MMIO window bounds for validation; populated with
 *        DeviceInfo entries for all devices with memory BARs.
 *
 * Returns true if any fixed BAR devices were found, false if there is
 * nothing to do.  Aborts on any validation error.
 */
bool fixed_bars_validate(FixedBarsInfo *info)
{
    scan_all_host_bridges(info);
    return any_fixed(info);
}
