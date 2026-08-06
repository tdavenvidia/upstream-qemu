/*
 * Serialize validated fixed-BAR device info into the etc/fixed-bars
 * fw_cfg blob consumed by QemuFixedBarsDxe in EDK2.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/nvram/fw_cfg.h"
#include "qemu/error-report.h"
#include "pci-fixed-bar-priv.h"

static uint8_t *create_blob(FixedBarsInfo *info, size_t *blob_size)
{
    QemuFixedBarsDevice *dout;
    QemuFixedBarsBar *bout;
    QemuFixedBarsHdr *hdr;
    DeviceInfo *dev;
    uint8_t *blob;
    uint8_t *ptr;
    BarInfo *bi;
    size_t sz;
    guint d, b;

    sz = sizeof(QemuFixedBarsHdr);
    for (d = 0; d < info->devices->len; d++) {
        dev = &g_array_index(info->devices, DeviceInfo, d);
        sz += sizeof(QemuFixedBarsDevice);
        sz += dev->bars->len * sizeof(QemuFixedBarsBar);
    }

    blob = g_malloc0(sz);
    hdr  = (QemuFixedBarsHdr *)blob;
    hdr->version     = cpu_to_le32(QEMU_FIXED_BARS_VERSION);
    hdr->num_devices = cpu_to_le32(info->devices->len);

    ptr = blob + sizeof(*hdr);
    for (d = 0; d < info->devices->len; d++) {
        dev  = &g_array_index(info->devices, DeviceInfo, d);
        dout = (QemuFixedBarsDevice *)ptr;

        dout->vendor_id = cpu_to_le16(dev->vendor_id);
        dout->device_id = cpu_to_le16(dev->device_id);
        dout->dev_flags = dev->is_fixed ? QEMU_FIXED_BARS_DEV_F_FIXED : 0;
        dout->rp_bus    = dev->rp_bus;
        dout->num_bars  = dev->bars->len;
        dout->reserved  = 0;
        ptr += sizeof(QemuFixedBarsDevice);

        for (b = 0; b < dev->bars->len; b++) {
            bi   = &g_array_index(dev->bars, BarInfo, b);
            bout = (QemuFixedBarsBar *)ptr;

            bout->bar     = bi->bar;
            bout->flags   = cpu_to_le32(bi->flags);
            bout->address = cpu_to_le64(bi->addr);
            bout->size    = cpu_to_le64(bi->size);
            ptr += sizeof(QemuFixedBarsBar);
        }
    }

    *blob_size = sz;
    return blob;
}

/*
 * fixed_bars_write_blob - validate fixed BAR addresses and write fw_cfg blob.
 *
 * @fw_cfg:       fw_cfg state to write the etc/fixed-bars blob into.
 * @mmio32_base:  base of the machine's 32-bit PCIe MMIO aperture.
 * @mmio32_size:  size of the machine's 32-bit PCIe MMIO aperture.
 * @mmio64_base:  base of the machine's 64-bit PCIe MMIO aperture.
 * @mmio64_size:  size of the machine's 64-bit PCIe MMIO aperture.
 *
 * Aborts if any BAR address fails validation.
 */
void fixed_bars_write_blob(FWCfgState *fw_cfg,
                            uint64_t mmio32_base,
                            uint64_t mmio32_size,
                            uint64_t mmio64_base,
                            uint64_t mmio64_size)
{
    FixedBarsInfo info;
    uint8_t *blob;
    size_t blob_size;
    guint d;

    info.devices          = g_array_new(false, true, sizeof(DeviceInfo));
    info.root_bus         = NULL;
    info.is_fixed_subtree = false;
    info.mmio32_base      = mmio32_base;
    info.mmio32_limit     = mmio32_base + mmio32_size - 1;
    info.mmio64_base      = mmio64_base;
    info.mmio64_limit     = mmio64_base + mmio64_size - 1;

    if (!fixed_bars_validate(&info)) {
        goto out;
    }

    /* Create and write blob. */
    blob = create_blob(&info, &blob_size);
    fw_cfg_add_file(fw_cfg, FW_CFG_FIXED_BARS, blob, blob_size);

out:
    for (d = 0; d < info.devices->len; d++) {
        g_array_free(g_array_index(info.devices, DeviceInfo, d).bars, true);
    }
    g_array_free(info.devices, true);
}
