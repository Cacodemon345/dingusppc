/*
DingusPPC - The Experimental PowerPC Macintosh emulator
Copyright (C) 2018-26 The DingusPPC Development Team
          (See CREDITS.MD for more details)

(You may also contact divingkxt or powermax2286 on Discord)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <core/bitops.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/pci/pcidevice.h>
#include <devices/deviceregistry.h>
#include <devices/video/atirage.h>
#include <devices/video/displayid.h>
#include <endianswap.h>
#include <loguru.hpp>
#include <memaccess.h>

#include <map>

namespace loguru {
    enum : Verbosity {
        Verbosity_ATIRAGE = loguru::Verbosity_9,
        Verbosity_ATIINTERRUPT = loguru::Verbosity_9,
        Verbosity_ATICURSOR = loguru::Verbosity_9,
        Verbosity_ATIDRAW = loguru::Verbosity_9,
    };
}

/* Mach64 post dividers. */
static const int mach64_post_div[8] = {
    1, 2, 4, 8, // standard post dividers
    3, 5, 6, 12 // alternate post dividers
};

/* Human readable Mach64 HW register names for easier debugging. */
static const std::map<uint16_t, std::string> mach64_reg_names = {
    #define one_reg_name(x) {ATI_ ## x, #x}
    one_reg_name(CRTC_H_TOTAL_DISP),
    one_reg_name(CRTC_H_SYNC_STRT_WID),
    one_reg_name(CRTC_V_TOTAL_DISP),
    one_reg_name(CRTC_V_SYNC_STRT_WID),
    one_reg_name(CRTC_VLINE_CRNT_VLINE),
    one_reg_name(CRTC_OFF_PITCH),
    one_reg_name(CRTC_INT_CNTL),
    one_reg_name(CRTC_GEN_CNTL),
    one_reg_name(DSP_CONFIG),
    one_reg_name(DSP_ON_OFF),
    one_reg_name(MEM_BUF_CNTL),
    one_reg_name(MEM_ADDR_CFG),
    one_reg_name(OVR_CLR),
    one_reg_name(OVR_WID_LEFT_RIGHT),
    one_reg_name(OVR_WID_TOP_BOTTOM),
    one_reg_name(CUR_CLR0),
    one_reg_name(CUR_CLR1),
    one_reg_name(CUR_OFFSET),
    one_reg_name(CUR_HORZ_VERT_POSN),
    one_reg_name(CUR_HORZ_VERT_OFF),
    one_reg_name(GP_IO),
    one_reg_name(DST_X_Y),
    one_reg_name(DST_Y_X),
    one_reg_name(HW_DEBUG),
    one_reg_name(SCRATCH_REG0),
    one_reg_name(SCRATCH_REG1),
    one_reg_name(SCRATCH_REG2),
    one_reg_name(SCRATCH_REG3),
    one_reg_name(CLOCK_CNTL),
    one_reg_name(BUS_CNTL),
    one_reg_name(EXT_MEM_CNTL),
    one_reg_name(MEM_CNTL),
    one_reg_name(DAC_REGS),
    one_reg_name(DAC_CNTL),
    one_reg_name(GEN_TEST_CNTL),
    one_reg_name(CUSTOM_MACRO_CNTL),
    one_reg_name(CONFIG_CHIP_ID),
    one_reg_name(CONFIG_STAT0),
    one_reg_name(SRC_CNTL),
    one_reg_name(SCALE_3D_CNTL),
    one_reg_name(FIFO_STAT),
    one_reg_name(GUI_STAT),
    one_reg_name(MPP_CONFIG),
    one_reg_name(MPP_STROBE_SEQ),
    one_reg_name(MPP_ADDR),
    one_reg_name(MPP_DATA),
    one_reg_name(TVO_CNTL),
    one_reg_name(SETUP_CNTL),
};

ATIRage::ATIRage(const std::string &dev_name, uint16_t dev_id) :
    PCIVideoCtrl(dev_name), HWComponent(dev_name)
{
    uint8_t asic_id;

    supports_types(HWCompType::MMIO_DEV | HWCompType::PCI_DEV | HWCompType::VIDEO_CTRL);

    // ATI Rage driver needs to know ASIC ID (manufacturer's internal chip code)
    // to operate properly
    switch (dev_id) {
    case ATI_RAGE_GT_DEV_ID:
        asic_id = 0x9A; // GT-B2U3 fabricated by UMC
        this->cmd_fifo_size = 48;
        break;
    case ATI_RAGE_PRO_DEV_ID:
        asic_id = 0x5C; // R3B/D/P-A4 fabricated by UMC
        this->cmd_fifo_size = 128;
        break;
    default:
        asic_id = 0xDD;
        LOG_F(WARNING, "%s: bogus ASIC ID assigned!", this->get_name_and_unit_address().c_str());
    }

    // set up PCI configuration space header
    this->vendor_id   = PCI_VENDOR_ATI;
    this->device_id   = dev_id;
    this->subsys_vndr = PCI_VENDOR_ATI;
    this->subsys_id   = 0x6987; // adapter ID
    this->class_rev   = (0x030000 << 8) | asic_id;
    this->min_gnt     = 8;
    this->irq_pin     = 1;
    for (int i = 0; i < this->aperture_count; i++) {
        this->bars_cfg[i] = (uint32_t)(-this->aperture_size[i] | this->aperture_flag[i]);
    }
    this->finish_config_bars();

    this->pci_notify_bar_change = [this](int bar_num) {
        this->notify_bar_change(bar_num);
    };

    // stuff default values into chip registers
    this->regs[ATI_CONFIG_CHIP_ID] = (asic_id << ATI_CFG_CHIP_MAJOR) | (dev_id << ATI_CFG_CHIP_TYPE);

    insert_bits<uint32_t>(this->regs[ATI_GUI_STAT], 32, ATI_FIFO_CNT, ATI_FIFO_CNT_size);
    set_bit(regs[ATI_CRTC_GEN_CNTL], ATI_CRTC_DISPLAY_DIS); // because blank_on is true

    this->draw_fb_is_dynamic = true;
}

HWComponent* ATIRage::set_property(const std::string &property, const std::string &value, int32_t unit_address) {
    if (unit_address == -1) {
        if (property == "gfxmem_size") {
            if (this->override_property(property, value)) {
                // allocate video RAM
                int vram = this->get_property_int("gfxmem_size");
                LOG_F(INFO, "%s: setting VRAM to %d MB", this->get_name_and_unit_address().c_str(), vram);
                this->vram_size = vram << 20; // convert MBs to bytes
                this->framebuffer_size = std::min(this->vram_size, ((uint32_t)8 << 20) - 0x800);
                this->vram_ptr = std::unique_ptr<uint8_t[]> (new uint8_t[this->vram_size]);
                return this;
            }
        }
        return PCIVideoCtrl::set_property(property, value, unit_address);
    }
    return nullptr;
}

void ATIRage::change_one_bar(uint32_t &aperture, uint32_t aperture_size,
                             uint32_t aperture_new, int bar_num) {
    if (aperture != aperture_new) {
        if (aperture)
            this->host_instance->pci_unregister_mmio_region(aperture,
                                                            aperture_size, this);

        aperture = aperture_new;
        if (aperture)
            this->host_instance->pci_register_mmio_region(aperture, aperture_size, this);

        LOG_F(INFO, "%s: aperture[%d] set to 0x%08X", this->get_name_and_unit_address().c_str(),
              bar_num, aperture);
    }
}

void ATIRage::notify_bar_change(int bar_num)
{
    switch (bar_num) {
    case 0:
        change_one_bar(this->aperture_base[bar_num],
                       std::min(this->aperture_size[bar_num] - 4096, BE_FB_OFFSET + this->vram_size),
                       this->bars[bar_num] & ~15, bar_num);
        break;
    case 2:
        change_one_bar(this->aperture_base[bar_num],
                       this->aperture_size[bar_num],
                       this->bars[bar_num] & ~15, bar_num);
        break;
    case 1:
        this->aperture_base[1] = this->bars[bar_num] & ~3;
        LOG_F(INFO, "%s: I/O space address set to 0x%08X", this->get_name_and_unit_address().c_str(),
              this->aperture_base[1]);
        break;
    }
}

uint32_t ATIRage::pci_cfg_read(uint32_t reg_offs, const AccessDetails details)
{
    if (reg_offs < 64) {
        return PCIDevice::pci_cfg_read(reg_offs, details);
    }

    switch (reg_offs) {
    case 0x40:
        return this->user_cfg;
    default:
        LOG_READ_UNIMPLEMENTED_CONFIG_REGISTER();
    }

    return 0;
}

void ATIRage::pci_cfg_write(uint32_t reg_offs, uint32_t value, const AccessDetails details)
{
    if (reg_offs < 64) {
        PCIDevice::pci_cfg_write(reg_offs, value, details);
        return;
    }

    switch (reg_offs) {
    case 0x40:
        this->user_cfg = value;
        break;
    default:
        LOG_WRITE_UNIMPLEMENTED_CONFIG_REGISTER();
    }
}

const char* ATIRage::get_reg_name(uint32_t reg_num) {
    auto iter = mach64_reg_names.find(reg_num);
    if (iter != mach64_reg_names.end()) {
        return iter->second.c_str();
    } else {
        return "unknown Mach64 register";
    }
}

uint32_t ATIRage::read_reg(uint32_t reg_offset, uint32_t size) {
    uint32_t reg_num = reg_offset >> 2;
    uint32_t offset = reg_offset & 3;
    uint64_t result = this->regs[reg_num];

    if (reg_num == ATI_DST_Y_X_ALIAS1)
        reg_num = ATI_DST_Y_X;

    if (reg_num == ATI_GUI_TRAJ_CNTL) {
        result = this->regs[ATI_DST_CNTL] & 0xFFFF;
        result |= (this->regs[ATI_SRC_CNTL] & 0xFF) << 16;
        result |= (this->regs[ATI_PAT_CNTL] & 7) << 24;
        result |= (this->regs[ATI_HOST_CNTL] & 3) << 28;
    }

    if (reg_num == ATI_SRC_HEIGHT1_WIDTH1) {
        result = this->regs[ATI_SRC_WIDTH1];
        result |= this->regs[ATI_SRC_HEIGHT1] << 16;
    }

    if (reg_num == ATI_SRC_HEIGHT2_WIDTH2) {
        result = this->regs[ATI_SRC_WIDTH2];
        result |= this->regs[ATI_SRC_HEIGHT2] << 16;
    }

    if (reg_num == ATI_DST_Y_X) {
        result = this->regs[ATI_DST_Y];
        result |= this->regs[ATI_DST_X] << 16;
    }
    
    if (reg_num == ATI_SRC_Y_X) {
        result = this->regs[ATI_SRC_Y];
        result |= this->regs[ATI_SRC_X] << 16;
    }
    
    if (reg_num == ATI_SRC_Y_X_START) {
        result = this->regs[ATI_SRC_Y_START];
        result |= this->regs[ATI_SRC_X_START] << 16;
    }

    switch (reg_num) {
    case ATI_CLOCK_CNTL:
        if (offset <= 2 && offset + size > 2) {
            uint8_t pll_addr = extract_bits<uint64_t>(result, ATI_PLL_ADDR, ATI_PLL_ADDR_size);
            insert_bits<uint64_t>(result, this->plls[pll_addr], ATI_PLL_DATA, ATI_PLL_DATA_size);
        }
        break;
    case ATI_DAC_REGS:
        switch (reg_offset) {
        case ATI_DAC_W_INDEX:
            insert_bits<uint64_t>(result, this->dac_wr_index, 0, 8);
            break;
        case ATI_DAC_MASK:
            insert_bits<uint64_t>(result, this->dac_mask, 16, 8);
            break;
        case ATI_DAC_R_INDEX:
            insert_bits<uint64_t>(result, this->dac_rd_index, 24, 8);
            break;
        case ATI_DAC_DATA:
            if (!this->comp_index) {
                uint8_t alpha; // temp variable for unused alpha
                get_palette_color(this->dac_rd_index, color_buf[0],
                                  color_buf[1], color_buf[2], alpha);
            }
            insert_bits<uint64_t>(result, color_buf[this->comp_index], 8, 8);
            if (++this->comp_index >= 3) {
                this->dac_rd_index++; // auto-increment reading index
                this->comp_index = 0; // reset color component index
            }
        }
        break;
    case ATI_GUI_STAT:
        result = uint64_t(this->cmd_fifo_size << 16); // HACK: pretend empty FIFO
        break;
    case ATI_DST_X_Y:
        result = (extract_bits(this->regs[ATI_DST_X], 0, ATI_DST_X_size)) | (extract_bits(this->regs[ATI_DST_Y], 0, ATI_DST_Y_size) << 16);
        break;
    case ATI_DST_Y_X:
        result = (extract_bits(this->regs[ATI_DST_X], 0, ATI_DST_X_size) << 16) | (extract_bits(this->regs[ATI_DST_Y], 0, ATI_DST_Y_size));
        break;
    case ATI_DP_BKGD_CLR:
    case ATI_DP_FRGD_CLR:
        uint32_t pix_fmt = extract_bits<uint32_t>(
            this->regs[ATI_CRTC_GEN_CNTL], ATI_CRTC_PIX_WIDTH, ATI_CRTC_PIX_WIDTH_size);

        switch (pix_fmt) {
        case 1:
            result = result & 0x1;
            break;
        case 2:
            result = result & 0xFF;
            break;
        case 3:
        case 4:
            result = result & 0xFFFF;
            break;
        case 5:
            result = result & 0xFFFFFF;
            break;
        case 6:
            break;
        default:
            LOG_F(ERROR, "Incorrect bit depth");
        }
        break;
    }

    if (offset || size != 4) { // slow path
        if ((offset + size) > 4) {
            result |= (uint64_t)(this->regs[reg_num + 1]) << 32;
        }
        result = extract_bits<uint64_t>(result, offset * 8, size * 8);
    }

    if (reg_num != ATI_CRTC_INT_CNTL && reg_num != ATI_GP_IO && reg_num != ATI_DAC_REGS)
        LOG_F(ATIRAGE, "%s: read  %s %04x.%c = %0*x", this->get_name_and_unit_address().c_str(),
            get_reg_name(reg_num), reg_offset, SIZE_ARG(size), size * 2, (uint32_t)result);
    return static_cast<uint32_t>(result);
}

#define LOG_VALUE(level) \
    do { \
        LOG_F(level, "%s: write %s %04x.%c = %0*x = %08x", this->get_name_and_unit_address().c_str(), \
            get_reg_name(reg_num), reg_offset, SIZE_ARG(size), size * 2, \
            (uint32_t)extract_bits<uint64_t>(value, offset * 8, size * 8), new_value \
        ); \
    } while (0)

#define WRITE_VALUE_AND_LOG(level) \
    do { \
        this->regs[reg_num] = new_value; \
        if (reg_num != ATI_GP_IO && reg_num != ATI_CRTC_INT_CNTL && reg_num != ATI_DAC_REGS) \
            LOG_VALUE(level); \
    } while (0)

void ATIRage::write_reg(uint32_t reg_offset, uint32_t value, uint32_t size) {
    uint32_t reg_num = reg_offset >> 2;
    uint32_t offset = reg_offset & 3;
    uint32_t old_value = this->regs[reg_num];
    uint32_t new_value;

    if (reg_num == ATI_DST_Y_X_ALIAS1)
        reg_num = ATI_DST_Y_X;

    if (reg_num == ATI_GUI_TRAJ_CNTL) {
        old_value = this->regs[ATI_DST_CNTL] & 0xFFFF;
        old_value |= (this->regs[ATI_SRC_CNTL] & 0xFF) << 16;
        old_value |= (this->regs[ATI_PAT_CNTL] & 7) << 24;
        old_value |= (this->regs[ATI_HOST_CNTL] & 3) << 28;
    }

    if (reg_num == ATI_SRC_HEIGHT1_WIDTH1) {
        old_value = this->regs[ATI_SRC_WIDTH1];
        old_value |= this->regs[ATI_SRC_HEIGHT1] << 16;
    }

    if (reg_num == ATI_SRC_HEIGHT2_WIDTH2) {
        old_value = this->regs[ATI_SRC_WIDTH2];
        old_value |= this->regs[ATI_SRC_HEIGHT2] << 16;
    }

    if (reg_num == ATI_DST_Y_X) {
        old_value = this->regs[ATI_DST_Y];
        old_value |= this->regs[ATI_DST_X] << 16;
    }

    if (reg_num == ATI_SRC_Y_X) {
        old_value = this->regs[ATI_SRC_Y];
        old_value |= this->regs[ATI_SRC_X] << 16;
    }
    
    if (reg_num == ATI_SRC_Y_X_START) {
        old_value = this->regs[ATI_SRC_Y_START];
        old_value |= this->regs[ATI_SRC_X_START] << 16;
    }

    if (reg_offset >= 0x200 && reg_offset <= 0x23F && (reg_offset + size >= 0x200) && (reg_offset + size <= 0x23F)) {
        if (this->host_data_active) {
            if (this->regs[ATI_HOST_CNTL] & 2) {
                switch (this->host_data_req) {
                    case 16:
                    {
                        if (size == 4) {
                            value = ((value & 0xFFFF) << 16) | (value >> 16);
                        }
                        // fallthrough
                    }
                    case 32:
                    {
                        value = BYTESWAP_SIZED(value, size);
                        break;
                    }
                }
            }
            this->host_data |= value << this->host_data_pos;
            this->host_data_pos += size * 8;
            if (this->host_data_pos >= this->host_data_req) {
                this->process_host_data();
            }
        }
        return;
    }

    if (offset || size != 4) { // slow path
        if ((offset + size) > 4) {
            ABORT_F("%s: unaligned DWORD writes not implemented", this->get_name_and_unit_address().c_str());
        }
        uint64_t val = old_value;
        insert_bits<uint64_t>(val, value, offset * 8, size * 8);
        value = static_cast<uint32_t>(val);
    }

    switch (reg_num) {
    case ATI_CRTC_H_TOTAL_DISP:
        new_value = value;
        break;
    case ATI_CRTC_VLINE_CRNT_VLINE:
        new_value = old_value;
        insert_bits<uint32_t>(new_value, value, ATI_CRTC_VLINE, ATI_CRTC_VLINE_size);
        break;
    case ATI_CRTC_OFF_PITCH:
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        this->crtc_update();
        return;
    case ATI_CRTC_INT_CNTL: {
        uint32_t bits_read_only =
            (1 << ATI_CRTC_VBLANK) |
            (1 << ATI_CRTC_VLINE_SYNC) |
            (1 << ATI_CRTC_FRAME) |
#if 1
#else
            (1 << ATI_CRTC2_VBLANK) |
            (1 << ATI_CRTC2_VLINE_SYNC) |
#endif
            0;

        uint32_t bits_AK =
            (1 << ATI_CRTC_VBLANK_INT_AK) |
            (1 << ATI_CRTC_VLINE_INT_AK) |
#if 1
            (1 << ATI_VIDEOIN_EVEN_INT_AK) |
            (1 << ATI_VIDEOIN_ODD_INT_AK) |
            (1 << ATI_OVERLAY_EOF_INT_AK) |
            (1 << ATI_VMC_EC_INT_AK) |
#else
            (1 << ATI_SNAPSHOT_INT_AK) |
            (1 << ATI_I2C_INT_AK) |
            (1 << ATI_CRTC2_VBLANK_INT_AK) |
            (1 << ATI_CRTC2_VLINE_INT_AK) |
            (1 << ATI_CUPBUF0_INT_AK) |
            (1 << ATI_CUPBUF1_INT_AK) |
            (1 << ATI_OVERLAY_EOF_INT_AK) |
            (1 << ATI_ONESHOT_CAP_INT_AK) |
            (1 << ATI_BUSMASTER_EOL_INT_AK) |
            (1 << ATI_GP_INT_AK) |
            (1 << ATI_SNAPSHOT2_INT_AK) |
            (1 << ATI_VBLANK_BIT2_INT_AK) |
#endif
            0;
/*
        uint32_t bits_EN =
            (1 << ATI_CRTC_VBLANK_INT_EN) |
            (1 << ATI_CRTC_VLINE_INT_EN) |
#if 1
            (1 << ATI_VIDEOIN_EVEN_INT_EN) |
            (1 << ATI_VIDEOIN_ODD_INT_EN) |
            (1 << ATI_OVERLAY_EOF_INT_EN) |
            (1 << ATI_VMC_EC_INT_EN) |
#else
            (1 << ATI_SNAPSHOT_INT_EN) |
            (1 << ATI_I2C_INT_EN) |
            (1 << ATI_CRTC2_VBLANK_INT_EN) |
            (1 << ATI_CRTC2_VLINE_INT_EN) |
            (1 << ATI_CUPBUF0_INT_EN) |
            (1 << ATI_CUPBUF1_INT_EN) |
            (1 << ATI_OVERLAY_EOF_INT_EN) |
            (1 << ATI_ONESHOT_CAP_INT_EN) |
            (1 << ATI_BUSMASTER_EOL_INT_EN) |
            (1 << ATI_GP_INT_EN) |
            (1 << ATI_SNAPSHOT2_INT_EN) |
#endif
            0;
*/
        uint32_t bits_AKed = bits_AK & value; // AK bits that are to be AKed
        uint32_t bits_not_AKed = bits_AK & ~value; // AK bits that are not to be AKed

        new_value = value & ~bits_AKed; // clear the AKed bits
        bits_read_only |= bits_not_AKed; // the not AKed bits will remain unchanged

        new_value = (old_value & bits_read_only) | (new_value & ~bits_read_only);
        WRITE_VALUE_AND_LOG(ATIINTERRUPT);
        return;
    }
    case ATI_CRTC_GEN_CNTL:
    {
        uint32_t bits_AK =
#if 1
#else
            (1 << ATI_CRTC_VSYNC_INT_AK) |
            (1 << ATI_CRTC2_VSYNC_INT_AK) |
#endif
            0;
/*
        uint32_t bits_EN =
#if 1
#else
            (1 << ATI_CRTC_VSYNC_INT_EN) |
            (1 << ATI_CRTC2_VSYNC_INT_EN) |
#endif
            0;
*/
        uint32_t bits_AKed = bits_AK & value; // AK bits that are to be AKed
        uint32_t bits_not_AKed = bits_AK & ~value; // AK bits that are not to be AKed

        new_value = value & ~bits_AKed; // clear the AKed bits
        uint32_t bits_read_only = bits_not_AKed; // the not AKed bits will remain unchanged

        new_value = (old_value & bits_read_only) | (new_value & ~bits_read_only);

        if (bit_changed(old_value, new_value, ATI_CRTC_DISPLAY_DIS)) {
            if (bit_set(new_value, ATI_CRTC_DISPLAY_DIS)) {
                this->blank_on = true;
                this->blank_display();
            } else {
                this->blank_on = false;
            }
        }

        WRITE_VALUE_AND_LOG(ATIRAGE);

        if (bit_changed(old_value, new_value, ATI_CRTC_ENABLE)
            || extract_bits(old_value, ATI_CRTC_PIX_WIDTH, ATI_CRTC_PIX_WIDTH_size) !=
            extract_bits(new_value, ATI_CRTC_PIX_WIDTH, ATI_CRTC_PIX_WIDTH_size)
        ) {
            draw_fb = true;
            if (bit_set(new_value, ATI_CRTC_ENABLE) &&
                !bit_set(new_value, ATI_CRTC_DISPLAY_DIS)) {
                this->crtc_update();
            }
        }
        return;
    }
    case ATI_CUR_CLR0:
    case ATI_CUR_CLR1:
        new_value = value;
        this->cursor_dirty = true;
        draw_fb = true;
        WRITE_VALUE_AND_LOG(ATICURSOR);
        return;
    case ATI_CUR_OFFSET:
        new_value = value;
        if (old_value != new_value)
            this->cursor_dirty = true;
        draw_fb = true;
        WRITE_VALUE_AND_LOG(ATICURSOR);
        return;
    case ATI_CUR_HORZ_VERT_OFF:
        new_value = value;
        if (
            extract_bits<uint32_t>(new_value, ATI_CUR_VERT_OFF, ATI_CUR_VERT_OFF_size) !=
            extract_bits<uint32_t>(old_value, ATI_CUR_VERT_OFF, ATI_CUR_VERT_OFF_size)
        )
            this->cursor_dirty = true;
        draw_fb = true;
        WRITE_VALUE_AND_LOG(ATICURSOR);
        return;
    case ATI_CUR_HORZ_VERT_POSN:
        new_value = value;
        draw_fb = true;
        WRITE_VALUE_AND_LOG(ATICURSOR);
        return;
    case ATI_GP_IO:
        new_value = value;
        if (offset <= 1 && offset + size > 1) {
            uint8_t gpio_levels = (new_value >> 8) & 0xFFU;
            gpio_levels = ((gpio_levels & 0x30) >> 3) | (gpio_levels & 1);
            uint8_t gpio_dirs = (new_value >> 24) & 0xFFU;
            gpio_dirs = ((gpio_dirs & 0x30) >> 3) | (gpio_dirs & 1);
            gpio_levels = this->disp_id->read_monitor_sense(gpio_levels, gpio_dirs);
            insert_bits<uint32_t>(new_value,
                                ((gpio_levels & 6) << 3) | (gpio_levels & 1), 8, 8);
        }
        break;
    case ATI_CLOCK_CNTL: {
        uint32_t bits_write_only =
            (1 << ATI_CLOCK_STROBE);

        new_value = value & ~bits_write_only; // clear the write only bits

        uint8_t pll_addr = extract_bits<uint32_t>(new_value, ATI_PLL_ADDR,
                                                  ATI_PLL_ADDR_size);
        if (offset <= 2 && offset + size > 2 && bit_set(new_value, ATI_PLL_WR_EN)) {
            uint8_t pll_data = extract_bits<uint32_t>(new_value, ATI_PLL_DATA,
                                                      ATI_PLL_DATA_size);
            this->plls[pll_addr] = pll_data;
            LOG_F(9, "%s: PLL #%d set to 0x%02X", this->get_name_and_unit_address().c_str(), pll_addr, pll_data);
        }
        else {
            insert_bits<uint32_t>(new_value, this->plls[pll_addr], ATI_PLL_DATA,
                                  ATI_PLL_DATA_size);
        }
        break;
    }
    case ATI_BUS_CNTL:
    {
        uint32_t bits_read_only =
#if 1
#else
            (1 << ATI_LAT16X) |
            (1 << ATI_MINOR_REV_ID) |
#endif
            0;
            ;

        uint32_t bits_write_only =
#if 1
#else
            (1 << ATI_BUS_MSTR_RESET) |
            (1 << ATI_BUS_FLUSH_BUF) |
#endif
            0;

        new_value = value & ~bits_write_only; // clear the write only bits
        new_value = (old_value & bits_read_only) | (new_value & ~bits_read_only);
        break;
    }
    case ATI_DAC_REGS:
        new_value = old_value; // no change
        switch (reg_offset) {
        case ATI_DAC_W_INDEX:
            this->dac_wr_index = value & 0xFFU;
            this->comp_index = 0;
            break;
        case ATI_DAC_MASK:
            this->dac_mask = (value >> 16) & 0xFFU;
            break;
        case ATI_DAC_R_INDEX:
            this->dac_rd_index = (value >> 24) & 0xFFU;
            this->comp_index = 0;
            break;
        case ATI_DAC_DATA:
            this->color_buf[this->comp_index] = (value >> 8) & this->dac_mask;
            if (++this->comp_index >= 3) {
                this->set_palette_color(this->dac_wr_index, color_buf[0],
                                        color_buf[1], color_buf[2], 0xFF);
                this->dac_wr_index++; // auto-increment color index
                this->comp_index = 0; // reset color component index
                draw_fb = true;
            }
        }
        break;
    case ATI_GEN_TEST_CNTL:
        new_value = value;
        if (bit_changed(old_value, new_value, ATI_GEN_CUR_ENABLE)) {
            if (bit_set(new_value, ATI_GEN_CUR_ENABLE))
                this->cursor_on = true;
            else
                this->cursor_on = false;
            draw_fb = true;
            WRITE_VALUE_AND_LOG(ATICURSOR);
            return;
        }
        if (bit_changed(old_value, new_value, ATI_GEN_GUI_RESETB)) {
            if (!bit_set(new_value, ATI_GEN_GUI_RESETB))
                LOG_F(9, "%s: reset GUI engine", this->get_name_and_unit_address().c_str());
        }
        if (bit_changed(old_value, new_value, ATI_GEN_SOFT_RESET)) {
            if (bit_set(new_value, ATI_GEN_SOFT_RESET))
                LOG_F(9, "%s: reset memory controller", this->get_name_and_unit_address().c_str());
        }
        if (new_value & 0xFFFFFC00) {
            LOG_F(WARNING, "%s: unhandled GEN_TEST_CNTL state=0x%X",
                  this->get_name_and_unit_address().c_str(), new_value);
        }
        break;
    case ATI_CONFIG_CHIP_ID:
        new_value = old_value; // prevent writes to this read-only register
        break;
    case ATI_CONFIG_STAT0:
    {
        uint32_t bits_read_only =
#if 1
#else
            (1 << ATI_MACROVISION_ENABLE) |
            (1 << ATI_ARITHMOS_ENABLE) |
#endif
            0;

        new_value = value;
        new_value = (old_value & bits_read_only) | (new_value & ~bits_read_only);
        break;
    }
    case ATI_DST_WIDTH:
    case ATI_DST_HEIGHT_WIDTH:
    case ATI_DST_WIDTH_HEIGHT:
    case ATI_DST_X_WIDTH:
    case ATI_DST_BRES_LNTH:
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        this->begin_drawing(reg_num, value);
        return;

    case ATI_DST_X_Y:
        insert_bits(this->regs[ATI_DST_X], extract_bits(value, 0, ATI_DST_X_size), 0, ATI_DST_X_size);
        insert_bits(this->regs[ATI_DST_Y], extract_bits(value, 16, ATI_DST_Y_size), 0, ATI_DST_Y_size);
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        return;
    
    case ATI_DST_Y_X:
        insert_bits(this->regs[ATI_DST_X], extract_bits(value, 16, ATI_DST_X_size), 0, ATI_DST_X_size);
        insert_bits(this->regs[ATI_DST_Y], extract_bits(value, 0, ATI_DST_Y_size), 0, ATI_DST_Y_size);
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        return;

    case ATI_SRC_HEIGHT2_WIDTH2:
        insert_bits(this->regs[ATI_SRC_WIDTH2], extract_bits(value, 16, 16), 0, 16);
        insert_bits(this->regs[ATI_SRC_HEIGHT2], extract_bits(value, 0, 16), 0, 16);
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        return;

    case ATI_SRC_HEIGHT1_WIDTH1:
        insert_bits(this->regs[ATI_SRC_WIDTH1], extract_bits(value, 16, 16), 0, 16);
        insert_bits(this->regs[ATI_SRC_HEIGHT1], extract_bits(value, 0, 16), 0, 16);
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        return;

    case ATI_SRC_Y_X:
        insert_bits(this->regs[ATI_SRC_X], extract_bits(value, 16, 16), 0, 16);
        insert_bits(this->regs[ATI_SRC_Y], extract_bits(value, 0, 16), 0, 16);
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        return;

    case ATI_SRC_Y_X_START:
        insert_bits(this->regs[ATI_SRC_X_START], extract_bits(value, 16, ATI_SRC_X_START_size), 0, ATI_DST_X_size);
        insert_bits(this->regs[ATI_SRC_Y_START], extract_bits(value, 0, ATI_SRC_Y_START_size), 0, ATI_DST_Y_size);
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        return;

    case ATI_SC_TOP_BOTTOM:
        insert_bits(this->regs[ATI_SC_TOP], extract_bits(value, 0, ATI_SC_TOP_size), 0, ATI_SC_TOP_size);
        insert_bits(this->regs[ATI_SC_BOTTOM], extract_bits(value, 16, ATI_SC_BOTTOM_size), 0, ATI_SC_BOTTOM_size);
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        return;
    case ATI_SC_LEFT_RIGHT:
        insert_bits(this->regs[ATI_SC_LEFT], extract_bits(value, 0, ATI_SC_LEFT_size), 0, ATI_SC_LEFT_size);
        insert_bits(this->regs[ATI_SC_RIGHT], extract_bits(value, 16, ATI_SC_RIGHT_size), 0, ATI_SC_RIGHT_size);
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        return;
    
    case ATI_DP_SET_GUI_ENGINE:
    case ATI_DP_SET_GUI_ENGINE2:
        WRITE_VALUE_AND_LOG(WARNING);
        break;
    
    case ATI_GUI_TRAJ_CNTL:
        this->regs[ATI_DST_CNTL] &= ~0xFFFF;
        this->regs[ATI_DST_CNTL] |= value & 0xFFFF;
        this->regs[ATI_SRC_CNTL] &= ~0xFF;
        this->regs[ATI_SRC_CNTL] |= (value >> 16) & 0xFF;
        this->regs[ATI_PAT_CNTL] &= ~0x7;
        this->regs[ATI_PAT_CNTL] |= (value >> 24) & 0x7;;
        this->regs[ATI_HOST_CNTL] &= ~0x3;
        this->regs[ATI_HOST_CNTL] |= (value >> 28) & 0x3;
        new_value = value;
        WRITE_VALUE_AND_LOG(ATIRAGE);
        return;

    default:
        new_value = value;
        break;
    }

    WRITE_VALUE_AND_LOG(ATIRAGE);
}

bool ATIRage::io_access_allowed(uint32_t offset) {
    if (offset >= this->aperture_base[1] && offset < (this->aperture_base[1] + 0x100)) {
        if (this->command & 1) {
            return true;
        }
        LOG_F(WARNING, "%s: I/O space disabled in the command reg", this->get_name_and_unit_address().c_str());
    }
    return false;
}

bool ATIRage::pci_io_read(uint32_t offset, uint32_t size, uint32_t* res) {
    if (!this->io_access_allowed(offset)) {
        return false;
    }

    *res = BYTESWAP_SIZED(this->read_reg(offset - this->aperture_base[1], size), size);
#if 0
    if ((offset - this->aperture_base[1]) < (ATI_GP_IO * 4) ||
        (offset - this->aperture_base[1]) > (ATI_GP_IO * 4) + 4) {
        LOG_F(INFO, "%s: Read: offset=%x, value=%x, size=%x", this->get_name_and_unit_address().c_str(),
            offset - this->aperture_base[1], *res, size
        );
    }
#endif
    return true;
}

bool ATIRage::pci_io_write(uint32_t offset, uint32_t value, uint32_t size) {
    if (!this->io_access_allowed(offset)) {
        return false;
    }

#if 0
    if ((offset - this->aperture_base[1]) < (ATI_GP_IO * 4) ||
        (offset - this->aperture_base[1]) > (ATI_GP_IO * 4) + 4) {
        LOG_F(INFO, "%s: Write: offset=%x, value=%x, size=%x", this->get_name_and_unit_address().c_str(),
            offset - this->aperture_base[1], BYTESWAP_SIZED(value, size), size
        );
    }
#endif
    this->write_reg(offset - this->aperture_base[1], BYTESWAP_SIZED(value, size), size);
    return true;
}

uint32_t ATIRage::read(uint32_t rgn_start, uint32_t offset, int size)
{
    bool swap = true;

    if (rgn_start == this->aperture_base[0] && offset < this->aperture_size[0]) {
        if (offset < this->framebuffer_size) { // little-endian VRAM region
            return read_mem(&this->vram_ptr[offset], size);
        }
        if (offset >= BE_FB_OFFSET) { // big-endian VRAM region

            switch ((this->regs[ATI_MEM_CNTL] >> ATI_UPPER_APER_ENDIAN) & 3) {
                case 0:
                    return read_mem(&this->vram_ptr[offset & (BE_FB_OFFSET - 1)], size);
                case 1:
                    if (size == 4) {
                        uint32_t val = read_mem_rev(&this->vram_ptr[uint64_t(offset) - BE_FB_OFFSET], size);
                        return ((val & 0xFFFF) << 16) | (val >> 16);
                    }
                    break;
            }
            return read_mem_rev(&this->vram_ptr[uint64_t(offset) - BE_FB_OFFSET], size);
        }
        //if (!bit_set(this->regs[ATI_BUS_CNTL], ATI_BUS_APER_REG_DIS)) {
            if (offset >= MM_REGS_0_OFF) { // memory-mapped registers, block 0
                uint32_t value = BYTESWAP_SIZED(this->read_reg(offset & 0x3FF, size), size);
#if 0
                if ((offset & 0x3ff) < (ATI_GP_IO * 4) || (offset & 0x3ff) > (ATI_GP_IO * 4) + 4) {
                    LOG_F(INFO, "%s: Read: offset=%x, value=%x, size=%x", this->get_name_and_unit_address().c_str(),
                        offset & 0x3FF, value, size);
                }
#endif
                return value;
            }
            if (offset >= MM_REGS_1_OFF
                //&& bit_set(this->regs[ATI_BUS_CNTL], ATI_BUS_EXT_REG_EN)
            ) { // memory-mapped registers, block 1
                uint32_t value = BYTESWAP_SIZED(this->read_reg((offset & 0x3FF) + 0x400, size), size);
#if 0
                if (((offset & 0x3ff) + 0x400) < (ATI_GP_IO * 4) || ((offset & 0x3ff) + 0x400) > (ATI_GP_IO * 4) + 4) {
                    LOG_F(INFO, "%s: Read: offset=%x, value=%x, size=%x", this->get_name_and_unit_address().c_str(),
                        (offset & 0x3FF) + 0x400, value, size
                    );
                }
#endif
                return value;
            }
        //}
        LOG_F(WARNING, "%s: read  unmapped aperture[0] region %08x.%c",
              this->get_name_and_unit_address().c_str(), offset, SIZE_ARG(size));
        return 0;
    }

    if (rgn_start == this->aperture_base[2] && offset < this->aperture_size[2]) {
        // The documentation for the Rage LT Pro marks the upper 2KB
        // of the 4KB aux. aperture reserved, but it's used by Mac OS
        // anyway for Rage II. Make it wrap around the 2KB boundary
        // instead.
        offset &= 0x7ff;
        if (offset >= MM_STDL_REGS_0_OFF) {
            return BYTESWAP_SIZED(this->read_reg(offset & 0x3FF, size), size);
        }
        // Rest of the region is Block 1.
        return BYTESWAP_SIZED(this->read_reg((offset & 0x3FF) + 0x400, size), size);
    }

    return PCIBase::read(rgn_start, offset, size);
}

void ATIRage::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size)
{
    value = BYTESWAP_SIZED(value, size);

    if (rgn_start == this->aperture_base[0] && offset < this->aperture_size[0]) {
        if (offset < this->framebuffer_size) { // little-endian VRAM region
            draw_fb = true;
            return write_mem_rev(&this->vram_ptr[offset], value, size);
        }
        if (offset >= BE_FB_OFFSET) { // big-endian VRAM region
            draw_fb = true;
            switch ((this->regs[ATI_MEM_CNTL] >> ATI_UPPER_APER_ENDIAN) & 3) {
                case 0:
                    return write_mem_rev(&this->vram_ptr[offset & (BE_FB_OFFSET - 1)], value, size);
                case 1:
                    if (size == 4) {
                        return write_mem(&this->vram_ptr[offset & (BE_FB_OFFSET - 1)], (((value & 0xFFFF) << 16) | (value >> 16)), size);
                    }
                    break;
            }
            return write_mem(&this->vram_ptr[offset & (BE_FB_OFFSET - 1)], value, size);
        }
        //if (!bit_set(this->regs[ATI_BUS_CNTL], ATI_BUS_APER_REG_DIS)) {
            if (offset >= MM_REGS_0_OFF) { // memory-mapped registers, block 0
#if 0
                if ((offset & 0x3ff) < (ATI_GP_IO * 4) || (offset & 0x3ff) > (ATI_GP_IO * 4) + 4) {
                    LOG_F(INFO, "%s: Write: offset=%x, value=%x, size=%x", this->get_name_and_unit_address().c_str(),
                        offset & 0x3FF, BYTESWAP_SIZED(value, size), size
                    );
                }
#endif
                return this->write_reg(offset & 0x3FF, value, size);
            }
            if (offset >= MM_REGS_1_OFF
                //&& bit_set(this->regs[ATI_BUS_CNTL], ATI_BUS_EXT_REG_EN)
            ) { // memory-mapped registers, block 1
#if 0
                if (((offset & 0x3ff) + 0x400) < (ATI_GP_IO * 4) ||
                    ((offset & 0x3ff) + 0x400) > (ATI_GP_IO * 4) + 4) {
                    LOG_F(INFO, "%s: Write: offset=%x, value=%x, size=%x", this->get_name_and_unit_address().c_str(),
                        (offset & 0x3FF) + 0x400, BYTESWAP_SIZED(value, size), size
                    );
                }
#endif
                return this->write_reg((offset & 0x3FF) + 0x400, value, size);
            }
        //}
        LOG_F(WARNING, "%s: write unmapped aperture[0] region %08x.%c = %0*x",
              this->get_name_and_unit_address().c_str(), offset, SIZE_ARG(size), size * 2, value);
        return;
    }

    if (rgn_start == this->aperture_base[2] && offset < this->aperture_size[2]) {
        // See the note about wrap around above.
        offset &= 0x7ff;
        if (offset >= MM_STDL_REGS_0_OFF) {
            return this->write_reg(offset & 0x3FF, value, size);
        }
        return this->write_reg((offset & 0x3FF) + 0x400, value, size);
    }

    LOG_F(WARNING, "%s: write unmapped aperture region %08x.%c = %0*x",
          this->get_name_and_unit_address().c_str(), offset, SIZE_ARG(size), size * 2, value);
}

float ATIRage::calc_pll_freq(int scale, int fb_div) const {
    return (ATI_XTAL * scale * fb_div) / this->plls[PLL_REF_DIV];
}

void ATIRage::verbose_pixel_format(int crtc_index) {
    if (crtc_index) {
        LOG_F(ERROR, "CRTC2 not supported yet");
        return;
    }

    uint32_t pix_fmt = extract_bits<uint32_t>(this->regs[ATI_CRTC_GEN_CNTL],
                                              ATI_CRTC_PIX_WIDTH, ATI_CRTC_PIX_WIDTH_size);

    const char* what = "Pixel format:";

    switch (pix_fmt) {
    case 1:
        LOG_F(INFO, "%s 4 bpp with DAC palette", what);
        break;
    case 2:
        // check the undocumented DAC_DIRECT bit
        if (bit_set(this->regs[ATI_DAC_CNTL], ATI_DAC_DIRECT)) {
            LOG_F(INFO, "%s 8 bpp direct color (RGB332)", what);
        } else {
            LOG_F(INFO, "%s 8 bpp with DAC palette", what);
        }
        break;
    case 3:
        LOG_F(INFO, "%s 15 bpp direct color (RGB555)", what);
        break;
    case 4:
        LOG_F(INFO, "%s 16 bpp direct color (RGB565)", what);
        break;
    case 5:
        LOG_F(INFO, "%s 24 bpp direct color (RGB888)", what);
        break;
    case 6:
        LOG_F(INFO, "%s 32 bpp direct color (ARGB8888)", what);
        break;
    default:
        LOG_F(ERROR, "%s: CRTC pixel format %d not supported", this->get_name_and_unit_address().c_str(), pix_fmt);
    }
}

void ATIRage::crtc_update() {
    uint32_t new_width, new_height, new_htotal, new_vtotal;

    // check for unsupported modes and fail early
    if (!bit_set(this->regs[ATI_CRTC_GEN_CNTL], ATI_CRTC_EXT_DISP_EN))
        ABORT_F("%s: VGA not supported", this->get_name_and_unit_address().c_str());

    if ((this->plls[PLL_VCLK_CNTL] & 3) != 3)
        ABORT_F("%s: VLCK source != VPLL", this->get_name_and_unit_address().c_str());

    bool need_recalc = false;

    new_width  = (extract_bits<uint32_t>(this->regs[ATI_CRTC_H_TOTAL_DISP],
                                         ATI_CRTC_H_DISP,
                                         ATI_CRTC_H_DISP_size) + 1) * 8;
    new_height = extract_bits<uint32_t>(this->regs[ATI_CRTC_V_TOTAL_DISP],
                                        ATI_CRTC_V_DISP, ATI_CRTC_V_DISP_size) + 1;

    if (new_width != this->active_width || new_height != this->active_height) {
        this->create_display_window(new_width, new_height);
        need_recalc = true;
    }

    new_htotal = (extract_bits<uint32_t>(this->regs[ATI_CRTC_H_TOTAL_DISP],
                                         ATI_CRTC_H_TOTAL,
                                         ATI_CRTC_H_TOTAL_size) + 1) * 8;
    new_vtotal = extract_bits<uint32_t>(this->regs[ATI_CRTC_V_TOTAL_DISP],
                                        ATI_CRTC_V_TOTAL, ATI_CRTC_V_TOTAL_size) + 1;

    if (new_htotal != this->hori_total || new_vtotal != this->vert_total) {
        this->hori_total = new_htotal;
        this->vert_total = new_vtotal;
        need_recalc = true;
    }

    uint32_t new_vert_blank = new_vtotal - new_height;
    if (new_vert_blank != this->vert_blank) {
        this->vert_blank = new_vert_blank;
        need_recalc = true;
    }

    int new_pixel_format = extract_bits<uint32_t>(this->regs[ATI_CRTC_GEN_CNTL],
                                                  ATI_CRTC_PIX_WIDTH,
                                                  ATI_CRTC_PIX_WIDTH_size);
    if (new_pixel_format != this->pixel_format) {
        this->pixel_format = new_pixel_format;
        need_recalc = true;
    }

    static uint8_t bits_per_pixel[8] = {0, 4, 8, 16, 16, 24, 32, 0};

    int new_fb_pitch = extract_bits<uint32_t>(this->regs[ATI_CRTC_OFF_PITCH],
        ATI_CRTC_PITCH, ATI_CRTC_PITCH_size) * bits_per_pixel[this->pixel_format];
    if (new_fb_pitch != this->fb_pitch) {
        this->fb_pitch = new_fb_pitch;
        need_recalc = true;
    }
    uint8_t* new_fb_ptr = &this->vram_ptr[extract_bits<uint32_t>(this->regs[ATI_CRTC_OFF_PITCH],
        ATI_CRTC_OFFSET, ATI_CRTC_OFFSET_size) * 8];
    if (new_fb_ptr != this->fb_ptr) {
        this->fb_ptr = new_fb_ptr;
        need_recalc = true;
    }

    // look up which VPLL ouput is requested
    int clock_sel = extract_bits<uint32_t>(this->regs[ATI_CLOCK_CNTL], ATI_CLOCK_SEL,
                                           ATI_CLOCK_SEL_size);

    // calculate VPLL output frequency
    float vpll_freq = calc_pll_freq(2, this->plls[VCLK0_FB_DIV + clock_sel]);

    // calculate post divider's index
    // NOTE: post divider's index has been extended by an additional
    // bit in Rage Pro. This bit is resided in PLL_EXT_CNTL register.
    int post_div_idx = ((this->plls[PLL_EXT_CNTL] >> (clock_sel + 2)) & 4) |
                       ((this->plls[VCLK_POST_DIV] >> (clock_sel * 2)) & 3);

    // pixel clock = source_freq / post_div
    float new_pixel_clock = vpll_freq / mach64_post_div[post_div_idx];
    if (new_pixel_clock != this->pixel_clock) {
        this->pixel_clock = new_pixel_clock;
        need_recalc = true;
    }

    if (!need_recalc)
        return;

    this->draw_fb = true;
    LOG_F(INFO, "%s: clock_sel:%d fb_div:%02x", this->get_name_and_unit_address().c_str(),
        clock_sel, this->plls[VCLK0_FB_DIV + clock_sel]);

    // calculate display refresh rate
    this->refresh_rate = pixel_clock / this->hori_total / this->vert_total;

    if (this->refresh_rate < 24 || this->refresh_rate > 120) {
        LOG_F(ERROR, "%s: Refresh rate is weird. Will try 60 Hz", this->get_name_and_unit_address().c_str());
        this->refresh_rate = 60;
        this->pixel_clock = this->refresh_rate * this->hori_total / this->vert_total;
    }

    // set up frame buffer converter
    switch (this->pixel_format) {
    case 1:
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_4bpp_indexed(dst_buf, dst_pitch);
        };
        break;
    case 2:
        if (bit_set(this->regs[ATI_DAC_CNTL], ATI_DAC_DIRECT)) {
            this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
                draw_fb = false;
                this->convert_frame_8bpp(dst_buf, dst_pitch);
            };
        }
        else {
            this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
                draw_fb = false;
                this->convert_frame_8bpp_indexed(dst_buf, dst_pitch);
            };
        }
        break;
    case 3:
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_15bpp<LE>(dst_buf, dst_pitch, true);
        };
        break;
    case 4:
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_16bpp<LE>(dst_buf, dst_pitch, true);
        };
        break;
    case 5:
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_24bpp(dst_buf, dst_pitch);
        };
        break;
    case 6:
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_32bpp<LE>(dst_buf, dst_pitch, true);
        };
        break;
    default:
        LOG_F(ERROR, "%s: unsupported pixel format %d", this->get_name_and_unit_address().c_str(), this->pixel_format);
    }

    LOG_F(INFO, "%s: primary CRT controller enabled:", this->get_name_and_unit_address().c_str());
    LOG_F(INFO, "Video mode: %s",
         bit_set(this->regs[ATI_CRTC_GEN_CNTL], ATI_CRTC_EXT_DISP_EN) ? "extended" : "VGA");
    LOG_F(INFO, "Video width: %d px", this->active_width);
    LOG_F(INFO, "Video height: %d px", this->active_height);
    LOG_F(INFO, "Vertical blank: %d px", this->vert_blank);
    verbose_pixel_format(0);
    LOG_F(INFO, "VPLL frequency: %f MHz", vpll_freq * 1e-6);
    LOG_F(INFO, "Pixel (dot) clock: %f MHz", this->pixel_clock * 1e-6);
    LOG_F(INFO, "Refresh rate: %f Hz", this->refresh_rate);
    LOG_F(INFO, "Framebuffer offset: %x",
        extract_bits<uint32_t>(this->regs[ATI_CRTC_OFF_PITCH],
        ATI_CRTC_OFFSET, ATI_CRTC_OFFSET_size) * 8
    );
    LOG_F(INFO, "Framebuffer pitch: %x (%x)",
        extract_bits<uint32_t>(this->regs[ATI_CRTC_OFF_PITCH],
        ATI_CRTC_PITCH, ATI_CRTC_PITCH_size), fb_pitch
    );

    this->stop_refresh_task();
    this->start_refresh_task();

    this->crtc_on = true;
}

void ATIRage::draw_hw_cursor(uint8_t* dst_row, int dst_pitch) {
    int vert_offset = extract_bits<uint32_t>(
        this->regs[ATI_CUR_HORZ_VERT_OFF], ATI_CUR_VERT_OFF, ATI_CUR_VERT_OFF_size);
    int cur_height = 64 - vert_offset;

    uint32_t color0 = this->regs[ATI_CUR_CLR0] | 0x000000FFUL;
    uint32_t color1 = this->regs[ATI_CUR_CLR1] | 0x000000FFUL;

    uint64_t* src_row = (uint64_t*)&this->vram_ptr[this->regs[ATI_CUR_OFFSET] * 8];
    dst_pitch -= 64 * 4;

    for (int h = cur_height; h > 0; h--) {
        for (int x = 2; x > 0; x--) {
            uint64_t px = *src_row++;
            for (int p = 32; p > 0; p--, px >>= 2, dst_row += 4) {
                switch (px & 3) {
                case 0:    // cursor color 0
                    WRITE_DWORD_BE_A(dst_row, color0);
                    break;
                case 1:    // cursor color 1
                    WRITE_DWORD_BE_A(dst_row, color1);
                    break;
                case 2:    // transparent
                    WRITE_DWORD_BE_A(dst_row, 0);
                    break;
                case 3:    // 1's complement of display pixel
                    WRITE_DWORD_BE_A(dst_row, 0x0000007F);
                    break;
                }
            }
        }
        dst_row += dst_pitch;
    }
}

void ATIRage::get_cursor_position(int& x, int& y) {
    x = extract_bits<uint32_t>(this->regs[ATI_CUR_HORZ_VERT_POSN], ATI_CUR_HORZ_POSN, ATI_CUR_HORZ_POSN_size) -
        extract_bits<uint32_t>(this->regs[ATI_CUR_HORZ_VERT_OFF ], ATI_CUR_HORZ_OFF , ATI_CUR_HORZ_OFF_size );
    y = extract_bits<uint32_t>(this->regs[ATI_CUR_HORZ_VERT_POSN], ATI_CUR_VERT_POSN, ATI_CUR_VERT_POSN_size);
}

PostInitResultType ATIRage::device_postinit()
{
    // initialize display identification
    this->disp_id = dynamic_cast<DisplayID*>(this->get_comp_by_type(HWCompType::DISPLAY));

    this->vbl_cb = [this](uint8_t irq_line_state) {
        insert_bits<uint32_t>(this->regs[ATI_CRTC_INT_CNTL], irq_line_state, ATI_CRTC_VBLANK, irq_line_state);
        if (irq_line_state) {
            set_bit(this->regs[ATI_CRTC_INT_CNTL], ATI_CRTC_VBLANK_INT);
            set_bit(this->regs[ATI_CRTC_INT_CNTL], ATI_CRTC_VLINE_INT);
#if 1
#else
            set_bit(this->regs[ATI_CRTC_GEN_CNTL], ATI_CRTC_VSYNC_INT);
#endif
        }

        bool do_interrupt =
            bit_set(this->regs[ATI_CRTC_INT_CNTL], ATI_CRTC_VBLANK_INT_EN) ||
            bit_set(this->regs[ATI_CRTC_INT_CNTL], ATI_CRTC_VLINE_INT_EN) ||
#if 1
#else
            bit_set(this->regs[ATI_CRTC_GEN_CNTL], ATI_CRTC_VSYNC_INT_EN) ||
#endif
            0;

        LOG_F(ATIINTERRUPT, "%s: irq_line_state:%d do_interrupt:%d CRTC_INT_CNTL:%08x",
            this->get_name_and_unit_address().c_str(), irq_line_state, do_interrupt, this->regs[ATI_CRTC_INT_CNTL]);

        if (do_interrupt) {
            this->pci_interrupt(irq_line_state);
        }
    };
    return PI_SUCCESS;
}

void ATIRage::update_display_connection()
{
    uint8_t mon_code = this->disp_id->read_monitor_sense(0, 0);
    this->regs[ATI_GP_IO] = ((mon_code & 6) << 11) | ((mon_code & 1) << 8);
}

// =================================== Draw Engine =====================================
void ATIRage::begin_drawing(uint32_t initiator, uint32_t value) {
    switch(initiator) {
    case ATI_DST_BRES_LNTH:
        this->regs[ATI_DST_BRES_LNTH] = value;
        if (!(value & (1 << 31))) {
            this->draw_line(value & 0x7FFF);
        }
        break;
    case ATI_DST_WIDTH:
        this->regs[ATI_DST_WIDTH] = value;
        this->draw_rect(extract_bits<uint32_t>(value, 0, 14), extract_bits<uint32_t>(this->regs[ATI_DST_HEIGHT], 0, 15));
        break;
    case ATI_DST_WIDTH_HEIGHT:
        this->regs[ATI_DST_WIDTH_HEIGHT] = value;
        this->draw_rect(extract_bits<uint32_t>(value, 0, 14), extract_bits<uint32_t>(value, 16, 15));
        break;
    case ATI_DST_HEIGHT_WIDTH:
        this->regs[ATI_DST_HEIGHT_WIDTH] = value;
        this->draw_rect(extract_bits<uint32_t>(value, 16, 14), extract_bits<uint32_t>(value, 0, 15));
        break;
    default:
        LOG_F(WARNING, "%s: unimplemented engine operation, initiator=0x%X", this->name.c_str(),
              initiator);
    }
}

uint32_t ATIRage::perform_mix_op(uint32_t src, uint32_t dst, uint8_t mix)
{
    switch (mix) {
        case 0:
            return ~dst;
        case 1:
            return 0;
        case 2:
            return ~0u;
        case 3:
            return dst;
        case 4:
            return ~src;
        case 5:
            return dst ^ src;
        case 6:
            return ~dst ^ src;
        case 7:
            return src;
        case 8:
            return ~dst | ~src;
        case 9:
            return dst | ~src;
        case 0xa:
            return ~dst | src;
        case 0xb:
            return dst | src;
        case 0xc:
            return dst & src;
        case 0xd:
            return ~dst & src;
        case 0xe:
            return dst & ~src;
        case 0xf:
            return ~dst & ~src;
        case 0x17:
            return (dst + src) / 2;
        default:
            LOG_F(WARNING, "%s: unknown mix 0x%02X", this->name.c_str(), mix);
    }
    return dst;
}

uint8_t ATIRage::get_bits_per_pel(uint8_t pix_width)
{
    switch (pix_width)
    {
        case 0:
            return 1;
        case 1:
            return 1;
        case 2:
            return 8;
        case 3:
            return 16; // Actually 15bpp, but treated as 16bpp.
        case 4:
        case 15:
            return 16;
        case 5:
            return 32;
        case 6:
        case 11:
        case 14:
            return 32;
        case 7:
        case 8:
            return 8;
        default:
            LOG_F(WARNING, "%s: Unknown pix width %u", this->name.c_str(), pix_width);
            break;
    }
    return 1;
}

void ATIRage::advance_line()
{
    int xsign = (this->regs[ATI_DST_CNTL] & 1) ? 1 : -1;
    int ysign = (this->regs[ATI_DST_CNTL] & 2) ? 1 : -1;
    bool y_major = this->regs[ATI_DST_CNTL] & (1 << 2);

    this->line_pos++;

    if (this->line_pos >= this->line_length) {
        this->line_draw = false;
        return;
    }

    if (y_major) {
        this->dst_x += xsign;
        this->src_x += xsign;

        if (this->bres_error >= 0) {
            this->dst_y += ysign;
            this->src_y += ysign;
            this->bres_error += this->bres_dec;
        } else {
            this->bres_error += this->bres_inc;
        }
    } else {
        this->dst_y += ysign;
        this->src_y += ysign;

        if (this->bres_error >= 0) {
            this->dst_x += xsign;
            this->src_x += xsign;
            this->bres_error += this->bres_dec;
        } else {
            this->bres_error += this->bres_inc;
        }
    }
}

void ATIRage::draw_line(uint32_t length)
{
    uint8_t frgd_src  = extract_bits<uint32_t>(this->regs[ATI_DP_SRC], ATI_DP_FRGD_SRC, ATI_DP_FRGD_SRC_size);
    uint8_t bkgd_src  = extract_bits<uint32_t>(this->regs[ATI_DP_SRC], ATI_DP_BKGD_SRC, ATI_DP_BKGD_SRC_size);
    uint8_t mono_src  = extract_bits<uint32_t>(this->regs[ATI_DP_SRC], ATI_DP_MONO_SRC, ATI_DP_MONO_SRC_size);
    this->line_length = length;
    this->line_pos    = 0;
    this->line_draw   = true;

    this->host_data_active = false;
    this->host_data_pos = 0;
    this->dst_width = this->regs[ATI_DST_WIDTH];
    this->dst_height = this->regs[ATI_DST_HEIGHT];
    this->dst_x = this->regs[ATI_DST_X];
    this->dst_y = this->regs[ATI_DST_Y];
    this->dst_x_start = 0;
    this->dst_y_start = 0;

    if (this->dst_x & 0x1000) {
        this->dst_x |= ~0xFFF;
    } 
    if (this->dst_y & 0x4000) {
        this->dst_y |= ~0x3FFF;
    }

    this->src_x = this->regs[ATI_SRC_X] & 0x1fff;
    this->src_y = this->regs[ATI_SRC_Y] & 0x7fff;
    if (this->src_x & 0x1000) {
        this->src_x |= ~0xFFF;
    } 
    if (this->src_y & 0x4000) {
        this->src_y |= ~0x3FFF;
    }
    this->src_x_start = 0;
    this->src_y_start = 0;

    this->bres_inc   = this->regs[ATI_DST_BRES_INC];
    this->bres_dec   = this->regs[ATI_DST_BRES_DEC];
    this->bres_error = this->regs[ATI_DST_BRES_ERR];

    if (this->bres_error & 0x40000) {
        this->bres_error |= ~0x3ffff;
    }

    if (this->bres_dec & 0x40000) {
        this->bres_dec |= ~0x3ffff;
    }
    
    if (frgd_src != 2 && bkgd_src != 2 && mono_src != 2) {
        while (this->line_draw) {
            uint32_t pix = 0;
            uint8_t mix = 3;
            bool draw = !(this->regs[ATI_DST_CNTL] & (1 << 6)) || ((this->regs[ATI_DST_CNTL] & (1 << 2)) || (this->bres_error >= 0));

            if ((line_length - line_pos) <= 1) {
                draw = draw && (this->regs[ATI_DST_CNTL] & (1 << 5));
            }

            if (draw) {
                pix = fetch_source(src_x, src_y, mix);

                process_pixel(pix, dst_x, dst_y, mix);
            }
            advance_line();
        }
    } else {
        uint8_t host_pix_fmt = extract_bits<uint32_t>(this->regs[ATI_DP_PIX_WIDTH], ATI_DP_HOST_PIX_WIDTH, ATI_DP_HOST_PIX_WIDTH_size);
        this->host_data_active = 1;
        this->host_data_pos = 0;
        this->host_data_req = (mono_src == 2) ? 1 : get_bits_per_pel(host_pix_fmt);
        LOG_F(WARNING, "%s: unimplemented rectangle draw op, DP_FRGD_SRC=0x%X, DP_MONO_SRC=0x%X", this->name.c_str(),
              frgd_src, mono_src);
    }
}

void ATIRage::draw_rect(uint32_t width, uint32_t height)
{
    uint8_t frgd_src           = extract_bits<uint32_t>(this->regs[ATI_DP_SRC], ATI_DP_FRGD_SRC, ATI_DP_FRGD_SRC_size);
    uint8_t bkgd_src           = extract_bits<uint32_t>(this->regs[ATI_DP_SRC], ATI_DP_BKGD_SRC, ATI_DP_BKGD_SRC_size);
    uint8_t mono_src           = extract_bits<uint32_t>(this->regs[ATI_DP_SRC], ATI_DP_MONO_SRC, ATI_DP_MONO_SRC_size);
    uint8_t src_block_fill_fcn = extract_bits<uint32_t>(this->regs[ATI_SRC_CNTL], ATI_SRC_BLOCK_FILL_FCN, ATI_SRC_BLOCK_FILL_FCN_size);

    this->regs[ATI_DST_HEIGHT] = height;
    this->regs[ATI_DST_WIDTH] = width;
    this->host_data_active = false;
    this->host_data_pos = 0;

    // Do nothing if SGRAM color registers are to be written into.
    if (src_block_fill_fcn == 1) {
        // ...but honor DST_X_TILE and DST_Y_TILE anyways.
        int dst_x = extract_bits<uint32_t>(this->regs[ATI_DST_X], ATI_DST_X_pos, ATI_DST_X_size);
        int dst_y = extract_bits<uint32_t>(this->regs[ATI_DST_Y], ATI_DST_Y_pos, ATI_DST_Y_size);
        int xsign = (this->regs[ATI_DST_CNTL] & 1) ? 1 : -1;
        int ysign = (this->regs[ATI_DST_CNTL] & 2) ? 1 : -1;
        
        if (bit_set(this->regs[ATI_DST_CNTL], 3)) {
            dst_x += dst_width * xsign;
        }
        if (bit_set(this->regs[ATI_DST_CNTL], 4)) {
            dst_y += dst_height * ysign;
        }

        insert_bits<uint32_t>(this->regs[ATI_DST_X], dst_x, ATI_DST_X_pos, ATI_DST_X_size);
        insert_bits<uint32_t>(this->regs[ATI_DST_Y], dst_y, ATI_DST_Y_pos, ATI_DST_Y_size);
        return;
    }

    this->src_width = this->regs[ATI_SRC_WIDTH1];
    this->src_height = this->regs[ATI_SRC_HEIGHT1];
    this->src_x = 0;
    this->src_y = 0;
    this->src_x_start = this->regs[ATI_SRC_X] & 0x1fff;
    this->src_y_start = this->regs[ATI_SRC_Y] & 0x7fff;
    this->line_draw   = false;
    
    this->poly_draw_chk = (this->regs[ATI_DST_CNTL] & (1 << 6));
    this->poly_draw_flip = false;

    if (this->src_x_start & 0x1000) {
        this->src_x_start |= ~0xFFF;
    } 
    if (this->src_y_start & 0x4000) {
        this->src_y_start |= ~0x3FFF;
    }

    if (bit_set(this->regs[ATI_SRC_CNTL], 2)) {
        this->src_x_start = 0;
        this->src_y_start = 0;
    }

    this->dst_width = this->regs[ATI_DST_WIDTH];
    this->dst_height = this->regs[ATI_DST_HEIGHT];
    this->dst_x = 0;
    this->dst_y = 0;
    this->dst_x_start = this->regs[ATI_DST_X];
    this->dst_y_start = this->regs[ATI_DST_Y];

    if (this->dst_x_start & 0x1000) {
        this->dst_x_start |= ~0xFFF;
    } 
    if (this->dst_y_start & 0x4000) {
        this->dst_y_start |= ~0x3FFF;
    }

    if (mono_src == 2 && ((this->regs[ATI_SRC_CNTL] & 0xc) >> 2) == 3) {
        this->dst_width = (this->dst_width + 8) & ~7;
    }

    if (frgd_src != 2 && bkgd_src != 2 && mono_src != 2) {
        this->blit_rect(width, height);
    } else {
        uint8_t host_pix_fmt = extract_bits<uint32_t>(this->regs[ATI_DP_PIX_WIDTH], ATI_DP_HOST_PIX_WIDTH, ATI_DP_HOST_PIX_WIDTH_size);
        this->host_data_active = 1;
        this->host_data_pos = 0;
        this->host_data_req = (mono_src == 2) ? 1 : get_bits_per_pel(host_pix_fmt);
        this->host_data = 0;
    }
}

void ATIRage::process_host_data()
{
    int xsign = (this->regs[ATI_DST_CNTL] & 1) ? 1 : -1;
    int ysign = (this->regs[ATI_DST_CNTL] & 2) ? 1 : -1;
    uint8_t  host_pix_fmt = extract_bits<uint32_t>(this->regs[ATI_DP_PIX_WIDTH], ATI_DP_HOST_PIX_WIDTH,
                                                 ATI_DP_HOST_PIX_WIDTH_size);
    uint32_t bits = get_bits_per_pel(host_pix_fmt);
    if (this->host_data_req == 1) {
        while (this->host_data_pos) {
            uint8_t mix = 3;
            auto pix = fetch_source(this->src_x_start + (src_x * xsign), this->src_y_start + (src_y * ysign), mix);
            bool draw = !(this->regs[ATI_DST_CNTL] & (1 << 6)) || ((this->regs[ATI_DST_CNTL] & (1 << 2)) || (this->bres_error >= 0));

            if (this->line_draw && (line_length - line_pos) <= 1) {
                draw = draw && (this->regs[ATI_DST_CNTL] & (1 << 5));
            }

            if (!this->line_draw) {
                draw = !this->poly_draw_chk;
                if (this->poly_draw_chk) {
                    uint8_t dummy_pix = 0;
                    uint32_t poly_pix = fetch_source(this->src_x_start + (src_x * xsign), this->src_y_start + (src_y * ysign), dummy_pix, true);
                    if (poly_pix) {
                        this->poly_draw_flip ^= 1;
                    }
                    draw = this->poly_draw_flip;
                }
            }

            if (draw) {
                auto pix = fetch_source(this->src_x_start + (src_x * xsign), this->src_y_start + (src_y * ysign), mix);

                process_pixel(pix, dst_x_start + (dst_x * xsign), dst_y_start + (dst_y * ysign), mix);
            }
            
            this->host_data = !(this->regs[ATI_DP_PIX_WIDTH] & (1 << ATI_DP_BYTE_PIX_ORDER)) ? (this->host_data << 1) : (this->host_data >> 1);
            if (this->regs[ATI_HOST_CNTL] & 1) {
                this->host_data = !(this->regs[ATI_DP_PIX_WIDTH] & (1 << ATI_DP_BYTE_PIX_ORDER)) ? (this->host_data << 7) : (this->host_data >> 7);
            }
            host_data_pos--;
            if (this->regs[ATI_HOST_CNTL] & 1) {
                host_data_pos -= 7;
            }

            if (this->line_draw) {
                advance_line();
                if (!this->line_draw) {
                    this->host_data_active = false;
                    return;
                }
            } else {
                advance_source_x();
                dst_x++;
                if (dst_x >= dst_width) {
                    dst_x = 0;
                    dst_y++;
                    advance_source_y();
                    if (dst_y >= dst_height) {
                        this->host_data_active = false;
                        return;
                    }
                }
            }
        }
    } else {
        while (this->host_data_pos) {
            uint8_t mix = 3;
            bool draw = !(this->regs[ATI_DST_CNTL] & (1 << 6)) || ((this->regs[ATI_DST_CNTL] & (1 << 2)) || (this->bres_error >= 0));

            if (this->line_draw && (line_length - line_pos) <= 1) {
                draw = draw && (this->regs[ATI_DST_CNTL] & (1 << 5));
            }

            if (!this->line_draw) {
                draw = !this->poly_draw_chk;
                if (this->poly_draw_chk) {
                    uint8_t dummy_pix = 0;
                    uint32_t poly_pix = fetch_source(this->src_x_start + (src_x * xsign), this->src_y_start + (src_y * ysign), dummy_pix, true);
                    if (poly_pix) {
                        this->poly_draw_flip ^= 1;
                    }
                    draw = this->poly_draw_flip;
                }
            }

            auto pix = fetch_source(this->src_x_start + (src_x * xsign), this->src_y_start + (src_y * ysign), mix);

            process_pixel(pix, dst_x_start + (dst_x * xsign), dst_y_start + (dst_y * ysign), mix);

            this->host_data_pos -= this->host_data_req;
            this->host_data >>= this->host_data_req;
            
            if (this->line_draw) {
                advance_line();
                if (!this->line_draw) {
                    this->host_data_active = false;
                    return;
                }
            } else {
                advance_source_x();
                dst_x++;
                if (dst_x >= dst_width) {
                    dst_x = 0;
                    dst_y++;
                    advance_source_y();
                    if (dst_y >= dst_height) {
                        this->host_data_active = false;
                        return;
                    }
                }
            }
        }
    }
}

uint32_t ATIRage::fetch_source(int32_t s_x, int32_t s_y, uint8_t& mix, bool force_blitsrc)
{
    /*
    Monochrome sources do not have a separate pitch and offset register.
    If those are used, non-monochrome sources can't use data from memory
    unless those are also 1-bit.
    */
    uint32_t pix         = 0;
    uint8_t  src_sel     = 1; // foreground.
    uint8_t  frgd_src    = extract_bits<uint32_t>(this->regs[ATI_DP_SRC], ATI_DP_FRGD_SRC, ATI_DP_FRGD_SRC_size);
    uint8_t  mono_src    = extract_bits<uint32_t>(this->regs[ATI_DP_SRC], ATI_DP_MONO_SRC, ATI_DP_MONO_SRC_size);
    uint8_t  bkgd_src    = extract_bits<uint32_t>(this->regs[ATI_DP_SRC], ATI_DP_BKGD_SRC, ATI_DP_BKGD_SRC_size);
    uint8_t  frgd_mix    = extract_bits<uint32_t>(this->regs[ATI_DP_MIX], ATI_DP_FRGD_MIX, ATI_DP_FRGD_MIX_size);
    uint8_t  bkgd_mix    = extract_bits<uint32_t>(this->regs[ATI_DP_MIX], ATI_DP_BKGD_MIX, ATI_DP_BKGD_MIX_size);
    uint8_t  src_pix_fmt = extract_bits<uint32_t>(this->regs[ATI_DP_PIX_WIDTH], ATI_DP_SRC_PIX_WIDTH,
                                                 ATI_DP_SRC_PIX_WIDTH_size);
    // grab trajectory params
    int src_offs   = extract_bits<uint32_t>(this->regs[ATI_SRC_OFF_PITCH], ATI_SRC_OFFSET, ATI_SRC_OFFSET_size);
    int src_pitch  = extract_bits<uint32_t>(this->regs[ATI_SRC_OFF_PITCH], ATI_SRC_PITCH, ATI_SRC_PITCH_size);
    src_offs      *= 8;
    src_pitch     *= get_bits_per_pel(src_pix_fmt);

    auto src_ptr = &this->vram_ptr[src_offs];
    mix = frgd_mix;

    if (force_blitsrc)
        mono_src = 0;

    if (mono_src != 0) {
        switch (mono_src) {
            case 1: {
                uint64_t mono_val = regs[ATI_PAT_REG0] | ((uint64_t)regs[ATI_PAT_REG1]) << 32ull;
                src_sel = mono_val & !!(1ull << (((s_y & 7) * 8) + ((this->regs[ATI_DP_PIX_WIDTH] & (1 << ATI_DP_BYTE_PIX_ORDER)) ? (7 - (s_x & 7)) : (s_x & 7))));
                break;
            }
            case 2: {
                src_sel = !(this->regs[ATI_DP_PIX_WIDTH] & (1 << ATI_DP_BYTE_PIX_ORDER)) ? (this->host_data >> 31) : (this->host_data & 1);
                break;
            }
            case 3: {
                uint64_t bit_offset = ((bit_set(this->regs[ATI_SRC_CNTL], 2) ? 0 : (uint64_t)s_y * (uint64_t)src_pitch)) + (((this->regs[ATI_DP_PIX_WIDTH] & (1 << ATI_DP_BYTE_PIX_ORDER)) ? s_x ^ 7 : s_x));
                auto mono_ptr = &src_ptr[bit_offset / 8];
                src_sel = *mono_ptr & (bit_offset & (1 << (bit_offset & 7)));
                break;
            }
        }
    }

    mix = src_sel ? frgd_mix : bkgd_mix;
    switch (force_blitsrc ? 3 : (src_sel ? frgd_src : bkgd_src)) {
        case 0: {
            pix = this->regs[ATI_DP_BKGD_CLR];
            break;
        }
        case 1: {
            pix = this->regs[ATI_DP_FRGD_CLR];
            break;
        }
        case 2: {
            pix = this->host_data;
            break;
        }
        case 4: {
            if (this->regs[ATI_PAT_CNTL] & 2) {
                pix = (((s_y & 1) ? this->regs[ATI_PAT_REG1] : this->regs[ATI_PAT_REG0]) >> (8 * (s_x & 3))) & 0xFF;
            } else if (this->regs[ATI_PAT_CNTL] & 4) {
                pix = (((s_x & 4) ? this->regs[ATI_PAT_REG1] : this->regs[ATI_PAT_REG0]) >> (8 * (s_x & 3))) & 0xFF;
            }
            break;
        }
        case 3: {
            s_x *= get_bits_per_pel(src_pix_fmt) / 8;
            auto src = &src_ptr[((bit_set(this->regs[ATI_SRC_CNTL], 2) ? 0 : s_y * src_pitch) + s_x)];
            switch (src_pix_fmt) {
                case 2:
                case 8:
                case 7:
                {
                    pix = *src;
                    break;
                }
                case 3:
                {
                    pix = *(uint16_t*)src;
                    break;
                }
                case 6:
                case 14:
                {
                    pix = *(uint32_t*)src;
                    break;
                }
            }
        }
    }
    return pix;
}

void ATIRage::process_pixel(uint32_t pix, int dst_x, int dst_y, uint8_t mix)
{
    uint8_t dst_pix_fmt = extract_bits<uint32_t>(this->regs[ATI_DP_PIX_WIDTH], ATI_DP_DST_PIX_WIDTH,
                                                 ATI_DP_DST_PIX_WIDTH_size);
    uint32_t dst_pix    = 0;
    bool     clr_cmp    = false;

    // grab trajectory params
    int dst_offs   = extract_bits<uint32_t>(this->regs[ATI_DST_OFF_PITCH], ATI_DST_OFFSET, ATI_DST_OFFSET_size);
    int dst_pitch  = extract_bits<uint32_t>(this->regs[ATI_DST_OFF_PITCH], ATI_DST_PITCH, ATI_DST_PITCH_size);

    int sc_left    = extract_bits<uint32_t>(this->regs[ATI_SC_LEFT], ATI_SC_LEFT_pos, ATI_SC_LEFT_size);
    int sc_right   = extract_bits<uint32_t>(this->regs[ATI_SC_RIGHT], ATI_SC_RIGHT_pos, ATI_SC_RIGHT_size);
    int sc_top     = extract_bits<uint32_t>(this->regs[ATI_SC_TOP], ATI_SC_TOP_pos, ATI_SC_TOP_size);
    int sc_bottom  = extract_bits<uint32_t>(this->regs[ATI_SC_BOTTOM], ATI_SC_BOTTOM_pos, ATI_SC_BOTTOM_size);

    dst_offs  *= 8;
    dst_pitch *= get_bits_per_pel(dst_pix_fmt);
    pix &= (1ull << get_bits_per_pel(dst_pix_fmt)) - 1;

    int x_inc = 1;

    if (!(dst_x >= sc_left && dst_x <= sc_right && dst_y >= sc_top && dst_y <= sc_bottom)) {
        return;
    }
    uint32_t write_msk = this->regs[ATI_DP_WRITE_MSK];

    dst_offs += dst_y * dst_pitch;

    x_inc *= get_bits_per_pel(dst_pix_fmt) / 8;

    auto dst_ptr = &this->vram_ptr[(dst_offs + dst_x * x_inc) % this->vram_size];
    switch (dst_pix_fmt) {
        case 2:
        case 8:
        case 7:
        {
            dst_pix = *dst_ptr;
            break;
        }
        case 3:
        {
            dst_pix = *(uint16_t*)dst_ptr;
            break;
        }
        case 6:
        case 14:
        {
            dst_pix = *(uint32_t*)dst_ptr;
            break;
        }
    }
    
    // We don't test for 3D texels here.
    uint32_t cmp_pix  = this->regs[ATI_CLR_CMP_CLR] & this->regs[ATI_CLR_CMP_MSK];
    uint32_t cmpd_pix = (bit_set(this->regs[ATI_CLR_CMP_CNTL], ATI_CLR_CMP_SRC) ? pix : dst_pix) & this->regs[ATI_CLR_CMP_MSK];

    switch (extract_bits(this->regs[ATI_CLR_CMP_CNTL], ATI_CLR_CMP_FCN, ATI_CLR_CMP_FCN_size)) {
        case 1:
            clr_cmp = true;
            break;
        case 4:
            clr_cmp = !!(cmp_pix == cmpd_pix);
            break;
        case 5:
            clr_cmp = !!(cmp_pix != cmpd_pix);
            break;
    }

    if (clr_cmp)
        return;

    dst_pix = (dst_pix & ~write_msk) | (perform_mix_op(pix, dst_pix, mix) & write_msk);
    switch (dst_pix_fmt) {
        case 2:
        case 8:
        case 7:
        {
            *dst_ptr = dst_pix;
            break;
        }
        case 3:
        {
            *(uint16_t*)dst_ptr = dst_pix;
            break;
        }
        case 6:
        case 14:
        {
            *(uint32_t*)dst_ptr = dst_pix;
            break;
        }
    }
}

void ATIRage::advance_source_x()
{
    bool pattern_rotation = (this->regs[ATI_SRC_CNTL] & 2) && (this->regs[ATI_SRC_CNTL] & 1); // both SRC_PATT_EN and SRC_PATT_ROTT_EN set
    src_x++;
    src_width--;
    if (src_width <= 0 && !(this->regs[ATI_SRC_CNTL] & (1 << 2))) {
        src_x = 0;
        if (pattern_rotation) {
            src_x_start = this->regs[ATI_SRC_X_START];
            if (src_x_start & (1 << 12)) {
                src_x_start |= ~0xFFF;
            }
            src_width = this->regs[ATI_SRC_WIDTH2];
        }
    }
}

void ATIRage::advance_source_y()
{
    bool pattern_rotation = (this->regs[ATI_SRC_CNTL] & 2) && (this->regs[ATI_SRC_CNTL] & 1); // both SRC_PATT_EN and SRC_PATT_ROTT_EN set
    if (bit_set(this->regs[ATI_SRC_CNTL], 2))
        return;
    src_x_start = this->regs[ATI_SRC_X];
    src_x = 0;
    src_width = this->regs[ATI_SRC_WIDTH1];

    if (this->src_x_start & 0x1000) {
        this->src_x_start |= ~0xFFF;
    } 

    src_y++;
    if (src_y >= src_height && (this->regs[ATI_SRC_CNTL] & 1)) {
        src_y = 0;
        if (pattern_rotation) {
            src_y_start = this->regs[ATI_SRC_Y_START];
            if (src_y_start & (1 << 14)) {
                src_y_start |= ~0x3FFF;
            }
            src_height = this->regs[ATI_SRC_HEIGHT2];
        }
    }
}

void ATIRage::blit_rect(uint32_t dst_width, uint32_t dst_height)
{
    // grab trajectory params
    int dst_x = extract_bits<uint32_t>(this->regs[ATI_DST_X], ATI_DST_X_pos, ATI_DST_X_size);
    int dst_y = extract_bits<uint32_t>(this->regs[ATI_DST_Y], ATI_DST_Y_pos, ATI_DST_Y_size);
    int xsign = (this->regs[ATI_DST_CNTL] & 1) ? 1 : -1;
    int ysign = (this->regs[ATI_DST_CNTL] & 2) ? 1 : -1;

    if (dst_x & 0x1000) {
        dst_x |= ~0xFFF;
    } 
    if (dst_y & 0x4000) {
        dst_y |= ~0x3FFF;
    }

    for (int y = 0; y < dst_height; y++) {
        for (int x = 0; x < dst_width; x++) {
            int xx = (x * xsign) + dst_x;
            int yy = (y * ysign) + dst_y;
            bool draw = !(this->poly_draw_chk);
            uint32_t pix = 0;
            uint8_t mix = 3;

            if (this->poly_draw_chk) {
                uint8_t dummy_pix = 0;
                uint32_t poly_pix = fetch_source(this->src_x_start + (src_x * xsign), this->src_y_start + (src_y * ysign), dummy_pix, true);
                if (poly_pix) {
                    this->poly_draw_flip ^= 1;
                }
                draw = this->poly_draw_flip;
            }

            if (draw) {
                pix = fetch_source(this->src_x_start + (src_x * xsign), this->src_y_start + (src_y * ysign), mix);

                process_pixel(pix, xx, yy, mix);
            }
            advance_source_x();
        }
        advance_source_y();
    }

    if (bit_set(this->regs[ATI_DST_CNTL], 3)) {
        dst_x += dst_width * xsign;
    }
    if (bit_set(this->regs[ATI_DST_CNTL], 4)) {
        dst_y += dst_height * ysign;
    }

    insert_bits<uint32_t>(this->regs[ATI_DST_X], dst_x, ATI_DST_X_pos, ATI_DST_X_size);
    insert_bits<uint32_t>(this->regs[ATI_DST_Y], dst_y, ATI_DST_Y_pos, ATI_DST_Y_size);
}

// ================================== Device config ====================================

static const PropMap AtiRage_Properties = {
    {"gfxmem_size",
        new IntProperty(2, std::vector<uint32_t>({2, 4, 6, 8}))},
    {"rom",
        new StrProperty("")},
};

static const DeviceDescription AtiRage_Descriptor = {
    ATIRage::create, {"Display@0"}, AtiRage_Properties, HWCompType::MMIO_DEV | HWCompType::PCI_DEV | HWCompType::VIDEO_CTRL
};

REGISTER_DEVICE(AtiRageGT, AtiRage_Descriptor);
REGISTER_DEVICE(AtiRageGW, AtiRage_Descriptor);
REGISTER_DEVICE(AtiRagePro, AtiRage_Descriptor);
