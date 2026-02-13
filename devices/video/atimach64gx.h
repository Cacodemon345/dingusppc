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

/** ATI Mach64 GX definitions. */

#ifndef ATI_MACH64_GX_H
#define ATI_MACH64_GX_H

#include <devices/common/pci/pcidevice.h>
#include <devices/video/displayid.h>
#include <devices/video/videoctrl.h>
#include <devices/video/atimach64defs.h>

#include <cinttypes>
#include <memory>

class AtiMach64Gx : public PCIVideoCtrl {
public:
    AtiMach64Gx(const std::string &dev_name);
    ~AtiMach64Gx() = default;

    static std::unique_ptr<HWComponent> create(const std::string &dev_name) {
        return std::unique_ptr<AtiMach64Gx>(new AtiMach64Gx(dev_name));
    }

    // PCI device methods
    bool supports_io_space(void) override {
        return true;
    }

    // I/O space access methods
    bool pci_io_read(uint32_t offset, uint32_t size, uint32_t* res) override;
    bool pci_io_write(uint32_t offset, uint32_t value, uint32_t size) override;

    // HWComponent methods
    PostInitResultType device_postinit() override;
    HWComponent* set_property(const std::string &property, const std::string &value, int32_t unit_address = -1) override;

    // MMIODevice methods
    uint32_t read(uint32_t rgn_start, uint32_t offset, int size) override;
    void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) override;

protected:
    void notify_bar_change(int bar_num);
    const char* get_reg_name(uint32_t reg_offset);
    const char* rgb514_get_reg_name(uint32_t reg_offset);
    const char* rgb514_get_ind_reg_name(uint32_t reg_offset);
    bool io_access_allowed(uint32_t offset);
    uint32_t read_reg(uint32_t reg_offset, uint32_t size);
    void write_reg(uint32_t reg_offset, uint32_t value, uint32_t size);
    void crtc_update();
    uint8_t rgb514_read_reg(uint8_t reg_addr);
    void rgb514_write_reg(uint8_t reg_addr, uint8_t value);
    uint8_t rgb514_read_ind_reg(uint8_t reg_addr);
    void rgb514_write_ind_reg(uint8_t reg_addr, uint8_t value);
    void verbose_pixel_format(int crtc_index);
    void draw_hw_cursor(uint8_t *dst_buf, int dst_pitch) override;
    void get_cursor_position(int& x, int& y) override;

private:
    void change_one_bar(uint32_t &aperture, uint32_t aperture_size, uint32_t aperture_new, int bar_num);

    uint32_t perform_mix_op(uint32_t src, uint32_t dst, uint8_t mix);
    void begin_drawing(uint32_t initiator, uint32_t value);
    void draw_rect(uint32_t width, uint32_t height);
    void draw_line(uint32_t length);
    void advance_line();
    void advance_source_x();
    void advance_source_y();
    void blit_rect(uint32_t dst_width, uint32_t dst_height);
    void process_host_data(uint64_t pixel, uint8_t size);
    void process_pixel(uint32_t pix, int dst_x, int dst_y, uint8_t mix);
    uint32_t fetch_source(int32_t src_x, int32_t src_y, uint8_t& mix, bool force_blitsrc = false);
    uint8_t get_bits_per_pel(uint8_t pix_width);

    uint32_t    regs[256] = {}; // internal registers

    int         vram_size;

    // main aperture (16MB)
    const uint32_t aperture_count = 1;
    const uint32_t aperture_size[1] = { 0x1000000 };
    const uint32_t aperture_flag[1] = { 0 };
    uint32_t aperture_base[1] = { 0 };

    uint32_t    config_cntl[2] = { 2, 0 };
    uint32_t    mm_regs_offset = MM_REGS_0_OFF;

    // RGB514 RAMDAC state
    uint8_t     dac_idx_lo = 0;
    uint8_t     dac_idx_hi = 0;

    uint8_t     clut_index = 0;
    uint8_t     comp_index = 0;
    uint8_t     clut_color[3] = {0};

    uint8_t     clut_index_rd = 0;
    uint8_t     comp_index_rd = 0;
    uint8_t     clut_color_rd[3] = {0};

    uint8_t     dac_regs[256] = {0};

    DisplayID*                  disp_id = nullptr;
    std::unique_ptr<uint8_t[]>  vram_ptr;

    // Accel state (for host-driven blits)
    int32_t  dst_x_start;
    int32_t  dst_y_start;
    int32_t  dst_x;
    int32_t  dst_y;
    uint32_t dst_width;
    uint32_t dst_height;

    int32_t  src_x;
    int32_t  src_y;
    int32_t  src_x_start;
    int32_t  src_y_start;
    uint32_t src_width;
    uint32_t src_height;

    uint64_t host_data;
    uint8_t  host_data_active;
    uint32_t host_data_req_bytes;
    uint32_t host_data_req_recv;

    int      line_length  = 0;
    int      line_pos     = 0;
    uint32_t bres_error   = 0;
    uint32_t bres_inc     = 0;
    uint32_t bres_dec     = 0;
    uint32_t host_skip    = 0;
    bool poly_draw_chk    = false;
    bool poly_draw_flip   = false;
    bool line_draw        = false;
    bool nonmono_host     = false;
    bool prev_host_data   = false;
};

#endif // ATI_MACH64_GX_H
