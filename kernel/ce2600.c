//-----------------------------------------------------------------------------
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// GPL LICENSE SUMMARY
//
// Copyright(c) 2013 Intel Corporation. All rights reserved.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of version 2 of the GNU General Public License as
// published by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
// The full GNU General Public License is included in this distribution
// in the file called LICENSE.GPL.
//
// Contact Information:
//      Intel Corporation
//      2200 Mission College Blvd.
//      Santa Clara, CA  97052
//
// BSD LICENSE
//
// Copyright(c) 2013 Intel Corporation. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   - Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   - Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//   - Neither the name of Intel Corporation nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//----------------------------------------------------------------------------

//******************************************************************************
//          P O W E R   M A N A G E M E N T   F O R   C E 2 6 0 0
//
// Note that on CE2600 there are no power islands we can control so the CPU and
// devices are NEVER ACTUALLY POWERED DOWN.  The CEFDK/PUnit collaborate to put
// the CPU into C6 after the Linux STR sequence is complete.
//
// The STR sequence is initiated by a Remote Procedure call from the NP-CPU, and
// the only wake event is a message (register write) from the NP-CPU to the
// PUnit, which causes the PUnit to interrupt the APP-CPU (Atom) out of C6.
//
// Since the CPU is not powered off/on, resume code does not begin with the
// reset vector; therefore, the current CEFDK implementation (12-Nov-2012) does
// NOT execute the beginning of the boot sequence, and we have to do a lot more
// work to re-initialize devices like PCIe, SATA, and USB.
//******************************************************************************
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "osal.h"
#include "clock_control.h"

#include "icepm_internal.h"
#include "kernel.h"

// true iff we are running on an A-step CE2600.
static bool is_A_step = false;

static uint32_t sata_mem_bar = 0;

//==============================================================================
//          B O A R D - S P E C I F I C   G P I O s 
//==============================================================================

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!! CHANGE BOARD-SPECIFIC INITIALIZATION IN ce2600_init() IF NECESSARY !!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
static int32_t wifi_power_gpio         = -1; // -1 => GPIO not present on board.
static int32_t sata_power_gpio         = -1; // -1 => GPIO not present on board.
static int32_t pcie_client_reset_gpio  = -1; // -1 => GPIO not present on board.


//==============================================================================
//        C L O C K   C O N T R O L   R E S O U R C E S
//==============================================================================

// Resources controlled through the clock_control component to gate/reset
// the devices controlled by each driver.

// See mspod_clocks, below, for why mspod clock/reset is included here.
static clock_res_t demux_clocks[] =
{
    CLOCK( TSD_CLK_EN,                  1, 0 ),
    CLOCK( TSD_SRAM_CLK_GATE_BYPASS,    1, 0 ),
    CLOCK( TSD_SUB_UNIT_CLK_GATE_BYPASS,1, 0 ),
    CLOCK( PREFILTER_XSI_CLK_EN,        1, 0 ),
    CLOCK( MSPOD_XSI_CLK_EN,            1, 0 ),
    RESET( MSPOD_RST,                   1, 0 ),
    RESET( PF_RST,                      1, 0 ),
    { NULL }
};

static clock_res_t gbe_clocks[] =
{
    CLOCK( RMII_REFCLK_50_CLK_EN,       1, 0 ),
    CLOCK( PAD_GBE_REF_CLK_EN,          1, 0 ),
    CLOCK( PAD_GBE_IN_CLK_EN,           1, 0 ),
    CLOCK( GBE_XSI_CLK_EN,              1, 0 ),
    CLOCK( GBE_HCLK_EN,                 1, 0 ),
    CLOCK( GBE_SRAM_CLK_GATE_BYPASS,    1, 0 ),
    // Do not touch INT_GBE_REF_CLK_EN. It will be controlled from the NP-CPU.
    // NP-CPU guarantees it will be enabled when we are not suspended.
    //CLOCK( INT_GBE_REF_CLK_EN,          1, 0 ),
    CLOCK( GBE_DDS_HREF_CLK_EN,         1, 0 ),
    RESET( GBE_RST,                     1, 0 ),
    { NULL },
};

static clock_res_t ismdclock_clocks[]   =
{
    RESET( CRU_RST,                     1, 0 ),
    { NULL }
};

static clock_res_t mspod_clocks[] =
{
// mspod driver may not be loaded.  If so, we will not get pci_set_power_state()
// callbacks for it.  Handle its clocks as if they were part of demux (above)
//  CLOCK( MSPOD_XSI_CLK_EN,            1, 0 ),
//  RESET( MSPOD_RST,                   1, 0 ),
    { NULL },
};

// pcie1/pcie2 share clocks/resets
// - gate/reset only after *BOTH* are suspendend
// - ungate/run when *FIRST* is resumed
static clock_res_t pcie_clocks[] =
{
    CLOCK( PCIE_BBCLK_EN,               1, 0 ),
    CLOCK( PCIE_IGCLK_EN,               1, 0 ),
    RESET( BBRST_B,                     1, 0 ),
    RESET( SBI_100_RST_CORE_B,          1, 0 ),
    RESET( SBI_BB_RST_B,                1, 0 ),
    RESET( PCIE_PHY_CMN_RESET,          1, 0 ),
    { NULL }
};
#define pcie1_clocks pcie_clocks
#define pcie2_clocks pcie_clocks

static clock_res_t sata_clocks[] =
{
    CLOCK( SATA_CLK_EN,                 1, 0 ),
    CLOCK( SATA_PLL_CLK_EN,             1, 0 ),
    RESET( SATA_PHY_CMN_RESET,          1, 0 ),

    //Do *not* toggle SATA_RST.  Silicon issue makes it impossible to take out
    //of reset correctly.  See HSD 236944.
    //RESET( SATA_RST,                    1, 0 ),

    { NULL }
};

static clock_res_t sec_clocks[] =
{
    CLOCK( SEC_SRAM_CLK_GATE_BYPASS,    1, 0 ),
    CLOCK( SEC_CLK_EN,                  1, 0 ),
    { NULL }
};

static clock_res_t usb_clocks[] =
{ 
    CLOCK( USB_UTMI_CLK_EN,             1, 0 ),
    CLOCK( USB_XSI_CLK_EN,              1, 0 ),
    RESET( USB_RST,                     1, 0 ),
    RESET( USB_PHY_RST,                 0, 1 ),
    { NULL }
};


//==============================================================================
//              S U P P O R T E D   D R I V E R S
//==============================================================================

// Declare/intialize driver_t structures.
// For a device named X, declare/intialize "driver_t drv_X"

DECL_PCI_DRIVER(gbe)
DECL_PCI_DRIVER(ismdclock)
DECL_PCI_DRIVER(mspod)

// PCIE driver controls 2 independent units of the same type.
// Separate suspend/resume calls are required for both.
DECL_PCI_DRIVER(pcie1)
DECL_PCI_DRIVER(pcie2)

DECL_PCI_DRIVER(sata)
DECL_PCI_DRIVER(sec)
DECL_PCI_DRIVER(demux)
DECL_PCI_DRIVER(usb)


static driver_t *_all_drivers[] =
{
    &drv_gbe,
    &drv_ismdclock,
    &drv_mspod,
    &drv_pcie1,
    &drv_pcie2,
    &drv_sata,
    &drv_sec,
    &drv_demux,
    &drv_usb,

    NULL    // EOL
};

//=============================================================================
// FPLL registers
//=============================================================================

// Sub-block IDs
#define SSC0        0x02    // SSC0 clock, affects SATA/PCIe
#define SSC1        0x03    // SSC1 clock, affects USB
#define SSC2        0x04    // SSC2 clock, affects PUB (what's that?)

// Do not touch SSC3. It will be controlled by the NP-CPU.
// NP-CPU guarantees it will be enabled when we are not suspended.
//#define SSC3        0x05    // SSC3 clock, affects GBe

// Register offsets within subblock
#define SSCDIVCTL   0x00
#define SSCINTPHASE 0x04

// Track state of SSC0, as it is used by multiple devices
static bool ssc0_off = false;

static void fpll_write( uint8_t subblock, uint8_t reg_offset, uint32_t value)
{
    uint32_t addr = (subblock<<8) | reg_offset;

    PWR_DEBUG("addr=0x%08x value=0x%08x\n", addr, value);
    iosf_write32(iosfh, IOSF_PORT_FPLL, addr, value);
}

//=============================================================================
// PCIe/SATA PHY (re)initialization
//=============================================================================
// Macros for constructing PHY register addresses (CRI addresses)
//
// Format is
//  bits 15-8:  Base address:
//      15-14:  Access type
//              00 Lane
//              10 Group (or common lane, if Group # == 000000)
//              11 Broadcast
//      13-8 :  Lane or group number
//  bits 7-6:   Function group
//                  For Common Lane:
//                      00 = PLL
//                      01 = Ref
//                      1x = Common
//                  Otherwise
//                      00 = PCS
//                      01 = TX
//                      1x = RX
//  bits 5-0:   Register offset in Function group (4 bytes per register)

#define PHY_REG(func,regnum) ((func) | 4*(regnum))

#define COMMON_LANE(phyreg)   (0x8000 | phyreg )
#define LANE0(phyreg)         (0x2000 | phyreg )
#define LANE1(phyreg)         (0x2100 | phyreg )

// Function Group
#define PCS      0x0000
#define TX       0x0040
#define RX10     0x0080
#define RX11     0x00C0
#define PLL      0x0000 // For Common Lane only
#define CMN      0x0080 // For Common Lane only



//=============================================================================
// Access helpers
//=============================================================================

static
void mod32(void *addr, uint32_t mask, uint32_t value)
{
    uint32_t tmp = readl(addr) & ~mask;
    writel(tmp | (value & mask), addr);
}

static
int pci_mod_config_byte(struct pci_dev *pdev, int where, u8 mask, u8 value)
{
    u8 tmp;

    pci_read_config_byte(pdev, where, &tmp);
    tmp &= ~mask;
    return pci_write_config_byte(pdev, where, tmp | (value & mask));
}

static
int  pci_mod_config_word(struct pci_dev *pdev, int where, u16 mask, u16 value)
{
    u16 tmp;

    pci_read_config_word(pdev, where, &tmp);
    tmp &= ~mask;
    return pci_write_config_word(pdev, where, tmp | (value & mask));
}

static
int  pci_mod_config_dword(struct pci_dev *pdev, int where, u32 mask, u32 value)
{
    u32 tmp;

    pci_read_config_dword(pdev, where, &tmp);
    tmp &= ~mask;
    return pci_write_config_dword(pdev, where, tmp | (value & mask));
}

static
void iosf_mod( uint32_t port, uint32_t offset, uint32_t value, uint32_t mask )
{
    iosf_result_t ret;

    ret = iosf_modify(iosfh, port, offset, mask, value);
    if (ret != IOSF_OK)
    {
        PWR_ERROR("PCIe iosf_modify(offset=0x%08x) failed: 0x%08x\n",
                  offset, ret);
    }
}


//=============================================================================
//       P C I e    I N I T I A L I Z A T I O N 
//=============================================================================
static
void mod_pcie_phy_common(   uint32_t function,
                            uint32_t offset,
                            uint32_t value,
                            uint32_t mask,
                            int      shift )
{
    value <<= shift;
    mask  <<= shift;
    offset = COMMON_LANE(PHY_REG(function,offset));

    PWR_DEBUG("0x%04x: value=0x%08x mask=0x%08x\n", offset, value, mask);
    iosf_mod(IOSF_PORT_PCIE_AFE, offset, mask, value);
}

static
void mod_pcie_phy_lanes(uint32_t function,
                        uint32_t offset,
                        uint32_t value,
                        uint32_t mask,
                        int      shift )
{
    uint32_t lane0 = LANE0(PHY_REG(function,offset));
    uint32_t lane1 = LANE1(PHY_REG(function,offset));

    value <<= shift;
    mask  <<= shift;

    PWR_DEBUG("0x%04x/0x%04x: value=0x%08x mask=0x%08x\n",
                lane0, lane1, value, mask);

    iosf_mod(IOSF_PORT_PCIE_AFE, lane0, mask, value);
    iosf_mod(IOSF_PORT_PCIE_AFE, lane1, mask, value);
}


static
void pcie_phy_init(void)
{
    // Switch to SSC CLK from FPLL.
    CCW(CLOCK_PCIE_PLL_REF_CLK_SEL, 1);

    mdelay(10);
    // Step 1:   Deassert PCIE MPHY common lane reset.
    CCW(CLOCK_PCIE_PHY_CMN_RESET, 1);
    mdelay(10);
    // Set i_cal_byass override enable and override the value
    if ( is_A_step )
    {
        // Set to 1 to enable ModPHY common fuse override
        mod_pcie_phy_common(PLL, 0x23, 1, 1, 0);

        // Per HIP request and Featherstone: 4101481, set
        // i_cal_bypass[1:0]=2'b01 to bypass the whole data lane cal
        mod_pcie_phy_common(PLL, 0x23, 1, 3, 6);
    }

    // Step 2:   Program KALIGN mode override
    mod_pcie_phy_lanes(PCS, 0x02, 3, 3, 9);

    // Step 3a:  PCIE MPHY Lane Register TX Register COMPSLOW
    mod_pcie_phy_lanes(TX, 0x15, 0x50, 0x7F, 24);

    // Step 3b:  PCIE MPHY Lane Register TX Register COMPTYP
    mod_pcie_phy_lanes(TX, 0x16, 0x49, 0x7F, 0);

    // Step 3c:  PCIE MPHY Lane Register TX Register COMPFAST
    mod_pcie_phy_lanes(TX, 0x16, 0x42, 0x7F, 8);

    // Step 4:   Deassert PCIE MPHY sbi_100_rst_core_b reset.
    CCW(CLOCK_SBI_100_RST_CORE_B, 1);

    // Step 5:   Deassert PCIE MPHY sbi_bb_rst_b reset.
    CCW(CLOCK_SBI_BB_RST_B, 1);

    // Step 6:   Deassert PCIE MPHY bbrst_b reset.
    CCW(CLOCK_BBRST_B, 1);

    mdelay(20);

    // Skip Step 7 (use controller default value, until told otherwise)

    // Optimal PCIE BIAS current settings
    if ( is_A_step )
    {
        // work-arounds
        mod_pcie_phy_lanes(PCS, 0x0E, 0xC, 0xF, 20);
        mod_pcie_phy_lanes(PCS, 0x0E, 0xC, 0xF, 16);
        mod_pcie_phy_lanes(PCS, 0x0F, 0x7, 0x3F, 8);
        mod_pcie_phy_lanes(RX11,0x01, 0x1, 0x1, 27);
        mod_pcie_phy_lanes(PCS, 0x0E, 0xF, 0x3F, 24);
        mod_pcie_phy_common(PLL, 0x03, 0xD0, 0xFF, 0);
    }
}
    

static
void pcie_host_init(void) 
{
    struct pci_dev *pdev[2];
    uint32_t        pciExConfig; 
    iosf_result_t   ret;
    int             i;
    
    ret = iosf_read32(iosfh, IOSF_PORT_HUNIT, 0x09, &pciExConfig);
    if (ret != IOSF_OK)
    {
        PWR_ERROR("PCIe iosf_read port HUNIT 0x09 failed: 0x%08x\n", ret);
    }
    pciExConfig &= 0xF0000000;

    pdev[0] = pci_get_device(VENDOR_ID_INTEL, 0x0899, NULL);
    pdev[1] = pci_get_device(VENDOR_ID_INTEL, 0x089a, NULL);

    // put PCIE0/PCIE1 into D0
    pci_mod_config_byte(pdev[0], 0xa4, 3, 0);   
    pci_mod_config_byte(pdev[1], 0xa4, 3, 0);   
    mdelay(10);

    // Select and Enable a High Priority Port, leave this to OEMs to override.
    for (i=0; i<2; i++)
    {
        void *base;

        //only map a 4K page
        base = ioremap_nocache(pciExConfig|(0x00<<20)|(0x1C<<15)|(i<<12), 4096);
        if (base == NULL)
        {
            PWR_ERROR("pcie brige Exconfig mem map failed\n");
            continue;
        }

        // Initialize "Slot Implemented" for Root Ports.
        // Program the "Slot Implemented" bit in The PCI Express Capabilities
        // Register (XCAP).
        // This should only be set if you actually have a physical slot on the
        // board, for soldered down implementations, leave the bit cleared.
        pci_mod_config_word(pdev[i], 0x42, 0x100, 0x100);

        // Initialize "Physical Slot Number" for Root Ports.
        // We'll start at 2 for the slot number, until a better scheme is
        // devised.
        pci_mod_config_dword(pdev[i], 0x54, 0xFFF80000, ((i+1)<<19));
        // Initialize "Slot Power Limit" for Root Ports
        // We'll use the Default, since we don't really know the application.
        pci_mod_config_dword(pdev[i], 0x54, 0x0001FF80, (10<<7));
        pci_mod_config_word(pdev[i], 0xD0, (0xFFFF<<0), (0xC000<<0));
        pci_mod_config_word(pdev[i], 0x4C, (0x3<<10), (0x1<<10));
        pci_mod_config_word(pdev[i], 0x50, (0x3<<0), (0x1<<0));
        pci_mod_config_byte(pdev[i], 0x50, (0x1<<6), (0x1<<6));
        // Misc. Port Configuration for Each Port:
        // per port configuration
        // 1.1   (skip)
        // 1.2   (skip)
        // 1.3   (leave defaults)
        // 1.4   (leave defaults)
        // 1.5   (leave defaults)
        // 1.6   Re-program defaults (MSI Capability Structure)
        pci_write_config_byte(pdev[i], 0x41, 0x80);
        // 1.7   (leave defaults)
        // 1.8   Common Clock Configuration (CCC)
        pci_mod_config_byte(pdev[i], 0x50, (0x1<<6), (0x1<<6));
        // 1.9   (leave defaults)
        // 1.10  (leave defaults)
        // 1.11  Common Clock Exit Latency (CCEL)
        pci_mod_config_dword(pdev[i], 0xD8, (0x7<<15), (0x3<<15));
        // 1.12  Gen1 Unique Clock N_FTS (G1UCNFTS)
        // 1.13  Gen1 Common Clock N_FTS (G1CCNFTS)
        // 1.14  Gen2 Unique Clock N_FTS (G2UCNFTS)
        // 1.15  Gen2 Common Clock N_FTS (G2CCNFTS)
        writel((0x74<<24)|(0x3A<<16)|(0x36<<8)|(0x1B<<0), base+0x314);
        // 1.16  Gen2 x1
        // 1.17  Gen2 x2
        // 1.18  Gen2 x4
        // 1.19  Gen1 x1
        // 1.20  Gen1 x2
        // 1.21  Gen1 x4
        writel((0x8<<20)|(0x5<<16)|(0x4<<12)|(0xC<<8)|(0x7<<4)|(0x4<<0),
                base+0x33C);
        // 1.22  IOSF Packet Fast Transmit Mode (IPF)
        pci_mod_config_dword(pdev[i], 0xD4, (0x1<<11), (0x1<<11));
        // 1.23  Upstream Posted Split Disable (UPSD)
        // 1.24  Upstream Posted Request Size (UPRS)
        // 1.25  Upstream Non-Posted Request Size (UNRS)
        pci_mod_config_dword(   pdev[i],
                                0xD0,
                               ((0x1<<24)|(0x1<<15)|(0x1<<14)),
                               ((0x0<<24)|(0x1<<15)|(0x1<<14)));
        // 1.26  Capability Version (CV)
        // 1.27  Capability ID (CID)
        writel(0x00010001, base+0x100);
        // 2.1   Root Port Static Clock Gate Enable (RPSCG)
        // 2.2   PCIe Link CLKREQ Enable (PCIELCLKREQEN)
        // 2.3   PCIe Backbone CLKREQ Enable (PCIEBBCLKREQEN)
        // 2.4   Shared Resource Dynamic Link Clock Gate Enable (SRDLCGEN)
        // 2.5   Shared Resource Dynamic Backbone Clock Gate Enable (SRDBCGEN)
        // 2.6   Root Port Dynamic Link Clock Gate Enable (RPDLCGEN)
        // 2.7   Root Port Dynamic Backbone Clock Gate Enable (RPDBCGEN)
        pci_mod_config_byte(pdev[i],
                            0xE1,
                            ((1<<7)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|(1<<0)),
                            ((1<<7)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|(1<<0)));
        // 2.8   Squelch Propagation Control Enable (SPCE)
        writel(0x0A002020, base+0x324);
        // 2.9   Root Port L1 Squelch Polling (RPL1SQPOL)
        // 2.10  Root Port Detect Squelch Polling (RPDTSQPOL)
        pci_mod_config_byte(pdev[i],
                            0xE8,
                            ((0x1<<1)|(0x1<<0)),
                            ((0x1<<1)|(0x1<<0)));
        // 3.1   Gen2 Active State L0s Preparation Latency (G2ASL0SPL)
        // 3.2   Gen1 Active State L0s Preparation Latency (G1ASL0SPL)
        writel(0x14140000, base+0x318); 
        // 3.3   Active State Link PM Support (APMS)
        pci_mod_config_dword(pdev[i], 0x4C, (0x3<<10), (0x1<<10));
        // 3.4   Active State Link PM Control (ASPM)
        pci_mod_config_dword(pdev[i], 0x50, (0x3<<0), (0x0<<0)); 
        iounmap(base);
    }

    pci_dev_put(pdev[0]);
    pci_dev_put(pdev[1]);
}


//=============================================================================
//       S A T A    I N I T I A L I Z A T I O N 
//=============================================================================
static
void mod_sata_phy_common(   uint32_t function,
                            uint32_t offset,
                            uint32_t value,
                            uint32_t mask,
                            int      shift )
{
    value <<= shift;
    mask  <<= shift;
    offset = COMMON_LANE(PHY_REG(function,offset));

    PWR_DEBUG("0x%04x: value=0x%08x mask=0x%08x\n", offset, value, mask);
    iosf_mod(IOSF_PORT_SATA_AFE, offset, mask, value);
}

static
void mod_sata_phy_lanes(uint32_t function,
                        uint32_t offset,
                        uint32_t value,
                        uint32_t mask,
                        int      shift )
{
    uint32_t lane0 = LANE0(PHY_REG(function,offset));
    uint32_t lane1 = LANE1(PHY_REG(function,offset));

    value <<= shift;
    mask  <<= shift;

    PWR_DEBUG("0x%04x/0x%04x: value=0x%08x mask=0x%08x\n",
                lane0, lane1, value, mask);

    iosf_mod(IOSF_PORT_SATA_AFE, lane0, mask, value);
    iosf_mod(IOSF_PORT_SATA_AFE, lane1, mask, value);
}


static
void sata_phy_init(void)
{
    // Set Enhanced Configuration Space (register EC) for VTunit
    iosf_modify(iosfh, 0, 0, 0xF0000001,0xE000001);
    // Set Enhanced Configuration Space (register HECREG) for Hunit
    iosf_modify(iosfh, 2, 9, 0xF0000001,0xE000001);

    // Step 0:  Select 100 MHz No SSC clock on SATA PLL REF CLK.
    CCW(CLOCK_SATA_PLL_REF_CLK_SEL, 1);

    // Step 1: setup CP_TOP MMIO (not necessary here)

    // Step 2:  Enable SATA external reference clock.
    CCW(CLOCK_SATA_PLL_REF_CLKBUF_EN, 1);

    // Step 3:  De-assert SATA controller reset.
    CCW(CLOCK_SATA_RST, 1);

    // Step 4:  De-assert SATA MPHY common lane reset.
    CCW(CLOCK_SATA_PHY_CMN_RESET, 1);

    mdelay(10);

    // Set KALIGN mode to 10COM.
    mod_sata_phy_lanes(PCS, 2, 0x2, 0x3, 2 );
    // Step 6:  Program SATA MPHY PLL feedback divider value.
    mod_sata_phy_common(PLL, 2, 0x1C, 0xFF, 24 );
    // Step 7:  Program SATA MPHY PLL PCS/MECLK divider value.
    mod_sata_phy_common( PLL, 6, 0x08, 0x7F, 24 );
    // Step 8a: REF_CLK Input Mux Enable.
    mod_sata_phy_common( PLL, 7, 0x1, 0x1, 26 );
    // Step 8b: REF_CLK Input Mux Select.
    mod_sata_phy_common( PLL, 4, 0x0, 0x3, 13 );

    // don't use BROADCAST registers for lanes because they can't be read...
    // Step 9:   Program SATA MPHY TX de-emphasis level.
    mod_sata_phy_lanes(PCS, 9, 0x1, 0x3, 13 );
    // Step 10a: SATA MPHY Lane Register TX Register COMPSLOW
    mod_sata_phy_lanes(TX, 5, 0x50, 0x7F, 24 );
    // Step 10b: SATA MPHY Lane Register TX Register COMPTYP
    mod_sata_phy_lanes(TX, 6, 0x49, 0x7F, 0 );
    // Step 10c: SATA MPHY Lane Register TX Register COMPFAST
    mod_sata_phy_lanes(TX, 6, 0x42, 0x7F, 8 );
    // Step 11:  Program SATA MPHY to override lane fuses.
    mod_sata_phy_lanes(PCS, 11, 0x1, 0x1, 5 );
    // Step 12: Program SATA MPHY to override latency optim fuse value
    mod_sata_phy_lanes(PCS,  9, 0x3, 0x3, 6 );
    mod_sata_phy_lanes(PCS,  9, 0x1, 0x3, 0 );
    mod_sata_phy_lanes(PCS,  8, 0x0, 0x7, 24 );
    mod_sata_phy_lanes(PCS, 10, 0x3, 0x3, 2 );

    // Step 13:  Program SATA MPHY to override common fuses.
    mod_sata_phy_common( CMN, 3, 0x1, 0x1, 0 );
    // Step 14: Program SATA MPHY to override SFR trim fuse value
    mod_sata_phy_common( PLL, 5, 0x2, 0x7, 5 );
    mod_sata_phy_common( PLL, 3, 0x0, 0x1, 31 );

    // Step 15
    if ( is_A_step )
    {
        // Puma6 A0 set to 0x1, >=B0 do not touch this bit
        mod_sata_phy_common( CMN, 3, 0x1, 0x3, 6 );
    }
    mod_sata_phy_common( CMN, 3, 0x1, 0x1, 5 );
    mod_sata_phy_common( CMN, 3, 0x0, 0x3, 3 );

    // HSD 233988 SV: intermittent SATA errors from both SIP and HIP
    // Step 16a: Program SATA GEN1 Proportional Filter settings
    mod_sata_phy_lanes(RX10, 7, 0x0998, 0xFFFF, 16 );
    // Step 16b: Program SATA GEN1 Integral Filter settings
    mod_sata_phy_lanes(RX10, 8, 0x0014, 0xFFFF, 0 );
    // Step 16c: Program SATA GEN2 Proportional Filter settings
    mod_sata_phy_lanes(RX10, 6, 0x099C, 0xFFFF, 16 );
    // Step 16d: Program SATA GEN2 Integral Filter settings
    mod_sata_phy_lanes(RX10, 7, 0x0012, 0xFFFF, 0 );

    //HSD 234024  SV: SATA ModPHY needs BIAS increase for more PI margin
    if ( is_A_step )
    {
        // work-arounds
        mod_sata_phy_lanes(PCS, 14, 0xC, 0xF, 20 );
        mod_sata_phy_lanes(PCS, 14, 0xC, 0xF, 16 );
        mod_sata_phy_lanes(PCS, 15, 0x7, 0x3F, 8 );
        mod_sata_phy_lanes(RX11, 1, 0x1, 0x1, 27 );
        mod_sata_phy_lanes(PCS, 14, 0xF, 0x3F, 24 );
    }

    mod_sata_phy_common( PLL, 3, 0xD0, 0xFF, 0 );
    mod_sata_phy_common( PLL, 3, 0x2A, 0x7F, 8 );
}

static
void sata_host_init(void)
{
    struct pci_dev *pdev;
    void *          base;
    int             i;
 
    pdev = pci_get_device(VENDOR_ID_INTEL, 0x0C82, NULL);

    pci_mod_config_byte(pdev, 0x74, 3, 0);              // Put SATA back into D0
    mdelay(10);
    
    pci_mod_config_word(pdev, 0x90, 7<<5, 0x3<<5);
    mdelay(1);
    
    pci_write_config_dword(pdev, 0x24, sata_mem_bar);   // Restore BAR
    pci_write_config_byte(pdev, 0x4, 0x7);
    mdelay(10);
    
    base = ioremap_nocache(sata_mem_bar, 4096);         // only map a 4K page
    if (base == NULL)
    {
        PWR_ERROR("sata host IO mem map failed!\n");
        pci_dev_put(pdev);
        return;
    }

    mod32(base+0x04, 1<<31, 1<<31);                     // enable AHCI mode
    mdelay(10);

    pci_write_config_byte(pdev, 0xA0, 0x6C);            // Program SIRI
    pci_write_config_dword(pdev, 0xA4, 0x2E141208);     // Program SIRD
    pci_dev_put(pdev);

    mdelay(10);

    // Reset AHCI controller and wait for reset to complete
    mod32(base+0x04, 1, 1);
    i = 0;
    while ( readl(base+0x4) & 1 )
    {
        mdelay(1);
        if (i++ == 10)
        {
           PWR_WARN("AHCI host reset failed\n");
           break;
        }
    }

    mdelay(10);
    writel(3, base+0xc);            // enable ports 0 and 1
    mod32(base+0x118, 1<<2, 1<<2);  // power up port0
    mod32(base+0x198, 1<<2, 1<<2);  // power up  port1
    mdelay(1);

    iounmap(base);
}

//=============================================================================
//       U S B   I N I T I A L I Z A T I O N 
//=============================================================================

// Assumes clocks are enabled, resets are asserted
static void usb_phy_init(void)
{
    // Step 1: IOSF sideband reset should be released.
    // Step 2: PLL Reg 1: set usb phy to 120MHz (bit 1)
    iosf_write32(iosfh, IOSF_PORT_USB_AFE, 0x7F02, 0x0ac02043);

    CCW(CLOCK_USB_RST, 1);  // take MAC out of reset
    udelay(19);

    // Step 3:  Release USB PHY reset usb_phy_rst
    CCW(CLOCK_USB_PHY_RST, 0);

    // Step 4:  Wait at least 120usec.
    udelay(120);
 
    // Step 5:  Release USB MAC reset usb_rst_n
    CCW(CLOCK_USB_RST, 1);
    udelay(120);

    // Lane 0/1/2 USB2 Per Port registers
    iosf_write32(iosfh, IOSF_PORT_USB_AFE, 0x4100, 0x0035ca81);
    iosf_write32(iosfh, IOSF_PORT_USB_AFE, 0x4200, 0x0035ca81);
    // USB port2 AFE settings need to change to get HS compliance pass
    iosf_write32(iosfh, IOSF_PORT_USB_AFE, 0x4300, 0x0035db81);
}

static void usb_host_init( void )
{
    int             j;
    uint32_t        bar;
    void *          base;
    struct pci_dev *pdev = NULL;

    while (NULL != (pdev = pci_get_device(0x192E, 0x0101, pdev)))
    {
        // enable MMIO space
        pci_write_config_dword(pdev, 0x04, 0x07);
        udelay(100);

        pci_read_config_dword(pdev, 0x14, &bar);

        base = ioremap_nocache(bar, 4096); // only map a 4K page
        if (base == NULL)
        {
            PWR_ERROR("USB host IO map error!\n");
            continue;
        }

        writel(0x00080002, base+0x130);
        // Wait for host reset to complete
        j=0;
        while ( readl(base+0x130) & (1<<1) )
        {
            if (j++ == 20)
            {
                PWR_WARN("USB host reset failed\n");
                break;
            }
            mdelay(1);
        }

        writel(0x3,    base+0x1F8);  // Host Mode
        writel(0x1102, base+0x174);  // Power up and reset Port
        // Wait for port reset to complete
        j=0;
        while ( readl(base+0x174) & (1<<8) )
        {
            if (j++ == 20)
            {
                PWR_WARN("USB host port reset failed\n");
                break;
            }
            mdelay(1);
        }

        iounmap(base);
    }
}   

//=============================================================================
//   E X P O R T E D   V I A   S O C - S P E C I F I C   V E C T O R
//=============================================================================

static icepm_ret_t _punit_init(pal_soc_info_t *soc)
{
    return ICEPM_OK;
}

static void _power_down(driver_t *drv)
{
    if ( ((drv == &drv_pcie1) && ! drv_pcie2.is_suspended )
    ||   ((drv == &drv_pcie2) && ! drv_pcie1.is_suspended ))
    {
        // pcie1 and pcie2 share clocks/resets.
        // Don't touch this one if the other is not yet suspended.
        ;
    }
    else
    {
        gate_clocks(drv->clocks);

        // Put pcie client to reset state
        if (drv == &drv_pcie1 || drv == &drv_pcie2)
        {
            if (pcie_client_reset_gpio != -1)
            {
                gpio_set_value(pcie_client_reset_gpio, 0);
            }
        }

        enter_reset(drv->clocks);

        // Turn off pcie power
        if (drv == &drv_pcie1 || drv == &drv_pcie2)
        {
            if ( wifi_power_gpio != -1)
            {
                gpio_set_value(wifi_power_gpio, 1);
            }
        }

        // Turn off SATA power
        if (drv == &drv_sata)
        {
            if ( sata_power_gpio != -1)
            {
                gpio_set_value(sata_power_gpio, 1);
            }
        }

        // Turn off FPLL clocks related to driver's device
        if (drv == &drv_usb)
        {
            // Turn off FPLL clock SSC1
            fpll_write( SSC1, SSCDIVCTL, 0x006A0003 );
            fpll_write( SSC1, SSCINTPHASE, 0x00000001 );
        }
        else if
        (((drv==&drv_sata) && drv_pcie1.is_suspended && drv_pcie2.is_suspended)
        ||((drv==&drv_pcie1) && drv_sata.is_suspended && drv_pcie2.is_suspended)
        ||((drv==&drv_pcie2) && drv_sata.is_suspended && drv_pcie1.is_suspended))
        {
            // This is the last device dependent on SSC0, so turn it off.
            fpll_write( SSC0, SSCDIVCTL, 0x006A0003 );
            fpll_write( SSC0, SSCINTPHASE, 0x00000001 );
            ssc0_off = true;
        }
    }

    drv->is_suspended = true;
}


static void _power_up(driver_t *drv)
{
    if (((drv == &drv_pcie1) && ! drv_pcie2.is_suspended )
    ||  ((drv == &drv_pcie2) && ! drv_pcie1.is_suspended ))
    {
        // pcie1 and pcie2 share clocks/resets.
        // Only have to manipulate them when the first comes out of suspend.
        // I.e., do nothing if the other is not suspended.
        ;
    }
    else
    {
        if ((drv == &drv_pcie1) || (drv == &drv_pcie2) || (drv == &drv_sata))
        {
            // Ensure pcie in reset state
            if (drv == &drv_pcie1 || drv == &drv_pcie2)
            {
                if ( pcie_client_reset_gpio != -1)
                {
                    gpio_set_value(pcie_client_reset_gpio, 0);
                }
            }

            enter_reset(drv->clocks);  // Make sure devices are really in reset
            if (ssc0_off)
            {
                // Turn on FPLL clock SSC0
                fpll_write( SSC0, SSCDIVCTL, 0x000A0119 );
                fpll_write( SSC0, SSCINTPHASE, 0x00000001 );
                ssc0_off = false;
            }
            enable_clocks(drv->clocks); // Enable clocks

            // Turn on wifi
            if (drv == &drv_pcie1 || drv == &drv_pcie2) {
                if ( wifi_power_gpio != -1)
                {
                    gpio_set_value(wifi_power_gpio, 0);
                }
            }

            // Turn on SATA
            if (drv == &drv_sata) {
                if ( sata_power_gpio != -1)
                {
                    gpio_set_value(sata_power_gpio, 0);
                }
            }

            mdelay(250);

            // Take pcie client out of reset
            if (drv == &drv_pcie1 || drv == &drv_pcie2)
            {
                if ( pcie_client_reset_gpio != -1)
                {
                    gpio_set_value(pcie_client_reset_gpio, 1);
                }
            }

            if (drv == &drv_sata)
            {
                sata_phy_init();  // Initialize PHYs, etc, and take out of reset
                sata_host_init();
            }
            else
            {
                pcie_phy_init();  // Initialize PHYs, etc, and take out of reset
                pcie_host_init();
            }
        }
        else if (drv == &drv_usb)
        {
            enter_reset(drv->clocks);

            // Turn on FPLL clock SSC1
            fpll_write( SSC1, SSCDIVCTL,   0x000A0114 );
            fpll_write( SSC1, SSCINTPHASE, 0x00002000 );
            mdelay(50); // TODO: ???
            fpll_write( SSC1, SSCINTPHASE, 0x00002001 );

            enable_clocks(drv->clocks);
            usb_phy_init();
            usb_host_init();
        }
        else
        {
            enter_reset(drv->clocks);   // Make sure devices are actually in reset
            enable_clocks(drv->clocks); // Enable clocks
            exit_reset(drv->clocks);    // Take out of reset
        }
    }

    drv->is_suspended = false;
}

static icepm_ret_t _mask_unmask_wake_event(IO_8051_wake_event_t wake_event)
{
    return ICEPM_OK;
}

static void _pre_post_sleep(void)
{
}

static void _procfs_event_map(struct seq_file *sfile)
{
    seq_printf(sfile, "No wake event map for this SoC\n");
}

static void _procfs_state(struct seq_file *sfile)
{
    procfs_print_drivers(sfile, _all_drivers);
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

//  Exported vector
static soc_t vector =
{
    _all_drivers,
    _power_down,
    _power_up,
    _punit_init,
    _mask_unmask_wake_event,
    _mask_unmask_wake_event,
    _pre_post_sleep,
    _pre_post_sleep,
    NULL,           // No active power modes defined for CE2600
    _procfs_event_map,
    _procfs_state
};


//=============================================================================
//                      E N T R Y   P O I N T
//=============================================================================

//-----------------------------------------------------------------------------
// ce2600_init
//-----------------------------------------------------------------------------
soc_t *ce2600_init(pal_soc_info_t *soc_info)
{
    uint32_t board_type;
    struct pci_dev *pdev;

    intelce_get_board_type(&board_type);
    switch (board_type)
    {
    case HP_BOARD_TYPE:
    case HP_MG_BOARD_TYPE:
        wifi_power_gpio        = 88;
        sata_power_gpio        = 1;
        pcie_client_reset_gpio = 100;
        break; 
    case GS_BOARD_TYPE:
        wifi_power_gpio        = 88;
        sata_power_gpio        = 63; 
        pcie_client_reset_gpio = 100; 
        break;
    default:
        PWR_ERROR("Unknown board type\n");
        return NULL;
    };

    pdev = pci_get_device( id_sata[0].vendor, id_sata[0].device, NULL );
    if (pdev == NULL)
    {
        // Puma 6 (not MG) does not have SATA controller
        drv_sata.is_suspended = true;
    }
    else
    {
        // Save SATA bar so it can be restored when SATA is reinitialized on
        // resume.
        pci_read_config_dword(pdev, 0x24, &sata_mem_bar);
        pci_dev_put(pdev);
    }

    is_A_step = (soc_info->stepping < SOC_STEPPING_B0);

    return &vector;
}
