//-----------------------------------------------------------------------------
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// GPL LICENSE SUMMARY
//
// Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
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
// Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
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

//------------------------------------------------------------------------------
//                     POWER MANAGEMENT FOR CE5300
//------------------------------------------------------------------------------
#include <linux/string.h>
#include <linux/mutex.h>

#include "osal.h"

#include "icepm_internal.h"
#include "kernel.h"

//==============================================================================
//        C L O C K   C O N T R O L   R E S O U R C E S
//==============================================================================

// The PUnit Firmware gates the clocks and asserts the resets of all devices on
// an island before it powers off the island.  It does this through the island
// registers in cp_top.
//
// For suspend-to-RAM there is no need for us to manipulate clocks/resets; the
// PUnit will take care of all clocks/resets on all island when we go into / out
// of STR.
//
// For runtime_pm, power islands can not be turned off until drivers for *all*
// devices on the island have been suspended; and islands 1 & 2 can only be
// powered off in STR.  In these cases we can save some power immediately when
// a driver is runtime-suspended by gating the clocks of its devices 


static clock_res_t audio_clocks[] =
{
    CLOCK(AU_DSP0_CLK_EN,               1, 0 ),
    CLOCK(AU_DSP1_CLK_EN,               1, 0 ),
    CLOCK(AU_XSI_CLK_EN,                1, 0 ),
    CLOCK(MCLK0_OUT_EN,                 1, 0 ),
    CLOCK(MCLK1_OUT_EN,                 1, 0 ),
    RESET(ADAC_RESET,                   1, 0 ),
    RESET(AU_DSP0_RST,                  1, 0 ),
    RESET(AU_DSP1_RST,                  1, 0 ),
    RESET(AU_IF_RST,                    1, 0 ),
    { NULL }
};

static clock_res_t display_clocks[] =
{
    CLOCK( LVDS_CLK_EN,                 1, 0 ),
    CLOCK( VDC_DA_CLK_EN,               1, 0 ),
    CLOCK( VDC_DB_CLK_EN,               1, 0 ),
    CLOCK( VDC_MDVO_CLK_EN,             1, 0 ),
    CLOCK( VDC_XSI_CLK_EN,              1, 0 ),
    CLOCK( VDC_CLK_EN,                  1, 0 ),
    RESET( VDC_RST,                     1, 0 ),
    RESET( HDMI_TX_RST,                 1, 0 ),
    RESET( HDMI_I2C_RST,                1, 0 ),
    RESET( HDMI_PLL_DIV_RESET,          1, 0 ),
    RESET( HDMI_TX_PLL_RESET,           1, 0 ),
    { NULL }
};

static clock_res_t gbe_clocks[] =
{
    CLOCK( RMII_REFCLK_50_CLK_EN,       1, 0 ),
    CLOCK( PAD_GBE_REF_CLK_EN,          1, 0 ),
    CLOCK( PAD_GBE_IN_CLK_EN,           1, 0 ),
    CLOCK( GBE_XSI_CLK_EN,              1, 0 ),
    CLOCK( GBE_HCLK_EN,                 1, 0 ),
    CLOCK( GBE_DDS_HREF_CLK_EN,         1, 0 ),
    CLOCK( INT_GBE_REF_CLK_EN,          1, 0 ),
    RESET( GBE_RST,                     1, 0 ),
    { NULL },
};

static clock_res_t graphics_clocks[] =
{
    CLOCK( SGX_CLK_EN,                  1, 0 ),
    RESET( SGX_RST,                     1, 0 ),
    { NULL }
};

static clock_res_t graphics_2D_clocks[]=
{
    CLOCK( GFX_2D_CLK_EN,               1, 0 ),
    RESET( GFX_2D_RST,                  1, 0 ),
    { NULL }
};

// videnc and mux drivers share this device. Gating/reset should not be done
// until D3 has been requested for BOTH; and devices should be ungated and
// taken out of reset only when the FIRST request for D0 is received.
static clock_res_t GVsparcT_clocks[] =
{
    CLOCK( GVT_CLK_EN,                  1, 0 ),
    RESET( GVT_VSPARC_RST,              1, 0 ),
    { NULL }
};

static clock_res_t hdmi_rx_ce_clocks[]   =
{
    // Workaround for HSD 237720 -- uncomment next line with HSD is resolved
    //CLOCK( HDMI_RX_CORE_CLK_EN,         1, 0 ),
    RESET( HDMI_RX_I2C_RESET,           1, 0 ),
    RESET( HDMI_RX_PLL_DIV_RESET,       1, 0 ),
    RESET( HDMI_RX_PLL_RESET,           1, 0 ),
    RESET( HDMI_RX_RST,                 1, 0 ),
    { NULL }
};

static clock_res_t ismdclock_clocks[]   =
{
    RESET( CRU_RST,                     1, 0 ),
    { NULL }
};

// See also GVsparcT_clocks
static clock_res_t mux_clocks[] =
{
    CLOCK( H264_DEF640_CLK_EN,          1, 0 ),
    CLOCK( MPG4_MFD_CLK_EN,             1, 0 ),
    CLOCK( RSB_MFD_CLK_EN,              1, 0 ),
    CLOCK( VC1_MFD_CLK_EN,              1, 0 ),
    { NULL }
};

// pcie1/pcie2 share clocks/resets
// - gate/reset only after *BOTH* are suspendend
// - ungate/run when *FIRST* is resumed
static clock_res_t pcie_clocks[] =
{
    CLOCK( PCIE_BBCLK_EN,               1, 0 ),
    CLOCK( PCIE_IGCLK_EN,               1, 0 ),
    RESET( BBRST_B,                     1, 0 ),
    RESET( SBI_BB_RST_B,                1, 0 ),
    RESET( SBI_100_RST_CORE_B,          1, 0 ),
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
    //RESET( SATA_RST,                    1, 0 ),
    { NULL }
};

static clock_res_t usb_clocks[] =
{ 
    CLOCK( USB_XSI_CLK_EN,              1, 0 ),
    CLOCK( USB_UTMI_CLK_EN,             1, 0 ),
    CLOCK( USB_XSI_CLK_EN,              1, 0 ),
    // RESET( USB_PHY_RST,                 1, 0 ),
    //  1st STR with thumb drive present:
    //  instead of:
    //      usb 1-1: reset-resume
    //      ehci_hcd 0000:01:0d.0: port 1 high speed
    //      ehci_hcd 0000:01:0d.0: GetStatus port:1 status 001005 0  ACK POWER
    //                                                        sig=se0 PE CONNECT
    //  see:
    //      usb 1-1: reset-resume
    //      ehci_hcd 0000:01:0d.0: port 1 high speed
    //      ehci_hcd 0000:01:0d.0: GetStatus port:1 status 001805 0  ACK POWER
    //                                                        sig=j PE CONNECT
    //      usb 1-1: device reset changed speed!
    //      hub 1-0:1.0: logical disconnect on port 1

    // RESET( USB_RST,                     1, 0 ),
    //  2nd STR:
    //  ehci_hcd 0000:01:0d.2: Port 0 phy low-power mode failed
    //  ehci_hcd 0000:01:0d.1: Port 0 phy low-power mode failed
    //  ehci_hcd 0000:01:0d.0: Port 0 phy low-power mode failed
    { NULL }
};

static clock_res_t vidcap_clocks[] =
{
    CLOCK( HDVCAP_CLK_EN,               1, 0 ),
    RESET( HDVCAP_RST,                  1, 0 ),
    { NULL }
};

static clock_res_t viddec_clocks[] =
{
    CLOCK( MPG2VDP_CLK_EN,              1, 0 ),
    CLOCK( MFD_RSB_CLK_EN,              1, 0 ),
    CLOCK( MFVDP_CLK_EN,                1, 0 ),
    CLOCK( VC1VDP_CLK_EN,               1, 0 ),
    CLOCK( H264_DEF640_CLK_EN,          1, 0 ),
    CLOCK( GVD_CLK_EN,                  1, 0 ),
    RESET( H264DEC_RST,                 1, 0 ),
    RESET( VC1VDP_RST,                  1, 0 ),
    RESET( MPG2VDP_RST,                 1, 0 ),
    RESET( MFVDP_RST,                   1, 0 ),
    RESET( GVD_VSPARC_RST,              1, 0 ),
    { NULL }
};

// See also GVsparcT_clocks
static clock_res_t videnc_clocks[] =
{
    CLOCK( H264VE_CLK_EN,               1, 0 ),
    CLOCK( EPU_CLK_EN,                  1, 0 ),
    RESET( H264VE_RST,                  1, 0 ),
    RESET( EPU_RST,                     1, 0 ),
    { NULL }
};

static clock_res_t vidpproc_clocks[] =
{
    CLOCK( DPE_CLK_EN,                  1, 0 ),
    RESET( DPE_RST,                     1, 0 ),
    { NULL }
};

//==============================================================================
//              S U P P O R T E D   D R I V E R S
//==============================================================================

// For a device named X, declare/intialize "driver_t drv_X"

DECL_PCI_DRIVER(audio)      //AUDIO DSP0/1 + AUDIO IF
DECL_PCI_DRIVER(display)    // VDC + HDMI
DECL_PCI_DRIVER(gbe)
DECL_PCI_DRIVER(graphics)
DECL_PCI_DRIVER(graphics_2D)
DECL_PCI_DRIVER(hdmi_rx_ce)
DECL_PCI_DRIVER(ismdclock)
DECL_PCI_DRIVER(mux)
// PCIE driver controls 2 independent units of the same type.
// Separate suspend/resume calls are required for each unit.
DECL_PCI_DRIVER(pcie1)
DECL_PCI_DRIVER(pcie2)
DECL_PCI_DRIVER(sata)
DECL_PCI_DRIVER(usb)
DECL_PCI_DRIVER(vidcap)
DECL_PCI_DRIVER(viddec)
DECL_PCI_DRIVER(videnc)
DECL_PCI_DRIVER(vidpproc)

// Can't power off island 1, but can clock-gate CE drivers. Can't mess with
// demux and tsout because they both access prefilter registers and have a
// prescribed suspend/resume order, which runtime_pm won't guarantee.
#define ISLAND1_DRIVERS \
            &drv_graphics_2D

// Can't power off island 2, don't mess with sec
#define ISLAND2_DRIVERS

#define ISLAND3_DRIVERS \
            &drv_ismdclock, \
            &drv_gbe, \
            &drv_pcie1, \
            &drv_pcie2, \
            &drv_sata, \
            &drv_usb

// Audio IF is on island 1, but that island can't be powered off.  We can,
// however clock-gate/reset that device
#define ISLAND4_DRIVERS \
            &drv_audio

#define ISLAND5_DRIVERS \
            &drv_viddec

#define ISLAND6_DRIVERS \
            &drv_mux, \
            &drv_videnc

#define ISLAND7_DRIVERS \
            &drv_vidpproc

#define ISLAND8_DRIVERS \
            &drv_hdmi_rx_ce, \
            &drv_vidcap, \
            &drv_display

#define ISLAND9_DRIVERS \
            &drv_graphics

static driver_t *all_drivers[] =
{
    ISLAND1_DRIVERS,
    //ISLAND2_DRIVERS,
    ISLAND3_DRIVERS,
    ISLAND4_DRIVERS,
    ISLAND5_DRIVERS,
    ISLAND6_DRIVERS,
    ISLAND7_DRIVERS,
    ISLAND8_DRIVERS,
    ISLAND9_DRIVERS,
    NULL
};

//==============================================================================
//              P O W E R   I S L A N D S
//==============================================================================

// For each island, a list of drivers associated with the devices the island.
static driver_t *drivers1[] = { ISLAND1_DRIVERS, NULL };
static driver_t *drivers2[] = { NULL };
static driver_t *drivers3[] = { ISLAND3_DRIVERS, NULL };
static driver_t *drivers4[] = { ISLAND4_DRIVERS, NULL };
static driver_t *drivers5[] = { ISLAND5_DRIVERS, NULL };
static driver_t *drivers6[] = { ISLAND6_DRIVERS, NULL };
static driver_t *drivers7[] = { ISLAND7_DRIVERS, NULL };
static driver_t *drivers8[] = { ISLAND8_DRIVERS, NULL };
static driver_t *drivers9[] = { ISLAND9_DRIVERS, NULL };

// Island descriptor
typedef struct
{
    uint32_t    ssc_bits;       // Mask for PUnit SSC register for the register
                                //   bits associated with this island. 0 if we
                                //   can't power this island off at runtime.

    driver_t ** drivers;        // Pointer to list of drivers for this island.

    uint32_t    drivers_on;     // Current state of island. Each 1 bit => a
                                //   driver with active devices. When entire
                                //   value is 0, island can be powered down.
} island_t;

#define ALWAYS_ON 0x80000000    // A value that will never be assigned as a
                                // driver ID, so once assigned to drivers_on
                                // it will never be cleared

// Table of CE5300 islands.
static island_t islands[] =
{
    {     0, NULL,     ALWAYS_ON }, // 0 PUnit
    {     0, drivers1, ALWAYS_ON }, // 1 Misc
    {     0, drivers2, ALWAYS_ON }, // 2 SEC
    { 3<< 8, drivers3, ALWAYS_ON }, // 3 GBE, PCIe, SATA, USB, cru_eth
    { 3<<10, drivers4, 0         }, // 4 Audio
    { 3<<12, drivers5, 0         }, // 5 Decode
    { 3<<14, drivers6, 0         }, // 6 Encode, Mux
    { 3<<16, drivers7, 0         }, // 7 DPE
    { 3<<18, drivers8, 0         }, // 8 Display, HDMI-RX, Vidcap
    { 3<<20, drivers9, 0         }, // 9 3D GFX
};
#define NUM_ISLANDS (sizeof(islands)/sizeof(islands[0]))
#define ISLAND_NUMBER(ip) ((ip) - islands)

// Mutex to protect the islands_t.drivers_on state variables against races
// between threads. These variables are only accessed in update_island().
static struct mutex island_lock;


//------------------------------------------------------------------------------
// Update the state of the island related to the specified driver.
//      drv: driver whose devices are changing power state.
//      on : true => devices being powered on, false => powered off
//------------------------------------------------------------------------------
static void update_island( driver_t * drv, bool on )
{
    island_t *  island              = (island_t*) drv->island;
    bool        change_power_state  = false;

    mutex_lock(&island_lock);

    if ( on )
    {
        // We are powering on device(s) controlled by the driver. If all
        // devices were off we will first need to power on the island.
        change_power_state = (island->drivers_on == 0);
        island->drivers_on |= drv->drv_id;  // Mark the device(s) on
    }
    else
    {
        // We are powering off device(s) controlled by the driver. If that
        // results in all drivers for the island now being suspended, we
        // will need to power off the island.
        island->drivers_on &= ~(drv->drv_id); // Mark the device(s) off
        change_power_state = (island->drivers_on == 0);
    }

    PWR_DEBUG("Island %d driver %d %s (now on: 0x%08x)\n",
                    ISLAND_NUMBER(island),
                    drv->drv_id,
                    on ? "on" : "off",
                    island->drivers_on);

    if ( change_power_state )
    {
        int i;
        uint32_t ssc = 0;

        // Build the appropriate value for the PUnit SSC register.
        for ( i=0; i<NUM_ISLANDS; i++ )
        {
            if ( islands[i].drivers_on == 0 )
            {
                ssc |= islands[i].ssc_bits;
            }
        }

        // Use the register value to change the island state.
        punit_mode(ssc);
    }

    mutex_unlock(&island_lock);
}

//=============================================================================
//   E X P O R T E D   V I A   S O C - S P E C I F I C   V E C T O R
//=============================================================================

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
static void power_down_5300(driver_t *drv)
{
    if (((drv == &drv_pcie1) && ! drv_pcie2.is_suspended )
    ||  ((drv == &drv_pcie2) && ! drv_pcie1.is_suspended ))
    {
        // pcie1 and pcie2 share clocks/resets. Do nothing until the
        // driver has called to shutdown BOTH devices.
        ;
    }
    else
    {
        // "Proper" shut-down for any device that hasa PHY on-chip (e.g., SATA,
        // USB, PCIe, VDAC, HDMI Tx/Rx) requires:
        //  - device driver: put device I/Os  into lowest power state (may be a
        //    combination of writes to both MAC/PHY).
        //  - icepm: clock gate the device(s) but DO NOT RESET them.  Reset
        //    clears the signals between MAC and PHY, possibly causing PHY to
        //    come out of low power
        //  - PUnit: turn-on firewalls (which properly hold the signals between
        //    MAC/PHY to keep the PHY in low power), then put devices into reset
        gate_clocks(drv->clocks);

        if (((drv == &drv_mux)    && (drv_videnc.is_suspended))
        ||  ((drv == &drv_videnc) && (drv_mux.is_suspended)))
        {
            // GVsparcT is shared by mux and videnc -- clock gate only after
            // both are suspended.
            cp_top(GVsparcT_clocks, CLK_CLOCK, false);
        }
    }

    update_island( drv, false );
    drv->is_suspended = true;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
static void power_up_5300(driver_t *drv)
{
    update_island( drv, true );
    drv->is_suspended = false;

    if (((drv == &drv_pcie1) && ! drv_pcie2.is_suspended )
    ||  ((drv == &drv_pcie2) && ! drv_pcie1.is_suspended ))
    {
        // pcie1 & pcie2 share clocks/resets. Do nothing if already enabled.
        ;
    }
    else
    {
        // For each device:
        //  - enable clocks first,
        //  - then take device out of reset

        // I used to do this as a santity check in case the devices were not
        // really in reset state, but it screws up devices on the PCIe bus.
        //enter_reset(drv->clocks);   // Make sure devices are actually in reset

        enable_clocks(drv->clocks); // Enable clocks
        exit_reset(drv->clocks);    // Take out of reset

        if (((drv == &drv_mux)    && (drv_videnc.is_suspended))
        ||  ((drv == &drv_videnc) && (drv_mux.is_suspended)))
        {
            // GVsparcT is shared by mux and videnc -- enable only when
            // the first one is resumed.
            //
            cp_top(GVsparcT_clocks, CLK_RESET, false);
                                    // Make sure device is actually in reset
            cp_top(GVsparcT_clocks, CLK_CLOCK, true);  // Enable clocks
            cp_top(GVsparcT_clocks, CLK_RESET, true);  // Take out of reset
        }
        else if ((drv == &drv_pcie1) || (drv == &drv_pcie2))
        {
            // PCIe may require an additional 50ms to recover link and
            // recognize end point device before resuming driver.
            mdelay(50);
        }
    }
}

//------------------------------------------------------------------------------
// Produce proc file output through seq file
//------------------------------------------------------------------------------
static void print_island_state(struct seq_file *sfile)
{
    int i;

    for (i=1; i<NUM_ISLANDS; i++)
    {
        seq_printf(sfile, "%s Island %d\n",
                   islands[i].drivers_on ? "ON " : "off", i);
        procfs_print_drivers(sfile, islands[i].drivers);
    }
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

//  Exported vector
static soc_t vector =
{
    all_drivers,
    power_down_5300,
    power_up_5300,
    punit_init,
    punit_mask_wake_event,
    punit_unmask_wake_event,
    punit_pre_sleep,
    punit_post_sleep,
    NULL,            // Active power modes are only defined for CE4200
    punit_event_map,
    print_island_state
};

//=============================================================================
//                      E N T R Y   P O I N T
//=============================================================================

// Initialize the island-specific entries in the driver_t structures for all
// drivers with devices on the specified island.
static void init_drivers(island_t * island)
{
    driver_t ** p_drv;
    uint32_t    id = 1;

    for (p_drv = island->drivers; *p_drv != NULL; p_drv++, id <<= 1)
    {
        // Point the driver to the island.
        (*p_drv)->island    = island;

        // Set driver ID to a single bit flag that will be unique for this
        // island
        (*p_drv)->drv_id    = id;

        // Initialize the island state to assume the driver's devices are on
        island->drivers_on |= id;
    }
}

soc_t * ce5300_init(pal_soc_info_t *soc_info)
{
    int i;

    mutex_init(&island_lock);
    for ( i=1; i < NUM_ISLANDS; i++ )
    {
        init_drivers( &islands[i] );
    }

    return &vector;
}

//mutex_destroy(&island_lock);
