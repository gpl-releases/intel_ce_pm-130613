//-----------------------------------------------------------------------------
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// GPL LICENSE SUMMARY
//
// Copyright(c) 2009-2013 Intel Corporation. All rights reserved.
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
// Copyright(c) 2009-2013 Intel Corporation. All rights reserved.
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
//                     POWER MANAGEMENT FOR CE4200
//------------------------------------------------------------------------------
#include <linux/string.h>


#include <linux/pci.h>
#include <linux/pci-intelce_pm.h>
            // Contains Intel CE kernel patches for functions:
            //      intel_pm_register_callback()
            //      suspend_devices_rooted()
            //      resume_devices_rooted()
            //      clear_async_error()

#include "osal.h"

#include "icepm_internal.h"
#include "kernel.h"

static bool in_standby3 = false;

//==============================================================================
//        C L O C K   C O N T R O L   R E S O U R C E S
//==============================================================================

// Resources controlled through the clock_control component to gate/reset
// the devices controlled by each driver.

static clock_res_t audio_clocks[] =
{
    CLOCK( S_AU_DSP_CLK_EN,             1, 0 ),
    CLOCK( L_AU_DSP_CLK_EN,             1, 0 ),
    CLOCK( AU_XSI_CLK_EN,               1, 0 ),
    RESET( AU_DSP0_RST,                 1, 0 ),
    RESET( AU_DSP1_RST,                 1, 0 ),
    RESET( AU_IF_RST,                   1, 0 ),
    { .name = NULL }
};

static clock_res_t demux_clocks[] =
{
    CLOCK( TSD_CLK_EN,                  1, 0 ),
    CLOCK( PREFILTER_XSI_CLK_EN,        1, 0 ), // Prefilter shared with tsout
    RESET( TSD_RST,                     1, 0 ),
    RESET( PF_RST,                      1, 0 ), // Prefilter shared with tsout
    { .name = NULL }
};

static clock_res_t display_clocks[] =
{
    CLOCK( VDC_DA_CLK_EN,               1, 0 ),
    CLOCK( VDC_DB_CLK_EN,               1, 0 ),
    CLOCK( VDC_MDVO_CLK_EN,             1, 0 ),
    CLOCK( VDC_XSI_CLK_EN,              1, 0 ),
    CLOCK( VDC_CLK_EN,                  1, 0 ),
    RESET( HDMI_RST,                    1, 0 ),
    RESET( HDMI_I2C_RST,                1, 0 ),
    RESET( VDC_RST,                     1, 0 ),
    { .name = NULL }
};

static clock_res_t eMMC_clocks[] =
{
    RESET( NAND_RST,                    1, 0 ),
    { .name = NULL }
};

static clock_res_t gbe_clocks[] =
{
    CLOCK( RMII_REFCLK_50_CLK_EN,       1, 0 ),
    CLOCK( PAD_GBE_REF_CLK_EN,          1, 0 ),
    CLOCK( PAD_GBE_IN_CLK_EN,           1, 0 ),
    CLOCK( GBE_XSI_CLK_EN,              1, 0 ),
    CLOCK( GBE_HCLK_EN,                 1, 0 ),
    RESET( GBE_RST,                     1, 0 ),
    { .name = NULL },
};

static clock_res_t graphics_clocks[] =
{
    CLOCK( GFX_CLK_EN,                  1, 0 ),
    RESET( GFX_RST,                     1, 0 ),
    { .name = NULL }
};

static clock_res_t graphics_2D_clocks[]=
{
    CLOCK( GFX2D_CLK_EN,                1, 0 ),
    RESET( GFX2D_RST,                   1, 0 ),
    { .name = NULL }
};

static clock_res_t ismdclock_clocks[]   =
{
    CLOCK( GBE_DDS_HREF_CLK_EN,         1, 0 ),
    RESET( CRU_RST,                     1, 0 ),
    { .name = NULL }
};

static clock_res_t mspod_clocks[] =
{
    // tsout accesses mspod registers -- don't disable clock with mspod.
    // Handle it when tsout suspended.
    // CLOCK( MSPOD_XSI_CLK_EN,            1, 0 ),
    { .name = NULL },
};

// pcie1/pcie2 share clocks/resets
// - gate/reset only after *BOTH* are suspendend
// - ungate/run when *FIRST* is resumed
static clock_res_t pcie_clocks[] =
{
    CLOCK( PCIE0_CLK_OUT_DIS,           0, 1 ),
    CLOCK( PCIE1_CLK_OUT_DIS,           0, 1 ),
    CLOCK( PCIE_LGCLK_EN,               1, 0 ),
    CLOCK( PCIE_BBCLK_EN,               1, 0 ),
    RESET( PCIE_RST,                    1, 0 ),
    { .name = NULL }
};
#define pcie1_clocks pcie_clocks
#define pcie2_clocks pcie_clocks

static clock_res_t sata_clocks[] =
{
    CLOCK( SATA_CLK_EN,                 1, 0 ),
    RESET( SATA_RST,                    1, 0 ),
    { .name = NULL }
};

static clock_res_t sec_clocks[] =
{
    CLOCK( MEU_CLK_EN,                  1, 0 ),
    RESET( SEC_RST,                     1, 0 ),
    { .name = NULL }
};

static clock_res_t sven_clocks[] =
{
    CLOCK( ODLA_CLK_EN,                 1, 0 ),
    CLOCK( PMU_CLK_EN,                  1, 0 ),
    CLOCK( OMAR_CLK_EN,                 1, 0 ),
    RESET( DFX_RST,                     1, 0 ),
    { .name = NULL },
};

static clock_res_t tsout_clocks[] =
{
    // demux driver also accesses prefilter. Since demux must suspend *after*
    //      tsout, the prefilter clocks are handled there.
    //CLOCK( PREFILTER_XSI_CLK_EN,        1, 0 ),
    //RESET( PF_RST,                      1, 0 ),

    // tsout accesses mspod registers -- don't disable clock with mspod,
    // do it when tsout suspended.
    CLOCK( MSPOD_XSI_CLK_EN,            1, 0 ),
    { .name = NULL }
};

static clock_res_t usb_clocks[] =
{ 
    CLOCK( USB_CLK480_EN,               1, 0 ),
    CLOCK( USB_XSI_CLK_EN,              1, 0 ),
    RESET( USB_RST_CFG,                 1, 0 ),
    RESET( USB_RST_SW,                  1, 0 ),
    RESET( USB_RST,                     1, 0 ),
    { .name = NULL }
};

static clock_res_t vidcap_clocks[] =
{
    CLOCK( HDVCAP_CLK_EN,               1, 0 ),
    RESET( HDVCAP_RST,                  1, 0 ),
    { .name = NULL }
};

static clock_res_t vidpproc_clocks[] =
{
    CLOCK( DPE_CLK_EN,                  1, 0 ),
    RESET( DPE_RST,                     1, 0 ),
    { .name = NULL }
};

// mux, viddec, and videnc drivers share devices as follows:
//
//      Device                  Drivers
//      ----------------------  -----------------
//      (mux)    2E5C MFD       mux viddec
//      (viddec) 0703 GVsparcT      viddec videnc
//      (videnc) 0706 H264VE               videnc
//
// For shared devices, gating/reset should not be done until D3 has been
// requested for BOTH; and devices should be ungated and taken out of reset
// only when the FIRST request for D0 is received.
static clock_res_t mux_clocks[] =
{
    // MFD clocks -- MFD also used by viddec
    CLOCK( H264_DEF640_CLK_EN,          1, 0 ),
    CLOCK( MPG4_MFD_CLK_EN,             1, 0 ),
    CLOCK( RSB_MFD_CLK_EN,              1, 0 ),
    CLOCK( VC1_MFD_CLK_EN,              1, 0 ),
    RESET( MFD_RST,                     1, 0 ),
    { .name = NULL }
};
static clock_res_t viddec_clocks[] =
{
    // GVsparcT clocks - GVsparcT also used by videnc
    CLOCK( GBLVSPARC_XSI_CLK_EN,        1, 0 ),
    RESET( GBLVSPARC_RST,               1, 0 ),
    { .name = NULL }
};
static clock_res_t videnc_clocks[] =
{
    CLOCK( H264VE_CLK_EN,               1, 0 ),
    RESET( H264VE_RST,                  1, 0 ),
    { .name = NULL }
};


//==============================================================================
//              S U P P O R T E D   D R I V E R S
//==============================================================================

// Declare/intialize driver_t structures used by us to drive mode changes.

// For a device named X, declare/intialize "driver_t drv_X"
DECL_PCI_DRIVER(audio)      //AUDIO DSP0/1 + AUDIO IF
DECL_PCI_DRIVER(demux)
DECL_PCI_DRIVER(display)    // VDC + HDMI
DECL_PCI_DRIVER(eMMC)
DECL_PCI_DRIVER(gbe)
DECL_PCI_DRIVER(graphics_2D)
DECL_PCI_DRIVER(graphics)
DECL_PCI_DRIVER(ismdclock)
DECL_PCI_DRIVER(mspod)
DECL_PCI_DRIVER(mux)
// PCIE driver controls 2 independent units of the same type.
// Separate suspend/resume calls are required for both.
DECL_PCI_DRIVER(pcie1)
DECL_PCI_DRIVER(pcie2)
DECL_PCI_DRIVER(sata)
DECL_PCI_DRIVER(sec)
DECL_PCI_DRIVER(sven)
DECL_PCI_DRIVER(tsout)
DECL_PCI_DRIVER(usb)        // Not Intel device!
DECL_PCI_DRIVER(vidcap)
DECL_PCI_DRIVER(viddec)
DECL_PCI_DRIVER(videnc)
DECL_PCI_DRIVER(vidpproc)


//==============================================================================
//              A C T I V E   P O W E R   M O D E S
//==============================================================================

#define SSC_ISLAND0     (3 << 0)
#define SSC_ISLAND1     (3 << 2)
#define SSC_ISLAND2     (3 << 4)
#define SSC_ISLAND2_5   (3 << 6)
#define SSC_ISLAND3     (3 << 8)
#define SSC_ISLAND4     (3 << 10)


// Lists of drivers on each island.

#define ISLAND1_DRIVERS \
                    &drv_tsout, \
                    &drv_demux, \
                    &drv_ismdclock,\
                    &drv_sven, \
                    &drv_gbe

// The following PCI devices are on island 2, but have no associated cp_top
// clocks/resets:
//              DAA-SPI     (0x0707)
//              EXP         (0x2E65)
//              GPIO        (0x2E67)
//              I2C         (0x2E68)
//              UART        (0x2E66)
//              SmartCard   (0x2E69)
//              SPI Master  (0x2E6A)
#define ISLAND2_DRIVERS \
                    &drv_mspod, \
                    &drv_sec

#define ISLAND2p5_DRIVERS \
                    &drv_sata, \
                    &drv_usb

// NOTE!:  videnc/mux hardware is on island 2.5, but the drivers are also
//         dependent on FW in MFD vsparc on island 3. So when island 3 goes
//         down, they must suspend.  I'm not including them in island 2.5
//         because 2.5 can't go down if island 3 doesn't also go down.
//
//         Order of suspension of the 3 vsparc drivers MUST be:
//              videnc
//              mux
//              viddec

// For STANDBY3 order of list MATTERS: drivers are suspended in order shown,
// resumed in reverse order
#define ISLAND3_DRIVERS \
                    &drv_graphics_2D, \
                    &drv_graphics,  \
                    &drv_vidcap,    \
                    &drv_audio,     \
                    &drv_vidpproc,  \
                    &drv_videnc,    \
                    &drv_mux,       \
                    &drv_viddec,    \
                    &drv_display,   \
                    &drv_pcie2,     \
                    &drv_pcie1,     \
                    &drv_eMMC


static driver_t * island1_drivers[] =
{
    ISLAND1_DRIVERS,
    NULL                // End of list
};

static driver_t * island2_drivers[] =
{
    ISLAND2_DRIVERS,
    NULL                // End of list
};

static driver_t * island2p5_drivers[] =
{
    ISLAND2p5_DRIVERS,
    NULL                // End of list
};

static driver_t * island3_drivers[] =
{
    ISLAND3_DRIVERS,
    NULL                // End of list
};

// List of pointers to all supported drivers.
static driver_t *all_drivers[] =
{
    ISLAND3_DRIVERS,
    ISLAND2p5_DRIVERS,
    ISLAND2_DRIVERS,
    ISLAND1_DRIVERS,
    NULL                // End of list
};


//==============================================================================
//           A C T I V E   P M   M O D E   C H A N G E  S U P P O R T
//==============================================================================

//-----------------------------------------------------------------------------
// resume_drivers
//
// Invoke the resume functions of all drivers on the passed list.
//
// Drivers are listed in the order in which they should be suspended --
// they must be resumed in the opposite order.
//-----------------------------------------------------------------------------
static
icepm_ret_t resume_drivers(driver_t **drv_list)
{
    icepm_ret_t         ret         = ICEPM_OK;
    pm_message_t        pm_message = { .event = PM_EVENT_RESUME };
    int                 i;
    struct pci_dev    * pcidev;


    // Find the end of the list, counting the number of entries.
    for (i=0; drv_list[i] != NULL; i++)
    {
        ;
    }

    // Process the list backwards
    while ( --i >= 0 )
    {
        driver_t *drv = drv_list[i];

        PWR_DEBUG("%s", drv->name);

        if ( ! drv->is_suspended )
        {
            PWR_DEBUG_CONTINUE(" -- not suspended\n");
            continue;
        }

        pcidev=pci_get_device(drv->pci_id[0].vendor,drv->pci_id[0].device,NULL);
        if (pcidev == NULL)
        {
            PWR_WARN("***PCI device 0x%04x:0x%04x not found\n",
                      drv->pci_id[0].vendor,
                      drv->pci_id[0].device);
        }

        // suspend/resume pointers:
        //  Legacy registration: in struct pci_driver.
        //  Regular registration: in pm field of generic driver structure.
        if ((NULL == pcidev)
        ||  (NULL == pcidev->driver)
        ||  !(pcidev->driver->resume ||
              (pcidev->driver->driver.pm && pcidev->driver->driver.pm->resume))
        )
        {
            // DRIVER NOT REGISTERED (not loaded or no driver for this device)
            PWR_DEBUG_CONTINUE(" -- no driver registered\n");
        }
        else
        {
            PWR_DEBUG_CONTINUE("\n");

            pci_set_power_state(pcidev,PCI_D0);

            if (0 != resume_devices_rooted(&(pcidev->dev), pm_message))
            {
                // Should be no reason why a driver can't resume, so something
                // is seriously wrong.  But there's nothing we can do about it,
                // so give an error but continue processing the list.
                PWR_ERROR("RESUME FAILED FOR %s", drv->name);
                ret = ICEPM_ERR_INCOMPLETE;
            }
        }

        if (pcidev)
        {
            pci_dev_put(pcidev);
        }
    }
    return ret;
}


//-----------------------------------------------------------------------------
// suspend_drivers
//
// Invoke the suspend functions of all drivers on the passed list.
//-----------------------------------------------------------------------------
static
icepm_ret_t suspend_drivers(driver_t **drv_list)
{
    icepm_ret_t         ret         = ICEPM_OK;
    pm_message_t        pm_message  = { .event = PM_EVENT_SUSPEND };
    int                 i;
    struct pci_dev    * pcidev;


    for ( i=0; drv_list[i] != NULL; i++)
    {
        driver_t *drv = drv_list[i];

        PWR_DEBUG("%s", drv->name);

        if ( drv->is_suspended )
        {
            PWR_DEBUG_CONTINUE(" -- already suspended\n");
            continue;
        }

        pcidev=pci_get_device(drv->pci_id[0].vendor,drv->pci_id[0].device,NULL);
        if (pcidev == NULL)
        {
            PWR_WARN("***PCI device 0x%04x:0x%04x not found\n",
                      drv->pci_id[0].vendor,
                      drv->pci_id[0].device);
        }

        // suspend/resume pointers:
        //  Legacy registration: in struct pci_driver.
        //  Regular registration: in pm field of generic driver structure.
        if ((NULL == pcidev)
        ||  (NULL == pcidev->driver)
        ||  !(pcidev->driver->suspend
             || (pcidev->driver->driver.pm && pcidev->driver->driver.pm->suspend)))
        {
            // DRIVER NOT REGISTERED (not loaded or no driver for this device)
            PWR_DEBUG_CONTINUE(" -- no driver registered\n");
            gate_clocks( drv->clocks );
        }
        else
        {
            PWR_DEBUG_CONTINUE("\n");

            if (suspend_devices_rooted(&(pcidev->dev), pm_message))
            {
                // Failure causes the kernel to set its async_error global. This
                // causes any subsequent attempts to suspend any driver to be
                // silently ignored. The kernel normally clears this variable
                // when it completes recovery from an aborted suspend-to-RAM
                // sequence. But we are not in a STR sequence. This kernel patch
                // allows us to clear it.
                clear_async_error();
                PWR_ERROR("SUSPEND FAILED FOR %s", drv->name);
                ret = ICEPM_ERR_DEV_BUSY;
            }
            else
            {
                pci_set_power_state(pcidev,PCI_D3hot);
            }
        }
        
        if (pcidev)
        {
            pci_dev_put(pcidev);
        }

        if (ret != ICEPM_OK)
        {
            //------------------------------------------------------------------
            // Abort suspend and back out all drivers we just suspended
            //------------------------------------------------------------------
            PWR_DEBUG("BACKOUT SUSPENDED DRIVERS\n");
            resume_drivers(drv_list);
            break;
        }
    }
    return ret;
}

//=============================================================================
//   E X P O R T E D   V I A   S O C - S P E C I F I C   V E C T O R
//=============================================================================

//-----------------------------------------------------------------------------
// If we are in the STR sequence, after the last driver is suspended Linux will
// transition to the CEFDK which will handle all clock gating, resets, and
// (through the PUnit) power islands. We don't have to do anything.
//
// For STANDBY3, Linux will not be shutting down the CPU, so we will not be
// going through the CEFDK; so we have to do all the work.
//
// Per CE4200 PUnit designer, "proper" shut-down for any device that
// has a PHY on-chip (e.g., SATA, USB, PCIe, VDAC, HDMI Tx/Rx) requires:
//
//  - device driver: put device I/Os  into lowest power state (may be a
//    combination of writes to both MAC/PHY).
//
//  - icepm: clock gate the device(s) but DO NOT RESET them.  Reset
//    clears the signals between MAC and PHY, possibly causing PHY to
//    come out of low power
//
//  - PUnit: turn-on firewalls (which properly hold the signals between
//    MAC/PHY to keep the PHY in low power), then put devices into reset
//-----------------------------------------------------------------------------
static void  power_down_4200(driver_t *drv)
{
    if ( ! in_STR )
    {
        // See the block comment for mux_clocks, viddec_clocks, and
        // videnc_clocks (above) for the special considerations for the
        // mux, viddec, and videnc drivers.
        //
        // pcie1 and pcie2 share clocks/resets. Do nothing until the
        // driver has called to shutdown BOTH devices.

        if (((drv == &drv_pcie1) && ! drv_pcie2.is_suspended )
        ||  ((drv == &drv_pcie2) && ! drv_pcie1.is_suspended ))
        {
            ;
        }
        else if (drv == &drv_mux)
        {
            if (drv_viddec.is_suspended)
            {
                gate_clocks(drv_mux.clocks);
            }
        }
        else if (drv == &drv_viddec)
        {
            if (drv_mux.is_suspended)
            {
                gate_clocks(drv_mux.clocks);
            }
            if (drv_videnc.is_suspended)
            {
                gate_clocks(drv_viddec.clocks);
            }
        }
        else if (drv == &drv_videnc)
        {
            gate_clocks(drv_videnc.clocks);
            if (drv_viddec.is_suspended)
            {
                gate_clocks(drv_viddec.clocks);
            }
        }
        else
        {
            gate_clocks(drv->clocks);
        }

        // TODO: For runtime_pm: scoreboard new state of driver's devices and
        // turn off their power island(s) if appropriate.
    }
    drv->is_suspended = true;
}

//-----------------------------------------------------------------------------
// If we are in the STR sequence, before the first driver is resumed the
// CEFDK/PUnit will handle all clock gating, resets, and power islands. We
// don't have to do anything.
//
// For STANDBY3, we never went to the CEFDK, so we have to do all the work.
//
// Per CE4200 PUnit designer, for each device should
//  - enable clocks first,
//  - then take device out of reset
//-----------------------------------------------------------------------------
static void  power_up_4200(driver_t *drv)
{
#define UNGATE(DRV) { \
        enter_reset(DRV);/* Make sure devices are actually in reset */ \
        enable_clocks(DRV); /* Enable clocks */ \
        exit_reset(DRV);    /* Take out of reset */ \
    }

    if ( ! in_STR )
    {
        // TODO: For runtime_pm: scoreboard new state of driver's devices and
        // turn on their power island(s) if appropriate.

        // See the block comment for mux_clocks, viddec_clocks, and
        // videnc_clocks (above) for the special considerations for the
        // mux, viddec, and videnc drivers.
        //
        // pcie1 & pcie2 share clocks/resets. Only manipulate them when the
        // FIRST one comes out of suspend.

        if (((drv == &drv_pcie1) && ! drv_pcie2.is_suspended )
        ||  ((drv == &drv_pcie2) && ! drv_pcie1.is_suspended ))
        {
            ;
        }
        else if (drv == &drv_mux)
        {
            if (drv_viddec.is_suspended)
            {
                UNGATE(drv_mux.clocks);
            }
        }
        else if (drv == &drv_viddec)
        {
            if (drv_videnc.is_suspended)
            {
                UNGATE(drv_viddec.clocks);
            }
            if (drv_mux.is_suspended)
            {
                UNGATE(drv_mux.clocks);
            }
        }
        else if (drv == &drv_videnc)
        {
            UNGATE(drv_videnc.clocks);
            if (drv_viddec.is_suspended)
            {
                UNGATE(drv_viddec.clocks);
            }
        }
        else
        {
            UNGATE(drv->clocks);
            if ((drv == &drv_pcie1) || (drv == &drv_pcie2))
            {
                // PCIe may require an additional 50ms to recover link and
                // recognize end point device before resuming driver.
                mdelay(50);
            }
        }
    }
    drv->is_suspended = false;
}


static
icepm_ret_t set_mode_4200(char *mode)
{
    icepm_ret_t ret = ICEPM_OK;

    if ( !strcasecmp(mode, "ON") )
    {
        punit_mode(0);          // Turn island on BEFORE resuming drivers
        ret = resume_drivers(island3_drivers);
        in_standby3 = false;
    }
    else if ( !strcasecmp(mode, "STANDBY3") )
    {
        if ( ICEPM_OK == (ret = suspend_drivers(island3_drivers)) )
        {
            // Turn island off AFTER suspending drivers
            punit_mode(SSC_ISLAND3);
        }
        in_standby3 = true;
    }
    else
    {
        PWR_ERROR("Undefined power mode: %s\n", mode);
        ret = ICEPM_ERR_UNKNOWN_MODE;
    }

    return ret;
}


//------------------------------------------------------------------------------
// Produce proc file output through seq file
//------------------------------------------------------------------------------
static void _procfs_state(struct seq_file *sfile)
{
    seq_printf(sfile, "ON  Island 1\n");
    procfs_print_drivers(sfile, island1_drivers);

    seq_printf(sfile, "ON  Island 2\n");
    procfs_print_drivers(sfile, island2_drivers);

    seq_printf(sfile, "ON  Island 2.5\n");
    procfs_print_drivers(sfile, island2p5_drivers);

    seq_printf(sfile, "%s Island 3\n", in_standby3 ? "off" : "ON ");
    procfs_print_drivers(sfile, island3_drivers);
}


//  Exported vector
static soc_t functions =
{
    all_drivers,
    power_down_4200,
    power_up_4200,
    punit_init,
    punit_mask_wake_event,
    punit_unmask_wake_event,
    punit_pre_sleep,
    punit_post_sleep,
    set_mode_4200,
    punit_event_map,
    _procfs_state
};


//=============================================================================
//                      E N T R Y   P O I N T
//=============================================================================

//-----------------------------------------------------------------------------
// ce4200_init
//-----------------------------------------------------------------------------
soc_t * ce4200_init(pal_soc_info_t *soc_info)
{
    return &functions;
}
