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

#include <linux/pci.h>

#include "icepm_internal.h"
#include "kernel.h"


// PCI ID table for a single Intel device
#define IDTAB(DRV_NAME, DEV_ID)                     \
    struct pci_device_id id_##DRV_NAME[] =          \
    {                                               \
        { PCI_DEVICE(VENDOR_ID_INTEL, DEV_ID) },    \
        { 0 }                                       \
    }

// PCI ID table for a single Intel device,
// but that is equal in length to an IDTAB_2 table
#define IDTAB_(DRV_NAME, DEV_ID)                    \
    struct pci_device_id id_##DRV_NAME[] =          \
    {                                               \
        { PCI_DEVICE(VENDOR_ID_INTEL, DEV_ID) },    \
        { 0 },                                      \
        { 0 }                                       \
    }

// PCI ID table for two Intel devices
#define IDTAB_2(DRV_NAME, DEV_ID1, DEV_ID2)         \
    struct pci_device_id id_##DRV_NAME[] =          \
    {                                               \
        { PCI_DEVICE(VENDOR_ID_INTEL, DEV_ID1) },   \
        { PCI_DEVICE(VENDOR_ID_INTEL, DEV_ID2) },   \
        { 0 }                                       \
    }

// PCI ID table for a single external (non-Intel) device
#define IDTAB_EXT(DRV_NAME, VENDOR_ID, DEV_ID)      \
    struct pci_device_id id_##DRV_NAME[] =          \
    {                                               \
        { PCI_DEVICE(VENDOR_ID, DEV_ID) },          \
        { 0 }                                       \
    }

        IDTAB_2 ( audio,            0x2E5F, 0x2E60  ); // Audio DSPs, Audio IF
        IDTAB_2 ( display,          0x2E61, 0x2E63  ); // VDC, HDMI Tx
        IDTAB   ( eMMC,             0x070B          ); // EMMC
        IDTAB   ( gbe,              0x2E6E          ); // GBE
        IDTAB   ( graphics_2D,      0x070A          ); // GC300
        IDTAB   ( hdmi_rx_ce,       0x089E          ); // HDMI Rx
        IDTAB   ( iosf,             0x0C40          ); // CE5300 host/root
        IDTAB   ( ismdclock,        0x2E6F          ); // CRU_ETH
        IDTAB   ( mspod,            0x2E6B          ); // MSPOD
        IDTAB   ( nexp,             0x2E65          ); // EXP
        IDTAB   ( pwm,              0x089F          ); // PWM
        IDTAB_2 ( sec,              0x2E64, 0x0702  ); // SEC, MEU
        IDTAB   ( sven,             0x2E6D          ); // DFX
        IDTAB   ( tsout,            0x2E5D          ); // TS Prefilter
        IDTAB_EXT(usb,              0x192E, 0x0101  ); // USB
        IDTAB   ( vidcap,           0x0704          ); // HDVCAP
        IDTAB   ( vidpproc,         0x2E62          ); // DPE

// 3D Graphics device on CE4200 had different PCI ID from subsequent SoCs
        IDTAB   ( graphics,         0x089B          ); // SIMG
static  IDTAB   ( graphics_4200,    0x2E5B          ); // SIMG

// PCIE devices had different PCI IDs on CE4200
        IDTAB   ( pcie1,            0x0899          ); // PCIE
        IDTAB   ( pcie2,            0x089A          ); // PCIE
static  IDTAB   ( pcie1_4200,       0x0101          ); // PCIE
static  IDTAB   ( pcie2_4200,       0x0202          ); // PCIE

// SATA device had different PCI ID on CE4200 than on subsequent SoCs
        IDTAB   ( sata,             0x0C82          ); // SATA
static  IDTAB   ( sata_4200,        0x2E71          ); // SATA

// Demux driver shares TS Prefilter with tsout driver, tsout registers for
// prefilter.
// On CE2600: there is no tsout driver, demux registers for both devices.
        IDTAB_  ( demux,            0x0705          ); // TSD2
static  IDTAB_2 ( demux_2600,       0x0705, 0x2E5D  ); // TSD2,TS Prefilter

//-----------------------------------------------------------------------------
// The relationship of the viddec/videnc/mux drivers to devices is ugly.
//  - The devices actually present vary from SoC to SoC.
//  - The devices accessed by each driver vary from SoC to SoC.
//  - Some devices are accessed by more than one driver.
//  - Some drivers access more than one device.
//
//      device         4200 drivers  5300 drivers
//      -------------  ------------  ------------
//      089C EPU        -            ENC
//      0706 H264VE    ENC           ENC
//      089D GVsparcD   -            DEC
//      0703 GVsparcT  DEC enc       MUX enc
//      2E5C MFD       MUX dec       -
//
// The drivers shown in CAPITALS are the ones we assign to the devices. The
// assignments are done as follows:
// - For both SoCs, both EPU and GVsparcD are assigned to videnc -- it does not
//   matter that EPU will never actually be detected on CE4200.
// - Then any device with only one driver is assigned to that driver.
// - Then the devices associated with two drivers are assigned to the driver
//   that does not already have an assigned device.
//
// Note that in the SoC-specific code, devices accessed by more than one driver
// must not be put into a low power state until BOTH drivers are suspended;
// and they must be powered on before the FIRST driver is resumed.
//-----------------------------------------------------------------------------
// 4200
static  IDTAB   ( mux_4200,        0x2E5C          ); // MFD
static  IDTAB_  ( viddec_4200,     0x0703          ); // GVsparc-T
// 5300
        IDTAB   ( mux,             0x0703          ); // GVsparc-T
        IDTAB_  ( viddec,          0x089D          ); // GVsparc-D
// Both
        IDTAB_2 ( videnc,          0x0706, 0x089C  ); // H264VE, EPU


// Return 0 on success, 1 on failure
int init_ids( pal_soc_info_t *soc_info )
{
    int ret = 0;

    if (( sizeof(id_graphics_4200)  != sizeof(id_graphics) )
    ||  ( sizeof(id_mux_4200)       != sizeof(id_mux)      )
    ||  ( sizeof(id_pcie1_4200)     != sizeof(id_pcie1)    )
    ||  ( sizeof(id_pcie2_4200)     != sizeof(id_pcie2)    )
    ||  ( sizeof(id_sata_4200)      != sizeof(id_sata)     )
    ||  ( sizeof(id_demux_2600)     != sizeof(id_demux)    )
    ||  ( sizeof(id_viddec_4200)    != sizeof(id_viddec)   ))
    {
        PWR_ERROR("Internal error: make id table sizes equal\n");
        ret = 1;
    }
    else
    {
        switch ( soc_info->name )
        {
        case SOC_NAME_CE2600:
            memcpy(id_demux   , id_demux_2600   , sizeof(id_demux   ) );
            break;
        case SOC_NAME_CE4200:
            memcpy(id_graphics, id_graphics_4200, sizeof(id_graphics) );
            memcpy(id_mux     , id_mux_4200     , sizeof(id_mux     ) );
            memcpy(id_pcie1   , id_pcie1_4200   , sizeof(id_pcie1   ) );
            memcpy(id_pcie2   , id_pcie2_4200   , sizeof(id_pcie2   ) );
            memcpy(id_viddec  , id_viddec_4200  , sizeof(id_viddec  ) );
            memcpy(id_sata    , id_sata_4200    , sizeof(id_sata    ) );
            break;
        default:
            break;
        }
    }

    return ret;
}
