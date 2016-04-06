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

//----------------------------------------------------------------------------
// Internal header file to be included by all files in kernel module.
//----------------------------------------------------------------------------

#ifndef _KERNEL_H_
#define _KERNEL_H_

#include <linux/pci.h>
#include <linux/fs.h>       // For struct file
#include <linux/types.h>    // For pid_t
#include <linux/seq_file.h>

#include "pal.h"
#include "osal.h"
#include "clock_control.h"

#include "intel_ce_pm.h"

// Type of a clock_control resource (register bit field in cp_top unit)
typedef enum
{
    CLK_CLOCK,          // Resource is used to gate/enable a device clock
    CLK_RESET           // Resource is used to put device into or out of reset
} clock_type_t;

typedef struct
{
    char          * name;   // Clock resource name, as ASCII string;
                            //      NULL terminates list of clocks
    clock_control_resource_t res;
                            // Clock resource name passed to clock_control
    clock_type_t    type;   // Does resource control a clock or a reset?
    unsigned        on;     // Value to write to this resource to run the device
                            //      (enable clock or take out of reset)
    unsigned        off;    // Value to write to this resource to halt the
                            //      device (gate clock or put into reset)
} clock_res_t;

// Initializer macros for a clock_res_t
#define CLOCK(name, on, off) { #name, CLOCK_##name, CLK_CLOCK, on, off }
#define RESET(name, on, off) { #name, CLOCK_##name, CLK_RESET, on, off }

// Kernel PCI driver
typedef struct
{
    char          * name;           // Name of driver
    struct pci_device_id * pci_id;  // List of PCI vender/revision IDs of PCI
                                    //   devices controlled by this driver.
    bool            is_suspended;   // Current state of driver
    clock_res_t   * clocks;         // Clock_control resources for clocks/resets
                                    //   of device(s) controlled by this driver.

    // SUPPORT FOR LINUX runtime_pm.  THESE VARIABLES MAY BE NULL IF WE DON'T
    // SUPPORT runtime_pm ON THE TARGET SoC.

    void          * island;         // Pointer to SOC-specific data structure
                                    //   describing the power island on which
                                    //   the driver's device(s) are located.
    uint16_t        drv_id;         // Bit flag (single bit is set) that will be
                                    //   used to identify this driver. The value
                                    //   here is guaranteed unique for each
                                    //   driver on the same island, so they can
                                    //   be OR'd together to summarize island
                                    //   state.
} driver_t;


// Declare and initialize a driver_t
#define DECL_PCI_DRIVER(NAME)               \
    static driver_t drv_##NAME =            \
    {                                       \
        .name           = #NAME,            \
        .is_suspended   = false,            \
        .clocks         = NAME##_clocks,    \
        .pci_id         = id_##NAME,        \
    };


// Current Suspend-to-RAM state
//  true  => We are somewhere in the STR sequence: any time between suspend of
//           first driver and resume of last driver.
//  false => Not in STR sequence.

extern bool in_STR;

// Shorthand to do a blocking write of a clock_control (cp_top) resource
#define CCW(clock,value) clock_control_write(clock,value,CLOCK_TRUE)

//=============================================================================
// Info exported by the SOC-specific init() code
//=============================================================================

#define NUM_EVENT_MAP_ENTRIES 32

// NOTE:  the mask_wake_event() and unmask_wake_event() functions update the
// current mask, but it is not used to actually update the mask register until
// the enable_events() function is called.
// 
// This is because CE4200/CE5300 PUnits will process wake events even during
// Active power management, causing power islands to unexpectedly power back
// on.  Therefore, we normally mask *all* events (via disable_events())
// and only write the user-specified mask just before entering STR or Soft Off.
typedef struct
{
    // NULL-terminated list of all drivers
    driver_t **  drivers;

    //--------------------------------------------------
    // Callback functions for SoC-specific functionality
    //--------------------------------------------------

    // Power-down all devices associated with the specified driver.
    // Function may assume devices are currently powered up.
    void        (*power_down)(driver_t *drv);

    // Power-up all devices associated with the specified driver.
    // Function may assume devices are currently powered up.
    void        (*power_up)(driver_t *drv);

    // Initialize code related to the PUnit of the SoC
    icepm_ret_t (*init_punit)(pal_soc_info_t *soc);

    // Change the wake event mask for the SoC
    icepm_ret_t (*mask_wake_event)(IO_8051_wake_event_t event);
    icepm_ret_t (*unmask_wake_event)(IO_8051_wake_event_t event);

    // Actions to take before STR or Soft Off
    void        (*pre_sleep)(void);

    //Actions to take after resume from STR or on CPU power on
    void        (*post_sleep)(void);

    // Change current active power mode. Ignored if NULL.
    // Only CE4200 implements an active power mode (STANDBY3), and that will be
    // deprecated once Linux runtime_pm is supported.
    icepm_ret_t (*set_mode) (char * mode);

    // Dump wake event map through procfs
    void        (*procfs_event_map)(struct seq_file *sfile);

    // Dump island/device power state through procfs
    void        (*procfs_state)(struct seq_file *sfile);
} soc_t;


//==============================================================================
//          P C I   V E N D O R   I D
//==============================================================================
#define VENDOR_ID_INTEL     0x8086


//==============================================================================
// PCI ID tables for drivers whose devices are power-managed by the SoC.
//==============================================================================

extern struct pci_device_id id_audio[];
extern struct pci_device_id id_demux[];
extern struct pci_device_id id_display[];
extern struct pci_device_id id_eMMC[];
extern struct pci_device_id id_gbe[];
extern struct pci_device_id id_graphics[];
extern struct pci_device_id id_graphics_2D[];
extern struct pci_device_id id_hdmi_rx_ce[];
extern struct pci_device_id id_iosf[];
extern struct pci_device_id id_ismdclock[];
extern struct pci_device_id id_mspod[];
extern struct pci_device_id id_mux[];
extern struct pci_device_id id_nexp[];
extern struct pci_device_id id_pcie1[];
extern struct pci_device_id id_pcie2[];
extern struct pci_device_id id_pwm[];
extern struct pci_device_id id_sata[];
extern struct pci_device_id id_sec[];
extern struct pci_device_id id_sven[];
extern struct pci_device_id id_tsout[];
extern struct pci_device_id id_usb[];
extern struct pci_device_id id_vidcap[];
extern struct pci_device_id id_viddec[];
extern struct pci_device_id id_videnc[];
extern struct pci_device_id id_vidpproc[];

int init_ids( pal_soc_info_t *soc_info );


//==============================================================================
// Initialization of SoC-specific code
//==============================================================================
soc_t *ce2600_init(pal_soc_info_t *soc_info);
soc_t *ce4100_init(pal_soc_info_t *soc_info);
soc_t *ce4200_init(pal_soc_info_t *soc_info);
soc_t *ce5300_init(pal_soc_info_t *soc_info);

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void check_drivers(void);

//-----------------------------------------------------------------------------
// Clock/reset manipulation via cp_top unit
//-----------------------------------------------------------------------------
void cp_top( clock_res_t *clk, clock_type_t type, bool on );

// Gate all clocks in passed NULL-terminated list.
static __inline void gate_clocks(clock_res_t *clocks)
{
    cp_top(clocks, CLK_CLOCK, false);
}

// Enable all clocks in passed NULL-terminated list.
static __inline void enable_clocks(clock_res_t *clocks)
{
    cp_top(clocks, CLK_CLOCK,true);
}

// Assert all resets in passed NULL-terminated list.
static __inline void enter_reset(clock_res_t *clocks)
{
    cp_top(clocks, CLK_RESET, false);
}

// De-assert all resets in passed NULL-terminated list.
static __inline void exit_reset(clock_res_t *clocks)
{
    cp_top(clocks, CLK_RESET, true); 
}

//----------------------------------------------------------------------------
// proc filesystem
//----------------------------------------------------------------------------
int procfs_init(soc_t *soc);
void procfs_deinit(void);
void procfs_print_drivers(struct seq_file *s, driver_t **drivers);

//----------------------------------------------------------------------------
// IOSF-SB access to hardware components
//----------------------------------------------------------------------------
#include "iosf.h"
extern iosf_handle iosfh;

//-----------------------------------------------------------------------------
// User-space event notification
//-----------------------------------------------------------------------------
int         us_init(void);
void        us_cleanup(void);
icepm_ret_t us_register(char *name, struct file *filep);
void        us_unregister(struct file *filep);
icepm_ret_t us_wait_for_event(struct file *filep, icepm_event_t *p_event);
icepm_ret_t us_event_acknowledged(struct file *filep, int rc);
int         us_notify_all_clients(icepm_event_t event);

//-----------------------------------------------------------------------------
// CE4200/CE5300 PUnit
//-----------------------------------------------------------------------------
icepm_ret_t punit_init(pal_soc_info_t *soc_info);
void        punit_close(void);
void        punit_mode(uint32_t ssc);
icepm_ret_t punit_mask_wake_event(IO_8051_wake_event_t event);
icepm_ret_t punit_unmask_wake_event(IO_8051_wake_event_t event);
void        punit_pre_sleep(void);
void        punit_post_sleep(void);
void        punit_event_map(struct seq_file *sfile);

extern IO_8051_wake_event_t wake_event;

#endif // _KERNEL_H_
