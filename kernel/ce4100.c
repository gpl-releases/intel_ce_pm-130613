
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

#include <linux/string.h>

#include "icepm_internal.h"
#include "kernel.h"

//-----------------------------------------------------------------------------
// Power management for CE4100
//
// The only thing ever supported for this SoC was a standby power mode, in which
// clocks were gated for the units controlled by the following drivers:
//      - audio
//      - display
//      - graphics
//      - viddec
//      - vidpproc
//
// This was originally an experimental feature.  Reportedly, in the final
// analysis it only saved 10-20mW.
// 
// The feature has been removed - the interfaces here are no-ops in case some
// customer code actually calls icepm_set_mode() on CE4100.
//-----------------------------------------------------------------------------

static driver_t *all_drivers[] = { NULL };

//=============================================================================
//   E X P O R T E D   V I A   S O C - S P E C I F I C   V E C T O R
//=============================================================================

static
icepm_ret_t set_mode_4100(char * mode_name)
{
    icepm_ret_t ret = ICEPM_OK;

    if ( strcasecmp(mode_name,"ON") && strcasecmp(mode_name, "STANDBY") )
    {
        PWR_ERROR("Undefined power mode: %s\n", mode_name);
        ret = ICEPM_ERR_UNKNOWN_MODE;
    }

    return ret;
}

static icepm_ret_t punit_init_4100(pal_soc_info_t *soc)
{
    return ICEPM_OK;
}

static void power_down_4100(driver_t *drv)
{
    drv->is_suspended = true;
}

static void power_up_4100(driver_t *drv)
{
    drv->is_suspended = false;
}

static icepm_ret_t mask_unmask_event_4100(IO_8051_wake_event_t wake_event)
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
    seq_printf(sfile, "No power management for this SoC\n");
}

//  Exported vector
static soc_t functions =
{
    all_drivers,
    power_down_4100,
    power_up_4100,
    punit_init_4100,
    mask_unmask_event_4100,
    mask_unmask_event_4100,
    _pre_post_sleep,
    _pre_post_sleep,
    set_mode_4100,
    _procfs_event_map,
    _procfs_state
};

//=============================================================================
//                      E N T R Y   P O I N T
//=============================================================================

soc_t *ce4100_init(pal_soc_info_t *soc_info)
{
    return &functions; 
}
