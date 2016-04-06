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
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Internal header file containing items used by in both user space and kernel
// space.
//-----------------------------------------------------------------------------

#ifndef __PWR_COMMON_H_
#define __PWR_COMMON_H_

#include <stdint.h>
#include "intel_ce_pm.h"

//-----------------------------------------------------------------------------
// IOCTL info for user-space interface
//-----------------------------------------------------------------------------
#include <asm/ioctl.h>

typedef struct
{
    // [in] required for all ioctls
    unsigned        header_major;
    unsigned        header_minor;

    // [out] command success/failure status
    icepm_ret_t     ret;

    union {
        // [in]
        // Mode name for SETMODE.
        // Driver name for REGISTER.
        struct
        {
            char *      buffer; // Location of string
            unsigned    length; // Length of string, including terminator
        } name;

        // [out] Event returned by WAIT
        icepm_event_t   event;

        // [in] Callback return code sent with DONE
        unsigned        callback_rc;

        // [in/out] Wake event for WAKE_MASK, WAKE_UNMASK, WAKE_GET
        IO_8051_wake_event_t wake_event;
    };
} ioctl_arg_t;

#define PWR_IOC_MAGIC   'p'

#define PWR_IOC_SETMODE     _IOW (PWR_IOC_MAGIC, 1, ioctl_arg_t)
#define PWR_IOC_REGISTER    _IOWR(PWR_IOC_MAGIC, 2, ioctl_arg_t)
#define PWR_IOC_UNREGISTER  _IOW (PWR_IOC_MAGIC, 3, ioctl_arg_t)
#define PWR_IOC_WAIT        _IOWR(PWR_IOC_MAGIC, 4, ioctl_arg_t)
#define PWR_IOC_DONE        _IOW (PWR_IOC_MAGIC, 5, ioctl_arg_t)
#define PWR_IOC_WAKE_MASK   _IOW (PWR_IOC_MAGIC, 6, ioctl_arg_t)
#define PWR_IOC_WAKE_UNMASK _IOW (PWR_IOC_MAGIC, 7, ioctl_arg_t)
#define PWR_IOC_WAKE_GET    _IOW (PWR_IOC_MAGIC, 8, ioctl_arg_t)

//----------------------------------------------------------------------------
// Message printing
//----------------------------------------------------------------------------
#ifdef __KERNEL__
    #include <linux/kernel.h> /* for printk */
    #define IO_TAG KERN_ERR "ICEPM: "
    #define PRT    printk
#else
    #include <stdio.h>
    #define IO_TAG "ICEPM: " __BASE_FILE__ ":"
    #define PRT    printf
#endif

// Error Printing
#define PWR_ERROR(arg...){ PRT(IO_TAG"%s:ERROR: ", __func__); PRT(arg); }
#define PWR_WARN(arg...) { PRT(IO_TAG"%s:WARNING: ", __func__); PRT(arg); }
#define PWR_PRINT(arg...){ PRT(IO_TAG"%s: ", __func__); PRT(arg); }

// Debug printing
#ifdef __KERNEL__
    extern unsigned _icepm_trace_enabled;

    #define PWR_DEBUG(arg...)           \
        if (_icepm_trace_enabled)       \
        {                               \
            PRT(IO_TAG"%s: ", __func__);\
            PRT(arg);                   \
        }

    // Debug output that does not generate a tag.
    // For continuing debug lines started with PWR_DEBUG
    #define PWR_DEBUG_CONTINUE(arg...) if (_icepm_trace_enabled){ PRT(arg); }
#endif


#endif // __PWR_COMMON_H_
