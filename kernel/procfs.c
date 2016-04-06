//-----------------------------------------------------------------------------
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// GPL LICENSE SUMMARY
//
// Copyright(c) 2012-2013 Intel Corporation. All rights reserved.
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
// Copyright(c) 2012-2013 Intel Corporation. All rights reserved.
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
#include <linux/proc_fs.h>
#include <linux/ctype.h>

#include "icepm_internal.h"
#include "kernel.h"

#ifdef DEBUG
unsigned _icepm_trace_enabled = 1;
#else
unsigned _icepm_trace_enabled = 0;
#endif

EXPORT_SYMBOL(_icepm_trace_enabled);

static soc_t *_soc = NULL;

static struct proc_dir_entry * proc_icepm = NULL;


//----------------------------------------------------------------------------
// Generic seq_file operations
//----------------------------------------------------------------------------
static void * _seq_start(struct seq_file * file, loff_t * offset)
{
    static int position = 0;
    return (*offset == 0) ? &position : NULL;    
}

static void * _seq_next(struct seq_file * file, void * v, loff_t * offset)
{
    return NULL;
}

static void _seq_stop(struct seq_file * file, void * v)
{
}

//----------------------------------------------------------------------------
// /proc/icepm/trace
//
// Write integer to set debug trace mode.
// Read to determine current mode.
//
// Currently, only the following values are supported:
//      0: no debug output
//      non-zero: print debug trace
//----------------------------------------------------------------------------

static struct proc_dir_entry * proc_icepm_trace = NULL;

static
int trace_read( char*   page,
                char**  start,
                off_t   off,
                int     count,
                int*    eof,
                void*   data)
{
    return sprintf(page, "%d\n", _icepm_trace_enabled);
}

static
int trace_write(struct file*    file,
                const char*     buffer,
                unsigned long   count,
                void*           data)
{
    char        my_buf[10];
    int         i;
    int         ret     = count;
    unsigned    val     = 0;
    bool        is_hex  = false;

    if (count > sizeof(my_buf))
    {
        PWR_ERROR("Input too long\n");
        ret = -EINVAL;
        goto exit;
    }

    if (copy_from_user(my_buf, buffer, count))
    {
        PWR_ERROR("copy_from_user() failed");
        ret = -EFAULT;
        goto exit;
    }

    if ((my_buf[0]=='0') && (my_buf[1]=='x'))
    {
        i = 2;
        is_hex = true;
    }
    else
    {
        i = 0;
    }
 
    for (; (i < count) && (my_buf[i] != '\n') && (my_buf[i] != '\0'); i++)
    {
        unsigned c = my_buf[i];

        if ( is_hex )
        {
            if ( !isxdigit(c) )
            {
                PWR_ERROR("Non-numeric input[%d]: 0x%02x\n", i, c);
                ret = -EINVAL;
                goto exit;
            }

            val = (16*val);
            if (('0' <= c) && (c <= '9'))
            {
                val += (c - '0');
            }
            else if (('a' <= c) && (c <= 'f'))
            {
                val += (10 + (c - 'a'));
            }
            else
            {
                val += (10 + (c - 'A'));
            }
        }
        else
        {
            if ( !isdigit(c) )
            {
                PWR_ERROR("Non-numeric input[%d]: 0x%02x\n", i, c);
                ret = -EINVAL;
                goto exit;
            }

            val = (10*val) + (c - '0');
        }
    }

    _icepm_trace_enabled = val;

exit:
    return ret;
}

//----------------------------------------------------------------------------
// /proc/icepm/devicecheck
//
// *Any* write triggers a pass through the CE devices. Those without an assigned
// driver are powered off if possible.
//
// This is done after all CE driver startup scripts have been executed, when
// runtime_pm is enabled from user space.
//----------------------------------------------------------------------------

static struct proc_dir_entry * proc_icepm_devicecheck = NULL;

static
int devicecheck_write(  struct file*    file,
                        const char*     buffer,
                        unsigned long   count,
                        void*           data)
{
    check_drivers();
    return count;
}

//----------------------------------------------------------------------------
// /proc/icepm/eventmap
//
// Read ASCII summary of wake event map exported by 8051 firmware.
//----------------------------------------------------------------------------
static struct proc_dir_entry * proc_icepm_eventmap = NULL;

static int _seq_show_eventmap(struct seq_file *s, void *v)
{
    _soc->procfs_event_map(s);
    return 0;
}

static struct seq_operations _eventmap_seq_ops =
{
    .start  = _seq_start,
    .next   = _seq_next,
    .stop   = _seq_stop,
    .show   = _seq_show_eventmap
};

static int eventmap_open(struct inode *inode, struct file *file)
{
    // call seq_open() to connect the file with our sequence operations.
    return seq_open(file, &_eventmap_seq_ops);
}

static struct file_operations eventmap_ops =
{
    .owner  = THIS_MODULE,
    .open   = eventmap_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release= seq_release
};

//----------------------------------------------------------------------------
// /proc/icepm/state
//
// Read ASCII summary of driver and island power states.
//----------------------------------------------------------------------------
static struct proc_dir_entry * proc_icepm_state = NULL;

// Utility function used by SoC-specific code to dump the states of a list of
// drivers
void procfs_print_drivers(struct seq_file *s, driver_t **drivers)
{
    driver_t *drv;
        
    for ( drv = *drivers; drv != NULL; drv = (*++drivers) )
    {
        seq_printf(s, "%11s %s\n", drv->is_suspended ? "off" : "ON ", drv->name);
    }
}

static int _seq_show_state(struct seq_file *s, void *v)
{
    // SoC-specific code will print island state, if appropriate, and call back
    // to profs_print_drivers() to print driver states for each island. 
    _soc->procfs_state(s);
    return 0;
}

static struct seq_operations _state_seq_ops =
{
    .start  = _seq_start,
    .next   = _seq_next,
    .stop   = _seq_stop,
    .show   = _seq_show_state
};

static int state_open(struct inode *inode, struct file *file)
{
    // call seq_open() to connect the file with our sequence operations.
    return seq_open(file, &_state_seq_ops);
}

static struct file_operations state_ops =
{
    .owner  = THIS_MODULE,
    .open   = state_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release= seq_release
};

//----------------------------------------------------------------------------
// Initialize proc filesystem
//----------------------------------------------------------------------------
int procfs_init(soc_t *soc)
{
    _soc = soc;

    proc_icepm = proc_mkdir("icepm", 0);
    if (proc_icepm == 0)
    {
        PWR_ERROR("/proc/icepm/ creation failed\n");
        return -1;
    }

    ///////////////////////

    proc_icepm_trace = create_proc_entry(   "trace",
                                            S_IFREG|S_IRWXU|S_IRWXG|S_IRWXO,
                                            proc_icepm);
    if (proc_icepm_trace == 0)
    {
        PWR_ERROR("/proc/icepm/trace creation failed\n");
        return -1;
    }
    proc_icepm_trace->read_proc  = trace_read;
    proc_icepm_trace->write_proc = trace_write;

    ///////////////////////

    proc_icepm_devicecheck = create_proc_entry("devicecheck",
                                                S_IFREG|S_IRWXU|S_IRWXG|S_IRWXO,
                                                proc_icepm);
    if (proc_icepm_devicecheck == 0)
    {
        PWR_ERROR("/proc/icepm/devicecheck creation failed\n");
        return -1;
    }
    proc_icepm_devicecheck->write_proc = devicecheck_write;

    ///////////////////////

    proc_icepm_eventmap = create_proc_entry("eventmap",
                                            S_IFREG|S_IRWXU|S_IRWXG|S_IRWXO,
                                            proc_icepm);
    if (proc_icepm_eventmap == 0)
    {
        PWR_ERROR("/proc/icepm/eventmap creation failed\n");
        return -1;
    }
    proc_icepm_eventmap->proc_fops = &eventmap_ops;

    ///////////////////////

    proc_icepm_state = create_proc_entry(   "state",
                                            S_IFREG|S_IRWXU|S_IRWXG|S_IRWXO,
                                            proc_icepm);
    if (proc_icepm_state == 0)
    {
        PWR_ERROR("/proc/icepm/state creation failed\n");
        return -1;
    }
    proc_icepm_state->proc_fops = &state_ops;

    return 0;
}

//----------------------------------------------------------------------------
// De-initialize proc filesystem
//----------------------------------------------------------------------------
void procfs_deinit(void)
{
    _icepm_trace_enabled = 0;

    if ( proc_icepm )
    {
        if ( proc_icepm_trace )
        {
            remove_proc_entry("trace", proc_icepm);
            proc_icepm_trace = NULL;
        }
    
        if (proc_icepm_devicecheck)
        {
            remove_proc_entry("devicecheck", proc_icepm);
            proc_icepm_devicecheck = NULL;
        }

        if ( proc_icepm_eventmap )
        {
            remove_proc_entry("eventmap", proc_icepm);
            proc_icepm_eventmap = NULL;
        }

        if ( proc_icepm_state )
        {
            remove_proc_entry("state", proc_icepm);
            proc_icepm_state = NULL;
        }

        remove_proc_entry("icepm", 0);
        proc_icepm = NULL;
    }
}
