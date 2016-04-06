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

#include <linux/version.h>
#include <linux/moduleparam.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/pci-intelce_pm.h>
            // Contains Intel CE kernel patches for functions:
            //      intel_pm_register_callback()
            //      suspend_devices_rooted()
            //      resume_devices_rooted()
            //      clear_async_error()
#include <linux/string.h>
#include <linux/suspend.h>          // For suspend/hibernate notifiers

#include "osal.h"
#include "clock_control.h"

#include "icepm_internal.h"
#include "kernel.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Intel Corporation, (C) 2009-2013 - All Rights Reserved");

#define MOD_NAME "intel_ce_pm"

bool        in_STR =  false;
iosf_handle iosfh = NULL;


static os_lock_t        cmd_mutex   = NULL;
static int              major_number= -1;
static pal_soc_info_t   soc_info;
static soc_t          * soc;

static char current_mode[10] = "ON";

void icepm_trace(const char *driver_name, ...)
{
    if (_icepm_trace_enabled)
    {
        va_list ap;
        char *fmt;

        va_start(ap, driver_name);
        printk("ICEPM: %s - ", driver_name);
        fmt = va_arg(ap, char *);
        vprintk(fmt, ap);
        va_end(ap); 
    }
}
EXPORT_SYMBOL(icepm_trace);

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
static
icepm_ret_t validate_version( unsigned version_major, unsigned version_minor)
{
    icepm_ret_t ret = ICEPM_OK;

    if ((ICEPM_HEADER_VERSION_MAJOR != version_major)
    ||  (ICEPM_HEADER_VERSION_MINOR  < version_minor))
    {
        PWR_ERROR("\n"
            "libicepm.so incompatible with intel_ce_pm.ko\n"
            "    libicepm header version: %d.%d\n"
            "    intel_ce_pm.ko header version: %d.%d\n",
            ICEPM_HEADER_VERSION_MAJOR, ICEPM_HEADER_VERSION_MINOR,
            version_major, version_minor
            );
        ret = ICEPM_ERR_INCOMPATIBLE;
    }

    return ret;
}


//-----------------------------------------------------------------------------
// get_drv
//
// Given driver PCI device ID, return pointer to driver descriptor
// Return NULL if driver not defined.
//-----------------------------------------------------------------------------
static
driver_t * get_drv(unsigned short vendor, unsigned short devid)
{
    int                     i;
    struct pci_device_id *  id;

    for (i=0; soc->drivers[i] != NULL; i++)
    {
        driver_t *drv = soc->drivers[i];

        for (id = drv->pci_id; id->vendor != 0; id++ )
        {
            if ((id->vendor == vendor) && (id->device == devid))
            {
                goto found;
            }
        }
    }

found:
    return soc->drivers[i];
}


//==============================================================================
//       CLOCK/RESET MANIPULATION FOR DEVICE POWER STATE TRANSITIONS
//==============================================================================

//-----------------------------------------------------------------------------
// Utility to set selected cp_top register bits to a specified state.
// 
// Parameters:
//
// clk      NULL-terminated list of clock resources (identifiers of cp_top
//          register bits).
//
// type     Process only the resets (if CLK_RESET) or clocks (if CLK_CLOCK) in
//          the list of resources.
//
// Set clock resource state according to this table:
//
//            | type==CLK_RESET  | type==CLK_CLOCK
//   ---------+------------------+----------------
//   on==true | Take out of reset| Enable clocks
//   ---------+--------- --------+----------------
//   on==false| Put into reset   | Gate clocks
//-----------------------------------------------------------------------------
void cp_top(clock_res_t *clk, clock_type_t type, bool on )
{
    clock_control_ret_t ret;
    bool                first_print = true;


    for ( ; clk->name != NULL; clk++)
    {
        if ( clk->type == type )
        {
            if ( first_print )
            {
                PWR_DEBUG("%s:", on ? "ON" : "OFF" );
                first_print = false;
            }
            PWR_DEBUG_CONTINUE(" %s", clk->name);

            ret = CCW( clk->res, on ? clk->on : clk->off );
            if ( ret != CLOCK_RET_OK )
            {
                // Should never happen, nothing we can do if it does.
                PWR_ERROR(  "\n****** CATASTROPHIC ERROR %u on "
                            "clock_control_write(%s)\n",
                            ret,
                            clk->name);
            }
        }
    }
    if ( ! first_print )
    {
        PWR_DEBUG_CONTINUE("\n");
    }
}


//==============================================================================
//           A C T I V E   P M   M O D E   C H A N G E  S U P P O R T
//==============================================================================


//-----------------------------------------------------------------------------
// _set_mode
// Support for Active Power modes outside of Linux runtime_pm.
// Only older SoCs have this, and it will be deprecated when runtime_pm is
// implemented.
//-----------------------------------------------------------------------------
static
icepm_ret_t _set_mode(char * mode)
{
    icepm_ret_t rc;

    if (soc->set_mode == NULL)
    {
        PWR_ERROR("Unsupported power mode: %s\n", mode);
        rc = ICEPM_ERR_UNKNOWN_MODE;
    }
    else if ( !strcasecmp(current_mode, mode) )
    {
        // We are already in that mode -- nothing to do
        PWR_DEBUG("Already in '%s'\n", mode);
        rc = ICEPM_OK;
    }
    else
    {
        rc = soc->set_mode(mode);

        // If the mode change was successful (or "partially" successful)
        // update the global
        if ((rc == ICEPM_OK) || (rc == ICEPM_ERR_INCOMPLETE))
        {
            // Save the current mode
            if (sizeof(current_mode) < strlen(mode)+1)
            {
                PWR_ERROR("Variable too short for mode name '%s'\n", mode);
                rc = ICEPM_ERR_INTERNAL;
            }
            else
            {
                strcpy(current_mode, mode);
            }
        }
    }

    return rc;
}


//============================================================================
//              K E R N E L   L E V E L   I N T E R F A C E
//============================================================================

//-----------------------------------------------------------------------------
// icepm_set_mode
//-----------------------------------------------------------------------------
icepm_ret_t icepm_set_mode(char *mode)
{
    icepm_ret_t rc;

    os_lock(cmd_mutex);

    PWR_DEBUG("Enter mode '%s':\n", mode);
    rc = _set_mode(mode);

    os_unlock(cmd_mutex);
    return rc;
}
EXPORT_SYMBOL(icepm_set_mode);

//-----------------------------------------------------------------------------
// icepm_mask_wake_event
//-----------------------------------------------------------------------------
icepm_ret_t icepm_mask_wake_event(IO_8051_wake_event_t event)
{
    icepm_ret_t rc;

    os_lock(cmd_mutex);
    rc = soc->mask_wake_event(event);
    os_unlock(cmd_mutex);
    return rc;
}
EXPORT_SYMBOL(icepm_mask_wake_event);

//-----------------------------------------------------------------------------
// icepm_unmask_wake_event
//-----------------------------------------------------------------------------
icepm_ret_t icepm_unmask_wake_event(IO_8051_wake_event_t event)
{
    icepm_ret_t rc;

    os_lock(cmd_mutex);
    rc = soc->unmask_wake_event(event);
    os_unlock(cmd_mutex);
    return rc;
}
EXPORT_SYMBOL(icepm_unmask_wake_event);

//-----------------------------------------------------------------------------
// icepm_get_wake_reason
//-----------------------------------------------------------------------------
icepm_ret_t icepm_get_wake_reason(IO_8051_wake_event_t *event)
{
    os_lock(cmd_mutex);
    *event = wake_event;
    os_unlock(cmd_mutex);
    return ICEPM_OK;
}
EXPORT_SYMBOL(icepm_get_wake_reason);


//============================================================================
//     L I N U X   F I L E   O P E R A T I O N S (USER-LEVEL INTERFACE)
//============================================================================

//----------------------------------------------------------------------------
// Allocate storage for and retrieve an ASCII string from user space.
// 
// Args:
// str [out]    return pointer to local copy here.
// address      the user space address of the string.
// length       length of string, including terminator ('\0').
//
// It is the caller's responsibility to free the memory when done with it.
//----------------------------------------------------------------------------
static
icepm_ret_t get_user_space_string(char **str, void *address, unsigned length)
{
    icepm_ret_t ret = ICEPM_OK;

    *str = OS_ALLOC(length);
    if (*str == NULL)
    {
        PWR_ERROR("Out of memory\n");
        ret = ICEPM_ERR_NOMEM;
    }
    else if (copy_from_user(*str, address, length))
    {
        PWR_ERROR("Copy from user failed\n");
        ret = ICEPM_ERR_INTERNAL;
    }

    return ret;
}


//----------------------------------------------------------------------------
// ioctl handler
//----------------------------------------------------------------------------
static
int fop_ioctl(  struct inode * inode,
                struct file *  filep,
                unsigned int   cmd,
                unsigned long  user_args )
{
    char *          name   = NULL;
    ioctl_arg_t     args;

    if (copy_from_user(&args, (void*)user_args, sizeof(args)))
    {
        PWR_ERROR("Copy from user failed\n");
        args.ret = ICEPM_ERR_FAIL;
        goto exit;
    }

    args.ret = validate_version( (unsigned) args.header_major,
                            (unsigned) args.header_minor);
    if (args.ret != ICEPM_OK)
    {
        goto exit;
    }

    switch (cmd)
    {
    case PWR_IOC_SETMODE:
        args.ret = get_user_space_string(&name,args.name.buffer,args.name.length);
        if (args.ret == ICEPM_OK)
        {
            args.ret = icepm_set_mode(name);
        }
        break;

    case PWR_IOC_REGISTER:
        os_lock(cmd_mutex);
        args.ret = get_user_space_string(&name, args.name.buffer, args.name.length);
        if (args.ret == ICEPM_OK)
        {
            args.ret = us_register(name, filep);
        }
        os_unlock(cmd_mutex);
        break;

    case PWR_IOC_UNREGISTER:
        os_lock(cmd_mutex);
        us_unregister(filep);
        os_unlock(cmd_mutex);
        break;

    case PWR_IOC_WAIT:
        // This ioctl is called by the notification thread that our user space
        // library spins on behalf of the client application. It blocks waiting
        // for a power management event from the kernel.

        // Don't take lock -- we are going to block

        args.ret = us_wait_for_event(filep, &args.event);
        break;

    case PWR_IOC_DONE:
        // This ioctl is called by the notification thread that our user space
        // library spins on behalf of the client application.

        // Don't need lock -- only one process will be notified (and send
        // this ioctl to acknowledge) and a time.  And it deadlocks if the 
        // notification was sent due to a call to icepm_set_mode(), which
        // already has the lock.
        args.ret = us_event_acknowledged(filep, args.callback_rc);
        break;

    case PWR_IOC_WAKE_MASK:
        args.ret = icepm_mask_wake_event(args.wake_event);
        break;

    case PWR_IOC_WAKE_UNMASK:
        args.ret = icepm_unmask_wake_event(args.wake_event);
        break;

    case PWR_IOC_WAKE_GET:
        args.ret = icepm_get_wake_reason( &args.wake_event );
        break;

    default:
        PWR_ERROR("Unknown ioctl: %d\n", cmd);
        args.ret = ICEPM_ERR_INTERNAL;
        break;
    }

exit:
    if (copy_to_user((void*)user_args, &args, sizeof(args)))
    {
        PWR_ERROR("Copy to user failed\n");
        args.ret = ICEPM_ERR_FAIL;
    }

    if (name)
    {
        OS_FREE(name);
    }

    return (args.ret == ICEPM_OK) ? 0 : -EINVAL;
}


//------------------------------------------------------------------------------
// Unlocked ioctl handler => ioctl that does not use big kernel lock
//------------------------------------------------------------------------------
long fop_unlocked_ioctl(struct file  * filep,
                        unsigned int   cmd,
                        unsigned long  arg)
{
    return fop_ioctl(filep->f_dentry->d_inode, filep, cmd, arg);
}


//------------------------------------------------------------------------------
// File close operation
//------------------------------------------------------------------------------
int fop_release(struct inode *inode, struct file *file)
{
    us_unregister(file);
    return 0;
}


//------------------------------------------------------------------------------
// File operation table.
//------------------------------------------------------------------------------
static struct file_operations fops =
{
    owner:          THIS_MODULE,
    release:        fop_release,
#ifdef HAVE_UNLOCKED_IOCTL
    unlocked_ioctl: fop_unlocked_ioctl,
#else
    ioctl:          fop_ioctl,
#endif
};

//============================================================================
// L I N U X   P C I   P O W E R   M A N A G E M E N T   C A L L B A C K S
//============================================================================

static char * dstate2string(pci_power_t state)
{
    char *  retval;

    switch (state)
    {
    case PCI_D0:    retval = "D0";                      break;
    case PCI_D1:    retval = "D1";                      break;
    case PCI_D2:    retval = "D2";                      break;
    case PCI_D3hot: retval = "D3";                      break;
    default:        retval = "Unrecognized D-state";    break;
    }

    return retval;
}

//-----------------------------------------------------------------------------
// pcicallback_set_state
//
// Driver calls to kernel's pci_set_power_state() eventually call back to here.
//-----------------------------------------------------------------------------
static int pcicallback_set_state(struct pci_dev * dev, pci_power_t state)
{
    int         ret  = 0;
    driver_t *  drv  = get_drv(dev->vendor, dev->device);


    if ( drv == NULL )
    {
        // Not in our tables, not one we care about. Ignore it.
        PWR_DEBUG( "%s %s (%04x:%04x) -- ignored\n",
                   dstate2string(state),
                   dev->driver && dev->driver->name ? dev->driver->name : "???",
                   dev->vendor,
                   dev->device
                   );
        ret = -1;
    }
    else
    {
        PWR_DEBUG( "%s %s (%04x:%04x)",
                   dstate2string(state), drv->name, dev->vendor, dev->device);

        switch (state)
        {
        case PCI_D0:
            if ( ! drv->is_suspended )
            {
                // Nothing to do
                PWR_DEBUG_CONTINUE(" -- not suspended\n");
            }
            else
            {
                PWR_DEBUG_CONTINUE("\n");
                soc->power_up(drv); // SoC-specific actions
            }
            break;

        case PCI_D1:
        case PCI_D2:
        case PCI_D3hot:
            if ( drv->is_suspended )
            {
                PWR_DEBUG_CONTINUE("-- already suspended\n");
            }
            else
            {
                PWR_DEBUG_CONTINUE("\n");
                soc->power_down(drv); // SoC-specific actions
            }
            break;

        default:
            PWR_ERROR("-- Unrecognized state (%u)\n", state);
            ret = -1;
            break;
        }
    }

    return ret;
}


//-----------------------------------------------------------------------------
// pcicallback_choose_state
//
// Driver calls to pci_choose_state eventually call back to this function.
//-----------------------------------------------------------------------------
static pci_power_t pcicallback_choose_state(struct pci_dev * dev)
{
    // Let the kernel choose the correct device power state based on the
    // target sleep state.
    return PCI_POWER_ERROR;
}

static bool pcicallback_is_manageable(struct pci_dev * dev)
{
    return true;
}

static bool pcicallback_can_wakeup(struct pci_dev * dev)
{
    return true;
}

static int pcicallback_sleep_wake(struct pci_dev * dev, bool enable)
{
    return 0;
}

static struct intel_pm_pci_ops_t __icepm_pci_pm_ops =
{
    .is_manageable = pcicallback_is_manageable,
    .set_state     = pcicallback_set_state,
    .choose_state  = pcicallback_choose_state,
    .can_wakeup    = pcicallback_can_wakeup,
    .sleep_wake    = pcicallback_sleep_wake,
};


//-----------------------------------------------------------------------------
// notification_handler
//
// Function that we register for kernel power event notifications.
//
// Kernel sequence for STR (and resume) is:
//      - send PM_SUSPEND_PREPARE to entities registered for notifications
//      - suspend all kernel drivers
//      - turn control over to CEFDK, which tells PUnit to shut down
//      <wake event occurs>
//      - PUnit turns control over to CEFDK, which returns to kernel
//      - resume all kernel drivers
//      - send PM_POST_SUSPEND to entities registered for notifications
//-----------------------------------------------------------------------------
static
int notification_handler(
            struct notifier_block * nb,     // Block that was used to register
            unsigned long           type,   // Notification type
            void *                  ignore) // Always NULL -- ignore
{
    icepm_ret_t ret = ICEPM_OK;

    switch (type)
    {
    case PM_SUSPEND_PREPARE:
        PWR_DEBUG("PM_SUSPEND_PREPARE\n");
        if ( strcasecmp(current_mode, "ON") )
        {
            // We are in a standby mode.  PUnit FW is not validated to go from
            // STANDBY to STR -- put it into full ON.
            //
            // NOTE: Even if we did not do this, on return from STR the system
            // will be in full ON:  the CEFDK puts it in that state before
            // returning to kernel so all driver resume() functions can run.
            PWR_DEBUG("Go to Full-On\n");
            ret = _set_mode("ON");
            if (ret != ICEPM_OK)
            {
                PWR_ERROR("Attempt to come out of Standby failed!\n");
                goto exit;
            }
        }

        soc->pre_sleep();

        if ( us_notify_all_clients(ICEPM_EVT_SUSPEND))
        {
            ret = ICEPM_ERR_DEV_BUSY;
            us_notify_all_clients(ICEPM_EVT_RESUME);
        }
        else
        {
            in_STR = true;
        }
        break;

    case PM_POST_SUSPEND:
        // System just resumed or error occurred during suspend.
        // Kernel driver .resume() callbacks have been executed for drivers
        // that were suspended, user space tasks are thawed.

        PWR_DEBUG("PM_POST_SUSPEND\n");

        in_STR = false;

        // Linux will return *all* devices to D0 on resume, even if they have no
        // driver installed.  Turn off any CE devices for which there is no
        // driver.
        check_drivers();

        soc->post_sleep();

        // On CE4300, we used to transition back to STANDBY3 here it we were in
        // that mode before we suspended.  But that no longer works because
        // - in STANDBY3 the power island containing the eMMC device is off
        // - the Linux eMMC driver also has a post-suspend notification handler,
        //   with the lowest possible priority (so we cannot guarantee we can
        //   run after it).  The driver touches registers; with the island
        //   powered off this hangs the system.
        //
        // It is now the application/middleware's responsibility to return the
        // system to STANDBY3 if that is what is desired.

        if (us_notify_all_clients(ICEPM_EVT_RESUME))
        {
            ret = ICEPM_ERR_INCOMPLETE;
        }
        break;

    default:
        // Hibernation (suspend-to-disk) not supported.
        ret = ICEPM_ERR_INTERNAL;
    }
exit:
    return (ret == ICEPM_OK) ? 0 : 1;
}


static struct notifier_block nb =
{
    notification_handler,   // Notification handler
    0,                      // Ignored when registering
    1000 /* TODO */         // Notifier chain is sorted in descending
                            //   order of this value
};
static bool             registered_notifier = false;


//------------------------------------------------------------------------------
// Check if the driver for the specified PCI device is loaded.
//------------------------------------------------------------------------------
static bool driver_is_loaded( driver_t *drv )
{
    bool            ret = false;
    struct pci_dev *pcidev;

    // Get kernel structure for the device
    pcidev = pci_get_device(drv->pci_id->vendor, drv->pci_id->device, NULL);
    if (pcidev != NULL)
    {
        // See if a driver is registered for it
        if (pcidev->driver != NULL)
        {
            ret = true;
        }
        pci_dev_put(pcidev);
    }
    return ret;
}

//------------------------------------------------------------------------------
// Put every CE device for which a driver is not loaded into a low-power state.
//------------------------------------------------------------------------------
void check_drivers(void)
{
    int i;

    for (i=0; soc->drivers[i] != NULL; i++)
    {
        driver_t *drv = soc->drivers[i];

        if ( ! driver_is_loaded(drv) )
        {
            PWR_DEBUG("power off %s\n", drv->name);
            soc->power_down(drv);
        }
    }
}

//============================================================================
//   M O D U L E   I N I T   A N D   S H U T D O W N   C O D E
//============================================================================

//----------------------------------------------------------------------------
// cleanup
//
// Module cleanup
//----------------------------------------------------------------------------
static void cleanup(void)
{
    if (iosfh != NULL)
    {
        iosf_close(iosfh);
        iosfh = NULL;
    }

    procfs_deinit();

    if (registered_notifier)
    {
        unregister_pm_notifier(&nb);
        registered_notifier = false;
    }

    us_cleanup();

    if (major_number >= 0)
    {
        unregister_chrdev(major_number,MOD_NAME);
        major_number = -1;
    }
    
    if (cmd_mutex)
    {
        os_destroy_lock(cmd_mutex);
        cmd_mutex = NULL;
    }

    intel_pm_register_callback(0);

    punit_close();
}


//----------------------------------------------------------------------------
// init
//
// Module initialization
//----------------------------------------------------------------------------
static int __init init(void)
{
    int             ret = -EINVAL;
    iosf_result_t   iosf_ret = IOSF_OK;

    if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39))
    {
        PWR_ERROR("*** Power Management not supported on this kernel\n");
        goto exit;
    }

    if (PAL_SUCCESS != pal_get_soc_info(&soc_info))
    {
        PWR_ERROR("Unable to retrieve SOC info from PAL\n");
        goto exit;
    }

    if ( init_ids(&soc_info) )
    {
        goto exit;
    }

    switch ( soc_info.name )
    {
    case SOC_NAME_CE2600:
        soc = ce2600_init( &soc_info );
        iosf_ret = iosf_open(0, &iosfh);
        break;
    case SOC_NAME_CE4100:
        soc = ce4100_init( &soc_info );
        break;
    case SOC_NAME_CE4200:
        soc = ce4200_init( &soc_info );
        break;
    case SOC_NAME_CE5300:
        soc = ce5300_init( &soc_info );
        iosf_ret = iosf_open(0, &iosfh);
        break;
    default:
        PWR_ERROR("Unsupported SoC (%d)\n", soc_info.name );
        goto exit;
    }

    if (soc == NULL)
    {
        // *_init() failed -- error message already emitted
        goto exit;
    }

    if (iosf_ret != IOSF_OK )
    {
        PWR_ERROR("iosf_open()=%d\n", iosf_ret);
        goto exit;
    }

    if (soc->init_punit(&soc_info) != ICEPM_OK)
    {
        goto exit;
    }

    in_STR =  false;

    major_number = register_chrdev(0, MOD_NAME, &fops);
    if (major_number < 0)
    {
        PWR_ERROR("Unable to register device %s\n", MOD_NAME);
        ret = -ENODEV;
        goto exit;
    }

    if (NULL == (cmd_mutex = os_create_lock()))
    {
        goto exit;
    }

    intel_pm_register_callback(&__icepm_pci_pm_ops);

    us_init();

    if ( register_pm_notifier(&nb) ) // Kernel function, returns 0 on success
    {
        PWR_ERROR("register_pm_notifier failed\n");
        goto exit;
    }
    registered_notifier = true;

    if ( procfs_init(soc) )
    {
        goto exit;
    }

    ret = 0;
exit:
    if (ret != 0)
    {
        cleanup();
    }
    return ret;
}

//----------------------------------------------------------------------------
// Register init and exit functions
//----------------------------------------------------------------------------
module_init(init);
module_exit(cleanup);
