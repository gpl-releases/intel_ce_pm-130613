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
// Header file to be included by all clients (drivers and applications) of
// power manager.
//------------------------------------------------------------------------------

#ifndef _INTEL_CE_PM_H_
#define _INTEL_CE_PM_H_

#ifndef DOXYGEN_SKIP /* DOXYGEN_SKIP only defined when docs are generated */
#define ICEPM_HEADER_VERSION_MAJOR    3
#define ICEPM_HEADER_VERSION_MINOR    0
#endif

// For definition of the enumeration IO_8051_wake_event_t
// At this writing (11/1/2011), the members of this enumeration are:
//
//      IO_8051_WAKE_EVT_NONE
//
//      IO_8051_WAKE_EVT_BLUETOOTH
//      IO_8051_WAKE_EVT_CEC
//      IO_8051_WAKE_EVT_FP_INT
//      IO_8051_WAKE_EVT_IR
//      IO_8051_WAKE_EVT_NUM_EVENTS	
//      IO_8051_WAKE_EVT_POWER_BUTTON
//      IO_8051_WAKE_EVT_RF4CE
//      IO_8051_WAKE_EVT_RF_CUSTOM
//      IO_8051_WAKE_EVT_RTC
//      IO_8051_WAKE_EVT_WATCHDOG
//      IO_8051_WAKE_EVT_WoLAN
//      IO_8051_WAKE_EVT_WoWLAN
#include "io8051EventMap.h"


/**@defgroup pwr        Power Management
 *
 * The Intel CE Power Management model supports four system states:
 *
 * - Full off. This is an initial state only.  The system performs a cold boot
 *   into Active as soon as external power is applied to the system.
 *
 * - Active. In this state the CPU has power and code is executing. All SoCs
 *   default to Full-On mode (all devices powered on) in Active state. Depending
 *   on the SoC, additional Standby modes may be supported.  In a Standby mode,
 *   a group of unused devices is powered off. The controlling system software
 *   uses the API #icepm_set_mode to request a transition from one supported
 *   Active power mode to another. The supported Active power modes for an SoC
 *   are defined in the platform/power/operational_modes section of its
 *   platform configuration file (found in /etc/plaform_config/).
 *
 * - Soft-off. In this state the system is powered down and all non-persistent
 *   state is lost. This state is entered by executing the Linux 'poweroff'
 *   command. A wake event in this state will cause the system to warm boot
 *   back to Active state.
 *
 * - Suspend-to-RAM (STR). In this state the system is powered down, but RAM is
 *   in self-refresh mode so all state is retained. This state is entered by
 *   executing the standard Linux STR sequence, i.e., by writing the string
 *   "mem" to /sys/power/state.  A wake event in this state will return the
 *   system to Active state, and execution of applications will resume where
 *   they left off.
 */

/** @ingroup pwr
 *
 * Events that can be returned to user space processes that register a callback
 */
typedef enum
{
    ICEPM_EVT_SUSPEND,      ///< We are about to suspend to RAM
    ICEPM_EVT_RESUME,       ///< We have resumed from suspend to RAM
    ICEPM_EVT_INTERRUPTED,  ///< Notification wait interrupted by signal
    ICEPM_EVT_ERROR,        ///< Internal error (should not occur)
    ICEPM_EVT_SUSPEND2,     ///< 2nd suspend event (sent to icepm daemon only)
    ICEPM_EVT_RESUME2       ///< 2nd resume event (sent to icepm daemon only)
} icepm_event_t;


/** @ingroup pwr
 *
 * Values that can be returned by functions in the power management API.
 */
typedef enum
{
    ICEPM_OK,
        ///< Function executed without error.
    ICEPM_ERR_INTERNAL,
        ///< "Should not occur" error.
    ICEPM_ERR_INVALID_EVENT,
        ///< Undefined value of type IO_8051_wake_event_t passed to function.
    ICEPM_ERR_UNKNOWN_MODE,
        ///< Specified power mode not currently defined.
    ICEPM_ERR_INCOMPATIBLE,
        ///< Incompatible versions of library and kernel module are in use.
    ICEPM_ERR_NOMEM,
        ///< Memory allocation failed.
    ICEPM_ERR_MISSING_DEVICE,
        ///< User space library tried unsuccessfully to open /dev/intel_ce_pm.
    ICEPM_ERR_INCOMPLETE,
        ///< One or more device drivers could not be resumed.
    ICEPM_ERR_NULL_PTR,
        ///< Null pointer passed for required argument.
    ICEPM_ERR_FAIL,
        ///< Call to external function failed.
    __NOT_USED__,
        ///< Available for new error code
    ICEPM_ERR_DEV_BUSY
        ///< Driver could not be suspended.
} icepm_ret_t;


/** @ingroup pwr
 *
 * Signature of callback function registered by driver.
 *
 * The callback should take the necessary steps to handle the passed event, and
 * should return 0 on success, 1 on failure.
 *
 * The callback function is invoked asynchronously and should take the
 * necessary precautions to prevent contention for global data structures.
 *
 * Since suspend/resume calls to other drivers are delayed until the callback
 * function returns, it should complete its work and return as soon as possible.
 *
 * @param [in] event
 * Event of which the callback is being notified.
 *
 * In the case of ICEPM_EVT_SUSPEND, the driver should
 *  - Immediately return a value of 0 (success) if already in suspend state.
 *  - If the driver has "active" device instances (e.g., if a decoder is in
 *    paused or playing state), fail by returning 1.
 *  - Enter suspend state:
 *      - Reject (with an error return) any subsequent client requests.
 *      - Suspend threads, disable IRQs, disable hardware, etc., as
 *        necessary to completely prevent access to hardware.
 *      - Make sure all state needed to bring the device out of reset
 *        and back to the current state is stored in RAM.
 *  - Wait for outstanding transactions to/from the device to complete.
 *  - Return 0 to caller.
 *
 * In the case of ICEPM_EVT_RESUME, the driver should
 *  - Immediately return a value of 0 (success) if already in active state.
 *  - Do whatever is necessary to re-discover changes in environment (e.g.,
 *    presence or absence of I/O devices).
 *  - Use stored state information to bring the device from reset to the state
 *    it was in when the suspend() function was called.
 *  - Resume threads, re-enable interrupt handlers, etc.
 *  - Allow normal processing of subsequent client requests.
 *  - Return 0 to caller.
 *
 * @param [in] cookie
 * The cookie passed by the driver when it registered this callback function.
 */
typedef int (* icepm_callback_t)(icepm_event_t event, void *cookie);

/** @ingroup pwr
 *
 * THIS FUNCTION IS INTENDED TO BE CALLED BY USER-SPACE DRIVERS ONLY
 *
 * Register a callback function to be invoked on power state changes.
 *
 * All Intel CE drivers affected by power management must:
 *  - call this function to register a callback during driver load-time
 *    initialization.
 *  - call this function with a NULL function pointer prior to shutting down.
 *
 * @param [in] func
 * A pointer to the callback function to be invoked when a power management
 * event occurs. If NULL, the driver is unregistered.
 *
 * @param [in] drv_name
 * A 0-terminated ASCII string identifying the driver.
 *
 * @param [in] cookie
 * A pointer to any data structure that the driver would like to be passed
 * to the callback function when it is invoked.  May be NULL.
 */
icepm_ret_t icepm_register_callback( icepm_callback_t    func,
                                     char *              drv_name,
                                     void *              cookie);

/** @ingroup pwr
 *
 * Change the current active power mode.
 *
 * When the system is in Active state, all SoCs default to Full-On mode.  Some
 * SoCs also support one or more Standby states, in which the CPU is active but
 * unused subsystems may be powered off.  This function can be used by
 * controlling system software to move between Full-On and Standby states
 * supported by the target SoC.
 *
 * @param [in] mode_name
 * The name of the target power mode.  A 0-terminated ASCII string matching
 * one of the operational modes defined in the platform configuration file.
 */
icepm_ret_t icepm_set_mode(char *mode_name);

/** @ingroup pwr
 *
 * Adds the specified event to the list of currently masked wake events.
 *
 * A wake event is an event that will bring the SOC out of a Suspend-to-RAM
 * (STR) or Soft Off state back to Active state. A masked wake event will not
 * wake the system.
 *
 * @param [out] event
 * The wake event to be masked. If the event is IO_8051_WAKE_EVT_NONE, all
 * events will be <b>unmasked</b>.
 *
 * If the specified event is not implemented on the target board this call is a
 * no-op.
 */
icepm_ret_t icepm_mask_wake_event(IO_8051_wake_event_t event);

/** @ingroup pwr
 *
 * Removes the specified event from the list of currently masked wake events.
 *
 * A wake event is an event that will bring the SOC out of a Suspend-to-RAM
 * (STR) or Soft Off state back to Active state. A masked wake event will not
 * wake the system.
 *
 * @param [out] event
 * The wake event to be unmasked. If the event is IO_8051_WAKE_EVT_NONE, all
 * events will be <b>masked</b>. This is useful for initializing the mask list
 * prior to unmasking individual events, but the the system should not normally
 * enter Soft Off or Suspend-to-RAM state with this mask set.
 *
 * If the specified event is not implemented on the target board this call is a
 * no-op.
 */
icepm_ret_t icepm_unmask_wake_event(IO_8051_wake_event_t event);

/** @ingroup pwr
 *
 * Returns the wake event that caused the last return to Active state from
 * Suspend-to-RAM or Soft Off state.
 *
 * If multiple events occurred in close prioximity, the returned value will be
 * the highest priority event.
 *
 * If IO_8051_WAKE_EVT_NONE is returned after boot, it indicates a cold boot
 * (first application of power to the system) as opposed to a wake from Soft
 * Off state.
 *
 * If IO_8051_WAKE_EVT_NONE is returned after an attempt to suspend to RAM, it
 * indicates the suspend to RAM was aborted because a driver could not be
 * suspended.
 *
 * @param [out] event
 * The wake event that caused the last return to Active state.
 */
icepm_ret_t icepm_get_wake_reason(IO_8051_wake_event_t *event);

// For internal use
#define DAEMON_NAME "ICEPM_DAEMON_432957"

#endif // _INTEL_CE_PM_H_
