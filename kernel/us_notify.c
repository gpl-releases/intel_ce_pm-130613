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

#include <linux/string.h>
#include <linux/suspend.h>          // For suspend/hibernate notifiers

#include "osal.h"

#include "icepm_internal.h"
#include "kernel.h"

//******************************************************************************
//     SUPPORT FOR NOTIFICATION OF USER-SPACE PROCESSES ABOUT POWER EVENTS
//******************************************************************************


//------------------------------------------------------------------------------
// Descriptor of a user space process that has registered for power event
// notification.  A pointer to the descriptor is stored in the 'private_data'
// field of the file pointer through which the process makes ioctls to us.
//------------------------------------------------------------------------------
typedef struct us_client
{
    struct us_client *  next;   // Next client in list
    struct us_client *  prev;   // Previous client in list
    char *              name;   // Name of client
    icepm_event_t       prev_event;
                                // Last event client received without error
    icepm_event_t       event;  // Event to fire to client
    pid_t               pid;    // Client process ID
    os_event_t          waitq;  // Client will be blocked on this while waiting
                                //   for event notifications.
    uint32_t            rc;     // Completion code returned by client in
                                //    response to event notification.
                                //    0 => success, non-zero => failure.
} us_client_t;

//------------------------------------------------------------------------------
//              R E G I S T E R E D   C L I E N T   L I S T
//------------------------------------------------------------------------------

// Head of doubly-linked circular list of registered user space event clients.
//
// The head element contains no real data -- its 'next' field points to the
// first real entry on the list, its 'prev' entry to the last. (Both pointers
// point back to the head, if the list is empty)
//
// The list is processed from beginning to end when firing pre-suspend events,
// and in the opposite order when firing post-resume events. The list is sorted
// by order of registration, so the last-registered process is suspended first
// (and resumed last).

static us_client_t  client_list;

// The power daemon, if any, is always notified twice per event:  the first time
// before any other user processes, the second time after all of them.
// The daemon can be identified by the magic name with which it registers.

static us_client_t *daemon;

// Serialize access to the list across different threads.

static os_lock_t    client_list_mutex   = NULL;

// Wait on this event for a user space client to complete processing its
// notification.  The client will make an ioctl call with the processing
// return code, and the ioctl handler will fire this event, allowing us to move
// to the next client (or, if none, return to the kernel).
//
// We only need one event object as we only notify one client at a time.

static os_event_t   wait_for_user_space;

#if 0
static
void dump_list(char *label)
{
    us_client_t *us;

    printk("***** %s: ", label);
    if ( daemon )
    {
        printk("%s ", daemon->name);
    }

    for (us=client_list.next; us != &client_list; us = us->next)
    {
        printk("%s ",us->name);
    }
    printk("\n");
}
#endif

//------------------------------------------------------------------------------
// Remove a client from the list, free its resources.
//------------------------------------------------------------------------------
static
void remove_client( us_client_t *us )
{
    if ( us == daemon)
    {
        daemon = NULL;
    }
    else
    {
        os_lock(client_list_mutex);         // >>>>>>>>>>>>>>>>>>>>>> LOCK
        us->prev->next = us->next;
        us->next->prev = us->prev;
        os_unlock(client_list_mutex);       // <<<<<<<<<<<<<<<<<<<<<< UNLOCK
    }

    os_event_destroy(&us->waitq);
    OS_FREE(us);
}


//------------------------------------------------------------------------------
// Allocate a new client structure and add it to the list of registered clients.
// Return pointer to the new structure.
//------------------------------------------------------------------------------
static
us_client_t *add_client( char *name )
{
    us_client_t *us = NULL;
    
    if ( !strcmp(name,DAEMON_NAME) && (daemon != NULL))
    {
        PWR_ERROR("Daemon already registered");
        goto exit;
    }

    us = OS_ALLOC(sizeof(us_client_t) + strlen(name) + 1);
    if (us == NULL)
    {
        PWR_ERROR("No memory for user space driver\n");
        goto exit;
    }

    us->name = (char *)(us + 1);
    strcpy(us->name, name);

    us->prev_event  = ICEPM_EVT_RESUME;
    us->pid         = 0;
    us->rc          = 0;
    os_event_create(&us->waitq, 1);

    if ( !strcmp(name,DAEMON_NAME) )
    {
        daemon = us;
    }
    else    // Add to head of client list
    {
        os_lock(client_list_mutex);     // >>>>>>>>>>>>>>>>>> LOCK

        us->next = client_list.next;
        us->prev = &client_list;

        us->prev->next = us;
        us->next->prev = us;

        os_unlock(client_list_mutex);   // <<<<<<<<<<<<<<<<<< UNLOCK
    }

exit:
    return us;
}


//------------------------------------------------------------------------------
// Register a client: add client to the list, and save a pointer to the new
// list entry in the file pointer used by the client to send us ioctls.
//------------------------------------------------------------------------------
icepm_ret_t us_register(char *name, struct file *filep)
{
    icepm_ret_t  ret = ICEPM_OK;
    us_client_t *us  = add_client(name);

    if (us == NULL)
    {
        ret = ICEPM_ERR_NOMEM;
    }
    else
    {
        filep->private_data = (void*)us; // Save in file context

        // Setting pid non-zero activates checking of this driver in
        // other threads -- only set it when registration complete
        us->pid = current->tgid;
    }

    return ret;
}


//------------------------------------------------------------------------------
// Unregister a client: remove the client structure pointer from the descriptor
// of the file used by the client process to send us ioctls; then remove the
// client structure from the list of clients.
//------------------------------------------------------------------------------
void us_unregister(struct file *filep)
{
    us_client_t *us = (us_client_t*)(filep->private_data);

    if (us)
    {
        us->pid = 0;   // Disable checking of this driver in other threads
        filep->private_data = NULL;
        remove_client(us);
    }
}


//------------------------------------------------------------------------------
//              C L I E N T   N O T I F I C A T I O N
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// The user space library spins an event thread on behalf of the client.  The
// event thread makes a blocking ioctl to wait for an event; that ioctl ends up
// here.
//
// The thread gets unblocked in notify_client() (which stuffs the notification
// event into the client structure's 'event' field and sets its wait object) or
// by an interrupted/error wait.
//
// The p_event parameter is used to return the event of which the client should
// be notified.
//------------------------------------------------------------------------------
icepm_ret_t us_wait_for_event(struct file *filep, icepm_event_t *p_event)
{
    osal_result     wait_ret;
    icepm_ret_t     ret     = ICEPM_OK;
    us_client_t*    us      = (us_client_t*)(filep->private_data);

    if (us == NULL)
    {
        PWR_ERROR("Driver not registered for notifications\n");
        ret = ICEPM_ERR_INTERNAL;
    }
    else
    {
        while (OSAL_TIMEOUT==(wait_ret=(os_event_wait(&us->waitq, 100000000))))
        {
            ;
        }

        switch (wait_ret)
        {
        case OSAL_SUCCESS:
            os_event_reset(&(us->waitq));
            *p_event = us->event;
            break;
        case OSAL_NOT_DONE:
            *p_event = ICEPM_EVT_INTERRUPTED;
            break;
        default:
            *p_event = ICEPM_EVT_ERROR;
            break;
        }
    }

    return ret;
}


//-----------------------------------------------------------------------------
// Notify a registered user space client of a power management event by 
// unblocking its event thread.  Then block ourselves until the client returns
// us a result code.
//
// Return 0 on successful event handling.
//-----------------------------------------------------------------------------
static
int notify_client(us_client_t *us, icepm_event_t event)
{
    int ret = 0;

    if ( us == NULL )
    {
        // Nothing to do. Passing NULL is allowed so we don't have to check if
        // a daemon is registered every place where we notify the daemon.
        ;
    }
    else if ( us->prev_event == event)
    {
        // No state change, nothing to do.  This might happen, for example,
        // when backing out "suspend" events when some process aborts the 
        // suspend sequence before all clients have been notified.
        PWR_DEBUG("%s: NO-OP\n", us->name);
    }
    else
    {
        int secs;

        PWR_DEBUG("%s\n", us->name);

        // Put event where the client thread can find it, then wake the thread.
        us->event = event;
        os_event_set(&(us->waitq));
    
        // Wait for client thread to return a result code (via ioctl).
        // Every 5 ms, check if user space process is still alive, return to
        // wait if it is.
        for (secs=5;
             OSAL_SUCCESS != os_event_wait(&wait_for_user_space, 5000);
             secs += 5)
        {
            if (us->pid == 0)
            {
                PWR_DEBUG("Process no longer exists\n");
                ret = 0;
                goto exit;
            }
            PWR_WARN("Waiting %d seconds for response from user space (%s)\n",
                     secs, us->name);
        }
    
        os_event_reset(&wait_for_user_space);
    
        ret = us->rc;
        if (ret)
        {
            PWR_ERROR("Client '%s' returned error for event %u\n",
                      us->name, event);
        }
        else
        {
            us->prev_event = event;
        }
    }
exit:
    return ret;
}


//------------------------------------------------------------------------------
// After the client event thread is unblocked with a new event notification, we
// wait for the client to return a result code via a PWR_IOC_DONE iotcl. That
// ioctl comes here, where we unblock our notification thread so it can notify
// the next client (or return to the kernel).
//
// filep    File through which client makes ioctl to us.
// rc       Result code returned by client.
//------------------------------------------------------------------------------
icepm_ret_t us_event_acknowledged(struct file *filep, int rc)
{
    icepm_ret_t ret = ICEPM_OK;
    us_client_t* us = (us_client_t*)(filep->private_data);

    if (us == NULL)
    {
        PWR_ERROR("Driver not registered for notifications\n");
        ret = ICEPM_ERR_INTERNAL;
    }
    else
    {
        // Place return code where our notification thread can find it, then
        // unblock the thread.
        us->rc = rc;
        os_event_set( &wait_for_user_space );
    }

    // On return from the ioctl, client event thread will make another
    // blocking call to wait for the next event.
    return ret;
}

//-----------------------------------------------------------------------------
// us_notify_all_clients
//
// Send specified event to all registered user space clients.
//-----------------------------------------------------------------------------
int us_notify_all_clients(icepm_event_t event)
{
    int          ret = 0;
    us_client_t *us;

    os_lock(client_list_mutex);

    // Notify daemon before any other user space processes
    notify_client(daemon, event);

    if ( event == ICEPM_EVT_SUSPEND)
    {
        // Process client list from beginning to end
        for (us = client_list.next; us != &client_list; us = us->next)
        {
            if ( notify_client(us, event) )
            {
                ret = 1;
                break;  // ABORT suspend process
            }
        }
    }
    else // RESUME
    {
        // Process client list from end to beginning
        for (us = client_list.prev; us != &client_list; us = us->prev)
        {
            if ( notify_client(us, event) )
            {
                ret = 1;
                // But process the rest of the list
            }
        }
    }

    // Notify daemon again after all other user space processes
    notify_client(daemon, (event == ICEPM_EVT_SUSPEND)
                            ? ICEPM_EVT_SUSPEND2
                            : ICEPM_EVT_RESUME2 );

    os_unlock(client_list_mutex);

    return ret;
}

//============================================================================
//          I N I T   A N D   S H U T D O W N   C O D E
//============================================================================

void us_cleanup(void)
{
    os_event_destroy( &wait_for_user_space );

    if (daemon)
    {
        remove_client(daemon);
        daemon = NULL;
    }

    if (client_list_mutex)
    {
        while (client_list.next != &client_list)
        {
            remove_client(client_list.next);
        }
        os_destroy_lock(client_list_mutex);
        client_list_mutex = NULL;
    }
}


int __init us_init(void)
{
    int ret = -EINVAL;

    daemon  = NULL;

    client_list.next  = &client_list;
    client_list.prev  = &client_list;

    client_list_mutex = os_create_lock();
    if ( NULL != client_list_mutex )
    {
        os_event_create( &wait_for_user_space, 1 );
        ret = 0;
    }

    return ret;
}
