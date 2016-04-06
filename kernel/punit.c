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

//==============================================================================
//                  P U N I T   A C C E S S 
//              No PUnit on SoCs prior to CE4200                 
//==============================================================================

#include <linux/bitops.h>       // For definition of BIT(x)

#include "icepm_internal.h"

#include "kernel.h"


IO_8051_wake_event_t wake_event = IO_8051_WAKE_EVT_NONE;

// P-Unit PCI device ID (vendor ID is VENDOR_ID_INTEL)
#define PUNIT_DEVICE_ID     0x070C

// Value of the miniBAR block, as read from PCI config space.  This is a base
// I/O port used to access the message bus. To access PUnit registers via the
// message bus:
//
// - write the message bus address of the register (with an "outl" instruction)
//   to the I/O port whose address is in 'miniBAR'.
//
// - then read data from or write data to the register with inl/outl on the
//   port at miniBAR+4.
static uint32_t miniBAR;

// Map of wake events to bits in wake event status/mask registers for the board 
// on which we are running. Entry N is the wake event enumerator associated with
// bit N in the registers
static uint8_t wake_event_map[NUM_EVENT_MAP_ENTRIES];

static uint32_t wake_event_mask = 0;

// Message bus addresses used to access PUnit registers via the miniBAR/IOSF
#define REG_PM_CMD              0x5F
#define REG_PM_ICS              0x62
#define REG_PM_SSC              0x60
#define REG_PM_SSS              0x61
#define REG_PM_STS              0x5E
#define REG_WAKE_EVENT_STATUS   0x7D
#define REG_WAKE_EVENT_MASK     0xD0
#define REG_WAKE_EVENT_MAP_0    0xD1
#define REG_WAKE_EVENT_MAP_1    0xD2
#define REG_WAKE_EVENT_MAP_2    0xD3
#define REG_WAKE_EVENT_MAP_3    0xD4
#define REG_WAKE_EVENT_MAP_4    0xD5
#define REG_WAKE_EVENT_MAP_5    0xD6
#define REG_WAKE_EVENT_MAP_6    0xD7
#define REG_WAKE_EVENT_MAP_7    0xD8

// CE4200 still uses legacy WAKESM register, instead of REG_WAKE_EVENT_*
// TODO: remove after CE4200 moves to new wake registers
static bool is_ce4200 = false;
#define REG_WAKESM              0x7D

// ----------------------------------------------------
// The legacy WAKESM register
// ----------------------------------------------------
//  Bits 31:16 - MASK BITS: 1 => event will not cause wake event
//      31:  PLATFORM WAKE EVENT (PM_WAKEUP GPIO)
//      30:     [reserved]
//      29:     [reserved]
//      28:  IR EVENT
//      27:  RF EVENT
//      26:     [reserved]
//      25:  CEC EVENT
//      24:     [reserved]
//      23:     [GND -- ignore]
//      22:     [GND -- ignore]
//    * 21:  magic_pkt_wakeup [prefilter]
//    * 20:  SEC - sec_punit_wake_event
//    * 19:  RTC - xrrtc_irq8_sus
//      18:  RTC - xrrtc_irq8_core
//    * 17:  GBE - gbe_pme_wake
//    * 16:  TSD - tsd_int
//
//    * WE SHOULD NEVER ENABLE THESE EVENTS (or check corresponding status)
//      - The prefilter, SEC, and GBE events are useless.
//      - The TSD event useless AND we were flooded by them as soon as the TSD
//        driver was loaded on CE4200.
//      - The RTC sus event is redundant and will be removed in future.
//
//  Bits 15:0 - STATUS BITS: 1 => event caused last wake event
//      15:  PLATFORM WAKE EVENT (PM_WAKEUP GPIO)
//      14:     [reserved]
//      13:     [reserved]
//      12:  IR EVENT
//      11:  RF EVENT
//      10:     [reserved]
//       9:  CEC EVENT
//       8:     [reserved]
//       7:     [GND -- ignore]
//       6:     [GND -- ignore]
//       5:  magic_pkt_wakeup [prefilter]
//       4:  SEC - sec_punit_wake_event
//       3:  RTC - xrrtc_irq8_sus
//       2:  RTC - xrrtc_irq8_core
//       1:  GBE - gbe_pme_wake
//       0:  TSD - tsd_int

// Treating the GND bits the same as reserved bits, just to be safe.
#define WAKESM_RESERVED_BITS \
  (BIT(30) | BIT(29) | BIT(26) | BIT(24) | BIT(23) | BIT(22) | \
   BIT(14) | BIT(13) | BIT(10) | BIT(8)  | BIT(7)  | BIT(6))


#define WAKESM_ALWAYS_MASKED (BIT(21) | BIT(20) | BIT(19) | BIT(17) | BIT(16))


//------------------------------------------------------------------------------
// Write a 32-bit value to a PUnit register.
//------------------------------------------------------------------------------
static
void reg_write(uint32_t addr, uint32_t value)
{
    if ( is_ce4200 )
    {
        // No IOSF on CE4200 -- use miniBAR.
        outl(addr,  miniBAR);   // Write address of register
        outl(value, miniBAR+4); // Write new contents of register
    }
    else
    {
        iosf_write32(iosfh, IOSF_PORT_PUNIT, addr, value);
    }
}

//------------------------------------------------------------------------------
// Read a 32-bit value from a PUnit register.
//------------------------------------------------------------------------------
static
uint32_t reg_read(uint32_t addr)
{
    uint32_t value;
    
    if ( is_ce4200 )
    {
        // No IOSF on CE4200 -- use miniBAR.
        outl(addr, miniBAR);      // Write address of register
        value =  inl(miniBAR+4);  // Read contents of register
    }
    else
    {
        iosf_read32(iosfh, IOSF_PORT_PUNIT, addr, &value);
    }
    return value;
}


//------------------------------------------------------------------------------
// Change power mode by asking the PUnit to turn islands on/off.
//
//  ssc  Value to be written to the PUnit's PM_SSC register.  The exact format
//       of the value varies from one SoC to another depending on the power
//       islands supported.
//------------------------------------------------------------------------------
void punit_mode(uint32_t ssc)
{
    static uint32_t transaction_number = 0;
    uint32_t        reg;

    PWR_DEBUG("Set power island config to 0x%08x\n", ssc);

    // Clear and disable interrupts
    reg = reg_read(REG_PM_ICS);
    reg &= 0xffffffcf;          // Clear interrupt pending/enabled bits
    reg_write(REG_PM_ICS, reg );

    // Specify desired island configuration
    reg_write(REG_PM_SSC, ssc);

    // Send command to PUnit
    reg = reg_read(REG_PM_CMD);
    reg &= 0xfe000000;                  // Preserve reserved bits
    reg |= transaction_number << 21;    // Set cookie
    reg |= BIT(9) | BIT(0);             // "Set config" command, IMMEDIATE mode
    reg_write(REG_PM_CMD, reg);

    //Wait for command completion
    do
    {
        OS_SLEEP(1);                                     //Sleep for 1ms   
        reg = reg_read(REG_PM_STS);
    } while ((reg & BIT(8))                              // Punit still busy
    ||       (((reg >> 9) & 0xf) != transaction_number));// Didn't start yet?

    // Roll transaction number
    if (++transaction_number >= 16)
    {
        transaction_number = 0;
    }
}


//-----------------------------------------------------------------------------
// _set_event_mask
//-----------------------------------------------------------------------------
static void _set_event_mask(uint32_t mask)
{
    if ( ! is_ce4200 )
    {
        reg_write(REG_WAKE_EVENT_MASK, mask);
        PWR_DEBUG("REG_WAKE_EVENT_MASK=0x%08x\n",reg_read(REG_WAKE_EVENT_MASK));
    }
    else
    {
        uint32_t mask_reg;

        PWR_DEBUG("Write event mask 0x%08x to PUnit register\n", mask);

        mask_reg  = reg_read(REG_WAKESM);

        // Clear mask bits.  Preserve status bits and reserved bits.
        mask_reg &= (0x0000ffff | WAKESM_RESERVED_BITS);

        // Set the new mask.
        // WAKESM register has only 16 mask bits, starting at bit 16.
        // They are represented in the upper 16 bits of the map.
        mask_reg |= WAKESM_ALWAYS_MASKED | (mask & 0xffff0000);

        reg_write(REG_WAKESM, mask_reg);
        PWR_DEBUG("WAKESM=0x%08x\n", mask_reg);
    }
}

//-----------------------------------------------------------------------------
// punit_pre_sleep
// 
// To be called before Soft Off or STR.
//-----------------------------------------------------------------------------
void punit_pre_sleep(void)
{
    if (is_ce4200)
    {
        // Workaround for CE4200 FW bug -- wake events are recognized even
        // when CPU is running, forcing all power islands on.  So only unmask
        // wake events before turning off CPU.
        _set_event_mask(wake_event_mask);
    }
}

//-----------------------------------------------------------------------------
// punit_post_sleep
// 
// To be called after power on or resume from STR.
//-----------------------------------------------------------------------------
void punit_post_sleep(void)
{
    int         bitnum;
    uint32_t    reg;

    // Determine the event that woke us.
    // 
    // On cold boot, this will be IO_8051_WAKE_EVT_NONE, as no bits will be set
    // in the wake status register
    //
    // On warm boot (resume from Soft Off) or resume from STR, this will be the
    // source of the wake event.

    wake_event = IO_8051_WAKE_EVT_NONE;

    // Retrieve and clear the wake event status bits from the PUnit.
    if ( is_ce4200 )
    {
        // WAKESM register has only 16 bits of event status, starting at bit 0.
        // They are represented in the upper 16 bits of the map.
        reg = reg_read( REG_WAKESM ) << 16;
        bitnum = 16;  // Bit mask starts with bit 16 (through 31)

        // In addition to clearing the status bits, workaround a CE4200 FW bug:
        // wake events are recognized even when CPU is running, forcing all
        // power islands on.  So mask all wake events while CPU is running.
        reg_write(REG_WAKESM, 0xffff0000);
    }
    else
    {
        reg = reg_read( REG_WAKE_EVENT_STATUS );
        bitnum = 0;  // Bit mask starts with bit 0 (through 31)
        reg_write(REG_WAKE_EVENT_STATUS, 0);
    }

    // There should only be a single wake bit set. We'll check them all anyway.
    // If more than one is set, we'll return the first and issue a warning.
    for ( ; bitnum < NUM_EVENT_MAP_ENTRIES; bitnum++)
    {
        if (reg & (1 << bitnum))
        {
            if (wake_event != IO_8051_WAKE_EVT_NONE)
            {
                PWR_ERROR("********************************\n");
                PWR_ERROR("* Multiple events in status register: 0x%08x\n",reg);
                PWR_ERROR("********************************\n");
                break;
            }

            wake_event = wake_event_map[bitnum];

            if (wake_event == IO_8051_WAKE_EVT_NONE)
            {
                // Internal error -- 8051 FW passed bad map or set wrong bit
                PWR_ERROR("********************************\n");
                PWR_ERROR("* BAD MAP: received wake event %d == NONE\n",bitnum);
                PWR_ERROR("********************************\n");
                break;
            }
        }
    }
}

//-----------------------------------------------------------------------------
// punit_mask_wake_event
//-----------------------------------------------------------------------------
icepm_ret_t punit_mask_wake_event(IO_8051_wake_event_t event)
{
    icepm_ret_t rc = ICEPM_OK;

    if (event >= IO_8051_WAKE_EVT_NUM_EVENTS)
    {
        PWR_ERROR("Invalid wake event: %u\n", event);
        rc = ICEPM_ERR_INVALID_EVENT;
    } 
    else if (event == IO_8051_WAKE_EVT_NONE)
    {
        wake_event_mask = 0; // Mask none
    }
    else
    {
        // Ignore if not in map -- must be an event not implemented on target
        // board.  Otherwise, set corresponding bit in mask.
        int i;
        for (i=0; i<NUM_EVENT_MAP_ENTRIES; i++)
        {
            if (wake_event_map[i] == event)
            {
                wake_event_mask |= (1 << i);
                break;
            }
        }

        if ( i >= NUM_EVENT_MAP_ENTRIES )
        {
            PWR_DEBUG("Event %u not in map\n", event);
        }
    }

    if ( !is_ce4200 )
    {
        _set_event_mask(wake_event_mask);
    }

    PWR_DEBUG("Event mask now 0x%08x\n", wake_event_mask);
    return rc;
}


//-----------------------------------------------------------------------------
// punit_unmask_wake_event
//-----------------------------------------------------------------------------
icepm_ret_t punit_unmask_wake_event(IO_8051_wake_event_t event)
{
    int i;
    icepm_ret_t rc = ICEPM_OK;

    if (event >= IO_8051_WAKE_EVT_NUM_EVENTS)
    {
        PWR_ERROR("Invalid wake event: %u\n", event);
        rc = ICEPM_ERR_INVALID_EVENT;
    }
    else if (event == IO_8051_WAKE_EVT_NONE)
    {
        wake_event_mask = 0xffffffff; // Mask all
    }
    else
    {
        // Ignore if not in map -- must be an event not implemented on target
        // board.  Otherwise, set corresponding bit in mask.
        for (i=0; i<NUM_EVENT_MAP_ENTRIES ; i++)
        {
            if (wake_event_map[i] == event)
            {
                wake_event_mask &= ~(1 << i);
            }
        }
    }

    if ( !is_ce4200 )
    {
        _set_event_mask(wake_event_mask);
    }

    PWR_DEBUG("Event mask now 0x%08x\n", wake_event_mask);
    return rc;
}

// Produce proc file output to seq file
void punit_event_map(struct seq_file *sfile)
{
    int i;
    char * event_name;

    for (i=0; i<NUM_EVENT_MAP_ENTRIES; i++)
    {
        switch ( wake_event_map[i] )
        {
        case IO_8051_WAKE_EVT_NONE:         event_name = NULL;          break;
        case IO_8051_WAKE_EVT_RTC:          event_name = "RTC";         break;
        case IO_8051_WAKE_EVT_IR:           event_name = "IR";          break;
        case IO_8051_WAKE_EVT_CEC:          event_name = "CEC";         break;
        case IO_8051_WAKE_EVT_WoWLAN:       event_name = "WoWLAN";      break;
        case IO_8051_WAKE_EVT_WoLAN:        event_name = "WoLAN";       break;
        case IO_8051_WAKE_EVT_WATCHDOG:     event_name = "WATCHDOG";    break;
        case IO_8051_WAKE_EVT_POWER_BUTTON: event_name = "POWER_BUTTON"; break;
        case IO_8051_WAKE_EVT_FP_INT:       event_name = "FP_INT";      break;
        case IO_8051_WAKE_EVT_BLUETOOTH:    event_name = "BLUETOOTH";   break;
        case IO_8051_WAKE_EVT_RF4CE:        event_name = "RF4CE";       break;
        case IO_8051_WAKE_EVT_RF_CUSTOM:    event_name = "RF_CUSTOM";   break;
        default:
            event_name = NULL;
            seq_printf(sfile, "[%d]=??? (0x%08x)\n", i, wake_event_map[i]);
            break;
        } 

        if (event_name)
        {
            seq_printf(sfile, "[%2d] %s\n", i, event_name);
        }
    }
}

static icepm_ret_t get_event_map(void)
{
    int         i, j;
    icepm_ret_t ret = ICEPM_ERR_INTERNAL;

    if ( is_ce4200 )
    {
        // Initialize map to work with legacy WAKESM register
        for (i=0; i<NUM_EVENT_MAP_ENTRIES; i++)
        {
            wake_event_map[i] = IO_8051_WAKE_EVT_NONE;
        }
        wake_event_map[18] = IO_8051_WAKE_EVT_RTC;          // 0x00040000
        wake_event_map[25] = IO_8051_WAKE_EVT_CEC;          // 0x02000000
        wake_event_map[28] = IO_8051_WAKE_EVT_IR;           // 0x10000000
        wake_event_map[31] = IO_8051_WAKE_EVT_POWER_BUTTON; // 0x80000000
    }
    else
    {
        // Read event map from map registers
        *(uint32_t*)(wake_event_map   ) = reg_read(REG_WAKE_EVENT_MAP_0);
        *(uint32_t*)(wake_event_map+ 4) = reg_read(REG_WAKE_EVENT_MAP_1);
        *(uint32_t*)(wake_event_map+ 8) = reg_read(REG_WAKE_EVENT_MAP_2);
        *(uint32_t*)(wake_event_map+12) = reg_read(REG_WAKE_EVENT_MAP_3);
        *(uint32_t*)(wake_event_map+16) = reg_read(REG_WAKE_EVENT_MAP_4);
        *(uint32_t*)(wake_event_map+20) = reg_read(REG_WAKE_EVENT_MAP_5);
        *(uint32_t*)(wake_event_map+24) = reg_read(REG_WAKE_EVENT_MAP_6);
        *(uint32_t*)(wake_event_map+28) = reg_read(REG_WAKE_EVENT_MAP_7);

        if (wake_event_map[31] != IO_8051_WAKE_EVT_RTC)
        {
            // map[31] *must* be for RTC (PUnit FW requires that)
            PWR_ERROR("Bad event map: [31] == %d\n", wake_event_map[31]);
            goto exit;
        }

        // Make sure no event (other than IO_8051_WAKE_EVT_NONE) appears more
        // than once in the table
        for ( i=0; i<NUM_EVENT_MAP_ENTRIES; i++)
        {
            if ( wake_event_map[i] == IO_8051_WAKE_EVT_NONE)
            {
                continue;
            }

            for ( j=i+1; j<NUM_EVENT_MAP_ENTRIES; j++)
            {
                if (wake_event_map[i] == wake_event_map[j])
                {
                    PWR_ERROR("Duplicate event map entry: "
                              "offsets %d and %d both are %u\n",
                              i, j, wake_event_map[i] );
                    goto exit;
                }
            }
        }
    }

    ret = ICEPM_OK;
exit:
    return ret;
}


void punit_close(void)
{
    // Assume system is going down
    punit_pre_sleep();
}


icepm_ret_t punit_init(pal_soc_info_t *soc_info)
{
    icepm_ret_t     ret = ICEPM_ERR_INTERNAL;
    os_pci_dev_t    dev;

    // Enable pci device.
    os_pci_enable_device(VENDOR_ID_INTEL, PUNIT_DEVICE_ID);

    switch ( (unsigned) soc_info->name)
    {
    case SOC_NAME_CE4200:
        is_ce4200 = true;

        // Get base I/O address of miniBAR (for message bus access)
        if (OSAL_SUCCESS != OS_PCI_FIND_FIRST_DEVICE(   VENDOR_ID_INTEL,
                                                        PUNIT_DEVICE_ID,
                                                        &dev))
        {
            PWR_ERROR("OS_PCI_FIND_FIRST_DEVICE failed\n");
            goto end;
        }
        else if (OSAL_SUCCESS != OS_PCI_READ_CONFIG_32( dev, 0x18, &miniBAR))
        {
            PWR_ERROR("OS_PCI_READ_CONFIG_32 failed\n");
            OS_PCI_FREE_DEVICE(dev);
            goto end;
        }
        else
        {
            // Low order bits hardwired/reserved; mask off
            miniBAR &= 0x0000FFFC;
            OS_PCI_FREE_DEVICE(dev);
        }
        break;
    case SOC_NAME_CE5300:
        is_ce4200 = false;
        break;
    default:
        PWR_ERROR("Invalid SoC (%d) for punit_init\n", soc_info->name);
        goto end;
    }

    ret = get_event_map();

    punit_post_sleep();

    if ( !is_ce4200 )
    {
        _set_event_mask(0);
    }

end:
    return ret;
}
