/*
 *	BSD 3-Clause License
 *
 * 	©2023 Janusz Kostorz
 * 	All rights reserved.
 *
 * 	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *		1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *		2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *		3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * 	THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * 	IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * 	End of BSD license
 *
 */

/*
 * Hardware: Arduino Pro Micro (ATMega32U4) + TSOP1736 on pin 3 (B0) + [logic analyzer on pin 16 (B2)]
 *
 */

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "hidrc.h"

volatile uint8_t usb;
volatile int8_t infrared = 0;
volatile uint32_t rc = 0;

int16_t main(void)
{

    // Disable interrupts
    cli();

    // Configure ports for led and debug
    LED_DDRB |= LED_IO;
    LED_PORT |= LED_IO;
    LOGIC_DDRB |= LOGIC_IO;
    LOGIC_PORT |= LOGIC_IO;

    // Enable USB pad regulator, configure PLL prescaler, start PLL
    UHWCON |= (1 << UVREGE);
    PLLCSR |= (1 << PINDIV);
    PLLCSR |= (1 << PLLE);

    // Wait for PLL lock
    while (!(PLLCSR & (1 << PLOCK)))
        ;

    // Enable USB controller, enable VBUS pad, unfreze USB clock
    USBCON |= (1 << USBE) | (1 << OTGPADE);
    USBCON &= ~(1 << FRZCLK);

    // Disable USB low speed, reconnect internall pullup D+ and D-
    UDCON &= ~(1 << LSM);
    UDCON &= ~(1 << DETACH);

    // Enable USB interrupts - End Of Resume, Start Of Frame
    UDIEN |= (1 << EORSTE) | (1 << SOFE);

    // Clear usb config flag
    usb = 0;

    // Configure pullup and external interrupt INT0 with falling edge for IR sensor scanning
    PORTD |= (1 << 0);
    EICRA |= (1 << ISC01);
    EIMSK |= (1 << INT0);

    // Enable watchdog, interrupt after 125ms
    wdt_reset();
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = (1 << WDIE) | (1 << WDP1) | (1 << WDP0);

    // Enable interrupts
    sei();

    // Wait for USB
    while (!usb)
        wait_ms(100);

    // Read eeprom
    for (uint8_t i = 0; i < sizeof(eep); i++)
        eeprom[i] = ee_read_l(i);

    // Bad eeprom id - fill default values
    if (((eeprom[0] & ~idRC_Mask) != idData) || (eeprom[1] == eeprom[2] && eeprom[2] == eeprom[3] && eeprom[3] == eeprom[4] && eeprom[4] == eeprom[5] && eeprom[5] == eeprom[6] && eeprom[6] == eeprom[7] && eeprom[7] == eeprom[8]))
    {
        eeprom[0] = idRC;
        eeprom[1] = RC_PLAY;
        eeprom[2] = RC_PAUSE;
        eeprom[3] = RC_NEXT;
        eeprom[4] = RC_PREV;
        eeprom[5] = RC_STOP;
        eeprom[6] = RC_PLPAUSE;
        eeprom[7] = RC_BRIGHT_INC;
        eeprom[8] = RC_BRIGHT_DEC;
    }

    uint8_t hid = 0;

    // Main loop - translate IR remote control command to USB HID command
    while (1)
    {

        // Reset watchdog
        wdt_reset();

        // New interrupt from ir sensor
        if (infrared == 1)
        {

            // Logic analyzer
            LOGIC_PORT &= ~LOGIC_IO;
            LOGIC_PORT |= LOGIC_IO;

            // Decode RC5 pulses
            if ((eeprom[0] & idRC_Mask) == idRC_RC5)
            {

                // Save last code
                uint32_t lrc = rc;

                // Full code control
                rc = 3;

                // Start pulse
                wait_us(IR_RC5_PULSE * 0.5);

                // Check pulses
                for (uint8_t i = 0; i < 13 && infrared == 1; i++)
                {
                    wait_us(IR_RC5_PULSE);

                    // Shift on full pulse
                    rc = rc << 1;

                    // Shift before address
                    if (i == 2)
                        rc = rc << 3;

                    // Shift before command
                    if (i == 7)
                        rc = rc << 2;

                    // Logic analyzer
                    LOGIC_PORT &= ~LOGIC_IO;

                    // "1"
                    if (PIND & (1 << 0))
                    {
                        while ((PIND & (1 << 0)) && infrared == 1)
                            LOGIC_PORT ^= LOGIC_IO;
                        LOGIC_PORT &= ~LOGIC_IO;
                        wait_us(IR_RC5_PULSE / 2);
                        rc |= 1;
                    }

                    // "0"
                    else
                    {
                        while (!(PIND & (1 << 0)) && infrared == 1)
                            LOGIC_PORT ^= LOGIC_IO;
                        LOGIC_PORT &= ~LOGIC_IO;
                        wait_us(IR_RC5_PULSE / 2);
                    }

                    // Logic analyzer
                    LOGIC_PORT |= LOGIC_IO;
                }

                // Check full remote control code
                if (!(rc & 0b000010000000000000000000))
                    rc = lrc;

                // Check RC5 extended
                if (!(rc & 0b000000100000000000000000))
                    rc |= 0b000000000000000001000000;

                // Check repeat command
                if ((rc & 0b111111110000000000000000) == (lrc & 0b111111110000000000000000))
                {
                    // Lock repeat other than this commands
                    if (hid != HID_BRIGHT_INC && hid != HID_BRIGHT_DEC)
                        rc = rc & 0b111111110000000000000000;
                }
            }

            // Decode RC6 pulses
            else if ((eeprom[0] & idRC_Mask) == idRC_RC6)
            {
                // Save last code
                uint32_t lrc = rc;

                // Full code control
                rc = 1;

                // Leader pulse
                wait_us(IR_RC6_PULSE * 3.5);

                // Check pulses
                for (uint8_t i = 0; i < 22 && infrared == 1; i++)
                {
                    wait_us(IR_RC6_PULSE);

                    // Shift on full pulse
                    rc = rc << 1;

                    // Logic analyzer
                    LOGIC_PORT &= ~LOGIC_IO;

                    // "1"
                    if (!(PIND & (1 << 0)))
                    {
                        // Extra time for leader and trailer bits
                        if (i == 0 || i == 5)
                            wait_us(IR_RC6_PULSE);
                        while (!(PIND & (1 << 0)) && infrared == 1)
                            LOGIC_PORT ^= LOGIC_IO;
                        LOGIC_PORT &= ~LOGIC_IO;
                        wait_us(IR_RC6_PULSE / 2);
                        rc |= 1;
                    }

                    // "0"
                    else
                    {
                        while ((PIND & (1 << 0)) && infrared == 1)
                            LOGIC_PORT ^= LOGIC_IO;
                        LOGIC_PORT &= ~LOGIC_IO;
                        wait_us(IR_RC6_PULSE / 2);
                    }

                    // Logic analyzer
                    LOGIC_PORT |= LOGIC_IO;

                    // Extra time for leader and trailer bits
                    if (i == 5 || i == 0)
                        wait_us(IR_RC6_PULSE);
                }

                // Check full remote control code
                if (!(rc & 0b010000000000000000000000))
                    rc = lrc;

                // Check leader and trailer bits
                else if ((rc & 0b001100000000000000000000) != 0b001100000000000000000000)
                    rc = lrc;

                // Check repeat command
                if ((rc & 0b111111110000000000000000) == (lrc & 0b111111110000000000000000))
                {
                    // Lock repeat other than this commands
                    if (hid != HID_BRIGHT_INC && hid != HID_BRIGHT_DEC)
                        rc = rc & 0b111111110000000000000000;
                }
            }

            // Decode IR NEC Enhaced pulses
            else if ((eeprom[0] & idRC_Mask) == idRC_NEC)
            {
                // Clear old code
                rc = 0;

                // Long leading pulse - return 0b000000000000000011111111 for code
                // Short leading pulse - return 0b000000000000000011110111 for repeat
                for (uint8_t i = 0; i < 24 && infrared == 1; i++)
                {
                    // Shift on full pulse
                    rc = rc << 1;
                    rc |= PIND & (1 << 0);

                    // Logic analyzer
                    if (i == 20)
                    {
                        LOGIC_PORT ^= LOGIC_IO;
                        LOGIC_PORT ^= LOGIC_IO;
                    }
                    if (i == 15)
                    {
                        // Pulse rising edge sync
                        while (!(PIND & (1 << 0)) && infrared == 1)
                            LOGIC_PORT ^= LOGIC_IO;
                        LOGIC_PORT &= ~LOGIC_IO;

                        // Go to half pulse
                        wait_us((IR_NEC_PULSE / 2));

                        // Logic analyzer
                        LOGIC_PORT |= LOGIC_IO;
                    }
                    else
                        wait_us(IR_NEC_PULSE);
                }

                // Long leading pulse - code
                if (rc == 0b000000000000000011111111)
                {

                    rc = 0;

                    // Decode pulses
                    for (uint8_t i = 0; i < 32 && infrared == 1; i++)
                    {
                        // Pulse rising edge sync
                        while (!(PIND & (1 << 0)) && infrared == 1)
                            LOGIC_PORT ^= LOGIC_IO;
                        LOGIC_PORT &= ~LOGIC_IO;
                        wait_us(IR_NEC_PULSE * 1.5);
                        rc = rc << 1;
                        rc |= PIND & (1 << 0);
                        // Logic analyzer
                        LOGIC_PORT |= LOGIC_IO;
                        if (rc & 1)
                            wait_us(IR_NEC_PULSE * 2);
                    }

                    // Convert lsb to msb and swap Addr Hi with AddrLo
                    rc = (((rc & 0xaaaaaaaa) >> 1) | ((rc & 0x55555555) << 1));
                    rc = (((rc & 0xcccccccc) >> 2) | ((rc & 0x33333333) << 2));
                    rc = (((rc & 0xf0f0f0f0) >> 4) | ((rc & 0x0f0f0f0f) << 4));
                    rc = (((rc & 0xff000000) >> 8) | ((rc & 0x00ff0000) << 8) | (rc & 0x0000ffff));
                }

                // Short leading pulse - repeat command
                else if (rc == 0b000000000000000011110111)
                {
                    // Lock repeat for this commands
                    if (hid == HID_STOP || hid == HID_PLAY || hid == HID_PLPAUSE || hid == HID_PAUSE || hid == HID_NEXT || hid == HID_PREV)
                        rc = 0;
                }

                // Bad leading pulse for IR NEC format - ignore
                else
                    rc = 0;
            }

            // Repeat for NEC
            if ((eeprom[0] & idRC_Mask) == idRC_NEC && rc == 0b000000000000000011110111)
                ;
            // Bit mask for RC5 - s1, s2, address * 5, command * 7, (ignore toggle)
            // Bit mask for RC6 - leader, mode * 3, trailer, control * 8 information * 8, (ignore toggle)
            else if (rc == eeprom[1] || (((eeprom[0] & idRC_Mask) == idRC_RC5 && (rc & 0b000001100001111101111111) == eeprom[1])) || ((eeprom[0] & idRC_Mask) == idRC_RC6 && (rc & 0b001111101111111111111111) == eeprom[1]))
                hid = HID_PLAY;
            else if (rc == eeprom[2] || (((eeprom[0] & idRC_Mask) == idRC_RC5 && (rc & 0b000001100001111101111111) == eeprom[2])) || ((eeprom[0] & idRC_Mask) == idRC_RC6 && (rc & 0b001111101111111111111111) == eeprom[2]))
                hid = HID_PAUSE;
            else if (rc == eeprom[3] || (((eeprom[0] & idRC_Mask) == idRC_RC5 && (rc & 0b000001100001111101111111) == eeprom[3])) || ((eeprom[0] & idRC_Mask) == idRC_RC6 && (rc & 0b001111101111111111111111) == eeprom[3]))
                hid = HID_NEXT;
            else if (rc == eeprom[4] || (((eeprom[0] & idRC_Mask) == idRC_RC5 && (rc & 0b000001100001111101111111) == eeprom[4])) || ((eeprom[0] & idRC_Mask) == idRC_RC6 && (rc & 0b001111101111111111111111) == eeprom[4]))
                hid = HID_PREV;
            else if (rc == eeprom[5] || (((eeprom[0] & idRC_Mask) == idRC_RC5 && (rc & 0b000001100001111101111111) == eeprom[5])) || ((eeprom[0] & idRC_Mask) == idRC_RC6 && (rc & 0b001111101111111111111111) == eeprom[5]))
                hid = HID_STOP;
            else if (rc == eeprom[6] || (((eeprom[0] & idRC_Mask) == idRC_RC5 && (rc & 0b000001100001111101111111) == eeprom[6])) || ((eeprom[0] & idRC_Mask) == idRC_RC6 && (rc & 0b001111101111111111111111) == eeprom[6]))
                hid = HID_PLPAUSE;
            else if (rc == eeprom[7] || (((eeprom[0] & idRC_Mask) == idRC_RC5 && (rc & 0b000001100001111101111111) == eeprom[7])) || ((eeprom[0] & idRC_Mask) == idRC_RC6 && (rc & 0b001111101111111111111111) == eeprom[7]))
                hid = HID_BRIGHT_INC;
            else if (rc == eeprom[8] || (((eeprom[0] & idRC_Mask) == idRC_RC5 && (rc & 0b000001100001111101111111) == eeprom[8])) || ((eeprom[0] & idRC_Mask) == idRC_RC6 && (rc & 0b001111101111111111111111) == eeprom[8]))
                hid = HID_BRIGHT_DEC;
            else
                hid = 0;

            if (!usb)
                return -1;
            UENUM = ENDPOINT;

            if (hid)
            {

                // Led on , logic 0
                LED_PORT &= ~LED_IO;
                LOGIC_PORT ^= LOGIC_IO;

                // Send hid consumer command
                while (!(UEINTX & (1 << RWAL)))
                    ;
                UEDATX = PAGE_CONSUMER;
                UEDATX = hid;
                UEINTX = 0b00111010;
                UEDATX = PAGE_CONSUMER;
                UEDATX = 0;
                UEINTX = 0b00111010;

                // // Ignore first repeat NEC code and wait for end of all transmisions
                if ((eeprom[0] & idRC_Mask) == idRC_NEC)
                {

                    for (uint8_t i = 0; i < (IR_NEC_PULSES_ALL - IR_NEC_PULSES_MCOD); i++)
                    {
                        if (!(PIND & (1 << 0)))
                            i = 0;
                        wait_us(IR_NEC_PULSE);
                    }
                }

                // Led off, logic 1
                LOGIC_PORT ^= LOGIC_IO;
                LED_PORT |= LED_IO;
            }

            // Accept next infrared pulses
            LOGIC_PORT |= LOGIC_IO;
            infrared = 0;
        }
    }
}

// New code from remote control
ISR(INT0_vect)
{
    infrared = 1;
}

ISR(USB_GEN_vect)
{
    uint8_t udint = UDINT;
    UDINT = 0;

    if (udint & (1 << EORSTI))
    {

        UENUM = 0;
        UECONX = (1 << EPEN);
        UECFG0X = 0;
        UECFG1X |= 0x22;
        usb = 0;

        // Check endpoint config
        if (!(UESTA0X & (1 << CFGOK)))
            return;

        UERST = 1;
        UERST = 0;

        UEIENX = (1 << RXSTPE);
        return;
    }
}

ISR(USB_COM_vect)
{
    UENUM = 0;

    if (UEINTX & (1 << RXSTPI))
    {
        // Get bmRequestType, bRequest, wValue, wIndex, wLength from host
        uint8_t bmRequestType = UEDATX;
        uint8_t bRequest = UEDATX;
        uint16_t wValue = UEDATX;
        wValue |= UEDATX << 8;
        uint16_t wIndex = UEDATX;
        wIndex |= UEDATX << 8;
        uint16_t wLength = UEDATX;
        wLength |= UEDATX << 8;

        // Handshake the interrupts
        UEINTX &= ~((1 << RXSTPI) | (1 << RXOUTI) | (1 << TXINI));

        if (bRequest == GET_STATUS)
        {
            // Wait until the banks are ready to be filled
            while (!(UEINTX & (1 << TXINI)))
                ;

            UEDATX = 0;
            UEDATX = 0;
            UEINTX &= ~(1 << TXINI);
            return;
        }

        if (bRequest == SET_ADDRESS)
        {
            // Wait until the banks are ready to be filled
            UEINTX &= ~(1 << TXINI);
            while (!(UEINTX & (1 << TXINI)))
                ;

            UDADDR = wValue | (1 << ADDEN);
            return;
        }

        if (bRequest == GET_DESCRIPTOR)
        {

            uint8_t *descriptor;
            uint8_t descriptor_length;

            switch (wValue)
            {

            case 0x0100:
                descriptor = device_descriptor;
                descriptor_length = sizeof(device_descriptor);
                break;

            case 0x0600:
                descriptor = device_qualifier_descriptor;
                descriptor_length = sizeof(device_qualifier_descriptor);
                break;

            case 0x0200:
                descriptor = configuration_descriptor;
                descriptor_length = sizeof(configuration_descriptor);
                break;

            case 0x2100:
                descriptor = configuration_descriptor;
                descriptor += HID_OFFSET;
                descriptor_length = sizeof(configuration_descriptor) - HID_OFFSET;
                break;

            case 0x2200:
                descriptor = interface_descriptor;
                descriptor_length = sizeof(interface_descriptor);
                break;

            case 0x0300:
                descriptor = string_descriptor;
                descriptor_length = sizeof(string_descriptor);
                break;

            case 0x0301:
                descriptor = string_descriptor_iManufacturer;
                descriptor_length = sizeof(string_descriptor_iManufacturer);
                string_descriptor_iManufacturer[0] = descriptor_length;
                break;

            case 0x0302:
                descriptor = string_descriptor_iProduct;
                descriptor_length = sizeof(string_descriptor_iProduct);
                string_descriptor_iProduct[0] = descriptor_length;
                break;

            case 0x0303:
                descriptor = string_descriptor_iSerialNumber;
                descriptor_length = sizeof(string_descriptor_iSerialNumber);
                string_descriptor_iSerialNumber[0] = descriptor_length;
                break;

            default:
                // Enable stall request and endpoint
                UECONX |= (1 << STALLRQ) | (1 << EPEN);
            }

            // Truncate to 255 or wLength or descriptor_length
            wLength = wLength > 255 ? 255 : wLength;
            descriptor_length = wLength < descriptor_length ? wLength : descriptor_length;

            // Sending descriptor
            while (descriptor_length > 0)
            {
                // Wait for banks to be ready for data transmission
                while (!(UEINTX & (1 << TXINI)))
                    ;

                // If there is another packet, exit to handle it
                if (UEINTX & (1 << RXOUTI))
                    return;

                // Truncate packets to 32 bytes
                uint8_t j = descriptor_length > 32 ? 32 : descriptor_length;

                // Send 32 bytes
                for (int i = 0; i < j; i++)
                    UEDATX = descriptor[i];

                descriptor_length -= j;
                descriptor += j;

                UEINTX &= ~(1 << TXINI);
            }
            return;
        }

        if (bRequest == GET_CONFIGURATION && bmRequestType == 0x80)
        {
            while (!(UEINTX & (1 << TXINI)))
                ;

            UEDATX = usb;
            UEINTX &= ~(1 << TXINI);
            return;
        }

        if (bRequest == SET_CONFIGURATION && bmRequestType == 0)
        {
            usb = wValue;
            UEINTX &= ~(1 << TXINI);
            UENUM = ENDPOINT;
            UECONX = 1;

            // Config endpoint to interrupt in
            UECFG0X = (1 << EPTYPE1) | (1 << EPTYPE0) | (1 << EPDIR);

            // Config endpoint to 8 bytes, dual bank, allocate memory
            UECFG1X = (1 << EPBK0) | (1 << ALLOC);

            // Reset endpoint fifo
            UERST = (1 << EPRST4) | (1 << EPRST3) | (1 << EPRST2) | (1 << EPRST1);
            UERST = 0;
            return;
        }

        if (wIndex == 0)
        {
            if (bmRequestType == 0xA1)
            {
                if ((bRequest == GET_REPORT) | (bRequest == GET_IDLE))
                {
                    while (!(UEINTX & (1 << TXINI)))
                        ;

                    return;
                }
            }
        }
    }
    UECONX |= (1 << STALLRQ) | (1 << EPEN);
}

ISR(WDT_vect)
{
    LOGIC_PORT &= ~LOGIC_IO;

    infrared = -1;
}

// EEPROM read
uint32_t ee_read_l(uint8_t addr)
{
    uint32_t r = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        r = r << 8;
        while (EECR & (1 << EEPE))
            ;
        EEAR = (addr << 2) + i;
        EECR |= (1 << EERE);
        r |= EEDR;
    }
    r = (r & 0xffff0000) >> 16 | (r & 0x0000ffff) << 16;
    r = (r & 0xff00ff00) >> 8 | (r & 0x00ff00ff) << 8;
    return r;
}

// Delay functions
static inline void asm_wait_us(uint16_t count)
{
    asm volatile("cp %A0, __zero_reg__ \n\t"
                 "cpc %B0, __zero_reg__ \n\t"
                 "breq loop_out_%= \n\t"
                 "loop%=: \n\t"
                 "sbiw %0, 1 \n\t"
                 "brne loop%= \n\t"
                 "loop_out_%=: \n\t"
                 : "=w"(count)
                 : "0"(count));
}

void wait_ms(uint16_t time)
{
    while (time--)
        wait_us(1000);
}
