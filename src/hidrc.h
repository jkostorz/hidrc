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

#ifndef HIDRC
    #define HIDRC

    // Config output for led
    #define LED_DDRB        DDRB
    #define LED_PORT        PORTB
    #define LED_IO          (1 << 0)

    // Config output for debugging
    #define LOGIC_DDRB      DDRB
    #define LOGIC_PORT      PORTB
    #define LOGIC_IO        (1 << 2)

    // id
    #define idVendor                0x16d0      // MCS Electronics
    #define idProduct               0x1241      // HID Device - Janusz Kostorz !!! please don't use this id in forks !!!
    #define idData                  0x16d01241  // EEPROM id
    #define idRC_Mask               0b00000000000011110000000000000000
    #define idRC_NEC                0b00000000000000010000000000000000
    #define idRC_RC5                0b00000000000001010000000000000000
    #define idRC_RC6                0b00000000000001110000000000000000
    #define idRC_SIRC               0b00000000000010000000000000000000

    // IR NEC standard - LSB to MSB - Addr Lo + Addr Hi + Cmd + Neg Cmd
    #define IR_NEC_PULSE        560         // IR NEC pulse time (us)
    #define IR_NEC_PULSES_ALL   192         // IR NEC all pulses in one loop
    #define IR_NEC_PULSES_MCOD  105         // IR NEC minimum pulses for full transmision code

    // IR RC5 standard
    #define IR_RC5_PULSE        889         // IR RC5 pulse time (us)

    // IR RC6 standard
    #define IR_RC6_PULSE        444         // IR RC6 pulse time (us)

    // Remote control - ONKYO RC-645S
    #ifdef ONKYO_RC645S
        #define idRC            idRC_NEC
        #define RC_PLAY         0x02d21ae5  // RC-645S HDD mode - play button
        #define RC_PAUSE        0x02d21ee1  // RC-645S HDD mode - pause button
        #define RC_NEXT         0x02d21ce3  // RC-645S HDD mode - next button
        #define RC_PREV         0x02d21de2  // RC-645S HDD mode - prev button
        #define RC_STOP         0x02d21be4  // RC-645S HDD mode - stop button
        #define RC_PLPAUSE      0xffffffff
        #define RC_BRIGHT_INC   0x02d257a8  // RC-645S HDD mode - level + button
        #define RC_BRIGHT_DEC   0x02d256a9  // RC-645S HDD mode - level - button
    #endif

    // Remote control - ONKYO RC-725DV
    #ifdef ONKYO_RC725DV
        #define idRC            idRC_NEC
        #define RC_PLAY         0x2bd21be4  // RC-725DV - play button
        #define RC_PAUSE        0x2bd21fe0  // RC-725DV - pause button
        #define RC_NEXT         0x2bd21de2  // RC-725DV - next button
        #define RC_PREV         0x2bd21ee1  // RC-725DV - prev button
        #define RC_STOP         0x2bd21ce3  // RC-725DV - stop button
        #define RC_PLPAUSE      0xffffffff
        #define RC_BRIGHT_INC   0x2bd20cf3  // RC-725DV - >> button
        #define RC_BRIGHT_DEC   0x2bd20df2  // RC-725DV - << button
    #endif

    // Remote control - ONKYO RC-959S
    #ifdef ONKYO_RC959S
        #define idRC            idRC_NEC
        #define RC_PLAY         0xffffffff
        #define RC_PAUSE        0xffffffff
        #define RC_NEXT         0x09d21ce3  // RC-959S - network audio next button
        #define RC_PREV         0x09d21de2  // RC-959S - network audio prev button
        #define RC_STOP         0x09d25aa5  // RC-959S - network audio menu button
        #define RC_PLPAUSE      0x09d21fe0  // RC-959S - network audio play pause button
        #define RC_BRIGHT_INC   0x04d255aa  // RC-959S - display button
        #define RC_BRIGHT_DEC   0x03d2956a  // RC-959S - dimmer button
    #endif

    // RC5 test
    #ifdef idRC_TEST5
        #define idRC            idRC_RC5
        #define RC_PLAY         0xffffffff
        #define RC_PAUSE        0xffffffff
        #define RC_NEXT         0x00040056
        #define RC_PREV         0x00040055
        #define RC_STOP         0xffffffff
        #define RC_PLPAUSE      0x00040057
        #define RC_BRIGHT_INC   0x00040051
        #define RC_BRIGHT_DEC   0x00040050
    #endif

    // RC6 test
    #ifdef idRC_TEST6
        #define idRC            idRC_RC6
        #define RC_PLAY         0xffffffff
        #define RC_PAUSE        0xffffffff
        #define RC_NEXT         0x0030005b
        #define RC_PREV         0x0030005a
        #define RC_STOP         0xffffffff
        #define RC_PLPAUSE      0x0030005c
        #define RC_BRIGHT_INC   0x00300058
        #define RC_BRIGHT_DEC   0x00300059
    #endif

    // No remote control define
    #ifndef idRC
        #define idRC            0
        #define RC_PLAY         0xffffffff
        #define RC_PAUSE        0xffffffff
        #define RC_NEXT         0xffffffff
        #define RC_PREV         0xffffffff
        #define RC_STOP         0xffffffff
        #define RC_PLPAUSE      0xffffffff
        #define RC_BRIGHT_INC   0xffffffff
        #define RC_BRIGHT_DEC   0xffffffff
    #endif

    // EEPROM data
    volatile uint32_t EEMEM eep[] = {
        idData | idRC,                      // l_idData and idRC
        RC_PLAY,                            // l_RcCode_Play
        RC_PAUSE,                           // l_RcCode_Pause
        RC_NEXT,                            // l_RcCode_Next
        RC_PREV,                            // l_RcCode_Previous
        RC_STOP,                            // l_RcCode_Stop
        RC_PLPAUSE,                         // l_RcCode_PlayPause
        RC_BRIGHT_INC,                      // l_RcCode_Bright_Increment
        RC_BRIGHT_DEC,                      // l_RcCode_Bright_Decrement
        -1,                                 // l_RcCode_Reserved
        -1,                                 // l_RcCode_Reserved
        -1,                                 // l_RcCode_Reserved
        -1,                                 // l_RcCode_Reserved
        -1,                                 // l_RcCode_Reserved
        -1,                                 // l_RcCode_Reserved
        -1,                                 // l_RcCode_Reserved
        -1,                                 // l_RcCode_Reserved
    };
    volatile uint32_t eeprom[sizeof(eep)];

    // USB requests
    #define GET_STATUS              0x00
    #define GET_REPORT              0x01
    #define GET_IDLE                0x02
    #define SET_ADDRESS             0x05
    #define GET_DESCRIPTOR          0x06
    #define GET_CONFIGURATION       0x08
    #define SET_CONFIGURATION       0x09
    #define SET_IDLE                0x0a

    #define HID_PLAY                0b00000001
    #define HID_PAUSE               0b00000010
    #define HID_NEXT                0b00000100
    #define HID_PREV                0b00001000
    #define HID_STOP                0b00010000
    #define HID_PLPAUSE             0b00100000
    #define HID_BRIGHT_INC          0b01000000
    #define HID_BRIGHT_DEC          0b10000000

    #define ENDPOINT                0b00000011
    #define PAGE_CONSUMER           0x01

    // USB descriptors
    static uint8_t interface_descriptor[] = {
        0x05, 0x0c,                         // Usage Page - Consumer
        0x09, 0x01,                         // Usage - Consumer Control
        0xa1, 0x01,                         // Collection
            0x85, 0x01,                     // Report ID
            0x75, 0x01,                     // Report Size
            0x95, 0x08,                     // Report Count
            0x15, 0x00,                     // Logical Minimum
            0x25, 0x01,                     // Logical Maximum
            0x09, 0xb0,                     // Usage - Play
            0x09, 0xb1,                     // Usage - Pause
            0x09, 0xb5,                     // Usage - Scan Next Track
            0x09, 0xb6,                     // Usage - Scan Previous Track
            0x09, 0xb7,                     // Usage - Stop
            0x09, 0xcd,                     // Usage - Play/Pause
            0x09, 0x6f,                     // Display Brightness Increment
            0x09, 0x70,                     // Display Brightness Decrement
            0x81, 0x02,                     // Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0xC0,                               // End Collection
    };

    #define HID_OFFSET 18

    static uint8_t configuration_descriptor[] = {
        9,                                  // bLength
        0x02,                               // bDescriptorType
        34, 0,                              // wTotalLength
        0x01,                               // bNumInterfaces
        0x01,                               // bConfigurationValue
        0,                                  // iConfiguration
        0b10000000,                         // bmAttributes
        25,                                 // bMaxPower

        9,                                  // bLength
        0x04,                               // bDescriptorType
        0x00,                               // bInterfaceNumber
        0x00,                               // bAlternateSetting
        1,                                  // bNumEndpoints
        0x03,                               // bInterfaceClass
        0x00,                               // bInterfaceSubClass
        0x01,                               // bInterfaceProtocol
        0,                                  // iInterface

        9,                                  // bLength
        0x21,                               // bDescriptorType
        0x11, 0x01,                         // bcdHID
        0x00,                               // bCountryCode
        1,                                  // bNumDescriptors
        0x22,                               // bDescriptorType
        sizeof(interface_descriptor), 0,    // wDescriptorLength

        7,                                  // bLength
        0x05,                               // bDescriptorType
        0b10000011,                         // bEndPointAddress
        0b00000011,                         // bmAttributes
        32, 0,                              // wMaxPacketSize
        0x01                                // wInterval
    };

    static uint8_t device_qualifier_descriptor[]  = {
        10,                                 // bLength
        6,                                  // bDescriptorType
        0x00, 0x02,                         // bcdUSB
        0,                                  // bDeviceClass
        0,                                  // bDeviceSubClass
        0,                                  // bDeviceProtocol
        32,                                 // bMaxPacketSize0
        1,                                  // bNumConfigurations
        0,                                  // bReserved
    };

    static uint8_t device_descriptor[]  = {
        18,                                 // bLength
        1,                                  // bDescriptorType
        0x00, 0x02,                         // bcdUSB
        0,                                  // bDeviceClass
        0,                                  // bDeviceSubClass
        0,                                  // bDeviceProtocol
        32,                                 // bMaxPacketSize0
        (idVendor & 255),                   // idVendor
            ((idVendor >> 8) & 255),
        (idProduct & 255),                  // idProduct
            ((idProduct >> 8) & 255),
        0x00, 0x01,                         // bcdDevice
        1,                                  // iManufacturer
        2,                                  // iProduct
        3,                                  // iSerialNumber
        1,                                  // bNumConfigurations
    };

    static uint8_t string_descriptor[] = {
        4,                                  // bLength
        0x03,                               // bDescriptionType
        0x09,0x04,                          // bString
    };

    static uint8_t string_descriptor_iManufacturer[] = {
        0,                                  // bLength
        0x03,                               // bDescriptionType
        'J', 0x00,                          // bString
        '.', 0x00,
        'K', 0x00,
        'o', 0x00,
        's', 0x00,
        't', 0x00,
        'o', 0x00,
        'r', 0x00,
        'z', 0x00,
    };

    static uint8_t string_descriptor_iProduct[] = {
        0,                                  // bLength
        0x03,                               // bDescriptionType
        'H', 0x00,                          // bString
        'I', 0x00,
        'D', 0x00,
        'R', 0x00,
        'C', 0x00,
    };

    static uint8_t string_descriptor_iSerialNumber[] = {
        0,                                  // bLength
        0x03,                               // bDescriptionType
        '2', 0x00,                          // bString
        '3', 0x00,
        '0', 0x00,
        '4', 0x00,
        '1', 0x00,
        '9', 0x00,
    };

    uint32_t ee_read_l(uint8_t addr);

    static inline void asm_wait_us(uint16_t count);
    void wait_ms(uint16_t time);
    #define wait_us(us) asm_wait_us((uint16_t)(((((us)*1000L) / (1000000000 / F_CPU)) - 1) / 4))

#endif
