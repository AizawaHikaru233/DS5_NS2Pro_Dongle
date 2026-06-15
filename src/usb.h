//
// Created by awalol on 2026/3/4.
//

#ifndef DS5_BRIDGE_USB_H
#define DS5_BRIDGE_USB_H

#include <stdint.h>

extern uint8_t mute[2]; // 0: SPEAKER(0x02) 1: MIC(0x05)
extern float volume[2]; // 0: SPEAKER(0x02) 1: MIC(0x05)

enum UsbBridgePersona : uint8_t {
    USB_BRIDGE_PERSONA_MANAGER = 0,
    USB_BRIDGE_PERSONA_DUALSENSE = 1,
};

void usb_bridge_connect();
void usb_bridge_disconnect();
bool usb_bridge_is_connected();
UsbBridgePersona usb_bridge_persona();
bool usb_bridge_is_manager_persona();
bool usb_bridge_is_dualsense_persona();
void usb_bridge_hold_dualsense_for_ms(uint32_t hold_ms);
bool usb_bridge_should_hold_dualsense();
void usb_bridge_request_persona(UsbBridgePersona persona);
void usb_bridge_task();

#endif //DS5_BRIDGE_USB_H
