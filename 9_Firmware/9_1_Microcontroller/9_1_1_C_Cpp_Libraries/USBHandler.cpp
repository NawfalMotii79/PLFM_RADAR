#include "USBHandler.h"
#include "diag_log.h"
#include <cstring>

USBHandler::USBHandler() {
    DIAG("USB", "USBHandler constructed, calling reset()");
    reset();
}

void USBHandler::reset() {
    DIAG("USB", "USBHandler::reset(): state -> WAITING_FOR_START");
    current_state = USBState::WAITING_FOR_START;
    start_flag_received = false;
    buffer_index = 0;
    current_settings.resetToDefaults();
    fault_ack_received = false;
}

void USBHandler::processUSBData(const uint8_t* data, uint32_t length) {
    if (data == nullptr || length == 0) {
        DIAG_WARN("USB", "processUSBData: null/empty data");
        return;
    }
    
    DIAG("USB", "processUSBData: %lu bytes, state=%d", (unsigned long)length, (int)current_state);
    
    // FAULT_ACK: host sends exactly 4 bytes [0x40, 0x00, 0x00, 0x00].
    // Requires exact 4-byte packet length: settings packets are always
    // >= 82 bytes, so a lone 4-byte payload is unambiguous. Scanning
    // inside larger packets would false-trigger on the IEEE 754
    // encoding of 2.0 (0x4000000000000000) embedded in settings doubles.
    static const uint8_t FAULT_ACK_SEQ[4] = {0x40, 0x00, 0x00, 0x00};
    if (length == 4 && memcmp(data, FAULT_ACK_SEQ, 4) == 0) {
        fault_ack_received = true;
        DIAG("USB", "FAULT_ACK received");
        return;
    }
    
    switch (current_state) {
        case USBState::WAITING_FOR_START:
            processStartFlag(data, length);
            break;
            
        case USBState::RECEIVING_SETTINGS:
            processSettingsData(data, length);
            break;
            
        case USBState::READY_FOR_DATA:
            // Ready to receive radar data commands
            DIAG("USB", "  READY_FOR_DATA: ignoring %lu bytes", (unsigned long)length);
            break;
    }
}

void USBHandler::processStartFlag(const uint8_t* data, uint32_t length) {
    // Start flag: bytes [23, 46, 158, 237]
    const uint8_t START_FLAG[] = {23, 46, 158, 237};
    
    // Guard: need at least 4 bytes to contain a start flag.
    // Without this, length - 4 wraps to ~4 billion (uint32_t unsigned underflow)
    // and the loop reads far past the buffer boundary.
    if (length < 4) return;
    
    // Check if start flag is in the received data
    for (uint32_t i = 0; i <= length - 4; i++) {
        if (memcmp(data + i, START_FLAG, 4) == 0) {
            start_flag_received = true;
            current_state = USBState::RECEIVING_SETTINGS;
            buffer_index = 0;  // Reset buffer for settings data
            DIAG("USB", "START FLAG found at offset %lu, state -> RECEIVING_SETTINGS", (unsigned long)i);
            
            // If there's more data after the start flag, process it
            if (length > i + 4) {
                DIAG("USB", "  %lu trailing bytes after start flag, forwarding to settings parser", (unsigned long)(length - i - 4));
                processSettingsData(data + i + 4, length - i - 4);
            }
            return;
        }
    }
    DIAG("USB", "  no start flag in %lu bytes", (unsigned long)length);
}

void USBHandler::processSettingsData(const uint8_t* data, uint32_t length) {
    // Add data to buffer
    uint32_t bytes_to_copy = (length < (MAX_BUFFER_SIZE - buffer_index)) ? 
                             length : (MAX_BUFFER_SIZE - buffer_index);
    
    memcpy(usb_buffer + buffer_index, data, bytes_to_copy);
    buffer_index += bytes_to_copy;
    DIAG("USB", "  settings buffer: +%lu bytes, total=%lu/%u", (unsigned long)bytes_to_copy, (unsigned long)buffer_index, MAX_BUFFER_SIZE);
    
    // Check if we have a complete settings packet (contains "SET" and "END")
    if (buffer_index >= 74) {  // Minimum size for valid settings packet
        // Look for "SET" at beginning and "END" somewhere in the packet
        bool has_set = (memcmp(usb_buffer, "SET", 3) == 0);
        bool has_end = false;
        
        DIAG_BOOL("USB", "  packet starts with SET", has_set);
        
        for (uint32_t i = 3; i <= buffer_index - 3; i++) {
            if (memcmp(usb_buffer + i, "END", 3) == 0) {
                has_end = true;
                DIAG("USB", "  END marker found at offset %lu, packet_len=%lu", (unsigned long)i, (unsigned long)(i + 3));
                
                // Parse the complete packet up to "END"
                if (has_set && current_settings.parseFromUSB(usb_buffer, i + 3)) {
                    current_state = USBState::READY_FOR_DATA;
                    DIAG("USB", "  Settings parsed OK, state -> READY_FOR_DATA");
                } else {
                    DIAG_ERR("USB", "  Settings parse FAILED (has_set=%d)", has_set);
                }
                break;
            }
        }
        
        // If we didn't find a valid packet but buffer is full, reset
        if (buffer_index >= MAX_BUFFER_SIZE && !has_end) {
            DIAG_WARN("USB", "  Buffer full (%u) without END marker -- resetting", MAX_BUFFER_SIZE);
            buffer_index = 0;  // Reset buffer to avoid overflow
        }
    }
}