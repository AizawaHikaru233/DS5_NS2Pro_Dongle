#include "ns2pro_haptics_bridge.h"

#include "ns2pro_rumble.h"
#include "ns2pro_serial_bridge.h"

void ns2pro_haptics_bridge_on_audio(const int8_t *samples, size_t len) {
    ns2pro_rumble_on_haptics_audio(samples, len);

    uint8_t rumble_report[64];
    if (!ns2pro_rumble_build_output_report(rumble_report)) {
        return;
    }

    const bool queued = ns2pro_serial_bridge_send_output_report(rumble_report, sizeof(rumble_report));
    ns2pro_rumble_note_output_attempt(queued, queued ? 3 : ns2pro_serial_bridge_last_output_status());
    if (queued) {
        ns2pro_rumble_note_output_sent(true);
    }
}
