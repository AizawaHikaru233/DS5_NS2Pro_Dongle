#ifndef DS5_BRIDGE_NS2PRO_HAPTICS_BRIDGE_H
#define DS5_BRIDGE_NS2PRO_HAPTICS_BRIDGE_H

#include <cstddef>
#include <cstdint>

void ns2pro_haptics_bridge_on_audio(const int8_t *samples, size_t len);

#endif
