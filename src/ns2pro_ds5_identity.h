#ifndef DS5_BRIDGE_NS2PRO_DS5_IDENTITY_H
#define DS5_BRIDGE_NS2PRO_DS5_IDENTITY_H

#include <cstdint>
#include <vector>

enum class Ns2ProIdentityProfile : uint8_t {
    Wired = 0,
    Ble = 1,
};

std::vector<uint8_t> ns2pro_build_local_feature_report(
    uint8_t report_id,
    bool ns2pro_tuned,
    Ns2ProIdentityProfile profile
);

#endif
