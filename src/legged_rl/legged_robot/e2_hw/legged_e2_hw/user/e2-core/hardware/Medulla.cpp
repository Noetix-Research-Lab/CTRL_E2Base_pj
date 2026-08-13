//
// Created by han on 9/20/23.
//

#include "Medulla.h"
#include "string.h"


Medulla::Medulla(const std::string& name, const uint32_t address):
        EthercatSlaveBase(name, address){}

void Medulla::updateRead()
{
    // Copy only the known wire prefix.  A newer Medulla firmware may append
    // fields to its PDO; those bytes are deliberately left uninterpreted.
    memcpy(&medulla_data_, ec_slave[address_].inputs, kMedullaInputPdoBytes);
}

void Medulla::updateWrite()
{
    const size_t mapped_output_bytes = static_cast<size_t>(ec_slave[address_].Obytes) +
        (ec_slave[address_].Obits > 0 ? 1U : 0U);
    if (mapped_output_bytes > kMedullaOutputPdoBytes) {
        // The host does not assign semantics to firmware extension bytes.  A
        // deterministic zero is safer than retaining stale IO-map contents.
        memset(ec_slave[address_].outputs + kMedullaOutputPdoBytes, 0,
               mapped_output_bytes - kMedullaOutputPdoBytes);
    }
    memcpy(ec_slave[address_].outputs, &medulla_cmd_, kMedullaOutputPdoBytes);
}
