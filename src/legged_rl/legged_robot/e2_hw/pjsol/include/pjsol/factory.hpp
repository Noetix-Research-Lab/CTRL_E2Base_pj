#pragma once

#include "export.hpp"
#include "mechanism.hpp"
#include "mech_type.hpp"
#include <memory>
#include <string>

namespace pjsol {

PJSOL_API std::unique_ptr<IMechanism> create(const std::string& type,
                                             const Geom& geom,
                                             const JointLimit& limit = {});

PJSOL_API std::unique_ptr<IMechanism> create(MechType type,
                                             const Geom& geom,
                                             const JointLimit& limit = {});

} // namespace pjsol
