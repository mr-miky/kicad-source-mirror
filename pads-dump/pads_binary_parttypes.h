// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <filesystem>
#include <iosfwd>

namespace pads_dump {
// Serialize the binary PARTTYPE -> ordered decal-index/name relationship directly
// from the SDB controller tables.  This is intentionally independent from
// BINARY_PARSER::linkPartsToDecals(): old databases may resolve each placed part
// through a direct selected-decal index while still retaining the complete
// primary+alternate list in the PARTTYPE table.
void write_parttype_decals_json(std::ostream& out, const std::filesystem::path& pcb);
}
