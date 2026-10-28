// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <filesystem>
#include <iosfwd>
#include <string>

namespace pads_dump {
// Serialize an independent, validated PADS BASIC binary owner -> piece -> vertex
// association as a JSON object keyed by decal name. No ASCII input is used.
// Throws std::runtime_error if a unique consistent pool cannot be located.
void write_decal_graphics_json(std::ostream& out, const std::filesystem::path& pcb);
}
