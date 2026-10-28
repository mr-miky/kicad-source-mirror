// SPDX-License-Identifier: GPL-3.0-or-later
#include "pads_binary_parttypes.h"
#include "pads_sdb.h"

#include <cstdint>
#include <fstream>
#include <iterator>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace pads_dump {
namespace {

void quote(std::ostream& os, const std::string& s)
{
    os << '"';
    const char* hex = "0123456789abcdef";
    for(unsigned char c : s)
    {
        if(c == '"' || c == '\\') os << '\\' << char(c);
        else if(c < 32) os << "\\u00" << hex[c >> 4] << hex[c & 15];
        else os << char(c);
    }
    os << '"';
}

struct DecalRef
{
    int32_t index = -1;
    std::string name;
};

struct PartTypeDecals
{
    std::string name;
    std::vector<DecalRef> decals;
};

} // namespace

void write_parttype_decals_json(std::ostream& os, const std::filesystem::path& pcb)
{
    std::ifstream input(pcb, std::ios::binary);
    if(!input)
        throw std::runtime_error("Impossibile aprire PCB per PARTTYPE");

    std::vector<uint8_t> bytes(std::istreambuf_iterator<char>{input}, {});
    PADS_IO::PADS_SDB sdb;
    sdb.Load(std::move(bytes));

    const PADS_IO::SDB_SECTION* sec14 = sdb.Section(14);
    const PADS_IO::SDB_SECTION* sec17 = sdb.Section(17);

    if(!sec14 || sec14->count == 0 || sec14->stride == 0)
        throw std::runtime_error("Controller decal SDB mancante");

    const size_t decalStride = sec14->stride;
    std::vector<std::string> decalNames;
    std::vector<std::string> viaStyles;
    decalNames.reserve(sec14->count);

    for(uint32_t i = 0; i < sec14->count; ++i)
    {
        const uint32_t off = sec14->physicalOffset + i * decalStride;
        PADS_IO::SDB_RECORD rec = sdb.RecordAt(off);
        std::string name = rec.Str(0, 41);
        decalNames.push_back(name);

        // DecalHeader +0x4C is the PARTDECAL descriptor reference.  The built-in
        // and custom VIA styles carry zero here instead.  Verified on the 0x2017
        // sample (JMPVIA_AAAAA/STANDARDVIA) and the 0x2026 sample, where MICROVIA
        // also has zero while ordinary decals have non-zero descriptor references.
        if(!name.empty() && decalStride >= 0x50 && rec.I32(0x4C) == 0)
            viaStyles.push_back(name);
    }

    os << "{\"sdb_version\":" << sdb.Version() << ",\"via_styles\":[";
    for(size_t i = 0; i < viaStyles.size(); ++i)
    {
        if(i) os << ',';
        quote(os, viaStyles[i]);
    }
    os << "],\"parttypes\":[";

    if(!sec17 || sec17->count == 0)
    {
        os << "]}";
        return;
    }

    if(sec17->totalBytes % sec17->count != 0)
        throw std::runtime_error("Stride PARTTYPE SDB non intero");

    const uint32_t recSize = sec17->totalBytes / sec17->count;
    if(recSize != 208 && recSize != 224)
        throw std::runtime_error("Stride PARTTYPE SDB non supportato: " + std::to_string(recSize));

    // The logical PARTTYPE ring begins 44 bytes before section 17's physical cursor.
    // Both observed 208- and 224-byte layouts carry the primary+alternate decal list
    // as duplicated index pairs at +0x60: i,i  j,j ... -1,-1.  30439_10_A0
    // (SDB 0x2017) verifies MOUNT_HOLE as 2,2 3,3 4,4 5,5, matching the ASCII
    // VITE_3MM:HOLE_3MM:HOLE_10MM:HOLE_7MM declaration exactly.
    static constexpr uint32_t LOGICAL_LEAD = 44;
    static constexpr uint32_t NAME_OFF = 44;
    static constexpr uint32_t DECAL_LIST_OFF = 0x60;

    if(sec17->physicalOffset < LOGICAL_LEAD)
        throw std::runtime_error("Framing PARTTYPE SDB non valido");

    const uint32_t start = sec17->physicalOffset - LOGICAL_LEAD;
    bool firstPartType = true;

    for(uint32_t i = 0; i < sec17->count; ++i)
    {
        const uint32_t off = start + i * recSize;
        PADS_IO::SDB_RECORD rec = sdb.RecordAt(off);
        PartTypeDecals pt;
        pt.name = rec.Str(NAME_OFF, 36);

        for(uint32_t pos = DECAL_LIST_OFF; pos + 8 <= recSize; pos += 8)
        {
            const int32_t a = rec.I32(pos);
            const int32_t b = rec.I32(pos + 4);
            if(a < 0 || a != b)
                break;
            if(static_cast<size_t>(a) >= decalNames.size())
                throw std::runtime_error("Indice decal PARTTYPE fuori range");
            pt.decals.push_back({a, decalNames[static_cast<size_t>(a)]});
        }

        // Compatibility fallback for a legacy layout that exposes only the historical
        // single index at +0x70.  Do not use it when the duplicated list was present.
        if(pt.decals.empty() && recSize == 208 && 0x70 + 4 <= recSize)
        {
            const int32_t a = rec.I32(0x70);
            if(a >= 0 && static_cast<size_t>(a) < decalNames.size())
                pt.decals.push_back({a, decalNames[static_cast<size_t>(a)]});
        }

        if(pt.name.empty() || pt.decals.empty())
            continue;

        if(!firstPartType) os << ',';
        firstPartType = false;
        os << "{\"name\":";
        quote(os, pt.name);
        os << ",\"decals\":[";
        for(size_t j = 0; j < pt.decals.size(); ++j)
        {
            if(j) os << ',';
            os << "{\"index\":" << pt.decals[j].index << ",\"name\":";
            quote(os, pt.decals[j].name);
            os << '}';
        }
        os << "]}";
    }

    os << "]}";
}

} // namespace pads_dump
