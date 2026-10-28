// SPDX-License-Identifier: GPL-3.0-or-later
#include "pads_binary_graphics.h"
#include "pads_sdb.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <ostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace pads_dump {
namespace {

constexpr size_t VERTEX_SIZE = 12;
constexpr int32_t ANGLE_SCALE = 1800000;

struct Layout
{
    uint16_t version = 0;
    size_t ownerStride = 0;
    size_t pieceStride = 0;
    size_t ownerNameOffset = 0;
};

struct Owner
{
    std::string name;
    uint32_t ownerIndex = 0;
    int32_t pieceStart = 0;
    int32_t vertexStart = 0;
    int32_t arcStart = 0;
    int32_t pieceCount = 0;
    uint32_t itemKind = 0;
};

struct Vertex
{
    int32_t x = 0;
    int32_t y = 0;
    int32_t attr = -1;
};

struct ArcInfo
{
    int32_t xmin = 0;
    int32_t ymin = 0;
    int32_t xmax = 0;
    int32_t ymax = 0;
    double cx = 0.0;
    double cy = 0.0;
    double radius = 0.0;
    double startAngle = 0.0;
    double deltaAngle = 0.0;
    uint32_t recordIndex = 0;
    uint32_t recordOffset = 0;
};

struct Label
{
    std::string type;
    uint32_t recordIndex = 0;
    int32_t x = 0;
    int32_t y = 0;
    int32_t height = 0;
    int32_t width = 0;
    double rotation = 0.0;
    int layer = 0;
    bool visible = false;
    bool mirrored = false;
    std::string hjust;
    std::string vjust;
    uint8_t rightReading = 0;
};

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

Layout layoutFor(uint16_t version)
{
    return {
        version,
        version <= 0x2022 ? size_t(100) : size_t(112),
        version <= 0x2024 ? size_t(16) : size_t(20),
        version <= 0x2022 ? size_t(32) : size_t(44)
    };
}

uint8_t ringU8(const PADS_IO::PADS_SDB& sdb, const PADS_IO::SDB_SECTION& section,
               size_t rotation, size_t stride, uint32_t index, size_t field)
{
    if(section.totalBytes == 0 || index >= section.count || field >= stride)
        throw std::runtime_error("Accesso record circolare PADS non valido");

    const size_t logical = static_cast<size_t>(index) * stride + field;
    const size_t physical = (rotation + logical) % section.totalBytes;
    return sdb.RecordAt(section.physicalOffset + static_cast<uint32_t>(physical)).U8(0);
}

uint32_t ringU32(const PADS_IO::PADS_SDB& sdb, const PADS_IO::SDB_SECTION& section,
                 size_t rotation, size_t stride, uint32_t index, size_t field)
{
    uint32_t value = 0;
    for(size_t byte = 0; byte < 4; ++byte)
        value |= static_cast<uint32_t>(ringU8(sdb, section, rotation, stride, index, field + byte)) << (byte * 8);
    return value;
}

int32_t ringI32(const PADS_IO::PADS_SDB& sdb, const PADS_IO::SDB_SECTION& section,
                size_t rotation, size_t stride, uint32_t index, size_t field)
{
    return static_cast<int32_t>(ringU32(sdb, section, rotation, stride, index, field));
}

std::string ringStr(const PADS_IO::PADS_SDB& sdb, const PADS_IO::SDB_SECTION& section,
                    size_t rotation, size_t stride, uint32_t index, size_t field, size_t length)
{
    std::string value;
    value.reserve(length);
    for(size_t byte = 0; byte < length; ++byte)
    {
        const char c = static_cast<char>(ringU8(sdb, section, rotation, stride, index, field + byte));
        if(c == '\0') break;
        value.push_back(c);
    }
    return value;
}

size_t ringPhysicalOffset(const PADS_IO::SDB_SECTION& section, size_t rotation, size_t stride,
                          uint32_t index, size_t field)
{
    const size_t logical = static_cast<size_t>(index) * stride + field;
    return section.physicalOffset + (rotation + logical) % section.totalBytes;
}

const char* typeName(uint32_t n)
{
    switch(n)
    {
    case 0: return "OPEN";
    case 1: return "CLOSED";
    case 2: return "CIRCLE";
    case 4: return "TYPE4";
    case 5: return "COPCUT";
    case 10: return "KPTCLS";
    case 11: return "KPTCIR";
    default: return "UNKNOWN";
    }
}

void writeRestrictionList(std::ostream& os, uint32_t mask)
{
    struct Flag { uint32_t bit; const char* name; };
    static constexpr Flag flags[] = {
        {0x01, "placement"}, {0x02, "trace"}, {0x04, "copper"},
        {0x08, "via"}, {0x10, "testpoint"}, {0x40, "accordion"},
    };

    os << '[';
    bool first = true;
    for(const auto& flag : flags)
    {
        if((mask & flag.bit) == 0) continue;
        if(!first) os << ',';
        first = false;
        quote(os, flag.name);
    }
    os << ']';
}

constexpr uint32_t KNOWN_RESTRICTION_MASK = 0x01U | 0x02U | 0x04U | 0x08U | 0x10U | 0x40U;

std::vector<std::string> readDecalNames(const PADS_IO::PADS_SDB& sdb)
{
    const PADS_IO::SDB_SECTION* sec14 = sdb.Section(14);
    if(!sec14 || sec14->count == 0 || sec14->stride == 0)
        throw std::runtime_error("Controller decal SDB mancante");

    std::vector<std::string> names;
    names.reserve(sec14->count);
    for(uint32_t i = 0; i < sec14->count; ++i)
        names.push_back(sdb.RecordAt(sec14->physicalOffset + i * sec14->stride).Str(0, 41));
    return names;
}

std::optional<uint32_t> resolveDecalLabelOwner(const PADS_IO::PADS_SDB& sdb,
                                               const PADS_IO::SDB_SECTION& text,
                                               size_t recordBase, size_t recordSize,
                                               uint32_t startIndex)
{
    std::set<uint32_t> visited;
    uint32_t index = startIndex;

    while(index < text.physicalCount && visited.insert(index).second)
    {
        const size_t metaOff = recordBase + static_cast<size_t>(index + 1) * recordSize;
        PADS_IO::SDB_RECORD meta = sdb.RecordAt(static_cast<uint32_t>(metaOff));
        const uint32_t association = meta.U32(12);
        const uint8_t kind = static_cast<uint8_t>(association >> 24);
        const uint32_t target = association & 0x00FFFFFFU;

        // 0x0E terminates a decal field chain; low 24 bits are the DecalHeader index.
        if(kind == 0x0E)
            return target;

        // 0x08 links to another field-presentation record in section 8.
        if(kind != 0x08 || target >= text.physicalCount)
            return std::nullopt;

        index = target;
    }

    return std::nullopt;
}

std::map<std::string, std::vector<Label>> readDecalLabels(const PADS_IO::PADS_SDB& sdb,
                                                          const std::vector<std::string>& decalNames)
{
    std::map<std::string, std::vector<Label>> result;
    const PADS_IO::SDB_SECTION* s8 = sdb.Section(8);
    const PADS_IO::SDB_SECTION* s9 = sdb.Section(9);
    if(!s8 || !s9 || s8->physicalCount == 0)
        return result;

    constexpr size_t GEOMETRY_BYTES = 36;
    const size_t recordSize = s8->stride;
    if(recordSize != 64 && recordSize != 72)
        return result;

    const size_t ringRotation = recordSize - GEOMETRY_BYTES;
    if(s8->physicalOffset < ringRotation)
        return result;

    const size_t recordBase = s8->physicalOffset - ringRotation;

    for(uint32_t index = 0; index < s8->physicalCount; ++index)
    {
        const size_t geomOff = recordBase + static_cast<size_t>(index) * recordSize;
        const size_t metaOff = recordBase + static_cast<size_t>(index + 1) * recordSize;
        PADS_IO::SDB_RECORD geom = sdb.RecordAt(static_cast<uint32_t>(geomOff));
        PADS_IO::SDB_RECORD meta = sdb.RecordAt(static_cast<uint32_t>(metaOff));

        if(meta.U16(4) != 0xFFFE)
            continue;

        const uint32_t presentation = meta.U32(24);
        const uint8_t fieldKind = static_cast<uint8_t>(presentation >> 16);
        const uint8_t labelType = fieldKind & 0x1FU;
        if(labelType != 0x01 && labelType != 0x02 && labelType != 0x03)
            continue;

        const std::optional<uint32_t> owner = resolveDecalLabelOwner(sdb, *s8, recordBase, recordSize, index);
        if(!owner || *owner >= decalNames.size() || decalNames[*owner].empty())
            continue;

        Label label;
        label.recordIndex = index;
        label.type = labelType == 0x02 ? "REF_DES" : (labelType == 0x03 ? "PART_TYPE" : "ATTRIBUTE");
        label.height = geom.I32(ringRotation);
        label.width = geom.I32(ringRotation + 4);
        label.x = geom.I32(ringRotation + 8);
        label.y = geom.I32(ringRotation + 12);
        label.rotation = static_cast<double>(geom.I32(ringRotation + 16)) / static_cast<double>(ANGLE_SCALE);
        label.mirrored = geom.I32(ringRotation + 20) != 0;
        label.visible = (fieldKind & 0x20U) != 0;
        label.layer = static_cast<int16_t>(presentation & 0xFFFFU);
        label.rightReading = fieldKind & 0x18U;

        if((presentation & 0x11000000U) == 0x11000000U)
        {
            label.hjust = "CENTER";
            label.vjust = "CENTER";
        }
        else
        {
            label.hjust = "LEFT";
            label.vjust = (presentation & 0x20000000U) ? "UP" : "DOWN";
        }

        result[decalNames[*owner]].push_back(std::move(label));
    }

    return result;
}

std::vector<Owner> readOwners(const PADS_IO::PADS_SDB& sdb, const Layout& layout)
{
    const PADS_IO::SDB_SECTION* sec10 = sdb.Section(10);
    if(!sec10)
        throw std::runtime_error("Controller drawing SDB mancante");
    if(sec10->totalBytes != static_cast<uint64_t>(sec10->count) * layout.ownerStride)
        throw std::runtime_error("Stride owner decal SDB non valido");

    std::vector<Owner> owners;
    for(uint32_t ownerIndex = 0; ownerIndex < sec10->count; ++ownerIndex)
    {
        std::string name = ringStr(sdb, *sec10, 68, layout.ownerStride, ownerIndex,
                                   layout.ownerNameOffset, layout.ownerStride - layout.ownerNameOffset);
        if(name.empty()) continue;

        const uint32_t lagIndex = (ownerIndex + 1) % sec10->count;
        Owner owner;
        owner.name = std::move(name);
        owner.ownerIndex = ownerIndex;
        owner.pieceStart = ringI32(sdb, *sec10, 68, layout.ownerStride, lagIndex, 8);
        owner.vertexStart = ringI32(sdb, *sec10, 68, layout.ownerStride, lagIndex, 12);
        owner.arcStart = ringI32(sdb, *sec10, 68, layout.ownerStride, lagIndex, 16);
        owner.pieceCount = ringI32(sdb, *sec10, 68, layout.ownerStride, lagIndex, 24);
        owner.itemKind = ringU32(sdb, *sec10, 68, layout.ownerStride, lagIndex, 28);

        // Low 16 bits == 2 is PARTDECAL.  This excludes board DRW owners and VIA styles.
        if((owner.itemKind & 0xFFFFU) == 2 && owner.pieceCount > 0)
            owners.push_back(std::move(owner));
    }
    return owners;
}

ArcInfo deriveArc(const PADS_IO::PADS_SDB& sdb, const PADS_IO::SDB_SECTION& arcs,
                  int32_t arcStart, int32_t attr, const Vertex& /*start*/, const Vertex& /*end*/)
{
    const int64_t arcIndex64 = static_cast<int64_t>(arcStart) + static_cast<int64_t>(attr);
    if(arcStart < 0 || attr < 0 || arcIndex64 < 0 || static_cast<uint64_t>(arcIndex64) >= arcs.count
       || arcs.totalBytes != static_cast<uint64_t>(arcs.count) * 20)
        throw std::runtime_error("Indice arco decal PADS fuori range");

    const uint32_t arcIndex = static_cast<uint32_t>(arcIndex64);
    const uint32_t off = arcs.physicalOffset + arcIndex * 20;
    PADS_IO::SDB_RECORD rec = sdb.RecordAt(off);

    ArcInfo arc;
    arc.xmin = rec.I32(0);
    arc.ymin = rec.I32(4);
    arc.xmax = rec.I32(8);
    arc.ymax = rec.I32(12);
    arc.cx = (static_cast<double>(arc.xmin) + arc.xmax) / 2.0;
    arc.cy = (static_cast<double>(arc.ymin) + arc.ymax) / 2.0;
    arc.radius = (static_cast<double>(arc.xmax) - arc.xmin) / 2.0;

    // Section-13 +16 is not an opaque flag: it packs the exact ASCII arc angles
    // as two signed i16 values in tenths of a degree.  Low 16 bits = begin angle,
    // high 16 bits = sweep.  This matters for major arcs: CON_BNC carries 1186/3027
    // (118.6 deg, +302.7 deg), which cannot be recovered by normalizing endpoint
    // angles to +/-180 degrees.
    const uint32_t packedAngles = rec.U32(16);
    const int16_t startTenths = static_cast<int16_t>(packedAngles & 0xFFFFU);
    const int16_t deltaTenths = static_cast<int16_t>((packedAngles >> 16) & 0xFFFFU);
    arc.startAngle = static_cast<double>(startTenths) / 10.0;
    arc.deltaAngle = static_cast<double>(deltaTenths) / 10.0;
    arc.recordIndex = arcIndex;
    arc.recordOffset = off;
    return arc;
}

void writeArc(std::ostream& os, const ArcInfo& arc)
{
    os << "{\"cx\":" << arc.cx << ",\"cy\":" << arc.cy
       << ",\"radius\":" << arc.radius << ",\"start_angle\":" << arc.startAngle
       << ",\"delta_angle\":" << arc.deltaAngle << ",\"bbox\":["
       << arc.xmin << ',' << arc.ymin << ',' << arc.xmax << ',' << arc.ymax << ']'
       << ",\"record_index\":" << arc.recordIndex << ",\"record_offset\":" << arc.recordOffset << '}';
}

void writeLabel(std::ostream& os, const Label& label)
{
    os << "{\"type\":";
    quote(os, label.type);
    os << ",\"record_index\":" << label.recordIndex << ",\"x\":" << label.x << ",\"y\":" << label.y
       << ",\"height\":" << label.height << ",\"width\":" << label.width
       << ",\"rotation\":" << label.rotation << ",\"layer\":" << label.layer
       << ",\"visible\":" << (label.visible ? "true" : "false")
       << ",\"mirrored\":" << (label.mirrored ? "true" : "false")
       << ",\"hjust\":"; quote(os, label.hjust);
    os << ",\"vjust\":"; quote(os, label.vjust);
    os << ",\"right_reading\":" << static_cast<unsigned>(label.rightReading) << '}';
}

} // namespace

void write_decal_graphics_json(std::ostream& os, const std::filesystem::path& pcb)
{
    std::ifstream input(pcb, std::ios::binary);
    if(!input) throw std::runtime_error("Impossibile aprire PCB per geometria");

    std::vector<uint8_t> bytes(std::istreambuf_iterator<char>{input}, {});
    PADS_IO::PADS_SDB sdb;
    sdb.Load(std::move(bytes));

    const Layout layout = layoutFor(sdb.Version());
    const PADS_IO::SDB_SECTION* pieces = sdb.Section(11);
    const PADS_IO::SDB_SECTION* vertices = sdb.Section(12);
    const PADS_IO::SDB_SECTION* arcs = sdb.Section(13);
    if(!pieces || !vertices || !arcs)
        throw std::runtime_error("Controller graphic/vertex/arc SDB mancante");
    if(pieces->totalBytes != static_cast<uint64_t>(pieces->count) * layout.pieceStride
       || vertices->totalBytes != static_cast<uint64_t>(vertices->count) * VERTEX_SIZE)
        throw std::runtime_error("Framing graphic/vertex SDB non valido");

    const std::vector<std::string> decalNames = readDecalNames(sdb);
    const std::map<std::string, std::vector<Label>> labels = readDecalLabels(sdb, decalNames);
    const std::vector<Owner> owners = readOwners(sdb, layout);
    if(owners.empty())
        throw std::runtime_error("Nessun owner PARTDECAL nel PCB");

    const size_t pieceHead = layout.pieceStride - 8;
    const size_t pieceRotation = pieces->totalBytes - pieceHead;
    std::set<std::string> seenNames;

    os << '{';
    bool firstOwner = true;
    for(const Owner& owner : owners)
    {
        if(!seenNames.insert(owner.name).second)
            throw std::runtime_error("Nome decal duplicato: " + owner.name);
        if(owner.pieceStart < 0 || owner.vertexStart < 0 || owner.pieceCount <= 0
           || static_cast<uint64_t>(owner.pieceStart) + static_cast<uint64_t>(owner.pieceCount) > pieces->count)
            throw std::runtime_error("Range GraphicPiece decal non valido: " + owner.name);

        if(!firstOwner) os << ',';
        firstOwner = false;
        quote(os, owner.name);
        os << ":{\"sdb_version\":" << layout.version
           << ",\"owner_stride\":" << layout.ownerStride
           << ",\"piece_stride\":" << layout.pieceStride
           << ",\"owner_index\":" << owner.ownerIndex
           << ",\"piece_pool_offset\":" << pieces->physicalOffset
           << ",\"vertex_pool_offset\":" << vertices->physicalOffset
           << ",\"arc_pool_offset\":" << arcs->physicalOffset
           << ",\"piece_start\":" << owner.pieceStart
           << ",\"vertex_start\":" << owner.vertexStart
           << ",\"arc_start\":" << owner.arcStart;

        os << ",\"labels\":[";
        auto labelIt = labels.find(owner.name);
        if(labelIt != labels.end())
        {
            for(size_t li = 0; li < labelIt->second.size(); ++li)
            {
                if(li) os << ',';
                writeLabel(os, labelIt->second[li]);
            }
        }
        os << "]";

        os << ",\"items\":[";
        int32_t vertexCursor = owner.vertexStart;

        for(int32_t pieceNo = 0; pieceNo < owner.pieceCount; ++pieceNo)
        {
            if(pieceNo) os << ',';
            const uint32_t pieceIndex = static_cast<uint32_t>(owner.pieceStart + pieceNo);
            const uint32_t nextIndex = (pieceIndex + 1) % pieces->count;
            const int32_t width = ringI32(sdb, *pieces, pieceRotation, layout.pieceStride,
                                          pieceIndex, layout.pieceStride - 8);
            const int32_t cornerCount = ringI32(sdb, *pieces, pieceRotation, layout.pieceStride,
                                                pieceIndex, layout.pieceStride - 4);
            const uint32_t kind = ringU8(sdb, *pieces, pieceRotation, layout.pieceStride, nextIndex, 0);
            int layer = static_cast<int>(ringU8(sdb, *pieces, pieceRotation, layout.pieceStride, nextIndex, 1));
            if(layer >= 128) layer -= 256;
            const uint32_t extra = ringU32(sdb, *pieces, pieceRotation, layout.pieceStride, nextIndex, 4);
            const uint32_t style = layout.pieceStride == 20
                                           ? ringU32(sdb, *pieces, pieceRotation, layout.pieceStride, nextIndex, 8)
                                           : 0;

            if(cornerCount <= 0 || static_cast<uint64_t>(vertexCursor) + static_cast<uint64_t>(cornerCount) > vertices->count)
                throw std::runtime_error("Range vertici decal non valido: " + owner.name);
            if(kind != 0 && kind != 1 && kind != 2 && kind != 4 && kind != 5 && kind != 10 && kind != 11)
                throw std::runtime_error("GraphicPiece decal sconosciuto: " + owner.name);

            std::vector<Vertex> decoded;
            decoded.reserve(static_cast<size_t>(cornerCount));
            for(int32_t k = 0; k < cornerCount; ++k)
            {
                const uint32_t off = vertices->physicalOffset + static_cast<uint32_t>(vertexCursor + k) * VERTEX_SIZE;
                PADS_IO::SDB_RECORD rec = sdb.RecordAt(off);
                decoded.push_back({rec.I32(0), rec.I32(4), rec.I32(8)});
            }

            os << "{\"type\":"; quote(os, typeName(kind));
            os << ",\"type_id\":" << kind << ",\"layer\":" << layer
               << ",\"width\":" << width << ",\"vertex_count\":" << cornerCount
               << ",\"extra\":" << extra << ",\"line_style\":" << style
               << ",\"record_offset\":"
               << ringPhysicalOffset(*pieces, pieceRotation, layout.pieceStride, pieceIndex, layout.pieceStride - 8)
               << ",\"vertices\":[";

            for(size_t k = 0; k < decoded.size(); ++k)
            {
                if(k) os << ',';
                os << "{\"x\":" << decoded[k].x << ",\"y\":" << decoded[k].y << ",\"extra\":" << decoded[k].attr;
                if(k > 0 && decoded[k - 1].attr >= 0)
                {
                    const ArcInfo arc = deriveArc(sdb, *arcs, owner.arcStart, decoded[k - 1].attr,
                                                  decoded[k - 1], decoded[k]);
                    os << ",\"arc_from_previous\":";
                    writeArc(os, arc);
                }
                os << '}';
            }
            os << ']';

            const bool closedWithoutRepeatedPoint = kind == 1 && decoded.size() >= 3
                    && (decoded.front().x != decoded.back().x || decoded.front().y != decoded.back().y);
            if(closedWithoutRepeatedPoint && decoded.back().attr >= 0)
            {
                const ArcInfo closingArc = deriveArc(sdb, *arcs, owner.arcStart, decoded.back().attr,
                                                     decoded.back(), decoded.front());
                os << ",\"closing_arc\":";
                writeArc(os, closingArc);
            }

            // KPTCIR/KPTCLS are a KEEP_OUT parent plus one geometry child.  Preserve the
            // serialized flat fields too: they are useful when comparing other PADS dialects.
            if(kind == 10 || kind == 11)
            {
                os << ",\"parent\":{\"type\":\"KEEP_OUT\",\"restriction_mask\":" << extra
                   << ",\"restrictions\":";
                writeRestrictionList(os, extra);
                os << ",\"unknown_restriction_bits\":" << (extra & ~KNOWN_RESTRICTION_MASK)
                   << ",\"side\":" << layer << '}';
                os << ",\"children\":[{\"type\":";
                quote(os, kind == 11 ? "CIRCLE" : "CLOSED");
                os << ",\"source_type\":"; quote(os, typeName(kind));
                if(kind == 10) os << ",\"line_count\":" << (cornerCount > 0 ? cornerCount - 1 : 0);
                os << ",\"vertices\":[";
                for(size_t k = 0; k < decoded.size(); ++k)
                {
                    if(k) os << ',';
                    os << "{\"x\":" << decoded[k].x << ",\"y\":" << decoded[k].y
                       << ",\"extra\":" << decoded[k].attr;
                    if(k > 0 && decoded[k - 1].attr >= 0)
                    {
                        const ArcInfo arc = deriveArc(sdb, *arcs, owner.arcStart, decoded[k - 1].attr,
                                                      decoded[k - 1], decoded[k]);
                        os << ",\"arc_from_previous\":";
                        writeArc(os, arc);
                    }
                    os << '}';
                }
                os << "]}]";
            }

            vertexCursor += cornerCount;
            os << '}';
        }

        os << "]}";
    }
    os << '}';
}

} // namespace pads_dump
