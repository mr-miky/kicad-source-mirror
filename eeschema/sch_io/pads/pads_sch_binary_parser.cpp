/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "pads_sch_binary_parser.h"

#include "pads_sch_sdb.h"

#include <sch_io/ole_image.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <ranges>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include <ki_exception.h>
#include <wx/strconv.h>

namespace PADS_SCH_BINARY
{

namespace
{
    static constexpr std::array<uint16_t, 16> JUSTIFICATION_BY_NIBBLE_0 = { 0, 4, 1, 5, 8,  12, 9,  13,
                                                                            2, 6, 3, 7, 10, 14, 11, 15 };
    static constexpr std::array<uint16_t, 16> JUSTIFICATION_BY_NIBBLE_90 = { 0, 4, 2, 6, 8, 12, 10, 14,
                                                                             1, 5, 3, 7, 9, 13, 11, 15 };


    uint16_t terminalJustification( uint16_t aNibble, bool aRotated )
    {
        return aRotated ? JUSTIFICATION_BY_NIBBLE_90[aNibble] : JUSTIFICATION_BY_NIBBLE_0[aNibble];
    }


    constexpr size_t   OUTER_DIRECTORY_OFFSET = 0x20;
    constexpr size_t   OUTER_DESCRIPTOR_BYTES = 28;
    constexpr size_t   OUTER_USED_BYTES_OFFSET = 12;
    constexpr size_t   SHEET_HEADER_BYTES = 20;
    constexpr size_t   SHEET_DESCRIPTOR_COUNT = 24;
    constexpr size_t   SHEET_DESCRIPTOR_BYTES = 28;
    constexpr size_t   SHEET_COUNT_OFFSET = 12;
    constexpr size_t   SHEET_USED_BYTES_OFFSET = 16;
    constexpr size_t   SHEET_RECORD_BYTES = 48;
    constexpr size_t   TEXT_RECORD_BYTES = 32;
    constexpr size_t   SYMBOL_RECORD_BYTES = 80;
    constexpr size_t   SYMBOL_PIECE_BYTES = 6;
    constexpr size_t   SYMBOL_VERTEX_BYTES = 6;
    constexpr size_t   SYMBOL_ARC_BYTES = 14;
    constexpr size_t   USED_DECAL_BYTES = 108;
    constexpr size_t   TERMINAL_BYTES = 26;
    constexpr size_t   PART_TYPE_BYTES = 76;
    constexpr size_t   GATE_BYTES = 12;
    constexpr size_t   PIN_BYTES = 24;
    constexpr size_t   SIGNAL_PIN_BYTES = 64;
    constexpr size_t   NET_MEMBERSHIP_BYTES = 2;
    constexpr size_t   NET_RECORD_BYTES = 88;
    constexpr size_t   BUS_RECORD_BYTES = 44;
    constexpr size_t   JUNCTION_RECORD_BYTES = 12;
    constexpr size_t   OFFPAGE_RECORD_BYTES = 32;
    constexpr size_t   CONNECTION_RECORD_BYTES = 40;
    constexpr size_t   CONNECTION_VERTEX_BYTES = 8;
    constexpr size_t   NET_NAME_RECORD_BYTES = 48;
    constexpr size_t   PLACEMENT_GROUP_BYTES = 24;
    constexpr size_t   ATTRIBUTE_OFFSET_BYTES = 4;
    constexpr size_t   FONT_RECORD_BYTES = 36;
    constexpr uint32_t DEFAULT_CODE_PAGE = 1252;

    struct GLOBAL_NET_RECORD
    {
        SOURCE_PROVENANCE          source;
        SOURCE_STRING              name;
        uint32_t                   preservedIdentity = 0;
        uint32_t                   preservedRelationship = 0xFFFFFFFF;
        uint32_t                   membershipStart = 0;
        uint16_t                   membershipCount = 0;
        uint32_t                   aliasStringOffset = 0xFFFFFFFF;
        uint16_t                   aliasCount = 0;
        uint16_t                   kindFlags = 0;
        bool                       tombstone = false;
        std::vector<SOURCE_STRING> aliasMembers;
    };


    struct CONNECTIVITY_GLOBALS
    {
        size_t                         membershipBase = 0;
        uint32_t                       membershipCount = 0;
        size_t                         fontBase = 0;
        uint32_t                       fontCount = 0;
        std::vector<uint16_t>          membershipSheets;
        std::vector<GLOBAL_NET_RECORD> nets;
    };

    struct PLACEMENT_LAYOUT
    {
        uint16_t version;
        bool     decoded;
        size_t   placementBytes;
        size_t   placedPinBytes;
        size_t   fieldBytes;
        size_t   pinStart;
        size_t   componentIdentity;
        size_t   componentGroup;
        size_t   x;
        size_t   y;
        size_t   angle;
        size_t   mirror;
        size_t   partType;
        size_t   decal;
        size_t   gate;
        size_t   pinCount;
        size_t   fieldCount;
        size_t   reference;
        size_t   referenceFont;
        size_t   partTypeFont;
        size_t   referenceField;
        size_t   referenceFieldAngle;
        size_t   partTypeField;
        size_t   partTypeFieldAngle;
        size_t   referenceHeight;
        size_t   partTypeHeight;
        size_t   referenceWidth;
        size_t   partTypeWidth;
        size_t   itemVisibility;
        size_t   placedPinOrdinal;
        size_t   customFont;
        size_t   customX;
        size_t   customAngle;
        size_t   customJustification;
        size_t   customAttributeIndex;
        size_t   customHeight;
        size_t   customWidth;
        size_t   customDisplayFlags;
        size_t   customTail;
    };

    constexpr PLACEMENT_LAYOUT PLACEMENT_LAYOUTS[] = { []
                                                       {
                                                           PLACEMENT_LAYOUT layout{};
                                                           layout.version = 0x000C;
                                                           return layout;
                                                       }(),
                                                       { .version = 0x000D,
                                                         .decoded = true,
                                                         .placementBytes = 136,
                                                         .placedPinBytes = 12,
                                                         .fieldBytes = 24,
                                                         .pinStart = 0x14,
                                                         .componentIdentity = 0x18,
                                                         .componentGroup = 0x1C,
                                                         .x = 0x20,
                                                         .y = 0x22,
                                                         .angle = 0x24,
                                                         .mirror = 0x26,
                                                         .partType = 0x42,
                                                         .decal = 0x44,
                                                         .gate = 0x4A,
                                                         .pinCount = 0x4C,
                                                         .fieldCount = 0x4E,
                                                         .reference = 0x5E,
                                                         .referenceFont = 0,
                                                         .partTypeFont = 2,
                                                         .referenceField = 0x28,
                                                         .referenceFieldAngle = 0x2C,
                                                         .partTypeField = 0x30,
                                                         .partTypeFieldAngle = 0x34,
                                                         .referenceHeight = 0x50,
                                                         .partTypeHeight = 0x52,
                                                         .referenceWidth = 0x58,
                                                         .partTypeWidth = 0x59,
                                                         .itemVisibility = 0x87,
                                                         .placedPinOrdinal = 4,
                                                         .customFont = 0,
                                                         .customX = 8,
                                                         .customAngle = 12,
                                                         .customJustification = 14,
                                                         .customAttributeIndex = 16,
                                                         .customHeight = 18,
                                                         .customWidth = 20,
                                                         .customDisplayFlags = 21,
                                                         .customTail = 22 } };

    const PLACEMENT_LAYOUT& placementLayout( uint16_t aVersion )
    {
        auto layout = std::ranges::find_if( PLACEMENT_LAYOUTS,
                                            [&]( const PLACEMENT_LAYOUT& aLayout )
                                            {
                                                return aLayout.version == aVersion;
                                            } );

        if( layout == std::end( PLACEMENT_LAYOUTS ) )
            THROW_IO_ERROR( wxString::Format( wxS( "unsupported PADS placement layout v0x%04X" ), aVersion ) );

        return *layout;
    }


    size_t outerControllerOffset( const PADS_SCH_SDB& aSdb, size_t aController )
    {
        size_t offset = aSdb.PayloadOffset() + 4;

        for( size_t controller = 1; controller < aController; ++controller )
            offset += aSdb.Pools()[controller].usedBytes;

        return offset;
    }


    SOURCE_PROVENANCE sourceAt( const wxString& aFile, uint16_t aVersion, const wxString& aObjectClass, int aController,
                                size_t aRecord, size_t aOffset, size_t aLength, int aSheet )
    {
        return { aFile, aVersion, aObjectClass, aController, aRecord, aOffset, aLength, aSheet };
    }


    [[noreturn]] void throwDecodeError( const SOURCE_PROVENANCE& aSource, const wxString& aMessage )
    {
        THROW_IO_ERROR( FormatParserError( aSource, aMessage ) );
    }


    int64_t decodeCoordinate( uint16_t aRaw )
    {
        return static_cast<int64_t>( aRaw ) * 4 - 198144;
    }


    int64_t decodeDatabaseCoordinate( int32_t aRaw, const SOURCE_PROVENANCE& aSource )
    {
        if( aRaw < std::numeric_limits<int16_t>::min() || aRaw > std::numeric_limits<int16_t>::max() )
            throwDecodeError( aSource, wxS( "embedded OLE database coordinate is not sign-extended 16-bit" ) );

        return decodeCoordinate( static_cast<uint16_t>( static_cast<int16_t>( aRaw ) ) );
    }


    int64_t decodeLocalCoordinate( uint16_t aRaw )
    {
        return static_cast<int64_t>( static_cast<int16_t>( aRaw ) ) * 4;
    }


    int64_t decodeTerminalCoordinate( uint16_t aRaw )
    {
        return static_cast<int64_t>( static_cast<int16_t>( aRaw ) ) * 4;
    }


    SOURCE_STRING decodeFixedString( const std::vector<uint8_t>& aBytes, size_t aOffset, size_t aBytesAvailable,
                                     const SOURCE_PROVENANCE& aSource, std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        size_t end = aOffset;

        while( end < aOffset + aBytesAvailable && aBytes[end] != 0 )
            ++end;

        if( end == aOffset + aBytesAvailable )
            throwDecodeError( aSource, wxS( "unterminated fixed string" ) );

        return PADS_SCH_BINARY_PARSER::DecodeString( { aBytes.begin() + aOffset, aBytes.begin() + end },
                                                     DEFAULT_CODE_PAGE, aSource, aDiagnostics );
    }


    SOURCE_PROPERTY sourceProperty( const wxString& aName, const wxString& aValue, const SOURCE_PROVENANCE& aSource )
    {
        SOURCE_PROPERTY property;
        property.name.text = aName;
        property.name.source = aSource;
        property.value.text = aValue;
        property.value.source = aSource;
        property.disposition = PROPERTY_DISPOSITION::EXACT;
        property.source = aSource;
        return property;
    }


    bool samePointValue( const SOURCE_POINT& aLeft, const SOURCE_POINT& aRight )
    {
        return aLeft.x == aRight.x && aLeft.y == aRight.y;
    }


    bool samePresentationValue( const MODEL_TEXT_PRESENTATION& aLeft, const MODEL_TEXT_PRESENTATION& aRight )
    {
        return aLeft.height == aRight.height && aLeft.width == aRight.width && aLeft.font.text == aRight.font.text
               && aLeft.horizontalJustification == aRight.horizontalJustification
               && aLeft.verticalJustification == aRight.verticalJustification && aLeft.bold == aRight.bold
               && aLeft.italic == aRight.italic && aLeft.underline == aRight.underline
               && aLeft.visible == aRight.visible;
    }


    bool sameGraphicValue( const MODEL_GRAPHIC& aLeft, const MODEL_GRAPHIC& aRight )
    {
        return aLeft.kind == aRight.kind && aLeft.text.text == aRight.text.text && aLeft.lineStyle == aRight.lineStyle
               && aLeft.strokeWidth == aRight.strokeWidth && aLeft.fill == aRight.fill
               && samePresentationValue( aLeft.presentation, aRight.presentation ) && aLeft.angle == aRight.angle
               && aLeft.arcSweepAngle == aRight.arcSweepAngle && aLeft.arcClockwise == aRight.arcClockwise
               && samePointValue( aLeft.arcCenter, aRight.arcCenter )
               && samePointValue( aLeft.arcBoundsStart, aRight.arcBoundsStart )
               && samePointValue( aLeft.arcBoundsEnd, aRight.arcBoundsEnd )
               && std::ranges::equal( aLeft.points, aRight.points, samePointValue );
    }


    bool sameWorksheetValue( const MODEL_WORKSHEET& aLeft, const MODEL_WORKSHEET& aRight )
    {
        std::vector<const MODEL_GRAPHIC*> leftDrawing;
        std::vector<const MODEL_GRAPHIC*> rightDrawing;
        std::vector<const MODEL_GRAPHIC*> leftText;
        std::vector<const MODEL_GRAPHIC*> rightText;

        for( const MODEL_GRAPHIC& graphic : aLeft.graphics )
            ( graphic.kind == MODEL_GRAPHIC_KIND::TEXT ? leftText : leftDrawing ).push_back( &graphic );

        for( const MODEL_GRAPHIC& graphic : aRight.graphics )
            ( graphic.kind == MODEL_GRAPHIC_KIND::TEXT ? rightText : rightDrawing ).push_back( &graphic );

        const auto equalPointers = []( const MODEL_GRAPHIC* aLeftGraphic, const MODEL_GRAPHIC* aRightGraphic )
        {
            return sameGraphicValue( *aLeftGraphic, *aRightGraphic );
        };

        return std::ranges::equal( leftDrawing, rightDrawing, equalPointers )
               && std::ranges::is_permutation( leftText, rightText, equalPointers );
    }


    bool isProvenGraphicStrokeWidth( uint8_t aWidth )
    {
        constexpr std::array<uint16_t, 13> widths{ 1, 2, 5, 7, 8, 10, 11, 15, 20, 25, 30, 31, 40 };
        return std::ranges::binary_search( widths, aWidth );
    }


    void decodeGraphicStrokeWidth( uint8_t aRawWidth, uint8_t aPackedPresentation,
                                   const SOURCE_PROVENANCE& aGraphicSource, MODEL_GRAPHIC& aGraphic,
                                   std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        SOURCE_PROVENANCE packedSource = aGraphicSource;
        packedSource.absoluteOffset += 5;
        packedSource.length = 1;
        SOURCE_PROPERTY packed = sourceProperty( wxS( "preserved_graphic_presentation" ),
                                                 wxString::Format( wxS( "%u" ), aPackedPresentation ), packedSource );
        packed.disposition = PROPERTY_DISPOSITION::PRESERVED;
        aGraphic.properties.push_back( std::move( packed ) );

        if( isProvenGraphicStrokeWidth( aRawWidth ) )
        {
            aGraphic.strokeWidth = static_cast<int64_t>( aRawWidth ) * 2;
            return;
        }

        SOURCE_PROVENANCE strokeSource = aGraphicSource;
        strokeSource.absoluteOffset += 4;
        strokeSource.length = 1;
        SOURCE_PROPERTY stroke = sourceProperty( wxS( "unsupported_graphic_stroke_width" ),
                                                 wxString::Format( wxS( "%u" ), aRawWidth ), strokeSource );
        stroke.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
        aDiagnostics.push_back( MakePropertyDiagnostic(
                RPT_SEVERITY_WARNING, stroke, wxS( "unproved graphic stroke width preserved; using hairline" ) ) );
        aGraphic.properties.push_back( std::move( stroke ) );
    }


    SOURCE_STRING decodedDefinitionFont( int16_t aHandle, const SOURCE_PROVENANCE& aSource )
    {
        SOURCE_STRING font;
        font.source = aSource;

        if( aHandle == -1 || aHandle == -4 )
        {
            font.text = wxS( "Default Font" );
            font.encoding = STRING_ENCODING_STATUS::CODE_PAGE;
            font.codePage = DEFAULT_CODE_PAGE;
            font.codePageName = wxS( "windows-1252" );
        }

        return font;
    }


    uint32_t pinElectricalType( uint8_t aType, const SOURCE_PROVENANCE& aSource,
                                std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        // PADS Logic canonical pin electrical types:
        // S, B, C, O, T, L, Z, P, G, U.  There is no canonical 'E'.
        switch( aType )
        {
        case 'U': return 0;
        case 'L': return 1;
        case 'S': return 2;
        case 'B': return 3;
        case 'T': return 4;
        case 'C':
        case 'O': return 5; // KiCad folds Or-tieable Source into Open Collector.
        case 'P':
        case 'G': return 7;
        case 'Z': return 8;
        default:
            PADS_SCH_BINARY_PARSER::RecordUnknownEnum( wxS( "pin electrical type" ), aType, aSource, aDiagnostics );
            return 8;
        }
    }


    SOURCE_POINT pageExtent( uint32_t aPage, const SOURCE_PROVENANCE& aSource )
    {
        // Controlled PADS Logic native-save fixtures prove that design settings
        // offset +80 is a u32 sheet-size enum, independent of the BORDER NAME
        // string stored at +264. Observed fixtures give 0=A, 2=C, 5=A4,
        // 6=A3, 7=A2, 8=A1, 9=A0 and 10=F; the contiguous ANSI sequence gives
        // 1=B, 3=D and 4=E.
        //
        // Dimensions are expressed in half-mils and use landscape orientation.
        switch( aPage )
        {
        case 0:  return { 22000, 17000, aSource }; // ANSI A
        case 1:  return { 34000, 22000, aSource }; // ANSI B
        case 2:  return { 44000, 34000, aSource }; // ANSI C
        case 3:  return { 68000, 44000, aSource }; // ANSI D
        case 4:  return { 88000, 68000, aSource }; // ANSI E
        case 5:  return { 23386, 16535, aSource }; // ISO A4
        case 6:  return { 33071, 23386, aSource }; // ISO A3
        case 7:  return { 46772, 33071, aSource }; // ISO A2
        case 8:  return { 66220, 46772, aSource }; // ISO A1
        case 9:  return { 93622, 66220, aSource }; // ISO A0
        case 10: return { 80000, 56000, aSource }; // ANSI F
        default: throwDecodeError( aSource, wxS( "invalid design page-size enum" ) );
        }
    }


    // PADS nine-way text justification.  Valid codes are a sparse 3x3
    // table, not contiguous vertical bands: 4 is center-bottom and 6 is center-top.
    MODEL_JUSTIFICATION horizontalJustification( uint16_t aValue )
    {
        switch( aValue & 0x00FF )
        {
        case 1:
        case 3:
        case 9: return MODEL_JUSTIFICATION::RIGHT;

        case 4:
        case 5:
        case 6:
        case 12: return MODEL_JUSTIFICATION::CENTER;

        default: return MODEL_JUSTIFICATION::LEFT;
        }
    }


    MODEL_JUSTIFICATION verticalJustification( uint16_t aValue )
    {
        switch( aValue & 0x00FF )
        {
        case 2:
        case 3:
        case 6: return MODEL_JUSTIFICATION::LEFT;   // KiCad TOP mapping

        case 8:
        case 9:
        case 12: return MODEL_JUSTIFICATION::CENTER;

        default: return MODEL_JUSTIFICATION::RIGHT; // KiCad BOTTOM mapping
        }
    }


    uint16_t rotatedFieldJustification( uint16_t aValue, int aAngle )
    {
        // PADS swaps the side-selector bits for field text rotated by
        // 90/270 degrees.  This mirrors the ASCII importer behavior.
        const int quarterTurns = NormalizeAngle( aAngle ) / 900;

        if( ( quarterTurns & 1 ) == 0 )
            return aValue;

        const uint16_t bit0 = aValue & 0x0001;
        const uint16_t bit1 = aValue & 0x0002;

        aValue &= static_cast<uint16_t>( ~0x0003 );

        if( bit0 )
            aValue |= 0x0002;

        if( bit1 )
            aValue |= 0x0001;

        return aValue;
    }


    MODEL_JUSTIFICATION freeTextHorizontalJustification( uint16_t aValue )
    {
        switch( aValue & 0x000F )
        {
        case 0:
        case 2:
        case 8: return MODEL_JUSTIFICATION::LEFT;

        case 4:
        case 6:
        case 10:
        case 12:
        case 14: return MODEL_JUSTIFICATION::CENTER;

        default: return MODEL_JUSTIFICATION::RIGHT;
        }
    }

    constexpr uint32_t CP1252_HIGH[] = { 0x20AC, 0xFFFD, 0x201A, 0x0192, 0x201E, 0x2026, 0x2020, 0x2021,
                                         0x02C6, 0x2030, 0x0160, 0x2039, 0x0152, 0xFFFD, 0x017D, 0xFFFD,
                                         0xFFFD, 0x2018, 0x2019, 0x201C, 0x201D, 0x2022, 0x2013, 0x2014,
                                         0x02DC, 0x2122, 0x0161, 0x203A, 0x0153, 0xFFFD, 0x017E, 0x0178 };


    wxString decodeWindows1252( const std::vector<uint8_t>& aBytes )
    {
        wxString result;

        for( uint8_t byte : aBytes )
        {
            uint32_t codePoint = byte;

            if( byte >= 0x80 && byte <= 0x9F )
                codePoint = CP1252_HIGH[byte - 0x80];

            result += wxUniChar( codePoint );
        }

        return result;
    }


    wxString decodeUnknownCodePage( const std::vector<uint8_t>& aBytes )
    {
        wxString result;

        for( uint8_t byte : aBytes )
            result += wxUniChar( byte < 0x80 ? byte : 0xFFFD );

        return result;
    }


    wxString decodeUtf8( const std::vector<uint8_t>& aBytes, bool& aHadInvalidBytes )
    {
        wxString result;
        aHadInvalidBytes = false;

        for( size_t i = 0; i < aBytes.size(); )
        {
            uint8_t  lead = aBytes[i];
            uint32_t codePoint = 0;
            size_t   continuationCount = 0;
            uint32_t minimum = 0;

            if( lead < 0x80 )
            {
                result += wxUniChar( lead );
                ++i;
                continue;
            }
            else if( lead >= 0xC2 && lead <= 0xDF )
            {
                codePoint = lead & 0x1F;
                continuationCount = 1;
                minimum = 0x80;
            }
            else if( lead >= 0xE0 && lead <= 0xEF )
            {
                codePoint = lead & 0x0F;
                continuationCount = 2;
                minimum = 0x800;
            }
            else if( lead >= 0xF0 && lead <= 0xF4 )
            {
                codePoint = lead & 0x07;
                continuationCount = 3;
                minimum = 0x10000;
            }
            else
            {
                result += wxUniChar( 0xFFFD );
                aHadInvalidBytes = true;
                ++i;
                continue;
            }

            bool valid = i + continuationCount < aBytes.size();

            for( size_t j = 1; valid && j <= continuationCount; ++j )
            {
                uint8_t continuation = aBytes[i + j];
                valid = ( continuation & 0xC0 ) == 0x80;

                if( valid )
                    codePoint = ( codePoint << 6 ) | ( continuation & 0x3F );
            }

            valid = valid && codePoint >= minimum && codePoint <= 0x10FFFF
                    && !( codePoint >= 0xD800 && codePoint <= 0xDFFF );

            if( !valid )
            {
                result += wxUniChar( 0xFFFD );
                aHadInvalidBytes = true;
                ++i;
                continue;
            }

            result += wxUniChar( codePoint );
            i += continuationCount + 1;
        }

        return result;
    }

    [[noreturn]] void throwValidationError( const SOURCE_PROVENANCE& aSource, const wxString& aMessage );


    template <typename Item, typename IdAccessor, typename SourceAccessor>
    void validateUniqueIds( const std::vector<Item>& aItems, const wxString& aObjectClass, IdAccessor aId,
                            SourceAccessor aSource )
    {
        std::map<uint32_t, SOURCE_PROVENANCE> declarations;

        for( const Item& item : aItems )
        {
            const auto& id = aId( item );

            if( !id.IsValid() )
                throwValidationError( aSource( item ), wxString::Format( wxS( "invalid %s ID" ), aObjectClass ) );

            auto [first, inserted] = declarations.emplace( id.Value(), aSource( item ) );

            if( !inserted )
            {
                const SOURCE_PROVENANCE& firstSource = first->second;
                wxString                 detail = wxString::Format(
                        wxS( "duplicate %s ID %u; first at v0x%04X %s controller %d record %llu sheet %d "
                                                             "offset 0x%llX" ),
                        aObjectClass, id.Value(), firstSource.version, firstSource.objectClass, firstSource.controller,
                        static_cast<unsigned long long>( firstSource.recordIndex ), firstSource.sheet,
                        static_cast<unsigned long long>( firstSource.absoluteOffset ) );
                throwValidationError( aSource( item ), detail );
            }
        }
    }


    [[noreturn]] void throwValidationError( const SOURCE_PROVENANCE& aSource, const wxString& aMessage )
    {
        THROW_IO_ERROR( FormatParserError( aSource, aMessage ) );
    }


    constexpr auto itemId = []( const auto& aItem ) -> const auto&
    {
        return aItem.id;
    };


    constexpr auto itemProvenance = []( const auto& aItem ) -> const SOURCE_PROVENANCE&
    {
        return aItem.source;
    };


    template <typename Item, typename Declarations>
    void validateNestedId( const Item& aItem, const wxString& aObjectClass, Declarations& aDeclarations )
    {
        if( !aItem.id.IsValid() )
            throwValidationError( aItem.source, wxString::Format( wxS( "invalid %s ID" ), aObjectClass ) );

        auto [first, inserted] = aDeclarations.emplace( aItem.id.Value(), aItem.source );

        if( !inserted )
        {
            throwValidationError(
                    aItem.source,
                    wxString::Format( wxS( "duplicate %s ID %llu; first at v0x%04X %s controller %d record %llu "
                                           "sheet %d offset 0x%llX" ),
                                      aObjectClass, static_cast<unsigned long long>( aItem.id.Value() ),
                                      first->second.version, first->second.objectClass, first->second.controller,
                                      static_cast<unsigned long long>( first->second.recordIndex ), first->second.sheet,
                                      static_cast<unsigned long long>( first->second.absoluteOffset ) ) );
        }
    }


    struct MODEL_REFERENCE_INDEX
    {
        std::unordered_map<uint32_t, const MODEL_SHEET*>             sheets;
        std::unordered_map<uint32_t, const MODEL_SYMBOL_DEFINITION*> definitions;
        std::unordered_map<uint32_t, const MODEL_PART_TYPE*>         partTypes;
        std::unordered_map<uint32_t, const MODEL_GATE*>              gates;
        std::unordered_map<uint32_t, const MODEL_PART_TYPE*>         gateOwners;
        std::unordered_map<uint32_t, const MODEL_PIN_DEFINITION*>    pins;
        std::unordered_map<uint32_t, const MODEL_SYMBOL_DEFINITION*> pinOwners;
        std::unordered_map<uint32_t, const MODEL_PLACEMENT*>         placements;
        std::unordered_map<uint32_t, const MODEL_NET*>               nets;

        explicit MODEL_REFERENCE_INDEX( const PADS_SCH_MODEL& aModel )
        {
            for( const MODEL_SHEET& sheet : aModel.sheets )
                sheets.emplace( sheet.id.Value(), &sheet );

            for( const MODEL_SYMBOL_DEFINITION& definition : aModel.definitions )
            {
                definitions.emplace( definition.id.Value(), &definition );

                for( const MODEL_PIN_DEFINITION& pin : definition.pins )
                {
                    pins.emplace( pin.id.Value(), &pin );
                    pinOwners.emplace( pin.id.Value(), &definition );
                }
            }

            for( const MODEL_PART_TYPE& partType : aModel.partTypes )
            {
                partTypes.emplace( partType.id.Value(), &partType );

                for( const MODEL_GATE& gate : partType.gates )
                {
                    gates.emplace( gate.id.Value(), &gate );
                    gateOwners.emplace( gate.id.Value(), &partType );
                }
            }

            for( const MODEL_PLACEMENT& placement : aModel.placements )
                placements.emplace( placement.id.Value(), &placement );

            for( const MODEL_NET& net : aModel.nets )
                nets.emplace( net.id.Value(), &net );
        }
    };


    bool endpointIsValid( const MODEL_REFERENCE_INDEX& aIndex, const MODEL_CONNECTION_ENDPOINT& aEndpoint )
    {
        if( aEndpoint.kind == MODEL_ENDPOINT_KIND::POINT )
            return !aEndpoint.placement && !aEndpoint.pin;

        if( aEndpoint.kind != MODEL_ENDPOINT_KIND::PIN || !aEndpoint.placement || !aEndpoint.pin )
            return false;

        auto placement = aIndex.placements.find( aEndpoint.placement->id.Value() );

        if( placement == aIndex.placements.end() )
            return false;

        return aIndex.pins.contains( aEndpoint.pin->id.Value() )
               && std::ranges::any_of( placement->second->pins,
                                       [&]( const PIN_REFERENCE& aPin )
                                       {
                                           return aPin.id == aEndpoint.pin->id;
                                       } );
    }


    struct SHEET_CONTROLLERS
    {
        std::array<SCH_SDB_POOL, 23> pools;
        std::array<size_t, 23>       offsets;
    };


    struct PLACEMENT_GLOBALS
    {
        size_t   attributeHeapBase = 0;
        uint32_t attributeHeapBytes = 0;
        size_t   groupBase = 0;
        uint32_t groupCount = 0;
        size_t   attributeOffsetBase = 0;
        uint32_t attributeOffsetCount = 0;
        size_t                           fontBase = 0;
        uint32_t                         fontCount = 0;
        MODEL_FONT_MODE                  fontMode = MODEL_FONT_MODE::STROKE;
        SOURCE_STRING                    defaultSystemFont;
    };


    PLACEMENT_GLOBALS placementGlobals( const PADS_SCH_SDB& aSdb, const wxString& aSourceName,
                                                 const DESIGN_SETTINGS& aSettings )
    {
        PLACEMENT_GLOBALS result;

        auto requireOuterStride = [&]( size_t aController, size_t aStride )
        {
            const SCH_SDB_POOL& pool = aSdb.Pools()[aController];

            if( pool.usedBytes != pool.count * aStride )
            {
                SOURCE_PROVENANCE source = sourceAt(
                        aSourceName, aSdb.Version(), wxS( "outer controller directory" ), aController, 0,
                        OUTER_DIRECTORY_OFFSET + aController * OUTER_DESCRIPTOR_BYTES + OUTER_USED_BYTES_OFFSET, 4,
                        -1 );
                throwDecodeError( source,
                                  wxString::Format( wxS( "controller byte count does not match %llu-byte records" ),
                                                    static_cast<unsigned long long>( aStride ) ) );
            }
        };

        requireOuterStride( 6, PLACEMENT_GROUP_BYTES );
        requireOuterStride( 7, ATTRIBUTE_OFFSET_BYTES );
        requireOuterStride( 19, FONT_RECORD_BYTES );
        result.attributeHeapBase = outerControllerOffset( aSdb, 2 );
        result.attributeHeapBytes = aSdb.Pools()[2].usedBytes;
        result.groupBase = outerControllerOffset( aSdb, 6 );
        result.groupCount = aSdb.Pools()[6].count;
        result.attributeOffsetBase = outerControllerOffset( aSdb, 7 );
        result.attributeOffsetCount = aSdb.Pools()[7].count;
        result.fontBase = outerControllerOffset( aSdb, 19 );
        result.fontCount = aSdb.Pools()[19].count;
        result.fontMode = aSettings.fontMode;
        result.defaultSystemFont = aSettings.defaultSystemFont;
        return result;
    }


    void decodeGlobalFont( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                           const PLACEMENT_GLOBALS& aGlobals, const wxString& aSourceName, uint16_t aVersion,
                           int16_t aHandle, const SOURCE_PROVENANCE& aHandleSource,
                           MODEL_TEXT_PRESENTATION& aPresentation, bool aStrictHandle,
                           std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        if( aHandle == -1 || aHandle == -4 )
        {
            if( aGlobals.fontMode == MODEL_FONT_MODE::SYSTEM
                && !aGlobals.defaultSystemFont.text.IsEmpty() )
            {
                aPresentation.font = aGlobals.defaultSystemFont;
            }
            else
            {
                aPresentation.font = decodedDefinitionFont( aHandle, aHandleSource );
            }

            return;
        }

        if( aHandle < 0 || static_cast<uint32_t>( aHandle ) >= aGlobals.fontCount )
        {
            if( aStrictHandle )
                throwDecodeError( aHandleSource, wxS( "placement font handle leaves outer controller 19" ) );

            SOURCE_PROPERTY property = sourceProperty(
                    wxS( "inline_font_payload" ), wxString::Format( wxS( "%u" ), uint16_t( aHandle ) ), aHandleSource );
            property.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
            aDiagnostics.push_back( MakePropertyDiagnostic(
                    RPT_SEVERITY_WARNING, property, wxS( "unsupported inline placement font payload preserved" ) ) );
            aPresentation.properties.push_back( std::move( property ) );
            return;
        }

        const size_t      fontOffset = aGlobals.fontBase + static_cast<size_t>( aHandle ) * FONT_RECORD_BYTES;
        SOURCE_PROVENANCE fontSource = sourceAt( aSourceName, aVersion, wxS( "placement font" ), 19, aHandle,
                                                 fontOffset, FONT_RECORD_BYTES, -1 );
        const uint32_t    style = aCursor.U32At( fontOffset );

        SOURCE_PROVENANCE nameSource = fontSource;
        nameSource.absoluteOffset += 4;
        nameSource.length = 32;
        SOURCE_STRING name = decodeFixedString( aBytes, fontOffset + 4, 32, nameSource, aDiagnostics );
        aPresentation.bold = ( style & 2 ) != 0;
        aPresentation.italic = ( style & 1 ) != 0;
        aPresentation.underline = ( style & 4 ) != 0;
        aPresentation.font = name;

        if( aPresentation.bold )
            aPresentation.font.text.Prepend( wxS( "Bold " ) );

        if( aPresentation.italic )
            aPresentation.font.text.Prepend( wxS( "Italic " ) );

        aPresentation.properties.push_back(
                sourceProperty( wxS( "font_handle" ), wxString::Format( wxS( "%d" ), aHandle ), fontSource ) );

        if( ( style & ~uint32_t{ 7 } ) != 0 )
        {
            SOURCE_PROPERTY property = sourceProperty( wxS( "unsupported_font_style_flags" ),
                                                       wxString::Format( wxS( "%u" ), style & ~7U ), fontSource );
            property.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
            aDiagnostics.push_back( MakePropertyDiagnostic(
                    RPT_SEVERITY_WARNING, property, wxS( "unsupported placement font style flags preserved" ) ) );
            aPresentation.properties.push_back( std::move( property ) );
        }
    }


    CONNECTIVITY_GLOBALS connectivityGlobals( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                                              const PADS_SCH_SDB& aSdb, const wxString& aSourceName,
                                              PADS_SCH_MODEL& aModel )
    {
        CONNECTIVITY_GLOBALS result;
        const SCH_SDB_POOL&  membershipPool = aSdb.Pools()[4];
        const SCH_SDB_POOL&  netPool = aSdb.Pools()[8];
        const SCH_SDB_POOL&  fontPool = aSdb.Pools()[19];

        if( membershipPool.usedBytes != membershipPool.count * NET_MEMBERSHIP_BYTES )
        {
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "net membership directory" ), 4, 0,
                                                 outerControllerOffset( aSdb, 4 ), membershipPool.usedBytes, -1 );
            throwDecodeError( source, wxS( "controller byte count does not match 2-byte records" ) );
        }

        if( netPool.usedBytes != netPool.count * NET_RECORD_BYTES )
        {
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "net directory" ), 8, 0,
                                                 outerControllerOffset( aSdb, 8 ), netPool.usedBytes, -1 );
            throwDecodeError( source, wxS( "controller byte count does not match 88-byte records" ) );
        }

        if( fontPool.usedBytes != fontPool.count * FONT_RECORD_BYTES )
        {
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "font directory" ), 19, 0,
                                                 outerControllerOffset( aSdb, 19 ), fontPool.usedBytes, -1 );
            throwDecodeError( source, wxS( "controller byte count does not match 36-byte records" ) );
        }

        result.membershipBase = outerControllerOffset( aSdb, 4 );
        result.membershipCount = membershipPool.count;
        result.fontBase = outerControllerOffset( aSdb, 19 );
        result.fontCount = fontPool.count;
        result.membershipSheets.reserve( membershipPool.count );

        for( size_t record = 0; record < membershipPool.count; ++record )
            result.membershipSheets.push_back( aCursor.U16At( result.membershipBase + record * 2 ) );

        const size_t      netBase = outerControllerOffset( aSdb, 8 );
        const size_t      aliasHeapBase = outerControllerOffset( aSdb, 1 );
        const uint32_t    aliasHeapBytes = aSdb.Pools()[1].usedBytes;
        std::vector<bool> claimedMemberships( membershipPool.count, false );

        result.nets.reserve( netPool.count );

        for( size_t record = 0; record < netPool.count; ++record )
        {
            const size_t      offset = netBase + record * NET_RECORD_BYTES;
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "global net" ), 8, record, offset,
                                                 NET_RECORD_BYTES, -1 );
            GLOBAL_NET_RECORD net;
            net.source = source;
            net.preservedIdentity = aCursor.U32At( offset );
            net.preservedRelationship = aCursor.U32At( offset + 84 );
            net.membershipStart = aCursor.U32At( offset + 4 );
            net.aliasStringOffset = aCursor.U32At( offset + 8 );
            net.membershipCount = aCursor.U16At( offset + 16 );
            net.aliasCount = aCursor.U16At( offset + 18 );
            net.kindFlags = aCursor.U16At( offset + 22 );
            net.tombstone = net.membershipCount == 0xFFFF;
            SOURCE_PROVENANCE nameSource = source;
            nameSource.absoluteOffset += 24;
            nameSource.length = 56;
            net.name = decodeFixedString( aBytes, offset + 24, 56, nameSource, aModel.diagnostics );

            if( aCursor.U16At( offset + 20 ) != 0 || aCursor.U32At( offset + 80 ) != 0xFFFFFFFF )
            {
                throwDecodeError( source, wxS( "global net record has nonzero padding or a live reserved link" ) );
            }

            if( !net.tombstone
                && ( net.membershipStart > membershipPool.count
                     || net.membershipCount > membershipPool.count - net.membershipStart ) )
            {
                throwDecodeError( source, wxS( "net sheet-membership slice leaves outer controller 4" ) );
            }

            for( uint32_t membership = net.membershipStart;
                 !net.tombstone && membership < net.membershipStart + net.membershipCount; ++membership )
            {
                if( claimedMemberships[membership] )
                    throwDecodeError( source, wxS( "duplicate net sheet-membership ownership" ) );

                if( result.membershipSheets[membership] >= aModel.sheets.size() )
                    throwDecodeError( source, wxS( "net membership references the wrong sheet object class" ) );

                claimedMemberships[membership] = true;
            }

            if( net.aliasCount == 0 )
            {
                if( net.aliasStringOffset != 0xFFFFFFFF )
                    throwDecodeError( source, wxS( "non-alias net has an alias-string handle" ) );
            }
            else
            {
                if( net.aliasStringOffset >= aliasHeapBytes )
                    throwDecodeError( source, wxS( "bus-alias string handle leaves outer controller 1" ) );

                size_t stringOffset = net.aliasStringOffset;

                for( size_t alias = 0; alias < net.aliasCount; ++alias )
                {
                    SOURCE_PROVENANCE aliasSource =
                            sourceAt( aSourceName, aModel.version, wxS( "bus alias member" ), 1, alias,
                                      aliasHeapBase + stringOffset, aliasHeapBytes - stringOffset, -1 );
                    SOURCE_STRING member =
                            decodeFixedString( aBytes, aliasHeapBase + stringOffset, aliasHeapBytes - stringOffset,
                                               aliasSource, aModel.diagnostics );
                    aliasSource.length = member.raw.size();
                    member.source.length = member.raw.size();
                    stringOffset += member.raw.size() + 1;
                    net.aliasMembers.push_back( std::move( member ) );
                }
            }

            result.nets.push_back( std::move( net ) );
        }

        return result;
    }


    SHEET_CONTROLLERS sheetControllers( const PADS_IO::BINARY_CURSOR& aCursor, const SCH_SDB_BLOCK& aBlock )
    {
        SHEET_CONTROLLERS result;
        size_t            descriptor = aBlock.offset + SHEET_HEADER_BYTES;
        size_t            payload = descriptor + 24 * SHEET_DESCRIPTOR_BYTES;

        for( size_t i = 0; i < result.pools.size(); ++i )
        {
            result.pools[i].count = aCursor.U32At( descriptor + i * SHEET_DESCRIPTOR_BYTES + 12 );
            result.pools[i].usedBytes = aCursor.U32At( descriptor + i * SHEET_DESCRIPTOR_BYTES + 16 );
            result.offsets[i] = payload;
            payload += result.pools[i].usedBytes;
        }

        return result;
    }


    void requireFixedController( const SHEET_CONTROLLERS& aControllers, size_t aController, size_t aStride,
                                 const wxString& aFile, uint16_t aVersion, int aSheet )
    {
        const SCH_SDB_POOL& pool = aControllers.pools[aController - 1];

        if( pool.usedBytes != pool.count * aStride )
        {
            SOURCE_PROVENANCE source =
                    sourceAt( aFile, aVersion, wxS( "controller directory" ), static_cast<int>( aController ), 0,
                              aControllers.offsets[aController - 1], pool.usedBytes, aSheet );
            throwDecodeError( source, wxString::Format( wxS( "controller byte count does not match %llu-byte records" ),
                                                        static_cast<unsigned long long>( aStride ) ) );
        }
    }


    enum class TEXT_ROLE
    {
        DEFINITION_FIELD,
        EMBEDDED_SYMBOL_TEXT,
        PAGE_TEXT
    };


    struct DEFINITION_TEXT_HEAP
    {
        size_t   recordBase = 0;
        uint32_t recordCount = 0;
        size_t   stringBase = 0;
        uint32_t stringBytes = 0;
    };


    MODEL_TEXT_PRESENTATION decodeTextPresentation( const PADS_IO::BINARY_CURSOR& aCursor, size_t aOffset,
                                                    const SOURCE_PROVENANCE& aSource, TEXT_ROLE aRole,
                                                    int16_t& aRelationship )
    {
        MODEL_TEXT_PRESENTATION presentation;
        presentation.source = aSource;
        // PADS text records store presentation sizes in whole mils; the semantic
        // model uses half-mils, so retaining the raw value made text half size.
        presentation.height = static_cast<int64_t>( aCursor.U16At( aOffset + 22 ) ) * 2;
        presentation.width = static_cast<int64_t>( aCursor.U8At( aOffset + 30 ) ) * 2;
        presentation.properties.push_back( sourceProperty(
                wxS( "display_flags" ), wxString::Format( wxS( "%u" ), aCursor.U8At( aOffset + 31 ) ), aSource ) );
        // PADS stores the nine-way justification selector in the text-local
        // frame.  For 90/270 degree text the low selector bits are swapped;
        // normalize them before decoding the semantic horizontal/vertical anchor.
        // Controlled title-block fixtures confirm:
        //   Middle Right: 0 deg raw 9, 90 deg raw 10 -> normalized 9
        //   Top Right:    0 deg raw 3, 90 deg raw 3  -> normalized 3
        const int      textAngle = NormalizeAngle( aCursor.U16At( aOffset + 16 ) );
        const uint16_t justification =
                rotatedFieldJustification( aCursor.U16At( aOffset + 18 ), textAngle );
        presentation.horizontalJustification = horizontalJustification( justification );
        presentation.verticalJustification = verticalJustification( justification );

        SOURCE_PROVENANCE fontSource = aSource;
        fontSource.absoluteOffset += 28;
        fontSource.length = 2;
        aRelationship = static_cast<int16_t>( aCursor.U16At( aOffset + 28 ) );

        // Page text spends word 28 on the successor ordinal, so it always falls back to the default font.
        const bool pageText = aRole == TEXT_ROLE::PAGE_TEXT;

        presentation.font = decodedDefinitionFont( pageText ? int16_t( -1 ) : aRelationship, fontSource );
        presentation.properties.push_back( sourceProperty( pageText ? wxS( "successor_ordinal" ) : wxS( "font_handle" ),
                                                           wxString::Format( wxS( "%d" ), aRelationship ),
                                                           fontSource ) );
        return presentation;
    }


    void decodeDefinitionTextRecord( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                                     const wxString& aSourceName, size_t aSheetIndex, const DEFINITION_TEXT_HEAP& aHeap,
                                     size_t aRecord, TEXT_ROLE aRole, MODEL_SYMBOL_DEFINITION& aDefinition,
                                     PADS_SCH_MODEL& aModel )
    {
        if( aRecord >= aHeap.recordCount )
            throwDecodeError( aDefinition.source, wxS( "definition field handle leaves controller 1" ) );

        const bool        isField = aRole == TEXT_ROLE::DEFINITION_FIELD;
        const size_t      offset = aHeap.recordBase + aRecord * TEXT_RECORD_BYTES;
        SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version,
                                             isField ? wxS( "definition field" ) : wxS( "embedded symbol text" ), 1,
                                             aRecord, offset, TEXT_RECORD_BYTES, static_cast<int>( aSheetIndex ) );
        const uint32_t    stringOffset = aCursor.U32At( offset + 8 );
        const uint16_t    stringBytes = aCursor.U16At( offset + 20 );

        if( stringBytes == 0 || stringOffset > aHeap.stringBytes || stringBytes > aHeap.stringBytes - stringOffset
            || aBytes[aHeap.stringBase + stringOffset + stringBytes - 1] != 0 )
        {
            throwDecodeError( source, wxS( "definition field string leaves controller 2" ) );
        }

        SOURCE_PROVENANCE stringSource =
                sourceAt( aSourceName, aModel.version, source.objectClass + wxS( " string" ), 2, aRecord,
                          aHeap.stringBase + stringOffset, stringBytes - 1, static_cast<int>( aSheetIndex ) );
        SOURCE_STRING string = PADS_SCH_BINARY_PARSER::DecodeString(
                { aBytes.begin() + stringSource.absoluteOffset,
                  aBytes.begin() + stringSource.absoluteOffset + stringSource.length },
                DEFAULT_CODE_PAGE, stringSource, aModel.diagnostics );

        int16_t                 relationship = 0;
        MODEL_TEXT_PRESENTATION presentation = decodeTextPresentation( aCursor, offset, source, aRole, relationship );
        const SOURCE_POINT      position{ decodeLocalCoordinate( aCursor.U16At( offset + 12 ) ),
                                     decodeLocalCoordinate( aCursor.U16At( offset + 14 ) ), source };
        const int               angle = NormalizeAngle( aCursor.U16At( offset + 16 ) );

        if( isField )
        {
            MODEL_FIELD field;
            field.source = source;
            field.name = std::move( string );
            field.position = position;
            field.angle = angle;
            field.presentation = std::move( presentation );
            aDefinition.fields.push_back( std::move( field ) );
            return;
        }

        MODEL_GRAPHIC graphic;
        graphic.source = source;
        graphic.kind = MODEL_GRAPHIC_KIND::TEXT;
        graphic.text = std::move( string );
        graphic.points.push_back( position );
        graphic.presentation = std::move( presentation );
        graphic.angle = angle;
        aDefinition.graphics.push_back( std::move( graphic ) );
    }


    struct DEFINITION_LAYOUT
    {
        size_t   symbolBase = 0;
        size_t   pieceBase = 0;
        size_t   vertexBase = 0;
        size_t   arcBase = 0;
        size_t   usedDecalBase = 0;
        size_t   terminalBase = 0;
        size_t   partBase = 0;
        size_t   gateBase = 0;
        size_t   pinBase = 0;
        size_t   signalPinBase = 0;
        size_t   pinNameBase = 0;
        uint32_t pinNameBytes = 0;
        uint32_t definitionIdBase = 0;
        uint32_t pinIdBase = 0;
        uint32_t partIdBase = 0;
        uint32_t gateIdBase = 0;

        DEFINITION_TEXT_HEAP             textHeap;
        std::vector<uint32_t>            definitionPieceStart;
        std::vector<uint32_t>            pieceVertexStart;
        std::vector<std::vector<size_t>> pieceArcRecords;
    };


    struct USED_DECAL
    {
        size_t                   record = 0;
        uint32_t                 definitionRecord = 0;
        uint16_t                 terminalStart = 0;
        uint8_t                  terminalCount = 0;
        uint32_t                 fieldStart = 0;
        MODEL_SYMBOL_DEFINITION* definition = nullptr;
    };


    void preserveRawDefinitionControllers( const std::vector<uint8_t>& aBytes, const SHEET_CONTROLLERS& aControllers,
                                           size_t aSheetIndex, const wxString& aSourceName, PADS_SCH_MODEL& aModel )
    {
        for( size_t controller = 3; controller <= 23; ++controller )
        {
            const SCH_SDB_POOL& pool = aControllers.pools[controller - 1];

            if( pool.usedBytes == 0 )
                continue;

            SOURCE_PROVENANCE source =
                    sourceAt( aSourceName, aModel.version, wxS( "definition controller" ), controller, 0,
                              aControllers.offsets[controller - 1], pool.usedBytes, static_cast<int>( aSheetIndex ) );
            aModel.preservedControllerPayloads.push_back(
                    { source,
                      PROPERTY_DISPOSITION::PRESERVED,
                      { aBytes.begin() + source.absoluteOffset,
                        aBytes.begin() + source.absoluteOffset + source.length } } );
        }
    }


    DEFINITION_LAYOUT definitionLayout( const PADS_IO::BINARY_CURSOR& aCursor, const SHEET_CONTROLLERS& aControllers,
                                        size_t aSheetIndex, const wxString& aSourceName, uint16_t aVersion )
    {
        const bool legacyV9 = aVersion == 0x0009;
        requireFixedController( aControllers, 3, SYMBOL_RECORD_BYTES, aSourceName, aVersion, aSheetIndex );
        requireFixedController( aControllers, 4, SYMBOL_PIECE_BYTES, aSourceName, aVersion, aSheetIndex );
        requireFixedController( aControllers, 5, SYMBOL_VERTEX_BYTES, aSourceName, aVersion, aSheetIndex );
        requireFixedController( aControllers, 6, SYMBOL_ARC_BYTES, aSourceName, aVersion, aSheetIndex );
        requireFixedController( aControllers, 7, legacyV9 ? 92 : USED_DECAL_BYTES, aSourceName, aVersion, aSheetIndex );
        requireFixedController( aControllers, 8, TERMINAL_BYTES, aSourceName, aVersion, aSheetIndex );
        requireFixedController( aControllers, 9, PART_TYPE_BYTES, aSourceName, aVersion, aSheetIndex );
        requireFixedController( aControllers, 10, GATE_BYTES, aSourceName, aVersion, aSheetIndex );
        requireFixedController( aControllers, 11, legacyV9 ? 8 : PIN_BYTES, aSourceName, aVersion, aSheetIndex );
        requireFixedController( aControllers, 12, legacyV9 ? 52 : SIGNAL_PIN_BYTES, aSourceName, aVersion, aSheetIndex );

        DEFINITION_LAYOUT layout;
        layout.symbolBase = aControllers.offsets[2];
        layout.pieceBase = aControllers.offsets[3];
        layout.vertexBase = aControllers.offsets[4];
        layout.arcBase = aControllers.offsets[5];
        layout.usedDecalBase = aControllers.offsets[6];
        layout.terminalBase = aControllers.offsets[7];
        layout.partBase = aControllers.offsets[8];
        layout.gateBase = aControllers.offsets[9];
        layout.pinBase = aControllers.offsets[10];
        layout.signalPinBase = aControllers.offsets[11];
        layout.pinNameBase = aControllers.offsets[13];
        layout.pinNameBytes = aControllers.pools[13].usedBytes;
        layout.definitionIdBase = static_cast<uint32_t>( aSheetIndex * 0x100000 + 1 );
        layout.pinIdBase = static_cast<uint32_t>( aSheetIndex * 0x100000 + 0x10000 );
        layout.partIdBase = static_cast<uint32_t>( aSheetIndex * 0x100000 + 0x20000 );
        layout.gateIdBase = static_cast<uint32_t>( aSheetIndex * 0x100000 + 0x30000 );
        layout.textHeap = { aControllers.offsets[0], aControllers.pools[0].count, aControllers.offsets[1],
                            aControllers.pools[1].usedBytes };

        // Prefix sums over file-supplied per-record counts. A 32-bit accumulator wraps on a
        // crafted file and the exact-total checks below then compare small against their pools
        // while the stored offsets are already nonsense, so accumulate in 64 bits
        uint64_t vertexCursor = 0;
        uint64_t pieceCursor = 0;

        for( size_t definition = 0; definition < aControllers.pools[2].count; ++definition )
        {
            layout.definitionPieceStart.push_back( static_cast<uint32_t>( pieceCursor ) );
            pieceCursor += aCursor.U16At( layout.symbolBase + definition * SYMBOL_RECORD_BYTES + 0x2A );
        }

        if( pieceCursor != aControllers.pools[3].count )
        {
            SOURCE_PROVENANCE source =
                    sourceAt( aSourceName, aVersion, wxS( "symbol definition" ), 3, 0, layout.symbolBase,
                              aControllers.pools[2].usedBytes, static_cast<int>( aSheetIndex ) );
            throwDecodeError( source, wxS( "symbol graphic-piece counts leave controller 4" ) );
        }

        for( size_t piece = 0; piece < aControllers.pools[3].count; ++piece )
        {
            layout.pieceVertexStart.push_back( static_cast<uint32_t>( vertexCursor ) );
            vertexCursor += aCursor.U16At( layout.pieceBase + piece * SYMBOL_PIECE_BYTES + 2 );
        }

        if( vertexCursor != aControllers.pools[4].count )
        {
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aVersion, wxS( "symbol piece" ), 4, 0, layout.pieceBase,
                                                 aControllers.pools[3].usedBytes, static_cast<int>( aSheetIndex ) );
            throwDecodeError( source, wxS( "symbol piece vertex counts do not consume controller 5" ) );
        }

        for( size_t definition = 0; definition < aControllers.pools[2].count; ++definition )
        {
            const size_t   definitionOffset = layout.symbolBase + definition * SYMBOL_RECORD_BYTES;
            const uint32_t firstPiece = layout.definitionPieceStart[definition];
            const uint32_t pieceEnd = definition + 1 < layout.definitionPieceStart.size()
                                              ? layout.definitionPieceStart[definition + 1]
                                              : aControllers.pools[3].count;
            const uint32_t firstVertex = aCursor.U32At( definitionOffset + 0x34 );
            const uint32_t vertexEnd = definition + 1 < aControllers.pools[2].count
                                               ? aCursor.U32At( definitionOffset + SYMBOL_RECORD_BYTES + 0x34 )
                                               : aControllers.pools[4].count;
            const bool     emptyMatches = firstPiece == pieceEnd && firstVertex == vertexEnd;
            const bool     ownedMatches =
                    firstPiece < pieceEnd && layout.pieceVertexStart[firstPiece] == firstVertex
                    && layout.pieceVertexStart[pieceEnd - 1]
                                       + aCursor.U16At( layout.pieceBase + ( pieceEnd - 1 ) * SYMBOL_PIECE_BYTES + 2 )
                               == vertexEnd;

            if( !emptyMatches && !ownedMatches )
            {
                SOURCE_PROVENANCE source =
                        sourceAt( aSourceName, aVersion, wxS( "symbol definition" ), 3, definition, definitionOffset,
                                  SYMBOL_RECORD_BYTES, static_cast<int>( aSheetIndex ) );
                throwDecodeError( source, wxS( "symbol piece/vertex ownership mismatch" ) );
            }
        }

        layout.pieceArcRecords.resize( aControllers.pools[3].count );
        size_t discoveredArcCount = 0;

        for( size_t piece = 0; piece < aControllers.pools[3].count; ++piece )
        {
            const size_t   pieceOffset = layout.pieceBase + piece * SYMBOL_PIECE_BYTES;
            const uint16_t pointCount = aCursor.U16At( pieceOffset + 2 );

            for( size_t point = 0; point < pointCount; ++point )
            {
                const size_t vertexOffset =
                        layout.vertexBase + ( layout.pieceVertexStart[piece] + point ) * SYMBOL_VERTEX_BYTES;

                if( static_cast<int16_t>( aCursor.U16At( vertexOffset + 4 ) ) >= 0 )
                    layout.pieceArcRecords[piece].push_back( discoveredArcCount++ );
            }
        }

        if( discoveredArcCount != aControllers.pools[5].count )
        {
            SOURCE_PROVENANCE source =
                    sourceAt( aSourceName, aVersion, wxS( "symbol arc" ), 6, discoveredArcCount, layout.arcBase,
                              aControllers.pools[5].usedBytes, static_cast<int>( aSheetIndex ) );
            throwDecodeError( source, wxS( "arc markers do not consume controller 6" ) );
        }

        return layout;
    }


    void decodeSymbolDefinitions( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                                  const SHEET_CONTROLLERS& aControllers, const DEFINITION_LAYOUT& aLayout,
                                  size_t aSheetIndex, const wxString& aSourceName,
                                  std::vector<MODEL_SYMBOL_DEFINITION*>& aDefinitionsByRecord,
                                  std::vector<size_t>& aPageGraphicRecords, PADS_SCH_MODEL& aModel )
    {
        // Definitions must not reallocate; aDefinitionsByRecord holds pointers into them
        aModel.definitions.reserve( aModel.definitions.size() + aControllers.pools[2].count );

        for( size_t record = 0; record < aControllers.pools[2].count; ++record )
        {
            const size_t      offset = aLayout.symbolBase + record * SYMBOL_RECORD_BYTES;
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "symbol definition" ), 3, record,
                                                 offset, SYMBOL_RECORD_BYTES, static_cast<int>( aSheetIndex ) );
            SOURCE_PROVENANCE nameSource = source;
            nameSource.length = 38;
            SOURCE_STRING  name = decodeFixedString( aBytes, offset, 38, nameSource, aModel.diagnostics );
            const uint8_t  objectClass = aCursor.U8At( offset + 0x29 );
            const uint32_t vertexStart = aCursor.U32At( offset + 0x34 );
            const uint32_t vertexEnd = record + 1 < aControllers.pools[2].count
                                               ? aCursor.U32At( offset + SYMBOL_RECORD_BYTES + 0x34 )
                                               : aControllers.pools[4].count;

            if( vertexStart > vertexEnd || vertexEnd > aControllers.pools[4].count )
                throwDecodeError( source, wxS( "symbol definition vertex slice leaves controller 5" ) );

            if( objectClass == 0 )
            {
                const uint32_t firstPiece = aLayout.definitionPieceStart[record];
                const uint32_t pieceEnd = record + 1 < aLayout.definitionPieceStart.size()
                                                  ? aLayout.definitionPieceStart[record + 1]
                                                  : aControllers.pools[3].count;
                const uint16_t groupTextCount = aCursor.U16At( offset + 64 );
                const uint16_t groupLastText = aCursor.U16At( offset + 66 );
                const bool     worksheetCandidate = pieceEnd - firstPiece == 69 && groupTextCount == 58;
                const bool     hasCircularTextOwnership = groupTextCount == 0
                                                      || ( groupTextCount <= aLayout.textHeap.recordCount
                                                           && groupLastText < aLayout.textHeap.recordCount );
                SOURCE_PROVENANCE originSource = source;
                originSource.objectClass = wxS( "page graphic group origin" );
                originSource.absoluteOffset += 60;
                originSource.length = 4;
                const SOURCE_POINT groupOrigin{ decodeCoordinate( aCursor.U16At( offset + 60 ) ),
                                                decodeCoordinate( aCursor.U16At( offset + 62 ) ), originSource };

                if( aCursor.U16At( offset + 42 ) != pieceEnd - firstPiece )
                    throwDecodeError( source, wxS( "page-graphic piece count does not match controller 4 slice" ) );

                for( size_t piece = firstPiece; piece < pieceEnd; ++piece )
                {
                    const size_t      pieceOffset = aLayout.pieceBase + piece * SYMBOL_PIECE_BYTES;
                    const uint8_t     pieceKind = aCursor.U8At( pieceOffset );
                    const uint8_t     lineStyle = aCursor.U8At( pieceOffset + 1 );
                    const uint16_t    pointCount = aCursor.U16At( pieceOffset + 2 );
                    const uint32_t    firstVertex = aLayout.pieceVertexStart[piece];
                    SOURCE_PROVENANCE graphicSource =
                            sourceAt( aSourceName, aModel.version, wxS( "page graphic" ), 4, piece, pieceOffset,
                                      SYMBOL_PIECE_BYTES, static_cast<int>( aSheetIndex ) );

                    if( firstVertex + pointCount > vertexEnd )
                        throwDecodeError( graphicSource, wxS( "page graphic crosses its controller-5 slice" ) );

                    MODEL_GRAPHIC graphic;
                    graphic.source = graphicSource;
                    decodeGraphicStrokeWidth( aCursor.U8At( pieceOffset + 4 ), aCursor.U8At( pieceOffset + 5 ),
                                              graphicSource, graphic, aModel.diagnostics );

                    switch( lineStyle )
                    {
                    case 0xFF: graphic.lineStyle = MODEL_LINE_STYLE::SOLID; break;
                    case 0: graphic.lineStyle = MODEL_LINE_STYLE::DASH; break;
                    case 1: graphic.lineStyle = MODEL_LINE_STYLE::DOT; break;
                    default:
                        PADS_SCH_BINARY_PARSER::RecordUnknownEnum( wxS( "page graphic line style" ), lineStyle,
                                                                   graphicSource, aModel.diagnostics );
                        graphic.lineStyle = MODEL_LINE_STYLE::DEFAULT;
                        break;
                    }

                    switch( pieceKind )
                    {
                    case 0:
                        graphic.kind = pointCount == 2 ? MODEL_GRAPHIC_KIND::LINE : MODEL_GRAPHIC_KIND::POLYLINE;
                        break;
                    case 1: graphic.kind = MODEL_GRAPHIC_KIND::POLYLINE; break;
                    case 2: graphic.kind = MODEL_GRAPHIC_KIND::CIRCLE; break;
                    case 4:
                        graphic.kind = MODEL_GRAPHIC_KIND::POLYLINE;
                        graphic.fill = MODEL_FILL_STYLE::FILLED;
                        break;
                    default:
                        PADS_SCH_BINARY_PARSER::RecordUnknownEnum( wxS( "page graphic kind" ), pieceKind, graphicSource,
                                                                   aModel.diagnostics );
                        graphic.kind = MODEL_GRAPHIC_KIND::POLYLINE;
                        break;
                    }

                    for( size_t point = 0; point < pointCount; ++point )
                    {
                        const size_t pointOffset = aLayout.vertexBase + ( firstVertex + point ) * SYMBOL_VERTEX_BYTES;
                        SOURCE_PROVENANCE pointSource = sourceAt(
                                aSourceName, aModel.version, wxS( "page graphic vertex" ), 5, firstVertex + point,
                                pointOffset, SYMBOL_VERTEX_BYTES, static_cast<int>( aSheetIndex ) );
                        graphic.points.push_back( { decodeLocalCoordinate( aCursor.U16At( pointOffset ) ),
                                                    decodeLocalCoordinate( aCursor.U16At( pointOffset + 2 ) ),
                                                    pointSource } );
                    }

                    const int16_t arcMarker = static_cast<int16_t>(
                            aCursor.U16At( aLayout.vertexBase + firstVertex * SYMBOL_VERTEX_BYTES + 4 ) );

                    if( pieceKind == 0 && arcMarker >= 0 )
                    {
                        if( aLayout.pieceArcRecords[piece].empty() )
                            throwDecodeError( graphicSource, wxS( "page arc has no controller-6 record" ) );

                        const size_t      arcRecord = aLayout.pieceArcRecords[piece].front();
                        const size_t      arcOffset = aLayout.arcBase + arcRecord * SYMBOL_ARC_BYTES;
                        SOURCE_PROVENANCE arcSource =
                                sourceAt( aSourceName, aModel.version, wxS( "page arc" ), 6, arcRecord, arcOffset,
                                          SYMBOL_ARC_BYTES, static_cast<int>( aSheetIndex ) );
                        graphic.kind = MODEL_GRAPHIC_KIND::ARC;
                        graphic.arcSweepAngle = aCursor.U16At( arcOffset );
                        graphic.arcClockwise = static_cast<int16_t>( aCursor.U16At( arcOffset + 2 ) ) < 0;
                        graphic.arcBoundsStart = { decodeLocalCoordinate( aCursor.U16At( arcOffset + 6 ) ),
                                                   decodeLocalCoordinate( aCursor.U16At( arcOffset + 8 ) ), arcSource };
                        graphic.arcBoundsEnd = { decodeLocalCoordinate( aCursor.U16At( arcOffset + 10 ) ),
                                                 decodeLocalCoordinate( aCursor.U16At( arcOffset + 12 ) ), arcSource };
                        graphic.arcCenter = { ( graphic.arcBoundsStart.x + graphic.arcBoundsEnd.x ) / 2,
                                              ( graphic.arcBoundsStart.y + graphic.arcBoundsEnd.y ) / 2, arcSource };
                    }

                    for( SOURCE_POINT& point : graphic.points )
                    {
                        point.x += groupOrigin.x;
                        point.y += groupOrigin.y;
                    }

                    if( graphic.kind == MODEL_GRAPHIC_KIND::ARC )
                    {
                        graphic.arcBoundsStart.x += groupOrigin.x;
                        graphic.arcBoundsStart.y += groupOrigin.y;
                        graphic.arcBoundsEnd.x += groupOrigin.x;
                        graphic.arcBoundsEnd.y += groupOrigin.y;
                        graphic.arcCenter.x += groupOrigin.x;
                        graphic.arcCenter.y += groupOrigin.y;
                    }

                    graphic.properties.push_back( sourceProperty( wxS( "page_graphic_group" ), name.text, source ) );

                    if( worksheetCandidate )
                        graphic.properties.push_back( sourceProperty( wxS( "worksheet_group" ), name.text, source ) );

                    if( !hasCircularTextOwnership )
                    {
                        SOURCE_PROVENANCE relationshipSource = source;
                        relationshipSource.absoluteOffset += 64;
                        relationshipSource.length = 4;
                        SOURCE_PROPERTY relationship = sourceProperty(
                                wxS( "preserved_drawing_text_relationship" ),
                                wxString::Format( wxS( "%u,%u" ), groupTextCount, groupLastText ), relationshipSource );
                        relationship.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
                        aModel.diagnostics.push_back( MakePropertyDiagnostic(
                                RPT_SEVERITY_WARNING, relationship,
                                wxS( "unsupported class-zero drawing text relationship preserved" ) ) );
                        graphic.properties.push_back( std::move( relationship ) );
                    }
                    aModel.graphics.push_back(
                            { graphicSource, { aModel.sheets[aSheetIndex].id, graphicSource }, std::move( graphic ) } );
                }

                if( hasCircularTextOwnership )
                    aPageGraphicRecords.push_back( record );
                continue;
            }

            if( objectClass != 0x06 )
                continue;

            MODEL_SYMBOL_DEFINITION definition;
            definition.id = DEFINITION_ID( aLayout.definitionIdBase + record );
            definition.source = source;
            definition.name = std::move( name );
            const uint32_t firstPiece = aLayout.definitionPieceStart[record];
            const uint32_t pieceEnd = record + 1 < aLayout.definitionPieceStart.size()
                                              ? aLayout.definitionPieceStart[record + 1]
                                              : aControllers.pools[3].count;

            for( size_t piece = firstPiece; piece < pieceEnd; ++piece )
            {
                const size_t   pieceOffset = aLayout.pieceBase + piece * SYMBOL_PIECE_BYTES;
                const uint8_t  pieceKind = aCursor.U8At( pieceOffset );
                const uint16_t pointCount = aCursor.U16At( pieceOffset + 2 );
                const uint32_t firstVertex = aLayout.pieceVertexStart[piece];

                if( firstVertex + pointCount > vertexEnd )
                    throwDecodeError( source, wxS( "symbol piece crosses its definition vertex slice" ) );

                SOURCE_PROVENANCE graphicSource =
                        sourceAt( aSourceName, aModel.version, wxS( "symbol graphic" ), 4, piece, pieceOffset,
                                  SYMBOL_PIECE_BYTES, static_cast<int>( aSheetIndex ) );
                MODEL_GRAPHIC graphic;
                graphic.source = graphicSource;
                decodeGraphicStrokeWidth( aCursor.U8At( pieceOffset + 4 ), aCursor.U8At( pieceOffset + 5 ),
                                          graphicSource, graphic, aModel.diagnostics );
                graphic.lineStyle = MODEL_LINE_STYLE::SOLID;

                switch( pieceKind )
                {
                case 0: graphic.kind = pointCount == 2 ? MODEL_GRAPHIC_KIND::LINE : MODEL_GRAPHIC_KIND::POLYLINE; break;
                case 1: graphic.kind = MODEL_GRAPHIC_KIND::POLYLINE; break;
                case 2: graphic.kind = MODEL_GRAPHIC_KIND::CIRCLE; break;
                case 4:
                    graphic.kind = MODEL_GRAPHIC_KIND::POLYLINE;
                    graphic.fill = MODEL_FILL_STYLE::FILLED;
                    break;
                default:
                    PADS_SCH_BINARY_PARSER::RecordUnknownEnum( wxS( "symbol graphic kind" ), pieceKind, graphicSource,
                                                               aModel.diagnostics );
                    graphic.kind = MODEL_GRAPHIC_KIND::POLYLINE;
                    break;
                }

                for( size_t point = 0; point < pointCount; ++point )
                {
                    const size_t      pointOffset = aLayout.vertexBase + ( firstVertex + point ) * SYMBOL_VERTEX_BYTES;
                    SOURCE_PROVENANCE pointSource =
                            sourceAt( aSourceName, aModel.version, wxS( "symbol vertex" ), 5, firstVertex + point,
                                      pointOffset, SYMBOL_VERTEX_BYTES, static_cast<int>( aSheetIndex ) );
                    graphic.points.push_back( { decodeLocalCoordinate( aCursor.U16At( pointOffset ) ),
                                                decodeLocalCoordinate( aCursor.U16At( pointOffset + 2 ) ),
                                                pointSource } );
                }

                if( pieceKind == 1 && graphic.points.size() == 5 && graphic.points.front().x == graphic.points.back().x
                    && graphic.points.front().y == graphic.points.back().y )
                {
                    const auto [minX, maxX] = std::ranges::minmax( graphic.points, {}, &SOURCE_POINT::x );
                    const auto [minY, maxY] = std::ranges::minmax( graphic.points, {}, &SOURCE_POINT::y );
                    const bool cornersOnly =
                            std::ranges::all_of( graphic.points,
                                                 [&]( const SOURCE_POINT& aPoint )
                                                 {
                                                     return ( aPoint.x == minX.x || aPoint.x == maxX.x )
                                                            && ( aPoint.y == minY.y || aPoint.y == maxY.y );
                                                 } );

                    if( cornersOnly )
                    {
                        graphic.kind = MODEL_GRAPHIC_KIND::RECTANGLE;
                        graphic.points = { { minX.x, minY.y, graphicSource }, { maxX.x, maxY.y, graphicSource } };
                    }
                }

                const int16_t arcMarker = static_cast<int16_t>(
                        aCursor.U16At( aLayout.vertexBase + firstVertex * SYMBOL_VERTEX_BYTES + 4 ) );

                if( pieceKind == 0 && arcMarker >= 0 )
                {
                    if( aLayout.pieceArcRecords[piece].empty() )
                        throwDecodeError( graphicSource, wxS( "symbol arc has no controller-6 record" ) );

                    const size_t arcRecord = aLayout.pieceArcRecords[piece].front();
                    const size_t arcOffset = aLayout.arcBase + arcRecord * SYMBOL_ARC_BYTES;
                    graphic.kind = MODEL_GRAPHIC_KIND::ARC;
                    SOURCE_PROVENANCE arcSource =
                            sourceAt( aSourceName, aModel.version, wxS( "symbol arc" ), 6, arcRecord, arcOffset,
                                      SYMBOL_ARC_BYTES, static_cast<int>( aSheetIndex ) );
                    graphic.arcSweepAngle = aCursor.U16At( arcOffset );
                    graphic.arcClockwise = static_cast<int16_t>( aCursor.U16At( arcOffset + 2 ) ) < 0;
                    graphic.arcBoundsStart = { decodeLocalCoordinate( aCursor.U16At( arcOffset + 6 ) ),
                                               decodeLocalCoordinate( aCursor.U16At( arcOffset + 8 ) ), arcSource };
                    graphic.arcBoundsEnd = { decodeLocalCoordinate( aCursor.U16At( arcOffset + 10 ) ),
                                             decodeLocalCoordinate( aCursor.U16At( arcOffset + 12 ) ), arcSource };
                    graphic.arcCenter = { ( graphic.arcBoundsStart.x + graphic.arcBoundsEnd.x ) / 2,
                                          ( graphic.arcBoundsStart.y + graphic.arcBoundsEnd.y ) / 2, arcSource };
                    graphic.properties.push_back( sourceProperty(
                            wxS( "arc_direction" ),
                            graphic.arcClockwise ? wxS( "clockwise" ) : wxS( "counterclockwise" ), arcSource ) );
                    graphic.properties.push_back( sourceProperty(
                            wxS( "arc_marker" ),
                            wxString::Format( wxS( "%d" ), static_cast<int16_t>( aCursor.U16At( arcOffset + 4 ) ) ),
                            arcSource ) );
                }

                definition.graphics.push_back( std::move( graphic ) );
            }

            aModel.definitions.push_back( std::move( definition ) );
            aDefinitionsByRecord[record] = &aModel.definitions.back();
        }
    }



// PADS binary pin body-root geometry
//
// The pin CAEDECAL gives the terminal/decorator direction, but its farthest
// point is not necessarily the owning symbol's body attachment point.  KiCad
// SCH_PIN length is connection-point -> pin-root, so ray-cast from the terminal
// electrical point into the owning symbol body and use the nearest positive
// intersection.  The pin-decal-derived length remains the fallback.
struct BINARY_RAY_POINT
{
    double x = 0.0;
    double y = 0.0;
};


static BINARY_RAY_POINT binaryPinBodyDirection( const SOURCE_POINT& aVector, int aAngle, uint32_t aXym )
{
    const double radians = static_cast<double>( aAngle ) * M_PI / 1800.0;
    const double c = std::cos( radians );
    const double s = std::sin( radians );

    BINARY_RAY_POINT result;
    result.x = static_cast<double>( aVector.x ) * c - static_cast<double>( aVector.y ) * s;
    result.y = static_cast<double>( aVector.x ) * s + static_cast<double>( aVector.y ) * c;

    if( aXym & 0x1 )
        result.x = -result.x;

    if( aXym & 0x2 )
        result.y = -result.y;

    return result;
}


static double binaryCross2d( const BINARY_RAY_POINT& aA, const BINARY_RAY_POINT& aB )
{
    return aA.x * aB.y - aA.y * aB.x;
}


static bool binaryRaySegmentIntersection( const BINARY_RAY_POINT& aOrigin,
                                          const BINARY_RAY_POINT& aDir,
                                          const BINARY_RAY_POINT& aStart,
                                          const BINARY_RAY_POINT& aEnd,
                                          double& aDistance )
{
    constexpr double EPSILON = 1e-9;

    const BINARY_RAY_POINT segment{ aEnd.x - aStart.x, aEnd.y - aStart.y };
    const BINARY_RAY_POINT offset{ aStart.x - aOrigin.x, aStart.y - aOrigin.y };
    const double denominator = binaryCross2d( aDir, segment );

    if( std::abs( denominator ) <= EPSILON )
        return false;

    const double distance = binaryCross2d( offset, segment ) / denominator;
    const double u = binaryCross2d( offset, aDir ) / denominator;

    if( distance <= EPSILON || u < -EPSILON || u > 1.0 + EPSILON )
        return false;

    aDistance = distance;
    return true;
}


static double binaryNormalizeAngle( double aAngle )
{
    const double full = 2.0 * M_PI;

    while( aAngle < 0.0 )
        aAngle += full;

    while( aAngle >= full )
        aAngle -= full;

    return aAngle;
}


static bool binaryPointOnArc( const BINARY_RAY_POINT& aPoint,
                              const BINARY_RAY_POINT& aStart,
                              const BINARY_RAY_POINT& aCenter,
                              int aSweepTenths, bool aClockwise )
{
    constexpr double ANGLE_EPSILON = 1e-7;
    const double sweep = std::abs( static_cast<double>( aSweepTenths ) ) * M_PI / 1800.0;

    if( sweep >= 2.0 * M_PI - ANGLE_EPSILON )
        return true;

    if( sweep <= ANGLE_EPSILON )
        return false;

    const double startAngle = std::atan2( aStart.y - aCenter.y, aStart.x - aCenter.x );
    const double pointAngle = std::atan2( aPoint.y - aCenter.y, aPoint.x - aCenter.x );

    if( aClockwise )
        return binaryNormalizeAngle( startAngle - pointAngle ) <= sweep + ANGLE_EPSILON;

    return binaryNormalizeAngle( pointAngle - startAngle ) <= sweep + ANGLE_EPSILON;
}


template<typename ACCEPT>
static void binaryRayCircleIntersections( const BINARY_RAY_POINT& aOrigin,
                                          const BINARY_RAY_POINT& aDir,
                                          const BINARY_RAY_POINT& aCenter,
                                          double aRadius, ACCEPT aAccept )
{
    constexpr double EPSILON = 1e-9;

    if( aRadius <= EPSILON )
        return;

    const BINARY_RAY_POINT rel{ aOrigin.x - aCenter.x, aOrigin.y - aCenter.y };
    const double projection = rel.x * aDir.x + rel.y * aDir.y;
    const double constant = rel.x * rel.x + rel.y * rel.y - aRadius * aRadius;
    const double discriminant = projection * projection - constant;

    if( discriminant < -EPSILON )
        return;

    const double root = std::sqrt( std::max( 0.0, discriminant ) );
    const double distances[2] = { -projection - root, -projection + root };

    for( double distance : distances )
    {
        if( distance <= EPSILON )
            continue;

        const BINARY_RAY_POINT point{ aOrigin.x + distance * aDir.x,
                                      aOrigin.y + distance * aDir.y };

        aAccept( distance, point );
    }
}


static bool binaryPinBodyRootDistance( const MODEL_SYMBOL_DEFINITION& aSymbol,
                                       const MODEL_PIN_DEFINITION& aPin,
                                       double& aDistance )
{
    constexpr double EPSILON = 1e-9;

    if( !aPin.pinDecalGeometryResolved )
        return false;

    BINARY_RAY_POINT dir = binaryPinBodyDirection( aPin.pinDecalVector, aPin.angle, aPin.xym );
    const double dirLength = std::hypot( dir.x, dir.y );

    if( dirLength <= EPSILON )
        return false;

    dir.x /= dirLength;
    dir.y /= dirLength;

    const BINARY_RAY_POINT origin{ static_cast<double>( aPin.position.x ),
                                   static_cast<double>( aPin.position.y ) };

    bool found = false;
    double best = 0.0;

    auto consider = [&]( double aCandidate )
    {
        if( aCandidate <= EPSILON )
            return;

        if( !found || aCandidate < best )
        {
            best = aCandidate;
            found = true;
        }
    };

    auto considerSegment = [&]( const SOURCE_POINT& aStart, const SOURCE_POINT& aEnd )
    {
        const BINARY_RAY_POINT start{ static_cast<double>( aStart.x ),
                                      static_cast<double>( aStart.y ) };
        const BINARY_RAY_POINT end{ static_cast<double>( aEnd.x ),
                                    static_cast<double>( aEnd.y ) };
        double distance = 0.0;

        if( binaryRaySegmentIntersection( origin, dir, start, end, distance ) )
            consider( distance );
    };

    for( const MODEL_GRAPHIC& graphic : aSymbol.graphics )
    {
        if( graphic.kind == MODEL_GRAPHIC_KIND::TEXT )
            continue;

        if( graphic.kind == MODEL_GRAPHIC_KIND::RECTANGLE && graphic.points.size() >= 2 )
        {
            const double minX = std::min( static_cast<double>( graphic.points[0].x ),
                                          static_cast<double>( graphic.points[1].x ) );
            const double maxX = std::max( static_cast<double>( graphic.points[0].x ),
                                          static_cast<double>( graphic.points[1].x ) );
            const double minY = std::min( static_cast<double>( graphic.points[0].y ),
                                          static_cast<double>( graphic.points[1].y ) );
            const double maxY = std::max( static_cast<double>( graphic.points[0].y ),
                                          static_cast<double>( graphic.points[1].y ) );

            const BINARY_RAY_POINT p0{ minX, minY };
            const BINARY_RAY_POINT p1{ maxX, minY };
            const BINARY_RAY_POINT p2{ maxX, maxY };
            const BINARY_RAY_POINT p3{ minX, maxY };

            double distance = 0.0;

            if( binaryRaySegmentIntersection( origin, dir, p0, p1, distance ) )
                consider( distance );

            if( binaryRaySegmentIntersection( origin, dir, p1, p2, distance ) )
                consider( distance );

            if( binaryRaySegmentIntersection( origin, dir, p2, p3, distance ) )
                consider( distance );

            if( binaryRaySegmentIntersection( origin, dir, p3, p0, distance ) )
                consider( distance );

            continue;
        }

        if( graphic.kind == MODEL_GRAPHIC_KIND::CIRCLE && graphic.points.size() >= 2 )
        {
            const BINARY_RAY_POINT p0{ static_cast<double>( graphic.points[0].x ),
                                       static_cast<double>( graphic.points[0].y ) };
            const BINARY_RAY_POINT p1{ static_cast<double>( graphic.points[1].x ),
                                       static_cast<double>( graphic.points[1].y ) };
            const BINARY_RAY_POINT center{ ( p0.x + p1.x ) / 2.0,
                                           ( p0.y + p1.y ) / 2.0 };
            const double radius = std::hypot( p1.x - p0.x, p1.y - p0.y ) / 2.0;

            binaryRayCircleIntersections(
                    origin, dir, center, radius,
                    [&]( double aCandidate, const BINARY_RAY_POINT& )
                    {
                        consider( aCandidate );
                    } );
            continue;
        }

        if( graphic.kind == MODEL_GRAPHIC_KIND::ARC && graphic.points.size() >= 2 )
        {
            const SOURCE_POINT& startSource = graphic.points.front();
            const BINARY_RAY_POINT start{ static_cast<double>( startSource.x ),
                                          static_cast<double>( startSource.y ) };
            const BINARY_RAY_POINT center{ static_cast<double>( graphic.arcCenter.x ),
                                           static_cast<double>( graphic.arcCenter.y ) };
            const double radius = std::hypot( start.x - center.x, start.y - center.y );

            binaryRayCircleIntersections(
                    origin, dir, center, radius,
                    [&]( double aCandidate, const BINARY_RAY_POINT& aPoint )
                    {
                        if( binaryPointOnArc( aPoint, start, center, graphic.arcSweepAngle,
                                              graphic.arcClockwise ) )
                        {
                            consider( aCandidate );
                        }
                    } );
            continue;
        }

        if( graphic.points.size() < 2 )
            continue;

        for( size_t i = 0; i + 1 < graphic.points.size(); ++i )
            considerSegment( graphic.points[i], graphic.points[i + 1] );
    }

    if( found )
        aDistance = best;

    return found;
}

    void decodeUsedDecals( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                           const SHEET_CONTROLLERS& aControllers, const DEFINITION_LAYOUT& aLayout, size_t aSheetIndex,
                           const wxString&                              aSourceName,
                           const std::vector<MODEL_SYMBOL_DEFINITION*>& aDefinitionsByRecord,
                           std::vector<USED_DECAL>& aUsedDecals, std::vector<USED_DECAL*>& aSemanticDecals,
                           PADS_SCH_MODEL& aModel )
    {
        for( size_t record = 0; record < aUsedDecals.size(); ++record )
        {
            const size_t      offset = aLayout.usedDecalBase + record * USED_DECAL_BYTES;
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "used decal" ), 7, record, offset,
                                                 USED_DECAL_BYTES, static_cast<int>( aSheetIndex ) );
            SOURCE_PROVENANCE nameSource = source;
            nameSource.length = 40;
            SOURCE_STRING name = decodeFixedString( aBytes, offset, 40, nameSource, aModel.diagnostics );
            USED_DECAL&   decal = aUsedDecals[record];
            decal.record = record;
            decal.terminalCount = aCursor.U8At( offset + 42 );
            decal.terminalStart = aCursor.U16At( offset + 44 );
            decal.definitionRecord = aCursor.U32At( offset + 48 );
            decal.fieldStart = aCursor.U32At( offset + 52 );

            if( name.text.empty() || decal.definitionRecord == 0xFFFFFFFF )
                continue;

            if( decal.definitionRecord >= aDefinitionsByRecord.size() )
                throwDecodeError( source, wxS( "unresolved symbol definition reference" ) );

            const size_t      definitionOffset = aLayout.symbolBase + decal.definitionRecord * SYMBOL_RECORD_BYTES;
            SOURCE_PROVENANCE targetNameSource =
                    sourceAt( aSourceName, aModel.version, wxS( "symbol definition" ), 3, decal.definitionRecord,
                              definitionOffset, 38, static_cast<int>( aSheetIndex ) );
            SOURCE_STRING targetName =
                    decodeFixedString( aBytes, definitionOffset, 38, targetNameSource, aModel.diagnostics );

            if( targetName.text != name.text )
            {
                const uint8_t targetClass = aCursor.U8At( definitionOffset + 0x29 );
                throwDecodeError( source, targetClass == 0x06 ? wxS( "used-decal handle name mismatch" )
                                                              : wxS( "used-decal handle targets wrong object class" ) );
            }

            decal.definition = aDefinitionsByRecord[decal.definitionRecord];

            if( !decal.definition )
                continue;

            const uint16_t embeddedTextCount =
                    aCursor.U16At( aLayout.symbolBase + decal.definitionRecord * SYMBOL_RECORD_BYTES + 0x40 );

            if( embeddedTextCount != 0 && decal.fieldStart > aLayout.textHeap.recordCount
                && decal.fieldStart < 0x80000000 )
                throwDecodeError( source, wxS( "embedded definition text handle leaves controller 1" ) );

            if( static_cast<uint32_t>( decal.terminalStart ) + decal.terminalCount > aControllers.pools[7].count )
                throwDecodeError( source, wxS( "used-decal terminal slice leaves controller 8" ) );

            aSemanticDecals.push_back( &decal );

            for( size_t pin = 0; pin < decal.terminalCount; ++pin )
            {
                const size_t      terminalRecord = decal.terminalStart + pin;
                const size_t      terminalOffset = aLayout.terminalBase + terminalRecord * TERMINAL_BYTES;
                SOURCE_PROVENANCE pinSource =
                        sourceAt( aSourceName, aModel.version, wxS( "symbol pin" ), 8, terminalRecord, terminalOffset,
                                  TERMINAL_BYTES, static_cast<int>( aSheetIndex ) );
                const uint16_t       pinDecalHandle = aCursor.U16At( terminalOffset );
                MODEL_PIN_DEFINITION definitionPin;
                definitionPin.id = PIN_ID( aLayout.pinIdBase + terminalRecord );
                definitionPin.source = pinSource;
                definitionPin.position = { decodeTerminalCoordinate( aCursor.U16At( terminalOffset + 2 ) ),
                                           decodeTerminalCoordinate( aCursor.U16At( terminalOffset + 4 ) ), pinSource };
                // PADS terminal words 3-6 are text positions, not text dimensions.  The
                // owning used-decal header carries pin-number and pin-name height/line-width pairs.
                // Ordinary CAEDECAL terminals remain drawable; text visibility is independent.
                definitionPin.presentation.source = pinSource;
                definitionPin.presentation.visible = true;
                SOURCE_PROVENANCE numberPresentationSource = source;
                numberPresentationSource.absoluteOffset = offset + 92;
                numberPresentationSource.length = 2;
                SOURCE_PROVENANCE namePresentationSource = source;
                namePresentationSource.absoluteOffset = offset + 94;
                namePresentationSource.length = 2;

                definitionPin.numberPresentation.source = numberPresentationSource;
                definitionPin.numberPresentation.height =
                        static_cast<int64_t>( aCursor.U16At( offset + 92 ) ) * 2;
                definitionPin.numberPresentation.width =
                        static_cast<int64_t>( aCursor.U8At( offset + 98 ) ) * 2;
                definitionPin.namePresentation.source = namePresentationSource;
                definitionPin.namePresentation.height =
                        static_cast<int64_t>( aCursor.U16At( offset + 94 ) ) * 2;
                definitionPin.namePresentation.width =
                        static_cast<int64_t>( aCursor.U8At( offset + 99 ) ) * 2;

                SOURCE_PROVENANCE numberFontSource = source;
                numberFontSource.absoluteOffset = offset + 104;
                numberFontSource.length = 2;
                SOURCE_PROVENANCE nameFontSource = source;
                nameFontSource.absoluteOffset = offset + 106;
                nameFontSource.length = 2;
                const int16_t numberFontHandle =
                        static_cast<int16_t>( aCursor.U16At( numberFontSource.absoluteOffset ) );
                const int16_t nameFontHandle =
                        static_cast<int16_t>( aCursor.U16At( nameFontSource.absoluteOffset ) );
                definitionPin.numberPresentation.font = decodedDefinitionFont( numberFontHandle, numberFontSource );
                definitionPin.namePresentation.font = decodedDefinitionFont( nameFontHandle, nameFontSource );

                if( aModel.settings.fontMode == MODEL_FONT_MODE::SYSTEM
                    && !aModel.settings.defaultSystemFont.text.IsEmpty() )
                {
                    if( numberFontHandle == -1 || numberFontHandle == -4 )
                        definitionPin.numberPresentation.font = aModel.settings.defaultSystemFont;

                    if( nameFontHandle == -1 || nameFontHandle == -4 )
                        definitionPin.namePresentation.font = aModel.settings.defaultSystemFont;
                }

                definitionPin.numberPresentation.visible = definitionPin.numberPresentation.height != 0;
                definitionPin.namePresentation.visible = definitionPin.namePresentation.height != 0
                                                         && ( aCursor.U16At( terminalOffset + 24 ) & 0x8000 ) == 0;

                // The first T-line pair locates the pin number; the second locates the pin name.
                definitionPin.numberOffset = { decodeLocalCoordinate( aCursor.U16At( terminalOffset + 6 ) ),
                                               decodeLocalCoordinate( aCursor.U16At( terminalOffset + 8 ) ),
                                               pinSource };
                definitionPin.nameOffset = { decodeLocalCoordinate( aCursor.U16At( terminalOffset + 10 ) ),
                                             decodeLocalCoordinate( aCursor.U16At( terminalOffset + 12 ) ), pinSource };
                const uint16_t presentationFlags = aCursor.U16At( terminalOffset + 22 );
                const uint16_t visibilityFlags = aCursor.U16At( terminalOffset + 24 );
                definitionPin.presentationFlags = presentationFlags;
                definitionPin.visibilityAndNumberPresentationFlags = visibilityFlags;

                // PADS binary stores the same terminal-local X/Y mirror state as
                // the ASCII T-line xym field, shifted left by one because bit 0
                // carries the terminal quarter-turn flag.
                definitionPin.xym = ( presentationFlags & 0x0006 ) >> 1;

                definitionPin.angle = ( presentationFlags & 0x0001 ) != 0 ? 900 : 0;
                definitionPin.numberAngle = ( presentationFlags & 0x0100 ) != 0 ? 900 : 0;
                definitionPin.nameAngle = ( visibilityFlags & 0x0001 ) != 0 ? 900 : 0;

                definitionPin.numberJustification =
                        terminalJustification( ( presentationFlags >> 12 ) & 0x0F, definitionPin.numberAngle != 0 );

                if( ( presentationFlags & 0x00F8 ) != 0 )
                    PADS_SCH_BINARY_PARSER::RecordUnknownEnum( wxS( "terminal side" ), presentationFlags, pinSource,
                                                               aModel.diagnostics );

                definitionPin.nameJustification =
                        terminalJustification( ( visibilityFlags >> 4 ) & 0x0F, definitionPin.nameAngle != 0 );
                const uint16_t nameOffsetFlags = visibilityFlags & 0x0F00;
                definitionPin.nameOffsetAngle = ( nameOffsetFlags & 0x0100 ) != 0 ? 900 : 0;
                definitionPin.numberOffsetAngle = ( nameOffsetFlags & 0x0200 ) != 0 ? 900 : 0;

                switch( visibilityFlags & 0x0F00 )
                {
                case 0x0000:
                case 0x0800: definitionPin.nameOffsetJustification = 0; break;
                case 0x0400:
                case 0x0C00: definitionPin.nameOffsetJustification = 1; break;
                case 0x0500:
                case 0x0D00:
                case 0x0F00: definitionPin.nameOffsetJustification = 2; break;
                default:
                    PADS_SCH_BINARY_PARSER::RecordUnknownEnum( wxS( "terminal name-offset presentation" ),
                                                               visibilityFlags & 0x0F00, pinSource,
                                                               aModel.diagnostics );
                    break;
                }

                switch( visibilityFlags & 0x0F00 )
                {
                case 0x0000: definitionPin.numberOffsetJustification = 0; break;
                case 0x0800:
                case 0x0C00: definitionPin.numberOffsetJustification = 1; break;
                case 0x0D00: definitionPin.numberOffsetJustification = 8; break;
                case 0x0F00: definitionPin.numberOffsetJustification = 2; break;
                default:
                    PADS_SCH_BINARY_PARSER::RecordUnknownEnum( wxS( "terminal number-offset presentation" ),
                                                               visibilityFlags & 0x0F00, pinSource,
                                                               aModel.diagnostics );
                    break;
                }

                definitionPin.visibilityFlags = ( visibilityFlags >> 8 ) & 0x00C0;

                if( pinDecalHandle == 0xFFFF )
                {
                    definitionPin.length = 0;
                }
                else if( pinDecalHandle >= aUsedDecals.size() )
                    throwDecodeError( pinSource, wxS( "unresolved pin-decal handle" ) );

                if( pinDecalHandle != 0xFFFF )
                {
                    MODEL_SYMBOL_DEFINITION* pinDecal = aUsedDecals[pinDecalHandle].definition;

                    if( !pinDecal )
                    {
                        const size_t   handleOffset = aLayout.usedDecalBase + pinDecalHandle * USED_DECAL_BYTES;
                        const uint32_t definitionRecord = aCursor.U32At( handleOffset + 48 );

                        if( definitionRecord >= aDefinitionsByRecord.size() || !aDefinitionsByRecord[definitionRecord] )
                            throwDecodeError( pinSource, wxS( "unresolved pin-decal handle" ) );

                        pinDecal = aDefinitionsByRecord[definitionRecord];
                    }

                    const wxString pinDecalName = pinDecal->name.text;
                    definitionPin.decalName = pinDecal->name;

                    int64_t      bestDistanceSquared = 0;
                    SOURCE_POINT bestVector;
                    int64_t      bestStrokeWidth = 0;
                    MODEL_LINE_STYLE bestLineStyle = MODEL_LINE_STYLE::DEFAULT;

                    for( const MODEL_GRAPHIC& pinGraphic : pinDecal->graphics )
                    {
                        if( pinGraphic.kind != MODEL_GRAPHIC_KIND::LINE
                            && pinGraphic.kind != MODEL_GRAPHIC_KIND::POLYLINE )
                        {
                            continue;
                        }

                        const bool containsOrigin = std::ranges::any_of(
                                pinGraphic.points,
                                []( const SOURCE_POINT& aPoint )
                                {
                                    return aPoint.x == 0 && aPoint.y == 0;
                                } );

                        if( !containsOrigin )
                            continue;

                        for( const SOURCE_POINT& point : pinGraphic.points )
                        {
                            const int64_t distanceSquared = point.x * point.x + point.y * point.y;

                            if( distanceSquared > bestDistanceSquared )
                            {
                                bestDistanceSquared = distanceSquared;
                                bestVector = point;
                                bestStrokeWidth = pinGraphic.strokeWidth;
                                bestLineStyle = pinGraphic.lineStyle;
                            }
                        }
                    }

                    if( bestDistanceSquared != 0 )
                    {
                        definitionPin.length = static_cast<int64_t>(
                                std::llround( std::sqrt( static_cast<double>( bestDistanceSquared ) ) ) );
                        definitionPin.pinDecalVector = bestVector;
                        definitionPin.pinDecalStrokeWidth = bestStrokeWidth;
                        definitionPin.pinDecalLineStyle = bestLineStyle;
                        definitionPin.pinDecalGeometryResolved = true;

                        // Prefer the actual terminal -> owning-symbol body root.
                        // The pin-decal-derived length above remains the fallback.
                        double bodyRootDistance = 0.0;

                        if( binaryPinBodyRootDistance( *decal.definition, definitionPin, bodyRootDistance ) )
                        {
                            definitionPin.length =
                                    static_cast<int64_t>( std::llround( bodyRootDistance ) );
                        }
                    }
                    else
                    {
                        SOURCE_PROPERTY lengthProperty =
                                sourceProperty( wxS( "pin_length" ), wxS( "unknown" ), pinSource );
                        lengthProperty.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
                        definitionPin.properties.push_back( std::move( lengthProperty ) );
                    }

                    const bool inverted = pinDecalName == wxS( "PINB" ) || pinDecalName == wxS( "PINORB" )
                                          || pinDecalName == wxS( "PCLKB" ) || pinDecalName == wxS( "PINIEB" );
                    const bool clock = pinDecalName == wxS( "PCLK" ) || pinDecalName == wxS( "PCLKB" );
                    definitionPin.graphicStyle = ( inverted ? 1U : 0U ) | ( clock ? 2U : 0U );
                }

                definitionPin.properties.push_back( sourceProperty(
                        wxS( "pin_name_height_half_mils" ),
                        wxString::Format( wxS( "%lld" ), definitionPin.presentation.height ), pinSource ) );
                definitionPin.properties.push_back( sourceProperty(
                        wxS( "pin_number_height_half_mils" ),
                        wxString::Format( wxS( "%lld" ), definitionPin.numberPresentation.height ), pinSource ) );
                definitionPin.properties.push_back( sourceProperty(
                        wxS( "pin_name_width_half_mils" ),
                        wxString::Format( wxS( "%lld" ), definitionPin.namePresentation.width ), pinSource ) );
                definitionPin.properties.push_back( sourceProperty(
                        wxS( "pin_number_width_half_mils" ),
                        wxString::Format( wxS( "%lld" ), definitionPin.numberPresentation.width ), pinSource ) );
                definitionPin.properties.push_back(
                        sourceProperty( wxS( "terminal_xym" ),
                                        wxString::Format( wxS( "%u" ), definitionPin.xym ), pinSource ) );
                decal.definition->pins.push_back( std::move( definitionPin ) );
            }
        }
    }


    std::vector<USED_DECAL*> buildFieldDecals( const std::vector<USED_DECAL*>& aSemanticDecals,
                                               const DEFINITION_LAYOUT& aLayout, size_t aSheetIndex,
                                               const wxString& aSourceName, uint16_t aVersion )
    {
        std::vector<USED_DECAL*> fieldDecals;

        std::ranges::copy_if( aSemanticDecals, std::back_inserter( fieldDecals ),
                              [&]( const USED_DECAL* aDecal )
                              {
                                  return aDecal->fieldStart < aLayout.textHeap.recordCount;
                              } );
        std::ranges::sort( fieldDecals,
                           []( const USED_DECAL* aLeft, const USED_DECAL* aRight )
                           {
                               return aLeft->fieldStart < aRight->fieldStart;
                           } );

        for( size_t i = 1; i < fieldDecals.size(); ++i )
        {
            if( fieldDecals[i - 1]->fieldStart != fieldDecals[i]->fieldStart )
                continue;

            SOURCE_PROVENANCE first =
                    sourceAt( aSourceName, aVersion, wxS( "used decal" ), 7, fieldDecals[i - 1]->record,
                              aLayout.usedDecalBase + fieldDecals[i - 1]->record * USED_DECAL_BYTES, USED_DECAL_BYTES,
                              static_cast<int>( aSheetIndex ) );
            SOURCE_PROVENANCE duplicate =
                    sourceAt( aSourceName, aVersion, wxS( "used decal" ), 7, fieldDecals[i]->record,
                              aLayout.usedDecalBase + fieldDecals[i]->record * USED_DECAL_BYTES, USED_DECAL_BYTES,
                              static_cast<int>( aSheetIndex ) );
            throwDecodeError(
                    duplicate,
                    wxString::Format( wxS( "duplicate field ID; first at v0x%04X %s controller %d record %llu "
                                           "sheet %d offset 0x%llX" ),
                                      first.version, first.objectClass, first.controller,
                                      static_cast<unsigned long long>( first.recordIndex ), first.sheet,
                                      static_cast<unsigned long long>( first.absoluteOffset ) ) );
        }

        return fieldDecals;
    }


    void decodePageGraphics( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                             const DEFINITION_LAYOUT& aLayout, size_t aSheetIndex, const wxString& aSourceName,
                             const std::vector<size_t>& aPageGraphicRecords, PADS_SCH_MODEL& aModel )
    {
        for( size_t record : aPageGraphicRecords )
        {
            const size_t      offset = aLayout.symbolBase + record * SYMBOL_RECORD_BYTES;
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "page graphic group" ), 3, record,
                                                 offset, SYMBOL_RECORD_BYTES, static_cast<int>( aSheetIndex ) );
            SOURCE_PROVENANCE nameSource = source;
            nameSource.length = 38;
            SOURCE_STRING  groupName = decodeFixedString( aBytes, offset, 38, nameSource, aModel.diagnostics );
            const uint16_t textCountForGroup = aCursor.U16At( offset + 64 );
            const uint16_t lastTextRecord = aCursor.U16At( offset + 66 );
            const bool     worksheetCandidate = aCursor.U16At( offset + 42 ) == 69 && textCountForGroup == 58;

            if( textCountForGroup > aLayout.textHeap.recordCount
                || ( textCountForGroup != 0 && lastTextRecord >= aLayout.textHeap.recordCount ) )
                throwDecodeError( source, wxS( "page-text ownership leaves controller 1" ) );

            MODEL_SYMBOL_DEFINITION textOwner;
            textOwner.source = source;

            if( textCountForGroup != 0 )
            {
                std::vector<size_t> textRecords( textCountForGroup );
                std::vector<bool>   visitedTextRecords( aLayout.textHeap.recordCount, false );
                size_t              textRecord = lastTextRecord;

                for( size_t reverseIndex = textCountForGroup; reverseIndex != 0; --reverseIndex )
                {
                    if( textRecord >= aLayout.textHeap.recordCount )
                        throwDecodeError( source, wxS( "page-text predecessor leaves controller 1" ) );

                    if( visitedTextRecords[textRecord] )
                        throwDecodeError( source, wxS( "page-text predecessor repeats controller 1 record" ) );

                    visitedTextRecords[textRecord] = true;

                    textRecords[reverseIndex - 1] = textRecord;

                    if( reverseIndex != 1 )
                    {
                        const size_t predecessor =
                                aCursor.U16At( aLayout.textHeap.recordBase + textRecord * TEXT_RECORD_BYTES + 24 );

                        if( predecessor >= aLayout.textHeap.recordCount )
                            throwDecodeError( source, wxS( "page-text predecessor leaves controller 1" ) );

                        textRecord = predecessor;
                    }
                }

                for( size_t textRecordIndex : textRecords )
                    decodeDefinitionTextRecord( aBytes, aCursor, aSourceName, aSheetIndex, aLayout.textHeap,
                                                textRecordIndex, TEXT_ROLE::PAGE_TEXT, textOwner, aModel );
            }

            if( textOwner.graphics.size() != textCountForGroup )
                throwDecodeError( source, wxS( "page-text records do not exactly match declared count" ) );

            for( MODEL_GRAPHIC& graphic : textOwner.graphics )
            {
                graphic.source.objectClass = wxS( "page text" );
                const SOURCE_POINT groupOrigin{ decodeCoordinate( aCursor.U16At( offset + 60 ) ),
                                                decodeCoordinate( aCursor.U16At( offset + 62 ) ), source };

                for( SOURCE_POINT& point : graphic.points )
                {
                    point.x += groupOrigin.x;
                    point.y += groupOrigin.y;
                }

                graphic.properties.push_back( sourceProperty( wxS( "page_graphic_group" ), groupName.text, source ) );

                if( worksheetCandidate )
                    graphic.properties.push_back( sourceProperty( wxS( "worksheet_group" ), groupName.text, source ) );

                aModel.graphics.push_back(
                        { graphic.source, { aModel.sheets[aSheetIndex].id, graphic.source }, std::move( graphic ) } );
            }

            auto belongsToGroup = [&]( const MODEL_PAGE_GRAPHIC& aGraphic )
            {
                if( aGraphic.sheet.id != aModel.sheets[aSheetIndex].id )
                    return false;

                return std::ranges::any_of( aGraphic.graphic.properties,
                                            [&]( const SOURCE_PROPERTY& aProperty )
                                            {
                                                return aProperty.name.text == wxS( "page_graphic_group" )
                                                       && aProperty.value.text == groupName.text;
                                            } );
            };

            size_t numericEdgeMarkers = 0;
            size_t alphabeticEdgeMarkers = 0;
            size_t textGraphics = 0;
            size_t drawingGraphics = 0;
            bool   hasTitleAnchor = false;
            bool   hasSheetAnchor = false;
            bool   hasRevisionAnchor = false;

            for( const MODEL_PAGE_GRAPHIC& pageGraphic : aModel.graphics )
            {
                if( !belongsToGroup( pageGraphic ) )
                    continue;

                if( pageGraphic.graphic.kind != MODEL_GRAPHIC_KIND::TEXT )
                {
                    ++drawingGraphics;
                    continue;
                }

                ++textGraphics;
                wxString text = pageGraphic.graphic.text.text.Upper();

                if( text.length() == 1 && text[0] >= '0' && text[0] <= '9' )
                    ++numericEdgeMarkers;

                if( text.length() == 1 && text[0] >= 'A' && text[0] <= 'Z' )
                    ++alphabeticEdgeMarkers;

                hasTitleAnchor |= text == wxS( "TITLE" ) || text.StartsWith( wxS( "TITLE:" ) );
                hasSheetAnchor |= text == wxS( "SHEET NUMBER" ) || text == wxS( "NUMBER OF SHEETS" )
                                  || text.StartsWith( wxS( "SHEET:" ) );
                hasRevisionAnchor |= text == wxS( "REVISION" ) || text.StartsWith( wxS( "REV:" ) )
                                     || text.StartsWith( wxS( "REVISION " ) );
            }

            const bool worksheet = drawingGraphics >= 30 && textGraphics >= 30 && numericEdgeMarkers >= 4
                                   && alphabeticEdgeMarkers >= 4 && hasTitleAnchor && hasSheetAnchor
                                   && hasRevisionAnchor;

            if( worksheet )
            {
                MODEL_WORKSHEET modelWorksheet;
                modelWorksheet.source = source;
                modelWorksheet.sheet = { aModel.sheets[aSheetIndex].id, source };
                modelWorksheet.name = groupName;

                for( auto graphic = aModel.graphics.begin(); graphic != aModel.graphics.end(); )
                {
                    if( belongsToGroup( *graphic ) )
                    {
                        modelWorksheet.graphics.push_back( std::move( graphic->graphic ) );
                        graphic = aModel.graphics.erase( graphic );
                    }
                    else
                    {
                        ++graphic;
                    }
                }

                auto existing = std::ranges::find_if( aModel.worksheets,
                                                      [&]( const MODEL_WORKSHEET& aWorksheet )
                                                      {
                                                          return aWorksheet.sheet.id == modelWorksheet.sheet.id;
                                                      } );

                if( existing == aModel.worksheets.end() )
                {
                    aModel.worksheets.push_back( std::move( modelWorksheet ) );
                }
                else if( !sameWorksheetValue( *existing, modelWorksheet ) )
                {
                    SOURCE_PROPERTY distinct =
                            sourceProperty( wxS( "distinct_worksheet_layout" ), groupName.text, source );
                    distinct.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
                    aModel.diagnostics.push_back( MakePropertyDiagnostic(
                            RPT_SEVERITY_WARNING, distinct,
                            wxS( "distinct worksheet layout preserved as schematic page graphics" ) ) );

                    for( MODEL_GRAPHIC& graphic : modelWorksheet.graphics )
                    {
                        graphic.properties.push_back( distinct );
                        aModel.graphics.push_back( { graphic.source,
                                                     { aModel.sheets[aSheetIndex].id, graphic.source },
                                                     std::move( graphic ) } );
                    }
                }
            }
        }

        auto canonicalWorksheet = std::ranges::find_if( aModel.worksheets,
                                                        []( const MODEL_WORKSHEET& aWorksheet )
                                                        {
                                                            return aWorksheet.name.text == wxS( "DRW5982" );
                                                        } );

        if( canonicalWorksheet != aModel.worksheets.end() )
        {
            using GROUP_KEY = std::pair<uint32_t, wxString>;
            std::map<GROUP_KEY, std::vector<const MODEL_GRAPHIC*>> candidates;
            std::vector<const MODEL_GRAPHIC*>                      canonicalGeometry;

            for( const MODEL_GRAPHIC& graphic : canonicalWorksheet->graphics )
            {
                if( graphic.kind != MODEL_GRAPHIC_KIND::TEXT )
                    canonicalGeometry.push_back( &graphic );
            }

            for( const MODEL_PAGE_GRAPHIC& pageGraphic : aModel.graphics )
            {
                auto groupProperty = std::ranges::find_if( pageGraphic.graphic.properties,
                                                           []( const SOURCE_PROPERTY& aProperty )
                                                           {
                                                               return aProperty.name.text == wxS( "worksheet_group" );
                                                           } );

                if( groupProperty != pageGraphic.graphic.properties.end()
                    && pageGraphic.graphic.kind != MODEL_GRAPHIC_KIND::TEXT )
                {
                    candidates[{ pageGraphic.sheet.id.Value(), groupProperty->value.text }].push_back(
                            &pageGraphic.graphic );
                }
            }

            std::set<GROUP_KEY> equivalentGroups;

            for( const auto& [key, geometry] : candidates )
            {
                if( geometry.size() == canonicalGeometry.size()
                    && std::ranges::equal( geometry, canonicalGeometry,
                                           [&]( const MODEL_GRAPHIC* aLeft, const MODEL_GRAPHIC* aRight )
                                           {
                                               return sameGraphicValue( *aLeft, *aRight );
                                           } ) )
                {
                    equivalentGroups.insert( key );
                }
            }

            std::erase_if( aModel.graphics,
                           [&]( const MODEL_PAGE_GRAPHIC& aGraphic )
                           {
                               return std::ranges::any_of(
                                       aGraphic.graphic.properties,
                                       [&]( const SOURCE_PROPERTY& aProperty )
                                       {
                                           return aProperty.name.text == wxS( "worksheet_group" )
                                                  && equivalentGroups.contains(
                                                          { aGraphic.sheet.id.Value(), aProperty.value.text } );
                                       } );
                           } );
        }
    }


    void decodeDefinitionFields( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                                 const DEFINITION_LAYOUT& aLayout, size_t aSheetIndex, const wxString& aSourceName,
                                 const std::vector<USED_DECAL*>& aFieldDecals, PADS_SCH_MODEL& aModel )
    {
        for( size_t i = 0; i < aFieldDecals.size(); ++i )
        {
            USED_DECAL&    decal = *aFieldDecals[i];
            const size_t   definitionOffset = aLayout.symbolBase + decal.definitionRecord * SYMBOL_RECORD_BYTES;
            const uint16_t embeddedCount = aCursor.U16At( definitionOffset + 0x40 );
            const uint32_t fieldEnd =
                    i + 1 < aFieldDecals.size() ? aFieldDecals[i + 1]->fieldStart : aLayout.textHeap.recordCount;

            if( decal.fieldStart > fieldEnd )
                throwDecodeError( decal.definition->source, wxS( "definition field slice is not monotone" ) );

            if( decal.definition->fields.empty() )
            {
                for( size_t standard = 0; standard < 2; ++standard )
                {
                    const size_t      usedOffset = aLayout.usedDecalBase + decal.record * USED_DECAL_BYTES;
                    SOURCE_PROVENANCE fieldSource =
                            sourceAt( aSourceName, aModel.version, wxS( "standard definition field" ), 7, decal.record,
                                      usedOffset + 60 + standard * 8, 8, static_cast<int>( aSheetIndex ) );
                    MODEL_FIELD field;
                    field.source = fieldSource;
                    field.name = { {},
                                   standard == 0 ? wxS( "REF-DES" ) : wxS( "PART-TYPE" ),
                                   STRING_ENCODING_STATUS::CODE_PAGE,
                                   fieldSource };
                    field.position = { decodeLocalCoordinate( aCursor.U16At( usedOffset + 60 + standard * 8 ) ),
                                       decodeLocalCoordinate( aCursor.U16At( usedOffset + 62 + standard * 8 ) ),
                                       fieldSource };
                    field.angle = NormalizeAngle( aCursor.U16At( usedOffset + 64 + standard * 8 ) );
                    field.presentation.source = fieldSource;
                    field.presentation.height =
                            static_cast<int64_t>( aCursor.U16At( usedOffset + 88 + standard * 2 ) ) * 2;
                    field.presentation.width =
                            static_cast<int64_t>( aCursor.U8At( usedOffset + 96 + standard ) ) * 2;
                    SOURCE_PROVENANCE fontSource = fieldSource;
                    fontSource.absoluteOffset = usedOffset + 100 + standard * 2;
                    fontSource.length = 2;
                    const int16_t fontHandle = static_cast<int16_t>( aCursor.U16At( fontSource.absoluteOffset ) );
                    field.presentation.font = decodedDefinitionFont( fontHandle, fontSource );
                    field.presentation.properties.push_back( sourceProperty(
                            wxS( "font_handle" ), wxString::Format( wxS( "%d" ), fontHandle ), fontSource ) );
                    const uint16_t justification = rotatedFieldJustification(
                            aCursor.U16At( usedOffset + 66 + standard * 8 ), field.angle );
                    field.presentation.horizontalJustification = horizontalJustification( justification );
                    field.presentation.verticalJustification = verticalJustification( justification );
                    decal.definition->fields.push_back( std::move( field ) );
                }
            }

            if( embeddedCount != 0 )
            {
                const uint16_t lastEmbeddedRecord = aCursor.U16At( definitionOffset + 0x42 );

                if( embeddedCount > aLayout.textHeap.recordCount || lastEmbeddedRecord >= aLayout.textHeap.recordCount )
                    throwDecodeError( decal.definition->source,
                                      wxS( "embedded definition text ownership leaves controller 1" ) );

                const size_t firstEmbeddedRecord =
                        ( static_cast<size_t>( lastEmbeddedRecord ) + aLayout.textHeap.recordCount + 1 - embeddedCount )
                        % aLayout.textHeap.recordCount;

                for( size_t textIndex = 0; textIndex < embeddedCount; ++textIndex )
                {
                    decodeDefinitionTextRecord( aBytes, aCursor, aSourceName, aSheetIndex, aLayout.textHeap,
                                                ( firstEmbeddedRecord + textIndex ) % aLayout.textHeap.recordCount,
                                                TEXT_ROLE::EMBEDDED_SYMBOL_TEXT, *decal.definition, aModel );
                }
            }

            for( size_t record = decal.fieldStart; record < fieldEnd; ++record )
                decodeDefinitionTextRecord( aBytes, aCursor, aSourceName, aSheetIndex, aLayout.textHeap, record,
                                            TEXT_ROLE::DEFINITION_FIELD, *decal.definition, aModel );
        }
    }


    void decodePartTypes( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                          const SHEET_CONTROLLERS& aControllers, const DEFINITION_LAYOUT& aLayout, size_t aSheetIndex,
                          const wxString& aSourceName, const std::vector<USED_DECAL>& aUsedDecals,
                          const CONNECTIVITY_GLOBALS* aConnectivity, PADS_SCH_MODEL& aModel )
    {
        size_t signalPinCursor = 0;

        for( size_t record = 0; record < aControllers.pools[8].count; ++record )
        {
            const size_t      offset = aLayout.partBase + record * PART_TYPE_BYTES;
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "part type" ), 9, record, offset,
                                                 PART_TYPE_BYTES, static_cast<int>( aSheetIndex ) );
            SOURCE_PROVENANCE nameSource = source;
            nameSource.length = 44;
            MODEL_PART_TYPE part;
            part.id = PART_TYPE_ID( aLayout.partIdBase + record );
            part.source = source;
            part.name = decodeFixedString( aBytes, offset, 44, nameSource, aModel.diagnostics );
            const uint32_t gateStart = aCursor.U32At( offset + 44 );
            const uint32_t pinStart = aCursor.U32At( offset + 48 );
            const uint32_t gateEnd = record + 1 < aControllers.pools[8].count
                                             ? aCursor.U32At( offset + PART_TYPE_BYTES + 44 )
                                             : aControllers.pools[9].count;
            const uint32_t pinEnd = record + 1 < aControllers.pools[8].count
                                            ? aCursor.U32At( offset + PART_TYPE_BYTES + 48 )
                                            : aControllers.pools[10].count;

            if( gateStart > gateEnd || gateEnd > aControllers.pools[9].count || pinStart > pinEnd
                || pinEnd > aControllers.pools[10].count )
            {
                throwDecodeError( source, wxS( "part-type gate or pin slice leaves its controller" ) );
            }

            if( aCursor.U16At( offset + 68 ) != gateEnd - gateStart )
                throwDecodeError( source, wxS( "stored gate count does not match controller-10 slice" ) );

            size_t         partPinCursor = pinStart;
            uint32_t       unitCursor = 0;
            const uint32_t pinNameHeapBase = aCursor.U32At( offset + 60 );

            if( pinNameHeapBase > aLayout.pinNameBytes )
                throwDecodeError( source, wxS( "part-type pin-name base leaves controller 14" ) );

            // PADS_V000D_HIERARCHICAL_GLOBAL_NET_PIN_NAMES_V5
            // A hierarchical sheet symbol ($$...) reuses the global-net directory
            // for pin names: controller-11 +0 is a global-net record index, NOT
            // an offset into the (often empty) local controller-14 string heap.
            // For all ordinary PARTTYPEs the original controller-14 logic stays.
            const bool hierarchicalNetNames = aModel.version == 0x000D
                                              && part.name.text.StartsWith( wxS( "$$" ) );

            auto decodePartPinName = [&]( uint32_t aNameRef, size_t aPinRecord,
                                          const SOURCE_PROVENANCE& aPinSource,
                                          const wxString& aNameClass ) -> SOURCE_STRING
            {
                if( aNameRef == 0xFFFFFFFF )
                    return {};

                if( hierarchicalNetNames )
                {
                    if( !aConnectivity || aNameRef >= aConnectivity->nets.size()
                        || aConnectivity->nets[aNameRef].tombstone
                        || aConnectivity->nets[aNameRef].name.text.IsEmpty() )
                    {
                        throwDecodeError( aPinSource,
                                          wxS( "hierarchical pin-name global-net handle is invalid" ) );
                    }

                    SOURCE_PROVENANCE handleSource = aPinSource;
                    handleSource.length = 4;
                    part.properties.push_back( sourceProperty(
                            wxString::Format( wxS( "hierarchical_pin_global_net_handle_%zu" ), aPinRecord ),
                            wxString::Format( wxS( "%u" ), aNameRef ), handleSource ) );
                    return aConnectivity->nets[aNameRef].name;
                }

                if( aNameRef >= aLayout.pinNameBytes - pinNameHeapBase )
                    throwDecodeError( aPinSource, wxS( "pin-name offset leaves controller 14" ) );

                const size_t namePosition = aLayout.pinNameBase + pinNameHeapBase + aNameRef;
                const size_t nameAvailable = aLayout.pinNameBytes - pinNameHeapBase - aNameRef;
                SOURCE_PROVENANCE pinNameSource = sourceAt(
                        aSourceName, aModel.version, aNameClass, 14, aPinRecord, namePosition,
                        nameAvailable, static_cast<int>( aSheetIndex ) );
                return decodeFixedString( aBytes, namePosition, nameAvailable, pinNameSource,
                                          aModel.diagnostics );
            };

            auto decodePartPin = [&]( MODEL_PIN_DEFINITION& aDefinitionPin, size_t aPinRecord, MODEL_GATE& aGate )
            {
                const size_t      pinOffset = aLayout.pinBase + aPinRecord * PIN_BYTES;
                SOURCE_PROVENANCE pinSource = sourceAt( aSourceName, aModel.version, wxS( "part pin" ), 11, aPinRecord,
                                                        pinOffset, PIN_BYTES, static_cast<int>( aSheetIndex ) );
                SOURCE_PROVENANCE numberSource = pinSource;
                numberSource.absoluteOffset += 4;
                numberSource.length = 16;
                aDefinitionPin.number =
                        decodeFixedString( aBytes, pinOffset + 4, 16, numberSource, aModel.diagnostics );
                const uint32_t nameOffset = aCursor.U32At( pinOffset );

                if( nameOffset != 0xFFFFFFFF )
                    aDefinitionPin.name = decodePartPinName( nameOffset, aPinRecord, pinSource, wxS( "pin name" ) );

                aDefinitionPin.electricalType =
                        pinElectricalType( aCursor.U8At( pinOffset + 21 ), pinSource, aModel.diagnostics );
                MODEL_GATE_PIN logicalPin;
                logicalPin.source = pinSource;
                logicalPin.definitionPin = { aDefinitionPin.id, pinSource };
                logicalPin.number = aDefinitionPin.number;
                logicalPin.name = aDefinitionPin.name;
                logicalPin.electricalType = aDefinitionPin.electricalType;
                logicalPin.swapGroup = aCursor.U8At( pinOffset + 20 );
                logicalPin.flags = aCursor.U16At( pinOffset + 22 );
                aDefinitionPin.properties.push_back(
                        sourceProperty( wxS( "swap_group" ),
                                        wxString::Format( wxS( "%u" ), aCursor.U8At( pinOffset + 20 ) ), pinSource ) );
                aGate.pins.push_back( { aDefinitionPin.id, pinSource } );
                aGate.logicalPins.push_back( std::move( logicalPin ) );
            };

            auto decodeConnectorPin = [&]( size_t aPinRecord )
            {
                const size_t      pinOffset = aLayout.pinBase + aPinRecord * PIN_BYTES;
                SOURCE_PROVENANCE pinSource =
                        sourceAt( aSourceName, aModel.version, wxS( "connector logical pin" ), 11, aPinRecord,
                                  pinOffset, PIN_BYTES, static_cast<int>( aSheetIndex ) );
                SOURCE_PROVENANCE numberSource = pinSource;
                numberSource.absoluteOffset += 4;
                numberSource.length = 16;
                MODEL_CONNECTOR_PIN pin;
                pin.source = pinSource;
                pin.number = decodeFixedString( aBytes, pinOffset + 4, 16, numberSource, aModel.diagnostics );
                const uint32_t nameOffset = aCursor.U32At( pinOffset );

                if( nameOffset != 0xFFFFFFFF )
                    pin.name = decodePartPinName( nameOffset, aPinRecord, pinSource,
                                                  wxS( "connector pin name" ) );

                pin.swapGroup = aCursor.U8At( pinOffset + 20 );
                pin.electricalType = pinElectricalType( aCursor.U8At( pinOffset + 21 ), pinSource, aModel.diagnostics );
                pin.flags = aCursor.U16At( pinOffset + 22 );
                return pin;
            };

            for( size_t gateRecord = gateStart; gateRecord < gateEnd; ++gateRecord )
            {
                const size_t      gateOffset = aLayout.gateBase + gateRecord * GATE_BYTES;
                SOURCE_PROVENANCE gateSource = sourceAt( aSourceName, aModel.version, wxS( "gate" ), 10, gateRecord,
                                                         gateOffset, GATE_BYTES, static_cast<int>( aSheetIndex ) );
                MODEL_GATE        gate;
                gate.id = GATE_ID( aLayout.gateIdBase + gateRecord );
                gate.source = gateSource;
                const uint16_t pinCount = aCursor.U16At( gateOffset + 8 );
                const uint16_t swapGroup = aCursor.U16At( gateOffset + 10 );
                const uint16_t primaryHandle = aCursor.U16At( gateOffset );

                if( pinCount == 0 )
                    continue;

                gate.unit = ++unitCursor;

                if( primaryHandle == 0xFFFF || primaryHandle >= aUsedDecals.size()
                    || !aUsedDecals[primaryHandle].definition )
                {
                    bool pinDecalGroup = primaryHandle == 0xFFFF && gateRecord + 1 < gateEnd;

                    for( size_t member = gateRecord + 1; pinDecalGroup && member < gateEnd; ++member )
                    {
                        const size_t   memberOffset = aLayout.gateBase + member * GATE_BYTES;
                        const uint16_t memberHandle = aCursor.U16At( memberOffset );
                        pinDecalGroup = aCursor.U16At( memberOffset + 8 ) == 0 && memberHandle < aUsedDecals.size()
                                        && aUsedDecals[memberHandle].definition;
                    }

                    if( !pinDecalGroup || partPinCursor + pinCount > pinEnd )
                        throwDecodeError( gateSource, wxS( "unresolved symbol definition reference" ) );

                    part.properties.push_back( sourceProperty(
                            wxString::Format( wxS( "pin_decal_group_%llu" ),
                                              static_cast<unsigned long long>( gateRecord - gateStart + 1 ) ),
                            wxString::Format( wxS( "%u" ), pinCount ), gateSource ) );

                    for( size_t member = gateRecord + 1; member < gateEnd; ++member )
                    {
                        const size_t   memberOffset = aLayout.gateBase + member * GATE_BYTES;
                        const uint16_t memberHandle = aCursor.U16At( memberOffset );

                        if( aCursor.U16At( memberOffset + 8 ) != 0 )
                            break;

                        SOURCE_PROVENANCE memberSource =
                                sourceAt( aSourceName, aModel.version, wxS( "pin-decal group member" ), 10, member,
                                          memberOffset, GATE_BYTES, static_cast<int>( aSheetIndex ) );

                        if( memberHandle >= aUsedDecals.size() || !aUsedDecals[memberHandle].definition )
                            throwDecodeError( memberSource, wxS( "unresolved pin-decal group member" ) );

                        gate.decalGroupMembers.push_back( { aUsedDecals[memberHandle].definition->id, memberSource } );
                    }

                    for( size_t pin = 0; pin < pinCount; ++pin, ++partPinCursor )
                        gate.connectorPins.push_back( decodeConnectorPin( partPinCursor ) );

                    part.gates.push_back( std::move( gate ) );
                    continue;
                }

                MODEL_SYMBOL_DEFINITION* definition = aUsedDecals[primaryHandle].definition;
                gate.definition = { definition->id, gateSource };
                gate.properties.push_back(
                        sourceProperty( wxS( "swap_group" ), wxString::Format( wxS( "%u" ), swapGroup ), gateSource ) );

                for( size_t alternate = 1; alternate < 4; ++alternate )
                {
                    const uint16_t handle = aCursor.U16At( gateOffset + alternate * 2 );

                    if( handle == 0xFFFF )
                        continue;

                    if( handle >= aUsedDecals.size() || !aUsedDecals[handle].definition )
                        throwDecodeError( gateSource, wxS( "unresolved alternate symbol definition reference" ) );

                    gate.alternateDefinitions.push_back( { aUsedDecals[handle].definition->id, gateSource } );
                }

                if( partPinCursor + pinCount > pinEnd || pinCount > definition->pins.size() )
                    throwDecodeError( gateSource, wxS( "gate pin slice leaves part type or symbol definition" ) );

                for( size_t pin = 0; pin < pinCount; ++pin, ++partPinCursor )
                {
                    MODEL_PIN_DEFINITION& definitionPin = definition->pins[pin];
                    decodePartPin( definitionPin, partPinCursor, gate );
                }

                part.gates.push_back( std::move( gate ) );
            }

            if( partPinCursor != pinEnd )
                throwDecodeError( source, wxS( "part-type gates do not consume its pin slice" ) );

            if( !part.gates.empty() && part.gates.front().definition.id.IsValid() )
            {
                auto defaultDefinition =
                        std::ranges::find_if( aModel.definitions,
                                              [&]( const MODEL_SYMBOL_DEFINITION& aDefinition )
                                              {
                                                  return aDefinition.id == part.gates.front().definition.id;
                                              } );

                if( defaultDefinition == aModel.definitions.end() )
                    throwDecodeError( source, wxS( "unresolved part-type default definition" ) );

                part.fields = defaultDefinition->fields;
            }

            const uint16_t signalPinCount = aCursor.U16At( offset + 70 );

            if( signalPinCursor + signalPinCount > aControllers.pools[11].count )
                throwDecodeError( source, wxS( "part-type signal-pin slice leaves controller 12" ) );

            for( size_t signal = 0; signal < signalPinCount; ++signal, ++signalPinCursor )
            {
                const size_t      signalOffset = aLayout.signalPinBase + signalPinCursor * SIGNAL_PIN_BYTES;
                SOURCE_PROVENANCE signalSource =
                        sourceAt( aSourceName, aModel.version, wxS( "signal pin" ), 12, signalPinCursor, signalOffset,
                                  SIGNAL_PIN_BYTES, static_cast<int>( aSheetIndex ) );
                SOURCE_PROVENANCE numberSource = signalSource;
                numberSource.length = 16;
                SOURCE_PROVENANCE nameSource2 = signalSource;
                nameSource2.absoluteOffset += 16;
                nameSource2.length = 48;
                SOURCE_STRING number = decodeFixedString( aBytes, signalOffset, 16, numberSource, aModel.diagnostics );
                SOURCE_STRING signalName =
                        decodeFixedString( aBytes, signalOffset + 16, 48, nameSource2, aModel.diagnostics );
                part.signalPins.push_back( { signalSource, number, signalName } );
                part.properties.push_back(
                        sourceProperty( wxS( "signal_pin_" ) + number.text, signalName.text, signalSource ) );
            }

            aModel.partTypes.push_back( std::move( part ) );
        }

        if( signalPinCursor != aControllers.pools[11].count )
        {
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "signal pin" ), 12, signalPinCursor,
                                                 aLayout.signalPinBase, aControllers.pools[11].usedBytes,
                                                 static_cast<int>( aSheetIndex ) );
            throwDecodeError( source, wxS( "unowned signal-pin record" ) );
        }
    }


    void decodeDefinitionsAndParts( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                                    const SCH_SDB_BLOCK& aBlock, size_t aSheetIndex, const wxString& aSourceName,
                                    const CONNECTIVITY_GLOBALS* aConnectivity, PADS_SCH_MODEL& aModel )
    {
        const SHEET_CONTROLLERS controllers = sheetControllers( aCursor, aBlock );

        if( aModel.version == 0x000C )
        {
            preserveRawDefinitionControllers( aBytes, controllers, aSheetIndex, aSourceName, aModel );
            return;
        }

        const DEFINITION_LAYOUT layout =
                definitionLayout( aCursor, controllers, aSheetIndex, aSourceName, aModel.version );

        std::vector<MODEL_SYMBOL_DEFINITION*> definitionsByRecord( controllers.pools[2].count );
        std::vector<size_t>                   pageGraphicRecords;
        std::vector<USED_DECAL>               usedDecals( controllers.pools[6].count );
        std::vector<USED_DECAL*>              semanticDecals;

        decodeSymbolDefinitions( aBytes, aCursor, controllers, layout, aSheetIndex, aSourceName, definitionsByRecord,
                                 pageGraphicRecords, aModel );
        decodeUsedDecals( aBytes, aCursor, controllers, layout, aSheetIndex, aSourceName, definitionsByRecord,
                          usedDecals, semanticDecals, aModel );

        const std::vector<USED_DECAL*> fieldDecals =
                buildFieldDecals( semanticDecals, layout, aSheetIndex, aSourceName, aModel.version );

        decodePageGraphics( aBytes, aCursor, layout, aSheetIndex, aSourceName, pageGraphicRecords, aModel );
        decodeDefinitionFields( aBytes, aCursor, layout, aSheetIndex, aSourceName, fieldDecals, aModel );
        decodePartTypes( aBytes, aCursor, controllers, layout, aSheetIndex, aSourceName, usedDecals,
                         aConnectivity, aModel );
    }


    struct PLACEMENT_DECODE
    {
        const std::vector<uint8_t>&   bytes;
        const PADS_IO::BINARY_CURSOR& cursor;
        const SHEET_CONTROLLERS&      controllers;
        const PLACEMENT_LAYOUT&       layout;
        const PLACEMENT_GLOBALS&      globals;
        const wxString&               sourceName;
        size_t                        sheetIndex;
    };


    struct PLACEMENT_TARGET
    {
        const MODEL_PART_TYPE*         part = nullptr;
        const MODEL_SYMBOL_DEFINITION* definition = nullptr;
        const MODEL_GATE*              gate = nullptr;
        uint32_t                       componentIdentity = 0;
        uint32_t                       groupHandle = 0;
        uint32_t                       attributeStart = 0;
        uint16_t                       attributeCount = 0;
        uint16_t                       decalHandle = 0;
        uint16_t                       unitIndex = 0;
    };


    struct INLINE_FIELD_LAYOUT
    {
        size_t position;
        size_t angle;
        size_t font;
        size_t height;
        size_t width;
    };


    std::pair<SOURCE_STRING, SOURCE_STRING> decodePlacementAttribute( const PLACEMENT_DECODE& aDecode,
                                                                      uint32_t aOffsetIndex, bool aRequireValue,
                                                                      PADS_SCH_MODEL& aModel )
    {
        if( aOffsetIndex >= aDecode.globals.attributeOffsetCount )
        {
            SOURCE_PROVENANCE source =
                    sourceAt( aDecode.sourceName, aModel.version, wxS( "attribute offset" ), 7, aOffsetIndex,
                              aDecode.globals.attributeOffsetBase, ATTRIBUTE_OFFSET_BYTES, -1 );
            throwDecodeError( source, wxS( "attribute offset index leaves outer controller 7" ) );
        }

        const size_t      offsetRecord = aDecode.globals.attributeOffsetBase + aOffsetIndex * ATTRIBUTE_OFFSET_BYTES;
        const uint32_t    heapOffset = aDecode.cursor.U32At( offsetRecord );
        SOURCE_PROVENANCE source = sourceAt( aDecode.sourceName, aModel.version, wxS( "placement attribute" ), 2,
                                             aOffsetIndex, aDecode.globals.attributeHeapBase + heapOffset, 0, -1 );

        if( heapOffset >= aDecode.globals.attributeHeapBytes )
            throwDecodeError( source, wxS( "attribute string offset leaves outer controller 2" ) );

        size_t       end = source.absoluteOffset;
        const size_t heapEnd = aDecode.globals.attributeHeapBase + aDecode.globals.attributeHeapBytes;

        while( end < heapEnd && aDecode.bytes[end] != 0 )
            ++end;

        if( end == heapEnd )
            throwDecodeError( source, wxS( "placement attribute is not NUL terminated" ) );

        source.length = end - source.absoluteOffset;
        size_t separator = source.absoluteOffset;

        while( separator < end && aDecode.bytes[separator] != 1 )
            ++separator;

        if( aRequireValue && separator == end )
            throwDecodeError( source, wxS( "placement attribute lacks key/value separator" ) );

        SOURCE_STRING name = PADS_SCH_BINARY_PARSER::DecodeString(
                { aDecode.bytes.begin() + source.absoluteOffset, aDecode.bytes.begin() + separator }, DEFAULT_CODE_PAGE,
                source, aModel.diagnostics );
        SOURCE_STRING value;

        if( separator != end )
        {
            SOURCE_PROVENANCE valueSource = source;
            valueSource.absoluteOffset = separator + 1;
            valueSource.length = end - separator - 1;
            value = PADS_SCH_BINARY_PARSER::DecodeString(
                    { aDecode.bytes.begin() + valueSource.absoluteOffset, aDecode.bytes.begin() + end },
                    DEFAULT_CODE_PAGE, valueSource, aModel.diagnostics );
        }

        return std::pair<SOURCE_STRING, SOURCE_STRING>{ std::move( name ), std::move( value ) };
    }


    PLACEMENT_TARGET resolvePlacementTarget( const PLACEMENT_DECODE& aDecode, size_t aOffset,
                                             const SOURCE_PROVENANCE& aSource, PADS_SCH_MODEL& aModel )
    {
        PLACEMENT_TARGET target;
        target.componentIdentity = aDecode.cursor.U32At( aOffset + aDecode.layout.componentIdentity );

        const uint16_t partHandle = aDecode.cursor.U16At( aOffset + aDecode.layout.partType );
        auto           part = std::ranges::find_if( aModel.partTypes,
                                                    [&]( const MODEL_PART_TYPE& aPart )
                                                    {
                                              return aPart.source.sheet == static_cast<int>( aDecode.sheetIndex )
                                                     && aPart.source.recordIndex == partHandle;
                                          } );

        if( part == aModel.partTypes.end() )
        {
            const bool definitionClass =
                    std::ranges::any_of( aModel.definitions,
                                         [&]( const MODEL_SYMBOL_DEFINITION& aDefinition )
                                         {
                                             return aDefinition.source.sheet == static_cast<int>( aDecode.sheetIndex )
                                                    && aDefinition.source.recordIndex == partHandle;
                                         } );
            throwDecodeError( aSource, definitionClass
                                               ? wxS( "placement part-type handle targets definition object class" )
                                               : wxS( "unresolved placement part-type reference" ) );
        }

        // PADS_V000D_HIERARCHICAL_GROUP_SENTINEL_V6
        // Hierarchical sheet symbols ($$ PARTTYPEs) have no component attribute
        // group.  Their componentGroup field is 0xFFFFFFFF, not an index into
        // outer controller 6.  All observed such placements have no custom
        // placement fields; REF-DES and PART-TYPE fields are embedded inline.
        // Never weaken normal component-group ownership/attribute validation.
        const uint32_t groupHandle = aDecode.cursor.U32At( aOffset + aDecode.layout.componentGroup );
        uint32_t       attributeStart = 0;
        uint16_t       attributeCount = 0;

        if( groupHandle == 0xFFFFFFFF )
        {
            if( aModel.version != 0x000D || !part->name.text.StartsWith( wxS( "$$" ) ) )
                throwDecodeError( aSource, wxS( "missing component group on non-hierarchical placement" ) );

            if( aDecode.cursor.U16At( aOffset + aDecode.layout.fieldCount ) != 0 )
                throwDecodeError( aSource, wxS( "ungrouped hierarchical placement has custom fields" ) );
        }
        else
        {
            if( groupHandle >= aDecode.globals.groupCount )
                throwDecodeError( aSource, wxS( "placement component-group handle leaves outer controller 6" ) );

            const size_t groupOffset = aDecode.globals.groupBase + groupHandle * PLACEMENT_GROUP_BYTES;
            attributeStart = aDecode.cursor.U32At( groupOffset );
            attributeCount = aDecode.cursor.U16At( groupOffset + 20 );

            if( attributeCount < 2 || attributeStart > aDecode.globals.attributeOffsetCount
                || attributeCount > aDecode.globals.attributeOffsetCount - attributeStart )
            {
                throwDecodeError( aSource, wxS( "placement component-group attribute slice leaves controller 7" ) );
            }

            auto [groupPartName, unusedGroupPartValue] =
                    decodePlacementAttribute( aDecode, attributeStart + 1, false, aModel );

            if( groupPartName.text != part->name.text )
                throwDecodeError( aSource, wxS( "placement component-group targets wrong part-type object class" ) );
        }

        const uint16_t decalHandle = aDecode.cursor.U16At( aOffset + aDecode.layout.decal );

        if( decalHandle >= aDecode.controllers.pools[6].count )
            throwDecodeError( aSource, wxS( "unresolved placement decal reference" ) );

        const size_t   decalOffset = aDecode.controllers.offsets[6] + decalHandle * USED_DECAL_BYTES;
        const uint32_t definitionHandle = aDecode.cursor.U32At( decalOffset + 48 );
        auto           definition =
                std::ranges::find_if( aModel.definitions,
                                      [&]( const MODEL_SYMBOL_DEFINITION& aDefinition )
                                      {
                                          return aDefinition.source.sheet == static_cast<int>( aDecode.sheetIndex )
                                                 && aDefinition.source.recordIndex == definitionHandle;
                                      } );

        if( definition == aModel.definitions.end() )
            throwDecodeError( aSource, wxS( "placement decal targets wrong definition object class" ) );

        const uint16_t unitIndex = aDecode.cursor.U16At( aOffset + aDecode.layout.gate );

        const MODEL_GATE* gate = nullptr;

        if( unitIndex < part->gates.size() )
            gate = &part->gates[unitIndex];
        else if( part->gates.size() == 1 && !part->gates.front().decalGroupMembers.empty() )
            gate = &part->gates.front();
        else if( part->gates.empty() && unitIndex == 0 && definition->pins.empty() )
            gate = nullptr;
        else
        {
            const bool definitionClass =
                    std::ranges::any_of( aModel.definitions,
                                         [&]( const MODEL_SYMBOL_DEFINITION& aDefinition )
                                         {
                                             return aDefinition.source.sheet == static_cast<int>( aDecode.sheetIndex )
                                                    && aDefinition.source.recordIndex == unitIndex;
                                         } );
            throwDecodeError( aSource, definitionClass ? wxS( "placement gate handle targets definition object class" )
                                                       : wxS( "unresolved placement gate reference" ) );
        }

        auto gateHasDefinition = [&]( const DEFINITION_REFERENCE& aReference )
        {
            return aReference.id == definition->id;
        };

        if( gate && !gateHasDefinition( gate->definition )
            && std::ranges::none_of( gate->alternateDefinitions, gateHasDefinition )
            && std::ranges::none_of( gate->decalGroupMembers, gateHasDefinition ) )
        {
            throwDecodeError( aSource, wxS( "placement decal and gate reference target different object classes" ) );
        }

        target.part = &*part;
        target.definition = &*definition;
        target.gate = gate;
        target.groupHandle = groupHandle;
        target.attributeStart = attributeStart;
        target.attributeCount = attributeCount;
        target.decalHandle = decalHandle;
        target.unitIndex = unitIndex;
        return target;
    }


    void decodePlacedPins( const PLACEMENT_DECODE& aDecode, uint32_t aPinStart, uint16_t aPinCount,
                           const MODEL_SYMBOL_DEFINITION& aDefinition, MODEL_PLACEMENT& aPlacement,
                           PADS_SCH_MODEL& aModel )
    {
        const size_t placedPinBase = aDecode.controllers.offsets[15];
        // PADS may serialize placed pins in a different order from the definition.
        // The handle in each placed-pin record gives the actual definition ordinal.
        // Each definition pin must be referenced exactly once per placement.
        std::vector<bool> seenPinOrdinals( aDefinition.pins.size(), false );

        for( size_t pin = 0; pin < aPinCount; ++pin )
        {
            const size_t      pinOffset = placedPinBase + ( aPinStart + pin ) * aDecode.layout.placedPinBytes;
            SOURCE_PROVENANCE pinSource =
                    sourceAt( aDecode.sourceName, aModel.version, wxS( "placed pin" ), 16, aPinStart + pin, pinOffset,
                              aDecode.layout.placedPinBytes, static_cast<int>( aDecode.sheetIndex ) );
            const uint16_t pinOrdinal = aDecode.cursor.U16At( pinOffset + aDecode.layout.placedPinOrdinal );

            if( pinOrdinal >= aDefinition.pins.size() )
                throwDecodeError( pinSource, wxS( "placed-pin handle leaves placement definition" ) );

            if( seenPinOrdinals[pinOrdinal] )
                throwDecodeError( pinSource, wxS( "duplicate placed-pin handle within placement" ) );

            seenPinOrdinals[pinOrdinal] = true;
            PLACED_PIN_REFERENCE pinReference{ aDefinition.pins[pinOrdinal].id, pinSource };
            pinReference.numberOffset = {
                static_cast<int64_t>( static_cast<int16_t>( aDecode.cursor.U16At( pinOffset + 6 ) ) ) * 2,
                static_cast<int64_t>( static_cast<int16_t>( aDecode.cursor.U16At( pinOffset + 8 ) ) ) * 2, pinSource
            };
            pinReference.numberPresentationFlags = aDecode.cursor.U16At( pinOffset + 10 );
            pinReference.numberAngle = ( pinReference.numberPresentationFlags & 0x0001 ) != 0 ? 900 : 0;

            pinReference.numberJustification = terminalJustification(
                    ( pinReference.numberPresentationFlags >> 4 ) & 0x0F, pinReference.numberAngle != 0 );

            pinReference.hasNumberPlacement = true;
            aPlacement.pins.push_back( std::move( pinReference ) );
        }
    }


    void decodeInlineField( const PLACEMENT_DECODE& aDecode, size_t aOffset, const SOURCE_PROVENANCE& aSource,
                            const wxString& aName, const SOURCE_STRING& aValue, bool aVisible,
                            const INLINE_FIELD_LAYOUT& aFieldLayout, MODEL_PLACEMENT& aPlacement,
                            PADS_SCH_MODEL& aModel )
    {
        SOURCE_PROVENANCE fieldSource = aSource;
        fieldSource.objectClass = wxS( "placement field" );
        fieldSource.absoluteOffset += aFieldLayout.position;
        fieldSource.length = 8;
        MODEL_FIELD field;
        field.source = fieldSource;
        field.name.text = aName;
        field.name.source = fieldSource;
        field.value = aValue;
        field.position = { decodeLocalCoordinate( aDecode.cursor.U16At( aOffset + aFieldLayout.position ) ),
                           decodeLocalCoordinate( aDecode.cursor.U16At( aOffset + aFieldLayout.position + 2 ) ),
                           fieldSource };
        field.angle = NormalizeAngle( aDecode.cursor.U16At( aOffset + aFieldLayout.angle ) );
        field.presentation.source = fieldSource;
        field.visible = aVisible;
        field.presentation.visible = aVisible;
        field.presentation.height =
                static_cast<int64_t>( aDecode.cursor.U16At( aOffset + aFieldLayout.height ) ) * 2;
        field.presentation.width =
                static_cast<int64_t>( aDecode.cursor.U8At( aOffset + aFieldLayout.width ) ) * 2;
        const uint16_t justification = rotatedFieldJustification(
                aDecode.cursor.U16At( aOffset + aFieldLayout.angle + 2 ), field.angle );
        field.presentation.horizontalJustification = horizontalJustification( justification );
        field.presentation.verticalJustification = verticalJustification( justification );
        SOURCE_PROVENANCE fontSource = fieldSource;
        fontSource.absoluteOffset = aOffset + aFieldLayout.font;
        fontSource.length = 2;
        decodeGlobalFont( aDecode.bytes, aDecode.cursor, aDecode.globals, aDecode.sourceName, aModel.version,
                          static_cast<int16_t>( aDecode.cursor.U16At( aOffset + aFieldLayout.font ) ), fontSource,
                          field.presentation, false, aModel.diagnostics );
        aPlacement.fields.push_back( std::move( field ) );
    }


    void decodeCustomFields( const PLACEMENT_DECODE& aDecode, size_t aOffset, const SOURCE_PROVENANCE& aSource,
                             const PLACEMENT_TARGET& aTarget, uint32_t& aFieldCursor, MODEL_PLACEMENT& aPlacement,
                             PADS_SCH_MODEL& aModel )
    {
        const size_t fieldBase = aDecode.controllers.offsets[16];

        const uint16_t customFieldCount = aDecode.cursor.U16At( aOffset + aDecode.layout.fieldCount );

        if( aFieldCursor > aDecode.controllers.pools[16].count
            || customFieldCount > aDecode.controllers.pools[16].count - aFieldCursor )
        {
            throwDecodeError( aSource, wxS( "placement field ownership does not match controller 17" ) );
        }

        for( size_t fieldOrdinal = 0; fieldOrdinal < customFieldCount; ++fieldOrdinal )
        {
            const size_t      fieldOffset = fieldBase + aFieldCursor * aDecode.layout.fieldBytes;
            SOURCE_PROVENANCE fieldSource =
                    sourceAt( aDecode.sourceName, aModel.version, wxS( "placement field" ), 17, aFieldCursor,
                              fieldOffset, aDecode.layout.fieldBytes, static_cast<int>( aDecode.sheetIndex ) );
            SOURCE_STRING name;
            SOURCE_STRING value;

            const uint8_t  displayFlags = aDecode.cursor.U8At( fieldOffset + aDecode.layout.customDisplayFlags );
            const uint16_t attributeIndex = aDecode.cursor.U16At( fieldOffset + aDecode.layout.customAttributeIndex );

            if( attributeIndex == 0xFFFF )
            {
                // Unbound PADS placement-field records carry presentation only;
                // they are not component attributes and must not become a
                // synthetic "*" field in KiCad.
                ++aFieldCursor;
                continue;
            }

            if( attributeIndex >= aTarget.attributeCount )
                throwDecodeError( fieldSource, wxS( "placement field attribute index leaves component group" ) );

            std::tie( name, value ) =
                    decodePlacementAttribute( aDecode, aTarget.attributeStart + attributeIndex, true, aModel );

            MODEL_FIELD field;
            field.source = fieldSource;
            field.name = std::move( name );
            field.value = std::move( value );
            field.position = {
                decodeLocalCoordinate( aDecode.cursor.U16At( fieldOffset + aDecode.layout.customX ) ),
                decodeLocalCoordinate( aDecode.cursor.U16At( fieldOffset + aDecode.layout.customX + 2 ) ), fieldSource
            };
            field.angle = NormalizeAngle( aDecode.cursor.U16At( fieldOffset + aDecode.layout.customAngle ) );

            if( field.angle % 900 != 0 )
                throwDecodeError( fieldSource, wxS( "unsupported placement-field rotation" ) );

            const uint16_t justification = rotatedFieldJustification(
                    aDecode.cursor.U8At( fieldOffset + aDecode.layout.customJustification ), field.angle );
            field.presentation.source = fieldSource;
            field.presentation.horizontalJustification = horizontalJustification( justification );
            field.presentation.verticalJustification = verticalJustification( justification );
            field.presentation.height = static_cast<int64_t>(
                                                aDecode.cursor.U16At( fieldOffset + aDecode.layout.customHeight ) )
                                        * 2;
            field.presentation.width = static_cast<int64_t>(
                                               aDecode.cursor.U8At( fieldOffset + aDecode.layout.customWidth ) )
                                       * 2;
            field.presentation.visible = ( displayFlags & 8 ) == 0;
            field.visible = field.presentation.visible;

            SOURCE_PROVENANCE fontSource = fieldSource;
            fontSource.length = 2;
            decodeGlobalFont( aDecode.bytes, aDecode.cursor, aDecode.globals, aDecode.sourceName, aModel.version,
                              static_cast<int16_t>( aDecode.cursor.U16At( fieldOffset + aDecode.layout.customFont ) ),
                              fontSource, field.presentation, true, aModel.diagnostics );
            field.properties.push_back( sourceProperty( wxS( "display_flags" ),
                                                        wxString::Format( wxS( "%u" ), displayFlags ), fieldSource ) );
            SOURCE_PROVENANCE attributeIndexSource = fieldSource;
            attributeIndexSource.absoluteOffset += aDecode.layout.customAttributeIndex;
            attributeIndexSource.length = 2;
            field.properties.push_back( sourceProperty(
                    wxS( "component_attribute_index" ),
                    wxString::Format( wxS( "%u" ),
                                      aDecode.cursor.U16At( fieldOffset + aDecode.layout.customAttributeIndex ) ),
                    attributeIndexSource ) );
            SOURCE_PROPERTY preservedTail = sourceProperty(
                    wxS( "preserved_field_tail" ),
                    wxString::Format( wxS( "%u" ), aDecode.cursor.U16At( fieldOffset + aDecode.layout.customTail ) ),
                    fieldSource );
            preservedTail.disposition = PROPERTY_DISPOSITION::PRESERVED;
            field.properties.push_back( std::move( preservedTail ) );
            aPlacement.fields.push_back( std::move( field ) );
            ++aFieldCursor;
        }
    }


    void decodeGroupAttributeFields( const PLACEMENT_DECODE& aDecode, const PLACEMENT_TARGET& aTarget,
                                     MODEL_PLACEMENT& aPlacement, PADS_SCH_MODEL& aModel )
    {
        for( uint16_t attributeIndex = 2; attributeIndex < aTarget.attributeCount; ++attributeIndex )
        {
            auto [name, value] =
                    decodePlacementAttribute( aDecode, aTarget.attributeStart + attributeIndex, true, aModel );

            // Preserve real PADS attributes even when their value is empty.
            // PCB DECAL is deliberately retained here so the builder can map
            // it to KiCad's native Footprint field.
            if( std::ranges::any_of( aPlacement.fields,
                                     [&]( const MODEL_FIELD& aField )
                                     {
                                         return aField.name.text.CmpNoCase( name.text ) == 0;
                                     } ) )
            {
                continue;
            }

            MODEL_FIELD field;
            field.source = name.source;
            field.name = std::move( name );
            field.value = std::move( value );
            field.visible = false;
            field.presentation.source = field.source;
            field.presentation.visible = false;
            aPlacement.fields.push_back( std::move( field ) );
        }
    }


    void recordPlacementProperties( const PLACEMENT_DECODE& aDecode, const SOURCE_PROVENANCE& aSource,
                                    const PLACEMENT_TARGET& aTarget, uint16_t aRawAngle, MODEL_PLACEMENT& aPlacement )
    {
        SOURCE_PROVENANCE rawAngleSource = aSource;
        rawAngleSource.absoluteOffset += aDecode.layout.angle;
        rawAngleSource.length = 2;
        aPlacement.properties.push_back(
                sourceProperty( wxS( "raw_angle" ), wxString::Format( wxS( "%u" ), aRawAngle ), rawAngleSource ) );
        aPlacement.properties.push_back( sourceProperty(
                wxS( "component_identity" ), wxString::Format( wxS( "%u" ), aTarget.componentIdentity ), aSource ) );
        aPlacement.properties.push_back( sourceProperty(
                wxS( "component_group_handle" ), wxString::Format( wxS( "%u" ), aTarget.groupHandle ), aSource ) );
        aPlacement.properties.push_back( sourceProperty(
                wxS( "decal_handle" ), wxString::Format( wxS( "%u" ), aTarget.decalHandle ), aSource ) );
        SOURCE_PROVENANCE rawMirrorSource = aSource;
        rawMirrorSource.absoluteOffset += aDecode.layout.mirror;
        rawMirrorSource.length = 2;
        aPlacement.properties.push_back( sourceProperty(
                wxS( "raw_mirror" ), wxString::Format( wxS( "%u" ), aPlacement.mirrorFlags ), rawMirrorSource ) );
        SOURCE_PROVENANCE visibilitySource = aSource;
        visibilitySource.absoluteOffset += aDecode.layout.itemVisibility;
        visibilitySource.length = 1;
        aPlacement.properties.push_back(
                sourceProperty( wxS( "item_visibility_flags" ),
                                wxString::Format( wxS( "%u" ), aPlacement.itemVisibilityFlags ), visibilitySource ) );
    }


    void decodePlacementRecord( const PLACEMENT_DECODE& aDecode, size_t aRecord, uint32_t& aExpectedPinStart,
                                uint32_t& aFieldCursor, PADS_SCH_MODEL& aModel )
    {
        const PLACEMENT_LAYOUT& layout = aDecode.layout;
        const size_t            offset = aDecode.controllers.offsets[14] + aRecord * layout.placementBytes;

        SOURCE_PROVENANCE source = sourceAt( aDecode.sourceName, aModel.version, wxS( "placement" ), 15, aRecord,
                                             offset, layout.placementBytes, static_cast<int>( aDecode.sheetIndex ) );

        const PLACEMENT_TARGET target = resolvePlacementTarget( aDecode, offset, source, aModel );
        const uint32_t         pinStart = aDecode.cursor.U32At( offset + layout.pinStart );
        const uint16_t         pinCount = aDecode.cursor.U16At( offset + layout.pinCount );

        if( pinStart != aExpectedPinStart || pinStart > aDecode.controllers.pools[15].count
            || pinCount > aDecode.controllers.pools[15].count - pinStart || pinCount != target.definition->pins.size() )
        {
            throwDecodeError( source, wxS( "placement pin ownership does not match controller 16" ) );
        }

        MODEL_PLACEMENT placement;

        if( aDecode.sheetIndex >= 0x0FFF || aRecord >= 0x100000 )
            throwDecodeError( source, wxS( "placement identity exceeds sheet/controller namespace" ) );

        placement.id = PLACEMENT_ID( static_cast<uint32_t>( aDecode.sheetIndex * 0x100000 + aRecord + 1 ) );
        placement.source = source;
        placement.sheet = { aModel.sheets[aDecode.sheetIndex].id, source };
        placement.partType = { target.part->id, source };

        if( target.gate )
            placement.gate = GATE_REFERENCE{ target.gate->id, source };

        placement.definition = { target.definition->id, source };
        placement.unit = target.unitIndex + 1;
        placement.position = { decodeCoordinate( aDecode.cursor.U16At( offset + layout.x ) ),
                               decodeCoordinate( aDecode.cursor.U16At( offset + layout.y ) ), source };
        const uint16_t rawAngle = aDecode.cursor.U16At( offset + layout.angle );
        placement.angle = NormalizeAngle( rawAngle );
        placement.mirrorFlags = aDecode.cursor.U16At( offset + layout.mirror );

        auto recordTransformEnum = [&]( size_t aFieldOffset )
        {
            SOURCE_PROVENANCE enumSource = source;
            enumSource.absoluteOffset += aFieldOffset;
            enumSource.length = 2;
            aModel.diagnostics.push_back(
                    { RPT_SEVERITY_WARNING, enumSource, wxS( "unknown placement transform enum preserved" ) } );
        };

        const bool knownAngle = rawAngle == 0 || rawAngle == 900 || rawAngle == 1800 || rawAngle == 2700;

        if( !knownAngle )
            recordTransformEnum( layout.angle );

        const bool knownMirror = placement.mirrorFlags <= 3;

        if( !knownMirror )
            recordTransformEnum( layout.mirror );

        placement.mirrored = placement.mirrorFlags != 0;
        SOURCE_PROVENANCE referenceSource = source;
        referenceSource.absoluteOffset += layout.reference;
        referenceSource.length = 40;
        placement.reference =
                decodeFixedString( aDecode.bytes, offset + layout.reference, 40, referenceSource, aModel.diagnostics );

        decodePlacedPins( aDecode, pinStart, pinCount, *target.definition, placement, aModel );
        aExpectedPinStart += pinCount;

        placement.itemVisibilityFlags = aDecode.cursor.U8At( offset + layout.itemVisibility );
        placement.referenceVisible = ( placement.itemVisibilityFlags & 0x01 ) == 0;
        placement.partTypeVisible = ( placement.itemVisibilityFlags & 0x02 ) == 0;
        placement.pinNamesVisible = ( placement.itemVisibilityFlags & 0x08 ) == 0;
        placement.pinNumbersVisible = ( placement.itemVisibilityFlags & 0x10 ) == 0;

        decodeInlineField( aDecode, offset, source, wxS( "REF-DES" ), placement.reference, placement.referenceVisible,
                           { layout.referenceField, layout.referenceFieldAngle, layout.referenceFont,
                             layout.referenceHeight, layout.referenceWidth },
                           placement, aModel );
        decodeInlineField( aDecode, offset, source, wxS( "PART-TYPE" ), target.part->name, placement.partTypeVisible,
                           { layout.partTypeField, layout.partTypeFieldAngle, layout.partTypeFont,
                             layout.partTypeHeight, layout.partTypeWidth },
                           placement, aModel );
        decodeCustomFields( aDecode, offset, source, target, aFieldCursor, placement, aModel );
        decodeGroupAttributeFields( aDecode, target, placement, aModel );
        recordPlacementProperties( aDecode, source, target, rawAngle, placement );
        aModel.placements.push_back( std::move( placement ) );
    }


    void decodePlacements( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                           const SCH_SDB_BLOCK& aBlock, size_t aSheetIndex, const wxString& aSourceName,
                           const PLACEMENT_GLOBALS& aGlobals, PADS_SCH_MODEL& aModel )
    {
        const PLACEMENT_LAYOUT& layout = placementLayout( aModel.version );

        if( !layout.decoded )
            throwDecodeError( aModel.source, wxS( "placement decoder selected for raw-preserved version" ) );

        const SHEET_CONTROLLERS controllers = sheetControllers( aCursor, aBlock );
        requireFixedController( controllers, 15, layout.placementBytes, aSourceName, aModel.version, aSheetIndex );
        requireFixedController( controllers, 16, layout.placedPinBytes, aSourceName, aModel.version, aSheetIndex );
        requireFixedController( controllers, 17, layout.fieldBytes, aSourceName, aModel.version, aSheetIndex );

        const PLACEMENT_DECODE decode{ aBytes, aCursor, controllers, layout, aGlobals, aSourceName, aSheetIndex };
        uint32_t               expectedPinStart = 0;
        uint32_t               fieldCursor = 0;

        for( size_t record = 0; record < controllers.pools[14].count; ++record )
            decodePlacementRecord( decode, record, expectedPinStart, fieldCursor, aModel );

        if( expectedPinStart != controllers.pools[15].count )
        {
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "placed pin" ), 16, expectedPinStart,
                                                 controllers.offsets[15], controllers.pools[15].usedBytes,
                                                 static_cast<int>( aSheetIndex ) );
            throwDecodeError( source, wxS( "unowned placed-pin record" ) );
        }

        if( fieldCursor != controllers.pools[16].count )
        {
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "placement field" ), 17, fieldCursor,
                                                 controllers.offsets[16], controllers.pools[16].usedBytes,
                                                 static_cast<int>( aSheetIndex ) );
            throwDecodeError( source, wxS( "unowned placement-field record" ) );
        }
    }


    struct CONNECTIVITY_DECODE
    {
        const std::vector<uint8_t>&   bytes;
        const PADS_IO::BINARY_CURSOR& cursor;
        const SHEET_CONTROLLERS&      controllers;
        const CONNECTIVITY_GLOBALS&   globals;
        const wxString&               sourceName;
        size_t                        sheetIndex;
        size_t                        busBase;
        size_t                        junctionBase;
        size_t                        offpageBase;
        size_t                        connectionBase;
        size_t                        vertexBase;
        size_t                        netNameBase;
    };


    struct VERTEX_TILING
    {
        std::vector<uint32_t> starts;
        std::vector<uint32_t> ends;
    };


    struct SHEET_CONNECTIVITY
    {
        std::vector<MODEL_NET*>          sheetNets;
        std::vector<MODEL_CONNECTION*>   connections;
        std::vector<MODEL_NET*>          connectionNets;
        std::vector<std::vector<size_t>> junctionBacklinks;
        std::vector<std::vector<size_t>> offpageBacklinks;
    };


    std::pair<int64_t, int64_t> transformedPinPosition( const MODEL_PLACEMENT&      aPlacement,
                                                        const MODEL_PIN_DEFINITION& aPin )
    {
        int64_t x = aPin.position.x;
        int64_t y = aPin.position.y;

        switch( NormalizeAngle( aPlacement.angle ) )
        {
        case 900: std::tie( x, y ) = std::pair{ -y, x }; break;
        case 1800: std::tie( x, y ) = std::pair{ -x, -y }; break;
        case 2700: std::tie( x, y ) = std::pair{ y, -x }; break;
        default: break;
        }

        if( aPlacement.mirrorFlags & 1 )
            x = -x;

        if( aPlacement.mirrorFlags & 2 )
            y = -y;

        return std::pair{ aPlacement.position.x + x, aPlacement.position.y + y };
    }


    SOURCE_POINT offpagePosition( const CONNECTIVITY_DECODE& aDecode, size_t aRecord, uint16_t aVersion )
    {
        const size_t      offset = aDecode.offpageBase + aRecord * OFFPAGE_RECORD_BYTES;
        SOURCE_PROVENANCE source = sourceAt( aDecode.sourceName, aVersion, wxS( "off-page reference" ), 20, aRecord,
                                             offset, OFFPAGE_RECORD_BYTES, static_cast<int>( aDecode.sheetIndex ) );
        return SOURCE_POINT{ decodeCoordinate( aDecode.cursor.U16At( offset + 22 ) ),
                             decodeCoordinate( aDecode.cursor.U16At( offset + 24 ) ), source };
    }


    SOURCE_POINT junctionPosition( const CONNECTIVITY_DECODE& aDecode, size_t aRecord, uint16_t aVersion )
    {
        const size_t      offset = aDecode.junctionBase + aRecord * JUNCTION_RECORD_BYTES;
        SOURCE_PROVENANCE source = sourceAt( aDecode.sourceName, aVersion, wxS( "junction" ), 19, aRecord, offset,
                                             JUNCTION_RECORD_BYTES, static_cast<int>( aDecode.sheetIndex ) );
        return SOURCE_POINT{ decodeCoordinate( aDecode.cursor.U16At( offset + 4 ) ),
                             decodeCoordinate( aDecode.cursor.U16At( offset + 6 ) ), source };
    }


    std::unordered_set<uint32_t> decodeBusGlobalRecords( const CONNECTIVITY_DECODE& aDecode, PADS_SCH_MODEL& aModel )
    {
        std::unordered_set<uint32_t> busGlobalRecords;

        for( size_t record = 0; record < aDecode.controllers.pools[17].count; ++record )
        {
            const uint32_t globalRecord = aDecode.cursor.U32At( aDecode.busBase + record * BUS_RECORD_BYTES + 8 );
            const uint8_t  netKind = globalRecord < aDecode.globals.nets.size()
                                             ? aDecode.globals.nets[globalRecord].kindFlags & 0xFF
                                             : 0;

            if( globalRecord >= aDecode.globals.nets.size() || aDecode.globals.nets[globalRecord].tombstone
                || ( netKind != 1 && netKind != 5 ) )
            {
                SOURCE_PROVENANCE source = sourceAt( aDecode.sourceName, aModel.version, wxS( "bus" ), 18, record,
                                                     aDecode.busBase + record * BUS_RECORD_BYTES, BUS_RECORD_BYTES,
                                                     static_cast<int>( aDecode.sheetIndex ) );
                throwDecodeError( source, wxS( "bus global-net handle targets wrong or unresolved object class" ) );
            }

            busGlobalRecords.insert( globalRecord );
        }

        return busGlobalRecords;
    }


    std::vector<MODEL_NET*> materializeSheetNets( const CONNECTIVITY_DECODE&          aDecode,
                                                  const std::unordered_set<uint32_t>& aBusGlobalRecords,
                                                  PADS_SCH_MODEL&                     aModel )
    {
        std::vector<MODEL_NET*> sheetNets( aDecode.globals.nets.size(), nullptr );
        // sheetNets holds pointers into nets, and one global record can own several memberships on
        // this sheet, so bound the reserve by the disjoint membership slices
        aModel.nets.reserve( aModel.nets.size() + aDecode.globals.membershipCount );

        for( size_t globalRecord = 0; globalRecord < aDecode.globals.nets.size(); ++globalRecord )
        {
            const GLOBAL_NET_RECORD& global = aDecode.globals.nets[globalRecord];

            if( global.tombstone )
                continue;

            for( uint32_t membership = global.membershipStart;
                 membership < global.membershipStart + global.membershipCount; ++membership )
            {
                if( aDecode.globals.membershipSheets[membership] != aDecode.sheetIndex )
                    continue;

                if( global.aliasCount != 0 || aBusGlobalRecords.contains( globalRecord ) )
                    continue;

                MODEL_NET net;
                net.id = NET_ID( membership );
                net.source = global.source;
                net.source.sheet = static_cast<int>( aDecode.sheetIndex );
                SOURCE_PROVENANCE membershipSource =
                        sourceAt( aDecode.sourceName, aModel.version, wxS( "net sheet membership" ), 4, membership,
                                  aDecode.globals.membershipBase + membership * NET_MEMBERSHIP_BYTES,
                                  NET_MEMBERSHIP_BYTES, static_cast<int>( aDecode.sheetIndex ) );
                net.sheet = { aModel.sheets[aDecode.sheetIndex].id, membershipSource };
                net.name = global.name;
                net.properties.push_back( sourceProperty(
                        wxS( "global_net_record" ), wxString::Format( wxS( "%llu" ), globalRecord ), global.source ) );
                SOURCE_PROPERTY identity =
                        sourceProperty( wxS( "preserved_net_identity" ),
                                        wxString::Format( wxS( "%u" ), global.preservedIdentity ), global.source );
                identity.disposition = PROPERTY_DISPOSITION::PRESERVED;
                net.properties.push_back( std::move( identity ) );
                SOURCE_PROVENANCE relationshipSource = global.source;
                relationshipSource.absoluteOffset += 84;
                relationshipSource.length = 4;
                SOURCE_PROPERTY relationship = sourceProperty(
                        wxS( "preserved_net_relationship" ),
                        wxString::Format( wxS( "%u" ), global.preservedRelationship ), relationshipSource );
                relationship.disposition = PROPERTY_DISPOSITION::PRESERVED;
                net.properties.push_back( std::move( relationship ) );
                aModel.nets.push_back( std::move( net ) );
                sheetNets[globalRecord] = &aModel.nets.back();
            }
        }

        return sheetNets;
    }


    std::unordered_map<uint32_t, const MODEL_PIN_DEFINITION*> indexDefinitionPins( const PADS_SCH_MODEL& aModel )
    {
        std::unordered_map<uint32_t, const MODEL_PIN_DEFINITION*> definitionPins;

        for( const MODEL_SYMBOL_DEFINITION& definition : aModel.definitions )
        {
            for( const MODEL_PIN_DEFINITION& pin : definition.pins )
                definitionPins.emplace( pin.id.Value(), &pin );
        }

        return definitionPins;
    }


    std::vector<MODEL_PLACEMENT*> indexSheetPlacements( const CONNECTIVITY_DECODE& aDecode, PADS_SCH_MODEL& aModel )
    {
        std::vector<MODEL_PLACEMENT*> placements( aDecode.controllers.pools[14].count, nullptr );

        for( MODEL_PLACEMENT& placement : aModel.placements )
        {
            if( placement.source.sheet != static_cast<int>( aDecode.sheetIndex ) )
                continue;

            if( placement.source.recordIndex >= placements.size() || placements[placement.source.recordIndex] )
                throwDecodeError( placement.source, wxS( "placement endpoint identity leaves controller 15" ) );

            placements[placement.source.recordIndex] = &placement;
        }

        return placements;
    }


    void decodeJunctions( const CONNECTIVITY_DECODE& aDecode, PADS_SCH_MODEL& aModel )
    {
        for( size_t record = 0; record < aDecode.controllers.pools[18].count; ++record )
        {
            const size_t      offset = aDecode.junctionBase + record * JUNCTION_RECORD_BYTES;
            SOURCE_PROVENANCE source =
                    sourceAt( aDecode.sourceName, aModel.version, wxS( "junction" ), 19, record, offset,
                              JUNCTION_RECORD_BYTES, static_cast<int>( aDecode.sheetIndex ) );

            const uint16_t status = aDecode.cursor.U16At( offset + 10 );

            if( status != 0 && status != 0x00FC && status != 0x00FD )
                throwDecodeError( source, wxS( "invalid junction object-class marker" ) );

            MODEL_JUNCTION junction;
            junction.source = source;
            junction.sheet = { aModel.sheets[aDecode.sheetIndex].id, source };
            junction.position = junctionPosition( aDecode, record, aModel.version );
            SOURCE_PROVENANCE connectionSource = source;
            connectionSource.absoluteOffset += 8;
            connectionSource.length = 2;
            junction.properties.push_back( sourceProperty(
                    wxS( "connection_record" ), wxString::Format( wxS( "%u" ), aDecode.cursor.U16At( offset + 8 ) ),
                    connectionSource ) );
            aModel.junctions.push_back( std::move( junction ) );
        }
    }


    VERTEX_TILING tileConnectivityVertices( const CONNECTIVITY_DECODE& aDecode, PADS_SCH_MODEL& aModel )
    {
        VERTEX_TILING tiling;


        tiling.starts.reserve( aDecode.controllers.pools[17].count + aDecode.controllers.pools[20].count );

        for( size_t record = 0; record < aDecode.controllers.pools[17].count; ++record )
            tiling.starts.push_back( aDecode.cursor.U32At( aDecode.busBase + record * BUS_RECORD_BYTES + 4 ) );

        for( size_t record = 0; record < aDecode.controllers.pools[20].count; ++record )
            tiling.starts.push_back(
                    aDecode.cursor.U32At( aDecode.connectionBase + record * CONNECTION_RECORD_BYTES + 4 ) );

        std::vector<size_t> objectOrder( tiling.starts.size() );

        for( size_t object = 0; object < objectOrder.size(); ++object )
            objectOrder[object] = object;

        std::ranges::sort( objectOrder,
                           [&]( size_t aLeft, size_t aRight )
                           {
                               return tiling.starts[aLeft] < tiling.starts[aRight];
                           } );
        tiling.ends.resize( tiling.starts.size() );

        for( size_t ordinal = 0; ordinal < objectOrder.size(); ++ordinal )
        {
            const size_t   object = objectOrder[ordinal];
            const uint32_t end = ordinal + 1 < objectOrder.size() ? tiling.starts[objectOrder[ordinal + 1]]
                                                                  : aDecode.controllers.pools[21].count;

            if( ( ordinal == 0 && tiling.starts[object] != 0 ) || tiling.starts[object] >= end
                || end > aDecode.controllers.pools[21].count )
            {
                SOURCE_PROVENANCE source =
                        sourceAt( aDecode.sourceName, aModel.version, wxS( "connectivity vertex ownership" ),
                                  object < aDecode.controllers.pools[17].count ? 18 : 21, object, aDecode.vertexBase,
                                  aDecode.controllers.pools[21].usedBytes, static_cast<int>( aDecode.sheetIndex ) );
                throwDecodeError( source, wxS( "connectivity vertex slices do not exactly tile controller 22" ) );
            }

            tiling.ends[object] = end;
        }

        if( tiling.starts.empty() && aDecode.controllers.pools[21].count != 0 )
        {
            SOURCE_PROVENANCE source =
                    sourceAt( aDecode.sourceName, aModel.version, wxS( "connection vertex" ), 22, 0, aDecode.vertexBase,
                              aDecode.controllers.pools[21].usedBytes, static_cast<int>( aDecode.sheetIndex ) );
            throwDecodeError( source, wxS( "unclaimed connection vertices" ) );
        }

        return tiling;
    }


    void appendVertices( const CONNECTIVITY_DECODE& aDecode, const VERTEX_TILING& aTiling, size_t aObject,
                         uint16_t aVersion, std::vector<SOURCE_POINT>& aVertices )
    {
        const uint32_t start = aTiling.starts[aObject];
        const uint32_t end = aTiling.ends[aObject];

        for( uint32_t vertex = start; vertex < end; ++vertex )
        {
            const size_t      offset = aDecode.vertexBase + vertex * CONNECTION_VERTEX_BYTES;
            SOURCE_PROVENANCE source =
                    sourceAt( aDecode.sourceName, aVersion, wxS( "connection vertex" ), 22, vertex, offset,
                              CONNECTION_VERTEX_BYTES, static_cast<int>( aDecode.sheetIndex ) );

            if( aDecode.cursor.U32At( offset ) != 0 )
                throwDecodeError( source, wxS( "nonzero connection-vertex padding" ) );

            aVertices.push_back( { decodeCoordinate( aDecode.cursor.U16At( offset + 4 ) ),
                                   decodeCoordinate( aDecode.cursor.U16At( offset + 6 ) ), source } );
        }
    }


    MODEL_CONNECTION_ENDPOINT
    decodeConnectionEndpoint( const CONNECTIVITY_DECODE& aDecode, const SOURCE_PROVENANCE& aSource, size_t aFieldOffset,
                              size_t aRelationshipOffset, const SOURCE_POINT& aWirePoint,
                              const std::vector<MODEL_PLACEMENT*>&                             aPlacements,
                              const std::unordered_map<uint32_t, const MODEL_PIN_DEFINITION*>& aDefinitionPins,
                              PADS_SCH_MODEL&                                                  aModel )
    {
        SOURCE_PROVENANCE endpointSource = aSource;
        endpointSource.objectClass = wxS( "connection endpoint" );
        endpointSource.absoluteOffset += aFieldOffset;
        endpointSource.length = 2;
        const uint16_t    raw = aDecode.cursor.U16At( endpointSource.absoluteOffset );
        const uint16_t    objectClass = raw >> 12;
        const uint16_t    objectRecord = raw & 0x0FFF;
        SOURCE_PROVENANCE relationshipSource = aSource;
        relationshipSource.objectClass = wxS( "connection endpoint relationship" );
        relationshipSource.absoluteOffset += aRelationshipOffset;
        relationshipSource.length = 4;
        const uint32_t            relationship = aDecode.cursor.U32At( relationshipSource.absoluteOffset );
        MODEL_CONNECTION_ENDPOINT result;
        result.source = endpointSource;

        switch( objectClass )
        {
        case 0:
            if( objectRecord >= aPlacements.size() || !aPlacements[objectRecord] )
                throwDecodeError( endpointSource, wxS( "unresolved placement endpoint" ) );

            {
                MODEL_PLACEMENT&     placement = *aPlacements[objectRecord];
                const PIN_REFERENCE* matchedPin = nullptr;
                size_t               coincidentPinCount = 0;

                for( const PIN_REFERENCE& pin : placement.pins )
                {
                    auto definitionPin = aDefinitionPins.find( pin.id.Value() );

                    if( definitionPin == aDefinitionPins.end() )
                        throwDecodeError( pin.source, wxS( "placement pin definition is unresolved" ) );

                    const auto position = transformedPinPosition( placement, *definitionPin->second );

                    if( position.first != aWirePoint.x || position.second != aWirePoint.y )
                        continue;

                    // Multiple source pins may legitimately share an electrical
                    // anchor (for example PADS FASTON decals).  A wire connected
                    // at this point connects every coincident pin in KiCad too.
                    // Keep one stable model reference; the placement retains all pins.
                    if( !matchedPin )
                        matchedPin = &pin;

                    ++coincidentPinCount;
                }

                if( !matchedPin )
                    throwDecodeError( endpointSource,
                                      wxString::Format( wxS( "placement has no pin at connection endpoint; "
                                                             "angle %d mirror %u" ),
                                                        placement.angle, placement.mirrorFlags ) );

                if( coincidentPinCount > 1 )
                {
                    SOURCE_PROPERTY property = sourceProperty(
                            wxS( "coincident_placement_pin_count" ),
                            wxString::Format( wxS( "%llu" ),
                                              static_cast<unsigned long long>( coincidentPinCount ) ),
                            endpointSource );
                    property.disposition = PROPERTY_DISPOSITION::PRESERVED;
                    result.properties.push_back( std::move( property ) );
                }

                result.kind = MODEL_ENDPOINT_KIND::PIN;
                result.placement = PLACEMENT_REFERENCE{ placement.id, endpointSource };
                result.pin = PIN_REFERENCE{ matchedPin->id, endpointSource };
                result.point = aWirePoint;
            }
            break;

        case 2:
            if( objectRecord >= aDecode.controllers.pools[19].count )
                throwDecodeError( endpointSource, wxS( "unresolved off-page endpoint" ) );

            result.kind = MODEL_ENDPOINT_KIND::POINT;
            result.point = offpagePosition( aDecode, objectRecord, aModel.version );
            break;

        case 3:
            if( objectRecord >= aDecode.controllers.pools[18].count )
                throwDecodeError( endpointSource, wxS( "unresolved junction endpoint" ) );

            result.kind = MODEL_ENDPOINT_KIND::POINT;
            result.point = junctionPosition( aDecode, objectRecord, aModel.version );
            break;

        default: throwDecodeError( endpointSource, wxS( "wrong endpoint object class" ) );
        }

        result.properties.push_back(
                sourceProperty( wxS( "raw_endpoint_handle" ), wxString::Format( wxS( "%u" ), raw ), endpointSource ) );
        SOURCE_PROPERTY relationshipProperty = sourceProperty(
                wxS( "raw_endpoint_relationship" ), wxString::Format( wxS( "%u" ), relationship ), relationshipSource );
        relationshipProperty.disposition = PROPERTY_DISPOSITION::PRESERVED;
        result.properties.push_back( std::move( relationshipProperty ) );
        return result;
    }


    void decodeConnections( const CONNECTIVITY_DECODE& aDecode, const VERTEX_TILING& aTiling,
                            const std::vector<MODEL_PLACEMENT*>&                             aPlacements,
                            const std::unordered_map<uint32_t, const MODEL_PIN_DEFINITION*>& aDefinitionPins,
                            SHEET_CONNECTIVITY& aConnectivity, PADS_SCH_MODEL& aModel )
    {
        aConnectivity.connections.assign( aDecode.controllers.pools[20].count, nullptr );
        aConnectivity.connectionNets.assign( aDecode.controllers.pools[20].count, nullptr );
        aConnectivity.junctionBacklinks.assign( aDecode.controllers.pools[18].count, {} );
        aConnectivity.offpageBacklinks.assign( aDecode.controllers.pools[19].count, {} );
        std::vector<std::array<uint16_t, 2>> connectionEndpointHandles( aDecode.controllers.pools[20].count );
        std::vector<size_t>                  connectionCounts( aConnectivity.sheetNets.size(), 0 );

        for( size_t record = 0; record < aDecode.controllers.pools[20].count; ++record )
        {
            const uint32_t netHandle =
                    aDecode.cursor.U32At( aDecode.connectionBase + record * CONNECTION_RECORD_BYTES + 8 );

            if( netHandle < connectionCounts.size() )
                ++connectionCounts[netHandle];
        }

        for( size_t net = 0; net < aConnectivity.sheetNets.size(); ++net )
        {
            if( aConnectivity.sheetNets[net] )
                aConnectivity.sheetNets[net]->connections.reserve( connectionCounts[net] );
        }

        for( size_t record = 0; record < aDecode.controllers.pools[20].count; ++record )
        {
            const size_t      offset = aDecode.connectionBase + record * CONNECTION_RECORD_BYTES;
            SOURCE_PROVENANCE source =
                    sourceAt( aDecode.sourceName, aModel.version, wxS( "connection" ), 21, record, offset,
                              CONNECTION_RECORD_BYTES, static_cast<int>( aDecode.sheetIndex ) );
            const uint32_t netHandle = aDecode.cursor.U32At( offset + 8 );

            if( netHandle >= aConnectivity.sheetNets.size() || !aConnectivity.sheetNets[netHandle] )
                throwDecodeError( source, wxS( "connection net handle targets wrong or unresolved object class" ) );

            const uint16_t marker = aDecode.cursor.U16At( offset + 34 );

            const uint8_t markerStatus = marker & 0xFF;

            if( marker >> 8 < 2 || marker >> 8 > 6
                || ( markerStatus != 0 && markerStatus != 0xFC && markerStatus != 0xFD ) )
                throwDecodeError( source, wxS( "invalid connection object-class marker" ) );

            MODEL_CONNECTION connection;
            connection.source = source;

            appendVertices( aDecode, aTiling, aDecode.controllers.pools[17].count + record, aModel.version,
                            connection.vertices );

            if( connection.vertices.size() < 2 )
                throwDecodeError( source, wxS( "connection lacks explicit endpoint vertices" ) );

            connection.endpoints.push_back( decodeConnectionEndpoint(
                    aDecode, source, 12, 16, connection.vertices.front(), aPlacements, aDefinitionPins, aModel ) );
            connection.endpoints.push_back( decodeConnectionEndpoint(
                    aDecode, source, 14, 20, connection.vertices.back(), aPlacements, aDefinitionPins, aModel ) );

            for( size_t endpointIndex = 0; endpointIndex < connection.endpoints.size(); ++endpointIndex )
            {
                MODEL_CONNECTION_ENDPOINT& decodedEndpoint = connection.endpoints[endpointIndex];
                const SOURCE_POINT&        wirePoint =
                        endpointIndex == 0 ? connection.vertices.front() : connection.vertices.back();

                if( decodedEndpoint.kind == MODEL_ENDPOINT_KIND::POINT
                    && ( decodedEndpoint.point.x != wirePoint.x || decodedEndpoint.point.y != wirePoint.y ) )
                {
                    throwDecodeError( decodedEndpoint.source,
                                      wxS( "typed endpoint position does not match wire endpoint vertex" ) );
                }

                decodedEndpoint.point = { wirePoint.x, wirePoint.y, wirePoint.source };
            }

            connectionEndpointHandles[record] = { aDecode.cursor.U16At( offset + 12 ),
                                                  aDecode.cursor.U16At( offset + 14 ) };

            for( size_t endpointIndex = 0; endpointIndex < connectionEndpointHandles[record].size(); ++endpointIndex )
            {
                const uint16_t       raw = connectionEndpointHandles[record][endpointIndex];
                const size_t         objectClass = raw >> 12;
                const size_t         objectRecord = raw & 0x0FFF;
                std::vector<size_t>* backlinks = nullptr;

                if( objectClass == 2 )
                    backlinks = &aConnectivity.offpageBacklinks[objectRecord];
                else if( objectClass == 3 )
                    backlinks = &aConnectivity.junctionBacklinks[objectRecord];

                if( backlinks )
                {
                    if( endpointIndex == 1 && connectionEndpointHandles[record][0] == raw )
                    {
                        SOURCE_PROVENANCE endpointSource = source;
                        endpointSource.objectClass = wxS( "connection endpoint" );
                        endpointSource.absoluteOffset += endpointIndex == 0 ? 12 : 14;
                        endpointSource.length = 2;
                        throwDecodeError( endpointSource, wxS( "duplicate typed endpoint backlink" ) );
                    }

                    backlinks->push_back( record );
                }
            }

            connection.properties.push_back(
                    sourceProperty( wxS( "raw_connection_marker" ), wxString::Format( wxS( "%u" ), marker ), source ) );
            aConnectivity.sheetNets[netHandle]->connections.push_back( std::move( connection ) );
            aConnectivity.connections[record] = &aConnectivity.sheetNets[netHandle]->connections.back();
            aConnectivity.connectionNets[record] = aConnectivity.sheetNets[netHandle];
        }
    }


    void validateJunctionBacklinks( const CONNECTIVITY_DECODE& aDecode, const SHEET_CONNECTIVITY& aConnectivity,
                                    PADS_SCH_MODEL& aModel )
    {
        for( size_t record = 0; record < aDecode.controllers.pools[18].count; ++record )
        {
            const size_t      offset = aDecode.junctionBase + record * JUNCTION_RECORD_BYTES;
            SOURCE_PROVENANCE source =
                    sourceAt( aDecode.sourceName, aModel.version, wxS( "junction" ), 19, record, offset,
                              JUNCTION_RECORD_BYTES, static_cast<int>( aDecode.sheetIndex ) );
            const uint16_t    owner = aDecode.cursor.U16At( offset + 8 );
            SOURCE_PROVENANCE ownerSource = source;
            ownerSource.absoluteOffset += 8;
            ownerSource.length = 2;

            if( owner >= aConnectivity.connections.size() || !aConnectivity.connections[owner] )
                throwDecodeError( ownerSource, wxS( "junction connection handle leaves controller 21" ) );

            if( std::ranges::find( aConnectivity.junctionBacklinks[record], owner )
                == aConnectivity.junctionBacklinks[record].end() )
                throwDecodeError( ownerSource, wxS( "junction connection handle does not point back" ) );

            for( size_t connection : aConnectivity.junctionBacklinks[record] )
            {
                if( aConnectivity.connectionNets[connection] != aConnectivity.connectionNets[owner] )
                {
                    throwDecodeError( ownerSource, wxS( "junction is shared across different nets" ) );
                }
            }
        }
    }


    void validateOffpageBacklinks( const CONNECTIVITY_DECODE& aDecode, const SHEET_CONNECTIVITY& aConnectivity,
                                   PADS_SCH_MODEL& aModel )
    {
        for( size_t record = 0; record < aDecode.controllers.pools[19].count; ++record )
        {
            const size_t      offset = aDecode.offpageBase + record * OFFPAGE_RECORD_BYTES;
            SOURCE_PROVENANCE source =
                    sourceAt( aDecode.sourceName, aModel.version, wxS( "off-page reference" ), 20, record, offset,
                              OFFPAGE_RECORD_BYTES, static_cast<int>( aDecode.sheetIndex ) );
            const uint16_t    owner = aDecode.cursor.U16At( offset + 8 );
            SOURCE_PROVENANCE ownerSource = source;
            ownerSource.absoluteOffset += 8;
            ownerSource.length = 2;

            if( owner >= aConnectivity.connections.size() || !aConnectivity.connections[owner] )
                throwDecodeError( ownerSource, wxS( "off-page net handle leaves controller 21" ) );

            if( std::ranges::find( aConnectivity.offpageBacklinks[record], owner )
                == aConnectivity.offpageBacklinks[record].end() )
                throwDecodeError( ownerSource, wxS( "off-page connection handle does not point back" ) );

            for( size_t connection : aConnectivity.offpageBacklinks[record] )
            {
                if( aConnectivity.connectionNets[connection] != aConnectivity.connectionNets[owner] )
                    throwDecodeError( ownerSource, wxS( "off-page reference is shared across different nets" ) );
            }
        }
    }


    std::vector<bool> decodeBuses( const CONNECTIVITY_DECODE& aDecode, const VERTEX_TILING& aTiling,
                                   const std::unordered_set<uint32_t>& aBusGlobalRecords,
                                   const SHEET_CONNECTIVITY& aConnectivity, PADS_SCH_MODEL& aModel )
    {
        std::vector<bool> claimedBusEntries( aDecode.controllers.pools[19].count, false );

        for( size_t record = 0; record < aDecode.controllers.pools[17].count; ++record )
        {
            const size_t      offset = aDecode.busBase + record * BUS_RECORD_BYTES;
            SOURCE_PROVENANCE source = sourceAt( aDecode.sourceName, aModel.version, wxS( "bus" ), 18, record, offset,
                                                 BUS_RECORD_BYTES, static_cast<int>( aDecode.sheetIndex ) );
            // PADS Logic v0x000D bus record +38 is vertex-count/status,
            // not an object-class enum limited to two, three or four points.
            // Verify the high byte against the actual controller-22 vertex
            // slice; this also rejects inconsistent or corrupt vertex counts.
            const uint16_t marker = aDecode.cursor.U16At( offset + 38 );
            const uint32_t markerVertexCount = marker >> 8;
            const uint8_t  markerStatus = marker & 0xFF;
            const uint32_t decodedVertexCount = aTiling.ends[record] - aTiling.starts[record];

            // 0xFC and 0xFD both occur in valid PADS v0x000D bus records.
            if( decodedVertexCount < 2 || markerVertexCount != decodedVertexCount
                || ( markerStatus != 0 && markerStatus != 0xFC && markerStatus != 0xFD ) )
                throwDecodeError( source, wxS( "invalid bus vertex-count/status marker" ) );

            const size_t globalRecord = aDecode.cursor.U32At( offset + 8 );

            if( globalRecord >= aDecode.globals.nets.size() || aDecode.globals.nets[globalRecord].tombstone
                || !aBusGlobalRecords.contains( globalRecord ) )
            {
                throwDecodeError( source, wxS( "bus global-net handle targets wrong or unresolved object class" ) );
            }

            const GLOBAL_NET_RECORD& global = aDecode.globals.nets[globalRecord];
            uint32_t                 membership = global.membershipStart;

            while( membership < global.membershipStart + global.membershipCount
                   && aDecode.globals.membershipSheets[membership] != aDecode.sheetIndex )
            {
                ++membership;
            }

            if( membership == global.membershipStart + global.membershipCount )
                throwDecodeError( source, wxS( "bus global-net handle does not belong to this sheet" ) );

            MODEL_BUS bus;
            bus.id = BUS_ID( static_cast<uint32_t>( aDecode.sheetIndex * 0x100000 + record + 1 ) );
            bus.source = source;
            bus.sheet = { aModel.sheets[aDecode.sheetIndex].id, source };
            bus.name = global.name;
            bus.aliases.push_back( global.name );
            bus.declaredMembers = global.aliasMembers;
            // Preserve the original 16-bit bus marker for future round-trip
            // work: the numeric status is validated but not yet interpreted.
            SOURCE_PROVENANCE busMarkerSource = source;
            busMarkerSource.absoluteOffset += 38;
            busMarkerSource.length = 2;
            SOURCE_PROPERTY busMarker =
                    sourceProperty( wxS( "raw_bus_vertex_status_marker" ),
                                    wxString::Format( wxS( "0x%04X" ), marker ), busMarkerSource );
            bus.properties.push_back( std::move( busMarker ) );
            SOURCE_PROPERTY identity =
                    sourceProperty( wxS( "preserved_net_identity" ),
                                    wxString::Format( wxS( "%u" ), global.preservedIdentity ), global.source );
            identity.disposition = PROPERTY_DISPOSITION::PRESERVED;
            bus.properties.push_back( std::move( identity ) );
            SOURCE_PROVENANCE relationshipSource = global.source;
            relationshipSource.absoluteOffset += 84;
            relationshipSource.length = 4;
            SOURCE_PROPERTY relationship =
                    sourceProperty( wxS( "preserved_net_relationship" ),
                                    wxString::Format( wxS( "%u" ), global.preservedRelationship ), relationshipSource );
            relationship.disposition = PROPERTY_DISPOSITION::PRESERVED;
            bus.properties.push_back( std::move( relationship ) );
            appendVertices( aDecode, aTiling, record, aModel.version, bus.vertices );

            std::vector<size_t>        entryRecords;
            uint16_t                   entryHandle = aDecode.cursor.U16At( offset + 24 );
            std::unordered_set<size_t> chain;

            while( ( entryHandle >> 12 ) == 2 )
            {
                const size_t entryRecord = entryHandle & 0x0FFF;

                if( entryRecord >= aDecode.controllers.pools[19].count )
                    throwDecodeError( source, wxS( "unresolved bus-entry handle" ) );

                if( !chain.insert( entryRecord ).second )
                    throwDecodeError( source, wxS( "cyclic bus-entry handle chain" ) );

                entryRecords.push_back( entryRecord );
                entryHandle = aDecode.cursor.U16At( aDecode.offpageBase + entryRecord * OFFPAGE_RECORD_BYTES + 4 );
            }

            if( record > 0x0FFF || entryHandle != 0xBFFF - record )
                throwDecodeError( source, wxS( "bus-entry chain terminates in wrong object class" ) );

            std::ranges::reverse( entryRecords );

            // PADS bus alias members define a set of scalar names and ranges
            // for the entire bus.  Their order is not the off-page entry order,
            // and equal list lengths do not imply a positional mapping.
            // Each entry's electrical net is validated through its actual
            // connection handle and membership ownership below.

            for( size_t entry = 0; entry < entryRecords.size(); ++entry )
            {
                const size_t      entryRecord = entryRecords[entry];
                const size_t      entryOffset = aDecode.offpageBase + entryRecord * OFFPAGE_RECORD_BYTES;
                SOURCE_PROVENANCE entrySource =
                        sourceAt( aDecode.sourceName, aModel.version, wxS( "bus entry" ), 20, entryRecord, entryOffset,
                                  OFFPAGE_RECORD_BYTES, static_cast<int>( aDecode.sheetIndex ) );

                if( aDecode.cursor.U8At( entryOffset + 30 ) != 0xFF )
                    throwDecodeError( entrySource, wxS( "bus-entry handle targets wrong off-page object class" ) );

                if( claimedBusEntries[entryRecord] )
                    throwDecodeError( entrySource, wxS( "duplicate bus-entry membership" ) );

                const uint16_t connectionHandle = aDecode.cursor.U16At( entryOffset + 8 );

                if( connectionHandle >= aConnectivity.connections.size()
                    || !aConnectivity.connections[connectionHandle] )
                    throwDecodeError( entrySource, wxS( "unresolved bus-entry connection reference" ) );

                MODEL_CONNECTION* entryConnection = aConnectivity.connections[connectionHandle];
                MODEL_NET*        memberNet = aConnectivity.connectionNets[connectionHandle];

                if( !memberNet )
                    throwDecodeError( entrySource, wxS( "bus-entry connection targets wrong member-net class" ) );

                const bool ownsConnection = std::ranges::any_of( memberNet->connections,
                                                                 [&]( const MODEL_CONNECTION& aConnection )
                                                                 {
                                                                     return &aConnection == entryConnection;
                                                                 } );

                if( !ownsConnection )
                    throwDecodeError( entrySource, wxS( "bus-entry connection targets wrong member-net class" ) );

                claimedBusEntries[entryRecord] = true;
                MODEL_BUS_ENTRY busEntry;
                busEntry.source = entrySource;
                busEntry.position = offpagePosition( aDecode, entryRecord, aModel.version );
                busEntry.memberNet = { memberNet->id, entrySource };
                // The bus entry's +8 handle is the exact wire owner; geometry
                // alone cannot distinguish coincident endpoints on one net.
                busEntry.connectionRecordIndex = connectionHandle;
                bus.entries.push_back( std::move( busEntry ) );
                bus.memberNets.push_back( { memberNet->id, entrySource } );
            }

            aModel.buses.push_back( std::move( bus ) );
        }

        return claimedBusEntries;
    }



    int64_t binaryOffPageBodyHeight( const MODEL_SYMBOL_DEFINITION& aDefinition )
    {
        bool    found = false;
        int64_t minY = 0;
        int64_t maxY = 0;

        auto consider = [&]( int64_t aY )
        {
            if( !found )
            {
                minY = aY;
                maxY = aY;
                found = true;
                return;
            }

            minY = std::min( minY, aY );
            maxY = std::max( maxY, aY );
        };

        for( const MODEL_GRAPHIC& graphic : aDefinition.graphics )
        {
            if( graphic.kind == MODEL_GRAPHIC_KIND::TEXT )
                continue;

            if( graphic.kind == MODEL_GRAPHIC_KIND::CIRCLE && graphic.points.size() >= 2 )
            {
                const double cx =
                        ( static_cast<double>( graphic.points[0].x ) + graphic.points[1].x ) / 2.0;
                const double cy =
                        ( static_cast<double>( graphic.points[0].y ) + graphic.points[1].y ) / 2.0;
                const double radius =
                        std::hypot( static_cast<double>( graphic.points[1].x - graphic.points[0].x ),
                                    static_cast<double>( graphic.points[1].y - graphic.points[0].y ) )
                        / 2.0;
                static_cast<void>( cx );
                consider( static_cast<int64_t>( std::llround( cy - radius ) ) );
                consider( static_cast<int64_t>( std::llround( cy + radius ) ) );
                continue;
            }

            if( graphic.kind == MODEL_GRAPHIC_KIND::ARC )
            {
                consider( graphic.arcBoundsStart.y );
                consider( graphic.arcBoundsEnd.y );
            }

            for( const SOURCE_POINT& point : graphic.points )
                consider( point.y );
        }

        if( !found || maxY <= minY )
            return 0;

        return maxY - minY;
    }


    void decodeOffpageLabels( const CONNECTIVITY_DECODE& aDecode, const SHEET_CONNECTIVITY& aConnectivity,
                              const std::vector<bool>& aClaimedBusEntries, PADS_SCH_MODEL& aModel )
    {
        for( size_t record = 0; record < aDecode.controllers.pools[19].count; ++record )
        {
            const size_t      offset = aDecode.offpageBase + record * OFFPAGE_RECORD_BYTES;
            SOURCE_PROVENANCE source =
                    sourceAt( aDecode.sourceName, aModel.version, wxS( "off-page reference" ), 20, record, offset,
                              OFFPAGE_RECORD_BYTES, static_cast<int>( aDecode.sheetIndex ) );
            const uint8_t rawKind = aDecode.cursor.U8At( offset + 30 );

            if( rawKind == 0xFF )
            {
                if( !aClaimedBusEntries[record] )
                    throwDecodeError( source, wxS( "unclaimed bus-entry record" ) );

                continue;
            }

            const uint16_t connectionHandle = aDecode.cursor.U16At( offset + 8 );

            if( connectionHandle >= aConnectivity.connections.size() || !aConnectivity.connections[connectionHandle] )
                throwDecodeError( source, wxS( "off-page net handle leaves controller 21" ) );

            MODEL_NET* ownerNet = aConnectivity.connectionNets[connectionHandle];

            if( !ownerNet )
                throwDecodeError( source, wxS( "off-page record targets wrong net object class" ) );

            MODEL_LABEL label;
            label.source = source;
            label.sheet = { aModel.sheets[aDecode.sheetIndex].id, source };
            label.text = ownerNet->name;
            label.position = offpagePosition( aDecode, record, aModel.version );

            // Apply the PADS off-page mirror flags to the label direction.  The ASCII
            // rotation/mirror pair maps to binary words +26/+28; ignoring +28 reverses
            // horizontal $OSR_SYMS attached to left-facing pins.
            int32_t        offpageAngle = NormalizeAngle( aDecode.cursor.U16At( offset + 26 ) );
            const uint16_t offpageMirrorFlags = aDecode.cursor.U16At( offset + 28 );

            if( offpageMirrorFlags & 1 )
                offpageAngle = NormalizeAngle( 1800 - offpageAngle );

            if( offpageMirrorFlags & 2 )
                offpageAngle = NormalizeAngle( -offpageAngle );

            // Preserve OFFPAGE orientation even when NETNAMES later sets text angle.
            label.symbolAngle = offpageAngle;
            label.symbolVariant = rawKind;
            label.offpageMirrorFlags = offpageMirrorFlags;
            SOURCE_PROVENANCE mirrorSource = source;
            mirrorSource.absoluteOffset += 28;
            mirrorSource.length = 2;
            label.properties.push_back(
                    sourceProperty( wxS( "offpage_mirror_flags" ),
                                    wxString::Format( wxS( "%u" ), offpageMirrorFlags ), mirrorSource ) );

            if( offpageMirrorFlags & ~3U )
            {
                SOURCE_PROPERTY& property = label.properties.back();
                property.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
                aModel.diagnostics.push_back( MakePropertyDiagnostic(
                        RPT_SEVERITY_WARNING, property,
                        wxS( "unsupported PADS off-page mirror flag bits preserved" ) ) );
            }

            if( rawKind == 0xFE )
            {
                label.kind = MODEL_LABEL_KIND::LOCAL;
            }
            else
            {
                const uint16_t decalHandle = aDecode.cursor.U16At( offset + 4 );

                if( decalHandle == 0xFFFF )
                {
                    label.kind = MODEL_LABEL_KIND::GLOBAL;
                }
                else
                {
                    if( decalHandle >= aDecode.controllers.pools[6].count )
                        throwDecodeError( source, wxS( "off-page decal handle leaves controller 7" ) );

                    SOURCE_PROVENANCE decalSource =
                            sourceAt( aDecode.sourceName, aModel.version, wxS( "used decal" ), 7, decalHandle,
                                      aDecode.controllers.offsets[6] + decalHandle * USED_DECAL_BYTES, 40,
                                      static_cast<int>( aDecode.sheetIndex ) );
                    SOURCE_STRING  decalName = decodeFixedString( aDecode.bytes, decalSource.absoluteOffset,
                                                                  decalSource.length, decalSource, aModel.diagnostics );
                    label.decalName = decalName;
                    const uint32_t definitionRecord = aDecode.cursor.U32At( decalSource.absoluteOffset + 48 );

                    if( definitionRecord >= aDecode.controllers.pools[2].count )
                        throwDecodeError( decalSource, wxS( "off-page decal definition leaves controller 3" ) );

                    const DEFINITION_ID definitionId(
                            static_cast<uint32_t>( aDecode.sheetIndex * 0x100000 + 1 + definitionRecord ) );

                    // Preserve the CAEDECAL's default power/ground name anchor.
                    // PADS stores this in REF-DES even though KiCad represents
                    // the displayed net name as the symbol Value.
                    auto definition = std::ranges::find(
                            aModel.definitions, definitionId, &MODEL_SYMBOL_DEFINITION::id );

                    if( definition == aModel.definitions.end() )
                        throwDecodeError( decalSource, wxS( "off-page decal definition is unresolved" ) );

                    label.offpageBodyHeight = binaryOffPageBodyHeight( *definition );

                    for( const MODEL_FIELD& field : definition->fields )
                    {
                        if( field.name.text.CmpNoCase( wxS( "REF-DES" ) ) != 0 )
                            continue;

                        label.hasDefaultPresentation = true;
                        label.defaultTextOffset = field.position;
                        label.defaultAngle = field.angle;
                        label.defaultPresentation = field.presentation;
                        break;
                    }
                    auto specialPartOwnsDefinition = [&]( const wxString& aPartName )
                    {
                        return std::ranges::any_of(
                                aModel.partTypes,
                                [&]( const MODEL_PART_TYPE& aPart )
                                {
                                    if( aPart.name.text != aPartName )
                                        return false;

                                    return std::ranges::any_of(
                                            aPart.gates,
                                            [&]( const MODEL_GATE& aGate )
                                            {
                                                return aGate.definition.id == definitionId
                                                       || std::ranges::any_of( aGate.alternateDefinitions,
                                                                               [&]( const DEFINITION_REFERENCE& aRef )
                                                                               {
                                                                                   return aRef.id == definitionId;
                                                                               } );
                                            } );
                                } );
                    };

                    if( specialPartOwnsDefinition( wxS( "$OSR_SYMS" ) ) )
                        label.kind = MODEL_LABEL_KIND::GLOBAL;
                    else if( specialPartOwnsDefinition( wxS( "$GND_SYMS" ) ) )
                        label.kind = MODEL_LABEL_KIND::GROUND;
                    else if( specialPartOwnsDefinition( wxS( "$PWR_SYMS" ) ) )
                        label.kind = MODEL_LABEL_KIND::POWER;
                    else
                    {
                        label.kind = MODEL_LABEL_KIND::UNSUPPORTED;
                        SOURCE_PROPERTY unsupportedDecal =
                                sourceProperty( wxS( "unsupported_offpage_decal" ), decalName.text, decalSource );
                        unsupportedDecal.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
                        aModel.diagnostics.push_back(
                                MakePropertyDiagnostic( RPT_SEVERITY_WARNING, unsupportedDecal,
                                                        wxS( "unsupported off-page decal class preserved" ) ) );
                        label.properties.push_back( std::move( unsupportedDecal ) );
                    }
                }
            }

            if( label.kind == MODEL_LABEL_KIND::GLOBAL )
            {
                for( uint32_t membership =
                             aDecode.globals
                                     .nets[aDecode.cursor.U32At( aDecode.connectionBase
                                                                 + connectionHandle * CONNECTION_RECORD_BYTES + 8 )]
                                     .membershipStart;
                     membership
                     < aDecode.globals.nets[aDecode.cursor.U32At( aDecode.connectionBase
                                                                  + connectionHandle * CONNECTION_RECORD_BYTES + 8 )]
                                       .membershipStart
                               + aDecode.globals
                                         .nets[aDecode.cursor.U32At( aDecode.connectionBase
                                                                     + connectionHandle * CONNECTION_RECORD_BYTES + 8 )]
                                         .membershipCount;
                     ++membership )
                {
                    const uint16_t peerSheet = aDecode.globals.membershipSheets[membership];

                    if( peerSheet != aDecode.sheetIndex )
                        label.linkedSheets.push_back( { aModel.sheets[peerSheet].id, source } );
                }
            }

            // PADS only displays a power/ground net name when a NETNAMES
            // presentation record exists for that instance.
            if( label.kind == MODEL_LABEL_KIND::GROUND || label.kind == MODEL_LABEL_KIND::POWER )
                label.presentation.visible = false;

            label.properties.push_back(
                    sourceProperty( wxS( "offpage_variant" ), wxString::Format( wxS( "%u" ), rawKind ), source ) );
            aModel.labels.push_back( std::move( label ) );
        }
    }


    MODEL_TEXT_PRESENTATION netNamePresentation( const CONNECTIVITY_DECODE& aDecode, size_t aOffset,
                                                 const SOURCE_PROVENANCE& aSource, PADS_SCH_MODEL& aModel )
    {
        MODEL_TEXT_PRESENTATION presentation;
        presentation.source = aSource;
        // NETNAMES stores height/line-width in whole mils.  The semantic
        // binary model stores text metrics in half-mils.
        presentation.height = static_cast<int64_t>( aDecode.cursor.U16At( aOffset + 2 ) ) * 2;
        presentation.width = static_cast<int64_t>( aDecode.cursor.U16At( aOffset + 4 ) ) * 2;
        // PADS NETNAMES uses the same 90/270-degree side-selector bit swap as
        // PADS field text.  Normalize before mapping the 9-way justification.
        const int textAngle = NormalizeAngle( aDecode.cursor.U16At( aOffset + 24 ) );
        const uint16_t justification = rotatedFieldJustification(
                aDecode.cursor.U16At( aOffset + 26 ), textAngle );
        presentation.horizontalJustification = horizontalJustification( justification );
        presentation.verticalJustification = verticalJustification( justification );
        const int16_t     fontHandle = static_cast<int16_t>( aDecode.cursor.U16At( aOffset ) );
        SOURCE_PROVENANCE fontSource = aSource;
        fontSource.length = 2;

        if( fontHandle == -1 )
        {
            presentation.font = decodedDefinitionFont( -1, fontSource );
        }
        else
        {
            if( fontHandle < 0 || static_cast<uint32_t>( fontHandle ) >= aDecode.globals.fontCount )
                throwDecodeError( fontSource, wxS( "net-name font handle leaves outer controller 19" ) );

            const size_t fontOffset = aDecode.globals.fontBase + static_cast<size_t>( fontHandle ) * FONT_RECORD_BYTES;
            SOURCE_PROVENANCE nameSource = sourceAt( aDecode.sourceName, aModel.version, wxS( "net-name font" ), 19,
                                                     fontHandle, fontOffset + 4, 32, -1 );
            presentation.font = decodeFixedString( aDecode.bytes, fontOffset + 4, 32, nameSource, aModel.diagnostics );
            const uint32_t style = aDecode.cursor.U32At( fontOffset );
            presentation.bold = style & 1;
            presentation.italic = style & 2;

            if( style & ~3U )
            {
                SOURCE_PROPERTY property = sourceProperty( wxS( "unsupported_font_style_flags" ),
                                                           wxString::Format( wxS( "%u" ), style & ~3U ), fontSource );
                property.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
                presentation.properties.push_back( property );
                aModel.diagnostics.push_back( MakePropertyDiagnostic(
                        RPT_SEVERITY_WARNING, property, wxS( "unsupported net-name font style flags preserved" ) ) );
            }
        }

        presentation.properties.push_back(
                sourceProperty( wxS( "font_handle" ), wxString::Format( wxS( "%d" ), fontHandle ), fontSource ) );
        return presentation;
    }


    void decodeNetNames( const CONNECTIVITY_DECODE& aDecode, const SHEET_CONNECTIVITY& aConnectivity,
                         PADS_SCH_MODEL& aModel )
    {
        for( size_t record = 0; record < aDecode.controllers.pools[22].count; ++record )
        {
            const size_t      offset = aDecode.netNameBase + record * NET_NAME_RECORD_BYTES;
            SOURCE_PROVENANCE source =
                    sourceAt( aDecode.sourceName, aModel.version, wxS( "net-name presentation" ), 23, record, offset,
                              NET_NAME_RECORD_BYTES, static_cast<int>( aDecode.sheetIndex ) );
            const uint32_t               globalRecord = aDecode.cursor.U32At( offset + 16 );
            const uint16_t               ownerHandle = aDecode.cursor.U16At( offset + 38 );
            const uint16_t               childHandle = aDecode.cursor.U16At( offset + 40 );
            std::vector<SOURCE_PROPERTY> preservedPresentation;
            auto preserve = [&]( const wxString& aName, const wxString& aValue, size_t aRelativeOffset, size_t aLength )
            {
                SOURCE_PROVENANCE propertySource = source;
                propertySource.absoluteOffset += aRelativeOffset;
                propertySource.length = aLength;
                SOURCE_PROPERTY property = sourceProperty( aName, aValue, propertySource );
                property.disposition = PROPERTY_DISPOSITION::PRESERVED;
                preservedPresentation.push_back( std::move( property ) );
            };
            wxString presentation06;

            for( size_t index = 6; index < 16; ++index )
                presentation06 += wxString::Format( wxS( "%02x" ), aDecode.bytes[offset + index] );

            preserve( wxS( "preserved_net_name_presentation_06" ), presentation06, 6, 10 );
            preserve( wxS( "preserved_net_name_secondary_offset" ),
                      wxString::Format( wxS( "%d,%d" ), static_cast<int16_t>( aDecode.cursor.U16At( offset + 28 ) ),
                                        static_cast<int16_t>( aDecode.cursor.U16At( offset + 30 ) ) ),
                      28, 4 );
            preserve( wxS( "preserved_net_name_presentation_20" ),
                      wxString::Format( wxS( "%u" ), aDecode.cursor.U16At( offset + 32 ) ), 32, 2 );
            preserve( wxS( "preserved_net_name_presentation_flags" ),
                      wxString::Format( wxS( "%u" ), aDecode.cursor.U16At( offset + 34 ) ), 34, 2 );
            preserve( wxS( "preserved_net_name_predecessor_handle" ),
                      wxString::Format( wxS( "%u" ), aDecode.cursor.U16At( offset + 36 ) ), 36, 2 );
            preserve( wxS( "preserved_net_name_predecessor_record" ),
                      wxString::Format( wxS( "%u" ), aDecode.cursor.U16At( offset + 42 ) ), 42, 2 );
            preserve( wxS( "preserved_net_name_successor_record" ),
                      wxString::Format( wxS( "%u" ), aDecode.cursor.U16At( offset + 44 ) ), 44, 2 );
            preserve( wxS( "preserved_net_name_tail" ),
                      wxString::Format( wxS( "%u" ), aDecode.cursor.U16At( offset + 46 ) ), 46, 2 );

            if( ( ownerHandle & 0xF000 ) == 0x4000 )
            {
                if( globalRecord >= aDecode.globals.nets.size() || aDecode.globals.nets[globalRecord].tombstone )
                    throwDecodeError( source, wxS( "bus net-name record targets wrong global-net object class" ) );

                const size_t busRecord = ownerHandle & 0x0FFF;
                auto         bus = std::ranges::find_if( aModel.buses,
                                                         [&]( const MODEL_BUS& aBus )
                                                         {
                                                     return aBus.sheet.id == aModel.sheets[aDecode.sheetIndex].id
                                                            && aBus.source.recordIndex == busRecord;
                                                 } );

                // The second index in PADS @@@B<bus>.<anchor> names a bus
                // polyline anchor, not a one-based index into memberNets.
                // In particular, @@@B0.0 is valid and must not be rejected.
                if( bus == aModel.buses.end() )
                    throwDecodeError( source, wxS( "net-name bus owner targets wrong object class" ) );

                if( bus->name.text != aDecode.globals.nets[globalRecord].name.text )
                    throwDecodeError( source, wxS( "net-name bus owner targets a different bus name" ) );

                if( childHandle >= bus->vertices.size() )
                    throwDecodeError( source, wxS( "net-name bus anchor leaves bus vertex range" ) );

                SOURCE_PROVENANCE anchorSource = source;
                anchorSource.absoluteOffset += 40;
                anchorSource.length = 2;
                SOURCE_PROPERTY anchorIndex = sourceProperty(
                        wxS( "preserved_bus_net_name_anchor_index" ),
                        wxString::Format( wxS( "%u" ), childHandle ), anchorSource );
                anchorIndex.disposition = PROPERTY_DISPOSITION::PRESERVED;
                bus->properties.push_back( std::move( anchorIndex ) );

                bus->properties.push_back( sourceProperty( wxS( "net_name_presentation_record" ),
                                                           wxString::Format( wxS( "%llu" ), record ), source ) );
                bus->properties.insert( bus->properties.end(), std::make_move_iterator( preservedPresentation.begin() ),
                                        std::make_move_iterator( preservedPresentation.end() ) );
                continue;
            }

            if( globalRecord >= aConnectivity.sheetNets.size() || !aConnectivity.sheetNets[globalRecord] )
                throwDecodeError( source, wxS( "net-name record targets wrong or unresolved net object class" ) );

            MODEL_NET&              ownerNet = *aConnectivity.sheetNets[globalRecord];
            const SOURCE_POINT      textOffset{ decodeTerminalCoordinate( aDecode.cursor.U16At( offset + 20 ) ),
                                           decodeTerminalCoordinate( aDecode.cursor.U16At( offset + 22 ) ), source };
            MODEL_TEXT_PRESENTATION presentation = netNamePresentation( aDecode, offset, source, aModel );
            presentation.properties.insert( presentation.properties.end(),
                                            std::make_move_iterator( preservedPresentation.begin() ),
                                            std::make_move_iterator( preservedPresentation.end() ) );

            if( ( ownerHandle & 0xF000 ) == 0x2000 )
            {
                const size_t offpageRecord = ownerHandle & 0x0FFF;
                auto         label = std::ranges::find_if( aModel.labels,
                                                           [&]( const MODEL_LABEL& aLabel )
                                                           {
                                                       return aLabel.sheet.id == aModel.sheets[aDecode.sheetIndex].id
                                                              && aLabel.source.controller == 20
                                                              && aLabel.source.recordIndex == offpageRecord;
                                                   } );

                if( label == aModel.labels.end() )
                {
                    const MODEL_BUS_ENTRY* busEntry = nullptr;

                    for( const MODEL_BUS& bus : aModel.buses )
                    {
                        if( bus.sheet.id != aModel.sheets[aDecode.sheetIndex].id )
                            continue;

                        auto candidate = std::ranges::find_if( bus.entries,
                                                               [&]( const MODEL_BUS_ENTRY& aEntry )
                                                               {
                                                                   return aEntry.source.recordIndex == offpageRecord
                                                                          && aEntry.memberNet.id == ownerNet.id;
                                                               } );

                        if( candidate != bus.entries.end() )
                        {
                            if( busEntry )
                                throwDecodeError( source, wxS( "net-name bus-entry owner is ambiguous" ) );

                            busEntry = &*candidate;
                        }
                    }

                    if( !busEntry )
                        throwDecodeError( source, wxS( "net-name off-page owner targets wrong object class" ) );

                    MODEL_TEXT busText;
                    busText.source = source;
                    busText.sheet = { aModel.sheets[aDecode.sheetIndex].id, source };
                    busText.text = ownerNet.name;
                    busText.position = { busEntry->position.x + textOffset.x, busEntry->position.y + textOffset.y,
                                         source };
                    busText.angle = NormalizeAngle( aDecode.cursor.U16At( offset + 24 ) );
                    busText.presentation = std::move( presentation );
                    busText.properties.push_back( sourceProperty(
                            wxS( "net_name_text_offset" ),
                            wxString::Format( wxS( "%lld,%lld" ), textOffset.x, textOffset.y ), source ) );
                    aModel.texts.push_back( std::move( busText ) );
                    continue;
                }

                if( label->text.text != ownerNet.name.text )
                    throwDecodeError( source, wxS( "net-name off-page owner targets wrong net object" ) );

                label->presentation = std::move( presentation );
                label->textOffset = textOffset;
                label->angle = NormalizeAngle( aDecode.cursor.U16At( offset + 24 ) );
                label->hasNetNamePresentation = true;
                label->properties.push_back(
                        sourceProperty( wxS( "net_name_text_offset" ),
                                        wxString::Format( wxS( "%lld,%lld" ), textOffset.x, textOffset.y ), source ) );
                continue;
            }

            auto placement =
                    std::ranges::find_if( aModel.placements,
                                          [&]( const MODEL_PLACEMENT& aPlacement )
                                          {
                                              return aPlacement.sheet.id == aModel.sheets[aDecode.sheetIndex].id
                                                     && aPlacement.source.recordIndex == ownerHandle;
                                          } );

            if( placement == aModel.placements.end() )
                throwDecodeError( source, wxS( "net-name placement owner targets wrong object class" ) );

            if( childHandle >= placement->pins.size() )
                throwDecodeError( source, wxS( "net-name placement pin ordinal leaves placement" ) );

            const PIN_ID ownerPin = placement->pins[childHandle].id;

            const MODEL_CONNECTION_ENDPOINT* endpoint = nullptr;

            for( const MODEL_CONNECTION& connection : ownerNet.connections )
            {
                for( const MODEL_CONNECTION_ENDPOINT& candidate : connection.endpoints )
                {
                    if( candidate.kind != MODEL_ENDPOINT_KIND::PIN || !candidate.placement || !candidate.pin
                        || candidate.placement->id != placement->id || candidate.pin->id != ownerPin )
                    {
                        continue;
                    }

                    if( endpoint )
                        throwDecodeError( source, wxS( "net-name placement pin owner is ambiguous" ) );

                    endpoint = &candidate;
                }
            }

            if( !endpoint )
                throwDecodeError( source, wxS( "net-name placement pin owner is unresolved" ) );

            MODEL_TEXT text;
            text.source = source;
            text.sheet = { aModel.sheets[aDecode.sheetIndex].id, source };
            text.text = ownerNet.name;
            text.position = { endpoint->point.x + textOffset.x, endpoint->point.y + textOffset.y, source };
            text.angle = NormalizeAngle( aDecode.cursor.U16At( offset + 24 ) );
            text.presentation = std::move( presentation );
            text.properties.push_back(
                    sourceProperty( wxS( "net_name_text_offset" ),
                                    wxString::Format( wxS( "%lld,%lld" ), textOffset.x, textOffset.y ), source ) );
            aModel.texts.push_back( std::move( text ) );
        }
    }


    void decodeConnectivity( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                             const SCH_SDB_BLOCK& aBlock, size_t aSheetIndex, const wxString& aSourceName,
                             const CONNECTIVITY_GLOBALS& aGlobals, PADS_SCH_MODEL& aModel )
    {
        if( aModel.version != 0x000D )
            throwDecodeError( aModel.source, wxS( "connectivity decoder selected for raw-preserved version" ) );

        const SHEET_CONTROLLERS controllers = sheetControllers( aCursor, aBlock );
        requireFixedController( controllers, 18, BUS_RECORD_BYTES, aSourceName, aModel.version, aSheetIndex );
        requireFixedController( controllers, 19, JUNCTION_RECORD_BYTES, aSourceName, aModel.version, aSheetIndex );
        requireFixedController( controllers, 20, OFFPAGE_RECORD_BYTES, aSourceName, aModel.version, aSheetIndex );
        requireFixedController( controllers, 21, CONNECTION_RECORD_BYTES, aSourceName, aModel.version, aSheetIndex );
        requireFixedController( controllers, 22, CONNECTION_VERTEX_BYTES, aSourceName, aModel.version, aSheetIndex );
        requireFixedController( controllers, 23, NET_NAME_RECORD_BYTES, aSourceName, aModel.version, aSheetIndex );

        const CONNECTIVITY_DECODE decode{ aBytes,
                                          aCursor,
                                          controllers,
                                          aGlobals,
                                          aSourceName,
                                          aSheetIndex,
                                          controllers.offsets[17],
                                          controllers.offsets[18],
                                          controllers.offsets[19],
                                          controllers.offsets[20],
                                          controllers.offsets[21],
                                          controllers.offsets[22] };

        const std::unordered_set<uint32_t> busGlobalRecords = decodeBusGlobalRecords( decode, aModel );
        SHEET_CONNECTIVITY                 connectivity;
        connectivity.sheetNets = materializeSheetNets( decode, busGlobalRecords, aModel );

        const std::unordered_map<uint32_t, const MODEL_PIN_DEFINITION*> definitionPins = indexDefinitionPins( aModel );
        const std::vector<MODEL_PLACEMENT*> placements = indexSheetPlacements( decode, aModel );

        decodeJunctions( decode, aModel );

        const VERTEX_TILING tiling = tileConnectivityVertices( decode, aModel );

        decodeConnections( decode, tiling, placements, definitionPins, connectivity, aModel );
        validateJunctionBacklinks( decode, connectivity, aModel );
        validateOffpageBacklinks( decode, connectivity, aModel );

        const std::vector<bool> claimedBusEntries =
                decodeBuses( decode, tiling, busGlobalRecords, connectivity, aModel );

        decodeOffpageLabels( decode, connectivity, claimedBusEntries, aModel );
        decodeNetNames( decode, connectivity, aModel );
    }


    void inheritDefaultSystemFont( MODEL_TEXT_PRESENTATION& aPresentation, const DESIGN_SETTINGS& aSettings )
    {
        if( aSettings.fontMode != MODEL_FONT_MODE::SYSTEM || aSettings.defaultSystemFont.text.IsEmpty() )
            return;

        if( aPresentation.font.text.IsEmpty()
            || aPresentation.font.text == wxS( "Default Font" ) )
        {
            aPresentation.font = aSettings.defaultSystemFont;
        }
    }


    void inheritDefaultSystemFonts( PADS_SCH_MODEL& aModel )
    {
        const DESIGN_SETTINGS& settings = aModel.settings;

        const auto applyField = [&]( MODEL_FIELD& aField )
        {
            inheritDefaultSystemFont( aField.presentation, settings );
        };

        const auto applyGraphic = [&]( MODEL_GRAPHIC& aGraphic )
        {
            if( aGraphic.kind == MODEL_GRAPHIC_KIND::TEXT )
                inheritDefaultSystemFont( aGraphic.presentation, settings );
        };

        for( MODEL_SHEET& sheet : aModel.sheets )
        {
            std::ranges::for_each( sheet.border, applyGraphic );
            std::ranges::for_each( sheet.titleBlockFields, applyField );
        }

        for( MODEL_SYMBOL_DEFINITION& definition : aModel.definitions )
        {
            std::ranges::for_each( definition.graphics, applyGraphic );
            std::ranges::for_each( definition.fields, applyField );

            for( MODEL_PIN_DEFINITION& pin : definition.pins )
            {
                inheritDefaultSystemFont( pin.presentation, settings );
                inheritDefaultSystemFont( pin.namePresentation, settings );
                inheritDefaultSystemFont( pin.numberPresentation, settings );
            }
        }

        for( MODEL_PART_TYPE& partType : aModel.partTypes )
            std::ranges::for_each( partType.fields, applyField );

        for( MODEL_PLACEMENT& placement : aModel.placements )
            std::ranges::for_each( placement.fields, applyField );

        for( MODEL_LABEL& label : aModel.labels )
        {
            inheritDefaultSystemFont( label.presentation, settings );

            if( label.hasDefaultPresentation )
                inheritDefaultSystemFont( label.defaultPresentation, settings );
        }

        for( MODEL_TEXT& text : aModel.texts )
            inheritDefaultSystemFont( text.presentation, settings );

        for( MODEL_PAGE_GRAPHIC& graphic : aModel.graphics )
            applyGraphic( graphic.graphic );

        for( MODEL_WORKSHEET& worksheet : aModel.worksheets )
            std::ranges::for_each( worksheet.graphics, applyGraphic );
    }


    void assignFieldIds( PADS_SCH_MODEL& aModel )
    {
        auto assign = []( MODEL_FIELD& aField, FIELD_ID_DOMAIN aDomain, uint32_t aOwner, size_t aOrdinal )
        {
            if( aOrdinal > FIELD_ID_MAX_ORDINAL )
                throwDecodeError( aField.source, wxS( "field ordinal exceeds identity capacity" ) );

            aField.id = MakeFieldId( aDomain, aOwner, static_cast<uint32_t>( aOrdinal ) );
        };

        for( MODEL_SHEET& sheet : aModel.sheets )
        {
            for( size_t ordinal = 0; ordinal < sheet.titleBlockFields.size(); ++ordinal )
            {
                MODEL_FIELD& field = sheet.titleBlockFields[ordinal];
                assign( field, FIELD_ID_DOMAIN::SHEET, sheet.id.Value(), ordinal );
            }
        }

        for( MODEL_SYMBOL_DEFINITION& definition : aModel.definitions )
        {
            for( size_t ordinal = 0; ordinal < definition.fields.size(); ++ordinal )
            {
                MODEL_FIELD& field = definition.fields[ordinal];
                assign( field, FIELD_ID_DOMAIN::DEFINITION, definition.id.Value(), ordinal );
            }
        }

        for( MODEL_PART_TYPE& partType : aModel.partTypes )
        {
            for( size_t ordinal = 0; ordinal < partType.fields.size(); ++ordinal )
            {
                MODEL_FIELD& field = partType.fields[ordinal];
                assign( field, FIELD_ID_DOMAIN::PART_TYPE, partType.id.Value(), ordinal );
            }
        }

        for( MODEL_PLACEMENT& placement : aModel.placements )
        {
            for( size_t ordinal = 0; ordinal < placement.fields.size(); ++ordinal )
            {
                MODEL_FIELD& field = placement.fields[ordinal];
                assign( field, FIELD_ID_DOMAIN::PLACEMENT, placement.id.Value(), ordinal );
            }
        }
    }


    template <typename Item>
    void validateSheetRef( const Item& aItem, const MODEL_REFERENCE_INDEX& aIndex, const wxString& aObjectClass )
    {
        if( !aIndex.sheets.contains( aItem.sheet.id.Value() ) )
        {
            throwValidationError( aItem.sheet.source,
                                  wxString::Format( wxS( "unresolved %s sheet reference" ), aObjectClass ) );
        }
    }


    void validateIdentity( const PADS_SCH_MODEL& aModel )
    {
        validateUniqueIds( aModel.sheets, wxS( "sheet" ), itemId, itemProvenance );
        validateUniqueIds( aModel.definitions, wxS( "definition" ), itemId, itemProvenance );
        validateUniqueIds( aModel.partTypes, wxS( "part type" ), itemId, itemProvenance );
        validateUniqueIds( aModel.placements, wxS( "placement" ), itemId, itemProvenance );
        validateUniqueIds( aModel.nets, wxS( "net" ), itemId, itemProvenance );
        validateUniqueIds( aModel.buses, wxS( "bus" ), itemId, itemProvenance );
        validateUniqueIds( aModel.images, wxS( "embedded image" ), itemId, itemProvenance );

        std::vector<bool> sheetIndexes( aModel.sheets.size() );

        for( const MODEL_SHEET& sheet : aModel.sheets )
        {
            if( sheet.index >= sheetIndexes.size() )
                throwValidationError( sheet.source, wxS( "sheet source index leaves the declared sheet range" ) );

            if( sheetIndexes[sheet.index] )
                throwValidationError( sheet.source, wxS( "duplicate sheet source index" ) );

            sheetIndexes[sheet.index] = true;
        }

        std::unordered_map<uint32_t, SOURCE_PROVENANCE> gateDeclarations;
        std::unordered_map<uint32_t, SOURCE_PROVENANCE> pinDeclarations;
        std::unordered_map<uint64_t, SOURCE_PROVENANCE> fieldDeclarations;

        for( const MODEL_PART_TYPE& partType : aModel.partTypes )
        {
            for( const MODEL_GATE& gate : partType.gates )
                validateNestedId( gate, wxS( "gate" ), gateDeclarations );
        }

        for( const MODEL_SYMBOL_DEFINITION& definition : aModel.definitions )
        {
            for( const MODEL_PIN_DEFINITION& pin : definition.pins )
                validateNestedId( pin, wxS( "pin" ), pinDeclarations );

            for( const MODEL_FIELD& field : definition.fields )
                validateNestedId( field, wxS( "field" ), fieldDeclarations );
        }

        for( const MODEL_SHEET& sheet : aModel.sheets )
        {
            for( const MODEL_FIELD& field : sheet.titleBlockFields )
                validateNestedId( field, wxS( "field" ), fieldDeclarations );
        }

        for( const MODEL_PART_TYPE& partType : aModel.partTypes )
        {
            for( const MODEL_FIELD& field : partType.fields )
                validateNestedId( field, wxS( "field" ), fieldDeclarations );
        }

        for( const MODEL_PLACEMENT& placement : aModel.placements )
        {
            for( const MODEL_FIELD& field : placement.fields )
                validateNestedId( field, wxS( "field" ), fieldDeclarations );
        }
    }


    void validateSymbolGraph( const PADS_SCH_MODEL& aModel, const MODEL_REFERENCE_INDEX& aIndex )
    {
        for( const MODEL_SHEET& sheet : aModel.sheets )
        {
            if( sheet.parent && !aIndex.sheets.contains( sheet.parent->id.Value() ) )
                throwValidationError( sheet.parent->source, wxS( "unresolved sheet reference" ) );
        }

        for( const MODEL_PART_TYPE& partType : aModel.partTypes )
        {
            for( const MODEL_GATE& gate : partType.gates )
            {
                auto definition = aIndex.definitions.find( gate.definition.id.Value() );

                if( definition == aIndex.definitions.end() && gate.decalGroupMembers.empty() )
                    throwValidationError( gate.definition.source, wxS( "unresolved symbol definition reference" ) );

                for( const DEFINITION_REFERENCE& alternate : gate.alternateDefinitions )
                {
                    if( !aIndex.definitions.contains( alternate.id.Value() ) )
                        throwValidationError( alternate.source, wxS( "unresolved alternate definition reference" ) );
                }

                for( const DEFINITION_REFERENCE& member : gate.decalGroupMembers )
                {
                    if( !aIndex.definitions.contains( member.id.Value() ) )
                        throwValidationError( member.source, wxS( "unresolved pin-decal group member" ) );
                }

                if( definition == aIndex.definitions.end() )
                    continue;

                if( !gate.logicalPins.empty() && gate.logicalPins.size() != gate.pins.size() )
                    throwValidationError( gate.source, wxS( "gate logical-pin count does not match definition pins" ) );

                for( size_t pinOrdinal = 0; pinOrdinal < gate.pins.size(); ++pinOrdinal )
                {
                    const PIN_REFERENCE& pin = gate.pins[pinOrdinal];
                    auto                 pinOwner = aIndex.pinOwners.find( pin.id.Value() );

                    if( pinOwner == aIndex.pinOwners.end() || pinOwner->second != definition->second )
                        throwValidationError( pin.source, wxS( "pin does not belong to gate definition" ) );

                    if( !gate.logicalPins.empty() && gate.logicalPins[pinOrdinal].definitionPin.id != pin.id )
                        throwValidationError( gate.logicalPins[pinOrdinal].definitionPin.source,
                                              wxS( "logical pin does not belong to gate definition pin" ) );
                }
            }
        }

        struct DEFINITION_EDGE
        {
            uint32_t          target;
            SOURCE_PROVENANCE source;
        };

        std::unordered_map<uint32_t, std::vector<DEFINITION_EDGE>> definitionEdges;

        for( const MODEL_PART_TYPE& partType : aModel.partTypes )
        {
            for( const MODEL_GATE& gate : partType.gates )
            {
                if( !gate.definition.id.IsValid() )
                    continue;

                for( const DEFINITION_REFERENCE& alternate : gate.alternateDefinitions )
                {
                    // PADS may repeat the primary CAEDECAL among a gate's alternatives
                    // (e.g. CAPNOTPOL: CAP, CAP).  This is a redundant selection,
                    // not a recursive symbol definition.  Preserve the alternative
                    // in the model but do not introduce a self-edge in the graph.
                    if( alternate.id == gate.definition.id )
                        continue;

                    definitionEdges[gate.definition.id.Value()].push_back( { alternate.id.Value(), alternate.source } );
                }
            }
        }

        std::unordered_map<uint32_t, uint8_t> definitionColors;
        std::function<void( uint32_t )>       visitDefinition =
                [&]( uint32_t aDefinition )
                {
                    definitionColors[aDefinition] = 1;

                    auto edges = definitionEdges.find( aDefinition );

                    if( edges == definitionEdges.end() )
                    {
                        definitionColors[aDefinition] = 2;
                        return;
                    }

                    for( const DEFINITION_EDGE& edge : edges->second )
                    {
                        if( definitionColors[edge.target] == 1 )
                            throwValidationError( edge.source, wxS( "cyclic symbol definition reference" ) );

                        if( definitionColors[edge.target] == 0 )
                            visitDefinition( edge.target );
                    }

                    definitionColors[aDefinition] = 2;
                };

        for( const auto& [definition, edges] : definitionEdges )
        {
            if( definitionColors[definition] == 0 )
                visitDefinition( definition );
        }
    }


    void validatePlacements( const PADS_SCH_MODEL& aModel, const MODEL_REFERENCE_INDEX& aIndex )
    {
        for( const MODEL_PLACEMENT& placement : aModel.placements )
        {
            if( !aIndex.sheets.contains( placement.sheet.id.Value() ) )
                throwValidationError( placement.sheet.source, wxS( "unresolved placement sheet reference" ) );

            auto partType = aIndex.partTypes.find( placement.partType.id.Value() );

            if( partType == aIndex.partTypes.end() )
                throwValidationError( placement.partType.source, wxS( "unresolved placement part-type reference" ) );

            auto placementDefinition = aIndex.definitions.find( placement.definition.id.Value() );

            if( placementDefinition == aIndex.definitions.end() )
                throwValidationError( placement.definition.source, wxS( "unresolved placement definition" ) );

            for( const PIN_REFERENCE& pin : placement.pins )
            {
                auto owner = aIndex.pinOwners.find( pin.id.Value() );

                if( owner == aIndex.pinOwners.end() || owner->second != placementDefinition->second )
                    throwValidationError( pin.source, wxS( "placement pin does not belong to selected definition" ) );
            }

            if( !placement.gate )
                continue;

            auto gate = aIndex.gates.find( placement.gate->id.Value() );
            auto gateOwner = aIndex.gateOwners.find( placement.gate->id.Value() );

            if( gate == aIndex.gates.end() || gateOwner == aIndex.gateOwners.end()
                || gateOwner->second != partType->second
                || ( gate->second->unit != placement.unit && gate->second->decalGroupMembers.empty() ) )
                throwValidationError( placement.gate->source, wxS( "placement gate or unit mismatch" ) );

            const MODEL_GATE& selectedGate = *gate->second;
            const bool        definitionMatches = selectedGate.definition.id == placement.definition.id
                                           || std::ranges::any_of( selectedGate.alternateDefinitions,
                                                                   [&]( const DEFINITION_REFERENCE& aDefinition )
                                                                   {
                                                                       return aDefinition.id == placement.definition.id;
                                                                   } )
                                           || std::ranges::any_of( selectedGate.decalGroupMembers,
                                                                   [&]( const DEFINITION_REFERENCE& aDefinition )
                                                                   {
                                                                       return aDefinition.id == placement.definition.id;
                                                                   } );

            if( !definitionMatches )
                throwValidationError( placement.definition.source,
                                      wxS( "placement definition does not belong to selected gate" ) );

            auto definition = aIndex.definitions.find( placement.definition.id.Value() );

            if( definition == aIndex.definitions.end() )
                throwValidationError( placement.definition.source, wxS( "unresolved placement definition" ) );

            for( const PIN_REFERENCE& pin : placement.pins )
            {
                auto owner = aIndex.pinOwners.find( pin.id.Value() );

                if( owner == aIndex.pinOwners.end() || owner->second != definition->second )
                    throwValidationError( pin.source, wxS( "placement pin does not belong to selected definition" ) );
            }
        }
    }


    void validateSheetBoundContent( const PADS_SCH_MODEL& aModel, const MODEL_REFERENCE_INDEX& aIndex )
    {
        for( const MODEL_NET& net : aModel.nets )
        {
            validateSheetRef( net, aIndex, wxS( "net" ) );

            for( const MODEL_CONNECTION& connection : net.connections )
            {
                if( connection.endpoints.empty() )
                    throwValidationError( connection.source, wxS( "connection has no endpoints" ) );

                for( const MODEL_CONNECTION_ENDPOINT& endpoint : connection.endpoints )
                {
                    if( !endpointIsValid( aIndex, endpoint ) )
                    {
                        throwValidationError( endpoint.source,
                                              wxS( "empty, mixed, or unresolved connection endpoint" ) );
                    }

                    if( !endpoint.placement )
                        continue;

                    auto placement = aIndex.placements.find( endpoint.placement->id.Value() );

                    if( placement != aIndex.placements.end() && placement->second->sheet.id != net.sheet.id )
                    {
                        throwValidationError( endpoint.source,
                                              wxS( "connection endpoint placement sheet does not match net sheet" ) );
                    }
                }
            }
        }

        for( const MODEL_BUS& bus : aModel.buses )
        {
            validateSheetRef( bus, aIndex, wxS( "bus" ) );

            for( const NET_REFERENCE& member : bus.memberNets )
            {
                auto net = aIndex.nets.find( member.id.Value() );

                if( net == aIndex.nets.end() )
                    throwValidationError( member.source, wxS( "unresolved bus member-net reference" ) );

                if( net->second->sheet.id != bus.sheet.id )
                    throwValidationError( member.source, wxS( "bus member-net sheet does not match bus sheet" ) );
            }

            for( const MODEL_BUS_ENTRY& entry : bus.entries )
            {
                auto net = aIndex.nets.find( entry.memberNet.id.Value() );

                if( net == aIndex.nets.end() )
                    throwValidationError( entry.memberNet.source, wxS( "unresolved bus-entry net reference" ) );

                if( net->second->sheet.id != bus.sheet.id )
                    throwValidationError( entry.memberNet.source,
                                          wxS( "bus-entry net sheet does not match bus sheet" ) );

                if( std::ranges::none_of( bus.memberNets,
                                          [&]( const NET_REFERENCE& aMember )
                                          {
                                              return aMember.id == entry.memberNet.id;
                                          } ) )
                {
                    throwValidationError( entry.source, wxS( "bus-entry net is absent from bus member nets" ) );
                }
            }
        }

        for( const MODEL_LABEL& label : aModel.labels )
        {
            validateSheetRef( label, aIndex, wxS( "label" ) );

            for( const SHEET_REFERENCE& linkedSheet : label.linkedSheets )
            {
                if( !aIndex.sheets.contains( linkedSheet.id.Value() ) )
                    throwValidationError( linkedSheet.source, wxS( "unresolved cross-sheet label reference" ) );
            }
        }

        for( const MODEL_JUNCTION& junction : aModel.junctions )
            validateSheetRef( junction, aIndex, wxS( "junction" ) );

        for( const MODEL_TEXT& text : aModel.texts )
            validateSheetRef( text, aIndex, wxS( "text" ) );

        for( const MODEL_PAGE_GRAPHIC& graphic : aModel.graphics )
            validateSheetRef( graphic, aIndex, wxS( "page-graphic" ) );

        std::set<uint32_t> worksheetSheets;

        for( const MODEL_WORKSHEET& worksheet : aModel.worksheets )
        {
            validateSheetRef( worksheet, aIndex, wxS( "worksheet" ) );

            if( !worksheetSheets.insert( worksheet.sheet.id.Value() ).second )
                throwValidationError( worksheet.source, wxS( "duplicate worksheet for sheet" ) );

            if( worksheet.graphics.empty() )
                throwValidationError( worksheet.source, wxS( "worksheet has no graphics" ) );
        }

        for( const MODEL_EMBEDDED_IMAGE& image : aModel.images )
        {
            validateSheetRef( image, aIndex, wxS( "embedded-image" ) );

            if( image.type != MODEL_EMBEDDED_IMAGE_TYPE::UNSUPPORTED && image.data.empty() )
                throwValidationError( image.source, wxS( "embedded image has no decoded payload" ) );
        }
    }


    void validateSheetHierarchy( const PADS_SCH_MODEL& aModel, const MODEL_REFERENCE_INDEX& aIndex )
    {
        enum class VISIT_STATE : uint8_t
        {
            VISITING,
            COMPLETE
        };
        std::unordered_map<uint32_t, VISIT_STATE> visitStates;

        for( const MODEL_SHEET& sheet : aModel.sheets )
        {
            const MODEL_SHEET*              current = &sheet;
            std::vector<const MODEL_SHEET*> path;

            while( current && !visitStates.contains( current->id.Value() ) )
            {
                visitStates.emplace( current->id.Value(), VISIT_STATE::VISITING );
                path.push_back( current );

                if( !current->parent )
                {
                    current = nullptr;
                    break;
                }

                auto parent = aIndex.sheets.find( current->parent->id.Value() );

                if( parent == aIndex.sheets.end() )
                    break;

                current = parent->second;
            }

            if( current && visitStates.at( current->id.Value() ) == VISIT_STATE::VISITING )
                throwValidationError( current->source, wxS( "cyclic sheet hierarchy" ) );

            for( const MODEL_SHEET* visited : path )
                visitStates[visited->id.Value()] = VISIT_STATE::COMPLETE;
        }
    }


    std::vector<MODEL_FIELD> decodeDesignSettings( const std::vector<uint8_t>&   aBytes,
                                                   const PADS_IO::BINARY_CURSOR& aCursor, const PADS_SCH_SDB& aSdb,
                                                   const wxString& aSourceName, PADS_SCH_MODEL& aModel )
    {
        const SCH_SDB_POOL& sheetPool = aSdb.Pools()[3];

        if( sheetPool.usedBytes != sheetPool.count * SHEET_RECORD_BYTES )
        {
            SOURCE_PROVENANCE source =
                    sourceAt( aSourceName, aModel.version, wxS( "sheet index" ), 3, 0,
                              OUTER_DIRECTORY_OFFSET + 3 * OUTER_DESCRIPTOR_BYTES + OUTER_USED_BYTES_OFFSET, 4, -1 );
            throwDecodeError( source, wxS( "sheet-index byte count does not match 48-byte record count" ) );
        }

        const SCH_SDB_POOL& settingsPool = aSdb.Pools()[5];

        if( settingsPool.count != 100 || settingsPool.usedBytes != 400 )
        {
            SOURCE_PROVENANCE source =
                    sourceAt( aSourceName, aModel.version, wxS( "design settings" ), 5, 0,
                              OUTER_DIRECTORY_OFFSET + 5 * OUTER_DESCRIPTOR_BYTES + OUTER_USED_BYTES_OFFSET, 4, -1 );
            throwDecodeError( source, wxS( "design-settings controller is not the required 400-byte record" ) );
        }

        const size_t      settingsOffset = outerControllerOffset( aSdb, 5 );
        SOURCE_PROVENANCE settingsSource = sourceAt( aSourceName, aModel.version, wxS( "design settings" ), 5, 0,
                                                     settingsOffset, settingsPool.usedBytes, -1 );
        aModel.settings.source = settingsSource;

        constexpr size_t PAGE_SIZE_ENUM_OFFSET = 80;

        SOURCE_PROVENANCE pageSource = settingsSource;
        pageSource.absoluteOffset += PAGE_SIZE_ENUM_OFFSET;
        pageSource.length = 4;

        // +264 is the PADS BORDER NAME / drafting-template name, not the
        // sheet-size field. Standard borders such as "SIZEA3" made the old
        // interpretation appear correct until a custom name (for example
        // "SIZEA3_TEST") separated the two concepts.
        const uint32_t pageSizeRaw = aCursor.U32At( settingsOffset + PAGE_SIZE_ENUM_OFFSET );
        aModel.settings.pageSize = pageExtent( pageSizeRaw, pageSource );
        aModel.settings.defaultLineWidth = static_cast<int64_t>( aCursor.U32At( settingsOffset + 12 ) ) * 2;
        aModel.settings.defaultBusWidth = static_cast<int64_t>( aCursor.U32At( settingsOffset + 16 ) ) * 2;

        SOURCE_PROVENANCE fontModeSource = settingsSource;
        fontModeSource.absoluteOffset += 252;
        fontModeSource.length = 4;
        const uint32_t fontModeRaw = aCursor.U32At( settingsOffset + 252 );
        aModel.settings.properties.push_back(
                sourceProperty( wxS( "font_mode_raw" ), wxString::Format( wxS( "%u" ), fontModeRaw ),
                                fontModeSource ) );

        switch( fontModeRaw )
        {
        case 0: aModel.settings.fontMode = MODEL_FONT_MODE::STROKE; break;
        case 1: aModel.settings.fontMode = MODEL_FONT_MODE::SYSTEM; break;
        default:
        {
            aModel.settings.fontMode = MODEL_FONT_MODE::UNKNOWN;
            SOURCE_PROPERTY& property = aModel.settings.properties.back();
            property.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
            aModel.diagnostics.push_back( MakePropertyDiagnostic(
                    RPT_SEVERITY_WARNING, property,
                    wxS( "unknown PADS schematic font mode preserved; using KiCad stroke font" ) ) );
            break;
        }
        }

        const SCH_SDB_POOL& fontPool = aSdb.Pools()[19];

        if( fontPool.usedBytes != fontPool.count * FONT_RECORD_BYTES )
        {
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "font directory" ), 19, 0,
                                                 outerControllerOffset( aSdb, 19 ), fontPool.usedBytes, -1 );
            throwDecodeError( source, wxS( "controller byte count does not match 36-byte records" ) );
        }

        if( fontPool.count > 0 )
        {
            const size_t defaultFontOffset = outerControllerOffset( aSdb, 19 );
            SOURCE_PROVENANCE defaultFontSource =
                    sourceAt( aSourceName, aModel.version, wxS( "default system font" ), 19, 0,
                              defaultFontOffset + 4, 32, -1 );
            aModel.settings.defaultSystemFont =
                    decodeFixedString( aBytes, defaultFontOffset + 4, 32, defaultFontSource, aModel.diagnostics );
        }
        else if( aModel.settings.fontMode == MODEL_FONT_MODE::SYSTEM )
        {
            SOURCE_PROPERTY property = sourceProperty( wxS( "default_system_font" ), wxS( "" ), fontModeSource );
            property.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
            aModel.diagnostics.push_back( MakePropertyDiagnostic(
                    RPT_SEVERITY_WARNING, property,
                    wxS( "PADS system-font mode has no default font record; using KiCad stroke font" ) ) );
            aModel.settings.properties.push_back( std::move( property ) );
        }

        std::vector<MODEL_FIELD> titleFields;
        const SCH_SDB_POOL&      titleFieldPool = aSdb.Pools()[1];
        size_t                   titleOffset = outerControllerOffset( aSdb, 1 );
        const size_t             titleEnd = titleOffset + titleFieldPool.usedBytes;
        size_t                   titleRecord = 0;

        while( titleOffset + 6 <= titleEnd
               && std::equal( aBytes.begin() + titleOffset, aBytes.begin() + titleOffset + 6, "Field\n" ) )
        {
            size_t terminator = titleOffset;

            while( terminator < titleEnd && aBytes[terminator] != 0 )
                ++terminator;

            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "title field" ), 1, titleRecord,
                                                 titleOffset, terminator - titleOffset + 1, -1 );

            size_t separator = titleOffset + 6;

            while( separator < terminator && aBytes[separator] != 1 )
                ++separator;

            if( terminator == titleEnd || terminator - titleOffset < 7 || separator == terminator
                || !std::equal( aBytes.begin() + titleOffset, aBytes.begin() + titleOffset + 6, "Field\n" ) )
            {
                throwDecodeError( source, wxS( "invalid title-field name record" ) );
            }

            std::vector<uint8_t> nameBytes( aBytes.begin() + titleOffset + 6, aBytes.begin() + separator );
            std::vector<uint8_t> valueBytes( aBytes.begin() + separator + 1, aBytes.begin() + terminator );
            SOURCE_PROVENANCE    nameSource = source;
            nameSource.absoluteOffset += 6;
            nameSource.length = nameBytes.size();
            SOURCE_PROVENANCE valueSource = source;
            valueSource.absoluteOffset = separator + 1;
            valueSource.length = valueBytes.size();
            MODEL_FIELD field;
            field.source = source;
            field.name = PADS_SCH_BINARY_PARSER::DecodeString( nameBytes, DEFAULT_CODE_PAGE, nameSource,
                                                               aModel.diagnostics );
            field.value = PADS_SCH_BINARY_PARSER::DecodeString( valueBytes, DEFAULT_CODE_PAGE, valueSource,
                                                                aModel.diagnostics );
            field.presentation.source = source;
            titleFields.push_back( std::move( field ) );
            titleOffset = terminator + 1;
            ++titleRecord;
        }

        // Some valid PADS schematics have no global title-block fields at all.
        // In that case outer controller 1 is explicitly empty (count == 0 and
        // usedBytes == 0).  Treat only a non-empty controller that yielded no
        // decodable Field records as malformed.
        if( titleFields.empty()
            && ( titleFieldPool.count != 0 || titleFieldPool.usedBytes != 0 ) )
        {
            SOURCE_PROVENANCE source =
                    sourceAt( aSourceName, aModel.version, wxS( "title field" ), 1, titleFields.size(),
                              outerControllerOffset( aSdb, 1 ), titleFieldPool.usedBytes, -1 );
            throwDecodeError( source, wxS( "title-field controller contains no decodable fields" ) );
        }

        return titleFields;
    }


    void decodeSheets( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                       const PADS_SCH_SDB& aSdb, const wxString& aSourceName,
                       const std::vector<MODEL_FIELD>& aTitleFields, PADS_SCH_MODEL& aModel )
    {
        const size_t sheetIndexOffset = outerControllerOffset( aSdb, 3 );
        auto         sheetBlocks = aSdb.Blocks()
                           | std::views::filter(
                                   []( const SCH_SDB_BLOCK& aBlock )
                                   {
                                       return aBlock.kind == SCH_SDB_BLOCK_KIND::SHEET;
                                   } );
        auto sheetBlock = sheetBlocks.begin();

        for( size_t index = 0; index < aSdb.Pools()[3].count; ++index, ++sheetBlock )
        {
            size_t            recordOffset = sheetIndexOffset + index * SHEET_RECORD_BYTES;
            SOURCE_PROVENANCE provenance = sourceAt( aSourceName, aModel.version, wxS( "sheet" ), 3, index,
                                                     recordOffset, SHEET_RECORD_BYTES, static_cast<int>( index ) );

            if( sheetBlock == sheetBlocks.end() || aCursor.U32At( recordOffset ) != sheetBlock->offset
                || aCursor.U32At( recordOffset + 4 ) != sheetBlock->bytes )
            {
                throwDecodeError( provenance, wxS( "sheet-index record references the wrong SDB object class" ) );
            }

            constexpr size_t nameOffset = 14;

            // In PADS Logic v0x000D these are hierarchy fields, not fixed
            // object-class sentinels.  The parent is a controller-3 record
            // index (0xFFFF means top level).  The qualifier identifies a
            // repeated child instance and can also be 0xFFFF (unspecified).
            const uint16_t parentRecord = aCursor.U16At( recordOffset + 10 );
            const uint16_t instanceQualifier = aCursor.U16At( recordOffset + 12 );

            if( parentRecord != 0xFFFF && parentRecord >= aSdb.Pools()[3].count )
                throwDecodeError( provenance, wxS( "sheet parent record index leaves controller 3" ) );

            if( parentRecord == index )
                throwDecodeError( provenance, wxS( "sheet cannot be its own parent" ) );

            if( parentRecord == 0xFFFF && instanceQualifier != 0xFFFF )
                throwDecodeError( provenance, wxS( "top-level sheet has an unexpected instance qualifier" ) );

            size_t nameEnd = recordOffset + nameOffset;

            while( nameEnd < recordOffset + SHEET_RECORD_BYTES && aBytes[nameEnd] != 0 )
                ++nameEnd;

            if( nameEnd == recordOffset + SHEET_RECORD_BYTES )
                throwDecodeError( provenance, wxS( "unterminated sheet name" ) );

            std::vector<uint8_t> nameBytes( aBytes.begin() + recordOffset + nameOffset, aBytes.begin() + nameEnd );
            SOURCE_PROVENANCE    nameSource = provenance;
            nameSource.absoluteOffset += nameOffset;
            nameSource.length = nameBytes.size();

            MODEL_SHEET sheet;
            sheet.id = SHEET_ID( aCursor.U16At( recordOffset + 8 ) );
            sheet.index = sheet.id.IsValid() ? sheet.id.Value() - 1 : std::numeric_limits<size_t>::max();
            sheet.source = provenance;
            sheet.name = PADS_SCH_BINARY_PARSER::DecodeString( nameBytes, DEFAULT_CODE_PAGE, nameSource,
                                                               aModel.diagnostics );

            if( sheet.name.text.empty() )
                sheet.name.text = wxS( "$$$NONE" );

            if( parentRecord != 0xFFFF )
            {
                const size_t parentOffset = sheetIndexOffset + parentRecord * SHEET_RECORD_BYTES;
                const uint16_t parentId = aCursor.U16At( parentOffset + 8 );
                SOURCE_PROVENANCE parentSource = provenance;
                parentSource.absoluteOffset += 10;
                parentSource.length = 2;

                if( parentId == 0 || parentId == 0xFFFF || parentId == sheet.id.Value() )
                    throwDecodeError( parentSource, wxS( "sheet parent has invalid sheet identity" ) );

                sheet.parent = SHEET_REFERENCE{ SHEET_ID( parentId ), parentSource };
            }

            if( instanceQualifier != 0xFFFF )
            {
                SOURCE_PROVENANCE qualifierSource = provenance;
                qualifierSource.absoluteOffset += 12;
                qualifierSource.length = 2;
                SOURCE_PROPERTY qualifier = sourceProperty(
                        wxS( "raw_sheet_instance_qualifier" ),
                        wxString::Format( wxS( "%u" ), instanceQualifier ), qualifierSource );
                qualifier.disposition = PROPERTY_DISPOSITION::PRESERVED;
                sheet.properties.push_back( std::move( qualifier ) );
            }

            sheet.pageSize = aModel.settings.pageSize;
            sheet.defaultLineWidth = aModel.settings.defaultLineWidth;
            sheet.defaultBusWidth = aModel.settings.defaultBusWidth;
            sheet.titleBlockFields = aTitleFields;

            auto title = std::ranges::find_if( sheet.titleBlockFields,
                                               []( const MODEL_FIELD& aField )
                                               {
                                                   return aField.name.text == wxS( "Title" );
                                               } );

            if( title != sheet.titleBlockFields.end() )
                sheet.title = title->value;

            aModel.sheets.push_back( std::move( sheet ) );
        }
    }


    void decodeFreeText( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                         const wxString& aSourceName, size_t aSheetIndex, size_t aTextBase, uint32_t aTextCount,
                         size_t aHeapBase, uint32_t aHeapBytes, const PLACEMENT_GLOBALS& aGlobals,
                         PADS_SCH_MODEL& aModel )
    {
        for( size_t record = 0; record < aTextCount; ++record )
        {
            size_t recordOffset = aTextBase + record * TEXT_RECORD_BYTES;

            if( aCursor.U16At( recordOffset + 24 ) != aCursor.U16At( recordOffset + 26 ) )
                continue;

            SOURCE_PROVENANCE textSource = sourceAt( aSourceName, aModel.version, wxS( "free text" ), 1, record,
                                                     recordOffset, TEXT_RECORD_BYTES, static_cast<int>( aSheetIndex ) );
            uint32_t          stringOffset = aCursor.U32At( recordOffset + 8 );
            uint16_t          stringBytes = aCursor.U16At( recordOffset + 20 );

            if( stringOffset > aHeapBytes )
            {
                SOURCE_PROVENANCE offsetSource = textSource;
                offsetSource.absoluteOffset += 8;
                offsetSource.length = 4;
                throwDecodeError( offsetSource, wxS( "free-text string offset leaves controller 2" ) );
            }

            if( stringBytes == 0 || stringBytes > aHeapBytes - stringOffset )
            {
                SOURCE_PROVENANCE lengthSource = textSource;
                lengthSource.absoluteOffset += 20;
                lengthSource.length = 2;
                throwDecodeError( lengthSource, wxS( "free-text string length leaves controller 2" ) );
            }

            if( aBytes[aHeapBase + stringOffset + stringBytes - 1] != 0 )
            {
                SOURCE_PROVENANCE terminatorSource =
                        sourceAt( aSourceName, aModel.version, wxS( "free text string terminator" ), 2, record,
                                  aHeapBase + stringOffset + stringBytes - 1, 1, static_cast<int>( aSheetIndex ) );
                throwDecodeError( terminatorSource, wxS( "free-text string is not NUL terminated" ) );
            }

            SOURCE_PROVENANCE stringSource =
                    sourceAt( aSourceName, aModel.version, wxS( "free text string" ), 2, record,
                              aHeapBase + stringOffset, stringBytes - 1, static_cast<int>( aSheetIndex ) );
            std::vector<uint8_t> string( aBytes.begin() + stringSource.absoluteOffset,
                                         aBytes.begin() + stringSource.absoluteOffset + stringSource.length );
            uint16_t             justification = aCursor.U16At( recordOffset + 18 );

            MODEL_TEXT text;
            text.source = textSource;
            text.sheet = { aModel.sheets[aSheetIndex].id, textSource };
            text.text =
                    PADS_SCH_BINARY_PARSER::DecodeString( string, DEFAULT_CODE_PAGE, stringSource, aModel.diagnostics );
            text.position = { decodeCoordinate( aCursor.U16At( recordOffset + 12 ) ),
                              decodeCoordinate( aCursor.U16At( recordOffset + 14 ) ), textSource };
            text.angle = NormalizeAngle( aCursor.U16At( recordOffset + 16 ) );
            text.presentation.source = textSource;
            text.presentation.height = aCursor.U16At( recordOffset + 22 );
            text.presentation.width = aCursor.U8At( recordOffset + 30 );
            const uint8_t displayFlags = aCursor.U8At( recordOffset + 31 );
            text.presentation.visible = ( displayFlags & 1 ) == 0;
            // PADS stores the low nine-way justification selector bits in the
            // text-local frame.  For 90/270 degree text bits 0 and 1 are swapped.
            // This is the same rule already used for rotated placement fields.
            //
            // Title-block fixture:
            //   Sheet Number: 90 deg, raw 0  -> 0 -> bottom-left
            //   Drawn By:     90 deg, raw 10 -> 9 -> middle-right
            const uint16_t rotatedJustification =
                    rotatedFieldJustification( justification, text.angle );

            text.presentation.horizontalJustification =
                    freeTextHorizontalJustification( rotatedJustification );
            text.presentation.verticalJustification =
                    verticalJustification( rotatedJustification );

            SOURCE_PROVENANCE fontHandleSource = textSource;
            fontHandleSource.length = 2;
            decodeGlobalFont( aBytes, aCursor, aGlobals, aSourceName, aModel.version,
                              static_cast<int16_t>( aCursor.U16At( recordOffset ) ), fontHandleSource,
                              text.presentation, false, aModel.diagnostics );

            SOURCE_PROVENANCE displaySource = textSource;
            displaySource.absoluteOffset += 31;
            displaySource.length = 1;
            text.presentation.properties.push_back( sourceProperty(
                    wxS( "display_flags" ), wxString::Format( wxS( "%u" ), displayFlags ), displaySource ) );

            SOURCE_PROVENANCE relationshipSource = textSource;
            relationshipSource.objectClass = wxS( "free text relationship" );
            relationshipSource.absoluteOffset += 28;
            relationshipSource.length = 2;
            uint16_t        relationship = aCursor.U16At( relationshipSource.absoluteOffset );
            SOURCE_PROPERTY relationshipProperty;
            relationshipProperty.name.text = wxS( "controller_1_relationship_word_28" );
            relationshipProperty.name.source = relationshipSource;
            relationshipProperty.value.raw = { static_cast<uint8_t>( relationship ),
                                               static_cast<uint8_t>( relationship >> 8 ) };
            relationshipProperty.value.text = wxString::Format( wxS( "%u" ), relationship );
            relationshipProperty.value.encoding = STRING_ENCODING_STATUS::CODE_PAGE;
            relationshipProperty.value.source = relationshipSource;
            relationshipProperty.value.codePage = DEFAULT_CODE_PAGE;
            relationshipProperty.value.codePageName = wxS( "windows-1252" );
            relationshipProperty.disposition = PROPERTY_DISPOSITION::PRESERVED;
            relationshipProperty.source = relationshipSource;
            text.properties.push_back( std::move( relationshipProperty ) );
            aModel.texts.push_back( std::move( text ) );
        }
    }


    void decodeSheetBlocks( const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
                            const PADS_SCH_SDB& aSdb, const wxString& aSourceName, PADS_SCH_MODEL& aModel )
    {
        size_t                              sheetIndex = 0;
        std::optional<PLACEMENT_GLOBALS>    placementData;
        std::optional<CONNECTIVITY_GLOBALS> connectivityData;
        const PLACEMENT_LAYOUT&             placementSchema = placementLayout( aModel.version );

        if( placementSchema.decoded )
        {
            placementData = placementGlobals( aSdb, aSourceName, aModel.settings );
            connectivityData = connectivityGlobals( aBytes, aCursor, aSdb, aSourceName, aModel );
        }

        for( const SCH_SDB_BLOCK& block : aSdb.Blocks() )
        {
            if( block.kind != SCH_SDB_BLOCK_KIND::SHEET )
                continue;

            size_t   descriptors = block.offset + SHEET_HEADER_BYTES;
            size_t   payload = descriptors + SHEET_DESCRIPTOR_COUNT * SHEET_DESCRIPTOR_BYTES;
            uint32_t textCount = aCursor.U32At( descriptors + SHEET_COUNT_OFFSET );
            uint32_t textBytes = aCursor.U32At( descriptors + SHEET_USED_BYTES_OFFSET );
            uint32_t heapBytes = aCursor.U32At( descriptors + SHEET_DESCRIPTOR_BYTES + SHEET_USED_BYTES_OFFSET );

            SOURCE_PROVENANCE controllerSource = sourceAt( aSourceName, aModel.version, wxS( "text controller" ), 1, 0,
                                                           payload, textBytes, static_cast<int>( sheetIndex ) );

            if( textBytes != textCount * TEXT_RECORD_BYTES )
                throwDecodeError( controllerSource,
                                  wxS( "text-controller byte count does not match 32-byte records" ) );

            size_t heapOffset = payload + textBytes;

            if( !placementSchema.decoded )
            {
                SOURCE_PROVENANCE heapSource = sourceAt( aSourceName, aModel.version, wxS( "text string controller" ),
                                                         2, 0, heapOffset, heapBytes, static_cast<int>( sheetIndex ) );
                aModel.preservedControllerPayloads.push_back(
                        { controllerSource,
                          PROPERTY_DISPOSITION::PRESERVED,
                          { aBytes.begin() + payload, aBytes.begin() + payload + textBytes } } );
                aModel.preservedControllerPayloads.push_back(
                        { heapSource,
                          PROPERTY_DISPOSITION::PRESERVED,
                          { aBytes.begin() + heapOffset, aBytes.begin() + heapOffset + heapBytes } } );
                decodeDefinitionsAndParts( aBytes, aCursor, block, sheetIndex, aSourceName, nullptr, aModel );
                ++sheetIndex;
                continue;
            }

            decodeFreeText( aBytes, aCursor, aSourceName, sheetIndex, payload, textCount, heapOffset, heapBytes,
                            *placementData, aModel );
            decodeDefinitionsAndParts( aBytes, aCursor, block, sheetIndex, aSourceName,
                                       &*connectivityData, aModel );
            decodePlacements( aBytes, aCursor, block, sheetIndex, aSourceName, *placementData, aModel );
            decodeConnectivity( aBytes, aCursor, block, sheetIndex, aSourceName, *connectivityData, aModel );
            ++sheetIndex;
        }
    }


    void decodeOleImages( const std::vector<uint8_t>& aBytes, const PADS_SCH_SDB& aSdb, const wxString& aSourceName,
                          PADS_SCH_MODEL& aModel )
    {
        for( size_t index = 0; index < aSdb.OleItems().size(); ++index )
        {
            const SCH_SDB_OLE_ITEM& item = aSdb.OleItems()[index];
            SOURCE_PROVENANCE       source =
                    sourceAt( aSourceName, aModel.version, wxS( "embedded OLE image" ), item.cfb.controller, index,
                              item.cfb.offset, item.cfb.bytes, static_cast<int>( item.sheetPlane ) );
            OLE_IMAGE_PAYLOAD    payload = ExtractOleImage( aBytes.data() + item.cfb.offset, item.cfb.bytes );
            MODEL_EMBEDDED_IMAGE image;
            image.id = IMAGE_ID( index );
            image.source = source;
            image.sheet = { aModel.sheets[item.sheetPlane].id, source };
            image.streamName = wxString::FromUTF8( payload.streamName );
            image.extent = item.extent;
            image.databaseBox = { item.left, item.bottom, item.right, item.top };
            SOURCE_PROVENANCE boxSource = source;
            boxSource.objectClass = wxS( "embedded OLE image database box" );
            boxSource.absoluteOffset = item.boxOffset;
            boxSource.length = 16;
            int64_t left = decodeDatabaseCoordinate( item.left, boxSource );
            int64_t bottom = decodeDatabaseCoordinate( item.bottom, boxSource );
            int64_t right = decodeDatabaseCoordinate( item.right, boxSource );
            int64_t top = decodeDatabaseCoordinate( item.top, boxSource );

            image.position = { left + ( right - left ) / 2, top + ( bottom - top ) / 2, boxSource };
            image.size = { std::abs( right - left ), std::abs( bottom - top ), boxSource };
            image.mirrorHorizontal = right < left;
            image.mirrorVertical = bottom < top;
            image.flags = item.flags;
            image.data = std::move( payload.data );

            switch( payload.type )
            {
            case OLE_IMAGE_TYPE::BMP: image.type = MODEL_EMBEDDED_IMAGE_TYPE::BMP; break;
            case OLE_IMAGE_TYPE::DIB: image.type = MODEL_EMBEDDED_IMAGE_TYPE::DIB; break;
            case OLE_IMAGE_TYPE::WMF: image.type = MODEL_EMBEDDED_IMAGE_TYPE::WMF; break;
            case OLE_IMAGE_TYPE::NONE:
                image.type = MODEL_EMBEDDED_IMAGE_TYPE::UNSUPPORTED;
                aModel.diagnostics.emplace_back(
                        RPT_SEVERITY_WARNING, source,
                        wxS( "embedded OLE object has no supported BMP, DIB, or WMF stream" ) );
                break;
            }

            if( right == left || bottom == top )
            {
                image.type = MODEL_EMBEDDED_IMAGE_TYPE::UNSUPPORTED;
                aModel.diagnostics.emplace_back(
                        RPT_SEVERITY_WARNING, boxSource,
                        wxS( "embedded OLE image has a zero-size database box and was skipped" ) );
            }

            if( item.flags != 1 )
            {
                SOURCE_PROVENANCE flagsSource = source;
                flagsSource.objectClass = wxS( "embedded OLE image flags" );
                flagsSource.absoluteOffset = item.boxOffset + 20;
                flagsSource.length = 4;
                PADS_SCH_BINARY_PARSER::RecordUnknownEnum( wxS( "embedded OLE image flags" ), item.flags, flagsSource,
                                                           aModel.diagnostics );
            }

            aModel.images.push_back( std::move( image ) );
        }
    }


} // namespace


wxString FormatParserError( const SOURCE_PROVENANCE& aSource, const wxString& aMessage )
{
    return wxString::Format(
            wxS( "%s: PADS schematic v0x%04X %s (controller %d, record %llu, sheet %d) at offset 0x%llX: %s" ),
            aSource.file, aSource.version, aSource.objectClass, aSource.controller,
            static_cast<unsigned long long>( aSource.recordIndex ), aSource.sheet,
            static_cast<unsigned long long>( aSource.absoluteOffset ), aMessage );
}


bool DIAGNOSTIC_PROPERTY_KEY::operator<( const DIAGNOSTIC_PROPERTY_KEY& aOther ) const
{
    return std::tie( source.file, source.version, source.objectClass, source.controller, source.recordIndex,
                     source.absoluteOffset, source.length, source.sheet, property.name, property.disposition )
           < std::tie( aOther.source.file, aOther.source.version, aOther.source.objectClass, aOther.source.controller,
                       aOther.source.recordIndex, aOther.source.absoluteOffset, aOther.source.length,
                       aOther.source.sheet, aOther.property.name, aOther.property.disposition );
}


std::optional<DIAGNOSTIC_PROPERTY_KEY> DiagnosticPropertyKey( const PARSER_DIAGNOSTIC& aDiagnostic )
{
    if( !aDiagnostic.property )
        return std::nullopt;

    return DIAGNOSTIC_PROPERTY_KEY{ aDiagnostic.source, *aDiagnostic.property };
}


PARSER_DIAGNOSTIC MakePropertyDiagnostic( SEVERITY aSeverity, const SOURCE_PROPERTY& aProperty,
                                          const wxString& aMessage )
{
    return MakePropertyDiagnostic( aSeverity, aProperty.source, aProperty.name.text, aProperty.disposition, aMessage );
}


PARSER_DIAGNOSTIC MakePropertyDiagnostic( SEVERITY aSeverity, const SOURCE_PROVENANCE& aSource, const wxString& aName,
                                          PROPERTY_DISPOSITION aDisposition, const wxString& aMessage )
{
    return { aSeverity, aSource, aMessage, DIAGNOSTIC_PROPERTY_IDENTITY{ aName, aDisposition } };
}


void PADS_SCH_MODEL::ValidateOrThrow() const
{
    validateIdentity( *this );

    const MODEL_REFERENCE_INDEX index( *this );

    validateSymbolGraph( *this, index );
    validatePlacements( *this, index );
    validateSheetBoundContent( *this, index );
    validateSheetHierarchy( *this, index );
}


// The legacy v0x0009 sheet is structurally close to v0x000D, but several
// controller record layouts are different.  Validate them with bounded reads
// before any semantic decoding: do not cast v9 data to the v13 record structs.
// This is the versioned foundation for the subsequent v9 semantic decoder.
static void validateLegacyV9Records( const PADS_SCH_SDB& aSdb, const std::vector<uint8_t>& aBytes )
{
    const PADS_IO::BINARY_CURSOR& cursor = aSdb.Cursor();
    auto reject = [&]( size_t aOffset, const wxString& aDetail )
    {
        THROW_IO_ERROR( wxString::Format(
                wxS( "PADS Logic binary v0x0009 at offset 0x%zX: %s" ), aOffset, aDetail.c_str() ) );
    };

    constexpr size_t SHEET_HEADER = 20;
    constexpr size_t DESCRIPTOR_COUNT = 24;
    constexpr size_t DESCRIPTOR_BYTES = 28;

    // Controller numbers below are 1-based.  Only known *fixed* record types
    // are checked; variable-length heaps are left to their owning decoders.
    struct LEGACY_STRIDE { size_t controller; size_t bytes; };
    constexpr LEGACY_STRIDE strides[] = {
        { 1, 28 }, { 3, 80 }, { 4, 6 }, { 5, 6 }, { 6, 14 },
        { 7, 92 }, { 8, 26 }, { 9, 76 }, { 10, 12 }, { 11, 8 },
        { 12, 52 }, { 13, 8 }, { 15, 128 }, { 16, 12 }, { 17, 20 },
        { 19, 12 }, { 20, 32 }, { 21, 40 }, { 22, 8 }, { 23, 40 }
    };

    for( const SCH_SDB_BLOCK& block : aSdb.Blocks() )
    {
        if( block.kind != SCH_SDB_BLOCK_KIND::SHEET )
            continue;

        const size_t directory = block.offset + SHEET_HEADER;
        size_t data = directory + DESCRIPTOR_COUNT * DESCRIPTOR_BYTES;
        std::vector<size_t>   offsets( 23 );
        std::vector<uint32_t> counts( 23 );
        std::vector<uint32_t> used( 23 );

        for( size_t i = 0; i < offsets.size(); ++i )
        {
            const size_t descriptor = directory + i * DESCRIPTOR_BYTES;
            offsets[i] = data;
            counts[i] = cursor.U32At( descriptor + 12 );
            used[i] = cursor.U32At( descriptor + 16 );

            if( data > block.offset + block.bytes
                || used[i] > block.offset + block.bytes - data )
                reject( descriptor + 16, wxS( "v9 sheet controller leaves its SDB block" ) );

            data += used[i];
        }

        for( const LEGACY_STRIDE& stride : strides )
        {
            const size_t i = stride.controller - 1;

            if( used[i] != static_cast<uint64_t>( counts[i] ) * stride.bytes )
                reject( directory + i * DESCRIPTOR_BYTES + 16,
                        wxString::Format( wxS( "v9 controller %zu is not %zu-byte records" ),
                                          stride.controller, stride.bytes ) );
        }

        // v9 text: 28 bytes, with string-heap offset at +4 (v13 uses +8).
        for( size_t i = 0; i < counts[0]; ++i )
        {
            const size_t offset = offsets[0] + i * 28;
            const uint32_t stringOffset = cursor.U32At( offset + 4 );

            if( stringOffset >= used[1] )
                reject( offset + 4, wxS( "v9 text string offset leaves controller 2" ) );

            const auto begin = aBytes.begin() + offsets[1] + stringOffset;
            const auto end = aBytes.begin() + offsets[1] + used[1];

            if( std::find( begin, end, static_cast<uint8_t>( 0 ) ) == end )
                reject( offset + 4, wxS( "v9 text is not NUL terminated" ) );
        }

        // v9 used decal: 92 bytes.  The definition ordinal at +48 is u16;
        // reading it as the v13 u32 changes the handle (e.g. 0 -> 131072).
        for( size_t i = 0; i < counts[6]; ++i )
        {
            const size_t offset = offsets[6] + i * 92;
            const auto nameBegin = aBytes.begin() + offset;
            const auto nameEnd = nameBegin + 40;

            if( std::find( nameBegin, nameEnd, static_cast<uint8_t>( 0 ) ) == nameEnd )
                reject( offset, wxS( "v9 used-decal name is unterminated" ) );

            const uint16_t definition = cursor.U16At( offset + 48 );

            if( definition == 0xFFFF )
                continue;

            if( definition >= counts[2] )
                reject( offset + 48, wxS( "v9 decal definition ordinal leaves controller 3" ) );

            const auto targetName = aBytes.begin() + offsets[2] + definition * 80;

            if( !std::equal( nameBegin, nameBegin + 38, targetName ) )
                reject( offset + 48, wxS( "v9 decal references a different symbol definition" ) );
        }

        uint64_t pinTotal = 0;
        uint64_t fieldTotal = 0;

        // v9 placement: 128 bytes, shifted by eight from the v13 layout.
        for( size_t i = 0; i < counts[14]; ++i )
        {
            const size_t offset = offsets[14] + i * 128;

            if( cursor.U16At( offset + 0x3A ) >= counts[8] )
                reject( offset + 0x3A, wxS( "v9 placement PARTTYPE handle is out of range" ) );

            if( cursor.U16At( offset + 0x3C ) >= counts[6] )
                reject( offset + 0x3C, wxS( "v9 placement decal handle is out of range" ) );

            pinTotal += cursor.U16At( offset + 0x44 );
            fieldTotal += cursor.U16At( offset + 0x46 );
        }

        if( pinTotal != counts[15] || fieldTotal != counts[16] )
            reject( offsets[14], wxS( "v9 placement pin/field totals disagree with controllers" ) );

        // v9 OFFPAGE keeps the 32-byte v13 record format, including +26 angle
        // and +28 mirror.  Its pointers must be in the matching controllers.
        for( size_t i = 0; i < counts[19]; ++i )
        {
            const size_t offset = offsets[19] + i * 32;

            if( cursor.U16At( offset + 4 ) >= counts[6]
                || cursor.U16At( offset + 8 ) >= counts[20] )
                reject( offset + 4, wxS( "v9 off-page decal/connection handle invalid" ) );
        }

        // v9 NETNAMES: 40 bytes instead of 48.  OFFPAGE owner at +30,
        // while the v13 record stores it at +38.
        for( size_t i = 0; i < counts[22]; ++i )
        {
            const size_t offset = offsets[22] + i * 40;
            const uint16_t owner = cursor.U16At( offset + 30 );

            if( ( owner & 0xF000 ) == 0x2000
                && ( owner & 0x0FFF ) >= counts[19] )
                reject( offset + 30, wxS( "v9 NETNAMES off-page owner out of range" ) );
        }
    }
}

// Semantic v0x0009 stage 3. The 80-byte symbol records, 6-byte graphics,
// vertices and 14-byte arcs are shared with v0x000D. Reuse the existing
// *semantic* graphic decoder, but keep legacy text and link records separate.
// Do not send a model with missing parts and connections to the KiCad builder.
// PADS Logic v0x0009 Stage 4. Graphical terminal, gate, and PARTTYPE
// relationships are decoded independently from v0x000D. In particular the
// 8-byte logical pin record is NOT the 24-byte v13 record. These objects are
// intentionally not passed to the KiCad builder before placements and wires
// can be decoded as well.
struct LEGACY_V9_PART_COUNTS
{
    size_t terminals = 0;
    size_t partTypes = 0;
    size_t gates = 0;
    size_t gatePins = 0;
    size_t signalPins = 0;
};

static LEGACY_V9_PART_COUNTS decodeLegacyV9PartsAndPins(
        const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
        const SHEET_CONTROLLERS& aControllers, const DEFINITION_LAYOUT& aLayout,
        size_t aSheetIndex, const wxString& aSourceName,
        const std::vector<MODEL_SYMBOL_DEFINITION*>& aDefinitionsByRecord,
        PADS_SCH_MODEL& aModel )
{
    LEGACY_V9_PART_COUNTS counts;
    const auto& pools = aControllers.pools;
    std::vector<MODEL_SYMBOL_DEFINITION*> decals( pools[6].count, nullptr );
    std::vector<bool> claimedTerminals( pools[7].count, false );

    // Resolve 92-byte USED_DECALs to their 80-byte symbol definitions. In v9
    // definition index is u16 at +48; terminal count/start are +42/+44.
    for( size_t record = 0; record < decals.size(); ++record )
    {
        const size_t offset = aControllers.offsets[6] + record * 92;
        const uint16_t definitionRecord = aCursor.U16At( offset + 48 );
        const uint16_t terminalCount = aCursor.U16At( offset + 42 );
        const uint16_t terminalStart = aCursor.U16At( offset + 44 );

        if( definitionRecord == 0xFFFF )
            continue;

        SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "used decal" ),
                                             7, record, offset, 92, static_cast<int>( aSheetIndex ) );

        if( definitionRecord >= aDefinitionsByRecord.size() )
            throwDecodeError( source, wxS( "v9 used-decal definition index is out of range" ) );

        if( terminalStart > pools[7].count || terminalCount > pools[7].count - terminalStart )
            throwDecodeError( source, wxS( "v9 used-decal terminal slice leaves controller 8" ) );

        MODEL_SYMBOL_DEFINITION* definition = aDefinitionsByRecord[definitionRecord];

        if( !definition )
            continue;

        decals[record] = definition;
        // Symbols with multiple USED_DECAL aliases must not duplicate their
        // physical terminal objects. For this fixture there are no such cases.
        if( !definition->pins.empty() && terminalCount != 0 )
            throwDecodeError( source, wxS( "v9 symbol terminal slice is claimed twice" ) );

        for( size_t pin = 0; pin < terminalCount; ++pin )
        {
            const size_t terminalRecord = terminalStart + pin;
            const size_t terminalOffset = aControllers.offsets[7] + terminalRecord * TERMINAL_BYTES;
            SOURCE_PROVENANCE pinSource = sourceAt(
                    aSourceName, aModel.version, wxS( "symbol pin" ), 8, terminalRecord,
                    terminalOffset, TERMINAL_BYTES, static_cast<int>( aSheetIndex ) );

            if( claimedTerminals[terminalRecord] )
                throwDecodeError( pinSource, wxS( "v9 terminal belongs to multiple decals" ) );

            claimedTerminals[terminalRecord] = true;
            MODEL_PIN_DEFINITION definitionPin;
            definitionPin.id = PIN_ID( aLayout.pinIdBase + terminalRecord );
            definitionPin.source = pinSource;
            definitionPin.position = {
                    decodeTerminalCoordinate( aCursor.U16At( terminalOffset + 2 ) ),
                    decodeTerminalCoordinate( aCursor.U16At( terminalOffset + 4 ) ), pinSource };
            definitionPin.numberOffset = {
                    decodeLocalCoordinate( aCursor.U16At( terminalOffset + 6 ) ),
                    decodeLocalCoordinate( aCursor.U16At( terminalOffset + 8 ) ), pinSource };
            definitionPin.nameOffset = {
                    decodeLocalCoordinate( aCursor.U16At( terminalOffset + 10 ) ),
                    decodeLocalCoordinate( aCursor.U16At( terminalOffset + 12 ) ), pinSource };
            const uint16_t presentationFlags = aCursor.U16At( terminalOffset + 22 );
            const uint16_t visibilityFlags = aCursor.U16At( terminalOffset + 24 );
            definitionPin.presentationFlags = presentationFlags;
            definitionPin.visibilityAndNumberPresentationFlags = visibilityFlags;
            definitionPin.xym = ( presentationFlags & 0x0006 ) >> 1;
            definitionPin.angle = ( presentationFlags & 0x0001 ) ? 900 : 0;
            definitionPin.numberAngle = ( presentationFlags & 0x0100 ) ? 900 : 0;
            definitionPin.nameAngle = ( visibilityFlags & 0x0001 ) ? 900 : 0;
            definitionPin.presentation.source = pinSource;
            definitionPin.presentation.visible = true;
            definitionPin.namePresentation.source = pinSource;
            definitionPin.numberPresentation.source = pinSource;

            // The pin-decal vector/line-body and text metrics require the
            // v9-specific USED_DECAL field map; do not synthesize geometry.
            const uint16_t pinDecalHandle = aCursor.U16At( terminalOffset );

            if( pinDecalHandle != 0xFFFF )
            {
                if( pinDecalHandle >= decals.size() )
                    throwDecodeError( pinSource, wxS( "v9 terminal pin-decal handle is out of range" ) );

                SOURCE_PROVENANCE decalNameSource = sourceAt(
                        aSourceName, aModel.version, wxS( "pin decal" ), 7, pinDecalHandle,
                        aControllers.offsets[6] + pinDecalHandle * 92, 40,
                        static_cast<int>( aSheetIndex ) );
                definitionPin.decalName = decodeFixedString(
                        aBytes, decalNameSource.absoluteOffset, 40, decalNameSource,
                        aModel.diagnostics );
            }

            definition->pins.push_back( std::move( definitionPin ) );
            ++counts.terminals;
        }
    }

    // v9 PARTTYPE keeps the 76-byte record. +44/+48/+52 are the gate,
    // logical-pin and signal-pin slice starts. Counts +64/+66/+68 correspond
    // to gates/signals/local pin-number aliases (rather than the v13 fields).
    aModel.partTypes.reserve( aModel.partTypes.size() + pools[8].count );
    size_t specialNameCursor = 0;

    for( size_t record = 0; record < pools[8].count; ++record )
    {
        const size_t offset = aControllers.offsets[8] + record * PART_TYPE_BYTES;
        SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "part type" ),
                                             9, record, offset, PART_TYPE_BYTES,
                                             static_cast<int>( aSheetIndex ) );
        const uint32_t gateStart = aCursor.U32At( offset + 44 );
        const uint32_t pinStart = aCursor.U32At( offset + 48 );
        const uint32_t signalStart = aCursor.U32At( offset + 52 );
        const uint32_t gateCount = aCursor.U16At( offset + 64 );
        const uint32_t signalCount = aCursor.U16At( offset + 66 );
        const uint32_t specialNameCount = aCursor.U16At( offset + 68 );
        const uint32_t gateEnd = record + 1 < pools[8].count
                ? aCursor.U32At( offset + PART_TYPE_BYTES + 44 ) : pools[9].count;
        const uint32_t pinEnd = record + 1 < pools[8].count
                ? aCursor.U32At( offset + PART_TYPE_BYTES + 48 ) : pools[10].count;
        const uint32_t signalEnd = record + 1 < pools[8].count
                ? aCursor.U32At( offset + PART_TYPE_BYTES + 52 ) : pools[11].count;

        if( gateStart > gateEnd || gateEnd > pools[9].count
            || pinStart > pinEnd || pinEnd > pools[10].count
            || signalStart > signalEnd || signalEnd > pools[11].count
            || gateCount != gateEnd - gateStart || signalCount != signalEnd - signalStart
            || specialNameCursor > pools[12].count
            || specialNameCount > pools[12].count - specialNameCursor )
        {
            throwDecodeError( source, wxS( "v9 PARTTYPE gate/pin/signal/name ownership mismatch" ) );
        }

        MODEL_PART_TYPE part;
        part.id = PART_TYPE_ID( aLayout.partIdBase + record );
        part.source = source;
        SOURCE_PROVENANCE nameSource = source;
        nameSource.length = 44;
        part.name = decodeFixedString( aBytes, offset, 44, nameSource, aModel.diagnostics );
        size_t logicalPinCursor = pinStart;
        uint32_t unit = 0;

        for( uint32_t gateRecord = gateStart; gateRecord < gateEnd; ++gateRecord )
        {
            const size_t gateOffset = aControllers.offsets[9] + gateRecord * GATE_BYTES;
            SOURCE_PROVENANCE gateSource = sourceAt(
                    aSourceName, aModel.version, wxS( "gate" ), 10, gateRecord,
                    gateOffset, GATE_BYTES, static_cast<int>( aSheetIndex ) );
            MODEL_GATE gate;
            gate.id = GATE_ID( aLayout.gateIdBase + gateRecord );
            gate.source = gateSource;
            gate.unit = ++unit;
            const uint16_t pinCount = aCursor.U16At( gateOffset + 8 );
            const uint16_t primaryHandle = aCursor.U16At( gateOffset );

            if( primaryHandle >= decals.size() || !decals[primaryHandle] )
                throwDecodeError( gateSource, wxS( "v9 gate primary decal is unresolved" ) );

            MODEL_SYMBOL_DEFINITION& definition = *decals[primaryHandle];
            gate.definition = { definition.id, gateSource };

            if( pinCount != definition.pins.size() || logicalPinCursor > pinEnd
                || pinCount > pinEnd - logicalPinCursor )
                throwDecodeError( gateSource, wxS( "v9 gate pin count disagrees with selected decal" ) );

            for( size_t alternate = 1; alternate < 4; ++alternate )
            {
                const uint16_t alternateHandle = aCursor.U16At( gateOffset + alternate * 2 );

                if( alternateHandle == 0xFFFF )
                    continue;

                if( alternateHandle >= decals.size() || !decals[alternateHandle]
                    || decals[alternateHandle]->pins.size() != pinCount )
                    throwDecodeError( gateSource, wxS( "v9 alternate decal is invalid" ) );

                gate.alternateDefinitions.push_back( { decals[alternateHandle]->id, gateSource } );
            }

            for( size_t pinOrdinal = 0; pinOrdinal < pinCount; ++pinOrdinal, ++logicalPinCursor )
            {
                const size_t pinOffset = aControllers.offsets[10] + logicalPinCursor * 8;
                SOURCE_PROVENANCE pinSource = sourceAt(
                        aSourceName, aModel.version, wxS( "logical pin" ), 11,
                        logicalPinCursor, pinOffset, 8, static_cast<int>( aSheetIndex ) );
                const uint16_t rawNumber = aCursor.U16At( pinOffset + 4 );
                SOURCE_STRING number;

                if( specialNameCount != 0 )
                {
                    if( rawNumber == 0 || rawNumber > specialNameCount )
                        throwDecodeError( pinSource, wxS( "v9 special pin number index is out of range" ) );

                    const size_t aliasOffset = aControllers.offsets[12]
                            + ( specialNameCursor + rawNumber - 1 ) * 8;
                    SOURCE_PROVENANCE aliasSource = sourceAt(
                            aSourceName, aModel.version, wxS( "pin-number alias" ), 13,
                            specialNameCursor + rawNumber - 1, aliasOffset, 8,
                            static_cast<int>( aSheetIndex ) );
                    number = decodeFixedString( aBytes, aliasOffset, 8, aliasSource, aModel.diagnostics );
                }
                else
                {
                    number.source = pinSource;
                    number.text = wxString::Format( wxS( "%u" ), rawNumber );
                }

                MODEL_PIN_DEFINITION& definitionPin = definition.pins[pinOrdinal];

                if( definitionPin.number.text.IsEmpty() )
                    definitionPin.number = number;

                const uint8_t rawType = aCursor.U8At( pinOffset + 7 );
                const uint32_t electricalType = pinElectricalType( rawType, pinSource, aModel.diagnostics );
                definitionPin.electricalType = electricalType;
                MODEL_GATE_PIN logicalPin;
                logicalPin.source = pinSource;
                logicalPin.definitionPin = { definitionPin.id, pinSource };
                logicalPin.number = std::move( number );
                logicalPin.name = definitionPin.name;
                logicalPin.electricalType = electricalType;
                logicalPin.swapGroup = aCursor.U8At( pinOffset + 6 );
                gate.pins.push_back( { definitionPin.id, pinSource } );
                gate.logicalPins.push_back( std::move( logicalPin ) );
                ++counts.gatePins;
            }

            part.gates.push_back( std::move( gate ) );
            ++counts.gates;
        }

        if( logicalPinCursor != pinEnd )
            throwDecodeError( source, wxS( "v9 PARTTYPE has unclaimed logical pins" ) );

        // A separate v9 signal-pin mapping pass is needed to interpret the
        // 52-byte SIGPIN payload. Verify its ownership but do not fabricate
        // potentially wrong signal-to-net associations.
        counts.signalPins += signalCount;
        specialNameCursor += specialNameCount;
        aModel.partTypes.push_back( std::move( part ) );
        ++counts.partTypes;
    }

    if( specialNameCursor != pools[12].count )
        throwDecodeError( aModel.source, wxS( "v9 unclaimed special pin-number aliases" ) );

    for( bool claimed : claimedTerminals )
    {
        if( !claimed )
            throwDecodeError( aModel.source, wxS( "v9 unclaimed symbol terminal record" ) );
    }

    return counts;
}

// PADS Logic 0x0009 Stage 5. Placed schematic objects are not just the
// v13 layouts with their strides changed: all offsets here are explicit v9
// offsets. Group attributes and placement pin presentation remain bounded.
struct LEGACY_V9_PLACEMENT_COUNTS
{
    size_t placements = 0;
    size_t pins = 0;
    size_t attributeFields = 0;
    size_t unboundCustomFields = 0;
    size_t reusedGateUnits = 0;
};

static LEGACY_V9_PLACEMENT_COUNTS decodeLegacyV9Placements(
        const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
        const PADS_SCH_SDB& aSdb, const SHEET_CONTROLLERS& aControllers,
        size_t aSheetIndex, const wxString& aSourceName, PADS_SCH_MODEL& aModel )
{
    LEGACY_V9_PLACEMENT_COUNTS counts;
    const auto& pools = aControllers.pools;
    requireFixedController( aControllers, 15, 128, aSourceName, aModel.version, aSheetIndex );
    requireFixedController( aControllers, 16, 12, aSourceName, aModel.version, aSheetIndex );
    requireFixedController( aControllers, 17, 20, aSourceName, aModel.version, aSheetIndex );

    const SCH_SDB_POOL& groupPool = aSdb.Pools()[6];
    const SCH_SDB_POOL& attributePool = aSdb.Pools()[7];
    const SCH_SDB_POOL& attributeHeap = aSdb.Pools()[2];

    if( uint64_t( groupPool.count ) * PLACEMENT_GROUP_BYTES != groupPool.usedBytes
        || uint64_t( attributePool.count ) * ATTRIBUTE_OFFSET_BYTES != attributePool.usedBytes )
        throwDecodeError( aModel.source, wxS( "v9 component-group/attribute directory stride mismatch" ) );

    const size_t groupBase = outerControllerOffset( aSdb, 6 );
    const size_t attributeBase = outerControllerOffset( aSdb, 7 );
    const size_t attributeHeapBase = outerControllerOffset( aSdb, 2 );

    auto attribute = [&]( size_t aRecord, bool aRequireValue )
            -> std::pair<SOURCE_STRING, SOURCE_STRING>
    {
        if( aRecord >= attributePool.count )
            throwDecodeError( aModel.source, wxS( "v9 attribute record outside outer controller 7" ) );

        const size_t recordOffset = attributeBase + aRecord * ATTRIBUTE_OFFSET_BYTES;
        const uint32_t stringOffset = aCursor.U32At( recordOffset );
        SOURCE_PROVENANCE source = sourceAt(
                aSourceName, aModel.version, wxS( "component attribute" ), 7,
                aRecord, recordOffset, ATTRIBUTE_OFFSET_BYTES, -1 );

        if( stringOffset >= attributeHeap.usedBytes )
            throwDecodeError( source, wxS( "v9 attribute string offset outside outer controller 2" ) );

        const size_t start = attributeHeapBase + stringOffset;
        const size_t limit = attributeHeapBase + attributeHeap.usedBytes;
        size_t end = start;

        while( end < limit && aBytes[end] != 0 )
            ++end;

        if( end == limit )
            throwDecodeError( source, wxS( "v9 attribute string is not NUL terminated" ) );

        size_t separator = start;

        while( separator < end && aBytes[separator] != 1 )
            ++separator;

        if( aRequireValue && separator == end )
            throwDecodeError( source, wxS( "v9 component field lacks name/value separator" ) );

        SOURCE_PROVENANCE nameSource = source;
        nameSource.absoluteOffset = start;
        nameSource.length = separator - start;
        SOURCE_STRING name = PADS_SCH_BINARY_PARSER::DecodeString(
                { aBytes.begin() + start, aBytes.begin() + separator }, DEFAULT_CODE_PAGE,
                nameSource, aModel.diagnostics );

        SOURCE_STRING value;

        if( separator < end )
        {
            SOURCE_PROVENANCE valueSource = source;
            valueSource.absoluteOffset = separator + 1;
            valueSource.length = end - separator - 1;
            value = PADS_SCH_BINARY_PARSER::DecodeString(
                    { aBytes.begin() + separator + 1, aBytes.begin() + end },
                    DEFAULT_CODE_PAGE, valueSource, aModel.diagnostics );
        }

        return { std::move( name ), std::move( value ) };
    };

    uint32_t pinCursor = 0;
    uint32_t fieldCursor = 0;
    aModel.placements.reserve( aModel.placements.size() + pools[14].count );

    for( size_t record = 0; record < pools[14].count; ++record )
    {
        const size_t offset = aControllers.offsets[14] + record * 128;
        SOURCE_PROVENANCE source = sourceAt(
                aSourceName, aModel.version, wxS( "placement" ), 15, record,
                offset, 128, static_cast<int>( aSheetIndex ) );

        const uint16_t partHandle = aCursor.U16At( offset + 0x3A );
        const uint16_t decalHandle = aCursor.U16At( offset + 0x3C );
        const uint16_t rawUnit = aCursor.U16At( offset + 0x42 );
        const uint32_t startPin = aCursor.U32At( offset + 0x0C );
        const uint16_t countPins = aCursor.U16At( offset + 0x44 );
        const uint16_t countFields = aCursor.U16At( offset + 0x46 );
        const uint32_t groupHandle = aCursor.U32At( offset + 0x14 );

        if( partHandle >= pools[8].count || decalHandle >= pools[6].count
            || groupHandle >= groupPool.count )
            throwDecodeError( source, wxS( "v9 placement part/decal/group reference out of range" ) );

        if( startPin != pinCursor || pinCursor > pools[15].count
            || countPins > pools[15].count - pinCursor
            || fieldCursor > pools[16].count
            || countFields > pools[16].count - fieldCursor )
            throwDecodeError( source, wxS( "v9 placement pin/field slice is not contiguous" ) );

        const uint16_t definitionRecord = aCursor.U16At(
                aControllers.offsets[6] + decalHandle * 92 + 48 );
        auto part = std::ranges::find_if( aModel.partTypes,
                [&]( const MODEL_PART_TYPE& aPart )
                {
                    return aPart.source.sheet == static_cast<int>( aSheetIndex )
                           && aPart.source.recordIndex == partHandle;
                } );
        auto definition = std::ranges::find_if( aModel.definitions,
                [&]( const MODEL_SYMBOL_DEFINITION& aDefinition )
                {
                    return aDefinition.source.sheet == static_cast<int>( aSheetIndex )
                           && aDefinition.source.recordIndex == definitionRecord;
                } );

        if( part == aModel.partTypes.end() || definition == aModel.definitions.end()
            || countPins != definition->pins.size() )
            throwDecodeError( source, wxS( "v9 placed decal/pin/part semantic reference mismatch" ) );

        // The source unit ordinal is NOT always the index in part.gates:
        // PADS connector choices can be reused across many physical units.
        // PADS_V9_GATE_UNIT_SELECTION_V17: however, different *logical gates*
        // may deliberately share the exact same decal (e.g., SWBIP32974
        // A: pins 1-3, B: pins 4-6). In that case the authored unit must
        // select the matching gate, not the first graphical match.
        // Keep the decal-only fallback for connectors with reused graphics;
        // materializeLegacyV9GateUnits() resolves their physical units later.
        const MODEL_GATE* matchingGate = nullptr;

        for( const MODEL_GATE& gate : part->gates )
        {
            const bool primary = gate.definition.id == definition->id;
            const bool alternate = std::ranges::any_of(
                    gate.alternateDefinitions,
                    [&]( const DEFINITION_REFERENCE& aRef ) { return aRef.id == definition->id; } );

            if( primary || alternate )
            {
                if( gate.unit == static_cast<uint32_t>( rawUnit ) + 1 )
                {
                    matchingGate = &gate;
                    break;
                }

                if( !matchingGate )
                    matchingGate = &gate;
            }
        }

        if( !matchingGate )
            throwDecodeError( source, wxS( "v9 selected placement decal is absent from its PARTTYPE gates" ) );

        const size_t groupOffset = groupBase + groupHandle * PLACEMENT_GROUP_BYTES;
        const uint32_t attributeStart = aCursor.U32At( groupOffset );
        const uint16_t attributeCount = aCursor.U16At( groupOffset + 20 );

        if( attributeCount < 2 || attributeStart > attributePool.count
            || attributeCount > attributePool.count - attributeStart )
            throwDecodeError( source, wxS( "v9 placement group attribute slice out of range" ) );

        const SOURCE_STRING groupPartName = attribute( attributeStart + 1, false ).first;

        if( groupPartName.text != part->name.text )
            throwDecodeError( source, wxS( "v9 placement group PARTTYPE does not match placement handle" ) );

        MODEL_PLACEMENT placement;
        placement.id = PLACEMENT_ID( static_cast<uint32_t>( aSheetIndex * 0x100000 + record + 1 ) );
        placement.source = source;
        placement.sheet = { aModel.sheets[aSheetIndex].id, source };
        placement.partType = { part->id, source };
        placement.definition = { definition->id, source };
        placement.gate = GATE_REFERENCE{ matchingGate->id, source };
        placement.unit = rawUnit + 1;
        placement.position = { decodeCoordinate( aCursor.U16At( offset + 0x18 ) ),
                               decodeCoordinate( aCursor.U16At( offset + 0x1A ) ), source };
        placement.angle = NormalizeAngle( aCursor.U16At( offset + 0x1C ) );
        placement.mirrorFlags = aCursor.U16At( offset + 0x1E );
        placement.mirrored = placement.mirrorFlags != 0;
        placement.itemVisibilityFlags = aCursor.U8At( offset + 0x7F );
        placement.referenceVisible = ( placement.itemVisibilityFlags & 1 ) == 0;
        placement.partTypeVisible = ( placement.itemVisibilityFlags & 2 ) == 0;
        placement.pinNamesVisible = ( placement.itemVisibilityFlags & 8 ) == 0;
        // PADS item-visibility flags use hide semantics here, matching
        // the ASCII placement representation: 0x08 hides pin names and
        // 0x10 hides pin numbers. Keep the logical pin number itself even
        // when hidden (for example A/K on diode gate/decal mappings).
        placement.pinNumbersVisible = ( placement.itemVisibilityFlags & 0x10 ) == 0;

        if( placement.angle % 900 != 0 || placement.mirrorFlags > 3 )
            throwDecodeError( source, wxS( "v9 placement has unsupported transform" ) );

        SOURCE_PROVENANCE referenceSource = source;
        referenceSource.absoluteOffset += 0x56;
        referenceSource.length = 40;
        placement.reference = decodeFixedString(
                aBytes, offset + 0x56, 40, referenceSource, aModel.diagnostics );

        if( placement.reference.text.IsEmpty() )
            throwDecodeError( source, wxS( "v9 placement has empty reference" ) );

        if( rawUnit >= part->gates.size() )
            ++counts.reusedGateUnits;

        placement.properties.push_back( sourceProperty(
                wxS( "v9_raw_gate_ordinal" ), wxString::Format( wxS( "%u" ), rawUnit ), source ) );
        placement.properties.push_back( sourceProperty(
                wxS( "component_group_handle" ), wxString::Format( wxS( "%u" ), groupHandle ), source ) );
        placement.properties.push_back( sourceProperty(
                wxS( "decal_handle" ), wxString::Format( wxS( "%u" ), decalHandle ), source ) );

        for( uint16_t pin = 0; pin < countPins; ++pin )
        {
            const size_t pinOffset = aControllers.offsets[15] + ( pinCursor + pin ) * 12;
            SOURCE_PROVENANCE pinSource = sourceAt( aSourceName, aModel.version, wxS( "placed pin" ),
                    16, pinCursor + pin, pinOffset, 12, static_cast<int>( aSheetIndex ) );
            const uint16_t pinOrdinal = aCursor.U16At( pinOffset + 4 );

            if( pinOrdinal != pin || pinOrdinal >= definition->pins.size() )
                throwDecodeError( pinSource, wxS( "v9 placed-pin ordinal does not match selected decal" ) );

            PLACED_PIN_REFERENCE placedPin{ definition->pins[pinOrdinal].id, pinSource };
            placedPin.numberOffset = {
                    static_cast<int64_t>( static_cast<int16_t>( aCursor.U16At( pinOffset + 6 ) ) ) * 2,
                    static_cast<int64_t>( static_cast<int16_t>( aCursor.U16At( pinOffset + 8 ) ) ) * 2,
                    pinSource };
            placedPin.numberPresentationFlags = aCursor.U16At( pinOffset + 10 );
            placedPin.numberAngle = ( placedPin.numberPresentationFlags & 1 ) ? 900 : 0;
            placedPin.numberJustification = terminalJustification(
                    ( placedPin.numberPresentationFlags >> 4 ) & 15, placedPin.numberAngle != 0 );
            placedPin.hasNumberPlacement = true;
            placement.pins.push_back( std::move( placedPin ) );
            ++counts.pins;
        }

        pinCursor += countPins;

        // Inline REF-DES and PART-TYPE have the v13 layout shifted by eight
        // bytes. Text font metadata requires further v9 font inspection.
        auto inlineField = [&]( const wxString& aName, const SOURCE_STRING& aValue,
                                bool aVisible, size_t aPosition, size_t aAngle,
                                size_t aHeight, size_t aWidth )
        {
            SOURCE_PROVENANCE fieldSource = source;
            fieldSource.objectClass = wxS( "inline placement field" );
            fieldSource.absoluteOffset += aPosition;
            fieldSource.length = 8;
            MODEL_FIELD field;
            field.source = fieldSource;
            field.name.text = aName;
            field.name.source = fieldSource;
            field.value = aValue;
            field.position = { decodeLocalCoordinate( aCursor.U16At( offset + aPosition ) ),
                               decodeLocalCoordinate( aCursor.U16At( offset + aPosition + 2 ) ), fieldSource };
            field.angle = NormalizeAngle( aCursor.U16At( offset + aAngle ) );
            field.presentation.source = fieldSource;
            field.visible = aVisible;
            field.presentation.visible = aVisible;
            field.presentation.height = int64_t( aCursor.U16At( offset + aHeight ) ) * 2;
            field.presentation.width = int64_t( aCursor.U8At( offset + aWidth ) ) * 2;
            const uint16_t justification = rotatedFieldJustification(
                    aCursor.U16At( offset + aAngle + 2 ), field.angle );
            field.presentation.horizontalJustification = horizontalJustification( justification );
            field.presentation.verticalJustification = verticalJustification( justification );
            placement.fields.push_back( std::move( field ) );
        };

        inlineField( wxS( "REF-DES" ), placement.reference, placement.referenceVisible,
                     0x20, 0x24, 0x48, 0x50 );
        inlineField( wxS( "PART-TYPE" ), part->name, placement.partTypeVisible,
                     0x28, 0x2C, 0x4A, 0x51 );

        // Preserve source component attributes, including empty values and
        // PCB DECAL. Presentation-only records are not synthetic attributes.
        for( uint16_t index = 2; index < attributeCount; ++index )
        {
            auto [name, value] = attribute( attributeStart + index, true );
            MODEL_FIELD field;
            field.source = name.source;
            field.name = std::move( name );
            field.value = std::move( value );
            field.visible = false;
            field.presentation.source = field.source;
            field.presentation.visible = false;
            placement.fields.push_back( std::move( field ) );
            ++counts.attributeFields;
        }

        for( uint16_t field = 0; field < countFields; ++field, ++fieldCursor )
        {
            const size_t fieldOffset = aControllers.offsets[16] + fieldCursor * 20;
            SOURCE_PROVENANCE fieldSource = sourceAt( aSourceName, aModel.version,
                    wxS( "custom placement field" ), 17, fieldCursor, fieldOffset,
                    20, static_cast<int>( aSheetIndex ) );
            const uint16_t attributeIndex = aCursor.U16At( fieldOffset + 12 );

            if( attributeIndex == 0xFFFF )
            {
                ++counts.unboundCustomFields;
                continue;
            }

            if( attributeIndex >= attributeCount )
                throwDecodeError( fieldSource, wxS( "v9 custom field attribute index out of range" ) );

            // Group attributes are already in placement.fields above; a
            // bound field supplies their local text presentation/visibility.
            auto [name, value] = attribute( attributeStart + attributeIndex, true );
            auto entry = std::ranges::find_if( placement.fields,
                    [&]( const MODEL_FIELD& aField ) { return aField.name.text == name.text; } );

            if( entry == placement.fields.end() )
                throwDecodeError( fieldSource, wxS( "v9 bound custom field is not a group attribute" ) );

            entry->position = {
                    decodeLocalCoordinate( aCursor.U16At( fieldOffset + 4 ) ),
                    decodeLocalCoordinate( aCursor.U16At( fieldOffset + 6 ) ), fieldSource };
            entry->angle = NormalizeAngle( aCursor.U16At( fieldOffset + 8 ) );
            entry->presentation.source = fieldSource;
            entry->presentation.height = int64_t( aCursor.U16At( fieldOffset + 14 ) ) * 2;
            entry->presentation.width = int64_t( aCursor.U8At( fieldOffset + 16 ) ) * 2;
            entry->visible = ( aCursor.U8At( fieldOffset + 17 ) & 8 ) == 0;
            entry->presentation.visible = entry->visible;
        }

        aModel.placements.push_back( std::move( placement ) );
        ++counts.placements;
    }

    if( pinCursor != pools[15].count || fieldCursor != pools[16].count )
        throwDecodeError( aModel.source, wxS( "v9 placed pin / custom field ownership is incomplete" ) );

    return counts;
}

// PADS Logic binary v0x0009 Stage 6. No v0x000D layout reinterpretation:
// v9 retains 40-byte CONNECTION, 8-byte vertex, 12-byte junction and 32-byte
// off-page records, but NETNAMES is 40 (not 48) bytes. Links are always typed
// handles; a wire endpoint referencing a placement must resolve to precisely
// one real transformed pin, not merely a nearby coordinate.
struct LEGACY_V9_CONNECTIVITY_COUNTS
{
    size_t nets = 0;
    size_t wires = 0;
    size_t vertices = 0;
    size_t pinEndpoints = 0;
    size_t junctions = 0;
    size_t offpages = 0;
    size_t netNames = 0;
};

static LEGACY_V9_CONNECTIVITY_COUNTS decodeLegacyV9Connectivity(
        const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
        const PADS_SCH_SDB& aSdb, const SHEET_CONTROLLERS& aControllers,
        size_t aSheetIndex, const wxString& aSourceName, PADS_SCH_MODEL& aModel )
{
    LEGACY_V9_CONNECTIVITY_COUNTS counts;
    const auto& pools = aControllers.pools;
    requireFixedController( aControllers, 18, BUS_RECORD_BYTES, aSourceName, aModel.version, aSheetIndex );
    requireFixedController( aControllers, 19, JUNCTION_RECORD_BYTES, aSourceName, aModel.version, aSheetIndex );
    requireFixedController( aControllers, 20, OFFPAGE_RECORD_BYTES, aSourceName, aModel.version, aSheetIndex );
    requireFixedController( aControllers, 21, CONNECTION_RECORD_BYTES, aSourceName, aModel.version, aSheetIndex );
    requireFixedController( aControllers, 22, CONNECTION_VERTEX_BYTES, aSourceName, aModel.version, aSheetIndex );
    requireFixedController( aControllers, 23, 40, aSourceName, aModel.version, aSheetIndex );

    if( pools[17].count != 0 )
        throwDecodeError( aModel.source, wxS( "v9 bus decoding requires a separate semantic pass" ) );

    const SCH_SDB_POOL& membershipPool = aSdb.Pools()[4];
    const SCH_SDB_POOL& netPool = aSdb.Pools()[8];

    if( uint64_t( membershipPool.count ) * NET_MEMBERSHIP_BYTES != membershipPool.usedBytes
        || uint64_t( netPool.count ) * NET_RECORD_BYTES != netPool.usedBytes )
        throwDecodeError( aModel.source, wxS( "v9 global net/membership stride mismatch" ) );

    const size_t membershipBase = outerControllerOffset( aSdb, 4 );
    const size_t netBase = outerControllerOffset( aSdb, 8 );
    const size_t offpageBase = aControllers.offsets[19];
    const size_t junctionBase = aControllers.offsets[18];
    const size_t connectionBase = aControllers.offsets[20];
    const size_t vertexBase = aControllers.offsets[21];
    const size_t nameBase = aControllers.offsets[22];

    // Each active global net is mapped to its actual sheet membership. The
    // old SDB contains tombstone records that must never become KiCad nets.
    constexpr size_t NO_NET = static_cast<size_t>( -1 );
    std::vector<size_t> netByHandle( netPool.count, NO_NET );
    std::vector<bool> tombstone( netPool.count, false );
    std::vector<bool> claimedMembership( membershipPool.count, false );
    aModel.nets.reserve( aModel.nets.size() + netPool.count );

    for( size_t record = 0; record < netPool.count; ++record )
    {
        const size_t offset = netBase + record * NET_RECORD_BYTES;
        SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version, wxS( "global net" ),
                                             8, record, offset, NET_RECORD_BYTES, -1 );
        const uint32_t membershipStart = aCursor.U32At( offset + 4 );
        const uint16_t membershipCount = aCursor.U16At( offset + 16 );
        const uint16_t aliasCount = aCursor.U16At( offset + 18 );
        tombstone[record] = membershipCount == 0xFFFF;

        if( tombstone[record] )
            continue;

        if( aliasCount != 0 )
            throwDecodeError( source, wxS( "v9 bus alias net requires separate semantic decoder" ) );

        if( membershipStart > membershipPool.count
            || membershipCount > membershipPool.count - membershipStart )
            throwDecodeError( source, wxS( "v9 global net sheet-membership slice out of range" ) );

        SOURCE_PROVENANCE nameSource = source;
        nameSource.absoluteOffset += 24;
        nameSource.length = 56;
        const SOURCE_STRING name = decodeFixedString(
                aBytes, nameSource.absoluteOffset, 56, nameSource, aModel.diagnostics );

        for( size_t index = 0; index < membershipCount; ++index )
        {
            const size_t membershipRecord = membershipStart + index;
            const uint16_t sheet = aCursor.U16At( membershipBase + membershipRecord * NET_MEMBERSHIP_BYTES );

            if( sheet >= aModel.sheets.size() || claimedMembership[membershipRecord] )
                throwDecodeError( source, wxS( "v9 net sheet membership is invalid or claimed twice" ) );

            claimedMembership[membershipRecord] = true;

            if( sheet != aSheetIndex )
                continue;

            if( netByHandle[record] != NO_NET )
                throwDecodeError( source, wxS( "v9 net has multiple memberships on one sheet" ) );

            SOURCE_PROVENANCE membershipSource = sourceAt(
                    aSourceName, aModel.version, wxS( "net sheet membership" ), 4,
                    membershipRecord, membershipBase + membershipRecord * NET_MEMBERSHIP_BYTES,
                    NET_MEMBERSHIP_BYTES, static_cast<int>( aSheetIndex ) );
            MODEL_NET net;
            net.id = NET_ID( static_cast<uint32_t>( membershipRecord ) );
            net.source = source;
            net.source.sheet = static_cast<int>( aSheetIndex );
            net.sheet = { aModel.sheets[aSheetIndex].id, membershipSource };
            net.name = name;
            net.properties.push_back( sourceProperty(
                    wxS( "global_net_record" ), wxString::Format( wxS( "%zu" ), record ), source ) );
            netByHandle[record] = aModel.nets.size();
            aModel.nets.push_back( std::move( net ) );
            ++counts.nets;
        }
    }

    // Connection records own consecutive, non-overlapping vertex slices.
    std::vector<uint32_t> starts( pools[20].count + 1 );
    std::vector<uint32_t> ownerNet( pools[20].count );
    std::vector<std::array<uint16_t, 2>> handles( pools[20].count );
    std::vector<std::vector<size_t>> offBacklinks( pools[19].count );
    std::vector<std::vector<size_t>> junctionBacklinks( pools[18].count );

    for( size_t record = 0; record < pools[20].count; ++record )
    {
        const size_t offset = connectionBase + record * CONNECTION_RECORD_BYTES;
        SOURCE_PROVENANCE source = sourceAt(
                aSourceName, aModel.version, wxS( "connection" ), 21, record,
                offset, CONNECTION_RECORD_BYTES, static_cast<int>( aSheetIndex ) );
        starts[record] = aCursor.U32At( offset + 4 );
        ownerNet[record] = aCursor.U32At( offset + 8 );
        handles[record] = { aCursor.U16At( offset + 12 ), aCursor.U16At( offset + 14 ) };
        const uint16_t marker = aCursor.U16At( offset + 34 );
        const uint8_t status = marker & 0xFF;

        if( ownerNet[record] >= netByHandle.size() || tombstone[ownerNet[record]]
            || netByHandle[ownerNet[record]] == NO_NET )
            throwDecodeError( source, wxS( "v9 connection targets missing/tombstoned net" ) );

        if( marker >> 8 < 2 || marker >> 8 > 6
            || ( status != 0 && status != 0xFC && status != 0xFD ) )
            throwDecodeError( source, wxS( "v9 unknown connection object-class marker" ) );

        // Nonuniform wire lengths are normal. Strict ordering plus the
        // second pass enforces full and nonoverlapping vertex ownership.
        if( record == 0 ? starts[record] != 0 : starts[record] <= starts[record - 1] )
            throwDecodeError( source, wxS( "v9 connection vertex starts are not ordered" ) );
    }

    starts.back() = pools[21].count;

    for( size_t record = 0; record < pools[20].count; ++record )
    {
        const size_t offset = connectionBase + record * CONNECTION_RECORD_BYTES;
        SOURCE_PROVENANCE source = sourceAt(
                aSourceName, aModel.version, wxS( "connection" ), 21, record,
                offset, CONNECTION_RECORD_BYTES, static_cast<int>( aSheetIndex ) );
        const uint32_t start = starts[record];
        const uint32_t end = starts[record + 1];

        if( start >= end || end > pools[21].count || end - start < 2 )
            throwDecodeError( source, wxS( "v9 connection vertex ownership is invalid" ) );

        MODEL_CONNECTION connection;
        connection.source = source;
        connection.vertices.reserve( end - start );

        for( size_t vertex = start; vertex < end; ++vertex )
        {
            const size_t vertexOffset = vertexBase + vertex * CONNECTION_VERTEX_BYTES;
            SOURCE_PROVENANCE vertexSource = sourceAt(
                    aSourceName, aModel.version, wxS( "connection vertex" ), 22,
                    vertex, vertexOffset, CONNECTION_VERTEX_BYTES, static_cast<int>( aSheetIndex ) );

            if( aCursor.U32At( vertexOffset ) != 0 )
                throwDecodeError( vertexSource, wxS( "v9 connection vertex has nonzero padding" ) );

            connection.vertices.push_back( {
                    decodeCoordinate( aCursor.U16At( vertexOffset + 4 ) ),
                    decodeCoordinate( aCursor.U16At( vertexOffset + 6 ) ), vertexSource } );
            ++counts.vertices;
        }

        for( size_t endpoint = 0; endpoint < 2; ++endpoint )
        {
            const uint16_t raw = handles[record][endpoint];
            const uint16_t objectClass = raw >> 12;
            const uint16_t objectRecord = raw & 0x0FFF;
            const SOURCE_POINT& wirePoint = endpoint == 0 ? connection.vertices.front()
                                                            : connection.vertices.back();
            SOURCE_PROVENANCE endpointSource = source;
            endpointSource.objectClass = wxS( "connection endpoint" );
            endpointSource.absoluteOffset += 12 + endpoint * 2;
            endpointSource.length = 2;
            MODEL_CONNECTION_ENDPOINT decoded;
            decoded.source = endpointSource;
            decoded.point = wirePoint;

            if( objectClass == 0 )
            {
                auto placement = std::ranges::find_if( aModel.placements,
                        [&]( const MODEL_PLACEMENT& aPlacement )
                        {
                            return aPlacement.source.sheet == static_cast<int>( aSheetIndex )
                                   && aPlacement.source.recordIndex == objectRecord;
                        } );

                if( placement == aModel.placements.end() )
                    throwDecodeError( endpointSource, wxS( "v9 connection placement endpoint unresolved" ) );

                const PIN_REFERENCE* matchingPin = nullptr;

                for( const PIN_REFERENCE& placedPin : placement->pins )
                {
                    const MODEL_PIN_DEFINITION* definitionPin = nullptr;

                    for( const MODEL_SYMBOL_DEFINITION& definition : aModel.definitions )
                    {
                        auto found = std::ranges::find(
                                definition.pins, placedPin.id, &MODEL_PIN_DEFINITION::id );

                        if( found != definition.pins.end() )
                        {
                            definitionPin = &*found;
                            break;
                        }
                    }

                    if( !definitionPin )
                        throwDecodeError( placedPin.source, wxS( "v9 placed pin has no definition" ) );

                    const auto position = transformedPinPosition( *placement, *definitionPin );

                    if( position.first != wirePoint.x || position.second != wirePoint.y )
                        continue;

                    if( matchingPin )
                        throwDecodeError( endpointSource, wxS( "v9 connection is ambiguous between placement pins" ) );

                    matchingPin = &placedPin;
                }

                if( !matchingPin )
                    throwDecodeError( endpointSource, wxS( "v9 connection has no pin at wire endpoint" ) );

                decoded.kind = MODEL_ENDPOINT_KIND::PIN;
                decoded.placement = PLACEMENT_REFERENCE{ placement->id, endpointSource };
                decoded.pin = PIN_REFERENCE{ matchingPin->id, endpointSource };
                ++counts.pinEndpoints;
            }
            else if( objectClass == 2 || objectClass == 3 )
            {
                if( endpoint == 1 && handles[record][0] == raw )
                    throwDecodeError( endpointSource, wxS( "v9 repeated typed endpoint in same wire" ) );

                const bool isOffpage = objectClass == 2;
                const size_t count = isOffpage ? pools[19].count : pools[18].count;
                const size_t objectBase = isOffpage ? offpageBase : junctionBase;
                const size_t objectStride = isOffpage ? OFFPAGE_RECORD_BYTES : JUNCTION_RECORD_BYTES;
                const size_t objectX = isOffpage ? 22 : 4;
                const size_t objectY = isOffpage ? 24 : 6;

                if( objectRecord >= count )
                    throwDecodeError( endpointSource, wxS( "v9 offpage/junction endpoint out of range" ) );

                const size_t objectOffset = objectBase + objectRecord * objectStride;

                if( decodeCoordinate( aCursor.U16At( objectOffset + objectX ) ) != wirePoint.x
                    || decodeCoordinate( aCursor.U16At( objectOffset + objectY ) ) != wirePoint.y )
                    throwDecodeError( endpointSource, wxS( "v9 typed endpoint position does not match wire vertex" ) );

                if( isOffpage )
                    offBacklinks[objectRecord].push_back( record );
                else
                    junctionBacklinks[objectRecord].push_back( record );

                decoded.kind = MODEL_ENDPOINT_KIND::POINT;
            }
            else
            {
                throwDecodeError( endpointSource, wxS( "v9 unknown connection endpoint object class" ) );
            }

            decoded.properties.push_back( sourceProperty(
                    wxS( "raw_endpoint_handle" ), wxString::Format( wxS( "%u" ), raw ), endpointSource ) );
            const size_t relationshipOffset = offset + 16 + endpoint * 4;
            SOURCE_PROVENANCE relationshipSource = endpointSource;
            relationshipSource.absoluteOffset = relationshipOffset;
            relationshipSource.length = 4;
            SOURCE_PROPERTY relationshipProperty = sourceProperty(
                    wxS( "raw_endpoint_relationship" ),
                    wxString::Format( wxS( "%u" ), aCursor.U32At( relationshipOffset ) ), relationshipSource );
            relationshipProperty.disposition = PROPERTY_DISPOSITION::PRESERVED;
            decoded.properties.push_back( std::move( relationshipProperty ) );
            connection.endpoints.push_back( std::move( decoded ) );
        }

        connection.properties.push_back( sourceProperty(
                wxS( "raw_connection_marker" ),
                wxString::Format( wxS( "%u" ), aCursor.U16At( offset + 34 ) ), source ) );
        aModel.nets[netByHandle[ownerNet[record]]].connections.push_back( std::move( connection ) );
        ++counts.wires;
    }

    if( pools[20].count == 0 && pools[21].count != 0 )
        throwDecodeError( aModel.source, wxS( "v9 unowned connection vertices" ) );

    // Typed endpoints must reference their owner connection, and every
    // junction/off-page backlink must remain on the same global net.
    for( size_t record = 0; record < pools[18].count; ++record )
    {
        const size_t offset = junctionBase + record * JUNCTION_RECORD_BYTES;
        SOURCE_PROVENANCE source = sourceAt(
                aSourceName, aModel.version, wxS( "junction" ), 19, record,
                offset, JUNCTION_RECORD_BYTES, static_cast<int>( aSheetIndex ) );
        const uint16_t connectionHandle = aCursor.U16At( offset + 8 );
        // PADS v0x0009 junction class is the low byte of the raw marker.
        // Observed high bytes include 00, FF, 45, 54, 32 and 02. Their
        // semantics are unknown, so preserve the original 16-bit word.
        const uint16_t rawClassMarker = aCursor.U16At( offset + 10 );
        const uint8_t  junctionClass = rawClassMarker & 0xFF;

        if( rawClassMarker != 0 && junctionClass != 0xFC && junctionClass != 0xFD )
            throwDecodeError( source, wxS( "v9 junction has unknown class marker" ) );

        if( connectionHandle >= pools[20].count
            || std::ranges::find( junctionBacklinks[record], connectionHandle ) == junctionBacklinks[record].end() )
            throwDecodeError( source, wxS( "v9 junction backlink does not resolve to its wire" ) );

        for( size_t backlink : junctionBacklinks[record] )
        {
            if( ownerNet[backlink] != ownerNet[connectionHandle] )
                throwDecodeError( source, wxS( "v9 junction joins different nets" ) );
        }

        MODEL_JUNCTION junction;
        junction.source = source;
        junction.sheet = { aModel.sheets[aSheetIndex].id, source };
        junction.position = {
                decodeCoordinate( aCursor.U16At( offset + 4 ) ),
                decodeCoordinate( aCursor.U16At( offset + 6 ) ), source };
        SOURCE_PROVENANCE markerSource = source;
        markerSource.absoluteOffset += 10;
        markerSource.length = 2;
        SOURCE_PROPERTY markerProperty = sourceProperty(
                wxS( "raw_junction_class_marker" ),
                wxString::Format( wxS( "%u" ), rawClassMarker ), markerSource );
        markerProperty.disposition = PROPERTY_DISPOSITION::PRESERVED;
        junction.properties.push_back( std::move( markerProperty ) );
        aModel.junctions.push_back( std::move( junction ) );
        ++counts.junctions;
    }

    // v9 OFFPAGE records use a decal+connection handle and the same
    // angle/mirror fields as the later versions.
    for( size_t record = 0; record < pools[19].count; ++record )
    {
        const size_t offset = offpageBase + record * OFFPAGE_RECORD_BYTES;
        SOURCE_PROVENANCE source = sourceAt(
                aSourceName, aModel.version, wxS( "off-page reference" ), 20, record,
                offset, OFFPAGE_RECORD_BYTES, static_cast<int>( aSheetIndex ) );
        const uint16_t decalHandle = aCursor.U16At( offset + 4 );
        const uint16_t connectionHandle = aCursor.U16At( offset + 8 );
        const uint8_t variant = aCursor.U8At( offset + 30 );
        const uint16_t mirror = aCursor.U16At( offset + 28 );

        // Only the low two mirror bits have verified geometric meaning.
        // Other bits exist in real v9 files and are preserved below.
        const uint16_t knownMirrorFlags = mirror & uint16_t{ 3 };

        if( decalHandle >= pools[6].count || connectionHandle >= pools[20].count )
            throwDecodeError( source, wxS( "v9 off-page decal/connection handle invalid" ) );

        if( std::ranges::find( offBacklinks[record], connectionHandle ) == offBacklinks[record].end() )
            throwDecodeError( source, wxS( "v9 off-page backlink does not point to its connection" ) );

        for( size_t backlink : offBacklinks[record] )
        {
            if( ownerNet[backlink] != ownerNet[connectionHandle] )
                throwDecodeError( source, wxS( "v9 off-page joins different nets" ) );
        }

        const size_t decalOffset = aControllers.offsets[6] + decalHandle * 92;
        SOURCE_PROVENANCE decalSource = sourceAt(
                aSourceName, aModel.version, wxS( "used decal" ), 7, decalHandle,
                decalOffset, 92, static_cast<int>( aSheetIndex ) );
        const uint16_t definitionRecord = aCursor.U16At( decalOffset + 48 );

        if( definitionRecord >= pools[2].count )
            throwDecodeError( decalSource, wxS( "v9 off-page decal definition handle invalid" ) );

        const DEFINITION_ID definitionId(
                static_cast<uint32_t>( aSheetIndex * 0x100000 + 1 + definitionRecord ) );
        auto definition = std::ranges::find(
                aModel.definitions, definitionId, &MODEL_SYMBOL_DEFINITION::id );

        if( definition == aModel.definitions.end() )
            throwDecodeError( decalSource, wxS( "v9 off-page decal definition was not decoded" ) );

        MODEL_LABEL label;
        label.source = source;
        label.sheet = { aModel.sheets[aSheetIndex].id, source };
        label.text = aModel.nets[netByHandle[ownerNet[connectionHandle]]].name;
        label.position = {
                decodeCoordinate( aCursor.U16At( offset + 22 ) ),
                decodeCoordinate( aCursor.U16At( offset + 24 ) ), source };
        label.decalName = decodeFixedString( aBytes, decalOffset, 40, decalSource, aModel.diagnostics );
        label.symbolVariant = variant;
        label.offpageBodyHeight = binaryOffPageBodyHeight( *definition );
        for( const MODEL_FIELD& field : definition->fields )
        {
            if( field.name.text.CmpNoCase( wxS( "REF-DES" ) ) != 0 )
                continue;

            label.hasDefaultPresentation = true;
            label.defaultTextOffset = field.position;
            label.defaultAngle = field.angle;
            label.defaultPresentation = field.presentation;
            break;
        }
        label.presentation.source = source;
        label.presentation.visible = false;

        int authoredAngle = NormalizeAngle( aCursor.U16At( offset + 26 ) );

        if( knownMirrorFlags & 1 )
            authoredAngle = NormalizeAngle( 1800 - authoredAngle );

        if( knownMirrorFlags & 2 )
            authoredAngle = NormalizeAngle( -authoredAngle );

        // OFFPAGE transforms its symbol. NETNAMES independently rotates its text.
        label.symbolAngle = authoredAngle;
        SOURCE_PROVENANCE mirrorSource = source;
        mirrorSource.absoluteOffset += 28;
        mirrorSource.length = 2;
        SOURCE_PROPERTY mirrorProperty = sourceProperty(
                wxS( "offpage_mirror_flags" ), wxString::Format( wxS( "%u" ), mirror ), mirrorSource );

        if( mirror != knownMirrorFlags )
        {
            mirrorProperty.disposition = PROPERTY_DISPOSITION::UNSUPPORTED;
            aModel.diagnostics.push_back( MakePropertyDiagnostic(
                    RPT_SEVERITY_WARNING, mirrorProperty,
                    wxS( "unsupported PADS v9 off-page mirror flag bits preserved" ) ) );
        }

        label.properties.push_back( std::move( mirrorProperty ) );
        label.properties.push_back( sourceProperty(
                wxS( "v9_authored_offpage_angle" ),
                wxString::Format( wxS( "%u" ), aCursor.U16At( offset + 26 ) ), source ) );

        bool recognizedKind = false;

        for( const MODEL_PART_TYPE& part : aModel.partTypes )
        {
            if( part.source.sheet != static_cast<int>( aSheetIndex ) )
                continue;

            const bool hasDefinition = std::ranges::any_of( part.gates,
                    [&]( const MODEL_GATE& gate )
                    {
                        return gate.definition.id == definitionId
                            || std::ranges::any_of( gate.alternateDefinitions,
                                   [&]( const DEFINITION_REFERENCE& ref )
                                   {
                                       return ref.id == definitionId;
                                   } );
                    } );

            if( !hasDefinition )
                continue;

            if( part.name.text == wxS( "$GND_SYMS" ) )
                label.kind = MODEL_LABEL_KIND::GROUND;
            else if( part.name.text == wxS( "$PWR_SYMS" ) )
                label.kind = MODEL_LABEL_KIND::POWER;
            else if( part.name.text == wxS( "$OSR_SYMS" ) )
                label.kind = MODEL_LABEL_KIND::GLOBAL;
            else
                throwDecodeError( decalSource, wxS( "v9 off-page decal belongs to unexpected PARTTYPE" ) );

            recognizedKind = true;
            break;
        }

        if( !recognizedKind )
            throwDecodeError( decalSource, wxS( "v9 off-page decal has no owning special PARTTYPE" ) );

        // PADS v9 power/ground CAEDECALs are drawn upright in their native
        // coordinate frame. Their OFFPAGE mirror bits act in the *rotated*
        // drawing frame: bit 0 reflects X (horizontal), bit 1 reflects Y.
        // The REFIN/REFOUT convention has a different native direction and
        // retains the separately verified OSR transform from Stage 8.
        // Example: O20 (+12V, 90 deg, mirror 1) must point right (270 KiCad),
        // while O4 (+12V, 90 deg, mirror 3) is already correctly at 270.
        if( label.kind == MODEL_LABEL_KIND::POWER || label.kind == MODEL_LABEL_KIND::GROUND )
        {
            int powerAngle = NormalizeAngle( aCursor.U16At( offset + 26 ) );

            if( mirror & 1 )
                powerAngle = NormalizeAngle( -powerAngle );

            if( mirror & 2 )
                powerAngle = NormalizeAngle( 1800 - powerAngle );

            label.symbolAngle = powerAngle;
        }

        label.properties.push_back( sourceProperty(
                wxS( "offpage_variant" ), wxString::Format( wxS( "%u" ), variant ), source ) );
        aModel.labels.push_back( std::move( label ) );
        ++counts.offpages;
    }

    // In v9 NETNAMES has NO 8-byte v13 prefix: its net record is at +8,
    // text offset at +12/+14, angle +16, justification +18 and the owning
    // off-page typed handle at +30. Missing presentations remain invisible.
    std::vector<bool> seenName( pools[19].count, false );

    for( size_t record = 0; record < pools[22].count; ++record )
    {
        const size_t offset = nameBase + record * 40;
        SOURCE_PROVENANCE source = sourceAt(
                aSourceName, aModel.version, wxS( "net-name presentation" ), 23,
                record, offset, 40, static_cast<int>( aSheetIndex ) );
        const uint32_t globalHandle = aCursor.U32At( offset + 8 );
        const uint16_t owner = aCursor.U16At( offset + 30 );
        // PADS_V9_NETNAMES_PLACEMENT_OWNER_V15
        // The v9 owner/child handles are at +30/+32 (v13 uses +38/+40).
        // The owner may be an off-page (class 2) or a placed symbol (class 0).
        const uint16_t childOrdinal = aCursor.U16At( offset + 32 );

        if( globalHandle >= netByHandle.size() || tombstone[globalHandle]
            || netByHandle[globalHandle] == NO_NET )
            throwDecodeError( source, wxS( "v9 NETNAMES targets invalid/unresolved net" ) );

        if( ( owner & 0xF000 ) == 0 )
        {
            // A NETNAME attached to a component pin is free text at the
            // electrical endpoint, not an off-page label. Resolve by BOTH
            // placement identity and pin identity on the owning net.
            auto placement = std::ranges::find_if( aModel.placements,
                    [&]( const MODEL_PLACEMENT& aPlacement )
                    {
                        return aPlacement.source.sheet == static_cast<int>( aSheetIndex )
                               && aPlacement.source.recordIndex == owner;
                    } );

            if( placement == aModel.placements.end() )
                throwDecodeError( source, wxS( "v9 NETNAMES placement owner is unresolved" ) );

            if( childOrdinal >= placement->pins.size() )
                throwDecodeError( source, wxS( "v9 NETNAMES placement pin ordinal out of range" ) );

            const PIN_ID pinId = placement->pins[childOrdinal].id;
            const MODEL_NET& net = aModel.nets[netByHandle[globalHandle]];
            const MODEL_CONNECTION_ENDPOINT* matchingEndpoint = nullptr;

            for( const MODEL_CONNECTION& connection : net.connections )
            {
                for( const MODEL_CONNECTION_ENDPOINT& endpoint : connection.endpoints )
                {
                    if( endpoint.kind != MODEL_ENDPOINT_KIND::PIN || !endpoint.placement
                        || !endpoint.pin || endpoint.placement->id != placement->id
                        || endpoint.pin->id != pinId )
                        continue;

                    if( matchingEndpoint )
                        throwDecodeError( source, wxS( "v9 NETNAMES placement pin is ambiguous on net" ) );

                    matchingEndpoint = &endpoint;
                }
            }

            if( !matchingEndpoint )
                throwDecodeError( source, wxS( "v9 NETNAMES placement pin is not on owning net" ) );

            const int64_t dx = decodeLocalCoordinate( aCursor.U16At( offset + 12 ) );
            const int64_t dy = decodeLocalCoordinate( aCursor.U16At( offset + 14 ) );
            MODEL_TEXT text;
            text.source = source;
            text.sheet = { aModel.sheets[aSheetIndex].id, source };
            text.text = net.name;
            text.position = { matchingEndpoint->point.x + dx,
                              matchingEndpoint->point.y + dy, source };
            text.angle = NormalizeAngle( aCursor.U16At( offset + 16 ) );
            text.presentation.source = source;
            text.presentation.visible = true;
            // Match current v9 NETNAMES assumptions. Actual per-file defaults
            // are not yet decoded; adjust all v9 NETNAMES together later.
            text.presentation.height = 200;
            text.presentation.width = 20;
            const uint16_t justification = aCursor.U16At( offset + 18 );
            text.presentation.horizontalJustification = horizontalJustification( justification );
            text.presentation.verticalJustification = verticalJustification( justification );
            text.properties.push_back( sourceProperty(
                    wxS( "v9_netname_placement_owner" ),
                    wxString::Format( wxS( "%u,%u" ), owner, childOrdinal ), source ) );
            text.properties.push_back( sourceProperty(
                    wxS( "v9_netname_text_offset" ),
                    wxString::Format( wxS( "%lld,%lld" ), dx, dy ), source ) );
            SOURCE_PROPERTY assumedMetrics = sourceProperty(
                    wxS( "v9_netname_metrics_from_ascii_defaults" ), wxS( "100,10 mil" ), source );
            assumedMetrics.disposition = PROPERTY_DISPOSITION::APPROXIMATE;
            text.properties.push_back( std::move( assumedMetrics ) );
            aModel.texts.push_back( std::move( text ) );
            ++counts.netNames;
            continue;
        }

        const size_t offpageRecord = owner & 0x0FFF;

        if( ( owner & 0xF000 ) != 0x2000 || offpageRecord >= pools[19].count
            || seenName[offpageRecord] )
            throwDecodeError( source, wxS( "v9 NETNAMES has invalid/duplicate off-page owner" ) );

        const uint16_t connectionHandle = aCursor.U16At( offpageBase + offpageRecord * 32 + 8 );

        if( ownerNet[connectionHandle] != globalHandle )
            throwDecodeError( source, wxS( "v9 NETNAMES does not belong to off-page connection net" ) );

        auto label = std::ranges::find_if( aModel.labels,
                [&]( const MODEL_LABEL& aLabel )
                {
                    return aLabel.source.sheet == static_cast<int>( aSheetIndex )
                           && aLabel.source.controller == 20
                           && aLabel.source.recordIndex == offpageRecord;
                } );

        if( label == aModel.labels.end() )
            throwDecodeError( source, wxS( "v9 NETNAMES owner label is absent" ) );

        seenName[offpageRecord] = true;
        label->textOffset = {
                decodeLocalCoordinate( aCursor.U16At( offset + 12 ) ),
                decodeLocalCoordinate( aCursor.U16At( offset + 14 ) ), source };
        label->hasNetNamePresentation = true;
        label->angle = NormalizeAngle( aCursor.U16At( offset + 16 ) );
        label->presentation.source = source;
        label->presentation.visible = true;
        // The legacy v0x0009 NETNAMES record omits its default text metrics.
        // Preserve the authored 100/10-mil NETNAMESIZE from the matching ASCII
        // export (in half-mils here). The builder performs Stroke font scaling
        // once for POWER/GROUND VALUE fields and other schematic text.
        // Do not pre-shrink POWER/GROUND to 60 mil: it would be scaled twice.
        label->presentation.height = 200;
        label->presentation.width = 20;
        const uint16_t justification = aCursor.U16At( offset + 18 );
        label->presentation.horizontalJustification = horizontalJustification( justification );
        label->presentation.verticalJustification = verticalJustification( justification );
        label->properties.push_back( sourceProperty(
                wxS( "v9_netname_text_angle" ),
                wxString::Format( wxS( "%u" ), aCursor.U16At( offset + 16 ) ), source ) );
        SOURCE_PROPERTY assumedMetrics = sourceProperty(
                wxS( "v9_netname_metrics_from_ascii_defaults" ), wxS( "100,10 mil" ), source );
        assumedMetrics.disposition = PROPERTY_DISPOSITION::APPROXIMATE;
        label->properties.push_back( std::move( assumedMetrics ) );
        ++counts.netNames;
    }

    return counts;
}

// v0x0009 legacy record completion. The old sheet text controller consists
// of 28-byte records (not v13's 32 bytes). The 92-byte USED_DECAL header
// contains REF-DES / PART-TYPE local presentation, pin-size defaults and
// an independent definition pointer. Never reinterpret it as a 108-byte v13
// decal: its fields are not aligned with those of the newer format.
struct LEGACY_V9_FINAL_COUNTS
{
    size_t signalPins = 0;
    size_t standardFields = 0;
    size_t embeddedTexts = 0;
    size_t pageTexts = 0;
    size_t worksheets = 0;
    size_t placeholders = 0;
    size_t preservedTexts = 0;
    size_t resolvedPinBodies = 0;
};

static LEGACY_V9_FINAL_COUNTS decodeLegacyV9FinalDetails(
        const std::vector<uint8_t>& aBytes, const PADS_IO::BINARY_CURSOR& aCursor,
        const SHEET_CONTROLLERS& aControllers, size_t aSheetIndex,
        const wxString& aSourceName, PADS_SCH_MODEL& aModel )
{
    LEGACY_V9_FINAL_COUNTS counts;
    const auto& pools = aControllers.pools;
    requireFixedController( aControllers, 1, 28, aSourceName, aModel.version, aSheetIndex );
    requireFixedController( aControllers, 7, 92, aSourceName, aModel.version, aSheetIndex );
    requireFixedController( aControllers, 12, 52, aSourceName, aModel.version, aSheetIndex );
    const size_t textBase = aControllers.offsets[0];
    const size_t heapBase = aControllers.offsets[1];
    const uint32_t heapSize = pools[1].usedBytes;
    const size_t decalBase = aControllers.offsets[6];
    const size_t defBase = aControllers.offsets[2];
    const size_t partBase = aControllers.offsets[8];
    const size_t signalBase = aControllers.offsets[11];

    // SIGPIN in v9: u16 physical pin number, one-byte legacy '2' marker,
    // then a 49-byte NUL-terminated default net name. The 10 records in the fixture belong to $GND_SYMS and
    // $PWR_SYMS; don't fabricate instance overrides from these defaults.
    std::vector<bool> claimedSignals( pools[11].count, false );

    for( MODEL_PART_TYPE& part : aModel.partTypes )
    {
        if( part.source.sheet != static_cast<int>( aSheetIndex ) )
            continue;

        const size_t partOffset = partBase + part.source.recordIndex * PART_TYPE_BYTES;
        const uint32_t first = aCursor.U32At( partOffset + 52 );
        const uint16_t count = aCursor.U16At( partOffset + 66 );

        if( first > pools[11].count || count > pools[11].count - first )
            throwDecodeError( part.source, wxS( "v9 SIGPIN slice leaves controller 12" ) );

        for( size_t index = 0; index < count; ++index )
        {
            const size_t record = first + index;
            const size_t offset = signalBase + record * 52;
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version,
                    wxS( "signal pin" ), 12, record, offset, 52, static_cast<int>( aSheetIndex ) );

            if( claimedSignals[record] )
                throwDecodeError( source, wxS( "v9 SIGPIN record claimed twice" ) );

            claimedSignals[record] = true;
            MODEL_SIGNAL_PIN pin;
            pin.source = source;
            pin.number.source = source;
            pin.number.text = wxString::Format( wxS( "%u" ), aCursor.U16At( offset ) );
            SOURCE_PROVENANCE nameSource = source;
            if( aCursor.U8At( offset + 2 ) != '2' )
                throwDecodeError( source, wxS( "v9 SIGPIN has unrecognized name-prefix marker" ) );

            nameSource.absoluteOffset += 3;
            nameSource.length = 49;
            pin.name = decodeFixedString( aBytes, offset + 3, 49, nameSource, aModel.diagnostics );

            if( pin.number.text.IsEmpty() || pin.name.text.IsEmpty() )
                throwDecodeError( source, wxS( "v9 SIGPIN has empty physical pin or net name" ) );

            part.signalPins.push_back( std::move( pin ) );
            ++counts.signalPins;
        }
    }

    if( std::ranges::any_of( claimedSignals, []( bool claimed ) { return !claimed; } ) )
        throwDecodeError( aModel.source, wxS( "v9 unowned SIGPIN record" ) );

    // Each non-tombstoned used decal owns two standard text fields and its
    // terminal text metrics. +54/+62 store the field X/Y; the four u16s at
    // +70..+76 store text heights, while packed bytes at +78/+80 store widths.
    std::vector<bool> visitedDefinition( pools[2].count, false );

    for( size_t record = 0; record < pools[6].count; ++record )
    {
        const size_t usedOffset = decalBase + record * 92;
        const uint16_t definitionRecord = aCursor.U16At( usedOffset + 48 );

        if( definitionRecord == 0xFFFF )
            continue;

        if( definitionRecord >= pools[2].count )
            throwDecodeError( aModel.source, wxS( "v9 standard-field definition index is invalid" ) );

        auto definition = std::ranges::find_if( aModel.definitions,
                [&]( const MODEL_SYMBOL_DEFINITION& d )
                {
                    return d.source.sheet == static_cast<int>( aSheetIndex )
                           && d.source.recordIndex == definitionRecord;
                } );

        if( definition == aModel.definitions.end() )
            continue;

        if( visitedDefinition[definitionRecord] )
        {
            // A second decal may alias the same definition, but its standard
            // text presentation must not produce duplicate default fields.
            continue;
        }

        visitedDefinition[definitionRecord] = true;

        for( size_t standard = 0; standard < 2; ++standard )
        {
            const size_t posOffset = usedOffset + 54 + standard * 8;
            SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version,
                    wxS( "standard definition field" ), 7, record,
                    posOffset, 8, static_cast<int>( aSheetIndex ) );
            MODEL_FIELD field;
            field.source = source;
            field.name.source = source;
            field.name.text = standard == 0 ? wxS( "REF-DES" ) : wxS( "PART-TYPE" );
            field.position = { decodeLocalCoordinate( aCursor.U16At( posOffset ) ),
                               decodeLocalCoordinate( aCursor.U16At( posOffset + 2 ) ), source };
            field.angle = NormalizeAngle( aCursor.U16At( posOffset + 4 ) );
            const uint16_t justification = aCursor.U16At( posOffset + 6 );
            field.presentation.source = source;
            field.presentation.horizontalJustification = horizontalJustification( justification );
            field.presentation.verticalJustification = verticalJustification( justification );
            field.presentation.height = int64_t( aCursor.U16At( usedOffset + 70 + standard * 2 ) ) * 2;
            field.presentation.width = int64_t( aCursor.U8At( usedOffset + 78 + standard ) ) * 2;
            field.presentation.font = decodedDefinitionFont( -1, source );
            field.presentation.visible = true;
            field.visible = true;
            definition->fields.push_back( std::move( field ) );
            ++counts.standardFields;
        }

        for( MODEL_PIN_DEFINITION& pin : definition->pins )
        {
            const size_t terminalOffset = pin.source.absoluteOffset;
            pin.numberPresentation.height = int64_t( aCursor.U16At( usedOffset + 74 ) ) * 2;
            pin.namePresentation.height = int64_t( aCursor.U16At( usedOffset + 76 ) ) * 2;
            pin.numberPresentation.width = int64_t( aCursor.U8At( usedOffset + 80 ) ) * 2;
            pin.namePresentation.width = int64_t( aCursor.U8At( usedOffset + 81 ) ) * 2;
            // In legacy v9 the terminal presentation bit 0x2000 is the
            // PART/CAEDECAL default for displaying the pin number.
            pin.numberPresentation.visible = pin.numberPresentation.height != 0
                    && ( pin.presentationFlags & 0x2000 ) != 0;
            pin.namePresentation.visible = pin.namePresentation.height != 0
                    && ( pin.visibilityAndNumberPresentationFlags & 0x8000 ) == 0;
            pin.numberJustification = terminalJustification(
                    ( pin.presentationFlags >> 12 ) & 0xF, pin.numberAngle != 0 );
            pin.nameJustification = terminalJustification(
                    ( pin.visibilityAndNumberPresentationFlags >> 4 ) & 0xF,
                    pin.nameAngle != 0 );
            pin.numberPresentation.font = decodedDefinitionFont( -1, pin.source );
            pin.namePresentation.font = decodedDefinitionFont( -1, pin.source );
            const uint16_t handle = aCursor.U16At( terminalOffset );

            if( handle == 0xFFFF )
                continue;

            if( handle >= pools[6].count )
                throwDecodeError( pin.source, wxS( "v9 pin decal handle outside controller 7" ) );

            const size_t pinUsedOffset = decalBase + handle * 92;
            const uint16_t pinDefinitionRecord = aCursor.U16At( pinUsedOffset + 48 );
            auto pinDefinition = std::ranges::find_if( aModel.definitions,
                    [&]( const MODEL_SYMBOL_DEFINITION& d )
                    {
                        return d.source.sheet == static_cast<int>( aSheetIndex )
                               && d.source.recordIndex == pinDefinitionRecord;
                    } );

            if( pinDefinition == aModel.definitions.end() )
                throwDecodeError( pin.source, wxS( "v9 pin decal does not resolve to a graphic definition" ) );

            int64_t bestDistanceSquared = 0;
            SOURCE_POINT bestVector;
            int64_t bestStrokeWidth = 0;
            MODEL_LINE_STYLE bestLineStyle = MODEL_LINE_STYLE::DEFAULT;

            for( const MODEL_GRAPHIC& graphic : pinDefinition->graphics )
            {
                if( graphic.kind != MODEL_GRAPHIC_KIND::LINE
                    && graphic.kind != MODEL_GRAPHIC_KIND::POLYLINE )
                    continue;

                const bool containsOrigin = std::ranges::any_of( graphic.points,
                        []( const SOURCE_POINT& point )
                        {
                            return point.x == 0 && point.y == 0;
                        } );

                if( !containsOrigin )
                    continue;

                for( const SOURCE_POINT& point : graphic.points )
                {
                    const int64_t distance = point.x * point.x + point.y * point.y;

                    if( distance > bestDistanceSquared )
                    {
                        bestDistanceSquared = distance;
                        bestVector = point;
                        bestStrokeWidth = graphic.strokeWidth;
                        bestLineStyle = graphic.lineStyle;
                    }
                }
            }

            if( bestDistanceSquared == 0 )
                throwDecodeError( pin.source, wxS( "v9 pin decal has no drawable root vector" ) );

            pin.pinDecalVector = bestVector;
            pin.pinDecalStrokeWidth = bestStrokeWidth;
            pin.pinDecalLineStyle = bestLineStyle;
            pin.length = int64_t( std::llround( std::sqrt( double( bestDistanceSquared ) ) ) );
            pin.pinDecalGeometryResolved = true;
            const wxString name = pinDefinition->name.text;
            const bool inverted = name == wxS( "PINB" ) || name == wxS( "PINORB" )
                    || name == wxS( "PCLKB" ) || name == wxS( "PINIEB" );
            const bool clock = name == wxS( "PCLK" ) || name == wxS( "PCLKB" );
            pin.graphicStyle = ( inverted ? 1U : 0U ) | ( clock ? 2U : 0U );
            ++counts.resolvedPinBodies;
        }
    }

    // Claim page text through its owner-linked predecessor chain, not by
    // treating every controller-1 record as a standalone free text. Reserve
    // the free-text records already materialized by Stage 3.
    std::vector<bool> usedText( pools[0].count, false );

    for( size_t record = 0; record < pools[0].count; ++record )
    {
        const size_t offset = textBase + record * 28;

        if( aCursor.U16At( offset + 20 ) == aCursor.U16At( offset + 22 ) )
            usedText[record] = true;
    }

    auto makeTextGraphic = [&]( size_t record, const wxString& className )
    {
        const size_t offset = textBase + record * 28;
        SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version,
                className, 1, record, offset, 28, static_cast<int>( aSheetIndex ) );
        const uint32_t stringOffset = aCursor.U32At( offset + 4 );
        const uint16_t stringBytes = aCursor.U16At( offset + 16 );

        if( stringOffset > heapSize || stringBytes == 0
            || stringBytes > heapSize - stringOffset
            || aBytes[heapBase + stringOffset + stringBytes - 1] != 0 )
            throwDecodeError( source, wxS( "v9 owned text leaves controller 2 string heap" ) );

        SOURCE_PROVENANCE stringSource = sourceAt(
                aSourceName, aModel.version, className + wxS( " string" ), 2,
                record, heapBase + stringOffset, stringBytes - 1, static_cast<int>( aSheetIndex ) );
        MODEL_GRAPHIC graphic;
        graphic.source = source;
        graphic.kind = MODEL_GRAPHIC_KIND::TEXT;
        graphic.text = PADS_SCH_BINARY_PARSER::DecodeString(
                { aBytes.begin() + stringSource.absoluteOffset,
                  aBytes.begin() + stringSource.absoluteOffset + stringSource.length },
                DEFAULT_CODE_PAGE, stringSource, aModel.diagnostics );
        graphic.points.push_back( { decodeLocalCoordinate( aCursor.U16At( offset + 8 ) ),
                                    decodeLocalCoordinate( aCursor.U16At( offset + 10 ) ), source } );
        graphic.angle = NormalizeAngle( aCursor.U16At( offset + 12 ) );
        graphic.presentation.source = source;
        graphic.presentation.height = int64_t( aCursor.U16At( offset + 18 ) ) * 2;
        graphic.presentation.width = int64_t( aCursor.U16At( offset + 26 ) ) * 2;
        graphic.presentation.font = decodedDefinitionFont( -1, source );
        const uint16_t justification = rotatedFieldJustification(
                aCursor.U16At( offset + 14 ), graphic.angle );
        graphic.presentation.horizontalJustification = horizontalJustification( justification );
        graphic.presentation.verticalJustification = verticalJustification( justification );
        return graphic;
    };

    auto claimText = [&]( size_t record, const SOURCE_PROVENANCE& source )
    {
        if( record >= usedText.size() || usedText[record] )
            throwDecodeError( source, wxS( "v9 text record has duplicate/missing owner" ) );

        usedText[record] = true;
    };

    // Each class-zero page graphics group can own a circular predecessor
    // chain. SIZEB is a genuine worksheet: 34 paths + 35 text labels.
    for( size_t record = 0; record < pools[2].count; ++record )
    {
        const size_t offset = defBase + record * SYMBOL_RECORD_BYTES;

        if( aCursor.U8At( offset + 0x29 ) != 0 )
            continue;

        const uint16_t count = aCursor.U16At( offset + 64 );
        const uint16_t last = aCursor.U16At( offset + 66 );

        if( count == 0 )
            continue;

        SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version,
                wxS( "page graphic group" ), 3, record, offset, SYMBOL_RECORD_BYTES,
                static_cast<int>( aSheetIndex ) );

        if( count > pools[0].count || last >= pools[0].count )
            throwDecodeError( source, wxS( "v9 page graphic text ownership is invalid" ) );

        std::vector<size_t> records( count );
        size_t current = last;

        for( size_t i = count; i != 0; --i )
        {
            claimText( current, source );
            records[i - 1] = current;

            if( i != 1 )
            {
                current = aCursor.U16At( textBase + current * 28 + 20 );

                if( current >= pools[0].count )
                    throwDecodeError( source, wxS( "v9 page graphic predecessor out of range" ) );
            }
        }

        SOURCE_PROVENANCE nameSource = source;
        nameSource.length = 38;
        const SOURCE_STRING name = decodeFixedString(
                aBytes, offset, 38, nameSource, aModel.diagnostics );
        const int64_t originX = decodeCoordinate( aCursor.U16At( offset + 60 ) );
        const int64_t originY = decodeCoordinate( aCursor.U16At( offset + 62 ) );

        for( size_t textRecord : records )
        {
            MODEL_GRAPHIC graphic = makeTextGraphic( textRecord, wxS( "page text" ) );
            graphic.points[0].x += originX;
            graphic.points[0].y += originY;
            graphic.properties.push_back( sourceProperty(
                    wxS( "page_graphic_group" ), name.text, source ) );
            aModel.graphics.push_back( {
                    graphic.source, { aModel.sheets[aSheetIndex].id, graphic.source },
                    std::move( graphic ) } );
            ++counts.pageTexts;
        }

        // The existing v13 worksheet classifier recognizes >=30 graphics,
        // >=30 texts, and grid/title anchors. Here preserve the same intent
        // without relying on v13's record offsets.
        size_t drawings = 0;
        size_t texts = 0;
        size_t numericMarkers = 0;
        size_t alphabeticMarkers = 0;
        bool titleAnchor = false;
        bool sheetAnchor = false;
        bool revisionAnchor = false;
        auto belongs = [&]( const MODEL_PAGE_GRAPHIC& item )
        {
            if( item.sheet.id != aModel.sheets[aSheetIndex].id )
                return false;

            return std::ranges::any_of( item.graphic.properties,
                    [&]( const SOURCE_PROPERTY& prop )
                    {
                        return prop.name.text == wxS( "page_graphic_group" )
                                && prop.value.text == name.text;
                    } );
        };

        for( const MODEL_PAGE_GRAPHIC& item : aModel.graphics )
        {
            if( !belongs( item ) )
                continue;

            if( item.graphic.kind != MODEL_GRAPHIC_KIND::TEXT )
            {
                ++drawings;
                continue;
            }

            ++texts;
            const wxString s = item.graphic.text.text.Upper();

            if( s.length() == 1 && s[0] >= '0' && s[0] <= '9' )
                ++numericMarkers;

            if( s.length() == 1 && s[0] >= 'A' && s[0] <= 'Z' )
                ++alphabeticMarkers;

            titleAnchor |= s == wxS( "TITLE" ) || s.StartsWith( wxS( "TITLE:" ) );
            sheetAnchor |= s.StartsWith( wxS( "SHEET:" ) );
            revisionAnchor |= s == wxS( "REVISION" ) || s.StartsWith( wxS( "REV:" ) )
                    || s.StartsWith( wxS( "REVISION " ) );
        }

        if( drawings >= 30 && texts >= 30 && numericMarkers >= 4
            && alphabeticMarkers >= 4 && titleAnchor && sheetAnchor && revisionAnchor )
        {
            MODEL_WORKSHEET worksheet;
            worksheet.source = source;
            worksheet.sheet = { aModel.sheets[aSheetIndex].id, source };
            worksheet.name = name;

            for( auto it = aModel.graphics.begin(); it != aModel.graphics.end(); )
            {
                if( belongs( *it ) )
                {
                    worksheet.graphics.push_back( std::move( it->graphic ) );
                    it = aModel.graphics.erase( it );
                }
                else
                {
                    ++it;
                }
            }

            aModel.worksheets.push_back( std::move( worksheet ) );
            ++counts.worksheets;
        }
    }

    // Embedded text belongs to the symbol definition, not to the sheet.
    for( MODEL_SYMBOL_DEFINITION& definition : aModel.definitions )
    {
        if( definition.source.sheet != static_cast<int>( aSheetIndex ) )
            continue;

        const size_t offset = defBase + definition.source.recordIndex * SYMBOL_RECORD_BYTES;
        const uint16_t count = aCursor.U16At( offset + 64 );
        const uint16_t last = aCursor.U16At( offset + 66 );

        if( count == 0 )
            continue;

        if( count > pools[0].count || last >= pools[0].count )
            throwDecodeError( definition.source, wxS( "v9 embedded text ownership out of range" ) );

        std::vector<size_t> records( count );
        size_t current = last;

        for( size_t i = count; i != 0; --i )
        {
            claimText( current, definition.source );
            records[i - 1] = current;

            if( i != 1 )
            {
                current = aCursor.U16At( textBase + current * 28 + 20 );

                if( current >= pools[0].count )
                    throwDecodeError( definition.source, wxS( "v9 embedded text predecessor out of range" ) );
            }
        }

        for( size_t textRecord : records )
        {
            definition.graphics.push_back( makeTextGraphic(
                    textRecord, wxS( "embedded symbol text" ) ) );
            ++counts.embeddedTexts;
        }
    }

    // The unclaimed 28-byte entries in this generation are graphical field
    // placeholders ('*') and a historical 'VALUE' field-name payload. The
    // latter is retained with source provenance rather than silently lost.
    for( size_t record = 0; record < usedText.size(); ++record )
    {
        if( usedText[record] )
            continue;

        const size_t offset = textBase + record * 28;
        const uint32_t stringOffset = aCursor.U32At( offset + 4 );
        const uint16_t stringBytes = aCursor.U16At( offset + 16 );
        SOURCE_PROVENANCE source = sourceAt( aSourceName, aModel.version,
                wxS( "legacy definition field placeholder" ), 1, record,
                offset, 28, static_cast<int>( aSheetIndex ) );

        if( stringOffset > heapSize || stringBytes == 0 || stringBytes > heapSize - stringOffset
            || aBytes[heapBase + stringOffset + stringBytes - 1] != 0 )
            throwDecodeError( source, wxS( "v9 placeholder text out of bounds" ) );

        const SOURCE_STRING value = PADS_SCH_BINARY_PARSER::DecodeString(
                { aBytes.begin() + heapBase + stringOffset,
                  aBytes.begin() + heapBase + stringOffset + stringBytes - 1 },
                DEFAULT_CODE_PAGE, source, aModel.diagnostics );

        if( value.text == wxS( "*" ) )
        {
            ++counts.placeholders;
            continue;
        }

        SOURCE_PROPERTY preserved = sourceProperty(
                wxString::Format( wxS( "v9_unresolved_definition_text_%zu" ), record ),
                value.text, source );
        preserved.disposition = PROPERTY_DISPOSITION::PRESERVED;
        aModel.settings.properties.push_back( preserved );
        aModel.diagnostics.push_back( MakePropertyDiagnostic(
                RPT_SEVERITY_WARNING, preserved,
                wxS( "unlinked v9 definition-text record retained for review" ) ) );
        ++counts.preservedTexts;
    }

    return counts;
}

// A v9 "CONN <visual choices> <physical pins>" PARTTYPE stores graphical
// choices as gates, not a real physical unit table. KiCad needs exactly one
// gate for every physical pin and a physical pin number on each unit. Derive
// the authored unit/variant mapping from the placed connectors, preserving
// their actual decal, and keep the old choices as provenance only.
static size_t materializeLegacyV9GateUnits( const std::vector<uint8_t>& aBytes,
                                              PADS_SCH_MODEL& aModel, size_t aSheetIndex )
{
    size_t created = 0;
    uint32_t gateId = static_cast<uint32_t>( aSheetIndex * 0x100000 + 0x50000 );

    // PADS_V9_PHYSICAL_CONNECTOR_VARIANTS_V16: Stage selected per-connector
    // part types; append only after iteration to avoid invalidating part refs.
    std::vector<MODEL_PART_TYPE> variantParts;
    uint32_t variantPartId = static_cast<uint32_t>( aSheetIndex * 0x100000 + 0x60000 );

    for( MODEL_PART_TYPE& part : aModel.partTypes )
    {
        if( part.source.sheet != static_cast<int>( aSheetIndex ) || part.gates.size() < 2 )
            continue;

        // The v9 PARTTYPE's final four characters hold its type ("CON", etc).
        // Multi-pin ordinary symbols and the special ground/power packages
        // are not connector unit tables and must not be rewritten here.
        const size_t partTypeOffset = part.source.absoluteOffset;

        if( aBytes[partTypeOffset + 70] != 'C' || aBytes[partTypeOffset + 71] != 'O'
            || aBytes[partTypeOffset + 72] != 'N' )
            continue;

        if( !std::ranges::all_of( part.gates,
                []( const MODEL_GATE& gate ) { return gate.pins.size() == 1; } ) )
            continue;

        std::map<uint32_t, const MODEL_PLACEMENT*> exemplar;
        std::map<wxString, std::map<uint32_t, MODEL_PLACEMENT*>> connectorGroups;
        size_t placementsOfPart = 0;

        for( MODEL_PLACEMENT& placement : aModel.placements )
        {
            if( placement.source.sheet != static_cast<int>( aSheetIndex )
                || placement.partType.id != part.id )
                continue;

            if( placement.pins.size() != 1 || !placement.gate || placement.unit == 0 )
                throwDecodeError( placement.source,
                                  wxS( "v9 connector placement does not have one physical pin" ) );

            // IMPORTANT PADS v0x0009 SEMANTICS, not a malformed design:
            // CONN PARTTYPE describes allowed graphical choices. Every
            // *physical connector instance* selects its own decal, and
            // J8-1=EXTIN versus J13-1=EXTOUT is a legitimate choice for
            // the same CON\SIP\3P unit/pin number 1.
            //
            // PADS also uses alphabetic unit suffixes (J7-A/B) in addition
            // to numeric suffixes (J13-1/2/3). Preserve both, validating
            // against the explicit physical unit ordinal from the placement.
            const wxString reference = placement.reference.text;
            const int separator = reference.Find( '-', true );

            if( separator <= 0 || static_cast<size_t>( separator + 1 ) >= reference.length() )
                throwDecodeError( placement.source,
                                  wxS( "v9 connector lacks physical unit reference suffix" ) );

            const wxString suffix = reference.Mid( separator + 1 );
            unsigned long numericUnit = 0;
            const bool numericMatch = suffix.ToULong( &numericUnit )
                                      && numericUnit == placement.unit;
            const bool alphaMatch = placement.unit <= 26
                                    && suffix == wxString::Format(
                                               wxS( "%c" ),
                                               static_cast<int>( 'A' + placement.unit - 1 ) );

            if( !numericMatch && !alphaMatch )
                throwDecodeError( placement.source,
                                  wxS( "v9 connector reference/unit suffix mismatch" ) );

            const wxString connectorRef = reference.Left( separator );

            if( connectorRef.IsEmpty()
                || !connectorGroups[connectorRef].emplace( placement.unit, &placement ).second )
                throwDecodeError( placement.source,
                                  wxS( "v9 connector has duplicate or invalid physical unit" ) );

            exemplar.try_emplace( placement.unit, &placement );
            ++placementsOfPart;
        }

        if( exemplar.empty() )
            continue;

        const uint32_t lastUnit = exemplar.rbegin()->first;

        // The full connector is present in this fixture. A partial connector
        // cannot tell us the missing physical pin numbers: reject rather than
        // assign a graphical-choice ordinal as a physical pin number.
        if( lastUnit != exemplar.size() )
            throwDecodeError( part.source, wxS( "v9 connector has missing physical units" ) );

        if( gateId > aSheetIndex * 0x100000 + 0xFFFFF - lastUnit )
            throwDecodeError( part.source, wxS( "v9 connector gate identity range exhausted" ) );

        std::vector<MODEL_GATE> physicalGates;
        physicalGates.reserve( exemplar.size() );

        for( const auto& [unit, placed] : exemplar )
        {
            const auto sourceGate = std::ranges::find( part.gates, placed->gate->id,
                                                       &MODEL_GATE::id );

            if( sourceGate == part.gates.end() )
                throwDecodeError( placed->source, wxS( "v9 selected connector decal is unresolved" ) );

            const auto definition = std::ranges::find(
                    aModel.definitions, placed->definition.id, &MODEL_SYMBOL_DEFINITION::id );

            if( definition == aModel.definitions.end() || definition->pins.size() != 1
                || placed->pins[0].id != definition->pins[0].id )
                throwDecodeError( placed->source, wxS( "v9 connector physical pin cannot resolve definition" ) );

            MODEL_GATE physical = *sourceGate;
            physical.id = GATE_ID( gateId++ );
            physical.source = placed->source;
            physical.unit = unit;
            physical.definition = { definition->id, placed->source };
            physical.alternateDefinitions.clear();
            physical.decalGroupMembers.clear();
            physical.connectorPins.clear();
            physical.pins.clear();
            physical.logicalPins.clear();
            physical.pins.push_back( { placed->pins[0].id, placed->pins[0].source } );

            MODEL_GATE_PIN logical;
            logical.source = placed->pins[0].source;
            logical.definitionPin = { placed->pins[0].id, placed->pins[0].source };
            logical.number.source = logical.source;
            logical.number.text = wxString::Format( wxS( "%u" ), unit );
            logical.name = definition->pins[0].name;
            logical.electricalType = definition->pins[0].electricalType;
            physical.logicalPins.push_back( std::move( logical ) );
            physical.properties.push_back( sourceProperty(
                    wxS( "v9_original_graphical_gate" ),
                    wxString::Format( wxS( "%u" ), sourceGate->id.Value() ), placed->source ) );
            physicalGates.push_back( std::move( physical ) );
        }

        // The canonical physical gates match the first observed decal for
        // each pin number. Different physical connectors may choose another
        // allowed graphical decal for the same unit. One KiCad library symbol
        // must contain the *whole* connector's selected body configuration:
        // mixing canonical and selected units would split J13-1/-2/-3 into
        // incompatible imported LIB_IDs.
        std::vector<uint32_t> canonical;
        canonical.reserve( physicalGates.size() );

        for( const MODEL_GATE& unitGate : physicalGates )
            canonical.push_back( unitGate.definition.id.Value() );

        // Reuse one synthetic PARTTYPE for connectors with the same complete
        // unit-to-definition mapping. Verify distinct emitted KiCad LIB_IDs
        // after import: choosing an alternate decal must remain visible.
        std::map<std::vector<uint32_t>, size_t> variantBySignature;

        for( auto& [connectorRef, units] : connectorGroups )
        {
            std::vector<uint32_t> signature = canonical;

            for( const auto& [unit, placed] : units )
            {
                if( unit == 0 || unit > signature.size() )
                    throwDecodeError( placed->source,
                                      wxS( "v9 connector unit leaves physical gate table" ) );

                signature[unit - 1] = placed->definition.id.Value();
            }

            PART_TYPE_ID selectedPartId = part.id;
            const std::vector<MODEL_GATE>* selectedGates = &physicalGates;

            if( signature != canonical )
            {
                auto existing = variantBySignature.find( signature );

                if( existing == variantBySignature.end() )
                {
                    if( variantPartId >= aSheetIndex * 0x100000 + 0x70000
                        || gateId > aSheetIndex * 0x100000 + 0x60000 - physicalGates.size() )
                        throwDecodeError( part.source,
                                          wxS( "v9 connector variant identity range exhausted" ) );

                    MODEL_PART_TYPE variant = part;
                    variant.id = PART_TYPE_ID( variantPartId++ );
                    variant.gates = physicalGates;
                    // Trace the instance-selected graphics, not an invented
                    // logical PARTTYPE or a changed physical pin number.
                    variant.properties.push_back( sourceProperty(
                            wxS( "v9_instance_selected_graphical_variant" ),
                            connectorRef, part.source ) );

                    for( MODEL_GATE& unitGate : variant.gates )
                        unitGate.id = GATE_ID( gateId++ );

                    for( const auto& [unit, placed] : units )
                    {
                        MODEL_GATE& unitGate = variant.gates[unit - 1];

                        if( unitGate.definition.id == placed->definition.id )
                            continue;

                        // The placement was already checked against the PARTTYPE
                        // choice table during decodeLegacyV9Placements(). Recheck
                        // the provenance and the exact one-pin definition here.
                        const auto sourceGate = std::ranges::find(
                                part.gates, placed->gate->id, &MODEL_GATE::id );
                        const auto definition = std::ranges::find(
                                aModel.definitions, placed->definition.id,
                                &MODEL_SYMBOL_DEFINITION::id );

                        if( sourceGate == part.gates.end()
                            || definition == aModel.definitions.end()
                            || definition->pins.size() != 1
                            || placed->pins.size() != 1
                            || placed->pins[0].id != definition->pins[0].id
                            || unitGate.logicalPins.size() != 1 )
                            throwDecodeError( placed->source,
                                              wxS( "v9 connector selected decal/pin mismatch" ) );

                        unitGate.definition = { definition->id, placed->source };
                        unitGate.pins.clear();
                        unitGate.pins.push_back( { placed->pins[0].id, placed->pins[0].source } );
                        MODEL_GATE_PIN& logical = unitGate.logicalPins.front();
                        logical.definitionPin = { placed->pins[0].id, placed->pins[0].source };
                        logical.name = definition->pins[0].name;
                        logical.electricalType = definition->pins[0].electricalType;
                        unitGate.properties.push_back( sourceProperty(
                                wxS( "v9_selected_graphical_gate" ),
                                wxString::Format( wxS( "%u" ), sourceGate->id.Value() ),
                                placed->source ) );
                    }

                    const size_t newIndex = variantParts.size();
                    created += variant.gates.size();
                    variantParts.push_back( std::move( variant ) );
                    variantBySignature.emplace( signature, newIndex );
                    existing = variantBySignature.find( signature );
                }

                selectedPartId = variantParts[existing->second].id;
                selectedGates = &variantParts[existing->second].gates;
            }

            for( const auto& [unit, placed] : units )
            {
                placed->partType = { selectedPartId, placed->source };
                placed->gate = GATE_REFERENCE{ ( *selectedGates )[unit - 1].id,
                                              placed->source };
            }
        }

        part.gates = std::move( physicalGates );
        created += part.gates.size();
        static_cast<void>( placementsOfPart );
    }

    for( MODEL_PART_TYPE& variant : variantParts )
        aModel.partTypes.push_back( std::move( variant ) );

    return created;
}
static PADS_SCH_MODEL decodeLegacyV9GraphicsAndTexts( const std::vector<uint8_t>& aBytes,
                                            const wxString& aSourceName, const PADS_SCH_SDB& aSdb )
{
    const PADS_IO::BINARY_CURSOR& cursor = aSdb.Cursor();
    PADS_SCH_MODEL                model;
    model.version = aSdb.Version();
    model.subversion = cursor.U16At( 6 );
    model.source = { aSourceName, model.version, wxS( "model" ), -1, 0, 0, aBytes.size(), -1 };
    model.settings.codePage = DEFAULT_CODE_PAGE;

    const std::vector<MODEL_FIELD> titleFields =
            decodeDesignSettings( aBytes, cursor, aSdb, aSourceName, model );
    decodeSheets( aBytes, cursor, aSdb, aSourceName, titleFields, model );

    size_t   sheetIndex = 0;
    size_t   linkedDecals = 0;
    size_t   totalPieces = 0;
    size_t   totalVertices = 0;
    size_t   totalArcs = 0;
    size_t   totalTerminals = 0;
    size_t   totalPartTypes = 0;
    size_t   totalGates = 0;
    size_t   totalGatePins = 0;
    size_t   totalSignalPins = 0;
    size_t   totalPlacements = 0;
    size_t   totalPlacedPins = 0;
    size_t   totalGroupFields = 0;
    size_t   totalUnboundCustomFields = 0;
    size_t   totalReusedGateUnits = 0;
    size_t   totalNets = 0;
    size_t   totalWires = 0;
    size_t   totalWireVertices = 0;
    size_t   totalPinEndpoints = 0;
    size_t   totalJunctions = 0;
    size_t   totalOffpages = 0;
    size_t   totalNetNames = 0;
    size_t   totalLinkedSignals = 0;
    size_t   totalStandardFields = 0;
    size_t   totalEmbeddedTexts = 0;
    size_t   totalPageTexts = 0;
    size_t   totalWorksheets = 0;
    size_t   totalPreservedTexts = 0;
    size_t   totalResolvedPinBodies = 0;
    size_t   totalPlaceholders = 0;

    for( const SCH_SDB_BLOCK& block : aSdb.Blocks() )
    {
        if( block.kind != SCH_SDB_BLOCK_KIND::SHEET )
            continue;

        const SHEET_CONTROLLERS controllers = sheetControllers( cursor, block );
        const DEFINITION_LAYOUT layout =
                definitionLayout( cursor, controllers, sheetIndex, aSourceName, model.version );
        std::vector<MODEL_SYMBOL_DEFINITION*> definitionsByRecord( controllers.pools[2].count );
        std::vector<size_t>                   pageGraphicRecords;
        decodeSymbolDefinitions( aBytes, cursor, controllers, layout, sheetIndex,
                                 aSourceName, definitionsByRecord, pageGraphicRecords, model );

        totalPieces += controllers.pools[3].count;
        totalVertices += controllers.pools[4].count;
        totalArcs += controllers.pools[5].count;

        // v9 92-byte USED_DECAL references the 80-byte definition by u16
        // at +48, not the newer u32 record. Bound and name checks are already
        // performed by validateLegacyV9Records(). Resolve to real definitions.
        for( size_t record = 0; record < controllers.pools[6].count; ++record )
        {
            const size_t decalOffset = controllers.offsets[6] + record * 92;
            const uint16_t definitionRecord = cursor.U16At( decalOffset + 48 );

            if( definitionRecord == 0xFFFF || !definitionsByRecord[definitionRecord] )
                continue;

            ++linkedDecals;
        }

        // v9 controller 1 text records are 28 bytes (v13: 32). The record
        // offsets, local coordinates, text dimensions and the two independent
        // ownership words are all shifted by four bytes, not a blind cast.
        const size_t textBase = controllers.offsets[0];
        const size_t heapBase = controllers.offsets[1];
        const uint32_t heapBytes = controllers.pools[1].usedBytes;

        for( size_t record = 0; record < controllers.pools[0].count; ++record )
        {
            const size_t offset = textBase + record * 28;

            // Equal predecessor/successor words are the free text owners;
            // definition fields, embedded texts and worksheet texts must be
            // handled separately by their owning object.
            if( cursor.U16At( offset + 20 ) != cursor.U16At( offset + 22 ) )
                continue;

            SOURCE_PROVENANCE source = sourceAt( aSourceName, model.version, wxS( "free text" ), 1,
                                                 record, offset, 28, static_cast<int>( sheetIndex ) );
            const uint32_t stringOffset = cursor.U32At( offset + 4 );
            const uint16_t stringBytes = cursor.U16At( offset + 16 );

            if( stringOffset >= heapBytes || stringBytes == 0 || stringBytes > heapBytes - stringOffset
                || aBytes[heapBase + stringOffset + stringBytes - 1] != 0 )
                throwDecodeError( source, wxS( "legacy free-text string leaves controller 2" ) );

            SOURCE_PROVENANCE stringSource = sourceAt(
                    aSourceName, model.version, wxS( "free text string" ), 2, record,
                    heapBase + stringOffset, stringBytes - 1, static_cast<int>( sheetIndex ) );
            MODEL_TEXT text;
            text.source = source;
            text.sheet = { model.sheets[sheetIndex].id, source };
            text.text = PADS_SCH_BINARY_PARSER::DecodeString(
                    { aBytes.begin() + stringSource.absoluteOffset,
                      aBytes.begin() + stringSource.absoluteOffset + stringSource.length },
                    DEFAULT_CODE_PAGE, stringSource, model.diagnostics );
            text.position = { decodeCoordinate( cursor.U16At( offset + 8 ) ),
                              decodeCoordinate( cursor.U16At( offset + 10 ) ), source };
            text.angle = NormalizeAngle( cursor.U16At( offset + 12 ) );
            text.presentation.source = source;
            text.presentation.height = int64_t( cursor.U16At( offset + 18 ) ) * 2;
            text.presentation.width = int64_t( cursor.U16At( offset + 26 ) ) * 2;
            const uint16_t justification =
                    rotatedFieldJustification( cursor.U16At( offset + 14 ), text.angle );
            text.presentation.horizontalJustification = freeTextHorizontalJustification( justification );
            text.presentation.verticalJustification = verticalJustification( justification );
            model.texts.push_back( std::move( text ) );
        }

        const LEGACY_V9_PART_COUNTS parts = decodeLegacyV9PartsAndPins(
                aBytes, cursor, controllers, layout, sheetIndex, aSourceName,
                definitionsByRecord, model );
        totalTerminals += parts.terminals;
        totalPartTypes += parts.partTypes;
        totalGates += parts.gates;
        totalGatePins += parts.gatePins;
        totalSignalPins += parts.signalPins;
        const LEGACY_V9_FINAL_COUNTS details = decodeLegacyV9FinalDetails(
                aBytes, cursor, controllers, sheetIndex, aSourceName, model );
        totalLinkedSignals += details.signalPins;
        totalStandardFields += details.standardFields;
        totalEmbeddedTexts += details.embeddedTexts;
        totalPageTexts += details.pageTexts;
        totalWorksheets += details.worksheets;
        totalPreservedTexts += details.preservedTexts;
        totalResolvedPinBodies += details.resolvedPinBodies;
        totalPlaceholders += details.placeholders;
        const LEGACY_V9_PLACEMENT_COUNTS placements = decodeLegacyV9Placements(
                aBytes, cursor, aSdb, controllers, sheetIndex, aSourceName, model );
        totalPlacements += placements.placements;
        totalPlacedPins += placements.pins;
        totalGroupFields += placements.attributeFields;
        totalUnboundCustomFields += placements.unboundCustomFields;
        totalReusedGateUnits += placements.reusedGateUnits;
        materializeLegacyV9GateUnits( aBytes, model, sheetIndex );
        const LEGACY_V9_CONNECTIVITY_COUNTS connectivity = decodeLegacyV9Connectivity(
                aBytes, cursor, aSdb, controllers, sheetIndex, aSourceName, model );
        totalNets += connectivity.nets;
        totalWires += connectivity.wires;
        totalWireVertices += connectivity.vertices;
        totalPinEndpoints += connectivity.pinEndpoints;
        totalJunctions += connectivity.junctions;
        totalOffpages += connectivity.offpages;
        totalNetNames += connectivity.netNames;
        ++sheetIndex;
    }

    // Keep the measured decoding coverage in the semantic provenance. This
    // also makes all of the stage counters visible to a future diagnostic
    // exporter instead of discarding them after the earlier read-only passes.
    wxString audit = wxString::Format(
            wxS( "sheets=%zu definitions=%zu linked_decals=%zu graphics=%zu free_texts=%zu "
                 "pieces=%zu vertices=%zu arcs=%zu" ),
            sheetIndex, model.definitions.size(), linkedDecals, model.graphics.size(),
            model.texts.size(), totalPieces, totalVertices, totalArcs );
    audit += wxString::Format(
            wxS( " terminals=%zu parttypes=%zu gates=%zu gate_pins=%zu signal_records=%zu "
                 "placements=%zu placed_pins=%zu attributes=%zu custom_unbound=%zu reused_units=%zu" ),
            totalTerminals, totalPartTypes, totalGates, totalGatePins, totalSignalPins,
            totalPlacements, totalPlacedPins, totalGroupFields, totalUnboundCustomFields,
            totalReusedGateUnits );
    audit += wxString::Format(
            wxS( " nets=%zu wires=%zu wire_vertices=%zu pin_endpoints=%zu junctions=%zu "
                 "offpages=%zu netnames=%zu" ),
            totalNets, totalWires, totalWireVertices, totalPinEndpoints,
            totalJunctions, totalOffpages, totalNetNames );
    audit += wxString::Format(
            wxS( " sigpins=%zu default_fields=%zu embedded_texts=%zu page_texts=%zu "
                 "worksheets=%zu placeholders=%zu preserved_texts=%zu pin_bodies=%zu" ),
            totalLinkedSignals, totalStandardFields, totalEmbeddedTexts, totalPageTexts,
            totalWorksheets, totalPlaceholders, totalPreservedTexts, totalResolvedPinBodies );
    SOURCE_PROPERTY sourceAudit = sourceProperty( wxS( "v9_semantic_import_coverage" ), audit, model.source );
    sourceAudit.disposition = PROPERTY_DISPOSITION::PRESERVED;
    model.settings.properties.push_back( std::move( sourceAudit ) );

    // The v9 semantic model now contains real definitions, pin bodies, parts,
    // placements, nets, labels and the authored worksheet. Do not bypass
    // the model's own structural validation before the KiCad builder sees it.
    decodeOleImages( aBytes, aSdb, aSourceName, model );
    inheritDefaultSystemFonts( model );
    assignFieldIds( model );
    model.ValidateOrThrow();
    return model;
}

PADS_SCH_MODEL PADS_SCH_BINARY_PARSER::Parse( const std::vector<uint8_t>& aBytes, const wxString& aSourceName ) const
{
    PADS_SCH_SDB sdb;
    sdb.Load( aBytes );

    // Decode the legacy sheet, symbol geometry and free texts with their own
    // v9 layouts. This path intentionally rejects an incomplete schematic.
    if( sdb.Version() == 0x0009 )
    {
        validateLegacyV9Records( sdb, aBytes );
        return decodeLegacyV9GraphicsAndTexts( aBytes, aSourceName, sdb );
    }

    const PADS_IO::BINARY_CURSOR& cursor = sdb.Cursor();
    PADS_SCH_MODEL                model;
    model.version = sdb.Version();
    model.subversion = cursor.U16At( 6 );
    model.source = { aSourceName, model.version, wxS( "model" ), -1, 0, 0, aBytes.size(), -1 };
    model.settings.codePage = DEFAULT_CODE_PAGE;

    const std::vector<MODEL_FIELD> titleFields = decodeDesignSettings( aBytes, cursor, sdb, aSourceName, model );

    decodeSheets( aBytes, cursor, sdb, aSourceName, titleFields, model );
    decodeSheetBlocks( aBytes, cursor, sdb, aSourceName, model );
    decodeOleImages( aBytes, sdb, aSourceName, model );
    inheritDefaultSystemFonts( model );
    assignFieldIds( model );
    model.ValidateOrThrow();
    return model;
}


SOURCE_STRING PADS_SCH_BINARY_PARSER::DecodeString( const std::vector<uint8_t>& aBytes, uint32_t aCodePage,
                                                    const SOURCE_PROVENANCE&        aSource,
                                                    std::vector<PARSER_DIAGNOSTIC>& aDiagnostics,
                                                    const wxString&                 aRecordedCodePageName )
{
    SOURCE_STRING result{ aBytes, {}, STRING_ENCODING_STATUS::UTF8, aSource };
    result.codePage = aCodePage;
    const wxString defaultName = aCodePage == 65001  ? wxString( wxS( "UTF-8" ) )
                                 : aCodePage == 1252 ? wxString( wxS( "windows-1252" ) )
                                                     : wxString::Format( wxS( "unknown-%u" ), aCodePage );
    result.codePageName = aRecordedCodePageName.empty() ? defaultName : aRecordedCodePageName;

    if( aCodePage == 1252 )
        result.encoding = STRING_ENCODING_STATUS::CODE_PAGE;
    else if( aCodePage != 65001 )
        result.encoding = STRING_ENCODING_STATUS::UNKNOWN_CODE_PAGE;

    if( aCodePage != 65001 && aCodePage != 1252 )
    {
        aDiagnostics.push_back( { RPT_SEVERITY_WARNING, aSource,
                                  wxString::Format( wxS( "unknown code page %u; bytes preserved and "
                                                         "non-ASCII bytes decoded as U+FFFD" ),
                                                    aCodePage ) } );
    }

    if( aBytes.empty() )
        return result;

    if( aCodePage == 1252 )
    {
        result.text = decodeWindows1252( aBytes );
        return result;
    }

    if( aCodePage != 65001 )
    {
        result.text = decodeUnknownCodePage( aBytes );
        return result;
    }

    bool invalid = false;
    result.text = decodeUtf8( aBytes, invalid );

    if( invalid )
    {
        result.encoding = STRING_ENCODING_STATUS::INVALID_BYTES;
        aDiagnostics.push_back( { RPT_SEVERITY_WARNING, aSource,
                                  wxS( "invalid UTF-8 bytes replaced with U+FFFD; original bytes preserved" ) } );
    }

    return result;
}


void PADS_SCH_BINARY_PARSER::RecordUnknownEnum( const wxString& aEnumName, uint32_t aValue,
                                                const SOURCE_PROVENANCE&        aSource,
                                                std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
{
    aDiagnostics.push_back( { RPT_SEVERITY_WARNING, aSource,
                              wxString::Format( wxS( "unknown %s %u preserved" ), aEnumName, aValue ) } );
}

} // namespace PADS_SCH_BINARY
