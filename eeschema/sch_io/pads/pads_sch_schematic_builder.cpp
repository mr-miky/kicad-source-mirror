/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2025 KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <sch_io/pads/pads_sch_schematic_builder.h>

#include <sch_line.h>
#include <sch_junction.h>
#include <sch_label.h>
#include <sch_screen.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <sch_sheet_pin.h>
#include <sch_symbol.h>
#include <schematic.h>
#include <wildcards_and_files_ext.h>
#include <layer_ids.h>
#include <template_fieldnames.h>
#include <title_block.h>
#include <io/pads/pads_attribute_mapper.h>
#include <io/pads/pads_common.h>

#include <advanced_config.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <set>

namespace PADS_SCH
{

PADS_SCH_SCHEMATIC_BUILDER::PADS_SCH_SCHEMATIC_BUILDER( const PARAMETERS& aParams,
                                                        SCHEMATIC*        aSchematic ) :
    m_params( aParams ),
    m_schematic( aSchematic ),
    m_pageHeightIU( schIUScale.MilsToIU( aParams.sheet_size.height ) )
{
}


PADS_SCH_SCHEMATIC_BUILDER::~PADS_SCH_SCHEMATIC_BUILDER()
{
}


int PADS_SCH_SCHEMATIC_BUILDER::toKiCadUnits( double aPadsValue ) const
{
    // PADS Logic ASCII schematics always store geometry in mils. The UNITS field selects only
    // the design-rules unit and must not scale the schematic coordinates.
    return schIUScale.MilsToIU( aPadsValue );
}


int PADS_SCH_SCHEMATIC_BUILDER::toKiCadY( double aPadsY ) const
{
    return m_pageHeightIU - toKiCadUnits( aPadsY );
}


wxString PADS_SCH_SCHEMATIC_BUILDER::convertNetName( const std::string& aName ) const
{
    return PADS_COMMON::ConvertInvertedNetName( aName );
}


int PADS_SCH_SCHEMATIC_BUILDER::CreateWires( const std::vector<SCH_SIGNAL>& aSignals,
                                             SCH_SCREEN*                    aScreen )
{
    int wireCount = 0;

    for( const SCH_SIGNAL& signal : aSignals )
    {
        for( const WIRE_SEGMENT& wire : signal.wires )
        {
            SCH_LINE* schLine = CreateWire( wire );

            if( schLine )
            {
                schLine->SetFlags( IS_NEW );
                aScreen->Append( schLine );
                wireCount++;
            }
        }
    }

    return wireCount;
}


SCH_LINE* PADS_SCH_SCHEMATIC_BUILDER::CreateWire( const WIRE_SEGMENT& aWire )
{
    VECTOR2I start( toKiCadUnits( aWire.start.x ), toKiCadY( aWire.start.y ) );
    VECTOR2I end( toKiCadUnits( aWire.end.x ), toKiCadY( aWire.end.y ) );

    SCH_LINE* line = new SCH_LINE( start, SCH_LAYER_ID::LAYER_WIRE );
    line->SetEndPoint( end );

    return line;
}


int PADS_SCH_SCHEMATIC_BUILDER::CreateJunctions( const std::vector<SCH_SIGNAL>& aSignals,
                                                 SCH_SCREEN*                    aScreen )
{
    std::vector<VECTOR2I> junctionPoints = findJunctionPoints( aSignals );

    for( const VECTOR2I& pt : junctionPoints )
    {
        SCH_JUNCTION* junction = new SCH_JUNCTION( pt );
        junction->SetFlags( IS_NEW );
        aScreen->Append( junction );
    }

    return static_cast<int>( junctionPoints.size() );
}


std::vector<VECTOR2I> PADS_SCH_SCHEMATIC_BUILDER::findJunctionPoints( const std::vector<SCH_SIGNAL>& aSignals )
{
    std::vector<VECTOR2I> junctions;

    for( const SCH_SIGNAL& signal : aSignals )
    {
        // Count how many wire endpoints connect at each point
        std::map<std::pair<int, int>, int> pointCount;

        for( const WIRE_SEGMENT& wire : signal.wires )
        {
            VECTOR2I start( toKiCadUnits( wire.start.x ), toKiCadY( wire.start.y ) );
            VECTOR2I end( toKiCadUnits( wire.end.x ), toKiCadY( wire.end.y ) );

            pointCount[{ start.x, start.y }]++;
            pointCount[{ end.x, end.y }]++;
        }

        // Junction needed where 3+ wire endpoints meet
        for( const auto& [coords, count] : pointCount )
        {
            if( count >= 3 )
                junctions.emplace_back( coords.first, coords.second );
        }
    }

    return junctions;
}


int PADS_SCH_SCHEMATIC_BUILDER::CreateNetLabels( const std::vector<SCH_SIGNAL>& aSignals, SCH_SCREEN* aScreen,
                                                 const std::set<std::string>& aSignalOpcIds,
                                                 const std::map<std::string, OFF_PAGE_CONNECTOR>& aOffPageConnectors,
                                                 const std::set<std::string>& aSkipSignals,
                                                 const std::map<std::string, NETNAME_LABEL>& aNetNameLabels )
{
    int labelCount = 0;

    for( const SCH_SIGNAL& signal : aSignals )
    {
        if( signal.name.empty() || signal.name[0] == '$' )
            continue;

        if( aSkipSignals.count( signal.name ) )
            continue;

        if( signal.wires.empty() )
            continue;

        // Collect label placements from OPC wire endpoints. Each OPC produces one label.
        // The anchor ref (@@@O..) is retained so the transform authored on the
        // *OFFPAGE REFS* object can be looked up. Wire direction is only a fallback.
        struct PLACEMENT
        {
            VECTOR2I    labelPos;
            VECTOR2I    adjPos;
            std::string anchorRef;
        };

        std::vector<PLACEMENT> opcPlacements;

        for( const WIRE_SEGMENT& wire : signal.wires )
        {
            if( wire.vertices.size() < 2 )
                continue;

            // endpoint_a references first vertex, endpoint_b references last vertex
            if( !wire.endpoint_a.empty() && wire.endpoint_a.substr( 0, 3 ) == "@@@"
                && aSignalOpcIds.count( wire.endpoint_a ) )
            {
                VECTOR2I labelPos( toKiCadUnits( wire.vertices.front().x ),
                                   toKiCadY( wire.vertices.front().y ) );
                VECTOR2I adjPos( toKiCadUnits( wire.vertices[1].x ),
                                 toKiCadY( wire.vertices[1].y ) );
                opcPlacements.push_back( { labelPos, adjPos, wire.endpoint_a } );
            }

            if( !wire.endpoint_b.empty() && wire.endpoint_b.substr( 0, 3 ) == "@@@"
                && aSignalOpcIds.count( wire.endpoint_b ) )
            {
                size_t last = wire.vertices.size() - 1;
                VECTOR2I labelPos( toKiCadUnits( wire.vertices[last].x ),
                                   toKiCadY( wire.vertices[last].y ) );
                VECTOR2I adjPos( toKiCadUnits( wire.vertices[last - 1].x ),
                                 toKiCadY( wire.vertices[last - 1].y ) );
                opcPlacements.push_back( { labelPos, adjPos, wire.endpoint_b } );
            }
        }

        for( const PLACEMENT& placement : opcPlacements )
        {
            // Match the binary importer: PADS *NETNAMES* stores the visible off-page
            // text/body displacement from the electrical anchor and therefore gives the
            // reading direction.  Use adjacent wire direction only when that offset is zero.
            SPIN_STYLE orient = computeLabelOrientation( placement.labelPos, placement.adjPos );
            auto       opcIt = aOffPageConnectors.find( placement.anchorRef );
            auto       presentationIt = aNetNameLabels.find( placement.anchorRef );

            if( opcIt != aOffPageConnectors.end() && presentationIt != aNetNameLabels.end() )
                orient = SpinFromNetNameOffset( presentationIt->second, orient );

            SCH_GLOBALLABEL* label = CreateNetLabel( signal, placement.labelPos, orient );

            if( label )
            {
                // PADS $OSR_SYMS has exactly two off-page variants:
                //   flags2 0 = REFIN  -> input
                //   flags2 1 = REFOUT -> output
                // There is no PADS bidirectional off-page variant. Unexpected
                // values remain unspecified rather than inventing BIDI semantics.
                if( opcIt != aOffPageConnectors.end() && opcIt->second.symbol_lib == "$OSR_SYMS" )
                {
                    switch( opcIt->second.flags2 )
                    {
                    case 0: label->SetShape( LABEL_FLAG_SHAPE::L_INPUT ); break;
                    case 1: label->SetShape( LABEL_FLAG_SHAPE::L_OUTPUT ); break;
                    default: label->SetShape( LABEL_FLAG_SHAPE::L_UNSPECIFIED ); break;
                    }
                }

                // Apply the presentation authored by PADS *NETNAMES* to the
                // off-page/global label.
                if( presentationIt != aNetNameLabels.end() )
                {
                    const NETNAME_LABEL& nn = presentationIt->second;

                    if( nn.height > 0 )
                    {
                        int nominalSize = schIUScale.MilsToIU( nn.height );
                        int textHeight = KiROUND(
                                nominalSize * ADVANCED_CFG::GetCfg().m_PadsSchTextHeightScale );
                        int textWidth = KiROUND(
                                nominalSize * ADVANCED_CFG::GetCfg().m_PadsSchTextWidthScale );
                        label->SetTextSize( VECTOR2I( textWidth, textHeight ) );
                    }

                    if( nn.width_pct > 0 )
                        label->SetTextThickness( schIUScale.MilsToIU( nn.width_pct ) );

                    label->SetBold( nn.bold );
                    label->SetItalic( nn.italic );

                    if( !nn.font_face.empty() )
                        label->SetFontProp( wxString::FromUTF8( nn.font_face ) );
                }

                label->SetFlags( IS_NEW );
                aScreen->Append( label );
                labelCount++;
            }
        }
    }

    return labelCount;
}


SPIN_STYLE PADS_SCH_SCHEMATIC_BUILDER::SpinFromNetNameOffset(
        const NETNAME_LABEL& aLabel, SPIN_STYLE aFallback )
{
    if( aLabel.x_offset == 0 && aLabel.y_offset == 0 )
        return aFallback;

    if( std::abs( aLabel.x_offset ) >= std::abs( aLabel.y_offset ) )
        return aLabel.x_offset >= 0 ? SPIN_STYLE::RIGHT : SPIN_STYLE::LEFT;

    // PADS page Y increases upwards.  This is the authored body/text displacement,
    // so positive Y means the label body is above the electrical anchor.
    return aLabel.y_offset >= 0 ? SPIN_STYLE::UP : SPIN_STYLE::BOTTOM;
}


SPIN_STYLE PADS_SCH_SCHEMATIC_BUILDER::computeLabelOrientation( const VECTOR2I& aLabelPos,
                                                                const VECTOR2I& aAdjacentPos )
{
    // The wire goes from aLabelPos toward aAdjacentPos. The label text extends
    // in the opposite direction so it doesn't overlap the wire.
    int dx = aAdjacentPos.x - aLabelPos.x;
    int dy = aAdjacentPos.y - aLabelPos.y;

    if( std::abs( dx ) >= std::abs( dy ) )
    {
        return ( dx > 0 ) ? SPIN_STYLE::LEFT : SPIN_STYLE::RIGHT;
    }
    else
    {
        return ( dy > 0 ) ? SPIN_STYLE::UP : SPIN_STYLE::BOTTOM;
    }
}


SCH_GLOBALLABEL* PADS_SCH_SCHEMATIC_BUILDER::CreateNetLabel( const SCH_SIGNAL& aSignal,
                                                             const VECTOR2I&   aPosition,
                                                             SPIN_STYLE        aOrientation )
{
    wxString labelName = convertNetName( aSignal.name );

    SCH_GLOBALLABEL* label = new SCH_GLOBALLABEL( aPosition, labelName );
    label->SetShape( LABEL_FLAG_SHAPE::L_UNSPECIFIED );
    label->SetSpinStyle( aOrientation );

    int labelSize = schIUScale.MilsToIU( 50 );
    label->SetTextSize( VECTOR2I( labelSize, labelSize ) );

    return label;
}


VECTOR2I PADS_SCH_SCHEMATIC_BUILDER::chooseLabelPosition( const SCH_SIGNAL& aSignal )
{
    if( aSignal.wires.empty() )
        return VECTOR2I( 0, 0 );

    // Count how many wire segments share each endpoint. An endpoint referenced
    // only once is a dangling wire end -- the correct place for a global label.
    // Only first and last vertices are true endpoints; interior ones are bends.
    std::map<std::pair<int, int>, int> endpointCount;

    for( const WIRE_SEGMENT& wire : aSignal.wires )
    {
        if( wire.vertices.size() < 2 )
            continue;

        POINT first = wire.vertices.front();
        POINT last = wire.vertices.back();

        endpointCount[{ static_cast<int>( first.x * 1000 ), static_cast<int>( first.y * 1000 ) }]++;
        endpointCount[{ static_cast<int>( last.x * 1000 ), static_cast<int>( last.y * 1000 ) }]++;
    }

    // Also count pin connection positions so we can avoid placing on a pin
    std::set<std::pair<int, int>> pinEndpoints;

    for( const WIRE_SEGMENT& wire : aSignal.wires )
    {
        if( wire.vertices.size() < 2 )
            continue;

        // endpoint_a/endpoint_b hold pin references (e.g. "R1.1"); if non-empty,
        // that endpoint connects to a component pin. OPC references (starting
        // with "@@@") are off-page connectors, not physical pins.
        if( !wire.endpoint_a.empty() && wire.endpoint_a.substr( 0, 3 ) != "@@@" )
        {
            POINT pt = wire.vertices.front();
            pinEndpoints.insert( { static_cast<int>( pt.x * 1000 ), static_cast<int>( pt.y * 1000 ) } );
        }

        if( !wire.endpoint_b.empty() && wire.endpoint_b.substr( 0, 3 ) != "@@@" )
        {
            POINT pt = wire.vertices.back();
            pinEndpoints.insert( { static_cast<int>( pt.x * 1000 ), static_cast<int>( pt.y * 1000 ) } );
        }
    }

    // Prefer a dangling endpoint that is NOT at a pin
    for( const WIRE_SEGMENT& wire : aSignal.wires )
    {
        if( wire.vertices.size() < 2 )
            continue;

        for( const POINT* vtx : { &wire.vertices.front(), &wire.vertices.back() } )
        {
            auto key = std::make_pair( static_cast<int>( vtx->x * 1000 ), static_cast<int>( vtx->y * 1000 ) );

            if( endpointCount[key] == 1 && pinEndpoints.count( key ) == 0 )
                return VECTOR2I( toKiCadUnits( vtx->x ), toKiCadY( vtx->y ) );
        }
    }

    // Fallback: any dangling endpoint (even if at a pin)
    for( const WIRE_SEGMENT& wire : aSignal.wires )
    {
        if( wire.vertices.size() < 2 )
            continue;

        for( const POINT* vtx : { &wire.vertices.front(), &wire.vertices.back() } )
        {
            auto key = std::make_pair( static_cast<int>( vtx->x * 1000 ), static_cast<int>( vtx->y * 1000 ) );

            if( endpointCount[key] == 1 )
                return VECTOR2I( toKiCadUnits( vtx->x ), toKiCadY( vtx->y ) );
        }
    }

    // Last resort: first endpoint of the first wire that has vertices
    for( const WIRE_SEGMENT& wire : aSignal.wires )
    {
        if( !wire.vertices.empty() )
        {
            const POINT& vtx = wire.vertices[0];
            return VECTOR2I( toKiCadUnits( vtx.x ), toKiCadY( vtx.y ) );
        }
    }

    return VECTOR2I( 0, 0 );
}


int PADS_SCH_SCHEMATIC_BUILDER::CreateBusWires( const std::vector<SCH_SIGNAL>& aSignals,
                                                SCH_SCREEN*                    aScreen )
{
    int busCount = 0;

    for( const SCH_SIGNAL& signal : aSignals )
    {
        if( !IsBusSignal( signal.name ) )
            continue;

        for( const WIRE_SEGMENT& wire : signal.wires )
        {
            SCH_LINE* busLine = CreateBusWire( wire );

            if( busLine )
            {
                busLine->SetFlags( IS_NEW );
                aScreen->Append( busLine );
                busCount++;
            }
        }
    }

    return busCount;
}


SCH_LINE* PADS_SCH_SCHEMATIC_BUILDER::CreateBusWire( const WIRE_SEGMENT& aWire )
{
    VECTOR2I start( toKiCadUnits( aWire.start.x ), toKiCadY( aWire.start.y ) );
    VECTOR2I end( toKiCadUnits( aWire.end.x ), toKiCadY( aWire.end.y ) );

    SCH_LINE* line = new SCH_LINE( start, SCH_LAYER_ID::LAYER_BUS );
    line->SetEndPoint( end );

    return line;
}


bool PADS_SCH_SCHEMATIC_BUILDER::IsBusSignal( const std::string& aName )
{
    if( aName.empty() )
        return false;

    // Check for bus naming patterns commonly used in PADS
    // Pattern 1: NAME[n:m] or NAME[n..m] - range notation
    size_t bracketPos = aName.find( '[' );

    if( bracketPos != std::string::npos )
    {
        size_t closeBracket = aName.find( ']', bracketPos );

        if( closeBracket != std::string::npos )
        {
            std::string range = aName.substr( bracketPos + 1, closeBracket - bracketPos - 1 );

            if( range.find( ':' ) != std::string::npos || range.find( ".." ) != std::string::npos )
                return true;
        }
    }

    // Pattern 2: NAME<n:m> or NAME<n..m>
    size_t anglePos = aName.find( '<' );

    if( anglePos != std::string::npos )
    {
        size_t closeAngle = aName.find( '>', anglePos );

        if( closeAngle != std::string::npos )
        {
            std::string range = aName.substr( anglePos + 1, closeAngle - anglePos - 1 );

            if( range.find( ':' ) != std::string::npos || range.find( ".." ) != std::string::npos )
                return true;
        }
    }

    return false;
}


namespace
{
struct PADS_FIELD_FONT
{
    wxString face;
    bool     bold = false;
    bool     italic = false;
};


static PADS_FIELD_FONT resolvePadsAsciiFieldFont( const PART_ATTRIBUTE& aAttr,
                                                   const PARAMETERS&     aParams )
{
    std::string fontInfo = aAttr.font_name;

    // In System font mode PADS' "Default Font" means the document default,
    // e.g. DEFAULT FONT "Regular Verdana".
    if( fontInfo == "Default Font" )
    {
        std::string mode = aParams.font_mode;
        std::transform( mode.begin(), mode.end(), mode.begin(),
                        []( unsigned char c ) { return static_cast<char>( std::tolower( c ) ); } );

        if( mode == "system" && !aParams.default_font.empty() )
            fontInfo = aParams.default_font;
    }

    PADS_FIELD_FONT result;

    if( fontInfo.empty() || fontInfo == "Default Font" )
        return result;

    size_t      split = fontInfo.find( ' ' );
    std::string first = split == std::string::npos ? fontInfo : fontInfo.substr( 0, split );
    bool        styleToken = first.find( "Regular" ) != std::string::npos
                          || first.find( "Bold" ) != std::string::npos
                          || first.find( "Italic" ) != std::string::npos
                          || first.find( "Underline" ) != std::string::npos;

    if( styleToken )
    {
        result.bold = first.find( "Bold" ) != std::string::npos;
        result.italic = first.find( "Italic" ) != std::string::npos;

        if( split != std::string::npos )
        {
            size_t faceStart = fontInfo.find_first_not_of( ' ', split );

            if( faceStart != std::string::npos )
                result.face = wxString::FromUTF8( fontInfo.substr( faceStart ) );
        }
    }
    else
    {
        // Be permissive with files that store only the face name.
        result.face = wxString::FromUTF8( fontInfo );
    }

    return result;
}


static void applyPadsAsciiFieldFont( SCH_FIELD& aField, const PART_ATTRIBUTE& aAttr,
                                     const PARAMETERS& aParams )
{
    const PADS_FIELD_FONT font = resolvePadsAsciiFieldFont( aAttr, aParams );

    if( font.face.IsEmpty() )
        return;

    aField.SetFontProp( font.face );
    aField.SetBold( font.bold );
    aField.SetItalic( font.italic );

    if( aAttr.height > 0 )
    {
        // Match the binary importer: calibrate PADS system/TrueType field height
        // to KiCad's outline-font visual size.  A PADS Verdana field authored at
        // 97 mil matches visually at about 1.45 mm in KiCad:
        //   1.45 mm / ( 97 mil * 0.0254 mm/mil ) = 0.5885217956.
        // Stroke-font fields keep the normal ASCII PADS 0.50/0.46 path above.
        constexpr double PADS_SYSTEM_FIELD_HEIGHT_CORRECTION = 0.5885217956;
        int visualSize = KiROUND( schIUScale.MilsToIU( aAttr.height )
                                  * PADS_SYSTEM_FIELD_HEIGHT_CORRECTION );
        aField.SetTextSize( VECTOR2I( visualSize, visualSize ) );
    }
}
} // namespace


void PADS_SCH_SCHEMATIC_BUILDER::ApplyPartAttributes( SCH_SYMBOL*           aSymbol,
                                                      const PART_PLACEMENT& aPlacement )
{
    if( !aSymbol || !m_schematic )
        return;

    // Set reference designator, stripping any gate suffix (e.g., "U17-A" → "U17")
    if( !aPlacement.reference.empty() )
    {
        std::string ref = aPlacement.reference;
        size_t sepPos = ref.rfind( '-' );

        if( sepPos == std::string::npos )
            sepPos = ref.rfind( '.' );

        if( sepPos != std::string::npos
                && sepPos + 1 < ref.size()
                && std::isalpha( static_cast<unsigned char>( ref[sepPos + 1] ) ) )
        {
            ref = ref.substr( 0, sepPos );
        }

        aSymbol->SetRef( &m_schematic->CurrentSheet(), wxString::FromUTF8( ref ) );
    }

    PADS_ATTRIBUTE_MAPPER mapper;

    // PADS PART-TYPE and Value are distinct fields.  The displayed Value field
    // and the per-instance Value attribute are also distinct: the latter can exist
    // without any presentation record.  In that case preserve the text, but keep the
    // KiCad Value hidden just like PADS/binary import.  When both exist, the instance
    // attribute is authoritative and the displayed record supplies presentation only.
    if( aPlacement.power_net_name.empty() )
    {
        bool        hasValuePresentation = false;
        std::string valueText;

        aSymbol->SetValueFieldText( wxEmptyString );

        for( const PART_ATTRIBUTE& attr : aPlacement.attributes )
        {
            if( mapper.IsValueField( attr.name ) )
            {
                hasValuePresentation = true;

                if( !attr.value.empty() )
                    valueText = attr.value;

                break;
            }
        }

        // Per-instance attributes override the displayed/default value text.  This also
        // covers values that have no displayed field record at all (for example U1/U2 in
        // the real multi-sheet fixture).
        for( const auto& [name, value] : aPlacement.attr_overrides )
        {
            if( mapper.IsValueField( name ) && !value.empty() )
            {
                valueText = value;
                break;
            }
        }

        if( !valueText.empty() )
            aSymbol->SetValueFieldText( wxString::FromUTF8( valueText ) );

        if( !hasValuePresentation )
        {
            if( SCH_FIELD* valueField = aSymbol->GetField( FIELD_T::VALUE ) )
                valueField->SetVisible( false );
        }

        if( !aPlacement.part_type.empty() )
        {
            wxString   fieldName = wxString::FromUTF8( PADS_ATTRIBUTE_MAPPER::FIELD_PART_TYPE );
            SCH_FIELD* field = aSymbol->GetField( fieldName );

            if( !field )
            {
                SCH_FIELD newField( aSymbol, FIELD_T::USER, fieldName );
                newField.SetVisible( false );
                aSymbol->GetFields().push_back( newField );
                field = aSymbol->GetField( fieldName );
            }

            if( field )
                field->SetText( wxString::FromUTF8( aPlacement.part_type ) );
        }
    }

    // Look for PCB DECAL attribute to set footprint
    bool hasFootprint = false;

    for( const PART_ATTRIBUTE& attr : aPlacement.attributes )
    {
        if( mapper.IsFootprintField( attr.name ) && !attr.value.empty() )
        {
            aSymbol->SetFootprintFieldText( wxString::FromUTF8( attr.value ) );
            hasFootprint = true;
            break;
        }
    }

    // PCB DECAL is commonly present only in the component attribute values,
    // without a displayed placement-field record.  In that case it lives in
    // attr_overrides rather than attributes.
    if( !hasFootprint )
    {
        for( const auto& [name, value] : aPlacement.attr_overrides )
        {
            if( mapper.IsFootprintField( name ) && !value.empty() )
            {
                aSymbol->SetFootprintFieldText( wxString::FromUTF8( value ) );

                if( SCH_FIELD* footprint = aSymbol->GetField( FIELD_T::FOOTPRINT ) )
                    footprint->SetVisible( false );

                break;
            }
        }
    }

    // Apply visibility and position settings
    ApplyFieldSettings( aSymbol, aPlacement );
}


void PADS_SCH_SCHEMATIC_BUILDER::ApplyFieldSettings( SCH_SYMBOL*           aSymbol,
                                                     const PART_PLACEMENT& aPlacement )
{
    if( !aSymbol )
        return;

    PADS_ATTRIBUTE_MAPPER mapper;

    for( const PART_ATTRIBUTE& attr : aPlacement.attributes )
    {
        SCH_FIELD* field = nullptr;
        bool isRefOrValue = false;

        if( mapper.IsReferenceField( attr.name ) )
        {
            field = aSymbol->GetField( FIELD_T::REFERENCE );
            isRefOrValue = true;
        }
        else if( mapper.IsPartTypeField( attr.name ) )
        {
            field = aSymbol->GetField( wxString::FromUTF8( PADS_ATTRIBUTE_MAPPER::FIELD_PART_TYPE ) );
            isRefOrValue = true;
        }
        else if( mapper.IsValueField( attr.name ) )
        {
            field = aSymbol->GetField( FIELD_T::VALUE );
            isRefOrValue = true;
        }
        else if( mapper.IsFootprintField( attr.name ) )
        {
            field = aSymbol->GetField( FIELD_T::FOOTPRINT );
        }

        if( field )
        {
            bool isRef = mapper.IsReferenceField( attr.name );
            bool isPartType = mapper.IsPartTypeField( attr.name );

            if( isRef )
                field->SetVisible( attr.visible && ( aPlacement.item_visibility_flags & 0x01 ) == 0 );
            else if( isPartType )
                field->SetVisible( attr.visible && ( aPlacement.item_visibility_flags & 0x02 ) == 0 );
            else
                field->SetVisible( isRefOrValue ? attr.visible : false );

            // Placement-field coordinates are local to the PADS CAEDECAL.  Match the
            // binary importer and transform the authored local offset through the placed
            // KiCad symbol, including both rotation and mirror state.
            VECTOR2I localOffset( toKiCadUnits( attr.position.x ),
                                  -toKiCadUnits( attr.position.y ) );
            VECTOR2I fieldOffset = aSymbol->GetTransform().TransformCoordinate( localOffset );

            field->SetPosition( aSymbol->GetPosition() + fieldOffset );

            if( attr.rotation != 0.0 )
                field->SetTextAngleDegrees( attr.rotation );

            if( attr.height > 0 )
            {
                int scaledH = KiROUND( schIUScale.MilsToIU( attr.height )
                                        * ADVANCED_CFG::GetCfg().m_PadsSchTextHeightScale );
                int scaledW = KiROUND( schIUScale.MilsToIU( attr.height )
                                        * ADVANCED_CFG::GetCfg().m_PadsSchTextWidthScale );
                field->SetTextSize( VECTOR2I( scaledW, scaledH ) );
            }
            else
            {
                int fieldTextSize = schIUScale.MilsToIU( 50 );
                field->SetTextSize( VECTOR2I( fieldTextSize, fieldTextSize ) );
            }

            if( attr.width > 0 )
                field->SetTextThickness( schIUScale.MilsToIU( attr.width ) );

            // Map the PADS justification code so reference and value fields keep the
            // alignment authored in PADS instead of forcing center.
            GR_TEXT_H_ALIGN_T hJustify = GR_TEXT_H_ALIGN_LEFT;
            GR_TEXT_V_ALIGN_T vJustify = GR_TEXT_V_ALIGN_BOTTOM;
            PADS_COMMON::DecodeNineWayJustification( attr.justification, hJustify, vJustify,
                                                        attr.rotation );

            field->SetHorizJustify( hJustify );
            field->SetVertJustify( vJustify );

            applyPadsAsciiFieldFont( *field, attr, m_params );
        }
    }
}


int PADS_SCH_SCHEMATIC_BUILDER::CreateCustomFields( SCH_SYMBOL*           aSymbol,
                                                    const PART_PLACEMENT& aPlacement )
{
    if( !aSymbol )
        return 0;

    int fieldsCreated = 0;
    PADS_ATTRIBUTE_MAPPER mapper;

    auto isSignalPinOverride = []( const std::string& aName )
    {
        constexpr char prefix[] = "SIGPIN";
        constexpr size_t prefixLen = sizeof( prefix ) - 1;

        if( aName.size() <= prefixLen || aName.compare( 0, prefixLen, prefix ) != 0 )
            return false;

        for( size_t i = prefixLen; i < aName.size(); ++i )
        {
            if( !std::isdigit( static_cast<unsigned char>( aName[i] ) ) )
                return false;
        }

        return true;
    };

    auto applyPresentation = [&]( SCH_FIELD& aField, const PART_ATTRIBUTE& aAttr )
    {
        aField.SetVisible( aAttr.visible );

        // Keep custom fields in the same CAEDECAL-local coordinate system as the
        // standard fields, then transform through the placed symbol.
        VECTOR2I localOffset( toKiCadUnits( aAttr.position.x ),
                              -toKiCadUnits( aAttr.position.y ) );
        VECTOR2I fieldOffset = aSymbol->GetTransform().TransformCoordinate( localOffset );
        aField.SetPosition( aSymbol->GetPosition() + fieldOffset );

        if( aAttr.rotation != 0.0 )
            aField.SetTextAngleDegrees( aAttr.rotation );

        // Use the same PADS -> KiCad text calibration as Reference/Value.
        // Full-format attributes provide height; simplified records only
        // provide size, so keep size as the fallback.
        int nominalSize = aAttr.height > 0 ? aAttr.height : KiROUND( aAttr.size );

        if( nominalSize > 0 )
        {
            int scaledH = KiROUND( schIUScale.MilsToIU( nominalSize )
                                   * ADVANCED_CFG::GetCfg().m_PadsSchTextHeightScale );
            int scaledW = KiROUND( schIUScale.MilsToIU( nominalSize )
                                   * ADVANCED_CFG::GetCfg().m_PadsSchTextWidthScale );
            aField.SetTextSize( VECTOR2I( scaledW, scaledH ) );
        }

        if( aAttr.width > 0 )
            aField.SetTextThickness( schIUScale.MilsToIU( aAttr.width ) );

        GR_TEXT_H_ALIGN_T hJustify = GR_TEXT_H_ALIGN_LEFT;
        GR_TEXT_V_ALIGN_T vJustify = GR_TEXT_V_ALIGN_BOTTOM;
        PADS_COMMON::DecodeJustification( aAttr.justification, hJustify, vJustify );

        aField.SetHorizJustify( hJustify );
        aField.SetVertJustify( vJustify );
        applyPadsAsciiFieldFont( aField, aAttr, m_params );
    };

    std::set<std::string> processedNames;

    for( const PART_ATTRIBUTE& attr : aPlacement.attributes )
    {
        // Skip standard fields that are handled by ApplyPartAttributes
        if( mapper.IsStandardField( attr.name ) )
            continue;

        // PADS SIGPIN<n> attributes are per-instance signal-pin net overrides,
        // not user-visible component properties.
        if( isSignalPinOverride( attr.name ) )
            continue;

        // Skip empty attributes
        if( attr.value.empty() )
            continue;

        processedNames.insert( attr.name );

        // Get the mapped field name
        std::string fieldName = mapper.GetKiCadFieldName( attr.name );

        // Check if this field already exists on the symbol
        SCH_FIELD* existingField = aSymbol->GetField( wxString::FromUTF8( fieldName ) );

        if( existingField )
        {
            // Update existing field value and settings
            existingField->SetText( wxString::FromUTF8( attr.value ) );
            applyPresentation( *existingField, attr );
        }
        else
        {
            // Create a new custom field using FIELD_T::USER for custom fields
            SCH_FIELD newField( aSymbol, FIELD_T::USER, wxString::FromUTF8( fieldName ) );

            newField.SetText( wxString::FromUTF8( attr.value ) );
            applyPresentation( newField, attr );

            aSymbol->GetFields().push_back( newField );
            fieldsCreated++;
        }
    }

    // Create fields from attr_overrides that weren't in the attributes vector
    for( const auto& [name, value] : aPlacement.attr_overrides )
    {
        if( value.empty() || processedNames.count( name ) || mapper.IsStandardField( name )
                || isSignalPinOverride( name ) )
            continue;

        std::string fieldName = mapper.GetKiCadFieldName( name );
        SCH_FIELD* existingField = aSymbol->GetField( wxString::FromUTF8( fieldName ) );

        if( existingField )
        {
            existingField->SetText( wxString::FromUTF8( value ) );
            existingField->SetVisible( false );
        }
        else
        {
            SCH_FIELD newField( aSymbol, FIELD_T::USER, wxString::FromUTF8( fieldName ) );
            newField.SetText( wxString::FromUTF8( value ) );
            newField.SetVisible( false );
            newField.SetPosition( aSymbol->GetPosition() );

            aSymbol->GetFields().push_back( newField );
            fieldsCreated++;
        }
    }

    return fieldsCreated;
}


void PADS_SCH_SCHEMATIC_BUILDER::CreateTitleBlock( SCH_SCREEN* aScreen )
{
    if( !aScreen )
        return;

    // Look up the first non-empty value from a list of candidate field names
    auto findField =
            [this]( const std::initializer_list<const char*>& aCandidates ) -> std::string
            {
                for( const char* name : aCandidates )
                {
                    auto it = m_params.fields.find( name );

                    if( it != m_params.fields.end() && !it->second.empty() )
                        return it->second;
                }

                return {};
            };

    TITLE_BLOCK tb;

    std::string title = findField( { "Title", "TITLE1" } );

    if( title.empty() )
        title = m_params.job_name;

    if( !title.empty() )
        tb.SetTitle( wxString::FromUTF8( title ) );

    std::string date = findField( { "DATE", "Release Date", "Drawn Date" } );

    if( !date.empty() )
        tb.SetDate( wxString::FromUTF8( date ) );

    std::string revision = findField( { "Revision", "VER" } );

    if( !revision.empty() )
        tb.SetRevision( wxString::FromUTF8( revision ) );

    std::string company = findField( { "Company Name" } );

    if( !company.empty() )
        tb.SetCompany( wxString::FromUTF8( company ) );

    std::string drawingNumber = findField( { "DN", "Drawing Number" } );

    if( !drawingNumber.empty() )
        tb.SetComment( 0, wxString::FromUTF8( drawingNumber ) );

    std::string designer = findField( { "DESIGNER" } );

    if( !designer.empty() )
        tb.SetComment( 1, wxString::FromUTF8( designer ) );

    std::string drawnBy = findField( { "DRAWNBY", "Drawn By" } );

    if( !drawnBy.empty() )
        tb.SetComment( 2, wxString::FromUTF8( drawnBy ) );

    std::string builtFor = findField( { "BUILTFOR" } );

    if( !builtFor.empty() )
        tb.SetComment( 3, wxString::FromUTF8( builtFor ) );

    aScreen->SetTitleBlock( tb );
}


SCH_SHEET* PADS_SCH_SCHEMATIC_BUILDER::CreateHierarchicalSheet( int aSheetNumber, int aTotalSheets,
                                                                 SCH_SHEET*      aParentSheet,
                                                                 const wxString& aBaseFilename )
{
    if( !aParentSheet || !m_schematic )
        return nullptr;

    VECTOR2I pos = CalculateSheetPosition( aSheetNumber - 1, aTotalSheets );
    VECTOR2I size = GetDefaultSheetSize();

    SCH_SHEET* sheet = new SCH_SHEET( aParentSheet, pos, size );

    // Create a screen for this sheet
    SCH_SCREEN* screen = new SCH_SCREEN( m_schematic );
    sheet->SetScreen( screen );

    // Generate sheet filename based on base filename and sheet number
    wxFileName fn( aBaseFilename );
    wxString sheetFilename = wxString::Format( wxT( "%s_sheet%d.%s" ),
                                               fn.GetName(),
                                               aSheetNumber,
                                               FILEEXT::KiCadSchematicFileExtension );

    // Set the sheet filename field
    sheet->GetField( FIELD_T::SHEET_FILENAME )->SetText( sheetFilename );

    // Set sheet name
    wxString sheetName = wxString::Format( wxT( "Sheet %d" ), aSheetNumber );
    sheet->GetField( FIELD_T::SHEET_NAME )->SetText( sheetName );

    // Set full path for the screen if project is available
    if( m_schematic->IsValid() )
    {
        wxFileName screenFn( m_schematic->Project().GetProjectPath(), sheetFilename );
        screen->SetFileName( screenFn.GetFullPath() );
    }
    else
    {
        screen->SetFileName( sheetFilename );
    }

    sheet->SetFlags( IS_NEW );

    // Add the sheet to the parent's screen
    SCH_SCREEN* parentScreen = aParentSheet->GetScreen();

    if( parentScreen )
    {
        parentScreen->Append( sheet );
    }

    return sheet;
}


VECTOR2I PADS_SCH_SCHEMATIC_BUILDER::GetDefaultSheetSize() const
{
    // Default sheet symbol size in mils (approximately 2" x 1.5")
    return VECTOR2I( schIUScale.MilsToIU( 2000 ), schIUScale.MilsToIU( 1500 ) );
}


VECTOR2I PADS_SCH_SCHEMATIC_BUILDER::CalculateSheetPosition( int aSheetIndex, int aTotalSheets ) const
{
    // Arrange sheet symbols in a grid on the root sheet
    // Start position offset from origin
    const int startX = schIUScale.MilsToIU( 500 );
    const int startY = schIUScale.MilsToIU( 500 );

    // Spacing between sheet symbols
    const int spacingX = schIUScale.MilsToIU( 2500 );
    const int spacingY = schIUScale.MilsToIU( 2000 );

    // Calculate grid columns based on total sheets (aim for roughly square layout)
    int columns = static_cast<int>( std::ceil( std::sqrt( static_cast<double>( aTotalSheets ) ) ) );

    if( columns < 1 )
        columns = 1;

    int row = aSheetIndex / columns;
    int col = aSheetIndex % columns;

    return VECTOR2I( startX + col * spacingX, startY + row * spacingY );
}


SCH_SHEET_PIN* PADS_SCH_SCHEMATIC_BUILDER::CreateSheetPin( SCH_SHEET*         aSheet,
                                                           const std::string& aSignalName,
                                                           int                aPinIndex )
{
    if( !aSheet )
        return nullptr;

    wxString name = wxString::FromUTF8( aSignalName );

    // Position pins along the left edge of the sheet
    VECTOR2I sheetPos = aSheet->GetPosition();

    int pinSpacing = schIUScale.MilsToIU( 200 );
    int yOffset = schIUScale.MilsToIU( 100 ) + aPinIndex * pinSpacing;

    VECTOR2I pinPos( sheetPos.x, sheetPos.y + yOffset );

    SCH_SHEET_PIN* pin = new SCH_SHEET_PIN( aSheet, pinPos, name );
    pin->SetSide( SHEET_SIDE::LEFT );
    pin->SetShape( LABEL_FLAG_SHAPE::L_UNSPECIFIED );

    aSheet->AddPin( pin );

    return pin;
}


SCH_HIERLABEL* PADS_SCH_SCHEMATIC_BUILDER::CreateHierLabel( const std::string& aSignalName,
                                                            const VECTOR2I&    aPosition,
                                                            SCH_SCREEN*        aScreen )
{
    if( !aScreen )
        return nullptr;

    wxString name = wxString::FromUTF8( aSignalName );

    SCH_HIERLABEL* label = new SCH_HIERLABEL( aPosition, name );
    label->SetShape( LABEL_FLAG_SHAPE::L_UNSPECIFIED );
    label->SetFlags( IS_NEW );

    aScreen->Append( label );

    return label;
}



bool PADS_SCH_SCHEMATIC_BUILDER::IsGlobalSignal( const std::string& aSignalName,
                                                 const std::set<int>& aSheetNumbers )
{
    if( aSignalName.empty() )
        return false;

    // Signals appearing on multiple sheets should be global
    if( aSheetNumbers.size() > 1 )
        return true;

    // Common power net names are always global
    std::string upperName = aSignalName;
    std::transform( upperName.begin(), upperName.end(), upperName.begin(), ::toupper );

    static const std::set<std::string> globalPatterns = {
        "VCC", "VDD", "VEE", "VSS", "GND", "AGND", "DGND", "PGND",
        "V+", "V-", "VBAT", "VBUS", "VIN", "VOUT",
        "+5V", "+3V3", "+3.3V", "+12V", "-12V", "+24V",
        "0V", "EARTH", "CHASSIS"
    };

    if( globalPatterns.count( upperName ) > 0 )
        return true;

    // Check for voltage patterns like +1V8, +2V5, etc.
    if( ( upperName[0] == '+' || upperName[0] == '-' ) && upperName.length() >= 3 )
    {
        bool hasDigit = false;
        bool hasV = false;

        for( char c : upperName.substr( 1 ) )
        {
            if( std::isdigit( c ) )
                hasDigit = true;

            if( c == 'V' )
                hasV = true;
        }

        if( hasDigit && hasV )
            return true;
    }

    return false;
}

} // namespace PADS_SCH
