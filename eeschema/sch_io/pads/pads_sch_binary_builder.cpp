/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 */

#include <sch_io/pads/pads_sch_binary_builder.h>


#include <bitmap_base.h>
#include <lib_id.h>
#include <lib_symbol.h>
#include <connection_graph.h>
#include <embedded_files.h>
#include <font/font.h>
#include <page_info.h>
#include <pin_type.h>
#include <sch_bus_entry.h>
#include <sch_bitmap.h>
#include <sch_field.h>
#include <sch_junction.h>
#include <sch_label.h>
#include <sch_line.h>
#include <sch_no_connect.h>
#include <sch_pin.h>
#include <sch_screen.h>
#include <sch_shape.h>
#include <sch_sheet.h>
#include <sch_sheet_path.h>
#include <sch_symbol.h>
#include <sch_text.h>
#include <schematic.h>
#include <schematic_settings.h>
#include <io/pads/pads_attribute_mapper.h>
#include <io/pads/pads_common.h>
#include <sch_io/ole_image.h>
#include <sch_io/pads/pads_sch_symbol_builder.h>
#include <stroke_params.h>
#include <title_block.h>

#include <ki_exception.h>
#include <wildcards_and_files_ext.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <locale>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>
#include <wx/buffer.h>
#include <wx/image.h>
#include <wx/log.h>

namespace PADS_SCH_BINARY
{
namespace
{

    int toIU( int64_t aHalfMils )
    {
        return schIUScale.MilsToIU( static_cast<double>( aHalfMils ) / 2.0 );
    }


    VECTOR2I localPoint( const SOURCE_POINT& aPoint )
    {
        return { toIU( aPoint.x ), -toIU( aPoint.y ) };
    }


    // A saved schematic keys its library on the LIB_ID, so two placements of one part type that
    // build different symbols have to be told apart. Keyed by part-type name, valued by the
    // distinct symbols seen under it and the unique name each was given.
    using LIBRARY_SYMBOL_VARIANTS = std::map<wxString, std::vector<std::pair<std::unique_ptr<LIB_SYMBOL>, wxString>>>;


    VECTOR2I pagePoint( const SOURCE_POINT& aPoint, int aPageHeight )
    {
        return { toIU( aPoint.x ), aPageHeight - toIU( aPoint.y ) };
    }


    std::unique_ptr<SCH_BITMAP> makeEmbeddedImage( const MODEL_EMBEDDED_IMAGE& aSource, int aPageHeight,
                                                   std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        if( aSource.size.x <= 0 || aSource.size.y <= 0 )
        {
            aDiagnostics.emplace_back( RPT_SEVERITY_WARNING, aSource.source,
                                       wxS( "embedded OLE image has a degenerate page box and was skipped" ) );
            return nullptr;
        }

        auto             bitmap = std::make_unique<SCH_BITMAP>( pagePoint( aSource.position, aPageHeight ) );
        REFERENCE_IMAGE& refImage = bitmap->GetReferenceImage();
        bool             decoded = false;
        wxMemoryBuffer   buffer;

        switch( aSource.type )
        {
        case MODEL_EMBEDDED_IMAGE_TYPE::BMP:
            buffer.AppendData( aSource.data.data(), aSource.data.size() );
            {
                wxLogNull noLog;
                decoded = refImage.ReadImageFile( buffer );
            }
            break;

        case MODEL_EMBEDDED_IMAGE_TYPE::DIB:
            if( OleMakeBmpFromDib( aSource.data, buffer ) )
            {
                wxLogNull noLog;
                decoded = refImage.ReadImageFile( buffer );
            }
            break;

        case MODEL_EMBEDDED_IMAGE_TYPE::WMF:
        {
            wxImage image;
            int width = std::clamp<int64_t>( std::abs( static_cast<int64_t>( aSource.extent[2] ) - aSource.extent[0] ),
                                             1, 4096 );
            int height = std::clamp<int64_t>( std::abs( static_cast<int64_t>( aSource.extent[3] ) - aSource.extent[1] ),
                                              1, 4096 );

            if( OleRenderWmf( aSource.data, width, height, image ) )
            {
                // Clamp before rounding; llround on an out-of-range double is undefined
                double scaledHeight = static_cast<double>( width ) * aSource.size.y / aSource.size.x;
                int    fittedHeight =
                        static_cast<int>( std::llround( std::clamp( scaledHeight, 1.0, 4096.0 ) ) );
                image.Rescale( width, fittedHeight, wxIMAGE_QUALITY_HIGH );
                decoded = refImage.SetImage( image );
            }

            break;
        }

        case MODEL_EMBEDDED_IMAGE_TYPE::UNSUPPORTED: break;
        }

        if( !decoded )
        {
            aDiagnostics.emplace_back( RPT_SEVERITY_WARNING, aSource.source,
                                       wxS( "embedded OLE image could not be rasterized and was skipped" ) );
            return nullptr;
        }

        const int targetWidth = toIU( aSource.size.x );
        const int targetHeight = toIU( aSource.size.y );

        if( targetWidth <= 0 || targetHeight <= 0 )
            THROW_IO_ERROR( FormatParserError( aSource.source, wxS( "embedded image has invalid page size" ) ) );

        const wxImage* decodedImage = refImage.GetImage().GetImageData();

        if( decodedImage && decodedImage->IsOk() )
        {
            const int fittedHeight =
                    std::max( 1, static_cast<int>( std::llround( static_cast<double>( decodedImage->GetWidth() )
                                                                 * targetHeight / targetWidth ) ) );

            if( fittedHeight != decodedImage->GetHeight() )
            {
                wxImage fitted = decodedImage->Copy();
                fitted.Rescale( fitted.GetWidth(), fittedHeight, wxIMAGE_QUALITY_HIGH );
                refImage.SetImage( fitted );
            }
        }

        refImage.SetWidth( targetWidth );

        const VECTOR2I center = bitmap->GetPosition();

        if( aSource.mirrorHorizontal )
            bitmap->MirrorHorizontally( center.x );

        if( aSource.mirrorVertical )
            bitmap->MirrorVertically( center.y );

        return bitmap;
    }


    LINE_STYLE lineStyle( MODEL_LINE_STYLE aStyle )
    {
        switch( aStyle )
        {
        case MODEL_LINE_STYLE::DEFAULT:
        case MODEL_LINE_STYLE::SOLID: return LINE_STYLE::SOLID;
        case MODEL_LINE_STYLE::DASH: return LINE_STYLE::DASH;
        case MODEL_LINE_STYLE::DOT: return LINE_STYLE::DOT;
        case MODEL_LINE_STYLE::DASH_DOT: return LINE_STYLE::DASHDOT;
        }

        return LINE_STYLE::SOLID;
    }


    FILL_T fillStyle( MODEL_FILL_STYLE aFill )
    {
        switch( aFill )
        {
        case MODEL_FILL_STYLE::NONE: return FILL_T::NO_FILL;
        case MODEL_FILL_STYLE::FILLED: return FILL_T::FILLED_SHAPE;
        case MODEL_FILL_STYLE::HATCHED: return FILL_T::HATCH;
        }

        return FILL_T::NO_FILL;
    }


    ELECTRICAL_PINTYPE pinType( uint32_t aType )
    {
        switch( aType )
        {
        case 0: return ELECTRICAL_PINTYPE::PT_UNSPECIFIED; // PADS 'U' = Undefined
        case 1: return ELECTRICAL_PINTYPE::PT_INPUT;
        case 2: return ELECTRICAL_PINTYPE::PT_OUTPUT;
        case 3: return ELECTRICAL_PINTYPE::PT_BIDI;
        case 4: return ELECTRICAL_PINTYPE::PT_TRISTATE;
        case 5: return ELECTRICAL_PINTYPE::PT_OPENCOLLECTOR;
        case 6: return ELECTRICAL_PINTYPE::PT_OPENEMITTER;
        case 7: return ELECTRICAL_PINTYPE::PT_POWER_IN;
        default: return ELECTRICAL_PINTYPE::PT_UNSPECIFIED;
        }
    }


    GRAPHIC_PINSHAPE pinShape( uint32_t aStyle )
    {
        switch( aStyle )
        {
        case 1: return GRAPHIC_PINSHAPE::INVERTED;
        case 2: return GRAPHIC_PINSHAPE::CLOCK;
        case 3: return GRAPHIC_PINSHAPE::INVERTED_CLOCK;
        default: return GRAPHIC_PINSHAPE::LINE;
        }
    }


    SOURCE_POINT transformPadsTerminalVector( const SOURCE_POINT& aVector, int aAngle, uint32_t aXym )
    {
        const double radians = static_cast<double>( aAngle ) * M_PI / 1800.0;
        const double c = std::cos( radians );
        const double s = std::sin( radians );

        SOURCE_POINT result = aVector;
        result.x = std::llround( static_cast<double>( aVector.x ) * c
                                 - static_cast<double>( aVector.y ) * s );
        result.y = std::llround( static_cast<double>( aVector.x ) * s
                                 + static_cast<double>( aVector.y ) * c );

        if( aXym & 0x1 )
            result.x = -result.x;

        if( aXym & 0x2 )
            result.y = -result.y;

        return result;
    }


    SOURCE_POINT padsTerminalStemVector( const MODEL_PIN_DEFINITION& aPin )
    {
        if( aPin.pinDecalGeometryResolved )
            return transformPadsTerminalVector( aPin.pinDecalVector, aPin.angle, aPin.xym );

        // Compatibility fallback for malformed/incomplete libraries.  Normal
        // imports use the resolved pin-decal geometry above.
        SOURCE_POINT fallback;

        if( aPin.decalName.text.Contains( wxS( "VRT" ) ) )
            fallback.y = -aPin.length;
        else
            fallback.x = aPin.length;

        return transformPadsTerminalVector( fallback, aPin.angle, aPin.xym );
    }


    PIN_ORIENTATION pinOrientation( const MODEL_PIN_DEFINITION& aPin )
    {
        const SOURCE_POINT stemVector = padsTerminalStemVector( aPin );
        const int64_t      kicadDx = stemVector.x;
        const int64_t      kicadDy = -stemVector.y; // PADS Y-up -> KiCad Y-down

        if( std::abs( kicadDx ) >= std::abs( kicadDy ) )
            return kicadDx >= 0 ? PIN_ORIENTATION::PIN_RIGHT : PIN_ORIENTATION::PIN_LEFT;

        return kicadDy >= 0 ? PIN_ORIENTATION::PIN_DOWN : PIN_ORIENTATION::PIN_UP;
    }


    GR_TEXT_H_ALIGN_T horizontalJustification( MODEL_JUSTIFICATION aJustification )
    {
        switch( aJustification )
        {
        case MODEL_JUSTIFICATION::CENTER: return GR_TEXT_H_ALIGN_CENTER;
        case MODEL_JUSTIFICATION::RIGHT: return GR_TEXT_H_ALIGN_RIGHT;
        default: return GR_TEXT_H_ALIGN_LEFT;
        }
    }


    GR_TEXT_V_ALIGN_T verticalJustification( MODEL_JUSTIFICATION aJustification )
    {
        switch( aJustification )
        {
        case MODEL_JUSTIFICATION::LEFT: return GR_TEXT_V_ALIGN_TOP;
        case MODEL_JUSTIFICATION::RIGHT: return GR_TEXT_V_ALIGN_BOTTOM;
        default: return GR_TEXT_V_ALIGN_CENTER;
        }
    }


    KIFONT::FONT* resolvePadsPresentationFont( const MODEL_TEXT_PRESENTATION& aPresentation )
    {
        if( aPresentation.font.text.IsEmpty()
            || aPresentation.font.text == wxS( "Default Font" ) )
        {
            return KIFONT::FONT::GetFont();
        }

        KIFONT::FONT* font = KIFONT::FONT::GetFont(
                aPresentation.font.text, aPresentation.bold, aPresentation.italic );

        // KiCad returns its stroke font when an outline family cannot be loaded.
        // Fontconfig's compatible-family result is not guaranteed to be used here,
        // so make the known Linux replacement for PADS Verdana explicit.
        if( font && font->IsStroke()
            && aPresentation.font.text.CmpNoCase( wxS( "Verdana" ) ) == 0 )
        {
            KIFONT::FONT* fallback = KIFONT::FONT::GetFont(
                    wxS( "Noto Sans" ), aPresentation.bold, aPresentation.italic );

            if( fallback && !fallback->IsStroke() )
                font = fallback;
        }

        return font;
    }


    constexpr double PADS_SYSTEM_TEXT_HEIGHT_CORRECTION = 0.5885217956;
// Calibrated from a PADS Verdana field authored at 97 mil: 1.45 mm in KiCad matches the PADS visual size.
// 1.45 mm / ( 97 mil * 0.0254 mm/mil ) = 0.5885217956.


    int64_t padsVisualTextHeight(
            const MODEL_TEXT_PRESENTATION& aPresentation,
            double aOutlineHeightCorrection = PADS_SYSTEM_TEXT_HEIGHT_CORRECTION )
    {
        if( aPresentation.height <= 0 )
            return aPresentation.height;

        KIFONT::FONT* font = resolvePadsPresentationFont( aPresentation );

        if( !font || font->IsStroke() )
            return aPresentation.height;

        // PADS system-font height is a Windows character-cell height.  KiCad's
        // outline text size is visual/em based; use the caller's calibrated visual ratio.
        return std::max<int64_t>(
                1, static_cast<int64_t>( std::llround(
                           static_cast<double>( aPresentation.height )
                           * aOutlineHeightCorrection ) ) );
    }


    SPIN_STYLE padsLabelSpinStyle( const SOURCE_POINT& aPosition,
                                         const SOURCE_POINT& aTextOffset,
                                         const SOURCE_POINT* aAdjacentWirePoint )
    {
        // Match the ASCII importer: NETNAMES stores the text/body offset from
        // the electrical anchor and is the authoritative orientation when present.
        if( aTextOffset.x != 0 || aTextOffset.y != 0 )
        {
            if( std::abs( aTextOffset.x ) >= std::abs( aTextOffset.y ) )
                return aTextOffset.x >= 0 ? SPIN_STYLE::RIGHT : SPIN_STYLE::LEFT;

            return aTextOffset.y >= 0 ? SPIN_STYLE::UP : SPIN_STYLE::BOTTOM;
        }

        // Fall back to the adjacent wire only when NETNAMES has no usable offset.
        // The label body extends away from the wire.
        if( aAdjacentWirePoint )
        {
            const int64_t deltaX = aAdjacentWirePoint->x - aPosition.x;
            const int64_t deltaY = aAdjacentWirePoint->y - aPosition.y;

            if( deltaX != 0 || deltaY != 0 )
            {
                if( std::abs( deltaX ) >= std::abs( deltaY ) )
                    return deltaX < 0 ? SPIN_STYLE::RIGHT : SPIN_STYLE::LEFT;

                // PADS page Y increases upwards; KiCad screen Y increases downwards.
                return deltaY > 0 ? SPIN_STYLE::BOTTOM : SPIN_STYLE::UP;
            }
        }

        return SPIN_STYLE::RIGHT;
    }


    void applyTextPresentation(
            EDA_TEXT* aText, const MODEL_TEXT_PRESENTATION& aPresentation, bool aVisible,
            std::vector<PARSER_DIAGNOSTIC>& aDiagnostics,
            double aOutlineHeightCorrection = PADS_SYSTEM_TEXT_HEIGHT_CORRECTION )
    {
        if( aPresentation.height > 0 )
        {
            const int visualHeight =
                    toIU( padsVisualTextHeight( aPresentation, aOutlineHeightCorrection ) );
            aText->SetTextSize( { visualHeight, visualHeight } );
        }

        aText->SetHorizJustify( horizontalJustification( aPresentation.horizontalJustification ) );
        aText->SetVertJustify( verticalJustification( aPresentation.verticalJustification ) );
        aText->SetBold( aPresentation.bold );
        aText->SetItalic( aPresentation.italic );
        aText->SetVisible( aVisible );

        if( aPresentation.width > 0 )
            aText->SetTextThickness( toIU( aPresentation.width ) );

        if( !aPresentation.font.text.IsEmpty() && aPresentation.font.text != wxS( "Default Font" ) )
            aText->SetFont( resolvePadsPresentationFont( aPresentation ) );

        // PADS gives the rendered stroke; run it back through the bold factor once the face is
        // resolved, since only a stroke font applies that factor
        aText->MigrateLegacyBoldStrokeWidth();

        if( aPresentation.underline )
        {
            aDiagnostics.push_back( MakePropertyDiagnostic( RPT_SEVERITY_WARNING, aPresentation.source,
                                                            wxS( "underline" ), PROPERTY_DISPOSITION::UNSUPPORTED,
                                                            wxS( "PADS underline presentation is unsupported" ) ) );
        }
    }


    std::unique_ptr<SCH_SHAPE> makeShape( const MODEL_GRAPHIC& aGraphic, bool aPageCoordinates, int aPageHeight )
    {
        if( aGraphic.kind == MODEL_GRAPHIC_KIND::TEXT || aGraphic.points.size() < 2 )
            THROW_IO_ERROR( FormatParserError( aGraphic.source, wxS( "graphic has inconsistent geometry" ) ) );

        auto convert = [&]( const SOURCE_POINT& aPoint )
        {
            return aPageCoordinates ? pagePoint( aPoint, aPageHeight ) : localPoint( aPoint );
        };

        SHAPE_T shapeType = SHAPE_T::POLY;

        switch( aGraphic.kind )
        {
        case MODEL_GRAPHIC_KIND::RECTANGLE: shapeType = SHAPE_T::RECTANGLE; break;
        case MODEL_GRAPHIC_KIND::CIRCLE: shapeType = SHAPE_T::CIRCLE; break;
        case MODEL_GRAPHIC_KIND::ARC: shapeType = SHAPE_T::ARC; break;
        default: shapeType = SHAPE_T::POLY; break;
        }

        auto shape = std::make_unique<SCH_SHAPE>( shapeType, aPageCoordinates ? LAYER_NOTES : LAYER_DEVICE );

        if( aGraphic.kind == MODEL_GRAPHIC_KIND::RECTANGLE && aGraphic.points.size() >= 2 )
        {
            shape->SetStart( convert( aGraphic.points[0] ) );
            shape->SetEnd( convert( aGraphic.points[1] ) );
        }
        else if( aGraphic.kind == MODEL_GRAPHIC_KIND::CIRCLE && aGraphic.points.size() >= 2 )
        {
            SOURCE_POINT center;
            center.x = ( aGraphic.points[0].x + aGraphic.points[1].x ) / 2;
            center.y = ( aGraphic.points[0].y + aGraphic.points[1].y ) / 2;
            shape->SetStart( convert( center ) );
            shape->SetEnd( convert( aGraphic.points[0] ) );
        }
        else if( aGraphic.kind == MODEL_GRAPHIC_KIND::ARC && aGraphic.points.size() >= 2 )
        {
            const SOURCE_POINT& start = aGraphic.points.front();
            const SOURCE_POINT& end = aGraphic.points.back();
            double              startAngle = std::atan2( static_cast<double>( start.y - aGraphic.arcCenter.y ),
                                                         static_cast<double>( start.x - aGraphic.arcCenter.x ) );
            double              sweep = std::abs( aGraphic.arcSweepAngle ) * M_PI / 1800.0;

            if( aGraphic.arcClockwise )
                sweep = -sweep;

            double       radius = std::hypot( static_cast<double>( start.x - aGraphic.arcCenter.x ),
                                              static_cast<double>( start.y - aGraphic.arcCenter.y ) );
            SOURCE_POINT mid;
            mid.x = aGraphic.arcCenter.x + std::llround( radius * std::cos( startAngle + sweep / 2.0 ) );
            mid.y = aGraphic.arcCenter.y + std::llround( radius * std::sin( startAngle + sweep / 2.0 ) );
            shape->SetArcGeometry( convert( start ), convert( mid ), convert( end ) );
        }
        else
        {
            for( const SOURCE_POINT& point : aGraphic.points )
                shape->AddPoint( convert( point ) );
        }

        shape->SetStroke( STROKE_PARAMS( toIU( aGraphic.strokeWidth ), lineStyle( aGraphic.lineStyle ) ) );
        shape->SetFillMode( fillStyle( aGraphic.fill ) );
        return shape;
    }


    std::unique_ptr<SCH_TEXT> makeGraphicText( const MODEL_GRAPHIC&            aGraphic,
                                               std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        if( aGraphic.points.empty() )
            THROW_IO_ERROR( FormatParserError( aGraphic.source, wxS( "symbol text has no position" ) ) );

        auto text =
                std::make_unique<SCH_TEXT>( localPoint( aGraphic.points.front() ), aGraphic.text.text, LAYER_DEVICE );
        text->SetTextAngle( EDA_ANGLE( aGraphic.angle, TENTHS_OF_A_DEGREE_T ) );
        applyTextPresentation( text.get(), aGraphic.presentation, aGraphic.presentation.visible, aDiagnostics );
        return text;
    }


    std::unique_ptr<SCH_TEXT> makePageText( const SOURCE_STRING& aText, const SOURCE_POINT& aPosition, int aAngle,
                                            const MODEL_TEXT_PRESENTATION& aPresentation, int aPageHeight,
                                            std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        auto text = std::make_unique<SCH_TEXT>( pagePoint( aPosition, aPageHeight ), aText.text, LAYER_NOTES );
        text->SetTextAngle( EDA_ANGLE( aAngle, TENTHS_OF_A_DEGREE_T ) );

        // The s-expression writer emits no visibility for a plain text and the reader forces it
        // visible, so importing hidden would disagree with the first save
        if( !aPresentation.visible )
        {
            aDiagnostics.push_back( MakePropertyDiagnostic( RPT_SEVERITY_WARNING, aPresentation.source,
                                                            wxS( "display_flags" ), PROPERTY_DISPOSITION::UNSUPPORTED,
                                                            wxS( "hidden PADS text is unsupported" ) ) );
        }

        applyTextPresentation( text.get(), aPresentation, true, aDiagnostics );
        return text;
    }


    size_t appendPageGraphic( SCH_SCREEN* aScreen, const MODEL_GRAPHIC& aGraphic, int aPageHeight,
                              std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        if( aGraphic.kind == MODEL_GRAPHIC_KIND::TEXT )
        {
            if( aGraphic.points.empty() )
                THROW_IO_ERROR( FormatParserError( aGraphic.source, wxS( "page text has no position" ) ) );

            std::unique_ptr<SCH_TEXT> text = makePageText( aGraphic.text, aGraphic.points.front(), aGraphic.angle,
                                                           aGraphic.presentation, aPageHeight, aDiagnostics );
            aScreen->Append( text.get() );
            text.release();
            return 1;
        }

        if( ( aGraphic.kind == MODEL_GRAPHIC_KIND::LINE || aGraphic.kind == MODEL_GRAPHIC_KIND::POLYLINE )
            && aGraphic.fill == MODEL_FILL_STYLE::NONE )
        {
            if( aGraphic.points.size() < 2 )
                THROW_IO_ERROR(
                        FormatParserError( aGraphic.source, wxS( "page polyline has inconsistent geometry" ) ) );

            for( size_t point = 1; point < aGraphic.points.size(); ++point )
            {
                auto line =
                        std::make_unique<SCH_LINE>( pagePoint( aGraphic.points[point - 1], aPageHeight ), LAYER_NOTES );
                line->SetEndPoint( pagePoint( aGraphic.points[point], aPageHeight ) );
                line->SetStroke( STROKE_PARAMS( toIU( aGraphic.strokeWidth ), lineStyle( aGraphic.lineStyle ) ) );
                aScreen->Append( line.get() );
                line.release();
            }

            return aGraphic.points.size() - 1;
        }

        std::unique_ptr<SCH_SHAPE> shape = makeShape( aGraphic, true, aPageHeight );

        if( aGraphic.fill == MODEL_FILL_STYLE::FILLED )
            shape->SetFillMode( FILL_T::FILLED_WITH_BG_BODYCOLOR );

        aScreen->Append( shape.get() );
        shape.release();
        return 1;
    }


    std::string padsPowerDecalName( const MODEL_LABEL& aLabel )
    {
        if( !aLabel.decalName.text.IsEmpty() )
            return aLabel.decalName.text.ToStdString();

        return aLabel.text.text.ToStdString();
    }


    std::string padsPowerStyle( const MODEL_LABEL& aLabel )
    {
        return PADS_SCH::PADS_SCH_SYMBOL_BUILDER::GetPowerStyleFromVariant(
                padsPowerDecalName( aLabel ),
                aLabel.kind == MODEL_LABEL_KIND::GROUND ? "G" : "P" );
    }


    std::unique_ptr<LIB_SYMBOL> makePadsPowerLibrary( const MODEL_LABEL& aLabel )
    {
        wxString name;

        if( aLabel.kind == MODEL_LABEL_KIND::GROUND )
            name = aLabel.symbolVariant == 1   ? wxS( "PADS_GNDA" )
                   : aLabel.symbolVariant == 2 ? wxS( "PADS_GNDCH" )
                                               : wxS( "PADS_GND" );
        else
            name = wxString::Format( wxS( "PADS_POWER_%u" ), aLabel.symbolVariant );

        const std::string style = padsPowerStyle( aLabel );

        if( style.empty() )
            return nullptr;

        PADS_SCH::PARAMETERS params;
        PADS_SCH::PADS_SCH_SYMBOL_BUILDER symbolBuilder( params );
        std::unique_ptr<LIB_SYMBOL> symbol(
                symbolBuilder.BuildKiCadPowerSymbol( style, padsPowerDecalName( aLabel ) ) );

        if( !symbol )
            return nullptr;

        // Keep the binary import's private library identifier stable.
        symbol->SetName( name );
        return symbol;
    }


    int padsPowerOrientation( const MODEL_LABEL& aLabel,
                              const SOURCE_POINT* aAdjacentWirePoint, bool aPinUp )
    {
        if( !aAdjacentWirePoint )
            return SYM_ORIENT_0;

        const int64_t dx = aAdjacentWirePoint->x - aLabel.position.x;
        const int64_t dy = -( aAdjacentWirePoint->y - aLabel.position.y ); // PADS Y-up -> KiCad Y-down

        if( dx == 0 && dy == 0 )
            return SYM_ORIENT_0;

        if( std::abs( dx ) >= std::abs( dy ) )
        {
            if( dx > 0 )
                return aPinUp ? SYM_ORIENT_90 : SYM_ORIENT_270;

            return aPinUp ? SYM_ORIENT_270 : SYM_ORIENT_90;
        }

        if( dy > 0 )
            return aPinUp ? SYM_ORIENT_0 : SYM_ORIENT_180;

        return aPinUp ? SYM_ORIENT_180 : SYM_ORIENT_0;
    }


    std::unique_ptr<SCH_SYMBOL> makePowerSymbol( const MODEL_LABEL& aLabel, const SCH_SHEET_PATH& aPath,
                                                 int aPageHeight, int aOrdinal,
                                                 const SOURCE_POINT* aAdjacentWirePoint,
                                                 std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        const std::string powerStyle = padsPowerStyle( aLabel );
        const bool pinUp = powerStyle == "VCC" || powerStyle == "PWR_TRIANGLE";
        const int orientation = padsPowerOrientation( aLabel, aAdjacentWirePoint, pinUp );

        std::unique_ptr<LIB_SYMBOL> library = makePadsPowerLibrary( aLabel );

        if( !library )
            THROW_IO_ERROR( FormatParserError( aLabel.source, wxS( "could not construct power symbol" ) ) );

        auto   symbol = std::make_unique<SCH_SYMBOL>();
        LIB_ID libId;
        libId.SetLibNickname( wxS( "pads_import" ) );
        libId.SetLibItemName( library->GetName() );
        symbol->SetLibId( libId );
        symbol->SetLibSymbol( library.release() );
        symbol->SetPosition( pagePoint( aLabel.position, aPageHeight ) );
        symbol->SetOrientation( orientation );
        const wxString reference = wxString::Format( wxS( "#PWR%04d" ), aOrdinal );
        symbol->SetRef( &aPath, reference );
        symbol->AddHierarchicalReference( aPath.Path(), reference, 1 );
        symbol->SetValueFieldText( aLabel.text.text, &aPath );

        if( SCH_FIELD* field = symbol->GetField( FIELD_T::REFERENCE ) )
            field->SetVisible( false );

        if( SCH_FIELD* value = symbol->GetField( FIELD_T::VALUE ) )
        {
            value->SetText( aLabel.text.text );

            if( aLabel.hasNetNamePresentation )
            {
                // NETNAMES offsets are authored in page coordinates.
                value->SetPosition( symbol->GetPosition() + localPoint( aLabel.textOffset ) );
                value->SetTextAngle( EDA_ANGLE( aLabel.angle, TENTHS_OF_A_DEGREE_T ) );
                applyTextPresentation( value, aLabel.presentation,
                                       aLabel.presentation.visible, aDiagnostics );
            }
            else if( aLabel.hasDefaultPresentation )
            {
                // CAEDECAL REF-DES is local to the power/ground symbol.
                VECTOR2I textOffset = symbol->GetTransform().TransformCoordinate(
                        localPoint( aLabel.defaultTextOffset ) );
                value->SetPosition( symbol->GetPosition() + textOffset );
                value->SetTextAngle( EDA_ANGLE( aLabel.defaultAngle, TENTHS_OF_A_DEGREE_T ) );
                applyTextPresentation( value, aLabel.defaultPresentation,
                                       aLabel.presentation.visible, aDiagnostics );
            }
            else
            {
                applyTextPresentation( value, aLabel.presentation,
                                       aLabel.presentation.visible, aDiagnostics );
            }
        }

        return symbol;
    }


    const wxString PADS_PIN_NUMBER_CALIBRATION =
            wxS( "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-./[]" );
    const wxString PADS_PIN_NAME_CALIBRATION =
            wxS( "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ_#~+-/[]" );


    int fittedPinTextSize( const wxString& aCalibration, const MODEL_TEXT_PRESENTATION& aPresentation )
    {
        const int nominalSize = toIU( aPresentation.height );

        if( nominalSize <= 0 || aPresentation.font.text.IsEmpty()
            || aPresentation.font.text == wxS( "Default Font" ) )
        {
            return nominalSize;
        }

        KIFONT::FONT* sourceFont = KIFONT::FONT::GetFont(
                aPresentation.font.text, aPresentation.bold, aPresentation.italic );
        KIFONT::FONT* pinFont = KIFONT::FONT::GetFont();

        if( !sourceFont || !pinFont )
            return nominalSize;

        const VECTOR2I nominal( nominalSize, nominalSize );
        const int sourceThickness = aPresentation.width > 0 ? toIU( aPresentation.width ) : 0;
        const KIFONT::METRICS& metrics = KIFONT::METRICS::Default();
        const VECTOR2I sourceExtents = sourceFont->StringBoundaryLimits(
                aCalibration, nominal, sourceThickness, aPresentation.bold, aPresentation.italic, metrics );
        const VECTOR2I pinExtents = pinFont->StringBoundaryLimits(
                aCalibration, nominal, 0, false, false, metrics );

        if( sourceExtents.x <= 0 || sourceExtents.y <= 0 || pinExtents.x <= 0 || pinExtents.y <= 0 )
            return nominalSize;

        const double widthRatio = static_cast<double>( sourceExtents.x ) / pinExtents.x;
        const double heightRatio = static_cast<double>( sourceExtents.y ) / pinExtents.y;

        // PADS system-font height is a character-cell measure.  Use the same
        // visual-height calibration validated for symbol REF/VALUE fields:
        // 97 mil PADS -> about 1.45 mm in KiCad.
        constexpr double PADS_PIN_TEXT_HEIGHT_CORRECTION = 0.5885217956;
        const double fitted = static_cast<double>( nominalSize )
                              * std::min( widthRatio, heightRatio )
                              * PADS_PIN_TEXT_HEIGHT_CORRECTION;
        return std::max( 1, static_cast<int>( std::llround( fitted ) ) );
    }


    std::unique_ptr<SCH_PIN> makePin( const MODEL_PIN_DEFINITION& aPin, LIB_SYMBOL* aParent )
    {
        auto pin = std::make_unique<SCH_PIN>( aParent );
        pin->SetNumber( aPin.number.text );
        pin->SetName( aPin.name.text );
        pin->SetPosition( localPoint( aPin.position ) );
        pin->SetOrientation( pinOrientation( aPin ) );

        int pinLength = toIU( aPin.length );

        // aPin.length is resolved by the binary parser from the actual
        // owning-symbol body attachment point when possible, with the
        // pin-decal-derived stem length as fallback.

        pin->SetLength( pinLength );
        pin->SetType( pinType( aPin.electricalType ) );

        GRAPHIC_PINSHAPE graphicShape = pinShape( aPin.graphicStyle );

        // PADS PINIEB is the active-low input wedge, not a bubble.
        if( aPin.decalName.text == wxS( "PINIEB" ) )
            graphicShape = GRAPHIC_PINSHAPE::INPUT_LOW;

        pin->SetShape( graphicShape );
        pin->SetVisible( aPin.presentation.visible );

        // KiCad pins do not store a font face.  Fit their single scalar text size by comparing
        // representative uppercase PADS text with KiCad's stroke font at the same nominal size.
        pin->SetNameTextSize( fittedPinTextSize( PADS_PIN_NAME_CALIBRATION, aPin.namePresentation ) );
        pin->SetNumberTextSize( fittedPinTextSize( PADS_PIN_NUMBER_CALIBRATION,
                                                  aPin.numberPresentation ) );

        return pin;
    }


    void addPinStem( const MODEL_PIN_DEFINITION& aPin, LIB_SYMBOL* aParent, int aUnit )
    {
        if( !aPin.pinDecalGeometryResolved )
            return;

        const SOURCE_POINT stemVector = padsTerminalStemVector( aPin );
        const VECTOR2I     start = localPoint( aPin.position );
        const VECTOR2I     delta( toIU( stemVector.x ), -toIU( stemVector.y ) );

        if( delta.x == 0 && delta.y == 0 )
            return;

        auto stem = std::make_unique<SCH_SHAPE>( SHAPE_T::POLY, LAYER_DEVICE );
        stem->AddPoint( start );
        stem->AddPoint( start + delta );
        stem->SetStroke( STROKE_PARAMS( toIU( aPin.pinDecalStrokeWidth ),
                                        lineStyle( aPin.pinDecalLineStyle ) ) );
        stem->SetUnit( aUnit );
        aParent->AddDrawItem( stem.release() );
    }

    struct PADS_PIN_NAME_CANDIDATE
    {
        const MODEL_PIN_DEFINITION* pin = nullptr;
        wxString                    text;
        int                         unit = 0;
    };


    VECTOR2I padsTerminalNameOffset( const MODEL_PIN_DEFINITION& aPin )
    {
        const double radians = static_cast<double>( aPin.angle ) * M_PI / 1800.0;
        const double c = std::cos( radians );
        const double s = std::sin( radians );

        double x = static_cast<double>( aPin.nameOffset.x ) * c
                   - static_cast<double>( aPin.nameOffset.y ) * s;
        double y = static_cast<double>( aPin.nameOffset.x ) * s
                   + static_cast<double>( aPin.nameOffset.y ) * c;

        if( ( aPin.presentationFlags & 0x0002 ) != 0 )
            x = -x;

        if( ( aPin.presentationFlags & 0x0004 ) != 0 )
            y = -y;

        return { toIU( static_cast<int64_t>( std::llround( x ) ) ),
                 -toIU( static_cast<int64_t>( std::llround( y ) ) ) };
    }


    std::unique_ptr<SCH_TEXT> makePadsPinNameText(
            const PADS_PIN_NAME_CANDIDATE& aCandidate,
            std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        const MODEL_PIN_DEFINITION& pin = *aCandidate.pin;

        auto text = std::make_unique<SCH_TEXT>(
                localPoint( pin.position ) + padsTerminalNameOffset( pin ),
                aCandidate.text, LAYER_DEVICE );

        text->SetUnit( aCandidate.unit );
        text->SetTextAngle( EDA_ANGLE( pin.nameAngle, TENTHS_OF_A_DEGREE_T ) );

        const int textSize =
                fittedPinTextSize( PADS_PIN_NAME_CALIBRATION, pin.namePresentation );

        if( textSize > 0 )
            text->SetTextSize( { textSize, textSize } );

        if( pin.namePresentation.width > 0 )
            text->SetTextThickness( toIU( pin.namePresentation.width ) );

        text->SetBold( pin.namePresentation.bold );
        text->SetItalic( pin.namePresentation.italic );

        GR_TEXT_H_ALIGN_T hJustify = GR_TEXT_H_ALIGN_LEFT;
        GR_TEXT_V_ALIGN_T vJustify = GR_TEXT_V_ALIGN_BOTTOM;

        PADS_COMMON::DecodeNineWayJustification(
                pin.nameJustification, hJustify, vJustify );

        if( ( pin.presentationFlags & 0x0002 ) != 0 )
        {
            if( hJustify == GR_TEXT_H_ALIGN_LEFT )
                hJustify = GR_TEXT_H_ALIGN_RIGHT;
            else if( hJustify == GR_TEXT_H_ALIGN_RIGHT )
                hJustify = GR_TEXT_H_ALIGN_LEFT;
        }

        if( ( pin.presentationFlags & 0x0004 ) != 0 )
        {
            if( vJustify == GR_TEXT_V_ALIGN_TOP )
                vJustify = GR_TEXT_V_ALIGN_BOTTOM;
            else if( vJustify == GR_TEXT_V_ALIGN_BOTTOM )
                vJustify = GR_TEXT_V_ALIGN_TOP;
        }

        text->SetHorizJustify( hJustify );
        text->SetVertJustify( vJustify );

        if( pin.namePresentation.underline )
        {
            aDiagnostics.push_back(
                    MakePropertyDiagnostic(
                            RPT_SEVERITY_WARNING, pin.namePresentation.source,
                            wxS( "underline" ), PROPERTY_DISPOSITION::UNSUPPORTED,
                            wxS( "PADS pin-name underline is unsupported" ) ) );
        }

        return text;
    }


    struct MODEL_INDEX
    {
        using POINT_KEY = std::tuple<uint32_t, int64_t, int64_t>;
        using SHEET_POINT_KEY = std::tuple<uint32_t, int64_t, int64_t>;

        explicit MODEL_INDEX( const PADS_SCH_MODEL& aModel )
        {
            for( const MODEL_PLACEMENT& placement : aModel.placements )
                placementsBySheet[placement.sheet.id].push_back( &placement );

            for( const MODEL_NET& net : aModel.nets )
            {
                netsById.emplace( net.id, &net );
                netsBySheet[net.sheet.id].push_back( &net );

                for( const MODEL_CONNECTION& connection : net.connections )
                {
                    for( const MODEL_CONNECTION_ENDPOINT& endpoint : connection.endpoints )
                    {
                        if( endpoint.kind == MODEL_ENDPOINT_KIND::PIN )
                        {
                            connectedPinPointsBySheet[net.sheet.id].insert(
                                    { endpoint.point.x, endpoint.point.y } );
                        }
                    }

                    if( connection.vertices.size() < 2 )
                        continue;

                    endpointAdjacency[{ net.id.Value(), connection.vertices.front().x, connection.vertices.front().y }]
                            .push_back( connection.vertices[1] );
                    endpointAdjacency[{ net.id.Value(), connection.vertices.back().x, connection.vertices.back().y }]
                            .push_back( connection.vertices[connection.vertices.size() - 2] );

                    endpointAdjacencyBySheet[
                            { net.sheet.id.Value(), connection.vertices.front().x,
                              connection.vertices.front().y } ]
                            .push_back( connection.vertices[1] );
                    endpointAdjacencyBySheet[
                            { net.sheet.id.Value(), connection.vertices.back().x,
                              connection.vertices.back().y } ]
                            .push_back( connection.vertices[connection.vertices.size() - 2] );
                }
            }

            for( const MODEL_BUS& bus : aModel.buses )
                busesBySheet[bus.sheet.id].push_back( &bus );

            for( const MODEL_LABEL& label : aModel.labels )
                labelsBySheet[label.sheet.id].push_back( &label );

            for( const MODEL_JUNCTION& junction : aModel.junctions )
                junctionsBySheet[junction.sheet.id].push_back( &junction );

            for( const MODEL_TEXT& text : aModel.texts )
                textsBySheet[text.sheet.id].push_back( &text );

            for( const MODEL_PAGE_GRAPHIC& graphic : aModel.graphics )
                graphicsBySheet[graphic.sheet.id].push_back( &graphic );

            for( const MODEL_EMBEDDED_IMAGE& image : aModel.images )
                imagesBySheet[image.sheet.id].push_back( &image );

            for( const MODEL_SYMBOL_DEFINITION& definition : aModel.definitions )
            {
                definitionsById.emplace( definition.id, &definition );
                std::map<PIN_ID, const MODEL_PIN_DEFINITION*>& definitionPins = pinsByDefinition[definition.id];

                for( const MODEL_PIN_DEFINITION& pin : definition.pins )
                    definitionPins.emplace( pin.id, &pin );
            }

            for( const MODEL_PART_TYPE& partType : aModel.partTypes )
                partTypesById.emplace( partType.id, &partType );
        }

        template <typename T>
        static const std::vector<const T*>& ForSheet( const std::map<SHEET_ID, std::vector<const T*>>& aMap,
                                                      SHEET_ID                                         aSheet )
        {
            static const std::vector<const T*> empty;
            auto                               found = aMap.find( aSheet );
            return found == aMap.end() ? empty : found->second;
        }

        std::map<SHEET_ID, std::vector<const MODEL_PLACEMENT*>>      placementsBySheet;
        std::map<SHEET_ID, std::vector<const MODEL_NET*>>            netsBySheet;
        std::map<SHEET_ID, std::vector<const MODEL_BUS*>>            busesBySheet;
        std::map<SHEET_ID, std::vector<const MODEL_LABEL*>>          labelsBySheet;
        std::map<SHEET_ID, std::vector<const MODEL_JUNCTION*>>       junctionsBySheet;
        std::map<SHEET_ID, std::vector<const MODEL_TEXT*>>           textsBySheet;
        std::map<SHEET_ID, std::vector<const MODEL_PAGE_GRAPHIC*>>   graphicsBySheet;
        std::map<SHEET_ID, std::vector<const MODEL_EMBEDDED_IMAGE*>> imagesBySheet;
        std::map<NET_ID, const MODEL_NET*>                           netsById;
        std::map<POINT_KEY, std::vector<SOURCE_POINT>>               endpointAdjacency;
        std::map<SHEET_POINT_KEY, std::vector<SOURCE_POINT>>         endpointAdjacencyBySheet;
        std::map<SHEET_ID, std::set<std::pair<int64_t, int64_t>>>    connectedPinPointsBySheet;

        std::map<DEFINITION_ID, const MODEL_SYMBOL_DEFINITION*>                definitionsById;
        std::map<PART_TYPE_ID, const MODEL_PART_TYPE*>                         partTypesById;
        std::map<DEFINITION_ID, std::map<PIN_ID, const MODEL_PIN_DEFINITION*>> pinsByDefinition;
    };


    const MODEL_SYMBOL_DEFINITION& definitionById( const MODEL_INDEX& aIndex, DEFINITION_ID aId )
    {
        auto definition = aIndex.definitionsById.find( aId );

        if( definition == aIndex.definitionsById.end() )
            THROW_IO_ERROR( wxS( "resolved definition is missing during schematic staging" ) );

        return *definition->second;
    }


    const MODEL_PART_TYPE& partById( const MODEL_INDEX& aIndex, PART_TYPE_ID aId )
    {
        auto part = aIndex.partTypesById.find( aId );

        if( part == aIndex.partTypesById.end() )
            THROW_IO_ERROR( wxS( "resolved part type is missing during schematic staging" ) );

        return *part->second;
    }


    const MODEL_PIN_DEFINITION& pinById( const MODEL_INDEX& aIndex, const MODEL_SYMBOL_DEFINITION& aDefinition,
                                         PIN_ID aId )
    {
        auto definitionPins = aIndex.pinsByDefinition.find( aDefinition.id );

        if( definitionPins == aIndex.pinsByDefinition.end() )
            THROW_IO_ERROR( wxS( "resolved pin is missing during schematic staging" ) );

        auto pin = definitionPins->second.find( aId );

        if( pin == definitionPins->second.end() )
            THROW_IO_ERROR( wxS( "resolved pin is missing during schematic staging" ) );

        return *pin->second;
    }


    void addDefinitionUnit(
            const MODEL_INDEX& aIndex,
            LIB_SYMBOL* aLibrary,
            const MODEL_SYMBOL_DEFINITION& aDefinition,
            const MODEL_GATE* aGate,
            int aUnit,
            std::vector<PARSER_DIAGNOSTIC>& aDiagnostics,
            std::vector<PADS_PIN_NAME_CANDIDATE>& aPinNameCandidates,
            const std::vector<PLACED_PIN_REFERENCE>* aPlacementPins = nullptr,
            const MODEL_CONNECTOR_PIN* aConnectorPin = nullptr )
    {
        for( const MODEL_GRAPHIC& graphic : aDefinition.graphics )
        {
            if( graphic.kind == MODEL_GRAPHIC_KIND::TEXT )
            {
                std::unique_ptr<SCH_TEXT> text = makeGraphicText( graphic, aDiagnostics );
                text->SetUnit( aUnit );
                aLibrary->AddDrawItem( text.release() );
            }
            else
            {
                std::unique_ptr<SCH_SHAPE> shape = makeShape( graphic, false, 0 );
                shape->SetUnit( aUnit );
                aLibrary->AddDrawItem( shape.release() );
            }
        }

        auto addPin = [&]( const PIN_REFERENCE& aPinReference, size_t aPinOrdinal )
        {
            const MODEL_PIN_DEFINITION& sourcePin =
                    pinById( aIndex, aDefinition, aPinReference.id );

            std::unique_ptr<SCH_PIN> pin = makePin( sourcePin, aLibrary );
            wxString                 displayName = sourcePin.name.text;

            if( aGate && aPinOrdinal < aGate->logicalPins.size() )
            {
                const MODEL_GATE_PIN& logicalPin = aGate->logicalPins[aPinOrdinal];
                pin->SetNumber( logicalPin.number.text );
                displayName = logicalPin.name.text;
                pin->SetName( displayName );
                pin->SetType( pinType( logicalPin.electricalType ) );
            }

            if( aConnectorPin )
            {
                pin->SetNumber( aConnectorPin->number.text );
                displayName = aConnectorPin->name.text;
                pin->SetName( displayName );
                pin->SetType( pinType( aConnectorPin->electricalType ) );
            }

            if( !displayName.IsEmpty() && sourcePin.namePresentation.visible )
                aPinNameCandidates.push_back( { &sourcePin, displayName, aUnit } );

            pin->SetUnit( aUnit );
            aLibrary->AddDrawItem( pin.release() );
        };

        if( aPlacementPins && !aPlacementPins->empty() )
        {
            for( size_t pinOrdinal = 0; pinOrdinal < aPlacementPins->size(); ++pinOrdinal )
                addPin( ( *aPlacementPins )[pinOrdinal], pinOrdinal );
        }
        else if( aGate && !aGate->pins.empty() )
        {
            for( size_t pinOrdinal = 0; pinOrdinal < aGate->pins.size(); ++pinOrdinal )
                addPin( aGate->pins[pinOrdinal], pinOrdinal );
        }
        else
        {
            for( const MODEL_PIN_DEFINITION& sourcePin : aDefinition.pins )
            {
                std::unique_ptr<SCH_PIN> pin = makePin( sourcePin, aLibrary );

                if( aConnectorPin )
                {
                    pin->SetNumber( aConnectorPin->number.text );
                    pin->SetName( aConnectorPin->name.text );
                    pin->SetType( pinType( aConnectorPin->electricalType ) );
                }

                pin->SetUnit( aUnit );
                aLibrary->AddDrawItem( pin.release() );
                addPinStem( sourcePin, aLibrary, aUnit );
            }
        }
    }


    std::unique_ptr<LIB_SYMBOL> makeLibrarySymbol( const MODEL_INDEX& aIndex, const MODEL_PLACEMENT& aPlacement,
                                                   std::vector<PARSER_DIAGNOSTIC>& aDiagnostics, wxString& aReference,
                                                   int& aUnit )
    {
        const MODEL_PART_TYPE& part = partById( aIndex, aPlacement.partType.id );
        wxString               libraryName = part.name.text;
        auto                   library = std::make_unique<LIB_SYMBOL>( libraryName );
        const MODEL_GATE*      connectorGate = nullptr;
        std::vector<PADS_PIN_NAME_CANDIDATE> pinNameCandidates;

        for( const MODEL_GATE& gate : part.gates )
        {
            if( !gate.connectorPins.empty() )
            {
                connectorGate = &gate;
                break;
            }
        }

        if( connectorGate )
        {
            const MODEL_SYMBOL_DEFINITION& definition = definitionById( aIndex, aPlacement.definition.id );
            library->SetUnitCount( static_cast<int>( connectorGate->connectorPins.size() ), false );
            library->LockUnits( true );
            aReference = aPlacement.reference.text.BeforeLast( '-' );

            if( aReference.IsEmpty() )
                aReference = aPlacement.reference.text;

            aUnit = static_cast<int>( aPlacement.unit );

            for( size_t index = 0; index < connectorGate->connectorPins.size(); ++index )
            {
                const MODEL_CONNECTOR_PIN& connectorPin = connectorGate->connectorPins[index];
                addDefinitionUnit(aIndex,
                        library.get(),
                        definition,
                        connectorGate,
                        static_cast<int>( index + 1 ),
                        aDiagnostics,
                        pinNameCandidates,
                        &aPlacement.pins,
                        &connectorPin);

                if( aPlacement.reference.text.EndsWith( wxS( "-" ) + connectorPin.number.text ) )
                    aUnit = static_cast<int>( index + 1 );
            }
        }
        else if( part.gates.size() > 1 )
        {
            library->SetUnitCount( static_cast<int>( part.gates.size() ), false );
            library->LockUnits( true );
            aReference = aPlacement.reference.text.BeforeLast( '-' );

            if( aReference.IsEmpty() )
                aReference = aPlacement.reference.text;

            for( const MODEL_GATE& gate : part.gates )
            {
                const DEFINITION_ID definitionId =
                        gate.unit == aPlacement.unit ? aPlacement.definition.id : gate.definition.id;
                const MODEL_SYMBOL_DEFINITION&           definition = definitionById( aIndex, definitionId );
                const std::vector<PLACED_PIN_REFERENCE>* placementPins =
                        gate.unit == aPlacement.unit ? &aPlacement.pins : nullptr;
                addDefinitionUnit(aIndex,
                        library.get(),
                        definition,
                        &gate,
                        static_cast<int>( gate.unit ),
                        aDiagnostics,
                        pinNameCandidates,
                        placementPins);
            }
        }
        else
        {
            const MODEL_SYMBOL_DEFINITION& definition = definitionById( aIndex, aPlacement.definition.id );
            const MODEL_GATE*              gate = part.gates.empty() ? nullptr : &part.gates.front();
            addDefinitionUnit(aIndex,
                    library.get(),
                    definition,
                    gate,
                    0,
                    aDiagnostics,
                    pinNameCandidates,
                    &aPlacement.pins);
        }

        for( const MODEL_SIGNAL_PIN& signalPin : part.signalPins )
        {
            auto pin = std::make_unique<SCH_PIN>( library.get() );
            pin->SetNumber( signalPin.number.text );
            pin->SetName( signalPin.name.text );
            pin->SetType( ELECTRICAL_PINTYPE::PT_POWER_IN );
            pin->SetVisible( false );
            pin->SetLength( 0 );
            library->AddDrawItem( pin.release() );
        }

        // KiCad native pin names cannot reproduce arbitrary PADS per-pin
        // offsets. Use explicit SCH_TEXT only when every named/visible pin uses
        // a vertical pin decal. This covers DIO-V (D6 K/A1/A2) conservatively.
        const bool usePadsPinNamePlacement =
                !pinNameCandidates.empty()
                && std::ranges::all_of(
                        pinNameCandidates,
                        []( const PADS_PIN_NAME_CANDIDATE& aCandidate )
                        {
                            return aCandidate.pin
                                   && aCandidate.pin->decalName.text.Contains( wxS( "VRT" ) );
                        } );

        if( usePadsPinNamePlacement )
        {
            for( const PADS_PIN_NAME_CANDIDATE& candidate : pinNameCandidates )
            {
                std::unique_ptr<SCH_TEXT> pinName =
                        makePadsPinNameText( candidate, aDiagnostics );
                library->AddDrawItem( pinName.release() );
            }
        }

        library->SetShowPinNames( aPlacement.pinNamesVisible && !usePadsPinNamePlacement );
        library->SetShowPinNumbers( aPlacement.pinNumbersVisible );
        return library;
    }


    void applyField( SCH_SYMBOL* aSymbol, const MODEL_FIELD& aSource,
                     std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        SCH_FIELD* field = nullptr;

        if( aSource.name.text.CmpNoCase( wxS( "REF-DES" ) ) == 0 )
            field = aSymbol->GetField( FIELD_T::REFERENCE );
        else if( aSource.name.text.CmpNoCase( wxS( "PART-TYPE" ) ) == 0 )
        {
            const wxString fieldName = wxString::FromUTF8( PADS_ATTRIBUTE_MAPPER::FIELD_PART_TYPE );
            field = aSymbol->GetField( fieldName );

            if( !field )
            {
                SCH_FIELD newField( aSymbol, FIELD_T::USER, fieldName );
                aSymbol->AddField( newField );
                field = aSymbol->GetField( fieldName );
            }
        }
        else if( aSource.name.text.CmpNoCase( wxS( "VALUE" ) ) == 0 )
        {
            field = aSymbol->GetField( FIELD_T::VALUE );
        }
        else
        {
            field = aSymbol->GetField( aSource.name.text );

            if( !field )
            {
                SCH_FIELD newField( aSymbol, FIELD_T::USER, aSource.name.text );
                aSymbol->AddField( newField );
                field = aSymbol->GetField( aSource.name.text );
            }
        }

        if( !field )
            THROW_IO_ERROR( FormatParserError( aSource.source, wxS( "could not stage symbol field" ) ) );

        field->SetText( aSource.value.text );

        // Placement-field coordinates are local to the PADS CAEDECAL.  SCH_FIELD::SetPosition()
        // accepts a sheet coordinate and converts it back through the parent symbol, so first
        // transform the authored local offset into the placed symbol's sheet orientation.
        VECTOR2I fieldOffset =
                aSymbol->GetTransform().TransformCoordinate( localPoint( aSource.position ) );
        field->SetPosition( aSymbol->GetPosition() + fieldOffset );
        field->SetTextAngle( EDA_ANGLE( aSource.angle, TENTHS_OF_A_DEGREE_T ) );
        MODEL_TEXT_PRESENTATION fieldPresentation = aSource.presentation;

        // PADS stores a system-font field height as the Windows character-cell height,
        // whereas KiCad's text size behaves as the visual/em height.  Applying the raw
        // 100 mil value makes REF-DES and VALUE about one third too large.  Keep the
        // parser's whole-mil -> half-mil conversion exact, and adapt only system-font
        // symbol fields here.  Stroke-font fields and SCH_PIN fitting are unaffected.
        // Centralized in applyTextPresentation(): fields, free text, symbol text
        // and labels now share the same PADS outline-font height semantics.
        applyTextPresentation( field, fieldPresentation,
                               aSource.visible && fieldPresentation.visible, aDiagnostics );
    }


    /**
     * Give the symbol a name no other content shares, and return that name. Placements of one part
     * type that differ - a hidden pin number, a different gate's pins - are different symbols.
     */
    wxString resolveLibrarySymbolName( LIBRARY_SYMBOL_VARIANTS& aVariantsByName, LIB_SYMBOL* aSymbol )
    {
        const int compareFlags = ~( LIB_SYMBOL::COMPARE_FLAGS::UNIT | LIB_SYMBOL::COMPARE_FLAGS::UUID );
        auto&     variants = aVariantsByName[aSymbol->GetName()];

        for( const auto& [candidate, resolvedName] : variants )
        {
            if( candidate->Compare( *aSymbol, compareFlags ) == 0 )
                return resolvedName;
        }

        wxString resolved = aSymbol->GetName();

        if( !variants.empty() )
            resolved = wxString::Format( wxS( "%s_%zu" ), resolved, variants.size() );

        variants.emplace_back( std::make_unique<LIB_SYMBOL>( *aSymbol ), resolved );
        return resolved;
    }


    std::unique_ptr<SCH_SYMBOL> makeSymbol( LIBRARY_SYMBOL_VARIANTS& aVariantsByName, const MODEL_INDEX& aIndex,
                                            const MODEL_PLACEMENT& aPlacement, const SCH_SHEET_PATH& aPath,
                                            int aPageHeight, std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        wxString                    reference = aPlacement.reference.text;
        int                         unit = static_cast<int>( aPlacement.unit );
        std::unique_ptr<LIB_SYMBOL> library = makeLibrarySymbol( aIndex, aPlacement, aDiagnostics, reference, unit );
        const wxString              libraryName = resolveLibrarySymbolName( aVariantsByName, library.get() );

        library->SetName( libraryName );

        auto   symbol = std::make_unique<SCH_SYMBOL>();
        LIB_ID libId;
        libId.SetLibNickname( wxS( "pads_import" ) );
        libId.SetLibItemName( libraryName );
        symbol->SetLibId( libId );
        symbol->SetExcludedFromBoard( library->GetGraphicalPins( ALL_UNITS, ALL_BODY_STYLES ).empty() );
        symbol->SetLibSymbol( library.release() );
        symbol->SetPosition( pagePoint( aPlacement.position, aPageHeight ) );

        const int angle = NormalizeAngle( aPlacement.angle );
        int       orientation = SYM_ORIENT_0;

        switch( angle )
        {
        case 900:  orientation = SYM_ORIENT_90;  break;
        case 1800: orientation = SYM_ORIENT_180; break;
        case 2700: orientation = SYM_ORIENT_270; break;
        default:                                 break;
        }

        if( aPlacement.mirrorFlags & 1 )
            orientation |= SYM_MIRROR_Y;

        if( aPlacement.mirrorFlags & 2 )
            orientation |= SYM_MIRROR_X;

        symbol->SetOrientation( orientation );

        symbol->SetUnit( unit );
        symbol->SetRef( &aPath, reference );
        symbol->AddHierarchicalReference( aPath.Path(), reference, unit );

        // PART-TYPE is an inline placement field.  Value, when present, is a
        // separate component attribute/custom field.  Do not use PART-TYPE as
        // a fallback for an absent Value.
        symbol->SetValueFieldText( wxEmptyString, &aPath );

        for( const MODEL_FIELD& field : aPlacement.fields )
            applyField( symbol.get(), field, aDiagnostics );

        return symbol;
    }


    PAGE_INFO pageInfo( const MODEL_SHEET& aSheet )
    {
        PAGE_INFO page;
        page.SetWidthMils( static_cast<int>( aSheet.pageSize.x / 2 ) );
        page.SetHeightMils( static_cast<int>( aSheet.pageSize.y / 2 ) );
        return page;
    }


    void applyTitleBlock( SCH_SCREEN* aScreen, const MODEL_SHEET& aSheet )
    {
        TITLE_BLOCK title;
        title.SetTitle( aSheet.title.text );

        static const std::map<wxString, int> commentFields = {
            { wxS( "designed" ), 0 },   { wxS( "des date" ), 1 },       { wxS( "drawn by" ), 2 },
            { wxS( "checked by" ), 3 }, { wxS( "checked date" ), 4 },   { wxS( "approved" ), 5 },
            { wxS( "app date" ), 6 },   { wxS( "drawing number" ), 7 }, { wxS( "scale" ), 8 }
        };
        std::array<bool, 9>             reservedComments{};
        std::vector<const MODEL_FIELD*> fallbackComments;
        wxString                        companyName;

        for( const MODEL_FIELD& field : aSheet.titleBlockFields )
        {
            const wxString name = field.name.text.Lower();

            if( name == wxS( "title" ) )
                title.SetTitle( field.value.text );
            else if( name == wxS( "revision" ) )
                title.SetRevision( field.value.text );
            else if( name == wxS( "date" ) || name == wxS( "drawn date" ) )
                title.SetDate( field.value.text );
            else if( name == wxS( "company name" ) )
                companyName = field.value.text;
            else if( auto comment = commentFields.find( name ); comment != commentFields.end() )
            {
                reservedComments[comment->second] = true;
                title.SetComment( comment->second, field.value.text );
            }
            else
            {
                fallbackComments.push_back( &field );
            }
        }

        title.SetCompany( companyName );

        auto fallback = fallbackComments.begin();

        for( size_t comment = 0; comment < reservedComments.size() && fallback != fallbackComments.end(); ++comment )
        {
            if( !reservedComments[comment] )
                title.SetComment( comment, ( *fallback++ )->value.text );
        }

        aScreen->SetTitleBlock( title );
    }


    std::string quoteWorksheetText( const wxString& aText )
    {
        std::string result;

        for( char character : std::string( aText.utf8_str() ) )
        {
            if( character == '\\' || character == '"' )
                result.push_back( '\\' );

            if( character == '\n' )
                result += "\\n";
            else if( character != '\r' )
                result.push_back( character );
        }

        return result;
    }


    std::string worksheetVariable( const wxString& aText, const MODEL_SHEET& aSheet )
    {
        static const std::map<wxString, std::string> variables = {
            { wxS( "Title" ), "${TITLE}" },             { wxS( "Revision" ), "${REVISION}" },
            { wxS( "Drawn Date" ), "${ISSUE_DATE}" },   { wxS( "Company Name" ), "${COMPANY}" },
            { wxS( "Designed" ), "${COMMENT1}" },       { wxS( "Des Date" ), "${COMMENT2}" },
            { wxS( "Drawn By" ), "${COMMENT3}" },       { wxS( "Checked By" ), "${COMMENT4}" },
            { wxS( "Checked Date" ), "${COMMENT5}" },   { wxS( "Approved" ), "${COMMENT6}" },
            { wxS( "App Date" ), "${COMMENT7}" },       { wxS( "Drawing Number" ), "${COMMENT8}" },
            { wxS( "Scale" ), "${COMMENT9}" },          { wxS( "Sheet Number" ), "${#}" },
            { wxS( "Number of Sheets" ), "${##}" },     { wxS( "Sheet Name" ), "${SHEETNAME}" },
            { wxS( "Sheet Size" ), "${PAPER}" }
        };

        if( auto found = variables.find( aText ); found != variables.end() )
            return found->second;

        for( const MODEL_FIELD& field : aSheet.titleBlockFields )
        {
            if( field.name.text == aText )
                return quoteWorksheetText( field.value.text );
        }

        return quoteWorksheetText( aText );
    }


    double worksheetX( const SOURCE_POINT& aPoint, const SOURCE_POINT& aPageSize )
    {
        return static_cast<double>( aPageSize.x - aPoint.x ) * 0.0127;
    }


    double worksheetY( const SOURCE_POINT& aPoint )
    {
        return static_cast<double>( aPoint.y ) * 0.0127;
    }


    void appendWorksheetLine( std::ostringstream& aOutput, const SOURCE_POINT& aStart, const SOURCE_POINT& aEnd,
                              const SOURCE_POINT& aPageSize, int64_t aWidth )
    {
        aOutput << "  (line (name \"\") (start " << worksheetX( aStart, aPageSize ) << ' ' << worksheetY( aStart )
                << ") (end " << worksheetX( aEnd, aPageSize ) << ' ' << worksheetY( aEnd ) << ')';

        if( aWidth > 0 )
            aOutput << " (linewidth " << static_cast<double>( aWidth ) * 0.0127 << ')';

        aOutput << ")\n";
    }


    std::string serializeWorksheet( const MODEL_WORKSHEET& aWorksheet, const MODEL_SHEET& aSheet )
    {
        std::ostringstream output;
        output.imbue( std::locale::classic() );
        output.precision( 8 );
        output << "(kicad_wks (version 20220228) (generator pads_import)\n"
                  "  (setup (textsize 1.27 1.27) (linewidth 0) (textlinewidth 0)"
                  " (left_margin 0) (right_margin 0) (top_margin 0) (bottom_margin 0))\n";

        for( const MODEL_GRAPHIC& graphic : aWorksheet.graphics )
        {
            if( graphic.kind == MODEL_GRAPHIC_KIND::TEXT )
            {
                if( graphic.points.empty() || !graphic.presentation.visible )
                    continue;

                const int64_t worksheetVisualHeight = padsVisualTextHeight( graphic.presentation );
                KIFONT::FONT* worksheetFont = resolvePadsPresentationFont( graphic.presentation );
                double worksheetTextY = worksheetY( graphic.points.front() );

                // PADS positions top/bottom justified outline text against the
                // original Windows character-cell height.  padsVisualTextHeight()
                // deliberately shrinks that height to match KiCad's visual/em
                // size, so compensate the anchor by half the removed padding.
                // Center-justified text needs no shift.
                //
                // Keep the correction to unrotated worksheet text until rotated
                // template anchor directions are independently verified.
                if( worksheetFont && !worksheetFont->IsStroke() && graphic.angle == 0
                    && graphic.presentation.height > worksheetVisualHeight )
                {
                    const double anchorCompensation =
                            static_cast<double>( graphic.presentation.height - worksheetVisualHeight )
                            * 0.0127 / 2.0;

                    if( graphic.presentation.verticalJustification == MODEL_JUSTIFICATION::RIGHT )
                        worksheetTextY += anchorCompensation; // bottom
                    else if( graphic.presentation.verticalJustification == MODEL_JUSTIFICATION::LEFT )
                        worksheetTextY -= anchorCompensation; // top
                }

                output << "  (tbtext \"" << worksheetVariable( graphic.text.text, aSheet )
                       << "\" (name \"\") (pos "
                       << worksheetX( graphic.points.front(), aSheet.pageSize ) << ' '
                       << worksheetTextY;

                if( graphic.angle != 0 )
                    output << ") (rotate " << static_cast<double>( graphic.angle ) / 10.0;

                output << ") (font";
                
                if( worksheetFont && !worksheetFont->IsStroke() )
                    output << " (face \"" << quoteWorksheetText( worksheetFont->GetName() ) << "\")";
                
                output << " (size " << static_cast<double>( worksheetVisualHeight ) * 0.0127 << ' '
                       << static_cast<double>( worksheetVisualHeight ) * 0.0127 << ')';

                if( graphic.presentation.width > 0 )
                    output << " (linewidth " << static_cast<double>( graphic.presentation.width ) * 0.0127 << ')';

                if( graphic.presentation.bold )
                    output << " bold";

                if( graphic.presentation.italic )
                    output << " italic";

                output << ')';

                if( graphic.presentation.horizontalJustification != MODEL_JUSTIFICATION::LEFT
                    || graphic.presentation.verticalJustification != MODEL_JUSTIFICATION::CENTER )
                {
                    output << " (justify";

                    if( graphic.presentation.horizontalJustification == MODEL_JUSTIFICATION::CENTER )
                        output << " center";
                    else if( graphic.presentation.horizontalJustification == MODEL_JUSTIFICATION::RIGHT )
                        output << " right";

                    if( graphic.presentation.verticalJustification == MODEL_JUSTIFICATION::LEFT )
                        output << " top";
                    else if( graphic.presentation.verticalJustification == MODEL_JUSTIFICATION::RIGHT )
                        output << " bottom";

                    output << ')';
                }

                output << ")\n";
                continue;
            }

            if( graphic.kind == MODEL_GRAPHIC_KIND::CIRCLE && graphic.points.size() >= 2 )
            {
                const SOURCE_POINT& first = graphic.points[0];
                const SOURCE_POINT& second = graphic.points[1];
                const double        centerX = ( static_cast<double>( first.x ) + second.x ) / 2.0;
                const double        centerY = ( static_cast<double>( first.y ) + second.y ) / 2.0;
                const double        radius = std::hypot( static_cast<double>( second.x - first.x ),
                                                         static_cast<double>( second.y - first.y ) )
                                      / 2.0;
                constexpr int segments = 64;

                for( int segment = 0; segment < segments; ++segment )
                {
                    const double firstAngle = 2.0 * M_PI * segment / segments;
                    const double secondAngle = 2.0 * M_PI * ( segment + 1 ) / segments;
                    SOURCE_POINT start{ KiROUND( centerX + radius * std::cos( firstAngle ) ),
                                        KiROUND( centerY + radius * std::sin( firstAngle ) ), graphic.source };
                    SOURCE_POINT end{ KiROUND( centerX + radius * std::cos( secondAngle ) ),
                                      KiROUND( centerY + radius * std::sin( secondAngle ) ), graphic.source };
                    appendWorksheetLine( output, start, end, aSheet.pageSize, graphic.strokeWidth );
                }

                continue;
            }

            if( graphic.kind == MODEL_GRAPHIC_KIND::ARC )
            {
                const double startAngle =
                        std::atan2( static_cast<double>( graphic.arcBoundsStart.y - graphic.arcCenter.y ),
                                    static_cast<double>( graphic.arcBoundsStart.x - graphic.arcCenter.x ) );
                const double sweep = static_cast<double>( graphic.arcSweepAngle ) * M_PI / 1800.0
                                     * ( graphic.arcClockwise ? -1.0 : 1.0 );
                const int    segments = std::max( 1, static_cast<int>( std::ceil( std::abs( sweep ) * 16.0 / M_PI ) ) );
                const double radius =
                        std::hypot( static_cast<double>( graphic.arcBoundsStart.x - graphic.arcCenter.x ),
                                    static_cast<double>( graphic.arcBoundsStart.y - graphic.arcCenter.y ) );

                for( int segment = 0; segment < segments; ++segment )
                {
                    const double angle1 = startAngle + sweep * segment / segments;
                    const double angle2 = startAngle + sweep * ( segment + 1 ) / segments;
                    SOURCE_POINT start{ KiROUND( graphic.arcCenter.x + radius * std::cos( angle1 ) ),
                                        KiROUND( graphic.arcCenter.y + radius * std::sin( angle1 ) ), graphic.source };
                    SOURCE_POINT end{ KiROUND( graphic.arcCenter.x + radius * std::cos( angle2 ) ),
                                      KiROUND( graphic.arcCenter.y + radius * std::sin( angle2 ) ), graphic.source };
                    appendWorksheetLine( output, start, end, aSheet.pageSize, graphic.strokeWidth );
                }

                continue;
            }

            if( graphic.kind == MODEL_GRAPHIC_KIND::RECTANGLE && graphic.points.size() >= 2 )
            {
                const SOURCE_POINT topLeft{ graphic.points[0].x, graphic.points[0].y, graphic.source };
                const SOURCE_POINT topRight{ graphic.points[1].x, graphic.points[0].y, graphic.source };
                const SOURCE_POINT bottomRight{ graphic.points[1].x, graphic.points[1].y, graphic.source };
                const SOURCE_POINT bottomLeft{ graphic.points[0].x, graphic.points[1].y, graphic.source };
                appendWorksheetLine( output, topLeft, topRight, aSheet.pageSize, graphic.strokeWidth );
                appendWorksheetLine( output, topRight, bottomRight, aSheet.pageSize, graphic.strokeWidth );
                appendWorksheetLine( output, bottomRight, bottomLeft, aSheet.pageSize, graphic.strokeWidth );
                appendWorksheetLine( output, bottomLeft, topLeft, aSheet.pageSize, graphic.strokeWidth );
                continue;
            }

            for( size_t point = 1; point < graphic.points.size(); ++point )
            {
                appendWorksheetLine( output, graphic.points[point - 1], graphic.points[point], aSheet.pageSize,
                                     graphic.strokeWidth );
            }
        }

        output << ")\n";
        return output.str();
    }


    wxString sanitizedFilename( const wxString& aName, size_t aOrdinal, std::set<wxString>& aUsed )
    {
        wxString stem = aName;

        for( wxUniChar character : wxS( "<>:\"/\\|?*" ) )
            stem.Replace( wxString( character ), wxS( "_" ) );

        stem.Trim( true ).Trim( false );

        if( stem.IsEmpty() )
            stem = wxString::Format( wxS( "pads_sheet_%zu" ), aOrdinal + 1 );

        wxString candidate = stem + wxS( "." ) + FILEEXT::KiCadSchematicFileExtension;
        size_t   suffix = 2;

        while( aUsed.contains( candidate.Lower() ) )
            candidate = wxString::Format( wxS( "%s_%zu.%s" ), stem, suffix++, FILEEXT::KiCadSchematicFileExtension );

        aUsed.insert( candidate.Lower() );
        return candidate;
    }


    struct STAGED_SCHEMATIC
    {
        SCH_SHEET*                              destinationRoot = nullptr;
        std::unique_ptr<SCH_SCREEN>             replacementScreen;
        std::unique_ptr<SCH_SCREEN>             appendCache;
        EE_RTREE                                appendIndex;
        std::vector<std::unique_ptr<SCH_ITEM>>  appendItems;
        std::unique_ptr<SCH_SCREEN>             topLevelCache;
        EE_RTREE                                topLevelIndex;
        std::vector<std::unique_ptr<SCH_SHEET>> topLevelOwners;
        std::vector<SCH_SHEET*>                 topLevelSheets;
        SCH_SHEET_LIST                          hierarchy;
        std::optional<SCH_SHEET_PATH>           replacementCurrentSheet;
        std::unique_ptr<CONNECTION_GRAPH>       connectionGraph;
        std::unique_ptr<EMBEDDED_FILES>         replacementEmbeddedFiles;
        wxString                                replacementDrawingSheet;
        BUILD_RESULT                            result;

        // Runs across every sheet because a per-sheet restart collides on sheet two
        int nextPowerOrdinal = 1;

        LIBRARY_SYMBOL_VARIANTS librarySymbolVariants;

        static void ValidateScreen( const SCH_SCREEN* aScreen )
        {
            if( !aScreen )
                THROW_IO_ERROR( wxS( "staged sheet is missing its screen" ) );

            for( SCH_ITEM* item : aScreen->Items().OfType( SCH_SYMBOL_T ) )
            {
                const SCH_SYMBOL* symbol = static_cast<const SCH_SYMBOL*>( item );

                if( !symbol->GetLibSymbolRef() || symbol->GetLibId().GetLibItemName().empty() )
                    THROW_IO_ERROR( wxS( "staged symbol is missing its library link" ) );
            }
        }

        static void ValidateChildSheet( const SCH_SHEET* aSheet, std::set<wxString>& aFilenames )
        {
            if( !aSheet || !aSheet->GetScreen() )
                THROW_IO_ERROR( wxS( "staged child sheet is missing its screen" ) );

            ValidateScreen( aSheet->GetScreen() );

            wxString filename = aSheet->GetField( FIELD_T::SHEET_FILENAME )->GetText();

            if( filename.IsEmpty() || filename.Contains( wxS( "/" ) ) || filename.Contains( wxS( "\\" ) )
                || filename.Contains( wxS( ":" ) ) || filename.Contains( wxS( "*" ) ) )
            {
                THROW_IO_ERROR( wxS( "staged child sheet has an invalid filename" ) );
            }

            if( !aFilenames.insert( filename.Lower() ).second )
                THROW_IO_ERROR( wxS( "staged child sheet filenames are not unique" ) );
        }

        void Validate( bool aAppending ) const
        {
            if( replacementEmbeddedFiles
                && ( aAppending || replacementDrawingSheet.IsEmpty()
                     || !replacementEmbeddedFiles->HasFile( wxS( "pads_import.kicad_wks" ) ) ) )
            {
                THROW_IO_ERROR( wxS( "embedded worksheet staging is incomplete" ) );
            }

            if( !replacementEmbeddedFiles && !replacementDrawingSheet.IsEmpty() )
                THROW_IO_ERROR( wxS( "worksheet path has no staged embedded file" ) );

            if( topLevelCache )
            {
                if( replacementScreen || appendCache || topLevelOwners.empty()
                    || topLevelOwners.size() != topLevelSheets.size() || topLevelSheets.size() != result.counts.sheets )
                {
                    THROW_IO_ERROR( wxS( "top-level sheet staging has invalid ownership" ) );
                }

                for( const std::unique_ptr<SCH_SHEET>& sheet : topLevelOwners )
                {
                    if( !sheet || !sheet->GetScreen() || !sheet->GetScreen()->Items().OfType( SCH_SHEET_T ).empty() )
                        THROW_IO_ERROR( wxS( "staged top-level sheet has invalid content" ) );

                    ValidateScreen( sheet->GetScreen() );
                }

                if( hierarchy.size() != topLevelSheets.size() || !replacementCurrentSheet || !connectionGraph )
                    THROW_IO_ERROR( wxS( "top-level sheet staging has incomplete hierarchy state" ) );

                return;
            }

            if( aAppending && ( replacementScreen || !appendCache ) )
                THROW_IO_ERROR( wxS( "append staging has invalid screen ownership" ) );

            if( !aAppending && ( !destinationRoot || !replacementScreen || !replacementCurrentSheet ) )
                THROW_IO_ERROR( wxS( "replacement staging has no destination root or screen" ) );

            std::set<wxString> filenames;

            if( replacementScreen )
            {
                ValidateScreen( replacementScreen.get() );

                size_t childCount = 0;

                for( SCH_ITEM* item : replacementScreen->Items().OfType( SCH_SHEET_T ) )
                {
                    ValidateChildSheet( static_cast<SCH_SHEET*>( item ), filenames );
                    ++childCount;
                }

                if( result.counts.sheets > 1 && childCount != result.counts.sheets )
                    THROW_IO_ERROR( wxS( "staged replacement hierarchy does not own every source sheet" ) );

                if( result.counts.sheets == 1 && childCount != 0 )
                    THROW_IO_ERROR( wxS( "single-sheet staging unexpectedly contains child sheets" ) );
            }

            size_t appendedSheetCount = 0;

            for( const std::unique_ptr<SCH_ITEM>& item : appendItems )
            {
                if( !item )
                    THROW_IO_ERROR( wxS( "staged schematic contains an empty object" ) );

                if( item->Type() == SCH_SHEET_T )
                {
                    ValidateChildSheet( static_cast<const SCH_SHEET*>( item.get() ), filenames );
                    ++appendedSheetCount;
                }
                else if( item->Type() == SCH_SYMBOL_T )
                {
                    const SCH_SYMBOL* symbol = static_cast<const SCH_SYMBOL*>( item.get() );

                    if( !symbol->GetLibSymbolRef() || symbol->GetLibId().GetLibItemName().empty() )
                        THROW_IO_ERROR( wxS( "staged symbol is missing its library link" ) );
                }
            }

            if( aAppending && result.counts.sheets > 1 && appendedSheetCount != result.counts.sheets )
                THROW_IO_ERROR( wxS( "staged append hierarchy does not own every source sheet" ) );

            if( hierarchy.empty() || !connectionGraph )
                THROW_IO_ERROR( wxS( "staged schematic has incomplete hierarchy state" ) );
        }

        /// Pack the staged schematic into the form SCHEMATIC::AdoptContent() takes.  Everything
        /// this allocates is still owned off to the side, so a throw here changes nothing.
        SCHEMATIC_CONTENT PackContent( SCH_SHEET* aAppendToMe )
        {
            SCHEMATIC_CONTENT content;
            content.hierarchy = std::move( hierarchy );
            content.currentSheet = std::move( replacementCurrentSheet );
            content.connectionGraph = std::move( connectionGraph );

            if( replacementEmbeddedFiles )
            {
                content.embeddedFiles.emplace( std::move( *replacementEmbeddedFiles ) );
                content.drawingSheetFileName = std::move( replacementDrawingSheet );
            }

            if( replacementScreen )
            {
                content.targetSheet = destinationRoot;
                content.screen = std::move( replacementScreen );
            }
            else if( topLevelCache )
            {
                // The virtual root owns the top-level sheets, so it is the target here.
                content.screenItems = std::move( topLevelIndex );
                content.screenLibSymbols = std::move( topLevelCache );
                content.topLevelSheets = std::move( topLevelSheets );

                for( std::unique_ptr<SCH_SHEET>& sheet : topLevelOwners )
                    content.itemOwners.emplace_back( std::move( sheet ) );
            }
            else
            {
                content.targetSheet = aAppendToMe;
                content.preserveNetChains = true;
                content.screenItems = std::move( appendIndex );
                content.screenLibSymbols = std::move( appendCache );
                content.itemOwners = std::move( appendItems );
            }

            return content;
        }

        void Commit( SCHEMATIC* aSchematic, SCH_SHEET* aAppendToMe, const std::function<void()>& aBeforeCommit )
        {
            SCHEMATIC_CONTENT content = PackContent( aAppendToMe );

            if( aBeforeCommit )
                aBeforeCommit();

            aSchematic->AdoptContent( std::move( content ) );
        }
    };


    void collectDispositionDiagnostics( const std::vector<SOURCE_PROPERTY>& aProperties,
                                        std::vector<PARSER_DIAGNOSTIC>&     aDiagnostics )
    {
        for( const SOURCE_PROPERTY& property : aProperties )
        {
            if( property.disposition == PROPERTY_DISPOSITION::EXACT
                || property.disposition == PROPERTY_DISPOSITION::PRESERVED )
                continue;

            aDiagnostics.push_back( MakePropertyDiagnostic(
                    RPT_SEVERITY_WARNING, property,
                    wxString::Format( wxS( "PADS property '%s' retained with %s disposition" ), property.name.text,
                                      property.disposition == PROPERTY_DISPOSITION::APPROXIMATE
                                              ? wxS( "approximate" )
                                              : wxS( "unsupported" ) ) ) );
        }
    }


    void collectPresentationDiagnostics( const MODEL_TEXT_PRESENTATION&  aPresentation,
                                         std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        collectDispositionDiagnostics( aPresentation.properties, aDiagnostics );
    }


    void validatePropertyDispositions( const PADS_SCH_MODEL& aModel, std::vector<PARSER_DIAGNOSTIC>& aDiagnostics )
    {
        collectDispositionDiagnostics( aModel.settings.properties, aDiagnostics );

        for( const MODEL_SHEET& sheet : aModel.sheets )
        {
            collectDispositionDiagnostics( sheet.properties, aDiagnostics );

            for( const MODEL_GRAPHIC& graphic : sheet.border )
            {
                collectDispositionDiagnostics( graphic.properties, aDiagnostics );
                collectPresentationDiagnostics( graphic.presentation, aDiagnostics );
            }

            for( const MODEL_FIELD& field : sheet.titleBlockFields )
            {
                collectDispositionDiagnostics( field.properties, aDiagnostics );
                collectPresentationDiagnostics( field.presentation, aDiagnostics );
            }
        }

        for( const MODEL_SYMBOL_DEFINITION& definition : aModel.definitions )
        {
            collectDispositionDiagnostics( definition.properties, aDiagnostics );

            for( const MODEL_GRAPHIC& graphic : definition.graphics )
            {
                collectDispositionDiagnostics( graphic.properties, aDiagnostics );
                collectPresentationDiagnostics( graphic.presentation, aDiagnostics );
            }

            for( const MODEL_PIN_DEFINITION& pin : definition.pins )
            {
                collectDispositionDiagnostics( pin.properties, aDiagnostics );
                collectPresentationDiagnostics( pin.presentation, aDiagnostics );
                collectPresentationDiagnostics( pin.namePresentation, aDiagnostics );
                collectPresentationDiagnostics( pin.numberPresentation, aDiagnostics );
            }

            for( const MODEL_FIELD& field : definition.fields )
            {
                collectDispositionDiagnostics( field.properties, aDiagnostics );
                collectPresentationDiagnostics( field.presentation, aDiagnostics );
            }
        }

        for( const MODEL_PART_TYPE& part : aModel.partTypes )
        {
            collectDispositionDiagnostics( part.properties, aDiagnostics );

            for( const MODEL_GATE& gate : part.gates )
                collectDispositionDiagnostics( gate.properties, aDiagnostics );

            for( const MODEL_FIELD& field : part.fields )
            {
                collectDispositionDiagnostics( field.properties, aDiagnostics );
                collectPresentationDiagnostics( field.presentation, aDiagnostics );
            }
        }

        for( const MODEL_PLACEMENT& placement : aModel.placements )
        {
            collectDispositionDiagnostics( placement.properties, aDiagnostics );

            for( const MODEL_FIELD& field : placement.fields )
            {
                collectDispositionDiagnostics( field.properties, aDiagnostics );
                collectPresentationDiagnostics( field.presentation, aDiagnostics );
            }
        }

        for( const MODEL_NET& net : aModel.nets )
        {
            collectDispositionDiagnostics( net.properties, aDiagnostics );

            for( const MODEL_CONNECTION& connection : net.connections )
            {
                collectDispositionDiagnostics( connection.properties, aDiagnostics );

                for( const MODEL_CONNECTION_ENDPOINT& endpoint : connection.endpoints )
                    collectDispositionDiagnostics( endpoint.properties, aDiagnostics );
            }
        }

        for( const MODEL_BUS& bus : aModel.buses )
        {
            collectDispositionDiagnostics( bus.properties, aDiagnostics );

            for( const MODEL_BUS_ENTRY& entry : bus.entries )
                collectDispositionDiagnostics( entry.properties, aDiagnostics );
        }

        for( const MODEL_LABEL& label : aModel.labels )
        {
            collectDispositionDiagnostics( label.properties, aDiagnostics );
            collectPresentationDiagnostics( label.presentation, aDiagnostics );
        }

        for( const MODEL_JUNCTION& junction : aModel.junctions )
            collectDispositionDiagnostics( junction.properties, aDiagnostics );

        for( const MODEL_TEXT& text : aModel.texts )
        {
            collectDispositionDiagnostics( text.properties, aDiagnostics );
            collectPresentationDiagnostics( text.presentation, aDiagnostics );
        }

        for( const MODEL_PAGE_GRAPHIC& graphic : aModel.graphics )
        {
            collectDispositionDiagnostics( graphic.graphic.properties, aDiagnostics );
            collectPresentationDiagnostics( graphic.graphic.presentation, aDiagnostics );
        }

        for( const MODEL_WORKSHEET& worksheet : aModel.worksheets )
        {
            for( const MODEL_GRAPHIC& graphic : worksheet.graphics )
            {
                collectDispositionDiagnostics( graphic.properties, aDiagnostics );
                collectPresentationDiagnostics( graphic.presentation, aDiagnostics );
            }
        }

        for( const MODEL_EMBEDDED_IMAGE& image : aModel.images )
            collectDispositionDiagnostics( image.properties, aDiagnostics );

        for( const PRESERVED_CONTROLLER_PAYLOAD& payload : aModel.preservedControllerPayloads )
        {
            aDiagnostics.push_back( MakePropertyDiagnostic(
                    RPT_SEVERITY_WARNING, payload.source, wxS( "controller_payload" ), payload.disposition,
                    wxS( "PADS controller payload retained without schematic construction" ) ) );
        }

        std::set<DIAGNOSTIC_PROPERTY_KEY> parserOwnedProperties;

        for( const PARSER_DIAGNOSTIC& diagnostic : aModel.diagnostics )
        {
            if( std::optional key = DiagnosticPropertyKey( diagnostic ) )
                parserOwnedProperties.insert( std::move( *key ) );
        }

        std::erase_if( aDiagnostics,
                       [&]( const PARSER_DIAGNOSTIC& aBuilderDiagnostic )
                       {
                           std::optional key = DiagnosticPropertyKey( aBuilderDiagnostic );
                           return key && parserOwnedProperties.contains( *key );
                       } );
    }


    void stageSheetContent( STAGED_SCHEMATIC& aStaged, const MODEL_INDEX& aIndex,
                            const MODEL_SHEET& aSourceSheet, SCH_SCREEN* aScreen, const SCH_SHEET_PATH& aPath )
    {
        PAGE_INFO page = pageInfo( aSourceSheet );
        aScreen->SetPageSettings( page );
        applyTitleBlock( aScreen, aSourceSheet );
        const int pageHeight = page.GetHeightIU( schIUScale.IU_PER_MILS );

        std::set<std::pair<int, int>> connectedPinPoints;
        auto sourceConnected = aIndex.connectedPinPointsBySheet.find( aSourceSheet.id );

        if( sourceConnected != aIndex.connectedPinPointsBySheet.end() )
        {
            for( const auto& [x, y] : sourceConnected->second )
            {
                VECTOR2I point( toIU( x ), pageHeight - toIU( y ) );
                connectedPinPoints.insert( { point.x, point.y } );
            }
        }

        std::set<std::pair<int, int>> noConnectPoints;

        for( const MODEL_PLACEMENT* placement : MODEL_INDEX::ForSheet( aIndex.placementsBySheet, aSourceSheet.id ) )
        {
            std::unique_ptr<SCH_SYMBOL> symbol =
                    makeSymbol( aStaged.librarySymbolVariants, aIndex, *placement, aPath, pageHeight,
                                aStaged.result.diagnostics );
            aScreen->Append( symbol.get() );

            // PADS treats an otherwise unreferenced pin as intentionally unconnected.
            // KiCad requires an explicit marker for the same ERC meaning.
            for( const SCH_PIN* pin : symbol->GetPins( &aPath ) )
            {
                const VECTOR2I position = pin->GetPosition();
                const auto     key = std::make_pair( position.x, position.y );

                if( pin->IsVisible() && !connectedPinPoints.contains( key )
                    && noConnectPoints.insert( key ).second )
                {
                    auto noConnect = std::make_unique<SCH_NO_CONNECT>( position );
                    aScreen->Append( noConnect.get() );
                    noConnect.release();
                }
            }

            symbol.release();
            ++aStaged.result.counts.symbols;
        }

        for( const MODEL_GRAPHIC& graphic : aSourceSheet.border )
        {
            if( graphic.kind == MODEL_GRAPHIC_KIND::TEXT )
            {
                appendPageGraphic( aScreen, graphic, pageHeight, aStaged.result.diagnostics );
                ++aStaged.result.counts.texts;
                continue;
            }

            aStaged.result.counts.graphics +=
                    appendPageGraphic( aScreen, graphic, pageHeight, aStaged.result.diagnostics );
        }

        using SEGMENT_KEY = std::tuple<uint32_t, uint32_t, int64_t, int64_t, int64_t, int64_t>;
        std::set<SEGMENT_KEY> busEntrySegments;

        auto segmentKey = []( SHEET_ID aSheet, NET_ID aNet, const SOURCE_POINT& aStart, const SOURCE_POINT& aEnd )
        {
            if( std::tie( aStart.x, aStart.y ) <= std::tie( aEnd.x, aEnd.y ) )
                return SEGMENT_KEY( aSheet.Value(), aNet.Value(), aStart.x, aStart.y, aEnd.x, aEnd.y );

            return SEGMENT_KEY( aSheet.Value(), aNet.Value(), aEnd.x, aEnd.y, aStart.x, aStart.y );
        };

        for( const MODEL_BUS* bus : MODEL_INDEX::ForSheet( aIndex.busesBySheet, aSourceSheet.id ) )
        {
            if( bus->vertices.size() < 2 )
                THROW_IO_ERROR( FormatParserError( bus->source, wxS( "bus has inconsistent geometry" ) ) );

            for( size_t vertex = 1; vertex < bus->vertices.size(); ++vertex )
            {
                auto line = std::make_unique<SCH_LINE>( pagePoint( bus->vertices[vertex - 1], pageHeight ), LAYER_BUS );
                line->SetEndPoint( pagePoint( bus->vertices[vertex], pageHeight ) );
                line->SetStroke( STROKE_PARAMS( toIU( aSourceSheet.defaultBusWidth ), LINE_STYLE::SOLID ) );
                aScreen->Append( line.get() );
                line.release();
                ++aStaged.result.counts.buses;
            }

            wxString busLabel = bus->name.text;

            if( !bus->declaredMembers.empty() )
            {
                busLabel += wxS( "{" );

                for( size_t member = 0; member < bus->declaredMembers.size(); ++member )
                {
                    if( member )
                        busLabel += wxS( " " );

                    busLabel += bus->declaredMembers[member].text;
                }

                busLabel += wxS( "}" );
            }

            auto label = std::make_unique<SCH_LABEL>( pagePoint( bus->vertices.front(), pageHeight ), busLabel );
            aScreen->Append( label.get() );
            label.release();
            ++aStaged.result.counts.labels;

            for( const MODEL_BUS_ENTRY& entry : bus->entries )
            {
                auto ownerNet = aIndex.netsById.find( entry.memberNet.id );

                if( ownerNet == aIndex.netsById.end() )
                    THROW_IO_ERROR( FormatParserError( entry.memberNet.source,
                                                       wxS( "resolved bus-entry net is missing during staging" ) ) );

                auto adjacency = aIndex.endpointAdjacency.find(
                        { entry.memberNet.id.Value(), entry.position.x, entry.position.y } );

                if( adjacency == aIndex.endpointAdjacency.end() )
                    THROW_IO_ERROR( FormatParserError( entry.source, wxS( "bus-entry geometry is unresolved" ) ) );

                if( adjacency->second.size() != 1 )
                    THROW_IO_ERROR( FormatParserError( entry.source, wxS( "bus-entry geometry is ambiguous" ) ) );

                const VECTOR2I start = pagePoint( entry.position, pageHeight );
                const VECTOR2I end = pagePoint( adjacency->second.front(), pageHeight );
                const VECTOR2I delta = end - start;
                const int      span = std::max( std::abs( delta.x ), std::abs( delta.y ) );
                const int      entrySpan = std::min( span, schIUScale.MilsToIU( DEFAULT_SCH_ENTRY_SIZE ) );
                // A stub longer than a third of an inch overflows this product in 32 bits
                auto clamped = [&]( int aDelta )
                {
                    return static_cast<int>( static_cast<int64_t>( aDelta ) * entrySpan / span );
                };

                const VECTOR2I entryEnd = span == 0 ? end : start + VECTOR2I( clamped( delta.x ), clamped( delta.y ) );
                auto entryItem = std::make_unique<SCH_BUS_WIRE_ENTRY>( start );
                entryItem->SetSize( entryEnd - start );
                aScreen->Append( entryItem.get() );
                entryItem.release();

                if( entryEnd != end )
                {
                    auto wire = std::make_unique<SCH_LINE>( entryEnd, LAYER_WIRE );
                    wire->SetEndPoint( end );
                    wire->SetStroke( STROKE_PARAMS( 0, LINE_STYLE::SOLID ) );
                    aScreen->Append( wire.get() );
                    wire.release();
                    ++aStaged.result.counts.wires;
                }

                busEntrySegments.insert(
                        segmentKey( aSourceSheet.id, entry.memberNet.id, entry.position, adjacency->second.front() ) );
                ++aStaged.result.counts.busEntries;
            }
        }

        for( const MODEL_NET* net : MODEL_INDEX::ForSheet( aIndex.netsBySheet, aSourceSheet.id ) )
        {
            for( const MODEL_CONNECTION& connection : net->connections )
            {
                if( connection.vertices.size() < 2 )
                    THROW_IO_ERROR(
                            FormatParserError( connection.source, wxS( "connection has inconsistent geometry" ) ) );

                for( size_t vertex = 1; vertex < connection.vertices.size(); ++vertex )
                {
                    if( busEntrySegments.contains( segmentKey( aSourceSheet.id, net->id,
                                                               connection.vertices[vertex - 1],
                                                               connection.vertices[vertex] ) ) )
                    {
                        continue;
                    }

                    auto line = std::make_unique<SCH_LINE>( pagePoint( connection.vertices[vertex - 1], pageHeight ),
                                                            LAYER_WIRE );
                    line->SetEndPoint( pagePoint( connection.vertices[vertex], pageHeight ) );
                    line->SetStroke( STROKE_PARAMS( 0, LINE_STYLE::SOLID ) );
                    aScreen->Append( line.get() );
                    line.release();
                    ++aStaged.result.counts.wires;
                }

                SOURCE_POINT labelPoint = connection.vertices.front();
                auto         pinEndpoint = std::ranges::find( connection.endpoints, MODEL_ENDPOINT_KIND::PIN,
                                                              &MODEL_CONNECTION_ENDPOINT::kind );

                if( pinEndpoint != connection.endpoints.end() )
                    labelPoint = pinEndpoint->point;
                else if( connection.vertices.size() >= 2
                         && busEntrySegments.contains( segmentKey( aSourceSheet.id, net->id, connection.vertices[0],
                                                                   connection.vertices[1] ) ) )
                    labelPoint = connection.vertices[1];

                // Suppress the synthetic net-name label when this connection already
                // terminates on an explicit PADS label.  The fallback label point is
                // intentionally biased toward a pin endpoint, while an off-page/global
                // label normally lives at the opposite end of the same connection.
                // Comparing only against labelPoint therefore duplicates the source
                // label with a tiny default bidirectional SCH_GLOBALLABEL at the pin.
                const bool hasSourceLabel = std::ranges::any_of(
                        MODEL_INDEX::ForSheet( aIndex.labelsBySheet, aSourceSheet.id ),
                        [&]( const MODEL_LABEL* aLabel )
                        {
                            if( ( aLabel->kind != MODEL_LABEL_KIND::GLOBAL
                                  && aLabel->kind != MODEL_LABEL_KIND::POWER
                                  && aLabel->kind != MODEL_LABEL_KIND::GROUND )
                                || aLabel->text.text != net->name.text )
                            {
                                return false;
                            }

                            return std::ranges::any_of(
                                    connection.vertices,
                                    [&]( const SOURCE_POINT& aPoint )
                                    {
                                        return aLabel->position.x == aPoint.x
                                               && aLabel->position.y == aPoint.y;
                                    } );
                        } );

                if( !hasSourceLabel )
                {
                    auto label =
                            std::make_unique<SCH_GLOBALLABEL>( pagePoint( labelPoint, pageHeight ), net->name.text );
                    label->SetTextSize( VECTOR2I( 1, 1 ) );
                    aScreen->Append( label.get() );
                    label.release();
                }
            }
        }

        for( const MODEL_JUNCTION* junction : MODEL_INDEX::ForSheet( aIndex.junctionsBySheet, aSourceSheet.id ) )
        {
            auto item = std::make_unique<SCH_JUNCTION>( pagePoint( junction->position, pageHeight ) );
            aScreen->Append( item.get() );
            item.release();
            ++aStaged.result.counts.junctions;
        }

        for( const MODEL_LABEL* labelPointer : MODEL_INDEX::ForSheet( aIndex.labelsBySheet, aSourceSheet.id ) )
        {
            const MODEL_LABEL& label = *labelPointer;

            const SOURCE_POINT* adjacentWirePoint = nullptr;
            auto adjacency = aIndex.endpointAdjacencyBySheet.find(
                    { aSourceSheet.id.Value(), label.position.x, label.position.y } );

            if( adjacency != aIndex.endpointAdjacencyBySheet.end()
                && !adjacency->second.empty() )
            {
                adjacentWirePoint = &adjacency->second.front();
            }

            std::unique_ptr<SCH_ITEM> item;

            switch( label.kind )
            {
            case MODEL_LABEL_KIND::LOCAL:
            case MODEL_LABEL_KIND::BUS:
                item = std::make_unique<SCH_LABEL>( pagePoint( label.position, pageHeight ), label.text.text );
                break;

            case MODEL_LABEL_KIND::GLOBAL:
            {
                auto globalLabel =
                        std::make_unique<SCH_GLOBALLABEL>( pagePoint( label.position, pageHeight ), label.text.text );
                globalLabel->SetShape( LABEL_FLAG_SHAPE::L_UNSPECIFIED );
                item = std::move( globalLabel );
                break;
            }

            case MODEL_LABEL_KIND::HIERARCHICAL:
                item = std::make_unique<SCH_HIERLABEL>( pagePoint( label.position, pageHeight ), label.text.text );
                break;

            case MODEL_LABEL_KIND::GROUND:
            case MODEL_LABEL_KIND::POWER:
                item = makePowerSymbol( label, aPath, pageHeight, aStaged.nextPowerOrdinal++,
                                        adjacentWirePoint, aStaged.result.diagnostics );
                ++aStaged.result.counts.symbols;
                break;

            case MODEL_LABEL_KIND::UNSUPPORTED:
            {
                auto property =
                        std::ranges::find_if( label.properties,
                                              []( const SOURCE_PROPERTY& aProperty )
                                              {
                                                  return aProperty.name.text == wxS( "unsupported_offpage_decal" );
                                              } );

                if( property == label.properties.end() )
                {
                    aStaged.result.diagnostics.push_back( MakePropertyDiagnostic(
                            RPT_SEVERITY_WARNING, label.source, wxS( "unsupported_offpage_decal" ),
                            PROPERTY_DISPOSITION::UNSUPPORTED,
                            wxS( "PADS unsupported label has no KiCad schematic representation" ) ) );
                }

                continue;
            }
            }

            if( auto* text = dynamic_cast<EDA_TEXT*>( item.get() ) )
            {
                text->SetTextAngle( EDA_ANGLE( label.angle, TENTHS_OF_A_DEGREE_T ) );
                applyTextPresentation( text, label.presentation, label.presentation.visible,
                                       aStaged.result.diagnostics );

                // Normalize KiCad global-label justification after applying the PADS presentation.
                // Prefer the authored NETNAMES text offset: rotation and mirror bits do
                // not reliably encode the off-page reading direction.  SetSpinStyle()
                // must run last because applyTextPresentation() changes angle/alignment.
                if( auto* globalLabel = dynamic_cast<SCH_GLOBALLABEL*>( text ) )
                {
                    // PADS $OSR_SYMS defines only REFIN and REFOUT; it has no
                    // bidirectional off-page variant.  The parser already resolves
                    // the selected binary variant to its CAEDECAL name.
                    if( label.decalName.text.CmpNoCase( wxS( "REFIN" ) ) == 0 )
                        globalLabel->SetShape( LABEL_FLAG_SHAPE::L_INPUT );
                    else if( label.decalName.text.CmpNoCase( wxS( "REFOUT" ) ) == 0 )
                        globalLabel->SetShape( LABEL_FLAG_SHAPE::L_OUTPUT );

                    // Fit the native KiCad global-label body to the external height
                    // measured from the selected PADS off-page CAEDECAL.  The parser
                    // stores that source geometry in half-mils; if it is unresolved,
                    // retain the ordinary PADS text presentation.
                    if( label.offpageBodyHeight > 0 )
                    {
                        const int targetBody = toIU( label.offpageBodyHeight );
                        int low = 1;
                        int high = targetBody;
                        int best = low;

                        while( low <= high )
                        {
                            const int candidate = low + ( high - low ) / 2;
                            globalLabel->SetTextSize( VECTOR2I( candidate, candidate ) );

                            const BOX2I body = globalLabel->GetBodyBoundingBox( nullptr );
                            const bool horizontal = globalLabel->GetSpinStyle() == SPIN_STYLE::LEFT
                                                    || globalLabel->GetSpinStyle() == SPIN_STYLE::RIGHT;
                            const int bodyThickness = horizontal ? body.GetHeight() : body.GetWidth();

                            if( bodyThickness <= targetBody )
                            {
                                best = candidate;
                                low = candidate + 1;
                            }
                            else
                            {
                                high = candidate - 1;
                            }
                        }

                        globalLabel->SetTextSize( VECTOR2I( best, best ) );
                    }

                    globalLabel->SetSpinStyle(
                            padsLabelSpinStyle( label.position, label.textOffset,
                                                adjacentWirePoint ) );
                }
            }

            aScreen->Append( item.get() );
            item.release();
            ++aStaged.result.counts.labels;
        }

        for( const MODEL_TEXT* sourceTextPointer : MODEL_INDEX::ForSheet( aIndex.textsBySheet, aSourceSheet.id ) )
        {
            const MODEL_TEXT& sourceText = *sourceTextPointer;

            std::unique_ptr<SCH_TEXT> text =
                    makePageText( sourceText.text, sourceText.position, sourceText.angle, sourceText.presentation,
                                  pageHeight, aStaged.result.diagnostics );
            aScreen->Append( text.get() );
            text.release();
            ++aStaged.result.counts.texts;
        }

        for( const MODEL_PAGE_GRAPHIC* pageGraphicPointer :
             MODEL_INDEX::ForSheet( aIndex.graphicsBySheet, aSourceSheet.id ) )
        {
            const MODEL_PAGE_GRAPHIC& pageGraphic = *pageGraphicPointer;

            if( pageGraphic.graphic.kind == MODEL_GRAPHIC_KIND::TEXT )
            {
                appendPageGraphic( aScreen, pageGraphic.graphic, pageHeight, aStaged.result.diagnostics );
                ++aStaged.result.counts.texts;
            }
            else
            {
                aStaged.result.counts.graphics +=
                        appendPageGraphic( aScreen, pageGraphic.graphic, pageHeight, aStaged.result.diagnostics );
            }
        }

        for( const MODEL_EMBEDDED_IMAGE* sourceImage : MODEL_INDEX::ForSheet( aIndex.imagesBySheet, aSourceSheet.id ) )
        {
            std::unique_ptr<SCH_BITMAP> bitmap =
                    makeEmbeddedImage( *sourceImage, pageHeight, aStaged.result.diagnostics );

            if( !bitmap )
                continue;

            aScreen->Append( bitmap.get() );
            bitmap.release();
            ++aStaged.result.counts.images;
        }
    }

} // namespace


BUILD_RESULT PADS_SCH_BINARY_BUILDER::Build( const PADS_SCH_MODEL& aModel, SCHEMATIC* aSchematic,
                                             SCH_SHEET* aAppendToMe, const wxString& aSourcePath )
{
    if( !aSchematic )
        THROW_IO_ERROR( wxS( "cannot build a PADS schematic without a destination" ) );

    if( aSourcePath.IsEmpty() )
        THROW_IO_ERROR( wxS( "cannot build a PADS schematic without a source filename" ) );

    if( aModel.sheets.empty() )
        THROW_IO_ERROR( FormatParserError( aModel.source, wxS( "schematic model has no sheets" ) ) );

    if( aAppendToMe && !aAppendToMe->GetScreen() )
        THROW_IO_ERROR( wxS( "cannot append a PADS schematic to a sheet without a screen" ) );

    aModel.ValidateOrThrow();
    std::vector<const MODEL_SHEET*> sourceSheets;
    sourceSheets.reserve( aModel.sheets.size() );

    for( const MODEL_SHEET& sheet : aModel.sheets )
        sourceSheets.push_back( &sheet );

    std::ranges::sort( sourceSheets, {}, &MODEL_SHEET::index );

    MODEL_INDEX      modelIndex( aModel );
    STAGED_SCHEMATIC staged;
    staged.result.counts.sheets = aModel.sheets.size();
    staged.connectionGraph = std::make_unique<CONNECTION_GRAPH>( aSchematic );

    if( aAppendToMe )
        staged.nextPowerOrdinal = PADS_SCH::PADS_SCH_SYMBOL_BUILDER::NextFreePowerOrdinal( aAppendToMe );

    validatePropertyDispositions( aModel, staged.result.diagnostics );

    if( !aAppendToMe && !aModel.worksheets.empty() )
    {
        auto worksheet = std::ranges::find_if( aModel.worksheets,
                                               []( const MODEL_WORKSHEET& aWorksheet )
                                               {
                                                   return aWorksheet.name.text == wxS( "DRW5982" );
                                               } );

        if( worksheet == aModel.worksheets.end() )
            worksheet = aModel.worksheets.begin();

        auto sourceSheet = std::ranges::find( aModel.sheets, worksheet->sheet.id, &MODEL_SHEET::id );

        if( sourceSheet == aModel.sheets.end() )
            THROW_IO_ERROR( FormatParserError( worksheet->source, wxS( "worksheet references an unknown sheet" ) ) );

        const std::string serialized = serializeWorksheet( *worksheet, *sourceSheet );
        auto              embeddedWorksheet = std::make_shared<EMBEDDED_FILES::EMBEDDED_FILE>();
        embeddedWorksheet->name = wxS( "pads_import.kicad_wks" );
        embeddedWorksheet->type = EMBEDDED_FILES::EMBEDDED_FILE::FILE_TYPE::WORKSHEET;
        embeddedWorksheet->decompressedData.assign( serialized.begin(), serialized.end() );

        if( EMBEDDED_FILES::CompressAndEncode( *embeddedWorksheet ) != EMBEDDED_FILES::RETURN_CODE::OK )
            THROW_IO_ERROR( FormatParserError( worksheet->source, wxS( "could not embed the PADS worksheet" ) ) );

        embeddedWorksheet->is_valid = true;
        staged.replacementEmbeddedFiles = std::make_unique<EMBEDDED_FILES>();
        staged.replacementEmbeddedFiles->SetFileAddedCallback( aSchematic->GetEmbeddedFiles()->GetFileAddedCallback() );
        staged.replacementEmbeddedFiles->AddFile( embeddedWorksheet );
        staged.replacementDrawingSheet = embeddedWorksheet->GetLink();
    }

    const bool         multiSheet = aModel.sheets.size() > 1;
    std::set<wxString> usedFilenames;
    size_t             firstChildPage = 2;

    if( aAppendToMe )
    {
        for( const SCH_SHEET_PATH& path : aSchematic->BuildSheetListSortedByPageNumbers() )
        {
            unsigned long page = 0;

            if( path.GetPageNumber().ToULong( &page ) )
            {
                if( page >= std::numeric_limits<size_t>::max() )
                    THROW_IO_ERROR( wxS( "existing schematic page number leaves no append range" ) );

                firstChildPage = std::max( firstChildPage, static_cast<size_t>( page ) + 1 );
            }
        }

        if( multiSheet && sourceSheets.size() > std::numeric_limits<size_t>::max() - firstChildPage )
            THROW_IO_ERROR( wxS( "imported hierarchy page range overflows" ) );

        staged.appendCache = std::make_unique<SCH_SCREEN>( aSchematic );

        for( const auto& [name, symbol] : aAppendToMe->GetScreen()->GetLibSymbols() )
        {
            if( !symbol )
                THROW_IO_ERROR( wxString::Format( wxS( "destination library cache entry '%s' is null" ), name ) );

            auto clone = std::make_unique<LIB_SYMBOL>( *symbol );
            staged.appendCache->AddLibSymbol( name, std::move( clone ) );
        }

        for( SCH_ITEM* item : aAppendToMe->GetScreen()->Items().OfType( SCH_SHEET_T ) )
        {
            const SCH_SHEET* child = static_cast<const SCH_SHEET*>( item );
            usedFilenames.insert( child->GetField( FIELD_T::SHEET_FILENAME )->GetText().Lower() );
        }
    }

    if( !aAppendToMe )
    {
        if( !multiSheet )
        {
            staged.destinationRoot = aSchematic->GetTopLevelSheet();

            if( !staged.destinationRoot || !staged.destinationRoot->GetScreen() )
                THROW_IO_ERROR( wxS( "cannot replace a schematic without a top-level sheet and screen" ) );

            staged.replacementScreen = std::make_unique<SCH_SCREEN>( aSchematic );
            staged.replacementScreen->SetFileName( aSourcePath );
            SCH_SHEET_PATH rootPath;
            rootPath.push_back( staged.destinationRoot );
            staged.hierarchy.push_back( rootPath );
            staged.replacementCurrentSheet.emplace( rootPath );
            stageSheetContent( staged, modelIndex, *sourceSheets.front(), staged.replacementScreen.get(), rootPath );
        }
        else
        {
            staged.topLevelCache = std::make_unique<SCH_SCREEN>( aSchematic );

            for( size_t index = 0; index < sourceSheets.size(); ++index )
            {
                const MODEL_SHEET& sourceSheet = *sourceSheets[index];
                auto               sheet = std::make_unique<SCH_SHEET>( aSchematic );
                auto               screen = new SCH_SCREEN( aSchematic );
                sheet->SetScreen( screen );
                sheet->SyncUuidToScreen();
                sheet->SetParent( &aSchematic->Root() );
                sheet->GetField( FIELD_T::SHEET_NAME )->SetText( sourceSheet.name.text );
                wxString filename = sanitizedFilename( sourceSheet.name.text, index, usedFilenames );
                sheet->GetField( FIELD_T::SHEET_FILENAME )->SetText( filename );
                screen->SetFileName( filename );
                screen->SetPageNumber( wxString::Format( wxS( "%zu" ), index + 1 ) );
                SCH_SHEET_PATH path;
                path.push_back( sheet.get() );
                path.SetPageNumber( wxString::Format( wxS( "%zu" ), index + 1 ) );
                stageSheetContent( staged, modelIndex, sourceSheet, screen, path );
                staged.hierarchy.push_back( path );

                if( index == 0 )
                    staged.replacementCurrentSheet.emplace( path );

                staged.topLevelIndex.insert( sheet.get() );
                staged.topLevelSheets.push_back( sheet.get() );
                staged.topLevelOwners.push_back( std::move( sheet ) );
            }
        }
    }
    else if( !multiSheet )
    {
        const MODEL_SHEET& sourceSheet = *sourceSheets.front();
        SCH_SCREEN*        temporaryScreen = staged.appendCache.get();
        SCH_SHEET_PATH     path;
        path.push_back( aAppendToMe );
        stageSheetContent( staged, modelIndex, sourceSheet, temporaryScreen, path );

        std::vector<SCH_ITEM*> temporaryItems;

        for( SCH_ITEM* item : temporaryScreen->Items() )
            temporaryItems.push_back( item );

        for( SCH_ITEM* item : temporaryItems )
        {
            std::unique_ptr<SCH_ITEM> itemOwner( item );

            // remove() falls back to a full-tree search, so a failure means the screen no longer
            // holds the item and itemOwner is the only thing left that can free it
            if( !temporaryScreen->Items().remove( item ) )
                THROW_IO_ERROR( wxS( "staged append item is missing from its spatial index" ) );

            item->SetParent( aAppendToMe->GetScreen() );
            staged.appendItems.emplace_back( std::move( itemOwner ) );
        }
    }
    else
    {
        SCH_SHEET_PATH rootPath;
        rootPath.push_back( aAppendToMe );

        for( size_t index = 0; index < sourceSheets.size(); ++index )
        {
            const MODEL_SHEET& sourceSheet = *sourceSheets[index];
            VECTOR2I           position( schIUScale.MilsToIU( 500 + static_cast<int>( index % 4 ) * 2500 ),
                                         schIUScale.MilsToIU( 500 + static_cast<int>( index / 4 ) * 2000 ) );
            auto               child = std::make_unique<SCH_SHEET>(
                    aAppendToMe, position, VECTOR2I( schIUScale.MilsToIU( 2000 ), schIUScale.MilsToIU( 1500 ) ) );
            auto childScreen = new SCH_SCREEN( aSchematic );
            child->SetScreen( childScreen );
            child->GetField( FIELD_T::SHEET_NAME )->SetText( sourceSheet.name.text );
            wxString filename = sanitizedFilename( sourceSheet.name.text, index, usedFilenames );
            child->GetField( FIELD_T::SHEET_FILENAME )->SetText( filename );
            childScreen->SetFileName( filename );
            childScreen->SetPageNumber( wxString::Format( wxS( "%zu" ), firstChildPage + index ) );
            SCH_SHEET_PATH childPath( rootPath );
            childPath.push_back( child.get() );
            childPath.SetPageNumber( wxString::Format( wxS( "%zu" ), firstChildPage + index ) );
            stageSheetContent( staged, modelIndex, sourceSheet, childScreen, childPath );
            child->SetParent( aAppendToMe->GetScreen() );
            staged.appendItems.emplace_back( std::move( child ) );
        }
    }

    if( aAppendToMe )
    {
        SCH_SHEET_LIST currentHierarchy = aSchematic->Hierarchy();
        staged.hierarchy.assign( currentHierarchy.begin(), currentHierarchy.end() );

        for( SCH_ITEM* item : aAppendToMe->GetScreen()->Items() )
            staged.appendIndex.insert( item );

        for( const std::unique_ptr<SCH_ITEM>& item : staged.appendItems )
        {
            staged.appendIndex.insert( item.get() );

            if( item->Type() == SCH_SHEET_T )
            {
                SCH_SHEET_PATH path;
                path.push_back( aAppendToMe );
                path.push_back( static_cast<SCH_SHEET*>( item.get() ) );
                staged.hierarchy.push_back( path );
            }
        }
    }

    staged.Validate( aAppendToMe != nullptr );
    BUILD_RESULT result = staged.result;
    staged.Commit( aSchematic, aAppendToMe, m_beforeCommit );
    return result;
}

} // namespace PADS_SCH_BINARY
