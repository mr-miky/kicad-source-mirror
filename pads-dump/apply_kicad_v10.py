#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Apply the v10 legacy PADS placement-field fix to the KiCad source tree.

This is intentionally a small, idempotent source transformer instead of requiring
users to apply a unified diff by hand.  It targets the KiCad nightly revision used
for pads-dump development and fails closed if the expected anchors are absent.
"""
from __future__ import annotations

import argparse
import pathlib
import shutil
import sys

PARSER_REL = pathlib.Path("pcbnew/pcb_io/pads/pads_binary_parser.cpp")
IMPORTER_REL = pathlib.Path("pcbnew/pcb_io/pads/pcb_io_pads_binary.cpp")
MARKER = "v10 legacy placement field recovery"

RECOVERY = r'''
    // pads-dump v10 legacy placement field recovery:
    // 96-byte placement records do not carry the +96 field-list head used by the
    // newer layout, but section 8 still owns the complete list.  Association 0x08
    // links one field-presentation record to the next and 0x16 terminates at the
    // zero-based placement index.  Reconstruct only missing heads; keep the newer
    // direct placement link authoritative when it exists.
    if( m_version <= 0x2022 && m_partFieldStart.size() < m_parts.size() )
    {
        std::vector<int32_t> predecessor( aText.physicalCount, -1 );
        std::vector<int32_t> placementOwner( aText.physicalCount, -1 );

        for( uint32_t index = 0; index < aText.physicalCount; ++index )
        {
            SDB_RECORD metadata =
                    m_sdb.RecordAt( aRecordBase + static_cast<size_t>( index + 1 ) * aRecordSize );

            if( metadata.U16( 4 ) != 0xFFFE )
                continue;

            uint32_t association = metadata.U32( 12 );
            uint8_t  associationKind = static_cast<uint8_t>( association >> 24 );
            uint32_t target = association & 0x00FFFFFFU;

            if( associationKind == 0x08 )
            {
                if( target >= aText.physicalCount )
                    THROW_IO_ERROR( "Invalid PADS legacy placement field-list link" );

                if( predecessor[target] >= 0 && predecessor[target] != static_cast<int32_t>( index ) )
                    THROW_IO_ERROR( "Ambiguous PADS legacy placement field-list link" );

                predecessor[target] = static_cast<int32_t>( index );
            }
            else if( associationKind == 0x16 )
            {
                if( target >= m_parts.size() )
                    THROW_IO_ERROR( "Invalid PADS legacy placement field-list owner" );

                placementOwner[index] = static_cast<int32_t>( target );
            }
        }

        for( uint32_t terminal = 0; terminal < aText.physicalCount; ++terminal )
        {
            if( placementOwner[terminal] < 0 )
                continue;

            size_t partIndex = static_cast<size_t>( placementOwner[terminal] );

            if( m_partFieldStart.count( partIndex ) )
                continue;

            uint32_t           head = terminal;
            std::set<uint32_t> visited;

            while( predecessor[head] >= 0 )
            {
                if( !visited.insert( head ).second )
                    THROW_IO_ERROR( "Cyclic PADS legacy placement field-list link" );

                head = static_cast<uint32_t>( predecessor[head] );
            }

            m_partFieldStart[partIndex] = static_cast<int32_t>( head );
        }
    }

'''

LOOP_ANCHOR = "    for( const auto& [partIndex, fieldStart] : m_partFieldStart )\n"

LEVEL_OLD = """            uint32_t   presentation = metadata.U32( 24 );\n            uint8_t    fieldKind = static_cast<uint8_t>( presentation >> 16 );\n            attribute.visible = ( fieldKind & 0x20 ) != 0;\n"""
LEVEL_NEW = """            uint32_t   presentation = metadata.U32( 24 );\n            uint8_t    fieldKind = static_cast<uint8_t>( presentation >> 16 );\n            attribute.level = static_cast<int16_t>( presentation & 0xFFFFU );\n            attribute.visible = ( fieldKind & 0x20 ) != 0;\n"""

LAYER_OLD = """            field->SetVisible( attribute.visible );\n            field->SetLayer( F_Fab );\n"""
LAYER_NEW = """            field->SetVisible( attribute.visible );\n\n            PCB_LAYER_ID fieldLayer = getMappedLayer( attribute.level );\n\n            if( fieldLayer == UNDEFINED_LAYER )\n                fieldLayer = F_Fab;\n            else if( IsCopperLayer( fieldLayer ) )\n                fieldLayer = fieldLayer == B_Cu ? B_SilkS : F_SilkS;\n\n            field->SetLayer( fieldLayer );\n"""


def replace_once(text: str, old: str, new: str, what: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"anchor {what!r}: expected once, found {count}")
    return text.replace(old, new, 1)


def patch_parser(text: str) -> str:
    if MARKER not in text:
        text = replace_once(text, LOOP_ANCHOR, RECOVERY + LOOP_ANCHOR, "placement loop")
    if "attribute.level = static_cast<int16_t>( presentation & 0xFFFFU );" not in text:
        text = replace_once(text, LEVEL_OLD, LEVEL_NEW, "placement field level")
    return text


def patch_importer(text: str) -> str:
    if "PCB_LAYER_ID fieldLayer = getMappedLayer( attribute.level );" not in text:
        text = replace_once(text, LAYER_OLD, LAYER_NEW, "binary field layer")
    return text


def write_changed(path: pathlib.Path, new: str, dry_run: bool) -> bool:
    old = path.read_text(encoding="utf-8")
    if old == new:
        return False
    if dry_run:
        return True
    backup = path.with_suffix(path.suffix + ".v10.bak")
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(new, encoding="utf-8")
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("source", nargs="?", default=".", type=pathlib.Path,
                    help="radice sorgenti KiCad (default: directory corrente)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    root = args.source.resolve()
    parser = root / PARSER_REL
    importer = root / IMPORTER_REL

    if not parser.is_file() or not importer.is_file():
        print("Errore: non sembra una radice dei sorgenti KiCad", file=sys.stderr)
        return 2

    try:
        parser_old = parser.read_text(encoding="utf-8")
        importer_old = importer.read_text(encoding="utf-8")
        parser_new = patch_parser(parser_old)
        importer_new = patch_importer(importer_old)
    except RuntimeError as exc:
        print(f"Errore: sorgenti KiCad non compatibili con questo fix: {exc}", file=sys.stderr)
        return 1

    parser_changed = write_changed(parser, parser_new, args.dry_run)
    importer_changed = write_changed(importer, importer_new, args.dry_run)
    action = "Modificherebbe" if args.dry_run else "Modificati"
    changed = [str(PARSER_REL) for _ in [0] if parser_changed]
    if importer_changed:
        changed.append(str(IMPORTER_REL))

    if changed:
        print(action + ":")
        for item in changed:
            print("  " + item)
    else:
        print("Fix v10 gia' presente; nessuna modifica necessaria.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
