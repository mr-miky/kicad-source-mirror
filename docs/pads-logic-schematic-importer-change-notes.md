# PADS Logic schematic importers: implementation notes and verified fixes

**Scope:** KiCad PADS Logic schematic importers, binary `v0x0009` and `v0x000D`, and PADS ASCII schematic importer.  
**Repository:** `mr-miky/kicad-source-mirror`, branch `test-pads-sch-fields`.  
**Document reference:** GitHub branch HEAD `937a08daee141c0fc0bcf7b7daebbc9783889f1a` (2026-11-12).  
**Relevant commits:**

- Binary: [`baa305681778add47710d198505489b522c70c16`](https://github.com/mr-miky/kicad-source-mirror/commit/baa305681778add47710d198505489b522c70c16) — *PADS Logic Binary: extend v0x0009 and v0x000D schematic support*.
- ASCII: [`937a08daee141c0fc0bcf7b7daebbc9783889f1a`](https://github.com/mr-miky/kicad-source-mirror/commit/937a08daee141c0fc0bcf7b7daebbc9783889f1a) — *PADS Logic ASCII: fix off-page orientation and net labels*.

This is a **technical record of committed behavior**, not a request to change it. The binary commit also contains accumulated work from multiple incremental patches. Some related fixes (e.g., component-field mapping, paper size, worksheets) had already been committed **before** `baa30568`; they are listed separately below. The source-code references and status statements are based on the above GitHub revision and earlier project fixture analyses. This note does not claim an exhaustive audit of every earlier commit or a completed regression test on every fixture.

## 1. Binary decoder: structural semantics and strict validation

### 1.1 Legacy PADS Logic `v0x0009`

The old binary format must not be decoded by reinterpreting `v0x000D` structures with different strides. The importer now has a version-specific path, with checked record sizes, offsets, owner references and bounds.

| Feature | Legacy `v0x0009` detail / implementation |
| --- | --- |
| Outer SDB controller directory | 19 pools; first controller ordinal `0x08` (implemented in `pads_sch_sdb.cpp`). |
| Sheet text | 28-byte records rather than the later 32-byte layout. |
| `USED_DECAL` | 92-byte layout; symbol-definition index is a `u16` at `+48`, not a later-format `u32`. |
| Placement | 128-byte layout, with independently decoded offsets and presentation fields. |
| Logical pin records | 8-byte records, rather than the newer 24-byte records. |
| `NETNAMES` | 40-byte records rather than 48; owner/child handles at `+30/+32`, not `+38/+40`. |
| Connection data | Independently decoded 40-byte connections, vertices, junctions and 32-byte off-pages, with typed owner/connection validation. |
| Junction marker | The low byte encodes junction class; the full 16-bit word is preserved because the upper byte has observed nonzero values of not-yet-confirmed meaning. |
| Graphics and worksheets | `v0x0009` symbol/decal definitions, free text, embedded text and title-block/worksheet paths have independent ownership and layout checks. |
| Default special-net records | Distinguish PARTTYPE defaults from per-instance `SIGPIN` overrides; do not invent invisible power pins from legacy default-net records. |

**Semantic gate selection:** a logical unit cannot be identified solely by its graphic decal: two gates can share the same graphic and map to different physical pin numbers. For example, `SWBIP32974` units A and B share the decal while A maps pins `1,2,3` and B maps `4,5,6`. Select the authored logical unit when needed; keep the verified decal-only fallback for the separate connector-variant case.

**Connector variants:** for a PADS connector `PARTTYPE`, gate/decal choices can be *alternative graphics* rather than the physical unit table. Instances of `CON\SIP\3P` may select `EXTIN` or `EXTOUT` for the same physical unit. Reconstruct physical units and retain instance-selected graphics, without changing pin numbers or merging incompatible KiCad symbol variants. Preserve both numeric (`J13-1`) and alphabetic (`J7-A`) suffixes. Do not invent unknown physical pin numbers for incomplete placements.

### 1.2 Newer PADS Logic `v0x000D`

- **Permuted placed-pin records:** controller-16 instance-pin records need not be serialized in the same order as the definition. The explicit definition-pin ordinal determines ownership. The decoder accepts permutations and rejects out-of-range or duplicate indices (`seenPinOrdinals`). Fixture: `30341_mik.sch`.
- **Multiple coincident pin anchors:** more than one pin can legitimately occupy one electrical coordinate, e.g. FASTON-style decals. Resolve one stable wire-endpoint reference without losing the other coincident pins.
- **Repeated alternative decals:** a gate may list the primary CAEDECAL among its own alternatives (e.g. `CAPNOTPOL` with `CAP, CAP`). Treat this as a redundant graphical choice, not as a recursive symbol-definition edge.
- **Optional global title-block fields:** an entirely empty corresponding controller is valid; a nonempty controller without decodable records is still an error.
- **Hierarchical `$$...` PARTTYPE pin names:** controller-11 pin-name word `+0` can be an index into the *outer/global net-name directory* (outer controller 8), **not** a byte offset into sheet-local controller 14. Keep the existing controller-14 interpretation for ordinary PARTTYPEs. For `FrontPanel_A6.sch`, sheet index 7 (`ANALOG_IN_OUT`), 112 hierarchical pin names were matched to the ASCII export, with no mismatches in that comparison.
- **Hierarchical placement group sentinel:** a component-group handle of `0xFFFFFFFF` on a `$$...` sheet symbol means *no component attribute group*. Do not dereference it or fabricate instance attributes; keep the strict group checks for ordinary components.
- **Hierarchy record interpretation:** parent-sheet and instance qualifiers are decoded as reference fields rather than fixed class sentinels. **Note:** decoding the relationship does not by itself mean that KiCad sheet nesting is fully materialized in the builder.

## 2. Bus, net and off-page fixes

### 2.1 `v0x000D` buses and NETNAMES

- **Bus vertex count/status:** the marker at the bus-record `+38` contains a vertex-count/status encoding; it is not an enum restricted to buses with 2–4 points. The high byte is checked against the vertex slice. Status values `0xFC` and `0xFD` occur in valid files; their original raw value is retained.
- **Bus alias/member ordering:** the order of declared bus-member net names does **not** necessarily match the order of off-page bus entries. Equal counts do not establish positional membership. Electrical ownership is instead resolved by actual net/connection references.
- **`@@@B<bus>.<anchor>` references:** the suffix is a bus polyline *anchor* index, not a one-based index into the member list. `@@@B0.0` is valid.
- **Overlapping bus entries:** different entries may share the same global net **and** coordinate. Preserve/use the explicit connection-record handle from the PADS bus-entry record rather than guessing ownership from coordinates alone. This was observed with `DSP_CORE` entries on the same LRCLK net.
- **KiCad bus aliases:** create proper schematic-level alias definitions and label bus segments with concise `{ALIAS}` notation (e.g. `{CONTROL_BUS}`, `{DIGITAL_AUDIO}`), instead of expanding every scalar member into an oversized label. Avoid duplicating aliases across segments or sheets.
- **NETNAMES presentation:** the committed code decodes and preserves graphical text metadata, owner references, offsets, justification and angles for bus entries and selected placed-pin owners. **It still renders some NETNAMES as graphic text rather than real electrical labels**; see Open items.

### 2.2 Off-page labels and POWER/GROUND

- Separate the **symbol's authored orientation** (OFFPAGE angle/mirror, selected CAEDECAL) from the **NETNAMES text angle and offset**.
- Apply PADS mirror flags to power/ground symbols (notably rotated `+5V`, `+12V` instances) instead of deriving orientation from an adjacent wire.
- Distinguish `$OSR_SYMS` `REFIN`/`REFOUT` directions; do not silently map unverified kinds to a bidirectional variant.
- Honor NETNAMES visibility and default power/ground text anchors/presentations when available. Preserve presentation when source values are hidden.
- For the ASCII importer, derive off-page orientation from the authored OFFPAGE transform, not movable NETNAME text offsets, and preserve signal off-page references that share internal names with power-symbol nets.

## 3. Pins, text, fields, and worksheets

### 3.1 Pin and symbol presentation

- Use the actual terminal-to-owning-symbol body attachment where it can be established; use pin-decal geometry as fallback for KiCad pin length.
- Preserve graphical pin decoration and pin electrical type (including `PINIEB` active-low wedge semantics).
- Honor PADS placement-wide visibility and terminal/decal defaults for pin names and numbers; `v0x0009` has specific placed-pin presentation override bits.
- Emit KiCad `no_connect` markers for source pins intentionally left without wires, preserving ERC meaning.
- Keep distinct KiCad library-symbol variants when instance-specific graphics or pin visibility make symbols structurally different.

### 3.2 Text and layout

- Convert PADS 9-way text justification using its sparse alignment mapping. Rotated text (90/270 degrees) needs a transformation of the source selector.
- Distinguish PADS Stroke font metrics from Windows System/outline font metrics. Account for source-unit differences (whole mils versus semantic half-mils) and for the outline font's character-cell-to-visible-height conversion.
- Transform placement-field offsets from PADS-local symbol coordinates through the placed symbol before setting KiCad field positions.
- Preserve worksheet graphics, worksheet/title-block variables and supported paper formats; avoid classifying authored worksheet text as unowned page annotations.

### 3.3 Changes in earlier commits (not newly introduced by the two commits above)

- **Component field semantics:** map `PCB DECAL` to KiCad `Footprint` and `Part Number` to `MPN`; preserve intentionally empty real attributes (e.g. `Cost=""`); do not create synthetic `"*"` fields from unbound placement-presentation records.
- **Sheet size versus border name:** the design-settings word at `+80` is the sheet-size `u32` enum; the string at `+264` names the border/drafting template. Similar-looking names (e.g. `SIZEA3`) do not make these the same field.
- **ASCII/binary worksheet equivalence:** earlier work aligned worksheet/title-block field mapping and ISO A-series page extents in the ASCII importer with binary import semantics.

## 4. Source files and committed revisions

| Path | Responsibility |
| --- | --- |
| `eeschema/sch_io/pads/pads_sch_sdb.cpp` | Binary SDB version/controller layout, especially the `v0x0009` outer directory. |
| `eeschema/sch_io/pads/pads_sch_binary_parser.cpp` | Version-specific record decoding; parts, pins, hierarchy, buses, nets, OFFPAGE/NETNAMES and validity checks. |
| `eeschema/sch_io/pads/pads_sch_binary_model.h` | Semantic model fields, including bus-entry connection references and off-page presentation/geometry metadata. |
| `eeschema/sch_io/pads/pads_sch_binary_builder.cpp` | Conversion of validated model into KiCad symbols, buses, wires, labels, fields, fonts and worksheets. |
| `eeschema/sch_io/pads/pads_sch_schematic_builder.cpp`, `.h` | ASCII schematic construction, including off-page label and net-name behavior. |
| `eeschema/sch_io/pads/pads_sch_symbol_builder.cpp` | ASCII symbol/decal and pin rendering. |
| `eeschema/sch_io/pads/sch_io_pads.cpp` | ASCII import integration and power/off-page interpretation. |

The two commits cited at the top change only the eight schematic-source files above (four in each commit). The **uncommitted** local PCB work in `pcbnew/pcb_io/pads/pads_pcb_converter.cpp` was intentionally excluded.

## 5. Status: known limitations and further verification

These points must **not** be described as solved solely because related decode fields or source comments exist:

1. **Electrical NETNAMES labels (`v0x000D`):** some NETNAMES on bus-exiting wires are still rendered as `MODEL_TEXT` / `SCH_TEXT`, rather than connection-carrying `SCH_LABEL` / `SCH_GLOBALLABEL`. A separate `pads-schematic-v000d-netnames-electrical-labels-v13.py` proposal was prepared but **not committed** at the documented HEAD. Validate anchor location, global-net semantics and fallback deduplication before applying.
2. **`v0x0009` placed-pin ordering:** the legacy decoder still contains `pinOrdinal != pin`. Whether real legacy files permit arbitrary permutations remains to be established; do not broaden the decoder without evidence.
3. **Hierarchical sheet materialization:** parent and `$$...` references can be decoded, but the binary builder's complete reconstruction of nested sheets and boundary pins remains a separate validation/implementation task.
4. **Experimental font constants:** `pads_sch_binary_builder.cpp` still contains the comment `PADS_SCH_STROKE_DIRECT_EXPERIMENT` and hardcoded Stroke width/height scale factors `0.72` / `0.68`, whereas other paths use `ADVANCED_CFG::GetCfg().m_PadsSchTextWidthScale` and `m_PadsSchTextHeightScale`. These constants are committed, but **not demonstrated to be generally correct**. Changing them should be a separate, measured visual regression task—not an incidental cleanup during documentation.
5. **Opt-in pin-name workaround:** an `#if defined(PADS_ENABLE_EXPLICIT_DIO_V_PIN_NAMES)` branch exists for explicit graphical pin-name text. It is experimental and **inactive in the default build unless explicitly enabled**. Do not claim that it is the normal import behavior.
6. **End-to-end regression:** compare KiCad import with PADS and ASCII fixtures for electrical pin numbers, net membership, visible text, off-page direction, power mirroring, bus aliases and full sheet hierarchy; compiling alone is insufficient.

## 6. Useful regression fixtures

| Fixture | What it exercises |
| --- | --- |
| `30341_mik.sch` (`v0x000D`) | Permuted placed-pin records. |
| `FrontPanel_A6.sch` / `FrontPanel_A6.txt` | Multi-sheet hierarchy, `$$...` pin names, absent attribute-group sentinel, bus connectivity and text presentation. |
| `EXPANSION` sheet from `FrontPanel_A6` | Rotated/mirrored `+5V`, bus alias labels and NETNAMES on wires leaving buses. |
| `30446_PU42xxx_A11.sch` (`v0x0009`) | Reused logical gate graphics with different pin mappings; connector unit/variant selection. |
| `30439_10_A.sch` (`v0x0009`) | Off-page orientation, diode pin-number visibility and text sizing. |
| `SIZEA3_KICAD` controlled fixture | ASCII/binary worksheet geometry, paper size and title-block fields. |
| `ADAU1701` controlled fixture | `Footprint`, `MPN`, and intentionally empty `Cost` field preservation. |

## 7. Recommended maintenance workflow

- **Do not rewrite the two existing commits simply to expand their summaries.** Commit this document separately after review.
- Keep parser fixes separate from visual calibration and from PCB import changes.
- For each new fixture-driven correction, record the file version, sheet, record/controller, observed raw value, PADS/ASCII evidence, expected KiCad result, and whether the behavior has been verified by compilation and import.
- Preserve strict structural checks. Replace invalid assumptions only when actual source data demonstrates valid counterexamples.
- Use a new follow-up commit for the future NETNAMES electrical-label patch and test `v0x0009` and `v0x000D` imports independently.

## 8. Handy Git checks

```bash
git --no-pager log -3 --oneline
git show --stat baa305681778add47710d198505489b522c70c16
git show --stat 937a08daee141c0fc0bcf7b7daebbc9783889f1a
git diff --check
```

After copying this note into the checkout and reviewing it, create a **new documentation-only commit**, e.g.:

```bash
mkdir -p docs
cp /path/to/pads-logic-schematic-importer-change-notes.md docs/
git add docs/pads-logic-schematic-importer-change-notes.md
git diff --cached --check
git diff --cached --stat
git commit -m "Docs: record PADS Logic schematic importer fixes and remaining work"
```

This leaves the existing binary/ASCII commit history and the unrelated local PCB modification untouched.
