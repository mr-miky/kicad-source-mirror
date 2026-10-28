#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Generate member probes from the selected KiCad source headers (not the PCB)."""
import argparse
import pathlib
import re

def uncomment(text):
    return re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)

def generate(source, dest):
    pcb = source / 'pcbnew/pcb_io/pads'
    headers = [pcb / 'pads_parser.h', pcb / 'pads_binary_parser.h']
    texts = [uncomment(p.read_text(encoding='utf-8')) for p in headers]
    # Named data members declared on a line, including brace/default initializers.
    # Probes for irrelevant/private names are harmless: C++ requires filters them.
    fields = {'x', 'y'}
    for text in texts:
        for line in text.splitlines():
            declaration = line.split('=', 1)[0].strip()
            m = re.fullmatch(r'[\w:<>,\s*&]+\s+(\w+)\s*(?:;|\{.*)', declaration)
            if m:
                fields.add(m.group(1))
            # Default-initialized declarations have been split before '='.
            if '=' in line and '(' not in declaration:
                m = re.fullmatch(r'[\w:<>,\s*&]+\s+(\w+)', declaration)
                if m:
                    fields.add(m.group(1))
    # The declaration scanner is intentionally permissive, so it may also
    # collect C++ language keywords from declarations such as conversion
    # operators.  They are not legal member identifiers in an expression
    # such as v.operator and must never reach dump_fields.inc.
    cpp_keywords = {
        'alignas', 'alignof', 'and', 'and_eq', 'asm', 'auto', 'bitand',
        'bitor', 'bool', 'break', 'case', 'catch', 'char', 'char8_t',
        'char16_t', 'char32_t', 'class', 'compl', 'concept', 'const',
        'consteval', 'constexpr', 'constinit', 'const_cast', 'continue',
        'co_await', 'co_return', 'co_yield', 'decltype', 'default',
        'delete', 'do', 'double', 'dynamic_cast', 'else', 'enum',
        'explicit', 'export', 'extern', 'false', 'float', 'for', 'friend',
        'goto', 'if', 'inline', 'int', 'long', 'mutable', 'namespace',
        'new', 'noexcept', 'not', 'not_eq', 'nullptr', 'operator', 'or',
        'or_eq', 'private', 'protected', 'public', 'register',
        'reinterpret_cast', 'requires', 'return', 'short', 'signed',
        'sizeof', 'static', 'static_assert', 'static_cast', 'struct',
        'switch', 'template', 'this', 'thread_local', 'throw', 'true',
        'try', 'typedef', 'typeid', 'typename', 'union', 'unsigned',
        'using', 'virtual', 'void', 'volatile', 'wchar_t', 'while', 'xor',
        'xor_eq'
    }
    fields.difference_update(cpp_keywords)

    public = texts[1].split('class BINARY_PARSER', 1)[-1].split('private:', 1)[0]
    getters = sorted(set(re.findall(r'\b(Get\w+)\s*\(\s*\)\s*const\b', public)))
    getters = [g for g in getters if not g.endswith('ForTest')]
    required = {'GetParameters', 'GetParts', 'GetNets', 'GetRoutes', 'GetPartDecals'}
    if not required <= set(getters):
        raise SystemExit('API parser incompatibile: getter mancanti: ' + ', '.join(sorted(required-set(getters))))
    if len(fields) < 20:
        raise SystemExit('Estrazione campi incompleta: controllare gli header KiCad')
    dest.mkdir(parents=True, exist_ok=True)
    (dest / 'dump_fields.inc').write_text(''.join(f'DUMP_FIELD({f})\n' for f in sorted(fields)))
    (dest / 'dump_getters.inc').write_text(''.join(f'DUMP_GETTER({g})\n' for g in getters))
    (dest / 'schema.txt').write_text('Getter esportati:\n' + '\n'.join(getters) + '\n\nNomi campi riconosciuti:\n' + '\n'.join(sorted(fields)) + '\n')
    print(f'Generati {len(fields)} nomi di campo e {len(getters)} getter')

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('source', type=pathlib.Path)
    p.add_argument('dest', type=pathlib.Path)
    a = p.parse_args()
    generate(a.source.resolve(), a.dest.resolve())
