# pads-dump — dump intermedio del parser PADS di KiCad

Programma C++ a riga di comando. Chiama direttamente `PADS_IO::BINARY_PARSER::Parse()`
dei sorgenti KiCad e serializza i risultati dei getter pubblici senza creare un BOARD.
Il parser originale non viene modificato. L'eseguibile è autonomo nell'uso,
ma la compilazione usa la build di KiCad e le sue librerie interne.

## Compilazione (Linux)

Occorrono i sorgenti della stessa revisione di KiCad 11 utilizzata per importare,
le dipendenze di compilazione di KiCad, un compilatore C++20, CMake e Python 3.
Il solo pacchetto KiCad installato non basta. Le istruzioni presuppongono una
build KiCad già configurabile: per le dipendenze della distribuzione seguire
https://dev-docs.kicad.org/en/build/ .

1. Estrarre questa cartella `pads-dump` nella radice dei sorgenti KiCad.
2. Aggiungere in FONDO al `CMakeLists.txt` principale di KiCad:

   ```cmake
   add_subdirectory(pads-dump)
   ```

3. Dalla radice dei sorgenti:

   ```bash
   cmake -S . -B build
   cmake --build build --target pads-dump -j4
   ```

   Riutilizzare le opzioni della propria build se già configurata. Il target
   costruisce il programma e le librerie necessarie, senza richiedere di compilare
   gli eseguibili degli editor. La configurazione richiede comunque le dipendenze
   previste dal progetto KiCad. L'eseguibile dipende dalle librerie della build:
   eseguirlo da lì prima di copiarlo altrove.

## Utilizzo

```bash
./build/pads-dump/pads-dump /percorso/scheda.pcb scheda-dump.json
# Oppure stdout:
./build/pads-dump/pads-dump /percorso/scheda.pcb > scheda-dump.json
# Lettura con indentazione:
python3 -m json.tool scheda-dump.json > scheda-dump-pretty.json
```

Diagnostica su stderr. Exit code: 0 successo, 1 errore, 2 argomenti errati.
Il programma legge il PCB senza modificarlo; rifiuta un output che coincide con esso.
Un output esistente viene sostituito dopo il successo di parsing e serializzazione.

Il JSON contiene `parser.GetParts`, `parser.GetNets`, `parser.GetRoutes`,
`parser.GetPartDecals`, `parser.GetParameters` e gli altri getter pubblici senza
argomenti della revisione scelta (test esclusi). Include quindi geometrie, padstack,
terminali, attributi e livelli esposti dal parser. `generated/schema.txt` nella
cartella build elenca getter e nomi dei campi riconosciuti.

```bash
jq '.parser.GetParts[] | select(.name == "U1")' scheda-dump.json
jq '.parser.GetNets[] | {name, pins}' scheda-dump.json
jq '.parser.GetPartDecals[] | {decal: .key, terminals: .value.terminals, pad_stacks: .value.pad_stacks}' scheda-dump.json
```

## Semantica e limiti

- È il modello intermedio DOPO `Parse()`, inclusi i collegamenti e le correzioni
  che il parser fa al suo interno. Non è un dump dei byte o di ciascun passo di lettura.
  I dati scartati dal parser non sono recuperabili tramite questi getter.
- Coordinate e dimensioni rimangono nelle unità intermedie originali. Nel parser
  binario corrente sono BASIC: 38100 unità per mil, cioè 1500000 per mm.
  Non si applicano il cambio di origine e l'inversione Y del convertitore PCB.
- Gli enum sono numeri, da interpretare secondo gli header della stessa revisione.
- Le mappe sono array di `{ "key": ..., "value": ... }` per preservare chiavi
  numeriche; le coppie sono array di due elementi e gli optional assenti sono null.
- Le stringhe conservano i byte: ogni byte >= 127 viene rappresentato come
  `\u00XX`. Per recuperare esattamente una stringa con Python:
  `raw = s.encode("latin1")`; poi decodificarla con la codifica originale del file.
  Anche stringhe con byte non UTF-8 producono JSON valido.
- Il generatore riconosce le normali dichiarazioni di campo degli header PADS.
  Non è un parser C++ completo: cambiamenti nella sintassi degli header possono
  richiedere un adeguamento. Un oggetto senza campi riconosciuti genera errore.
- Se `Parse()` fallisce, non viene prodotto un dump parziale.
- Memoria: parser e JSON completo restano in RAM fino alla scrittura.

## Verifica effettuata

Sono stati verificati il generatore e la serializzazione C++20 con strutture
annidate, mappe, enum, optional, byte speciali e numeri. In questo ambiente mancano
sorgenti completi e dipendenze KiCad: la compilazione/link e l'esecuzione su un PCB
reale NON sono state verificate. Questa è una prima versione da compilare con
la propria revisione di KiCad 11; non un binario già collaudato.

## Riferimenti

https://docs.kicad.org/doxygen/pads__binary__parser_8h_source.html
https://docs.kicad.org/doxygen/pads__binary__parser_8cpp_source.html
https://gitlab.com/kicad/code/kicad

I file di questo progetto sono GPL-3.0-or-later. Le sorgenti KiCad restano esterne,
con le rispettive licenze.

## Esportazione decals (sperimentale)

La nuova modalità usa **lo stesso `BINARY_PARSER` del dump** e i dati
`GetPartDecals()`, senza tentare un secondo parsing del formato binario.

```bash
./build/pads-dump/pads-dump --export-decals scheda.pcb Libreria.pretty
# Soltanto per ispezione, genera footprint INCOMPLETI con i pad supportati:
./build/pads-dump/pads-dump --export-decals scheda.pcb Libreria.pretty --allow-incomplete
```

Dipendenza a runtime per questa modalità: `python3`; percorso script incluso
nel binario a tempo di build. Il file `pads-dump-export-report.json` nella
cartella destinazione elenca i problemi per decal.

**LIMITAZIONE IMPORTANTE:** nella versione del parser esaminata
`GetPartDecals().items` è vuoto per le decals del PCB di riferimento. La
modalità standard NON scrive footprint incompleti; per questo occorre
`--allow-incomplete`. Questo produce una **bozza** non utilizzabile senza
verifica: la grafica non viene ancora ricostruita, i pad PADS speciali `OF`
e altre geometrie non implementate vengono segnalati nel report.
Le etichette e i padstack con forme non supportate non sono convertiti.
Il recupero della grafica tramite la catena proprietario/pieces/vertici del
binario deve essere integrato nel parser KiCad per ottenere decals complete.

## Catena binaria decal -> GraphicPiece -> vertici (nuova implementazione)

`pads_binary_graphics.py` legge direttamente il PCB binario PADS BASIC,
individua il record proprietario della decal (112 byte), usa `piece_start`,
`vertex_start` e `piece_count`, decodifica i descrittori grafici di 20 byte
ed i vertici di 12 byte. **Non usa il file ASCII**; i file ASCII sono stati
usati solamente per la validazione. Il codice e' integrato in `export_decals.py`
e viene eseguito automaticamente con `--export-decals` usando il percorso
`source` del JSON prodotto da pads-dump.

I tipi `OPEN`, `CLOSED`, `CIRCLE` sui layer supportati diventano
`fp_line` / `fp_circle`. Per i tipi, i layer e gli stili non supportati,
il report segnala esplicitamente la mancata conversione; l'opzione
`--allow-incomplete` resta necessaria quando non si riesce a preservare
la geometria, i testi o i pad.

**Limiti:** rilevamento conservativo delle sezioni per il layout PADS BASIC
studiato; verificato nei due file di prova e in un file reale con 43 decals
grafiche / 165 GraphicPiece. Non funzionante per tutte le varianti di layout
(ad esempio `usbfree_R11_Layout.pcb` presenta una diversa disposizione dei
record). Non viene ancora patchato `PART_DECAL.items` in C++: la catena e'
letta dal modulo Python invocato dall'esportatore di pads-dump.

## Associazione binaria Decal -> GraphicPiece -> vertici (C++)

`pads_binary_graphics.cpp` usa direttamente `PADS_SDB` e i controller binari
owner/GraphicPiece/vertici/archi, senza usare l'esportazione ASCII. Il programma aggiunge la
proprietà `decal_graphics` nel dump JSON, che `export_decals.py` utilizza
per creare le primitive grafiche KiCad. Per evitare false associazioni,
se non trova una sola disposizione valida delle sezioni produce `{}` e
segnala l'errore su stderr: non ricostruisce footprint falsi.

Le basi delle sezioni sono ora fornite dal container `PADS_SDB`; **non** è un'integrazione in `PADS_IO::BINARY_PARSER::ParsePartDecals`
e **non** garantisce la compatibilità con tutti i file PADS. È stato
compilato e verificato come componente C++ indipendente su due test PCB
(10/11 GraphicPiece) e su `30449_00_rev_E0(2).pcb` (43 decals, 165 piece).
L'eseguibile KiCad completo richiede la build del sorgente nightly originale.

## v4 exporter fixes

- PADS decal layer 0 is exported as F.SilkS, matching KiCad's PADS loader.
- Copper-numbered decal graphics are redirected to the corresponding silkscreen.
- Solder mask, paste, silkscreen, assembly and drill layers are mapped from GetLayerInfos.
- Unassigned/documentation graphics are preserved on Cmts.User instead of causing the footprint to be skipped.
- PADS OF pads are exported as KiCad oval pads.
- PADS pad size convention is corrected: sizeA is Y/height and sizeB is X/width.

## v6: supporto dialect SDB legacy e fix Python

- Lo stride dei record owner viene scelto dalla versione SDB come nel parser KiCad:
  `<= 0x2022` usa 100 byte, versioni successive 112 byte.
- Lo stride `GraphicPiece` segue lo stesso cut-over di KiCad:
  `<= 0x2024` usa 16 byte, versioni successive 20 byte.
- Nei `GraphicPiece` da 16 byte non esiste il DWORD `line_style`; viene quindi
  riportato come `0` (solid).
- `KPTCIR` e' riportato esplicitamente nel JSON al posto del vecchio nome `TYPE11`.
- La diagnostica `decal_graphics` include `sdb_version`, `owner_stride` e
  `piece_stride` per rendere immediatamente verificabile il dialect usato.
- `generate.py` filtra tutte le keyword C++ (incluso `operator`) prima di generare
  `dump_fields.inc`, evitando il precedente errore `v.operator`.
- L'exporter Python viene eseguito con **lo stesso interprete trovato da CMake**
  (`Python3_EXECUTABLE`) invece di cercare genericamente `python3` nel `PATH`.

File verificati per il detector C++:

- `30439_10_A0.pcb`: SDB `0x2017`, owner 100, GraphicPiece 16.
- `30449_00_rev_E0(2).pcb`: SDB `0x2026`, owner 112, GraphicPiece 20.

## v7 notes

- Binary layout follows the SDB version: owner records are 100 bytes through 0x2022 and
  112 bytes afterwards; GraphicPiece records are 16 bytes through 0x2024 and 20 bytes afterwards.
- `decal_graphics` preserves the original flat KPTCLS/KPTCIR record and adds a semantic
  `parent` (`KEEP_OUT`) plus `children` geometry view. Restriction bits are decoded as
  placement/trace/copper/via/testpoint/accordion, with unknown bits retained numerically.
- `parttype_decals` is decoded directly from the SDB and includes the ordered primary+alternate
  decal list. The legacy 0x2017 sample confirms duplicated index pairs at PARTTYPE +0x60.
- VIA styles are classified structurally from DecalHeader +0x4C == 0 and are not exported as
  footprints. Unused/orphan PARTDECALs are still retained.
- The Python exporter respects `plated=false`: non-plated drilled pads become KiCad
  `np_thru_hole`. The old unconditional `(attr smd)` was removed.
- KPTCLS is exported as a KiCad rule-area polygon only when every PADS restriction has an exact
  KiCad equivalent. KPTCIR is kept exact in JSON; no silent circle-to-polygon approximation is
  performed. Test-point and accordion restrictions remain explicit unsupported semantics.

Recommended build on an 8-core machine:

```bash
rm -f build/pads-dump/generated/dump_fields.inc \
      build/pads-dump/generated/dump_getters.inc \
      build/pads-dump/generated/schema.txt
cmake --build build --target pads-dump -j8
```

## PADS keepout metadata

PADS KPTCIR/KPTCLS items are decoded as a `KEEP_OUT` parent plus a geometry child.  KiCad
can enforce the P/R/C/V subset (placement, trace, copper pour and via restrictions), but it
has no one-to-one footprint keepout flag for the PADS `T` (test point) or `A` (accordion)
restrictions.  The exporter therefore keeps the original PADS restriction letters in a hidden
footprint property named `PADS_Keepout_Restrictions` instead of rejecting the whole footprint.
If more than one distinct mask occurs in a footprint, the values are separated by semicolons.

KPTCIR is a mathematically exact circle in PADS, while KiCad footprint rule areas are polygonal.
The exporter tessellates KPTCIR to a 64-sided polygon and reports this as a non-fatal warning;
the original restriction mask remains in the hidden property.  Unknown PADS restriction bits
remain fatal because their semantics have not been established.

## v9: archi PARTDECAL e label Reference/Value

- `pads_binary_graphics.cpp` non cerca piu' euristicamente i pool nel file: usa direttamente
  `PADS_SDB` e le sezioni 10/11/12/13/14 del container, con gli stessi record circolari
  versionati usati dal parser KiCad.
- Il campo `vertex.attr` viene risolto come indice relativo `owner.arcStart + attr` nella
  sezione 13. Il bounding box dell'arco viene trasformato in centro/raggio/sweep e serializzato
  in JSON come `arc_from_previous`.
- L'exporter genera `fp_arc` nativi KiCad per gli archi delle decal. Sul campione `CON_BNC`
  del PCB 30439 vengono recuperati tre archi, incluso il tratto con sweep circa -104.7 gradi.
- Le label della decal vengono ricostruite dalla sezione 8 seguendo la catena di associazione
  `0x08 -> field` fino al terminatore `0x0E -> DecalHeader index`. `REF_DES` e `PART_TYPE`
  vengono esportati come proprieta' KiCad native `Reference` e `Value`, mantenendo posizione,
  altezza, spessore, rotazione, layer, visibilita', mirror e giustificazione disponibili.
- La tessellazione `KPTCIR` passa da 128 a 64 lati.

### Nota sul quinto DWORD del record arco

Il DWORD a `Section 13 + 0x10` contiene due `int16` little-endian in decimi di grado:
`low16 = start angle`, `high16 = sweep angle`. Questo e' necessario per distinguere gli archi
maggiori da quelli minori con gli stessi endpoint. Nel `CON_BNC` di 30439, per esempio,
`0x0BD304A2` decodifica `118.6 deg / +302.7 deg`; ridurlo al delta normalizzato `-57.3 deg`
disegnerebbe l'arco sbagliato.

## v10: major arc split e Reference legacy

- Gli archi PADS con `abs(sweep) > 180 deg` non vengono piu' affidati a un singolo
  `fp_arc`. Sono suddivisi sullo stesso centro/raggio in `ceil(abs(sweep)/180)` archi
  KiCad, quindi ogni tratto e' <= 180 gradi. Non e' una tessellazione: la geometria
  circolare resta esatta. Il `CON_BNC` di 30439 passa da un arco PADS di 302.7 gradi
  a due `fp_arc` da 151.35 gradi.
- Sui dialect legacy (`sdb_version <= 0x2022`) il footprint generato aggiunge un
  `fp_text user "${REFERENCE}"` nella posizione/stile del `Ref.Des.` della decal e
  nasconde la proprieta' Reference di libreria. Questo serve al flusso di verifica
  in cui un PCB PADS gia' importato viene aggiornato dalla libreria: KiCad conserva
  lo stato hidden della Reference obbligatoria gia' piazzata, mentre il testo utente
  viene aggiornato insieme al footprint e mostra il vero reference designator.
- Il testo `${REFERENCE}` usa la posizione della **decal di libreria**. Eventuali
  spostamenti per-istanza fatti in PADS richiedono invece il fix del parser/importer
  descritto sotto e una nuova importazione del PCB.

### Fix opzionale del parser/importer KiCad per placement legacy

Nel layout placement da 96 byte (verificato su SDB 0x2017) non esiste il campo +96
usato dai record moderni per raggiungere la lista dei field. La sezione 8 conserva
pero' la relazione completa:

```text
0x08 -> altro field-presentation record
0x16 -> placement index (zero-based)
```

Sul PCB `30439_10_A0.pcb` vengono ricostruite 16 catene su 16 placement; per J4
la catena e' `43 -> 42 -> placement 5`. Il fix v10 ricava la testa della lista dai
link inversi, popola `PART.attributes`, conserva `ATTRIBUTE.level`, e il loader
binario rimappa i field su silkscreen quando il livello PADS punta a rame, come gia'
fa l'importer ASCII.

Poiche' questo fix modifica due file dei sorgenti KiCad, il pacchetto contiene uno
script idempotente che evita di applicare a mano una patch:

```bash
cd /home/michele/sources/kicad-pads-dump/source-rpm/kicad-80ef6f30834e00bdbe57f166489bf47a45a76713
python3 pads-dump/apply_kicad_v10.py --dry-run .
python3 pads-dump/apply_kicad_v10.py .
```

Lo script crea backup `.v10.bak` solo alla prima modifica. Per vedere le posizioni
per-istanza originali delle Reference nel PCB occorre poi ricompilare il componente
KiCad interessato e reimportare il PADS binario. `kicad-v10-legacy-fields.patch` e'
incluso soltanto come diff leggibile; non e' necessario usarlo manualmente.
