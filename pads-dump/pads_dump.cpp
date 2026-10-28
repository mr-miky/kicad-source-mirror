// SPDX-License-Identifier: GPL-3.0-or-later
#include "pads_binary_parser.h"
#include "json_writer.h"
#include "pads_binary_graphics.h"
#include "pads_binary_parttypes.h"
#include <wx/app.h>
#include <wx/init.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

// Console application: initialize wx base without requiring a GUI/display.
class DumpApp : public wxAppConsole {};
wxIMPLEMENT_APP_NO_MAIN(DumpApp);

static int runExporter(const std::string& jsonPath, const std::string& prettyPath, bool incomplete) {
    const pid_t pid = fork();
    if (pid < 0) throw std::runtime_error("fork() fallita");
    if (pid == 0) {
        if (incomplete)
            execl(PADS_DUMP_PYTHON_PATH, PADS_DUMP_PYTHON_PATH, PADS_DUMP_EXPORTER_PATH, jsonPath.c_str(),
                   prettyPath.c_str(), "--allow-incomplete", (char*) nullptr);
        else
            execl(PADS_DUMP_PYTHON_PATH, PADS_DUMP_PYTHON_PATH, PADS_DUMP_EXPORTER_PATH, jsonPath.c_str(),
                   prettyPath.c_str(), (char*) nullptr);
        _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) throw std::runtime_error("waitpid() fallita");
    return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
}

int main(int argc, char** argv) {
    const bool exportMode = argc >= 2 && std::string(argv[1]) == "--export-decals";
    const bool incomplete = exportMode && argc == 5 && std::string(argv[4]) == "--allow-incomplete";
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Uso: pads-dump input.pcb [output.json]\n"
                  << "     pads-dump --export-decals input.pcb Libreria.pretty [--allow-incomplete]\n"; return 0;
    }
    if ((!exportMode && (argc < 2 || argc > 3)) || (exportMode && !(argc == 4 || incomplete))) {
        std::cerr << "Uso: pads-dump input.pcb [output.json] | --export-decals input.pcb Libreria.pretty [--allow-incomplete]\n"; return 2;
    }
    try {
        wxInitializer wx;
        if (!wx.IsOk()) throw std::runtime_error("Inizializzazione wxWidgets fallita");
        const auto input = std::filesystem::absolute(argv[exportMode ? 2 : 1]);
        if (!exportMode && argc == 3) {
            const auto output = std::filesystem::absolute(argv[2]);
            if (input.lexically_normal() == output.lexically_normal()
                || (std::filesystem::exists(output) && std::filesystem::equivalent(input, output)))
                throw std::runtime_error("Il file di uscita coincide con il PCB di ingresso");
        }
        PADS_IO::BINARY_PARSER parser;
        parser.Parse(wxString::FromUTF8(input.string().c_str()));
        // Serialize completely before opening output: parsing/serialization errors
        // do not leave a truncated dump or overwrite an existing result.
        std::ostringstream json;
        json.imbue(std::locale::classic());
        json << "{\n\"format\":\"kicad-pads-parser-dump-v1\",\n\"source\":";
        dump::quote(json, input.string());
        json << ",\n\"string_encoding\":\"byte-preserving-u00xx\",\n\"parser\":{\n";
        bool first = true;
#define DUMP_GETTER(name) dump::member(json, first, #name, parser.name()); json << '\n';
#include "dump_getters.inc"
#undef DUMP_GETTER
        json << "},\n\"decal_graphics\":";
        try { pads_dump::write_decal_graphics_json(json, input); }
        catch (const std::exception& e) {
            std::cerr << "Geometria decal non disponibile: " << e.what() << "\n";
            json << "{}";
        }
        json << ",\n\"parttype_decals\":";
        try { pads_dump::write_parttype_decals_json(json, input); }
        catch (const std::exception& e) {
            std::cerr << "Relazioni PARTTYPE/decal non disponibili: " << e.what() << "\n";
            json << "{\"parttypes\":[]}";
        }
        json << "\n}\n";
        if (exportMode) {
            // Temporary dump is private; the existing parser/serializer is reused.
            char tmp[] = "/tmp/pads-dump-export-XXXXXX";
            const int fd = mkstemp(tmp);
            if (fd == -1) throw std::runtime_error("mkstemp() fallita");
            close(fd);
            const std::filesystem::path temp(tmp);
            try {
                {
                    std::ofstream output(temp, std::ios::binary | std::ios::trunc);
                    output << json.str();
                    if (!output) throw std::runtime_error("Errore scrivendo il dump temporaneo");
                }
                const int result = runExporter(temp.string(),
                               std::filesystem::absolute(argv[3]).string(), incomplete);
                std::filesystem::remove(temp);
                return result;
            } catch (...) {
                std::filesystem::remove(temp);
                throw;
            }
        } else if (argc == 3) {
            std::ofstream output(argv[2], std::ios::binary | std::ios::trunc);
            if (!output) throw std::runtime_error("Impossibile aprire il file di uscita");
            output << json.str(); output.close();
            if (!output) throw std::runtime_error("Errore scrivendo il dump");
        } else {
            std::cout << json.str();
            if (!std::cout) throw std::runtime_error("Errore scrivendo su stdout");
        }
        std::cerr << "Dump completato: " << parser.GetParts().size() << " componenti, "
                  << parser.GetNets().size() << " net, " << parser.GetRoutes().size() << " route\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "pads-dump: " << e.what() << '\n'; return 1;
    }
}
