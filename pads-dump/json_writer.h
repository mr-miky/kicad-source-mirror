// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <optional>
#include <ostream>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

namespace dump {
inline void quote(std::ostream& out, std::string_view s) {
    static constexpr char hex[] = "0123456789abcdef";
    out << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') out << '\\' << char(c);
        else if (c < 32 || c >= 127) {
            // Byte-preserving escape: see README for string encoding semantics.
            out << "\\u00" << hex[c >> 4] << hex[c & 15];
        } else out << char(c);
    }
    out << '"';
}
template<class T> struct optional_type : std::false_type {};
template<class T> struct optional_type<std::optional<T>> : std::true_type {};
template<class T> struct pair_type : std::false_type {};
template<class A, class B> struct pair_type<std::pair<A,B>> : std::true_type {};
template<class T> void write(std::ostream&, const T&);
template<class T> void object(std::ostream&, const T&);

template<class T> void write(std::ostream& out, const T& v) {
    using U = std::remove_cvref_t<T>;
    if constexpr (std::is_same_v<U, std::string> || std::is_same_v<U, std::string_view>) quote(out, v);
    else if constexpr (std::is_same_v<U, bool>) out << (v ? "true" : "false");
    else if constexpr (std::is_enum_v<U>) write(out, static_cast<std::underlying_type_t<U>>(v));
    else if constexpr (std::is_integral_v<U>) {
        if constexpr (std::is_signed_v<U>) out << static_cast<long long>(v);
        else out << static_cast<unsigned long long>(v);
    }
    else if constexpr (std::is_floating_point_v<U>) {
        if (!std::isfinite(v)) throw std::runtime_error("Valore numerico non finito nel parser");
        out << std::setprecision(17) << v;
    }
    else if constexpr (optional_type<U>::value) {
        if (v) write(out, *v); else out << "null";
    }
    else if constexpr (pair_type<U>::value) {
        out << '['; write(out, v.first); out << ','; write(out, v.second); out << ']';
    }
    else if constexpr (requires { typename U::mapped_type; }) {
        // Map entries preserve non-string keys without coercion.
        out << '['; bool first = true;
        for (const auto& [k, value] : v) {
            if (!first) out << ',';
        first = false;
            out << "{\"key\":"; write(out, k);
            out << ",\"value\":"; write(out, value); out << '}';
        }
        out << ']';
    }
    else if constexpr (std::ranges::range<U>) {
        out << '['; bool first = true;
        for (const auto& value : v) {
            if (!first) out << ',';
            first = false;
            write(out, value);
        }
        out << ']';
    }
    else object(out, v);
}
template<class T> void member(std::ostream& out, bool& first, std::string_view name, const T& v) {
    if (!first) out << ',';
        first = false;
    quote(out, name); out << ':'; write(out, v);
}
template<class T> void object(std::ostream& out, const T& v) {
    out << '{'; bool first = true;
#define DUMP_FIELD(name) if constexpr (requires { v.name; }) member(out, first, #name, v.name);
#include "dump_fields.inc"
#undef DUMP_FIELD
    if (first) throw std::runtime_error("Tipo senza campi serializzabili: aggiornare generate.py");
    out << '}';
}
}
