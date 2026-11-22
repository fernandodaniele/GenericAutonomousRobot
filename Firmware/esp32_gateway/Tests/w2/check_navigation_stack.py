"""Comprueba frames Xtensa del ELF real; no requiere hardware ni paquetes.

Es una cota inferior de una cadena real, no una medicion del high-water mark.
Un fallo demuestra falta de espacio; pasar no certifica todo el stack del SDK.
"""
import argparse
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("objdump", type=Path)
    parser.add_argument("httpd_header", type=Path)
    args = parser.parse_args()
    header = args.httpd_header.read_text(encoding="utf-8")
    default = re.search(r"#define HTTPD_DEFAULT_CONFIG\(\).*?\.stack_size\s*=\s*(\d+)", header, re.S)
    if default is None:
        raise SystemExit("FAIL: no se reconoce el stack predeterminado del SDK")
    budget = int(default[1])
    assembly = subprocess.check_output([str(args.objdump.resolve()), "-d", str(args.elf.resolve())], text=True)
    # Incluye los frames SDK que permanecen activos al invocar el handler.
    chain = ["httpd_thread", "httpd_server", "httpd_process_session",
             "httpd_sess_process", "httpd_req_new", "httpd_parse_req", "httpd_uri",
             "WebServer_Navigation", "Stm32Navigation_Json", "snprintf", "_svfprintf_r"]
    # AUTO-2E agrega el objeto compatible de estabilizacion tras el JSON base.
    # Si no se inlineo, este frame tambien precede a snprintf/newlib.
    if re.search(r"^[0-9a-f]+ <Navigation_WaitJson>:", assembly, re.M):
        chain.insert(-2, "Navigation_WaitJson")
    frames = {}
    for name in chain:
        match = re.search(r"^[0-9a-f]+ <" + re.escape(name) +
                          r">:\n[^\n]*\bentry\s+a1,\s*(0x[0-9a-f]+|\d+)", assembly, re.M)
        if match is None:
            raise SystemExit("FAIL: revisar cadena/ISA; falta frame de " + name)
        frames[name] = int(match[1], 0)
        print(f"{name}: {frames[name]} bytes")
    total = sum(frames.values())
    print(f"Frames de la cadena: {total}; stack HTTP: {budget}; diferencia: {budget-total}")
    if total >= budget:
        raise SystemExit("FAIL: la cadena local excede el stack HTTP antes de llamadas auxiliares/contexto")
    if frames["WebServer_Navigation"] > 128:
        raise SystemExit("FAIL: el handler vuelve a reservar un frame grande en stack")
    print("PASS: buffer NAV fuera del stack HTTP; margen real pendiente de medir en hardware")


if __name__ == "__main__":
    main()
