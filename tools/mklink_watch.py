#!/usr/bin/env python3
"""Upload a variable table to the MKLink Remote board and/or follow its streams.

The board samples memory with the probe's `cmd.dump_memory(...)` and decodes the
frames itself, but it does not parse ELF files: this script pulls the variable
addresses out of an ELF32 image with the standard library only and POSTs them to
the board's tiny HTTP service.

Examples
--------
    # watch two globals, 1 kHz sampling
    python3 tools/mklink_watch.py --elf build/app.elf --ip 192.168.4.1 \
            --vars g_temperature,g_adc

    # force types / raw addresses
    python3 tools/mklink_watch.py --elf app.elf --ip 192.168.4.1 \
            --vars g_state=u8,g_pitch:f32,0x20001000:u32@magic

    # slower sampling
    python3 tools/mklink_watch.py --elf app.elf --ip 192.168.4.1 \
            --vars g_adc --period-ms 5

    # follow the RTT stream forwarded on the hub port (8081)
    python3 tools/mklink_watch.py --ip 192.168.4.1 --rtt
    python3 tools/mklink_watch.py --ip 192.168.4.1 --rtt --hex
"""

import argparse
import json
import socket
import struct
import sys
import urllib.request

SHT_SYMTAB = 2
STT_OBJECT = 1
STT_NOTYPE = 0
SHN_UNDEF = 0


def parse_elf_symbols(path):
    """Return {name: (address, size)} for object/notype symbols with a size."""
    with open(path, "rb") as fh:
        data = fh.read()

    if data[:4] != b"\x7fELF":
        raise SystemExit("%s is not an ELF file" % path)
    if data[4] == 2:
        raise SystemExit("64-bit ELF is not supported (Cortex-M images are ELF32)")
    if data[5] != 1:
        raise SystemExit("big-endian ELF is not supported")

    (e_shoff,) = struct.unpack_from("<I", data, 0x20)
    (e_shentsize,) = struct.unpack_from("<H", data, 0x2E)
    (e_shnum,) = struct.unpack_from("<H", data, 0x30)

    sections = []
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        (sh_name, sh_type, _flags, _addr, sh_offset, sh_size,
         sh_link, _info, _align, sh_entsize) = struct.unpack_from("<IIIIIIIIII", data, off)
        sections.append({
            "type": sh_type,
            "offset": sh_offset,
            "size": sh_size,
            "link": sh_link,
            "entsize": sh_entsize,
        })

    symbols = {}
    for sec in sections:
        if sec["type"] != SHT_SYMTAB or sec["entsize"] == 0:
            continue
        strtab = sections[sec["link"]]
        count = sec["size"] // sec["entsize"]
        for i in range(count):
            off = sec["offset"] + i * sec["entsize"]
            st_name, st_value, st_size, st_info, _other, st_shndx = struct.unpack_from(
                "<IIIBBH", data, off)
            if st_shndx == SHN_UNDEF or st_name == 0 or st_size == 0:
                continue
            info = st_info & 0xF
            if info not in (STT_OBJECT, STT_NOTYPE):
                continue
            end = data.index(b"\0", strtab["offset"] + st_name)
            name = data[strtab["offset"] + st_name:end].decode("utf-8", "replace")
            symbols[name] = (st_value, st_size)
    return symbols


def guess_type(size):
    return {1: "u8", 2: "u16", 4: "u32", 8: "u64"}.get(size, "u32")


def parse_var_spec(spec, symbols, type_overrides):
    """`name`, `name:type`, `name=type`, `0xADDR:type@name` -> dict."""
    type_name = None
    size = 4
    address = None

    if "@" in spec:
        addr_part, name = spec.split("@", 1)
        if ":" in addr_part:
            addr_part, type_name = addr_part.split(":", 1)
        address = int(addr_part, 0)
    elif "=" in spec:
        name, type_name = spec.split("=", 1)
    elif ":" in spec:
        name, type_name = spec.split(":", 1)
    else:
        name = spec

    if address is None:
        if name not in symbols:
            raise SystemExit("symbol %s not found in ELF (need --elf?)" % name)
        address, size = symbols[name]

    if type_name is None:
        type_name = type_overrides.get(name) or guess_type(size)

    return {"name": name, "addr": "0x%08X" % address, "type": type_name}


def post_table(ip, port, payload):
    url = "http://%s:%d/watch" % (ip, port)
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=8) as resp:
        return resp.read().decode("utf-8", "replace")


def show_http(ip, port, path):
    url = "http://%s:%d%s" % (ip, port, path)
    with urllib.request.urlopen(url, timeout=8) as resp:
        return resp.read().decode("utf-8", "replace")


def follow_stream(ip, port, kind, as_hex):
    sock = socket.create_connection((ip, port), timeout=8)
    sock.sendall((json.dumps({"cmd": kind}) + "\n").encode())
    sock.settimeout(None)
    print("# connected to %s:%d, stream=%s (ctrl-c to stop)" % (ip, port, kind), flush=True)

    pending = b""
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            pending += chunk
            while len(pending) >= 3:
                ftype, length = pending[0], pending[1] | (pending[2] << 8)
                if len(pending) < 3 + length:
                    break
                payload = pending[3:3 + length]
                pending = pending[3 + length:]
                if ftype == 3:
                    sys.stdout.write(payload.decode("utf-8", "replace"))
                elif as_hex:
                    sys.stdout.write(payload.hex(" ") + "\n")
                else:
                    sys.stdout.write(payload.decode("utf-8", "replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description="MKLink Remote SuperWatch uploader")
    ap.add_argument("--ip", default="192.168.4.1", help="board address (AP: 192.168.4.1)")
    ap.add_argument("--port", type=int, default=8080, help="HTTP port (default 8080)")
    ap.add_argument("--hub-port", type=int, default=8081, help="stream hub port (default 8081)")
    ap.add_argument("--elf", help="ELF image used to resolve symbols")
    ap.add_argument("--vars", help="comma separated: name | name:type | 0xADDR:type@name")
    ap.add_argument("--type", action="append", default=[],
                    help="type override, e.g. --type g_pitch=f32")
    ap.add_argument("--period-ms", type=int, default=1, help="sampling period (default 1 ms)")
    ap.add_argument("--rtt", action="store_true", help="follow the RTT stream on the hub port")
    ap.add_argument("--watch-raw", action="store_true",
                    help="follow the raw dump frames on the hub port (hex)")
    ap.add_argument("--hex", action="store_true", help="hex output when following a stream")
    ap.add_argument("--show", action="store_true", help="print GET /watch and exit")
    args = ap.parse_args(argv)

    if args.show:
        print(show_http(args.ip, args.port, "/watch"))
        return 0

    if args.rtt:
        follow_stream(args.ip, args.hub_port, "rtt", args.hex)
        return 0

    if args.watch_raw:
        follow_stream(args.ip, args.hub_port, "watch", True)
        return 0

    if not args.vars:
        ap.error("--vars is required unless --rtt/--watch-raw/--show is used")

    type_overrides = {}
    for item in args.type:
        name, _, value = item.partition("=")
        type_overrides[name] = value

    symbols = {}
    if args.elf:
        symbols = parse_elf_symbols(args.elf)
        print("# %d symbols in %s" % (len(symbols), args.elf))

    specs = [s.strip() for s in args.vars.split(",") if s.strip()]
    variables = [parse_var_spec(spec, symbols, type_overrides) for spec in specs]

    payload = {"period_ms": args.period_ms, "vars": variables}
    print("# uploading %d variables @ %d ms" % (len(variables), args.period_ms))
    for var in variables:
        print("#   %-20s %s %s" % (var["name"], var["addr"], var["type"]))

    print(post_table(args.ip, args.port, payload).strip())
    return 0


if __name__ == "__main__":
    sys.exit(main())
