#!/usr/bin/env python3
"""
compute_storage_symbols.py

Scan board yaml files under this boards directory and corresponding hwdef.dat
entries under libraries/AP_HAL_HPMICRO/hwdef to compute and suggest values for
HAL_HPM_STORAGE_FLASH_BASE_ADDR and HAL_HPM_STORAGE_OFFSET_ADDR.

Usage:
    python3 compute_storage_symbols.py

By default the script prints per-board suggestions as JSON objects to stdout.
"""
import argparse
import os
import re
import sys
import json


def parse_size_str(s):
    # accepts strings like '4M', '32M', '512K', or bare numbers (bytes)
    if s is None:
        return None
    s = str(s).strip()
    m = re.match(r"^(\d+)\s*([KkMm]|KB|Mb|MB|kb|mb)?$", s)
    if not m:
        return None
    val = int(m.group(1))
    unit = m.group(2)
    if not unit:
        return val
    unit = unit.upper()
    if unit in ("K", "KB"):
        return val * 1024
    if unit in ("M", "MB"):
        return val * 1024 * 1024
    return val


def find_yaml_flash_size(yaml_path):
    # naive yaml parse: look for on-board-flash: then size: line under it
    flash_size = None
    with open(yaml_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    in_flash = False
    for ln in lines:
        if re.match(r"^\s*on-board-flash\s*:\s*$", ln):
            in_flash = True
            continue
        if in_flash:
            m = re.match(r"^\s*size\s*:\s*(\S+)", ln)
            if m:
                flash_size = parse_size_str(m.group(1))
                break
            # leave flash block if dedent
            if re.match(r"^\S", ln):
                in_flash = False
    return flash_size


def read_hwdef(hwdef_path):
    with open(hwdef_path, 'r', encoding='utf-8') as f:
        return f.read()


def extract_hal_storage_size_from_hwdef(text):
    # search for either 'define HAL_STORAGE_SIZE 12345' or '#sym:HAL_STORAGE_SIZE 12345'
    m = re.search(r"define\s+HAL_STORAGE_SIZE\s+(\d+)", text)
    if m:
        return int(m.group(1))
    m = re.search(r"#sym:\s*HAL_STORAGE_SIZE\s+(\d+)", text)
    if m:
        return int(m.group(1))
    return None


def extract_define_hex(text, symbol):
    m = re.search(r"define\s+%s\s+(0x[0-9A-Fa-f]+)" % re.escape(symbol), text)
    if m:
        return int(m.group(1), 16)
    return None


def suggest_values(flash_size_bytes, storage_size_bytes):
    # storage region placed at end of flash
    if flash_size_bytes is None or storage_size_bytes is None:
        return None, None
    offset = flash_size_bytes - storage_size_bytes
    base = 0x80000000 + offset
    return base, offset


def format_hex(v):
    return "0x%08x" % (v if v is not None else 0)


# No in-place update in this variant; callers expect the values returned/printed.


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--boards-dir', default=os.path.join(os.path.dirname(__file__), 'boards'), help='boards directory containing subfolders with yaml')
    parser.add_argument('--hwdef-dir', default=os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'hwdef')), help='hwdef directory to find hwdef.dat for each board (default: libraries/AP_HAL_HPMICRO/hwdef relative)')
    args = parser.parse_args()

    boards_dir = os.path.abspath(args.boards_dir)
    hwdef_root = os.path.abspath(os.path.join(os.path.dirname(boards_dir), 'hwdef'))
    # allow user override
    if args.hwdef_dir:
        hwdef_root = os.path.abspath(args.hwdef_dir)

    # print machine-friendly header
    # iterate and emit one JSON object per board

    # find yaml files in subdirs
    boards = []
    for entry in os.listdir(boards_dir):
        p = os.path.join(boards_dir, entry)
        if not os.path.isdir(p):
            continue
        # look for a yaml file named after the folder
        y1 = os.path.join(p, entry + '.yaml')
        if os.path.exists(y1):
            boards.append((entry, y1))
            continue
        # fallback: scan for any .yaml in subdir
        for fn in os.listdir(p):
            if fn.endswith('.yaml'):
                boards.append((entry, os.path.join(p, fn)))
                break

    if not boards:
        # try fallback: some repo layouts have an extra 'boards' subdir
        nested = os.path.join(boards_dir, 'boards')
        if os.path.isdir(nested):
            boards_dir = nested
            for entry in os.listdir(boards_dir):
                p = os.path.join(boards_dir, entry)
                if not os.path.isdir(p):
                    continue
                y1 = os.path.join(p, entry + '.yaml')
                if os.path.exists(y1):
                    boards.append((entry, y1))
                    continue
                for fn in os.listdir(p):
                    if fn.endswith('.yaml'):
                        boards.append((entry, os.path.join(p, fn)))
                        break

    if not boards:
        print('no boards found in', boards_dir)
        return 1

    for name, yaml_path in boards:
        flash = find_yaml_flash_size(yaml_path)
        if flash is None:
            # emit not found
            out = {
                'board': name,
                'yaml': yaml_path,
                'error': 'flash size not found in yaml'
            }
            print(json.dumps(out))
            continue

        # find hwdef
        hwdef_path = os.path.join(hwdef_root, name, 'hwdef.dat')
        if not os.path.exists(hwdef_path):
            out = {
                'board': name,
                'yaml': yaml_path,
                'flash_bytes': flash,
                'error': 'hwdef.dat not found'
            }
            print(json.dumps(out))
            continue

        txt = read_hwdef(hwdef_path)
        storage_size = extract_hal_storage_size_from_hwdef(txt)
        used_default_storage = False
        if storage_size is None:
            # default suggestion 64KB
            storage_size = 64 * 1024
            used_default_storage = True

        cur_base = extract_define_hex(txt, 'HAL_HPM_STORAGE_FLASH_BASE_ADDR')
        cur_off = extract_define_hex(txt, 'HAL_HPM_STORAGE_OFFSET_ADDR')

        base, offset = suggest_values(flash, storage_size)
        if base is None:
            out = {
                'board': name,
                'yaml': yaml_path,
                'flash_bytes': flash,
                'storage_bytes': storage_size,
                'error': 'could not compute suggestion'
            }
            print(json.dumps(out))
            continue

        out = {
            'board': name,
            'yaml': yaml_path,
            'hwdef': hwdef_path,
            'flash_bytes': flash,
            'storage_bytes': storage_size,
            'used_default_storage': used_default_storage,
            'current': {
                'HAL_HPM_STORAGE_FLASH_BASE_ADDR': format_hex(cur_base) if cur_base is not None else None,
                'HAL_HPM_STORAGE_OFFSET_ADDR': format_hex(cur_off) if cur_off is not None else None,
            },
            'suggestion': {
                'HAL_HPM_STORAGE_FLASH_BASE_ADDR': format_hex(base),
                'HAL_HPM_STORAGE_OFFSET_ADDR': format_hex(offset)
            }
        }
        print(json.dumps(out))

    return 0


if __name__ == '__main__':
    sys.exit(main())
