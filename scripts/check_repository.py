#!/usr/bin/env python3
"""Check source syntax, project paths, local includes and firmware integrity; no hardware I/O."""
import ast
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def main():
    errors = []
    py_files = list(ROOT.rglob('*.py'))
    for path in py_files:
        try:
            ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
        except (SyntaxError, UnicodeError) as exc:
            errors.append(f'{path.relative_to(ROOT)}: {exc}')
    project = ET.parse(ROOT / 'fpga/ov5640_ddr3_hdmi.gprj')
    refs = [x.attrib['path'] for x in project.findall('.//File') if x.attrib.get('enable') == '1']
    for ref in refs:
        if not (ROOT / 'fpga' / ref).is_file():
            errors.append('Missing FPGA source: ' + ref)
    for ref in ('web/app.py', 'web/server.py', 'web/fpga_web_modes.py', 'web/assets.py',
                'web/static/index.html', 'web/static/style.css', 'web/static/app.js',
                'web/config/carweb.env.example', 'web/config/robot-console.service.example'):
        if not (ROOT / ref).is_file():
            errors.append('Missing host-console input: ' + ref)
    detector = ROOT / 'software/detector'
    includes = [detector / name for name in ('include', 'src', 'uart', 'sim')]
    for path in detector.rglob('*'):
        if path.suffix not in ('.cpp', '.hpp', '.h'):
            continue
        for inc in re.findall(r'^\s*#include\s+"([^"]+)"', path.read_text(encoding='utf-8'), re.M):
            if not any((base / inc).is_file() for base in [path.parent] + includes):
                errors.append(f'Missing C++ include: {path.relative_to(ROOT)} -> {inc}')
    for path in ROOT.rglob('*'):
        if any(part == '.git' for part in path.relative_to(ROOT).parts):
            continue
        if path.is_file() and ('.bak' in path.name or path.suffix in ('.log', '.jsonl', '.pyc')):
            errors.append('Unexpected generated/history file: ' + str(path.relative_to(ROOT)))
    firmware = ROOT / 'firmware/integrated_t4v48_uc325c.fs'
    expected = '205046b2936cde17a770f291c3819977b2f951901ccf9222402300de8d5b0827'
    if hashlib.sha256(firmware.read_bytes()).hexdigest() != expected:
        errors.append('Firmware SHA256 mismatch')
    manifest = json.loads((ROOT / 'firmware/source_manifest.json').read_text(encoding='utf-8'))
    for ref, digest in manifest['submission_fpga_hashes'].items():
        path = ROOT / 'fpga' / ref
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            errors.append('Delivered FPGA SHA256 mismatch: ' + ref)
    bound = dict(manifest['build_reports'])
    bound.update(manifest['web_integration']['web_files_sha256'])
    bound[manifest['board_validation']['path']] = manifest['board_validation']['sha256']
    bound[manifest['runtime_assets']['manifest']] = manifest['runtime_assets']['sha256']
    for ref, digest in bound.items():
        path = ROOT / ref
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            errors.append('Bound delivery SHA256 mismatch: ' + ref)
    assets = json.loads((ROOT / 'models/assets_manifest.json').read_text(encoding='utf-8'))
    for ref, record in assets['files'].items():
        path = ROOT / 'models' / ref
        if not path.is_file():
            errors.append('Missing runtime asset: ' + ref)
        elif path.stat().st_size != record['bytes'] or hashlib.sha256(path.read_bytes()).hexdigest() != record['sha256']:
            errors.append('Runtime asset size/SHA256 mismatch: ' + ref)
    for error in errors:
        print('ERROR:', error)
    print(f'Checked {len(py_files)} Python files, {len(refs)} FPGA project inputs and local C++ includes.')
    return 1 if errors else 0

if __name__ == '__main__':
    sys.exit(main())
