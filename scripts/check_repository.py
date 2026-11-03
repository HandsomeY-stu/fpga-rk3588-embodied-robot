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


def read_gowin_project(path):
    """Accept Gowin's legacy XML version=1 declaration without changing the file."""
    data = path.read_bytes()
    # Some Expat releases reject this vendor declaration. Only the XML prolog
    # is adapted; the device, project version and source list remain untouched.
    data = re.sub(
        rb'\A((?:\xef\xbb\xbf)?<\?xml[ \t\r\n]+version[ \t\r\n]*=[ \t\r\n]*)([\'"])1\2',
        lambda match: match[1] + match[2] + b'1.0' + match[2],
        data, count=1)
    return ET.fromstring(data)


def hash_matches(path, expected, portable=None, expected_size=None):
    """Check exact bytes, or an explicitly recorded CRLF-to-LF equivalent."""
    if not path.is_file():
        return False
    data = path.read_bytes()
    if (expected_size is None or len(data) == expected_size) and hashlib.sha256(data).hexdigest() == expected:
        return True
    # Git's text attributes normalize line endings. The alternate digest is
    # bound to the original digest/size, and never ignores any other changes.
    if not portable or portable.get('raw_sha256') != expected:
        return False
    if expected_size is not None and portable.get('raw_bytes') != expected_size:
        return False
    normalized = data.replace(b'\r\n', b'\n')
    return (len(normalized) == portable['lf_bytes']
            and hashlib.sha256(normalized).hexdigest() == portable['lf_sha256'])


def main():
    errors = []
    py_files = list(ROOT.rglob('*.py'))
    for path in py_files:
        try:
            ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
        except (SyntaxError, UnicodeError) as exc:
            errors.append(f'{path.relative_to(ROOT)}: {exc}')
    refs = []
    try:
        project = read_gowin_project(ROOT / 'fpga/ov5640_ddr3_hdmi.gprj')
        refs = [x.attrib['path'] for x in project.findall('.//File') if x.attrib.get('enable') == '1']
    except (OSError, ET.ParseError) as exc:
        errors.append(f'Cannot read Gowin project: {exc}')
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
    portable = manifest.get('portable_text_hashes', {})
    for ref, digest in manifest['submission_fpga_hashes'].items():
        path = ROOT / 'fpga' / ref
        if not hash_matches(path, digest, portable.get('fpga/' + ref)):
            errors.append('Delivered FPGA SHA256 mismatch: ' + ref)
    bound = dict(manifest['build_reports'])
    bound.update(manifest['web_integration']['web_files_sha256'])
    bound[manifest['board_validation']['path']] = manifest['board_validation']['sha256']
    bound[manifest['runtime_assets']['manifest']] = manifest['runtime_assets']['sha256']
    for ref, digest in bound.items():
        path = ROOT / ref
        if not hash_matches(path, digest, portable.get(ref)):
            errors.append('Bound delivery SHA256 mismatch: ' + ref)
    assets = json.loads((ROOT / 'models/assets_manifest.json').read_text(encoding='utf-8'))
    for ref, record in assets['files'].items():
        path = ROOT / 'models' / ref
        if not path.is_file():
            errors.append('Missing runtime asset: ' + ref)
        elif not hash_matches(path, record['sha256'], portable.get('models/' + ref), record['bytes']):
            errors.append('Runtime asset size/SHA256 mismatch: ' + ref)
    for error in errors:
        print('ERROR:', error)
    print(f'Checked {len(py_files)} Python files, {len(refs)} FPGA project inputs and local C++ includes.')
    return 1 if errors else 0

if __name__ == '__main__':
    sys.exit(main())
