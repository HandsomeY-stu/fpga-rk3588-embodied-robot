#!/usr/bin/env python3
"""Regress Gowin XML compatibility and byte/line-ending integrity checks."""
import hashlib
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

# The following repository checker rejects generated .pyc files.
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_repository import ROOT, hash_matches, read_gowin_project


class RepositoryChecks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='robot-repo-check-')
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'input'

    def parse(self, data):
        self.path.write_bytes(data)
        parsed = read_gowin_project(self.path)
        self.assertEqual(self.path.read_bytes(), data)
        return parsed

    def test_original_gowin_project_has_all_55_inputs(self):
        path = ROOT / 'fpga/ov5640_ddr3_hdmi.gprj'
        before = path.read_bytes()
        project = read_gowin_project(path)
        refs = [x.attrib['path'] for x in project.findall('.//File') if x.attrib.get('enable') == '1']
        self.assertEqual(len(refs), 55)
        # Linux paths are case-sensitive, unlike this project's Windows host.
        names = {p.relative_to(ROOT / 'fpga').as_posix() for p in (ROOT / 'fpga').rglob('*') if p.is_file()}
        self.assertTrue(set(refs).issubset(names))
        self.assertEqual(path.read_bytes(), before)

    def test_legacy_declaration_does_not_change_project_attributes(self):
        project = self.parse(b'<?xml version="1" encoding="UTF-8"?>\r\n<Project version="1"><Version>5</Version></Project>')
        self.assertEqual(project.attrib['version'], '1')
        self.assertEqual(project.findtext('Version'), '5')

    def test_legacy_single_quotes_and_bom(self):
        project = self.parse(b"\xef\xbb\xbf<?xml version = '1' encoding='UTF-8'?>\n<Project/>")
        self.assertEqual(project.tag, 'Project')

    def test_standard_declaration(self):
        self.assertEqual(self.parse(b'<?xml version="1.0"?><Project/>').tag, 'Project')

    def test_malformed_body_is_still_rejected(self):
        self.path.write_bytes(b'<?xml version="1"?><Project><File></Project>')
        with self.assertRaises(ET.ParseError):
            read_gowin_project(self.path)

    @staticmethod
    def record(data):
        normalized = data.replace(b'\r\n', b'\n')
        return dict(raw_sha256=hashlib.sha256(data).hexdigest(), raw_bytes=len(data),
                    lf_sha256=hashlib.sha256(normalized).hexdigest(), lf_bytes=len(normalized))

    def test_exact_binary_digest_remains_supported(self):
        data = b'\x00\xff\r\n\x10'
        self.path.write_bytes(data)
        self.assertTrue(hash_matches(self.path, hashlib.sha256(data).hexdigest()))
        self.path.write_bytes(data.replace(b'\r\n', b'\n'))
        self.assertFalse(hash_matches(self.path, hashlib.sha256(data).hexdigest()))

    def test_git_lf_checkout_and_crlf_checkout_match_recorded_text(self):
        raw = b'module robot;\r\nendmodule\r\n'
        record = self.record(raw)
        for data in (raw, raw.replace(b'\r\n', b'\n')):
            self.path.write_bytes(data)
            self.assertTrue(hash_matches(self.path, record['raw_sha256'], record, len(raw)))

    def test_content_change_fails_even_with_normalized_line_endings(self):
        record = self.record(b'module robot;\r\nendmodule\r\n')
        self.path.write_bytes(b'module changed;\nendmodule\n')
        self.assertFalse(hash_matches(self.path, record['raw_sha256'], record))

    def test_stale_portable_record_does_not_authorize_different_expected_digest(self):
        record = self.record(b'first\r\n')
        self.path.write_bytes(b'first\n')
        self.assertFalse(hash_matches(self.path, '0' * 64, record))
        self.assertFalse(hash_matches(self.path, record['raw_sha256'], record, 999))

    def test_missing_file_fails(self):
        self.assertFalse(hash_matches(self.path, '0' * 64))


if __name__ == '__main__':
    unittest.main(verbosity=2)
