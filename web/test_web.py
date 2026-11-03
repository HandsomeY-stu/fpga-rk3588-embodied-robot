#!/usr/bin/env python3
"""Offline console regression: loopback HTTP with serial/process access mocked."""
import importlib.util
import json
import os
import sys
import threading
import types
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

class FakeUART:
    """Record protocol intent without touching a device."""
    def __init__(self):
        self.calls = []
    def stop(self):
        self.calls.append(('stop',))
        return True
    def mode(self, mode):
        self.calls.append(('mode', mode))
        return True
    def osd(self, mode):
        self.calls.append(('osd', mode))
        return True

class ConsoleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if importlib.util.find_spec('termios') is None:
            sys.modules['termios'] = types.ModuleType('termios')
        spec = importlib.util.spec_from_file_location('console_backend_under_test', HERE / 'server.py')
        cls.backend = importlib.util.module_from_spec(spec)
        # Uart is instantiated on import in the inherited service. Block that attempt.
        with mock.patch.object(os, 'O_NOCTTY', getattr(os, 'O_NOCTTY', 0), create=True), \
             mock.patch.object(os, 'O_NONBLOCK', getattr(os, 'O_NONBLOCK', 0), create=True), \
             mock.patch('os.open', side_effect=OSError('offline test: device access disabled')):
            spec.loader.exec_module(cls.backend)
        cls.backend.U = FakeUART()
        cls.backend.Handler.log_message = lambda *args: None
        cls.http = ThreadingHTTPServer(('127.0.0.1', 0), cls.backend.Handler)
        cls.thread = threading.Thread(target=cls.http.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = 'http://127.0.0.1:' + str(cls.http.server_address[1])

    @classmethod
    def tearDownClass(cls):
        cls.http.shutdown()
        cls.http.server_close()
        cls.thread.join(timeout=3)

    def setUp(self):
        self.backend.S = self.backend.State()
        self.backend.U.calls.clear()

    def request(self, path, payload=None):
        data = None if payload is None else json.dumps(payload).encode()
        req = urllib.request.Request(self.base + path, data=data, headers={'Content-Type': 'application/json'})
        try:
            response = urllib.request.urlopen(req, timeout=3)
        except urllib.error.HTTPError as exc:
            response = exc
        with response:
            return response.status, response.headers, response.read()

    def test_page_and_task_controls(self):
        code, _, data = self.request('/')
        self.assertEqual(code, 200)
        text = data.decode()
        for label in ('任务三', '任务四', '任务五', '任务六', '/static/style.css', '/static/app.js'):
            self.assertIn(label, text)
        self.assertEqual(self.backend.U.calls, [])

    def test_stylesheet(self):
        code, headers, data = self.request('/static/style.css')
        self.assertEqual(code, 200)
        self.assertIn('text/css', headers['Content-Type'])
        self.assertGreater(len(data), 100)

    def test_javascript_and_query(self):
        code, headers, data = self.request('/static/app.js?v=1')
        self.assertEqual(code, 200)
        self.assertIn('javascript', headers['Content-Type'])
        self.assertIn(b'/api/status', data)
        self.assertIn(b'/api/estop', data)

    def test_static_path_whitelist(self):
        for path in ('/static/../../firmware/integrated_t4v48_uc325c.fs', '/static/%2e%2e/server.py', '/static/unknown.txt'):
            self.assertEqual(self.request(path)[0], 404)

    def test_status_without_telemetry(self):
        code, _, data = self.request('/api/status')
        value = json.loads(data)
        self.assertEqual(code, 200)
        self.assertFalse(value['tlm_ok'])
        self.assertIsNone(value['fpga_mode'])
        self.assertEqual(self.backend.U.calls, [])

    def test_invalid_mode_does_not_send(self):
        _, _, data = self.request('/api/fpga_mode', {'mode': 99})
        self.assertFalse(json.loads(data)['ok'])
        self.assertEqual(self.backend.U.calls, [])

    def test_auto_mode_needs_fresh_telemetry(self):
        _, _, data = self.request('/api/fpga_mode', {'mode': 1})
        self.assertFalse(json.loads(data)['ok'])
        self.assertEqual(self.backend.U.calls, [])

    def test_stop(self):
        _, _, data = self.request('/api/stop', {})
        self.assertTrue(json.loads(data)['ok'])
        self.assertIn(('stop',), self.backend.U.calls)
        self.assertTrue(self.backend.S.tcp_inhibit)

    def test_estop(self):
        _, _, data = self.request('/api/estop', {})
        self.assertTrue(json.loads(data)['ok'])
        self.assertGreaterEqual(self.backend.U.calls.count(('stop',)), 5)

    def test_target_config(self):
        _, _, data = self.request('/api/target', {'key': 'bag'})
        self.assertEqual(json.loads(data)['cls'], '27,31,33')
        self.assertEqual(self.backend.U.calls, [])

    def test_osd2_stays_rejected(self):
        code, _, data = self.request('/api/osd', {'mode': 2})
        self.assertEqual(code, 400)
        self.assertFalse(json.loads(data)['ok'])
        self.assertEqual(self.backend.U.calls, [])

    def test_paths_resolve_in_repository(self):
        root = HERE.parent
        self.assertEqual(Path(self.backend.HERE), root / 'software/task6_red_bottle')
        self.assertTrue(Path(self.backend.GRAB_SCRIPT).is_file())
        self.assertTrue(Path(self.backend.FRAME_SCRIPT).is_file())

    def test_detector_keeps_exclusive_camera_ownership(self):
        self.backend.S.person_proc = mock.Mock()
        self.backend.S.person_proc.poll.return_value = None
        with mock.patch.object(self.backend.subprocess, 'Popen') as start:
            self.backend.start_vision()
        start.assert_not_called()

    def test_preview_keeps_exclusive_camera_ownership(self):
        self.backend.S.preview_proc = mock.Mock()
        self.backend.S.preview_proc.poll.return_value = None
        with mock.patch.object(self.backend.subprocess, 'Popen') as start:
            self.backend.start_vision()
        start.assert_not_called()

if __name__ == '__main__':
    unittest.main(verbosity=2)
