import base64
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from orm_r3_test_store import TestSessionStore, TestSessionExists

# HTTP route tests require no serial hardware or kinematics dependencies.
with patch.dict(sys.modules, {"orm_r3_control": types.SimpleNamespace(ORMR3Control=object)}):
    from orm_r3_control_server import ORMR3ControlRequestHandler

PNG = base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jRZkAAAAASUVORK5CYII=')


class TestSessions(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = TestSessionStore(self.temp.name)
        self.data = dict(test_id='run-1', position_id='point-1',
                         image='data:image/png;base64,' + base64.b64encode(PNG).decode(),
                         desired_cartesian_coordinates={'x': .1, 'y': 0, 'z': .2},
                         desired_joint_angles=[1, 2, 3, 4, 5, 6], actual_joint_angles=[0]*6)

    def post(self, path, data):
        handler = object.__new__(ORMR3ControlRequestHandler)
        handler.path = path
        handler.test_store = self.store
        payload = json.dumps(data).encode()
        handler.headers = {'Content-Length': str(len(payload))}
        handler.rfile = io.BytesIO(payload)
        responses = []
        handler._send_json = lambda code, body: responses.append((code, body))
        handler.log_error = lambda *args: None
        handler.do_POST()
        return responses[0]

    def test_routes_and_appending(self):
        self.assertEqual(self.post('/test/start', {'test_id': 'run-1'})[0], 201)
        folder = Path(self.temp.name) / 'run-1'
        self.assertEqual(json.loads((folder/'test_records.json').read_text()), [])
        for _ in range(2):
            code, response = self.post('/test/state', self.data)
            self.assertEqual(code, 201)
            self.assertEqual((folder/response['record']['image']).read_bytes(), PNG)
        records = json.loads((folder/'test_records.json').read_text())
        self.assertEqual(len(records), 2)
        self.assertNotEqual(records[0]['image'], records[1]['image'])
        self.assertEqual(records[0]['desired_joint_angles'], self.data['desired_joint_angles'])
        self.assertEqual(self.post('/test/start', {'test_id': 'run-1'})[0], 409)
        self.assertEqual(len(json.loads((folder/'test_records.json').read_text())), 2)

    def test_missing_actual_feedback(self):
        self.store.start('run-1')
        self.data['actual_joint_angles'] = [0, None, 2, None, None, 5]
        code, response = self.post('/test/state', self.data)
        self.assertEqual(code, 201)
        self.assertEqual(response['record']['actual_joint_angles'], [0, None, 2, None, None, 5])
        self.data['desired_joint_angles'] = [None] * 6
        self.assertEqual(self.post('/test/state', self.data)[0], 400)

    def get(self, path):
        handler = object.__new__(ORMR3ControlRequestHandler)
        handler.path = path
        handler.test_store = self.store
        handler.wfile = io.BytesIO()
        responses = []
        headers = {}
        handler._send_json = lambda code, body: responses.append((code, body))
        handler.send_response = lambda code: responses.append((code, None))
        handler.send_header = lambda key, value: headers.update({key: value})
        handler.end_headers = lambda: None
        handler.log_error = lambda *args: None
        handler.do_GET()
        return responses[0], handler.wfile.getvalue(), headers

    def test_read_routes(self):
        self.assertEqual(self.get('/test/sessions')[0], (200, {'sessions': []}))
        self.store.start('run-1')
        self.store.start('empty')
        record = self.store.add_state(self.data)['record']
        self.assertEqual(self.get('/test/sessions')[0][1]['sessions'], ['empty', 'run-1'])
        self.assertEqual(self.get('/test/states?test_id=empty')[0][1]['records'], [])
        self.assertEqual(self.get('/test/states?test_id=run-1')[0][1]['records'], [record])
        response, content, headers = self.get('/test/image?test_id=run-1&filename=' + record['image'])
        self.assertEqual(response[0], 200)
        self.assertEqual(content, PNG)
        self.assertEqual(headers['Content-Type'], 'image/png')
        self.assertEqual(self.get('/test/image?test_id=empty&filename=' + record['image'])[0][0], 404)
        for url, code in [('/test/states', 400), ('/test/states?test_id=missing', 404),
                          ('/test/states?test_id=../outside', 400),
                          ('/test/image?test_id=run-1&filename=../../outside', 400),
                          ('/test/unknown', 404)]:
            self.assertEqual(self.get(url)[0][0], code)

    def test_read_symlink_rejected(self):
        self.store.start('run-1')
        record = self.store.add_state(self.data)['record']
        folder = Path(self.temp.name)/'run-1'
        image = folder/record['image']
        image.unlink()
        image.symlink_to(folder/'test_records.json')
        self.assertEqual(self.get('/test/image?test_id=run-1&filename=' + record['image'])[0][0], 400)

    def test_annotations_and_session_scale(self):
        self.store.start('run-1')
        first = self.store.add_state(self.data)['record']
        second = self.store.add_state(self.data)['record']
        annotation = dict(test_id='run-1', record_id=first['record_id'],
                          image_width=640, image_height=480, x=100, y=200)
        self.assertEqual(self.post('/test/pose_estimate', annotation)[0], 200)
        code, scale = self.post('/test/scale', {**annotation, 'size_pixels': 100})
        self.assertEqual(code, 200)
        self.assertEqual(scale['mm_per_pixel'], 1)
        self.assertEqual(scale['size_mm'], 100)
        data = self.get('/test/states?test_id=run-1')[0][1]
        self.assertEqual(data['calibration'], scale)
        self.assertEqual(data['records'][0]['end_effector_pixel']['x'], 100)
        self.assertNotIn('end_effector_pixel', data['records'][1])
        self.store.add_state(self.data)
        self.assertEqual(self.store.records('run-1')[0]['end_effector_pixel']['y'], 200)
        self.assertEqual(self.store.calibration('run-1'), scale)
        for changes in [dict(x=-1), dict(y=480), dict(x=float('nan'))]:
            self.assertEqual(self.post('/test/pose_estimate', {**annotation, **changes})[0], 400)
        for size in [0, -1, 1000]:
            self.assertEqual(self.post('/test/scale', {**annotation, 'size_pixels': size})[0], 400)
        self.assertEqual(self.post('/test/pose_estimate', {**annotation, 'record_id':'missing'})[0], 404)

    def test_invalid_input(self):
        self.assertEqual(self.post('/test/state', self.data)[0], 404)
        for name in ['../outside', '/tmp/outside', 'a/b', '..']:
            self.assertEqual(self.post('/test/start', {'test_id': name})[0], 400)
        self.store.start('run-1')
        for changes in [dict(image='bad'), dict(image='data:image/png;base64,!!!!'),
                        dict(actual_joint_angles=[0]), dict(actual_joint_angles=[float('nan')]*6),
                        dict(desired_cartesian_coordinates={'x': True, 'y': 0, 'z': 0})]:
            self.assertEqual(self.post('/test/state', {**self.data, **changes})[0], 400)
        folder = Path(self.temp.name)/'run-1'
        self.assertEqual(list(folder.iterdir()), [folder/'test_records.json'])

    def test_atomic_record_failure_cleanup(self):
        self.store.start('run-1')
        with patch.object(Path, 'replace', side_effect=OSError('disk failure')):
            with self.assertRaises(OSError):
                self.store.add_state(self.data)
        folder = Path(self.temp.name)/'run-1'
        self.assertEqual(json.loads((folder/'test_records.json').read_text()), [])
        self.assertEqual(list(folder.iterdir()), [folder/'test_records.json'])


if __name__ == '__main__':
    unittest.main()
