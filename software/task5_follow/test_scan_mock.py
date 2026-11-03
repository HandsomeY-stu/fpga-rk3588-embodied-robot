# -*- coding: utf-8 -*-
"""离线回归测试：检查scan_mock相关逻辑和异常处理。"""
import sys, types, time

def _mk(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m

_rc = _mk('run_c')
_rc.pull_frame = lambda timeout=1.0: True
_rc.LOCAL_FRAME = object()
_rc.Sender = object
_rc.TlmReader = object

_rd = _mk('red_nav_e2e')
_rd.detect_red = None

_pc = _mk('prop_ctrl')
_pc.DIR_NAME = {5: '左转', 6: '右转', 4: '前进'}
class _PF:
    def __init__(self, *a, **k): pass
    def step(self, *a, **k): return None
_pc.PropFollow = _PF

_nl = _mk('nav_layer')
_nl.wrap = lambda a: (a + 180) % 360 - 180
_nl.DEG_PER_PX = 1.0 / 19.7
_nl.SIDE_BLIND_MM = 80
_nl.BODY_MARGIN_MM = 120
_nl.NavCtl = object

import semantic_nav as sn

SPEC = dict(det=dict(rmin=45, rdom=10, min_pixels=120, mode='bbox'),
            w_min=15, w_max=300, aspect_min=0.25, aspect_max=4.0)

class Args:
    scan_mode = 'cont'
    scan_pct = 3
    scan_dir = 5
    cx_center = 640

LOG = []
def log(msg):
    LOG.append(msg)

def wrap(a):
    return (a + 180) % 360 - 180

def run_case(name, det_dict, deadline_s, expect):
    """det_dict: 每帧检测返回值（None=无检测）；固定复用"""
    def fake_detect(frame, **kw):
        mp = kw.get('min_pixels', 120)
        if det_dict is None or det_dict['pixels'] < mp:
            return None
        return dict(det_dict)
    _rd.detect_red = fake_detect

    t0 = time.time()
    yaw0 = 10.0
    class Tlm:
        @property
        def yaw_deg(self):
            return wrap(yaw0 - (time.time() - t0) * 28.0)   # 左转 28°/s
        def fresh_range(self):
            return None

    sender = types.SimpleNamespace(send=lambda *a: None)
    result = None
    try:
        result = sn.scan_until_found(sender, SPEC, Args(), Tlm(),
                                     time.time() + deadline_s, log)
    except Exception as e:
        print('  !! 异常: %r' % e)
        return False
    logs = '\n'.join(LOG)
    missing = [x for x in expect if x not in logs]
    print('  返回: %s' % ('None' if result is None else 'dict'))
    print('  %s %s' % ('PASS' if not missing else 'FAIL',
                       '' if not missing else '缺: %s' % missing))
    return not missing

print('=== 用例1：90px 弱候选持续出现 → hits≥3 后应触发「提前认领」===')
LOG.clear()
ok1 = run_case('early-claim', dict(cx=500, w=25, h=90, pixels=90), 25,
               ['提前认领'])

print('=== 用例2：只出现一瞬（前2帧有，之后消失）→ 不提前认领；一圈后按记忆去确认 ===')
LOG.clear()
_cnt = {'n': 0}
def fake_seq(frame, **kw):
    mp = kw.get('min_pixels', 120)
    _cnt['n'] += 1
    if _cnt['n'] > 2:                 # 前 2 帧可见，之后永久消失
        return None
    if 90 < mp:
        return None
    return dict(cx=500, w=25, h=90, pixels=90)
_rd.detect_red = fake_seq
t0 = time.time()
result2 = sn.scan_until_found(types.SimpleNamespace(send=lambda *a: None),
                              SPEC, Args(),
                              type('T', (), {'yaw_deg': property(
                                  lambda s: wrap(10 - (time.time() - t0) * 28)),
                                  'fresh_range': lambda s: None})(),
                              time.time() + 22, lambda m: LOG.append(m))
logs2 = '\n'.join(LOG)
ok2 = ('提前认领' not in logs2) and ('按记忆朝最可信方位' in logs2)
print('  返回: %s' % ('None' if result2 is None else 'dict'))
print('  %s（无提前认领=%s，一圈记忆确认=%s）' % (
    'PASS' if ok2 else 'FAIL', '提前认领' not in logs2,
    '按记忆朝最可信方位' in logs2))

print('=== 用例3：全程无检测 → 不崩溃、一圈后换向 ===')
LOG.clear()
ok3 = run_case('none', None, 20, ['换向'])

print()
print('总结: 提前认领=%s 记忆确认=%s 无候选不崩=%s' % (ok1, ok2, ok3))
sys.exit(0 if (ok1 and ok2 and ok3) else 1)
