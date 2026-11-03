"""网页请求到FPGA任务模式的映射，集中管理模式切换接口。"""
import time

MODE_NAMES = {0: '手动', 1: 'FPGA 自主避障', 4: '任务四·段落一 300mm'}

def mode_status(state, now=None):
    now = time.monotonic() if now is None else now
    fresh = state.tlm_ok and now - state.tlm_updated_at < 1.5 and not state.tlm_pause
    return dict(fpga_mode=state.tlm.get('mode') if fresh else None,
                fpga_force_stop=state.tlm.get('force_stop') if fresh else None,
                fpga_mode_name=MODE_NAMES.get(state.tlm.get('mode'), '未知') if fresh else '遥测未更新')

def request_mode(state, uart, mode, clock=time.monotonic, sleep=time.sleep):
    if type(mode) is not int or mode not in MODE_NAMES:
        return False, '仅支持手动(0)、自主避障(1)、段落一(4)；任意路点导航未启用'
    if mode == 0:
        with state.lock:
            state.fpga_generation += 1
            state.fpga_auto_requested = False
            state.cmd_dir = state.cmd_spd = 0
            state.cmd_until = 0.0
            ok = uart.stop()
            ok = uart.mode(0) and ok
        return ok, '已发送停车并退出到手动，状态以遥测为准' if ok else '串口发送失败'
    if not state.fpga_switch_lock.acquire(blocking=False):
        return False, '正在切换模式，请稍后'
    try:
        with state.lock:
            if state.task6_proc is not None and state.task6_proc.poll() is None:
                return False, '请先关闭任务六，再启动 FPGA 自动任务'
            if state.person_proc is not None and state.person_proc.poll() is None and state.person_uart:
                return False, '请先取消“同步 FPGA OSD”（可保留目标识别），避免串口发送冲突'
            actual = mode_status(state, clock())
            if actual['fpga_mode'] is None:
                return False, '遥测未更新，无法确认模式；请检查 FPGA 与遥测连接'
            if actual['fpga_mode'] == mode:
                if actual['fpga_force_stop']:
                    return False, '当前模式已停车；请先关闭该模式回手动，再重新开启'
                return True, '已处于' + MODE_NAMES[mode]
            state.fpga_generation += 1
            token = state.fpga_generation
            state.fpga_auto_requested = True
            state.cmd_dir = state.cmd_spd = 0
            state.cmd_until = 0.0
            state.last_src = 'web-fpga'
            ok = uart.stop()
            ok = uart.mode(mode) and ok
        deadline = clock() + 1.2
        while ok and clock() < deadline:
            with state.lock:
                if state.fpga_generation != token:
                    return False, '模式切换已被停车或手动操作取消'
                observed = mode_status(state, clock())
                if observed['fpga_mode'] == mode:
                    suffix = '（安全门仍保持停车）' if observed['fpga_force_stop'] else ''
                    return True, '遥测确认：' + MODE_NAMES[mode] + suffix
            sleep(.04)
        with state.lock:
            if state.fpga_generation == token:
                state.fpga_auto_requested = False
                uart.stop()
                uart.mode(0)
        return False, '未收到目标模式确认，已请求停车并回手动；请检查连接和急停状态'
    finally:
        state.fpga_switch_lock.release()
