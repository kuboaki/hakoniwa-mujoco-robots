"""
spikehat - libspikehat_sim の Python ctypes バインディング

実機の libspikehat/python/spikehat.py と同じインターフェースを提供する。
違いは .so のパスと SpikeHat.__init__ の引数のみ。

使い方:
    from spikehat import SpikeHat, DEVICE_MOTOR_L
    with SpikeHat("examples/test_motor.xml") as hat:
        hat.port_config(0, DEVICE_MOTOR_L)
        hat.motor_run_for_seconds(0, 3.0, 50)
        print(f"位置: {hat.motor_get_position(0)} 度")
"""
import ctypes
import os
import atexit
import signal

_here  = os.path.dirname(os.path.abspath(__file__))
_ext   = 'dylib' if __import__('sys').platform == 'darwin' else 'so'
# hakoniwa-mujoco-robots のビルド出力 (src/cmake-build/main_for_sample/spikehat/) を既定とする。
# SPIKEHAT_SIM_LIB 環境変数で .so/.dylib のパスを上書きできる。
_so = os.environ.get('SPIKEHAT_SIM_LIB') or os.path.join(
    _here, '..', '..', '..', 'src', 'cmake-build', 'main_for_sample', 'spikehat',
    f'libspikehat_sim.{_ext}')
_lib   = ctypes.CDLL(os.path.realpath(_so))
_hat_p = ctypes.c_void_p
_int_p = ctypes.POINTER(ctypes.c_int)

# --- 関数シグネチャ定義（実機版と同じ） ---
_lib.spikehat_open.restype  = _hat_p
_lib.spikehat_open.argtypes = [ctypes.c_char_p]

_lib.spikehat_close.restype  = None
_lib.spikehat_close.argtypes = [_hat_p]

_lib.spikehat_port_config.restype  = ctypes.c_int
_lib.spikehat_port_config.argtypes = [_hat_p, ctypes.c_int, ctypes.c_int]

_lib.spikehat_motor_pwm.restype  = ctypes.c_int
_lib.spikehat_motor_pwm.argtypes = [_hat_p, ctypes.c_int, ctypes.c_float]

_lib.spikehat_motor_start.restype  = ctypes.c_int
_lib.spikehat_motor_start.argtypes = [_hat_p, ctypes.c_int, ctypes.c_int]

_lib.spikehat_motor_stop.restype  = ctypes.c_int
_lib.spikehat_motor_stop.argtypes = [_hat_p, ctypes.c_int]

_lib.spikehat_motor_coast.restype  = ctypes.c_int
_lib.spikehat_motor_coast.argtypes = [_hat_p, ctypes.c_int]

_lib.spikehat_motor_run_for_seconds.restype  = ctypes.c_int
_lib.spikehat_motor_run_for_seconds.argtypes = [_hat_p, ctypes.c_int,
                                                 ctypes.c_float, ctypes.c_int]

_lib.spikehat_motor_run_for_degrees.restype  = ctypes.c_int
_lib.spikehat_motor_run_for_degrees.argtypes = [_hat_p, ctypes.c_int,
                                                 ctypes.c_int, ctypes.c_int]

_lib.spikehat_motor_run_to_position.restype  = ctypes.c_int
_lib.spikehat_motor_run_to_position.argtypes = [_hat_p, ctypes.c_int,
                                                 ctypes.c_int, ctypes.c_int]

_lib.spikehat_motor_get_speed.restype  = ctypes.c_int
_lib.spikehat_motor_get_speed.argtypes = [_hat_p, ctypes.c_int, _int_p]

_lib.spikehat_motor_get_position.restype  = ctypes.c_int
_lib.spikehat_motor_get_position.argtypes = [_hat_p, ctypes.c_int, _int_p]

_lib.spikehat_distance_read.restype  = ctypes.c_int
_lib.spikehat_distance_read.argtypes = [_hat_p, ctypes.c_int, _int_p]

_lib.spikehat_color_read_hsv.restype  = ctypes.c_int
_lib.spikehat_color_read_hsv.argtypes = [_hat_p, ctypes.c_int,
                                          _int_p, _int_p, _int_p]

_lib.spikehat_color_read_rgb.restype  = ctypes.c_int
_lib.spikehat_color_read_rgb.argtypes = [_hat_p, ctypes.c_int,
                                          _int_p, _int_p, _int_p]

_lib.spikehat_force_read.restype  = ctypes.c_int
_lib.spikehat_force_read.argtypes = [_hat_p, ctypes.c_int, _int_p, _int_p]

_lib.spikehat_force_is_pressed.restype  = ctypes.c_int
_lib.spikehat_force_is_pressed.argtypes = [_hat_p, ctypes.c_int, _int_p]

_lib.spikehat_force_get_force.restype  = ctypes.c_int
_lib.spikehat_force_get_force.argtypes = [_hat_p, ctypes.c_int, _int_p]

_lib.spikehat_sleep.restype  = None
_lib.spikehat_sleep.argtypes = [_hat_p, ctypes.c_float]

# --- シム専用拡張API (spikehat_sim.h) ---
_lib.spikehat_sim_set_ctrl.restype  = ctypes.c_int
_lib.spikehat_sim_set_ctrl.argtypes = [_hat_p, ctypes.c_int, ctypes.c_double]

_lib.spikehat_sim_get_qpos.restype  = ctypes.c_int
_lib.spikehat_sim_get_qpos.argtypes = [_hat_p, ctypes.c_int,
                                        ctypes.POINTER(ctypes.c_double)]

# デバイス種別定数 (spikehat.h の enum に対応)
DEVICE_NONE     = 0
DEVICE_MOTOR_M  = 1
DEVICE_MOTOR_L  = 2
DEVICE_COLOR    = 3
DEVICE_DISTANCE = 4
DEVICE_FORCE    = 5


class SpikeHat:
    """
    MuJoCo シミュレーター上の SpikeHat 操作クラス（with文対応）

    実機版との違い:
      - __init__ の引数は シリアルデバイスパスの代わりに MuJoCo XML ファイルのパス
      - デフォルトは examples/test_motor.xml

    Parameters
    ----------
    xml_path : str
        MuJoCo XML ファイルのパス
    """

    def __init__(self, xml_path: str = None):
        if xml_path is None:
            xml_path = os.path.join(_here, '..', 'examples', 'test_motor.xml')
        self._hat = _lib.spikehat_open(os.path.realpath(xml_path).encode())
        if not self._hat:
            raise RuntimeError(f"シムを初期化できません: {xml_path}")
        atexit.register(self.close)
        self._prev_sigint  = signal.signal(signal.SIGINT,  self._sig_handler)
        self._prev_sigterm = signal.signal(signal.SIGTERM, self._sig_handler)

    def _sig_handler(self, sig, frame):
        self.close()
        prev = self._prev_sigint if sig == signal.SIGINT else self._prev_sigterm
        if callable(prev):
            prev(sig, frame)
        else:
            signal.signal(sig, signal.SIG_DFL)
            signal.raise_signal(sig)

    def close(self):
        if self._hat:
            _lib.spikehat_close(self._hat)
            self._hat = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # ポート設定
    def port_config(self, port: int, device_type: int) -> int:
        return _lib.spikehat_port_config(self._hat, port, device_type)

    # モーターAPI
    def motor_pwm(self, port: int, power: float) -> int:
        """直接PWM制御 (-1.0〜+1.0)"""
        return _lib.spikehat_motor_pwm(self._hat, port, power)

    def motor_start(self, port: int, speed: int) -> int:
        """速度制御で回転開始 (-100〜+100)"""
        return _lib.spikehat_motor_start(self._hat, port, speed)

    def motor_stop(self, port: int) -> int:
        return _lib.spikehat_motor_stop(self._hat, port)

    def motor_coast(self, port: int) -> int:
        return _lib.spikehat_motor_coast(self._hat, port)

    def motor_run_for_seconds(self, port: int, seconds: float,
                              speed: int) -> int:
        return _lib.spikehat_motor_run_for_seconds(
            self._hat, port, seconds, speed)

    def motor_run_for_degrees(self, port: int, degrees: int,
                              speed: int) -> int:
        return _lib.spikehat_motor_run_for_degrees(
            self._hat, port, degrees, speed)

    def motor_run_to_position(self, port: int, position_deg: int,
                              speed: int) -> int:
        return _lib.spikehat_motor_run_to_position(
            self._hat, port, position_deg, speed)

    def motor_get_speed(self, port: int) -> int:
        v = ctypes.c_int()
        if _lib.spikehat_motor_get_speed(
                self._hat, port, ctypes.byref(v)) != 0:
            raise RuntimeError("速度データなし")
        return v.value

    def motor_get_position(self, port: int) -> int:
        v = ctypes.c_int()
        if _lib.spikehat_motor_get_position(
                self._hat, port, ctypes.byref(v)) != 0:
            raise RuntimeError("位置データなし")
        return v.value

    # センサーAPI
    def distance_read(self, port: int) -> int:
        """距離をmm単位で返す"""
        v = ctypes.c_int()
        if _lib.spikehat_distance_read(
                self._hat, port, ctypes.byref(v)) != 0:
            raise RuntimeError("距離データなし")
        return v.value

    def color_read_hsv(self, port: int) -> tuple:
        """色をHSVタプルで返す (hue, sat, val)"""
        h, s, v = ctypes.c_int(), ctypes.c_int(), ctypes.c_int()
        if _lib.spikehat_color_read_hsv(
                self._hat, port,
                ctypes.byref(h), ctypes.byref(s), ctypes.byref(v)) != 0:
            raise RuntimeError("カラーデータなし")
        return (h.value, s.value, v.value)

    def color_read_rgb(self, port: int) -> tuple:
        """色をRGBタプルで返す (r, g, b) 各0〜255"""
        r, g, b = ctypes.c_int(), ctypes.c_int(), ctypes.c_int()
        if _lib.spikehat_color_read_rgb(
                self._hat, port,
                ctypes.byref(r), ctypes.byref(g), ctypes.byref(b)) != 0:
            raise RuntimeError("カラーデータなし")
        return (r.value, g.value, b.value)

    def force_read(self, port: int) -> tuple:
        """力センサーの値を返す (force: int, pressed: bool)"""
        f, p = ctypes.c_int(), ctypes.c_int()
        if _lib.spikehat_force_read(
                self._hat, port,
                ctypes.byref(f), ctypes.byref(p)) != 0:
            raise RuntimeError("フォースデータなし")
        return (f.value, bool(p.value))

    def force_is_pressed(self, port: int) -> bool:
        """センサーが押されているか返す"""
        p = ctypes.c_int()
        if _lib.spikehat_force_is_pressed(self._hat, port,
                                           ctypes.byref(p)) != 0:
            raise RuntimeError("フォースデータなし")
        return bool(p.value)

    def force_get_force(self, port: int) -> int:
        """力を N 単位で返す（0〜10）"""
        f = ctypes.c_int()
        if _lib.spikehat_force_get_force(self._hat, port,
                                          ctypes.byref(f)) != 0:
            raise RuntimeError("フォースデータなし")
        return f.value

    def sim_set_ctrl(self, actuator_id: int, value: float) -> int:
        """
        シム専用: アクチュエーターのctrl値を直接設定する（spikehat_sim.h）。
        actuator_id は mujoco.mj_name2id(model, mjOBJ_ACTUATOR, name) で取得する。
        """
        return _lib.spikehat_sim_set_ctrl(self._hat, actuator_id,
                                           ctypes.c_double(value))

    def sim_get_qpos(self, qpos_adr: int) -> float:
        """
        シム専用: 関節位置 data->qpos[qpos_adr] を取得する（spikehat_sim.h）。
        qpos_adr は model.jnt_qposadr[joint_id] で取得する。
        """
        v = ctypes.c_double()
        if _lib.spikehat_sim_get_qpos(self._hat, qpos_adr, ctypes.byref(v)) != 0:
            raise RuntimeError("qposデータなし")
        return v.value

    def sleep(self, seconds: float) -> None:
        """
        指定秒数スリープする。
        実機版: OS の sleep と同等。
        シム版: MuJoCo のシミュレーションステップを進める。
        注意: time.sleep() を直接使うとシム版ではセンサー値が
              更新されないため、必ずこのメソッドを使うこと。
        """
        _lib.spikehat_sleep(self._hat, seconds)
