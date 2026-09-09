import os
from pathlib import Path
import openwakeword
import sounddevice as sd
from openwakeword.model import Model

from .audio_device import resolve_input_device

_MODEL_FILE = "hello_rokey_8332_32.tflite"
_PKG_ROOT = Path(__file__).resolve().parent.parent

# 모델(.tflite)은 setup.py 가 share/voice_pkg/resource/ 에 설치한다.
# copy-install / symlink-install 모두에서 확실한 건 get_package_share_directory.
# (기존 _PKG_ROOT 기반 경로는 copy-install 시 lib/.../site-packages 를 가리켜 실패했음)
_MODEL_CANDIDATES = []
try:
    from ament_index_python.packages import get_package_share_directory
    _MODEL_CANDIDATES.append(
        Path(get_package_share_directory("voice_pkg")) / "resource" / _MODEL_FILE
    )
except Exception:
    pass
_MODEL_CANDIDATES += [
    _PKG_ROOT / "resource" / _MODEL_FILE,
    _PKG_ROOT / "share" / "voice_pkg" / "resource" / _MODEL_FILE,
    _PKG_ROOT.parent / "resource" / _MODEL_FILE,   # src/voice_pkg/resource
]
MODEL_NAME = next((str(p) for p in _MODEL_CANDIDATES if p.exists()), str(_MODEL_CANDIDATES[0]))

SAMPLE_RATE = 16000
FRAME = 1280


class WakeupWord:
    def __init__(self):
        openwakeword.utils.download_models()
        self.model = None
        self.model_name = os.path.splitext(os.path.basename(MODEL_NAME))[0]
        self.stream = None

    def is_wakeup(self):
        audio_chunk, _ = self.stream.read(FRAME)
        audio_chunk = audio_chunk.flatten()
        confidence = self.model.predict(audio_chunk)[self.model_name]
        print("confidence: ", confidence)
        if confidence > 0.3:
            print("Wakeword detected!")
            return True
        return False

    def open(self):
        self.model = Model(wakeword_models=[MODEL_NAME])
        self.stream = sd.InputStream(
            samplerate=SAMPLE_RATE, channels=1, dtype="int16", blocksize=FRAME,
            device=resolve_input_device(),
        )
        self.stream.start()

    def close(self):
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
            self.stream = None

def main(args=None):
    wakeup = WakeupWord()
    wakeup.open()
    try:
        while wakeup.is_wakeup() is False:
            pass
    finally:
        wakeup.close()

if __name__ == "__main__":
    main()