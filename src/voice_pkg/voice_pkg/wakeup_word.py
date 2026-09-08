import os
from pathlib import Path
import openwakeword
import sounddevice as sd
from openwakeword.model import Model

from .audio_device import resolve_input_device

_MODEL_FILE = "hello_rokey_8332_32.tflite"
_PKG_ROOT = Path(__file__).resolve().parent.parent
_MODEL_CANDIDATES = [
    _PKG_ROOT / "resource" / _MODEL_FILE,
    _PKG_ROOT / "share" / "voice_pkg" / "resource" / _MODEL_FILE,
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