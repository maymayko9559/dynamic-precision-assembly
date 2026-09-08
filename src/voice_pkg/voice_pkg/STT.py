from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
import tempfile
import os

from dotenv import load_dotenv

from ament_index_python.packages import get_package_share_directory

from .audio_device import resolve_input_device


load_dotenv(
    dotenv_path=os.path.join(
        get_package_share_directory("voice_pkg"),
        "resource",
        ".env"
    )
)

openai_api_key = os.getenv("OPENAI_API_KEY")


class STT:

    def __init__(self, openai_api_key):
        self.client = OpenAI(api_key=openai_api_key)
        self.duration = 5
        self.samplerate = 16000

    def speech2text(self):
        print("음성 녹음을 시작합니다. \n 5초 동안 말해주세요...")

        audio = sd.rec(
            int(self.duration * self.samplerate),
            samplerate=self.samplerate,
            channels=1,
            dtype="int16",
            device=resolve_input_device(),
        )
        sd.wait()

        print("녹음 완료. Whisper에 전송 중...")

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            wav.write(temp_wav.name, self.samplerate, audio)

        with open(temp_wav.name, "rb") as f:
            transcript = self.client.audio.transcriptions.create(
                model="whisper-1", file=f)

        os.remove(temp_wav.name)

        print("STT 결과: ", transcript.text)

        return transcript.text


def main(args=None):
    stt = STT(openai_api_key)
    output_message = stt.speech2text()


if __name__ == "__main__":
    main()