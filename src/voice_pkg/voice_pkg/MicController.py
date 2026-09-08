import pyaudio
import wave
import io

class MicConfig:
    chunk: int = 12000
    rate: int = 16000
    channels: int = 1
    record_seconds: int = 5
    fmt: int = pyaudio.paInt16
    device_index: int = 10
    buffer_size: int = 24000


class MicController:
    def __init__(self, config: MicConfig = MicConfig()):
        self.config = config
        self.frames = []
        self.audio = None
        self.stream = None
        self.sample_width = None

    def open_stream(self):
        self.audio = pyaudio.PyAudio()
        self.sample_width = self.audio.get_sample_size(self.config.fmt)
        self.stream = self.audio.open(
            format=self.config.fmt,
            channels=self.config.channels,
            rate=self.config.rate,
            input=True,
            frames_per_buffer=self.config.chunk,
        )

    def close_stream(self):
        print("stop recording")
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.audio:
            self.audio.terminate()
            self.audio = None





    def record_audio(self) -> bytes:
        mic = MicController()
        mic.open_stream()

        print("start recording...")
        frames = []

        for _ in range(0, int(mic.config.rate / mic.config.chunk * mic.config.record_seconds)):
            data = mic.stream.read(mic.config.chunk)
            frames.append(data)

        mic.close_stream()

        wav_io = io.BytesIO()
        wf = wave.open(wav_io, 'wb')
        wf.setnchannels(mic.config.channels)
        wf.setsampwidth(mic.sample_width)
        wf.setframerate(mic.config.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return wav_io.getvalue()
