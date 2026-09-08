import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from .audio_device import resolve_input_device

def update_plot(frame):
    data, _ = stream.read(CHUNK)
    line.set_ydata(data.flatten())

    return line,


CHANNELS = 1
RATE = 16000
CHUNK = 12000

stream = sd.InputStream(samplerate=RATE,
                        channels=CHANNELS,
                        dtype="int16",
                        blocksize=CHUNK,
                        device=resolve_input_device())
stream.start()


fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot(np.arange(0, CHUNK), np.arange(0, CHUNK))
ax.set_title("Real-Time Audio Waveform")
ax.set_xlabel("Samples")
ax.set_ylabel("Amplitude")
ax.set_ylim(-2**15, 2**15)


ani = animation.FuncAnimation(fig, update_plot, blit=True, interval=50)
plt.show()

stream.stop()
stream.close()
