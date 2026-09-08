import os

import sounddevice as sd


def resolve_input_device():
    override = os.environ.get("VOICE_MIC_DEVICE")
    if override:
        return int(override) if override.isdigit() else override
    try:
        for idx, dev in enumerate(sd.query_devices()):
            if dev["max_input_channels"] > 0 and int(round(dev["default_samplerate"])) == 16000:
                return idx
    except Exception:
        pass
    return None


if __name__ == "__main__":
    dev = resolve_input_device()
    print("resolved device:", dev if dev is not None else "(ALSA 기본값 fallback)")
    if dev is not None:
        info = sd.query_devices(dev)
        assert info["max_input_channels"] > 0, f"입력 장치가 아니다: {info['name']}"
        print(f"  name={info['name']} rate={info['default_samplerate']}")
    print("\n전체 입력 장치:")
    for idx, d in enumerate(sd.query_devices()):
        if d["max_input_channels"] > 0:
            print(f"  [{idx}] {d['name']} @ {int(round(d['default_samplerate']))}Hz")
