#!/usr/bin/env bash
# Bài 7 – cài nhận dạng giọng nói (Vosk + model tiếng Việt, tuỳ chọn Whisper) trên Ubuntu 22.04 + ROS 2 Humble.
#   bash scripts/install_voice_deps.sh            # Vosk (mặc định, ~32 MB model, offline)
#   bash scripts/install_voice_deps.sh whisper    # + faster-whisper (tải model "small" ~480 MB lần đầu chạy)
# Chạy lại nhiều lần vẫn an toàn. Model Vosk lưu ở ~/.cache/tb4_lab04/vosk-model-small-vn-0.4
#
# Ghim "numpy<2": như Bài 6 – NumPy 2 làm hỏng cv_bridge (apt, build với NumPy 1.x) của Humble.
set -u

say(){ printf '\n\033[1;36m== %s ==\033[0m\n' "$*"; }
die(){ printf '\033[1;31m[LỖI]\033[0m %s\n' "$*"; exit 1; }
WANT_WHISPER=0
[ "${1:-}" = "whisper" ] && WANT_WHISPER=1
MODEL_NAME="${VOSK_MODEL_NAME:-vosk-model-small-vn-0.4}"
MODEL_DIR="${HOME}/.cache/tb4_lab04"
MODEL_URL="https://alphacephei.com/vosk/models/${MODEL_NAME}.zip"

say "0. Kiểm tra môi trường"
if [ -n "${CONDA_DEFAULT_ENV:-}" ] || [ -n "${VIRTUAL_ENV:-}" ]; then
  die "Đang trong conda/venv (${CONDA_DEFAULT_ENV:-${VIRTUAL_ENV:-}}). Chạy 'conda deactivate' rồi mở terminal mới."
fi
PY=/usr/bin/python3
[ -x "$PY" ] || die "Không thấy /usr/bin/python3"
"$PY" --version

say "1. Gói apt (cần sudo): PortAudio (đọc micro), ALSA utils (arecord), unzip, espeak-ng (đọc phản hồi)"
sudo apt-get update -y && sudo apt-get install -y python3-pip libportaudio2 alsa-utils unzip wget \
  espeak-ng python3-yaml python3-matplotlib || die "apt thất bại"

say "2. Thư viện Python: vosk, sounddevice"
"$PY" -m pip install --user "vosk>=0.3.45" "sounddevice>=0.4.6" "numpy<2" \
  || die "pip install thất bại (cần Internet tới pypi.org)"
if [ "$WANT_WHISPER" = 1 ]; then
  say "2b. faster-whisper (Whisper chạy CPU int8)"
  "$PY" -m pip install --user "faster-whisper>=1.0" "numpy<2" || die "pip install faster-whisper thất bại"
fi

say "3. Model Vosk tiếng Việt: ${MODEL_NAME}"
mkdir -p "$MODEL_DIR"
if [ -f "$MODEL_DIR/$MODEL_NAME/am/final.mdl" ]; then
  echo "Đã có: $MODEL_DIR/$MODEL_NAME"
else
  wget -c -O "$MODEL_DIR/$MODEL_NAME.zip" "$MODEL_URL" || die "Không tải được $MODEL_URL"
  unzip -q -o "$MODEL_DIR/$MODEL_NAME.zip" -d "$MODEL_DIR" || die "Giải nén thất bại"
  rm -f "$MODEL_DIR/$MODEL_NAME.zip"
fi

say "4. Kiểm tra"
"$PY" - "$MODEL_DIR/$MODEL_NAME" "$WANT_WHISPER" <<'PYEOF'
import json, sys
ok = True
def chk(name, fn):
    global ok
    try:
        print("[PASS] {}: {}".format(name, fn()))
    except Exception as e:
        ok = False
        print("[FAIL] {}: {}: {}".format(name, type(e).__name__, e))
import numpy
chk("numpy 1.x", lambda: numpy.__version__ if numpy.__version__.startswith("1.") else 1 / 0)
def vosk_ok():
    from vosk import KaldiRecognizer, Model, SetLogLevel
    SetLogLevel(-1)
    rec = KaldiRecognizer(Model(sys.argv[1]), 16000)
    rec.AcceptWaveform(b"\0\0" * 16000)
    return "model nạp được, 1 s im lặng -> {!r}".format(json.loads(rec.FinalResult()).get("text", ""))
chk("vosk + model tiếng Việt", vosk_ok)
def mic():
    import sounddevice as sd
    ins = [d["name"] for d in sd.query_devices() if d["max_input_channels"] > 0]
    if not ins:
        raise RuntimeError("không thấy micro nào (máy ảo: bật chuyển micro vào máy ảo)")
    return "{} thiết bị thu: {}".format(len(ins), "; ".join(ins[:4]))
chk("sounddevice (micro)", mic)
if sys.argv[2] == "1":
    chk("faster-whisper", lambda: __import__("faster_whisper").__version__)
sys.exit(0 if ok else 1)
PYEOF
RC=$?
echo
[ $RC = 0 ] && echo "XONG. Thử: ros2 run tb4_lab04 speech_recognizer_node.py   (nói \"đi đến khu vực A\")" \
            || echo "Có mục [FAIL] ở trên – xem README Bài 7 mục xử lý sự cố."
exit $RC
