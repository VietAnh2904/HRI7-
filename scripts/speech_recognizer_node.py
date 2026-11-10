#!/usr/bin/env python3
"""Bài 7 – Nhận dạng giọng nói (Speech Recognition): micro (hoặc file WAV) -> văn bản -> /voice/text.

  ros2 run tb4_lab04 speech_recognizer_node.py                                    # Vosk + micro
  ros2 run tb4_lab04 speech_recognizer_node.py --ros-args -p engine:=whisper      # Whisper + micro
  ros2 run tb4_lab04 speech_recognizer_node.py --ros-args -p wav_files:="['a.wav','b.wav']"
  ros2 run tb4_lab04 speech_recognizer_node.py --list-devices                     # liệt kê micro

Hai bộ nhận dạng (cài bằng scripts/install_voice_deps.sh):
  * vosk    (mặc định) – offline, nhẹ (model tiếng Việt vosk-model-small-vn-0.4 ~32 MB), chạy CPU thời gian
            thực, tự phát hiện hết câu. use_grammar:=true giới hạn từ vựng vào câu lệnh + tên khu vực ->
            chính xác hơn nhiều NHƯNG không còn nghe ra tên khu vực lạ ("khu vực Z" thành [unk]).
  * whisper – faster-whisper (CTranslate2, int8 CPU) hoặc openai-whisper; tiếng Việt tốt hơn Vosk, chậm hơn
            (model small ~1–3 s / câu trên CPU). Node tự cắt câu bằng VAD năng lượng; tên các khu vực được
            đưa vào initial_prompt để Whisper viết đúng chính tả. Kết quả "bịa" (chép prompt, lặp từ) bị bỏ,
            /voice/feedback báo "Không nghe rõ, hãy nói lại".

Topic:
  /voice/text     std_msgs/String  câu đã nhận dạng xong (voice_navigator.py nghe topic này)
  /voice/partial  std_msgs/String  kết quả tạm thời khi đang nói (chỉ Vosk) – để hiển thị
  /voice/state    std_msgs/String  listening | speech | decoding | error: ...

Tham số (--ros-args -p tên:=giá_trị):
  engine          vosk         vosk | whisper
  language        vi           mã ngôn ngữ cho Whisper
  vosk_model      ''           thư mục model Vosk (trống = ~/.cache/tb4_lab04/vosk-model-small-vn-0.4)
  whisper_model   small        tiny | base | small | medium | large-v3 | đường dẫn model faster-whisper
  whisper_prompt  names        names (tên khu vực) | none | chuỗi tuỳ ý – KHÔNG ghi câu lệnh mẫu vào đây
  use_grammar     false        Vosk: giới hạn từ vựng (xem trên)
  zones_file      ''           maps/zones.yaml – lấy tên khu vực cho grammar / prompt
  device          ''           tên hoặc số thứ tự micro (trống = mặc định của hệ thống)
  sample_rate     16000
  vad_threshold   0.0          RMS (0..1) coi là có tiếng nói; 0 = tự đo nhiễu nền 1 s đầu
  min_speech      0.3  max_speech 8.0  end_silence 0.7    (giây) – cắt câu cho Whisper
  wav_files       []           nhận dạng các file WAV này rồi thoát (kiểm thử lặp lại được, không cần micro)
  wav_gap         2.0          giây nghỉ giữa các file (để robot kịp xử lý lệnh trước)
"""

import json
import math
import os
import queue
import struct
import sys
import threading
import time
import wave

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402

SAMPLE_RATE = 16000
DEFAULT_VOSK_MODEL = os.path.join('~', '.cache', 'tb4_lab04', 'vosk-model-small-vn-0.4')


# ------------------------------------------------------------------ phần thuần (có unit test)
def pcm16_to_floats(data):
    n = len(data) // 2
    return [v / 32768.0 for v in struct.unpack('<{}h'.format(n), data[:2 * n])]


def floats_to_pcm16(samples):
    return struct.pack('<{}h'.format(len(samples)),
                       *[max(-32768, min(32767, int(round(s * 32767.0)))) for s in samples])


def rms(samples):
    if not samples:
        return 0.0
    return math.sqrt(sum(s * s for s in samples) / len(samples))


def resample_linear(samples, src_rate, dst_rate):
    """Đổi tần số lấy mẫu (nội suy tuyến tính – đủ cho tiếng nói vào ASR)."""
    if src_rate == dst_rate or not samples:
        return list(samples)
    n_out = int(round(len(samples) * float(dst_rate) / src_rate))
    out = []
    step = float(src_rate) / dst_rate
    last = len(samples) - 1
    for k in range(n_out):
        x = k * step
        i = int(x)
        if i >= last:
            out.append(samples[last])
        else:
            f = x - i
            out.append(samples[i] * (1.0 - f) + samples[i + 1] * f)
    return out


def read_wav(path, rate=SAMPLE_RATE):
    """WAV PCM 8/16/32 bit, mono/stereo -> danh sách float mono ở `rate` Hz."""
    with wave.open(path, 'rb') as w:
        ch, width, sr, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if width == 2:
        vals = [v / 32768.0 for v in struct.unpack('<{}h'.format(len(raw) // 2), raw)]
    elif width == 1:
        vals = [(b - 128) / 128.0 for b in raw]
    elif width == 4:
        vals = [v / 2147483648.0 for v in struct.unpack('<{}i'.format(len(raw) // 4), raw)]
    else:
        raise ValueError('{}: WAV {} bit không hỗ trợ'.format(path, 8 * width))
    if ch > 1:
        vals = [sum(vals[k:k + ch]) / ch for k in range(0, len(vals), ch)]
    return resample_linear(vals, sr, rate)


class EnergyVAD(object):
    """Cắt luồng âm thanh thành từng câu nói theo năng lượng (RMS) mỗi khung 30 ms.

    Bắt đầu câu khi >= `start_frames` khung liên tiếp vượt ngưỡng; kết thúc khi im lặng `end_silence` giây
    (hoặc câu dài quá `max_speech`). Câu ngắn hơn `min_speech` bị bỏ (tiếng gõ phím, ho...). Ngưỡng 0 ->
    tự hiệu chỉnh: lấy nhiễu nền trong `calib` giây đầu x 3 (tối thiểu 0.01)."""

    def __init__(self, rate=SAMPLE_RATE, threshold=0.0, min_speech=0.3, max_speech=8.0, end_silence=0.7,
                 frame_ms=30, start_frames=3, pre_roll=0.3, calib=1.0):
        self.rate = rate
        self.frame = int(rate * frame_ms / 1000)
        self.threshold = threshold
        self.min_frames = int(min_speech * 1000 / frame_ms)
        self.max_frames = int(max_speech * 1000 / frame_ms)
        self.end_frames = int(end_silence * 1000 / frame_ms)
        self.start_frames = start_frames
        self.pre_frames = int(pre_roll * 1000 / frame_ms)
        self.calib_frames = int(calib * 1000 / frame_ms) if threshold <= 0 else 0
        self._calib = []
        self._buf = []
        self._pre = []
        self._speech = []
        self._loud_run = 0
        self._silence = 0
        self.in_speech = False

    def feed(self, samples):
        """Nhận mẫu float, trả về danh sách các câu (list float) vừa kết thúc."""
        self._buf.extend(samples)
        out = []
        while len(self._buf) >= self.frame:
            fr, self._buf = self._buf[:self.frame], self._buf[self.frame:]
            done = self._frame(fr)
            if done is not None:
                out.append(done)
        return out

    def flush(self):
        if self.in_speech and len(self._speech) >= self.min_frames:
            utt = [s for f in self._speech for s in f]
            self._reset()
            return utt
        self._reset()
        return None

    def _reset(self):
        self._speech, self._pre = [], []
        self._loud_run = self._silence = 0
        self.in_speech = False

    def _frame(self, fr):
        e = rms(fr)
        if self.calib_frames:
            self._calib.append(e)
            if len(self._calib) >= self.calib_frames:
                noise = sorted(self._calib)[len(self._calib) // 2]
                self.threshold = max(0.01, 3.0 * noise)
                self.calib_frames = 0
            return None
        loud = e >= self.threshold
        if not self.in_speech:
            self._pre.append(fr)
            if len(self._pre) > self.pre_frames + self.start_frames:
                self._pre.pop(0)
            self._loud_run = self._loud_run + 1 if loud else 0
            if self._loud_run >= self.start_frames:
                self.in_speech = True
                self._speech = list(self._pre)
                self._pre = []
                self._silence = 0
            return None
        self._speech.append(fr)
        self._silence = 0 if loud else self._silence + 1
        if self._silence >= self.end_frames or len(self._speech) >= self.max_frames:
            voiced = len(self._speech) - self._silence
            utt = [s for f in self._speech for s in f]
            self._reset()
            return utt if voiced >= self.min_frames else None
        return None


def zone_phrases(zones):
    """Tên khu vực (để đưa vào grammar Vosk / prompt Whisper)."""
    out = []
    for z in zones:
        zid = str(z['id'])
        out.append('khu vực ' + zid.lower())
        for nm in [z.get('name') or ''] + list(z.get('aliases') or []):
            nm = nm.strip().lower()
            if nm and nm not in out:
                out.append(nm)
    return out


def vosk_grammar(zones):
    """Danh sách cụm từ cho KaldiRecognizer(model, rate, grammar). Các từ không có trong từ điển của model
    bị Vosk bỏ qua (có cảnh báo); '[unk]' để câu lạ không bị ép thành câu lệnh."""
    from voice_command import LETTER_SPOKEN
    base = ['đi đến', 'đi tới', 'di chuyển từ', 'đến', 'tới', 'qua', 'rồi', 'về', 'từ', 'đi', 'di chuyển',
            'dừng lại', 'huỷ', 'có những khu vực nào', 'tôi đang ở đâu', 'khu vực', 'phòng', 'hành lang']
    phrases = list(base)
    for z in zones:
        zid = str(z['id']).lower()
        for f in [zid] + LETTER_SPOKEN.get(zid, []):
            phrases.append('khu vực ' + f)
    phrases += zone_phrases(zones)
    seen, out = set(), []
    for p in phrases:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out + ['[unk]']


def whisper_prompt(zones, mode='names'):
    """initial_prompt cho Whisper. CHỈ chứa tên gọi, KHÔNG chứa câu lệnh mẫu: kiểm thử với câu lệnh mẫu trong
    prompt, Whisper gặp âm thanh khó nghe thì chép lại nguyên câu mẫu ("Đi đến khu vực Z" thành "Đi đến khu vực A"
    với độ tin cậy cao) -> robot đi nhầm. mode: names | none | <chuỗi tuỳ ý>."""
    if mode in (None, '', 'none'):
        return None
    if mode != 'names':
        return mode
    names = [z.get('name') for z in zones if z.get('name') and z['name'] != 'Khu vực ' + str(z['id'])]
    return 'Robot di chuyển trong tòa nhà có {}.'.format(', '.join(n.lower() for n in names)) if names else None


def suspicious_transcript(text, prompt=None):
    """Lý do loại bỏ một kết quả Whisper "bịa" (hallucination), hoặc None nếu dùng được.

    * chép lại prompt: cả câu (>= 2 từ) nằm nguyên trong initial_prompt;
    * lặp từ: một từ lặp >= 3 lần liền nhau hoặc chiếm >= 1/2 số từ của câu dài ("B, B, B, B").
    Độ tin cậy (avg_logprob) KHÔNG dùng được để lọc: câu chép prompt có độ tin cậy cao nhất (đã đo)."""
    from voice_command import normalize
    t = normalize(text)
    words = t.split()
    if not words:
        return None
    if prompt and len(words) >= 2:
        pw = normalize(prompt).split()
        if (' ' + t + ' ') in (' ' + ' '.join(pw) + ' '):
            return 'trùng nội dung initial_prompt (Whisper chép prompt khi không nghe rõ)'
        # chép một đoạn dài của prompt ("phòng học, hành lang, sảnh chính, ...") dù không nguyên văn cả câu
        grams = {tuple(pw[k:k + 4]) for k in range(len(pw) - 3)}
        covered = set()
        for k in range(len(words) - 3):
            if tuple(words[k:k + 4]) in grams:
                covered.update(range(k, k + 4))
        if len(covered) * 2 >= len(words):
            return 'chép một đoạn của initial_prompt'
    run = 1
    for a, b in zip(words, words[1:]):
        run = run + 1 if a == b else 1
        if run >= 3:
            return 'lặp từ "{}"'.format(a)
    for n in (2, 3, 4):                 # lặp cả cụm liền nhau: "phòng thí nghiệm, phòng thí nghiệm" (đã gặp)
        for k in range(len(words) - 2 * n + 1):
            if words[k:k + n] == words[k + n:k + 2 * n]:
                return 'lặp cụm "{}"'.format(' '.join(words[k:k + n]))
    if len(words) >= 4:
        top = max(words.count(w) for w in set(words))
        if top * 2 >= len(words) and top >= 3:
            return 'lặp từ'
    return None


# ------------------------------------------------------------------ bộ nhận dạng
class VoskEngine(object):
    streaming = True

    def __init__(self, model_dir, rate, grammar=None, log=print):
        try:
            from vosk import KaldiRecognizer, Model, SetLogLevel
        except ImportError:
            raise RuntimeError('Chưa cài vosk: bash scripts/install_voice_deps.sh  (hoặc pip3 install vosk)')
        model_dir = os.path.expanduser(model_dir or DEFAULT_VOSK_MODEL)
        if not os.path.isdir(model_dir):
            raise RuntimeError('Không có model Vosk ở {} – chạy: bash scripts/install_voice_deps.sh'.format(
                model_dir))
        SetLogLevel(-1)
        self.model = Model(model_dir)
        self.rate = rate
        self.grammar = grammar
        self._make = lambda: (KaldiRecognizer(self.model, rate, json.dumps(grammar, ensure_ascii=False))
                              if grammar else KaldiRecognizer(self.model, rate))
        self.rec = self._make()
        gtxt = '{} cụm'.format(len(grammar)) if grammar else 'tắt (từ vựng tự do)'
        log('Vosk: model {} | grammar: {}'.format(model_dir, gtxt))

    def accept(self, pcm):
        """-> (câu hoàn chỉnh hoặc None, kết quả tạm hoặc None)"""
        if self.rec.AcceptWaveform(pcm):
            text = json.loads(self.rec.Result()).get('text', '')
            return clean_text(text), None
        return None, json.loads(self.rec.PartialResult()).get('partial', '')

    def finish(self):
        text = json.loads(self.rec.FinalResult()).get('text', '')
        self.rec = self._make()
        return clean_text(text)

    def transcribe(self, samples):
        rec = self._make()
        rec.AcceptWaveform(floats_to_pcm16(samples))
        return clean_text(json.loads(rec.FinalResult()).get('text', ''))


class WhisperEngine(object):
    streaming = False

    def __init__(self, model, language, prompt, log=print):
        self.language = language
        self.prompt = prompt
        self.kind = None
        try:
            from faster_whisper import WhisperModel
            self.model = WhisperModel(model, device='cpu', compute_type='int8')
            self.kind = 'faster-whisper'
        except ImportError:
            try:
                import whisper
                self.model = whisper.load_model(model)
                self.kind = 'openai-whisper'
            except ImportError:
                raise RuntimeError('Chưa cài faster-whisper / openai-whisper: bash scripts/install_voice_deps.sh '
                                   'whisper')
        log('Whisper ({}): model {} | ngôn ngữ {}'.format(self.kind, model, language))

    def transcribe(self, samples):
        import numpy as np
        audio = np.asarray(samples, dtype=np.float32)
        if self.kind == 'faster-whisper':
            segs, _ = self.model.transcribe(audio, language=self.language, beam_size=5,
                                            initial_prompt=self.prompt, vad_filter=False,
                                            condition_on_previous_text=False)
            return clean_text(' '.join(s.text for s in segs))
        res = self.model.transcribe(audio, language=self.language, initial_prompt=self.prompt, fp16=False,
                                    condition_on_previous_text=False)
        return clean_text(res.get('text', ''))


def clean_text(text):
    """[unk] của Vosk (từ ngoài grammar) -> "không rõ": "đi đến khu vực [unk]" thành "đi đến khu vực không rõ"
    -> voice_command báo khu vực không tồn tại (robot đứng yên) thay vì bỏ mất từ. Câu chỉ có [unk] -> ''."""
    words = (text or '').split()
    if not words or all(w == '[unk]' for w in words):
        return ''
    return ' '.join('không rõ' if w == '[unk]' else w for w in words)


# ------------------------------------------------------------------ node
def build_node_class():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SpeechRecognizer(Node):
        def __init__(self):
            super().__init__('tb4_speech_recognizer')
            p = self.declare_parameter
            p('engine', 'vosk')
            p('language', 'vi')
            p('vosk_model', '')
            p('whisper_model', 'small')
            p('whisper_prompt', 'names')
            p('use_grammar', False)
            p('zones_file', '')
            p('device', '')
            p('sample_rate', SAMPLE_RATE)
            p('vad_threshold', 0.0)
            p('min_speech', 0.3)
            p('max_speech', 8.0)
            p('end_silence', 0.7)
            p('wav_files', [''])
            p('wav_gap', 2.0)
            self.pub_text = self.create_publisher(String, 'voice/text', 10)
            self.pub_partial = self.create_publisher(String, 'voice/partial', 10)
            self.pub_state = self.create_publisher(String, 'voice/state', 10)
            self.pub_feedback = self.create_publisher(String, 'voice/feedback', 10)
            self.audio_q = queue.Queue(maxsize=200)
            self.stop_evt = threading.Event()
            self.engine = None
            self.zones = self._load_zones()

        def param(self, n):
            return self.get_parameter(n).value

        def _load_zones(self):
            path = self.param('zones_file') or lc.find_config('maps', 'zones.yaml')[0]
            if not path or not os.path.isfile(path):
                self.get_logger().warn('Không thấy maps/zones.yaml – grammar/prompt không có tên khu vực')
                return []
            import yaml
            with open(path, 'r', encoding='utf-8') as f:
                return (yaml.safe_load(f) or {}).get('zones') or []

        def state(self, s):
            self.pub_state.publish(String(data=s))

        def publish_text(self, text, source=''):
            if not text:
                return False
            why = suspicious_transcript(text, getattr(self.engine, 'prompt', None))
            if why:
                self.get_logger().warning('BỎ QUA{}: "{}" – {}'.format(source, text, why))
                self.pub_feedback.publish(String(data='Không nghe rõ ("{}"), hãy nói lại.'.format(text)))
                self.state('rejected')
                return False
            self.get_logger().info('NGHE ĐƯỢC{}: "{}"'.format(source, text))
            self.pub_text.publish(String(data=text))
            return True

        def make_engine(self):
            log = self.get_logger().info
            if self.param('engine') == 'whisper':
                return WhisperEngine(self.param('whisper_model'), self.param('language'),
                                     whisper_prompt(self.zones, self.param('whisper_prompt')), log)
            if self.param('engine') != 'vosk':
                raise RuntimeError('engine phải là vosk | whisper')
            grammar = vosk_grammar(self.zones) if self.param('use_grammar') else None
            return VoskEngine(self.param('vosk_model'), int(self.param('sample_rate')), grammar, log)

        # ---------------------------------------------------------- file WAV
        def wait_listener(self, timeout=10.0):
            """Chờ voice_navigator subscribe /voice/text: publish ngay khi vừa khởi động thì câu đầu tiên bị
            mất (DDS chưa kết nối xong, topic không lưu tin)."""
            end = time.monotonic() + timeout
            while self.pub_text.get_subscription_count() == 0 and time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.pub_text.get_subscription_count() == 0:
                self.get_logger().warn('Chưa có node nào nghe /voice/text (voice_navigator chưa chạy?)')
                return False
            return True

        def run_wav(self, files):
            rate = int(self.param('sample_rate'))
            ok = 0
            self.wait_listener()
            for k, path in enumerate(files):
                path = os.path.expanduser(path)
                try:
                    samples = read_wav(path, rate)
                except (OSError, ValueError, wave.Error) as exc:
                    self.get_logger().error('Không đọc được {}: {}'.format(path, exc))
                    continue
                self.state('decoding')
                t0 = time.monotonic()
                text = self.engine.transcribe(samples)
                self.get_logger().info('[{}/{}] {} ({:.1f} s audio, giải mã {:.2f} s)'.format(
                    k + 1, len(files), os.path.basename(path), len(samples) / rate, time.monotonic() - t0))
                if text:
                    ok += int(self.publish_text(text, ' (file)'))
                else:
                    self.get_logger().warn('  không nhận ra lời nói nào')
                self._sleep(float(self.param('wav_gap')))
            self.state('done')
            return 0 if ok == len(files) else 1

        def _sleep(self, sec):
            end = time.monotonic() + sec
            while time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

        # ---------------------------------------------------------- micro
        def run_mic(self):
            try:
                import sounddevice as sd
            except (ImportError, OSError) as exc:
                raise RuntimeError('Chưa có sounddevice/PortAudio ({}): bash scripts/install_voice_deps.sh'
                                   .format(exc))
            rate = int(self.param('sample_rate'))
            dev = self.param('device')
            device = int(dev) if str(dev).strip().isdigit() else (dev or None)

            def callback(indata, frames, t, status):
                try:
                    self.audio_q.put_nowait(bytes(indata))
                except queue.Full:
                    pass

            vad = EnergyVAD(rate, float(self.param('vad_threshold')), float(self.param('min_speech')),
                            float(self.param('max_speech')), float(self.param('end_silence')))
            with sd.RawInputStream(samplerate=rate, blocksize=int(rate * 0.1), device=device, channels=1,
                                   dtype='int16', callback=callback):
                self.get_logger().info('Đang nghe micro ({}) – nói: "Đi đến khu vực A" ...'.format(
                    device if device is not None else 'mặc định'))
                self.state('listening')
                last_partial = ''
                calibrated = False
                while rclpy.ok() and not self.stop_evt.is_set():
                    rclpy.spin_once(self, timeout_sec=0.0)
                    try:
                        data = self.audio_q.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    if self.engine.streaming:
                        text, partial = self.engine.accept(data)
                        if text:
                            self.publish_text(text)
                            last_partial = ''
                        elif partial and partial != last_partial:
                            last_partial = partial
                            self.pub_partial.publish(String(data=partial))
                        continue
                    for utt in vad.feed(pcm16_to_floats(data)):
                        self.state('decoding')
                        t0 = time.monotonic()
                        text = self.engine.transcribe(utt)
                        self.get_logger().info('  ({:.1f} s lời nói, giải mã {:.2f} s)'.format(
                            len(utt) / rate, time.monotonic() - t0))
                        self.publish_text(text)
                        self.state('listening')
                    if not calibrated and vad.calib_frames == 0:
                        calibrated = True
                        self.get_logger().info('VAD: ngưỡng RMS = {:.3f}'.format(vad.threshold))
                    self.state('speech' if vad.in_speech else 'listening')
            return 0

    return SpeechRecognizer


def list_devices():
    try:
        import sounddevice as sd
    except (ImportError, OSError) as exc:
        print('Chưa có sounddevice/PortAudio: {}'.format(exc))
        return 1
    print(sd.query_devices())
    return 0


def main():
    if '--list-devices' in sys.argv:
        return list_devices()
    import rclpy
    rclpy.init(args=sys.argv)
    Node = build_node_class()
    node = Node()
    try:
        try:
            node.engine = node.make_engine()
        except RuntimeError as exc:
            node.get_logger().error(str(exc))
            node.state('error: ' + str(exc))
            return 2
        files = [f for f in (node.param('wav_files') or []) if f]
        if files:
            return node.run_wav(files)
        try:
            return node.run_mic()
        except RuntimeError as exc:
            node.get_logger().error(str(exc))
            return 2
        except Exception as exc:  # noqa: BLE001 (PortAudioError: không có micro ...)
            if type(exc).__name__ == 'PortAudioError':
                node.get_logger().error('Lỗi micro: {} – kiểm tra: speech_recognizer_node.py --list-devices'
                                        .format(exc))
                return 2
            raise
    finally:
        node.stop_evt.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    lc.install_ctrl_c_handlers()
    sys.exit(lc.run_main(main, interrupted_code=0))
