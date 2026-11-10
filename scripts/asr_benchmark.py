#!/usr/bin/env python3
"""Bài 7 – Đo độ chính xác nhận dạng giọng nói + hiểu câu lệnh (không cần Gazebo / Nav2 chạy).

  ros2 run tb4_lab04 asr_benchmark.py --record                  # đọc từng câu vào micro (4 s / câu) rồi chấm
  ros2 run tb4_lab04 asr_benchmark.py --record --repeat 3       # mỗi câu 3 lần (đo ổn định hơn)
  ros2 run tb4_lab04 asr_benchmark.py --dir ~/tb4_results/asr_<thời_gian>   # chấm lại file đã ghi
  ros2 run tb4_lab04 asr_benchmark.py --dir <...> --engine whisper --whisper-model small
  ros2 run tb4_lab04 asr_benchmark.py --tts                      # giọng máy espeak-ng (CHỈ thử đường ống)

Mỗi câu trong config/asr_test_sentences.yaml: âm thanh -> engine (Vosk / Whisper, đúng tham số như
speech_recognizer_node) -> bộ chặn kết quả "bịa" -> voice_command.parse_command -> so với kết quả mong đợi.
Kết luận mỗi câu:
  ĐÚNG     hiểu đúng ý định + khu vực
  BỎ       bộ chặn loại (robot báo "Không nghe rõ") – an toàn, người dùng nói lại
  SAI      không hiểu / từ chối – an toàn, robot đứng yên
  ĐI NHẦM  hiểu thành lệnh đi tới SAI khu vực – lỗi nguy hiểm nhất, phải bằng 0
Lưu kết quả: <thư mục>/asr_results.csv + summary.txt (số liệu cho báo cáo, yêu cầu 2 của đề).
"""

import argparse
import csv
import os
import subprocess
import sys
import time
import wave

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402
import speech_recognizer_node as sr  # noqa: E402
from voice_command import ZoneCatalog, parse_command  # noqa: E402

RATE = sr.SAMPLE_RATE


def load_sentences(path):
    import yaml
    with open(path, 'r', encoding='utf-8') as f:
        items = (yaml.safe_load(f) or {}).get('sentences') or []
    for k, it in enumerate(items):
        if 'text' not in it or 'expect' not in it:
            raise ValueError('{}: câu {} cần text và expect'.format(path, k + 1))
    return items


def judge(cmd, item):
    """'ĐÚNG' | 'SAI' | 'ĐI NHẦM'"""
    want = item['expect']
    ok = cmd.intent == want
    if ok and want == 'navigate':
        ok = (cmd.start == item.get('start')) and cmd.targets == list(item.get('targets') or [])
    if ok:
        return 'ĐÚNG'
    if cmd.intent == 'navigate':
        return 'ĐI NHẦM'
    return 'SAI'


def write_wav(path, samples):
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(sr.floats_to_pcm16(samples))


def record(path, seconds, device=None):
    import sounddevice as sd
    data = sd.rec(int(seconds * RATE), samplerate=RATE, channels=1, dtype='float32', device=device)
    sd.wait()
    write_wav(path, [float(v) for v in data[:, 0]])


def make_engine(args, zones):
    if args.engine == 'whisper':
        return sr.WhisperEngine(args.whisper_model, 'vi', sr.whisper_prompt(zones, args.whisper_prompt))
    grammar = sr.vosk_grammar(zones) if args.grammar else None
    return sr.VoskEngine(args.vosk_model, RATE, grammar)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Đo độ chính xác nhận dạng giọng nói + hiểu câu lệnh.')
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--record', action='store_true', help='ghi âm từ micro')
    src.add_argument('--dir', help='thư mục chứa file đã ghi (cau01_1.wav ...)')
    src.add_argument('--tts', action='store_true', help='tổng hợp bằng espeak-ng (KHÔNG đại diện giọng người)')
    ap.add_argument('--sentences', help='mặc định config/asr_test_sentences.yaml')
    ap.add_argument('--zones', help='mặc định maps/zones.yaml')
    ap.add_argument('--engine', default='vosk', choices=['vosk', 'whisper'])
    ap.add_argument('--vosk-model', default='')
    ap.add_argument('--grammar', action='store_true', help='Vosk: giới hạn từ vựng (use_grammar)')
    ap.add_argument('--whisper-model', default='small')
    ap.add_argument('--whisper-prompt', default='names')
    ap.add_argument('--repeat', type=int, default=1, help='số lần đọc mỗi câu (--record)')
    ap.add_argument('--seconds', type=float, default=4.0, help='thời gian ghi mỗi câu (--record)')
    ap.add_argument('--device', default=None, help='micro (số hoặc tên)')
    ap.add_argument('--out', help='thư mục lưu (mặc định ~/tb4_results/asr_<thời_gian>)')
    args = ap.parse_args(argv)

    import yaml
    sent_path = args.sentences or lc.find_config('config', 'asr_test_sentences.yaml')[0]
    zones_path = args.zones or lc.find_config('maps', 'zones.yaml')[0]
    if not sent_path or not zones_path:
        print('LỖI: thiếu config/asr_test_sentences.yaml hoặc maps/zones.yaml (chạy zone_segmentation.py)')
        return 2
    items = load_sentences(sent_path)
    with open(zones_path, 'r', encoding='utf-8') as f:
        zones = yaml.safe_load(f)['zones']
    cat = ZoneCatalog(zones)
    out = os.path.expanduser(args.dir or args.out or lc.results_dir('asr_' + lc.timestamp()))
    os.makedirs(out, exist_ok=True)
    device = int(args.device) if args.device and str(args.device).isdigit() else args.device

    # ---- 1. âm thanh
    files = []
    for k, it in enumerate(items):
        for r in range(max(1, args.repeat) if args.record else 1):
            files.append((k, os.path.join(out, 'cau{:02d}_{}.wav'.format(k + 1, r + 1))))
    if args.record:
        print('Ghi âm {} câu x {} lần, {:.0f} s mỗi lần. Đọc TO, RÕ, cách micro ~30 cm.'.format(
            len(items), max(1, args.repeat), args.seconds))
        for k, path in files:
            input('\n[{}] Nhấn Enter rồi đọc:  "{}"'.format(os.path.basename(path), items[k]['text']))
            print('    ... đang ghi')
            record(path, args.seconds, device)
    elif args.tts:
        print('CẢNH BÁO: giọng máy espeak-ng – chỉ để thử đường ống, kết quả KHÔNG đại diện giọng người.')
        for k, path in files:
            tmp = path + '.tmp.wav'
            subprocess.run(['espeak-ng', '-v', 'vi', '-s', '135', '-w', tmp, items[k]['text']], check=True)
            write_wav(path, sr.read_wav(tmp, RATE))
            os.remove(tmp)
    else:                               # --dir: mọi file cauNN_M.wav (mọi lần đọc)
        import re
        files = []
        for name in sorted(os.listdir(out)):
            m = re.match(r'cau(\d+)_(\d+)\.wav$', name)
            if m and 1 <= int(m.group(1)) <= len(items):
                files.append((int(m.group(1)) - 1, os.path.join(out, name)))
        if not files:
            print('LỖI: không có file cauNN_M.wav trong {}'.format(out))
            return 2

    # ---- 2. nhận dạng + hiểu câu
    try:
        engine = make_engine(args, zones)
    except RuntimeError as exc:
        print('LỖI:', exc)
        return 2
    rows, count = [], {'ĐÚNG': 0, 'SAI': 0, 'ĐI NHẦM': 0, 'BỎ': 0}
    print('\n{:<12} {:<8} {:>5}  {:<42} {}'.format('file', 'kết quả', 'giây', 'nghe được', 'hiểu thành'))
    for k, path in files:
        it = items[k]
        t0 = time.monotonic()
        text = engine.transcribe(sr.read_wav(path, RATE))
        dt = time.monotonic() - t0
        why = sr.suspicious_transcript(text, getattr(engine, 'prompt', None))
        if why:
            verdict, got = 'BỎ', why
        else:
            cmd = parse_command(text, cat)
            verdict = judge(cmd, it)
            got = '{} {}{}'.format(cmd.intent, 'từ {} '.format(cmd.start) if cmd.start else '',
                                   ','.join(cmd.targets))
        count[verdict] += 1
        rows.append([os.path.basename(path), it['text'], text, verdict, got, round(dt, 2)])
        print('{:<12} {:<8} {:>5.1f}  {:<42} {}'.format(
            os.path.basename(path)[:12], verdict, dt, repr(text)[:42], got))
    n = len(rows)
    summary = ('Engine: {}{} | {} lượt đọc\nĐÚNG {}/{} ({:.0f} %) | BỎ (không nghe rõ) {} | SAI (đứng yên) {} | '
               'ĐI NHẦM {}'.format(args.engine, ' + grammar' if args.grammar else '', n, count['ĐÚNG'], n,
                                   100.0 * count['ĐÚNG'] / max(1, n), count['BỎ'], count['SAI'],
                                   count['ĐI NHẦM']))
    print('\n' + summary)
    if args.tts:
        print('(giọng máy – không dùng số này trong báo cáo)')
    with open(os.path.join(out, 'asr_results.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['file', 'cau_mau', 'nghe_duoc', 'ket_qua', 'hieu_thanh', 'giai_ma_s'])
        w.writerows(rows)
    with open(os.path.join(out, 'summary.txt'), 'w', encoding='utf-8') as f:
        f.write(summary + '\n')
    print('Đã lưu: {}/asr_results.csv, summary.txt'.format(out))
    return 0 if count['ĐI NHẦM'] == 0 else 1


if __name__ == '__main__':
    lc.install_ctrl_c_handlers()
    sys.exit(lc.run_main(main))
