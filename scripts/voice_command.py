"""Bài 7 – Phân tích câu lệnh điều hướng bằng tiếng Việt (văn bản do ASR trả về) -> danh sách khu vực.

Không phải node, không import ROS (unit test chạy được không cần ROS). Dùng bởi voice_navigator.py.

    from voice_command import ZoneCatalog, parse_command, plan_route
    cat = ZoneCatalog(zones)                      # zones = danh sách khu vực trong maps/zones.yaml
    cmd = parse_command('Di chuyển từ khu vực A đến phòng thí nghiệm', cat)
    cmd.intent  -> 'navigate'   cmd.start -> 'A'   cmd.targets -> ['I']
    legs = plan_route(cmd, current_zone='F')     # -> ['A', 'I'] (robot không ở A -> qua A trước)

Câu lệnh hiểu được (không phân biệt hoa thường, có/không dấu, dấu câu tuỳ ý):
  * "Đi đến khu vực A." / "Tới phòng thí nghiệm" / "Về sảnh chính"                 -> 1 điểm đến
  * "Di chuyển từ khu vực A đến khu vực B."                                        -> xuất phát + đích
  * "Đi từ phòng A qua hành lang C rồi đến phòng B" / "đi đến A, B rồi C"          -> nhiều điểm
  * "Dừng lại" / "Huỷ"                                                             -> dừng
  * "Có những khu vực nào?"  /  "Tôi đang ở đâu?"                                    -> liệt kê / vị trí
Khu vực gọi bằng ID ("khu vực A", "khu B", "vùng C", "phòng D", "khu vực bê" – cách đọc chữ cái tiếng
Việt), bằng tên ("phòng học") hoặc tên gọi khác (aliases trong config/zone_names.yaml).
Câu có khu vực KHÔNG tồn tại ("đi đến khu vực Z", "tới nhà bếp") -> intent 'invalid', không đi đâu cả
(thà không đi còn hơn đi nhầm chỗ).
"""

import difflib
import re
import unicodedata

# ------------------------------------------------------------------------------------- chuẩn hoá chữ
_PUNCT = re.compile(r'[\.,!?;:"“”‘’\'()\[\]{}…\-–—/\\]+')


def normalize(text):
    """Chữ thường, NFC, bỏ dấu câu, gộp khoảng trắng."""
    text = unicodedata.normalize('NFC', str(text or '')).lower()
    text = _PUNCT.sub(' ', text)
    return ' '.join(text.split())


def strip_accents(text):
    """'Đi đến khu vực Bê' -> 'di den khu vuc be' (so khớp khi ASR sai dấu hoặc gõ không dấu)."""
    text = unicodedata.normalize('NFD', text)
    text = ''.join(c for c in text if unicodedata.category(c) != 'Mn')
    return unicodedata.normalize('NFC', text).replace('đ', 'd').replace('Đ', 'D')


# Cách đọc chữ cái trong tiếng Việt (Vosk/Whisper tiếng Việt hay viết ra thế này). Chỉ giữ cách đọc
# không trùng nhau giữa các chữ.
LETTER_SPOKEN = {
    'a': ['a'], 'b': ['bê', 'bờ', 'bi'], 'c': ['xê', 'cê', 'xi'], 'd': ['đê', 'dê', 'đờ'],
    'e': ['e'], 'f': ['ép', 'ép phờ', 'ép phê'], 'g': ['giê', 'gờ'], 'h': ['hát', 'hắt', 'hờ'],
    'i': ['i'], 'j': ['gi'], 'k': ['ca', 'ka'], 'l': ['e lờ', 'lờ'], 'm': ['em mờ', 'mờ'],
    'n': ['en nờ', 'nờ'], 'o': ['o'], 'p': ['pê', 'pờ'], 'q': ['quy', 'cu'], 'r': ['e rờ', 'rờ'],
    's': ['ét', 'ét xì', 'sờ'], 't': ['tê', 'tờ'], 'u': ['u'], 'v': ['vê', 'vờ'], 'w': ['vê kép'],
    'x': ['ích', 'ích xì', 'xờ'], 'y': ['i dài', 'y'], 'z': ['dét', 'zét'],
}
# Từ đứng trước ID khu vực ("khu vực A", "phòng B", "hành lang C" ...)
HEAD_WORDS = ['khu vực', 'khu', 'vùng', 'phòng', 'hành lang', 'điểm', 'zone', 'area', 'room']

# Từ khoá vai trò (dạng có dấu; dạng không dấu chỉ dùng khi CẢ câu không có dấu – tránh "tôi" = "tới")
KW_FROM = {'từ': 'from', 'from': 'from'}
KW_TO = {'đến': 'to', 'tới': 'to', 'về': 'to', 'sang': 'to', 'vào': 'to', 'ra': 'to', 'to': 'to',
         'rồi': 'to', 'sau đó': 'to', 'tiếp theo': 'to', 'tiếp đến': 'to', 'và': 'to', 'then': 'to',
         'and': 'to', 'cuối cùng': 'to'}
KW_VIA = {'qua': 'via', 'ghé': 'via', 'ghé qua': 'via', 'ngang qua': 'via', 'đi qua': 'via',
          'via': 'via', 'through': 'via'}
KW_PLAIN = {'tu': 'from', 'den': 'to', 'toi': 'to', 've': 'to', 'vao': 'to', 'roi': 'to',
            'sau do': 'to', 'qua': 'via', 'ghe': 'via', 'ghe qua': 'via', 'di qua': 'via'}

STOP_PHRASES = ['dừng lại', 'dừng', 'đứng lại', 'đứng yên', 'huỷ', 'hủy', 'hủy bỏ', 'huỷ bỏ',
                'thôi', 'stop', 'cancel', 'dung lai', 'dung', 'huy',
                # ASR hay nghe nhầm "dừng lại" (đo với Whisper): nhận rộng tay vì dừng nhầm thì vô hại
                'vũng lại', 'rừng lại', 'dùng lại', 'đừng lại', 'dựng lại', 'rưng lại']
LIST_PHRASES = ['những khu vực nào', 'khu vực nào', 'danh sách khu vực', 'liệt kê', 'có những phòng nào',
                'list zones', 'nhung khu vuc nao', 'liet ke']
WHERE_PHRASES = ['đang ở đâu', 'ở đâu', 'vị trí hiện tại', 'đang ở khu vực nào', 'where am i',
                 'dang o dau', 'vi tri hien tai']
# Từ đệm – không phải tên khu vực (bỏ qua khi tìm khu vực không tồn tại)
FILLERS = set('''đi di chuyển hãy giúp tôi mình robot rô bốt rôbốt bạn nhé nha nhá ngay lập tức với ạ
 mau làm ơn vui lòng muốn cần đó kia này cho em anh chị đang được lại tiếp please go move the
 nào thì là của chạy hộ giùm luôn nhanh chậm khu vực vùng phòng hành lang điểm dừng đứng yên ơi'''.split())
# Bản không dấu (câu gõ không dấu). Bỏ các từ mà khi bỏ dấu trùng một chữ cái ID ("ạ" -> "a").
FILLERS_PLAIN = {strip_accents(w) for w in FILLERS if len(strip_accents(w)) > 1}


def _tokens(text):
    return text.split()


def _is_id_name(text):
    """'khu vực c', 'phòng a', 'zone b' ... (từ đầu + một ID ngắn hoặc cách đọc chữ cái)."""
    t = strip_accents(normalize(text))
    for head in HEAD_WORDS:
        h = strip_accents(head)
        if t.startswith(h + ' '):
            rest = t[len(h) + 1:]
            if len(rest) <= 2 or rest in {strip_accents(f) for v in LETTER_SPOKEN.values() for f in v}:
                return True
    return False


class Mention(object):
    __slots__ = ('zone', 'start', 'end', 'fuzzy', 'text')

    def __init__(self, zone, start, end, text, fuzzy=False):
        self.zone, self.start, self.end, self.text, self.fuzzy = zone, start, end, text, fuzzy


class ZoneCatalog(object):
    """Bảng tra tên gọi -> ID khu vực, dựng từ danh sách khu vực của maps/zones.yaml."""

    def __init__(self, zones):
        self.zones = [z for z in zones]
        self.by_id = {str(z['id']).upper(): z for z in self.zones}
        self.aliases = []                 # (tokens không dấu, zone_id, fuzzy_ok, tên hiển thị)
        seen = set()
        for z in self.zones:
            zid = str(z['id']).upper()
            names = [z.get('name') or ''] + list(z.get('aliases') or [])
            for nm in names:
                nn = normalize(nm)
                if nn:
                    # Tên dạng "<khu vực> <chữ cái>" KHÔNG được khớp gần đúng: "khu vực Z" giống
                    # "khu vực C" tới 90 % nhưng là khu vực khác hẳn.
                    self._add(seen, nn, zid, fuzzy_ok=len(nn) >= 6 and not _is_id_name(nn))
            forms = [zid.lower()]
            if len(zid) == 1:
                forms += LETTER_SPOKEN.get(zid.lower(), [])
            for head in HEAD_WORDS:
                for f in forms:
                    self._add(seen, head + ' ' + f, zid, fuzzy_ok=False)
        # khớp dài nhất trước
        self.aliases.sort(key=lambda a: -len(a[0]))
        self.letter_forms = {}
        for z in self.zones:
            zid = str(z['id']).upper()
            for f in [zid.lower()] + (LETTER_SPOKEN.get(zid.lower(), []) if len(zid) == 1 else []):
                self.letter_forms.setdefault(tuple(strip_accents(f).split()), zid)

    def _add(self, seen, text, zid, fuzzy_ok):
        toks = tuple(strip_accents(text).split())
        key = (toks, zid)
        if toks and key not in seen:
            seen.add(key)
            self.aliases.append((toks, zid, fuzzy_ok, text))

    def name(self, zid):
        z = self.by_id.get(zid)
        if z is None:
            return zid
        nm = z.get('name') or ''
        return nm if nm.lower() == 'khu vực {}'.format(zid.lower()) else '{} ({})'.format(nm, zid)

    def describe(self):
        return ', '.join(self.name(str(z['id']).upper()) for z in self.zones)

    def find_mentions(self, plain_tokens):
        """Tìm tên khu vực (khớp dài nhất, trái -> phải) trong dãy token không dấu."""
        out = []
        i, n = 0, len(plain_tokens)
        while i < n:
            hit = None
            for toks, zid, _, _ in self.aliases:
                L = len(toks)
                if L <= n - i and tuple(plain_tokens[i:i + L]) == toks:
                    hit = Mention(zid, i, i + L, ' '.join(plain_tokens[i:i + L]))
                    break
            if hit is None:
                i += 1
                continue
            out.append(hit)
            i = hit.end
        return out

    def fuzzy(self, phrase_plain, cutoff=0.84):
        """Khớp gần đúng một cụm (không dấu) với tên khu vực dài (KHÔNG áp dụng cho ID chữ cái)."""
        best, score = None, 0.0
        for toks, zid, ok, _ in self.aliases:
            if not ok:
                continue
            r = difflib.SequenceMatcher(None, phrase_plain, ' '.join(toks)).ratio()
            if r > score:
                best, score = zid, r
        return (best, score) if score >= cutoff else (None, score)


class Command(object):
    """Kết quả phân tích. intent: navigate | stop | list | where | invalid | empty."""

    def __init__(self, text, intent, start=None, targets=None, unknown=None, message='', fuzzy=None):
        self.text = text
        self.intent = intent
        self.start = start
        self.targets = list(targets or [])
        self.unknown = list(unknown or [])
        self.message = message
        self.fuzzy = list(fuzzy or [])

    def as_dict(self):
        return {'text': self.text, 'intent': self.intent, 'start': self.start, 'targets': self.targets,
                'unknown': self.unknown, 'message': self.message, 'fuzzy': self.fuzzy}

    def __repr__(self):
        return 'Command({})'.format(self.as_dict())


def _contains(text, phrases):
    """Câu có chứa một trong các cụm (so khớp nguyên từ, không dấu)."""
    padded = ' ' + strip_accents(text) + ' '
    return any(' ' + strip_accents(p) + ' ' in padded for p in phrases)


def _is_filler(tok, has_marks):
    return tok in FILLERS if has_marks else tok in FILLERS_PLAIN


def _keyword_table(has_marks):
    table = {}
    for d in (KW_FROM, KW_TO, KW_VIA):
        for k, v in d.items():
            table[tuple(k.split())] = v
    if not has_marks:
        for k, v in KW_PLAIN.items():
            table[tuple(k.split())] = v
    return sorted(table.items(), key=lambda kv: -len(kv[0]))


def parse_command(text, catalog):
    norm = normalize(text)
    if not norm:
        return Command(text, 'empty', message='Không nghe rõ câu lệnh.')
    plain = strip_accents(norm)
    has_marks = plain != norm
    toks = _tokens(norm)                 # có dấu (so khớp từ khoá)
    ptoks = _tokens(plain)               # không dấu (so khớp tên khu vực)
    mentions = catalog.find_mentions(ptoks)

    if not mentions:
        if _contains(norm, STOP_PHRASES):
            return Command(text, 'stop', message='Dừng robot.')
        if _contains(norm, LIST_PHRASES):
            return Command(text, 'list', message='Các khu vực: ' + catalog.describe() + '.')
        if _contains(norm, WHERE_PHRASES):
            return Command(text, 'where', message='Hỏi vị trí hiện tại.')

    # ---- gán vai trò: duyệt token, gặp từ khoá -> đổi vai trò cho các tên khu vực phía sau
    kw_table = _keyword_table(has_marks)
    covered = [None] * len(toks)        # index mention phủ token
    for k, m in enumerate(mentions):
        for t in range(m.start, m.end):
            covered[t] = k
    role = None
    roles = {}
    segments = []                       # (vai trò, [token không phải tên khu vực, không phải từ khoá])
    cur_free = []
    i = 0
    while i < len(toks):
        if covered[i] is not None:
            k = covered[i]
            roles[k] = role or 'to'
            if role == 'from':
                role = 'to'             # "từ A B" hiếm; sau xuất phát mặc định là điểm đến
            segments.append((roles[k], cur_free, k))
            cur_free = []
            i = mentions[k].end
            continue
        kw = None
        src = toks if has_marks else ptoks
        for words, r in kw_table:
            L = len(words)
            if tuple(src[i:i + L]) == words:
                kw = (r, L)
                break
        if kw is not None:
            if cur_free:
                segments.append((role, cur_free, None))
            cur_free = []
            role = kw[0]
            i += kw[1]
            continue
        cur_free.append(i)
        i += 1
    if cur_free:
        segments.append((role, cur_free, None))

    # ---- cụm từ còn lại sau "đến/qua/từ" mà không phải tên khu vực -> khu vực không tồn tại?
    unknown, fuzzy_hits = [], []
    extra = []                          # (vị trí token, role, zid) từ khớp gần đúng
    for r, idxs, k in segments:
        words = [toks[t] for t in idxs if not _is_filler(toks[t], has_marks)]
        if not words:
            continue
        heads = ('khu', 'vung', 'phong', 'hanh', 'diem', 'zone', 'area', 'room')
        has_head = any(strip_accents(toks[t]) in heads for t in idxs)
        if r is None and not has_head:
            continue                    # câu dẫn trước từ khoá đầu tiên ("robot ơi ...") – bỏ qua
        if k is not None and not has_head:
            continue                    # từ thừa ngay trước một tên khu vực hợp lệ ("bên phòng thí nghiệm" – ASR
            #                             nghe "đến" thành "bên") -> không phải khu vực lạ
        phrase = ' '.join(toks[t] for t in idxs)
        zid = None
        if not _is_id_name(phrase):     # "khu vực Z" -> không đoán gần đúng
            zid, _ = catalog.fuzzy(strip_accents(phrase))
            if zid is None:             # thử bỏ từ đệm
                zid, _ = catalog.fuzzy(strip_accents(' '.join(words)))
        if zid is not None:
            fuzzy_hits.append((phrase, zid))
            extra.append((idxs[0], r or 'to', zid))
            continue
        # chữ cái trơn sau từ khoá: "từ A đến B"
        key = tuple(strip_accents(' '.join(words)).split())
        if r is not None and key in catalog.letter_forms and len(words) <= 2:
            extra.append((idxs[0], r, catalog.letter_forms[key]))
            continue
        unknown.append(' '.join(words) if not has_head else phrase)

    if unknown:
        msg = 'Không có khu vực "{}" trên bản đồ. Các khu vực hiện có: {}.'.format(
            '", "'.join(unknown), catalog.describe())
        return Command(text, 'invalid', unknown=unknown, message=msg)

    items = [(m.start, roles.get(k, 'to'), m.zone) for k, m in enumerate(mentions)] + extra
    items.sort(key=lambda t: t[0])
    if not items:
        if _contains(plain, STOP_PHRASES):
            return Command(text, 'stop', message='Dừng robot.')
        return Command(text, 'invalid', message='Không tìm thấy tên khu vực trong câu "{}". Các khu vực '
                       'hiện có: {}.'.format(norm, catalog.describe()))
    start = None
    targets = []
    for _, r, zid in items:
        if r == 'from' and start is None and not targets:
            start = zid
        else:
            targets.append(zid)
    # bỏ điểm trùng liên tiếp
    dedup = []
    for z in targets:
        if not dedup or dedup[-1] != z:
            dedup.append(z)
    targets = dedup
    if not targets:
        return Command(text, 'invalid', start=start,
                       message='Thiếu điểm đến: "từ {}" rồi đi đâu?'.format(catalog.name(start)))
    if start is not None and targets == [start]:
        return Command(text, 'invalid', start=start, message='Điểm xuất phát và điểm đến trùng nhau ({}) – hãy nói '
                       'lại.'.format(catalog.name(start)))
    if start is not None and targets[0] == start:
        targets = targets[1:]
    route = ([catalog.name(start)] if start else []) + [catalog.name(t) for t in targets]
    msg = 'Đi ' + ' -> '.join(route)
    if fuzzy_hits:
        msg += ' (hiểu "{}" là {})'.format(fuzzy_hits[0][0], catalog.name(fuzzy_hits[0][1]))
    return Command(text, 'navigate', start=start, targets=targets, message=msg,
                   fuzzy=[z for _, z in fuzzy_hits])


def plan_route(cmd, current_zone):
    """Danh sách khu vực sẽ lần lượt tới. Câu "từ A đến B" khi robot KHÔNG ở A -> tới A trước rồi tới B;
    robot đang ở A -> đi thẳng tới B. current_zone = None (chưa biết vị trí) -> coi như không ở A."""
    if cmd.intent != 'navigate':
        return []
    legs = []
    if cmd.start is not None and cmd.start != current_zone:
        legs.append(cmd.start)
    for z in cmd.targets:
        if not legs or legs[-1] != z:
            legs.append(z)
    return legs
