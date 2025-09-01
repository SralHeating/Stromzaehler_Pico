# Minimalistischer SML-Parser für MicroPython
# Hinweis: SML ist TL-encodiert (Type/Length) nach IEC 62056-21/OBIS. Diese Implementierung
# zielt auf Robustheit bei begrenzten Ressourcen ab und extrahiert generisch alle OBIS-Einträge.
# Für volle Spezifikationsabdeckung siehe Open-Source-Projekte (sml-parse, libSML), hier jedoch
# eine praktikable Variante für Iskra MT631.

try:
    from ubinascii import hexlify
except ImportError:
    from binascii import hexlify

SML_START = b"\x1b\x1b\x1b\x1b\x01\x01\x01\x01"
SML_END = b"\x1b\x1b\x1b\x1b\x1a"

class Buffer:
    def __init__(self):
        self.buf = bytearray()

    def append(self, data: bytes):
        if not data:
            return
        self.buf.extend(data)
        # Begrenze Buffergröße
        if len(self.buf) > 4096:
            self.buf = self.buf[-4096:]

    def extract_frames(self):
        frames = []
        while True:
            start = self.buf.find(SML_START)
            if start < 0:
                # kein Start, Buffer klein halten
                if len(self.buf) > 1024:
                    self.buf = self.buf[-1024:]
                break
            # suche nach Endsequenz ab start
            end = self.buf.find(SML_END, start + len(SML_START))
            if end < 0:
                # noch nicht komplett
                if start > 0:
                    self.buf = self.buf[start:]
                break
            # SML-Endsequenz gefolgt von optionalen 0x00-Pads (0..3) und 2 Byte CRC
            tail_idx = end + len(SML_END)
            if len(self.buf) < tail_idx + 2:
                break
            pad = 0
            while tail_idx + pad < len(self.buf) and pad < 4 and self.buf[tail_idx + pad] == 0x00:
                pad += 1
            if len(self.buf) < tail_idx + pad + 2:
                break
            frame = bytes(self.buf[start:tail_idx + pad + 2])
            frames.append(frame)
            self.buf = self.buf[tail_idx + pad + 2 : ]
        return frames

# Hilfsfunktionen

def decode_obis_str(b):
    # OBIS 6-Byte: A B C D E F (F meist 0xFF). String: A-B:C.D.E
    if len(b) != 6:
        return None
    return "%d-%d:%d.%d.%d" % (b[0], b[1], b[2], b[3], b[4])

# optionale menschenlesbare Aliase
KNOWN_ALIASES = {
    b"\x01\x00\x01\x08\x00\xff": "1-0:1.8.0",  # Wirkenergie Bezug gesamt
    b"\x01\x00\x02\x08\x00\xff": "1-0:2.8.0",  # Wirkenergie Lieferung gesamt
    b"\x01\x00\x10\x07\x00\xff": "1-0:16.7.0", # Momentanleistung Summe
    b"\x01\x00\x20\x07\x00\xff": "1-0:32.7.0", # Spannung L1
    b"\x02\x00\x20\x07\x00\xff": "2-0:32.7.0", # Spannung L2
    b"\x03\x00\x20\x07\x00\xff": "3-0:32.7.0", # Spannung L3
    b"\x01\x00\x31\x07\x00\xff": "1-0:49.7.0", # Frequenz
    b"\x01\x00\x01\x08\x01\xff": "1-0:1.8.1",
    b"\x01\x00\x01\x08\x02\xff": "1-0:1.8.2",
}

UNIT_MAP = {
    30: "Wh",  # active energy
    27: "W",   # active power
    33: "V",
    35: "A",
    52: "Hz",
}


def parse_obis_from_frame(frame: bytes):
    data = frame
    res = {}

    def read_int_at(pos):
        if pos >= len(data):
            return None, pos
        t = data[pos]
        if (t & 0x70) == 0x50:  # integer/unsigned
            ln = t & 0x0F
            if ln == 0 or pos + 1 + ln > len(data):
                return None, pos
            valb = data[pos+1:pos+1+ln]
            v = 0
            for b in valb:
                v = (v << 8) | (b & 0xFF)
            # signed Korrektur
            if (t & 0x08) and ln > 0 and (valb[0] & 0x80):
                v = v - (1 << (8*ln))
            return v, pos + 1 + ln
        return None, pos

    visited = set()
    i = 0
    while i + 6 <= len(data):
        ob = data[i:i+6]
        # Kandidaten: 6 Bytes, letzter 0xFF, nicht alles 0
        if ob[5] == 0xFF and ob != b"\x00\x00\x00\x00\x00\xFF":
            key = decode_obis_str(ob)
            # nur erste Vorkommen verarbeiten
            if key and i not in visited:
                visited.add(i)
                # Suche Value + Scaler + Unit in den nächsten ~48 Bytes
                p = i + 6
                endp = min(len(data), p + 48)
                scaler = 0
                unit = None
                value = None
                while p < endp:
                    # Scaler 0x62 ss (signed int8)
                    if data[p] == 0x62 and p+1 < len(data):
                        s = data[p+1]
                        if s & 0x80:
                            s -= 256
                        scaler = s
                        p += 2
                        continue
                    # Unit 0x52 uu
                    if data[p] == 0x52 and p+1 < len(data):
                        unit = UNIT_MAP.get(data[p+1], data[p+1])
                        p += 2
                        continue
                    # Integer-Wert
                    v, np = read_int_at(p)
                    if v is not None:
                        value = v
                        p = np
                        # nicht break; ggf. folgt scaler danach, also weiterlaufen
                    p += 1
                if value is not None:
                    if scaler:
                        try:
                            value = value * (10 ** scaler)
                        except Exception:
                            pass
                    res[key] = value
                    alias = KNOWN_ALIASES.get(ob)
                    if alias:
                        res[alias] = value
                    # optional die Einheit bereitstellen
                    if unit is not None:
                        res[key+"_unit"] = unit
        i += 1

    # Fallback: kompletten Frame hex als raw
    res["_raw_hex"] = hexlify(frame).decode()
    return res
