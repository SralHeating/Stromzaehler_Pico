# RX-String-Reader: liest UART-RX an GPIO1 und druckt SML-Frames und extrahierte OBIS-Werte
import time

try:
	from ubinascii import hexlify
except ImportError:
	from binascii import hexlify

from machine import UART, Pin

# Lokale UART-Konfiguration (keine Abhängigkeit zu main.py)
class _Config:
	UART_ID = 0
	UART_BAUDRATE = 9600
	UART_RX_PIN = 1  # UART0 RX = GP1
	UART_INVERT = False
	UART_TIMEOUT_MS = 1500
config = _Config

# SML-Frame-Signaturen
SML_START = b"\x1b\x1b\x1b\x1b\x01\x01\x01\x01"
SML_END = b"\x1b\x1b\x1b\x1b\x1a"

# OBIS-Hilfen und Einheiten
def decode_obis_str(b):
	if len(b) != 6:
		return None
	return "%d-%d:%d.%d.%d" % (b[0], b[1], b[2], b[3], b[4])

KNOWN_ALIASES = {
	b"\x01\x00\x01\x08\x00\xff": "1-0:1.8.0",  # Wirkenergie Bezug
	b"\x01\x00\x02\x08\x00\xff": "1-0:2.8.0",  # Wirkenergie Lieferung
	b"\x01\x00\x10\x07\x00\xff": "1-0:16.7.0", # Momentanleistung
	b"\x01\x00\x20\x07\x00\xff": "1-0:32.7.0", # Spannung L1
	b"\x02\x00\x20\x07\x00\xff": "2-0:32.7.0", # Spannung L2
	b"\x03\x00\x20\x07\x00\xff": "3-0:32.7.0", # Spannung L3
	b"\x01\x00\x31\x07\x00\xff": "1-0:49.7.0", # Frequenz
}

UNIT_MAP = {
	30: "Wh",
	27: "W",
	33: "V",
	35: "A",
	52: "Hz",
}

# Ziel-OBIS für ISKRA MT631
OBIS_ENERGY_IMPORT = b"\x01\x00\x01\x08\x00\xff"   # 1-0:1.8.0
OBIS_POWER_TOTAL   = b"\x01\x00\x10\x07\x00\xff"   # 1-0:16.7.0

# SML-OBIS-Parser (robust gegen signed/unsigned Integers)
def parse_obis_from_frame(frame: bytes):
	data = frame
	res = {}

	def read_int_at(pos):
		if pos >= len(data):
			return None, pos, 0, 0
		t = data[pos]
		h = t & 0xF0
		# Integers: signed (0x5x) und unsigned (0x6x)
		if h in (0x50, 0x60):
			ln = t & 0x0F
			if ln == 0 or pos + 1 + ln > len(data):
				return None, pos, 0, 0
			valb = data[pos+1:pos+1+ln]
			v = 0
			for b in valb:
				v = (v << 8) | (b & 0xFF)
			# signed Korrektur bei 0x5x
			if h == 0x50 and ln > 0 and (valb[0] & 0x80):
				v = v - (1 << (8*ln))
			return v, pos + 1 + ln, ln, h
		return None, pos, 0, 0

	visited = set()
	i = 0
	while i + 6 <= len(data):
		ob = data[i:i+6]
		if ob[5] == 0xFF and ob != b"\x00\x00\x00\x00\x00\xFF":
			key = decode_obis_str(ob)
			if key and i not in visited:
				visited.add(i)
				p = i + 6
				endp = min(len(data), p + 64)
				scaler = None
				unit = None
				value = None
				accept_after = p
				ints = []
				while p < endp:
					v, np, ln, h = read_int_at(p)
					if v is not None:
						# Kandidaten für Unit (kleine bekannte Codes)
						if ln == 1 and v in UNIT_MAP and unit is None:
							unit = UNIT_MAP.get(v)
							accept_after = max(accept_after, np)
						# Kandidat für Scaler (kleiner signed int)
						if ln == 1 and -6 <= v <= 6 and scaler is None:
							scaler = v
							accept_after = max(accept_after, np)
						ints.append((p, np, ln, v))
						# Wert bevorzugt: erstes Integer >=3 Bytes nach Unit/Scaler
						if value is None and ln >= 3 and p >= accept_after:
							value = v
						p = np
						continue
					p += 1
				# Falls noch keinen Wert gefunden: nimm das längste Integer im Fenster
				if value is None and ints:
					ints.sort(key=lambda t: (t[2], t[1]-t[0]))
					value = ints[-1][3]
				if value is not None:
					if scaler not in (None, 0):
						try:
							value = value * (10 ** scaler)
						except Exception:
							pass
					res[key] = value
					alias = KNOWN_ALIASES.get(ob)
					if alias:
						res[alias] = value
					if unit is not None:
						res[key+"_unit"] = unit
		i += 1

	res["_raw_hex"] = hexlify(frame).decode()
	return res


def _build_uart_hw(inverted=None):
	inv = config.UART_INVERT if inverted is None else inverted
	invert_kw = {}
	try:
		if inv:
			invert_kw = {'invert': UART.INV_RX}
	except Exception:
		invert_kw = {}
	# Nutze Default-Pinbelegung des UART (UART0: TX=GP0, RX=GP1)
	return UART(
		config.UART_ID,
		baudrate=config.UART_BAUDRATE,
		bits=8,
		parity=None,
		stop=1,
		timeout=0,
		timeout_char=0,
		rxbuf=2048,
		**invert_kw,
	)


def _ascii_preview(b: bytes) -> str:
	try:
		return b.decode()
	except Exception:
		out = []
		for v in b:
			out.append(chr(v) if 32 <= v < 127 else '.')
		return ''.join(out)


class _Buf:
	def __init__(self):
		self.b = bytearray()
	def append(self, data: bytes):
		if data:
			self.b.extend(data)
			if len(self.b) > 8192:
				self.b = self.b[-8192:]
	def extract_frames(self):
		frames = []
		while True:
			start = self.b.find(SML_START)
			if start < 0:
				if len(self.b) > 1024:
					self.b = self.b[-1024:]
				break
			end = self.b.find(SML_END, start + len(SML_START))
			if end < 0:
				if start > 0:
					self.b = self.b[start:]
				break
			tail_idx = end + len(SML_END)
			if len(self.b) < tail_idx + 3:
				break
			pad_len = self.b[tail_idx]
			if pad_len > 8:
				pad_len = 8
			end_total = tail_idx + 1 + pad_len + 2
			if len(self.b) < end_total:
				break
			frames.append(bytes(self.b[start:end_total]))
			self.b = self.b[end_total:]
		return frames


def _read_sml_int(data: bytes, pos: int):
	if pos >= len(data):
		return None, pos, 0
	t = data[pos]
	h = t & 0xF0
	if h in (0x50, 0x60):
		ln = t & 0x0F
		if ln == 0 or pos + 1 + ln > len(data):
			return None, pos, 0
		valb = data[pos+1:pos+1+ln]
		v = 0
		for b in valb:
			v = (v << 8) | (b & 0xFF)
		if h == 0x50 and ln > 0 and (valb[0] & 0x80):
			v = v - (1 << (8*ln))
		return v, pos + 1 + ln, ln
	return None, pos, 0


def extract_obis_value(data: bytes, obis_sig: bytes):
	# Suche das erste Auftreten des gewünschten OBIS-Feldes und extrahiere Wert+Scaler
	start = 0
	while True:
		idx = data.find(obis_sig, start)
		if idx < 0:
			return None
		p = idx + 6
		endp = min(len(data), p + 64)
		scaler = 0
		value = None
		while p < endp:
			t = data[p]
			# Scaler 0x62 ss (signed int8)
			if t == 0x62 and p+1 < len(data):
				s = data[p+1]
				if s & 0x80:
					s -= 256
				scaler = s
				p += 2
				continue
			# Unit 0x52 uu -> überspringen
			if t == 0x52 and p+1 < len(data):
				# unit_code = data[p+1]
				p += 2
				continue
			# Integer-Wert
			v, np, ln = _read_sml_int(data, p)
			if v is not None:
				# bevorzuge erste Zahl mit >=3 Byte Länge
				if value is None or ln >= 3:
					value = v
				p = np
				# Wenn wir sowohl scaler gesehen haben als auch eine lange Zahl, reicht das
				continue
			p += 1
		if value is not None:
			try:
				return value * (10 ** scaler)
			except Exception:
				return value
		# Falls an dieser Stelle kein Wert, suche nächstes Auftreten
		start = idx + 1


def main():
	uart_inverted = config.UART_INVERT
	uart = _build_uart_hw(uart_inverted)
	print("Starte RX-Reader auf GPIO", config.UART_RX_PIN, "@", config.UART_BAUDRATE, "baud (HW-UART), inv=", uart_inverted)
	last_wait = time.ticks_ms()
	buf = _Buf()
	print("waiting... any=", uart.any() if hasattr(uart, 'any') else -1, "inv=", uart_inverted)
	while True:
		try:
			lvl = 0
			try:
				lvl = uart.any()
			except Exception:
				lvl = 0
			if lvl:
				try:
					data = uart.read(lvl)
				except Exception:
					data = None
				if data:
					buf.append(data)
					for f in buf.extract_frames():
						print("SML FRAME len=", len(f), "hex=", hexlify(f).decode())
						# Nur gewünschte OBIS extrahieren
						v_energy = extract_obis_value(f, OBIS_ENERGY_IMPORT)
						v_power  = extract_obis_value(f, OBIS_POWER_TOTAL)
						out = {}
						if v_energy is not None:
							out["1-0:1.8.0"] = v_energy
						if v_power is not None:
							out["1-0:16.7.0"] = v_power
						if out:
							print("SML VALUES:", out)
			now = time.ticks_ms()
			if time.ticks_diff(now, last_wait) >= 1000:
				print("waiting... any=", lvl, "inv=", uart_inverted, "buf=", len(buf.b))
				last_wait = now
			time.sleep_ms(20)
		except KeyboardInterrupt:
			break
		except Exception as e:
			print("Loop error:", e)
			time.sleep_ms(200)


# Starte immer, auch wenn als Modul importiert
main()
