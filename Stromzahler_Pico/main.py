# Hauptprogramm: WLAN verbinden, UART lesen, SML parsen, MQTT publizieren
import time
import json

import network
from machine import UART, Pin
# PIO Soft-UART (RP2040) für invertierbares RX
try:
	from rp2 import PIO, asm_pio, StateMachine as RP2StateMachine
	_RP2_AVAILABLE = True
except Exception:
	_RP2_AVAILABLE = False

# Integrierte Konfiguration (vormals config.py)
class _Config:
	WIFI_SSID = "Milchstrasse"
	WIFI_PASSWORD = "9bG56Rew34vT"
	# ioBroker MQTT Broker
	MQTT_BROKER = "192.168.1.6"
	MQTT_PORT = 1883
	MQTT_USER = "ioBroker"
	MQTT_PASSWORD = "iobroker"
	MQTT_CLIENT_ID = "pico2w-sml"
	MQTT_TOPIC_BASE = "sensors/iskra_mt631"
	MQTT_PUBLISH_JSON_TOPIC = MQTT_TOPIC_BASE + "/values"
	MQTT_PUBLISH_RAW_TOPIC = MQTT_TOPIC_BASE + "/raw"
	MQTT_KEEPALIVE = 60
	MQTT_RETAIN = False
	MQTT_PUBLISH_PER_OBIS = True
	# UART (optische Schnittstelle, SML)
	UART_ID = 0
	UART_BAUDRATE = 9600
	UART_RX_PIN = 1
	UART_TX_PIN = None
	UART_INVERT = False
	UART_TIMEOUT_MS = 1500
	# Sonstiges
	READ_INTERVAL_MS = 20
	PUBLISH_EACH_FRAME = True
	# Debug: kurze Diagnose, wenn keine SML-Frames erkannt werden
	DEBUG_NO_SML = True
	# Automatisch RX-Invertierung testen, wenn Daten kommen aber keine SML-Frames
	AUTO_UART_INVERT = True
	# Soft-PIO RX verwenden (empfohlen auf RP2040, erlaubt saubere RX-Invertierung)
	USE_PIO_RX = True

config = _Config

# Integrierter SML-Parser (vormals sml.py)
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
			# SML-Ende: 0x1b1b1b1b1a + 1 Byte Pad-Länge + Pad(0x00)*N + 2 Byte CRC
			tail_idx = end + len(SML_END)
			# Brauchen mindestens PadLen (1) + CRC(2)
			if len(self.buf) < tail_idx + 3:
				break
			pad_len = self.buf[tail_idx]
			# Begrenzen, um wildes Pad zu vermeiden
			if pad_len > 8:
				pad_len = 8
			# Prüfe, ob genug Bytes für Pad + CRC vorhanden
			end_total = tail_idx + 1 + pad_len + 2
			if len(self.buf) < end_total:
				break
			frame = bytes(self.buf[start:end_total])
			frames.append(frame)
			self.buf = self.buf[end_total:]
		return frames

# PIO Soft-UART RX Programme (normal und invertiert)
if _RP2_AVAILABLE:
	@asm_pio(in_shiftdir=PIO.SHIFT_RIGHT, autopush=True, push_thresh=8)
	def _pio_uart_rx():
		# Warte auf Startbit (Leitung geht auf 0, idle=1)
		label("waitstart")
		wait(0, pin, 0)
		# Warte 1.5 Bit bis Mitte des ersten Datenbits
		nop()[3]   # 0.5 Bit (bei freq=8*baud)
		nop()[7]   # +1 Bit
		set(x, 7) # 8 Datenbits
		label("bitloop")
		# 1 (Instr) + 6 (Delay) + 1 (jmp) = 8 Zyklen/Bit
		in_(pins, 1)[6]
		jmp(x_dec, "bitloop")
		# Stopbit ignorieren (1 Bit warten)
		nop()[7]
		jmp("waitstart")

	@asm_pio(in_shiftdir=PIO.SHIFT_RIGHT, autopush=True, push_thresh=8)
	def _pio_uart_rx_inv():
		# Invertierte Leitung: idle=0, Startbit ist 1
		label("waitstart")
		wait(1, pin, 0)
		# Warte 1.5 Bit bis Mitte des ersten Datenbits
		nop()[3]
		nop()[7]
		set(x, 7)
		label("bitloop")
		# 1 (Instr) + 6 (Delay) + 1 (jmp) = 8 Zyklen/Bit
		in_(pins, 1)[6]
		jmp(x_dec, "bitloop")
		nop()[7]
		jmp("waitstart")

	class PioUartRx:
		def __init__(self, rx_pin: int, baudrate: int, inverted: bool = False, sm_id: int = 0):
			self.rx_pin_num = rx_pin
			self.baudrate = int(baudrate)
			self.inverted = bool(inverted)
			self.sm_id = sm_id
			self._sm = None
			self._start()

		def _start(self):
			if self._sm:
				try:
					self._sm.active(0)
				except Exception:
					pass
			pin = Pin(self.rx_pin_num, Pin.IN)  # kein Pull-Up, Lesekopf ist Push-Pull
			prog = _pio_uart_rx_inv if self.inverted else _pio_uart_rx
			self._sm = RP2StateMachine(self.sm_id, prog, freq=self.baudrate * 8, in_base=pin, jmp_pin=pin)
			self._sm.active(1)

		def set_inverted(self, inv: bool):
			if bool(inv) != self.inverted:
				self.inverted = bool(inv)
				self._start()

		def read(self):
			# Liefert None wenn nichts da ist, sonst bytes
			sm = self._sm
			try:
				n = sm.rx_fifo()
			except Exception:
				n = 0
			if not n:
				return None
			out = bytearray()
			for _ in range(n):
				v = sm.get() & 0xFF
				# Bei invertiertem Signal sind die Datenbits invertiert -> rückgängig machen
				if self.inverted:
					v ^= 0xFF
				out.append(v)
			return bytes(out)

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
		# Integers: signed (0x5x) und unsigned (0x6x)
		if (t & 0xF0) in (0x50, 0x60):
			ln = t & 0x0F
			if ln == 0 or pos + 1 + ln > len(data):
				return None, pos
			valb = data[pos+1:pos+1+ln]
			v = 0
			for b in valb:
				v = (v << 8) | (b & 0xFF)
			# signed Korrektur bei 0x5x
			if (t & 0xF0) == 0x50 and ln > 0 and (valb[0] & 0x80):
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
				# Suche Value + Scaler + Unit in den nächsten ~64 Bytes
				p = i + 6
				endp = min(len(data), p + 64)
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
					# Integer-Wert (signed/unsigned)
					v, np = read_int_at(p)
					if v is not None:
						value = v
						p = np
						continue
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

# Entferntes sml-Namespace; direkte Nutzung von Buffer und parse_obis_from_frame


def wifi_connect():
	wlan = network.WLAN(network.STA_IF)
	wlan.active(True)
	if not wlan.isconnected():
		wlan.connect(config.WIFI_SSID, config.WIFI_PASSWORD)
		t0 = time.ticks_ms()
		while not wlan.isconnected() and time.ticks_diff(time.ticks_ms(), t0) < 20000:
			time.sleep_ms(200)
	return wlan.isconnected(), wlan.ifconfig() if wlan.isconnected() else None


def mqtt_connect():
	# Lazy-Import und automatische Installation von umqtt.simple bei Bedarf
	try:
		from umqtt.simple import MQTTClient
	except ImportError:
		print("umqtt.simple nicht gefunden, versuche Installation mit mip...")
		try:
			ok, _ = wifi_connect()
			if not ok:
				raise RuntimeError("WLAN nicht verbunden")
			import mip
			mip.install("umqtt.simple")
			from umqtt.simple import MQTTClient
			print("umqtt.simple installiert")
		except Exception as e:
			raise ImportError("umqtt.simple nicht verfügbar: {}".format(e))

	client = MQTTClient(client_id=config.MQTT_CLIENT_ID,
						server=config.MQTT_BROKER,
						port=config.MQTT_PORT,
						user=config.MQTT_USER,
						password=config.MQTT_PASSWORD,
						keepalive=config.MQTT_KEEPALIVE)
	# LWT optional: nicht konfiguriert
	client.connect()
	return client


def _build_uart(inverted=None):
	inv = getattr(config, 'UART_INVERT', False) if inverted is None else inverted
	invert_kw = {}
	try:
		if inv:
			invert_kw = { 'invert': UART.INV_RX }
	except Exception:
		invert_kw = {}
	# RX explizit setzen, mit neutralem Eingang (Push-Pull-Kopf liefert Pegel)
	rx_arg = {}
	if getattr(config, 'UART_RX_PIN', None) is not None:
		rx_arg = { 'rx': Pin(config.UART_RX_PIN, Pin.IN) }
	return UART(
		config.UART_ID,
		
		parity=None,
		stop=1,
		timeout=getattr(config, 'UART_TIMEOUT_MS', 1500),
		timeout_char=50,
		rxbuf=2048,
		**rx_arg,
		**invert_kw
	)


def setup_uart():
	# PIO-Soft-RX bevorzugen, wenn verfügbar/aktiviert, sonst Hardware-UART
	use_pio = getattr(config, 'USE_PIO_RX', False) and _RP2_AVAILABLE
	if use_pio:
		try:
			uart = PioUartRx(config.UART_RX_PIN, config.UART_BAUDRATE, inverted=getattr(config, 'UART_INVERT', False), sm_id=0)
			print("PIO Soft-UART RX aktiv (invert=", getattr(config, 'UART_INVERT', False), ")")
			return uart
		except Exception as e:
			print("PIO RX init failed:", e, "-> fallback auf HW-UART")
	# UART0 RX auf GP1; viele IR-Köpfe liefern 5V TTL -> Pegelwandler verwenden.
	return _build_uart(getattr(config, 'UART_INVERT', False))


def publish_dict(client, topic, d):
	try:
		payload = json.dumps({k: v for k, v in d.items() if not k.startswith('_')})
		client.publish(topic, payload, retain=config.MQTT_RETAIN)
	except Exception as e:
		print("MQTT publish failed:", e)


def publish_per_obis(client, base, d):
	if not getattr(config, 'MQTT_PUBLISH_PER_OBIS', False):
		return
	for k, v in d.items():
		if k.startswith('_'):
			continue
		t = base + "/" + k
		try:
			client.publish(t, str(v), retain=config.MQTT_RETAIN)
		except Exception as e:
			print("MQTT publish obis failed:", k, e)

# Leichte Status-Publish-Hilfe

def publish_status(client, base, msg):
	try:
		client.publish(base + "/status", msg)
	except Exception as e:
		print("MQTT status publish failed:", e)


# Zähler und Publisher für den Fall, dass keine SML-Daten erkannt werden
# Zählt jede Sekunde hoch (0..100) und publiziert den Wert per MQTT

def no_sml_counter_tick(mqtt, last_tick_ms, counter, topic):
	now = time.ticks_ms()
	if time.ticks_diff(now, last_tick_ms) >= 1000:
		counter += 1
		if counter > 100:
			counter = 0
		if mqtt:
			try:
				mqtt.publish(topic, str(counter), retain=False)
			except Exception as e:
				print("MQTT counter publish failed:", e)
		last_tick_ms = now
	return last_tick_ms, counter


def main():
	print("WLAN verbinden...")
	ok, cfg = wifi_connect()
	if not ok:
		print("WLAN-Verbindung fehlgeschlagen")
	else:
		print("WLAN ok:", cfg)

	print("MQTT verbinden...")
	mqtt = None
	try:
		mqtt = mqtt_connect()
		print("MQTT verbunden")
	except Exception as e:
		print("MQTT-Verbindung fehlgeschlagen:", e)

	uart = setup_uart()
	uart_inverted = getattr(config, 'UART_INVERT', False)
	buf = Buffer()

	print("Starte Hauptschleife…")

	# Initialisierung des No-SML-Zählers
	no_sml_counter = 0
	no_sml_last_tick = time.ticks_ms()
	counter_topic = config.MQTT_TOPIC_BASE + "/no_sml_counter"

	last_ping = time.ticks_ms()
	last_no_frame_diag = time.ticks_ms()
	no_frame_since = time.ticks_ms()
	last_heartbeat = time.ticks_ms()
	while True:
		try:
			had_frames = False
			# Drain UART/PIO mehrfach pro Loop, um FIFO-Überläufe zu vermeiden
			for _ in range(32):
				data = uart.read()
				if not data:
					break
				if getattr(config, 'DEBUG_NO_SML', False):
					try:
						print("rx:", hexlify(data[:24]).decode())
					except Exception:
						pass
				buf.append(data)
				frames = buf.extract_frames()
				if frames:
					had_frames = True
					no_frame_since = time.ticks_ms()
					for f in frames:
						values = parse_obis_from_frame(f)
						if mqtt:
							if config.PUBLISH_EACH_FRAME:
								publish_dict(mqtt, config.MQTT_PUBLISH_JSON_TOPIC, values)
								publish_per_obis(mqtt, config.MQTT_TOPIC_BASE, values)
							if "_raw_hex" in values:
								try:
									mqtt.publish(config.MQTT_PUBLISH_RAW_TOPIC, values["_raw_hex"], retain=False)
								except Exception as e:
									print("MQTT raw publish failed:", e)
						else:
							print("SML:", values)

			# Diagnose/Heartbeat: auch ohne UART-Daten aktiv
			now = time.ticks_ms()
			if getattr(config, 'DEBUG_NO_SML', False) and not had_frames and time.ticks_diff(now, last_no_frame_diag) > 3000:
				b = buf.buf
				s_idx = b.find(SML_START)
				e_idx = b.find(SML_END)
				snip = hexlify(bytes(b[-32:])).decode() if len(b) else ""
				print("SML diag: start=", s_idx, "end=", e_idx, "buf=", len(b), "tail_hex=", snip)
				if mqtt:
					publish_status(mqtt, config.MQTT_TOPIC_BASE, "diag start=%d end=%d buf=%d" % (s_idx, e_idx, len(b)))
				last_no_frame_diag = now

			# Auto-UART-Invertierung nach 5s ohne Frames
			if getattr(config, 'AUTO_UART_INVERT', True) and time.ticks_diff(now, no_frame_since) > 5000:
				try:
					# PIO-Soft-UART bevorzugt umschalten, sonst HW-UART neu aufbauen
					if _RP2_AVAILABLE and isinstance(uart, PioUartRx):
						uart_inverted = not uart_inverted
						uart.set_inverted(uart_inverted)
						buf = Buffer()
						print("PIO RX invert toggled ->", uart_inverted)
						if mqtt:
							publish_status(mqtt, config.MQTT_TOPIC_BASE, "uart_invert=%s" % uart_inverted)
					else:
						uart = _build_uart(not uart_inverted)
						uart_inverted = not uart_inverted
						buf = Buffer()
						print("HW UART RX invert toggled ->", uart_inverted)
						if mqtt:
							publish_status(mqtt, config.MQTT_TOPIC_BASE, "uart_invert=%s" % uart_inverted)
				except Exception as e:
					print("UART invert toggle failed:", e)
				no_frame_since = now

			# Heartbeat alle 3s
			if time.ticks_diff(now, last_heartbeat) > 3000:
				print("alive: inv=", uart_inverted, "buf=", len(buf.buf))
				if mqtt:
					publish_status(mqtt, config.MQTT_TOPIC_BASE, "alive buf=%d inv=%s" % (len(buf.buf), uart_inverted))
				last_heartbeat = now

			# Wenn keine SML-Frames erkannt wurden, Zähler tick/publish ausführen
			if not had_frames:
				no_sml_last_tick, no_sml_counter = no_sml_counter_tick(
					mqtt, no_sml_last_tick, no_sml_counter, counter_topic
				)

			# Reconnect-Logik
			if mqtt is None:
				try:
					mqtt = mqtt_connect()
				except:
					pass

			# Keepalive/Ping
			if mqtt and time.ticks_diff(time.ticks_ms(), last_ping) > 30000:
				try:
					mqtt.ping()
				except Exception:
					try:
						mqtt.disconnect()
					except Exception:
						pass
					mqtt = None
				last_ping = time.ticks_ms()

			time.sleep_ms(config.READ_INTERVAL_MS)
		except KeyboardInterrupt:
			break
		except Exception as e:
			print("Loop error:", e)
			time.sleep_ms(200)


if __name__ == "__main__":
	main()
