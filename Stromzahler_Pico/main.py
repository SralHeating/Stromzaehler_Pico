# Hauptprogramm: WLAN verbinden, UART lesen, SML parsen, MQTT publizieren
import time
import json

import network
from machine import UART, Pin

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
	UART_TIMEOUT_MS = 200
	# Sonstiges
	READ_INTERVAL_MS = 200
	PUBLISH_EACH_FRAME = True

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


def setup_uart():
	# UART0 RX auf GP1; viele IR-Köpfe liefern 5V TTL -> Pegelwandler verwenden.
	invert_kw = {}
	try:
		if getattr(config, 'UART_INVERT', False):
			invert_kw = { 'invert': UART.INV_RX }
	except Exception:
		invert_kw = {}
	uart = UART(config.UART_ID, baudrate=config.UART_BAUDRATE, bits=8, parity=None, stop=1, **invert_kw)
	if config.UART_RX_PIN is not None:
		Pin(config.UART_RX_PIN, Pin.IN)
	return uart


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
	buf = Buffer()

	# Initialisierung des No-SML-Zählers
	no_sml_counter = 0
	no_sml_last_tick = time.ticks_ms()
	counter_topic = config.MQTT_TOPIC_BASE + "/no_sml_counter"

	last_ping = time.ticks_ms()
	while True:
		try:
			had_frames = False
			data = uart.read()
			if data:
				buf.append(data)
				frames = buf.extract_frames()
				if frames:
					had_frames = True
				for f in frames:
					values = parse_obis_from_frame(f)
					if mqtt:
						if config.PUBLISH_EACH_FRAME:
							publish_dict(mqtt, config.MQTT_PUBLISH_JSON_TOPIC, values)
							publish_per_obis(mqtt, config.MQTT_TOPIC_BASE, values)
						# Rohdaten optional
						if "_raw_hex" in values:
							try:
								mqtt.publish(config.MQTT_PUBLISH_RAW_TOPIC, values["_raw_hex"], retain=False)
							except Exception as e:
								print("MQTT raw publish failed:", e)
					else:
						print("SML:", values)

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
