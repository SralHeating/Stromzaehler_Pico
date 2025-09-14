# Edge Counter: zählt steigende Flanken an GPIO1 und publiziert jede Sekunde per MQTT
import time
import json

import network
from machine import Pin, disable_irq, enable_irq


class _Config:
	# WLAN
	WIFI_SSID = "Milchstrasse"
	WIFI_PASSWORD = "9bG56Rew34vT"
	# MQTT (ioBroker)
	MQTT_BROKER = "192.168.1.6"
	MQTT_PORT = 1883
	MQTT_USER = "ioBroker"
	MQTT_PASSWORD = "iobroker"
	MQTT_CLIENT_ID = "pico2w-edge-counter"
	MQTT_TOPIC_BASE = "sensors/iskra_mt631"
	MQTT_KEEPALIVE = 60
	MQTT_RETAIN = False
	# GPIO zum Zählen (RX-Signal liegt auch auf GPIO1)
	GPIO_PIN = 1
	# Publish-Intervall in ms
	PUBLISH_INTERVAL_MS = 1000
	# Heartbeat-Intervall in ms
	HEARTBEAT_MS = 3000


config = _Config


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
	try:
		from umqtt.simple import MQTTClient
	except ImportError:
		print("umqtt.simple nicht gefunden, versuche Installation mit mip...")
		ok, _ = wifi_connect()
		if not ok:
			raise RuntimeError("WLAN nicht verbunden")
		import mip
		mip.install("umqtt.simple")
		from umqtt.simple import MQTTClient

	client = MQTTClient(client_id=config.MQTT_CLIENT_ID,
						server=config.MQTT_BROKER,
						port=config.MQTT_PORT,
						user=config.MQTT_USER,
						password=config.MQTT_PASSWORD,
						keepalive=config.MQTT_KEEPALIVE)
	client.connect()
	return client


_edge_count = 0
_irq_ref = None


def _on_edge(_pin):
	global _edge_count
	_edge_count += 1


def setup_edge_irq():
	global _irq_ref
	pin = Pin(config.GPIO_PIN, Pin.IN)
	# Zähle ausschließlich steigende Flanken
	_irq_ref = pin.irq(trigger=Pin.IRQ_RISING, handler=_on_edge)


def publish_count(mqtt, topic_base):
	# Kumulierten Zählerstand atomar lesen, aber NICHT zurücksetzen
	state = disable_irq()
	try:
		count = _edge_count
	finally:
		enable_irq(state)
	try:
		mqtt.publish(topic_base + "/rx_rise_count", str(count), retain=config.MQTT_RETAIN)
	except Exception as e:
		print("MQTT publish failed:", e)


def main():
	print("WLAN verbinden...")
	ok, cfg = wifi_connect()
	print("WLAN:", ok, cfg)

	print("MQTT verbinden...")
	mqtt = None
	try:
		mqtt = mqtt_connect()
		print("MQTT verbunden")
	except Exception as e:
		print("MQTT-Verbindung fehlgeschlagen:", e)

	setup_edge_irq()
	print("Edge-IRQ aktiv auf GPIO", config.GPIO_PIN, "Flanke: rising")

	last_pub = time.ticks_ms()
	last_hb = time.ticks_ms()
	while True:
		try:
			now = time.ticks_ms()
			if time.ticks_diff(now, last_pub) >= config.PUBLISH_INTERVAL_MS:
				if mqtt:
					publish_count(mqtt, config.MQTT_TOPIC_BASE)
				last_pub = now

			if time.ticks_diff(now, last_hb) >= config.HEARTBEAT_MS:
				state = disable_irq()
				c = _edge_count
				enable_irq(state)
				print("alive: edges_pending=", c)
				last_hb = now

			# Keep MQTT alive
			if mqtt and time.ticks_diff(now, last_pub) > 30000:
				try:
					mqtt.ping()
				except Exception:
					try:
						mqtt.disconnect()
					except Exception:
						pass
					mqtt = None

			time.sleep_ms(20)
		except KeyboardInterrupt:
			break
		except Exception as e:
			print("Loop error:", e)
			time.sleep_ms(200)


if __name__ == "__main__":
	main()
