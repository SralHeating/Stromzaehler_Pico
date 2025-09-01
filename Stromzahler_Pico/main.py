# Hauptprogramm: WLAN verbinden, UART lesen, SML parsen, MQTT publizieren
import time
import json

import network
from machine import UART, Pin

import config
import sml


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
    buf = sml.Buffer()

    last_ping = time.ticks_ms()
    while True:
        try:
            data = uart.read()
            if data:
                buf.append(data)
                frames = buf.extract_frames()
                for f in frames:
                    values = sml.parse_obis_from_frame(f)
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
