# WLAN, MQTT und UART Konfiguration
# Bitte anpassen

WIFI_SSID = "YOUR_SSID"
WIFI_PASSWORD = "YOUR_PASSWORD"

# ioBroker MQTT Broker
MQTT_BROKER = "192.168.1.10"  # IP/Hostname deines ioBroker MQTT Brokers
MQTT_PORT = 1883
MQTT_USER = None  # z.B. "iobroker"
MQTT_PASSWORD = None  # z.B. "secret"
MQTT_CLIENT_ID = "pico2w-sml"
MQTT_TOPIC_BASE = "sensors/iskra_mt631"
MQTT_PUBLISH_JSON_TOPIC = MQTT_TOPIC_BASE + "/values"
MQTT_PUBLISH_RAW_TOPIC = MQTT_TOPIC_BASE + "/raw"
MQTT_KEEPALIVE = 60
MQTT_RETAIN = False           # ob MQTT-Nachrichten retained werden sollen
MQTT_PUBLISH_PER_OBIS = True  # zusätzlich Einzelwerte je OBIS publizieren

# UART (optische Schnittstelle, SML)
UART_ID = 0                  # UART0
UART_BAUDRATE = 9600         # SML üblich: 9600 8N1
UART_RX_PIN = 1              # GP1 = UART0 RX (nur RX nötig)
UART_TX_PIN = None           # nicht benötigt
UART_INVERT = False          # bei invertierenden IR-Köpfen ggf. True
UART_TIMEOUT_MS = 200

# Sonstiges
READ_INTERVAL_MS = 200       # Polling-Intervall für UART
PUBLISH_EACH_FRAME = True    # bei jedem vollständigen SML-Frame publizieren
