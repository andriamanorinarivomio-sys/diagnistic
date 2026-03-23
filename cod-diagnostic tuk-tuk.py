import tkinter as tk
from tkinter import ttk, messagebox
import threading, time, random, csv, os, datetime, sys
from collections import deque
import RPi.GPIO as GPIO
import Adafruit_ADS1x15
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from smbus2 import SMBus
from scipy.signal import butter, filtfilt

# ================== DS18B20 ==================
try:
    from w1thermsensor import W1ThermSensor
    DS18B20_ENABLED = True
    sensor_oil = W1ThermSensor()
except ImportError:
    DS18B20_ENABLED = False
    sensor_oil = None
    print("Module w1thermsensor non installé, DS18B20 désactivé.")

# ================== GPIO ==================
HALL_RPM_PIN = 17
HALL_KM_PIN = 23
LED1_PIN = 27    # LED VERTE - allumée en condition normale
LED2_PIN = 22    # LED ORANGE - clignote pour anomalies ORANGE (pas graves)
LED3_PIN = 24    # LED ROUGE - clignote pour anomalies ROUGE (critiques)
PASSES_PAR_TOUR = 1          # Changé de 3 à 1 car on utilise un seul aimant pour le régime moteur
DEBOUNCE_TIME = 0.003
POLL_DELAY = 0.0005
SMOOTHING = 0.2

# Constantes pour le kilométrage (Bajaj RE 4S)
ROUE_DIAMETRE = 0.55  # mètres
CIRCONFERENCE_ROUE = 3.14159 * ROUE_DIAMETRE  # ≈ 1.727 m
KM_PAR_IMPULSION = CIRCONFERENCE_ROUE / 1000  # ≈ 0.001727 km par passage d'aimant

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(HALL_RPM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(HALL_KM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LED1_PIN, GPIO.OUT)  # LED VERTE
GPIO.setup(LED2_PIN, GPIO.OUT)  # LED ORANGE
GPIO.setup(LED3_PIN, GPIO.OUT)  # LED ROUGE
GPIO.output(LED1_PIN, GPIO.HIGH)  # LED1 verte allumée en permanence (normal)
GPIO.output(LED2_PIN, GPIO.LOW)   # LED2 orange éteinte
GPIO.output(LED3_PIN, GPIO.LOW)   # LED3 rouge éteinte

# ================== VARIABLES GLOBALES POUR LES LEDS ==================
alerte_orange_active = False
alerte_rouge_active = False
led2_clignotement = False
led3_clignotement = False
leds_thread_running = True

# Thread de clignotement pour les LEDs ORANGE et ROUGE
def thread_clignotement_leds():
    global leds_thread_running, led2_clignotement, led3_clignotement
    global alerte_orange_active, alerte_rouge_active
    
    etat_orange = False
    etat_rouge = False
    
    while leds_thread_running:
        # Gestion LED ORANGE (clignotement rapide)
        if led2_clignotement:
            etat_orange = not etat_orange
            GPIO.output(LED2_PIN, etat_orange)
        else:
            if etat_orange:
                GPIO.output(LED2_PIN, GPIO.LOW)
                etat_orange = False
        
        # Gestion LED ROUGE (clignotement rapide)
        if led3_clignotement:
            etat_rouge = not etat_rouge
            GPIO.output(LED3_PIN, etat_rouge)
        else:
            if etat_rouge:
                GPIO.output(LED3_PIN, GPIO.LOW)
                etat_rouge = False
        
        # Gestion LED VERTE : éteinte si une alerte est active
        if alerte_orange_active or alerte_rouge_active:
            GPIO.output(LED1_PIN, GPIO.LOW)   # Éteindre LED verte
        else:
            GPIO.output(LED1_PIN, GPIO.HIGH)  # Allumer LED verte
        
        time.sleep(0.1)  # Clignotement à 10 Hz

# Démarrer le thread de clignotement
threading.Thread(target=thread_clignotement_leds, daemon=True).start()

# ================== MPU6050 ==================
I2C_BUS = 1
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
FS = 500
DT = 1 / FS
N_FFT = 1024
CUTOFF = 80

MPU_ENABLED = False
bus = None
mpu_offset = 0
mpu_ax_offset = 0
mpu_ay_offset = 0

try:
    bus = SMBus(I2C_BUS)
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    MPU_ENABLED = True
    print("MPU6050 détecté")
except:
    print("MPU6050 non détecté - mode simulation activé")

def read_word(reg):
    try:
        h = bus.read_byte_data(MPU_ADDR, reg)
        l = bus.read_byte_data(MPU_ADDR, reg+1)
        val = (h << 8) | l
        if val >= 32768:
            val -= 65536
        return val
    except:
        return 0

def read_accel_z():
    if MPU_ENABLED and bus:
        az = read_word(ACCEL_XOUT_H + 4)
        return az / 16384.0
    else:
        return random.uniform(-0.5, 0.5)

def read_accel_x():
    if MPU_ENABLED and bus:
        ax = read_word(ACCEL_XOUT_H)
        return ax / 16384.0
    else:
        return random.uniform(-0.5, 0.5)

def read_accel_y():
    if MPU_ENABLED and bus:
        ay = read_word(ACCEL_XOUT_H + 2)
        return ay / 16384.0
    else:
        return random.uniform(-0.5, 0.5)

# Calibration MPU
if MPU_ENABLED:
    print("Calibration MPU... ne bouge pas")
    samples = 500
    z_sum = 0
    x_sum = 0
    y_sum = 0
    for _ in range(samples):
        z_sum += read_accel_z()
        x_sum += read_accel_x()
        y_sum += read_accel_y()
        time.sleep(0.005)
    mpu_offset = z_sum / samples
    mpu_ax_offset = x_sum / samples
    mpu_ay_offset = y_sum / samples
    print(f"Offset Z = {mpu_offset:.5f} g")
    print(f"Offset X = {mpu_ax_offset:.5f} g")
    print(f"Offset Y = {mpu_ay_offset:.5f} g")

b, a = butter(4, CUTOFF / (FS / 2), btype='low')

# ================== VARIABLES ==================
running = True
rpm = 0.0
kilometers = 0.0
total_passages = 1
total_tours = 0.0
last_rpm_state = GPIO.input(HALL_RPM_PIN)
last_rpm_trigger = time.monotonic()
total_km_passages = 0
last_km_state = GPIO.input(HALL_KM_PIN)

# ================== SYSTÈME DE CONFIRMATION TEMPORELLE DES DÉFAUTS ==================
fault_start_time = {}
fault_confirmed = {}
CONFIRMATION_DELAY = 3.0

ALL_FAULT_CODES = [
    "P0562", "P0563", "P0560",
    "P0217", "P0520", "P0218",
    "P0171",
    "P0300", "P0301",
    "P0700",
    "P0505", "P0506", "P0507",
    "P0420"
]

for code in ALL_FAULT_CODES:
    fault_start_time[code] = 0.0
    fault_confirmed[code] = False

fault_lock = threading.Lock()

# ================== HISTORIQUE DES DÉFAUTS ==================
fault_history = deque(maxlen=100)
history_file = "fault_history.csv"

def save_fault_to_history(fault, duree):
    fault_record = {
        "timestamp": datetime.datetime.now(),
        "code": fault.get("code", "N/A"),
        "niveau": fault.get("niveau", "N/A"),
        "capteur": fault.get("capteur", "N/A"),
        "valeur": fault.get("valeur", "N/A"),
        "probleme": fault.get("probleme", "N/A"),
        "action": fault.get("action", "N/A"),
        "duree": duree
    }
    fault_history.append(fault_record)
    
    try:
        file_exists = os.path.isfile(history_file)
        with open(history_file, "a", newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(["timestamp", "code", "niveau", "capteur", "valeur", "probleme", "action", "duree"])
            writer.writerow([
                fault_record["timestamp"].strftime("%Y-%m-%d %H:%M:%S"),
                fault_record["code"],
                fault_record["niveau"],
                fault_record["capteur"],
                fault_record["valeur"],
                fault_record["probleme"],
                fault_record["action"],
                f"{duree:.1f}"
            ])
    except Exception as e:
        print(f"Erreur sauvegarde historique: {e}")

def check_fault_with_confirmation(fault_code, condition, current_time, popup=None, fault_data=None):
    with fault_lock:
        if condition:
            if fault_start_time[fault_code] == 0:
                fault_start_time[fault_code] = current_time
                fault_confirmed[fault_code] = False
            elif (current_time - fault_start_time[fault_code]) >= CONFIRMATION_DELAY:
                if not fault_confirmed[fault_code]:
                    fault_confirmed[fault_code] = True
                    if fault_data and popup:
                        duree = current_time - fault_start_time[fault_code]
                        save_fault_to_history(fault_data, duree)
        else:
            if fault_start_time[fault_code] != 0 and not fault_confirmed[fault_code]:
                pass
            fault_start_time[fault_code] = 0
            fault_confirmed[fault_code] = False
    
    return fault_confirmed[fault_code]

# ================== ADS1115 ==================
ads = Adafruit_ADS1x15.ADS1115(busnum=1)
GAIN = 1

# Définition des canaux ADS1115
LM35_CHANNEL = 0        # Canal A0 - Température moteur (LM35)
ACS712_CHANNEL = 1      # Canal A1 - Capteur courant ACS712
VOLTAGE_CHANNEL = 2     # Canal A2 - Capteur tension (pont diviseur)

# Paramètres de calibration
LM35_OFFSET = 5.0
DS18B20_OFFSET = -1.0

# Paramètres ACS712 (capteur de courant 20A)
ACS712_ZERO = 2.5       # Tension à courant nul (2.5V pour ACS712 20A)
ACS712_SENSITIVITY = 0.1  # 100 mV/A pour ACS712 20A

# Paramètres capteur tension (pont diviseur)
VOLTAGE_DIVIDER_RATIO = 5.0  # Rapport de division (ex: 10k/2.2k = 4.54, ajuster selon montage)
VOLTAGE_OFFSET = 0.0

# Variables pour les températures avec filtrage
last_temp_block = 0.0
last_temp_oil = 0.0
last_current = 0.0
last_voltage = 0.0

# Buffers pour filtrage
lm35_buffer = deque(maxlen=10)
lm35_smoothed = 0.0
lm35_alpha = 0.3

ds18b20_buffer = deque(maxlen=5)
ds18b20_smoothed = 0.0
ds18b20_alpha = 0.2
ds18b20_last_valid = 0.0
ds18b20_timeout = 0

current_buffer = deque(maxlen=10)
current_smoothed = 0.0
current_alpha = 0.3

voltage_buffer = deque(maxlen=10)
voltage_smoothed = 0.0
voltage_alpha = 0.3

# ================== VARIABLES VIBRATION ==================
vibration_buffer = deque(maxlen=N_FFT)
vibration_filtered = deque(maxlen=N_FFT)
vibration_fft = deque(maxlen=N_FFT//2 + 1)
vibration_rms = 0.0
vibration_rms_history = deque(maxlen=100)
vibration_raw_history = deque(maxlen=100)

for _ in range(N_FFT):
    vibration_buffer.append(0.0)
    vibration_filtered.append(0.0)
for _ in range(N_FFT//2 + 1):
    vibration_fft.append(0.0)
for _ in range(100):
    vibration_rms_history.append(0.0)
    vibration_raw_history.append(0.0)

# ================== VARIABLES MPU 3 AXES ==================
mpu_buffer_size = 100
buf_x = deque(maxlen=mpu_buffer_size)
buf_y = deque(maxlen=mpu_buffer_size)
buf_z_filt = deque(maxlen=mpu_buffer_size)
buf_z_raw = deque(maxlen=mpu_buffer_size)

for _ in range(mpu_buffer_size):
    buf_x.append(0.0)
    buf_y.append(0.0)
    buf_z_filt.append(0.0)
    buf_z_raw.append(0.0)

ax_f = ay_f = az_f = 0.0
alpha = 0.15

# ================== VARIABLES SUIVI ENTRETIEN ==================
maintenance_km = {
    "Huile Moteur": 0.0,
    "Filtre à huile": 0.0,
    "Crépine": 0.0,
    "Filtre à air": 0.0,
    "Bougies": 0.0,
    "Carburateur": 0.0,
    "Filtre à essence": 0.0,
    "Soupapes": 0.0,
    "Chaîne distribution": 0.0,
    "Câbles": 0.0,
    "Culasse": 0.0,
    "Reniflard": 0.0,
    "Liquide de frein": 0.0,
    "Plaquettes de frein": 0.0,
    "Pneumatiques": 0.0,
    "Batterie": 0.0,
    "Alternateur": 0.0,
    "Embrayage": 0.0,
    "Roulements de roue": 0.0,
    "Amortisseurs": 0.0,
    "Cardans": 0.0,
    "Boîte de vitesses": 0.0,
    "Pont arrière": 0.0,
    "Refroidissement": 0.0
}

last_reset_km = {
    "Huile Moteur": 0.0,
    "Filtre à huile": 0.0,
    "Crépine": 0.0,
    "Filtre à air": 0.0,
    "Bougies": 0.0,
    "Carburateur": 0.0,
    "Filtre à essence": 0.0,
    "Soupapes": 0.0,
    "Chaîne distribution": 0.0,
    "Câbles": 0.0,
    "Culasse": 0.0,
    "Reniflard": 0.0,
    "Liquide de frein": 0.0,
    "Plaquettes de frein": 0.0,
    "Pneumatiques": 0.0,
    "Batterie": 0.0,
    "Alternateur": 0.0,
    "Embrayage": 0.0,
    "Roulements de roue": 0.0,
    "Amortisseurs": 0.0,
    "Cardans": 0.0,
    "Boîte de vitesses": 0.0,
    "Pont arrière": 0.0,
    "Refroidissement": 0.0
}

# ================== DIAGNOSTIC POPUP ==================
class CodeErreur:
    codes_utilises = set()
    
    @classmethod
    def generer(cls, categorie):
        import random
        while True:
            code = f"P{random.randint(1000, 9999)}"
            if code not in cls.codes_utilises:
                cls.codes_utilises.add(code)
                return code

class DiagnosticPopup:
    _instance = None
    
    @classmethod
    def get_instance(cls, parent):
        if cls._instance is None or cls._instance.popup is None or not cls._instance.popup.winfo_exists():
            cls._instance = cls(parent)
        return cls._instance
    
    def __init__(self, parent):
        self.parent = parent
        self.popup = None
        self.anomalies = []
        self.entretien_alerts = []
    
    def show_if_needed(self):
        if (self.anomalies or self.entretien_alerts) and (self.popup is None or not self.popup.winfo_exists()):
            print(f"Ouverture automatique de la fenêtre diagnostic - {len(self.anomalies)} anomalie(s), {len(self.entretien_alerts)} alerte(s) entretien!")
            self.creer_popup()
            return True
        return False
    
    def creer_popup(self):
        self.popup = tk.Toplevel(self.parent)
        self.popup.title("DIAGNOSTIC BAJAJ RE 4S")
        self.popup.geometry("450x500")
        self.popup.configure(bg="#1e1e1e")
        self.popup.attributes("-topmost", True)
        
        self.popup.bind("<Button-1>", self.debut_deplacement)
        self.popup.bind("<B1-Motion>", self.deplacement)
        
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Diagnostic.TFrame", background="#1e1e1e")
        style.configure("Header.TLabel", background="#1e1e1e", foreground="#3498db", 
                       font=("Consolas", 12, "bold"))
        style.configure("Normal.TLabel", background="#1e1e1e", foreground="white", 
                       font=("Consolas", 10))
        
        header_frame = ttk.Frame(self.popup, style="Diagnostic.TFrame")
        header_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(header_frame, text="DIAGNOSTIC BAJAJ RE 4S (Moteur traditionnel)", 
                 style="Header.TLabel").pack(side=tk.LEFT)
        ttk.Label(header_frame, text=datetime.datetime.now().strftime("%H:%M:%S"),
                 style="Normal.TLabel").pack(side=tk.RIGHT)
        
        ttk.Separator(self.popup, orient='horizontal').pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(self.popup, text="ANOMALIES ACTIVES", 
                 style="Header.TLabel").pack(anchor=tk.W, padx=10, pady=(5,0))
        
        self.canvas_anomalies = tk.Canvas(self.popup, bg="#1e1e1e", height=200,
                                         highlightthickness=0)
        self.canvas_anomalies.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        scrollbar = ttk.Scrollbar(self.popup, orient="vertical", 
                                  command=self.canvas_anomalies.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas_anomalies.configure(yscrollcommand=scrollbar.set)
        
        self.frame_anomalies = ttk.Frame(self.canvas_anomalies, style="Diagnostic.TFrame")
        self.canvas_anomalies.create_window((0, 0), window=self.frame_anomalies, anchor="nw")
        self.frame_anomalies.bind("<Configure>", self.on_frame_configure)
        
        ttk.Separator(self.popup, orient='horizontal').pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(self.popup, text="ENTRETIEN PRÉVENTIF", 
                 style="Header.TLabel").pack(anchor=tk.W, padx=10, pady=(5,0))
        self.frame_entretien = ttk.Frame(self.popup, style="Diagnostic.TFrame")
        self.frame_entretien.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        button_frame = ttk.Frame(self.popup, style="Diagnostic.TFrame")
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        ttk.Button(button_frame, text="Actualiser", 
                  command=self.rafraichir_affichage,
                  style="Accent.TButton").pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Fermer", 
                  command=self.fermer,
                  style="Accent.TButton").pack(side=tk.RIGHT, padx=5)
        
        self.rafraichir_affichage()
    
    def on_frame_configure(self, event):
        self.canvas_anomalies.configure(scrollregion=self.canvas_anomalies.bbox("all"))
    
    def debut_deplacement(self, event):
        self.x = event.x
        self.y = event.y
    
    def deplacement(self, event):
        deltax = event.x - self.x
        deltay = event.y - self.y
        x = self.popup.winfo_x() + deltax
        y = self.popup.winfo_y() + deltay
        self.popup.geometry(f"+{x}+{y}")
    
    def ajouter_anomalie(self, niveau, capteur, valeur, seuil, probleme, action, code=None):
        if code is None:
            code = CodeErreur.generer(categorie=capteur[:3])
        anomalie = {
            "niveau": niveau,
            "code": code,
            "capteur": capteur,
            "valeur": valeur,
            "seuil": seuil,
            "probleme": probleme,
            "action": action,
            "timestamp": datetime.datetime.now()
        }
        self.anomalies.append(anomalie)
        return anomalie
    
    def ajouter_entretien(self, composant, km_actuel, km_seuil, action):
        alert = {
            "composant": composant,
            "km_actuel": km_actuel,
            "km_seuil": km_seuil,
            "action": action,
            "timestamp": datetime.datetime.now()
        }
        self.entretien_alerts.append(alert)
    
    def rafraichir_affichage(self):
        if self.popup and self.popup.winfo_exists():
            for widget in self.frame_anomalies.winfo_children():
                widget.destroy()
            for widget in self.frame_entretien.winfo_children():
                widget.destroy()
            
            if not self.anomalies:
                ttk.Label(self.frame_anomalies, text="Aucune anomalie détectée",
                         font=("Consolas", 10), foreground="#2ecc71",
                         background="#1e1e1e").pack(pady=20)
            else:
                for ano in sorted(self.anomalies, key=lambda x: 
                                 ["ROUGE", "ORANGE", "JAUNE", "BLEU"].index(x["niveau"])):
                    self._creer_carte_anomalie(ano)
            
            if not self.entretien_alerts:
                ttk.Label(self.frame_entretien, text="Entretien à jour",
                         font=("Consolas", 10), foreground="#2ecc71",
                         background="#1e1e1e").pack(pady=10)
            else:
                for alert in self.entretien_alerts:
                    self._creer_carte_entretien(alert)
    
    def _creer_carte_anomalie(self, ano):
        couleurs = {
            "ROUGE": {"bg": "#5a1e1e", "fg": "#ff6b6b", "border": "#ff4444"},
            "ORANGE": {"bg": "#5a3a1e", "fg": "#ffb86b", "border": "#ff9900"},
            "JAUNE": {"bg": "#5a5a1e", "fg": "#f1c40f", "border": "#ffff00"},
            "BLEU": {"bg": "#1e3a5a", "fg": "#6bb0ff", "border": "#3498db"},
        }
        c = couleurs[ano["niveau"]]
        
        frame = tk.Frame(self.frame_anomalies, bg=c["bg"], 
                        highlightbackground=c["border"], highlightthickness=2,
                        bd=0, padx=5, pady=5)
        frame.pack(fill=tk.X, pady=3, padx=2)
        
        ligne1 = tk.Frame(frame, bg=c["bg"])
        ligne1.pack(fill=tk.X)
        tk.Label(ligne1, text=f"[{ano['code']}]", bg=c["bg"], fg=c["fg"],
                font=("Consolas", 9, "bold")).pack(side=tk.LEFT)
        tk.Label(ligne1, text=ano["niveau"], bg=c["bg"], fg="white",
                font=("Consolas", 9, "bold"), width=6).pack(side=tk.LEFT, padx=5)
        tk.Label(ligne1, text=ano["timestamp"].strftime("%H:%M:%S"),
                bg=c["bg"], fg="gray", font=("Consolas", 8)).pack(side=tk.RIGHT)
        
        tk.Label(frame, text=f"{ano['probleme']}", bg=c["bg"], fg="white",
                font=("Consolas", 10, "bold"), wraplength=350, justify=tk.LEFT).pack(anchor=tk.W, pady=2)
        tk.Label(frame, text=f"{ano['capteur']}: {ano['valeur']} (seuil: {ano['seuil']})",
                bg=c["bg"], fg=c["fg"], font=("Consolas", 9)).pack(anchor=tk.W)
        
        action_lines = ano['action'].split('\n')
        for line in action_lines:
            if line.strip():
                tk.Label(frame, text=f"{line.strip()}", bg=c["bg"], fg="#f0f0f0",
                        font=("Consolas", 8), wraplength=350, justify=tk.LEFT).pack(anchor=tk.W, pady=1)
    
    def _creer_carte_entretien(self, alert):
        frame = tk.Frame(self.frame_entretien, bg="#2c3e50", 
                        highlightbackground="#27ae60", highlightthickness=1,
                        bd=0, padx=5, pady=3)
        frame.pack(fill=tk.X, pady=2)
        
        ligne1 = tk.Frame(frame, bg="#2c3e50")
        ligne1.pack(fill=tk.X)
        tk.Label(ligne1, text=f"{alert['composant']}", bg="#2c3e50", fg="#f39c12",
                font=("Consolas", 10, "bold")).pack(side=tk.LEFT)
        tk.Label(ligne1, text=f"{alert['km_actuel']:.0f}/{alert['km_seuil']} km",
                bg="#2c3e50", fg="white", font=("Consolas", 9)).pack(side=tk.RIGHT)
        
        progress_frame = tk.Frame(frame, bg="#34495e", height=5, width=300)
        progress_frame.pack(pady=2)
        progress_frame.pack_propagate(False)
        ratio = min(1.0, alert['km_actuel'] / alert['km_seuil'])
        progress_width = int(300 * ratio)
        tk.Frame(progress_frame, bg="#27ae60", height=5, width=progress_width).place(x=0, y=0)
        
        tk.Label(frame, text=f"→ {alert['action']}", bg="#2c3e50", fg="#b0b0b0",
                font=("Consolas", 8), wraplength=320, justify=tk.LEFT).pack(anchor=tk.W)
    
    def fermer(self):
        if self.popup:
            self.popup.destroy()
            self.popup = None

# ================== FONCTION DE GESTION DES LEDS ==================
def mettre_a_jour_leds(anomalies):
    global alerte_orange_active, alerte_rouge_active
    global led2_clignotement, led3_clignotement
    
    anomalies_rouge = [a for a in anomalies if a.get("niveau") == "ROUGE"]
    anomalies_orange = [a for a in anomalies if a.get("niveau") == "ORANGE"]
    
    alerte_rouge_active = len(anomalies_rouge) > 0
    alerte_orange_active = len(anomalies_orange) > 0
    
    led3_clignotement = alerte_rouge_active
    led2_clignotement = alerte_orange_active

# ================== FONCTIONS DE LECTURE CAPTEURS ==================

def read_acs712(samples=10):
    """Lecture du capteur de courant ACS712 sur canal A1"""
    global current_buffer, current_smoothed
    
    valeurs = []
    for _ in range(samples):
        try:
            value = ads.read_adc(ACS712_CHANNEL, gain=GAIN)
            voltage = value * (3.3 / 32767)
            # Conversion tension → courant (ACS712: 2.5V = 0A, 100mV/A)
            current = (voltage - ACS712_ZERO) / ACS712_SENSITIVITY
            if -30 < current < 30:
                valeurs.append(current)
            time.sleep(0.005)
        except:
            pass
    
    if len(valeurs) < 3:
        return current_smoothed if current_smoothed != 0 else 0.0
    
    valeurs.sort()
    median = valeurs[len(valeurs)//2]
    current_buffer.append(median)
    
    if len(current_buffer) > 0:
        moyenne = sum(current_buffer) / len(current_buffer)
        current_smoothed = current_alpha * moyenne + (1 - current_alpha) * current_smoothed
    
    return current_smoothed

def read_voltage(samples=10):
    """Lecture du capteur de tension sur canal A2 (pont diviseur)"""
    global voltage_buffer, voltage_smoothed
    
    valeurs = []
    for _ in range(samples):
        try:
            value = ads.read_adc(VOLTAGE_CHANNEL, gain=GAIN)
            voltage_ads = value * (3.3 / 32767)
            # Conversion avec rapport du pont diviseur
            voltage = voltage_ads * VOLTAGE_DIVIDER_RATIO + VOLTAGE_OFFSET
            if 0 < voltage < 20:
                valeurs.append(voltage)
            time.sleep(0.005)
        except:
            pass
    
    if len(valeurs) < 3:
        return voltage_smoothed if voltage_smoothed > 0 else 12.0
    
    valeurs.sort()
    median = valeurs[len(valeurs)//2]
    voltage_buffer.append(median)
    
    if len(voltage_buffer) > 0:
        moyenne = sum(voltage_buffer) / len(voltage_buffer)
        voltage_smoothed = voltage_alpha * moyenne + (1 - voltage_alpha) * voltage_smoothed
    
    return voltage_smoothed

def read_lm35(samples=10):
    global lm35_buffer, lm35_smoothed
    
    valeurs = []
    for _ in range(samples):
        try:
            value = ads.read_adc(LM35_CHANNEL, gain=GAIN)
            voltage = value * (3.3 / 32767)
            temp = voltage / 0.01
            
            if 0 < temp < 150:
                valeurs.append(temp)
            time.sleep(0.01)
        except:
            pass
    
    if len(valeurs) < 3:
        return lm35_smoothed if lm35_smoothed > 0 else 25.0
    
    valeurs.sort()
    median = valeurs[len(valeurs)//2]
    lm35_buffer.append(median)
    
    if len(lm35_buffer) > 0:
        moyenne = sum(lm35_buffer) / len(lm35_buffer)
        lm35_smoothed = lm35_alpha * moyenne + (1 - lm35_alpha) * lm35_smoothed
    
    return lm35_smoothed

def read_ds18b20():
    global ds18b20_buffer, ds18b20_smoothed, ds18b20_last_valid, ds18b20_timeout
    
    if not DS18B20_ENABLED:
        return None
    
    try:
        t = sensor_oil.get_temperature()
        
        if t is None or t == 85 or t < -20 or t > 150:
            ds18b20_timeout += 1
            if ds18b20_timeout > 5:
                return ds18b20_last_valid if ds18b20_last_valid > 0 else 25.0
            return ds18b20_last_valid if ds18b20_last_valid > 0 else None
        
        ds18b20_timeout = 0
        t_corrige = t + DS18B20_OFFSET
        
        ds18b20_buffer.append(t_corrige)
        
        if len(ds18b20_buffer) > 0:
            moyenne = sum(ds18b20_buffer) / len(ds18b20_buffer)
            ds18b20_smoothed = ds18b20_alpha * moyenne + (1 - ds18b20_alpha) * ds18b20_smoothed
        
        if ds18b20_smoothed > 0:
            ds18b20_last_valid = ds18b20_smoothed
        
        return ds18b20_smoothed
        
    except Exception as e:
        print(f"Erreur DS18B20: {e}")
        ds18b20_timeout += 1
        if ds18b20_timeout > 5:
            return ds18b20_last_valid if ds18b20_last_valid > 0 else None
        return ds18b20_last_valid if ds18b20_last_valid > 0 else None

def temperature_reader():
    global last_temp_block, last_temp_oil
    global last_current, last_voltage
    global lm35_smoothed, ds18b20_smoothed, ds18b20_last_valid
    global current_smoothed, voltage_smoothed
    
    lm35_smoothed = 25.0
    ds18b20_smoothed = 25.0
    ds18b20_last_valid = 25.0
    current_smoothed = 0.0
    voltage_smoothed = 12.0
    
    while running:
        try:
            # Lecture température moteur (LM35)
            t_block = read_lm35(samples=8)
            if 0 < t_block < 120:
                last_temp_block = t_block + LM35_OFFSET
            
            # Lecture température huile (DS18B20)
            t_oil = read_ds18b20()
            if t_oil is not None and 0 < t_oil < 120:
                last_temp_oil = t_oil
            
            # Lecture courant (ACS712 sur A1)
            current = read_acs712(samples=8)
            if -20 < current < 20:
                last_current = current
            
            # Lecture tension (sur A2)
            voltage = read_voltage(samples=8)
            if 8 < voltage < 16:
                last_voltage = voltage
            
        except Exception as e:
            print(f"Erreur temperature_reader: {e}")
        
        time.sleep(0.2)

# ================== FONCTION DE DÉTECTION D'INSTABILITÉ RPM ==================
def detect_rpm_instability(rpm_history, seuil_variation=100):
    if len(rpm_history) < 10:
        return 0
    
    recent_rpm = list(rpm_history)[-10:]
    rpm_min = min(recent_rpm)
    rpm_max = max(recent_rpm)
    rpm_variation = rpm_max - rpm_min
    
    return rpm_variation if rpm_variation > seuil_variation else 0

# ================== FONCTION DE DIAGNOSTIC ==================
def analyser_diagnostic(rpm, tension, courant, vibration_rms, temp_moteur, 
                        temp_huile, vitesse, kilometres, maintenance_km, popup):
    
    current_time = time.time()
    
    popup.anomalies.clear()
    popup.entretien_alerts.clear()
    
    # ================== DIAGNOSTIC ÉLECTRIQUE ==================
    fault_data = {
        "code": "P0562",
        "niveau": "ROUGE",
        "capteur": "Tension alternateur",
        "valeur": f"{tension:.1f}V",
        "probleme": "Alternateur HS - Charge nulle",
        "action": "Vérifier faisceau, régulateur/redresseur, connexions batterie"
    }
    if check_fault_with_confirmation("P0562", (tension < 11 and rpm > 1000), current_time, popup, fault_data):
        popup.ajouter_anomalie(
            niveau="ROUGE",
            capteur="Tension alternateur",
            valeur=f"{tension:.1f}V",
            seuil=">11V",
            probleme="Alternateur HS - Charge nulle (confirmé 3s)",
            action="Vérifier faisceau, régulateur/redresseur, connexions batterie",
            code="P0562"
        )
    
    fault_data = {
        "code": "P0563",
        "niveau": "ORANGE",
        "capteur": "Tension alternateur",
        "valeur": f"{tension:.1f}V",
        "probleme": "Surtension - Régulateur défaillant",
        "action": "Remplacer régulateur/redresseur (commun sur Bajaj)"
    }
    if check_fault_with_confirmation("P0563", (tension > 15.5), current_time, popup, fault_data):
        popup.ajouter_anomalie(
            niveau="ORANGE",
            capteur="Tension alternateur",
            valeur=f"{tension:.1f}V",
            seuil="<15.5V",
            probleme="Surtension - Régulateur défaillant (confirmé 3s)",
            action="Remplacer régulateur/redresseur (commun sur Bajaj)",
            code="P0563"
        )
    
    fault_data = {
        "code": "P0560",
        "niveau": "JAUNE",
        "capteur": "Batterie",
        "valeur": f"{tension:.1f}V / {courant:.1f}A",
        "probleme": "Batterie faible ou alternateur fatigué",
        "action": "Vérifier niveau électrolyte, recharger, tester alternateur"
    }
    if check_fault_with_confirmation("P0560", (tension < 12.2 and courant < 2), current_time, popup, fault_data):
        popup.ajouter_anomalie(
            niveau="JAUNE",
            capteur="Batterie",
            valeur=f"{tension:.1f}V / {courant:.1f}A",
            seuil=">12.2V, >2A",
            probleme="Batterie faible ou alternateur fatigué (confirmé 3s)",
            action="Vérifier niveau électrolyte, recharger, tester alternateur",
            code="P0560"
        )
    
    # ================== DIAGNOSTIC THERMIQUE ==================
    fault_data = {
        "code": "P0217",
        "niveau": "ROUGE",
        "capteur": "Température moteur",
        "valeur": f"{temp_moteur:.1f}°C",
        "probleme": "SURCHAUFFE CRITIQUE - Risque serrage moteur",
        "action": "Vérifier ventilateur\nNettoyer ailettes cylindre\nVérifier richesse mélange\nVérifier calage allumage"
    }
    if check_fault_with_confirmation("P0217", (temp_moteur > 130), current_time, popup, fault_data):
        popup.ajouter_anomalie(
            niveau="ROUGE",
            capteur="Température moteur",
            valeur=f"{temp_moteur:.1f}°C",
            seuil="<130°C",
            probleme="SURCHAUFFE CRITIQUE - Risque serrage moteur (confirmé 3s)",
            action="Vérifier ventilateur (tourne libre ?)\nNettoyer ailettes cylindre\nVérifier richesse mélange (trop pauvre ?)\nVérifier calage allumage",
            code="P0217"
        )
    
    fault_data = {
        "code": "P0520",
        "niveau": "ROUGE",
        "capteur": "Température + Huile",
        "valeur": f"Moteur:{temp_moteur:.0f}°C Huile:{temp_huile:.0f}°C",
        "probleme": "Manque de lubrification sévère",
        "action": "URGENT - Arrêt immédiat\nVérifier niveau huile\nVérifier pompe à huile"
    }
    if check_fault_with_confirmation("P0520", (temp_moteur > 115 and temp_huile < 50), current_time, popup, fault_data):
        popup.ajouter_anomalie(
            niveau="ROUGE",
            capteur="Température + Huile",
            valeur=f"Moteur:{temp_moteur:.0f}°C Huile:{temp_huile:.0f}°C",
            seuil="Huile>50°C",
            probleme="Manque de lubrification sévère (confirmé 3s)",
            action="URGENT - Arrêt immédiat\nVérifier niveau huile (casse moteur possible)\nVérifier pompe à huile",
            code="P0520"
        )
    else:
        fault_data = {
            "code": "P0218",
            "niveau": "ORANGE",
            "capteur": "Température moteur",
            "valeur": f"{temp_moteur:.1f}°C",
            "probleme": "Surchauffe progressive",
            "action": "Nettoyer ailettes refroidissement\nVérifier ventilateur\nVérifier réglage richesse carburateur"
        }
        if check_fault_with_confirmation("P0218", (temp_moteur > 115), current_time, popup, fault_data):
            popup.ajouter_anomalie(
                niveau="ORANGE",
                capteur="Température moteur",
                valeur=f"{temp_moteur:.1f}°C",
                seuil="<115°C",
                probleme="Surchauffe progressive (confirmé 3s)",
                action="Nettoyer ailettes refroidissement\nVérifier fonctionnement ventilateur\nVérifier réglage richesse carburateur",
                code="P0218"
            )
    
    # ================== DIAGNOSTIC CARBURATION ==================
    fault_data = {
        "code": "P0171",
        "niveau": "ORANGE",
        "capteur": "Mélange air/essence",
        "valeur": f"T°:{temp_moteur:.0f}°C, RPM:{rpm:.0f}",
        "probleme": "Mélange trop pauvre",
        "action": "Nettoyer gicleurs carburateur\nVérifier filtre à air\nRechercher fuite admission"
    }
    if check_fault_with_confirmation("P0171", (temp_moteur > 110 and rpm < 800), current_time, popup, fault_data):
        popup.ajouter_anomalie(
            niveau="ORANGE",
            capteur="Mélange air/essence",
            valeur=f"T°:{temp_moteur:.0f}°C, RPM:{rpm:.0f}",
            seuil="T°<110°C, RPM>800",
            probleme="Mélange trop pauvre (cause principale surchauffe) (confirmé 3s)",
            action="Nettoyer gicleurs carburateur\nVérifier filtre à air\nRechercher fuite admission",
            code="P0171"
        )
    
    # ================== DIAGNOSTIC VIBRATIONS ==================
    fault_data = {
        "code": "P0300",
        "niveau": "ORANGE",
        "capteur": "Vibrations",
        "valeur": f"{vibration_rms:.3f}g",
        "probleme": "Vibrations excessives",
        "action": "Bougie défectueuse ?\nCompression inégale ?\nSilentblocs moteur usés ?\nCardan transmission déséquilibré ?"
    }
    if check_fault_with_confirmation("P0300", (vibration_rms > 0.6), current_time, popup, fault_data):
        popup.ajouter_anomalie(
            niveau="ORANGE",
            capteur="Vibrations",
            valeur=f"{vibration_rms:.3f}g",
            seuil="<0.6g",
            probleme="Vibrations excessives (risque casse silentblocs) (confirmé 3s)",
            action="Bougie défectueuse ? (allumage irrégulier)\nCompression inégale ? (soupapes)\nSilentblocs moteur usés ?\nCardan transmission déséquilibré ?",
            code="P0300"
        )
    elif check_fault_with_confirmation("P0301", (vibration_rms > 0.4), current_time, popup, fault_data):
        popup.ajouter_anomalie(
            niveau="JAUNE",
            capteur="Vibrations",
            valeur=f"{vibration_rms:.3f}g",
            seuil="<0.4g",
            probleme="Vibrations anormales (confirmé 3s)",
            action="Vérifier fixation moteur\nVérifier équilibrage roues\nSurveiller évolution",
            code="P0301"
        )
    
    # ================== DIAGNOSTIC TRANSMISSION ==================
    fault_data = {
        "code": "P0700",
        "niveau": "ORANGE",
        "capteur": "RPM/Vitesse",
        "valeur": f"{rpm} tr/min, {vitesse} km/h",
        "probleme": "Patinage d'embrayage probable",
        "action": "Vérifier jeu câble (2-3mm requis)\nTester reprise en côte\nPrévoir remplacement kit embrayage si usé"
    }
    if check_fault_with_confirmation("P0700", (rpm > 3500 and vitesse < 25), current_time, popup, fault_data):
        popup.ajouter_anomalie(
            niveau="ORANGE",
            capteur="RPM/Vitesse",
            valeur=f"{rpm} tr/min, {vitesse} km/h",
            seuil="RPM<3500 ou Vit>25",
            probleme="Patinage d'embrayage probable (câble ou disques usés) (confirmé 3s)",
            action="Vérifier jeu câble (2-3mm requis)\nTester reprise en côte\nPrévoir remplacement kit embrayage si usé",
            code="P0700"
        )
    
    # ================== DIAGNOSTIC RALENTI ==================
    if vibration_rms < 0.3:
        fault_data = {
            "code": "P0505",
            "niveau": "JAUNE",
            "capteur": "Régime ralenti",
            "valeur": f"{rpm} tr/min (vib: {vibration_rms:.2f}g)",
            "probleme": "Ralenti trop bas",
            "action": "Nettoyer vis de ralenti\nVérifier circuit ralenti carburateur\nRégler à 800-850 tr/min à chaud"
        }
        if check_fault_with_confirmation("P0505", (rpm < 750 and rpm > 0), current_time, popup, fault_data):
            popup.ajouter_anomalie(
                niveau="JAUNE",
                capteur="Régime ralenti",
                valeur=f"{rpm} tr/min (vib: {vibration_rms:.2f}g)",
                seuil=">750 tr/min et vib<0.3g",
                probleme="Ralenti trop bas (moteur stable, confirmé 3s)",
                action="Nettoyer vis de ralenti\nVérifier circuit ralenti carburateur\nRégler à 800-850 tr/min à chaud",
                code="P0505"
            )
        elif check_fault_with_confirmation("P0506", (rpm > 2000 and rpm > 0), current_time, popup, fault_data):
            popup.ajouter_anomalie(
                niveau="JAUNE",
                capteur="Régime ralenti",
                valeur=f"{rpm} tr/min (vib: {vibration_rms:.2f}g)",
                seuil="<950 tr/min et vib<0.3g",
                probleme="Ralenti trop haut (moteur stable, confirmé 3s)",
                action="Vérifier câble d'accélérateur (trop tendu ?)\nNettoyer carburateur\nRégler vis de ralenti",
                code="P0506"
            )
        
        rpm_variation = detect_rpm_instability(rpm_history)
        if rpm_variation > 0 and rpm > 0 and rpm < 950:
            fault_data = {
                "code": "P0507",
                "niveau": "JAUNE",
                "capteur": "Régime ralenti",
                "valeur": f"{rpm} tr/min (variation: ±{rpm_variation/2:.0f})",
                "probleme": "Ralenti instable - Variations anormales",
                "action": "Nettoyer circuit de ralenti (gicleur)\nVérifier l'absence de fuite d'admission\nContrôler l'état des bougies"
            }
            if check_fault_with_confirmation("P0507", rpm_variation > 0, current_time, popup, fault_data):
                popup.ajouter_anomalie(
                    niveau="JAUNE",
                    capteur="Régime ralenti",
                    valeur=f"{rpm} tr/min (variation: ±{rpm_variation/2:.0f})",
                    seuil="< ±50 tr/min",
                    probleme="Ralenti instable - Moteur traditionnel à carburateur (confirmé 3s)",
                    action="Nettoyer circuit de ralenti (gicleur encrassé probable)\nVérifier l'absence de fuite d'admission\nContrôler l'état des bougies",
                    code="P0507"
                )
    else:
        with fault_lock:
            fault_start_time["P0505"] = 0
            fault_confirmed["P0505"] = False
            fault_start_time["P0506"] = 0
            fault_confirmed["P0506"] = False
            fault_start_time["P0507"] = 0
            fault_confirmed["P0507"] = False
    
    # ================== DIAGNOSTIC TEMPÉRATURE ÉCHAPPEMENT ==================
    temp_echappement_estimee = temp_moteur * 2.2
    fault_data = {
        "code": "P0420",
        "niveau": "JAUNE",
        "capteur": "Température échappement",
        "valeur": f"{temp_echappement_estimee:.0f}°C",
        "probleme": "Température échappement excessive",
        "action": "Vérifier richesse mélange\nVérifier calage allumage"
    }
    if check_fault_with_confirmation("P0420", (temp_moteur > 120 and rpm > 2000 and temp_echappement_estimee > 280), current_time, popup, fault_data):
        popup.ajouter_anomalie(
            niveau="JAUNE",
            capteur="Température échappement (estimée)",
            valeur=f"{temp_echappement_estimee:.0f}°C",
            seuil="<280°C",
            probleme="Température échappement excessive (confirmé 3s)",
            action="Vérifier richesse mélange (trop pauvre)\nVérifier calage allumage",
            code="P0420"
        )
    
    # ================== ENTRETIEN PRÉVENTIF ==================
    if "Huile Moteur" in maintenance_km:
        km_huile = maintenance_km["Huile Moteur"]
        if km_huile > 2:
            popup.ajouter_entretien(
                composant="Vidange huile moteur",
                km_actuel=km_huile,
                km_seuil=5000,
                action="Vidanger l'huile (20W50 recommandée)"
            )
    
    if "Filtre à air" in maintenance_km:
        km_filtre = maintenance_km["Filtre à air"]
        if km_filtre > 4500:
            popup.ajouter_entretien(
                composant="Filtre à air (nettoyage)",
                km_actuel=km_filtre,
                km_seuil=5000,
                action="Nettoyer le filtre à air (cause fréquente de surchauffe)"
            )
        elif km_filtre > 0.500:
            popup.ajouter_entretien(
                composant="Filtre à air (remplacement)",
                km_actuel=km_filtre,
                km_seuil=10000,
                action="Remplacer le filtre à air"
            )
    
    if "Soupapes" in maintenance_km:
        km_soupapes = maintenance_km["Soupapes"]
        if km_soupapes > 4500:
            popup.ajouter_entretien(
                composant="Jeu aux soupapes",
                km_actuel=km_soupapes,
                km_seuil=5000,
                action="Contrôler et régler (0.10mm admission/0.15mm échappement à chaud)"
            )
    
    if "Bougies" in maintenance_km:
        km_bougies = maintenance_km["Bougies"]
        if km_bougies > 14000:
            popup.ajouter_entretien(
                composant="Bougies",
                km_actuel=km_bougies,
                km_seuil=15000,
                action="Remplacer les 2 bougies (par paire)"
            )
    
    if "Carburateur" in maintenance_km:
        km_carbu = maintenance_km["Carburateur"]
        if km_carbu > 9500:
            popup.ajouter_entretien(
                composant="Carburateur",
                km_actuel=km_carbu,
                km_seuil=10000,
                action="Nettoyage complet du carburateur et des gicleurs"
            )
    
    if "Embrayage" in maintenance_km:
        km_embrayage = maintenance_km["Embrayage"]
        if km_embrayage > 4500:
            popup.ajouter_entretien(
                composant="Embrayage (réglage)",
                km_actuel=km_embrayage,
                km_seuil=5000,
                action="Ajuster le jeu du câble d'embrayage (2-3mm)"
            )
        elif km_embrayage > 48000:
            popup.ajouter_entretien(
                composant="Embrayage (remplacement)",
                km_actuel=km_embrayage,
                km_seuil=50000,
                action="Remplacer le kit d'embrayage (disques, ressorts)"
            )
    
    if "Refroidissement" in maintenance_km:
        km_refroid = maintenance_km["Refroidissement"]
        if km_refroid > 4500:
            popup.ajouter_entretien(
                composant="Refroidissement",
                km_actuel=km_refroid,
                km_seuil=5000,
                action="Nettoyer les ailettes du cylindre et vérifier ventilateur"
            )
    
    mettre_a_jour_leds(popup.anomalies)
    
    if len(popup.anomalies) > 0 or len(popup.entretien_alerts) > 0:
        popup.show_if_needed()

def get_popup_diagnostic(root, maintenance_km):
    popup = DiagnosticPopup.get_instance(root)
    
    def mise_a_jour_diagnostic():
        try:
            vitesse_approx = kilometers * 10
            analyser_diagnostic(
                rpm=rpm,
                tension=last_voltage,
                courant=last_current,
                vibration_rms=vibration_rms,
                temp_moteur=last_temp_block,
                temp_huile=last_temp_oil,
                vitesse=vitesse_approx,
                kilometres=kilometers,
                maintenance_km=maintenance_km,
                popup=popup
            )
        except Exception as e:
            print(f"Erreur mise à jour diagnostic: {e}")
        
        if popup.popup and popup.popup.winfo_exists():
            popup.rafraichir_affichage()
        
        root.after(2000, mise_a_jour_diagnostic)
    
    mise_a_jour_diagnostic()
    return popup

# ================== THREADS CAPTEURS ==================
def hall_rpm_reader():
    global total_passages, total_tours, rpm, last_rpm_state, last_rpm_trigger
    while running:
        state = GPIO.input(HALL_RPM_PIN)
        now = time.monotonic()
        if (now - last_rpm_trigger) > 2.0:
            rpm = 0.0
        if last_rpm_state == 1 and state == 0:
            delta_time = now - last_rpm_trigger
            if delta_time >= DEBOUNCE_TIME:
                total_passages += 1
                total_tours = total_passages
                if delta_time > 0:
                    current_rpm = 60.0 / delta_time
                    rpm = rpm * (1 - SMOOTHING) + current_rpm * SMOOTHING
                last_rpm_trigger = now
        last_rpm_state = state
        time.sleep(POLL_DELAY)

def hall_km_reader():
    global total_km_passages, kilometers, last_km_state
    while running:
        state = GPIO.input(HALL_KM_PIN)
        if last_km_state == 1 and state == 0:
            total_km_passages += 1
            kilometers = total_km_passages * KM_PAR_IMPULSION
        last_km_state = state
        time.sleep(POLL_DELAY)

def vibration_reader():
    global vibration_buffer, vibration_filtered, vibration_fft, vibration_rms
    global vibration_rms_history, vibration_raw_history
    global buf_x, buf_y, buf_z_filt, buf_z_raw, ax_f, ay_f, az_f
    fft_counter = 0
    dc_offset = 0.0
    dc_alpha = 0.01
    while running:
        try:
            if MPU_ENABLED:
                ax_raw = read_accel_x() - mpu_ax_offset
                ay_raw = read_accel_y() - mpu_ay_offset
                az_raw = read_accel_z() - mpu_offset
            else:
                t = time.time()
                ax_raw = 0.2 * np.sin(t * 2) + 0.05 * np.random.randn()
                ay_raw = 0.15 * np.cos(t * 2.5) + 0.05 * np.random.randn()
                az_raw = 0.3 * np.sin(t * 3) + 0.05 * np.random.randn() + 0.1
            ax_f = alpha * ax_raw + (1 - alpha) * ax_f
            ay_f = alpha * ay_raw + (1 - alpha) * ay_f
            az_f = alpha * az_raw + (1 - alpha) * az_f
            buf_x.append(ax_f)
            buf_y.append(ay_f)
            buf_z_filt.append(az_f)
            buf_z_raw.append(az_raw)
            dc_offset = dc_alpha * az_raw + (1 - dc_alpha) * dc_offset
            az_vibration = az_raw - dc_offset
            vibration_buffer.append(az_vibration)
            vibration_raw_history.append(az_vibration)
            if len(vibration_buffer) >= 50:
                recent_data = list(vibration_buffer)[-50:]
                vibration_rms = np.sqrt(np.mean(np.array(recent_data)**2))
                vibration_rms_history.append(vibration_rms)
            fft_counter += 1
            if fft_counter >= 250:
                if len(vibration_buffer) >= N_FFT:
                    buffer_array = np.array(list(vibration_buffer)[-N_FFT:])
                    buffer_array = buffer_array - np.mean(buffer_array)
                    filtered = filtfilt(b, a, buffer_array)
                    vibration_filtered = deque(filtered, maxlen=N_FFT)
                    window = np.hanning(N_FFT)
                    fft_vals = np.fft.rfft(filtered * window)
                    fft_mag = np.abs(fft_vals) * 2 / N_FFT
                    vibration_fft = deque(fft_mag, maxlen=N_FFT//2 + 1)
                fft_counter = 0
            time.sleep(DT)
        except Exception as e:
            print(f"Erreur vibration_reader: {e}")
            time.sleep(0.1)

# ================== FICHIERS CSV ==================
time_history = deque(maxlen=120)
rpm_history = deque(maxlen=120)
temp_block_history = deque(maxlen=120)
temp_oil_history = deque(maxlen=120)
current_history = deque(maxlen=120)
voltage_history = deque(maxlen=120)

fft_frequencies = np.fft.rfftfreq(N_FFT, DT)

CSV_FOLDER = "data_csv"
os.makedirs(CSV_FOLDER, exist_ok=True)

def data_logger():
    """Enregistrement des données dans un fichier CSV toutes les 0.5s"""
    current_file = None
    file_start_time = datetime.datetime.now()
    
    while running:
        t = datetime.datetime.now()
        time_history.append(t)
        rpm_history.append(rpm)
        temp_block_history.append(last_temp_block)
        temp_oil_history.append(last_temp_oil)
        current_history.append(last_current)
        voltage_history.append(last_voltage)
        
        if current_file is None or (t - file_start_time).total_seconds() >= 48 * 3600:
            file_start_time = t
            timestamp = t.strftime("%Y%m%d_%H%M%S")
            current_file = os.path.join(CSV_FOLDER, f"data_{timestamp}.csv")
            with open(current_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["temps", "rpm", "vibration_rms", "tension", "temperature_moteur", "temperature_huile", "courant", "kilometrage"])
        
        with open(current_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([t.strftime("%Y-%m-%d %H:%M:%S"), int(rpm), round(vibration_rms, 4), 
                           round(last_voltage, 2), round(last_temp_block, 2), round(last_temp_oil, 2), 
                           round(last_current, 2), round(kilometers, 3)])
        time.sleep(0.5)

# ================== TKINTER ==================
root = tk.Tk()
root.title("Dashboard Bajaj RE 4S")
root.geometry("800x480")
root.attributes("-fullscreen", True)
root.configure(bg="#2c3e50")

def quit_fullscreen(event=None):
    root.attributes("-fullscreen", False)
    root.geometry("800x480")

def toggle_fullscreen(event=None):
    root.attributes("-fullscreen", not root.attributes("-fullscreen"))

root.bind("<Escape>", quit_fullscreen)
root.bind("<F11>", toggle_fullscreen)

style = ttk.Style()
style.theme_use("clam")
style.configure("TFrame", background="#2c3e50")
style.configure("TLabel", background="#2c3e50", foreground="white", font=("Helvetica", 15, "bold"))

container = ttk.Frame(root)
container.pack(fill=tk.BOTH, expand=True)

# ================== INTERFACE 1 ==================
interface1 = ttk.Frame(container)
interface1.place(relx=0.5, rely=0.5, anchor="center")

title_label = tk.Label(interface1, text="⚡ BAJAJ RE 4S DASHBOARD ⚡",
                       font=("Helvetica", 20, "bold"), bg="#2c3e50", fg="#e74c3c")
title_label.grid(row=0, column=0, columnspan=3, pady=(5, 15))

interface1.columnconfigure(0, weight=1, uniform="col")
interface1.columnconfigure(1, weight=1, uniform="col")
interface1.columnconfigure(2, weight=1, uniform="col")

col1 = ttk.Frame(interface1)
col2 = ttk.Frame(interface1)
col3 = ttk.Frame(interface1)
col1.grid(row=1, column=0, sticky="nsew", padx=15)
col2.grid(row=1, column=1, sticky="nsew", padx=15)
col3.grid(row=1, column=2, sticky="nsew", padx=15)

def create_wide_value_box(parent, title, color, unit="", row=0):
    frame = tk.Frame(parent, bg="#34495e", bd=3, relief=tk.RAISED, width=130, height=110)
    frame.grid(row=row, column=0, pady=8, sticky="ew")
    frame.grid_propagate(False)
    frame.pack_propagate(False)
    
    title_label = tk.Label(frame, text=title, bg="#34495e", fg=color,
                           font=("Helvetica", 11, "bold"))
    title_label.pack(pady=(12, 3))
    
    decor_line = tk.Frame(frame, bg=color, height=2, width=80)
    decor_line.pack(pady=3)
    
    value_label = tk.Label(frame, text="0" + unit, bg="#34495e", fg="white",
                           font=("Helvetica", 22, "bold"))
    value_label.pack(expand=True)
    
    return value_label

def go_interface2():
    interface1.place_forget()
    interface2.place(relx=0.5, rely=0.5, anchor="center")

def go_interface3():
    interface1.place_forget()
    interface3.place(relx=0.5, rely=0.5, anchor="center")

def go_interface4():
    interface1.place_forget()
    interface4.place(relx=0.5, rely=0.5, anchor="center")

def go_interface5():
    interface1.place_forget()
    interface5.place(relx=0.5, rely=0.5, anchor="center")

def go_interface7():
    interface1.place_forget()
    interface7.place(relx=0.5, rely=0.5, anchor="center")
    update_history_display()

# COLONNE 1
rpm_label = create_wide_value_box(col1, "RPM", "#1abc9c", row=0)
km_label = create_wide_value_box(col1, "KILOMÉTRAGE", "#f1c40f", " km", row=1)
vibration_rms_label = create_wide_value_box(col1, "VIBRATION", "#e67e22", " g", row=2)

# COLONNE 2
motor_state_label = create_wide_value_box(col2, "ÉTAT MOTEUR", "#2ecc71", row=0)
voltage_label = create_wide_value_box(col2, "TENSION", "#3498db", " V", row=1)

button_frame = tk.Frame(col2, bg="#34495e", bd=3, relief=tk.RAISED, width=130, height=110)
button_frame.grid(row=2, column=0, pady=8, sticky="ew")
button_frame.grid_propagate(False)
button_frame.pack_propagate(False)

tk.Button(button_frame, text="GRAPHES", command=go_interface2,
          font=("Helvetica", 10, "bold"), bg="#2980b9", fg="white",
          width=12, height=1, bd=2, relief=tk.RAISED).pack(pady=(10, 3))
tk.Button(button_frame, text="FICHIERS", command=go_interface3,
          font=("Helvetica", 10, "bold"), bg="#8e44ad", fg="white",
          width=12, height=1, bd=2, relief=tk.RAISED).pack(pady=3)
tk.Button(button_frame, text="HISTORIQUE", command=go_interface7,
          font=("Helvetica", 10, "bold"), bg="#f39c12", fg="white",
          width=12, height=1, bd=2, relief=tk.RAISED).pack(pady=3)

# COLONNE 3
temp_oil_label = create_wide_value_box(col3, "TEMP HUILE", "#e67e22", " °C", row=0)
temp_block_label = create_wide_value_box(col3, "TEMP MOTEUR", "#9b59b6", " °C", row=1)
current_label = create_wide_value_box(col3, "COURANT", "#f39c12", " A", row=2)

# ================== INTERFACE 2 ==================
interface2 = ttk.Frame(container)
graph_frame = ttk.Frame(interface2)
graph_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)

titles = [("RPM", "#1abc9c"), ("VIBRATION RMS", "#e67e22"), ("TEMPÉRATURES", "#9b59b6"), ("SPECTRE FFT", "#3498db")]

axes = []
canvases = []
lines = []

for r in range(2):
    for c in range(2):
        fig = Figure(figsize=(4.2, 2.0), dpi=85, facecolor="#2c3e50")
        fig.subplots_adjust(top=0.88, bottom=0.15, left=0.12, right=0.95)
        ax = fig.add_subplot(111)
        ax.set_facecolor("#1e2b38")
        ax.tick_params(colors="white", labelsize=9)
        ax.set_title(titles[r*2+c][0], color="white", fontsize=12, pad=10, fontweight="bold")
        ax.grid(True, axis='y', linestyle='--', alpha=0.4, color='white', linewidth=0.8)
        ax.grid(True, axis='x', linestyle=':', alpha=0.2, color='white', linewidth=0.5)
        
        canvas = FigureCanvasTkAgg(fig, graph_frame)
        canvas.get_tk_widget().grid(row=r, column=c, padx=12, pady=12, sticky="nsew")
        
        axes.append(ax)
        canvases.append(canvas)
        
        if r == 0 and c == 0:
            line, = ax.plot([0], [0], color=titles[0][1], linewidth=2.5)
            ax.set_ylim(0, 1000)
            ax.set_ylabel("tr/min", color="white", fontsize=8)
        elif r == 0 and c == 1:
            line, = ax.plot([0], [0], color=titles[1][1], linewidth=2.5)
            ax.set_ylim(0, 1)
            ax.set_ylabel("g", color="white", fontsize=8)
        elif r == 1 and c == 0:
            line1, = ax.plot([0], [0], color="#9b59b6", linewidth=2.5, label="Moteur")
            line2, = ax.plot([0], [0], color="#f39c12", linewidth=2.5, label="Huile")
            lines.extend([line1, line2])
            ax.legend(loc="upper right", fontsize=9, facecolor="#2c3e50", labelcolor="white", framealpha=0.8)
            ax.set_ylim(0, 120)
            ax.set_ylabel("°C", color="white", fontsize=8)
            line = line1
        else:
            line, = ax.plot([0], [0], color=titles[3][1], linewidth=2)
            ax.set_xlim(0, 200)
            ax.set_ylim(0, 0.5)
            ax.set_xlabel("Fréquence (Hz)", color="white", fontsize=9)
            ax.set_ylabel("Amplitude", color="white", fontsize=8)
        
        lines.append(line)
        
        if r == 1:
            ax.set_xlabel("Temps (x0.5s)", color="white", fontsize=9)

for i in range(2):
    graph_frame.rowconfigure(i, weight=2)
    graph_frame.columnconfigure(i, weight=2)

btns2 = ttk.Frame(interface2)
btns2.pack(pady=8)
tk.Button(btns2, text="← Retour", 
          command=lambda: [interface2.place_forget(), interface1.place(relx=0.5, rely=0.5, anchor="center")],
          bg="#c0392b", fg="white", font=("Helvetica", 11, "bold"), width=10,
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=8)
tk.Button(btns2, text="🔬 MPU 3 AXES", command=go_interface4,
          bg="#9b59b6", fg="white", font=("Helvetica", 11, "bold"), width=12,
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=8)
tk.Button(btns2, text="✖ Exit", 
          command=lambda: [setattr(sys.modules[__name__], 'running', False), GPIO.cleanup(), root.destroy()],
          bg="#34495e", fg="white", font=("Helvetica", 11, "bold"), width=10,
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=8)

# ================== INTERFACE 3 ==================
interface3 = ttk.Frame(container)
label3 = tk.Label(interface3, text="Fichiers CSV Enregistrés", 
                  font=("Helvetica", 20, "bold"), bg="#2c3e50", fg="#e67e22")
label3.pack(pady=10)

file_listbox = tk.Listbox(interface3, width=60, height=15, font=("Helvetica", 12))
file_listbox.pack(pady=10)

def refresh_file_list():
    file_listbox.delete(0, tk.END)
    files = sorted(os.listdir(CSV_FOLDER))
    for f in files:
        if f.endswith(".csv"):
            file_listbox.insert(tk.END, f)

def visualize_file():
    sel = file_listbox.curselection()
    if not sel:
        messagebox.showwarning("Attention", "Sélectionnez un fichier d'abord!")
        return
    filename = os.path.join(CSV_FOLDER, file_listbox.get(sel[0]))
    top = tk.Toplevel(root)
    top.title(filename)
    top.geometry("700x400")
    top.configure(bg="#2c3e50")
    tree = ttk.Treeview(top)
    tree.pack(fill=tk.BOTH, expand=True)
    with open(filename, newline='') as f:
        reader = csv.reader(f)
        headers = next(reader)
        tree["columns"] = headers
        tree["show"] = "headings"
        for h in headers:
            tree.heading(h, text=h)
            tree.column(h, width=100)
        for row in reader:
            tree.insert("", tk.END, values=row)

def delete_file():
    sel = file_listbox.curselection()
    if not sel:
        messagebox.showwarning("Attention", "Sélectionnez un fichier d'abord!")
        return
    filename = os.path.join(CSV_FOLDER, file_listbox.get(sel[0]))
    if messagebox.askyesno("Supprimer", f"Voulez-vous supprimer {filename}?"):
        os.remove(filename)
        refresh_file_list()

entretien_frame = ttk.Frame(interface3)
entretien_frame.pack(pady=5)
tk.Button(entretien_frame, text="ENTRETIEN", command=go_interface5,
          font=("Helvetica", 14, "bold"), bg="#27ae60", fg="white",
          width=20, height=2, bd=3, relief=tk.RAISED).pack()

separator = ttk.Separator(interface3, orient='horizontal')
separator.pack(fill='x', pady=10)

btns3 = ttk.Frame(interface3)
btns3.pack(pady=5)
tk.Button(btns3, text="Rafraîchir", command=refresh_file_list,
          bg="#2980b9", fg="white", font=("Helvetica", 12, "bold")).pack(side=tk.LEFT, padx=5)
tk.Button(btns3, text="Visualiser", command=visualize_file,
          bg="#27ae60", fg="white", font=("Helvetica", 12, "bold")).pack(side=tk.LEFT, padx=5)
tk.Button(btns3, text="Supprimer", command=delete_file,
          bg="#e74c3c", fg="white", font=("Helvetica", 12, "bold")).pack(side=tk.LEFT, padx=5)
tk.Button(btns3, text="← Retour", 
          command=lambda: [interface3.place_forget(), interface1.place(relx=0.5, rely=0.5, anchor="center")],
          bg="#c0392b", fg="white", font=("Helvetica", 12, "bold")).pack(pady=10)

# ================== INTERFACE 4 (MPU 3 AXES) ==================
interface4 = ttk.Frame(container)
mpu_frame = ttk.Frame(interface4)
mpu_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

fig1 = Figure(figsize=(4.5, 2.2), dpi=85, facecolor="#2c3e50")
fig1.subplots_adjust(top=0.85, bottom=0.15, left=0.1, right=0.95)
ax1 = fig1.add_subplot(111)
ax1.set_facecolor("#1e2b38")
ax1.set_title("Accélération 3 axes (filtrée)", color="white", fontsize=12, pad=10)
ax1.set_ylabel("g", color="white", fontsize=8)
ax1.set_xlabel("Échantillons", color="white", fontsize=8)
ax1.tick_params(colors="white", labelsize=8)
ax1.grid(True, axis='y', linestyle='--', alpha=0.3, color='white')
ax1.set_ylim(-2, 2)

line_ax, = ax1.plot([], [], color="#e74c3c", linewidth=1.5, label="X")
line_ay, = ax1.plot([], [], color="#2ecc71", linewidth=1.5, label="Y")
line_az_filt, = ax1.plot([], [], color="#3498db", linewidth=1.5, label="Z filtré")
ax1.legend(loc="upper right", fontsize=8, facecolor="#2c3e50", labelcolor="white")

canvas1 = FigureCanvasTkAgg(fig1, mpu_frame)
canvas1.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)

fig2 = Figure(figsize=(4.5, 2.2), dpi=85, facecolor="#2c3e50")
fig2.subplots_adjust(top=0.85, bottom=0.15, left=0.1, right=0.95)
ax2 = fig2.add_subplot(111)
ax2.set_facecolor("#1e2b38")
ax2.set_title("Vibration brute (axe Z)", color="white", fontsize=12, pad=10)
ax2.set_ylabel("g", color="white", fontsize=8)
ax2.set_xlabel("Échantillons", color="white", fontsize=8)
ax2.tick_params(colors="white", labelsize=8)
ax2.grid(True, axis='y', linestyle='--', alpha=0.3, color='white')
ax2.set_ylim(-2, 2)

line_z_raw, = ax2.plot([], [], color="#f39c12", linewidth=2, label="Z brut")
ax2.legend(loc="upper right", fontsize=8, facecolor="#2c3e50", labelcolor="white")

canvas2 = FigureCanvasTkAgg(fig2, mpu_frame)
canvas2.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)

mpu_values_label = tk.Label(interface4, text="X: 0.000 g | Y: 0.000 g | Z: 0.000 g",
                           font=("Helvetica", 14, "bold"), bg="#2c3e50", fg="white")
mpu_values_label.pack(pady=5)

def update_mpu_display():
    mpu_values_label.config(
        text=f"X: {ax_f:+.3f} g | Y: {ay_f:+.3f} g | Z: {az_f:+.3f} g"
    )
    x_data = list(range(len(buf_x)))
    line_ax.set_data(x_data, list(buf_x))
    line_ay.set_data(x_data, list(buf_y))
    line_az_filt.set_data(x_data, list(buf_z_filt))
    ax1.set_xlim(0, mpu_buffer_size)
    ax1.relim()
    ax1.autoscale_view()
    canvas1.draw()
    
    line_z_raw.set_data(x_data, list(buf_z_raw))
    ax2.set_xlim(0, mpu_buffer_size)
    ax2.relim()
    ax2.autoscale_view()
    canvas2.draw()
    
    interface4.after(50, update_mpu_display)

btns4 = ttk.Frame(interface4)
btns4.pack(pady=5)
tk.Button(btns4, text="← Retour", 
          command=lambda: [interface4.place_forget(), interface1.place(relx=0.5, rely=0.5, anchor="center")],
          bg="#c0392b", fg="white", font=("Helvetica", 11, "bold"), width=10,
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=8)
tk.Button(btns4, text="Graphes", command=go_interface2,
          bg="#2980b9", fg="white", font=("Helvetica", 11, "bold"), width=10,
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=8)
tk.Button(btns4, text="Exit", 
          command=lambda: [setattr(sys.modules[__name__], 'running', False), GPIO.cleanup(), root.destroy()],
          bg="#34495e", fg="white", font=("Helvetica", 11, "bold"), width=10,
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=8)

# ================== INTERFACE 5 (SUIVI ENTRETIEN) ==================
interface5 = ttk.Frame(container)

title5 = tk.Label(interface5, text="SUIVI D'ENTRETIEN BAJAJ RE 4S",
                  font=("Helvetica", 18, "bold"), bg="#2c3e50", fg="#e67e22")
title5.pack(pady=10)

table_frame = ttk.Frame(interface5)
table_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)

columns = ("composant", "operation", "frequence", "kilometrage", "statut")
tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=12)

tree.heading("composant", text="Composant")
tree.heading("operation", text="Opération")
tree.heading("frequence", text="Fréquence (km)")
tree.heading("kilometrage", text="Kilométrage (km)")
tree.heading("statut", text="Statut")

tree.column("composant", width=130, anchor="w")
tree.column("operation", width=150, anchor="w")
tree.column("frequence", width=120, anchor="center")
tree.column("kilometrage", width=100, anchor="center")
tree.column("statut", width=100, anchor="center")

donnees = [
    ("Huile Moteur", "Vidange", "5000", 0.0, "✅ OK"),
    ("Filtre à huile", "Remplacement", "5000", 0.0, "✅ OK"),
    ("Crépine", "Nettoyage", "5000", 0.0, "✅ OK"),
    ("Filtre à air", "Nettoyage", "5000", 0.0, "✅ OK"),
    ("Bougies", "Remplacement", "15000", 0.0, "✅ OK"),
    ("Carburateur", "Nettoyage", "10000", 0.0, "✅ OK"),
    ("Filtre à essence", "Remplacement", "15000", 0.0, "✅ OK"),
    ("Soupapes", "Réglage", "5000", 0.0, "✅ OK"),
    ("Chaîne distribution", "Contrôle", "10000", 0.0, "✅ OK"),
    ("Câbles", "Lubrification", "2500", 0.0, "✅ OK"),
    ("Culasse", "Décalaminage", "20000", 0.0, "✅ OK"),
    ("Reniflard", "Vérification", "10000", 0.0, "✅ OK"),
]

item_ids = {}
item_data_map = {}

style = ttk.Style()
style.configure("Treeview", 
                background="#ffffff",
                foreground="#000000",
                fieldbackground="#ffffff",
                font=("Helvetica", 10))
style.configure("Treeview.Heading",
                background="#2980b9",
                foreground="white",
                font=("Helvetica", 11, "bold"))
style.map("Treeview",
          background=[("selected", "#3498db")],
          foreground=[("selected", "white")])

tree.tag_configure('odd', background='#ffffff', foreground='#000000')
tree.tag_configure('even', background='#f2f2f2', foreground='#000000')
tree.tag_configure('warning', background='#fff3cd', foreground='#856404')
tree.tag_configure('danger', background='#f8d7da', foreground='#721c24')

for i, item_data in enumerate(donnees):
    tag = 'even' if i % 2 == 0 else 'odd'
    composant = item_data[0]
    freq = float(item_data[2])
    km_value = maintenance_km[composant]
    
    if km_value >= freq:
        statut = "À FAIRE"
        tag = 'danger'
    elif km_value >= freq * 0.8:
        statut = "Bientôt"
        tag = 'warning'
    else:
        statut = "OK"
    
    item_id = tree.insert("", tk.END, 
                         values=(composant, item_data[1], f"{freq:.0f}", f"{km_value:.0f}", statut), 
                         tags=(tag,))
    item_ids[composant] = item_id
    item_data_map[composant] = {"freq": freq, "operation": item_data[1]}

scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=tree.yview)
tree.configure(yscrollcommand=scrollbar.set)

tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

def update_maintenance_table():
    global maintenance_km
    try:
        for composant, item_id in item_ids.items():
            if composant in maintenance_km:
                km_value = maintenance_km[composant]
                freq = item_data_map[composant]["freq"]
                
                if km_value >= freq:
                    statut = "À FAIRE"
                    tag = 'danger'
                elif km_value >= freq * 0.8:
                    statut = "Bientôt"
                    tag = 'warning'
                else:
                    statut = "OK"
                    tag = 'even' if list(item_ids.keys()).index(composant) % 2 == 0 else 'odd'
                
                current_values = list(tree.item(item_id, 'values'))
                current_values[3] = f"{km_value:.0f}"
                current_values[4] = statut
                tree.item(item_id, values=current_values, tags=(tag,))
    except Exception as e:
        print(f"Erreur update maintenance: {e}")
    
    interface5.after(1000, update_maintenance_table)

def reset_component():
    selected = tree.selection()
    if not selected:
        messagebox.showwarning("Attention", "Veuillez sélectionner un composant!")
        return
    
    item = tree.item(selected[0])
    composant = item['values'][0]
    
    if messagebox.askyesno("Confirmation", f"Réinitialiser le compteur de {composant} ?"):
        maintenance_km[composant] = 0.0
        last_reset_km[composant] = kilometers
        messagebox.showinfo("Succès", f"{composant} réinitialisé!")

def reset_all():
    if messagebox.askyesno("Confirmation", "Réinitialiser TOUS les compteurs d'entretien ?"):
        for composant in maintenance_km:
            maintenance_km[composant] = 0.0
            last_reset_km[composant] = kilometers
        messagebox.showinfo("Succès", "Tous les compteurs réinitialisés!")

btns5 = ttk.Frame(interface5)
btns5.pack(pady=5)

tk.Button(btns5, text="🔄 Réinitialiser sélection", command=reset_component,
          bg="#e67e22", fg="white", font=("Helvetica", 11, "bold"),
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=5)

tk.Button(btns5, text="⚠️ Tout réinitialiser", command=reset_all,
          bg="#c0392b", fg="white", font=("Helvetica", 11, "bold"),
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=5)

tk.Button(btns5, text="← Retour", 
          command=lambda: [interface5.place_forget(), interface1.place(relx=0.5, rely=0.5, anchor="center")],
          bg="#34495e", fg="white", font=("Helvetica", 11, "bold"), width=10,
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=5)

def maintenance_counters_updater():
    global maintenance_km, last_reset_km, kilometers
    while running:
        try:
            for composant in maintenance_km:
                if composant in last_reset_km:
                    km_since_reset = max(0, kilometers - last_reset_km[composant])
                    maintenance_km[composant] = km_since_reset
            time.sleep(1)
        except Exception as e:
            print(f"Erreur maintenance updater: {e}")
            time.sleep(5)

threading.Thread(target=maintenance_counters_updater, daemon=True).start()

# ================== INTERFACE 7 (HISTORIQUE DES DÉFAUTS) ==================
def update_history_display():
    for item in history_tree.get_children():
        history_tree.delete(item)
    
    niveau = niveau_filter.get()
    search_text = search_entry.get().upper()
    
    for fault in reversed(fault_history):
        if niveau != "Tous" and fault["niveau"] != niveau:
            continue
        if search_text and search_text not in fault["code"] and search_text not in fault["capteur"].upper():
            continue
        
        history_tree.insert("", 0, values=(
            fault["timestamp"].strftime("%Y-%m-%d %H:%M:%S"),
            fault["code"],
            fault["niveau"],
            fault["capteur"],
            fault["valeur"],
            fault["probleme"][:50] + "..." if len(fault["probleme"]) > 50 else fault["probleme"],
            f"{fault['duree']:.1f}"
        ), tags=(fault["niveau"],))

def clear_history():
    if messagebox.askyesno("Confirmation", "Voulez-vous vraiment effacer tout l'historique ?"):
        fault_history.clear()
        try:
            if os.path.exists(history_file):
                os.remove(history_file)
        except:
            pass
        update_history_display()
        messagebox.showinfo("Succès", "Historique effacé!")

def show_statistics():
    if not fault_history:
        messagebox.showinfo("Statistiques", "Aucun défaut enregistré.")
        return
    
    niveaux = {"ROUGE": 0, "ORANGE": 0, "JAUNE": 0, "BLEU": 0}
    codes = {}
    
    for fault in fault_history:
        niveaux[fault["niveau"]] = niveaux.get(fault["niveau"], 0) + 1
        codes[fault["code"]] = codes.get(fault["code"], 0) + 1
    
    most_common = max(codes.items(), key=lambda x: x[1]) if codes else ("N/A", 0)
    
    stats_text = f"""
      STATISTIQUES DES DÉFAUTS
    ──────────────────────────
    
    Total défauts : {len(fault_history)}
    
    Par niveau :
      ROUGE   : {niveaux["ROUGE"]}
      ORANGE  : {niveaux["ORANGE"]}
      JAUNE   : {niveaux["JAUNE"]}
      BLEU    : {niveaux["BLEU"]}
    
    Défaut le plus fréquent :
    {most_common[0]} : {most_common[1]} fois
    
    Dernier défaut :
    {fault_history[-1]["timestamp"].strftime("%d/%m/%Y %H:%M")} - {fault_history[-1]["code"]} ({fault_history[-1]["niveau"]})
    """
    
    messagebox.showinfo("Statistiques", stats_text)

def load_history_from_file():
    try:
        if os.path.exists(history_file):
            with open(history_file, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                next(reader)
                for row in reader:
                    if len(row) >= 8:
                        fault = {
                            "timestamp": datetime.datetime.strptime(row[0], "%Y-%m-%d %H:%M:%S"),
                            "code": row[1],
                            "niveau": row[2],
                            "capteur": row[3],
                            "valeur": row[4],
                            "probleme": row[5],
                            "action": row[6],
                            "duree": float(row[7])
                        }
                        fault_history.append(fault)
    except Exception as e:
        print(f"Erreur chargement historique: {e}")

load_history_from_file()

interface7 = ttk.Frame(container)

title7 = tk.Label(interface7, text="HISTORIQUE DES DÉFAUTS",
                  font=("Helvetica", 18, "bold"), bg="#2c3e50", fg="#f39c12")
title7.pack(pady=10)

filter_frame = tk.Frame(interface7, bg="#34495e", bd=2, relief=tk.RAISED)
filter_frame.pack(fill=tk.X, padx=15, pady=5)

tk.Label(filter_frame, text="Filtrer par niveau:", bg="#34495e", fg="white",
         font=("Helvetica", 10)).pack(side=tk.LEFT, padx=5, pady=5)

niveau_filter = ttk.Combobox(filter_frame, values=["Tous", "ROUGE", "ORANGE", "JAUNE", "BLEU"], 
                             state="readonly", width=15)
niveau_filter.set("Tous")
niveau_filter.pack(side=tk.LEFT, padx=5, pady=5)

tk.Label(filter_frame, text="Rechercher code:", bg="#34495e", fg="white",
         font=("Helvetica", 10)).pack(side=tk.LEFT, padx=5, pady=5)

search_entry = tk.Entry(filter_frame, width=15, bg="#2c3e50", fg="white",
                        insertbackground="white")
search_entry.pack(side=tk.LEFT, padx=5, pady=5)

tk.Button(filter_frame, text="🔍 Filtrer", 
          command=update_history_display,
          bg="#2980b9", fg="white", font=("Helvetica", 9, "bold"),
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=5, pady=5)

tk.Button(filter_frame, text="Effacer historique", 
          command=clear_history,
          bg="#c0392b", fg="white", font=("Helvetica", 9, "bold"),
          bd=2, relief=tk.RAISED).pack(side=tk.RIGHT, padx=5, pady=5)

table_frame = ttk.Frame(interface7)
table_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)

columns = ("timestamp", "code", "niveau", "capteur", "valeur", "probleme", "duree")
history_tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=15)

history_tree.heading("timestamp", text="Date/Heure")
history_tree.heading("code", text="Code")
history_tree.heading("niveau", text="Niveau")
history_tree.heading("capteur", text="Capteur")
history_tree.heading("valeur", text="Valeur")
history_tree.heading("probleme", text="Problème")
history_tree.heading("duree", text="Durée (s)")

history_tree.column("timestamp", width=140, anchor="center")
history_tree.column("code", width=80, anchor="center")
history_tree.column("niveau", width=70, anchor="center")
history_tree.column("capteur", width=120, anchor="w")
history_tree.column("valeur", width=80, anchor="center")
history_tree.column("probleme", width=200, anchor="w")
history_tree.column("duree", width=70, anchor="center")

history_scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=history_tree.yview)
history_tree.configure(yscrollcommand=history_scrollbar.set)

history_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
history_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

history_tree.tag_configure('ROUGE', background='#ffcccc', foreground='#000000')
history_tree.tag_configure('ORANGE', background='#ffe5cc', foreground='#000000')
history_tree.tag_configure('JAUNE', background='#ffffcc', foreground='#000000')
history_tree.tag_configure('BLEU', background='#cce5ff', foreground='#000000')

btns7 = ttk.Frame(interface7)
btns7.pack(pady=5)

tk.Button(btns7, text="Actualiser", 
          command=update_history_display,
          bg="#2980b9", fg="white", font=("Helvetica", 11, "bold"),
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=5)

tk.Button(btns7, text="Statistiques", 
          command=show_statistics,
          bg="#27ae60", fg="white", font=("Helvetica", 11, "bold"),
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=5)

tk.Button(btns7, text="← Retour", 
          command=lambda: [interface7.place_forget(), interface1.place(relx=0.5, rely=0.5, anchor="center")],
          bg="#34495e", fg="white", font=("Helvetica", 11, "bold"), width=10,
          bd=2, relief=tk.RAISED).pack(side=tk.LEFT, padx=5)

# ================== UPDATE AFFICHAGE ==================
def update_display():
    rpm_label.config(text=f"{int(rpm)}")
    km_label.config(text=f"{kilometers:.3f}")
    vibration_rms_label.config(text=f"{vibration_rms:.3f}")
    motor_state_label.config(text=f"{motor_state}")
    voltage_label.config(text=f"{last_voltage:.1f}")
    current_label.config(text=f"{last_current:.2f}")
    temp_oil_label.config(text=f"{last_temp_oil:.1f}")
    temp_block_label.config(text=f"{last_temp_block:.1f}")
    root.after(500, update_display)

def update_graphs():
    if len(time_history) < 3:
        root.after(500, update_graphs)
        return
    
    axes[0].clear()
    if len(rpm_history) > 1:
        axes[0].plot(list(range(len(rpm_history))), list(rpm_history), color=titles[0][1], linewidth=2)
    axes[0].set_title(titles[0][0], color="white", fontsize=10)
    axes[0].tick_params(colors="white", labelsize=8)
    axes[0].set_facecolor("#2c3e50")
    if len(rpm_history) > 0:
        axes[0].set_ylim(0, max(1000, max(rpm_history)) * 1.2)
    
    axes[1].clear()
    if len(vibration_rms_history) > 1:
        axes[1].plot(list(range(len(vibration_rms_history))), list(vibration_rms_history), color=titles[1][1], linewidth=2)
    axes[1].set_title(titles[1][0], color="white", fontsize=10)
    axes[1].tick_params(colors="white", labelsize=8)
    axes[1].set_facecolor("#2c3e50")
    if len(vibration_rms_history) > 0:
        axes[1].set_ylim(0, max(0.1, max(vibration_rms_history)) * 1.2)
    else:
        axes[1].set_ylim(0, 0.1)
    
    axes[2].clear()
    if len(temp_block_history) > 1 and len(temp_oil_history) > 1:
        axes[2].plot(list(range(len(temp_block_history))), list(temp_block_history), color="#9b59b6", linewidth=2, label="Moteur")
        axes[2].plot(list(range(len(temp_oil_history))), list(temp_oil_history), color="#f39c12", linewidth=2, label="Huile")
    axes[2].set_title(titles[2][0], color="white", fontsize=10)
    axes[2].tick_params(colors="white", labelsize=8)
    axes[2].set_facecolor("#2c3e50")
    axes[2].legend(loc="upper right", fontsize=6, facecolor="#2c3e50", labelcolor="white")
    if len(temp_block_history) > 0:
        max_temp = max(max(temp_block_history, default=0), max(temp_oil_history, default=0))
        axes[2].set_ylim(0, max(100, max_temp * 1.2))
    
    axes[3].clear()
    if len(vibration_fft) == len(fft_frequencies) and len(vibration_fft) > 10:
        fft_data = list(vibration_fft)
        freqs = fft_frequencies
        mask = freqs >= 5
        if np.any(mask):
            freqs_plot = freqs[mask]
            fft_data_plot = np.array(fft_data)[mask]
            axes[3].plot(freqs_plot, fft_data_plot, color=titles[3][1], linewidth=1.5)
            axes[3].fill_between(freqs_plot, 0, fft_data_plot, color=titles[3][1], alpha=0.3)
            if len(fft_data_plot) > 1:
                idx_peak = np.argmax(fft_data_plot)
                peak_freq = freqs_plot[idx_peak]
                peak_amp = fft_data_plot[idx_peak]
                if peak_amp > 0.01:
                    axes[3].plot(peak_freq, peak_amp, 'ro', markersize=6, markeredgecolor='white', markeredgewidth=1)
                    axes[3].text(peak_freq + 8, peak_amp, f'{peak_freq:.1f} Hz', 
                                color='white', fontsize=7, fontweight='bold',
                                bbox=dict(boxstyle='round,pad=0.2', facecolor='red', alpha=0.9))
    
    axes[3].set_title(titles[3][0], color="white", fontsize=10)
    axes[3].set_xlim(5, 200)
    if vibration_fft and len(vibration_fft) > 0:
        fft_array = np.array(list(vibration_fft))
        fft_array_filtered = fft_array[fft_frequencies >= 5]
        if len(fft_array_filtered) > 0 and max(fft_array_filtered) > 0:
            axes[3].set_ylim(0, max(fft_array_filtered) * 1.3)
        else:
            axes[3].set_ylim(0, 0.1)
    else:
        axes[3].set_ylim(0, 0.1)
    axes[3].set_xlabel("Fréquence (Hz)", color="white", fontsize=8)
    axes[3].set_ylabel("Amplitude (g)", color="white", fontsize=8)
    axes[3].tick_params(colors="white", labelsize=8)
    axes[3].set_facecolor("#2c3e50")
    axes[3].grid(True, axis='y', linestyle='--', alpha=0.3, color='white')
    
    for canvas in canvases:
        canvas.draw()
    root.after(500, update_graphs)

# ================== LANCEMENT THREADS ==================
threading.Thread(target=hall_rpm_reader, daemon=True).start()
threading.Thread(target=hall_km_reader, daemon=True).start()
threading.Thread(target=temperature_reader, daemon=True).start()
threading.Thread(target=vibration_reader, daemon=True).start()
threading.Thread(target=data_logger, daemon=True).start()

refresh_file_list()

diagnostic_popup = get_popup_diagnostic(root, maintenance_km)

root.after(500, update_display)
root.after(500, update_graphs)
root.after(500, update_mpu_display)
root.after(1000, update_maintenance_table)
root.after(500, update_history_display)

def on_closing():
    global running, leds_thread_running
    running = False
    leds_thread_running = False
    time.sleep(0.5)
    if bus:
        bus.close()
    GPIO.cleanup()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
