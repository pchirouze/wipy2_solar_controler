#---------------------------------------------------------#
#        GESTION SOLAIRE SUR WIPY  2        #
#---------------------------------------------------------#
"""- Lecture de 5 capteurs de température OneWire DS18x20
        Raccorder un pull de 4.7k entre la pin data et le 3.3V
        T1: Température capteur solaire
        T2: Température bas de cuve
        T3: Température haut de cuve
        T4: Température arrivée échangeur
        T5: Température retour échangeur """

import time, json,  onewire,  machine
from machine import  Pin, RTC, Timer
from umqtt import  MQTTClient
from network import  WLAN
import pycom
import _thread

# ----------  Configuration ----------
WATCHDOG = True
#WATCHDOG = False
DEBUG = True
#DEBUG = False
# ----------- Constantes -------------
# WIFI ID et PSWD
SSID='freebox_PC'
PWID='parapente'
# N° port commande circulateur solaire
CDE_POMPE = 'P19'    
# N° port data bus OneWire
P_BUS_OW = 'P22' 
# N° port marche manuelle circulateur
P_CIRC_MANU = 'P4'
# Entée impulsion débimetre
P_FLOWMETER = 'P21'
#  Liste parametres Dt sécurité choc th(SPh), Dt seuil marche pompe(SPn),Tcpt sécurité gel(SPb), debit(Qs) , N
data_levels={'SPh':50.00, 'SPn':4.50, 'SPb': -12.00, 'Qs':720,  'N':5}  
VOL_PULSE = 0.00444 # Litre par pulse du débimetre
ON=const(1)
OFF=const(0)
NBTHERMO = 5 

#------------- Variables globales -----------------
mes_send=False
cumul=0.0
MQTT_server="iot.eclipse.org"
etape_wifi = 0
# Pas de débimetre : counter = None
counter = None
# Débimetre : counter = 0
#counter = 0

class   Solar_controller():
    """   ---- Classe regulation solaire ----
- Pilotage relais statique commande circulateur solaire (collecteur ouvert : etat 0 >> active relais)
        Securité choc thermique panneau solaire
        Sécurité gel grand froid (<-12°C) 
        Alarme température haute capteur
        Securité alarme température stock 
- Communication WIFI et controle a distance protocole MQTT
        Température temps réel
        Puissance 
        Energie cumulé
        Débit ou controle rotation pompe(option)
- Calcul energie récupéré
        Puissance instantané
        Energie cumulé par jour
    """ 

    def __init__(self, pin, data):
        self.pompe=pin
        self.pompe.init(pin.OUT)
        self.secu_th=float(data['SPh'])
        self.seuil_start=float(data['SPn'])
        self.seuil_tb=float(data['SPb'])
        self.debit=int(data['Qs'])
        self.N=int(data['N'])
        self.cpt=0
        self.pw=0
        self.ew=0
        self.start_t=time.ticks_ms()
        self.pompe(OFF)
        self.dT = 0
        self.deb = 0
        self.pulse_count = 0
        self.t_cycle = 0
        self.t_debut = time.ticks_ms()

    def run(self, temps, dateh):
        """ Doc String """
        self.debit = self._flowmeter(counter)
        temps['Q'] = self.debit
        # print(self.debit)
        if e_cde_manu.value() == 1:         # En manuel = 0
            self.dT=temps['T1'] - temps['T2']
            if self.dT > self.secu_th :
        # Sécurité choc thermique dT > 50.0°C(demarrage pompe par impulsion)            
                self.cpt+=1
                if self.cpt==1:
                    self.pompe(ON)
                    self.pw,  self.ew = self._calc_puissance(temps['T4'],  temps['T5'], self.debit)
                elif self.cpt < self.N:
                    self.pompe(OFF)
                    self.pw =0
                elif self.cpt >= self.N: 
                    self.cpt=0
            # Test pour marche normale
            elif self.dT > self.seuil_start:
                self.pompe(ON)
                self.pw,  self.ew = self._calc_puissance(temps['T4'],  temps['T5'], self.debit)
            # Test pour securité capteur solaire fort gel
            elif temps['T1'] < self.seuil_tb:
                self.pompe(ON)
                self.pw = 0       
            # Securite surchauffe ballon (refroidi le stock dans les panneaux la nuit si T cuve > 75 entre 1h00 et 8h00 du matin)
            elif temps['T3'] > 75 and  dateh[3] > 0 and dateh[3] < 8:
                self.pompe(ON)
                self.pw=0
            else :
                self.pompe(OFF)
                self.pw=0
            return self.pw,  self.ew ,  self.pompe()
        else:
            print('Cde manuelle circulateur active')
            self.pompe(ON)
            return self.pw,  self.ew ,  self.pompe()

    def _calc_puissance(self,  ta, td, flow):
        global cumul
#        print(ta, td, debit, ew)
        if flow > 0:
            pw=((ta-td)*1.16*flow)
#            print(flow) 
            cumul +=  pw   *  (time.ticks_diff(self.start_t,  time.ticks_ms()))/3600000
        else:
            pw=0 
        self.start_t=time.ticks_ms()
        return pw, cumul

# Fonction débimetre par comptage impulsion sur une entrée logique (option)       
    def   _flowmeter(self, cnt=None):
        ''' Debimetre a impulsion   ''' 
        global counter
        if cnt is None:
            self.deb=data_levels['Qs']
            return self.deb
        else:
            lock.acquire()
            self.pulse_count = counter
            counter=0
            lock.release()
            self.t_cycle = time.ticks_diff(self.t_debut, time.ticks_ms())
            self.t_debut = time.ticks_ms()
            self.deb = int(self.pulse_count * 3600000/ self.t_cycle * VOL_PULSE)
            return self.deb

# Callbacks connexion MQTT protocole to free broker 
def incoming_mess(topic, msg):
    ''' Callback sur reception message '''
    global mes_send, data_levels

    if topic is not None : 
        if DEBUG : print('Subscribed: ', topic.decode(), msg.decode())
        if topic==b'/regsol/send' and  msg == b'start':
            mes_send =True
            return
        if topic== b'/regsol/send' and msg == b'stop':
            mes_send =False
            return
        if topic==b'/regsol/data':
            data_levels = msg.decode().split
            f1=open('param.dat', 'w')
            f1.write(json.dumps(data_levels))
            f1.close()
#        if topic==b'/regchauf/mesur':   #-------------- Essai Ok --------
#            print(msg.decode().split)

#  Gestion watch dog par callback timer
def wdt_callback(alarm):
    ''' Fonction reset sur watchdog '''
    # import machine
    print(alarm)
    print("\n\nReset par WatchDog\n\n")
    time.sleep(0.5)
    machine.reset()

# Callback function for each rising edge pulse 
def PinPulsecounter(arg):
    ''' Debimétre a detecteur effet hall (fréquence f(Q)) '''
    global counter

    if counter is not None:
        if lock.locked() is not True:
            lock.acquire()
            counter +=1
            lock.release()
    
#
#============= Debut Programme ====================        
#
flag=False
inp_count = Pin(P_FLOWMETER,mode=Pin.IN)
inp_count.callback(Pin.IRQ_RISING, PinPulsecounter)
lock = _thread.allocate_lock()
temp={}

ds=onewire.DS18X20(onewire.OneWire(Pin(P_BUS_OW)))
reg=Solar_controller(Pin(CDE_POMPE), data_levels)
e_cde_manu = Pin(P_CIRC_MANU,mode=Pin.IN)
# Led heartbeat 
pycom.heartbeat(False)

# Lecture fichier parametres
try:
    f=open('param.dat', 'r')
    data_levels=json.loads(f.read())
except:
    print('Erreur lecture fichier parametres')
finally:
    #Cree fichier parametres par defaut
    f=open('param.dat','w')
    f.write(json.dumps(data_levels))
    f.close()

# Lecture fichiers affections thermometres
try:
    f=open('thermo.dat', 'r')
    data=f.read()
    thermometres = json.loads(data)
except:
    print('Erreur lecture fichier thermometres')
    thermometres = {}
    dev = ds.roms
    if len(dev) == NBTHERMO:
# Affectation des thermometres et enregistrement (converti ID en int: bug bytearray en json)
        for i in range(len(dev)):
            thermometres['T'+ chr(0x31+i)] = int.from_bytes(dev[i],'little') 
        f=open('thermo.dat','w')
        f.write(json.dumps(thermometres))
    else:
        print('Seulement ', len(dev), 'thermometres detectés sur ', NBTHERMO )
        machine.reset()
finally:
    f.close()

#====================
# Boucle main
#====================
all_th = False
all_t_read = 0

while True:
# Init watchdog
#    watchdog=Timer.Alarm(wdt_callback, 20, periodic=False)
    if WATCHDOG:
        wdg =machine.WDT(timeout = 15000)
    pycom.rgbled(0x000800)
    t=time.localtime()

#Lecture thermometres OneWire (Raffraichi un thermometre par boucle)
    for key in thermometres:
        idt= thermometres[key].to_bytes(8,'little')        
        ds.start_convertion(idt)
        time.sleep(0.7)
        t_lue = ds.read_temp_async(idt)/100.0
        if t_lue >=4095 : # ds18 debranché valeur = 4095.xx
            print('Defaut capteur ',key )
            temp[key]=0.0
        else:
            temp[key] = t_lue
        if all_t_read == NBTHERMO:
            all_th = True
        else:
            all_t_read +=1
        
        if all_th is True:
# Gestion solaire
            temp['PWR'], temp['ENR'], temp['PMP'] = reg.run(temp, t)
# cumul journalier de la puissance collecté a 0h01 heure
            print('{} {} {} {}:{}:{}  {}'.format(t[2], t[1], t[0], t[3], t[4], t[5], temp))
            if t[3]==0 and t[4]==1 :
                if flag is False :
                    try:
                        f=open('energie.dat', 'r')
                        a=float(f.read())
                    except:
                        f=open('energie.dat', 'w')
                        a=0.0
                        f.write(str(a))
                    finally:
                        f.close()
                    a+=cumul   
# Remise a 0 cumul annuel au 1/1 a 0h01
                    if t[1]==1 and t[2]==1:
                        a=0.0
                    f=open('energie.dat', 'w')
                    f.write(str(a))
                    f.close()
                    cumul=0.0
                    flag=True
            else:
                flag=False
                
#Etape 0 : Initialisation connexion WIFI
            if etape_wifi == 0:
                lswifi=[]
                wlan=WLAN(mode=WLAN.STA)
                try:
                    lswifi=wlan.scan()
                except:
                    print('Pas de wifi')
                for r in lswifi:
# freebox et signal > -80 dB                
                    if r[0] == SSID and r[4] > -80 :      
# Initialisation connexion WIFI
                        wlan.ifconfig(config='dhcp')
                        wlan.connect(SSID, auth=(WLAN.WPA2, PWID), timeout=50)
                        time.sleep(2)       # Time sleep indispensable
                        etape_wifi=1

# Etape 1 : Attente connexion WIFI etablie
            if etape_wifi == 1:
                if wlan.isconnected():
                    print('Connecte WIFI : ',  wlan.ifconfig())
# Lecture fournisseur date/heure et init timer, creation client MQTT
                    rtc=RTC()
                    rtc.ntp_sync("pool.ntp.org")
                    time.timezone(3600)
                    client = MQTTClient("solaire",MQTT_server, port = 1883,  keepalive = 100)
# Connexion MQTT
                    try:
                        client.connect(clean_session=True)
                        client.set_callback(incoming_mess)
                        client.subscribe('/regsol/send', qos= 0)
# client.subscribe('/regchauf/mesur', qos=0)    #------ Essai Ok -------
                        print('Connecte au serveur MQTT : ',  MQTT_server)
                        etape_wifi = 2
                    except:
                        print('MQTT connexion erreur')

# Etape 2 : Connexion WIFI et broker MQTT Ok, traite message publié et souscrit  
            if etape_wifi == 2:
                if not wlan.isconnected():
                    etape_wifi = 0
                try:
                    client.check_msg()
                except:
                    client.disconnect()
                    print('MQTT check message entrant erreur')
                if mes_send is True:
                    if DEBUG: print('Message MQTT: ', json.dumps(temp))
                    try:
                        client.publish('/regsol/mesur',json.dumps(temp))
                    except:
                        client.disconnect()
                        print('MQTT publication erreur')
                else:
                    client.ping()       # Keep alive command
        #time.sleep(0.5)
        if DEBUG: print('EtapeWifi : ', etape_wifi)
        if WATCHDOG: wdg.feed()
client.disconnect()

