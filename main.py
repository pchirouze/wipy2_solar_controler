#---------------------------------------------------------#
#        GESTION SOLAIRE SUR WIPY  2        #
#---------------------------------------------------------#
"""- Lecture de 5 capteurs de température OneWire DS18x20
        Raccorder un pull de 4.7k entre la pin data et le 3.3V
        Tcap: Température capteur solaire
        Tcub: Température bas de cuve
        Tcuh: Température haut de cuve
        Taec: Température arrivée échangeur
        Trec: Température retour échangeur """

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
data_levels={'SPh':50.00, 'SPn':4.50, 'SPb': -12.00, 'Qs':680,  'N':5}  
VOL_PULSE = 0.00444 # Litre par pulse du débimetre
ON=const(1)
OFF=const(0)
NBTHERMO = 5 
# Table d'affectations des capteurs suivants leur ID (ds.roms)
AFFECT_TH = ['Tcap', 'Tcub', 'Trec', 'Tcuh', 'Taec']  # Ajuster l'ordre des items pour attribuer le bon ID des capteurs
#------------- Variables globales -----------------
mes_send=False

#MQTT_server="iot.eclipse.org"
MQTT_server="m23.cloudmqtt.com"
MQTT_port = 1883
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
        if pycom.nvs_get('power') is None:
            pycom.nvs_set('power',0)
            self.ew = 0.0
        else:
            self.ew = float(pycom.nvs_get('power')/100)
        self.pw=0.0
        self.start_t=time.ticks_ms()
        self.pompe(OFF)
        self.dT = 0
        self.deb = 0
        self.pulse_count = 0
        self.t_cycle = 0
        self.t_debut = time.ticks_ms()

    def run(self, temps, dateh):
        """ Doc String """
        self.debit = self._flowmeter(self.pompe.value(), counter)
        temps['Q'] = self.debit
        # print(self.debit)
        if e_cde_manu.value() == 1:         # En manuel = 0
            self.dT=temps['Tcap'] - temps['Tcub']
            if self.dT > self.secu_th :
        # Sécurité choc thermique dT > 50.0°C(demarrage pompe par impulsion)            
                self.cpt+=1
                if self.cpt==1:
                    self.pompe(ON)
                    self._calc_puissance(temps['Taec'],  temps['Trec'], self.debit)
                elif self.cpt < self.N:
                    self.pompe(OFF)
                    self.pw =0
                elif self.cpt >= self.N: 
                    self.cpt=0
            # Test pour marche normale
            elif self.dT > self.seuil_start:
                self.pompe(ON)
                self._calc_puissance(temps['Taec'],  temps['Trec'], self.debit)
            # Test pour securité capteur solaire fort gel
            elif temps['Tcap'] < self.seuil_tb:
                self.pompe(ON)
                self.pw = 0       
            # Securite surchauffe ballon (refroidi le stock dans les panneaux la nuit si T cuve > 80 entre 1h00 et 8h00 du matin)
            elif temps['Tcuh'] > 80 and  dateh[3] > 0 and dateh[3] < 8:
                self.pompe(ON)
                self.pw=0
            else :
                self.pompe(OFF)
                self.pw=0
            return self.pw,  self.ew ,  self.pompe()
        else:
            print('Cde manuelle circulateur active')
            self.pompe(ON)
            return self.pw,  self.ew,  self.pompe()

    def _calc_puissance(self,  ta, td, flow):
        if flow > 0:
            self.pw=((ta-td)*1.16*flow/1000)
            if self.pw > 0:
                self.ew +=  self.pw   *  (time.ticks_diff(self.start_t,  time.ticks_ms()))/3600000
            else:
                self.pw = 0 
            pycom.nvs_set('power',int(self.ew*100))
        else:
            self.pw=0 
        self.start_t=time.ticks_ms()
        return

# Fonction débimetre par comptage impulsion sur une entrée logique (option)       
    def   _flowmeter(self, pmp, cnt=None):
        ''' Debimetre a impulsion   ''' 
        global counter
        if cnt is None:
            if pmp == ON :
                self.deb=data_levels['Qs']
            else :
                self.deb = 0
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
# Led heartbeat 
pycom.heartbeat(False)
flag=False
inp_count = Pin(P_FLOWMETER,mode=Pin.IN)
inp_count.callback(Pin.IRQ_RISING, PinPulsecounter)
lock = _thread.allocate_lock()
temp={}

ds=onewire.DS18X20(onewire.OneWire(Pin(P_BUS_OW)))
reg=Solar_controller(Pin(CDE_POMPE), data_levels)
e_cde_manu = Pin(P_CIRC_MANU,mode=Pin.IN)

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
            thermometres[AFFECT_TH[i]] = int.from_bytes(dev[i],'little') 
        f=open('thermo.dat','w')
        f.write(json.dumps(thermometres))
    else:
        print('Seulement ', len(dev), 'thermometres detectés sur ', NBTHERMO )
        pycom.rgbled(0xff0000)          #LED en rouge defaut
        machine.reset()
finally:
    f.close()

all_th = False
all_t_read = 0
alive = False
on_time = False
temp_tm1 = {}
# Init watchdog
if WATCHDOG:
    wdg =machine.WDT(timeout = 15000)

#====================
# Boucle main
#====================
while True:
    t=time.localtime()
#Lecture thermometres OneWire (Raffraichi un thermometre par boucle)
    for key in thermometres:
        pycom.rgbled(0x000800)
        idt= thermometres[key].to_bytes(8,'little')        
        ds.start_convertion(idt)
        time.sleep(0.7)
        t_lue = ds.read_temp_async(idt)
        if t_lue >=4095 : # ds18 debranché valeur = 4095.xx
            print('Defaut capteur ',key )
            pycom.rgbled(0xff0000)
            temp[key]=0.0
        else:
            if len(temp_tm1) != NBTHERMO :
                temp[key] = t_lue
                temp_tm1[key] = t_lue
            elif abs(temp_tm1[key] - t_lue) < 1.0 :
                temp[key] = t_lue   # Filtre une valeur 
            temp_tm1[key] = t_lue
        if all_t_read == NBTHERMO:
            all_th = True
        else:
            all_t_read +=1
        
        if all_th is True:
            if DEBUG : print('Heure courante :',t[3] )
# Gestion solaire
            temp['PWR'], temp['ENR'], temp['PMP'] = reg.run(temp, t)
# Cumul journalier de la puissance collecté a 0h01 heure et date reseau Ok
            print('{} {} {} {}:{}:{}  {}'.format(t[2], t[1], t[0], t[3], t[4], t[5], temp))
            if t[0] != 1970 and t[3] == 0 and t[4] == 1 :
                if flag is False :
                    try:
                        f=open('energie.csv', 'r')
                        a=float(f.read())
                        f.close()
                    except:
                        a = 0.0
                    finally:
                        f=open('energie.csv', 'a+')
                        a += temp['ENR']
                        record = str(t[0]) + ',' + str(t[7]) + ',' + str(a) + '\n'
                        f.write(record)
                        f.close()
                        pycom.nvs_set('power',int(0))
                        reg.ew = 0.0
                    flag=True
            else:
                flag=False
                
# Etape 0 : Initialisation connexion WIFI
            if etape_wifi == 0:
                lswifi=[]
                wlan=WLAN(mode=WLAN.STA)
                try:
                    lswifi=wlan.scan()
                except:
                    print('Pas de wifi')
                for r in lswifi:
# freebox et signal > -80 dB                
                    if r[0] == SSID and r[4] > -87 :      
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
                    client = MQTTClient("solaire",MQTT_server, port = MQTT_port,  keepalive = 100)
                    etape_wifi = 2

# Etape 2 : Connexion MQTT
            if etape_wifi == 2:
                try:
                    client.connect(clean_session=True)
                    client.set_callback(incoming_mess)
                    client.subscribe('/regsol/send', qos= 0)
                    print('Connecte au serveur MQTT : ',  MQTT_server)
                    etape_wifi = 3
                except:
                    print('MQTT connexion erreur')
                if not wlan.isconnected():
                    etape_wifi = 0

# Etape 3 : Connexion WIFI et broker MQTT Ok, traite message publié et souscrit  
            if etape_wifi == 3:
                if not wlan.isconnected():
                    print('Deconnexion WIFI')
                    etape_wifi = 0
                try:
                    client.check_msg()
                except:
                    client.disconnect()
                    print('MQTT check message entrant erreur')
                    etape_wifi = 1
                if mes_send is True:
                    if DEBUG: print('Message MQTT: ', json.dumps(temp))
                    try:
                        client.publish('/regsol/mesur',json.dumps(temp))
                    except:
                        client.disconnect()
                        print('MQTT publication erreur')
                        etape_wifi = 1
                alive = not alive       # Toggle
                client.publish('/regsol/alive', bytes(str(alive), "utf8"))
        
                if machine.reset_cause() == machine.WDT_RESET and not on_time:
                    print('Reset par watchdog\n\n')
                    txtlog = 'Reset watchdog: ' + str(time.localtime()) + '\n'
                    f=open('log.txt','a+')
                    f.write(txtlog)   
                    f.close()
                    on_time =True
        pycom.rgbled(0x000000)
        time.sleep(0.7)
        if DEBUG: print('EtapeWifi : ', etape_wifi)
        if WATCHDOG: wdg.feed()
client.disconnect()

