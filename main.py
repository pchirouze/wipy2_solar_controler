#---------------------------------------------------------#
#        GESTION SOLAIRE SUR WIPY  2        #
#---------------------------------------------------------#
"""- Lecture de 5 capteurs de température OneWire DS18x20
        Raccorder un pull de 4.7k entre la pin data et le 3.3V, si vdd et gnd reste a 0
        alimentation capteur sur GND et VDD 3.3V
        sinon 'GPx' pour pin gnd et Vdd > alimentation  par ports de sorties        
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

# Identifiant interne capteur a changé si changement de capteur(5 derniers digits)
THERMOMETRES = {27702:'T1', 28196:'T2', 29859:'T3', 27423:'T4', 23570:'T5'}
# WIFI ID et PSWD
SSID='freebox_PC'
PWID='parapente'
# N° port commande circulateur solaire
CDE_POMPE = 'P19'    
# N° port data bus OneWire
p_bus_ow = 'P22' 
# N° port marche manuelle circulateur
p_circ_manu = 'P4'
#  Liste parametres Dt sécurité choc th(SPh), Dt seuil marche pompe(SPn),Tcpt sécurité gel(SPb), debit(Qs) , N
data_levels={'SPh':50.00, 'SPn':4.50, 'SPb': -12.00, 'Qs':720,  'N':5}  

mes_send=False
cumul=0.0
MQTT_server="iot.eclipse.org"
wifi = False
mqtt_ok = False

ON=const(1)
OFF=const(0)

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

    def run(self, temps, debit, dateh):
        if e_cde_manu.value() == 1:
            self.dT=temps['T1'] - temps['T2']
            if self.dT > self.secu_th :
            # Sécurité choc thermique dT > 50.0°C(demarrage pompe par impulsion)            
                self.cpt+=1
                if self.cpt==1:
                    self.pompe(ON)
                    self.pw,  self.ew = self._calc_puissance(temps['T4'],  temps['T5'], debit)
                elif self.cpt < self.N:
                    self.pompe(OFF)
                    self.pw =0
                elif self.cpt >= self.N: 
                    self.cpt=0
            # Test pour marche normale
            elif self.dT > self.seuil_start:
                self.pompe(ON)
                self.pw,  self.ew = self._calc_puissance(temps['T4'],  temps['T5'], debit)
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

    def _calc_puissance(self,  ta, td, debit):
        global cumul
#        print(ta, td, debit, ew)
        if debit > 0:
            pw=((ta-td)*1.16*debit)
#            print(debit) 
            cumul +=  pw   *  (time.ticks_diff(self.start_t,  time.ticks_ms()))/3600000
        else:
           pw=0 
        self.start_t=time.ticks_ms()
        return pw, cumul

# Fonction débimetre par comptage impulsion sur une entrée logique (option)       
def   debimetre(pin=None, ve=0):
    ''' Debimetre a impulsion   ''' 
    if pin is None:
        debit=data_levels['Qs']
        return debit
    else:
        pass

  # Callbacks connexion MQTT protocole to free broker 
def incoming_mess(topic, msg):
    ''' Callback sur reception message '''
    global mes_send, data_levels
    if topic is not None : 
        print(topic.decode(), msg.decode())
        if topic==b'/regsol/cde' and  msg == b'start':
            mes_send =True
            return
        if topic== b'/regsol/cde' and msg == b'stop':
            mes_send =False
            return
        if topic==b'/regsol/data':
            data_levels = msg.decode().split
            f=open('param.dat', 'w')
            f.write(json.dumps(data_levels))
            f.close()

#  Gestion watch dog par callback timer
def wdt_callback(alarm):
    ''' Fonction reset sur watchdog '''
    import machine
    print("\n\nReset par WatchDog\n\n")
    time.sleep(0.5)
    machine.reset()


#============= Debut Programme ====================        
ds=onewire.DS18X20(onewire.OneWire(Pin(p_bus_ow)))
reg=Solar_controller(Pin(CDE_POMPE), data_levels)
e_cde_manu = Pin(p_circ_manu,mode=Pin.IN)

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
# LED 3 couleurs carte Wipy
pycom.heartbeat(False)
#====================
# Boucle main
#====================
flag=False
temp_m1={}
while True:
# Init Timer pour watchdog
    watchdog=Timer.Alarm(wdt_callback, 20, periodic=False)
    pycom.rgbled(0x000800)
    t=time.localtime()
    temp={}
#Lecture thermometres OneWire
    tmp =  ds.read_temps()
    dev = ds.roms    
#    print(dev)
    if len(dev) == len(THERMOMETRES):
        for i in range(len(dev)):
            tx=THERMOMETRES[int.from_bytes(dev[i][2:4],'little')]
            temp[tx]=tmp[i]/100
        if temp_m1=={}: temp_m1=temp.copy()
    else:
        temp['T1']=0
        temp['T2']=0 
        temp['T3']=0
        temp['T4']=0
        temp['T5']=0    
        print('Defaut lecture thermometres')   
        temp_m1 = temp.copy()            
# Enleve erreur de mesure eventuelle
    for key in temp.keys():
        ecart= temp[key] - temp_m1[key]
        if abs(ecart) > 10.0 :
            print('Mesure {} errone {} C'.format(key, temp[key]))
            temp[key]=temp_m1[key]
    temp_m1=temp.copy()

# Gestion solaire
    a,  b, circul = reg.run(temp, debimetre(), t )
    temp['PWR']= a
    temp['ENR']= b
    if circul==1: 
        temp['PMP']= 0
    else:
        temp['PMP']= 1
# cumul journalier de la puissance collecté a 0h01 heure
    print('{} {} {} {}:{}:{}  {}'.format(t[2], t[1], t[0], t[3], t[4], t[5], temp))
    if t[3]==0 and t[4]==1 :
        if flag==False :
            try:
                f=open('energie.dat', 'r')
                a=float(f.read())
            except:
                f=open('energie.dat', 'w')
                a=0
                f.write(str(a))
            finally:
                f.close()
            a+=cumul   
# Remise a 0 cumul annuel au 1/1 a 0h01
            if t[1]==1 and t[2]==1:
                a=0
            f=open('energie.dat', 'w')
            f.write(str(a))
            f.close()
            cumul=0
            flag=True
    else:
        flag=False
        
#Gestion protocole Telnet, FTP, MQTT en WiFI
#    print (wifi, mqtt_ok)    
    if wifi==False: 
        wlan=WLAN(mode=WLAN.STA)
        lswifi=wlan.scan()
        if lswifi==None: lswifi=[] # Bug scan pass
        for r in lswifi:
    # freebox et signal > -80 dB                
            if r[0] == SSID and r[4] > -80 :      
#                wlan.ifconfig(config=('192.168.0.30', '255.255.255.0', '192.168.0.254', '212.27.40.240'))        
                if not wlan.isconnected():
                    wlan.ifconfig(config='dhcp')
                    wlan.connect(SSID, auth=(WLAN.WPA2, PWID), timeout=50)
                    time.sleep(2)       # Time sleep indispensable
                wifi=True
                rtc=RTC()
                rtc.ntp_sync("pool.ntp.org")
                time.timezone(3600)
                mqtt_ok=False
    else:
    # Creation et initialisation protocole MQTT   
        if mqtt_ok==False:
            print('Connecte WIFI : ',  wlan.ifconfig())
            client =MQTTClient("pchirouze",MQTT_server, port = 1883,  keepalive = 100)
            try:
                client.connect(clean_session=True)
                client.set_callback(incoming_mess)
                client.subscribe('/regsol/cde', qos= 0)
                #print('Connecte WIFI : ',  wlan.ifconfig())
                print('Connecte au serveur MQTT : ',  MQTT_server)
                mqtt_ok = True
            except:
                print('MQTT connexion erreur')
                #client.disconnect()
                mqtt_ok=False
                #machine.reset()
        else:
            try:
                client.check_msg()
            except:
                mqtt_ok=False
                client.disconnect()
                print('MQTT check message entrant erreur')
                machine.reset()
            if mes_send==True:
                try:
                    client.publish('/regsol/mesur',json.dumps(temp))
                except:
                    mqtt_ok=False
                    client.disconnect()
                    print('MQTT publication erreur')
                    machine.reset()
            else:
                client.ping()       # Keep alive command

    time.sleep(1)
# Tue l'instance pour relance nouvelle instance Timer watchdog
    watchdog.__del__()
client.disconnect()

