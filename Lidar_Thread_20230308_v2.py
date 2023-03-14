from rplidar import RPLidar
import numpy as np
import time
import matplotlib.pyplot as plt
from rpi_hardware_pwm import HardwarePWM
import threading

###################################################
#Intialisation des moteurs
##################################################

stop_prop = 7.5
point_mort_prop = 0.5
vitesse_max_m_s = 8

angle_gauche_max = 6.25
angle_droite_max = 8.75
angle_min = -1.25
angle_max = 1.25
angle_centre= 7.5
angle_degre=0

pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
pwm_prop.start(stop_prop)
pwm_dir.start(angle_centre)
print("PWM activées")

#connexion et démarrage du lidar
lidar = RPLidar("/dev/ttyUSB0",baudrate=115200)
lidar.connect()
print (lidar.get_info())
lidar.start_motor()
time.sleep(2)
acqui_tableau_lidar_mm = [0]*360 #création d'un tableau de 360 zéros
tableau_lidar_mm = [0]*360
drapeau_nouveau_scan = False
Run_Lidar = False


def lidar_scan() :
    global drapeau_nouveau_scan
    global acqui_tableau_lidar_mm
    global Run_Lidar
    global lidar
    print ("tâche lidar_scan démarrée")
    scan_avant_en_cours = False
    angle_old = 0
    while Run_Lidar == True :
        try : 
            for _,_,angle_lidar,distance in lidar.iter_measures(scan_type='express'): #Le tableau se remplissant continuement, la boucle est infinie
                angle = min(359,max(0,359-int(angle_lidar)))
                if (angle >= 260) or (angle <= 100) :
                    acqui_tableau_lidar_mm[angle]=distance #[1] : angle et[2] : distance
                #print("dernier angle mesure " + str(dernier_angle_mesure) + "time : " + str(time.time()))
                if (angle < 260) and (angle > 150) and scan_avant_en_cours == True :
                    drapeau_nouveau_scan = True
                    scan_avant_en_cours = False
                if(angle >= 260) or (angle <= 100) :
                    scan_avant_en_cours = True
                    #print("scan avant")
                if(Run_Lidar == False) :
                    break
        except :
            print("souci acquisition lidar")




def set_direction_degre(angle_degre) :
    angle = -angle_degre * 1.25/18
    if angle > angle_max : 
            angle = angle_max
    if angle < angle_min :
        angle = angle_min
    pwm_dir.change_duty_cycle(angle_centre + angle)
    
def set_vitesse_m_s(vitesse_m_s):
    if vitesse_m_s > vitesse_max_m_s :
        vitesse_m_s = vitesse_max_m_s
    elif vitesse_m_s < -vitesse_max_m_s :
        vitesse_m_s = -vitesse_max_m_s
    if vitesse_m_s == 0 :
        pwm_prop.change_duty_cycle(stop_prop)
    elif vitesse_m_s > 0 :
        vitesse = vitesse_m_s * 1.5/8
        pwm_prop.change_duty_cycle(stop_prop + point_mort_prop + vitesse )
    elif vitesse_m_s < 0 :
        vitesse = vitesse_m_s * 1.5/8
        pwm_prop.change_duty_cycle(stop_prop - point_mort_prop + vitesse )

def set_speed(pourcent):
    com_max = 1.25
    commande = pourcent*com_max/100
    if commande  > com_max:
        commande = com_max
    elif commande  < -com_max :
        commande = -com_max
    if(commande ==0):
        pwm_prop.change_duty_cycle(stop_prop)
    elif commande  > 0 :
        pwm_prop.change_duty_cycle(stop_prop + point_mort_prop + commande)
    else:
        pwm_prop.change_duty_cycle(stop_prop - point_mort_prop + commande)

def recule():
    set_speed(-100)
    time.sleep(0.3)
    set_speed(0)
    time.sleep(0.1)
    set_speed(-30)

def conduite_autonome():
    global drapeau_nouveau_scan
    global acqui_tableau_lidar_mm
    global tableau_lidar_mm
    global Run_Lidar
    print ("tâche conduite autonome démarrée")
    while Run_Lidar == True :
        if(drapeau_nouveau_scan == False) :
            time.sleep(0.01)
        else :
            for i in range(-100,101) :
                tableau_lidar_mm[i] = acqui_tableau_lidar_mm[i]
            acqui_tableau_lidar_mm = [0]*360
            #suppression des points omis (valeur = 0)
            for i in range(-98,99) :
                if (tableau_lidar_mm[i]==0) :
                    if (tableau_lidar_mm[i-1] != 0) and (tableau_lidar_mm[i+1] != 0) :
                        tableau_lidar_mm[i] = (tableau_lidar_mm[i-1] + tableau_lidar_mm[i+1])/2           
            
            print(tableau_lidar_mm)
            print(time.time())
            drapeau_nouveau_scan = False
        


Run_Lidar = True
thread_scan_lidar = threading.Thread(target= lidar_scan)
thread_scan_lidar.start()
time.sleep(1)

thread_conduite_autonome = threading.Thread(target = conduite_autonome)
thread_conduite_autonome.start()

while True :
    try : 
        time.sleep(1)
    except KeyboardInterrupt: #récupération du CTRL+C
        print("arrêt du programme")
        Run_Lidar = False
        break
    
thread_conduite_autonome.join()
thread_scan_lidar.join()          



#arrêt et déconnexion du lidar

lidar.stop_motor()
lidar.stop()
time.sleep(1)
lidar.disconnect()
pwm_prop.stop()
pwm_dir.stop()
print("PWM arrêtées")

#affichage des données acquises sur l'environnement
print(len(tableau_lidar_mm))
print(tableau_lidar_mm)
teta = [0]*360 #création d'un tableau de 360 zéros
for i in range(360) :
    teta[i]=i*np.pi/180
fig = plt.figure()
ax = plt.subplot(111, projection='polar')
line = ax.scatter(teta, tableau_lidar_mm, s=5)
line.set_array(tableau_lidar_mm)
ax.set_rmax(3000)
ax.grid(True)
plt.show()










