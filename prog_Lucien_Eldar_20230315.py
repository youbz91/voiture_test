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

angle_pwm_min = 6.3 #gauche
angle_pwm_max = 9 #droite
angle_degre_max = +30 #vers la gauche
angle_pwm_centre= 7.65
angle_degre=0

pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
pwm_prop.start(stop_prop)
time.sleep(1.0)
pwm_prop.start(2.0)
time.sleep(0.1)
pwm_prop.start(1.0)
time.sleep(0.1)
pwm_prop.start(stop_prop)
pwm_dir.start(angle_pwm_centre)
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
    angle_pwm = angle_pwm_centre - (angle_pwm_max - angle_pwm_min) * angle_degre /(2 * angle_degre_max )
    if angle_pwm > angle_pwm_max : 
        angle_pwm = angle_pwm_max
    if angle_pwm < angle_pwm_min :
        angle_pwm = angle_pwm_min
    pwm_dir.change_duty_cycle(angle_pwm)
    
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

def recule():
    set_vitesse_m_s(-vitesse_max_m_s)
    time.sleep(0.2)
    set_vitesse_m_s(0)
    time.sleep(0.1)
    set_vitesse_m_s(-1)


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
            print(tableau_lidar_mm)
            #suppression des points omis (valeur = 0)
            for i in range(-98,99) :
                if (tableau_lidar_mm[i]==0) :
                    if (tableau_lidar_mm[i-1] != 0) and (tableau_lidar_mm[i+1] != 0) :
                        tableau_lidar_mm[i] = (tableau_lidar_mm[i-1] + tableau_lidar_mm[i+1])/2           
            drapeau_nouveau_scan = False
            
             ####################################################
            # programme Lucien Eldar avec des secteurs de 20 degres
            ###################################################
            
            #un secteur par tranche de 20° donc 10 secteurs numérotés de 0 à 9
            min_secteur = [0]*10
            
            
            
            for index_secteur in range(0,10) :
                angle_secteur = -90 + index_secteur*20
                min_secteur[index_secteur] = 8000
                for angle_lidar in range(angle_secteur-10,angle_secteur+10) :
                    if tableau_lidar_mm[angle_lidar] < min_secteur[index_secteur] and tableau_lidar_mm[angle_lidar] != 0 :
                        min_secteur[index_secteur] = tableau_lidar_mm[angle_lidar]
                if min_secteur[index_secteur] == 8000 : #aucune valeur correcte
                    min_secteur[index_secteur] = 0
            print("min_secteur[] :")
            print(min_secteur)
            
            if (min_secteur[4]<=130 and min_secteur[4] !=0)\
                    or (min_secteur[3]<=130 and min_secteur[3] !=0)\
                        or (min_secteur[2]<=130 and min_secteur[2] !=0) :
                angle_degre = -30
                set_direction_degre(angle_degre)
                print("mur à droite")
                recule()
                
            elif (min_secteur[5]<=130 and min_secteur[5] !=0)\
                    or (min_secteur[6]<=130 and min_secteur[6] !=0)\
                        or (min_secteur[7]<=130 and min_secteur[7] !=0) :
                angle_degre = +30
                set_direction_degre(angle_degre)
                print("mur à gauche")
                recule()
            elif np.argmax(min_secteur)>=3 and np.argmax(min_secteur)<=6 :
                angle_degre = 0.02*(min_secteur[7]-min_secteur[2])
                set_direction_degre(angle_degre)
                vitesse_m_s = 0.00025*np.max(min_secteur)
                set_vitesse_m_s(vitesse_m_s)
            else :
                angle_degre = (-90+20*np.argmax(min_secteur))/4
                print("secteur intéressant : "+str(angle_degre))
                set_direction_degre(angle_degre)
                vitesse_m_s = 0.2
                set_vitesse_m_s(vitesse_m_s)
                
            
            ###################################################
            
       
            
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

# #affichage des données acquises sur l'environnement
# print(len(tableau_lidar_mm))
# print(tableau_lidar_mm)
# teta = [0]*360 #création d'un tableau de 360 zéros
# for i in range(360) :
#     teta[i]=i*np.pi/180
# fig = plt.figure()
# ax = plt.subplot(111, projection='polar')
# line = ax.scatter(teta, tableau_lidar_mm, s=5)
# line.set_array(tableau_lidar_mm)
# ax.set_rmax(3000)
# ax.grid(True)
# plt.show()












