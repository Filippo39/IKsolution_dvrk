#!/usr/bin/env python

# Author: Filippo Mantovan
# Date: 05-01-2021

# (C) Copyright 2015-2017 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_arm_test.py <arm-name>
# --- end cisst license ---

import random
import dvrk
import numpy
import math
import vg
import PyKDL
from sympy import Point3D, Line3D, Plane
from sympy import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


###########################################################
###################### 3D PLOT ############################
###########################################################
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x =[]
y =[]
z =[]

# To avoid exponential print
numpy.set_printoptions(suppress=True)

# Create a Python proxy for PSM1, name must match ros namespace
p = dvrk.psm('PSM1')


p.home()


##################################################################################
##################### Generazione randomica dei 6 giunti #########################
##################################################################################

''' 
	limiti di giunto gradi      radianti
	Joint 1: -70 / +70          -1,22173  / 1,22173
	Joint 2: -50 / +50          -0,872665 / 0,872665
	Joint 3:  0 / +0,235         0        / +0,235
	Joint 4: -175 / +175        -3,05433  / 3,05433
	Joint 5: -175 / +175        -3,05433  / 3,05433
	Joint 6: -175 / +175        -3,05433  / 3,05433

'''

inf_j1 = math.radians(-70)
sup_j1 = math.radians(+70)
inf_j2 = math.radians(-50)
sup_j2 = math.radians(+50)
inf_j3 = 0
sup_j3 = 0.235
inf_j4 = math.radians(-175)
sup_j4 = math.radians(+175)
inf_j5 = math.radians(-175)
sup_j5 = math.radians(+175)
inf_j6 = math.radians(-175)
sup_j6 = math.radians(+175)
count = 0
casi_saltati = 0
err_medio_1R = 0
err_medio_2R = 0
err_medio_3P = 0
err_medio_4R = 0
err_medio_5R = 0
err_medio_6R = 0
count_t5_miss = 0

# testing IK for n times
for i in range (0,50):
	print ('#####################################################################################')
	print ('##################################      ' + str(i) + '     #######################################')
	print ('#####################################################################################')
	R1 = round( random.uniform(inf_j1, sup_j1), 3 )
	R2 = round( random.uniform(inf_j2, sup_j2), 3 )
	P3 = round( random.uniform(inf_j3, sup_j3), 3 )
	R4 = round( random.uniform(inf_j4, sup_j4), 3 )
	R5 = round( random.uniform(inf_j5, sup_j5), 3 )
	R6 = round( random.uniform(inf_j6, sup_j6), 3 )
	joints_array = numpy.array( [R1, R2, P3, R4, R5, R6] )

	p.move_joint_some(numpy.array(joints_array), numpy.array([0, 1, 2, 3, 4, 5])) 

	print 
	print ("Current Joint position ---- TOPIC: position_joint_current(state_joint_current) : ")
	print ( p.get_current_joint_position() )
	print
	
	current_position = p.get_current_position()
	xEE = current_position.p.x()
	yEE = current_position.p.y()
	zEE = current_position.p.z()
	EE = numpy.array ( [ xEE, yEE, zEE ] )
	xRCM = 0
	yRCM = 0
	zRCM = 0

	# rotazione EE
	rotazioneEE = current_position.M
	array_posa = current_position.p
	array1 = [rotazioneEE.UnitX()[0], rotazioneEE.UnitY()[0], rotazioneEE.UnitZ()[0]]
	array2 = [rotazioneEE.UnitX()[1], rotazioneEE.UnitY()[1], rotazioneEE.UnitZ()[1]]
	array3 = [rotazioneEE.UnitX()[2], rotazioneEE.UnitY()[2], rotazioneEE.UnitZ()[2]]

	rotazione_numpy = numpy.array([array1,array2,array3])


	# tip_to_yaw che e l'INVERSA di tooltip offset 
	tooltip_T_numpy = numpy.array([ [0,0,-1], 
									[-1,0,0], 
									[0,1,0]])
	
	''' PROCEDIMENTO DA USARE NEL CASO UTILIZZASSI psm-needle-driver-tip.json COME FILE DI CONFIGURAZIONE '''
	tip_to_yaw= numpy.array([ [0,0,-1,0], 
							  [-1,0,0,0], 
							  [0,1,0,-0.01],
							  [0,0,0,1]])

	rotazione_numpy_v2 = numpy.array([ [array1[0], array1[1], array1[2], array_posa[0]],
									   [array2[0], array2[1], array2[2], array_posa[1]],
									   [array3[0], array3[1], array3[2], array_posa[2]],
									   [0,0,0,1] ])
	ptip = numpy.array( [array_posa[0], array_posa[1], array_posa[2]] )
	base_to_yaw = numpy.matmul (rotazione_numpy_v2, tip_to_yaw)
	
	xEE = base_to_yaw[0][3]
	yEE = base_to_yaw[1][3]
	zEE = base_to_yaw[2][3]
	EE = numpy.array ( [ xEE, yEE, zEE ] )
	
	print ("EE: ", EE)

	# Ha senso considerare solo casi con zEE < 0 per come opera il robot
	if (zEE > 0):
		print
		print ("zEE > 0 caso non considerato!")
		print
		casi_saltati += 1
		continue;
	count +=1

	x.append(xEE)
	y.append(yEE)
	z.append(zEE)

	result = numpy.matmul(rotazione_numpy, tooltip_T_numpy)

	distanza_RCM_EE = math.sqrt( (xEE - xRCM)**2 + (yEE - yRCM)**2 + (zEE - zRCM)**2)
	
	''' sovrascrivo perche uso la tooltip con offset di 0.01'''
	vettore_normale_z = numpy.array ( [base_to_yaw[0][2], base_to_yaw[1][2], base_to_yaw[2][2]] )
	vettore_distanza_RCM_EE = numpy.array( [xEE, yEE, zEE] )
	cross_z_distanza = numpy.cross(vettore_distanza_RCM_EE, vettore_normale_z) 

	############################################################################################
	# Ora che ho gli strumenti posso calcolare l'equazione dei due piani (con punto + normale) #
	############################################################################################

	# 1. normale z e EE
	lx = vettore_normale_z[0] 
	my = vettore_normale_z[1] 
	nz = vettore_normale_z[2] 
	d = 0 # per adesso

	# Impongo il passaggio per il punto EE per ricavare d
	lxo = lx * xEE
	myo = my * yEE
	nzo = nz * zEE
	d = - ( lxo + myo + nzo)
	# Equazione --> lx + my + nz + d = 0

	# 2. normale (cross_z_distanza) e RCM
	lx_2 = cross_z_distanza[0] 
	my_2 = cross_z_distanza[1] 
	nz_2 = cross_z_distanza[2] 
	d_2 = 0 # per adesso

	# Impongo il passaggio per il punto RCM per ricavare d
	lxo_2 = lx_2 * xRCM
	myo_2 = my_2 * yRCM
	nzo_2 = nz_2 * zRCM
	d_2 = - ( lxo_2 + myo_2 + nzo_2)
	# Equazione --> lx_2 + my_2 + nz_2 + d_2 = 0

	#####################################################################################
	# Ora devo fare l'intersezione tra i due piani per ottenere l'equazione della retta #
	#####################################################################################


	h = Plane(Point3D(xEE,yEE,zEE), normal_vector=(lx,my,nz))
	g = Plane(Point3D(xRCM,yRCM,zRCM), normal_vector=(lx_2,my_2,nz_2))
	line = h.intersection(g)

	x_3dpoint = N((line[0].points)[0].x,6)
	y_3dpoint = N((line[0].points)[0].y,6)
	z_3dpoint = N((line[0].points)[0].z,6)

	# retta direttore tra EE e 3dpoint
	retta_direttore = numpy.array ( [xEE - x_3dpoint , yEE - y_3dpoint, zEE - z_3dpoint], dtype = numpy.float64 )

	offset = 0.0091
	min_err_t5 = 9999
	for i in range (0,2):
		

		lunghezza_vettore = numpy.linalg.norm (retta_direttore)
		retta_direttore_normalizzata = numpy.array ( [retta_direttore[0]/lunghezza_vettore, retta_direttore[1]/lunghezza_vettore, retta_direttore[2]/lunghezza_vettore ] )


		SR5 = EE - offset * retta_direttore_normalizzata


		###############################################################
		####################### CALCOLO GIUNTI ########################
		###############################################################
		d3 = math.sqrt(SR5[0]**2 + SR5[1]**2 + SR5[2]**2)


		if (zEE <= 0):
			d3 = d3 + 0.0156
		else:
			d3 = 0.0156 - d3

		theta1 = math.atan ( SR5[0] / SR5[2] )
		theta2 = math.asin(SR5[1]/(d3 - 0.0156))

		
		ny = numpy.array( [0,1,0] )
		base_to_pitch = SR5 - numpy.array([0,0,0])
		base_to_pitch = base_to_pitch / numpy.linalg.norm(base_to_pitch) #normalizzo
		npitch =  numpy.cross( vettore_distanza_RCM_EE, vettore_normale_z) 
		npitch = npitch / numpy.linalg.norm(npitch) #normalizzo
		theta4 = vg.angle ( npitch, numpy.cross (ny, base_to_pitch))
		theta4 = math.radians ( theta4)

		pitch_to_base = numpy.array([0,0,0]) - SR5
		if ( numpy.dot ( pitch_to_base, numpy.cross ( numpy.cross (ny, base_to_pitch), npitch)) >= 0 ):
			theta4 = - theta4
		

		pitch_to_yaw = EE - SR5
		theta5 = vg.angle (pitch_to_yaw, pitch_to_base)
		theta5 = 180 - theta5
		theta5 = math.radians (theta5)
		dot = False
		# Si guarda il <=0 perche e visto un po al contrario infatti si fa anche il 180-angolo per il complementare
		if (numpy.dot (npitch, numpy.cross(pitch_to_yaw, pitch_to_base)) <=0):
			theta5 = -theta5
			dot = True
		
		if (theta5 > math.radians(90) or theta5 < math.radians(-90)) and (theta4 > math.radians(90) or theta4 < math.radians(-90)):
			theta5 = -theta5

		elif (theta5 > math.radians(90) or theta5 < math.radians(-90)):
			theta5 = -theta5

		if (dot and theta5 < math.radians(-90)):
			theta5 = -theta5
		
		yaw_to_pitch = SR5 - EE
		yaw_to_tip = ptip - EE
		theta6 = vg.angle(yaw_to_pitch,yaw_to_tip)
		theta6 = 180 - theta6
		theta6 = math.radians (theta6)
		if (numpy.dot (vettore_normale_z , numpy.cross(yaw_to_pitch, yaw_to_tip) ) >= 0):
			theta6 = -theta6


		###############################################################
		####################### CALCOLO ERRORE ########################
		###############################################################

		
		err_1R = abs (-theta1 - p.get_current_joint_position()[0])
		err_2R = abs (-theta2 - p.get_current_joint_position()[1])
		err_3P = abs (d3 - p.get_current_joint_position()[2])

		# theta4 puo essere girato di 3.14 quindi sistemo questo problema inoltre se accade che theta4 differisce di piu da quello trovato da ik allora giro il segno di theta5
		err_4R = abs (theta4%math.pi - p.get_current_joint_position()[3]%math.pi)
		err_5R = abs (theta5%math.pi - p.get_current_joint_position()[4]%math.pi)
		err_6R = abs (theta6%math.pi - p.get_current_joint_position()[5]%math.pi)
		plusminus = u'\u00b1'


		'''
		 	Visto che produco 2 soluzioni, devo accertarmi che una delle due che si basa su offset 0.0091 o -0.0091 rispetti i vincoli di giunto
		 	se entrambe rispettano i vincoli di giunto allora mi trovo ad avere 2 soluzioni per la cinematica inversa del manipolatore
		'''

		if ( (-theta1 >= -1.22173 and -theta1 <= 1.22173) and (-theta2 >= -0.872665 and -theta2 <= +0.872665) and (d3 >= 0 and d3 <= 0.235) and
			(theta4 >= -3.07 and theta4 <= +3.07) and (theta5 >= -3.07 and theta5 <= +3.07)  and (theta6 >= -3.07 and theta6 <= +3.07) ):

			if (err_5R < min_err_t5):
				min_err_t5 = err_5R

			print 
			print ("Variabili di giunto e relativi errori: ")
			print ("theta1: "+str(-theta1) + "  errore di  " + str(err_1R)) 
			print ("theta2: "+str(-theta2) + "  errore di  " + str(err_2R)) 
			print ("d3: "+str(d3)  + "  errore di  " + str(err_3P))
			print ("theta4: "+str(theta4) + "  errore di  " + str(err_4R)) 
			print ("theta5: "+str(theta5)  + "  errore di  " + str(err_5R))
			print ("theta6: "+str(theta6)  + "  errore di  " + str(err_6R))
			print

		
		else: 
			print
			print ("Variabili di giunto e relativi errori: ")
			print ("theta1: "+str(-theta1) + "  errore di  " + str(err_1R)) 
			print ("theta2: "+str(-theta2) + "  errore di  " + str(err_2R)) 
			print ("d3: "+str(d3)  + "  errore di  " + str(err_3P))
			print ("theta4: "+str(theta4) + "  errore di  " + str(err_4R)) 
			print ("theta5: "+str(theta5)  + "  errore di  " + str(err_5R))
			print ("theta6: "+str(theta6)  + "  errore di  " + str(err_6R))
			print
		
		offset = -offset

	# if the error is over then 0.0005 then i cosnsider thet theta5 is wrong
	if (min_err_t5 > 0.0005):
		count_t5_miss += 1


print ('Casi totali: '+str(count))
print('Casi non considerati(causa zEE > 0): '+str(casi_saltati))
print ('Errori segno theta5: '+str(count_t5_miss))

ax.scatter(x, y, z, c='r', marker='o')

plt.title('Positions of the end effector')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
