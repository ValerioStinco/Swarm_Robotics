'''Generate 2 experiment files .argos for client and server.
Areas are distributed in the same locations but can have different colors'''

import random

#Parameters setting
P_gen = 0.75		#higher value -> more probability to have more areas
P_blue_ser = 0.7	#higher value -> more probability that server areas are blue
P_blue_cli = 0.7	#higher value -> more probability that client areas are blue

#Possible task locations
possible_tasks= [' position="0.75,0.75" radius="0.05" color="',
' position="0.75,0.25" radius="0.05" color="',
' position="0.75,-0.25" radius="0.05" color="',
' position="0.75,-0.75" radius="0.05" color="',
' position="0.25,0.75" radius="0.05" color="',
' position="0.25,0.25" radius="0.05" color="',
' position="0.25,-0.25" radius="0.05" color="',
' position="0.25,-0.75" radius="0.05" color="',
' position="-0.25,0.75" radius="0.05" color="',
' position="-0.25,0.25" radius="0.05" color="',
' position="-0.25,-0.25" radius="0.05" color="',
' position="-0.25,-0.75" radius="0.05" color="',
' position="-0.75,0.75" radius="0.05" color="',
' position="-0.75,0.25" radius="0.05" color="',
' position="-0.75,-0.25" radius="0.05" color="',
' position="-0.75,-0.75" radius="0.05" color="']

#Random areas generation
Sareas=[]
Careas=[]
counter=0
for i in range (16):
	if (random.uniform(0, 1) < P_gen):
		current_area='\t\t\t<Area{}'.format(counter)
		current_area+=possible_tasks[i]

		if(random.uniform(0, 1) < P_blue_ser):
			current_Sarea = current_area + '0,0,255'
		else:
			current_Sarea = current_area + '255,0,0'

		if(random.uniform(0, 1) < P_blue_cli):
			current_Carea = current_area + '0,0,255'
		else:
			current_Carea = current_area + '255,0,0'
		
		current_Sarea+=',255" ></Area{}>\n'.format(counter)
		current_Carea+=',255" ></Area{}>\n'.format(counter)
		Sareas.append(current_Sarea)
		Careas.append(current_Carea)
		counter+=1

#Writing client experiment xml
f=open("kilobot_ALF_dhtf_template.argos", "r")
temp_lines=f.readlines()
f.close()

f=open("kilobot_ALF_dhtf_client_g.argos", "w")
for i in range(len(Careas)):
	temp_lines.insert(-71, Careas[i])
temp_lines.insert(42, '\t\t\tmode="CLIENT">  <!-- The label can be set to "CLIENT" or "SERVER" to achieve the desired behaviour-->\n')
f.writelines(temp_lines)
f.close()


#Writing server experiment xml
f=open("kilobot_ALF_dhtf_template.argos", "r")
temp_lines=f.readlines()
f.close()

f=open("kilobot_ALF_dhtf_server_g.argos", "w")
for i in range(len(Sareas)):
	temp_lines.insert(-71, Sareas[i])
temp_lines.insert(42, '\t\t\tmode="SERVER">  <!-- The label can be set to "CLIENT" or "SERVER" to achieve the desired behaviour-->\n')
f.writelines(temp_lines)
f.close()
