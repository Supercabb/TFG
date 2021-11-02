# -*- coding: utf-8 -*-

import arcade
import os

import logging
import time
import random
import csv
import copy
import math

from itertools import permutations  
from lp_solve import *
import numpy as np
from datetime import datetime

widthWindow=800
heightWindow=1024
radiusRobot=800
anchoCinta=1200
velocidadCinta=500
tipo_prueba="PRUEBA G"

timesPickRobot={}
timesPlaceRobot={}

timesMinPlaceY={}
timesMinPickY={}

timesMaxPlaceY={}
timesMaxPickY={}

#Funcion para cargar la posicion de los objetos desde un fichero CSV con datos reales
def LoadObjectsFromFile(objects_file):

    objects=[]
    actual_object={}
    actual_id=-1
    
    with open(objects_file, newline='') as csvfile:
        objectsCSV = csv.reader(csvfile, delimiter=';', quotechar='|')
    
        for row in objectsCSV:
            if(len(row)==1):
                continue
            if(row[0]=="ImageId"):
                continue
                 
            imageID=row[0]     
            YRelW=row[1]     
            XRelH=row[2] 
            utility=row[3]
            newObj={"YRelW":float(YRelW), "XRelH":float(XRelH), "utility":float(utility)}
            
            if(actual_id!=imageID):
                
                if(len(actual_object)>0):
                    objects.append(actual_object)
                
                actual_object={"idImage":imageID, "objects":[]}
                actual_id=imageID
                   
            actual_object["objects"].append(newObj)
        return objects
  
#Funcion para cargar los tiempos del robot y hacer la correccion por desplazamiento
def GetAndCorrectRobotTimes(timesFile, velCinta):

    timesPickRobotTmp={}
    timesPlaceRobotTmp={}
    
    timesPickRobotMult5={}
    timesPlaceRobotMult5={}
    
    timesPickCorrected={}
    timesPlaceCorrected={}
    
    ###Cargamos los datos actuales
    with open(timesFile, newline='') as csvfile:
         timesPicksCSV = csv.reader(csvfile, delimiter=';', quotechar='|')
         for row in timesPicksCSV:
    
             if(len(row)==1):
                 continue
             if(row[0]=="TypeMove"):
                 continue
             
             if(row[0]=="Pick"):
                 xPos=int(row[1])
                 yPos=int(row[2])
        
                 if(yPos not in timesPickRobotTmp.keys()):
                    timesPickRobotTmp[yPos]={}
                 
                 timesPickRobotTmp[yPos][xPos]=float(row[3])
                              
                 
             elif(row[0]=="Place"):
                 xPos=int(row[1])
                 yPos=int(row[2])
    
                 
                 if(yPos not in timesPlaceRobotTmp.keys()):
                    timesPlaceRobotTmp[yPos]={}
                 
                 timesPlaceRobotTmp[yPos][xPos]=float(row[3])
                             
                 
             else:
                 raise ValueError("Error leyendo fichero de tiempos del robot!!")

    
    ###Corregimos que sean multiples de 5 tambien en la x y en la y
    minY=min(timesPickRobotTmp.keys())          
    maxY=max(timesPickRobotTmp.keys())          
    
    iniY=(int(minY/5)*5)-5
    finY=(int(maxY/5)*5)+10
                 
    for yActual in range(iniY,finY,5):
        yClosest = min(timesPlaceRobotTmp.keys(), key=lambda i: abs(i-yActual))
        
        minX = min(timesPickRobotTmp[yClosest].keys())
        maxX = max(timesPickRobotTmp[yClosest].keys())
    
        iniX=(int(minX/5)*5)-5
        finX=(int(maxX/5)*5)+10
            
        timesPickRobotMult5[yActual]={}
        
        
        
        for x in range(iniX,finX,5):        
            ##Buscamos el valor mas cercano a la x
            xClosest = min(timesPickRobotTmp[yClosest].keys(), key=lambda i: abs(i-x))
            timesPickRobotMult5[yActual][x]=timesPickRobotTmp[yClosest][xClosest]
            
    for yActual in range(iniY,finY,5):
        
        yClosest = min(timesPlaceRobotTmp.keys(), key=lambda i: abs(i-yActual))
        minX = min(timesPlaceRobotTmp[yClosest].keys())
        maxX = max(timesPlaceRobotTmp[yClosest].keys())
    
        iniX=(int(minX/5)*5)-5
        finX=(int(maxX/5)*5)+10
    
        timesPlaceRobotMult5[yActual]={}        
        
        for x in range(iniX,finX,5):
            ##Buscamos el valor mas cercano a la x
            xClosest = min(timesPlaceRobotTmp[yClosest].keys(), key=lambda i: abs(i-x))
            timesPlaceRobotMult5[yActual][x]=timesPlaceRobotTmp[yClosest][xClosest]
    ###################
            
            
    timesPickCorrected=copy.deepcopy(timesPickRobotMult5)
    timesPlaceCorrected=copy.deepcopy(timesPlaceRobotMult5)
            
    ####Hacemos la correccion de tiempos        
    for yActual in timesPickRobotMult5.keys():
        for xActual in timesPickRobotMult5[yActual].keys():
            tempPosFinalObjeto=copy.deepcopy(timesPickRobotMult5[yActual])
            tempPosDiffTiempoObjeto={}
    
            for i in tempPosFinalObjeto.keys():
                tempPosFinalObjeto[i]=i+int((velCinta*tempPosFinalObjeto[i])/5)*5
                
            for i in tempPosFinalObjeto.keys():
                #Solo vamos a mirar posiciones que no se salgan del rango del robot
                if(tempPosFinalObjeto[i] in tempPosFinalObjeto.keys()):
                    ##El robot tarda en recorrer 5 mm 5/velCinta segundos
                    ###Por lo tanto el objeto llegara a cada X (i-xActual)*(5/velCinta)                    
                    ###Y la diferencia con el tiempo de pick a esa posicion debe ser minima
                    tempPosDiffTiempoObjeto[i]=abs(((i-xActual)/velCinta))-timesPickRobotMult5[yActual][i] 
            
            
            if(len(tempPosDiffTiempoObjeto)>0):                
                #Buscamos el tiempo menor en valor absoluto            
                finalX = min(tempPosDiffTiempoObjeto.keys(), key=lambda i: abs(tempPosDiffTiempoObjeto[i]))
                timesPickCorrected[yActual][xActual]=timesPickRobotMult5[yActual][finalX]
                timesPlaceCorrected[yActual][xActual]=timesPlaceRobotMult5[yActual][finalX]                               
            else:
                timesPickCorrected[yActual][xActual]=timesPickRobotMult5[yActual][xActual]
                timesPlaceCorrected[yActual][xActual]=timesPlaceRobotMult5[yActual][xActual]           
            
        return timesPickCorrected, timesPlaceCorrected

#Cargamos los tiempos del robot y los corregimos
timesPickCorrected, timesPlaceCorrected=GetAndCorrectRobotTimes('c:\\Users\\Supercabb\\Documents\\Mates\\TFG\\Datos\\log_prueba_2_steps_1.csv', velocidadCinta)


########Rellenamos las variables de tiempos que se usan en el simulador con [x][y] en lugar de [y][x]#######
timesPickRobot={}
timesPlaceRobot={}

timesMinPlaceY={}
timesMinPickY={}

timesMaxPlaceY={}
timesMaxPickY={}

for yActual in timesPickCorrected.keys():
     for xActual in timesPickCorrected[yActual].keys():         
         
         if(xActual not in timesPickRobot.keys()):
             timesPickRobot[xActual]={}
             
         timesPickRobot[xActual][yActual]=timesPickCorrected[yActual][xActual]
         
     timesMinPickY[yActual]=timesPickCorrected[yActual][min(timesPickCorrected[yActual], key=timesPickCorrected[yActual].get)]
     timesMaxPickY[yActual]=timesPickCorrected[yActual][max(timesPickCorrected[yActual], key=timesPickCorrected[yActual].get)]

for yActual in timesPlaceCorrected.keys():
     for xActual in timesPlaceCorrected[yActual].keys():

         if(xActual not in timesPlaceRobot.keys()):
             timesPlaceRobot[xActual]={}
         
         timesPlaceRobot[xActual][yActual]=timesPlaceCorrected[yActual][xActual]

     timesMinPlaceY[yActual]=timesPlaceCorrected[yActual][min(timesPlaceCorrected[yActual], key=timesPlaceCorrected[yActual].get)]
     timesMaxPlaceY[yActual]=timesPlaceCorrected[yActual][max(timesPlaceCorrected[yActual], key=timesPlaceCorrected[yActual].get)]
#########################################################################################
"""      
minPick=timesMinPickY[min(timesMinPickY.keys(), key=(lambda k: timesMinPickY[k]))]
minPlace=timesMinPlaceY[min(timesMinPlaceY.keys(), key=(lambda k: timesMinPlaceY[k]))]

maxPick=timesMaxPickY[max(timesMaxPickY.keys(), key=(lambda k: timesMaxPickY[k]))]
maxPlace=timesMaxPlaceY[max(timesMaxPlaceY.keys(), key=(lambda k: timesMaxPlaceY[k]))]

     
print(str(minPick+minPlace))
print(str(maxPick+maxPlace))
"""

####Cargamos la distribucion de objetos de la aplicacion####
objects_reality_positions=LoadObjectsFromFile("c:\\Users\\Supercabb\\Documents\\Mates\\TFG\\Datos\\objetos.txt")
############################################################


class Robot:
    def __init__(self, tipo_algoritmo, centroXRobot, anchoCinta):
        self.tipo_algoritmo=tipo_algoritmo
        self.centroXRobot=centroXRobot
        self.anchoCinta=anchoCinta
        self.estado="WAITING_OBJECT"
        self.timePickActual=0
        self.timePlaceActual=0
        self.pickObjActual={}        
        self.IdxObjectPickActual=0

    def transformCoordinates(self, objects):
        for obj in objects:           
            obj["x"]-=self.centroXRobot
            obj["y"]=self.anchoCinta/2-obj["y"]
        
    def update(self, delta_time, simulador):
        objects=simulador.getObjects()
        objectsRobot=copy.deepcopy(objects)
        self.transformCoordinates(objectsRobot)
        
        if(self.estado=="WAITING_OBJECT"):
            pickObjIdx=self.selectObjectToPick(objectsRobot)
            self.IdxObjectPickActual=pickObjIdx
            
            if(pickObjIdx>=0):
                self.estado="MOVE_TO_PICK"
                self.timePickActual=self.getPickTimeObject(objectsRobot[pickObjIdx])+self.timePlaceActual-delta_time
                self.timePlaceActual=self.getPlaceTimeObject(objectsRobot[pickObjIdx])  
                self.pickObjActual=objects[pickObjIdx]  
        elif(self.estado=="MOVE_TO_PICK"):
            self.timePickActual-=delta_time
            if(self.timePickActual<=0):
                 self.timePlaceActual+=self.timePickActual                
                 simulador.pickObject(self.pickObjActual)
                 self.estado="MOVE_TO_PLACE"  
                 
        elif(self.estado=="MOVE_TO_PLACE"):
            self.timePlaceActual-=delta_time
            if(self.timePlaceActual<=0):
                self.estado="WAITING_OBJECT"
        
    def getPlaceTimeObject(self, obj):
        
        xGrid=math.trunc(obj["x"]/5)*5             
        yGrid=math.trunc(obj["y"]/5)*5

    
        if xGrid in timesPlaceRobot.keys():
            if yGrid in timesPlaceRobot[xGrid].keys():
                #print(str(xGrid)+" "+str(yGrid)+" "+str(timesPickRobot[xGrid][yGrid]))
                return timesPlaceRobot[xGrid][yGrid]
           
                
        raise ValueError("No tengo place time para la posicion "+str(xGrid)+" "+str(yGrid))        
        
    def getPickTimeObject(self, obj):
        
        
        xGrid=math.trunc(obj["x"]/5)*5             
        yGrid=math.trunc(obj["y"]/5)*5
        
        if xGrid in timesPickRobot.keys():
            if yGrid in timesPickRobot[xGrid].keys():
                #print(str(xGrid)+" "+str(yGrid)+" "+str(timesPickRobot[xGrid][yGrid]))
                return timesPickRobot[xGrid][yGrid]
            
        raise ValueError("No tengo pick time para la posicion "+str(xGrid)+" "+str(yGrid))

    def objectIsAvaliableForPick(self, obj):
        
        lon_x_respect_y=math.trunc(abs(math.sqrt(math.pow(radiusRobot,2)-math.pow(obj["y"],2))))
        x_min=-lon_x_respect_y   
        x_max=lon_x_respect_y
        
        if(obj["x"]>=x_min and obj["x"]<=x_max) and obj["picked"]==False:                                 
            movement_object_on_pickX=velocidadCinta*self.getPickTimeObject(obj)
            if(obj["x"]+movement_object_on_pickX>=x_min and obj["x"]+movement_object_on_pickX<=x_max):            
                return True
        
        return False
       
    
    def selectObjectPickMasterSinBarreras(self, objectsRobot):
        
        objectsAvaliable=[]
        
        for idx, obj in enumerate(objectsRobot):
            if(self.objectIsAvaliableForPick(obj)):
                objectsAvaliable.append({"id":idx,"obj":obj})

        max_utility=0
        
        for obj in objectsAvaliable:
            if(obj["obj"]["u"]>=max_utility):
                max_utility=obj["obj"]["u"]
            
        maxDistanceX=-800;        
        object_more_close_to_exit=-1;
        
        for obj in objectsAvaliable:
            if(obj["obj"]["u"]==max_utility):
                if(obj["obj"]["x"]>maxDistanceX):
                    maxDistanceX=obj["obj"]["x"]
                    object_more_close_to_exit=obj["id"]

        return object_more_close_to_exit
    
    def moveObjectsPermutationForwardSimulate(self, objetsToMove, moveX):
        for obj in objetsToMove:
            obj["obj"]["x"]=obj["obj"]["x"]+moveX    
    
    
    def simulateOrderFuerzaBruta(self, orden):        
        
        if(len(orden)<=0):
            return -1
        
        if(not (self.objectIsAvaliableForPick(orden[0]["obj"]))):
            return -1
        
        orderCopy=copy.deepcopy(orden)
        utilityPicked=0
        canPickObject=False
        
        while(len(orderCopy)>0 and self.objectIsAvaliableForPick(orderCopy[0]["obj"])):
           utilityPicked+=orderCopy[0]["obj"]["u"]
           canPickObject=True
           time_pick_place=self.getPickTimeObject(orderCopy[0]["obj"])+self.getPlaceTimeObject(orderCopy[0]["obj"])
           space_to_move=time_pick_place*velocidadCinta
           self.moveObjectsPermutationForwardSimulate(orderCopy, space_to_move)              
           orderCopy.pop(0)
        
        if(canPickObject):
            return utilityPicked
        
        return -1
    
        
        
    
    def selectObjectOptimoFuerzaBruta(self, objectsRobot):
        
        minim_close_to_robot_objects=2000
        objectsToCheck=[]
        max_objects_to_check=8
        
        for idx, obj in enumerate(objectsRobot):    
            lon_x_respect_y=math.trunc(abs(math.sqrt(math.pow(radiusRobot,2)-math.pow(obj["y"],2))))            
            x_max=lon_x_respect_y   
            x_min=-lon_x_respect_y   
             
            if((obj["x"]>(x_min-minim_close_to_robot_objects)) and (obj["picked"]==False) and obj["x"]<x_max) and len(objectsToCheck)<max_objects_to_check:
                objectsToCheck.append({"id":idx,"obj":obj})
        
        perm = list(permutations(objectsToCheck))
        
        
        max_utility=-1
        selected_object=-1
        for index,order in enumerate(perm):  
            utility_order=self.simulateOrderFuerzaBruta(list(order))
            if(utility_order>max_utility):
                selected_object=order[0]["id"]
                max_utility=utility_order
        
        
        
        if(selected_object!=-1):    
            return selected_object
        
        return -1
        
    def selectObjectPickMasterConBarreras(self, objectsRobot):
        
        ini_barrera=-250
        end_barrera=250
       
        objectsAvaliable=[]
        
        for idx, obj in enumerate(objectsRobot):
            if(obj["x"]>=ini_barrera and obj["x"]<=end_barrera and obj["picked"]==False):
                objectsAvaliable.append({"id":idx,"obj":obj})

        max_utility=0
        
        for obj in objectsAvaliable:
            if(obj["obj"]["u"]>=max_utility):
                max_utility=obj["obj"]["u"]
            
        maxDistanceX=-800;        
        object_more_close_to_exit=-1;
        
        for obj in objectsAvaliable:
            if(obj["obj"]["u"]==max_utility):
                if(obj["obj"]["x"]>maxDistanceX):
                    maxDistanceX=obj["obj"]["x"]
                    object_more_close_to_exit=obj["id"]

        if(object_more_close_to_exit!=-1):
            if(not self.objectIsAvaliableForPick(objectsRobot[object_more_close_to_exit])):
                 print(objectsRobot[object_more_close_to_exit])
                 raise ValueError("Pickmaster con barreras ha seleccionado un objeto fuera de alcance!!")
                
        return object_more_close_to_exit    
        
    
    
    
    def selectObjectPickMasterPLE(self, objectsRobot):
         objectsToCheck=[]
         max_objects_to_check=7
         #max_objects_to_check=12
         
         start_time = time.time()
         
         for idx, obj in enumerate(objectsRobot):    
            lon_x_respect_y=math.trunc(abs(math.sqrt(math.pow(radiusRobot,2)-math.pow(obj["y"],2))))            
            x_max=lon_x_respect_y   
            x_min=-lon_x_respect_y   
             
            if((obj["picked"]==False) and obj["x"]<x_max and len(objectsToCheck)<max_objects_to_check):
                objectsToCheck.append({"id":idx,"obj":obj, "min_x":x_min, "max_x":x_max})
    

         num_objects_to_check=len(objectsToCheck)
         
         if(num_objects_to_check<2):
             return -1

         selected_object=self.solvePLE(objectsToCheck, num_objects_to_check)
        
         #print("--- %s seconds ---" % (time.time() - start_time))
        
         return selected_object

    def getMaxMinTimePickAndPlaceObj(self, obj):
        minTimePick=0
        minTimePlace=0
        maxTimePick=0
        maxTimePlace=0
        yGrid=math.trunc(obj["y"]/5)*5
        
        if(yGrid in timesMinPlaceY.keys()):
            minTimePlace=timesMinPlaceY[yGrid]

        if(yGrid in timesMinPickY.keys()):
            minTimePick=timesMinPickY[yGrid]

        if(yGrid in timesMaxPlaceY.keys()):
            maxTimePlace=timesMaxPlaceY[yGrid]

        if(yGrid in timesMaxPickY.keys()):
            maxTimePick=timesMaxPickY[yGrid]

        
        return minTimePick, minTimePlace, maxTimePick, maxTimePlace

    def solvePLE(self, objectsToCheck, num_objects_to_check):
        
        variables=np.arange(1, (num_objects_to_check**2)+1, 1).tolist()
        lowerBoundVars=[0]*num_objects_to_check**2
        upperBoundVars=[1]*num_objects_to_check**2
        
        restrictions=[]
        vectorInequaciones=[]
        desigualdades=[]
        coeficients_function=[]
        
        #Restricciones, un objeto cada vez y un orden cada vez
        for i in range(num_objects_to_check):
            orden=np.zeros((num_objects_to_check, num_objects_to_check))
            orden[i,:]=1
            restrictions.append(orden.flatten('C').tolist())
            restrictions.append(orden.flatten('F').tolist())
            vectorInequaciones.append(1)
            vectorInequaciones.append(1)
            desigualdades.append(-1)
            desigualdades.append(-1)
        
        #Debemos respetar el orden
        for i in range(num_objects_to_check-1):
            orden=np.zeros((num_objects_to_check, num_objects_to_check))
            orden[:,i]=1
            orden[:,i+1]=-1
            restrictions.append(orden.flatten('C').tolist())
            vectorInequaciones.append(0)
            desigualdades.append(1)

                               
        #Restricciones de posicion
        #Calculo columna de pick and place de todos los objetos, maximo y minimo    
        maximos_objetos_pick_place = np.zeros((num_objects_to_check,1))
        minimos_objetos_pick_place = np.zeros((num_objects_to_check,1))

        maximos_objetos_pick = np.zeros((num_objects_to_check,1))
        minimos_objetos_pick = np.zeros((num_objects_to_check,1))

          
        for idx, obj in enumerate(objectsToCheck):
            minTimePick, minTimePlace, maxTimePick, maxTimePlace = self.getMaxMinTimePickAndPlaceObj(obj["obj"])
            maximos_objetos_pick_place[idx,0]=(maxTimePick+maxTimePlace)*velocidadCinta
            minimos_objetos_pick_place[idx,0]=(minTimePick+minTimePlace)*velocidadCinta
            maximos_objetos_pick[idx,0]=maxTimePick*velocidadCinta
            minimos_objetos_pick[idx,0]=minTimePick*velocidadCinta
            coeficients_function.extend([obj["obj"]["u"]]*num_objects_to_check)


        
        for i in range(num_objects_to_check):
            for idx, obj in enumerate(objectsToCheck):
                restriccion_matrix_no_salga=np.zeros((num_objects_to_check, num_objects_to_check))
                restriccion_matrix_no_entre=np.zeros((num_objects_to_check, num_objects_to_check))
                for j in range(0,i):
                    restriccion_matrix_no_salga[:,j]=maximos_objetos_pick_place[:,0]
                    restriccion_matrix_no_entre[:,j]=minimos_objetos_pick_place[:,0]
                
                suma_valores_actual=restriccion_matrix_no_salga[idx,:].sum()
                restriccion_matrix_no_salga[idx,i]=maximos_objetos_pick[idx,0]+suma_valores_actual
                restrictions.append(restriccion_matrix_no_salga.flatten('C').tolist())
                valorIneq=suma_valores_actual+(obj["max_x"]-obj["obj"]["x"])
                vectorInequaciones.append(valorIneq)
                desigualdades.append(-1)
            
                
                #restriccion_matrix_no_entre[idx,i]=(obj["obj"]["x"]-obj["min_x"])+minimos_objetos_pick[idx,0]
                restriccion_matrix_no_entre[idx,i]=(obj["obj"]["x"]-obj["min_x"])
                restrictions.append(restriccion_matrix_no_entre.flatten('C').tolist())                
                valorIneq=0
                vectorInequaciones.append(valorIneq)
                desigualdades.append(1)                  
                
        
        
        [utilidad, res_orden, duals] = lp_solve(coeficients_function, restrictions, vectorInequaciones, desigualdades, lowerBoundVars, upperBoundVars, variables)
        
        
        selected_object=-1
        for i in range(0,num_objects_to_check**2,num_objects_to_check):
            if(i<len(res_orden)) and (res_orden[i]==1):
                selected_object=int(i/num_objects_to_check )
                
                
        if(selected_object!=-1):
            if(not self.objectIsAvaliableForPick(objectsToCheck[selected_object]["obj"])):
                print(objectsToCheck[selected_object])
                raise ValueError("Algoritmo PLE ha seleccionado un objeto fuera de alcance!!")   
            
        if(selected_object!=-1):
            return objectsToCheck[selected_object]["id"]    
                
        return selected_object
        
        
    
    def selectObjectToPick(self, objectsRobot):
        
        if(self.tipo_algoritmo=="PICKMASTER_SIN_BARRERAS"):
            return self.selectObjectPickMasterSinBarreras(objectsRobot)
        elif(self.tipo_algoritmo=="PICKMASTER_BARRERAS"):
            return self.selectObjectPickMasterConBarreras(objectsRobot)
        elif(self.tipo_algoritmo=="FUERZA_BRUTA"):
            return self.selectObjectOptimoFuerzaBruta(objectsRobot)  
        elif(self.tipo_algoritmo=="PLE"):
            return self.selectObjectPickMasterPLE(objectsRobot)        
        else:
            raise ValueError("Algoritmo de robot "+self.tipo_algoritmo+" no implementado!!")        
        
        return -1

class Simulador:
    def __init__(self, scale, distancia_minus_robot, offsetY, tipo_algoritmo):
        self.scale=scale
        self.distancia_minus_robot=distancia_minus_robot
        self.offsetY = offsetY
        self.objects=[]
        self.utilityPicked=0
        self.utilityLost=0
        self.totalPicks=0
        self.time_elapsed=0.01
        self.tipo_algoritmo=tipo_algoritmo
        
        dateTimeObj = datetime.now()
        timestampStr = dateTimeObj.strftime("%d-%m-%Y_%H-%M")
        
        path_file_results="c:\\Users\\Supercabb\\Documents\\Mates\\TFG\\Pruebas\\"+self.tipo_algoritmo+"_"+tipo_prueba+"_"+timestampStr+".csv"
        
        self.file_results = open(path_file_results,'w')               
        self.file_results.write("time;utilityPicked;utilityLost;picks_por_minuto;totalPicks\n")        
        self.file_results.flush()

    def clearPoints(self):
        self.objects.clear()

    def pickObject(self, obj):
        
        if(obj["picked"]==True):
             raise ValueError("Objeto pickado dos veces!!")
        
        objs = [x for x in self.objects if x==obj]
        
        if(len(objs)!=1):
            raise ValueError("Intento hacer pick a un objeto que no existe!!")
                
        for x in self.objects:
            x["lastPicked"]=False
            
        objs[0]["picked"]=True                
        objs[0]["lastPicked"]=True                

    def addPoint(self, x, y, u):
        obj={"x": x, "y":y, "u":u, "picked":False, "lastPicked": False}
        
        objs = [x for x in self.objects if x==obj]

        if(len(objs)==0):   
            self.objects.append(obj)
        
    def moveObjectsForward(self, moveX):
        for obj in self.objects:
            obj["x"]=obj["x"]+moveX
        
    def removeObjectsOut(self):
        
        objToRemove=[]
        for obj in self.objects:
            
            if((obj["x"]*self.scale)>widthWindow):                
                objToRemove.append(obj)
            
        for obj in objToRemove: 
            if(obj["picked"]==False):
                self.utilityLost+=obj["u"]
            else:
                self.utilityPicked+=obj["u"]
                self.totalPicks+=1
                
            self.objects.remove(obj)
            
        return objToRemove
        
    
    
    def getObjects(self):
        return self.objects
    
    def getCenterRobotActionArea(self):
        return heightWindow-self.offsetY        
    
    def paint(self):
        global tipo_prueba
        scale=self.scale
        distancia_minus_robot=self.distancia_minus_robot
        offsetY=self.offsetY

        #Para la representacion grafica movemos el robot a la derecha el minimo tiempo de pick
        offsetX=int(timesMinPickY[min(timesMinPickY, key=timesMinPickY.get)]*velocidadCinta)

        arcade.draw_rectangle_filled(widthWindow/2, heightWindow-(radiusRobot*scale)-offsetY, widthWindow, anchoCinta*scale, arcade.color.BLACK)
        arcade.draw_circle_outline(widthWindow-(radiusRobot*scale)-distancia_minus_robot+offsetX, heightWindow-(radiusRobot*scale)-offsetY, radiusRobot*scale, arcade.color.RED, 3)

        YCintaCenter=heightWindow-(radiusRobot*scale)-offsetY


        for obj in self.objects:
            if(obj["picked"]==False):
                arcade.draw_circle_filled(obj["x"]*scale, YCintaCenter+(anchoCinta*scale/2)-(obj["y"]*scale), 5, arcade.color.BLUE)
            elif(obj["lastPicked"]==True):
                arcade.draw_circle_filled(obj["x"]*scale, YCintaCenter+(anchoCinta*scale/2)-(obj["y"]*scale), 5, arcade.color.GREEN)
            else:
                arcade.draw_circle_filled(obj["x"]*scale, YCintaCenter+(anchoCinta*scale/2)-(obj["y"]*scale), 5, arcade.color.RED)

        picks_por_minuto=self.totalPicks/(self.time_elapsed/60.0)        



        arcade.draw_text(self.tipo_algoritmo, 10, heightWindow-offsetY-15, arcade.color.BLACK, 15)
        arcade.draw_text(tipo_prueba, 650, heightWindow-offsetY-15, arcade.color.BLACK, 15)
        arcade.draw_text("utilityPicked="+str(round(self.utilityPicked,3)), 10, heightWindow-offsetY-35, arcade.color.BLACK, 15)
        arcade.draw_text("utilityLost="+str(round(self.utilityLost,3)), 175, heightWindow-offsetY-35, arcade.color.BLACK, 15)
        arcade.draw_text("picksMinute="+str(round(picks_por_minuto,1)), 325, heightWindow-offsetY-35, arcade.color.BLACK, 15)
        arcade.draw_text("totalPicks="+str(round(self.totalPicks)), 500, heightWindow-offsetY-35, arcade.color.BLACK, 15)
        arcade.draw_text("totalUtility="+str(round(self.utilityPicked+self.utilityLost,3)), 620, heightWindow-offsetY-35, arcade.color.BLACK, 15)

    def write_metrics(self):
        picks_por_minuto=self.totalPicks/(self.time_elapsed/60.0)
        dateTimeObj = datetime.now()
        timestampStr = dateTimeObj.strftime("%d-%m-%Y %H:%M:%S")
        strRow=timestampStr+";"+str(self.utilityPicked)+";"+str(self.utilityLost)+";"+str(picks_por_minuto)+";"+str(self.totalPicks)+"\n"
        self.file_results.write(strRow)
        self.file_results.flush()

scala=0.2
dist_minus_robot=100

rob1=Robot("PICKMASTER_SIN_BARRERAS",widthWindow/scala-radiusRobot-dist_minus_robot/scala, anchoCinta)
rob2=Robot("PICKMASTER_BARRERAS",widthWindow/scala-radiusRobot-dist_minus_robot/scala, anchoCinta)
rob3=Robot("PLE",widthWindow/scala-radiusRobot-dist_minus_robot/scala, anchoCinta)

sim1=Simulador(scala, dist_minus_robot, 10, rob1.tipo_algoritmo)
sim2=Simulador(scala, dist_minus_robot, 350, rob2.tipo_algoritmo)
sim3=Simulador(scala, dist_minus_robot, 690, rob3.tipo_algoritmo)


#rob3=Robot("FUERZA_BRUTA",widthWindow/scala-radiusRobot-dist_minus_robot/scala, anchoCinta)



timelapseAddObjectsIni=0.0
timelapseAddObjectsFin=0.7
wait_time_add_Objects=random.uniform(timelapseAddObjectsIni, timelapseAddObjectsFin)
time_elapsed_total_add=0 


timeUpdateLogics=0.05


def moveObjects(time_move_seconds):
    space=velocidadCinta*time_move_seconds
    
    sim1.moveObjectsForward(space)
    sim2.moveObjectsForward(space)
    sim3.moveObjectsForward(space)


def CheckNoPickedObjets():    
    sim1.removeObjectsOut()
    sim2.removeObjectsOut()
    sim3.removeObjectsOut()
    
 
utility_total=0
def AddObjects(time_elapsed):
    global time_elapsed_total_add
    global wait_time_add_Objects
    time_elapsed_total_add+=time_elapsed
    if(time_elapsed_total_add>=wait_time_add_Objects):
        yObject=random.randint(5,anchoCinta-5)
        utility=np.random.normal(1,0.5)
        #utility=random.uniform(1,1.75)
        #utility=1
        sim1.addPoint(0, yObject, utility)
        sim2.addPoint(0, yObject, utility)
        sim3.addPoint(0, yObject, utility)                
            
        wait_time_add_Objects=random.uniform(timelapseAddObjectsIni, timelapseAddObjectsFin)   
        time_elapsed_total_add=0
        
        
actual_image_id=0    
space_mm_X_muestra=690    
def AddObjectsFromFile(time_elapsed):
    global actual_image_id
    global time_elapsed_total_add
    global wait_time_add_Objects
    time_elapsed_total_add+=time_elapsed
    wait_time_add_Objects=space_mm_X_muestra/velocidadCinta
    if(time_elapsed_total_add>=wait_time_add_Objects):  
        objects_to_add=objects_reality_positions[actual_image_id]["objects"]
        
        ###Añadimos los objetos############
        for obj in objects_to_add:            
            sim1.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, obj["utility"])
            sim2.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, obj["utility"])
            sim3.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, obj["utility"])
        ###################################
        
        actual_image_id+=1
        actual_image_id%=len(objects_reality_positions)
        time_elapsed_total_add=0
    
def AddObjectsFromFileUtilityFixed(time_elapsed):
    global actual_image_id
    global time_elapsed_total_add
    global wait_time_add_Objects
    time_elapsed_total_add+=time_elapsed
    wait_time_add_Objects=space_mm_X_muestra/velocidadCinta
    if(time_elapsed_total_add>=wait_time_add_Objects):  
        objects_to_add=objects_reality_positions[actual_image_id]["objects"]
        
        ###Añadimos los objetos############
        for obj in objects_to_add:            
            sim1.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, 1)
            sim2.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, 1)
            sim3.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, 1)
        ###################################
        
        actual_image_id+=1
        actual_image_id%=len(objects_reality_positions)
        time_elapsed_total_add=0
        
def AddObjectsFromFileUtilityRandom(time_elapsed):
    global actual_image_id
    global time_elapsed_total_add
    global wait_time_add_Objects
    time_elapsed_total_add+=time_elapsed
    wait_time_add_Objects=space_mm_X_muestra/velocidadCinta
    if(time_elapsed_total_add>=wait_time_add_Objects):  
        objects_to_add=objects_reality_positions[actual_image_id]["objects"]
        
        ###Añadimos los objetos############
        
        for obj in objects_to_add:            
            utility=np.random.uniform(0.1,1)
            sim1.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, utility)
            sim2.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, utility)
            sim3.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, utility)
        ###################################
        
        actual_image_id+=1
        actual_image_id%=len(objects_reality_positions)
        time_elapsed_total_add=0   
        
def AddObjectsFromFileYRandom(time_elapsed):
    global actual_image_id
    global time_elapsed_total_add
    global wait_time_add_Objects
    time_elapsed_total_add+=time_elapsed
    wait_time_add_Objects=space_mm_X_muestra/velocidadCinta
    if(time_elapsed_total_add>=wait_time_add_Objects):  
        objects_to_add=objects_reality_positions[actual_image_id]["objects"]
        
        
        ###Añadimos los objetos############
        for obj in objects_to_add:            
            random=np.random.uniform(0,1)
            sim1.addPoint(obj["XRelH"]*space_mm_X_muestra, random*anchoCinta, obj["utility"])
            sim2.addPoint(obj["XRelH"]*space_mm_X_muestra, random*anchoCinta, obj["utility"])
            sim3.addPoint(obj["XRelH"]*space_mm_X_muestra, random*anchoCinta, obj["utility"])
        ###################################
        
        actual_image_id+=1
        actual_image_id%=len(objects_reality_positions)
        time_elapsed_total_add=0        
        
def AddObjectsFromFileXRandom(time_elapsed):
    global actual_image_id
    global time_elapsed_total_add
    global wait_time_add_Objects
    time_elapsed_total_add+=time_elapsed
    wait_time_add_Objects=space_mm_X_muestra/velocidadCinta
    if(time_elapsed_total_add>=wait_time_add_Objects):  
        objects_to_add=objects_reality_positions[actual_image_id]["objects"]
        
        
        ###Añadimos los objetos############
        for obj in objects_to_add:            
            random=np.random.uniform(0,1)
            sim1.addPoint(random*space_mm_X_muestra, obj["YRelW"]*anchoCinta, obj["utility"])
            sim2.addPoint(random*space_mm_X_muestra, obj["YRelW"]*anchoCinta, obj["utility"])
            sim3.addPoint(random*space_mm_X_muestra, obj["YRelW"]*anchoCinta, obj["utility"])
        ###################################
        
        actual_image_id+=1
        actual_image_id%=len(objects_reality_positions)
        time_elapsed_total_add=0      
        
def AddObjectsFromFileOnlyNObjects(time_elapsed, n_objects):
    global actual_image_id
    global time_elapsed_total_add
    global wait_time_add_Objects
    time_elapsed_total_add+=time_elapsed
    wait_time_add_Objects=space_mm_X_muestra/velocidadCinta
    if(time_elapsed_total_add>=wait_time_add_Objects):  
        
        while(len(objects_reality_positions[actual_image_id]["objects"])!=n_objects):
            actual_image_id+=1
            actual_image_id%=len(objects_reality_positions)
        
        objects_to_add=objects_reality_positions[actual_image_id]["objects"]
        
        ###Añadimos los objetos############
        for obj in objects_to_add:            
            sim1.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, obj["utility"])
            sim2.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, obj["utility"])
            sim3.addPoint(obj["XRelH"]*space_mm_X_muestra, obj["YRelW"]*anchoCinta, obj["utility"])
        ###################################
        
        actual_image_id+=1
        actual_image_id%=len(objects_reality_positions)
        time_elapsed_total_add=0        
    


    
def update(delta_time):  
    global tipo_prueba
    delta_time=timeUpdateLogics
    moveObjects(delta_time)
    
    if(tipo_prueba=="PRUEBA A"):
        AddObjectsFromFile(delta_time)
    elif(tipo_prueba=="PRUEBA B"):
        AddObjectsFromFileUtilityFixed(delta_time)
    elif(tipo_prueba=="PRUEBA C"):
        AddObjectsFromFileUtilityRandom(delta_time)
    elif(tipo_prueba=="PRUEBA D"):
        AddObjectsFromFileYRandom(delta_time)
    elif(tipo_prueba=="PRUEBA E"):
        AddObjectsFromFileXRandom(delta_time)
    elif(tipo_prueba=="PRUEBA F"):
        AddObjectsFromFileOnlyNObjects(delta_time, 2)
    elif(tipo_prueba=="PRUEBA G"):
        AddObjectsFromFileOnlyNObjects(delta_time, 10)
    else:
        raise ValueError("Prueba "+tipo_prueba+" no implementada!!")  
    
    
    rob1.update(delta_time, sim1)
    rob2.update(delta_time, sim2)
    rob3.update(delta_time, sim3)
    
    CheckNoPickedObjets()

    sim1.time_elapsed+=delta_time
    sim2.time_elapsed+=delta_time
    sim3.time_elapsed+=delta_time

    
def write_all_metrics(delta_time):
    sim1.write_metrics()
    sim2.write_metrics()
    sim3.write_metrics()    
    
class Game(arcade.Window):
    def __init__(self, width, height):
        super().__init__(width, height, title="Simulation Window")
        arcade.set_background_color(arcade.color.WHITE)

    def on_draw(self):
        arcade.start_render()

        sim1.paint()
        sim2.paint()
        sim3.paint()
        

    def on_key_press(self, key, modifiers):
        if key == ord("q"):
            raise SystemExit
        elif key == ord("p"):
            arcade.pause(20)



random.seed()
game = Game(widthWindow, heightWindow)
game.activate()


arcade.schedule(update, timeUpdateLogics)
arcade.schedule(write_all_metrics, 5)
arcade.run()
arcade.unschedule(update)
arcade.unschedule(write_all_metrics)


    
    


