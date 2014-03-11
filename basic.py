from numpy import exp
from numpy import median, mean
from BrickPi import *
import random

angle = 210
power = [100, 100]
port = [PORT_A, PORT_B]
speed = -100

MOTA = 1.00
MOTB = 1.0

goal = 30

sonar = [goal]*10


SPEEDSIZE = 10
SONARSPEEDSIZE = 3
curSpeedA = [0]*SPEEDSIZE
curSpeedB = [0]*SPEEDSIZE
curSpeedC = [0]*SONARSPEEDSIZE

def move(distance):
  genericMove(1.08 * distance, 1.08*distance, 220)
  

def turn(degree):    
   if abs(degree) < 30:
     genericMove(-degree/5.2, degree/5.2, 200)
   elif abs(degree) < 70:
     genericMove(-degree/6.4, degree/6.4, 200)
   elif abs(degree) < 130:
     genericMove(-degree/6.5, degree/6.5, 200)
   else:
     genericMove(-degree/6.5, degree/6.5, 200)
   return

def stopMotion():
    BrickPi.MotorEnable[PORT_A] = 1
    BrickPi.MotorEnable[PORT_B] = 1
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPi.MotorSpeed[PORT_B] = 0

    
def setupSensors():
  BrickPiSetup()
  #BrickPi.SensorType[PORT_3] = TYPE_SENSOR_TOUCH
  BrickPi.SensorType[PORT_4] = TYPE_SENSOR_TOUCH
  BrickPi.SensorType[PORT_3] = TYPE_SENSOR_ULTRASONIC_CONT
  BrickPi.SensorType[PORT_2] = TYPE_SENSOR_LIGHT_OFF
  BrickPi.MotorEnable[PORT_A] = 1
  BrickPi.MotorEnable[PORT_B] = 1
  BrickPi.MotorEnable[PORT_C] = 1
  BrickPiSetupSensors()


def blink():
    t = time.time()
    while(time.time() - t < 1.0):
      BrickPi.SensorType[PORT_2] = TYPE_SENSOR_LIGHT_ON
      BrickPiSetupSensors()
      BrickPiUpdateValues()
      time.sleep(0.05)
      BrickPi.SensorType[PORT_2] = TYPE_SENSOR_LIGHT_OFF
      BrickPiSetupSensors()
      BrickPiUpdateValues()
      time.sleep(0.05) 


def updateValues(sonar, res):
    size = len(sonar)
    i = 0
    while i < size - 1:
        sonar[i] = sonar[i+1]
        i += 1
    sonar[size - 1] = res
    return sonar

def measureSonar():
   MEASURES = 10
   sonar = [0]*MEASURES
   i = 0
   while i < MEASURES:
     sonar[i] = BrickPi.Sensor[PORT_3]
     BrickPiUpdateValues()
     i += 1
     time.sleep(0.01)  
   return int(median(sonar))

def measureSonarOld():
    global sonar
    res = BrickPi.Sensor[PORT_3]
    sonar = updateValues(sonar, res)
    return int(median(sonar))

def rotateSonar(d):
    sonarList = []
    i = 0
    toGo = 720
    if d < 0:
      toGo = -toGo
    BrickPiUpdateValues()
    (_,_,save) = updateTurns()
    currentEnc = save
    aimFor = save + toGo
    ERROR = 2
    power = 0
# vars for recording biggest gap between two measures    
    gap = 0
    lastM = 0
    curM = 0
    while not(currentEnc > (aimFor - ERROR) and currentEnc < (aimFor + ERROR)):
       toGo = aimFor - currentEnc

       sig = toGo / abs(toGo) if toGo != 0 else 0
       BrickPi.MotorEnable[PORT_C] = 1
       BrickPi.MotorSpeed[PORT_C] = sig * power
       
       time.sleep(0.01)
       BrickPiUpdateValues()
       curM = (abs(save - BrickPi.Encoder[PORT_C])/2)
       if (curM - lastM) > gap:
         gap = curM - lastM
       sonarList.append( ( curM, BrickPi.Sensor[PORT_3]) )
       lastM = curM
   
       (_,_,currentEncN) = updateTurns()
       speed = updateSonarSpeed(currentEncN-currentEnc)
       currentEnc = currentEncN
       if abs(speed) < 4.0:
         power += 1
       if abs(speed) > 6.5:
         power -= 1
       #print power, " ", speed
    BrickPi.MotorSpeed[PORT_C] = 0              
    BrickPiUpdateValues()

    print gap
    
    return sonarList

def setEngines(speedA, speedB):
  BrickPi.MotorEnable[PORT_A] = 1
  BrickPi.MotorEnable[PORT_B] = 1
  BrickPi.MotorSpeed[PORT_A] = int(speedA)
  BrickPi.MotorSpeed[PORT_B] = int(speedB)
  BrickPiUpdateValues()

def updateTurns():
  BrickPiUpdateValues()
  return (BrickPi.Encoder[PORT_A], BrickPi.Encoder[PORT_B],BrickPi.Encoder[PORT_C])

def measureSpeed(speed):
  (startA,startB, startC) = updateTurns()
  i = 0
  while i < 20:
    time.sleep(0.01)
    setEngines(speed, speed)
    BrickPiUpdateValues()
    i+=1
  (endA,endB,endC) = updateTurns()
  return (startA - endA, startB - endB, startC - endC)

def testSpeed():
  s = 0
  while(s < 120):
    s += 10
    print s,": ", measureSpeed(s)
    setEngines(0,0)
    time.sleep(3.)
  return  

def getCurrSpeed():
   return (mean(curSpeedA), mean(curSpeedB), mean(curSpeedC)) 

def updateSpeed(speedA, speedB, speedC):
   size = SPEEDSIZE
   i = 0
   while i < size-1:
     curSpeedA[i] = curSpeedA[i+1]
     curSpeedB[i] = curSpeedB[i+1]
     i+=1
   curSpeedA[size-1] = speedA
   curSpeedB[size-1] = speedB
   return getCurrSpeed()


def updateSonarSpeed(speedC):
   size = SONARSPEEDSIZE
   i = 0
   while i < size-1:
     curSpeedC[i] = curSpeedC[i+1]
     i+=1
   curSpeedC[size-1] = speedC
   (_,_,curSonarSpeed) = getCurrSpeed()
   return curSonarSpeed



def genericMove(lWheel, rWheel, maxSpeed):
  global curSpeedA, curSpeedB

  cycleConst = 38

  (startA,startB,_) = updateTurns()

  (curValueA, curValueB) = (startA, startB)

  toGoAGes = rWheel * cycleConst #const
  toGoBGes = lWheel * cycleConst #const

  aimA = startA + rWheel * cycleConst
  aimB = startB + lWheel * cycleConst

  toGoA = toGoAGes
  toGoB = toGoBGes
  
# speed shit
  (curSpeedA, curSpeedB) = ([0]*SPEEDSIZE,[0]*SPEEDSIZE) # reset globals
  (curPowerA, curPowerB) = (0,0) # start speed zero!
  (updSpeedA, updSpeedB) = (0,0) # save current Speed in method
  (maxA, maxB) = (0,0) # save maxSpeedA maxSpeedB in current trajectory
  (pushA, pushB) = (0,0) # attenuate or amplify speed
  ATTMINSPEED = 1.0
  ATTMAXSPEED = 1.6
  PUSHER = 1
  while (abs(toGoA) > 10 or abs(toGoB) > 10): #(abs(fractionAbsolvedA-1) > .02 and abs(fractionAbsolvedB-1) > .02):
    
    aSig = (int(toGoA / abs(toGoA))) if (toGoA != 0) else 0
    bSig = (int(toGoB / abs(toGoB))) if (toGoB != 0) else 0
    
    difference = abs(toGoB) - abs(toGoA)
    DIMISH = 30
    
    speedA = abs(maxSpeed * exp(-(float(difference)/DIMISH)**2)) if difference >= 0 else maxSpeed
    speedB = abs(maxSpeed * exp(-(float(difference)/DIMISH)**2)) if difference <= 0 else maxSpeed
    
    # attenuate speed to the end of trajectory
    speedA = speedA * attenuateMaxSpeed(toGoA)
    speedB = speedB * attenuateMaxSpeed(toGoB)

    speedA = speedA + pushA
    speedB = speedB + pushB
    ###print "toGoA:", toGoA, " speed: ", speedA, " - ", "toGoB:", toGoB, " speed: ", speedB, " - pushA:", pushA, " - pushB:" , pushB

    # set motors
    setEngines(aSig * speedA,bSig * speedB)
    
    time.sleep(0.01)
    # update turns
    

    (curValueAn,curValueBn,_) = updateTurns()
    (newUpdSpeedA, newUpdSpeedB,_) = updateSpeed(curValueAn-curValueA, curValueBn-curValueB,0)
    ###print (newUpdSpeedA, newUpdSpeedB)
 
    
    if abs(newUpdSpeedA) < ATTMINSPEED and abs(updSpeedA) < ATTMINSPEED:
      pushA += PUSHER
    if abs(newUpdSpeedA) >= ATTMAXSPEED and abs(updSpeedA) >= ATTMAXSPEED and pushA > 0:
      pushA -= PUSHER
    
    if abs(newUpdSpeedB) < ATTMINSPEED and abs(updSpeedB) < ATTMINSPEED:
      pushB += PUSHER
    if abs(newUpdSpeedA) >= ATTMAXSPEED and abs(updSpeedA) >= ATTMAXSPEED and pushB > 0:
      pushB -= PUSHER
    
    (updSpeedA, updSpeedB) = (newUpdSpeedA, newUpdSpeedB)

    # update restway to go
    curValueA = curValueAn
    curValueB = curValueBn
    (toGoA,toGoB) = (aimA - curValueA, aimB - curValueB)

  setEngines(0,0)
  


def attenuateMaxSpeed(toGo):
  d = 1200.0
  K = 0.4
  p = float(-(2*(1-K)/pow(d, 3))*pow(abs(toGo),3) + (3*(1-K)/pow(d, 2)) * pow(abs(toGo),2)) + K if (abs(toGo) < d) else 1
  #p = float(-(2/pow(d, 3))*pow(abs(toGo),3) + (3/pow(d, 2)) * pow(abs(toGo),2)) if (abs(toGo) < d) else 1
  return p



setupSensors()
#blink()
#print rotateSonar(360)
#print rotateSonar(-1)   
#rotateSonar(-360)
#turn(-45)
#time.sleep(1)

i = 10
while i < 3:
  #move(-100)
  #time.sleep(1)
  turn(10)
  time.sleep(1)
  turn(20)
  time.sleep(1)
  turn(30)
  time.sleep(1)
  #move(-100)
  time.sleep(1)
  turn(40)
  time.sleep(1)
  turn(-100)
  time.sleep(1)
  turn(90)
  time.sleep(1)
  turn(-90)
  time.sleep(1)
  i+=1
  print i
#time.sleep(1)
#turn(-10)
#time.sleep(1)
#turn(-20)
#time.sleep(1)
#turn(-30)
#time.sleep(1)
#turn(-40)
#move(-200)

print "out of basic"
