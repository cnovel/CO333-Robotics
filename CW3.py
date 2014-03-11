from BrickPi import *
import particleDataStructures
import basic
from numpy import cos, sin, pi, arctan, sqrt, exp, arccos, arctan2
import random
import place_rec_bits

global signatures

distSonarToCenter = 13
robotX = float(84.0)
robotY = float(30.0)
robotA = float(0.0)
nbOfParticles = 30

mult = 3

particleSet = []

sonarVar = 3


def lineToCoord(x0, y0, x1, y1):
   middleX = 100.0
   middleY = 700.0
   print "drawLine:"+str( (middleX + mult * x0, middleY - mult * y0, middleX + mult * x1, middleY - mult * y1) )

def particlesToCoord(particleSet):
  middleX = 100.0
  middleY = 700.0
  particles = [(mult * particleSet[i][0] + middleX, - mult * particleSet[i][1]+middleY, particleSet[i][2]) for i in range(nbOfParticles)]
  print "drawParticles:" + str(particles)
   
def meanParticles():
  global particleSet
  meanX = 0
  meanY = 0
  meanSinT = 0
  meanCosT = 0
  i = 0
  while i < nbOfParticles:
    meanX += particleSet[i][0]*particleSet[i][3]
    meanY += particleSet[i][1]*particleSet[i][3]
    meanSinT += sin(particleSet[i][2]*pi/180)*particleSet[i][3]
    meanCosT += cos(particleSet[i][2]*pi/180)*particleSet[i][3]
    i += 1

  meanT = arctan2(meanSinT, meanCosT)*180/pi
  #meanX /= nbOfParticles
  #meanY /= nbOfParticles
  #meanT /= nbOfParticles

  return (meanX, meanY, meanT)


def particleInit():
   global particleSet
   i = 0
   particleSet = []
   while i < nbOfParticles:
      particleSet.append([robotX, robotY, robotA, 1.0/nbOfParticles])
      i += 1
   return particleSet


def updateSL(D, sigma1, sigma2):
  global particleSet, robotX, robotY,robotA
  i = 0
  #robotXn = robotX + float(D) *cos(robotA / 180 * pi)
  #robotYn = robotY + float(D) *sin(robotA / 180 * pi)

  while i < nbOfParticles:
    e = random.gauss(0, sigma1)
    f = random.gauss(0, sigma2)
    particleSet[i][0] += (D+e)*cos(particleSet[i][2] / 180 * pi) 
    particleSet[i][1] += (D+e)*sin(particleSet[i][2] / 180 * pi)
    particleSet[i][2] += f
    particleSet[i][2] = particleSet[i][2] % 360
    i += 1

  updateParticlesSonar()
  mean = meanParticles()
  canvas.drawLine( (robotX, robotY, mean[0], mean[1]) )
  robotX = mean[0]
  robotY = mean[1]
  robotA = mean[2]

  #particlesToCoord(particleSet)



def updateR(a, sigma):
  global particleSet,robotA
  robotA = (robotA + a)%360
  i = 0
  while i < nbOfParticles:
    g = random.gauss(0, sigma)
    particleSet[i][2] += a + g
    particleSet[i][2] = particleSet[i][2]%360
    i +=1


def driveLine(distance):
  basic.move(-distance)
  updateSL(distance, 1, 0.5)
  

def rotation(degree):
  basic.turn(degree)
  updateR(degree, abs(degree/90*2.6))


def square():
  driveLine(40)
  rotation(90)
  driveLine(40)
  rotation(90)
  driveLine(40)
  rotation(90)
  driveLine(40)
  rotation(90)

def sonarMeasure():
  while True:
    result = BrickPiUpdateValues()
    if not result:
      print basic.measureSonar()  
      time.sleep(0.1)
  
def navigateToWaypoint(X, Y):
  (x, y , t) = meanParticles()
  print "Estimate direction is = ", t
  print "aim to: ", (X,Y), " from ", str((x,y,t))
  diffX = (X - x) 
  diffY = (Y - y)
  dist = sqrt(diffX*diffX + diffY*diffY)

  aimAngle = arctan(diffY / diffX) if (diffX != 0) else pi/2 if (diffY >= 0) else -pi/2
  if diffX < 0:
    aimAngle += pi
  #ux = cos(t*pi/180)
  #uy = sin(t*pi/180)
  #angle = -arctan((diffX*ux + diffY*uy)/(diffX*uy - diffY*ux))
  
  print aimAngle*180/pi - t
  rotAngle = aimAngle*180/pi - t
  if (rotAngle > 180):
    while rotAngle > 180:
      rotAngle -= 360
  elif (rotAngle < -180):
    while rotAngle < -180:
      rotAngle += 360 
  print dist
  rotation(rotAngle)
  driveLine(dist)
  correction = True
  if correction:
    (x, y , t) = meanParticles()    
    (m, wall, beta) = getFacingWall(X, Y, t)
    z = basic.measureSonar()
    z += distSonarToCenter
    if (abs(m - z) > 5 and z < 250 and beta < 30):
      driveLine(z-m)
  

def calculate_likelihood(x, y, theta, z):
   measure = getFacingWall(x,y,theta)
   #print measure
   if len(measure) == 1:
     return 0
   elif ( z >= 255 or abs(measure[2]) > 60 ):
     return 0.3
   else:
     diffMZ = measure[0] - z
     pZM = exp(-diffMZ*diffMZ/(2*sonarVar*sonarVar)) + 0.05
     return pZM

def intersection(x, y, theta, wall):
  if (theta != 90 and theta != 270):
    a = sin(theta * pi / 180)/cos(theta * pi / 180)
    b = y - a * x
  else:
    a = 0
    b = y

  if (wall[0] == wall[2]):
    intersectionY = a*wall[0] + b
    if ( (wall[1] <= intersectionY <= wall[3]) or (wall[3] <= intersectionY <= wall[1]) ):
      return True
    else:
      return False
  else:
    if a == 0:
      if ( (wall[0]  <= x <= wall[2]) or (wall[2] <= x <= wall[0]) ):
        return True
      else:
        return False

    else:
      intersectionX = (wall[1] - b)/a
      if ( (wall[0]  <= intersectionX <= wall[2]) or (wall[2] <= intersectionX <= wall[0]) ):
        return True
      else:
        return False

def getFacingWall(x,y,theta):
  global mymap
  walls = mymap.walls
  measures = []
  if insideTheMap(x,y):
    for wall in walls:
      m = (wall[3] - wall[1])*(wall[0] - x) - (wall[2] - wall[0])*(wall[1] - y)
      d = (wall[3] - wall[1])*cos(theta * pi / 180) - (wall[2] - wall[0])*sin(theta * pi / 180) 
      m = ((m / d) if d != 0 else 0)
      beta = 180*arccos( (cos(theta*pi/180) * (wall[1] - wall[3]) + sin(theta*pi/180) * (wall[2] - wall[0])) / sqrt( (wall[1] - wall[3]) * (wall[1] - wall[3]) + (wall[2] - wall[0]) * (wall[2] - wall[0])) )/pi

      if(m > 0):
        measures.append((m,wall, beta))
    sorted_measures = sorted(measures, key = lambda tup: tup[0])
    for meas in sorted_measures:
      if intersection(x, y, theta, meas[1]):
        return meas
  else:
   return [0]
  return None # should never happen!!

def insideTheMap(x,y):
  if (x < 0):
    return False
  elif (y < 0):
    return False
  elif (0 <= x <= 84 and y > 168):
    return False
  elif (84 <= x <= 168 and y > 210):
    return False
  elif (168 <= x <= 210 and y > 84):
    return False
  elif x > 210:
    return False
  else:
    return True

def updateParticlesSonar():
  global particleSet, nbOfParticles

  z = basic.measureSonar()
  z += distSonarToCenter
  weightSum = 0
  for particle in particleSet:
    particle[3] *= calculate_likelihood(particle[0], particle[1], particle[2], z)
    weightSum += particle[3]

  for particle in particleSet:
    particle[3] /= weightSum

  cumulativeProbability = [0] * nbOfParticles
  cumulativeProbability[0] = particleSet[0][3]
  i = 1
  while i < nbOfParticles:
    cumulativeProbability[i] = cumulativeProbability[i-1] + particleSet[i][3]
    i += 1

  i = 0
  newParticleSet = []
  while i < nbOfParticles:
    r = random.random()
    j = 0
    while r > cumulativeProbability[j]:
      j += 1
    newParticleSet.append( [particleSet[j][0], particleSet[j][1], particleSet[j][2], 1.0/nbOfParticles] )
    i += 1

  particleSet = newParticleSet

def routine():
  particleTuple = [tuple(l)[0:3] for l in particleSet]
  particles.data = particleTuple
  particles.draw()
  navigateToWaypoint(180, 30)
  updateParticlesSonar()

  particleTuple = [tuple(l)[0:3] for l in particleSet]
  particles.data = particleTuple
  particles.draw()
  navigateToWaypoint(180, 54)
  updateParticlesSonar()

  particleTuple = [tuple(l)[0:3] for l in particleSet]
  particles.data = particleTuple
  particles.draw()
  navigateToWaypoint(126, 54)
  updateParticlesSonar()

  particleTuple = [tuple(l)[0:3] for l in particleSet]
  particles.data = particleTuple
  particles.draw()
  navigateToWaypoint(126, 168)
  updateParticlesSonar()

  particleTuple = [tuple(l)[0:3] for l in particleSet]
  particles.data = particleTuple
  particles.draw()
  navigateToWaypoint(126, 126)
  updateParticlesSonar()

  particleTuple = [tuple(l)[0:3] for l in particleSet]
  particles.data = particleTuple
  particles.draw()
  navigateToWaypoint(30, 54)
  updateParticlesSonar()

  particleTuple = [tuple(l)[0:3] for l in particleSet]
  particles.data = particleTuple
  particles.draw()
  navigateToWaypoint(84, 54)
  updateParticlesSonar()

  particleTuple = [tuple(l)[0:3] for l in particleSet]
  particles.data = particleTuple
  particles.draw()
  navigateToWaypoint(84, 30)
  updateParticlesSonar()

  particleTuple = [tuple(l)[0:3] for l in particleSet]
  particles.data = particleTuple
  particles.draw()



def challenge():
  global robotX, robotY, robotA, signatures
  s = raw_input('Press ENTER to launch the robot !')
  index, angle = place_rec_bits.recognize_location_challenge()
  print "We are in location " + str(index) + " with an orientation of " + str(angle) + " degrees."
  x1 = 84
  y1 = 30
 
  x2 = 180
  y2 = 30

  x4 = 126
  y4 = 54

  x5 = 126
  y5 = 168

  x6 = 126
  y6 = 126

  x7 = 30
  y7 = 54


  # We consider the best path to be :
  # 1 <-> 2 <-> 4 <-> 5 <-> 6 <-> 7 <-> 1

  # initialize the robot location
  if index == 1:
    robotX = float(84.0)
    robotY = float(30.0)
    robotA = float(angle)
  elif index == 2:
    robotX = float(180.0)
    robotY = float(30.0)
    robotA = float(angle)
  elif index == 3:
    robotX = float(126.0)
    robotY = float(54.0)
    robotA = float(angle)
  elif index == 4:
    robotX = float(126.0)
    robotY = float(168.0)
    robotA = float(angle)
  else:
    robotX = float(30.0)
    robotY = float(54.0)
    robotA = float(angle)

  particleInit()
  basic.move(-13)

  # let's roll!
  if index == 1:
    if (angle > 78 and angle < 234):
      navigateToWaypoint(x7, y7)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x6, y6)
      updateParticlesSonar()

      navigateToWaypoint(x5, y5)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x4, y4)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x2, y2)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x1, y1)
      basic.blink()

    else:
      navigateToWaypoint(x2, y2)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x4, y4)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x5, y5)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x6, y6)
      updateParticlesSonar()

      navigateToWaypoint(x7, y7)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x1, y1)
      basic.blink()

  elif index == 2:
    if (angle > 168 and angle < 348):
      navigateToWaypoint(x1, y1)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x7, y7)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x6, y6)
      updateParticlesSonar()

      navigateToWaypoint(x5, y5)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x4, y4)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x2, y2)
      basic.blink()

    else:
      navigateToWaypoint(x4, y4)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x5, y5)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x6, y6)
      updateParticlesSonar()

      navigateToWaypoint(x7, y7)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x1, y1)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x2, y2)
      basic.blink()

  elif index == 3:
    if (angle > 33 and angle < 213):
      navigateToWaypoint(x5, y5)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x6, y6)
      updateParticlesSonar()

      navigateToWaypoint(x7, y7)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x1, y1)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x2, y2)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x4, y4)
      basic.blink()

    else:
      navigateToWaypoint(x2, y2)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x1, y1)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x7, y7)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x6, y6)
      updateParticlesSonar()

      navigateToWaypoint(x5, y5)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x4, y4)
      basic.blink()

  elif index == 4:
    navigateToWaypoint(x6, y6)
    updateParticlesSonar()
 
    navigateToWaypoint(x7, y7)
    basic.blink()
    updateParticlesSonar()

    navigateToWaypoint(x1, y1)
    basic.blink()
    updateParticlesSonar()

    navigateToWaypoint(x2, y2)
    basic.blink()
    updateParticlesSonar()

    navigateToWaypoint(x4, y4)
    basic.blink()
    updateParticlesSonar()

    navigateToWaypoint(x5, y5)
    basic.blink()

  else:
    if (angle > 6 and angle < 186):
      navigateToWaypoint(x6, y6)
      updateParticlesSonar()

      navigateToWaypoint(x5, y5)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x4, y4)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x2, y2)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x1, y1)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x7, y7)
      basic.blink()

    else:
      navigateToWaypoint(x1, y1)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x2, y2)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x4, y4)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x5, y5)
      basic.blink()
      updateParticlesSonar()

      navigateToWaypoint(x6, y6)
      updateParticlesSonar()

      navigateToWaypoint(x7, y7)
      basic.blink()



## MAIN ##

basic.setupSensors()

particles = particleDataStructures.Particles()
particleSet = particleInit()

particleTuple = [tuple(l)[0:3] for l in particleSet]
particles.data = particleTuple
#particles.draw()

canvas = particleDataStructures.Canvas();      # global canvas we are going to draw on

mymap = particleDataStructures.Map();
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
#mymap.draw();


#basic.move(100)
#time.sleep(1)
#basic.genericMove(100, -100, 100)
#basic.turn(90)
#driveLine(particleSet)
#leftRotation(particleSet)
#square()

#basic.turn(360)

#navigateToWaypoint(50,50)
#navigateToWaypoint(50, -20)
#navigateToWaypoint(60, 0)
#navigateToWaypoint(0,0)
#sonarMeasure()

#routine()

challenge()
