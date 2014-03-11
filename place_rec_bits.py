#!/usr/bin/env python
# By Jacek Zienkiewicz and Andrew Davison, Imperial College London, 2014
# Based on original C code by Adrien Angeli, 2009

import random
import os
import basic
global signatures
# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = 72):
        self.sig = [0] * no_bins
        
    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];
        
        # Fills the filenames variable with names like loc_%%.dat 
        # where %% are 2 digits (00, 01, 02...) indicating the location number. 
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))

    # Get the index of a filename for the new signature. If all filenames are 
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1
            
        if (n >= self.size):
            return -1;
        else:    
            return n;
 
    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])
            
    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)
            
        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    ls.sig[i] = int(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."
        
        return ls
        
# FILL IN: spin robot or sonar to capture a signature and store it in ls
def characterize_location(ls, learning):
    listOfTuple = basic.rotateSonar(1)
    
    listOfTuple2 = basic.rotateSonar(-1)
    for k in range(len(ls.sig)):
      ls.sig[k] = -1
    #print ls.sig
    for tuple in listOfTuple:
      if tuple[0] <= 360:
        i = int(tuple[0]*len(ls.sig)/360)
        #print str(i) + " / " + str(len(ls.sig))
        if (i < len(ls.sig) ):
          if ls.sig[i] == - 1:
            #print "in bin " + str(i) + " we put " + str(tuple[1])
            ls.sig[i] = tuple[1]

    if learning:
      for tuple in listOfTuple2:
        if tuple[0] <= 360:
          i = int((360-tuple[0])*len(ls.sig)/360)
          if (i < len(ls.sig)):
            if ls.sig[i] == -1:
              ls.sig[i] = tuple[1]
            else:
              ls.sig[i] += tuple[1]
              ls.sig[i] /= 2

    i = 1
    while i < len(ls.sig) - 1:
      #print str(i) + " / " + str(len(ls.sig))
      if ls.sig[i] == -1:
        ls.sig[i] = (ls.sig[i-1] + ls.sig[i+1])/2
      i += 1

    print ls.sig


def characterize_location_challenge(ls):
    listOfTuple = basic.rotateSonar(1)
    
    for k in range(len(ls.sig)):
      ls.sig[k] = -1

    for tuple in listOfTuple:
      if tuple[0] <= 360:
        i = int(tuple[0]*len(ls.sig)/360)
        #print str(i) + " / " + str(len(ls.sig))
        if (i < len(ls.sig) ):
          if ls.sig[i] == - 1:
            #print "in bin " + str(i) + " we put " + str(tuple[1])
            ls.sig[i] = tuple[1]

    i = 1
    while i < len(ls.sig) - 1:
      #print str(i) + " / " + str(len(ls.sig))
      if ls.sig[i] == -1:
        ls.sig[i] = (ls.sig[i-1] + ls.sig[i+1])/2
      i += 1



# FILL IN: compare two signatures
def compare_signatures(ls1, ls2):
    dist = 0
    noDepthBins = 25
    binSize = int(255/25)
    depthMap1 = [0]*noDepthBins
    depthMap2 = [0]*noDepthBins
    i = 0
    while i < len(ls1.sig):
      a = int(ls1.sig[i]/binSize)
      b = int(ls2.sig[i]/binSize)
      if (a >= noDepthBins):
        a = noDepthBins -1
      if (b >= noDepthBins):
        b = noDepthBins -1
      depthMap1[a] += 1
      depthMap2[b] += 1

      i += 1

    i = 0
    dist = 0
    while i < noDepthBins:
      dist += (depthMap1[i] - depthMap2[i])**2
      i += 1

    return dist

# This function finds the rotate angle between two signatures of the same location
def findAngle(ls1, ls2):
    bestAngle = -1
    bestDist = float("inf")

    i = 0
    while i < len(ls1.sig):
      j = 0
      dist = 0
      while j < len(ls1.sig):
        dist += (ls1.sig[j] - ls2.sig[(j+i)%len(ls1.sig)])**2
        j += 1
      if dist < bestDist:
        bestDist = dist
        bestAngle = i
      i += 1

    bestAngle = bestAngle*360/len(ls1.sig)
    return bestAngle

# This function characterizes the current location, and stores the obtained 
# signature into the next available file.
def learn_location():
    ls = LocationSignature()
    characterize_location(ls, True)
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return
    
    signatures.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."

# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen
def recognize_location():
    ls_obs = LocationSignature();
    characterize_location(ls_obs, False);
    bestDist = float("inf")
    bestIndex = -1
    threshold = 5000

    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    for idx in range(signatures.size):
        print "STATUS:  Comparing signature " + str(idx+1) + " with the observed signature."
        ls_read = signatures.read(idx);
        dist    = compare_signatures(ls_obs, ls_read)
        print "Distance between " + str(idx +1) + " and the observed signature is " + str(dist)
        if dist < threshold and dist < bestDist:
          bestDist = dist
          bestIndex = idx

    if (bestIndex != -1):
      print "Best location is location " + str(bestIndex + 1)
      angle = findAngle(ls_obs, signatures.read(bestIndex))
      print "The robot is rotated by " + str(angle) + " degrees"
    else:
      print "Apparently we are lost!"


def recognize_location_challenge():
    global signatures
    ls_obs = LocationSignature();
    characterize_location_challenge(ls_obs);
    bestDist = float("inf")
    bestIndex = -1
    threshold = 5000

    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    for idx in range(signatures.size):
        ls_read = signatures.read(idx);
        dist    = compare_signatures(ls_obs, ls_read)
        if dist < threshold and dist < bestDist:
          bestDist = dist
          bestIndex = idx

    if (bestIndex != -1):
      angle = findAngle(ls_obs, signatures.read(bestIndex))
      return bestIndex+1, angle
    else:
      print "Apparently we are lost!"


# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files(). 
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned.

def learn(signatures):
  signatures.delete_loc_files()
  a = raw_input("Press Enter to learn the location 1...")

  print "Learning Location 1"
  learn_location()
  a = raw_input("Press Enter to learn the next location...")

  print "Learning Location 2"
  learn_location()
  a = raw_input("Press Enter to learn the next location...")

  print "Learning Location 3"
  learn_location()
  raw_input("Press Enter to learn the next location...")

  print "Learning Location 4"
  learn_location()
  raw_input("Press Enter to learn the next location...")

  print "Learning Location 5"
  learn_location()

signatures = SignatureContainer(5)

#basic.setupSensors()
#learning = True

#if learning:
#  learn(signatures)

#while True:
#  raw_input("Press Enter to recognize a location...")
#  recognize_location()
