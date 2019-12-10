#!/usr/bin/python
from omxplayer.player import OMXPlayer
import spidev
import time
import serial
import os
import psutil
from pathlib import Path
import subprocess
import random

#-------------------
def load_library():
#-------------------
    global library
    # Load directory listing and build library
    path = "/media/Lantern/Lantern_pics/"
    startup_clip = 'lantern_slides.mp4'
    startup_time = time.time()

    library = []
    for f in os.listdir(path) :
        if f != startup_clip:
            year = f.split(".")
            if year[1]=="mp4" :
                year = year[0].split("_")
            year = year[0].split(".")
            if len(year) < 2:
                month = random.randint(1,12)
            else:
                month = year[1]
            year = int(year[0])
            library.append( [int(year), int(month), path+f, startup_time] )

    library.sort()
    print(str(len(library)) + " entries in library")


#--------------------
def init_hardware():
#--------------------
    spi = spidev.SpiDev()
    spi.open(0,0)
    spi.max_speed_hz = 7629
    ser = serial.Serial("/dev/ttyS0", 57600)

#-----------------
def get_files(y):
#-----------------
    pl = []
    min_dist = 1000
    # Scan to see how close we can come to the requested date
    for i in range(len(library)) :
        dist = abs(y-library[i][0]) 
        if dist < min_dist :
            min_dist = dist;
            
    # Gather all files within the min distance        
    for i in range(len(library)) :
      dist = abs(y-library[i][0])
      if dist <= min_dist :
          pl.append(i)
          
    print("Found " + str(len(pl)) + " starting at " + str(library[pl[0]][0]) + "." + str(library[pl[0]][1]))
    return pl


def bytime(e):
    return e[2]

#---------------            
def do_style(cmd):
#---------------    
    PRINT("dO sTYLE")


#----------------------------------
def debug_mode() :
#---------------------------
    while(True):
        target = input("Target: ")
        try:
           target = int(target)
        except:
            target = 0        
        if target == 0 :
            exit();
        get_files(target)


# --------------------------------------------------------------------
#                              M a i n
#---------------------------------------------------------------------
# init_hardware()
library = []
load_library()     
debug_mode()
    
#  Warp Core Off, Gauges off, Spinner off 
spi.xfer( bytes( "w0\n","ascii"))
time.sleep(.5)
spi.xfer( bytes( "g0,0,0\n","ascii"))
time.sleep(.1)
spi.xfer( bytes( "s0,0\n","ascii"))

# Video player to the startup screen and pause
player = OMXPlayer(path+startup_clip, args=['--display=5 --loop'])
time.sleep(1)
player.pause()

# Wait until body has finished calibration
print("Waiting for calibration...")
done = False
while not done :
    d = spi.xfer( bytes("n\n","ascii"))
    if d[1] == 1 :
        done = True
    print( "tick")
    time.sleep(1)
                   
print("Done")
#  Tell face that we're ready
ser.write( bytes( "R\n","ascii"))

   
while True:
   c=ser.read_until()
   cmd=c.decode('ascii', "ignore")
   
   if cmd.startswith("G") :
        # 'GO' Command: Get a list of the closest file(s)
        try:
            player.load(path+startup_clip)
            time.sleep(1)
            player.pause()
        except:
            print("player died")
        year = int(cmd[1:5])
        print("Requested year: " + str(year))
        pl = get_files(year)
        
        for i in range(len(pl)):
            p = library[pl[i]]
            print( str(pl[i]) + ": (" + str(p[0]) + "." + p[1] + " ) " + p[2] + " @" + str(p[3]) )

        # do_style(cmd)
        # Show some style: Warp on, front/right gears @ 50%
        spi.xfer( bytes( "w1\n","ascii"))
        time.sleep(1)
        spi.xfer( bytes( "g80,0,80\n","ascii"))
        time.sleep(.1)
        spi.xfer( bytes( "s1,0\n","ascii"))
    
        # Tell hands to move to the first entry's year
        cmd = cmd.replace("G", "h",1)
        cmd = "h" + str(library[pl[0]][0]) + ".5\n"
        print( "sending " + cmd)
        spi.xfer( bytes( cmd,"ascii"))

        # Poll for hands to be done          
        done = False
        while not done :
            d = spi.xfer( bytes("n\n","ascii"))
            if d[1] == 1 :
                done = True
            print( "tick")
            time.sleep(1)
                   
        print("Done")

        #  Quiet style now
        spi.xfer( bytes( "w0\n","ascii"))
        time.sleep(.5)
        spi.xfer( bytes( "g0,0,0\n","ascii"))
        time.sleep(.1)
        spi.xfer( bytes( "s0,0\n","ascii"))
        
        #  Tell face that we're here
        ser.write( bytes( "A\n","ascii"))

        # play the least-recently played file in the list
        t = library[pl[0]][2]
        tx = pl[0]
        for i in range(len(pl)):
            if library[pl[i]][2] < t :
              tx = pl[i]

        p = library[tx];
        # Note the time we're playing this in the library
        library[tx][2] = time.time()
        print( str(tx) + ": (" + str(p[0]) + ") " + p[1] + "   @" + str(p[2]) )
        player.load(p[1])


