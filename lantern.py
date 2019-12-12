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

#--------[ Library Configuration ]--------
path         = "/media/Lantern/Lantern_pics/"
startup_clip = 'lantern_slides.mp4'


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
            year = year[0]
            library.append( {"year":int(year), "month":int(month), "file": path+f, "last_play": startup_time} )

    library.sort(key = lambda x:x['year'])
    print(str(len(library)) + " entries in library")


#--------------------
def init_hardware():
#--------------------
    spi = spidev.SpiDev()
    spi.open(0,0)
    spi.max_speed_hz = 7629
    ser = serial.Serial("/dev/ttyS0", 57600)
    #  Warp Core Off, Gauges off, Spinner off 
    spi.xfer( bytes( "w0\n","ascii"))
    time.sleep(.5)
    spi.xfer( bytes( "g0,0,0\n","ascii"))
    time.sleep(.1)
    spi.xfer( bytes( "s0,0\n","ascii"))


#-----------------
def get_files(y):
#-----------------
    pl = []
    min_dist = 1000
    # Scan to see how close we can come to the requested date
    for i in range(len(library)) :
        dist = abs(y-library[i]['year']) 
        if dist < min_dist :
            min_dist = dist;
            
    # Gather all files within the min distance        
    for i in range(len(library)) :
      dist = abs(y-library[i]["year"])
      if dist <= min_dist :
          pl.append(i)
          
    print("Found " + str(len(pl)) +': ')
    return pl


def bytime(e):
    return e[2]

#---------------            
def do_style(enable):
#---------------
    if enable == 0:
        #  Quiet style 
        spi.xfer( bytes( "w0\n","ascii"))
        time.sleep(.5)
        spi.xfer( bytes( "g0,0,0\n","ascii"))
        time.sleep(.1)
        spi.xfer( bytes( "s0,0\n","ascii"))
        
    else:
        # Show some style: Warp on, front/right gears @ 50%
        spi.xfer( bytes( "w1\n","ascii"))
        time.sleep(1)
        spi.xfer( bytes( "g80,0,80\n","ascii"))
        time.sleep(.1)
        spi.xfer( bytes( "s1,0\n","ascii"))


#----------------------------
def move_hands(year,month) :
#----------------------------
    # Tell hands to move to the first entry's year
    cmd = "h" + str(year) + "." + str(month) + "\n"
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


#---------------------------------
def player_stopped() :
#---------------------------------
    print("Player done.")

    
# --------------------------------------------------------------------
#                              M a i n
#---------------------------------------------------------------------
# init_hardware()
library = []
load_library()     

# Video player to the startup screen and pause
player = OMXPlayer(path+startup_clip, args=['--display=5 --loop'])
time.sleep(1)
player.pause()
player.stopEvent = player_stopped

# Wait until body has finished calibration
if False :
    print("Waiting for calibration...")
    done = False
    while not done :
        d = spi.xfer( bytes("n\n","ascii"))
        if d[1] == 1 :
            done = True
        print( "tick")
        time.sleep(1)               
    print("Done")

# Main Command Loop  
while True:
    if True :
        # Debug mode
        cmd = "G0000\n"
        target = input("Target: ")
        try:
           year = int(target)
        except:
            year = 0        
        if year == 0 :
            exit();

    else :
        # Normal Mode
        c=ser.read_until()
        cmd=c.decode('ascii', "ignore")
        year = int(cmd[1:5])
  
    if cmd.startswith("G") :
        # 'GO' Command: Switch to startup clip, then go get a list of the closest file(s)
        try:
            player.load(path+startup_clip)
            time.sleep(1)
            player.pause()
        except:
            print("player died")
            
        print("Requested year: " + str(year))
        pl = get_files(year)
        
        for i in range(len(pl)):
            p = library[pl[i]]
            print( str(pl[i]) + ": (" + str(p['year']) + "." + str(p['month']) + " ) " + p['file'] + " @" + str(p['last_play']) )

        # do_style(1)
            
        # Tell hands to move to the first entry's year
        p = library[pl[0]]
        # move_hands(p["year"], p["month"])
        
        #  Quiet style now
        # do_style(0)
        
        #  Tell face that we're here
        # ser.write( bytes( "A\n","ascii"))

        # Play the least-recently played file in the list
        t = library[pl[0]]["last_play"]
        tx = pl[0]
        for i in range(len(pl)):
            if library[pl[i]]["last_play"] < t :
              tx = pl[i]

        p = library[tx];
        # Note the time we're playing this in the library
        library[tx]["last_play"] = time.time()
        print( str(tx) + ": (" + str(p["year"]) + ") " + p["file"] + "   @" + str(p["last_play"]) )
        player.load(p["file"])

        # Wait for player to complete
        


