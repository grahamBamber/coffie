#include "tsic.h"
#include <stdio.h>
#include <unistd.h>
#include "pigpiomgr.h"
#using namespace std;
import pigpio as pigpio
import time
#//-----------------------------------------------------------------------------
#//
#// Copyright (C) 2014-2015 James Ward
#//
#// This software is free software; you can redistribute it and/or
#// modify it under the terms of the GNU Lesser General Public
#// License as published by the Free Software Foundation; either
#// version 2.1 of the License, or (at your option) any later version.
#//
#// This software is distributed in the hope that it will be useful,
#// but WITHOUT ANY WARRANTY; without even the implied warranty of
#// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
#// Lesser General Public License for more details.
#//
#//-----------------------------------------------------------------------------#

#// Also requires the PIGPIO library for compilation, which can be
#// downloaded here:
#//     http://abyz.co.uk/rpi/pigpio/
#//
#// This version uses PIGPIOD via the pigpiod_if functions
#//
#// Alternatively, there's a Kernel Module also available: tsic-kernel
#// which can be downloaded here:
#//     https://code.google.com/p/tsic-kernel

#//-----------------------------------------------------------------------------

#/// the total number of bits to read from the TSIC sensor
TSIC_BITS = 20

#/// the length of the bit frame used by the TSIC sensor in microseconds
TSIC_FRAME_US = 125

#/// scale factor used to convert sensor values to fixed point integer
SCALE_FACTOR = 1000

#/// minimum temperature for sensor (must match device data)
MIN_TEMP = -50

#/// maximum temperature for sensor (must match device data)
MAX_TEMP = 150
#/// special value used to denote invalid sensor data
INVALID_TEMP = -100000

#//-----------------------------------------------------------------------------

#/// calculate parity for an eight bit value
def parity8( value ):
    value = (value ^ (value >> 4)) & 0x0F
    return (0x6996 >> value) & 1

#//-----------------------------------------------------------------------------

#// Decode two 9-bit packets from the sensor, and return the temperature.
#// Returns either a fixed point integer temperature multiplied by SCALE_FACTOR,
#// or INVALID_TEMP in case of error
def tsicDecode( packet0, packet1 ):
#{
#    // strip off the parity bits (LSB)
    parity0 = packet0 & 1
    packet0 >>= 1
    parity1 = packet1 & 1
    packet1 >>= 1
    #print "Packet 0" +str(packet0)
    #print "Packet 1" +str(packet1)
    #// check the parity on both bytes
    valid = ( parity0 == parity8(packet0) ) and ( parity1 == parity8(packet1) )

    #// if the parity is wrong, return INVALID_TEMP
    if ( not valid ): 
        #//cerr << "tsic: parity error\n";
        return INVALID_TEMP
    #}

    #// if any of the top 5 bits of packet 0 are high, that's an error
    if ( (packet0 & 0xF8) != 0 ): 
        #//cerr << "tsic: prefix error\n";
        return INVALID_TEMP
    #}

    #// this is our raw 11 bit word */
    raw = (packet0 << 8) | packet1

    #// convert raw integer to temperature in degrees C
    temp = (MAX_TEMP - MIN_TEMP) * SCALE_FACTOR * raw / 2047 + MIN_TEMP * SCALE_FACTOR;

    #// check that the temperature lies in the measurable range
    if ( (temp >= MIN_TEMP * SCALE_FACTOR) and (temp <= MAX_TEMP * SCALE_FACTOR) ):
        #// all looks good
        return temp
    else: 
        #// parity looked good, but the value is out of the valid range
        #//cerr << "tsic: range error\n";
        return INVALID_TEMP
    
#}//tsicDecode

#//-----------------------------------------------------------------------------

class TSIC(object):
#TSIC::TSIC() :

    def __init__(self):
        self.m_gpio = 0
        self.m_open = False
        self.m_valid = False
        self.m_temperature = 0.0
        self.m_callback = -1
        self.m_count = 0
        self.m_lastLow = 0
        self.m_lastHigh = 0
        self.m_word = 0
        self.pi = ''
#{
#}

#//-----------------------------------------------------------------------------

#TSIC::~TSIC()
#{
#    close();
#}

#//-----------------------------------------------------------------------------

    def openTSIC( self, gpio ):

    #// close the device if already open
    #close();

        pi = pigpio.pi()
        self.pi = pi
    #// ensure PIGPIO is initialised
        if ( not pi ):
            return false

    #// set the GPIO pin to be an input
        pi.set_mode( gpio, pigpio.INPUT ) 
          #  return false

    #// set the GPIO pin pull up
        pi.set_pull_up_down( gpio, pigpio.PUD_UP )

    #// local static function used to forward GPIO alerts to an
    #// associated instance of the TSIC class
    #!!!!struct local {
    #!!!!    static void alertFunction(
    #!!!!        unsigned gpio, unsigned level, uint32_t tick, void *userData
    #!!!!    ) {
    #!!!!        TSIC *self = reinterpret_cast<TSIC*>( userData );
    #!!!!        if ( self != 0 ) self->alertFunction( gpio, level, tick );
    #!!!!    }
    #!!!!};

    #// set a function to receive events when the input changes state
        m_callback = pi.callback( gpio, pigpio.EITHER_EDGE, self.alertFunction);
    #!!!!if ( m_callback < 0 ) {
    #!!!!    // note: in case of failure leaves GPIO pin set as input
    #!!!!    return false;
    #}

    #// wait for a packet to arrive
        success = False
        for c in range(0,2):
            if (success):
		break
        #// sample rate is 10Hz, so we need to wait at least 1/10th second
            time.sleep( 0.11  )
        #// attempt to read the value
            value = 0.0
            success = self.getDegrees(value)
            print "Success" +str(success)
            #y
            print "Valie" +str(value)

    #// did we receive some data?
        if (not success):
        
#// no: remove alert function and return false
            #!!!!callback_cancel( m_callback );
            #!!!!m_callback = -1;
            return False;
                  

    #// store the GPIO pin
        self.m_gpio = gpio

    #// set flag to indicate we are open
        self.m_open = True

    #// success
        return True
#}//open

#//-----------------------------------------------------------------------------

    def closeTSIC(self):
#{
        if ( not self.m_open ):
	    return 


    #// remove the alert function
        #!!!!callback_cancel( m_callback );
        #!!!!m_callback = -1;

    #// note: leaves the GPIO pin set as input

    #// reset members
        self.m_gpio = 0
        self.m_open = False
        self.m_valid = False
        self.m_temperature = 0.0
        self.m_count = 0
        self.m_word = 0
        self.m_lastLow = 0
        self.m_lastHigh = 0
#}//close

#//-----------------------------------------------------------------------------

    def getDegrees( self, value ):
#{
    #std::lock_guard<std::mutex> lock( m_mutex );
        value = self.m_temperature
        #print value
        return self.m_valid
#}//getDegrees

#//-----------------------------------------------------------------------------

    def alertFunction(self, gpio, level , tick):
        #gpio = 0 #  // GPIO number (which should match the member variable)
        #level #      // GPIO level
        #tick  = pi.get_current_tick()  #// time stamp in microseconds
#) {
 #       print "LEVEL:" + str(level)
  #      print "TICK: " +str(tick) 
        if ( level == 1 ):
        #// bus went high
            self.m_lastHigh = tick
            timeLow = tick - self.m_lastLow

        #// assuming 125us frame, 25% duty (low) and 75% duty (high)
        #// we treat anything more than 50% duty as a high bit
            if ( timeLow < TSIC_FRAME_US/2 ):
            #// high bit
                self.m_word = (self.m_word << 1) | 1
            elif ( timeLow < TSIC_FRAME_US ):
            #// low bit
                self.m_word <<= 1
            elif ( timeLow > TSIC_FRAME_US*2 ):
            #// low for more than one frame, which should never happen and

            #// must therefore be an invalid bit: start again
                self.m_count = 0
                self.m_word  = 0

            #// ignore the invalid bit
                return
        
            self.m_count +=1;
            if ( self.m_count == TSIC_BITS ):
            #// decode the packet
                result = tsicDecode((self.m_word >> 10) & 0x1FF , self.m_word & 0x1FF  )
            

            #// update the temperature value
            #{
            #    // lock the mutex
            #    std::lock_guard<std::mutex> lock( m_mutex );

            #    // update the temperature value and validity flag
                if ( result != INVALID_TEMP ):
                    #self.m_temperature = result  / SCALE_FACTOR 
                    self.m_temperature = result
                    print "TEMPDONE" +str(result)
                    self.m_valid = True
                else:
                    self.m_valid = False
            #}

            #// prepare to receive a new packet
                self.m_count = 0
                self.m_word = 0
        
        else:
        #// bus went low
            self.m_lastLow = tick

        #// calculate time spent high
            timeHigh = tick - self.m_lastHigh;

        #// if the bus has been high for more than one frame, reset the
        #// counters to start a new packet
            if ( timeHigh > TSIC_FRAME_US*2 ):
                self.m_count = 0
                self.m_word  = 0
       # }
    #}
#}//alertFunction

#//-----------------------------------------------------------------------------
if __name__ == "__main__":
    tsic = TSIC()
    tsic.openTSIC(24)
    oc = 0.0
    valid = False
    while(True):
        valid = tsic.getDegrees(oc)

        time.sleep(0.1)
        print "VALID:" +str(tsic.m_valid)
        print "TEMP:" +str(tsic.m_temperature)

    tsic.closeTSIC()



