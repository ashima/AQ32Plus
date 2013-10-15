#  \file       ctdecode.py
#  \brief      Decoder for simple base64 small channelized telemetry.
#  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
#              file distributed under the MIT Expat License. See LICENSE file.
#              https://github.com/ashima/AQ32Plus
#  \remark     Ported for AQ32Plus.

import sys, argparse
import struct, serial
import base64
import evrstrings

def procargs() :
  """ Adds parser arguments and parses the command line arguments. """

  p = argparse.ArgumentParser( description="Decode base64 channelized telemetry.")
  p.add_argument("-i", dest='infile',  help="input file")
  p.add_argument("-s", dest='sport',  help="serial port for input" )
  p.add_argument("-o", dest='outf', help="output file basename" ) 

  return p.parse_args()


state = 0
i = 0
mlen = 0
tim = 0
mid = 0
noise = 0
buff=""
 
def pktError(s):
  l = len(s)
  if l < 8 :
    print ("PktError len < 8 : '%s'" % s)
  else :
    (mid,_) = struct.unpack_from( "<hl", base64.decodestring(buff) )
    mlen = msgLenths.get(mid)
    if mlen :
      print ("PktError mid=%d (0x%4x) data-recived=%d  msg len=%d : '%s'"% (mid, mid, l, mlen, s )) 
    else :
      print ("PktError mid=%d (0x%4x) data-recived=%d  unknown mid : '%s'"% (mid, mid, l, s )) 

  pass

msgFmts = {
  0x00 + 0x02 : ("<HLHHxx", "EVR"),
  0x00 + 0x08 : ("<HLHHxx", "State"),
  0x20 + 0x00 : ("<HLLh", "RawPres"),
  0x20 + 0x01 : ("<HLHhxx", "RawTemp"),
  0x20 + 0x02 : ("<HLBBLLLLLLLLxx", "RawGPS"),
  0x20 + 0x03 : ("<HLhhhhx", "RawACC"),
  0x40 + 0x00 : ("<HLfff", "WACC100Hz"),
  0x40 + 0x01 : ("<HLffffxx", "MotCmd"),
  0x60 + 0x00 : ("<HLffffxx", "HSF"),
  0x60 + 0x01 : ("<HLhhhHHHhhhhhh", "BMP180"),
  0x60 + 0x02 : ("<HLfxx", "Height"),
}

msgLenths = dict([ (i, 4*int((struct.calcsize(msgFmts[i][0])+2)/3)) for i in msgFmts ])

p2f = {
  "L": "d", "l": "d",
  "H": "d", "h": "d",
  "B": "d", "b": "d",
  "c": "s", "f": "f",
  }

zeros = ("\0" *64)

def pkt_decode(m, bs,(outfa,outfb)) :
  #print "{",len(s)
  s = base64.decodestring(bs) 

  if outfb :
    outfb.write( (s+zeros)[0:64] )

  if outfa :
    f = msgFmts[m][0]
    #print "m=",m," f=",f, "s=",bs, "#f=", struct.calcsize(f), "#s=", len(s)
    x = struct.unpack(f,s)
    p = " ".join([ "%"+p2f[i] for i in f if p2f.get(i) ])
    if mid == 0x02 :
      d = " \"%s\"" % evrstrings.evrStringTable[ x[2] ]; 
    else:
      d = ""
    outfa.write( ( "%10s " % msgFmts[m][1]) + (p % x) + d +"\n" )

def step_pkt_rd(x,fds) :
  global state, mlen, tim, mid, noise, buff,i
  #print "state = ",state,x, "mlen=",mlen
  if state == 0 :
    if x == '$' :
      buff = ""
      i = 0
      state = 1
    else:
      noise = noise +1

  elif state == 1 :
    if x == '$' :
      pktError(buff);
      state = 0
      step_pkt_rd(x,fds)
    else:
      buff += x
      i = i + 1
      if i == 8 :
        (mid,_) = struct.unpack_from( "<hl", base64.decodestring(buff) )
        mlen = msgLenths.get(mid)
        #print mid,mlen,state
        if mlen != None :
          state = 2

  elif state == 2 :
    if x == '$' :
      pktError(buff);
      state = 0
      step_pkt_rd(x,fds)
    else:
      buff += x
      i = i + 1
      if i >= mlen :
        pkt_decode(mid, buff, fds)
        state = 0
        mlen = 0

def rd():
  args = procargs()

  if args.sport :
    inf = serial.Serial( args.sport, 115200 )
  elif args.infile == "-" or args.infile == None :
    inf = sys.stdin
  else :
    inf = open(args.infile, "r")
  
  if args.outf :
    outfr = open(args.outf+".raw", "w")
    outfa = open(args.outf+".asc", "w")
    outfb = open(args.outf+".bin", "w")
  else :
    outfr = None
    outfa = sys.stdout
    outfb = None

  x = " "
  while x != "" :
    x = inf.read(1)
    if x != "" :
      if outfr :
        outfr.write(x)
      step_pkt_rd(x,(outfa,outfb))

rd()
