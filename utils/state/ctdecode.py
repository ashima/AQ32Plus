#  \file       ctdecode.py
#  \brief      Decoder for simple base64 small channelized telemetry.
#  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
#              file distributed under the MIT Expat License. See LICENSE file.
#              https://github.com/ashima/AQ32Plus
#  \remark     Ported for AQ32Plus.

import sys
import struct
import base64

state = 0
i = 0
mlen = 0
tim = 0
mid = 0
noise = 0
buff=""
 
def pktError(s):
  print ("10s %d %s")%( "PktError",0,s)

msgFmts = {
  0x02 : ("<HLHHxx", "EVR"),
  0x08 : ("<HLHHxx", "State"),
  0x10 : ("<HLBBB", "RawPres"),
  0x11 : ("<HLHB", "RawTemp"),
  0x12 : ("<HLBBLLLLLLLLxx", "RawGPS"),
  0x18 : ("<HLfff", "ACC100Hz"),
  0x20 : ("<HLffffxx", "HSF"),
  0x21 : ("<HLhhhHHHhhhhhh", "BMP180"),
  0x22 : ("<HLfxx", "Height"),
}

msgLenths = dict([ (i, 4*int((struct.calcsize(msgFmts[i][0])+2)/3)) for i in msgFmts ])

p2f = {
  "L": "d", "l": "d",
  "H": "d", "h": "d",
  "B": "d", "b": "d",
  "c": "s", "f": "f",
  }

zeros = ("\0" *128)
outstyle = 1

def pkt_decode(m, bs) :
  #print "{",len(s)
  s = base64.decodestring(bs) 

  if (outstyle==2) :
    sys.stdout.write( s[0:128] +zeros )

  if (outstyle==1) :
    f = msgFmts[m][0]
#    print "m=",m," f=",f, "s=",bs, "#f=", struct.calcsize(f), "#s=", len(s)
    x = struct.unpack(f,s)

    if m == 0x10 :
      (m,t,x,y,z) = x
      x = (m,t, ((z<<16) | (y<<8) | x) )
      f = "HLL"

    p = " ".join([ "%"+p2f[i] for i in f if p2f.get(i) ])
    sys.stdout.write( ( "%10s " % msgFmts[m][1]) + (p % x) + "\n" )

def step_pkt_rd(x) :
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
      step_pkt_rd(x)
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
      step_pkt_rd(x)
    else:
      buff += x
      i = i + 1
      if i >= mlen :
        pkt_decode(mid, buff)
        state = 0
        mlen = 0

def rd():
  x = " "
  while x != "" :
    x = sys.stdin.read(1)
    if x != "" :
      step_pkt_rd(x)

rd()
