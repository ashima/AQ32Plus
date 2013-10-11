#  \file       genMcAccumulate.py
#  \brief      Code generator for accumulating data for an ellipsoidal fit for
#              for magnetometer calibration.
#  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
#              file distributed under the MIT Expat License. See LICENSE file.
#              https://github.com/ashima/AQ32Plus
#  \remark     Ported for AQ32Plus.
#
import argparse
from itertools import *
from simpSeq import *

#------------------------ ----------------------------------------------
# I Really ought to be building an AST and emitting gen'd code from there,
# that would reduce the amount of duplication and testing needed,
# but for now just do it as text...

def procargs() :
  p = argparse.ArgumentParser( description="Generate magCal accumulation code")
  p.add_argument("-l", dest='lang', choices=[ 'cc', 'matlab' ], 
                 help="output language" )
  p.add_argument("-i", dest='i',type=int, help="language dependent parameter" )
  return p.parse_args()

def tonames(w):
  return [ "xyz"[i] for i in w if i<3 ]

def lineComment(c,arr) :
  return c + ("\n"+c).join(arr) + "\n"

#----------------------------------------------------------------------
# Matlab emitter

def emitMatlab_CollectA( x1, x2 ) :
  out = ""
  out += "function [A,cp] = mcCollectA(u)\n"
  out += "  A  = zeros( %d, %d );\n" % (len(x1),len(x1))
  out += "  cp = zeros( %d, 1);\n" % (len(x1))
  for (i,j) in x2[:-3] :
    if i != j :
      out += "  A(1+%d,1+%d) = \\\n" % ( j, i )
    out += "  A(1+%d,1+%d) = u(1+%d); # %s\n" % (
            i, j, inx((i,j)), "".join(tonames((i,j)) ))
  out += "\n"
  for (i,j) in x2[-3:] :
    out += "  cp(1+%d,1) = u(1+%d); # %s\n" % (
            i, inx((i,j)), "".join(tonames((i,j)) ))
  out += "\nend\n"   
  return out
  
def emitMatlab_CollectO(X2, x2x2, x4, OmegaTOmega,f) :
  out = ""
  out += "function [OmegaTOmega, Omega1] = mcCollectO(accOmega)\n"
  
  for ((i,j),w) in zip(x2x2, OmegaTOmega) :
    ff = f[i]*f[j]
    if ff != 0 :
      out += "  OmegaTOmega(1+%d,1+%d) = %d * accOmega(1+%d) ; # %s\n" % (
              i,j, f[i]*f[j], inx(w), "".join(tonames(w)) )
  
  out += "\n\n"
  
  for i in X2 :
    if f[i] != 0 :
      w = loc_2(i)+(3,3)
      out += ("  Omega1(1+%d) = %d * accOmega(1+%d); # %s\n" % (
                i, f[i], inx(w),"".join(tonames(w)) ) )
  out += "\nend\n"
  return out

def emitMatlab_Accumulate(x4) :
  out = ""
  out += "function mcAccumulate(x,y,z)\n"
  out += "  global accOmega\n"
  for s in reversed(x4) :
    out += "  accOmega(1+%d) += %s ;\n" % (inx(s) , " * ".join(tonames(s)) )

  out += "\nend\n"
  return out

#----------------------------------------------------------------------
# C++ emitter

def emitC_CollectO(X2, x2x2, x4, OmegaTOmega,f) :
  out = ""
  out += """
void mcCollectO( matrix<double,%2d,%2d> &OmegaTOmega,
                  matrix<double,%2d, 1> &Omega1,
                  matrix<double,%2d, 1> &accOmega )
  {\n""" % ( len(X2), len(X2), len(X2), len(x4) )
  
  for ((i,j),w) in zip(x2x2, OmegaTOmega) :
    ff = f[i]*f[j]
    if ff != 0 :
      out += "  OmegaTOmega(%2d,%2d) = %d * accOmega(%2d,0) ; // %s\n" % (
              i,j, f[i]*f[j], inx(w), "".join(tonames(w)) )
  
  out += "\n\n"
  
  for i in X2 :
    if f[i] != 0 :
      w = loc_2(i)+(3,3)
      out += ("  Omega1(%2d,0) = %d * accOmega(%2d,0); // %s\n" % (
                i, f[i], inx(w),"".join(tonames(w)) ) )
  out += "\n  }\n"

  return out

def emitC_CollectA( x1, x2 ) :
  out = ""
  out += """
void mcCollectA( matrix<double,%2d,%2d> &A, matrix<double,%2d, 1> &cp,
                  matrix<double,%2d, 1> &u )
  {\n""" % (len(x1), len(x1), len(x1), len(x2) )
  for (i,j) in x2[:-len(x1)] :
    if i != j :
      out += "  A(%2d,%2d) = \n" % ( j, i )
    out += "  A(%2d,%2d) = u(%2d,0); // %s\n" % (
            i, j, inx((i,j)), "".join(tonames((i,j)) ))
  out += "\n"
  for (i,j) in x2[-len(x1):] :
    out += "  cp(%2d,0) = u(%2d,0); // %s\n" % (
            i, inx((i,j)), "".join(tonames((i,j)) ))
  out += "  }\n" 
  
  return out

def emitC_Accumulate(x4) :
  out = ""
  out += """
void mcAccumulate(matrix<double,%d,1> &accOmega,
                  float x, float y, float z)
  {\n""" % len(x4)
  for s in reversed(x4) :
    out += "  accOmega(%2d,0) += %s ;\n" % (inx(s) , " * ".join(tonames(s)) )

  out += "  }\n"

  return out
#---------------------------------------------------------------------- 
#Generate index relationships :

X1   = range(4)
I1   = simpSeq(1, X1) 
I2   = simpSeq(2, X1) # These are the unique combinations in Omega
I4   = simpSeq(4, X1) # These are the unique combinations in accOmega.
  
X2   = map(inx, I2)
I2I2 = simpSeq(2, X2)

# This maps Omega^T Omega into accOmega
I4ofI2I2 = [ sorted( loc_2(i) + loc_2(j)) for (i,j) in I2I2 ]

f = [ 1,2,1,2,2,1,2,2,2,0 ]

hd = [
"\\brief     This is an AUTOMATICALLY GENERATED file for accumulating data",
"           for an ellipsoidal fit for magnetometer calibration.",
"\\copyright Copyright (C) 2013 Ashima Research. All rights reserved. This",
"           file distributed under the MIT Expat License. See LICENSE file.",
"           https://github.com/ashima/AQ32Plus",
"\\remark    Do not edit this file by hand!",
"\\remark    Generated by genMcAccumulate.py",
]

args = procargs()
  
# Kludgey, but quick.
if args.lang == 'matlab'  :
  print lineComment("% ", hd)
  if args.i == 1 :
    print emitMatlab_CollectO(X2[:-1], I2I2[:-1], I4[:-1], I4ofI2I2[:-1],f)
  if args.i == 2 :
    print emitMatlab_Accumulate(I4[:-1])
  if args.i == 3 :
    print emitMatlab_CollectA(X1[:-1], I2[:-1])

elif args.lang == 'cc'  :
  print lineComment("// ", hd)
  print "#include <inttypes.h>"
  print "#include <matDumb.h>\n"
  print emitC_CollectO(X2[:-1], I2I2[:-1], I4[:-1], I4ofI2I2[:-1],f)
  print emitC_Accumulate(I4[:-1])
  print emitC_CollectA(X1[:-1], I2[:-1])

