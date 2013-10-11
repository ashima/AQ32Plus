from itertools import *
from math import sqrt,factorial

def le(x) :
  return len(x) < 2 or ( x[0] <= x[1] and le( x[1:] ))

def simpSeq(d,l) :
  return filter(le, [ list(reversed(u)) for u in product(*[l]*d) ] )

def simp(d,h) :
  return h if h <= 1 else (factorial(d+h-1) / (factorial(d)*factorial(h-1)))

def inx(i) :
  l = len(i)
  return i[0] if l == 1 else simp(l, i[-1]) + inx( i[:-1] )

def isimp_2(n):
  return int(sqrt(1.0 + 8.0 * n)-1.0)/2

def loc_2(n):
  j = isimp_2(n)
  return ( n - simp(2,j) , j)


