#!/bin/sh
export TEXINPUTS=.:..:

N=`basename $PWD`
echo N = $N

C="pdflatex -jobname $N -output-directory $1 -halt-on-error main.tex "
echo C = $C

$C && $C && $C
#pdflatex main.tex && pdflatex main.tex && pdflatex main.tex

