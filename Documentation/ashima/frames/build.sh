#!/bin/sh
export TEXINPUTS=.:..:

N=`basename $PWD`
echo N = $N

C="pdflatex -halt-on-error -jobname $N main.tex"
echo C = $C

$C && $C && $C
#pdflatex main.tex && pdflatex main.tex && pdflatex main.tex

