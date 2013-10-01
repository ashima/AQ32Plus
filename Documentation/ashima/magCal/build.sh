#!/bin/sh
export TEXINPUTS=.:..:
pdflatex main.tex && pdflatex main.tex && pdflatex main.tex

