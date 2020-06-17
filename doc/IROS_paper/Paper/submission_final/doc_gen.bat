del /s /f *.ps *.dvi *.aux *.toc *.idx *.ind *.ilg *.log *.out *.brf *.blg *.bbl

pdflatex bare_conf.tex
bibtex	 bare_conf
pdflatex bare_conf.tex
pdflatex bare_conf.tex
del /s /f *.ps *.dvi *.aux *.toc *.idx *.ind *.ilg *.log *.out *.brf *.blg *.bbl