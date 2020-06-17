MD ".\__temp_latex"

(
echo \documentclass{book}
echo \author{Andrea Casalino}
echo \title{Debug}
echo \RequirePackage[margin=2cm]{geometry}
echo \geometry{left=2cm,right=2cm,marginparwidth=6.8cm,marginparsep=1.5cm,top=1.5cm,bottom=1.5cm,footskip=2\baselineskip}
echo \usepackage[T1]{fontenc}
echo \usepackage[utf8]{inputenc}
echo \usepackage[default]{lato}
echo \usepackage{graphicx,color, import}
echo \usepackage{amssymb, amsmath}
echo \usepackage{url}
echo \usepackage[]{algorithm2e}
echo \usepackage[toc,page]{appendix}
echo \usepackage{cite}
echo \begin{document}
echo \maketitle
echo \newpage
echo \input{../additional_Sections.tex}
echo \bibliography{../ref}{}
echo \bibliographystyle{plain}
echo \end{document}
)>".\__temp_latex\main.tex"

cd __temp_latex
pdflatex main.tex
bibtex 	 main
pdflatex main.tex
pdflatex main.tex

COPY "main.pdf" "..\documentation_no_doxy.pdf"
cd ..\
RD ".\__temp_latex" /S /Q
