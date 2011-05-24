all: Final.pdf

Final.pdf: todo.tex

todo.tex: firmware.tex hardware.tex software.tex thoughts.tex ui.tex bib.tex \
	hardware-design.tex software-design.tex firmware-design.tex testing.tex
	grep -h TODO $^ | perl -ne 'print "$$_\n";' > todo.tex

%.pdf: %.tex
	pdflatex $<
