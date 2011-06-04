all: Final.pdf

Final.pdf: todo.tex block_diagram.pdf run-loop6.pdf hardware_labels_small.png software_flow.pdf

hardware_labels_small.png: hardware_labels.png
	convert -resize 1500x1500 $< $@

todo.tex: firmware.tex hardware.tex software.tex thoughts.tex ui.tex bib.tex \
	hardware-design.tex software-design.tex firmware-design.tex testing.tex
	grep -h TODO $^ | perl -ne 'print "$$_\n";' > todo.tex

%.pdf: %.tex
	pdflatex $<
	pdflatex $<
