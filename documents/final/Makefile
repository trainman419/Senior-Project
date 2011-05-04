all: Final.pdf

Final.pdf: todo.tex

todo.tex: firmware.tex hardware.tex software.tex thoughts.tex ui.tex
	grep -h TODO $^ > todo.tex

%.pdf: %.tex
	pdflatex $<
