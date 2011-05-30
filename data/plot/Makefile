TRG=run-loop1.png run-loop2.png run-loop2.svg run-loop3.png run-loop4.png run-loop5.png run-loop6.png run-loop6.svg run-loop6.pdf run-2011-05-24-1.png run-2011-05-24-2.png run-2011-05-24-3.png run-2011-05-30-1.png run-2011-05-30-2.png run-2011-05-30-3.png run-2011-05-30-1.pdf
all: $(TRG)

clean:
	rm $(TRG)

run-loop4.tmp: run-loop4.log plot-4.pl
	perl plot-4.pl $< > $@

run-loop5.tmp: run-loop5.log plot-4.pl
	perl plot-4.pl $< > $@

run-loop6.tmp: run-loop6.log plot-4.pl
	perl plot-4.pl $< > $@

%.tmp: %.log plot.pl
	perl plot.pl $< > $@

%.cir: %.logc circles.pl
	perl circles.pl $< > $@

%.com: %.logc combined.pl
	perl combined.pl $< > $@

#%.png: %.cir circles.plot
#	gnuplot -e "in='$<'; type='png'" circles.plot

%.png: %.cir %.com combined.plot
	gnuplot -e "in='$<'; type='png'" combined.plot

%.pdf: %.cir %.com combined.plot
	gnuplot -e "in='$<'; type='pdf'" combined.plot

%.png: %.tmp log.plot
	gnuplot -e "in='$<'; type='png'" log.plot

%.svg: %.tmp log.plot
	gnuplot -e "in='$<'; type='svg'" log.plot

%.pdf: %.tmp log.plot
	gnuplot -e "in='$<'; type='pdf'" log.plot

