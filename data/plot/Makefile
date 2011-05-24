TRG=run-loop1.png run-loop2.png run-loop2.svg run-loop3.png run-loop4.png run-loop5.png run-loop6.png run-loop6.svg run-loop6.huge.png
all: $(TRG)

clean:
	rm $(TRG)

%.tmp: %.log
	perl plot.pl $^ > $@

run-loop4.tmp: run-loop4.log plot-4.pl
	perl plot-4.pl $< > $@

run-loop5.tmp: run-loop5.log plot-4.pl
	perl plot-4.pl $< > $@

run-loop6.tmp: run-loop6.log plot-4.pl
	perl plot-4.pl $< > $@

%.png: %.tmp
	gnuplot -e 'set terminal png size 1600,1200; set output "$@"; plot "$^" using 1:2:(sin($$4)):(cos($$4)) with vectors, "$^" using 1:2 with lines, "$^" using 1:2:(cos($$3)):(sin($$3)) with vectors;'

%.svg: %.tmp
	gnuplot -e 'set terminal svg; set output "$@"; plot "$^" using 1:2:(sin($$4)):(cos($$4)) with vectors, "$^" using 1:2 with lines, "$^" using 1:2:(cos($$3)):(sin($$3)) with vectors;'

%.huge.png: %.tmp
	gnuplot -e 'set terminal png size 3200,2400; set output "$@"; plot "$^" using 1:2:(sin($$4)):(cos($$4)) with vectors, "$^" using 1:2 with lines, "$^" using 1:2:(cos($$3)):(sin($$3)) with vectors;'

%.pdf: %.tmp
	gnuplot -e 'set terminal pdf; set output "$@"; plot "$^" using 1:2:(sin($$4)):(cos($$4)) with vectors, "$^" using 1:2 with lines, "$^" using 1:2:(cos($$3)):(sin($$3)) with vectors;'

