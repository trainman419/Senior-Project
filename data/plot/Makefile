TRG=run-loop1.png run-loop2.png run-loop2.svg
all: $(TRG)

clean:
	rm $(TRG)
	rm run-loop1.tmp run-loop2.tmp

%.tmp: %.log
	perl plot.pl $^ > $@

%.png: %.tmp
	gnuplot -e 'set terminal png size 1600,1200; set output "$@"; plot "$^" using 1:2:(sin($$4)):(cos($$4)) with vectors, "$^" using 1:2 with lines, "$^" using 1:2:(sin($$3)):(cos($$3)) with vectors;'

%.svg: %.tmp
	gnuplot -e 'set terminal svg; set output "$@"; plot "$^" using 1:2:(sin($$4)):(cos($$4)) with vectors, "$^" using 1:2 with lines, "$^" using 1:2:(sin($$3)):(cos($$3)) with vectors;'
