all: covariance battery-2011-05-21-14\:01\:45.png battery-2011-05-24.png

run:
	./covariance

%.png: %.log battery.plot
	gnuplot -e "in='$<'" battery.plot
