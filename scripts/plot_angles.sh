gnuplot -e " set term png; set output 'out.png'; plot 'out.txt' u 0:1 with linespoints ls 1, 'out.txt' u 0:2 with linespoints ls 2; "
