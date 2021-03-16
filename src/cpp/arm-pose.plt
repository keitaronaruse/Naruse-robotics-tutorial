set xrange[-3:3]
set yrange[-3:3]
set size square
set size ratio 1
plot "arm-pose.txt" using 1:2 w lp
