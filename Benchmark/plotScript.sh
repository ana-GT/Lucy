#!/bin/bash

DAT1="Joints.dat"

echo 'plot "'$DAT1'" using 1 title "1" with lines, "'$DAT1'" using 2 title "2" with lines, "'$DAT1'" using 3 title "3" with lines,"'$DAT1'" using 4 title "4" with lines,"'$DAT1'" using 5 title "5" with lines,"'$DAT1'" using 6 title "6" with lines,"'$DAT1'" using 7 title "7" with lines ' | gnuplot -persist

DAT2="CollidingJoints.dat"

echo 'plot "'$DAT2'" using 1 title "1" with lines, "'$DAT2'" using 2 title "2" with lines, "'$DAT2'" using 3 title "3" with lines,"'$DAT2'" using 4 title "4" with lines,"'$DAT2'" using 5 title "5" with lines,"'$DAT2'" using 6 title "6" with lines,"'$DAT2'" using 7 title "7" with lines ' | gnuplot -persist
