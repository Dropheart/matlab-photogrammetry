#!/bin/sh

for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36
do
        INPUT="rok$i.HEIC"
	OUTPUT="rok$i.png"
	echo "heif-convert -q 100 $INPUT $OUTPUT"
	heif-convert -q 100 $INPUT $OUTPUT
done
