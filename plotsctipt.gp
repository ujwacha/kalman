# Set graph title and labels
set title "Time vs f(t) and noise"
set xlabel "Time"
set ylabel "f(t)"
set ylabel "Noise"
set ylabel "Kalman_Filtered"


# Optional: Customize appearance
set grid
set key right top  # Move legend to the top-right corner

# Plot the data
plot "data.txt" using 1:2 with lines title "f(t)", \
     "data.txt" using 1:3 with lines  title "noise", \
     "data.txt" using 1:4 with lines  title "kalman"

set terminal png
set output "plot.png"
replot  # Save the graph to the file
