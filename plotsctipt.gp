# Set graph title and labels
set title "Kalman Filter Simulation"
set xlabel "Time"
set ylabel "Pendulum Theta"
set ylabel "Noisy Measurement"
set ylabel "Kalman Filtered Data"


# Optional: Customize appearance
set grid
set key right top  # Move legend to the top-right corner

# Plot the data
plot "data.txt" using 1:2 with lines title "Pendulum Theta", \
     "data.txt" using 1:3 with lines  title "Noisy Measurement", \
     "data.txt" using 1:4 with lines  title "Kalman Filtered Data",

set terminal png
set output "plot.png"
replot  # Save the graph to the file
