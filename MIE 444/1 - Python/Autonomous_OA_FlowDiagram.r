library(ggplot2)
library(grid)

# Create a base plot for the flowchart
plot_flowchart <- ggplot() + 
  xlim(0, 10) + 
  ylim(-4, 18) + 
  theme_void()

# Helper function to draw rectangles with text
draw_rectangle <- function(x, y, width, height, text, color) {
  plot_flowchart <<- plot_flowchart +
    annotate(
      "rect",
      xmin = x - width / 2,
      xmax = x + width / 2,
      ymin = y - height / 2,
      ymax = y + height / 2,
      fill = color,
      color = "black",
      size = 0.5
    ) +
    annotate(
      "text",
      x = x,
      y = y,
      label = text,
      size = 5,
      hjust = 0.5,
      vjust = 0.5
    )
}

# Helper function to draw arrows
draw_arrow <- function(x1, y1, x2, y2, color = "black") {
  plot_flowchart <<- plot_flowchart +
    geom_segment(
      aes(x = x1, y = y1, xend = x2, yend = y2),
      arrow = arrow(type = "closed", length = unit(0.2, "cm")),
      color = color,
      size = 0.8
    )
}

# Draw flowchart elements
draw_rectangle(5, 16.5, 2.5, 1, "Start: Initialize\nthresholds and history", "lightgreen")
draw_arrow(5, 16, 5, 15.5)
draw_rectangle(5, 15, 2.5, 1, "Read sensor data:\n(front, right, left, back, fr, fl)", "lightblue")
draw_arrow(5, 14.5, 5, 13.5)
draw_rectangle(5, 13, 2, 1, "Detect obstacles:\nCheck thresholds", "lightyellow")
draw_arrow(5, 12.5, 5, 11.5)
# Adding thresholds on the side 
draw_rectangle(2, 13, 2.5, 2, "Thresholds:\n Front/Back = 1.5'\nDiagonal Left/Right = 3.5'\nLeft/Right = 1'", "lightgreen")

draw_rectangle(5, 11, 2, 1, "Fetch last move\nfrom history", "lightblue")
draw_arrow(5, 10.5, 5, 9.5)

# Decision point for obstacles
draw_rectangle(5, 9, 4, 1, "Obstacles detected?", "lightyellow")
draw_arrow(5, 8.5, 2.5, 7.5)
draw_arrow(5, 8.5, 7.5, 7.5)

# If obstacles detected
draw_rectangle(2.5, 7, 3, 1, "If last move =\nturn left/right, repeat it", "orange")
draw_arrow(2.5, 6.5, 2.5, 5.5)
draw_rectangle(2.5, 5, 3, 1, "Else: Compare (right+fr)/2\nwith (left+fl)/2", "orange")
draw_arrow(2.5, 4.5, 2.5, 3.5)
draw_rectangle(2.5, 3, 3, 1, "Decide turn\nleft or right", "orange")
draw_arrow(2.5, 2.5, 5, 1.5)

# If no obstacles detected
draw_rectangle(7.5, 7, 3, 1, "Compare left,\nfront, right", "orange")
draw_arrow(7.5, 6.5, 7.5, 5.5)
draw_rectangle(7.5, 5, 3, 1, "If left > front/right:\nrotate left", "orange")
draw_arrow(7.5, 4.5, 7.5, 3.5)
draw_rectangle(7.5, 3, 3, 1, "Else: Move forward", "orange")
draw_arrow(7.5, 2.5, 5, 1.5)

# Add yes or no logic 
draw_rectangle(1.5, 8.25, 0.75, 1, "YES", "violet")
draw_rectangle(8.5, 8.25, 0.75, 1, "NO", "violet")
# Common path after decision
draw_rectangle(5, 1, 2, 1, "Update movement\nhistory", "lightcoral")
draw_arrow(5, 0.5, 5, 0)
draw_rectangle(5, 0, 2, 1, "Execute next move", "lightgreen")
draw_arrow(5, -0.5, 5, -1.5)
draw_rectangle(5, -2, 2.5, 1, "Repeat until goal\nor stopping condition", "lightblue")

# Plot the flowchart
plot_flowchart
# Print the flowchart
print(plot_flowchart)

# Save the plot as PNG
ggsave("flowchart_plot.png", plot = plot_flowchart, width = 10, height = 18, dpi = 300)

# Output message
cat("Flowchart saved as flowchart_plot.png")