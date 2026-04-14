import matplotlib.pyplot as plt

# Dimensions (all in mm)
line_width = 0.204
substrate_height = 0.102
via_drill = 0.20
via_pad_A = 0.20
via_pad_B = 0.45
spacing_via_center_to_edge = 0.50

fig, ax = plt.subplots(figsize=(10, 5))

# RF line
rf_line_y = 0
ax.add_patch(plt.Rectangle((-5, rf_line_y - line_width/2), 10, line_width,
                           facecolor="orange", edgecolor="black", label="RF Line"))

# Polygon edges
polygon_offset = 0.30
polygon_y1 = rf_line_y + line_width/2 + polygon_offset
polygon_y2 = rf_line_y - line_width/2 - polygon_offset
ax.axhline(polygon_y1, color="blue", linestyle="--", label="Polygon edge")
ax.axhline(polygon_y2, color="blue", linestyle="--")

# Via positions
via_positions = [2, 4, 6, 8]

for x in via_positions:
    # Case A pads
    ax.add_patch(plt.Circle((x, polygon_y1), via_pad_A/2, facecolor="green", alpha=0.5))
    ax.add_patch(plt.Circle((x, polygon_y2), via_pad_A/2, facecolor="green", alpha=0.5))
    
    # Drill holes (inside pads)
    ax.add_patch(plt.Circle((x, polygon_y1), via_drill/2, facecolor="black"))
    ax.add_patch(plt.Circle((x, polygon_y2), via_drill/2, facecolor="black"))

    # Case B pads
    ax.add_patch(plt.Circle((-x, polygon_y1), via_pad_B/2, facecolor="red", alpha=0.3))
    ax.add_patch(plt.Circle((-x, polygon_y2), via_pad_B/2, facecolor="red", alpha=0.3))
    
    ax.add_patch(plt.Circle((-x, polygon_y1), via_drill/2, facecolor="black"))
    ax.add_patch(plt.Circle((-x, polygon_y2), via_drill/2, facecolor="black"))

# ----------- Dimension Arrows -----------

# RF line width
ax.annotate("", xy=(0, rf_line_y + line_width/2), xytext=(0, rf_line_y - line_width/2),
            arrowprops=dict(arrowstyle="<->"))
ax.text(0.2, rf_line_y, "0.204 mm", va='center')

# Spacing to polygon
ax.annotate("", xy=(1, rf_line_y + line_width/2),
            xytext=(1, polygon_y1),
            arrowprops=dict(arrowstyle="<->", color='blue'))
ax.text(1.2, (rf_line_y + polygon_y1)/2, "0.30 mm", color="blue")

# Via pitch
ax.annotate("", xy=(2, polygon_y1 + 0.8),
            xytext=(4, polygon_y1 + 0.8),
            arrowprops=dict(arrowstyle="<->"))
ax.text(3, polygon_y1 + 0.9, "Via pitch", ha='center')

# Substrate height annotation
ax.text(-9, -1.5, f"Substrate height = {substrate_height} mm", fontsize=10)

# Labels
ax.text(2, polygon_y1 + 0.5, "Via A (small pad)", color="green")
ax.text(-4, polygon_y1 + 0.6, "Via B (large pad)", color="red")

# Formatting
ax.set_xlim(-10, 10)
ax.set_ylim(-2, 2)
ax.set_aspect('equal')
ax.grid(True, linestyle=':', alpha=0.5)
ax.axis("off")

plt.title("Enhanced Via Fence Setup for 10.5 GHz Microstrip Line")

plt.savefig("/mnt/data/via_fence_enhanced.png", dpi=300, bbox_inches="tight")
plt.show()ax.text(0.5, rf_line_y + line_width/2 + 0.15, "0.30 mm", color="blue")
ax.text(0.5, rf_line_y, "0.204 mm line", color="black")
ax.text(2, polygon_y1 + 0.4, "Via A Ø0.20 mm pad", color="green")
ax.text(-2, polygon_y1 + 0.5, "Via B Ø0.45 mm pad", color="red")

# Formatting
ax.set_xlim(-10, 10)
ax.set_ylim(-2, 2)
ax.set_aspect('equal', adjustable='box')
ax.axis("off")
ax.legend(loc="upper right")
plt.title("Via Fence Setup for 10.5 GHz Microstrip Line")

plt.savefig("/mnt/data/via_fence_setup.png", dpi=300, bbox_inches="tight")
plt.close()

