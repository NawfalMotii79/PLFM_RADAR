import matplotlib.pyplot as plt

# Dimensions (all in mm)
line_width = 0.204
substrate_height = 0.102
via_drill = 0.20
via_pad_A = 0.20  # minimal pad case
via_pad_B = 0.45  # robust pad case
spacing_via_center_to_edge = 0.50

# RF line and fence representation
fig, ax = plt.subplots(figsize=(8, 4))

# Draw RF line (centered at y=0)
rf_line_y = 0
ax.add_patch(
    plt.Rectangle(
        (-5, rf_line_y - line_width / 2),
        10,
        line_width,
        facecolor="orange",
        edgecolor="black",
        label="RF Line",
    )
)

# Draw ground plane edges (polygon edge at 0.30 mm from RF line edge)
polygon_offset = 0.30
polygon_y1 = rf_line_y + line_width / 2 + polygon_offset
polygon_y2 = rf_line_y - line_width / 2 - polygon_offset
ax.axhline(polygon_y1, color="blue", linestyle="--", label="Polygon edge")
ax.axhline(polygon_y2, color="blue", linestyle="--")

# Draw vias (case A and case B)
via_positions = [2, 4, 6, 8]  # x positions for visualization
for x in via_positions:
    # Case A
    ax.add_patch(
        plt.Circle(
            (x, polygon_y1),
            via_pad_A / 2,
            facecolor="green",
            alpha=0.5,
            label="Via pad A" if x == 2 else "",
        )
    )
    ax.add_patch(plt.Circle((x, polygon_y2), via_pad_A / 2, facecolor="green", alpha=0.5))
    # Case B
    ax.add_patch(
        plt.Circle(
            (-x, polygon_y1),
            via_pad_B / 2,
            facecolor="red",
            alpha=0.3,
            label="Via pad B" if x == 2 else "",
        )
    )
    ax.add_patch(plt.Circle((-x, polygon_y2), via_pad_B / 2, facecolor="red", alpha=0.3))

# Add dimensions text
ax.text(0.5, rf_line_y + line_width / 2 + 0.15, "0.30 mm", color="blue")
ax.text(0.5, rf_line_y, "0.204 mm line", color="black")
ax.text(2, polygon_y1 + 0.4, "Via A Ø0.20 mm pad", color="green")
ax.text(-2, polygon_y1 + 0.5, "Via B Ø0.45 mm pad", color="red")

# Formatting
ax.set_xlim(-10, 10)
ax.set_ylim(-2, 2)
ax.set_aspect("equal", adjustable="box")
ax.axis("off")
ax.legend(loc="upper right")
plt.title("Via Fence Setup for 10.5 GHz Microstrip Line")

plt.savefig("/mnt/data/via_fence_setup.png", dpi=300, bbox_inches="tight")
plt.close()
