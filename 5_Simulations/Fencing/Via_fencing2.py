import matplotlib.pyplot as plt

# Dimensions (all in mm)
line_width = 0.204
via_pad_A = 0.20
via_pad_B = 0.45
polygon_offset = 0.30
via_pitch = 0.50  # center-to-center spacing
via_center_offset = 0.50  # RF line edge to via center

# RF line and fence representation
fig, ax = plt.subplots(figsize=(8, 4))

# Draw RF line
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
polygon_y1 = rf_line_y + line_width / 2 + polygon_offset
polygon_y2 = rf_line_y - line_width / 2 - polygon_offset
ax.axhline(polygon_y1, color="blue", linestyle="--", label="Polygon edge")
ax.axhline(polygon_y2, color="blue", linestyle="--")

# Draw vias (case A and case B)
via_positions = [2, 2 + via_pitch]  # two vias for showing spacing
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

# Add text annotations
ax.text(0.5, rf_line_y + line_width / 2 + 0.15, "0.30 mm", color="blue")
ax.text(0.5, rf_line_y, "0.204 mm line", color="black")
ax.text(2, polygon_y1 + 0.4, "Via A Ø0.20 mm pad", color="green")
ax.text(-2, polygon_y1 + 0.5, "Via B Ø0.45 mm pad", color="red")

# Add pitch dimension (horizontal between vias)
ax.annotate(
    "",
    xy=(2, polygon_y1 + 0.2),
    xytext=(2 + via_pitch, polygon_y1 + 0.2),
    arrowprops=dict(arrowstyle="<->", color="purple"),
)
ax.text(
    2 + via_pitch / 2, polygon_y1 + 0.3, f"{via_pitch:.2f} mm pitch", color="purple", ha="center"
)

# Add distance from RF line edge to via center
line_edge_y = rf_line_y + line_width / 2
via_center_y = polygon_y1
ax.annotate(
    "",
    xy=(2.4, line_edge_y),
    xytext=(2.4, via_center_y),
    arrowprops=dict(arrowstyle="<->", color="brown"),
)
ax.text(
    2.5, (line_edge_y + via_center_y) / 2, f"{via_center_offset:.2f} mm", color="brown", va="center"
)

# Formatting
ax.set_xlim(-5, 5)
ax.set_ylim(-2, 2)
ax.set_aspect("equal", adjustable="box")
ax.axis("off")
ax.legend(loc="upper right")
plt.title("Via Fence Setup for 10.5 GHz Microstrip Line (Pitch + Offset)")

plt.savefig("via_fence_setup_pitch_offset.png", dpi=300, bbox_inches="tight")
plt.close()
