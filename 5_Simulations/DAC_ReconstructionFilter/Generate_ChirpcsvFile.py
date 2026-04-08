import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def generate_multi_ramp_csv(
    Fs=125e6,
    Tb=1e-6,
    Tau=2e-6,
    fmax=30e6,
    fmin=10e6,
    Duration=6e-6,
    filename="multi_ramp_output.csv",
    show_plot=True,
    save_plot_png=None,
    plot_window=None,
    hold_per_sample=1,
):
    """
    Generate CSV with repeated frequency ramp (chirp) bursts and DAC-style stairs.

    One-ramp model (N = 0..n-1, Ts = 1/Fs, n = floor(Tb/Ts)):
        theta_n = 2*pi*( (N^2 * Ts^2 * (fmax-fmin) / (2*Tb)) + fmin*N*Ts )
        y_ramp  = 1 + sin(theta_n)

    Ramps repeat every Tau (zeros between ramps).

    Parameters:
      Fs, Tb, Tau, fmax, fmin, Duration: signal parameters
      filename        : CSV output filename
      show_plot       : show a time-domain plot
      save_plot_png   : if provided, save the plot to this PNG path
      plot_window     : None => plot full duration; float (s) => plot first window
      hold_per_sample : integer >=1.
                        1 => save raw DAC samples at Fs
                        >1 => ZOH expand: each sample repeated 'hold_per_sample' times
                             so the CSV itself is staircase-like.
    """
    # --- Derived quantities
    Ts = 1.0 / Fs
    n = int(np.floor(Tb / Ts))  # samples per ramp
    prf_samples = int(np.floor(Tau / Ts))  # samples per repetition period
    total_samples = int(np.floor(Duration / Ts))

    # Time vector for raw DAC samples
    t = np.arange(total_samples) * Ts

    # --- Build one ramp (chirp)
    N = np.arange(n)
    theta_n = 2.0 * np.pi * ((N**2) * (Ts**2) * (fmax - fmin) / (2.0 * Tb) + fmin * N * Ts)
    ramp = 1.0 + np.sin(theta_n)

    # --- Assemble repeated ramps (zero elsewhere)
    y = np.zeros(total_samples)
    idx = 0
    ramps_inserted = 0
    while idx + n <= total_samples:
        y[idx : idx + n] = ramp
        ramps_inserted += 1
        idx += prf_samples
        if prf_samples <= 0:
            break

    # --- ZOH expand for CSV if requested
    if hold_per_sample < 1:
        raise ValueError("hold_per_sample must be >= 1")
    if hold_per_sample == 1:
        t_csv = t
        y_csv = y
    else:
        # Repeat each sample 'hold_per_sample' times (constant within Ts)
        y_csv = np.repeat(y, hold_per_sample)
        Ta = Ts / hold_per_sample
        t_csv = np.arange(y_csv.size) * Ta

    # --- Save CSV (no header)
    df = pd.DataFrame({"time(s)": t_csv, "voltage(V)": y_csv})
    df.to_csv(filename, index=False, header=False)
    print(f"CSV saved: {filename}")
    print(
        f"Total raw samples: {total_samples} | Ramps inserted: {ramps_inserted} | CSV points: {len(y_csv)}"
    )

    # --- Plot (staircase)
    if show_plot or save_plot_png:
        # Choose plotting vectors (use raw DAC samples to keep lines crisp)
        t_plot = t
        y_plot = y

        # Determine window
        if plot_window is None:
            k = slice(0, t_plot.size)
        else:
            tmax = min(plot_window, Duration)
            k_idx = np.where(t_plot <= tmax)[0]
            if k_idx.size == 0:
                k = slice(0, min(t_plot.size, 1000))
            else:
                k = slice(k_idx[0], k_idx[-1] + 1)

        # Optional decimation for huge plots
        max_points = 200_000
        num_pts = (k.stop - k.start) if isinstance(k, slice) else k_idx.size
        if num_pts > max_points:
            step = int(np.ceil(num_pts / max_points))
            if isinstance(k, slice):
                k = slice(k.start, k.stop, step)

        plt.figure(figsize=(10, 4.5))
        # STEP PLOT => staircase appearance without fabricating extra samples
        plt.step(t_plot[k] * 1e6, y_plot[k], where="post", label="DAC ZOH (stairs)")
        plt.xlabel("Time (µs)")
        plt.ylabel("Amplitude")
        plt.title("Repeated chirp ramps (DAC-like staircase)")
        plt.grid(True)
        plt.legend(loc="upper right")
        plt.tight_layout()

        if save_plot_png:
            plt.savefig(save_plot_png, dpi=150)
            print(f"Plot saved: {save_plot_png}")
        if show_plot:
            plt.show()
        else:
            plt.close()


# ---------------- Example usage ----------------
if __name__ == "__main__":
    # Example: large duration, CSV with ZOH (staircase) samples
    generate_multi_ramp_csv(
        Fs=125e6,
        Tb=1e-6,
        Tau=2e-6,
        fmax=30e6,
        fmin=10e6,
        Duration=6e-6,  # try longer duration
        filename="multi_ramp_stairs.csv",
        show_plot=True,
        save_plot_png=None,
        plot_window=None,  # None => full duration
        hold_per_sample=1,  # set >1 to make CSV itself staircase (ZOH-expanded)
    )
