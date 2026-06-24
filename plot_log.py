#!/usr/bin/env python3
import sys
import os
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TARGET = 0.3
TOL = 0.1
OUT = "assets/wallfollower_plot.png"


def main():
    if len(sys.argv) < 2:
        print("Aufruf: python3 plot_log.py <logdatei.csv>")
        sys.exit(1)

    path = sys.argv[1]
    data = pd.read_csv(path)
    data = data[data["distance_front"] > 0.001].copy()

    t = (data["timestamp"] - data["timestamp"].iloc[0]).to_numpy()
    right = data["distance_right"].to_numpy()

    low = TARGET - TOL
    high = TARGET + TOL
    in_band = ((right >= low) & (right <= high)).mean() * 100

    fig, ax = plt.subplots(figsize=(9, 3.4), dpi=130)
    ax.axhspan(low, high, color="#2a9d8f", alpha=0.15,
               label="Zielband {0} +/- {1} m".format(TARGET, TOL))
    ax.axhline(TARGET, color="#2a9d8f", linestyle="--", linewidth=1)
    ax.plot(t, right, color="#264653", linewidth=0.9, label="gemessener Wandabstand rechts")
    ax.set_xlabel("Zeit [s]")
    ax.set_ylabel("Abstand rechts [m]")
    ax.set_title("Wall Follower")
    ax.set_ylim(0, 1.2)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.25)
    fig.tight_layout()

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT)
    print("Gespeichert nach {0}".format(OUT))
    print("{0:.0f} Prozent der Zeit im Zielband, {1} Messpunkte".format(in_band, len(right)))


if __name__ == "__main__":
    main()