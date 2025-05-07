import numpy as np
from matplotlib import pyplot as plt

class Logger():
    def __init__(self, initial):
        self.log = {}
        for item in initial:
            self.log[item] = []

    def log_data(self, plot_name, plot_data):
        self.log[plot_name].append(plot_data)

    def initialize_plot_group(self, frequency=1):
        self.frequency = frequency
        #self.keys_to_plot = keys_to_plot

        # Raggruppa per prefisso (es. com, torque, etc.)
        grouped_keys = {}
        for key in self.log.keys():
            prefix = key.split("_")[0]
            grouped_keys.setdefault(prefix, []).append(key)

        self.grouped_keys = grouped_keys

        self.fig, self.ax = plt.subplots(len(grouped_keys), 1, figsize=(6, 4 * len(grouped_keys)))
        if len(grouped_keys) == 1:
            self.ax = [self.ax]  # ensure list-like

        self.lines = {}
        for i, (prefix, keys) in enumerate(grouped_keys.items()):
            for key in keys:
                self.lines[key], = self.ax[i].plot([], [], label=key)
            self.ax[i].set_title(prefix)
            self.ax[i].legend()
            self.ax[i].grid(True)

        plt.ion()
        plt.tight_layout()
        plt.show()

    def update_plot_group(self, time):
        if time % self.frequency != 0:
            return

        # Raggruppa chiavi per prefisso (prima parte, separata da "_")
        grouped_keys = {}
        #for key in self.keys_to_plot:
        for key in self.log.keys():
            prefix = key.split("_")[0]
            grouped_keys.setdefault(prefix, []).append(key)

        for i, (prefix, keys) in enumerate(grouped_keys.items()):
            for key in keys:
                if key not in self.log:
                    continue
                data = self.log[key]
                self.lines[key].set_data(np.arange(len(data)), data)

            self.ax[i].relim()
            self.ax[i].autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def initialize_plot(self, keys_to_plot, frequency=1):
        self.frequency = frequency
        self.keys_to_plot = keys_to_plot

        self.fig, self.ax = plt.subplots(len(keys_to_plot), 1, figsize=(6, 4 * len(keys_to_plot)))

        if len(keys_to_plot) == 1:
            self.ax = [self.ax]  # Ensure list-like structure

        self.lines = {}
        for i, key in enumerate(keys_to_plot):
            self.lines[key], = self.ax[i].plot([], [], label=key)
            self.ax[i].set_title(key)
            self.ax[i].legend()
            self.ax[i].grid(True)

        plt.ion()
        plt.tight_layout()
        plt.show()

    def update_plot(self, time):
        if time % self.frequency != 0:
            return

        for i, key in enumerate(self.keys_to_plot):
            if key not in self.log:
                continue
            data = self.log[key]
            self.lines[key].set_data(np.arange(len(data)), data)
            self.ax[i].relim()
            self.ax[i].autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
