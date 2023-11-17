#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   plots_ulg_files.py
@Time    :   2023/11/17 15:14:36
@Author  :   Roney D. Silva
@Contact :   roneyddasilva@gmail.com
"""

import locale

import matplotlib.pyplot as plt

# import numpy as np
import pandas as pd
import os

locale.setlocale(locale.LC_ALL, "pt_BR.UTF-8")
# plt.style.use("default")
plt.style.use("mpl.mplstyle")
my_colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
FIG_L = 6.29
FIG_A = (90.0) / 25.4

ulg_file = "19_49_06"
ulg_folder = "./../../build/px4_sitl_default/rootfs/log/2023-11-17/"
os.system("rm csv_files/*.csv")
os.system("ulog2csv --output csv_files " + ulg_folder + ulg_file + ".ulg")


def plot_attitude_states():
    veh_att_s = pd.read_csv(
        "csv_files/" + ulg_file + "_vehicle_attitude_setpoint_0.csv"
    )
    veh_att = pd.read_csv("csv_files/" + ulg_file + "_vehicle_attitude_0.csv")
    veh_ang = pd.read_csv("csv_files/" + ulg_file + "_vehicle_angular_velocity_0.csv")
    fig, ax = plt.subplots(3, 2, num=1, sharex=True, figsize=(FIG_L, FIG_A))
    ax[0, 0].plot(
        1e-6 * (veh_att_s["timestamp"] - veh_att_s["timestamp"][0]),
        veh_att_s["q_d[" + str(0) + "]"],
        "-o",
        label="Reference $q_0$",
    )
    ax[0, 0].plot(
        1e-6 * (veh_att["timestamp"] - veh_att["timestamp"][0]),
        veh_att["q[" + str(0) + "]"],
        "-o",
        label="Vehicle $q_0$",
    )

    for i in range(3):
        ax[i, 0].plot(
            1e-6 * (veh_att_s["timestamp"] - veh_att_s["timestamp"][0]),
            veh_att_s["q_d[" + str(i + 1) + "]"],
            "-o",
            label="Reference",
        )
        ax[i, 0].plot(
            1e-6 * (veh_att["timestamp"] - veh_att["timestamp"][0]),
            veh_att["q[" + str(i + 1) + "]"],
            "-o",
            label="Vehicle",
        )

        ax[i, 0].set_ylabel(r"$q_{" + str(i + 1) + "}$")
        ax[i, 0].legend()
        ax[i, 1].plot(
            1e-6 * (veh_att_s["timestamp"] - veh_att_s["timestamp"][0]),
            veh_att_s[veh_att_s.keys()[i + 1]],
            "-o",
            label="Reference",
        )
        ax[i, 1].plot(
            1e-6 * (veh_ang["timestamp"] - veh_att_s["timestamp"][0]),
            veh_ang[veh_ang.keys()[i + 2]],
            "-o",
            label="Vehicle",
        )
        ax[i, 1].set_ylabel(r"$\omega_{" + str(i + 1) + "}$")
        ax[i, 1].legend()


def plot_commands():
    torque = pd.read_csv("csv_files/" + ulg_file + "_vehicle_torque_setpoint_0.csv")
    thrust = pd.read_csv("csv_files/" + ulg_file + "_vehicle_thrust_setpoint_0.csv")
    fig, ax = plt.subplots(4, 1, num=2, sharex=True, figsize=(FIG_L, FIG_A))
    for i in range(3):
        ax[i].plot(
            1e-6 * (torque["timestamp"] - torque["timestamp"][0]),
            torque["xyz[" + str(i) + "]"],
            "-o",
        )
        ax[i].set_ylabel(r"$M_{" + str(i) + "}$")
    ax[3].plot(
        1e-6 * (thrust["timestamp"] - thrust["timestamp"][0]), thrust["xyz[2]"], "-o"
    )
    ax[3].set_ylabel(r"$T$")


if __name__ == "__main__":
    plot_attitude_states()
    plot_commands()
    plt.show()
