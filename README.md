# Reinforcement Learning in IEEE 802.11ax Networks

This repository contains the source code developed as part of a master's thesis focused on the application of reinforcement learning (RL) to optimize performance in IEEE 802.11ax (Wi-Fi 6) networks.

## 🧠 About the Project

The goal of this project is to evaluate how distributed RL agents can improve wireless network performance while avoiding selfish or aggressive behavior. The research explores how different reward strategies affect agent decisions and overall network fairness and throughput.

Key questions explored:
- Can RL agents improve network performance in a distributed IEEE 802.11ax setting?
- How to prevent RL-driven selfish behavior (e.g., unfair contention window settings)?
- Can RL agents defend the network from other misbehaving (selfish) nodes?

## 📈 Technologies & Concepts

- **IEEE 802.11ax (Wi-Fi 6)** – Wireless communication standard.
- **ns-3** – Network simulator used for modeling and testing the network.
- **Reinforcement Learning (RL)** – Used to dynamically adapt MAC layer parameters.
- **Jain's Fairness Index** – Metric for evaluating fairness.
- **Throughput analysis** – Primary performance measure.

## 🧪 Features

- Simulation of RL agents adjusting MAC parameters (e.g., contention window).
- Evaluation of different reward strategies (throughput, fairness, hybrid).
- Scenarios with mixed legacy and RL-enabled stations.
- Simulated selfish attacks and defense mechanisms.

