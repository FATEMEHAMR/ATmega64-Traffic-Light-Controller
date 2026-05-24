# ATmega64 Traffic Light Controller

This project implements a four-way traffic light controller using the **ATmega64 microcontroller** and **AVR Assembly**.  
The controller is designed for a road intersection with two directions:

- North–South (NS)
- East–West (EW)

Each direction has three traffic lights: **Red**, **Yellow**, and **Green**.  
The system is based on a deterministic **Finite State Machine (FSM)** and uses **Timer1 in CTC mode** to generate accurate timing without blocking delays.

## Features

- Four-state traffic light FSM
- Accurate 10 ms timing using Timer1 CTC interrupt
- Real-time timing adjustment using external interrupts
- Short-press and long-press detection for push buttons
- Green-light duration adjustment with minimum and maximum limits
- Yellow-light duration adjustment in 250 ms steps
- Traffic sensor mode for prioritizing congested directions
- Automatic saving and restoring of timing parameters
- Proteus simulation support

## System Overview

The traffic controller runs continuously through the following sequence:

```text
NS Green → NS Yellow → EW Green → EW Yellow → NS Green → ...
