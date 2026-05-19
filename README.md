# Peer Robotics — Resource Allocation & Route Optimization

MATLAB implementation of **simultaneous facility location and path optimization** (FLPO) using **Deterministic Annealing (DA)** and a generalized **Law of Optimality** heuristic. Assigns resource locations and routes in static sensor networks while minimizing total distortion.

## Overview

The algorithm jointly optimizes:

1. **Where** to place facilities (resources) relative to sensor centroids
2. **How** to route paths between sensors, resources, and a destination

Deterministic Annealing gradually reduces a temperature parameter to escape local minima in this non-convex problem.

## Features

- **Deterministic Annealing (DA)** — Global optimization over placement and routing
- **Law of Optimality** — Heuristic constraint applied during annealing for path-quality guarantees
- **Distortion metrics** — Separate sensor-to-resource (`Distortion1`) and resource-to-resource (`Distortion2`) cost functions
- **Visualization** — Plots centroids, resources, destination, and association paths on completion

## Tech Stack

| Component | Technology |
|-----------|------------|
| Language | MATLAB R2020b+ |

## Project Structure

```
Peer-Robotics/
├── Law_Of_Optimality.m      # Main optimization script
├── Distortion1.m            # Sensor ↔ resource distortion
├── Distortion2.m            # Resource ↔ resource distortion
├── Report.docx              # Methodology and results write-up
├── Problem Statement.jpg    # Problem diagram
├── IMG_3576.JPG / IMG_3577.JPG   # Supplementary figures
└── Research Papers/
    └── Research Paper - Simultaneous Facility Location and Path Optimization.pdf
```

## Getting Started

### Prerequisites

- MATLAB R2020b or later

### Installation

```bash
git clone https://github.com/priyqnk/Peer-Robotics.git
cd Peer-Robotics
```

### Usage

1. Open MATLAB and set the working directory to the project folder.
2. Run the main script:

   ```matlab
   Law_Of_Optimality
   ```

3. Monitor temperature `T` during annealing. On completion, the plot shows:
   - **Blue triangles** — Sensor centroids
   - **Red circles** — Placed resources
   - **Black diamond** — Destination
   - **Lines** — Optimized association paths

## Research Context

This work draws on *Simultaneous Facility Location and Path Optimization in Static and Dynamic Networks*. See `Report.docx` and the PDF in `Research Papers/` for full methodology, parameter choices, and experimental results.
