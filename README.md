# Peer Robotics: Resource Allocation and Route Optimization

This project implements a generalized version of the **Law of Optimality** trick to solve simultaneous resource allocation and route optimization problems, specifically applied to Facility Location and Path Optimization (FLPO).

## Overview

The implementation uses **Deterministic Annealing (DA)** to optimize the placement of resources and the paths between them and sensors/centroids. The algorithm is designed to handle static and dynamic networks efficiently.

### Key Components
- **Deterministic Annealing (DA)**: An optimization framework used to find global minima in complex, non-convex landscapes.
- **Law of Optimality**: A specific heuristic or constraint applied during the DA process to ensure path optimality alongside resource placement.
- **MATLAB Implementation**: Core logic is contained in generalized MATLAB scripts for easy simulation and visualization.

## Project Structure

- `Law_Of_Optimality_Ver2_Generalized_Updated.m`: The main script implementing the optimization algorithm.
- `Distortion1_Ver2.m`: Utility function for computing distortion (distance) metrics between sensors and resources.
- `Distortion2_Ver2.m`: Utility function for computing internal resource-to-resource distortion.
- `Report.docx`: Detailed project report and analysis.
- `Problem Statement.jpg`: Visual description of the problem setup.

## Installation & Usage

### Prerequisites
- MATLAB (R2020b or later recommended)
- (Optional) Python 3.x for auxiliary scripts

### Running the Optimization
1. Open MATLAB and navigate to the project directory.
2. Run the main script:
   ```matlab
   Law_Of_Optimality_Ver2_Generalized_Updated
   ```
3. The script will output the temperature (T) during the annealing process and generate a final visualization plot showing:
   - Centroids (Blue Triangles)
   - Resources (Red Circles)
   - Destination (Black Diamond)
   - Association paths

## Research Context

This work is informed by research in "Simultaneous Facility Location and Path Optimization in Static and Dynamic Networks." The `Report.docx` contains a deeper exploration of the methodology and results.
