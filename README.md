# Bulk Robust Assignment – C++ Implementation

This repository contains the C++ implementation accompanying the paper

> **Effective Solution Algorithms for Bulk-Robust Optimization Problems**

submitted to *Mathematical Programming Computation (MPC)*.

The code implements the algorithms and computational experiments described in the paper for solving the **bulk robust assignment problem** using mixed-integer programming and combinatorial routines based on max-flow / min-cut computations.

---

## Dependencies

The code was developed and tested on **Linux** with the following dependencies:

- **C++17**
- **CMake ≥ 3.16**
- **LEMON graph library**
- **Gurobi Optimizer**

---

## Installing Dependencies

### LEMON graph library

On Ubuntu/Debian systems:

```bash
sudo apt-get install liblemon-dev
```

On other systems, install LEMON from:

https://lemon.cs.elte.hu

---

### Gurobi Optimizer

Download and install Gurobi from:

https://www.gurobi.com

After installation, configure the environment variables:

```bash
export GUROBI_HOME=/path/to/gurobi
export PATH=$PATH:$GUROBI_HOME/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GUROBI_HOME/lib
```

Make sure a valid Gurobi license is available.

---

## Building the Code

Clone the repository:

```bash
git clone https://github.com/paoloparonuzzi/bulk-robust-assignment.git
cd bulk-robust-assignment
```

Create a build directory and compile the project:

```bash
mkdir build
cd build
cmake ..
make -j
```

The executable will be generated as:

```
bulkRobustMatch
```

---

## Running the Program

The executable expects the following command-line arguments:

```bash
./bulkRobustMatch <instance_file> <time_limit> <algorithm> <lp> <edges_per_scenario> <output_file>
```

### Arguments

| Argument | Description |
|--------|-------------|
| `instance_file` | Path to the input instance file |
| `time_limit` | Time limit (in seconds) for the optimization |
| `algorithm` | Algorithm variant to use (see paper for details) |
| `lp` | Solve the LP relaxation (`1`) or the full MIP (`0`) |
| `edges_per_scenario` | Number of edges in each uncertainty scenario |
| `output_file` | Path to the file where results will be written |

### Example

./bulkRobustMatch ../instances/Match_20_0.txt 3600 0 0 3 results.txt


This example runs the solver with:

- a time limit of **3600 seconds**
- **algorithm variant 0**
- solving the **full MIP formulation** (`lp = 0`)
- **3 uncertain edges per scenario**

The results are written to `results.txt`.

The folder `instances/` contains all instances used for testing.

---

## Repository Structure

```
.
├── CMakeLists.txt
├── main.cpp
├── Problem.cpp
├── Problem.h
├── Instance.cpp
├── Instance.h
├── compactModel.cpp
├── assignmentModel.cpp
├── masterModel.cpp
├── BendersCut.cpp
├── SubProblem.cpp
├── matchingAlgorithm.cpp
├── linearRelaxation.cpp
├── FindGUROBI.cmake
└── instances/
```

## Reproducibility

The implementation allows reproducing the computational experiments described in the associated paper.

The provided instances allow testing the implementation and verifying that the code runs correctly.

---

## License

This project is released under the MIT License.

