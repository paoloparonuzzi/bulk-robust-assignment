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

The program expects an instance file as input.

Example:

```bash
./bulkRobustMatch ../instances/example_instance.txt
```

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

This code is provided for research purposes.
If you use this code in academic work, please cite the associated paper.

