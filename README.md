# GenericEKF

## Overview
GenericEKF is a C++ implementation of an Extended Kalman Filter (EKF) designed for flexibility and reusability. It supports customizable motion and measurement models using templates and smart pointers.

## Features
- Template-based design for state and measurement dimensions.
- Modular architecture with separate motion and measurement models.
- Runtime flexibility to change models.
- Example usage provided in `main.cpp`.

## Requirements
- C++17 or later
- Eigen3 library

## Build Instructions
1. Ensure Eigen3 is installed on your system.
2. Clone the repository.
3. Run the following commands:
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

## Usage
Run the executable generated in the `build` directory:
```bash
./KalmanFilter
```

## Future Improvements
- Add unit tests.
- Optimize matrix operations for better performance.
- Enhance documentation.

## License
This project is licensed under the MIT License.
