# AutoMobileRobotics Homework 2 

This project implements the Iterative Closest Point (ICP) algorithm for point cloud registration, with both traditional and PCA-based alignment versions.

## Project Structure

```
as2/
├── task1/                    
│   ├── task1_icp.py         
│   ├── task1_pca.py         
│   ├── task1_visulization.py 
│   ├── bunny1.ply           
│   └── bunny2.ply           
├── task2/                    
└── README.md                 
```

## Requirements

- Python 3.7+
- Open3D
- NumPy

## Usage

### Run

Run the traditional ICP implementation:
```bash
python task1_icp.py
```

Run PCA-based alignment implementation:
```bash
python task1_pca.py
```

Run visulization results:
```bash
python task1_visulization.py
```


## Features

1. **Traditional ICP Implementation**
   - Point-to-point correspondence
   - SVD-based transformation estimation
   - Iterative refinement

2. **PCA-based alignment implementation**
   - Principal component analysis for initial alignment
   - Eigenvalue decomposition for finding main directions
   - Reflection handling for correct orientation
   - Fast global registration without correspondence
   - Can be used as initialization for ICP
  
3. **Visualization**
   - Real-time visualization of registration process
   - Color-coded point clouds
   - Coordinate frame display

## Implementation Details

### ICP Algorithm Steps

1. Find nearest neighbors between point clouds
2. Compute optimal transformation using SVD
3. Apply transformation to source point cloud
4. Repeat until convergence

### PCA Alignment Steps

1. Calculate centroids and center the point clouds
2. Compute covariance matrices
3. Perform eigenvalue decomposition
4. Build rotation matrix from eigenvectors
5. Handle reflection cases
6. Compute translation vector
7. Construct transformation matrix

## License

This project is part of the AutoMobileRobotics course homework. 