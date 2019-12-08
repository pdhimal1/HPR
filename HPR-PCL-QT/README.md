This project is a simple implementation of the Hidden Point Removal operator described
in Katz et. al. "Direct Visibility of Point Sets", 2007.

The program can read point clouds in PCD and PLY files.

HPR Operator
------------------------

See `PCLViewer::HPR()` for an implementation of the HPR operator.

Requirements:
-------------------------
Requires:
  - Cmake 3.15
  - c++14
  - Qt5
  - VTK
  - PCL 1.8

Running the program:
-------------------------
To run the executable: 
   - `./HPR -pcdFile ../../data/ism_test_michael.pcd`
   - `./HPR -pcdFile ../../data//bunny.ply`
   
The radius:
-------------------------