#include "ros/ros.h"

class MATRIX {
  public:
    int row, col;
    int m[4][4];
    MATRIX(int row, int col);
    MATRIX();   
    static MATRIX matrix_multiply(MATRIX m1, MATRIX m2);
    static MATRIX matrix_subtract(MATRIX m1, MATRIX m2);
    static MATRIX matrix_add(MATRIX m1, MATRIX m2);
    static MATRIX matrix_transpose(MATRIX m);

};