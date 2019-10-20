#include "ros/ros.h"

class MATRIX 
{
    public:
        int row, col;
        int m[4][4];
        MATRIX(int row, int col);
        MATRIX();   
        static MATRIX multiply(MATRIX m1, MATRIX m2);
        static MATRIX subtract(MATRIX m1, MATRIX m2);
        static MATRIX add(MATRIX m1, MATRIX m2);
        static MATRIX transpose(MATRIX m);
};