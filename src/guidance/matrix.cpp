/*
  ************************ CLASS *********************************
  => Provides methods for adding, subtracting, multiplying matices
  => Method to get transpose of matix
  ****************************************************************
*/
  
#include "asv_ros_revamp/guidance/matrix.hpp"

MATRIX::MATRIX(int row, int col)
{
  this->row = row;
  this->col = col;
}

MATRIX::MATRIX()
{

}

MATRIX MATRIX::matrix_multiply(MATRIX m1, MATRIX m2)
{
	int row,col,i;
  MATRIX r;
  
  for(row = 0; row < m1.row; row++)
  {
    for(col=0; col < m2.col; col++)
    {
      r.m[row][col] = 0.0;
      for(i=0; i < m1.col; i++)
      {
          r.m[row][col]+= m1.m[row][i] * m2.m[i][col];
      }
    }
  }
  r.row = m1.row;
  r.col = m2.col;
  return r;
}

MATRIX MATRIX::matrix_subtract(MATRIX m1, MATRIX m2)
{
  int row,col ;
  MATRIX r;  
  for(row=0; row < m1.row; row++)
  {
    for(col = 0; col < m2.col; col++)
      r.m[row][col] = m1.m[row][col] - m2.m[row][col];
  }
  r.row = m1.row;
  r.col = m2.col;
  return r;
}

MATRIX MATRIX::matrix_add(MATRIX m1, MATRIX m2)
{  
  int row,col ;
  MATRIX r;

  for(row = 0; row<m1.row; row++)
    for(col = 0; col < m2.col; col++)
         r.m[row][col] = m1.m[row][col] + m2.m[row][col];
   r.row = m1.row;
   r.col = m2.col;
   return r;
}

MATRIX MATRIX::matrix_transpose(MATRIX m)
{ 
  int row,col;
  MATRIX r;

  for(row=0;row<m.row;row++){
    for(col=0;col<m.col;col++)
      r.m[col][row] = r.m[row][col];
  }
  r.row=m.col;
  r.col=m.row;
  return r;
}