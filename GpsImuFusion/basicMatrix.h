#ifndef __basicMatrix_INCLUDED__
#define __basicMatrix_INCLUDED__

#include <stdio.h>
#include <iostream>
#include <cstdlib>

class Matrix
{
    public:
        void Put(int row,int col, double value);

        Matrix(int new_rows,int new_cols, bool Identity);

        double Get(int row,int col);

        Matrix * MultipliedBy(Matrix * otherMatrix);

        Matrix * Transpose(void);

        Matrix * Add(Matrix * otherMatrix);

        Matrix * Subtract(Matrix * otherMatrix);

        Matrix * Inverse(void);

    //private:
        double data[4];
        int rows;
        int columns;

        int rowColToOffset(int row,int col);

        void getRow(int row, double* return_row);

        void getCol(int col, double* return_col);

        Matrix * MultipliedByScalar(double value);

        double Determinant(void);

        Matrix * MatrixExcludingRowAndCol(int omitRow, int omitCol);
};




#endif