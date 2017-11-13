#include "basicMatrix.h"

//
// Created by Mahmoud on 13/09/2017.
//

int Matrix::rowColToOffset(int row,int col)
{
    return (this->columns * row) + col;
}

void Matrix::Put(int row,int col, double value)
{
    this->data[rowColToOffset(row,col)] = value;
}

Matrix::Matrix(int new_rows,int new_cols, bool Identity)
{
    if(Identity)
    {
        if(new_cols != new_rows)
        {
            printf("Identity matrix should be square");
            return;
        }
    }
    for(int i = 0;i<(new_cols*new_rows);i++)
    {
        data[i] = 0;
    }
    this->rows = new_rows;
    this->columns = new_cols;

    if(Identity)
    {
        for(int i=0;i<new_rows;i++)
        {
            this->Put(i,i,1);
        }
    }
}

double Matrix::Get(int row,int col)
{
    return this->data[rowColToOffset(row,col)];
}

void Matrix::getRow(int row, double* return_row)
{
    int startIndex = rowColToOffset(row,0);
    int endIndex = startIndex + this->columns;
    for (int i = startIndex; i < endIndex; ++i)
    {
        return_row[i - startIndex] = this->data[i];
    }
}

void Matrix::getCol(int col, double* return_col)
{
    for (int r = 0; r <this->rows ; ++r)
    {
        return_col[r] = Get(r,col);
    }
}

Matrix * Matrix::MultipliedByScalar(double value)
{
    Matrix * resultantMatrix;
    resultantMatrix = new Matrix(this->rows,this->columns,false);
    // for from 0 to length of data
    for (int i = 0; i <(this->rows * this->columns) ; ++i)
    {
        resultantMatrix->data[i] = this->data[i] * value;
    }
    return resultantMatrix;
}

Matrix * Matrix::MultipliedBy(Matrix * otherMatrix)
{
    double row[2];
    double col[2];
    double value = 0;
    if(this->columns != otherMatrix->rows)
    {
        printf("Matrices are not compatible for multiplication");
        return nullptr; // "AHAWH" function return Matrix ...... now return void :( missing python
    }
    Matrix  * resultantMatrix;
    resultantMatrix = new Matrix(this->rows,otherMatrix->columns,false);
    for (int r = 0; r < this->rows ; ++r)
    {
        for(int c = 0; c < otherMatrix->columns; ++c)
        {
            this->getRow(r,row);
            otherMatrix->getCol(c,col);
            value = 0;
            // for from 0 to length of row array
            for (int i = 0; i < this->columns; ++i)
            {
                value += row[i] * col[i];
            }
            resultantMatrix->Put(r,c,value);
        }
    }
    return resultantMatrix;
}

Matrix * Matrix::Transpose(void)
{
    double row[2];
    Matrix * transpose = new Matrix(this->columns, this->rows,false);

    for (int r = 0; r < this->rows; ++r)
    {
        this->getRow(r,row);
        // this->columns
        for (int i = 0; i < this->columns; ++i)
        {
            transpose->Put(i,r,row[i]);
        }
    }
    return transpose;
}

Matrix * Matrix::Add(Matrix * otherMatrix)
{
    if((this->rows != otherMatrix->rows)||(this->columns != otherMatrix->columns))
    {
        printf("Cannot add matrices of different dimensions \n");
        return nullptr;
    }

    Matrix  * resultantMatrix = new Matrix(this->rows,this->columns,false);

    for (int i = 0; i <(this->rows * this->columns) ; ++i)
    {
        resultantMatrix->data[i] = this->data[i] + otherMatrix->data[i];
    }

    return resultantMatrix;
}

Matrix * Matrix::Subtract(Matrix * otherMatrix)
{
    if((this->rows != otherMatrix->rows)||(this->columns != otherMatrix->columns))
    {
        printf("Cannot subtract matrices of different dimensions");
        return nullptr;
    }

    Matrix  * resultantMatrix = new Matrix(this->rows,this->columns,false);

    for (int i = 0; i <(this->rows * this->columns) ; ++i)
    {
        resultantMatrix->data[i] = this->data[i] - otherMatrix->data[i];
    }

    return resultantMatrix;
}

double Matrix::Determinant(void)
{
    if((this->rows != this->columns))
    {
        printf("Cannot take the determinant of a non-square matrix");
        return 0;
    }

    if(this->rows == 1)
        return this->Get(0, 0);
    else if(this->rows == 2)
        return (this->Get(0, 0) * this->Get(1, 1) - this->Get(0, 1) * this->Get(1, 0));
    else
    {
        int sign = 1;
        double total = 0.0;
        int r = 0;
        for (int c = 0; c < this->columns; ++c)
        {
            double value = this->Get(r, c);
            value *= this->MatrixExcludingRowAndCol(r, c)->Determinant();
            value *= sign;
            sign *= -1;
            total += value;
        }
        return total;
    }
}

Matrix * Matrix::MatrixExcludingRowAndCol(int omitRow, int omitCol)
{
    Matrix  * resultantMatrix = new Matrix(this->rows - 1,this->columns - 1,false);

    int newIndex = 0;

    for (int i = 0; i < (this->rows * this->columns); i++)
    {
        if(int(i / this->columns) == omitRow)
            continue;
        else if((i % this->columns) == omitCol)
            continue;

        resultantMatrix->data[newIndex] = this->data[i];
        newIndex++;
    }

    return resultantMatrix;
}

Matrix * Matrix::Inverse(void)
{
    if(this->rows != this->columns)
    {
        printf("Cannot take inverse of a non-square matrix");
        return nullptr;
    }
    if(this->Determinant() == 0)
    {
        printf("Cannot take inverse of matrix");
        return nullptr;
    }

    Matrix  * newMatrix = new Matrix(this->rows, this->columns, false);

    if ((this->rows == 1) && (this->columns == 1))
    {
        // this is a special case hack, but it should be ok...
        if (this->Get(0, 0) == 0.0)
        {
            return nullptr;
        }
        newMatrix->Put(0, 0, 1.0/this->Get(0, 0));
        return newMatrix;
    }

    for (int r = 0; r < this->rows ; r++)
    {
        int sign = 1;
        if((r % 2) == 1)
            sign = -1;
        for (int c = 0; c < this->columns; c++)
        {
            newMatrix->Put(r, c, double(sign) * this->MatrixExcludingRowAndCol(r, c)->Determinant());
            sign *= -1;
        }
    }

    Matrix * diagSwapped = newMatrix->Transpose();
    return diagSwapped->MultipliedByScalar(1/this->Determinant());
}

