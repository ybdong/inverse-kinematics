#ifndef _MATRIX_H_
#define _MATRIX_H_
#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>


//using namespace std;

class Matrix{
	friend std::istream& operator >> (std::istream& is, Matrix& A);
	friend std::ostream& operator << (std::ostream& os, const Matrix B);
	//friend void print_file(const Matrix &A,const char* str);
	friend Matrix  operator  +(Matrix& A, Matrix& B);
	friend Matrix  operator  -(Matrix& A, Matrix& B);
	friend Matrix  operator  *(Matrix& A, Matrix& B);
	friend Matrix  operator  *(double lambda,Matrix& B);
	
public:
	
	Matrix(){ data = NULL; row = 0; col = 0; }
	Matrix(int m, int n);
	Matrix(int m, int n, double *a, int size);
	Matrix(const Matrix& B);
//	Matrix(const char* str);

	~Matrix(){ delete[]data; row = 0; col = 0; }
	//重载算数操作符
	Matrix&  operator  =(Matrix const& B);
	Matrix&  operator  -();
	Matrix&  operator +=(Matrix const& B);
	Matrix&  operator -=(Matrix const& B);
	Matrix&  operator *=(Matrix const& B);
	Matrix&  operator *=(double lambda);
	//double&  operator()(unsigned m, unsigned n);
	double&   operator()(unsigned m, unsigned n) const;
	//逆，转置，范数
	Matrix   inv() const;
	Matrix   trans()const;
	Matrix   resize(int m, int n);
	double norm2_1d() const;
	int get_row()const  { return row; }
	int get_col()const  { return col; }
	void SetMatrix(unsigned m, unsigned n, double v){data[m*row + n] = v;}
	
private:
	size_t      row;
	size_t      col;
	double*     data;

};
Matrix eye(unsigned n);
Matrix zeros(unsigned n);
Matrix ones(unsigned n);

#endif
