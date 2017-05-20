#include "Matrix.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

using namespace std;


 Matrix::Matrix(int m, int n):row(m),col(n){
	if (m < 1 || n < 1)
		throw out_of_range("The row or column number should be larger than 0.");
	data = new double[m*n];
	for (int i = 0; i != m*n; ++i)
	{
		data[i] = 0;
	}
}

//把a存入m*n矩阵,其余补零size为数组a大小
 Matrix::Matrix(int m, int n, double *a, int size) :row(m), col(n){
	data = new double[m*n];
	if (size<m*n)
	{
	for (int i = 0; i != size; ++i)
		data[i] = a[i];
		for (int i = size; i != m*n; ++i)
			data[i] = 0;

	}
	else{
		for (int i = 0; i != m*n; ++i)
			data[i] = a[i];
	}

}
//拷贝构造
 Matrix::Matrix(const Matrix& B):row(B.row),col(B.col){
	if ((row != B.row) || (col != B.col))
		throw invalid_argument("The Matrix should be matched.");
	data = new double[row*col];
	for (int i = 0; i != row*col; ++i)
		data[i] = B.data[i];
} 
//从文件中读入矩阵
/*Matrix::Matrix(const char* str){
	row = 0;
	col = 0;
	ifstream infile(str, ios::in);
	if (!infile)
		throw domain_error("Cannot find the file.");
	char ch = ' ';
	while (infile.get(ch) && ch != '\n')
	{
		if (ch == ' ')
			++col;
	}
	++col;
	infile.clear();
	infile.seekg(0, ios::beg);//已经读到文件尾，重新回到文件头
	while (infile.get(ch))
	{
		if (ch == '\n')
			++row;
	}
	++row;
	infile.clear();
	infile.seekg(0, ios::beg);//已经读到文件尾，重新回到文件头
	data = new double[row*col];
	int i = 0;
	while (i != row*col)
		infile >> data[i++];
	infile.clear;
	infile.close;
}



*/

//重载算数操作符
  Matrix&  Matrix::operator  =(Matrix const& B){
	row = B.row;
	col = B.col;
	data = new double[row*col];
	for (int i = 0; i != row*col; ++i)
		data[i] = B.data[i];
	return *this;
}
 Matrix&  Matrix::operator  -(){
	 for (int i = 0; i != row*col; ++i)
		 data[i] = -data[i];
	 return *this;
 }

 Matrix&  Matrix::operator +=(Matrix const& B){
	if ((row != B.row) || (col != B.col))
		throw invalid_argument("The Matrix should be matched.");
	row = B.row;
	col = B.col;
	for (int i = 0; i != row*col; ++i)
		data[i] += B.data[i];
	return *this;
}

 Matrix&  Matrix::operator -=(Matrix const& B){
	if ((row != B.row) || (col != B.col))
		throw invalid_argument("The Matrix should be matched.");
	row = B.row;
	col = B.col;
	for (int i = 0; i != row*col; ++i)
		data[i] -= B.data[i];
	return *this;
}

 Matrix&  Matrix::operator  *=(double lambda){
	for (int i = 0; i != row*col; ++i)
		data[i] *=lambda;
	return *this;
}
 Matrix&  Matrix::operator  *= (Matrix const& B){
	if (col != B.row)
		throw invalid_argument("The Matrix should be matched.");
	
	double *temp= new double[row*B.col];
	for (int i = 0; i != row; ++i)
	{
		for (int j = 0; j != B.col; ++j)
		{
			temp[i*B.col + j] = 0;
			for (int k = 0; k != col; ++k)
				temp[i*B.col + j] += data[i*col + k] * B.data[k*B.col + j];
		}
	}
	col = B.col;
	for (int i = 0; i != row*col; ++i)
		data[i] = temp[i];
	return *this;
}
 /*
  double&  Matrix::operator()(unsigned m, unsigned n){
	if (m >= row || n >= col)
		throw out_of_range("The size are out of range.");
	return data[m*col + n];
}*/
double&  Matrix::operator()(unsigned m, unsigned n) const{
	
		if (m > row || n > col)
		throw out_of_range("The size are out of range.");
		return data[m*col + n];
}
//逆，转置，范数
 Matrix Matrix::inv() const{
	if (row != col)
		throw invalid_argument("The Matrix should be square.");
	Matrix mat(row, row);
	for (int i = 0; i !=row; ++i)
		mat.SetMatrix(i, i, 1);
	Matrix org(*this);
	double ratio, a;
	for (int i = 0; i !=row; ++i)
	{
		for (int j = 0; j != row; ++j)
		{
			if (i != j)
			{
				ratio = org.data[j*row + i] / org.data[i*row + i];
				for (int k = 0; k != col; ++k)
				{
					org.data[j*row + k] -= ratio*org.data[i*row + k];
					mat.data[j*row + k] -= ratio*mat.data[i*row + k];
				}
			}
		}
	}
	for (int i = 0; i !=row; ++i)
	{
		a = org.data[i*row + i];
		for (int j = 0; j != col; ++j)
		{
			mat.data[i*row + j] /= a;
		
		}

	}
	return mat;
}
 Matrix Matrix::trans()const{
	Matrix mat(col,row);

 	for (size_t i = 0; i != col; ++i)
	{
		for (size_t j = 0; j != row; ++j)
		{
			mat.data[i*row+j] = data[j*col+i]; 

		}
	}
	return mat;
}
 Matrix Matrix::resize(int m, int n){
	 if (m*n!=row*col)
		 throw invalid_argument("The Matrix should be matched.");
	 row = m;
	 col = n;
	 return *this;
 }

 double  Matrix::norm2_1d() const{
	double sum = 0;
	for (int i = 0; i != row*col; ++i)
		sum += data[i]*data[i];
	double norm = sqrt(sum);
	return norm;
}

istream& operator >> (std::istream& is, Matrix& A){
	for (size_t i = 0; i != A.row*A.col; ++i)
	{
			is >> A.data[i];
		}
	return is;
}

ostream& operator << (std::ostream& os, const Matrix B){
	for (int i = 0; i != B.get_row(); ++i)
	{
		for (int j = 0; j != B.get_col(); ++j)
			os << B(i, j) << " "; 
		    cout << endl;
	}
	cout << "-------------------------" << endl;
	return os;
}
//friend void print_file(const Matrix &A,const char* str);
Matrix  operator  +(Matrix& A, Matrix& B){
	if((A.row != B.row) || (A.col!=B.col))
		throw invalid_argument("The Matrix should be matched.");
	Matrix mat(A.row, A.col);
	mat.data = new double[A.row*A.col];
	for (int i = 0; i != mat.row*mat.col; ++i)
		mat.data[i] = A.data[i] + B.data[i];
	return mat;
}
Matrix  operator  -(Matrix& A, Matrix& B){
	if ((A.row != B.row) || (A.col != B.col))
		throw invalid_argument("The Matrix should be matched.");
	Matrix mat(A.row, A.col);
	for (int i = 0; i != mat.row*mat.col; ++i)
		mat.data[i] = A.data[i] - B.data[i];
	return mat;
}

Matrix operator  *(double lambda, Matrix& B){
	Matrix mat(B.row, B.col);
	for (int i = 0; i != mat.row*mat.col; ++i)
		mat.data[i] = lambda*B.data[i];
	return mat;
}
Matrix  operator  *(Matrix& A, Matrix& B){
	if (A.col != B.row)
		throw invalid_argument("The Matrix should be matched.");
	Matrix T(A.row, B.col);
	
	for (int i = 0; i != A.row; ++i)
	{
		for (int j = 0; j != B.col; ++j)
		{
			T.data[i*B.col + j] = 0;
			for (int k = 0; k != A.col; ++k)
				T.data[i*B.col + j] += A.data[i*A.col + k] * B.data[k*B.col + j];
		}
	}
	return T;
}

Matrix eye(unsigned n){
	Matrix mat(n, n);
	for (int i = 0; i != n; ++i)
		mat.SetMatrix(i, i, 1);
	return mat;
}
Matrix zeros(unsigned n){
	Matrix mat(n, n);
	return mat;
}
Matrix ones(unsigned n){
	Matrix mat(n, n);
	for (int i = 0; i != n;++i)
	for (int j = 0; j !=n; ++j)
		mat.SetMatrix(i, j, 1);
	return mat;
}