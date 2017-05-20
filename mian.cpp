#include <iostream>
#include <iomanip>
#include "bvh.h"
#include "Matrix.h"
#include <math.h>
#define PI 3.1415926 
using namespace std;

//运动重定向
//平移变换矩阵
Matrix Trans(const double* a)
{
	Matrix A = eye(4);
	for (int i = 0; i != 3; i++)
		A(i, 3) = a[i];
	return A;
}
//旋转变换矩阵
Matrix Rotation(double theta, int type)
{
	Matrix A = eye(4);
	if ( type == 0)
    {
		A(1, 1) =  cos(theta);
		A(1, 2) = -sin(theta);
		A(2, 1) =  sin(theta);
		A(2, 2) =  cos(theta);
		return A;
	}
	if (type == 1)
	{
		A(0, 0) =  cos(theta);
		A(0, 2) =  sin(theta);
		A(2, 0) = -sin(theta);
		A(2, 2) =  cos(theta);
		return A;
	}
	else
	{
		A(0, 0) =  cos(theta);
		A(0, 1) = -sin(theta);
		A(1, 0) =  sin(theta);
		A(1, 1) =  cos(theta);
		return A;
	}
}
//读取位置坐标以及相对位置坐标，最后一个节点是否是末节点两种不同情况
double* Getposition(vector<int> node, const BVH &A,int current_frame)
{
	if (node.size()==0)
		throw  invalid_argument("There is no node.");
	
	if (A.GetJoint(node.back())->has_site == false)
	{
		double *a = new double[3 * node.size()];
		for (int i = 0; i != 3; i++)
			a[i] = A.GetMotion( current_frame,i);
		for (int i = 1; i < node.size(); i++)
		{
			a[3 * i] = A.GetJoint(node[i])->offset[0];
			a[3 * i + 1] = A.GetJoint(node[i])->offset[1];
			a[3 * i + 2] = A.GetJoint(node[i])->offset[2];
		}
		return a;
	}
	else
	{
		double *a = new double[3 * node.size()+3];
		for (int i = 0; i != 3; i++)
			a[i] = A.GetMotion(current_frame, i);
		for (int i = 1; i < node.size(); i++)
		{
			a[3 * i + 0] = A.GetJoint(node[i])->offset[0];
			a[3 * i + 1] = A.GetJoint(node[i])->offset[1];
			a[3 * i + 2] = A.GetJoint(node[i])->offset[2];
		}
		for (int i = 0; i != 3; ++i)
			a[3 * node.size() + i] = A.GetJoint(node.back())->site[i];
		return a;
	}


}

//关节链起点不是根节点，扩充关节链，向前补齐至根节点
/*vector<int>  FillNode_Line(vector<int>node, const BVH &A)
{
		vector<int> node_stack;
		vector<int> node1;
		node_stack.push_back(A.GetJoint(node.back())->index);
		while (A.GetJoint(node_stack.back())->parent->index != 0)
			node_stack.push_back(A.GetJoint(node_stack.back())->parent->index);
		node_stack.push_back(0);
		while (node_stack.size() != 0)
		{
			node1.push_back(node_stack.back());
			node_stack.pop_back();
		}
		return node1;
	
}*/
//获取各关节旋转角信息
double* GetRotation(vector<int> node, const BVH &A, int current_frame)
{
	double *a= new double[3 * node.size()];
	for (int i = 0; i < node.size(); ++i)
	{
		if (node[i] == 0)
		{
			a[i*3 + 0] = A.GetMotion(current_frame, A.GetJoint(0)->channels[3]->index);
			a[i*3 + 1] = A.GetMotion(current_frame, A.GetJoint(0)->channels[4]->index);
			a[i*3 + 2] = A.GetMotion( current_frame,A.GetJoint(0)->channels[5]->index);
		}
		else
		{
			a[i*3 + 0] = A.GetMotion(current_frame, A.GetJoint(node[i])->channels[0]->index);
			a[i*3 + 1] = A.GetMotion( current_frame,A.GetJoint(node[i])->channels[1]->index);
			a[i*3 + 2] = A.GetMotion( current_frame,A.GetJoint(node[i])->channels[2]->index);
		}
	}
		for (int i = 0; i != 3 * node.size(); ++i)
		{
			a[i] = a[i] *PI/ 180;
		}
	
	return a;
}
double* GetRotation(vector<int> node, const double *theta)
{
	double *a = new double[3 * node.size()];
	for (int i = 0; i < node.size(); ++i)
	{
			a[i * 3 + 0] = theta[3 * node[i] + 0];
			a[i * 3 + 1] = theta[3 * node[i] + 1];
			a[i * 3 + 2] = theta[3 * node[i] + 2];
		}
	return a;
}
//更新旋转角信息
void   SetRotation(vector<int> node, double* theta, BVH &A, int current_frame)
{
for (int i = 0; i != 3 * node.size(); ++i)
		{
			theta[i] = theta[i] *180/ PI;
		}
	for (int i = 0; i < node.size(); ++i)
	{
		
		if (node[i] == 0)
		{
			 A.SetMotion(current_frame, A.GetJoint(0)->channels[3]->index,theta[0]);
			 A.SetMotion(current_frame, A.GetJoint(0)->channels[4]->index,theta[1]);
			 A.SetMotion(current_frame, A.GetJoint(0)->channels[5]->index,theta[2]);
		}
		else
		{
			A.SetMotion(current_frame, A.GetJoint(node[i])->channels[0]->index, theta[3 * i + 0]);
			A.SetMotion(current_frame, A.GetJoint(node[i])->channels[1]->index, theta[3 * i + 1]);
			A.SetMotion(current_frame, A.GetJoint(node[i])->channels[2]->index, theta[3 * i + 2]);
		}
	}
}
//前向
Matrix Forwardkinematic(const double*displ_all,const double*theta_all,int number,int* type)
{
	Matrix a(3,1);
	Matrix A = eye(4);
	for (int i = 0; i != number; ++i)
	{
		double  displ[] = { displ_all[3 * i + 0], displ_all[3 * i + 1], displ_all[3 * i + 2] };
		Matrix T = Trans(displ);
		Matrix R0 = Rotation(theta_all[3 * i + 0], type[0]);
		Matrix R1 = Rotation(theta_all[3 * i + 1], type[1]);
		Matrix R2 = Rotation(theta_all[3 * i + 2], type[2]);
		A = A*T*R0*R1*R2;

	}
	double displ2[] = { displ_all[3 * number + 0], displ_all[3 * number + 1], displ_all[3 * number + 2] };
	Matrix T2 = Trans(displ2);
	     A = A*T2;
	for (int i = 0; i != 3; ++i)
		a(i,0)= A(i, 3);
	return a;
}
Matrix Forwardkinematic(vector< double*> displ_all, const double*theta_all, vector<vector<int>> nodes, int* type)
{
	int number = nodes.size();
	Matrix  a(3*number, 1);
	for (int i = 0; i < nodes.size(); i++)
	{
		int num = nodes[i].size();
		const double *displ, *theta;
		displ = displ_all[i];
		theta = GetRotation(nodes[i], theta_all);
		/*for (int k = 0; k < 3 * nodes[i].size(); k++)
			cout << theta[k] << " ";
		cout << endl;*/
		Matrix A = Forwardkinematic(displ, theta,num, type);
	//	cout << A;
		for (int j = 0; j < 3; j++)
		{
			a(3 * i + j,0) = A(j, 0);
	//		cout << a(3 * i + j, 0) << endl;
		}
	}
	return a;

}
//雅克比
Matrix  Jacobian(const double* displ, double* theta,vector<vector<int>> nodes,int number, int* type)
{
	double EPS = 0.1;
	Matrix J(3, 3 * number);
	Matrix orig = Forwardkinematic(displ,theta, number,type);
	for (int i = 0; i != 3 * number; ++i)
	{
		theta[i] = theta[i]+ EPS;
   
		Matrix	dest = Forwardkinematic(displ, theta, number, type);
		theta[i] = theta[i] -EPS;
		for (int j = 0; j != 3; ++j)
			J(j, i) = (dest(j, 0) - orig(j, 0)) / EPS;
	}
	return J;
} 
Matrix  Jacobian(const vector<double*> displ, double* theta, vector<vector<int>> nodes, int number, int* type)
{
	double EPS = 0.1;
	int num = nodes.size();
	Matrix J(3*num, 3 * number);
	Matrix orig = Forwardkinematic(displ, theta, nodes, type);
	for (int i = 0; i != 3 * number; ++i)
	{
		theta[ i] = theta[ i] + EPS;

		Matrix	dest = Forwardkinematic(displ, theta, nodes, type);
		theta[ i] = theta[i] - EPS;
		for (int j = 0; j != 3*num; ++j)
			J(j, i) = (dest(j, 0) - orig(j, 0)) / EPS;
	}
	return J;
}
//逆向
void Inversekinematic(BVH &bvh, const vector <double*> displ_dst, double* theta_dst, const vector <double*> displ, const double* theta, vector< vector<int>> nodes, vector<int > node, int number, int* type, int frame, int &count)
{  
	int num = nodes.size();
	Matrix orig = Forwardkinematic(displ, theta, nodes, type);
	Matrix dest = Forwardkinematic(displ_dst, theta_dst,nodes, type);

	Matrix delta_x =  orig-dest ;
	double p, lambda = 0.2, beta =1;
	p = (delta_x).norm2_1d();
	for (int i = 0; i != 100; ++i)
	{
		if (p < 0.01)
		{
			cout << "this frame is converged." << endl;
			SetRotation(node, theta_dst,bvh,frame);
			count++;
			break;
		}
		Matrix J = Jacobian(displ_dst, theta_dst, nodes,number, type);
		//cout << J;
		Matrix J_lambda = J*(J.trans());
		Matrix A=J_lambda+lambda*eye(3*num);
		 A = A.inv();
		 Matrix J_final(3*number,3*num);
		 J_final= (J.trans())*A;
		Matrix delta_p = beta*J_final*delta_x;
		//cout << delta_p;
		for (int i = 0; i != 3 * number; ++i)
		{
			theta_dst[i] = theta_dst[i] + delta_p(i,0);
		//	cout << theta_dst[i] << " ";
		}//cout << endl;
		dest = Forwardkinematic(displ_dst, theta_dst,nodes, type);
		delta_x =  orig -dest;
		p = (delta_x).norm2_1d();
	}
	return;
}
int main(){

	const char* bvh_src_name = "data/input/yanyuan_dongzuo.bvh";
	const char* bvh_dst_name = "data/input/yanyuan_Tpose.bvh";
	const char* bvh_outfile_name = "data/output/yanyuan_dongzuoresult.bvh";
	//解析、初始化
	BVH A(bvh_src_name);
	BVH B(bvh_dst_name);
	for (int i = 0; i != A.GetnumNodes_Line(); i++)
	{
		vector<int> node = A.Getnodes_Line(i);
	/*	for (int j = 0; j != node.size(); j++)
			cout << node[j]<<" ";
		cout << endl;*/
	}
	B.SetNumframe(A.GetNumFrame());
	B.ResizeMotion(A.GetNumFrame(), A.Getnumchannel());
	for (int i = 0; i != A.GetNumFrame(); ++i)
	{
		for (int j = 0; j !=3; ++j)
			B.SetMotion(i, j, 0.5*A.GetMotion(i, j));
		for (int j = 3; j != A.Getnumchannel(); ++j)
			B.SetMotion(i, j, A.GetMotion(i, j));
	}
	//
	vector<double *>displ;
	vector<double *>displ_dst;
	int count = 0;
	int number;
	int index[] = { A.GetJoint(0)->channels[3]->type, A.GetJoint(0)->channels[4]->type, A.GetJoint(0)->channels[5]->type };
	int *type = &index[0];
	number = A.GetnumJoint();
	vector<vector<int>> nodes;
	vector <int> node;
	for (int i = 0; i < A.GetnumJoint(); i++)
		node.push_back(i);
	for (int i = 0; i != A.GetnumNodes_Line(); ++i)    
		nodes.push_back ( A.Getnodes_Line(i));

	for (int ff = 0; ff != 900; ff++)
		{
			double* disl_temp;
			cout << "current frame: " << ff + 1 << "   " << "total frames: " << B.GetNumFrame() << endl;
			for (int i = 0; i < A.GetnumNodes_Line(); i++)
			{
				 disl_temp = Getposition(nodes[i], A, ff);
				for (int j = 0; j !=3* nodes[i].size()+3; ++j)
				{
					disl_temp[j] /= 2;
					/*cout << disl_temp[j] << " ";*/
				}
				displ.push_back(disl_temp);
			//
				displ_dst.push_back(Getposition(nodes[i],B, ff));
			}
			double *theta = GetRotation(node, A, ff);
			Inversekinematic(B, displ_dst, theta, displ, theta, nodes,node,number, type, ff,count);

		/*	displ.clear();
			displ_dst.clear();
			delete theta;
			delete disl_temp;*/

			cout << count;
	     }

	WriteBVH(bvh_outfile_name, B);/* */
}