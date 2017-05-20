#include <fstream>
#include <string>
#include <iostream>
#include <iomanip>
#include "bvh.h"
#define BUFFER_LENGTH 1024*4


//默认构造函数
BVH::BVH(){
	motion = NULL;
	Clear();
}
//从文件写入
BVH::BVH(const char* bvh_file_name){
	motion = NULL;
	Clear();
	Load(bvh_file_name);
}


BVH::~BVH(){
	Clear();

}
void BVH::Clear(){
	for (int i = 0; i != channels.size(); ++i)
		delete channels[i];
	for (int i = 0; i != Joints.size(); ++i)
		delete Joints[i];
	if (motion != NULL)
		delete motion;
	is_load_success = false;
	file_name = "";
	motion_name = "";
	num_channels = 0;
	channels.clear();
	Joints.clear();
	nodes_line.clear();
	joint_index.clear();

	num_frame = 0;
	interval = 0;
	motion = NULL;
}
void BVH::Load(const char* bvh_file_name)
{

	ifstream file;
	char   line[BUFFER_LENGTH];
	char*  token;
	char   separator[] = " :,\t";
	vector< Joint * >   joint_stack;
	Joint* joint = NULL;
	Joint* new_joint = NULL;
	bool   is_site = false;
	double x, y, z;
	int    i, j;
	vector<int> node;
	vector <int> tempnode;
    //初始化
	Clear();
	//配置路径名与文件名
	file_name = bvh_file_name;
	const char* mn_first = bvh_file_name;
	const char* mn_last = bvh_file_name +strlen(bvh_file_name);
	if (strrchr(bvh_file_name, '\\') != NULL)
		mn_first = strrchr(bvh_file_name, '\\') + 1;
	else if (strrchr(bvh_file_name, '/')!=NULL)
		mn_first = strrchr(bvh_file_name, '/') + 1;
	if (strrchr(bvh_file_name, '.') != NULL)
		mn_last = strrchr(bvh_file_name, '.');
	if (mn_last < mn_first)
		mn_last = bvh_file_name + strlen(bvh_file_name);
	motion_name.assign(mn_first, mn_last);
	//解析BVH文件
	file.open(bvh_file_name, ios::in);
	if (file.is_open() == 0) 
		return; //如果无法打开，退出
	while (!file.eof())
	{
		if (file.eof())
			goto bvh_error;
		//读取一行，得到第一个单词
		file.getline(line, BUFFER_LENGTH);
		token = strtok(line, separator);
		//遇到空行，继续下一行
		if (token == NULL) continue;
		//关节的开始
		if (strcmp(token, "{") == 0)
		{
			//当前关节添加到栈中
			joint_stack.push_back(joint);
			joint = new_joint;
			continue;
		}
		//关节的结束
		if (strcmp(token, "}") == 0)
		{
			//释放栈中当前关节
			joint = joint_stack.back();
			joint_stack.pop_back();
			is_site = false;
			continue;
		}
		//判断关节信息
		if ((strcmp(token, "ROOT") == 0) || strcmp(token, "JOINT") == 0)
		{
			//创建关节
			new_joint = new Joint();
			new_joint->index = Joints.size();
			new_joint->parent = joint;
			new_joint->type = token;
			new_joint->has_site = false;
			for (int i = 0; i != 3; ++i)
			{
				new_joint->offset[i] = 0;
				new_joint->site[i] = 0;
			}
			Joints.push_back(new_joint);
			if (joint)
				joint->children.push_back(new_joint);
			//获取关节名
			token = strtok(NULL, "");
			while (*token == ' ')
				token++;
			new_joint->name = token;
			//添加到索引
			joint_index[new_joint->name] = new_joint;
			continue; 
		}
		//末端信息获取
		if ((strcmp(token, "End") == 0))
		{
			new_joint = joint;
			is_site = true;
			continue;
		}
		//偏移量或者末端位置的信息
		if ((strcmp(token, "OFFSET") == 0))
		{
			token = strtok(NULL, separator);
			x = token ? atof(token) : 0;
			token = strtok(NULL, separator);
			y = token ? atof(token) : 0;
			token = strtok(NULL, separator);
			z = token ? atof(token) : 0;
			
		
		    //坐标的偏移量
		    if (is_site)
		    {
			     joint->has_site = true;
			     joint->site[0] = x;
			     joint->site[1] = y;
			     joint->site[2] = z;
		    }
		    else
		    //
		    {
			     joint->offset[0] = x;
			     joint->offset[1] = y;
			     joint->offset[2] = z;
		    }
		    continue;
	    }
		//关节的通道信息
		if (strcmp(token, "CHANNELS") == 0)
		{
			//通道数量
			token = strtok(NULL, separator);
			joint->channels.resize(token ? atof(token) : 0);
			for (int i = 0; i != joint->channels.size(); ++i)
			{
				//创建通道
				Channel* channel = new Channel();
				channel->joint = joint;
				channel->index = channels.size();
				channels.push_back(channel);
				joint->channels[i] = channel;
				//获取通道类别
				token = strtok(NULL, separator);
				if (strcmp(token, "Xrotation") == 0)
				{
					channel->type = X_ROTATION;
					channel->name = "Xrotation";
				}
				else if (strcmp(token, "Yrotation") == 0)
				{
					channel->type = Y_ROTATION;
					channel->name = "Yrotation";
				}
				else if (strcmp(token, "Zrotation") == 0)
				{
					channel->type = Z_ROTATION;
					channel->name = "Zrotation";
				}
				else if (strcmp(token, "Xposition") == 0)
				{
					channel->type = X_POSITION;
					channel->name = "Xposition";
				}
				else if (strcmp(token, "Yposition") == 0)
				{
					channel->type = Y_POSITION;
					channel->name = "Yposition";
				}
				else if (strcmp(token, "Zposition") == 0)
				{
					channel->type = Z_POSITION;
					channel->name = "Zposition";
				}

			}
		}
		//运动数据段前停止
		if (strcmp(token, "MOTION") == 0)
			break;
	
	}
	//遍历骨骼链
	for(int i = 1; i != Joints.size(); ++i )
	{   
		if ((Joints[i]->children).size() == 0)
		{
			tempnode.push_back(i);
			while (Joints[tempnode.back()]->parent->index != 0)
			{
				tempnode.push_back(Joints[tempnode.back()]->parent->index);
				
			}tempnode.push_back(0);
			while (tempnode.size() != 0)
			{
				node.push_back(tempnode.back());
				tempnode.pop_back();
			}
			nodes_line.push_back(node);
			node.clear(); 
 		
		}
		
	}
	//编辑运动信息
	file.getline(line, BUFFER_LENGTH);
	token = strtok(line, separator);
	if (strcmp(token, "Frames") != 0)
		goto bvh_error;
	token = strtok(NULL, separator);
	if (token == NULL)
		goto bvh_error;
	num_frame = atof(token);
	file.getline(line, BUFFER_LENGTH);
	token = strtok(line, ":");
	if (strcmp(token, "Frame Time") != 0)
		goto bvh_error;
	token = strtok(NULL, separator);
	if (token == NULL)
		goto bvh_error;
	interval = atof(token);
	num_channels = channels.size();
	motion = new double[num_frame*num_channels];
	//数据获取
	for (int i = 0; i != num_frame; ++i)
	{
		file.getline(line, BUFFER_LENGTH);
		token = strtok(line, separator);
		for (int j = 0; j != num_channels; ++j)
		{
			if (token == NULL)
				goto bvh_error;
			motion[i*num_channels + j] = atof(token);
			token = strtok(NULL, separator);
		}
	}
	file.close();
	is_load_success = true;
	return; 
    bvh_error:
		file.close();
}
//写出单个关节信息
void WiiteBVH(const char *bvh_file_name,Joint *joint)
{
	double *a;
	double *b;
	ofstream outfile(bvh_file_name, ios::app);
	outfile << fixed << std::setprecision(6);
	outfile << joint->type << " " << joint->name << endl;
	outfile << "{" << endl;
	a = joint->offset;
	outfile << "OFFSET " << a[0] << " " << a[1] << " " << a[2] << endl;
	outfile << "CHANNELS " << (joint->channels).size() << " ";
	for (int i = 0; i != (joint->channels).size(); ++i)
		outfile << (joint->channels[i])->name << " ";
	outfile << endl;
	if (joint->children.size() != 0)
	{
		for (int i = 0; i != joint->children.size(); ++i)
			WiiteBVH(bvh_file_name, joint->children[i]);
	}
	if (joint->has_site == true)
	{
		outfile << "End Site" << endl;
		outfile << "{" << endl;
		b = joint->site;
		outfile << "OFFSET " << b[0] << " " << b[1] << " " << b[2] << endl;
		outfile << "}" << endl;
	}
	outfile << "}" << endl;
}

void   WriteBVH(const char *bvh_file_name,const BVH &A)
{	Joint *joint = A.GetJoint(0);
	ofstream outfile(bvh_file_name);

	if (outfile.is_open())
	{
		outfile << "HIERARCHY" << endl;
		WiiteBVH(bvh_file_name, joint);
	}
	outfile.close();
	outfile.open(bvh_file_name,ios::app);
	if (outfile.is_open())
	{
		outfile << fixed << std::setprecision(6);
		outfile << "MOTION" << endl;
		outfile << "Frames:    " <<A.GetNumFrame()<< endl;
		outfile << "Frame Time:    " << A.GetInterval() << endl;
		for (int i = 0; i != A.GetNumFrame(); ++i)
		{
			for (int j = 0; j != A.Getnumchannel(); ++j)
			{
              outfile <<A.GetMotion(i,j)<<"    ";
			}
			outfile << endl;
		}
		

	}
	outfile.close();
	return ;
}
