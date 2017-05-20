#ifndef _BVH_H_
#define _BVH_H_

#include <vector>
#include <map>
#include <string>
using namespace std;

enum ChannelEnum
{
	X_ROTATION,Y_ROTATION,Z_ROTATION,
	X_POSITION,Y_POSITION,Z_POSITION
};

struct Channel;

struct Joint
{
	string            type;        //关节类型
	string            name;       //关节名字
	int               index;      //关节序号
	Joint*            parent;     //父关节
	vector<Joint*>    children;   //子关节
	double            offset[3];  //根节点初始位置及相对位置
	bool              has_site;   //是否是末节点
	double            site[3];    //末端位置
	vector<Channel*>  channels;   //通道信息
};

struct Channel
{
	Joint*           joint;      //所属关节
	ChannelEnum      type;       //通道类型
	string           name;       //通道名字
	int              index;      //通道序号

};

class BVH
{
public:
	BVH();
	BVH(const char* bvh_file_name);
	~BVH();
	void Clear();
	void Load(const char* bvh_file_name);
	//是否加载成功
	bool IsLoadSuccess() const { return is_load_success; }
	//获取文件路径，文件名
	const string& Getfilename() const { return file_name; }
	const string& Getmotionname() const { return motion_name; }
	//获取关节及通道信息
	const int GetnumJoint() const { return Joints.size(); }
	 Joint* GetJoint(int no) const { return Joints[no]; }
    double* GetOffsite(int no)const { return (Joints[no])->offset; }
	double* GetSite(int no)const { return (Joints[no])->site; }
	const Joint* GetJoint(const string &j) const {
		map<string, Joint*>::const_iterator i = joint_index.find(j);
		return (i != joint_index.end()) ? (*i).second : NULL;
	}
	const Joint* GetJoint(const char* j) const {
		map<string, Joint*>::const_iterator i = joint_index.find(j);
		return (i != joint_index.end()) ? (*i).second : NULL;
	}
	const int Getnumchannel() const { return channels.size(); }
	const Channel* GetChannel(int no) const { return channels[no]; }
	//获取具体数据信息
	int    GetNumFrame() const { return num_frame; }
	double GetInterval() const {return interval;   }
	int    GetnumNodes_Line()  const { return  nodes_line.size(); }
	vector<int> Getnodes_Line(int no)   const { return nodes_line[no]; }
	double GetMotion(int f, int c) const { return motion[f*num_channels + c]; }
	//double* GetMotion(int* no);
	//double* GetMotion() { return motion; 
	//修改数据信息
	void   SetNumframe(int n)    { num_frame = n; }
	void   ResizeMotion(int m, int n) { motion=new double[m*n]; }
	void   SetMotion(int f, int c, double v)  { motion[f*num_channels + c] = v; }
	//写出bvh文件
	friend void  WriteBVH(const char *bvh_file_name,const BVH &A);
/*	void   WriteBrace(FILE *file);
	void   WriteJoint(FILE *file, Joint* joint);
	void   Writeframe(FILE *file, int num_frame, double intervel);
	void   WriteMotion(FILE *file, double* motion);*/
private:
	bool            is_load_success;     //是否加载成功
	string          motion_name;         //文件名
	string          file_name;           //文件路径名
	int             num_channels;        //通道总数
	vector<Channel*> channels;           //所有通道信息
	vector<Joint*>   Joints;             //所有关节信息
	map<string, Joint*>   joint_index;   //关节名字对应该关节数据结构
	vector<vector<int> >  nodes_line;    //骨骼链
	int             num_frame;           //帧数
	double          interval;            //频率
	double*         motion;              //运动数据
	//double*        coordinate          //每帧每个关节位置坐标
};


#endif