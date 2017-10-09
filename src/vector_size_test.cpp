#include<ros/ros.h>

int main(int argc,char **argv){
    ros::init(argc,argv,"vector_size_test");
    std::vector<int> val1{1, 2, 3};
    std::vector<int> val2;

    std::cout<<"size of val1 :"<<val1.size()<<std::endl;
    std::cout<<"size of val2 which wasn't done reserve(3000) :"<<val2.size()<<std::endl;
    std::cout<<"size of val2 wheater val2 is empty before reserve:"<<val2.empty()<<std::endl;
    val2.reserve(3000);
    std::cout<<"size of val2 which was done reserve(3000) :"<<val2.size()<<std::endl;
    std::cout<<"size of val2 wheater val2 is empty :"<<val2.empty()<<std::endl;
    for(int i=0;i<100;i++)
        val2.push_back(i);
    std::cout<<"size of val2 which was done push_back(i). i:0~99 size:"<<val2.size()<<std::endl;
    std::cout<<"size of val2 which was done push_back(i). end-begin :"<<val2.end()-val2.begin()<<std::endl;
    val1.resize(3000);
    std::cout<<"size of val1 which was done resize(3000) :"<<val1.size()<<std::endl;
    std::cout<<"size of val1 which was done resize(3000) end-begin :"<<val1.end()-val1.begin()<<std::endl;

    val1.clear();
    std::cout<<"size of val1 which was done clear() :"<<val1.size()<<std::endl;

    val2.shrink_to_fit();//キャパシティにフィットしたサイズまで縮小
    std::cout<<"size of val2 which was done shrink_to_fit():"<<val2.size()<<std::endl;
    val2.reserve(3000);
    std::cout<<"size of val2 which was done reserve(3000):"<<val2.size()<<std::endl;
    for(int i=0;i<100;i++)
        val2.push_back(i);
    std::cout<<"size of val2 which was done pushback(i). i:0~99 :"<<val2.size()<<std::endl;
	for(int i=0;i<100;i++)
		val1[i]=i;
    std::cout<<"size of val1 which was done val1[i]=i :"<<val1.size()<<std::endl;
	
    return 0;
}
