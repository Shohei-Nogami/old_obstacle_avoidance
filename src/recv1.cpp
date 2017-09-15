/* ----------------------------------
   受信用
   起動順番：Linux　-> Windows の順番

--------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include "ros/ros.h"
#include "obst_avoid/wheel_msg.h"
#define BUFFER_SIZE 2048 //256 = 1 byte, 64bit OS -> 2048 , 32bit OS -> 1024
#define DATABUFFER 1024 
#define IP_Number "192.168.0.2"


  /* エンコーダデータ */

    struct data_encoder{
             char d_x[DATABUFFER];
             char d_y[DATABUFFER];
             };

/* ポート番号、ソケット */
 unsigned short port = 8000; //9876; 受信用
 unsigned short ports = 8001; //9876; 送信用
 
 int recvSocket; 
 int sendSocket;
 
 int values_l; // 左車輪エンコーダ
 int values_r; // 右車輪エンコーダ

 int pwm_l = 0; // 左車輪用PWM制御信号 [0 255]
 int pwm_r = 0; // 右車輪用PWM制御信号 [0 255]
 struct data_encoder de; //エンコーダデータ用変数

/* ----------------------------------------------------              
              sockaddr_in 構造体  

 struct sockaddr_in
 {  
    short sin_family; AF_INETアドレスファミリ(IPv4を表す)  
    unsigned short sin_port;  IPポート番号
    struct in_addr sin_addr;  IPアドレス（送受信用)
    char sin_zero[8];  
 };  
inet_addr(*) : *の文字列を32ビットのバイナリ値に変換
                        
----------------------------------------------------  */

//struct sockaddr_in recvSockAddr;// 受信用

 struct sockaddr_in recvSockAddr; // 受信用
 struct sockaddr_in from_addr;    // 送信元のアドレスC itoa
 struct sockaddr_in addr; // 送信用
 
 socklen_t sin_size;
 int sock; //受信用変数

/* 各種パラメータ */
 int status, status_s;
 int numrcv, numscv;
 char buffer[BUFFER_SIZE];
 unsigned long on = 1;
void pwmCallback(const obst_avoid::wheel_msg::ConstPtr& msg);

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "udp_recv_node");
 values_l = 0; 
 values_r = 0; 
/* ------------------------------------------
    sockaddr_in 構造体のセット ：受信用 

  memset(*s, int char, unsigned n); -> メモリ初期化
  *s: メモリブロックのアドレス, char:文字, n: ブロックサイズ
   *s のアドレスからn個分charで指定した文字を代入する

 ------------------------------------------------ */
 memset(&recvSockAddr, 0, sizeof(recvSockAddr));// &recvSockAddrのアドレスから，sizeof(recvSockAddr)サイズ分だけ0を代入

 // 受信用のポート・アドレス設定

 recvSockAddr.sin_port = htons(port); // ポート番号
 recvSockAddr.sin_family = AF_INET; // 通信種類 AF_INET 通常の通信
 recvSockAddr.sin_addr.s_addr = inet_addr("192.168.0.2"); //htonl(192.168.0.2); //INADDR_ANY

/* sockaddr_in 構造体のセット ：送信用*/

 memset(&addr, 0, sizeof(addr));
 
 addr.sin_port = htons(ports); //ポート番号
 addr.sin_family = AF_INET; //通信種類
 addr.sin_addr.s_addr = inet_addr("192.168.0.3"); //htonl(192.168.0.3); //INADDR_ANY


/* ソケット生成 */

 recvSocket = socket(AF_INET, SOCK_DGRAM, 0); //受信 (通信種類(IPv4), 通信形式（タイプ):UDP = SOCK_DGRAM,プロトコル種類(0:自動設定)
 sendSocket = socket(AF_INET, SOCK_DGRAM, 0); //送信

 /* バインド */
 status = bind(recvSocket, (const struct sockaddr *) &recvSockAddr, sizeof(recvSockAddr));
  if(status < 0)
    {
      perror("bind() failed" );
       return -1;
     }

 ros::NodeHandle nh;
 ros::Subscriber sub_pwm;
 ros::Publisher pub_encoder;
 std::cout<<"ready subscribe\n";
 sub_pwm=nh.subscribe("wheel_data",1,pwmCallback);
 ros::spin();
  //ソケットのクローズ
    close(recvSocket);
    close(sock);
}
void pwmCallback(const obst_avoid::wheel_msg::ConstPtr& msg){
/* パケット受信 */
	std::cout<<"msg is "<<msg->vel_data.c_str()<<'\n';
   numrcv = recvfrom(recvSocket, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&from_addr, &sin_size);
    if(numrcv < 0){
           status = close(recvSocket); 
           printf("numrcv = %d\n\n", numrcv); 
           return ;//-1;
        }
     //文字を数字に変える
       sock = atoi(buffer);
       printf("receave data = %d\n", sock);

  /* エンコーダデータ */
//    values_l = values_l + 1; 
//    values_r = values_r + 2;
	
	std::strcpy(de.d_x,msg->vel_data.c_str());
     printf("sending data = %s\n",de.d_x);
  /* 送信 */

   numscv = sendto(sendSocket, de.d_x, sizeof(de.d_x), 0, (struct sockaddr *)&addr, sizeof(addr));
   printf("numscv = %d\n",numscv);
    if (numscv != sizeof(de.d_x))
    {
      perror("sendto() failed" );
       return ;//-1;
     }

// }
}
