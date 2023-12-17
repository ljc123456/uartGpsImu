#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <pthread.h>
#include <chrono>
#include <list>
#include <thread>
#include <mutex>

using namespace std;
#define MAXLEN 32

#define PCSIM
#ifdef PCSIM
const std::string com_gps="/dev/ttyUSB0";
const std::string com_imu="/dev/ttyUSB1";
#else
const std::string com_gps="/dev/ttyS1";
const std::string com_imu="/dev/ttyS3";
#endif

std::string imlogName;
std::string gpslogName;

vector<list<string>> msgList(MAXLEN);
int r_in=0;
int r_out=0;

void writeGpslog()
{
    std::ofstream gpsfile(gpslogName, std::ios::app); 

    while(true)
    {    
        if(r_in!=r_out)
        {
            if(msgList.at(r_out).size()>0)
            {
                string msg;
                for (const auto& element : msgList.at(r_out)) {
                    msg=msg+element;  
                }
                gpsfile<<msg;
                msgList.at(r_out).clear();
            }
		    r_out++;
            r_out=r_out%MAXLEN;          
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }    
    gpsfile.close();
}

// void *serial_lsd(void *d) //线程函数，串口输出流水灯
// {
//     int i = 0;
//     int tfd = *(int *)d;
//     if (tfd > 0)
//     {
//         while (1)
//         {
//             char buf[20], str[] = "0000000000";
//             if (i < 10)
//             {
//                 str[i] = '*';
//             }
//             else
//             {
//                 str[19 - i] = '*';
//                 i %= 20;
//             }
//             sprintf(buf, "%s\r\n", str);
//             write(tfd, buf, strlen(buf));
//             usleep(100000); // 延时100ms
//         }
//     }
//     return 0;
// }
 
int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    /* 五个参量 fd打开文件 speed设置波特率 bit数据位设置   nevent奇偶校验位 stop停止位 */
    struct termios newtio, oldtio; // 串口配置结构体
    /*
        struct termios
    {
            tcflag_t c_iflag;
            tcflag_t c_oflag;
            tcflag_t c_cflag;
            tcflag_t c_lflag;
            cc_t           c_cc[NCCS];
    };
    上面五个结构成员名称分别代表：
    c_iflag：输入模式
    c_oflag：输出模式
    c_cflag：控制模式
    c_lflag：本地模式
    c_cc[NCCS]：特殊控制模式
     */
    if (tcgetattr(fd, &oldtio) != 0) //获取当前设置
    /*
    tcgetattr可以初始化一个终端对应的termios结构，tcgetattr函数原型如下：
    #include<termios.h>
    int tcgetattr(int fd, struct termios *termios_p);
    这个函数调用把低昂前终端接口变量的值写入termios_p参数指向的结构。如果这些值其后被修改了，可以通过调用函数tcsetattr来重新配置。
    tcsetattr函数原型如下：
    #include<termios.h>
    int tcsetattr(int fd , int actions , const struct termios *termios_h);
    参数actions控制修改方式，共有三种修改方式，如下所示：
    TCSANOW：立刻对值进行修改
    TCSADRAIN：等当前的输出完成后再对值进行修改
    TCSAFLUSH：等当前的输出完成之后，再对值进行修改，但丢弃还未从read调用返回的当前的可用的任何输入。
    在我们的代码中，我们设置为NOW立即对值进行修改。
 
     */
    {
        perror("SetupSerial 1");
        printf("tcgetattr( fd,&oldtio) -> %d\n", tcgetattr(fd, &oldtio));
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
 
    /*步骤一，设置字符大小*/
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE; //屏蔽数据位
    /*
    c_cflag代表控制模式
    CLOCAL含义为忽略所有调制解调器的状态行，这个目的是为了保证程序不会占用串口。
    CREAD代表启用字符接收器，目的是是的能够从串口中读取输入的数据。
    CS5/6/7/8表示发送或接收字符时使用5/6/7/8比特。
    CSTOPB表示每个字符使用两位停止位。
    HUPCL表示关闭时挂断调制解调器。
    PARENB：启用奇偶校验码的生成和检测功能。
    PARODD：只使用奇校验而不使用偶校验。
 
     */
    /*设置停止位*/
    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8; // 数据位为 8
        break;
    }
    /*设置奇偶校验位*/
    switch (nEvent)
    {
    case 'O':                     //奇数
        newtio.c_cflag |= PARENB; //有校验
        newtio.c_cflag |= PARODD; // 奇校验
        newtio.c_iflag |= (INPCK | ISTRIP);
        /*
        c_iflag代表输入模式
        BRKINT：当在输入行中检测到一个终止状态时，产生一个中断。
        TGNBRK：忽略输入行中的终止状态。
        TCRNL：将接受到的回车符转换为新行符。
        TGNCR：忽略接受到的新行符。
        INLCR：将接受到的新行符转换为回车符。
        IGNPAR：忽略奇偶校检错误的字符。
        INPCK：对接收到的字符执行奇偶校检。
        PARMRK：对奇偶校检错误作出标记。
        ISTRIP：将所有接收的字符裁减为7比特。
        IXOFF：对输入启用软件流控。
        IXON：对输出启用软件流控。
         */
        break;
    case 'E': //偶数
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;  //有校验
        newtio.c_cflag &= ~PARODD; // 偶校验
        /*
        c_cc特殊的控制字符
 
        标准模式和非标准模式下,c_cc数组的下标有不同的值:
 
        标准模式：
 
        VEOF:EOF字符
        VEOL:EOF字符
        VERASE:ERASE字符
        VINTR:INTR字符
        VKILL:KILL字符
        VQUIT:QUIT字符
        VSTART:START字符
        VSTOP:STOP字符
 
        非标准模式:
 
        VINTR:INTR字符
        VMIN:MIN值
        VQUIT:QUIT字符
        VSUSP:SUSP字符
        VTIME:TIME值
        VSTART:START字符
        VSTOP:STOP字符
 
         */
        break;
    case 'N':                      //无奇偶校验位
        newtio.c_cflag &= ~PARENB; // 无校验
        break;
    }
    /*设置波特率*/
    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        /*
        cfsetispeed和cfsetospeed用来设置输入输出的波特率，函数模型如下：
 
        int cfsetispeed(struct termios *termptr, speed_t speed);
        int cfsetospeed(struct termios *termptr, speed_t speed);
        参数说明：
        struct termios *termptr：指向termios结构的指针
        speed_t speed：需要设置的波特率
        返回值：成功返回0，否则返回-1
 
         */
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
 
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    /*设置停止位*/
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB; // 一位停止位， 两位停止位 |= CSTOPB
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB; //两位停止位
 
    /*设置等待时间和最小接收字符*/
    newtio.c_cc[VTIME] = 0; // 等待时间，单位百毫秒 （读）
    newtio.c_cc[VMIN] = 0;  // 等待时间，单位百毫秒 （读）
 
    /*处理未接收字符*/
    tcflush(fd, TCIFLUSH);
    /*
    tcflush用于清空终端为完成的输入/输出请求及数据，它的函数原型如下：
        int tcflush(int fd, int queue_selector);
        其中queue_selector时控制tcflush的操作，取值可以为如下参数中的一个：
        TCIFLUSH清除正收到的数据，且不会读出来；TCOFLUSH清除正写入的数据，且不会发送至终端；TCIOFLUSH清除所有正在发送的I/O数据。
 
/*激活新配置*/
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    /*
    tcsetattr函数原型如下：
        #include<termios.h>
        int tcsetattr(int fd , int actions , const struct termios *termios_h);
        参数actions控制修改方式，共有三种修改方式，如下所示：
        TCSANOW：立刻对值进行修改
        TCSADRAIN：等当前的输出完成后再对值进行修改
        TCSAFLUSH：等当前的输出完成之后，再对值进行修改，但丢弃还未从read调用返回的当前的可用的任何输入。
        在我们的代码中，我们设置为NOW立即对值进行修改。
 
     */
    printf("set done!\n");
    return 0;
}

int open_port(const std::string &comdev)
{
    /* fd 打开串口 comport表示第几个串口 */ 
    int fd = open(comdev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    // O_RDWR 读写方式打开
    //  O_NOCTTY如果路径名指向终端设备，不要把这个设备用作控制终端。
    // O_NDELAY 非阻塞（默认为阻塞，打开后也可以使用fcntl()重新设置）
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return (-1);
    }
    else
    {
        printf("open tty .....\n"); //板子的端口设置为S7
    }
 
    if (fcntl(fd, F_SETFL, 0) < 0)
        // fd：文件描述符
        //设置：fcntl(fd, F_SETFL, FNDELAY); //非阻塞
        //              fcntl(fd, F_SETFL, 0); // 阻塞
        printf("fcntl failed!\n");
    else
        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    if (isatty(STDIN_FILENO) == 0)
        // isatty()函数：判断文件描述词是否是为终端机，如果为终端机则返回1, 否则返回0。
        // STDIN_FILENO：接收键盘的输入
        // STDOUT_FILENO：向屏幕输出
        printf("standard input is not a terminal device\n");
    else
        printf("isatty success!\n");
    printf("fd-open=%d\n", fd);
    return fd;
}

int init_serial(const std::string &com,const int bps)
{
    int fd,i;
    if ((fd = open_port(com)) < 0)
    {
        perror("open_port error");
        return 0;
    }

    if ((i = set_opt(fd, bps, 8, 'N', 1)) < 0) //设置需要与串口调试器保持一致
    {
        perror("set_opt error");
        return 0;
    }

    return fd;
}

void str_split(const string &s,vector<string> &res)
{
    string t;
    stringstream ss;
    ss << s;

    while (getline(ss, t, ','))
    {
        res.push_back(t);
    }

    for (auto s : res)
        cout << s << endl;
}


list<string> clock_msgList;

bool updateClock(string &buffline)
{
    bool retFlag=false;
    size_t  pos= buffline.find("$GNRMC");

    std::cout<<"pos:"<<pos<<std::endl;
    std::cout<<buffline<<std::endl;
    if (pos != std::string::npos)
    {                                
        buffline=buffline.substr(pos,buffline.size()-pos);
        size_t  nextpos= buffline.find("*");
        std::cout<<"npos:"<<nextpos<<std::endl;
        if(nextpos!=std::string::npos)
        {
            std::cout<<"ok"<<std::endl;
            std::cout<<buffline<<std::endl;

            vector<string> res_str;
            str_split(buffline,res_str);

            if(res_str.size()>10) 
            {
                cout<<res_str.at(9).size()<<","<<res_str.at(1).size()<<endl;

                if((res_str.at(9).size()>=6) && (res_str.at(1).size()>=10))
                {
                    string t_string="20"+res_str.at(9).substr(4,2)+"-"
                                                    +res_str.at(9).substr(2,2)+"-"
                                                    +res_str.at(9).substr(0,2)
                                                    +" "
                                                    +res_str.at(1).substr(0,2)+":"
                                                    +res_str.at(1).substr(2,2)+":"
                                                    +res_str.at(1).substr(4,2);
                    std::cout<<t_string<<endl;
                                
                    struct tm time_tm;
                    time_t timep;
                    strptime(t_string.c_str(), "%Y-%m-%d %H:%M:%S", &time_tm);

                    time_tm.tm_wday = 0;
                    time_tm.tm_yday = 0;
                    time_tm.tm_isdst = 0;


                    long time_local_sec = 0;
                    long time_utc_sec = 0;
                    long utc_diff_sec = 0;
                    struct tm tm_local {};
                    struct tm tm_utc {};
                    time_t time_now = time(NULL);
                    gmtime_r(&time_now, &tm_utc);
                    localtime_r(&time_now, &tm_local);
    
                    time_local_sec = tm_local.tm_sec +
                                                    60*(tm_local.tm_min +
                                                        60*(tm_local.tm_hour +
                                                            24*(tm_local.tm_yday +
                                                                365*tm_local.tm_year)));
    
                    time_utc_sec = tm_utc.tm_sec +
                                                60*(tm_utc.tm_min +
                                                    60*(tm_utc.tm_hour +
                                                        24*(tm_utc.tm_yday +
                                                            365*tm_utc.tm_year)));
    
                    utc_diff_sec = time_local_sec - time_utc_sec;


                    utc_diff_sec = 8*3600;

                    time_t time_fixed = mktime(&time_tm) + utc_diff_sec;

                    std::tm* now_t = std::localtime(&time_fixed);  

                    std::stringstream ss;
                    ss << std::put_time(now_t, "%F %T");
                    std::string str = ss.str();

                    string cmd="date --set=\""+str+"\"";

                    system(cmd.c_str());
                                
                    retFlag=true;
                }
            }
        }
    }  
    return retFlag;
}

bool processMsgList()
{
    bool exitFlag=false;  
    int cnt=clock_msgList.size();
    if(cnt>10)
    {
        printf("%d\n\r",cnt); 
        string msg;
        for(int i=0;i<cnt;i++)
        {
		    msg=msg+clock_msgList.front();
		    clock_msgList.pop_front(); 
            size_t pos=msg.find("$GNRMC");
            if(pos!=std::string::npos)
            {                
                size_t epos=msg.find("$G",pos+6);
                if(epos!=std::string::npos)
                {
                    cout<<"start:"<<pos<<endl;
                    cout<<"end:"<<epos<<endl;
                    if(epos>pos)
                    {
                        msg=msg.substr(pos,epos-pos);                    

                        printf("====%s\n\r",msg.c_str()); 
                        exitFlag=updateClock(msg); 
                        break; 
                    } 
                }
            }   
	    }
    }
    return exitFlag;
}

/// @brief 
/// @return 
int getFNRMC()
{    
    bool exitFlag=false;
    int fd;
    int nread, i;
    fd_set rd;
    
    bool startAdd=false;

    fd= init_serial(com_gps,460800);   
    if(fd<=0)
    {
        return 1;
    }
    FD_ZERO(&rd);   
    
    while (1)
    {
        FD_SET(fd, &rd);
        if (select(fd + 1, &rd, NULL, NULL, NULL) <= 0)    // select函数做I/O复用，判断所监测的文件描述符有没有准备好，如果没问题返回0.
        {
            printf("------\n\r");
            perror("select");
        }
        else
        {        
            if (FD_ISSET(fd, &rd)) {//若i在集合中，说明描述符i就绪
                
                char buff[64] = {0};
                int nread = read(fd, buff, 32);
                if (nread < 0) {//接收数据出错
                    perror("recv error!\n");
                    FD_CLR(fd, &rd);
                    return -1;
                }
                printf("%s\n\r",buff);

                clock_msgList.push_back(buff);   

                if(clock_msgList.size()>12)
                {
                    exitFlag=processMsgList();
                }
                //mutexClock.unlock(); 
                if(exitFlag)
                {
                    break;
                }
            }
        }
    }
 
    close(fd);
    return 0;
}

int main(void)
{
    //=======校准时钟==========
    getFNRMC();
    clock_msgList.clear();
    //=======================
    
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::cout  << std::put_time(std::localtime(&t), "%F_%T") << std::endl;
 
	//转为字符串
	std::stringstream ss;
	ss << std::put_time(std::localtime(&t), "%F_%T");
	std::string str = ss.str();

    #ifdef PCSIM
        imlogName="../data/"+str+"_im.txt";
        gpslogName="../data/"+str+"_gps.txt";
    #else
        imlogName="/userdata/"+str+"_im.txt";
        gpslogName="/userdata/"+str+"_gps.txt";
    #endif

    //std::cout<<unlink("../data/"+str+"im.txt")<<std::endl;
    //std::cout<<unlink("../data/gps.txt")<<std::endl;


    std::ofstream imfile(imlogName, std::ios::app);
    

    int fd0,fd1;
    int nread, i;
    //char buff0[1024] = "Hello\n";
    //char buff1[1024] = "Hello\n";
    //char *b = buff0;
    int j;
    fd_set rd;

    

    fd0= init_serial(com_gps,460800);    
    if(fd0<=0)
    {
        return 1;
    }
    
    std::thread gpslog(&writeGpslog);
	gpslog.detach();

    fd1= init_serial(com_imu,115200);    
    if(fd1<=0)
    {
       return 1;
    }
 
    //pthread_t id;
    //thread_create(&id, NULL, serial_lsd, &fd);
 
    FD_ZERO(&rd);
    
    bool gpsStartWrite=false;

    while (1)
    {
        FD_SET(fd0, &rd);
        FD_SET(fd1, &rd); 
        int maxfd = (fd0 > fd1)?fd0:fd1;

        fd_set tmp_inset=rd;
        if (select(maxfd + 1, &rd, NULL, NULL, NULL) <= 0)
        // select函数做I/O复用，判断所监测的文件描述符有没有准备好，如果没问题返回0.
        {
            printf("------\n\r");
            perror("select");
        }
        else
        {
            //有描述符就绪
            for (int i = 0; i <= maxfd; ++i) 
            {
                if (FD_ISSET(i, &rd)) {//若i在集合中，说明描述符i就绪
                    char buf[128] = {0};
                    int nread = read(i, buf, 32);
                    if (nread < 0) {//接收数据出错
                        perror("recv error!\n");
                        FD_CLR(i, &rd);
                        return -1;
                    }

                    auto now = std::chrono::system_clock::now();
                    std::time_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
                    auto now_c = std::chrono::system_clock::to_time_t(now);
                    std::tm* now_t = std::localtime(&now_c);                   

                    if(i==fd0)
                    {
                        bool overFlag=false;
                        if(nread!=32)
                        {
                            overFlag=true;
                        }
                        if(!gpsStartWrite)
                        {
                            std::string buffline=buf;
                            size_t  pos= buffline.find("$GNGGA");
                            if (pos != std::string::npos)
                            {                                
                                gpsStartWrite=true;
                            }                            
                        }

                        if(gpsStartWrite)
                        {
			                //printf("(%02x)\n\r",buf[31]);	
                            std::cout<<buf<<" @"<<r_in<<","<<msgList.at(r_in).size()<<"("<<nread;     
                            msgList.at(r_in).push_back(buf);                     
                        }

                        if(overFlag)
                        {
                            overFlag=false;
                        }
                    }
                    else
                    {
                        if(msgList.at(r_in).size()>0)
                        {
                            r_in++;
                            r_in=r_in%MAXLEN;
                        }
                    }

                    if(i==fd1)
                    {
                        printf("%4d-%02d-%02d_%02d:%02d:%02d.%03d ",now_t->tm_year+1900,
                                                    now_t->tm_mon+1,
                                                    now_t->tm_mday,
                                                    now_t->tm_hour,
                                                    now_t->tm_min,
                                                    now_t->tm_sec, 
                                                    (int)(timestamp%1000));

                        imfile<<std::dec<<std::setfill('0')<<std::setw(3)<<std::put_time(std::localtime(&now_c),"%Y-%m-%d_%H:%M:%S.")<<timestamp%1000<<" ";
                        for (int n = 0; n < nread; n++)
                        {
                            printf("%02hhx", buf[n]);
                            int ch=buf[n]&0xff;
                            //imfile<<std::uppercase<<std::hex<<std::setfill('0')<<std::setw(2)<<ch;
                            imfile<<std::hex<<std::setfill('0')<<std::setw(2)<<ch;
                        }

                        std::cout<<("\n");
                        imfile<<"\n";

                        //std::cout<<("]\n");
                    }
                }
            }  
        }
    }
 
    close(fd0);
    close(fd1);

    imfile.close();
    

    return 1;
}
