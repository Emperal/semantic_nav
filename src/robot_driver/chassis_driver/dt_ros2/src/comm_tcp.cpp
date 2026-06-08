#include "../include/comm_tcp.h"
#include <rclcpp/rclcpp.hpp>


SHM_ROBOT_STATES * Comm_TCP :: get_arm_states(){
    return p_shared_robot_states_;
}



void Comm_TCP :: close_all(int &fd,int &fd2,modbus_t*&ctx_){
    close(fd);
    close(fd2); 
    modbus_close(ctx_);
    modbus_free(ctx_);
}

int Comm_TCP :: init_driver(int &fd,int &fd2,modbus_t*&ctx_,std::string modbus_ip,int modbus_port,
                            std::string udp_listen_ip,int udp_listen_port,std::string udp_broadcast_ip,
                            int udp_broadcast_port){
// --------- 获取 ROS2 参数 ----------
    // std::string modbus_ip;
    // int modbus_port;
    // std::string udp_listen_ip;
    // int udp_listen_port;
    // std::string udp_broadcast_ip;
    // int udp_broadcast_port;

    // --------- 初始化 Modbus TCP ----------
    ctx_ = modbus_new_tcp(modbus_ip.c_str(), modbus_port);
    if (modbus_connect(ctx_) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("Comm_TCP_init"), "Failed to connect modbus tcp");
        return -1;
    }

    p_shared_cmd_buffer_ = (SHM_COMMAND_INFO *)mem_buf_mbtcp;

    // --------- UDP Socket 1 (用于接收) ----------
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(fd == -1){
        RCLCPP_ERROR(rclcpp::get_logger("Comm_TCP_init"),"Socket failed");
        // perror("socket");
        // exit(-1);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(udp_listen_port);
    inet_pton(AF_INET, udp_listen_ip.c_str(), &addr.sin_addr.s_addr);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        // perror("bind fd");
        // exit(-1);
        RCLCPP_ERROR(rclcpp::get_logger("Comm_TCP_init"),"Socket failed");
        return -1;
    }

    p_shared_robot_states_ = (SHM_ROBOT_STATES *)mem_buf;

    // --------- UDP Socket 2 (用于广播) ----------
    fd2 = socket(AF_INET, SOCK_DGRAM, 0);
    int op = 1;
    setsockopt(fd2, SOL_SOCKET, SO_BROADCAST, &op, sizeof(op));

    memset(&addr2, 0, sizeof(addr2));
    addr2.sin_family = AF_INET;
    addr2.sin_port = htons(udp_broadcast_port);
    inet_pton(AF_INET, udp_broadcast_ip.c_str(), &addr2.sin_addr.s_addr);

    p_shared_stream_cmd_ = (SHM_STREAM_CMD *)mem_buf2;

    return 0;
}

void Comm_TCP :: read_modbustcp_server_simple(modbus_t* ctx_){
    int count = modbus_read_registers(ctx_, 0, 6, mem_buf_mbtcp);
    printf("Read from server simple...%d==%d\n",count,6);
}

int Comm_TCP :: check_state(modbus_t* ctx_,int timeout){
    int count=0;
    while(p_shared_cmd_buffer_->writeflag)
    {
        read_modbustcp_server_simple(ctx_);
        usleep(1000);
        if(timeout>0)
        {
            count++;
            if(count>timeout)
            {
                p_shared_cmd_buffer_->writeflag = 0;
                return -1;
            }
        }
    } 
    return 0;
}

int Comm_TCP :: write_modbustcp_server(modbus_t *ctx_){
    int nb=0;
    //Obtain the required write count
    if(p_shared_cmd_buffer_->size%2==1)
        nb = p_shared_cmd_buffer_->size/2+1;
    else
        nb = p_shared_cmd_buffer_->size/2;
    
    //Send the data region first to ensure safety
    int nb_start=6;
    int nb_count = nb;
    
    int count = 0;
    printf("Begin New Command %d=>%d...\n",p_shared_cmd_buffer_->command,nb);
    while(nb_count>0)
    {
        if(nb_count>nb_block_size)
        {
            count = modbus_write_registers(ctx_, nb_start, nb_block_size,mem_buf_mbtcp+nb_start);
            printf("Write to cmd data to server %d/%d->%d...\n",nb_start,nb_block_size,count);  
            
            if(count!=nb_block_size)
                return -1;            
            nb_start+=nb_block_size;
            nb_count-=nb_block_size;
        }
        else
        {
            count = modbus_write_registers(ctx_, nb_start, nb_count,mem_buf_mbtcp+nb_start);
            printf("Write to cmd data to server %d/%d...\n",count,nb_count);  
            
            if(count!=nb_count)
                return -1;
            nb_start=nb;
            nb_count=0; 
        }
    }
}

int Comm_TCP :: modbus_tcp_send_command(modbus_t *ctx_,int timeout){
       int count=0;
    
    p_shared_cmd_buffer_->writeflag = 1;
    if(-1==write_modbustcp_server(ctx_))
        return -1;
    usleep(1000);
    while(p_shared_cmd_buffer_->writeflag)
    {
        read_modbustcp_server_simple(ctx_);
        usleep(1000);       
        if(timeout>0)
        {
            count++;
            if(count>timeout)
            {
                p_shared_cmd_buffer_->writeflag = 0;
                return -1;
            }
        }
    }
    return 0;
}

int Comm_TCP :: stream_send_start(modbus_t *ctx_,int rob_id, double max_vel, int timeout){
    if(check_state(ctx_,timeout)==-1)
    {
        printf("ERROR:SHM Busy...\n");
        return -1;
    }
    /*****************************************/
    p_shared_cmd_buffer_->command = SHM_CMD_STREAM_JOINT;
    p_shared_cmd_buffer_->size = sizeof(SHM_CMD_STREAM_DATA);
    
    SHM_CMD_STREAM_DATA cmd_data;
    cmd_data.robot_id = rob_id;
    cmd_data.vel = max_vel;    
    memcpy(p_shared_cmd_buffer_->data,&cmd_data,p_shared_cmd_buffer_->size);
    /*****************************************/
    if(modbus_tcp_send_command(ctx_,timeout)==-1)
    {
        printf("ERROR:SHM Command Timeout...\n");
        return -1;
    }  
    return 0; 
}

void Comm_TCP :: stream_buffer(short tick_id, short cmd_type, double * data){
    int count=0;
    double sum=0;
    p_shared_stream_cmd_->tick_id = tick_id;
    
    switch(cmd_type)
    {
        case SHM_STREAM_CMD_JOINT:
        {
            count = MOTOR_COUNT;
            break;   
        }
        case SHM_STREAM_CMD_CART:
        {
            count = ROBOT_TASK_DOF;
            break;   
        }        
        case SHM_STREAM_CMD_WOBJ:
        {
            count = ROBOT_TASK_DOF;
            break;   
        }          
    }
    for(int i=0;i<count;i++)
    {
        p_shared_stream_cmd_->data[i]=data[i];
        sum+=data[i];
    }
    p_shared_stream_cmd_->check_sum = sum;
    p_shared_stream_cmd_->stream_command = cmd_type;
}

void Comm_TCP :: udp_send(int fd2){
    sendto(fd2, (unsigned char *)p_shared_stream_cmd_, sizeof(SHM_STREAM_CMD), 0, (struct sockaddr *)&addr2, sizeof(addr2));
}

void Comm_TCP :: udp_recv(int fd){
    int num = recvfrom(fd, mem_buf,2048, 0, NULL, NULL);
    if (num==-1){
        RCLCPP_ERROR(rclcpp::get_logger("comm_tcp"),"receive error!");
        // perror("bind");
        // exit(-1);
    }
}

    // int Comm_TCP :: init_driver(){
    //     ctx_ = modbus_new_tcp("192.168.0.250", 12345);
    //     modbus_connect(ctx_);
    
    //     p_shared_cmd_buffer_ = (SHM_COMMAND_INFO *)mem_buf_mbtcp;
    
    //     fd = socket(AF_INET, SOCK_DGRAM, 0);
    //     if(fd==-1){
    //         perror("socket");
    //         exit(-1);
    //     }
    
    //     fd2 = socket(AF_INET, SOCK_DGRAM, 0);
    //     int op=1;
    //     setsockopt(fd2, SOL_SOCKET, SO_BROADCAST, &op, sizeof(op));
        
    //     //2.客户端绑定本地IP和端口
    //     addr.sin_family = AF_INET;
    //     addr.sin_port = htons(9999);
    //     addr.sin_addr.s_addr = INADDR_ANY;
    //     //绑定
    //     int ret = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
    //     if (ret==-1){
    //             perror("bind");
    //             exit(-1);
    //     }
    //     p_shared_robot_states_ = (SHM_ROBOT_STATES *)mem_buf;     
    
    
    //     addr2.sin_family = AF_INET;
    //     addr2.sin_port = htons(9998);
    //     //绑定
    //     inet_pton(AF_INET, "192.168.0.255",&addr2.sin_addr.s_addr);
    //     p_shared_stream_cmd_ = (SHM_STREAM_CMD *)mem_buf2;
    // }