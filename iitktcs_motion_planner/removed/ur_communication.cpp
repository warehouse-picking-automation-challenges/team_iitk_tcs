#include <ros/ros.h>
#include <iitktcs_msgs_srvs/CheckClearProtectiveStop.h>

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <fcntl.h>
#include <sys/types.h>


//load
//play
//stop
//pause
//quit
//shutdown
//running
//robotmode

////CB2: integer is returned

////    NO_CONTROLLER_MODE = -1
////    ROBOT_RUNNING_MODE = 0 (This is "normal" mode)
////    ROBOT_FREEDRIVE_MODE = 1
////    ROBOT_READY_MODE = 2
////    ROBOT_INITIALIZING_MODE = 3
////    ROBOT_SECURITY_STOPPED_MODE = 4
////    ROBOT_EMERGENCY_STOPPED_MODE = 5
////    ROBOT_FAULT_MODE = 6
////    ROBOT_NO_POWER_MODE = 7
////    ROBOT_NOT_CONNECTED_MODE = 8
////    ROBOT_SHUTDOWN_MODE = 9

////CB3: text is returned
////"Robotmode: <mode>", where <mode> is

////    NO_CONTROLLER
////    DISCONNECTED
////    CONFIRM_SAFETY
////    BOOTING
////    POWER_OFF
////    POWER_ON
////    IDLE
////    BACKDRIVE
////    RUNNING
//get loaded program
//popup <popup-text>
//close popup
//addToLog <log-message>
//isProgramSaved
//programState
//power on
//power off
//brake release
//safetymode

////"Safetymode: <mode>", where <mode> is

////    NORMAL
////    REDUCED
////    PROTECTIVE_STOP
////    RECOVERY
////    SAFEGUARD_STOP
////    SYSTEM_EMERGENCY_STOP
////    ROBOT_EMERGENCY_STOP
////    VIOLATION
////    FAULT
//unlock protective stop
//close safety popup

std::string cmd_unlock_protective_stop = "unlock protective stop\n";
std::string cmd_close_safety_popup = "close safety popup\n";
std::string cmd_popup = "popup enabling FAIZAL 5 FROM PROTECTIVE STOP\n";
std::string cmd_close_popup = "close popup\n";

class UrCommunication {
public:
    int pri_sockfd_, sec_sockfd_;
    struct sockaddr_in pri_serv_addr_, sec_serv_addr_;
    struct hostent *server_;
    int flag_;

    UrCommunication(std::string host)
    {
        bzero((char *) &pri_serv_addr_, sizeof(pri_serv_addr_));
        pri_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (pri_sockfd_ < 0) {
            std::cout << "ERROR opening socket pri_sockfd\n";
        }

        server_ = gethostbyname(host.c_str());
        if (server_ == NULL) {
            std::cout << "ERROR, unknown host\n";
        }

        pri_serv_addr_.sin_family = AF_INET;
        pri_serv_addr_.sin_port = htons(29999);

        bcopy((char *) server_->h_addr, (char *)&pri_serv_addr_.sin_addr.s_addr, server_->h_length);

        flag_ = 1;
        setsockopt(pri_sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
                   sizeof(int));
        setsockopt(pri_sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_,
                   sizeof(int));
        setsockopt(pri_sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_,
                   sizeof(int));

        uint8_t buf[512];
        unsigned int bytes_read;

        if (connect(pri_sockfd_, (struct sockaddr *) &pri_serv_addr_, sizeof(pri_serv_addr_)) < 0)
        {
            std::cout << "Error connecting to dashboard server\n";
        }
        bytes_read = read(pri_sockfd_, buf, 512);
        std::cout << "DASHBOARD RESPONSE = " <<   buf << "\n";

    }

    ~UrCommunication()
    {
        close(pri_sockfd_);

    }

    bool checkClearProtectiveStopCallback(iitktcs_msgs_srvs::CheckClearProtectiveStop::Request &req,
                                          iitktcs_msgs_srvs::CheckClearProtectiveStop::Response &res,
                                          ros::ServiceClient &client_check_ps)
    {
    //    UrCommunication ur_comm("192.168.2.200");
        iitktcs_msgs_srvs::CheckClearProtectiveStop check_ps;
        check_ps.request.check.data = true;
        int bytes_rw;
        unsigned char buf[512];
//        sleep(2);
        if(client_check_ps.call(check_ps))
        {
            if(check_ps.response.flag_protective_stopped.data)
            {
                // send info that protective stopped had happened
                res.flag_protective_stopped.data = true;

                //        std::cout << "1. close saftey popup\n"
                //                  << "2. unlock protective stop\n";

//                int cmd = 2;
                //        std::cin  >> cmd;

                //            bytes_rw = write(this->pri_sockfd_, cmd_popup.c_str(), cmd_popup.size());
                //            std::cout << "BYTES WRITTEN = " << bytes_rw << "\n";
                //            usleep(10000);

                //            std::cout <<"PRESS ANY KEY and ENTER TO CONTINUE\n" ;
                //            std::cin >>  cmd;

                //            bytes_rw = write(this->pri_sockfd_, cmd_close_popup.c_str(), cmd_close_popup.size());
                //            std::cout << "BYTES WRITTEN = " << bytes_rw << "\n";
                //            usleep(10000);

                bytes_rw = write(this->pri_sockfd_, cmd_unlock_protective_stop.c_str(), cmd_unlock_protective_stop.size());
//                std::cout << "BYTES WRITTEN = " << bytes_rw << "\n";
                usleep(10000);

                bytes_rw = read(this->pri_sockfd_, buf, 512);
//                std::cout << "BYTES READ = " << bytes_rw << "\n";
//                std::cout << "DASHBOARD RESPONSE = " << buf << "\n";

                std::cout << "Protective stop cleared" << std::endl;

                usleep(10000);
                //            bytes_rw = write(this->pri_sockfd_, cmd_popup.c_str(), cmd_popup.size());
                //            std::cout << "BYTES WRITTEN = " << bytes_rw << "\n";
                //            usleep(10000);

                //            std::cout <<"PRESS ANY KEY and ENTER TO CONTINUE\n" ;
                //            std::cin >>  cmd;

                //            bytes_rw = write(this->pri_sockfd_, cmd_close_popup.c_str(), cmd_close_popup.size());
                //            std::cout << "BYTES WRITTEN = " << bytes_rw << "\n";
                //            usleep(10000);

                bytes_rw = write(this->pri_sockfd_, cmd_close_safety_popup.c_str(), cmd_close_safety_popup.size());
//                std::cout << "BYTES WRITTEN = " << bytes_rw << "\n";
                usleep(10000);

                bytes_rw = read(this->pri_sockfd_, buf, 512);
//                std::cout << "BYTES READ = " << bytes_rw << "\n";
//                std::cout << "DASHBOARD RESPONSE = " << buf << "\n";

            }
            else
                res.flag_protective_stopped.data = false;

            return true;
        }
        else
        {
            // In case failed to call service & still you want to clear protective stop, uncomment below
    //            bytes_rw = write(this->pri_sockfd_, cmd_unlock_protective_stop.c_str(), cmd_unlock_protective_stop.size());
    //            //                std::cout << "BYTES WRITTEN = " << bytes_rw << "\n";
    //            usleep(10000);

    //            bytes_rw = read(this->pri_sockfd_, buf, 512);
    //            //                std::cout << "BYTES READ = " << bytes_rw << "\n";
    //            //                std::cout << "DASHBOARD RESPONSE = " << buf << "\n";

    //            std::cout << "Protective stop cleared" << std::endl;

            std::cout << "Failed to call " << client_check_ps.getService() << std::endl;
            return false;
        }
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_communication");
    ros::NodeHandle nh;
    UrCommunication ur_comm("192.168.2.200");

    ros::ServiceClient client_check_protective_stop =
            nh.serviceClient<iitktcs_msgs_srvs::CheckClearProtectiveStop>("/iitktcs/ur_modern_driver/check_protective_stop");

    ros::ServiceServer service_check_clear_protective_stop =
            nh.advertiseService<iitktcs_msgs_srvs::CheckClearProtectiveStop::Request, iitktcs_msgs_srvs::CheckClearProtectiveStop::Response>
            ("iitktcs/motion_planner/check_clear_protective_stop",
             boost::bind(&UrCommunication::checkClearProtectiveStopCallback, &ur_comm, _1, _2,
                         boost::ref(client_check_protective_stop)));

    while(ros::ok())
        ros::spinOnce();
}
