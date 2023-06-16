#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

int main(int argc, char** argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "tcp_bool_subscriber");
    ros::NodeHandle nh;

    // 토픽 발행자 생성
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("received_topic", 10);

    // TCP 서버 설정
    boost::asio::io_service io_service;
    tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), 1200));

    try
    {
        // 클라이언트 연결 대기
        tcp::socket socket(io_service);
        acceptor.accept(socket);
        std::cout << "클라이언트 연결됨: " << socket.remote_endpoint() << std::endl;

        // 데이터 수신 및 토픽 발행
        while (ros::ok())
        {
            // 데이터 수신
            boost::asio::streambuf buffer;
            boost::asio::read_until(socket, buffer, '\n');
            std::istream is(&buffer);
            std::string data;
            std::getline(is, data);

            // 데이터를 std_msgs::Bool 형태로 변환
            std_msgs::Bool msg;
            if (data == "true")
                msg.data = true;
            else if (data == "false")
                msg.data = false;
            else
                continue;  // 잘못된 데이터인 경우 건너뛰기

            // ROS 토픽에 데이터 발행
            pub.publish(msg);
        }

        // 소켓 종료
        socket.close();
    }
    catch (const boost::system::system_error& e)
    {
        ROS_ERROR("TCP connection error: %s", e.what());
    }

    return 0;
}
