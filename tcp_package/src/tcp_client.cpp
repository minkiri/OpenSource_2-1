#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

int main(int argc, char** argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "tcp_bool_publisher");
    ros::NodeHandle nh;

    // 토픽 발행자 생성
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("tcp_topic", 10);

    // TCP 클라이언트 설정
    boost::asio::io_service io_service;
    tcp::socket socket(io_service);
    tcp::resolver resolver(io_service);
    tcp::resolver::query query("localhost", "1200");  // 서버 주소와 포트 번호 지정
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    try
    {
        // 서버에 연결
        boost::asio::connect(socket, endpoint_iterator);

        // 데이터 전송
        while (ros::ok())
        {
            std_msgs::Bool msg;
            msg.data = true;  // 전송할 데이터 설정

            // 메시지를 문자열로 직렬화
            std::ostringstream oss;
            oss << msg.data << '\n';
            std::string message = oss.str();

            // 데이터 전송
            boost::asio::write(socket, boost::asio::buffer(message));

            // ROS 토픽에도 데이터 발행
            pub.publish(msg);

            ros::spinOnce();
            ros::Rate(10).sleep();  // 10Hz로 제한
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
