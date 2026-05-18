#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <zmq.hpp>
int main()
{
   zmq::context_t context(1);
   zmq::socket_t  socket(context, zmq::socket_type::pub);
   socket.bind("tcp://*:6666");
   std::this_thread::sleep_for( std::chrono::seconds(3) );
   int count = 0;
   while (true) {
      std::string topic = "Topic";
      std::string message = "Message " + std::to_string(count++);
      zmq::message_t topicMsg(topic.size());
      memcpy(topicMsg.data(), topic.data(), topic.size());
      socket.send(topicMsg, zmq::send_flags::sndmore);
      zmq::message_t messageMsg(message.size());
      memcpy(messageMsg.data(), message.data(), message.size());
      socket.send(messageMsg, zmq::send_flags::none);
      std::cout << "Published: " << topic << " - " << message << std::endl;
      std::this_thread::sleep_for( std::chrono::seconds(3) );
   }

   return 0;
}
