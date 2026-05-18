#include <mutex>
#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <zmq.hpp>

std::mutex cout_mutex;

int pub()
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
      {
         std::lock_guard<std::mutex> lock(cout_mutex);
         std::cout << "thread Published: " << topic << " - " << message << std::endl;
      }
      std::this_thread::sleep_for( std::chrono::seconds(3) );
   }

   return 0;
}

int sub()
{
   zmq::context_t context(1);
   zmq::socket_t  socket(context, zmq::socket_type::sub);
   socket.connect("tcp://localhost:6666");
   socket.set(zmq::sockopt::subscribe, "Topic");
   int count = 0;
   while (true) {
      zmq::message_t topicMsg;
      zmq::recv_result_t res_topic = socket.recv(topicMsg, zmq::recv_flags::none);
      std::string topic(static_cast<char*>(topicMsg.data()), topicMsg.size());
      zmq::message_t messageMsg;
      zmq::recv_result_t res_message = socket.recv(messageMsg, zmq::recv_flags::none);
      std::string message(static_cast<char*>(messageMsg.data()), messageMsg.size());
      {
         std::lock_guard<std::mutex> lock(cout_mutex);
         std::cout << "thread Received: " << topic << " - " << message << std::endl;
      }
   }

   return 0;
}

int main()
{
   std::thread pub_thread(pub);
   std::thread sub_thread(sub);

   pub_thread.join();
   sub_thread.join();

   return 0;
}
