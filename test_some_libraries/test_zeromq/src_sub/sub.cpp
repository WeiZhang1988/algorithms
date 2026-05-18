#include <string>
#include <iostream>
#include <zmq.hpp>
int main()
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

      std::cout << "Received: " << topic << " - " << message << std::endl;
   }

   return 0;
}
