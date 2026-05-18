#include <mutex>
#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <zmq.hpp>
#include "message.pb.h"

std::mutex cout_mutex;

zmq::context_t context(2);

int srv()
{
   GOOGLE_PROTOBUF_VERIFY_VERSION;
   zmq::socket_t  socket(context, zmq::socket_type::rep);
   socket.bind("inproc://test");
   std::this_thread::sleep_for( std::chrono::seconds(3) );
   while (true) {
      zmq::message_t reqtMsg;
      zmq::recv_result_t res_req = socket.recv(reqtMsg, zmq::recv_flags::none);
      if (!res_req) continue;
      protomsg::ProtoRequest req;
      req.ParseFromArray(reqtMsg.data(), reqtMsg.size());
      {
         std::lock_guard<std::mutex> lock(cout_mutex);
         std::cout << "srv receive cmd: " << req.cmd() << ", num: " << req.num() << std::endl;
      }
      protomsg::ProtoReply rep;
      rep.set_rep("ok:" + req.cmd());
      rep.set_res(req.num() * 2);
      socket.send(zmq::buffer(rep.SerializeAsString()), zmq::send_flags::none);
   }

   return 0;
}

int cli()
{
   zmq::socket_t  socket(context, zmq::socket_type::req);
   socket.connect("inproc://test");
   int count = 0;
   while (true) {
      {
         std::lock_guard<std::mutex> lock(cout_mutex);
         std::cout << "cle request " << ++count << " times " << std::endl;
      }
      protomsg::ProtoRequest req;
      req.set_cmd("double");
      req.set_num(6);
      socket.send(zmq::buffer(req.SerializeAsString()), zmq::send_flags::none);
      zmq::message_t repMsg;
      zmq::recv_result_t res_rep = socket.recv(repMsg, zmq::recv_flags::none);
      if (!res_rep) continue;
      protomsg::ProtoReply rep;
      rep.ParseFromArray(repMsg.data(), repMsg.size());
      {
         std::lock_guard<std::mutex> lock(cout_mutex);
         std::cout << "reply: rep: " << rep.rep() << ", res: " << rep.res() << std::endl;
      }
      std::this_thread::sleep_for( std::chrono::seconds(3) );
   }

   return 0;
}

int main()
{
   std::thread srv_thread(srv);
   std::thread cli_thread(cli);

   srv_thread.join();
   cli_thread.join();

   return 0;
}
