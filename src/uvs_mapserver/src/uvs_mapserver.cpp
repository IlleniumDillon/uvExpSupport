#include "uvs_mapserver.hpp"


UVSMapServer::UVSMapServer()
    : Node("uvs_mapserver")
{
    loadWorld("jxl3028", world_);
}

UVSMapServer::~UVSMapServer()
{
}


void UVSMapServer::uvQueryElementCallback(const std::shared_ptr<uvs_message::srv::UvQueryElement::Request> request, std::shared_ptr<uvs_message::srv::UvQueryElement::Response> response)
{
}

void UVSMapServer::uvQueryMapCallback(const std::shared_ptr<uvs_message::srv::UvQueryMap::Request> request, std::shared_ptr<uvs_message::srv::UvQueryMap::Response> response)
{
}

void UVSMapServer::uvOptPoseListCallback(const uvs_message::msg::UvOptPoseList::SharedPtr msg)
{
}
