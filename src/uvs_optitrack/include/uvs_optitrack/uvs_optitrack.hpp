#ifndef UVS_OPTITRACK_HPP
#define UVS_OPTITRACK_HPP

#include "rclcpp/rclcpp.hpp"

#include <uvs_message/msg/uv_opt_pose_list.hpp>

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <map>
#include <string>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <memory>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

constexpr ConnectionType defaultConnectionType = ConnectionType_Multicast;
constexpr int maxMessageLength = 256;

class UvsOptitrack : public rclcpp::Node
{
public:
    UvsOptitrack();
    ~UvsOptitrack();

    bool tryConnect();
    void spin_once();

private:
    static void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);
    static void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);
    static void NATNET_CALLCONV ServerDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext);

    bool updateDataDescriptions();
private:
    rclcpp::Publisher<uvs_message::msg::UvOptPoseList>::SharedPtr publisher;

    std::shared_ptr<NatNetClient> pClient;
    sNatNetClientConnectParams connectParams;
    sServerDescription serverDescription;

    std::vector<sNatNetDiscoveredServer> discoveredServers;
    char discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;

    sDataDescriptions* pDataDefs = NULL;
    std::map<int, int> assetIDtoAssetDescriptionOrder;
    std::map<int, std::string> assetIDtoAssetName;
    bool updatedDataDescriptions = false;
    bool needUpdatedDataDescriptions = true;

    std::string strDefaultLocal = "";
    std::string strDefaultMotive = "";
};

#endif // UVS_OPTITRACK_HPP