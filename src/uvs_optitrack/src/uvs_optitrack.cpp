#include "uvs_optitrack.hpp"

int _kbhit() 
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

char getch()
{
    char buf = 0;
    termios old = { 0 };

    fflush( stdout );

    if ( tcgetattr( 0, &old ) < 0 )
        perror( "tcsetattr()" );

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if ( tcsetattr( 0, TCSANOW, &old ) < 0 )
        perror( "tcsetattr ICANON" );

    if ( read( 0, &buf, 1 ) < 0 )
        perror( "read()" );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;

    if ( tcsetattr( 0, TCSADRAIN, &old ) < 0 )
        perror( "tcsetattr ~ICANON" );

    //printf( "%c\n", buf );

    return buf;
}

UvsOptitrack::UvsOptitrack()    
    : Node("uvs_optitrack")
{
    // get default parameters
    this->declare_parameter("LocalIPv4Addr","");
    this->declare_parameter("ServerIPv4Addr","");
    strDefaultLocal = this->get_parameter("LocalIPv4Addr").as_string();
    strDefaultMotive = this->get_parameter("ServerIPv4Addr").as_string();
    // Print NatNet client version info
    unsigned char ver[4];
    NatNet_GetVersion(ver);
    RCLCPP_INFO(
        rclcpp::get_logger("NatNetLib"),
        "NatNet Library Version: %d.%d.%d.%d",
        ver[0], ver[1], ver[2], ver[3]
    );
    // Install logging callback
    NatNet_SetLogCallback(UvsOptitrack::MessageHandler);
    // Create NatNet client
    pClient = std::make_shared<NatNetClient>();
    // Set the frame callback handler
    pClient->SetFrameReceivedCallback(UvsOptitrack::DataHandler, this);

    publisher = this->create_publisher<uvs_message::msg::UvOptPoseList>("uvs_pose_list", 1);
}

UvsOptitrack::~UvsOptitrack()
{
    pClient->Disconnect();
    pClient.reset();
    if (pDataDefs)
    {
        NatNet_FreeDescriptions(pDataDefs);
    }
}

bool UvsOptitrack::tryConnect()
{
    // if local IP or server IP is not set, discover servers
    if (strDefaultLocal.empty() || strDefaultMotive.empty())
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Discovering servers... \n"
            "Press the number key that corresponds to any discovered server to connect to that server.\n"
            "Press Q at any time to quit.\n\n"
        );
        NatNetDiscoveryHandle discovery;
        NatNet_CreateAsyncServerDiscovery(&discovery, UvsOptitrack::ServerDiscoveredCallback, this);
        while ( const int c = getch() )
        {
            if ( c >= '1' && c <= '9' )
            {
                const size_t serverIndex = c - '1';
                if ( serverIndex < discoveredServers.size() )
                {
                    const sNatNetDiscoveredServer& discoveredServer = discoveredServers[serverIndex];

                    if ( discoveredServer.serverDescription.bConnectionInfoValid )
                    {
                        snprintf(discoveredMulticastGroupAddr, sizeof (discoveredMulticastGroupAddr),
                            "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
                            discoveredServer.serverDescription.ConnectionMulticastAddress[0],
                            discoveredServer.serverDescription.ConnectionMulticastAddress[1],
                            discoveredServer.serverDescription.ConnectionMulticastAddress[2],
                            discoveredServer.serverDescription.ConnectionMulticastAddress[3]
                        );
                        connectParams.connectionType = discoveredServer.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
                        connectParams.serverCommandPort = discoveredServer.serverCommandPort;
                        connectParams.serverDataPort = discoveredServer.serverDescription.ConnectionDataPort;
                        connectParams.serverAddress = discoveredServer.serverAddress;
                        connectParams.localAddress = discoveredServer.localAddress;
                        connectParams.multicastAddress = discoveredMulticastGroupAddr;
                    }
                    else
                    {
                        connectParams.connectionType = defaultConnectionType;
                        connectParams.serverCommandPort = discoveredServer.serverCommandPort;
                        connectParams.serverDataPort = 0;
                        connectParams.serverAddress = discoveredServer.serverAddress;
                        connectParams.localAddress = discoveredServer.localAddress;
                        connectParams.multicastAddress = NULL;
                    }
                    break;
                }
            }
            else if ( c == 'q' || c == 'Q' )
            {
                NatNet_FreeAsyncServerDiscovery( discovery );
                return false;
            }
        }
        NatNet_FreeAsyncServerDiscovery( discovery );
    }
    else
    {
        connectParams.connectionType = defaultConnectionType;
        connectParams.serverCommandPort = 1510;
        connectParams.serverDataPort = 1511;
        connectParams.serverAddress = strDefaultMotive.c_str();
        connectParams.localAddress = strDefaultLocal.c_str();
        connectParams.multicastAddress = discoveredMulticastGroupAddr;
    }
    // Connect to Motive
    pClient->Disconnect();
    int ret = pClient->Connect(connectParams);
    if (ret != ErrorCode_OK)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error: Unable to connect to server. Error code: %d",
            ret
        );
        return false;
    }
    updatedDataDescriptions = updateDataDescriptions();
    if (!updatedDataDescriptions)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error: Unable to retrieve data descriptions from server."
        );
        return false;
    }
    return true;
}

void UvsOptitrack::spin_once()
{
    if (needUpdatedDataDescriptions)
    {
        updatedDataDescriptions = updateDataDescriptions();
        needUpdatedDataDescriptions = false;
    }
}

void NATNET_CALLCONV UvsOptitrack::MessageHandler(Verbosity msgType, const char *msg)
{
    std::string prefix;
    switch (msgType)
    {
    case Verbosity_Debug:
        prefix = "[DEBUG]";
        break;
    case Verbosity_Info:
        prefix = "[INFO]";
        break;
    case Verbosity_Warning:
        prefix = "[WARN]";
        break;
    case Verbosity_Error:
        prefix = "[ERROR]";
        break;
    default:
        prefix = "[?????]";
        break;   
    }
    RCLCPP_INFO(
        rclcpp::get_logger("NatNetLib"),
        "%s %s",
        prefix.c_str(),
        msg
    );
}

void NATNET_CALLCONV UvsOptitrack::DataHandler(sFrameOfMocapData *data, void *pUserData)
{
    UvsOptitrack* pThis = static_cast<UvsOptitrack*>(pUserData);
    if (!pThis || !pThis->pClient) return;
    std::shared_ptr<sFrameOfMocapData> pDataCopy = std::make_shared<sFrameOfMocapData>();
    NatNet_CopyFrame(data, pDataCopy.get());

    bool bTrackedModelsChanged = ((pDataCopy->params & 0x02) != 0);
    if (bTrackedModelsChanged)
    {
        pThis->needUpdatedDataDescriptions = true;
        NatNet_FreeFrame(pDataCopy.get());
        return;
    }
    // Publish data
    uvs_message::msg::UvOptPoseList msg;
    for (int i = 0; i < pDataCopy->nRigidBodies; i++)
    {
        sRigidBodyData* rb = &pDataCopy->RigidBodies[i];
        uvs_message::msg::UvOptPose pose;
        pose.id = rb->ID;
        pose.name = pThis->assetIDtoAssetName[rb->ID];
        pose.pose.position.x = rb->x;
        pose.pose.position.y = rb->y;
        pose.pose.position.z = rb->z;
        pose.pose.orientation.x = rb->qx;
        pose.pose.orientation.y = rb->qy;
        pose.pose.orientation.z = rb->qz;
        pose.pose.orientation.w = rb->qw;
        msg.pose_list.push_back(pose);
    }
    pThis->publisher->publish(msg);

    NatNet_FreeFrame(pDataCopy.get());
}

void NATNET_CALLCONV UvsOptitrack::ServerDiscoveredCallback(const sNatNetDiscoveredServer *pDiscoveredServer, void *pUserContext)
{
    UvsOptitrack* pThis = static_cast<UvsOptitrack*>(pUserContext);
    char serverHotkey = '.';
    if ( pThis->discoveredServers.size() < 9 )
    {
        serverHotkey = static_cast<char>('1' + pThis->discoveredServers.size());
    }

    RCLCPP_INFO(
        pThis->get_logger(),
        "[%c] %s %d.%d.%d at %s (%s) from %s",
        serverHotkey,
        pDiscoveredServer->serverDescription.szHostApp,
        pDiscoveredServer->serverDescription.HostAppVersion[0],
        pDiscoveredServer->serverDescription.HostAppVersion[1],
        pDiscoveredServer->serverDescription.HostAppVersion[2],
        pDiscoveredServer->serverAddress,
        pDiscoveredServer->serverDescription.bConnectionInfoValid ? 
            (pDiscoveredServer->serverDescription.ConnectionMulticast ? "multicast" : "unicast") : 
            "WARNING: Legacy server, could not auto-detect settings. Auto-connect may not work reliably.",
        pDiscoveredServer->localAddress
    );

    pThis->discoveredServers.push_back( *pDiscoveredServer );
}

bool UvsOptitrack::updateDataDescriptions()
{
    if (pDataDefs)
    {
        NatNet_FreeDescriptions(pDataDefs);
    }
    int ret = pClient->GetDataDescriptionList(&pDataDefs);
    if (ret != ErrorCode_OK || pDataDefs == NULL)
    {
        return false;
    }
    assetIDtoAssetDescriptionOrder.clear();
    assetIDtoAssetName.clear();
    int assetID = 0;
    std::string assetName = "";
    int index = 0;
    int cameraIndex = 0;
    if (pDataDefs == nullptr || pDataDefs->nDataDescriptions <= 0)
        return false;

    for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
    {
        assetID = -1;
        assetName = "";

        if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
        {
            sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
            assetID = pRB->ID;
            assetName = std::string(pRB->szName);
        }
        else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
        {
            sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
            assetID = pSK->skeletonID;
            assetName = std::string(pSK->szName);

            // Add individual bones
            // skip for now since id could clash with non-skeleton RigidBody ids in our RigidBody lookup table
            /*
            if (insertResult.second == true)
            {
                for (int j = 0; j < pSK->nRigidBodies; j++)
                {
                    // Note:
                    // In the DataCallback packet (sFrameOfMocapData) skeleton bones (rigid bodies) ids are of the form:
                    //   parent skeleton ID   : high word (upper 16 bits of int)
                    //   rigid body id        : low word  (lower 16 bits of int)
                    //
                    // In DataDescriptions packet (sDataDescriptions) they are not, so apply the data id format here
                    // for correct lookup during data callback
                    std::pair<std::map<int, std::string>::iterator, bool> insertBoneResult;
                    sRigidBodyDescription rb = pSK->RigidBodies[j];
                    int id = (rb.parentID << 16) | rb.ID;
                    std::string skeletonBoneName = string(pSK->szName) + (":") + string(rb.szName) + string(pSK->szName);
                    insertBoneResult = g_AssetIDtoAssetName.insert(id, skeletonBoneName);
                }
            }
            */
        }
        else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
        {
            // Skip markersets for now as they dont have unique id's, but do increase the index
            // as they are in the data packet
            index++;
            continue;
            /*
            sMarkerSetDescription* pDesc = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
            assetID = index;
            assetName = pDesc->szName;
            */
        }

        else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
        {
            sForcePlateDescription* pDesc = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
            assetID = pDesc->ID;
            assetName = pDesc->strSerialNo;
        }
        else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Device)
        {
            sDeviceDescription* pDesc = pDataDefs->arrDataDescriptions[i].Data.DeviceDescription;
            assetID = pDesc->ID;
            assetName = std::string(pDesc->strName);
        }
        else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera)
        {
            // skip cameras as they are not in the data packet
            continue;
        }
        else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Asset)
        {
            sAssetDescription* pDesc = pDataDefs->arrDataDescriptions[i].Data.AssetDescription;
            assetID = pDesc->AssetID;
            assetName = std::string(pDesc->szName);
        }

        if (assetID == -1)
        {
            printf("\n[SampleClient] Warning : Unknown data type in description list : %d\n", pDataDefs->arrDataDescriptions[i].type);
        }
        else 
        {
            // Add to Asset ID to Asset Name map
            std::pair<std::map<int, std::string>::iterator, bool> insertResult;
            insertResult = assetIDtoAssetName.insert(std::pair<int,std::string>(assetID, assetName));
            if (insertResult.second == false)
            {
                printf("\n[SampleClient] Warning : Duplicate asset ID already in Name map (Existing:%d,%s\tNew:%d,%s\n)",
                    insertResult.first->first, insertResult.first->second.c_str(), assetID, assetName.c_str());
            }
        }

        // Add to Asset ID to Asset Description Order map
        if (assetID != -1)
        {
            std::pair<std::map<int, int>::iterator, bool> insertResult;
            insertResult = assetIDtoAssetDescriptionOrder.insert(std::pair<int, int>(assetID, index++));
            if (insertResult.second == false)
            {
                printf("\n[SampleClient] Warning : Duplicate asset ID already in Order map (ID:%d\tOrder:%d\n)", insertResult.first->first, insertResult.first->second);
            }
        }
    }
}
