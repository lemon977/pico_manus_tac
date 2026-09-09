// manus_ndjson_bridge.cpp
//
// 无 ROS 的 MANUS Quantum/Metaglove 采集桥：经 SDK Integrated 直连 dongle，
// 把每帧 raw skeleton（腕根世界系, HandMotion_None）以 NDJSON 逐行写到 stdout。
// 由 manus_collector.py 读取并落盘 logs/manus_*.jsonl。
//
// 每行一个手套一帧：
// {"type":"manus_frame","recv_wall_ns":..,"recv_mono_ns":..,"recv_qpc_ns":..,"publish_time":..,
//  "frame":N,"glove_id":..,"side":"left|right|unknown","calibration_applied":true,"node_count":25,
//  "nodes":[[x,y,z,qx,qy,qz,qw],..],"node_ids":[..],"parent_ids":[..],
//  "joint_types":["MCP","TIP",..],"chain_types":["Thumb","Index",..]}
//
// 环境变量：
//   MANUS_SETTINGS_DIR   可选，Integrated 设置目录（Windows Core3 设置的本地镜像）
//   MANUS_CALIB_LEFT     可选，左手 .mcal 路径（连接后自动下发）
//   MANUS_CALIB_RIGHT    可选，右手 .mcal 路径
//   MANUS_HAND_MOTION    可选，none|auto|imu（默认 none，与采集管线一致）
//   MANUS_WORLD_SPACE    可选，1|0（默认 1）
//
// 参考本仓库 SDKMinimalClient_Linux 与 ROS2 ManusDataPublisher，去掉 ROS 依赖。

#include "SDKMinimalClient.hpp"
#include "ManusSDKTypes.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <chrono>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ClientLogging.hpp"

using ManusSDK::ClientLog;

// ------------------------------------------------------------------ 全局状态

namespace {

std::atomic<bool> g_Running{true};

struct GloveFrame {
    RawSkeletonInfo info{};
    std::vector<SkeletonNode> nodes;
};

std::mutex g_SkelMutex;
std::map<uint32_t, GloveFrame> g_GloveData;   // glove_id -> latest frame
uint64_t g_LatestPublishTime = 0;

std::mutex g_LandscapeMutex;
std::map<uint32_t, Side> g_GloveSide;         // glove_id -> Side
uint32_t g_DongleCount = 0;
bool g_HaveLandscape = false;

std::map<uint32_t, std::vector<NodeInfo>> g_NodeInfoCache;  // glove_id -> node info
std::map<uint32_t, bool> g_CalibApplied;

void OnSignal(int) { g_Running = false; }

const char* SideToString(Side s) {
    switch (s) {
        case Side_Left:  return "left";
        case Side_Right: return "right";
        default:         return "unknown";
    }
}

const char* JointTypeToString(FingerJointType t) {
    switch (t) {
        case FingerJointType_Metacarpal:   return "MCP";
        case FingerJointType_Proximal:     return "PIP";
        case FingerJointType_Intermediate: return "IP";
        case FingerJointType_Distal:       return "DIP";
        case FingerJointType_Tip:          return "TIP";
        default:                           return "Invalid";
    }
}

const char* ChainTypeToString(ChainType t) {
    switch (t) {
        case ChainType_Arm:          return "Arm";
        case ChainType_Leg:          return "Leg";
        case ChainType_Neck:         return "Neck";
        case ChainType_Spine:        return "Spine";
        case ChainType_FingerThumb:  return "Thumb";
        case ChainType_FingerIndex:  return "Index";
        case ChainType_FingerMiddle: return "Middle";
        case ChainType_FingerRing:   return "Ring";
        case ChainType_FingerPinky:  return "Pinky";
        case ChainType_Pelvis:       return "Pelvis";
        case ChainType_Head:         return "Head";
        case ChainType_Shoulder:     return "Shoulder";
        case ChainType_Hand:         return "Hand";
        case ChainType_Foot:         return "Foot";
        case ChainType_Toe:          return "Toe";
        default:                     return "Invalid";
    }
}

uint64_t WallNs() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

uint64_t MonoNs() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

// ------------------------------------------------------------ SDKMinimalClient

SDKMinimalClient* SDKMinimalClient::s_Instance = nullptr;

SDKMinimalClient::SDKMinimalClient() { s_Instance = this; }
SDKMinimalClient::~SDKMinimalClient() { s_Instance = nullptr; }

ClientReturnCode SDKMinimalClient::Initialize() {
    // 不调用 PlatformSpecificInitialization：它会启用 ncurses(initscr) 接管终端并
    // 污染 stdout，还要求 stdin 是 TTY。本桥用 SIGINT 退出、stdout 专供 NDJSON，
    // 与 ROS 版 ManusDataPublisher 一样跳过平台键盘层。
    if (InitializeSDK() != ClientReturnCode::ClientReturnCode_Success) {
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    }
    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKMinimalClient::InitializeSDK() {
    // 采集固定走 Integrated（standalone，无需 MANUS Core），与本仓库采集管线一致。
    m_ConnectionType = ConnectionType::ConnectionType_Integrated;

    const SDKReturnCode t_Init = CoreSdk_InitializeIntegrated();
    if (t_Init != SDKReturnCode::SDKReturnCode_Success) {
        ClientLog::error("CoreSdk_InitializeIntegrated failed: {}", (int32_t)t_Init);
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    }

    if (const char* t_SettingsDir = std::getenv("MANUS_SETTINGS_DIR")) {
        if (t_SettingsDir[0] != '\0') {
            const SDKReturnCode t_Set = CoreSdk_SetSettingsLocation(t_SettingsDir);
            if (t_Set != SDKReturnCode::SDKReturnCode_Success) {
                ClientLog::error("CoreSdk_SetSettingsLocation('{}') failed: {}", t_SettingsDir, (int32_t)t_Set);
            } else {
                ClientLog::print("MANUS settings location: {}", t_SettingsDir);
            }
        }
    }

    if (RegisterAllCallbacks() != ClientReturnCode::ClientReturnCode_Success) {
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    }

    // z-up、右手系、米制世界坐标；与 wrist_rooted_world_space 约定一致。
    CoordinateSystemVUH t_VUH;
    CoordinateSystemVUH_Init(&t_VUH);
    t_VUH.handedness = Side::Side_Right;
    t_VUH.up = AxisPolarity::AxisPolarity_PositiveZ;
    t_VUH.view = AxisView::AxisView_XFromViewer;
    t_VUH.unitScale = 1.0f;  // meters

    bool t_WorldSpace = true;
    if (const char* t_Ws = std::getenv("MANUS_WORLD_SPACE")) {
        if (t_Ws[0] == '0') t_WorldSpace = false;
    }
    const SDKReturnCode t_Coord = CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, t_WorldSpace);
    if (t_Coord != SDKReturnCode::SDKReturnCode_Success) {
        ClientLog::error("CoreSdk_InitializeCoordinateSystemWithVUH failed: {}", (int32_t)t_Coord);
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    }
    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKMinimalClient::ShutDown() {
    const SDKReturnCode t_Result = CoreSdk_ShutDown();
    if (t_Result != SDKReturnCode::SDKReturnCode_Success) {
        return ClientReturnCode::ClientReturnCode_FailedToShutDownSDK;
    }
    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKMinimalClient::RegisterAllCallbacks() {
    const SDKReturnCode t_Raw = CoreSdk_RegisterCallbackForRawSkeletonStream(*OnRawSkeletonStreamCallback);
    if (t_Raw != SDKReturnCode::SDKReturnCode_Success) {
        ClientLog::error("Failed to register raw skeleton callback: {}", (int32_t)t_Raw);
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    }
    const SDKReturnCode t_Land = CoreSdk_RegisterCallbackForLandscapeStream(*OnLandscapeStreamCallback);
    if (t_Land != SDKReturnCode::SDKReturnCode_Success) {
        ClientLog::error("Failed to register landscape callback: {}", (int32_t)t_Land);
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    }
    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKMinimalClient::Connect() {
    SDKReturnCode t_Start = CoreSdk_LookForHosts(1, true);
    if (t_Start != SDKReturnCode::SDKReturnCode_Success) {
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
    }
    uint32_t t_Num = 0;
    if (CoreSdk_GetNumberOfAvailableHostsFound(&t_Num) != SDKReturnCode::SDKReturnCode_Success) {
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
    }
    if (t_Num == 0) {
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
    }
    std::unique_ptr<ManusHost[]> t_Hosts(new ManusHost[t_Num]);
    if (CoreSdk_GetAvailableHostsFound(t_Hosts.get(), t_Num) != SDKReturnCode::SDKReturnCode_Success) {
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
    }
    SDKReturnCode t_Conn = CoreSdk_ConnectToHost(t_Hosts[0]);
    if (t_Conn == SDKReturnCode::SDKReturnCode_NotConnected) {
        return ClientReturnCode::ClientReturnCode_FailedToConnect;
    }
    return ClientReturnCode::ClientReturnCode_Success;
}

// -------------------------------------------------------------------- 校准下发

static bool CalibrationReady(uint32_t p_GloveId, Side p_Side) {
    const auto t_Existing = g_CalibApplied.find(p_GloveId);
    if (t_Existing != g_CalibApplied.end()) return t_Existing->second;

    const char* t_EnvKey = nullptr;
    if (p_Side == Side::Side_Left) t_EnvKey = "MANUS_CALIB_LEFT";
    else if (p_Side == Side::Side_Right) t_EnvKey = "MANUS_CALIB_RIGHT";
    else return false;

    const char* t_Path = std::getenv(t_EnvKey);
    if (t_Path == nullptr || t_Path[0] == '\0') return false;

    std::ifstream t_File(t_Path, std::ios::binary | std::ios::ate);
    if (!t_File) {
        g_CalibApplied[p_GloveId] = false;
        ClientLog::error("Cannot open calibration file {} (env {})", t_Path, t_EnvKey);
        return false;
    }
    const std::streamsize t_Size = t_File.tellg();
    if (t_Size <= 0) {
        g_CalibApplied[p_GloveId] = false;
        ClientLog::error("Calibration file empty: {}", t_Path);
        return false;
    }
    t_File.seekg(0, std::ios::beg);
    std::vector<unsigned char> t_Bytes((size_t)t_Size);
    if (!t_File.read(reinterpret_cast<char*>(t_Bytes.data()), t_Size)) {
        g_CalibApplied[p_GloveId] = false;
        ClientLog::error("Failed reading calibration file: {}", t_Path);
        return false;
    }
    SetGloveCalibrationReturnCode t_Res{};
    const SDKReturnCode t_Sdk =
        CoreSdk_SetGloveCalibration(p_GloveId, t_Bytes.data(), (uint32_t)t_Bytes.size(), &t_Res);
    if (t_Sdk != SDKReturnCode::SDKReturnCode_Success ||
        t_Res != SetGloveCalibrationReturnCode_Success) {
        g_CalibApplied[p_GloveId] = false;
        ClientLog::error(
            "CoreSdk_SetGloveCalibration failed glove {} file {}: sdk={} result={}",
            p_GloveId, t_Path, (int32_t)t_Sdk, (int32_t)t_Res);
        return false;
    }
    g_CalibApplied[p_GloveId] = true;
    ClientLog::print("Applied calibration {} to glove {} ({})", t_Path, p_GloveId, SideToString(p_Side));
    // The frame already copied above predates the calibration update. Drop it;
    // the next publish is the first frame known to use the new profile.
    return false;
}

// -------------------------------------------------------------------- 主循环

void SDKMinimalClient::Run() {
    ClientLog::print("manus_ndjson_bridge running in integrated mode.");
    while (g_Running && Connect() != ClientReturnCode::ClientReturnCode_Success) {
        ClientLog::print("bridge could not connect, retrying in 1s.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!g_Running) return;

    HandMotion t_Motion = HandMotion::HandMotion_None;
    if (const char* t_M = std::getenv("MANUS_HAND_MOTION")) {
        if (strcmp(t_M, "auto") == 0) t_Motion = HandMotion::HandMotion_Auto;
        else if (strcmp(t_M, "imu") == 0) t_Motion = HandMotion::HandMotion_IMU;
    }
    const SDKReturnCode t_HM = CoreSdk_SetRawSkeletonHandMotion(t_Motion);
    if (t_HM != SDKReturnCode::SDKReturnCode_Success) {
        ClientLog::error("Failed to set hand motion mode: {}", (int32_t)t_HM);
    }

    ClientLog::print("bridge connected, streaming NDJSON to stdout.");

    uint64_t t_Frame = 0;
    // ~120Hz 轮询，与 ROS 发布器一致；每个手套单独判是否有新数据后输出。
    std::map<uint32_t, uint64_t> t_LastPublishOut;

    while (g_Running) {
        std::map<uint32_t, GloveFrame> t_Data;
        {
            std::lock_guard<std::mutex> t_Lock(g_SkelMutex);
            t_Data = g_GloveData;
        }
        std::map<uint32_t, Side> t_Sides;
        {
            std::lock_guard<std::mutex> t_Lock(g_LandscapeMutex);
            t_Sides = g_GloveSide;
        }

        const uint64_t t_WallNs = WallNs();
        const uint64_t t_MonoNs = MonoNs();

        for (auto& kv : t_Data) {
            const uint32_t t_GloveId = kv.first;
            GloveFrame& t_Gf = kv.second;
            if (t_Gf.info.nodesCount == 0 || t_Gf.nodes.empty()) continue;

            // 仅在有新 publishTime 时输出，避免重复刷同一帧。
            uint64_t t_Pub = t_Gf.info.publishTime.time;
            auto t_It = t_LastPublishOut.find(t_GloveId);
            if (t_It != t_LastPublishOut.end() && t_It->second == t_Pub && t_Pub != 0) {
                continue;
            }
            t_LastPublishOut[t_GloveId] = t_Pub;

            Side t_Side = Side::Side_Invalid;
            auto t_Si = t_Sides.find(t_GloveId);
            if (t_Si != t_Sides.end()) t_Side = t_Si->second;
            if (!CalibrationReady(t_GloveId, t_Side)) continue;

            // 缓存节点层级信息（每个 glove 拉一次）。
            auto t_Cache = g_NodeInfoCache.find(t_GloveId);
            if (t_Cache == g_NodeInfoCache.end()) {
                std::vector<NodeInfo> t_Info(t_Gf.info.nodesCount);
                const SDKReturnCode t_R =
                    CoreSdk_GetRawSkeletonNodeInfoArray(t_GloveId, t_Info.data(), t_Gf.info.nodesCount);
                if (t_R == SDKReturnCode::SDKReturnCode_Success) {
                    g_NodeInfoCache[t_GloveId] = t_Info;
                    t_Cache = g_NodeInfoCache.find(t_GloveId);
                }
            }

            // side 兜底：landscape 拿不到时用节点 side。
            if (t_Side == Side::Side_Invalid && t_Cache != g_NodeInfoCache.end() &&
                !t_Cache->second.empty()) {
                t_Side = t_Cache->second[0].side;
            }

            std::string t_Line;
            t_Line.reserve(4096);
            char t_Buf[128];

            t_Line += "{\"type\":\"manus_frame\"";
            snprintf(t_Buf, sizeof(t_Buf), ",\"recv_wall_ns\":%llu", (unsigned long long)t_WallNs);
            t_Line += t_Buf;
            snprintf(t_Buf, sizeof(t_Buf), ",\"recv_mono_ns\":%llu", (unsigned long long)t_MonoNs);
            t_Line += t_Buf;
            // MSVC steady_clock 使用 QueryPerformanceCounter；显式命名后与
            // Python time.perf_counter_ns() 处于同一个跨进程时钟域。
            snprintf(t_Buf, sizeof(t_Buf), ",\"recv_qpc_ns\":%llu", (unsigned long long)t_MonoNs);
            t_Line += t_Buf;
            snprintf(t_Buf, sizeof(t_Buf), ",\"publish_time\":%llu", (unsigned long long)t_Pub);
            t_Line += t_Buf;
            snprintf(t_Buf, sizeof(t_Buf), ",\"frame\":%llu", (unsigned long long)t_Frame);
            t_Line += t_Buf;
            snprintf(t_Buf, sizeof(t_Buf), ",\"glove_id\":%u", t_GloveId);
            t_Line += t_Buf;
            t_Line += ",\"side\":\"";
            t_Line += SideToString(t_Side);
            t_Line += "\"";
            const auto t_CalibIt = g_CalibApplied.find(t_GloveId);
            const bool t_CalibOk = (t_CalibIt != g_CalibApplied.end() && t_CalibIt->second);
            t_Line += t_CalibOk ? ",\"calibration_applied\":true" : ",\"calibration_applied\":false";
            snprintf(t_Buf, sizeof(t_Buf), ",\"node_count\":%u", t_Gf.info.nodesCount);
            t_Line += t_Buf;

            const uint32_t t_N = t_Gf.info.nodesCount;
            const bool t_HaveInfo = (t_Cache != g_NodeInfoCache.end() &&
                                     t_Cache->second.size() == t_N);

            // nodes
            t_Line += ",\"nodes\":[";
            for (uint32_t i = 0; i < t_N; ++i) {
                const ManusVec3& p = t_Gf.nodes[i].transform.position;
                const ManusQuaternion& q = t_Gf.nodes[i].transform.rotation;
                snprintf(t_Buf, sizeof(t_Buf),
                         "%s[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]",
                         (i ? "," : ""), p.x, p.y, p.z, q.x, q.y, q.z, q.w);
                t_Line += t_Buf;
            }
            t_Line += "]";

            // node_ids / parent_ids
            t_Line += ",\"node_ids\":[";
            for (uint32_t i = 0; i < t_N; ++i) {
                snprintf(t_Buf, sizeof(t_Buf), "%s%u", (i ? "," : ""), t_Gf.nodes[i].id);
                t_Line += t_Buf;
            }
            t_Line += "]";

            t_Line += ",\"parent_ids\":[";
            for (uint32_t i = 0; i < t_N; ++i) {
                int64_t t_Parent = -1;
                if (t_HaveInfo) t_Parent = (int64_t)t_Cache->second[i].parentId;
                snprintf(t_Buf, sizeof(t_Buf), "%s%lld", (i ? "," : ""), (long long)t_Parent);
                t_Line += t_Buf;
            }
            t_Line += "]";

            // joint_types / chain_types
            t_Line += ",\"joint_types\":[";
            for (uint32_t i = 0; i < t_N; ++i) {
                const char* t_J = t_HaveInfo ? JointTypeToString(t_Cache->second[i].fingerJointType) : "";
                t_Line += (i ? ",\"" : "\"");
                t_Line += t_J;
                t_Line += "\"";
            }
            t_Line += "]";

            t_Line += ",\"chain_types\":[";
            for (uint32_t i = 0; i < t_N; ++i) {
                const char* t_C = t_HaveInfo ? ChainTypeToString(t_Cache->second[i].chainType) : "";
                t_Line += (i ? ",\"" : "\"");
                t_Line += t_C;
                t_Line += "\"";
            }
            t_Line += "]";

            t_Line += "}\n";
            fputs(t_Line.c_str(), stdout);
            fflush(stdout);
            t_Frame++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(8));  // ~120Hz
    }
}

// -------------------------------------------------------------------- 回调

void SDKMinimalClient::OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_Info) {
    if (!s_Instance) return;
    std::lock_guard<std::mutex> t_Lock(g_SkelMutex);
    for (uint32_t i = 0; i < p_Info->skeletonsCount; ++i) {
        GloveFrame t_Gf;
        CoreSdk_GetRawSkeletonInfo(i, &t_Gf.info);
        t_Gf.info.publishTime = p_Info->publishTime;
        t_Gf.nodes.resize(t_Gf.info.nodesCount);
        if (t_Gf.info.nodesCount > 0) {
            CoreSdk_GetRawSkeletonData(i, t_Gf.nodes.data(), t_Gf.info.nodesCount);
        }
        g_GloveData[t_Gf.info.gloveId] = std::move(t_Gf);
    }
    g_LatestPublishTime = p_Info->publishTime.time;
}

// landscape：拿到每个 glove 的 side + dongle 数量，用于校准与调试。
void SDKMinimalClient::OnLandscapeStreamCallback(const Landscape* const p_Landscape) {
    if (!s_Instance || p_Landscape == nullptr) return;
    std::lock_guard<std::mutex> t_Lock(g_LandscapeMutex);
    g_DongleCount = p_Landscape->gloveDevices.dongleCount;
    for (uint32_t i = 0; i < p_Landscape->gloveDevices.gloveCount; ++i) {
        const GloveLandscapeData& t_G = p_Landscape->gloveDevices.gloves[i];
        g_GloveSide[t_G.id] = t_G.side;
    }
    g_HaveLandscape = true;
}

// ------------------------------------------------------------------------ main

int main() {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    ClientLog::print("Starting manus_ndjson_bridge.");
    SDKMinimalClient t_Client;
    if (t_Client.Initialize() != ClientReturnCode::ClientReturnCode_Success) {
#ifdef _WIN32
        ClientLog::error("Failed to initialize the SDK. Is ManusSDK.dll beside the executable or on PATH?");
#else
        ClientLog::error("Failed to initialize the SDK. Is libManusSDK_Integrated.so on the loader path?");
#endif
        return -1;
    }
    t_Client.Run();
    ClientLog::print("manus_ndjson_bridge shutting down.");
    t_Client.ShutDown();
    return 0;
}
