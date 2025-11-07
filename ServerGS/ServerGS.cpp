// UdpServer.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"

#pragma comment(lib, "ws2_32")
//#include <WinSock2.h>
#include <WS2tcpip.h>
#include <iostream>
#include <sstream>

#include <opencv2/line_descriptor.hpp>

//#include <System.h>
//#include <Frame.h>
//#include <User.h>
//#include <ServerMap.h>
//#include <MappingServer.h>
//#include <ServerMapper.h>
//#include <LoopCloser.h>
//#include <ServerMapOptimizer.h>
//#include <Visualizer.h>
//#include <WebAPI.h>
//#include <ThreadPool.h>

#include <WebAPI.h>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <Utils.h>
#include <Utils_Geometry.h>

#include <rapidjson\document.h>

#include <ConcurrentMap.h>

////BASE SLAM
#include <BaseSystem.h>
#include <BaseDevice.h>
#include <AbstractMap.h>

//EdgeSLAM
#include <../EdgeSLAM/include/SLAM.h>
#include <../EdgeSLAM/include/Camera.h>

//GaussianSparseSLAM
#include <GaussianSparseSLAM/include/Types.h>
#include <GaussianSparseSLAM/include/GSSLAM.h>
#include <GaussianSparseSLAM/include/User.h>
#include <GaussianSparseSLAM/include/Map.h>
#include <GaussianSparseSLAM/include/Frame.h>
#include <GaussianSparseSLAM/include/KeyFrame.h>
#include <GaussianSparseSLAM/include/Matcher.h>
#include <GaussianSparseSLAM/include/GaussianPoint.h>
#include <GaussianSparseSLAM/include/Visualizer.h>
#include <GaussianSparseSLAM/include/Initializer.h>
#include <GaussianSparseSLAM/include/Tracker.h>
#include <GaussianSparseSLAM/include/Mapper.h>

//libtorch
//#include <torch/torch.h>

using namespace cv::line_descriptor;

/////CPU 사용량 체크
static ULARGE_INTEGER lastCPU, lastSysCPU, lastUserCPU;
static int numProcessors;
static HANDLE self;
double sumCPU = 0.0;
int    nCPUCount = 0;

void cpuinit() {
	SYSTEM_INFO sysInfo;
	FILETIME ftime, fsys, fuser;
	GetSystemInfo(&sysInfo);
	numProcessors = sysInfo.dwNumberOfProcessors;

	GetSystemTimeAsFileTime(&ftime);
	memcpy(&lastCPU, &ftime, sizeof(FILETIME));


	self = GetCurrentProcess();
	GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
	memcpy(&lastSysCPU, &fsys, sizeof(FILETIME));
	memcpy(&lastUserCPU, &fuser, sizeof(FILETIME));
}

double getCurrentCPUValue() {
	FILETIME ftime, fsys, fuser;
	ULARGE_INTEGER now, sys, user;
	double percent;

	GetSystemTimeAsFileTime(&ftime);
	memcpy(&now, &ftime, sizeof(FILETIME));


	GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
	memcpy(&sys, &fsys, sizeof(FILETIME));
	memcpy(&user, &fuser, sizeof(FILETIME));
	percent = (sys.QuadPart - lastSysCPU.QuadPart) + (user.QuadPart - lastUserCPU.QuadPart);
	percent /= (now.QuadPart - lastCPU.QuadPart);
	percent /= numProcessors;
	lastCPU = now;
	lastUserCPU = user;
	lastSysCPU = sys;

	return percent * 100;
}

void PrintCPU() {
	sumCPU += getCurrentCPUValue();
	nCPUCount++;
	if (nCPUCount > 300 && nCPUCount % 10 == 0)
		std::cout << "CPU Usage = " << nCPUCount << " " << sumCPU << " = " << sumCPU / nCPUCount << std::endl;
}

/////CPU 사용량 체크

std::vector<std::string> split(std::string input, char delimiter) {
	std::vector<std::string> answer;
	std::stringstream ss(input);
	std::string temp;

	while (getline(ss, temp, delimiter)) {
		answer.push_back(temp);
	}

	return answer;
}

std::vector<std::string> receivedKeywords;
WSAData wsaData;
int code = WSAStartup(MAKEWORD(2, 2), &wsaData);
int sock = socket(AF_INET, SOCK_DGRAM, 17);
sockaddr_in serveraddr;

//argv
std::string ip;
int port;
int nNetworkSimul = 0;
bool bSemantic = false;
bool bSaveLatency = false;
std::string latencyPath;
bool bSaveVOLatency = false;
bool bSaveVisualization = false;
int nIncForVis = 20;
std::string volatencyPath;
bool bSaveTrajectory = false;
std::string trajectoryPath;
int nNumDevice = 1;
bool bSaveSegLatency = false;
std::string deepkey1 = "a";
std::string deepkey2 = "b";
std::string deepkey3 = "c";
bool bSam = false;
bool bObjBaselineTest = false;
bool bSaveAssoVisImage = false;
bool bMultiObjTest = false;
bool bGridTest = false;
bool bARAppTest = false;
bool bDepthAnything = false;

bool consoleHandler(int signal) {

	if (signal == CTRL_C_EVENT) {
		//sock = socket(AF_INET, SOCK_DGRAM, 0);
		//serveraddr.sin_family = AF_INET;
		//serveraddr.sin_port = htons(35001); // 포트 번호
		//serveraddr.sin_addr.s_addr = inet_addr("143.248.6.143");

		{
			std::stringstream ss;
			WebAPI* mpAPI = new WebAPI(ip, port);
			ss << "/Disconnect?src=SLAMServer&type=server";
			mpAPI->Send(ss.str(), "");
			delete mpAPI;
		}

		for (int i = 0, iend = receivedKeywords.size(); i < iend; i++) {
			std::stringstream ss;
			ss << "{\"type1\":" << "\"disconnect\"" << ",\"type2\":" << "\"all\"" << ",\"src\":" << "\"SLAMServer\"" << ",\"keyword\":\"" << receivedKeywords[i] << "\"" << "}";
			std::string temp = ss.str();
			auto buf = temp.c_str();
			int retval;
			retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
		}

		WebAPI::Close();
		exit(1);
	}
	return true;
}

////데이터 생성용 쓰레드
float sumLatency1 = 0.0;
int nTotalLatency1 = 0;

float sumLatency2 = 0.0;
int nTotalLatency2 = 0;

float sumLatency3 = 0.0;
int nTotalLatency3 = 0;

float sumLatency4 = 0.0;
int nTotalLatency4 = 0;

ConcurrentMap<int, std::chrono::high_resolution_clock::time_point> LatencyMapStart;
ConcurrentMap<std::string, float> LatencyMapSum;
ConcurrentMap<std::string, int>   LatencyMapCount;

void Evaluation(std::string keyword, std::string key2, int did) {
	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	auto start = LatencyMapStart.Get(did);
	auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	float t_test1 = du_test1 / 1000.0;

	float sum = LatencyMapSum.Get(key2);
	int N = LatencyMapCount.Get(key2);

	sum = sum + t_test1;
	N++;

	std::cout << keyword << " = " << did << " || " << N << " " << sum << " " << sum / N << std::endl;

	LatencyMapSum.Update(key2, sum);
	LatencyMapCount.Update(key2, N);

}

//BaseSLAM
BaseSLAM::BaseSystem* BaseSLAMSystem = new BaseSLAM::BaseSystem();
//StructOptimization::StructSLAM* StructSLAMSystem = new StructOptimization::StructSLAM();

//test object optical flow
bool bPrev = false;
//ObjectSLAM::BoxFrame* pPrevBF = nullptr;
//cv::Mat prev, prevGray, prevDesc;
//std::vector<cv::Point2f> prevCorners, currCorners;

void UpdateGSDepthFrame(GaussianSparseSLAM::GSSLAM* SLAM, std::string keyword, std::string user, int id, long long received_ts, bool bShow = false) {

	auto User = SLAM->GetUser(user);
	if (!User) {
		return;
	}
	User->mnUsed++;

	std::stringstream ss;
	ss << "/Download?keyword=resgsmapping" << "&id=" << id << "&src=" << user;
	WebAPI API("143.248.6.143", 35005);
	auto res = API.Send(ss.str(), "");
	cv::Mat tempdepth = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());

	cv::Mat imDepth;
	cv::Mat depthsrc = cv::imdecode(tempdepth, cv::IMREAD_ANYDEPTH); //16U, ushort
	//depthsrc.convertTo(imDepth, CV_32F, User->mpCamera->mDepthMapFactor);

	cv::Mat normalizedDepth;
	depthsrc.convertTo(normalizedDepth, CV_32F, 0.001);  // float 타입으로 변환
	imDepth = normalizedDepth.clone();

	////뎁스 시각화
	// 최소/최대 깊이 값 계산
	double minVal, maxVal;
	cv::minMaxLoc(normalizedDepth, &minVal, &maxVal);

	// 0~255 범위로 정규화
	normalizedDepth = (normalizedDepth - minVal) / (maxVal - minVal) * 255;
	normalizedDepth.convertTo(normalizedDepth, CV_8U);

	cv::Mat depthColor;
	cv::applyColorMap(normalizedDepth, depthColor, cv::COLORMAP_JET);
	SLAM->VisualizeImage(User->mapName, depthColor, 2);
	////뎁스 시각화
	User->mnUsed--;

}

void UpdateDepthFrame(GaussianSparseSLAM::GSSLAM* SLAM, std::string keyword, std::string user, int id, long long received_ts, bool bShow = false) {
	auto User = SLAM->GetUser(user);
	if (!User) {
		return;
	}
	User->mnUsed++;

	std::stringstream ss;
	ss << "/Download?keyword=resdepthanything" << "&id=" << id << "&src=" << user;
	WebAPI API("143.248.6.143", 35005);
	auto res = API.Send(ss.str(), "");
	cv::Mat tempdepth = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());

	cv::Mat imDepth;
	cv::Mat depthsrc = cv::imdecode(tempdepth, cv::IMREAD_ANYDEPTH); //16U, ushort
	//depthsrc.convertTo(imDepth, CV_32F, User->mpCamera->mDepthMapFactor);

	cv::Mat normalizedDepth;
	depthsrc.convertTo(normalizedDepth, CV_32F, 0.001);  // float 타입으로 변환
	imDepth = normalizedDepth.clone();

	//초기화 또는 매핑
	//auto frameStat = f->
	auto mapStat = User->GetMap()->GetState();
	auto userStat = User->GetState();

	if (mapStat == GaussianSparseSLAM::MapState::NoImages || mapStat == GaussianSparseSLAM::MapState::NotInitialized)
	{
		auto f = User->mFrames.Get(id);
		auto pMap = User->GetMap();
		auto frameStat = f->GetState();

		if (frameStat == GaussianSparseSLAM::FrameProcessStatus::initialization)
		{
			f->ComputeStereoFromRGBD(imDepth);
			auto resInit = SLAM->mpInitializer->StereoInitialization(f, pMap);

			pMap->SetState(resInit);

			if (resInit == GaussianSparseSLAM::MapState::Initialized)
			{
				
				//salad
				{
					std::stringstream ss2;
					ss2 << User->strMapName << "." << user << ".GSImage." << id;
					Utils::SendReqMessage("reqmapsalad", ss2.str(), f->mnKeyFrameId);
				}

				User->SetState(GaussianSparseSLAM::UserState::Success);

				//keyframe
				std::vector < cv::Vec2i> vecDatas;
				auto vpGPs = f->mvGaussianPoints.get();
				for (int i = 0; i < f->N; i++)
				{
					auto pGP = vpGPs[i];
					if (!pGP || pGP->isBad())
						continue;
					vecDatas.push_back(cv::Vec2i(i, pGP->mnId));
				}

				//사용자 주변 키프레임
				User->mpRefKF = SLAM->mpInitializer->mpInitKeyFrame1;
				User->KeyFrames.Update(f->mnFrameID, SLAM->mpInitializer->mpInitKeyFrame1);
				auto pNewKF = User->mpRefKF;
				User->mpRefKF->sourceName = user;

				//이전 프레임 갱신
				User->prevFrame = f;
				User->mnPrevFrameID = id;

				//뎁스 쵝화
				/*f->mvuRight = std::vector<float>(f->N, -1);
				f->mvDepth = std::vector<float>(f->N, -1);
				pNewKF->mvuRight = std::vector<float>(pNewKF->N, -1);
				pNewKF->mvDepth = std::vector<float>(pNewKF->N, -1);*/

				//서버에 정보 전송
				//키프레임 옵저베이션
				cv::Mat tmp = cv::Mat(vecDatas.size(), 1, CV_32SC2, vecDatas.data());
				std::stringstream ss;
				ss << User->mapName << "." << user << "." << f->mnFrameID;
				Utils::SendData("kf_gp_ids", ss.str(), tmp, User->mpRefKF->mnId, 4, 2);
				//인접 키프레임.
				tmp = cv::Mat::ones(2000, 1, CV_32FC1) * -1;
				Utils::SendData("reqmatches", ss.str(), tmp, User->mpRefKF->mnId, 4, 1);
				

				//가우시안 매핑에 키프레임 추가
				{
					
					cv::Mat Pcw = pNewKF->GetPose();
					cv::Mat kfdata = cv::Mat::zeros(1000, 1, CV_32FC1); //inlier, pose + point2f, octave, angle, point3f
					cv::Mat mpdata = cv::Mat::zeros(1, 1, CV_32FC1);
					int nTrackData = 0;
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(0, 0);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(0, 1);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(0, 2);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(1, 0);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(1, 1);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(1, 2);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(2, 0);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(2, 1);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(2, 2);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(0, 3);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(1, 3);
					kfdata.at<float>(nTrackData++) = Pcw.at<float>(2, 3);
					{
						std::stringstream ss;
						ss << User->mapName << "." << user << "." << pNewKF->mnFrameId;
						Utils::SendData("datakfpose", ss.str(), kfdata, pNewKF->mnId, 4, 2);
					}

					std::vector<cv::Vec4f> vecDatas;
					auto vpGPs = pNewKF->mvpMapPoints.get();
					for (int i = 0; i < vpGPs.size(); i++) {
						auto pGPi = vpGPs[i];
						if (!pGPi || pGPi->isBad())
							continue;
						cv::Mat X = pGPi->GetWorldPos();
						cv::Vec4f vec4(
							static_cast<float>(id),
							X.at<float>(0, 0),
							X.at<float>(1, 0),
							X.at<float>(2, 0)
						);
						vecDatas.push_back(vec4);
					}
					
					{
						std::stringstream ss;
						ss << User->mapName << "." << user << "." << pNewKF->mnFrameId;
						cv::Mat tmp = cv::Mat(vecDatas.size(), 1, CV_32FC4, vecDatas.data());
						Utils::SendData("datasparsemap", ss.str(), tmp, pNewKF->mnId, 4, 4);
					}

					std::stringstream ss;
					ss << User->mapName << "." << user << ".GSImage" << "." << pNewKF->mnFrameId;
					Utils::SendReqMessage("reqgsmapping", ss.str(), pNewKF->mnId); //datakf

					std::cout << "stereo initialization success " <<pNewKF->mnId<< std::endl;

				}
			}
			else {
				std::cout << "stereo init failed" << std::endl;
			}
		}
	}
	else if (mapStat == GaussianSparseSLAM::MapState::Initialized)
	{
		auto pKF = User->KeyFrames.Get(id);
		if (!pKF)
			std::cout << "keyframe error???" << std::endl;
		if (pKF)
		{
			auto pMap = User->GetMap();
			pKF->ComputeStereoFromRGBD(imDepth);

			//가우시안 스플래팅 요청
			int nKF = pMap->GetNumKeyFrames();
			if (nKF > 2)
			{

			}
		}
	}

	////뎁스 시각화
	// 최소/최대 깊이 값 계산 
	double minVal, maxVal;
	cv::minMaxLoc(normalizedDepth, &minVal, &maxVal);

	// 0~255 범위로 정규화
	normalizedDepth = (normalizedDepth - minVal) / (maxVal - minVal) * 255;
	normalizedDepth.convertTo(normalizedDepth, CV_8U);

	cv::Mat depthColor;
	cv::applyColorMap(normalizedDepth, depthColor, cv::COLORMAP_JET);
	SLAM->VisualizeImage(User->mapName, depthColor, 3);
	////뎁스 시각화
	User->mnUsed--;
}

std::atomic<int> nAvgObjTime = 0;
std::atomic<int> nAvgObjSize = 0;
std::atomic<float> fAvgObjProcessingTime = 0.0;
std::atomic<float> fAverageObjectSize = 0.0;
ConcurrentVector<float> vecTestObjProcessingTime, vecTestObjTraffic;

std::map<int, cv::Mat> mapTempDepthImages;
std::vector<std::pair<float, float>> scaleRatios;
int nTestKF = 0;
int firstKFid = -1;

////GaussianSparseSLAM
void GaussianSparseMapping(ThreadPool::ThreadPool* POOL, GaussianSparseSLAM::GSSLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {
	if (!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++;  
	 
	auto pNewKF = User->KeyFrames.Get(id);
	User->mpRefKF = pNewKF;
	User->mbNewKF = true;
	auto future = POOL->EnqueueJob(GaussianSparseSLAM::Mapper::ProcessMapping, POOL, SLAM, User->mapName, pNewKF, true); //frame
	future.get();
	//맵 전송

	//매핑 요청. 인접키프레임 아이디를 넘길가?
	//맵데이터, 포즈 등을 넘기기
	cv::Mat Pcw = pNewKF->GetPose();
	cv::Mat kfdata = cv::Mat::zeros(1000, 1, CV_32FC1); //inlier, pose + point2f, octave, angle, point3f
	cv::Mat mpdata = cv::Mat::zeros(1, 1, CV_32FC1);
	int nTrackData = 0;
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(0, 0);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(0, 1);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(0, 2);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(1, 0);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(1, 1);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(1, 2);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(2, 0);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(2, 1);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(2, 2);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(0, 3);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(1, 3);
	kfdata.at<float>(nTrackData++) = Pcw.at<float>(2, 3);
	{
		std::stringstream ss;
		ss << User->mapName<< "." << src << "." << pNewKF->mnFrameId;
		Utils::SendData("datakfpose", ss.str(), kfdata, pNewKF->mnId, 4, 2);
	}

	std::vector<cv::Vec4f> vecDatas;
	auto vpGPs = pNewKF->mvpMapPoints.get();
	for (int i = 0; i < vpGPs.size(); i++) {
		auto pGPi = vpGPs[i];
		if (!pGPi || pGPi->isBad())
			continue;
		cv::Mat X = pGPi->GetWorldPos();
		cv::Vec4f vec4(
			static_cast<float>(id),
			X.at<float>(0, 0),
			X.at<float>(1, 0),
			X.at<float>(2, 0)
		);
		vecDatas.push_back(vec4);
	}
	std::stringstream ss2;
	{                                                                                                                                                                                                                                                                               
		std::stringstream ss;
		ss << User->mapName << "." << src << "." << pNewKF->mnFrameId;
		auto vpNeighKFs = pNewKF->GetBestCovisibilityKeyFrames(10);
		for (auto pNeighKF : vpNeighKFs)
		{
			if (!pNeighKF || pNeighKF->isBad())
				continue;
			ss2 << "." << pNeighKF->mnId;
		}
		cv::Mat tmp = cv::Mat(vecDatas.size(), 1, CV_32FC4, vecDatas.data());
		Utils::SendData("datasparsemap", ss.str(), tmp, pNewKF->mnId, 4, 4);
	}
	 
	std::stringstream ss;
	ss << User->mapName << "." << src <<".GSImage" << "." << pNewKF->mnFrameId<<ss2.str();
	Utils::SendReqMessage("reqgsmapping", ss.str(), pNewKF->mnId);
	User->mnUsed--;
}
void TrackWithKeyFrames(ThreadPool::ThreadPool* POOL, GaussianSparseSLAM::GSSLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {
	auto vecstr = split(src, '.');
	std::string user = vecstr[0];

	if (vecstr.size() < 2)
		return;

	if (!SLAM->CheckUser(user)) {
		return;
	}
	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	User->mnUsed++;

	//키프레임 매칭 기록
	std::vector<GaussianSparseSLAM::KeyFrame*> vecKFs;
	std::vector<std::vector<cv::Point2i>> vecKFMatches;
	WebAPI API(ip, port);

	auto f2 = User->mFrames.Get(id);
	f2->reset_map_points();

	std::set<GaussianSparseSLAM::GaussianPoint*> spGPs;
	std::set<int> sAleadyIdxs;

	auto map = User->GetMap();
	

	std::map<int, GaussianSparseSLAM::KeyFrame*> mapKFs;
	{
		std::stringstream ss;
		ss << "/Download?keyword=" << "resintrapr" << "&id=" << id << "&src=" << user;
		auto res = API.Send(ss.str(), "");
		int num_pts = res.size() / sizeof(int);
		const int* ptr = reinterpret_cast<const int*>(res.data());
		std::vector<int> vecKFs(ptr, ptr + num_pts);

		for (auto kfid : vecKFs)
		{
			auto pKF = map->GetKeyFrame(kfid);
			if (pKF)
				mapKFs[pKF->mnFrameId] = pKF;
		}
	}
	

	////처음 키프레임 매칭
	{
		std::stringstream ss;
		ss << "/Download?keyword=" << "resmapmatch" << "&id=" << id << "&src=" << src;
		auto res = API.Send(ss.str(), "");

		int idRefKF = std::stoi(vecstr[2]);
		auto pRefKF = mapKFs[idRefKF];
		int num_pts = res.size() / sizeof(cv::Point2i);

		std::cout << user << " = " << src << " = " << num_pts << std::endl;

		if (pRefKF)
		{
			
			const cv::Point2i* ptr = reinterpret_cast<const cv::Point2i*>(res.data());
			std::vector<cv::Point2i> vecRefKF(ptr, ptr + num_pts);

			GaussianSparseSLAM::Tracker::KeyFrameMatch(f2, pRefKF, vecRefKF, sAleadyIdxs, spGPs);

			vecKFs.push_back(pRefKF);
			vecKFMatches.push_back(vecRefKF);
		}
		
	}
	////처음 키프레임 매칭

	//f2 셋포즈가 필요함
	for (int i = 4; i < vecstr.size(); i += 3)
	{
		std::string srcLastKF = vecstr[i] + "." + vecstr[i + 1];

		int idNeighKF = std::stoi(vecstr[i + 1]);
		std::stringstream ss;
		ss << "/Download?keyword=" << "resmapmatcha" << "&id=" << id << "&src=" << srcLastKF;
		auto res2 = API.Send(ss.str(), "");

		//cv::Mat mat2 = cv::Mat(res2.size() / 8, 1, CV_32SC2, (void*)res2.data());
		//vecNeighKF.assign((cv::Point2i*)mat2.datastart, (cv::Point2i*)mat2.dataend);

		int num_pts = res2.size() / sizeof(cv::Point2i);
		
		std::cout << user << " = " << srcLastKF << " = " << num_pts << std::endl;

		auto pNeighKF = mapKFs[idNeighKF];//map->GetKeyFrame(idNeighKF);
		if (pNeighKF)
		{
			const cv::Point2i* ptr = reinterpret_cast<const cv::Point2i*>(res2.data());
			std::vector<cv::Point2i> vecNeighKF(ptr, ptr + num_pts);

			GaussianSparseSLAM::Tracker::KeyFrameMatch(f2, pNeighKF, vecNeighKF, sAleadyIdxs, spGPs);
			vecKFs.push_back(pNeighKF);
			vecKFMatches.push_back(vecNeighKF);
		}
	}
	
	int nInliers = GaussianSparseSLAM::Tracker::Track(POOL, SLAM, id, user, f2, frame_ts);
	
	User->mnPrevFrameID = User->mnCurrFrameID.load();
	User->mnCurrFrameID = f2->mnFrameID;
	User->prevFrame = f2;
	
	std::set<int> sTemporalIdxs;
	if (User->GetState() == GaussianSparseSLAM::UserState::Success)
	{
		bool bKF = User->mpRefKF != nullptr;
		std::cout << "relocalization success = " << nInliers <<" = "<<id <<" "<<bKF<<std::endl;
	}else{
		User->SetState(GaussianSparseSLAM::UserState::Failed);
		std::cout << "relocalization failed = " <<(int)User->GetState()<<"=="<<nInliers <<" "<< vecKFMatches.size() <<"=" <<mapKFs.size()<< std::endl;
	}
	User->mnUsed--;
}
void Track(ThreadPool::ThreadPool* POOL, GaussianSparseSLAM::GSSLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts)
{
	auto vecstr = split(src, '.');
	std::string user = vecstr[0];

	if (!SLAM->CheckUser(user)) {
		return;
	}
	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	User->mnUsed++;

	std::string mapName = User->mapName;
	auto map = User->GetMap();

	//parsing
	auto mapState = map->GetState();

	//initialization
	//relocalization
	//track - prev, local map, reference key frame

	User->mnUsed--;
}
void TrackWithPrevFrame(ThreadPool::ThreadPool* POOL, GaussianSparseSLAM::GSSLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {

	auto vecstr = split(src, '.');
	std::string user = vecstr[0];

	//std::cout << "trackwithprevframe::start" << src << std::endl;

	if (!SLAM->CheckUser(user)) {
		return;
	}
	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	User->mnUsed++;

	std::string mapName = User->mapName;
	auto map = User->GetMap();

	//std::string srcRefKF = vecstr[1] + "." + vecstr[2];
	int idRefKF = std::stoi(vecstr[3]);
	auto pRefKF = map->GetKeyFrame(idRefKF); //User->KeyFrames.Get(idRefKF);

	//키프레임 매칭 기록
	std::vector<GaussianSparseSLAM::KeyFrame*> vecKFs;
	std::vector<std::vector<cv::Point2i>> vecKFMatches;

	WebAPI API(ip, port);

	//match reference frame
	std::vector<cv::Point2i> vecRefKF, vecPrevFrame;
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss;
	ss << "/Download?keyword=" << "resmatch" << "&id=" << id << "&src=" << src;
	auto res = API.Send(ss.str(), "");

	cv::Mat mat = cv::Mat(res.size() / 8, 1, CV_32SC2, (void*)res.data());
	vecRefKF.assign((cv::Point2i*)mat.datastart, (cv::Point2i*)mat.dataend);
	
	vecKFs.push_back(pRefKF);
	vecKFMatches.push_back(vecRefKF);

	//prev matching
	int idPrevFrame = -1;
	bool bPrevMatch = false;
	if(vecstr.size() > 4)
	{
		std::string srcPrev = vecstr[4] + "." + vecstr[5];
		idPrevFrame = std::stoi(vecstr[5]);
		std::stringstream ss;
		ss << "/Download?keyword=" << "resmatcha" << "&id=" << id << "&src=" << srcPrev;
		auto res2 = API.Send(ss.str(), "");
		cv::Mat mat2 = cv::Mat(res2.size() / 8, 1, CV_32SC2, (void*)res2.data());
		vecPrevFrame.assign((cv::Point2i*)mat2.datastart, (cv::Point2i*)mat2.dataend);
		bPrevMatch = true;
	}

	auto f2 = User->mFrames.Get(id);
	f2->reset_map_points();

	std::set<GaussianSparseSLAM::GaussianPoint*> spGPs;
	std::set<int> sAleadyIdxs;
	if (bPrevMatch)
	{
		auto fprev = User->mFrames.Get(idPrevFrame);
		f2->SetPose(User->PredictPose());
		GaussianSparseSLAM::Tracker::FrameMatch(f2, fprev, vecPrevFrame, sAleadyIdxs, spGPs);
	}
	//GaussianSparseSLAM::Tracker::TrackWithFrame(POOL, SLAM, id, user, f2, frame_ts);

	bool bLastKFMatch = false;
	float th_ref = 25.0;
	if (!bPrevMatch)
		th_ref = 175.0;
	auto vpRefGPs = pRefKF->mvpMapPoints.get();
	if (!bPrevMatch && !bLastKFMatch)
	{
		f2->SetPose(pRefKF->GetPose());
	}
	GaussianSparseSLAM::Tracker::KeyFrameMatch(f2, pRefKF, vecRefKF, sAleadyIdxs, spGPs, th_ref);
	
	for (int i = 7; i < vecstr.size(); i += 3)
	{
		std::string srcLastKF = vecstr[i] + "." + vecstr[i+1];
		int idNeighKF = std::stoi(vecstr[i+1]);
		int idNeighKF2 = std::stoi(vecstr[i + 2]);
		std::stringstream ss;
		ss << "/Download?keyword=" << "resmatcha" << "&id=" << id << "&src=" << srcLastKF;
		auto res2 = API.Send(ss.str(), "");


		//cv::Mat mat2 = cv::Mat(res2.size() / 8, 1, CV_32SC2, (void*)res2.data());
		//vecNeighKF.assign((cv::Point2i*)mat2.datastart, (cv::Point2i*)mat2.dataend);

		int num_pts = res2.size() / sizeof(cv::Point2i);
		const cv::Point2i* ptr = reinterpret_cast<const cv::Point2i*>(res2.data());
		std::vector<cv::Point2i> vecNeighKF(ptr, ptr + num_pts);
		
		auto pNeighKF = map->GetKeyFrame(idNeighKF2);

		//auto pNeighKF = User->KeyFrames.Get(idNeighKF);
		if (pNeighKF)
		{
			if (!bPrevMatch)
				f2->SetPose(pNeighKF->GetPose());
			GaussianSparseSLAM::Tracker::KeyFrameMatch(f2, pNeighKF, vecNeighKF, sAleadyIdxs, spGPs);
			vecKFs.push_back(pNeighKF);
			vecKFMatches.push_back(vecNeighKF);
		}
	}
	
	int nInliers = GaussianSparseSLAM::Tracker::Track(POOL, SLAM, id, user, f2, frame_ts);

	User->mnPrevFrameID = User->mnCurrFrameID.load();
	User->mnCurrFrameID = f2->mnFrameID;
	/*if (User->prevFrame)
		delete User->prevFrame;*/
	User->prevFrame = f2;

	std::set<int> sTemporalIdxs;
	if (User->GetState() == GaussianSparseSLAM::UserState::Success)
	{
		User->mSetMapPoints.Clear();
		auto vpGPs = f2->mvGaussianPoints.get();
		for (int i = 0; i < f2->N; i++)
		{
			auto pGPi = vpGPs[i];
			if (!pGPi || pGPi->isBad() || f2->mvbOutliers[i])
				continue;
			User->mSetMapPoints.Update(pGPi);
		}

		//Create Temporal Maaping
		if (false && nInliers < 200)
		{
			std::cout << "temporal mp :: start" << std::endl;
			int nnew = 0;
			
			cv::Mat Rcw2 = f2->GetRotation();
			cv::Mat Rwc2 = Rcw2.t();
			cv::Mat tcw2 = f2->GetTranslation();
			cv::Mat Tcw2(3, 4, CV_32F);
			Rcw2.copyTo(Tcw2.colRange(0, 3));
			tcw2.copyTo(Tcw2.col(3));

			const float& fx2 = f2->fx;
			const float& fy2 = f2->fy;
			const float& cx2 = f2->cx;
			const float& cy2 = f2->cy;
			const float& invfx2 = f2->invfx;
			const float& invfy2 = f2->invfy;

			std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
			long long ts = start.time_since_epoch().count();
			auto pMap = User->GetMap();

			for (int i = 0; i < vecKFs.size(); i++) {
				auto pKF = vecKFs[i];
				auto vecMatch = vecKFMatches[i];

				cv::Mat Rcw1 = pKF->GetRotation();
				cv::Mat Rwc1 = Rcw1.t();
				cv::Mat tcw1 = pKF->GetTranslation();
				cv::Mat Tcw1(3, 4, CV_32F);
				Rcw1.copyTo(Tcw1.colRange(0, 3));
				tcw1.copyTo(Tcw1.col(3));
				cv::Mat Ow1 = pKF->GetCameraCenter();

				const float& fx1 = pKF->fx;
				const float& fy1 = pKF->fy;
				const float& cx1 = pKF->cx;
				const float& cy1 = pKF->cy;
				const float& invfx1 = pKF->invfx;
				const float& invfy1 = pKF->invfy;

				//키프레임에 키포인트가 없고
				//현재 프레임에도 아무 매칭 기록이 없으면
				//뎁스로 포인트 생성하고
				//리프로젝션 에러 계산한 후에 추가

				for (int ikp = 0; ikp < vecMatch.size(); ikp++)
				{
					const int& idx_kf = vecMatch[ikp].x;
					const int& idx_f = vecMatch[ikp].y;
					   
					auto pKF_GP = pKF->mvpMapPoints.get(idx_kf);
					if (pKF_GP && !pKF_GP->isBad())
						continue;
					auto pF_GP = f2->mvGaussianPoints.get(idx_f);
					if (pF_GP && !pF_GP->isBad() && !f2->mvbOutliers[idx_f])
						continue;

					const cv::Point2f& kp1 = pKF->mvKeysUn[idx_kf].pt;
					const cv::Point2f& kp2 = f2->mvKeysUn[idx_f].pt;

					//cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.x - cx1) * invfx1, (kp1.y - cy1) * invfy1, 1.0);
					//cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.x - cx2) * invfx2, (kp2.y - cy2) * invfy2, 1.0);

					//cv::Mat ray1 = Rwc1 * xn1;
					//cv::Mat ray2 = Rwc2 * xn2;
					//cv::Mat A(4, 4, CV_32F);
					//A.row(0) = (xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0));
					//A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
					//A.row(2) = (xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0));
					//A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

					//cv::Mat w, u, vt;
					//cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

					//cv::Mat x3D = vt.row(3).t();

					//if (x3D.at<float>(3) == 0) {
					//	continue;
					//}
					//// Euclidean coordinates
					//x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);

					cv::Mat x3D = pKF->UnprojectStereo(idx_kf, Rwc1, Ow1);
					x3D = x3D.t();

					//depth test
					float z1 = Rcw1.row(2).dot(x3D) + tcw1.at<float>(2);
					if (z1 <= 0) {
						continue;
					}

					float z2 = Rcw2.row(2).dot(x3D) + tcw2.at<float>(2);
					if (z2 <= 0) {
						continue;
					}

					//reprojection test
					const float x1 = Rcw1.row(0).dot(x3D) + tcw1.at<float>(0);
					const float y1 = Rcw1.row(1).dot(x3D) + tcw1.at<float>(1);
					const float invz1 = 1.0 / z1;
					
					float u1 = fx1 * x1 * invz1 + cx1;
					float v1 = fy1 * y1 * invz1 + cy1;
					float errX1 = u1 - kp1.x;
					float errY1 = v1 - kp1.y;
					if ((errX1 * errX1 + errY1 * errY1) > 5.991) {
						//std::cout << "err1 = " << errX1 * errX1 + errY1 * errY1 << std::endl;
						continue;
					}

					const float x2 = Rcw2.row(0).dot(x3D) + tcw2.at<float>(0);
					const float y2 = Rcw2.row(1).dot(x3D) + tcw2.at<float>(1);
					const float invz2 = 1.0 / z2;
					
					float u2 = fx2 * x2 * invz2 + cx2;
					float v2 = fy2 * y2 * invz2 + cy2;
					float errX2 = u2 - kp2.x;
					float errY2 = v2 - kp2.y;
					if ((errX2 * errX2 + errY2 * errY2) > 5.991) {
						//std::cout << "err2 = " << errX2 * errX2 + errY2 * errY2 << std::endl;
						continue;
					}

					GaussianSparseSLAM::GaussianPoint* pMP = new GaussianSparseSLAM::GaussianPoint(x3D, pKF, pMap, ts);

					pMP->AddObservation(pKF, idx_kf);

					pKF->AddGaussianPoint(pMP, idx_kf);
					f2->mvGaussianPoints.update(idx_f, pMP);
					f2->mvbOutliers[idx_f] = false;

					pMap->AddGaussianPoint(pMP);
					pMap->mlpNewMPs.push_back(pMP);

					sTemporalIdxs.insert(idx_f);
					nnew++;
				}
			}
			std::cout << "temporal mp :: end = " <<nnew<< std::endl;
		}
		
	}

	if (f2->GetState() == GaussianSparseSLAM::FrameProcessStatus::mapping)
	{
		std::string tsrc = user + ".GSImage";
		Utils::SendReqMessage("requnidepth", tsrc, id);
		{
			std::stringstream ss2;
			ss2 << mapName << "." << user << ".GSImage." << id;
			Utils::SendReqMessage("reqmapsalad", ss2.str(), f2->mnKeyFrameId);
		}
	}

	/*std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
	float t_test1 = du_test1 / 1000.0;
	std::cout << "tracking test = " << id <<" " << " " << t_test1 << std::endl;*/

	int nVisID = User->GetVisID() + 4;

	//visualization
	//빨간색은 아예 맵포인트가 없는 것
	//파란색이면 리프로젝션 에러가 큰것
	//녹색이 인라이어
	{
		cv::Mat temp1 = User->ImageDatas.Get(id);
		cv::Mat img1 = cv::imdecode(temp1, cv::IMREAD_COLOR);
		cv::Mat visImg1 = img1.clone();
		auto vpGPs = f2->mvGaussianPoints.get();
		for (int i = 0; i < f2->N; i++)
		{
			cv::circle(visImg1, f2->mvKeys[i].pt, 1, cv::Scalar(0, 0, 255), -1);
			auto pGPi = vpGPs[i];
			if (sAleadyIdxs.count(i))
				cv::circle(visImg1, f2->mvKeys[i].pt, 2, cv::Scalar(255, 0, 0), -1);
			if (!pGPi || pGPi->isBad() || f2->mvbOutliers[i]){
				continue;
			}
			if(sTemporalIdxs.count(i))
				cv::circle(visImg1, f2->mvKeys[i].pt, 3, cv::Scalar(0, 255, 255), -1);
			else
				cv::circle(visImg1, f2->mvKeys[i].pt, 3, cv::Scalar(0, 255, 0), -1);
		}
		SLAM->VisualizeImage(mapName, visImg1, nVisID);
	}
	
	{
		auto pKF = User->mpRefKF;
		if (pKF)
		{
			int id2 = pKF->mnFrameId;
			cv::Mat temp1 = User->ImageDatas.Get(id);
			cv::Mat img1 = cv::imdecode(temp1, cv::IMREAD_COLOR);
			cv::Mat visImg1 = img1.clone();
			cv::Mat temp2 = User->ImageDatas.Get(id2);
			cv::Mat img2 = cv::imdecode(temp2, cv::IMREAD_COLOR);
			cv::Mat visImg2 = img2.clone();

			std::vector<cv::DMatch> matches;
			GaussianSparseSLAM::Matcher::matchEigen(matches, f2->mDescriptors, pKF->mDescriptors);

			std::vector<std::pair<cv::Point2f, cv::Point2f>> matches2;
			for (auto match : matches)
			{
				auto pt1 = f2->mvKeys[match.queryIdx].pt;
				auto pt2 = pKF->mvKeys[match.trainIdx].pt;
				matches2.push_back(std::make_pair(pt1, pt2));
			}
			cv::Mat res;
			SLAM->VisualizeMatchingImage(res, img1, img2, matches2, mapName, 5);
		}
	}


	User->mnUsed--;
}
void TrackWithLM(ThreadPool::ThreadPool* POOL, GaussianSparseSLAM::GSSLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts)
{
	//auto vecstr = split(tsrc,'.');
	//auto src = vecstr[2];

	if (!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++;

	std::string mapName = User->mapName;

	WebAPI API(ip, port);
	std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss;
	ss << "/Download?keyword=" << "reslmmatch" << "&id=" << id << "&src=" << src;
	auto res = API.Send(ss.str(), "");
	std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();

	cv::Mat mat = cv::Mat(res.size() / 8, 1, CV_32SC2, (void*)res.data());
	std::vector<cv::Point2i> vec;
	vec.assign((cv::Point2i*)mat.datastart, (cv::Point2i*)mat.dataend);

	auto f = User->mFrames.Get(id);
	f->reset_map_points();

	auto pMap = User->GetMap();
	auto mapAllMPs = pMap->mmpGaussianPoints.Get();

	//시각화
	cv::Mat temp = User->ImageDatas.Get(id);
	cv::Mat img = cv::imdecode(temp, cv::IMREAD_COLOR);
	cv::Mat visImg = img.clone();

	auto pRefKF = User->mpRefKF;
	cv::Mat R = pRefKF->GetRotation();
	cv::Mat t = pRefKF->GetTranslation();
	for (int i = 0; i < vec.size(); i++)
	{
		int gidx = vec[i].x;
		int pidx = vec[i].y;

		auto pGPi = mapAllMPs[gidx];

		if (!pGPi || pGPi->isBad())
			continue;
		f->mvGaussianPoints.update(pidx, pGPi);

		cv::Mat X = pGPi->GetWorldPos();
		cv::Mat proj = User->mpCamera->K * (R * X + t);
		float d = proj.at<float>(2);
		cv::Point2f pt(proj.at<float>(0) / d, proj.at<float>(1) / d);
		cv::circle(visImg, pt, 3, cv::Scalar(0, 0, 255), -1);
		cv::circle(visImg, f->mvKeysUn[pidx].pt, 3, cv::Scalar(255, 0, 0), -1);
		cv::line(visImg, pt, f->mvKeysUn[pidx].pt, cv::Scalar(0, 255, 0), 1);
	}
	//std::cout << "lm match " << id << " " << vec[0] << std::endl;
	GaussianSparseSLAM::Tracker::Track(POOL, SLAM, id, src, f, frame_ts);
	if (f->GetState() == GaussianSparseSLAM::FrameProcessStatus::mapping)
	{
		std::string tsrc = src + ".GSImage";
		Utils::SendReqMessage("requnidepth", tsrc, id);
	}

	//XFeat에 매칭 정보 전달

	SLAM->VisualizeImage(mapName, visImg, 7);

	User->mnUsed--;
}
void RequestXFeatLMMatch(ThreadPool::ThreadPool* POOL, GaussianSparseSLAM::GSSLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts)
{
	auto vecstr = split(src, '.');
	std::string user = vecstr[0];
	if (!SLAM->CheckUser(user)) {
		return;
	}
	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	User->mnUsed++;

	std::string mapName = User->mapName;
	std::string tsrc = user + ".GSImage";

	auto mapStat = User->GetMap()->GetState();
	auto userStat = User->GetState();

	//keypoint download
	WebAPI API(ip, port);
	std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss;
	ss << "/Download?keyword=" << "resdetectandmatch" << "&id=" << id << "&src=" << src;
	auto res = API.Send(ss.str(), "");
	std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();

	cv::Mat mat = cv::Mat(res.size() / 8, 1, CV_32FC2, (void*)res.data());
	auto f = User->mFrames.Get(id);
	std::vector<cv::Point2f> vecPTs;
	vecPTs.assign((cv::Point2f*)mat.datastart, (cv::Point2f*)mat.dataend);

	f->N = vecPTs.size();
	for (auto pt : vecPTs)
	{
		auto kp = cv::KeyPoint();
		kp.pt = pt;
		kp.octave = 0;
		f->mvKeys.push_back(kp);
	}

	f->UndistortKeyPoints();
	f->mvuRight = std::vector<float>(f->N, -1);
	f->mvDepth = std::vector<float>(f->N, -1);

	if (mapStat == GaussianSparseSLAM::MapState::NoImages || mapStat == GaussianSparseSLAM::MapState::NotInitialized)
	{
		//맵이 초기화 안되었으면 유니뎁스 요청 
		Utils::SendReqMessage("requnidepth", tsrc, id);
		f->SetState(GaussianSparseSLAM::FrameProcessStatus::initialization);
	}
	if (mapStat == GaussianSparseSLAM::MapState::Initialized)
	{
		if (userStat == GaussianSparseSLAM::UserState::Success)
		{
			f->SetState(GaussianSparseSLAM::FrameProcessStatus::tracking);
			//매칭도 요청

			auto pKF = User->mpRefKF;
			{
				//인접 
				int nn = 5;
				std::vector<GaussianSparseSLAM::KeyFrame*> vpNeighKFs = pKF->GetBestCovisibilityKeyFrames(nn);

				std::vector<int> vecDatas;
				vecDatas.push_back(pKF->mnId);
				for (auto pKF : vpNeighKFs) {
					if (pKF->isBad())
						continue;
					vecDatas.push_back(pKF->mnId);
				}
				cv::Mat tmp = cv::Mat(vecDatas.size(), 1, CV_32SC1, vecDatas.data());
				cv::Mat tmp2 = cv::Mat::ones(1000 - vecDatas.size(), 1, CV_32FC1) * -1;
				cv::vconcat(tmp, tmp2, tmp);


				std::stringstream sssrc;
				sssrc << mapName << "." << pKF->mnId << "." << user << "." << id;
				Utils::SendData("reqlmmatch", sssrc.str(), tmp, id, 4, 1);
			}

			{
				std::stringstream ssxfeat;
				ssxfeat << user << "." << pKF->mnFrameId << "." << user << "." << id;
				Utils::SendReqMessage("reqmatch", ssxfeat.str(), id);
			}
		}
		else
		{
			f->SetState(GaussianSparseSLAM::FrameProcessStatus::alignment);
			//살라드 등 요청
		}
	}

	User->mnUsed--;
}

void PreprocessGaussianFrameForAlignment(ThreadPool::ThreadPool* POOL, GaussianSparseSLAM::GSSLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {

	if (!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++; 

	//reqmapmatch 요청
	WebAPI API(ip, port);
	std::stringstream ss;
	ss << "/Download?keyword=" << "resintrapr" << "&id=" << id << "&src=" << src;
	auto res = API.Send(ss.str(), "");
	int num_pts = res.size() / sizeof(int);
	const int* ptr = reinterpret_cast<const int*>(res.data());
	std::vector<int> vecKFs(ptr, ptr + num_pts);

	{
		auto map = User->GetMap();
		
		std::stringstream ssxfeat;
		//src = device, target_name, target_fid , target_kfid, ...
		//id = frame id
		ssxfeat << src;

		for (auto kfid : vecKFs)
		{
			auto pKF = map->GetKeyFrame(kfid);
			if(pKF)
				ssxfeat << "." << pKF->sourceName << "." << pKF->mnFrameId<<"."<<pKF->mnId;
		}
		//prev, reference, last_kf
		//ssxfeat << src << "." << User->mpRefKF->mnFrameId << "." << src << "." << id << "." << User->mnPrevFrameID;
		Utils::SendReqMessage("reqmapmatch", ssxfeat.str(), id);
		std::cout << ssxfeat.str() << std::endl;
	}

	std::cout << "intra matching test = " << vecKFs.size() << std::endl;
	//resmapmatch에서 처리
	//현재 프레임과 키프레임들 매칭

	User->mnUsed--;
}
void PreprocessGaussianFrame(ThreadPool::ThreadPool* POOL, GaussianSparseSLAM::GSSLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {

	if (!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++;

	//Image Download
	//std::cout << "frame ts test " << frame_ts << std::endl;
	WebAPI API(ip, port);
	auto f = new GaussianSparseSLAM::Frame(User->mpCamera, SLAM->mpFeatureScaleInfo, id, frame_ts);
	User->mFrames.Update(id, f);
	User->mnCurrFrameID = id;

	{
		std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
		std::stringstream ss;
		ss << "/Download?keyword=" << "GSImage" << "&id=" << id << "&src=" << src;
		auto res = API.Send(ss.str(), "");
		std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();
		cv::Mat temp = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());

		//인코딩 정보 저장
		User->ImageDatas.Update(id, temp.clone());
	}

	//auto f = User->mFrames.Get(id);
	auto mapStat = User->GetMap()->GetState();
	auto userStat = User->GetState();

	std::string mapName = User->mapName;
	std::string tsrc = src + ".GSImage";

	//point download
	
	std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss;
	ss << "/Download?keyword=" << "resdetect" << "&id=" << id << "&src=" << src;
	auto res = API.Send(ss.str(), "");
	std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();
	cv::Mat mat = cv::Mat(res.size() / 8, 1, CV_32FC2, (void*)res.data());
	std::vector<cv::Point2f> vecPTs;
	vecPTs.assign((cv::Point2f*)mat.datastart, (cv::Point2f*)mat.dataend);    

	//descriptor download
	float scale, zero_point;
	{
		std::stringstream ss;
		ss << "/Download?keyword=" << "resdesc" << "&id=" << id << "&src=" << src;  
		auto res = API.Send(ss.str(), "");
		int matrix_data_size = res.size() - 8;
		int rows = matrix_data_size / 64;
		int cols = 64;
		cv::Mat mat = cv::Mat(rows, cols, CV_8SC1, (void*)res.data());
		
		const char* raw_data_ptr = res.data();
		const char* float_data_ptr = raw_data_ptr + matrix_data_size;
		std::memcpy(&scale, float_data_ptr, sizeof(float));
		std::memcpy(&zero_point, float_data_ptr + sizeof(float), sizeof(float));
		
		Utils::dequantizeDescriptors(mat, f->mDescriptors, scale, zero_point);
	}

	f->N = vecPTs.size();
	for (auto pt : vecPTs)
	{
		auto kp = cv::KeyPoint();
		kp.pt = pt;
		kp.octave = 0;
		f->mvKeys.push_back(kp);
	}
	f->UndistortKeyPoints();
	f->mvuRight = std::vector<float>(f->N, -1);
	f->mvDepth = std::vector<float>(f->N, -1);
	
	////초기화(맵이 없음)
	//뎁스 요청=  src = src+.GSImage
	//프레임스테이트 = 초기화
	//id = frame_ids
	////키프레임과 매칭
	//keyword = reqmatch
	//src = src + keyframe_id + src + frame_id
	//id = frame_id
	////로컬 맵 매칭
	//keyword = reqlmmatch
	//src = map + src + keyframe_id + src + frame_id
	//id = frame_id
	////정합(기기 자체가 최초 등록. 맵은 있음)
	//keyword = 
	//src = 
	//id = 

	//초기화 또는 매칭 요청
	if (mapStat == GaussianSparseSLAM::MapState::NoImages || mapStat == GaussianSparseSLAM::MapState::NotInitialized)
	{
		//맵이 초기화 안되었으면 유니뎁스 요청
		Utils::SendReqMessage("requnidepth", tsrc, id);
		f->SetState(GaussianSparseSLAM::FrameProcessStatus::initialization);
	}
	if (mapStat == GaussianSparseSLAM::MapState::Initialized)
	{
		if (userStat == GaussianSparseSLAM::UserState::Success)
		{
			f->SetState(GaussianSparseSLAM::FrameProcessStatus::tracking);

			//LM 요청
			//{
			//	int nn = 10;
			//	auto pKF = User->mpRefKF;
			//	std::vector<GaussianSparseSLAM::KeyFrame*> vpNeighKFs = pKF->GetBestCovisibilityKeyFrames(nn);
			//	std::vector<int> vecDatas;
			//	vecDatas.push_back(pKF->mnId);
			//	for (auto pKF : vpNeighKFs) {
			//		if (pKF->isBad())
			//			continue;
			//		vecDatas.push_back(pKF->mnId);
			//	}
			//	cv::Mat tmp = cv::Mat(vecDatas.size(), 1, CV_32SC1, vecDatas.data());
			//	cv::Mat tmp2 = cv::Mat::ones(1000 - vecDatas.size(), 1, CV_32SC1) * -1;
			//	cv::vconcat(tmp, tmp2, tmp);
			//	std::stringstream sssrc;
			//	sssrc << mapName << "." << User->mpRefKF->mnId << "." << src << "." << id;
			//	//Utils::SendReqMessage("reqlmmatch", sssrc.str(), id);
			//	Utils::SendData("reqlmmatch", sssrc.str(), tmp, id, 4, 1);
			//}

			{
				std::stringstream ssxfeat;
				//src = device, target_d, target_id , ...
				//id = frame id
				ssxfeat << src << "." << User->mpRefKF->sourceName << "." << User->mpRefKF->mnFrameId <<"."<<User->mpRefKF->mnId << "." << src << "." << User->mnPrevFrameID<<"."<<User->mnPrevFrameID;

				auto vecNeigh = User->mpRefKF->GetBestCovisibilityKeyFrames(5);
				std::set<GaussianSparseSLAM::KeyFrame*> setNeigh(vecNeigh.begin(), vecNeigh.end());

				if (User->mpLastCreatedKF && User->mpRefKF && (User->mpRefKF->mnId != User->mpLastCreatedKF->mnId))
				{
					//ssxfeat << "." << User->mpLastCreatedKF->sourceName << "." << User->mpLastCreatedKF->mnFrameId;
					if (!setNeigh.count(User->mpLastCreatedKF))
						setNeigh.insert(User->mpLastCreatedKF);
				}

				for (auto pNeigh : setNeigh)
				{
					ssxfeat << "." << pNeigh->sourceName << "." << pNeigh->mnFrameId<<"."<<pNeigh->mnId;
				}

				//prev, reference, last_kf
				//ssxfeat << src << "." << User->mpRefKF->mnFrameId << "." << src << "." << id << "." << User->mnPrevFrameID;
				Utils::SendReqMessage("reqmatch", ssxfeat.str(), id);
				//std::cout << "preprocessing " << ssxfeat.str() <<"=="<< User->mpRefKF->sourceName << "." << User->mpRefKF->mnFrameId << std::endl;
			}


			//matching test
			//reference keyframe
			auto pKF = User->mpRefKF;
			if (pKF && pKF->mDescriptors.rows > 0)
			{
				std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
				std::vector<cv::DMatch> matches;
				GaussianSparseSLAM::Matcher::matchEigen(matches, f->mDescriptors, pKF->mDescriptors);
				std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
				auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
				std::cout << "match test = " << matches.size() << " " << du_test1 << std::endl;
			}
		}
		else if (userStat == GaussianSparseSLAM::UserState::RECENTLY_LOST)
		{
			//최근 키프레임과 매칭
			auto pKF = User->mpRefKF;
			if (pKF)
			{
				std::stringstream ssxfeat;
				ssxfeat << src << "." << User->mpRefKF->sourceName << "." << User->mpRefKF->mnFrameId<<"."<<User->mpRefKF->mnId;
				Utils::SendReqMessage("reqmatch", ssxfeat.str(), id);
				//std::cout << "preprocessing " << ssxfeat.str() <<"=="<< User->mpRefKF->sourceName << "." << User->mpRefKF->mnFrameId << std::endl;
			}
		}
		else if (userStat == GaussianSparseSLAM::UserState::Failed)
		{
			std::cout << "tracking failed = "<<src<<"==" <<id<< std::endl;
			f->SetState(GaussianSparseSLAM::FrameProcessStatus::alignment);
			//살라드 등 요청
			{
				//reference curr prev, last_kf
				std::stringstream sssalad;
				sssalad << mapName << "." << src << ".GSImage";
				Utils::SendReqMessage("reqintrapr", sssalad.str(), id);
			}
		}
		else
		{
			f->SetState(GaussianSparseSLAM::FrameProcessStatus::alignment);
			//살라드 등 요청
			{
				//reference curr prev, last_kf
				std::stringstream sssalad;
				sssalad << mapName << "." << src << ".GSImage";
				Utils::SendReqMessage("reqintrapr", sssalad.str(), id);
			}
		}
	}

	User->mnPrevFrameID = id;
	User->mnUsed--;
}

void GaussianFrameUpdate(ThreadPool::ThreadPool* POOL, GaussianSparseSLAM::GSSLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {
	if (!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++;

	int nVisID = User->GetVisID() + 4;
	std::string mapName = User->mapName;
	std::string tsrc = src + ".GSImage";

	//auto mapStat = User->GetMap()->GetState();
	//auto userStat = User->GetState();

	////프레임 생성
	auto f = new GaussianSparseSLAM::Frame(User->mpCamera, SLAM->mpFeatureScaleInfo, id, frame_ts);
	User->mFrames.Update(id, f);

	////XFeat
	////Utils::SendReqMessage("reqdetect", tsrc, id);

	////프레임 상태 업데이트
	//if (mapStat == GaussianSparseSLAM::MapState::NoImages || mapStat == GaussianSparseSLAM::MapState::NotInitialized)
	//{
	//	//맵이 초기화 안되었으면 유니뎁스 요청
	//	Utils::SendReqMessage("requnidepth", tsrc, id);
	//	f->SetState(GaussianSparseSLAM::FrameProcessStatus::initialization);
	//}
	//if (mapStat == GaussianSparseSLAM::MapState::Initialized)
	//{
	//	if (userStat == GaussianSparseSLAM::UserState::Success)
	//	{
	//		f->SetState(GaussianSparseSLAM::FrameProcessStatus::tracking);
	//		//매칭도 요청

	//		//std::stringstream sssrc;
	//		//sssrc << mapName << "." << User->mpRefKF->mnId << "." << src << "."<<id;
	//		//Utils::SendReqMessage("reqlmmatch", sssrc.str(), id);
	//	}
	//	else
	//	{
	//		f->SetState(GaussianSparseSLAM::FrameProcessStatus::alignment);
	//		//살라드 등 요청
	//	}
	//}

	//초기화 체크
	WebAPI API(ip, port);
	std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss;
	ss << "/Download?keyword=" << "GSImage" << "&id=" << id << "&src=" << src;
	auto res = API.Send(ss.str(), "");
	std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();
	cv::Mat temp = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());
	cv::Mat img = cv::imdecode(temp, cv::IMREAD_COLOR);
	cv::Mat visImg = img.clone();

	//인코딩 정보 저장
	User->ImageDatas.Update(id, temp.clone());
	User->mnCurrFrameID = id;

	//UniDepth
	//User->mnPrevFrameID = User->mnCurrFrameID.load();

	//Visualize
	SLAM->VisualizeImage(mapName, visImg, nVisID);
	User->mnUsed--;
}
////GaussianSparseSLAM

bool isTrueOrFalse(const std::string& str) {
	// 소문자로 변환 (대소문자 구분 없이 체크하고 싶을 때)
	std::string lowerStr = str;
	std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(), ::tolower);

	if (lowerStr == "true") return true;
	if (lowerStr == "false") return false;
	return false;
}

void parsing(char* argv[], int& index) {
	std::string strTimeStamp = Utils::GetTimeStamp("_").str();
	std::string keyword = (argv[index++]);
	if (keyword == "--latency") {
		bSaveLatency = true;
		latencyPath = argv[index++];
		std::stringstream ss_file;
		//ss_file << latencyPath << "_" << strTimeStamp << ".csv";
		ss_file << latencyPath << ".csv";
		latencyPath = ss_file.str();
		std::cout << "latency path = " << latencyPath << std::endl;
	}
	else if (keyword == "--MULTI_OBJ_TEST") {
		bMultiObjTest = true;
	}
	else if (keyword == "--save_visualization") {
		bSaveVisualization = true;
		nIncForVis = std::atoi(argv[index++]);
	}
	else if (keyword == "--vo_latency") {
		bSaveVOLatency = true;
		volatencyPath = argv[index++];
		std::stringstream ss_file;
		ss_file << volatencyPath << "_" << strTimeStamp << ".csv";
		volatencyPath = ss_file.str();
		std::cout << "vo latency path = " << volatencyPath << std::endl;
	}
	else if (keyword == "--trajectory") {
		bSaveTrajectory = true;
		bSaveLatency = true;
		std::string tempStr = argv[index++];
		/*std::stringstream ss_file;
		ss_file << trajectoryPath << "_" << strTimeStamp << ".txt";
		trajectoryPath = ss_file.str();*/
		trajectoryPath = tempStr + ".txt";
		latencyPath = tempStr + ".csv";
		std::cout << "latency path = " << latencyPath << std::endl;
		std::cout << "trajectory path = " << trajectoryPath << std::endl;
	}
	else if (keyword == "--semantickeyword")
	{
		std::string strNumDevice = argv[index++];
		nNumDevice = std::stoi(strNumDevice);
		bSaveSegLatency = true;
		deepkey1 = std::string(argv[index++]) + "_" + strNumDevice;
		deepkey2 = std::string(argv[index++]) + "_" + strNumDevice;
		deepkey3 = std::string(argv[index++]) + "_" + strNumDevice;
		std::cout << deepkey1 << " " << deepkey2 << " " << deepkey3 << std::endl;
	}
	else if (keyword == "--SemanticSLAM")
	{
		bSemantic = true;
	}
	else if (keyword == "--netsim") {
		nNetworkSimul = atoi(argv[index++]);
		std::cout << "network simulation= " << nNetworkSimul << std::endl;
	}
	else if (keyword == "--asso_sam")
	{
		bSam = true;
	}
	else if (keyword == "--save_asso_vis_image")
	{
		bSaveAssoVisImage = true;
	}
	else if (keyword == "--obj_baseline_test")
	{
		bObjBaselineTest = true;
	}
	else if (keyword == "--grid" || keyword == "--GRID")
	{
		bGridTest = true;
	}
	else if (keyword == "--ar" || keyword == "--AR")
	{
		bGridTest = true;
		bARAppTest = true;
	}
	else if (keyword == "--serialize") {
	}
	else if (keyword == "--depthanything")
	{
		bDepthAnything = true;
	}
}

void parser(int argc, char* argv[]) {
	int nIndexARGV = 3;
	std::cout << "start parsing" << std::endl;
	for (int idx = 3; idx < argc;) {
		parsing(argv, idx);
	}
}

#define ServerADDR "143.248.6.143"
int main(int argc, char* argv[])
{
	HWND console = GetConsoleWindow();
	SetWindowPos(console, 0, -6, 826, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
	int nTimeoutValue = INT_MAX;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&nTimeoutValue, sizeof(nTimeoutValue));

	ip = argv[1];
	port = atoi(argv[2]);

	//argv parsing
	parser(argc, argv);

	WebAPI::ip = ip;
	WebAPI::port = port;
	WebAPI::Connect();

	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(35001); // 포트 번호
	//serveraddr.sin_addr.s_addr = inet_addr("143.248.6.143");
	std::string addr = ip;
	inet_pton(AF_INET, addr.c_str(), &serveraddr.sin_addr.s_addr);

	//143.248.6.143:35005
	//--ECHO_SERVER_IP = 143.248.6.143
	//--ECHO_SERVER_PORT = 35001

	SetConsoleCtrlHandler((PHANDLER_ROUTINE)consoleHandler, TRUE);

	sockaddr_in clientaddr;
	int addrlen;
	char rbuf[1024];

	////Thread Pool & Run SLAM system
	ThreadPool::ThreadPool* POOL = new ThreadPool::ThreadPool(50);
	EdgeSLAM::SLAM* SLAM = new EdgeSLAM::SLAM(POOL);
	GaussianSparseSLAM::GSSLAM* GSSLAM = new GaussianSparseSLAM::GSSLAM(POOL);
	GaussianSparseSLAM::Mapper::ip = ip;
	GaussianSparseSLAM::Mapper::port = port;
	////Run SLAM system

	//////////////Connect to echo server
	////Facade Connect
	{
		WebAPI* mpAPI = new WebAPI(ip, port);
		std::vector<std::string> sendKeywords;
		std::vector<std::string> pairKeywords;
		sendKeywords.push_back("ReferenceFrame");			pairKeywords.push_back("NONE");
		
		//XFeat
		sendKeywords.push_back("reqdetect");				pairKeywords.push_back("resdetect");
		sendKeywords.push_back("reqmatch");					pairKeywords.push_back("resmatch");
		sendKeywords.push_back("reqmatchglue");				pairKeywords.push_back("resmatchglue");
		sendKeywords.push_back("reqmatches");				pairKeywords.push_back("NONE");
		sendKeywords.push_back("reqkfmatches");				pairKeywords.push_back("NONE");
		sendKeywords.push_back("reqlmmatch");				pairKeywords.push_back("NONE");
		sendKeywords.push_back("kf_gp_ids");				pairKeywords.push_back("NONE");
		//맵의 키프레임들과 매칭 요청
		sendKeywords.push_back("reqmapmatch");				pairKeywords.push_back("NONE");

		//Unidepth

		//살라드 : place recognition
		sendKeywords.push_back("reqmapsalad"); pairKeywords.push_back("NONE");
		sendKeywords.push_back("reqintrapr"); pairKeywords.push_back("NONE");
		sendKeywords.push_back("reqinterpr"); pairKeywords.push_back("NONE");

		//사실적 매핑
		//비주얼 슬램으로 만드는 데이터는 datasparsemap
		//가우시안 스플래팅으로 만드는 데이터 : datagsmap
		sendKeywords.push_back("reqgsmapping");				pairKeywords.push_back("NONE");
		sendKeywords.push_back("datasparsemap"); pairKeywords.push_back("NONE");
		sendKeywords.push_back("datakf"); pairKeywords.push_back("NONE");
		sendKeywords.push_back("datakfpose"); pairKeywords.push_back("NONE");
		
		if (bDepthAnything)
		{
			sendKeywords.push_back("DepthAnything");					pairKeywords.push_back("NONE");
		}

		time_t timer = 0;
		struct tm t;
		timer = time(NULL); // 1970년 1월 1일 0시 0분 0초부터 시작하여 현재까지의 초

		for (int i = 0, iend = sendKeywords.size(); i < iend; i++) {

			localtime_s(&t, &timer);

			std::stringstream ss;
			ss << "{\"src\":\"" << "SLAMServer" << "\"," << "\"keyword\":\"" << sendKeywords[i] << "\",\"type1\":\"server\",\"type2\":\"" << pairKeywords[i] << "\"" << ",\"capacity\":" << 300 << "}"; //test\"}";
			//std::cout << ss.str() << std::endl;
			auto res = mpAPI->Send("/Connect", ss.str());
			//std::cout <<timer<<" "<< res << std::endl;
		}
		delete mpAPI;
	}

	//////////////Connect to echo server
	{
		WebAPI* mpAPI = new WebAPI(ip, port);

		receivedKeywords.push_back("Image");
		receivedKeywords.push_back("Depth");
		receivedKeywords.push_back("Gyro");
		receivedKeywords.push_back("Accelerometer");

		//가우시안 스플래팅 관련 + XFeat Match
		receivedKeywords.push_back("GSImage");
		receivedKeywords.push_back("FrameUpdate");
		receivedKeywords.push_back("resdetect");
		receivedKeywords.push_back("resdesc");
		receivedKeywords.push_back("resdetectandmatch");
		receivedKeywords.push_back("resmatch");
		receivedKeywords.push_back("resmatches");
		receivedKeywords.push_back("reskfmatches");
		receivedKeywords.push_back("reslmmatch");
		receivedKeywords.push_back("resmatchglue");
		receivedKeywords.push_back("resgaussianmapping");
		receivedKeywords.push_back("GSDeviceConnect");
		receivedKeywords.push_back("GSDeviceDisconnect");

		//XFeat 맵 매치
		receivedKeywords.push_back("resmapmatch");

		//place recognition : relocalization, loop closing
		receivedKeywords.push_back("resintrapr");
		receivedKeywords.push_back("resinterpr");

		//gsmapping
		receivedKeywords.push_back("resgsmapping");

		if (bDepthAnything)
		{
			receivedKeywords.push_back("resdepthanything");
		}
		receivedKeywords.push_back("resdepthanything");

		std::stringstream ss;
		for (int i = 0, iend = receivedKeywords.size(); i < iend; i++) {
			ss << "{\"type1\":" << "\"connect\"" << ",\"type2\":" << "\"all\"" << ",\"src\":" << "\"SLAMServer\"" << ",\"keyword\":\"" << receivedKeywords[i] << "\"" << "}";
			std::string temp = ss.str();
			auto buf = temp.c_str();
			int retval;
			retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
			ss.str("");
			//std::cout << temp << std::endl;
		}
		delete mpAPI;
	}
	
	std::map<int, float> segDatas;
	std::map<int, float> objDatas;


	////receive data
	while (true) {

		addrlen = sizeof(clientaddr);
		int retval = recvfrom(sock, rbuf, 1024, 0, (sockaddr*)&clientaddr, &addrlen);
		rbuf[retval] = '\0';

		rapidjson::Document document;
		if (document.Parse(rbuf).HasParseError()) {
			std::cout << "JSON parsing error" << std::endl;
			continue;
		}

		std::chrono::high_resolution_clock::time_point t_parsing_start = std::chrono::high_resolution_clock::now();

		if (document["keyword"].IsString() && document["id"].IsInt()) {
			int id = document["id"].GetInt();
			double ts = std::stod(document["ts"].GetString());
			std::string keyword = document["keyword"].GetString();
			std::string src = document["src"].GetString();
			std::string type2 = document["type2"].GetString();

			/*if(keyword=="Image")
				std::cout << std::setprecision(16) <<"latency test = " <<keyword<<", " << (t_parsing_start.time_since_epoch().count() - ts) / 1000000.0 << std::endl;*/

				//////load data with keyword
			int id2 = -1;
			std::stringstream ss;
			if (document.HasMember("id2")) {
				id2 = document["id2"].GetInt();
				ss << "/Download?keyword=" << keyword << "&id=" << id << "&id2=" << id2 << "&src=" << src;
			}
			else
				ss << "/Download?keyword=" << keyword << "&id=" << id << "&src=" << src;

			//datset ts
			double ts2 = ts;
			if (document.HasMember("ts2")) {
				ts2 = std::stod(document["ts2"].GetString()); //document["ts2"].GetDouble();// std::stod(document["ts2"].GetString());
			}

			////data processing
			if (keyword == "Image") {
				//POOL->EnqueueJob(Track, POOL, SLAM, src, ss.str(), id, ts, ts2);
				//POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::ObjectTracking, SLAM, src, id);
			}
			else if (keyword == "resdepthanything")    
			{
				POOL->EnqueueJob(UpdateDepthFrame, GSSLAM, keyword, src, id, ts, false);
			}
			else if (keyword == "FrameUpdate")
			{
				//이미지만 다운로드함. 현재는
				//POOL->EnqueueJob(GaussianFrameUpdate, POOL, GSSLAM, src, ss.str(), id, ts, ts2);
			}
			else if (keyword == "resdetect")
			{
				POOL->EnqueueJob(PreprocessGaussianFrame, POOL, GSSLAM, src, ss.str(), id, ts, ts2);
			}
			else if (keyword == "resdetectandmatch")
			{
				//POOL->EnqueueJob(RequestXFeatLMMatch, POOL, GSSLAM, src, ss.str(), id, ts, ts2);
			}
			else if (keyword == "resmatch")
			{
				//std::cout << "resmatch " << src << " " << id << std::endl;
				POOL->EnqueueJob(TrackWithPrevFrame, POOL, GSSLAM, src, ss.str(), id, ts, ts2);
			}
			else if (keyword == "reskfmatches")
			{
				POOL->EnqueueJob(GaussianSparseMapping, POOL, GSSLAM, src, ss.str(), id, ts, ts2);
			}
			else if (keyword == "reslmmatch")
			{
				//std::cout << "reslmmatch " << src << " " << id << std::endl;
				//POOL->EnqueueJob(TrackWithLM, POOL, GSSLAM, src, ss.str(), id, ts, ts2);
			}
			else if (keyword == "resintrapr")
			{
				POOL->EnqueueJob(PreprocessGaussianFrameForAlignment, POOL, GSSLAM, src, ss.str(), id, ts, ts2);
			}
			else if (keyword == "resmapmatch")
			{
				POOL->EnqueueJob(TrackWithKeyFrames, POOL, GSSLAM, src, ss.str(), id, ts, ts2);
			}
			else if (keyword == "resgsmapping") {
				POOL->EnqueueJob(UpdateGSDepthFrame, GSSLAM, keyword, src, id, ts, false);
			}
			else if (keyword == "GSDeviceDisconnect") {
				if (GSSLAM->CheckUser(src)) {
					auto pUser = GSSLAM->GetUser(src);
					pUser->mnUsed++;

					bool bGBA = pUser->mbPlaneGBA;
					bool bReset = pUser->mbResetAR;
					//SLAM->SaveTrajectory(src);

					std::string tempMapName = pUser->mapName;
					pUser->mnUsed--;
					GSSLAM->RemoveUser(src);
					if (keyword == "GSDeviceDisconnect" && !GSSLAM->CheckUser(src)) {
						GSSLAM->UpdateUserVisID();
					}

					if (GSSLAM->GetConnectedDevice() == 0) {

						if (bSaveTrajectory) {
							std::cout << "save start" << std::endl;
							std::cout << trajectoryPath << std::endl;
							GSSLAM->SaveTrajectory(trajectoryPath, tempMapName);
							std::cout << "save end" << std::endl;
							return 0;
						}

					}
				}
			}
			else if (keyword == "GSDeviceConnect")
			{
				WebAPI API(ip, port);
				auto res = API.Send(ss.str(), "");
				int nInt = 20;
				int nByte = 10;
				cv::Mat f = cv::Mat::zeros(nInt, 1, CV_32FC1);
				std::memcpy(f.data, res.data(), nInt * sizeof(float));
				int nSubStr = nInt * 4 + nByte; //문자열의 길이를 제거한 것. 11*4+3(bool)
				auto vstr = split(res.substr(nSubStr, res.size() - nSubStr), ',');
				std::string user = vstr[0];
				const std::string mapName = vstr[1];

				int nbFlagIdx = nInt * 4;
				bool bMapping = (int)res[nbFlagIdx] ? true : false;
				bool bTracking = (int)res[nbFlagIdx + 1] ? true : false;
				bool bIMU = (int)res[nbFlagIdx + 2] ? true : false;
				bool bResetVO = (int)res[nbFlagIdx + 3] ? true : false;
				bool bPlaneGBA = (int)res[nbFlagIdx + 4] ? true : false;
				bool bEdgeBase = (int)res[nbFlagIdx + 5] ? true : false;
				bool bKFMethod = (int)res[nbFlagIdx + 6] ? true : false;
				bool bCommuTest = (int)res[nbFlagIdx + 7] ? true : false;	//그리드와 키프레임
				bool bSave = (int)res[nbFlagIdx + 8] ? true : false; //다이렉트와 인다이렉트
				bool bPLP = (int)res[nbFlagIdx + 9] ? true : false;
				std::vector<bool> vbFlags(10, false);
				vbFlags[0] = bMapping;
				vbFlags[1] = bTracking;
				vbFlags[2] = bIMU;
				vbFlags[3] = bResetVO;
				vbFlags[4] = bPlaneGBA;
				vbFlags[5] = bEdgeBase;
				vbFlags[6] = bCommuTest;
				//vbFlags[7] = bContentSyncTest;
				vbFlags[8] = bSave;
				vbFlags[9] = bPLP; //PLP 최적화

				if (!GSSLAM->CheckMap(mapName)) {
					std::cout << "Create Map : " << mapName << std::endl;
					GSSLAM->CreateMap(mapName, (int)f.at<float>(11));
					std::cout << "Map Initialization Success : " << mapName << std::endl;
					GSSLAM->InitVisualizer(user, mapName, (int)f.at<float>(0), (int)f.at<float>(1), bSaveVisualization, nIncForVis);
					//GSSLAM->GetMap(mapName)->mpVisualizer->SetScale(400);
					//SLAM->mpVisualizer->SetScale(400);
					std::cout << "Visualizer Initialization Success" << std::endl;
				}

				if (!GSSLAM->CheckUser(user)) {
					std::cout << "Create User" << std::endl;
					GSSLAM->CreateUser(user, mapName, f, vbFlags);
					if (keyword == "GSDeviceConnect")
						GSSLAM->SetUserVisID(GSSLAM->GetUser(user));
					std::cout << "User " << user << " Created" << std::endl;
					//오브젝트 슬램 확장
					//auto p1 = GSSLAM->GetUser(user);
					//auto p2 = new ObjectSLAM::ObjectDevice();
					//ObjSystem->MapObjectDevices.Update(p1, p2);
				}

			}
			////data processing
			//delete mpAPI;
		}

	}

	return 0;
}

