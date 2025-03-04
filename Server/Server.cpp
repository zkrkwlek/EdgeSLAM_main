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
#include <SemanticLabel.h>
#include <rapidjson\document.h>
#include <SLAM.h>
#include <Map.h>
#include <Frame.h>
#include <ObjectFrame.h>
#include <Tracker.h>
#include <LocalMapper.h>
#include <User.h>
#include <Camera.h>
#include <Optimizer.h>
#include <SearchPoints.h>
#include <SemanticProcessor.h>
#include <GridProcessor.h>
#include <DepthProcessor.h>
#include <DynamicTrackingProcessor.h>
#include <PlaneEstimator.h>

#include <ContentProcessor.h>
#include <MarkerProcessor.h>
#include <CoordinateIntegration.h>
#include <PlanarOptimizer.h>
#include <Node.h>
#include <GraphNode.h>
#include <Confidence.h>
#include <LabelInfo.h>
#include <ConcurrentMap.h>

////BASE SLAM
#include <BaseSystem.h>
#include <BaseDevice.h>
#include <AbstractMap.h>

//line
#include <LineProcessor.h>
#include <LineWorldMap.h>
#include <LineFrame.h>
#include <MapLine.h>
#include <PluckerCoordinate.h>

#include <ThreadPool.h>
#include <WebAPI.h>
#include <FeatureDetector.h>
#include <FeatureTracker.h>
//#include <Frame.h>
//#include <ORBDetector.h>

#include <opencv2/aruco.hpp>

//#include <CovisibilityGraph.h>
//#include "CovisibilityGraph.cpp"
#include <MapLine.h>

//#include <StructSLAM.h>
//#include <SimpleFrame.h>
//#include <StructOptimizer.h>
//#include <StructGraph.h>
//#include <StructType.h>
//#include <StructMapper.h>
//#include <StructTracker.h>
//#include <StructMatcher.cc>
//#include <SimpleRecon.h>
//#include <SimpleRecon.hpp>
//#include <StructMap.h>

//ObjectSLAM
#include <ObjectDevice.h>
#include <BoundingBox.h>
#include <SegInstance.h>
#include <OrientedBoundingBox.h>
#include <BoxFrame.h>
#include <GlobalInstance.h>
#include <FrameInstance.h>
#include <ObjectSLAM.h>
#include <ObjectMap.h>
#include <ObjectPoint.h>
#include <ObjectInitializer.h>
#include <ObjectMatcher.h>
#include <ObjectMapper.h> 
#include <InstanceLinker.h>
#include <ObjectMatchingInfo.h>

#include <Gaussian/GaussianObject.h>
#include <Gaussian/GaussianMapManager.h>
#include <Gaussian/Visualizer.h>
#include <Gaussian/Optimization/ObjectOptimizer.h>
#include <AssociationManager.h>

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
	if(nCPUCount > 300 && nCPUCount % 10 == 0)
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
std::string volatencyPath;
bool bSaveTrajectory = false;
std::string trajectoryPath;
int nNumDevice = 1;
bool bSaveSegLatency = false;
std::string deepkey1 = "a";
std::string deepkey2 = "b";
std::string deepkey3 = "c";

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
			mpAPI->Send(ss.str(),"");
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

		/*{
			std::stringstream ss;
			ss << "{\"type1\":" << "\"disconnect\"" << ",\"type2\":" << "\"all\"" << ",\"src\":" << "\"SLAMServer\"" << ",\"keyword\":\"Image\"" << "}";
			auto buf = ss.str().c_str();
			int retval;
			retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
		}
		{
			std::stringstream ss;
			ss << "{\"type1\":" << "\"disconnect\"" << ",\"type2\":" << "\"all\"" << ",\"src\":" << "\"SLAMServer\"" << ",\"keyword\":\"DeviceConnect\"" << "}";
			auto buf = ss.str().c_str();
			int retval;
			retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
		}
		{
			std::stringstream ss;
			ss << "{\"type1\":" << "\"disconnect\"" << ",\"type2\":" << "\"all\"" << ",\"src\":" << "\"SLAMServer\"" << ",\"keyword\":\"DeviceDisconnect\"" << "}";
			auto buf = ss.str().c_str();
			int retval;
			retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
		}
		{
			std::stringstream ss;
			ss << "{\"type1\":" << "\"disconnect\"" << ",\"type2\":" << "\"all\"" << ",\"src\":" << "\"SLAMServer\"" << ",\"keyword\":\"Segmentation\"" << "}";
			auto buf = ss.str().c_str();
			int retval;
			retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
		}
		{
			std::stringstream ss;
			ss << "{\"type1\":" << "\"disconnect\"" << ",\"type2\":" << "\"all\"" << ",\"src\":" << "\"SLAMServer\"" << ",\"keyword\":\"DepthEstimation\"" << "}";
			auto buf = ss.str().c_str();
			int retval;
			retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
		}*/
		WebAPI::Close();
		exit(1);
	}
	return true;
}

//parallel

float BasicSimulatorFast(int port, std::string method, const unsigned char* data, int size, std::chrono::high_resolution_clock::time_point start) {
	WebAPI API("143.248.6.143", port);
	auto res1 = API.Send(method, data, size); 
	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	float t_test1 = du_test1 / 1000.0;
	return t_test1;
}

void BasicFastSimulator(ThreadPool::ThreadPool* POOL, int nServer){
	int datasize = 40000;
	cv::Mat data = cv::Mat::ones(datasize, 1, CV_8UC1);
	int N = 0;
	float sum1 = 0.0;
	while (true) {
		
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		std::future<float> a1, a2, a3, a4, a5, a6, a7, a8;
		if (nServer >= 1)
			a1 = POOL->EnqueueJob(BasicSimulatorFast, 45000, "/offloadingtasktest", (const unsigned char*)data.data, datasize, start);
		if (nServer >= 2)
			a2 = POOL->EnqueueJob(BasicSimulatorFast, 45001, "/offloadingtasktest", (const unsigned char*)data.data, datasize, start);
		if (nServer >= 3)
			a3 = POOL->EnqueueJob(BasicSimulatorFast, 45002, "/offloadingtasktest", (const unsigned char*)data.data, datasize, start);
		if (nServer >= 4)
			a4 = POOL->EnqueueJob(BasicSimulatorFast, 45003, "/offloadingtasktest", (const unsigned char*)data.data, datasize, start);
		if (nServer >= 5)
			a5 = POOL->EnqueueJob(BasicSimulatorFast, 45004, "/offloadingtasktest", (const unsigned char*)data.data, datasize, start);
		if (nServer >= 6)
			a6 = POOL->EnqueueJob(BasicSimulatorFast, 45005, "/offloadingtasktest", (const unsigned char*)data.data, datasize, start);
		if (nServer >= 7)
			a7 = POOL->EnqueueJob(BasicSimulatorFast, 45006, "/offloadingtasktest", (const unsigned char*)data.data, datasize, start);
		if (nServer >= 8)
			a8 = POOL->EnqueueJob(BasicSimulatorFast, 45007, "/offloadingtasktest", (const unsigned char*)data.data, datasize, start);
		if(nServer >= 1){
			float t = a1.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		if (nServer >= 2){
			float t = a2.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		if (nServer >= 3){
			float t = a3.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		if (nServer >= 4){
			float t = a4.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		if (nServer >= 5){
			float t = a5.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		if (nServer >= 6) {
			float t = a6.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		if (nServer >= 7) {
			float t = a7.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		if (nServer >= 8) {
			float t = a8.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		std::cout << "basic fast parallel test = " <<nServer<<" || "<< N << "=" << sum1 << " = " << sum1 / N << std::endl;// << " " << sum2 << " " << sum3 << " " << sum4 << std::endl;
		PrintCPU();
		cv::waitKey(100);
	}
}

void BasicSimulator(int nServer) {
	//std::cout << "Simulator test1 " << std::endl;
	int datasize = 40000;
	cv::Mat data = cv::Mat::ones(datasize, 1, CV_8UC1);
	WebAPI API1("143.248.6.143", 45000);
	WebAPI API2("143.248.6.143", 45001);
	WebAPI API3("143.248.6.143", 45002);
	WebAPI API4("143.248.6.143", 45003);
	WebAPI API5("143.248.6.143", 45004);
	WebAPI API6("143.248.6.143", 45005);
	WebAPI API7("143.248.6.143", 45006);
	WebAPI API8("143.248.6.143", 45007);

	float sum = 0.0;
	float n = 0;

	while (true) {

		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		if (nServer >= 1) {
			auto res1 = API1.Send("/offloadingtasktest", (const unsigned char*)data.data, datasize);
			std::chrono::high_resolution_clock::time_point end1 = std::chrono::high_resolution_clock::now();
			auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start).count();
			float t_test1 = du_test1 / 1000.0;
			sum += t_test1;
			n++;
		}
		if (nServer >= 2) {
			auto res2 = API2.Send("/offloadingtasktest", (const unsigned char*)data.data, datasize);
			std::chrono::high_resolution_clock::time_point end2 = std::chrono::high_resolution_clock::now();
			auto du_test2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start).count();
			float t_test2 = du_test2 / 1000.0;
			sum += t_test2;
			n++;
		}
		if (nServer >= 3) {
			auto res3 = API3.Send("/offloadingtasktest", (const unsigned char*)data.data, datasize);
			std::chrono::high_resolution_clock::time_point end3 = std::chrono::high_resolution_clock::now();
			auto du_test3 = std::chrono::duration_cast<std::chrono::milliseconds>(end3 - start).count();
			float t_test3 = du_test3 / 1000.0;
			sum += t_test3;
			n++;
		}
		if (nServer >= 4) {
			auto res4 = API4.Send("/offloadingtasktest", (const unsigned char*)data.data, datasize);
			std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
			auto du_test4 = std::chrono::duration_cast<std::chrono::milliseconds>(end4 - start).count();
			float t_test4 = du_test4 / 1000.0;
			sum += t_test4;
			n++;
		}
		if (nServer >= 5) {
			auto res4 = API5.Send("/offloadingtasktest", (const unsigned char*)data.data, datasize);
			std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
			auto du_test4 = std::chrono::duration_cast<std::chrono::milliseconds>(end4 - start).count();
			float t_test4 = du_test4 / 1000.0;
			sum += t_test4;
			n++;
		}
		if (nServer >= 6) {
			auto res4 = API6.Send("/offloadingtasktest", (const unsigned char*)data.data, datasize);
			std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
			auto du_test4 = std::chrono::duration_cast<std::chrono::milliseconds>(end4 - start).count();
			float t_test4 = du_test4 / 1000.0;
			sum += t_test4;
			n++;
		}
		if (nServer >= 7) {
			auto res4 = API7.Send("/offloadingtasktest", (const unsigned char*)data.data, datasize);
			std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
			auto du_test4 = std::chrono::duration_cast<std::chrono::milliseconds>(end4 - start).count();
			float t_test4 = du_test4 / 1000.0;
			sum += t_test4;
			n++;
		}
		if (nServer >= 8) {
			auto res4 = API8.Send("/offloadingtasktest", (const unsigned char*)data.data, datasize);
			std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
			auto du_test4 = std::chrono::duration_cast<std::chrono::milliseconds>(end4 - start).count();
			float t_test4 = du_test4 / 1000.0;
			sum += t_test4;
			n++;
		}
		
		std::cout << "basic parallell test1 = " << n << " = " << sum << "=" << sum / n << std::endl;// << "+" << sums[1] << "+" << sums[2] << "+" << sums[3] << "+" << sums[4] << std::endl;
		PrintCPU();
		cv::waitKey(100);
	}
	//std::thread t1(SimulatorThread1);
}

//sequantial
void BasicFastSeqSimulator(ThreadPool::ThreadPool* POOL, int Nserver) {
	int datasize = 40000;
	cv::Mat data = cv::Mat::ones(datasize, 1, CV_8UC1);
	int N = 0;
	float sum1 = 0.0;
	float sum2 = 0.0;
	float sum3 = 0.0;
	float sum4 = 0.0;
	while (true) {
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		std::future<float> a1, a2, a3, a4, a5, a6, a7, a8;

		auto aEnd = POOL->EnqueueJob(BasicSimulatorFast, 45003, "/offloadingSequantialTaskTest", (const unsigned char*)data.data, datasize, start);
		if (Nserver >= 1)
			a1 = POOL->EnqueueJob(BasicSimulatorFast, 45000, "/offloadingSequantialTaskTest", (const unsigned char*)data.data, datasize, start);
		if (Nserver >= 2)
			a2 = POOL->EnqueueJob(BasicSimulatorFast, 45001, "/offloadingSequantialTaskTest", (const unsigned char*)data.data, datasize, start);
		if (Nserver >= 3)
			a3 = POOL->EnqueueJob(BasicSimulatorFast, 45002, "/offloadingSequantialTaskTest", (const unsigned char*)data.data, datasize, start);
		if (Nserver >= 4)
			a4 = POOL->EnqueueJob(BasicSimulatorFast, 45004, "/offloadingSequantialTaskTest", (const unsigned char*)data.data, datasize, start);
		
		if (Nserver >= 1) {
			float t = a1.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		if (Nserver >= 2) {
			float t = a2.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		if (Nserver >= 3) {
			float t = a3.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}
		if (Nserver >= 4) {
			float t = a4.get();
			if (t <= 0.0)
				continue;
			sum1 += t;
			N++;
		}

		std::cout << "basic fast seq test = " << Nserver << " || " << N << "=" << sum1 << " = " << sum1 / N << std::endl;

		cv::waitKey(10);
	}
}

void BasicSimulator2() {
	
	int datasize = 40000;
	cv::Mat data = cv::Mat::ones(datasize, 1, CV_8UC1);
	WebAPI API1("143.248.6.143", 45000);
	WebAPI API2("143.248.6.143", 45001);
	WebAPI API3("143.248.6.143", 45002);
	WebAPI API4("143.248.6.143", 45003);

	std::vector<float> sums(5, 0.0);
	float n = 0;

	while (true) {
		
		std::chrono::high_resolution_clock::time_point start4 = std::chrono::high_resolution_clock::now();
		auto res4 = API4.Send("/offloadingSequantialTaskTest", (const unsigned char*)data.data, datasize);
		std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
		
		std::chrono::high_resolution_clock::time_point start1 = std::chrono::high_resolution_clock::now();
		auto res1 = API1.Send("/offloadingSequantialTaskTest", (const unsigned char*)data.data, datasize);
		std::chrono::high_resolution_clock::time_point end1 = std::chrono::high_resolution_clock::now();

		std::chrono::high_resolution_clock::time_point start2 = std::chrono::high_resolution_clock::now();
		auto res2 = API2.Send("/offloadingSequantialTaskTest", (const unsigned char*)data.data, datasize);
		std::chrono::high_resolution_clock::time_point end2 = std::chrono::high_resolution_clock::now();

		std::chrono::high_resolution_clock::time_point start3 = std::chrono::high_resolution_clock::now();
		auto res3 = API3.Send("/offloadingSequantialTaskTest", (const unsigned char*)data.data, datasize);
		std::chrono::high_resolution_clock::time_point end3 = std::chrono::high_resolution_clock::now();

		
		
		auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count();
		float t_test1 = du_test1 / 1000.0;
		auto du_test2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count();
		float t_test2 = du_test2 / 1000.0;
		auto du_test3 = std::chrono::duration_cast<std::chrono::milliseconds>(end3 - start3).count();
		float t_test3 = du_test3 / 1000.0;
		auto du_test4 = std::chrono::duration_cast<std::chrono::milliseconds>(end4 - start4).count();
		float t_test4 = du_test4 / 1000.0;

		int nidx = 0;
		sums[nidx++] += t_test1;
		sums[nidx++] += t_test2;
		sums[nidx++] += t_test3;
		//sums[nidx++] += t_test4;

		n++;
		std::cout << "basic seq test = " <<n<<" = "<< sums[0] / n << " " << sums[1] / n << " " << sums[2] / n << " " << sums[3] / n << " " << sums[4] / n << std::endl;
		//std::cout << res << std::endl;
		cv::waitKey(100);
	}
	//std::thread t1(SimulatorThread1);
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

void SimulatorSendConnect(std::string source, std::string sendkey, int capacity) {
	{
		WebAPI API(ip, port);
		std::stringstream ssa;
		ssa << "{\"src\":\"" << source << "\"," << "\"keyword\":\"" << sendkey << "\",\"type1\":\"device\",\"type2\":\"" << "NONE" << "\"" << ",\"capacity\":" << 1 << "}"; //test\"}";
		std::cout << ssa.str() << std::endl;
		API.Send("/Connect", ssa.str());
	}
}

void SimulatorReceiveConnect(std::string source, std::string receivekey, std::string key2) {
	{
		sockaddr_in clientaddr;
		int addrlen;
		char rbuf[1024];
		std::stringstream ss;
		ss << "{\"type1\":" << "\"connect\"" << ",\"type2\":" << "\"single\"" << ",\"src\":" << "\"" << source << "\"" << ",\"keyword\":\"" << receivekey << "\"" << "}";
		std::string temp = ss.str();
		auto buf = temp.c_str();
		int retval;
		retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));

		LatencyMapSum.Update(key2, 0.0);
		LatencyMapCount.Update(key2, 0);

	}
}

void Evaluation(std::string keyword, std::string key2, int did) {
	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	auto start = LatencyMapStart.Get(did);
	auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	float t_test1 = du_test1 / 1000.0;
	
	float sum = LatencyMapSum.Get(key2);
	int N = LatencyMapCount.Get(key2);

	sum = sum + t_test1;
	N++;

	std::cout << keyword << " = " <<did<<" || "<< N << " " << sum << " " << sum / N << std::endl;

	LatencyMapSum.Update(key2, sum);
	LatencyMapCount.Update(key2, N);

}

void TestDataGenerator(std::string keyword, std::string src) {
	WebAPI API(ip,port);
	float n = 0;
	int nID = 0;
	int datasize = 40000;
	cv::Mat data = cv::Mat::ones(datasize, 1, CV_8UC1);
	while (true) {
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		{
			std::stringstream ss;
			ss << "/Upload?keyword=" << keyword << "&id=" << ++nID << "&src=" << src;
			auto res = API.Send(ss.str(), data.data, sizeof(unsigned char)*data.rows);
			LatencyMapStart.Update(nID, start);
		}
		cv::waitKey(100);
	}

}

void SimulatorThread1() {
	//데이터를 생성하는 쓰레드
	WebAPI API("143.248.6.143", 35005);
	std::string SOURCE = "Simulator1";
	std::string sKeyword1 = "TestParallelData";
	std::string sKeyword2 = "TestSequantialData";
	std::string sPair = "NONE";
	
	{
		//패러랠 데이터
		{
			std::stringstream ssa;
			ssa << "{\"src\":\"" << SOURCE << "\"," << "\"keyword\":\"" << sKeyword1 << "\",\"type1\":\"device\",\"type2\":\"" << sPair << "\"" << ",\"capacity\":" << 1 << "}"; //test\"}";
			std::cout << ssa.str() << std::endl;
			API.Send("/Connect", ssa.str());
		}
		{
			std::stringstream ssa;
			ssa << "{\"src\":\"" << SOURCE << "\"," << "\"keyword\":\"" << sKeyword2 << "\",\"type1\":\"device\",\"type2\":\"" << sPair << "\"" << ",\"capacity\":" << 1 << "}"; //test\"}";
			std::cout << ssa.str() << std::endl;
			API.Send("/Connect", ssa.str());
		}
		////시퀀셜 데이터 생성
	}
	


	{
		//ResParallelTask1,ResSequantialTask1
		//UDP 연결
		sockaddr_in clientaddr;
		int addrlen;
		char rbuf[1024];

		////parallel
		{
			std::stringstream ss;
			ss << "{\"type1\":" << "\"connect\"" << ",\"type2\":" << "\"single\"" << ",\"src\":" << "\"Simulator1\"" << ",\"keyword\":\"" << "ResParallelTask1" << "\"" << "}";
			std::string temp = ss.str();
			auto buf = temp.c_str();
			int retval;
			retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
		}
		////sequantial
		{
			std::stringstream ss;
			ss << "{\"type1\":" << "\"connect\"" << ",\"type2\":" << "\"single\"" << ",\"src\":" << "\"Simulator1\"" << ",\"keyword\":\"" << "ResSequantialTask1" << "\"" << "}";
			std::string temp = ss.str();
			auto buf = temp.c_str();
			int retval;
			retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
		}
	}



	int nID = 0;
	std::vector<float> sums(5, 0.0);
	float n = 0;
	cv::Mat data = cv::Mat::ones(4000, 1, CV_8UC1);
	while (true) {
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		float t1, t2, t3, t4, t5, t6, t7, t8;
		{
			
			std::stringstream ss;
			ss << "/Upload?keyword="<< sKeyword1 <<"&id=" << ++nID << "&src="<<SOURCE;
			auto res = API.Send(ss.str(), data.data, sizeof(unsigned char)*data.rows);
			LatencyMapStart.Update(nID, start);
		}

		{
			std::stringstream ss;
			ss << "/Upload?keyword=" << sKeyword2 << "&id=" << ++nID << "&src=" << SOURCE;
			auto res = API.Send(ss.str(), data.data, sizeof(unsigned char)*data.rows);
			LatencyMapStart.Update(nID, start);
		}

		/*{

			std::stringstream ss;
			ss << "/Store?keyword=" << sKeyword1 << "&id=" << ++nID << "&src=" << SOURCE;
			auto res = API.Send(ss.str(), data.data, sizeof(unsigned char)*data.rows);
			LatencyMapStart.Update(nID, start);
		} 
		{

			std::stringstream ss;
			ss << "/Store?keyword=" << sKeyword1 << "&id=" << ++nID << "&src=" << SOURCE;
			auto res = API.Send(ss.str(), data.data, sizeof(unsigned char)*data.rows);
			LatencyMapStart.Update(nID, start);
		}
		{

			std::stringstream ss;
			ss << "/Store?keyword=" << sKeyword1 << "&id=" << ++nID << "&src=" << SOURCE;
			auto res = API.Send(ss.str(), data.data, sizeof(unsigned char)*data.rows);
			LatencyMapStart.Update(nID, start);
		}
		{

			std::stringstream ss;
			ss << "/Store?keyword=" << sKeyword1 << "&id=" << ++nID << "&src=" << SOURCE;
			auto res = API.Send(ss.str(), data.data, sizeof(unsigned char)*data.rows);
			LatencyMapStart.Update(nID, start);
		}*/



		cv::waitKey(500);
	}
}
//데이터 리시브
void SimulatorThread2() {

	

	//while (true) {

	//	addrlen = sizeof(clientaddr);
	//	int retval = recvfrom(sock, rbuf, 1024, 0, (sockaddr*)&clientaddr, &addrlen);
	//	rbuf[retval] = '\0';
	//	rapidjson::document document;
	//	//if (document.parse(rbuf).hasparseerror()) {
	//	//	std::cout << "json parsing error" << std::endl;
	//	//	continue;
	//	//}
	//	if (document["keyword"].isstring() && document["id"].isint()) {
	//		int id = document["id"].getint();
	//		double ts = std::stod(document["ts"].getstring());
	//		std::string keyword = document["keyword"].getstring();
	//		std::string src = document["src"].getstring();
	//		std::string type2 = document["type2"].getstring();

	//		//////load data with keyword
	//		int id2 = -1;
	//		std::stringstream ss;
	//		if (document.hasmember("id2")) {
	//			id2 = document["id2"].getint();
	//			ss << "/load?keyword=" << keyword << "&id=" << id << "&id2=" << id2 << "&src=" << src;
	//		}
	//		else
	//			ss << "/load?keyword=" << keyword << "&id=" << id << "&src=" << src;

	//		////receive data
	//		webapi* mpapi = new webapi("143.248.6.143", 35005);

	//		std::cout << "b" << std::endl;
	//	}
	//}
}

void SendWiseUITest(std::string src, int id, cv::Mat data, double ts) {
	{
		WebAPI API("143.248.6.143", 35005);
		std::stringstream ss;
		ss << "/Upload?keyword=WiseUITest&id=" << id << "&src=" << src << "&ts=" << std::fixed << std::setprecision(6) << ts << "&type2=" << src;
		auto res = API.Send(ss.str(), data.data, data.rows * sizeof(float));
	}
}

void DeviceCoordAlignTest(ThreadPool::ThreadPool* POOL, EdgeSLAM::SLAM* SLAM, std::string src, std::string url, int id, double ts) {
	if(!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++;
	
	std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();

	WebAPI API(ip, port);
	auto res = API.Send(url, "");
	int n2 = res.size();
	cv::Mat F = cv::Mat::eye(4, 4, CV_32FC1);
	F.at<float>(1, 1) = -1.0;
	cv::Mat temp = cv::Mat(4, 3, CV_32FC1, (void*)res.data());
	cv::Mat R = temp.rowRange(0, 3);
	cv::Mat t = temp.row(3).t();
	cv::Mat T = cv::Mat::eye(4, 4, CV_32FC1);
	R.copyTo(T.rowRange(0, 3).colRange(0, 3));
	t.copyTo(T.rowRange(0, 3).col(3));
	T = F * T * F;
	User->MapDeviceTrajectories.Update(id, T);

	auto map1 = User->MapServerTrajectories.Get();
	auto map2 = User->MapDeviceTrajectories.Get();

	std::vector<int> vec1,vec2,res_inter;
	for (auto iter = map1.begin(), iend = map1.end(); iter != iend; iter++) {
		vec1.push_back(iter->first);
	}
	std::unordered_set<int> m(vec1.begin(), vec1.end());
	for (auto iter = map2.begin(), iend = map2.end(); iter != iend; iter++) {
		vec2.push_back(iter->first);
	}
	for (auto a : vec2) {
		if (m.count(a)) {
			res_inter.push_back(a);
			m.erase(a);
		}
	}

	//gt = server = map1
	//data = device = map2
	//s = device->server
	int Nframe = 30;
	if (res_inter.size() > Nframe) {
		cv::Mat T1 = cv::Mat::zeros(0, 3, CV_32FC1);
		cv::Mat T2 = cv::Mat::zeros(0, 3, CV_32FC1);
		for (int i = res_inter.size() - 1, iend = res_inter.size() - Nframe; i > iend; i--) {
		//for (int i = res_inter.size() - 1, iend = 10; i > iend; i--) {
			int id = res_inter[i];
			cv::Mat O1 = map1[id].rowRange(0, 3).col(3).t();
			cv::Mat O2 = map2[id].rowRange(0, 3).col(3).t();
			T1.push_back(O1);
			T2.push_back(O2);
		}

		T1 = T1.t();
		T2 = T2.t();

		//평균
		cv::Mat mean1, mean2;
		cv::Mat temp2 = cv::Mat::ones(1, T1.cols, CV_32FC1);
		cv::reduce(T1, mean1, 1, cv::REDUCE_AVG, CV_32F);
		cv::reduce(T2, mean2, 1, cv::REDUCE_AVG, CV_32F);
		cv::Mat amean1 = mean1 * temp2;
		cv::Mat amean2 = mean2 * temp2;
		
		cv::Mat T1_zero = T1 - amean1;
		cv::Mat T2_zero = T2 - amean2;
		
		cv::Mat W = cv::Mat::zeros(3, 3, CV_32FC1);
		for (int i = 0; i < T1_zero.cols; i++) {
			cv::Mat t1 = T1_zero.col(i);
			cv::Mat t2 = T2_zero.col(i);
			W += (t2 * t1.t());
		}
		  
		cv::Mat w, u, vt;
		cv::SVD::compute(W.t(), w, u, vt);
		cv::Mat I = cv::Mat::eye(3,3, w.type());
		
		if (cv::determinant(u) * cv::determinant(vt) < 0)
			I.at<float>(2, 2) = -1;
		cv::Mat R = u * I * vt;
		cv::Mat A = R * T2_zero;
		A = A.reshape(0, 1).t();
		cv::Mat b = T1_zero.reshape(0,1).t();
		cv::Mat x;
		cv::solve(A, b, x, cv::DECOMP_SVD);
		float s = x.at<float>(0);
		
		cv::Mat t = mean1 - s * R * mean2;
		cv::Mat at = t * temp2;
		cv::Mat aligned = R * (s * T2) + at;
		cv::Mat errMat = aligned - T1;

		cv::Mat AA;
		cv::reduce(errMat.mul(errMat),AA,0,cv::REDUCE_SUM, CV_32F);
		cv::sqrt(AA, AA);
		float err = sqrt(AA.dot(AA))/T1.cols;
		/*float max_err = 0.0;
		for (int i = 0; i < AA.cols; i++) {
			float val = AA.at<float>(i);
			float erra = sqrt(val * val);
			if (erra > max_err) {
				max_err = erra;
			}
		}*/

		cv::Mat cov, mu;
		cv::calcCovarMatrix(errMat, cov, mu, cv::COVAR_NORMAL | cv::COVAR_COLS,CV_32F);
		cv::Mat info = cov.inv();
		float max_val = 0.0;
		for (int i = 0; i < errMat.cols; i++) {
			cv::Mat var = errMat.col(i);
			float chi2 = var.dot(info * var);
			if (max_val < chi2)
				max_val = chi2;
			if (chi2 > 1) {
				std::cout << "chi2 test = " << chi2 << std::endl;
			}
		}

		//A = R*t2_zero
		//b = t1_zeros
		std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();
		auto du_down = std::chrono::duration_cast<std::chrono::milliseconds>(t_down_end - t_down_start).count();
		//std::cout << "err = " << err <<" "<<max_val << " " << du_down << "::" << s << std::endl;

		User->ScaleFactor = 1.0 / s;
		{
			cv::Mat invR = R.t();
			cv::Mat invT = -invR * t;
			invR.copyTo(User->Tcoord.rowRange(0, 3).colRange(0, 3));
			invT.copyTo(User->Tcoord.rowRange(0, 3).col(3));
			
			//시각화 테스트
		}

		User->MapAlignedDeviceTrajectories.Update(id, aligned.col(0));
	}
	
	User->mnUsed--;
	return;
	float avg1 = 0.0;
	float avg2 = 0.0;
	float avg3 = 0.0;
	int nTemp = 0;
	for (int i = 1, iend = res_inter.size(); i < iend; i++) {
		int id1 = res_inter[i - 1];
		int id2 = res_inter[i];
		if (id2 - id1 == User->mnSkip) {
			nTemp++;
			cv::Mat Os1 = map1[id1].rowRange(0,3).col(3);
			cv::Mat Os2 = map1[id2].rowRange(0, 3).col(3);

			cv::Mat Od1 = map2[id1].rowRange(0, 3).col(3);
			cv::Mat Od2 = map2[id2].rowRange(0, 3).col(3);

			cv::Mat diff1 = Os2 - Os1;
			cv::Mat diff2 = Od2 - Od1;
			float d1 = sqrt(diff1.dot(diff1));
			float d2 = sqrt(diff2.dot(diff2));
			avg1 += d1;
			avg2 += d2;
			avg3 += (d2 / d1);
		}
	}
	/*auto vec1 = Utils::GetIntersectionMap(map1, map2);*/
	if (nTemp > 10) {
		avg1 /= nTemp;
		avg2 /= nTemp;
		avg3 /= nTemp;
		User->ScaleFactor = avg3;
		std::cout << "intersection = " << res_inter.size() << " " << nTemp << "==" << avg3 << " " << avg2 / avg1 << std::endl;
	}
	
	User->mnUsed--;
}

void NetworkDownloadSimulation(EdgeSLAM::SLAM* SLAM, std::string url,int id, int quality) {
	std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
	WebAPI API(ip, port);
	auto res = API.Send(url, "");
	std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();
	auto du_down = std::chrono::duration_cast<std::chrono::milliseconds>(t_down_end - t_down_start).count();
	std::stringstream ss;
	ss << nNetworkSimul << "," << "image," << id << "," <<quality<<"," << res.size() << "," << du_down << std::endl;
	SLAM->EvaluationLatency.push_back(ss.str());
}
void NetworkUploadSimulation(EdgeSLAM::SLAM* SLAM, std::string src, std::string keyword, int id, const cv::Mat& data, std::chrono::high_resolution_clock::time_point t_first) {
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	Utils::SendData(keyword, src, data, id);
	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
	auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_first).count();
	std::stringstream ss;
	ss << nNetworkSimul << "," << src << "," << id << "," <<"0"<<"," << data.rows * 4 << "," << duration <<","<< duration2 << std::endl;
	SLAM->EvaluationLatency.push_back(ss.str());
}

void SimTrack(ThreadPool::ThreadPool* POOL, EdgeSLAM::SLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {
	std::chrono::high_resolution_clock::time_point t_parsing_end = std::chrono::high_resolution_clock::now();;
	if (!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++;
	//image download

	std::cout << "start" << std::endl;

	std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
	WebAPI API(ip, port);
	auto res = API.Send(url, "");
	std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();

	int n2 = res.size();
	cv::Mat temp = cv::Mat(n2, 1, CV_8UC1, (void*)res.data());
	cv::Mat img = cv::imdecode(temp, cv::IMREAD_COLOR);

	cv::Mat K = User->GetCameraMatrix();
	EdgeSLAM::Frame* frame = new EdgeSLAM::Frame(img, User->mpCamera, id, frame_ts);
	frame->mnShared++;
	//auto frame = std::make_shared<EdgeSLAM::Frame>(img, User->mpCamera, id, received_ts);
	int nVisID = User->GetVisID() + 4;
	std::string mapName = User->mapName;

	std::chrono::high_resolution_clock::time_point t_track_start = std::chrono::high_resolution_clock::now();
	//EdgeSLAM::Tracker::TrackSimulation(POOL, SLAM, id, src, frame, img, frame_ts);
	std::chrono::high_resolution_clock::time_point t_track_end = std::chrono::high_resolution_clock::now();

	auto trackStat = User->GetState();
	bool bTrackSuccess = trackStat == EdgeSLAM::UserState::Success && frame->mvpMapPoints.size() > 0;
	if (bTrackSuccess) {
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		long long ts = start.time_since_epoch().count();

		std::string keyword = "ReferenceFrame";
		cv::Mat totaldata = cv::Mat::zeros(0, 0, CV_32FC1);

		if (User->mbBaseLocalMap)
		{
			cv::Mat localmapdata;
			EdgeSLAM::Tracker::GenerateLocalMapForSync(SLAM, User, localmapdata, id, ts);
			totaldata.push_back(localmapdata);
			//keyword = "UpdatedLocalMap";
		}

		cv::Mat kfdata, contentdata, planedata;
		EdgeSLAM::Tracker::GenerateReferenceFrameForSync(SLAM, User, frame, kfdata, id, ts);
		SemanticSLAM::ContentProcessor::GenerateGraphDataForSync(SLAM, User, contentdata, id, ts);
		SemanticSLAM::PlaneEstimator::GeneratePlaneDataForSync(SLAM, User, planedata, id, ts);

		totaldata.push_back(kfdata);
		totaldata.push_back(contentdata);
		totaldata.push_back(planedata);
		long long ts_end_graph = start.time_since_epoch().count();

		auto du_upload = Utils::SendData(keyword, src, totaldata, id, received_ts);

		if (bSaveLatency) {
			//auto du_parsing = std::chrono::duration_cast<std::chrono::milliseconds>(t_parsing_end - t_parsing_start).count();
			auto du_track = std::chrono::duration_cast<std::chrono::milliseconds>(t_track_end - t_track_start).count();
			auto du_download = std::chrono::duration_cast<std::chrono::milliseconds>(t_down_end - t_down_start).count();
			auto du_graph = ts_end_graph - ts;
			int N = SLAM->GetConnectedDevice();
			{
				std::stringstream ss;
				// "num,source,id,quality,download,processing,upload,size\n";
				ss << N << "," << src << "," << id << "," << User->mnQuality << "," << du_download << "," << du_track << "," << du_upload << "," << du_graph << "," << totaldata.rows * sizeof(float) << std::endl;
				SLAM->EvaluationLatency.push_back(ss.str());
			}
		}
		if (bSaveVOLatency)
		{
			int N = SLAM->GetConnectedDevice();
			std::stringstream ss;
			// "num,source,method,type,id,quality,size,latency\n";
			ss << N << "," << src << "," << "indirect,upload," << id<<"," << totaldata.rows * sizeof(float) << "," << du_upload<< std::endl;
			SLAM->EvaluationVirtualObjectLatency.push_back(ss.str());
		}

	}
	std::cout << "end" << std::endl;
	frame->mnShared--;
	User->mnUsed--;
}

//BaseSLAM
BaseSLAM::BaseSystem* BaseSLAMSystem = new BaseSLAM::BaseSystem();
//StructOptimization::StructSLAM* StructSLAMSystem = new StructOptimization::StructSLAM();
 
//test object optical flow
bool bPrev = false;
//ObjectSLAM::BoxFrame* pPrevBF = nullptr;
//cv::Mat prev, prevGray, prevDesc;
//std::vector<cv::Point2f> prevCorners, currCorners;

std::vector<cv::KeyPoint> prevKPs;
//test object optical flow

//정말 박스만 생성하고 넘기기
ObjectSLAM::ObjectSLAM* ObjSystem = nullptr;

ConcurrentMap<EdgeSLAM::KeyFrame*, ObjectSLAM::BoxFrame*> AllBoxFrames;

//prev와 curr의 instance를 비교
//seg와 비교 - 이건 항상
//sam과 비교 - 이건 미싱 오브젝트를 감지하였을 때

void AssociateMissingObject2(EdgeSLAM::SLAM* SLAM, ObjectSLAM::ObjectSLAM* ObjSLAM, const std::string& key, const int id, const std::string& mapName, const std::string& userName, ObjectSLAM::BoxFrame* pNewBF, ObjectSLAM::BoxFrame* pPrevBF, ObjectSLAM::InstanceMask* pPrevSegMask, ObjectSLAM::InstanceMask* pCurrSegMask, ObjectSLAM::InstanceMask* pRaft, bool bShow = true)
{
	std::string keyprocess = "AssociateMissingObject2 + "+key+" ===";
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

	ObjSystem->vecObjectAssoRes.push_back(keyprocess+" start");

	//flow로 변환 된 마스크와 현재 seg의 마스크와 iou 계산.
	//백그라운드에서 0과 매칭되면 미싱 인스턴스
	std::set<int> sAlreadyFrameMatch; //이미 매칭되었기에
	std::set<int> sAlreadyMapMatch, sAlreadyReqObject;

	//미싱된 오브젝트의 글로벌 아이디를 저장.
	auto pCurrSegInstance = pCurrSegMask->FrameInstances.Get();
	auto pPrevSegInstance = pPrevSegMask->FrameInstances.Get();

	//이미지 관련
	cv::Mat currImg = pNewBF->img.clone();
	int w = currImg.cols;
	int h = currImg.rows;
	int w2 = w - 5;
	int h2 = h - 5;
	const cv::Mat flow = pRaft->mask;
		
	std::map<int, ObjectSLAM::FrameInstance*> mapRaftInstance;
	std::map<int, ObjectSLAM::AssoMatchRes*> mapCurrAssores; //일단 성공한 assocation만 기록. 그런데 실패한 것도 기록할 수 있음.

	cv::Mat prevmask = cv::Mat::zeros(h, w, CV_8UC1);
	cv::Mat currmask = cv::Mat::zeros(h, w, CV_8UC1);
	//cv::Mat raftmask = cv::Mat::zeros(h, w, CV_8UC1);

	auto pKF = pNewBF->mpRefKF;

	int inc = 3;
	auto pPrevIns = pPrevSegMask->FrameInstances.Get();

	//라프트를 이용해 이전 프레임의 마스크 이동 + 추가로, 바운딩 박스와 센터까지 이동
	for (auto pair : pPrevIns)
	{
		int sid = pair.first;
		if (sid == 0)
			continue;
		auto p = pair.second;
		if (!p)
		{
			std::stringstream ss;
			ss << "prev::instance::error::" << pPrevSegMask->mnMaxId << " " << pPrevIns.size() << " " << pPrevSegMask->MapInstances.Size();
			ObjSLAM->vecObjectAssoRes.push_back(ss.str());
			continue;
		}
		//새로 테스트 하는 것
		auto pRaftIns = new ObjectSLAM::FrameInstance(pKF);
		if (ObjectSLAM::InstanceSim::ComputeRaftInstance(flow, p, pRaftIns)) {
			mapRaftInstance[sid] = pRaftIns;
		}
	} 

	//frame 인스턴스 매칭
	//이전 프레임의 인스턴스를 현재 프레임의 인스턴스와 매칭
	//현재 프레임의 인스턴스가 0(스태틱, 배경)과 매칭하면 인스턴스 추가
	//아니면 인스턴스 연결
	//배경과 매칭되는 경우에 인스턴스 마스크 추가를 요청함.
	// iou값이 일정값 이하인 경우 샘을 이용해 매칭
	for (auto pair1 : mapRaftInstance) {
		int id1 = pair1.first;
		if (id1 == 0)
			continue;

		ObjectSLAM::AssoMatchRes* assores = new ObjectSLAM::AssoMatchRes();
		assores->id1 = id1;

		const cv::Mat pmask = pair1.second->mask;
		float area1 = (float)cv::countNonZero(pmask);
		auto pPrevGlobal = pPrevSegMask->MapInstances.Get(id1);

		std::pair<int, float> bestFailMatch = std::make_pair(-1, 0.0);

		bool bres = false;

		for (auto pair2 : pCurrSegInstance) {
			int id2 = pair2.first;
			//현재 인스턴스 중에서 이미 매칭이 되었으면 패스
			if (sAlreadyFrameMatch.count(id2))
				continue;
			const cv::Mat mask = pair2.second->mask;

			cv::Mat overlap = pmask & mask;
			float nOverlap = (float)cv::countNonZero(overlap);

			//겹치는게 없으면 무시
			if (nOverlap == 0)
				continue;

			float iou = 0.0;
			float th = 0.5;
			if (id2 == 0) {
				iou = nOverlap / area1;
			}
			if (id2 > 0)
			{
				cv::Mat total = pmask | mask;
				float nUnion = (float)cv::countNonZero(total);
				iou = nOverlap / nUnion;
			}

			if (iou >= th)
			{
				bres = true;
				
				assores->id2 = id2;
				assores->iou = iou;

				if (id2 == 0)
				{
					//인스턴스가 없어서 매칭이 안되는 경우임.
					assores->res = false;
					assores->req = true;
				}
				else {
					assores->id1 = id1;
					assores->res = true;
					sAlreadyFrameMatch.insert(id2);
					
				}
			}
			else {
				if (id2 > 0)
				{
					//매칭이 실패 중 겹치는 값이 있으면 이전 마스크를 복원해서 매칭이 될 수 있도록 함.
					if (iou > bestFailMatch.second)
					{
						bestFailMatch.first = id2;
						bestFailMatch.second = iou;
					}
				}
			}

			if (bres) {
				/*if (id2 > 0)
					vres += mask;*/
				break;
			}
		}

		//현재 인스턴스가 제대로 안되어있으면 복구하는 용도인데 이제 필요한가?
		if (!bres && bestFailMatch.first > 0)
		{
			//raft에서의 에러 또는 마스킹이 작을 때 생김.
			//확인 상 대부분이 마스킹만 잘되면 동작할 듯.
			assores->id2 = bestFailMatch.first;
			assores->res = false;
			assores->req = false;
			assores->iou = bestFailMatch.second;
		}

		//이전 오브젝트 관리. 코드 확인 필요
		if (assores->req)
		{
			auto pPrevG = pPrevSegMask->MapInstances.Get(id1);
			if (pPrevG)
			{
				sAlreadyReqObject.insert(pPrevG->mnId);
			}
		}

		//Global Instance의 추가
		if (assores->res)
		{
			auto pPrevG = pPrevSegMask->MapInstances.Get(id1);
			if (pPrevG){
				sAlreadyMapMatch.insert(pPrevG->mnId);
				//pCurrSegMask->MapInstances.Update(assores->id2, pPrevG);
				//pPrevG->Connect(pCurrSegInstance[assores->id2], pNewBF, assores->id2);
				//pPrevG->EIFFilterOutlier();
				//pPrevG->Connect(pNewBF, assores->id);
				//pPrevG->AddMapPoints(pCurrSegInstance[assores->id]->setMPs);
			}
		}
		//pCurrSegMask->mapResAsso[id1] = assores;
		//assores->nDataType = 1;
		//assores->nAssotype = 1;
		assores->nType1 = ObjectSLAM::InstanceType::SEG;
		pCurrSegMask->mvResAsso.push_back(assores);
		if (assores->res)
		{
			mapCurrAssores[assores->id2] = assores;
		}
	}

	//ObjSystem->vecObjectAssoRes.push_back("associaet::iou::end");

	///출력
	{	
		std::stringstream ss;
		ss << "curr,prev," << pNewBF->mnId << "," << pPrevBF->mnId << ",X";
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
	}

	const cv::Mat T = pKF->GetPose();
	const cv::Mat K = pKF->K.clone();
	const cv::Mat R = T.rowRange(0, 3).colRange(0, 3);
	const cv::Mat t = T.rowRange(0, 3).col(3);

	////Global Object를 이용한 연결
	//1) 이전 프레임에 데이터가 있는데 raft가 실패해서 동작이 안되는 경우
	for (auto pair : pPrevIns)
	{
		auto pid = pair.first;
		if (!pair.second)
			continue;
		if (mapRaftInstance.count(pid))
			continue;
		//res = seg, map
		auto pPrevG = pPrevSegMask->MapInstances.Get(pid);
		if (!pPrevG)
			continue;

		std::set<EdgeSLAM::MapPoint*> spMPs;
		pPrevG->GetLocalMPs(spMPs, pKF, 45.0, 0.01);
		std::vector<cv::Point2f> vecProjPts;
		pPrevG->GetProjPTs(spMPs, vecProjPts, pKF);
		
		int nPT = vecProjPts.size();
		if (nPT == 0)
			nPT++;

		bool bres = false;
		for (auto pair : pCurrSegInstance)
		{
			auto cid = pair.first;
			if (cid == 0)
				continue;
			if (!pair.second)
				continue;
			if (mapCurrAssores.count(cid))
				continue;
			auto pIns = pair.second;

			float val;
			if (ObjectSLAM::InstanceSim::ComputSim(pIns->contour, vecProjPts, val))
			{
				bres = true;
				ObjectSLAM::AssoMatchRes* pAssoRes = new ObjectSLAM::AssoMatchRes();
				pAssoRes->id1 = pid;
				pAssoRes->id2 = cid;
				pAssoRes->res = true;
				pAssoRes->nType1 = ObjectSLAM::InstanceType::SEG;
				//pAssoRes->nDataType = 1;
				pCurrSegMask->mvResAsso.push_back(pAssoRes);
				mapCurrAssores[cid] = pAssoRes;
				break;
			}
		}

		//curr mask에 프로젝션. 실패시 넘어감.
	}
	
	//Global Instance 생성 및 연결
	//글로벌 인스턴스에 맵포인트 추가가 필요함.
	//생성이면 두 인스턴스의 맵포인트 추가
	//연결이면 현재 프레임의 인스턴스만 추가
	//////////////////////////////////////////////////////////////////일단 에러 체크가 필요함(진행중)

	//미싱 글로벌 인스턴스 연결 테스트
	std::map<int, int> mapPrevGlobalAndFrame;
	std::set<ObjectSLAM::GlobalInstance*> setGlobalInstances, setPrevGlobalIns;
	for (auto pair : pPrevSegInstance)
	{
		auto pid = pair.first;
		auto p = pair.second;
		if (pid == 0)
			continue;
		if (!p)
			continue;
		auto pPrevG = pPrevSegMask->MapInstances.Get(pid);
		if (!pPrevG)
			continue;
		setPrevGlobalIns.insert(pPrevG);
	}
	pPrevBF->GetNeighGlobalInstnace(setGlobalInstances);

	//처리시간 기록
	//글로벌 인스턴스와 프레임 인스턴스의 연결(슬램에서 로컬 맵 매칭과 같음.)
	std::map<int, cv::Point2f> mapGlobalCenters;
	std::map<int, std::vector<cv::Point2f>> mapGlobalPTs;
	std::map<int, cv::Rect> mapGlobalRect; //미싱 글로벌 인스턴스 샘 요청을 위한 용도

	cv::Mat tempImg = pNewBF->img.clone();

	auto pGlobalMissMask = new ObjectSLAM::InstanceMask();
	for (auto pG : setGlobalInstances) {
		if (!pG)
		{
			std::stringstream ss;
			ss << "global,map,nullptr";
			ObjSystem->vecObjectAssoRes.push_back(ss.str());
			continue;
		}
		if (pG->mbBad)
			continue;
		if (pG->mapConnected.Size() < 2){
			continue;
		}
		//이전 프레임의 인스턴스는 이미 계산함.
		if (setPrevGlobalIns.count(pG))
			continue;
		//이전에 매칭되었으면 안하는데 이제는 사용 안함.
		if (sAlreadyMapMatch.count(pG->mnId))
			continue;
		if (sAlreadyReqObject.count(pG->mnId))
			continue;

		//Local MP & Projection Points
		std::set<EdgeSLAM::MapPoint*> spMPs;
		pG->GetLocalMPs(spMPs, pKF, 45.0, 0.01);
		std::vector<cv::Point2f> vecProjPts;
		pG->GetProjPTs(spMPs, vecProjPts, pKF);

		int nMP = spMPs.size();
		if (nMP == 0)
			nMP++;

		if (vecProjPts.size() < 2)
			continue;

		int nPT = vecProjPts.size();
		bool bres = false;
		float max_val = 0.0;
		for (auto pair : pCurrSegInstance)
		{
			auto cid = pair.first;
			if (cid == 0)
				continue;
			if (!pair.second)
				continue;
			if (mapCurrAssores.count(cid))
				continue;
			auto pIns = pair.second;

			float val = 0.0;
			if (ObjectSLAM::InstanceSim::ComputSim(pIns->contour, vecProjPts, val))
			{
				std::cout << "object global matching " << std::endl;
				bres = true;
				ObjectSLAM::AssoMatchRes* pAssoRes = new ObjectSLAM::AssoMatchRes();
				pAssoRes->id1 = pG->mnId;
				pAssoRes->id2 = cid;
				pAssoRes->res = true;
				//pAssoRes->nAssotype = 1;
				//pAssoRes->nDataType = 2;
				pCurrSegMask->mvResAsso.push_back(pAssoRes);
				mapCurrAssores[cid] = pAssoRes;

				if (max_val < val)
					max_val = val;

				break;
			}
		}

		if (!bres && max_val < 0.1)
		{
			ObjectSLAM::AssoMatchRes* pAssoRes = new ObjectSLAM::AssoMatchRes();
			pAssoRes->id1 = pG->mnId;
			pAssoRes->res = false;
			pAssoRes->req = true;
			//pAssoRes->nAssotype = 1;
			//pAssoRes->nDataType = 2;

			pCurrSegMask->mvResAsso.push_back(pAssoRes);
			//std::cout << "req asdf " <<pNewBF->mnId<<" "<<pG->mnId<<" " << max_val << std::endl;
			//auto pt = pG->GetCenter(vecProjPts);
			auto rect = pG->GetRect(vecProjPts);
			mapGlobalRect[pG->mnId] = rect;
			pGlobalMissMask->MapInstances.Update(pG->mnId, pG);

			cv::rectangle(tempImg, rect, cv::Scalar(255, 255, 0), 2);
			for (auto pt : vecProjPts)
			{
				cv::circle(tempImg, pt, 3, cv::Scalar(255, 0, 255), 1);
			}
		}

	} 
	SLAM->VisualizeImage(mapName, tempImg, 7);
	pNewBF->mapMasks.Update("gmissing", pGlobalMissMask);

	{
		//처리 시간 기록
		if (ObjSystem->MapTimeStampForKF.Count(pKF->mnId)) {
			std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
			auto s = ObjSystem->MapTimeStampForKF.Get(pKF->mnId);
			auto e = t_end.time_since_epoch().count();
			auto du_latency = (e - s) / 1000000.0;
			std::string keylat = "assoseg";
			auto vec = ObjSystem->MapLatency.Get(keylat);
			vec.push_back(du_latency);
			ObjSystem->MapLatency.Update(keylat, vec);
		}
	}

	std::chrono::high_resolution_clock::time_point t_global_test = std::chrono::high_resolution_clock::now();
	
	{
		std::chrono::high_resolution_clock::time_point t_global_end = std::chrono::high_resolution_clock::now();
		auto du_seg = std::chrono::duration_cast<std::chrono::milliseconds>(t_global_end - t_global_test).count();
		std::stringstream ss;
		ss << "Global processing time," << du_seg;
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
	}
	//미싱 글로벌 인스턴스 연결 테스트
	//이 부분은 미싱 샘 요청이 없으면 안함.
	/*
	std::vector<std::pair<ObjectSLAM::GlobalInstance*, ObjectSLAM::GlobalInstance*>> vecNeedMerge;
	for (auto pair : pCurrSegMask->mapResAsso)
	{
		auto assores = pair.second;

		if (!assores->res)
			continue;
		int id1 = pair.first;
		int id2 = assores->id2;

		if (id1 == 0 || id2 == 0)
		{
			std::cout << "object matching error????" << std::endl;
			continue;
		}

		auto pPrevG = pPrevSegMask->MapInstances.Get(id1);
		auto pCurrG = pCurrSegMask->MapInstances.Get(id2);

		if (pPrevG && pCurrG && pPrevG->mnId != pCurrG->mnId)
		{
			vecNeedMerge.push_back(std::make_pair(pPrevG, pCurrG));
		}

		auto pPrevIns = pPrevSegMask->FrameInstances.Get(id1);
		auto pCurrIns = pCurrSegMask->FrameInstances.Get(id2);

		//오브젝트 맵 관리
		if(pPrevG)
		{
			cv::Mat dir = pPrevG->GetPosition() - pKF->GetCameraCenter();
			auto res = Utils::CalcSphericalCoordinate(dir, 45.0, 0.01); //0.1
			std::set<EdgeSLAM::KeyFrame*> spKFs;
			auto keykf = std::make_pair(res.x, res.y);
			if (pPrevG->MapKFs.Count(keykf))
			{
				spKFs = pPrevG->MapKFs.Get(keykf);
			}
			spKFs.insert(pKF);
			pPrevG->MapKFs.Update(keykf, spKFs);
			std::cout << pKF->mnId << " = spherical test = " << pPrevG->mnId << " " << res << " == " << spKFs.size() << std::endl;

			pPrevG->MapMPs.Clear();
			auto spMPs = pPrevG->AllMapPoints.Get();
			for (auto pMPi : spMPs)
			{
				if (!pMPi || pMPi->isBad())
					continue;
				cv::Mat dir2 = pPrevG->GetPosition() - pMPi->GetWorldPos();
				auto res2 = Utils::CalcSphericalCoordinate(dir2, 45.0, 0.01); //0.1
				auto keymp1 = std::make_pair(res2.x, res2.y);
				auto keymp2 = res2.z;
				std::map<int, std::set<EdgeSLAM::MapPoint*>> spMPs;
				if (pPrevG->MapMPs.Count(keymp1))
				{
					spMPs = pPrevG->MapMPs.Get(keymp1);
				}
				spMPs[keymp2].insert(pMPi);
				pPrevG->MapMPs.Update(keymp1, spMPs);
			}
			auto allMapMPs = pPrevG->MapMPs.Get();
			for (auto pair : allMapMPs)
			{
				int azi = pair.first.first;
				int ele = pair.first.second;
				for (auto pair2 : pair.second)
				{
					int dist = pair2.first;
					std::cout << "spherical mp test = " << azi<<" "<<ele<<" "<<dist << " == " << pair2.second.size() << std::endl;
				}
			}
			//asdf.Update(std::make_tuple(res.x, res.y, res.z), pKF);
		}

		//글로벌 인스턴스 연결 
		if (pPrevG && !pCurrG)
		{
			pCurrSegMask->MapInstances.Update(id2, pPrevG);
			pPrevG->Connect(pCurrIns, pNewBF, id2);
			//pPrevG->EIFFilterOutlier();
			//pPrevG->Connect(pNewBF, id2);
			//pPrevG->AddMapPoints(pCurrIns->setMPs);

		}
		if (!pPrevG && pCurrG)
		{
			pPrevSegMask->MapInstances.Update(id1, pCurrG);
			pCurrG->Connect(pPrevIns, pPrevBF, id1);
			//pCurrG->EIFFilterOutlier();
			//pCurrG->Connect(pPrevBF, id1);
			//pCurrG->AddMapPoints(pPrevIns->setMPs);
		}

		//글로벌 인스턴스 생성
		if (!pPrevG && !pCurrG)
		{
			auto pGlobal = new ObjectSLAM::GlobalInstance();
			pPrevSegMask->MapInstances.Update(id1, pGlobal);
			pCurrSegMask->MapInstances.Update(id2, pGlobal);

			pGlobal->Connect(pPrevIns, pPrevBF, id1);
			pGlobal->Connect(pCurrIns, pNewBF, id2);
			//pGlobal->EIFFilterOutlier();
			//pGlobal->Connect(pPrevBF, id1);
			//pGlobal->Connect(pNewBF, id2);
			//pGlobal->AddMapPoints(pCurrIns->setMPs);
			//pGlobal->AddMapPoints(pPrevIns->setMPs);
		}
	}

	//merge global instance
	for (auto pair : vecNeedMerge) {
		auto pG1 = pair.first;
		auto pG2 = pair.second;

		auto spMPs1 = pG1->AllMapPoints.Get();
		auto spMPs2 = pG2->AllMapPoints.Get();
		float n1 = spMPs1.size();
		float n2 = spMPs2.size();
		float c = 0;
		for (auto pMPi : spMPs1)
		{
			if (!pMPi || pMPi->isBad())
				continue;
			if (spMPs2.count(pMPi))
				c++;
		}

		float val = 0.0;
		val = c / (n1 + n2 - c);
		std::string res = "fail,";
		if (val > 0.5)
		{
			pG1->Merge(pG2);
			res = "success,";
		}
		std::stringstream ss;
		ss << "Merge," <<res<< pG1->mnId << "," << pG2->mnId << "," << val;
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
	}
	*/
	//////////////////////////////////////////////////////////////////일단 에러 체크가 필요함(진행중)

	//현재 프렝미의 인스턴스 중에서 매칭이 안된 애들을 이전 프레임 집합에서 찾기
	//1) 새로운 글로벌 인스턴스 생성
	//2) 프레임을 벗어났던 인스턴스 연결
	std::vector<cv::Point2f> vecPoints;
	//ObjectSLAM::InstanceLinker::FindInstances(pNewBF, pPrevBF, setMissingGlobal, vecPoints, true);
	//ObjectSLAM::InstanceLinker::LinkNeighBFs(pNewBF, resMatchIns, true);

	//미싱 된 인스턴스 계산
	//req 요청
	//gi 관리 수정 필요
	ObjectSLAM::InstanceMask* pSamMask = nullptr;
	if(!pNewBF->mapMasks.Count("missing"))
	{ 
		pSamMask = new ObjectSLAM::InstanceMask();
		pSamMask->id1 = pPrevBF->mnId; // previous frame
		pSamMask->id2 = id;				//current frame
		pNewBF->mapMasks.Update("missing", pSamMask);
	}
	else
	{
		pSamMask = pNewBF->mapMasks.Get("missing");
	}

	//cv::Mat ptdata = cv::Mat::zeros(0, 1, CV_32FC1);
	//id 관련하여 curr seg의 max id는 이미 알고있으니 추가할 때 되도록 하면 됨.
	//미싱 오브젝트에 추가되는 정보는 raft를 이용해서 조정 된 위치 정보.
	//다시 말하면, previous segmentation에서 플로우를 따라서 변화된 위치이기에 기존 정보와는 다름.
	//다만, id를 이용해서 미싱 정보에 인스턴스와 이전 프레임에 인스턴스가 같다는 것을 알려줌.
	
	
	//샘을 위한 데이터 추가
	cv::Mat ptdata(0, 1, CV_32FC1);
	auto vecResAsso = pCurrSegMask->mvResAsso.get();
	for (auto assores : vecResAsso)
	{
		if (assores->res || !assores->req)
			continue;
		if (assores->nType1 != ObjectSLAM::InstanceType::SEG)
			continue;
		int mid = assores->id1;

		if (!mapRaftInstance.count(mid))
			std::cout << "raft error" << std::endl;
		auto pRaftIns = mapRaftInstance[mid];
		if (!pRaftIns)
		{
			std::stringstream ss;
			ss << "reqsam2,err,raftins,nullptr";
			ObjSystem->vecObjectAssoRes.push_back(ss.str());
			continue; 
		}
		pSamMask->FrameInstances.Update(mid, pRaftIns);

		auto rect = pRaftIns->rect;
		cv::Mat temp = cv::Mat::zeros(4, 1, CV_32FC1);
		temp.at<float>(0) = rect.x;
		temp.at<float>(1) = rect.y;
		temp.at<float>(2) = rect.x + rect.width;
		temp.at<float>(3) = rect.y + rect.height;
		ptdata.push_back(temp);
	}
	
	//임시로 가우시안 객체 생성 체크
	std::set<ObjectSLAM::GOMAP::GaussianObject*> setTempGOs;
	for (auto res : vecResAsso)
	{
		auto pid = res->id1;
		auto cid = res->id2;

		auto bSuccess = res->res;
		auto bRequest = res->req;

		if (res->nType1 != ObjectSLAM::InstanceType::MAP)
			continue;

		if (!bSuccess) {
			continue;
		}
		auto pIns = pPrevSegMask->FrameInstances.Get(pid);
		auto cIns = pCurrSegMask->FrameInstances.Get(cid);

		auto pG1 = pPrevSegMask->GaussianMaps.Get(pid);
		auto pG2 = pCurrSegMask->GaussianMaps.Get(cid);

		if (!pG1 && !pG2)
		{
			auto pGO = ObjectSLAM::GaussianMapManager::InitializeObject(pIns, cIns);
			pPrevSegMask->GaussianMaps.Update(pid, pGO);
			pCurrSegMask->GaussianMaps.Update(cid, pGO);
			setTempGOs.insert(pGO);
			pGO->AddObservation(pPrevSegMask,pIns);
			pGO->AddObservation(pCurrSegMask, cIns);
		}
		if (pG1 && !pG2)
		{
			ObjectSLAM::GOMAP::Optimizer::ObjectOptimizer::ObjectPosOptimization(pG1);
			 
			ObjectSLAM::GaussianMapManager::UpdateObjectWithIncremental(pG1, cIns);
			pCurrSegMask->GaussianMaps.Update(cid, pG1);
			setTempGOs.insert(pG1);
			pG1->AddObservation(pCurrSegMask, cIns);
		}

	}
	//임시로 가우시안 객체 생성 체크

	//ObjSystem->vecObjectAssoRes.push_back("asso::4");
	//샘요청
	{
		for (auto pair : mapGlobalRect)
		{
			auto rect = pair.second;
			cv::Mat temp = cv::Mat::zeros(4, 1, CV_32FC1);
			temp.at<float>(0) = rect.x;
			temp.at<float>(1) = rect.y;
			temp.at<float>(2) = rect.x + rect.width;
			temp.at<float>(3) = rect.y + rect.height;
			ptdata.push_back(temp);
		}

		if (ptdata.rows == 0)
		{
			pCurrSegMask->UpdateAssociation(pPrevSegMask, pNewBF, pPrevBF, ObjSystem);
		}

		if (ptdata.rows > 0) {
			//reqest
			int nobj = ptdata.rows;
			ptdata.push_back(cv::Mat::zeros(1500 - nobj, 1, CV_32FC1));
			std::string tsrc = userName + ".Image." + std::to_string(nobj);
			auto sam2key = "reqsam2";
			std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
			auto du_upload = Utils::SendData(sam2key, tsrc, ptdata, id, 15, t_start.time_since_epoch().count());
		}
	}
	//ObjSystem->vecObjectAssoRes.push_back("asso::test5");
	pSamMask->nTrial++;

	////ptdata == 0이면 업데이트
	////mvResAsso로 수행하기

	//시각화 관련
	//if(ptdata.rows == 0)
	//	ObjectSLAM::AssociationManager::VisualizeAssociation(SLAM, pNewBF, pPrevBF, pCurrSegMask, pPrevSegMask, mapName);
	
	if(false){

		////시각화
		{
			//현재 인스턴스 기본
			for (auto pair : pCurrSegInstance) {
				if (pair.first == 0)
					continue;
				currmask += pair.second->mask * 0.5;
			}
			
			//어소시에이션 결과 추가
			for (auto assores : vecResAsso)
			{
				auto pid = assores->id1;
				auto cid = assores->id2;
				auto pIns = pPrevSegInstance[pid];

				//if (assores->nDataType != 1)
				//	continue;

				auto bSuccess = assores->res;
				auto bRequest = assores->req;

				if (!bSuccess)
				{
					if (bRequest) {
						prevmask += pIns->mask * 0.25;
						/*if (pRaftIns)
						{
							raftmask += pRaftIns->mask * 0.25;
						}*/
					}
					else {
						prevmask += pIns->mask * 0.5;
						/*if (pRaftIns)
						{
							raftmask += pRaftIns->mask * 0.5;
						}*/
					}
				}

				if (bSuccess) {
					auto cIns = pCurrSegInstance[cid];
					prevmask += pIns->mask;
					currmask += cIns->mask;
					/*if (pRaftIns)
					{
						raftmask += pRaftIns->mask;
					}*/
				}
			}
			
			std::set<ObjectSLAM::GlobalInstance*> spGlobals;
			auto mapGlobals = pCurrSegMask->MapInstances.Get();
			for (auto pair : mapGlobals)
			{
				if (!pair.second)
					continue;
				spGlobals.insert(pair.second);
			}
			//매칭 성공한 포인트의 위치 추가
			std::vector<std::pair<cv::Point2f, cv::Point2f>> vecPairVisualizedMatches;
			for (auto assores : vecResAsso)
			{
				if (!assores->res)
					continue;
				//if (assores->nDataType != 1)
				//	continue;
				int id1 = assores->id1;
				int id2 = assores->id2;

				auto pt1 = pPrevSegInstance[id1]->pt;
				auto pt2 = pCurrSegInstance[id2]->pt;
				vecPairVisualizedMatches.push_back(std::make_pair(pt1, pt2));
			}
			
			//시각화 이미지
			cv::Mat resImage;
			cv::cvtColor(prevmask, prevmask, cv::COLOR_GRAY2BGR);
			cv::cvtColor(currmask, currmask, cv::COLOR_GRAY2BGR);

			//////글로벌 인스턴스 출력
			if(true)
			{
				const cv::Mat T = pKF->GetPose();
				const cv::Mat K = pKF->K.clone();
				const cv::Mat R = T.rowRange(0, 3).colRange(0, 3);
				const cv::Mat t = T.rowRange(0, 3).col(3);
				
				for (auto pG : setGlobalInstances) {
					if (pG->mbBad)
						continue;
					if (!spGlobals.count(pG))
						continue;
					if (pG->mapConnected.Size() <= 2)
						continue;

					//draw map points
					auto vecMPs = pG->AllMapPoints.ConvertVector();
					for (auto pMPi : vecMPs) {
						if (!pMPi || pMPi->isBad())
							continue;
						auto pt = CommonUtils::Geometry::ProjectPoint(pMPi->GetWorldPos(), K, R, t);
						cv::circle(currmask, pt, 5, SemanticSLAM::SemanticProcessor::SemanticColors[pG->mnId + 1], -1);
					}
					auto vec = mapGlobalPTs[pG->mnId];
					for (auto pt : vec)
					{
						cv::circle(currmask, pt, 5, SemanticSLAM::SemanticProcessor::SemanticColors[pG->mnId + 1], -1);
					}
				}
				for (auto pG : setTempGOs)
				{
					ObjectSLAM::GaussianVisualizer::visualize2D(currmask, pG, K, R, t, cv::Scalar(255, 0, 255));
					ObjectSLAM::GaussianVisualizer::visualize3D(currmask, pG, K, R, t, cv::Scalar(255, 0, 255));
				}

				//아이디
				for (auto pG : setGlobalInstances) {
					if (pG->mbBad)
						continue;
					if (!spGlobals.count(pG))
						continue;
					if (pG->mapConnected.Size() <= 2)
						continue;

					//pG->UpdatePosition();
					//auto pt = pG->ProjectPoint(T, K);

					auto pt = mapGlobalCenters[pG->mnId];
					//cv::circle(currmask, pt, 5, cv::Scalar(0, 255, 255), 2);
					//cv::putText(currmask, std::to_string(pG->mnId), pt, 2, 1.3, cv::Scalar(255, 255, 0), 2);

					////bounding box
					std::vector<cv::Point2f> vecPoints;
					//pG->ProjectBB(vecPoints, K, T);
					//pG->DrawBB(currmask, vecPoints);
				}
			}
			//////글로벌 인스턴스 출력

			//////frame id 출력
			if (true)
			{
				for (auto assores : vecResAsso)
				{
					auto pid = assores->id1;
					//if (assores->nDataType != 1)
					//	continue;
					auto pIns = pPrevSegInstance[pid];
					auto textpt = pIns->pt;
					cv::putText(prevmask, std::to_string(pid), textpt, 2, 1.3, cv::Scalar(255, 0, 0), 2);
				}
				
				for (auto pair : pCurrSegInstance) {
					if (pair.first == 0)
						continue;
					auto textpt = pair.second->pt;
					cv::putText(currmask, std::to_string(pair.first), textpt, 2, 1.3, cv::Scalar(255, 0, 0), 2);
				}
				
				/*{
					std::stringstream ss;
					ss.str("");
					ss << "../res/aaresasso/" << id << "_raft.png";
					cv::imwrite(ss.str(), raftmask);
				}*/
			}
			//////id 출력

			//temp rect
			/*for (auto pair : mapGlobalRect)
			{
				auto rect = pair.second;
				cv::rectangle(currmask, rect, cv::Scalar(255, 255, 0), 2);
			} */
			
			SLAM->VisualizeMatchingImage(resImage, prevmask, currmask, vecPairVisualizedMatches, mapName, 2);
			{
				std::stringstream ss;
				ss.str("");
				ss << "../res/aaresasso/" << id << "_test.png";
				cv::imwrite(ss.str(), resImage);
			}
			
		}
		
		//cv::Rect uRect(0, 0, w, h);
		//cv::Rect lRect(0, h, w, h);
		//SLAM->VisualizeMatchingImage(resImage, resImage(uRect), resImage(lRect), vecPairMatches, User->mapName, 2);
		//SLAM->VisualizeMatchingImage(resImage, resImage(uRect), resImage(lRect), vecFailVisualizedMatches, mapName, 2, cv::Scalar(0,0,255));
	}
	
	{
		std::stringstream ss;
		ss << "eval::seg," << id << "," <<pCurrSegMask->mnOriSize<< "," << pCurrSegMask->mnMaxId - 1;
		ObjSystem->MapNumObjects.Update(id, ss.str());
	}

	if (bShow){
		std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		auto du_seg = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
		std::cout << keyprocess << "== " << id << " == " << du_seg <<" "<< mapRaftInstance.size() << std::endl;
	}
	{
		auto pCurrSegInstance = pCurrSegMask->FrameInstances.Get();
		auto pPrevSegInstance = pPrevSegMask->FrameInstances.Get();
		/*{
			std::stringstream ss;
			ss << "asso,size," << pPrevSegInstance.size() << "," << pCurrSegInstance.size();
			ObjSystem->vecObjectAssoRes.push_back(ss.str());
		}*/
		ObjSystem->vecObjectAssoRes.push_back(keyprocess + " end");
	}
	
}

void MatchingTest(ObjectSLAM::BoxFrame* pNewBF, EdgeSLAM::SLAM* SLAM, EdgeSLAM::User* User, ObjectSLAM::ObjectDevice* pObjDevice, const std::string& mapName) {

	auto pKF = pNewBF->mpRefKF;

	auto vpBFs = ObjSystem->GetConnectedBoxFrames(pKF);

	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	
	//BF에서 인스턴스와 레이블 별로 나누기.

	for (auto pBF : vpBFs) {
		std::vector<int> idxs;
		std::vector<std::pair<int, cv::Point2f>> matches;
		if (!pBF)
			continue;
		pBF->MatchingWithFrame(pNewBF->gray, pKF->GetPose(), pKF->K, idxs, matches);
	}

	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	auto du = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
	std::cout << "matching test = " << vpBFs.size() << " " << du << std::endl;
}

void raft(EdgeSLAM::SLAM* SLAM, std::string src, int id, long long received_ts) {
	
	if (bSaveSegLatency)
	{
		std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
		auto du_latency = (t_start.time_since_epoch().count() - (long long)received_ts) / 1000000.0;
		auto vec = ObjSystem->MapLatency.Get(deepkey3);
		vec.push_back(du_latency);
		ObjSystem->MapLatency.Update(deepkey3, vec);
	}
	
	auto vecstr = split(src, '.');
	auto user = vecstr[0];
	int id2 = stoi(vecstr[1]);
	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	User->mnUsed++;
	if (!ObjSystem->MapObjectDevices.Count(User)) {
		User->mnUsed--;
		return;
	}

	{
		std::stringstream ss;
		ss << "raft::start," << id;
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
	}

	auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
	std::string mapName = User->mapName;
	bool bSave = User->mbSave;
	bool bVis = User->GetVisID() == 0;

	auto pNewBF = ObjSystem->MapKeyFrameNBoxFrame.Get(id);//pObjDevice->mpCurrBF;
	auto pKF = pNewBF->mpRefKF;

	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss1;
	ss1 << "/Download?keyword=raft" << "&id=" << id << "&src=" << src;
	WebAPI API("143.248.6.143", 35005);
	auto res1 = API.Send(ss1.str(), "");

	std::stringstream ss2;
	ss2 << "/Download?keyword=raft_y" << "&id=" << id << "&src=" << user;
	auto res2 = API.Send(ss2.str(), "");

	cv::Mat temp1 = cv::Mat(res1.size(), 1, CV_8UC1, (void*)res1.data());
	cv::Mat flow_x = cv::imdecode(temp1, cv::IMREAD_ANYDEPTH);

	cv::Mat temp2 = cv::Mat(res2.size(), 1, CV_8UC1, (void*)res2.data());
	cv::Mat flow_y = cv::imdecode(temp2, cv::IMREAD_ANYDEPTH);
	cv::Mat flow;
	
	//flow_x.convertTo(flow_x, CV_8SC1);
	//flow_y.convertTo(flow_y, CV_8SC1);
	std::vector<cv::Mat> vecFlow;
	vecFlow.push_back(flow_x);
	vecFlow.push_back(flow_y);
	cv::merge(vecFlow, flow);
	//std::cout <<"raft size = " << flow.size() <<" "<<flow.type() << " " << CV_8UC2 << std::endl;

	//flow, id1, id2를 알게 하기
	ObjectSLAM::InstanceMask* pRaftMask = new ObjectSLAM::InstanceMask();
	pRaftMask->id1 = id; //target, current
	pRaftMask->id2 = id2;//reference, previous
	pRaftMask->mask = flow.clone();
	pNewBF->mapMasks.Update("raft", pRaftMask);

	//욜로 와의 결합 파트, 두 타겟을 알아야 함.
	//박스의 중점과 다른 추적 포인트를 알아야 함. 해당 포인트에 대해 마지막으로 마스크를 추정함.
	//트래킹 시에는 에러 포인트를 어떻게 감지할지 정해야 함.(마지막)
	if (!ObjSystem->MapKeyFrameNBoxFrame.Count(id2))
	{
		std::cout << "fail search box frame " << id2 << std::endl;
		std::stringstream ss;
		ss << "raft::end,fail::prevbox," << id << "," << id2;
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
		User->mnUsed--;
		return;
	}
	auto pPrevBF = ObjSystem->MapKeyFrameNBoxFrame.Get(id2);
	auto pPrevKF = pPrevBF->mpRefKF;

	ObjectSLAM::InstanceMask* pPrevRaftMask = nullptr;
	if (pPrevBF->mapMasks.Count("raft")) {
		pPrevRaftMask = pPrevBF->mapMasks.Get("raft");
	}
	
	//std::cout << "raft = " << id2 <<" "<<(int)flow_x.at<schar>(50,50)<<"   " << flow_x.type() << " " << flow_y.type() << " " << CV_32F << std::endl;
	
	std::vector < std::pair < cv::Point2f, cv::Point2f>> vecPairMatches;
	std::vector < std::pair < cv::Point2f, cv::Point2f>> vecPrevPairMatches;
	
	int w = flow_x.cols * 4-5;
	int h = flow_y.rows * 4-5;

	cv::Mat ptdata = cv::Mat::zeros(0, 1, CV_32FC1);
	std::set<int> spAlready;
	
	ObjectSLAM::InstanceMask* pPrevSegMask = nullptr;
	if (pPrevBF->mapMasks.Count("yoloseg")) {
		pPrevSegMask = pPrevBF->mapMasks.Get("yoloseg");
	}
	ObjectSLAM::InstanceMask* pCurrSegMask = nullptr;
	if (pNewBF->mapMasks.Count("yoloseg")) {
		pCurrSegMask = pNewBF->mapMasks.Get("yoloseg");
	}

	//association
	ObjectSLAM::InstanceMask* pPrevSam2 = nullptr;

	bool bAsso = false;
	if (pPrevSegMask && pCurrSegMask)
	{
		if (!pCurrSegMask->bInit) {
			pCurrSegMask->bInit = true;
			bAsso = true;
			//std::cout << "success asso in raft" << id << std::endl;
			//AssociateMissingObject(SLAM, user + ".Image", id, mapName, pNewBF, pPrevSegMask, pCurrSegMask, pRaftMask);
			//ObjectSLAM::AssociationManager::AssociationWithMAP(
			//	SLAM, ObjSystem, "seg", id, mapName, user, pNewBF, pPrevBF, pPrevSegMask, pCurrSegMask, pRaftMask, true);
			
			ObjectSLAM::AssociationManager::Association(
				SLAM, ObjSystem, "seg", id, id2, mapName, user, pNewBF, pPrevBF, pPrevSegMask, pCurrSegMask, pRaftMask, true);
			//AssociateMissingObject2(SLAM, ObjSystem, "seg", id, mapName, user, pNewBF, pPrevBF, pPrevSegMask, pCurrSegMask, pRaftMask, false);
		}
				
		//if (bAsso && pNewBF->mapMasks.Count("missing")) {
		//	auto pSamMask = pNewBF->mapMasks.Get("missing");
		//	auto pMissingInstance = pSamMask->FrameInstances.Get();
		//	for (auto pair : pMissingInstance)
		//	{
		//		auto rect = pair.second->rect;
		//		cv::Mat temp = cv::Mat::zeros(4, 1, CV_32FC1);
		//		temp.at<float>(0) = rect.x;
		//		temp.at<float>(1) = rect.y;
		//		temp.at<float>(2) = rect.x + rect.width;
		//		temp.at<float>(3) = rect.y + rect.height;
		//		ptdata.push_back(temp);
		//	}
		//}

		////sam 요청
		//if (ptdata.rows > 0) {
		//	//reqest
		//	int nobj = ptdata.rows;
		//	ptdata.push_back(cv::Mat::zeros(1500 - nobj, 1, CV_32FC1));
		//	std::string tsrc = user + ".Image." + std::to_string(nobj);
		//	auto sam2key = "reqsam2";
		//	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
		//	auto du_upload = Utils::SendData(sam2key, tsrc, ptdata, id, 15, t_start.time_since_epoch().count());
		//}
	}
	else {
		//std::cout << "fail seg in raft " << id << std::endl;
	}

	{
		auto pid = -1;
		if (pPrevSegMask)
			pid = 1;
		auto cid = -1;
		if (pCurrSegMask)
			cid = 1;
		std::stringstream ss;
		ss << "raft::end,id," << id<<","<< id2 << ",exist," << pid << "," << cid;
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
	}

	User->mnUsed--;
}
void sam3(EdgeSLAM::SLAM* SLAM, std::string user, int id, long long received_ts, bool bShow = true) {
	if (bSaveSegLatency)
	{
		std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
		auto du_latency = (t_start.time_since_epoch().count() - (long long)received_ts) / 1000000.0;
		auto vec = ObjSystem->MapLatency.Get(deepkey2);
		vec.push_back(du_latency);
		ObjSystem->MapLatency.Update(deepkey2, vec);
	}

	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	User->mnUsed++;
	if (!ObjSystem->MapObjectDevices.Count(User)) {
		User->mnUsed--;
		return;
	}

	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	auto pNewBF = ObjSystem->MapKeyFrameNBoxFrame.Get(id);//pObjDevice->mpCurrBF;
	ObjectSLAM::InstanceMask* pMask = pNewBF->mapMasks.Get("missing");// new ObjectSLAM::InstanceMask();
	pNewBF->mbSam2 = true;
	auto pKF = pNewBF->mpRefKF;
	auto pPrevBF = ObjSystem->MapKeyFrameNBoxFrame.Get(pMask->id1);

	auto pPrevSeg = pPrevBF->mapMasks.Get("yoloseg");
	auto pCurrSeg = pNewBF->mapMasks.Get("yoloseg");

	auto pPrevInstance = pPrevSeg->FrameInstances.Get();
	auto pCurrInstance = pCurrSeg->FrameInstances.Get();

	{
		std::stringstream ss;
		ss << "sam::start," << id << "," << pCurrInstance.size() << "," << pPrevInstance.size();
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
	}

	auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
	std::string mapName = User->mapName;
	bool bSave = User->mbSave;
	bool bVis = User->GetVisID() == 0;

	////data download
	int w = User->mpCamera->mnWidth;
	int h = User->mpCamera->mnHeight;

	ObjectSLAM::InstanceMask* pGlobalMiss = nullptr;
	std::map<int, ObjectSLAM::GlobalInstance*> mapMissGlobalIns;

	if (pNewBF->mapMasks.Count("gmissing")) {
		pGlobalMiss = pNewBF->mapMasks.Get("gmissing");
		mapMissGlobalIns = pGlobalMiss->MapInstances.Get();
	}

	cv::Mat prevmask = cv::Mat::zeros(h, w, CV_8UC1);
	cv::Mat currmask = cv::Mat::zeros(h, w, CV_8UC1);

	//키프레임 자세
	const cv::Mat T = pKF->GetPose();
	const cv::Mat K = pKF->K.clone();
	const cv::Mat R = T.rowRange(0, 3).colRange(0, 3);
	const cv::Mat t = T.rowRange(0, 3).col(3);

	//contour - centeroid, rect를 갱신한다면?
	//sam2
	cv::Mat vres = cv::Mat::zeros(h, w, CV_8UC1);
	std::map<int, ObjectSLAM::FrameInstance*> mapSamInstances;
	std::set<int> setNewIDs;

	auto vecMPs = pKF->mvpMapPoints.get();
	const cv::Mat background = pCurrInstance[0]->mask;

	cv::Mat nimg = cv::Mat::zeros(h, w, CV_8UC1);

	std::stringstream ss;
	ss << "/Download?keyword=sam2c" << "&id=" << id << "&src=" << user;
	WebAPI API("143.248.6.143", 35005);
	auto res = API.Send(ss.str(), "");

	cv::Mat temp = cv::Mat(res.size() / 2, 1, CV_16UC1, (void*)res.data());
	int idx = 0;
	std::vector<std::vector<cv::Point>> contours;
	while (true)
	{
		int iid = temp.at<ushort>(idx++);
		int ni = temp.at<ushort>(idx++);

		//std::cout << iid << " " << ni << " " << idx << std::endl;

		std::vector<cv::Point> contour;
		for (int ti = 0; ti < ni; ti++)
		{
			auto x = temp.at<ushort>(idx++);
			auto y = temp.at<ushort>(idx++);
			contour.push_back(cv::Point(x, y));
		}
		contours.push_back(contour);
		if (idx == temp.rows)
			break;
	}

	for (int i = 0; i < contours.size(); i++)
	{
		cv::Mat newmask = cv::Mat::zeros(h, w, CV_8UC1);;
		cv::drawContours(newmask, contours, i, cv::Scalar(255, 255, 255), -1);

		if (ObjectSLAM::InstanceSim::CheckStaticObject(contours[i], pCurrInstance, 2))
			continue;
		auto pIns = new ObjectSLAM::FrameInstance(pKF);
		pIns->contour = contours[i];

		pIns->mask = newmask.clone();
		pIns->area = cv::countNonZero(newmask);
		pIns->rect = cv::boundingRect(contours[i]);
		//pIns->rrect = cv::fitEllipse(contours[0]);

		cv::Moments mu = cv::moments(contours[i], false);
		pIns->pt = cv::Point2f(static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
			static_cast<float>(mu.m01 / (mu.m00 + 1e-5)));

		//associate map points
		pIns->Update(pKF);
		/*for (int idx = 0; idx < vecMPs.size(); idx++)
		{
			auto pMPi = vecMPs[idx];

			if (!pMPi || pMPi->isBad())
				continue;
			auto pt = pKF->mvKeys[idx].pt;
			if (cv::pointPolygonTest(pIns->contour, pt, false) < 0.0)
				continue;
			pIns->setMPs.insert(pMPi);
			pIns->setKPs.insert(idx);
		}*/
		mapSamInstances[i + 1] = pIns;
		nimg += newmask;

	}
}
void sam2(EdgeSLAM::SLAM* SLAM, std::string user, int id, long long received_ts, bool bShow = true) {

	if (bSaveSegLatency)
	{
		std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
		auto du_latency = (t_start.time_since_epoch().count() - (long long)received_ts) / 1000000.0;
		auto vec = ObjSystem->MapLatency.Get(deepkey2);
		vec.push_back(du_latency);
		ObjSystem->MapLatency.Update(deepkey2, vec);
	}

	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	User->mnUsed++;
	if (!ObjSystem->MapObjectDevices.Count(User)) {
		User->mnUsed--;
		return;
	}

	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	auto pNewBF = ObjSystem->MapKeyFrameNBoxFrame.Get(id);//pObjDevice->mpCurrBF;
	ObjectSLAM::InstanceMask* pMissingMask = pNewBF->mapMasks.Get("missing");// new ObjectSLAM::InstanceMask();
	pNewBF->mbSam2 = true;
	
	auto pKF = pNewBF->mpRefKF;
	auto pCurrSeg = pNewBF->mapMasks.Get("yoloseg");
	auto pCurrInstance = pCurrSeg->FrameInstances.Get();

	//auto pPrevBF = pNewBF->mpPrevBF; //ObjSystem->MapKeyFrameNBoxFrame.Get(pMissingMask->id1);
	//auto pPrevSeg = pPrevBF->mapMasks.Get("yoloseg");
	//auto pPrevInstance = pPrevSeg->FrameInstances.Get();
	
	{
		std::stringstream ss;
		ss << "sam::start," << id << "," << pCurrInstance.size();// << "," << pPrevInstance.size();
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
	}

	auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
	std::string mapName = User->mapName;
	bool bSave = User->mbSave;
	bool bVis = User->GetVisID() == 0;
	
	////data download
	int w = User->mpCamera->mnWidth;
	int h = User->mpCamera->mnHeight;

	////사용 안함
	/*ObjectSLAM::InstanceMask* pGlobalMiss = nullptr;
	std::map<int, ObjectSLAM::GlobalInstance*> mapMissGlobalIns;
	if (pNewBF->mapMasks.Count("gmissing")) {
		pGlobalMiss = pNewBF->mapMasks.Get("gmissing");
		mapMissGlobalIns = pGlobalMiss->MapInstances.Get();
	}*/

	cv::Mat prevmask = cv::Mat::zeros(h, w, CV_8UC1);
	cv::Mat currmask = cv::Mat::zeros(h, w, CV_8UC1);

	//키프레임 자세
	const cv::Mat T = pKF->GetPose();
	const cv::Mat K = pKF->K.clone();
	const cv::Mat R = T.rowRange(0, 3).colRange(0, 3);
	const cv::Mat t = T.rowRange(0, 3).col(3);

	//contour - centeroid, rect를 갱신한다면?
	//sam2
	cv::Mat vres = cv::Mat::zeros(h, w, CV_8UC1);
	std::map<int, ObjectSLAM::FrameInstance*> mapSamInstances;
	std::set<int> setNewIDs;

	auto vecMPs = pKF->mvpMapPoints.get();
	const cv::Mat background = pCurrInstance[0]->mask;

	////샘 인스턴스 생성 과정
	cv::Mat nimg = cv::Mat::zeros(h, w, CV_8UC1);;
	{
		std::stringstream ss;
		ss << "/Download?keyword=sam2c" << "&id=" << id << "&src=" << user;
		WebAPI API("143.248.6.143", 35005);
		auto res = API.Send(ss.str(), "");

		cv::Mat temp = cv::Mat(res.size() / 2, 1, CV_16UC1, (void*)res.data());
		int idx = 0;
		std::vector<std::vector<cv::Point>> contours;
		while (true)
		{
			int iid = temp.at<ushort>(idx++);
			int ni = temp.at<ushort>(idx++);

			//std::cout << iid << " " << ni << " " << idx << std::endl;

			std::vector<cv::Point> contour;
			for (int ti = 0; ti < ni; ti++)
			{
				auto x = temp.at<ushort>(idx++);
				auto y = temp.at<ushort>(idx++);
				contour.push_back(cv::Point(x, y));
			}
			contours.push_back(contour);
			if (idx == temp.rows)
				break;
		}

		for (int i = 0; i < contours.size(); i++)
		{
			cv::Mat newmask = cv::Mat::zeros(h, w, CV_8UC1);;
			cv::drawContours(newmask, contours, i, cv::Scalar(255, 255, 255), -1);
			
			if (ObjectSLAM::InstanceSim::CheckStaticObject(contours[i], pCurrInstance, 2))
				continue;
			auto pIns = new ObjectSLAM::FrameInstance(pKF);
			pIns->contour = contours[i];

			pIns->mask = newmask.clone();
			pIns->area = cv::countNonZero(newmask);
			pIns->rect = cv::boundingRect(contours[i]);
			//pIns->rrect = cv::fitEllipse(contours[0]);

			cv::Moments mu = cv::moments(contours[i], false);
			pIns->pt = cv::Point2f(static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
				static_cast<float>(mu.m01 / (mu.m00 + 1e-5)));

			//associate map points
			pIns->Update(pKF);
			/*for (int idx = 0; idx < vecMPs.size(); idx++)
			{
				auto pMPi = vecMPs[idx];

				if (!pMPi || pMPi->isBad())
					continue;
				auto pt = pKF->mvKeys[idx].pt;
				if (cv::pointPolygonTest(pIns->contour, pt, false) < 0.0)
					continue;
				pIns->setMPs.insert(pMPi);
				pIns->setKPs.insert(idx);
			}*/
			mapSamInstances[i+1] = pIns;
			nimg += newmask; 

		}
	}

	//샘 인스턴스 테스트

	ObjectSLAM::AssociationManager::AssociationWithSAM(
		SLAM, ObjSystem, "seg", id, mapName, user, pNewBF, pCurrSeg, mapSamInstances, true);

	User->mnUsed--;
	return;
	
	//정보 출력 내용
	/*std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	auto du_seg = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
	{
		std::stringstream ss;
		ss << "sam::end," << id<<","<<pCurrIns.size()<<","<<pPrevSeg->FrameInstances.Size();
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
	}

	if(bShow)
		std::cout << "SAM2" << "== " << id << " == " << du_seg <<" == "<< pMissingMask->id2 << " " << pMissingMask->id1 << std::endl;

	User->mnUsed--;*/
}

void yoloseg(EdgeSLAM::SLAM* SLAM, std::string keyword, std::string user, int id, long long received_ts, bool bShow = false) {
	
	if (bSaveSegLatency)
	{
		std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
		auto du_latency = (t_start.time_since_epoch().count() - (long long)received_ts) / 1000000.0;
		auto vec = ObjSystem->MapLatency.Get(deepkey1);
		vec.push_back(du_latency);
		ObjSystem->MapLatency.Update(deepkey1, vec);
	}
	auto User = SLAM->GetUser(user);
	if (!User){
		std::stringstream ss;
		ss << "yoloseg,err," << id;
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
		return;
	}
	 
	User->mnUsed++;
	if (!ObjSystem->MapObjectDevices.Count(User)) {
		std::stringstream ss;
		ss << "yoloseg,err," << id;
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
		User->mnUsed--;
		return;
	}

	{
		std::stringstream ss;
		ss << "yoloseg::start," << id;
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
	}
	
	if (bShow)
	{
		std::cout << "yoloseg start = " << id << std::endl;
	}

	auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
	std::string mapName = User->mapName;
	bool bSave = User->mbSave;
	bool bVis = User->GetVisID() == 0;

	auto pNewBF = ObjSystem->MapKeyFrameNBoxFrame.Get(id);//pObjDevice->mpCurrBF;
	auto pKF = pNewBF->mpRefKF;

	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	

	ObjectSLAM::InstanceMask* pMask = new ObjectSLAM::InstanceMask();
	//cv::Mat temp = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());
	//pMask->mask = cv::imdecode(temp, cv::IMREAD_GRAYSCALE);
	
	
	
	int w = User->mpCamera->mnWidth;
	int h = User->mpCamera->mnHeight;
	cv::Mat segcolor = cv::Mat::zeros(h, w, CV_8UC3);
	
	cv::Mat nimg = cv::Mat::zeros(h, w, CV_8UC1);
	{
		std::stringstream ss;
		ss << "/Download?keyword=yolosegc" << "&id=" << id << "&src=" << user;
		WebAPI API("143.248.6.143", 35005);
		auto res = API.Send(ss.str(), "");

		cv::Mat temp = cv::Mat(res.size() / 2, 1, CV_16UC1, (void*)res.data());
		int idx = 0;
		std::vector<std::vector<cv::Point>> contours;
		int insid = 0;

		cv::Mat newBackground = cv::Mat::zeros(h, w, CV_8UC1);
		while (true)
		{
			int iid = temp.at<ushort>(idx++);
			int ni = temp.at<ushort>(idx++);

			std::vector<cv::Point> contour;
			for (int ti = 0; ti < ni; ti++)
			{
				auto x = temp.at<ushort>(idx++);
				auto y = temp.at<ushort>(idx++);
				contour.push_back(cv::Point(x, y));
			}
			//contour까지는 읽고 넘기기
			if (iid == 60){
				if (idx == temp.rows)
					break;
				continue;
			}
			
			contours.push_back(contour);

			//instance 생성
			auto pIns = new ObjectSLAM::FrameInstance(pKF);
			pIns->contour = contour;

			cv::Mat newmask = cv::Mat::zeros(h, w, CV_8UC1);;
			cv::drawContours(newmask, contours, insid, cv::Scalar(255, 255, 255), -1);
			
			pIns->mask = newmask.clone();
			pIns->area = cv::countNonZero(newmask);
			pIns->rect = cv::boundingRect(contour);
			//pIns->rrect = cv::fitEllipse(contours[0]);

			cv::Moments mu = cv::moments(contour, true);
			pIns->pt = cv::Point2f(static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
				static_cast<float>(mu.m01 / (mu.m00 + 1e-5)));

			//mask update
			insid++;
			pMask->FrameInstances.Update(insid, pIns);
			pMask->MapInstances.Update(insid, nullptr);
			pMask->GaussianMaps.Update(insid, nullptr);
			newBackground += (newmask / 255 * insid);

			if (idx == temp.rows)
				break;

			//임시로 이미지 시각화 할 때(삭제예정)
			nimg += newmask;
		}

		auto pBackIns = new ObjectSLAM::FrameInstance(pKF);
		pBackIns->mask = newBackground == 0; //pMask->mask
		pBackIns->area = cv::countNonZero(pBackIns->mask);
		pMask->FrameInstances.Update(0, pBackIns);
		pMask->MapInstances.Update(0, nullptr);
		//pMask->mask = newBackground.clone();

		//현재 박스 프레임의 인스턴스에 맵포인트를 연결
		//map point assignment
		auto pInstance = pMask->FrameInstances.Get();
		auto vpMPs = pKF->mvpMapPoints.get();
		//수정 필요
		
		for (auto pair : pInstance)
		{
			if (pair.first == 0)
				continue;
			auto p = pair.second;	
			p->Update(pKF);
		}
		
		pMask->mnMaxId = contours.size() + 1;
		pMask->mnOriSize = contours.size();
		pNewBF->mapMasks.Update("yoloseg", pMask);

	}
	pMask->mask = nimg.clone();
	cv::Mat orimask = pMask->mask.clone();


	//박스 연결
	//for (int j = 0; j < info.rows; j++) {
	//	int insid = j + 1;
	//	int label = (int)info.at<ushort>(j, 0);

	//	/*if (label == 60)
	//		continue;*/

	//	cv::Point2f left(info.at<ushort>(j, 1), info.at<ushort>(j, 2));
	//	cv::Point2f right(info.at<ushort>(j, 3), info.at<ushort>(j, 4));
	//	cv::Rect rect(left, right);

	//	pMask->info[insid] = std::make_pair(label, 0.0);
	//	pMask->rect[insid] = rect;
	//}
	//pMask->mnMaxId = info.rows + 1;
	
	//contour and moment
	
	std::map<int, bool> mapStaticObject;

	if (false) {
		cv::Mat newBackground = cv::Mat::zeros(h, w, CV_8UC1);
		cv::Mat curr = pNewBF->img.clone();
		std::string keyprocess = "contour and moment ";
		std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
		
		std::map<int, std::vector<cv::Point>> mapContours;

		ObjectSLAM::InstanceMask* pContourMask = new ObjectSLAM::InstanceMask();

		//for (auto pair : pMask->info) {
		//	auto iid = pair.first;
		//	if (iid == 0)
		//		std::cout << "???????????????????????" << std::endl;
		//	auto label = pair.second.first;
		//	//60은 다이닝 테이블
		//	if (label == 60){
		//		//std::cout << "dining table" << std::endl;
		//		continue;
		//	}
		//	cv::Mat mask = pMask->mask == iid;
		//	std::vector<std::vector<cv::Point>> contours;
		//	std::vector<cv::Vec4i> hierarchy;

		//	int mode = cv::RETR_EXTERNAL;//RETR_TREE;
		//	int method = cv::CHAIN_APPROX_SIMPLE;//CHAIN_APPROX_SIMPLE;
		//	cv::findContours(mask, contours, hierarchy, mode, method);

		//	double max_area = 0;
		//	int maxidx = -1;

		//	std::vector<cv::Moments> vmu(contours.size());
		//	for (int i = 0; i < contours.size(); i++)
		//	{
		//		vmu[i] = moments(contours[i], true);

		//		if (vmu[i].m00 < 100)
		//			continue;
		//		if (max_area < vmu[i].m00)
		//		{
		//			max_area = vmu[i].m00;
		//			maxidx = i;
		//		}
		//	}

		//	if (maxidx >= 0) {
		//		auto pIns = new ObjectSLAM::FrameInstance();
		//		pIns->contour = contours[maxidx];

		//		cv::Mat newmask = cv::Mat::zeros(h, w, CV_8UC1);;
		//		cv::drawContours(newmask, contours, maxidx, cv::Scalar(255, 255, 255), -1);
		//		pIns->mask = newmask.clone();
		//		pIns->area = cv::countNonZero(newmask);
		//		pIns->rect = cv::boundingRect(contours[maxidx]);
		//		//pIns->rrect = cv::fitEllipse(contours[0]);

		//		cv::Moments mu = vmu[maxidx];
		//		pIns->pt = cv::Point2f(static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
		//			static_cast<float>(mu.m01 / (mu.m00 + 1e-5)));

		//		pMask->FrameInstances.Update(iid, pIns);
		//		pMask->MapInstances.Update(iid, nullptr);
		//		newBackground += (newmask / 255 * iid);
		//	}
		//}

		//백그라운드, 스태틱, 인식이 안된 애들
		//기존 마스크에서 0이 아닌 경우가 있었음.
		auto pBackIns = new ObjectSLAM::FrameInstance(pKF);
		pBackIns->mask = newBackground == 0; //pMask->mask
		pBackIns->area = cv::countNonZero(pBackIns->mask);
		pMask->FrameInstances.Update(0, pBackIns);
		pMask->MapInstances.Update(0, nullptr);
		//pMask->mask = newBackground.clone();

		//현재 박스 프레임의 인스턴스에 맵포인트를 연결
		//map point assignment
		auto pInstance = pMask->FrameInstances.Get();
		auto vpMPs = pKF->mvpMapPoints.get();
		for (int i = 0; i < vpMPs.size(); i++)
		{
			auto mp = vpMPs[i];
			if (!mp || mp->isBad())
				continue;
			auto pt = pKF->mvKeys[i].pt;
			auto sid = pMask->mask.at<uchar>(pt);

			if (pMask->FrameInstances.Count(sid)){
				pInstance[sid]->setMPs.insert(mp);
				pInstance[sid]->setKPs.insert(i);

				/*if (pInstance[sid]->mpGlobal)
				{
					pInstance[sid]->mpGlobal->AllMapPoints.Update(mp);
				}*/
			}
			/*else
				std::cout << "error instance " << (int)sid <<" "<<info.rows<<" "<<pMask->instance.size() << std::endl;*/
		}

		pNewBF->mapMasks.Update("yoloseg", pMask);

		//large area static object : scene graph concept
		
		for (auto pair1 : pInstance) {
			int id1 = pair1.first;
			if (id1 == 0)
				continue;

			auto pIns1 = pair1.second;
			auto rect1 = pIns1->rect;
			bool bStatic = false;

			int nObject = 0;

			for (auto pair2 : pInstance) {
				int id2 = pair2.first;
				auto pIns2 = pair2.second;

				if (id1 == id2)
					continue;

				auto pt2 = pIns2->pt;
				if (rect1.contains(pt2))
				{
					nObject++;
				}
			}
			if (nObject > 2) {
				bStatic = true;
			}
			mapStaticObject[id1] = bStatic;
		}

		std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		auto du_seg = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
		
		//std::cout << "test yoloseg = " << id << " " << pMask->mnMaxId << " " << pMask->instance.Size() << std::endl;

		//std::cout << keyprocess << "== " << mapContours.size() << " " << pMask->info.size() << " == " << du_seg << std::endl;
		//SLAM->VisualizeImage(mapName, pMask->instance[1], 3);
	}
	
	bool bAsso = false;
	ObjectSLAM::InstanceMask* pPrevSeg = nullptr;
	ObjectSLAM::BoxFrame* pPrevBF = nullptr;
	////yoloseg(
	{
		pPrevBF = pNewBF->mpPrevBF;
		if (pPrevBF && pPrevBF->mapMasks.Count("yoloseg"))
		{
			pPrevSeg = pPrevBF->mapMasks.Get("yoloseg");

			ObjectSLAM::AssociationManager::AssociationWithMAP(
				SLAM, ObjSystem, "seg", id, mapName, user, pNewBF, pPrevBF, pPrevSeg, pMask, true);

		}
	}
	
	//if (pNewBF->mapMasks.Count("raft")) {
	//	auto pRaft = pNewBF->mapMasks.Get("raft");

	//	if (ObjSystem->MapKeyFrameNBoxFrame.Count(pRaft->id2))
	//	{
	//		pPrevBF = ObjSystem->MapKeyFrameNBoxFrame.Get(pRaft->id2);
	//		
	//		if (pPrevBF->mapMasks.Count("yoloseg"))
	//		{
	//			pPrevSeg = pPrevBF->mapMasks.Get("yoloseg");
	//		}

	//		if (pPrevSeg && !pMask->bInit)
	//		{
	//			bAsso = true;
	//			pMask->bInit = true;
	//			//
	//			ObjectSLAM::AssociationManager::AssociationWithMAP(
	//				SLAM, ObjSystem, "seg", id, mapName, user, pNewBF, pPrevBF, pPrevSeg, pMask, true);
	//			//AssociateMissingObject2(SLAM, ObjSystem, "seg", id, mapName, user, pNewBF, pPrevBF, pPrevSeg, pMask, pRaft, false);
	//		}

	//	}
	//}
	
	///////

	cv::Mat img = pNewBF->img.clone();

	//인스턴스 표현 필요
	//for (int y = 0; y < h; y++) {
	//	for (int x = 0; x < w; x++) {
	//		//바닥, 테이블, 벽 등은 스태틱으로 취급 맵포인트만 해도 됨.
	//		int sid = orimask.at<uchar>(y, x);
	//		if (sid < 1)
	//			continue;
	//		auto pt = cv::Point2f(x, y);
	//		segcolor.at<cv::Vec3b>(y, x) = SemanticSLAM::SemanticProcessor::SemanticColors[sid];
	//	}
	//}

	//cv::addWeighted(img, 0.3, segcolor, 0.7, 0.0, img);
	//시각화

	if (bVis) {
		//SLAM->VisualizeImage(mapName, nimg, 0);
		
		/*auto pFrames = pMask->FrameInstances.Get();
		for (auto pair : pFrames)
		{
			if (pair.first == 0)
				continue;
			int sid = pair.first;
			auto pIns = pair.second;
			std::vector<std::vector<cv::Point>> contours;
			contours.push_back(pIns->contour);
			cv::drawContours(img, contours, 0, SemanticSLAM::SemanticProcessor::SemanticColors[sid], 5);
		}
		std::stringstream ss;
		ss << "../res/aseg/" << id << "_a.png";
		cv::imwrite(ss.str(), img);*/
	}

	{
		std::stringstream ss;
		ss << "yoloseg::end," << id;
		ObjSystem->vecObjectAssoRes.push_back(ss.str());
	}

	if (bShow)
	{
		std::cout << "yoloseg end = " << id << std::endl;
	}

	User->mnUsed--;
}

void detectron2(EdgeSLAM::SLAM* SLAM, std::string user, int id) {
	
	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	User->mnUsed++;
	if (!ObjSystem->MapObjectDevices.Count(User)){
		User->mnUsed--;
		return;
	}
	auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
	std::string mapName = User->mapName;
	bool bSave = User->mbSave;
	bool bVis = User->GetVisID() == 0;

	auto pNewBF = ObjSystem->MapKeyFrameNBoxFrame.Get(id);//pObjDevice->mpCurrBF;
	auto pKF = pNewBF->mpRefKF;

	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss;
	ss << "/Download?keyword=detectron" << "&id=" << id << "&src=" << user;
	WebAPI API("143.248.6.143", 35005);
	auto res = API.Send(ss.str(), "");
	
	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	auto du_seg = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - pNewBF->t_start).count();
	auto du_down = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
	std::cout << "segmentation = " << du_seg <<", "<<du_down << std::endl;

	int n2 = res.size();

	cv::Mat temp = cv::Mat::zeros(n2, 1, CV_8UC1);
	std::memcpy(temp.data, res.data(), res.size());
	cv::Mat labeled = cv::imdecode(temp, cv::IMREAD_GRAYSCALE);

	pNewBF->InitInstance(labeled);
	//pNewBF->seg = labeled.clone();

	int min_id = 10000;
	int max_id = 0;

	//map point label association
	std::map<int, std::pair<int, cv::Point2f>> mapInstances;
	std::map<int, cv::Mat> sinfos;
	{
		std::stringstream ss;	
		ss << "/Download?keyword=detectron_info" << "&id=" << id << "&src=" << user;
		WebAPI API("143.248.6.143", 35005);
		auto res = API.Send(ss.str(), "");

		int n2 = res.size();
		int n = n2 / 16;

		cv::Mat sdata = cv::Mat::zeros(n, 4, CV_32FC1);
		std::memcpy(sdata.data, res.data(), res.size());
		for (int i = 0; i < n; i++) {
			int sid = (int)sdata.at<float>(i, 0);

			if (sid > max_id)
				max_id = sid;
			if (sid < min_id)
				min_id = sid;

			sinfos[sid] = sdata.row(i).clone();
			mapInstances[sid] = std::make_pair(0, cv::Point2f(0, 0));

			//인스턴스 생성. 일단 박스 클래스로. 추후 교체
			bool isthing = ((int)sinfos[sid].at<float>(3)) == 1 ? true : false;
			int label = (int)sinfos[sid].at<float>(1);
			float conf = sinfos[sid].at<float>(2);
			auto pBox = new ObjectSLAM::SegInstance(pNewBF, pKF->fx, pKF->fy, pKF->cx, pKF->cy, label, conf, isthing, User, true);
			pBox->SetPose(pNewBF->GetPose());
			pNewBF->mmpBBs[sid] = pBox;

			if (isthing) {
				pBox->mStrLabel = SemanticSLAM::SemanticProcessor::vecStrCOCO_ObjLabels[label];
			}
			else {
				pBox->mStrLabel = SemanticSLAM::SemanticProcessor::vecStrCOCOLabels[label];
			}
		}
		pNewBF->mnMaxID = max_id;
		//std::cout << "Detectron2::test::" << min_id << ", " << max_id << std::endl;
	}
	pNewBF->sinfos = sinfos;
	pNewBF->mbDetectron2 = true;


	int w = labeled.cols;
	int h = labeled.rows;
	
	cv::Mat img = pNewBF->img.clone();
		
	//키포인트에 시멘틱 정보 할당.
	for (int i = 0; i < pKF->N; i++) {
		
		int sid = labeled.at<uchar>(pKF->mvKeys[i].pt);
		cv::Mat sinfo = sinfos[sid];

		auto pIns = pNewBF->mmpBBs[sid];
		if(pIns->isTable() || pIns->isFloor())
		{ 
			continue;
		}

		if(pNewBF->mvnInsIDs[i] == -1)
			pNewBF->mvnInsIDs[i] = sid;
		else {
			//비교, 나중에 수정
		}
		
	}
	
	//시각화
	cv::Mat segcolor = cv::Mat::zeros(h, w, CV_8UC3);
	cv::Mat inscolor = cv::Mat::zeros(h, w, CV_8UC3);
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			int sid = labeled.at<uchar>(y, x);
			cv::Mat sinfo = sinfos[sid];

			if (!sinfos.count(sid))
			{
				std::cout << "info error" << std::endl;
			}
			auto pair = mapInstances[sid];
			pair.first++;
			pair.second += cv::Point2f(x, y);
			int label = (int)sinfo.at<float>(1);
			float score = sinfo.at<float>(2);
			bool isthing = sinfo.at<float>(3) == 1.0 ? true : false;
			
			segcolor.at<cv::Vec3b>(y, x) = SemanticSLAM::SemanticProcessor::SemanticColors[label];
			if((label == 60 && isthing) || (label ==42 && !isthing))
				inscolor.at<cv::Vec3b>(y, x) = SemanticSLAM::SemanticProcessor::SemanticColors[label];
			
			mapInstances[sid] = pair;
		}
	}
	//시각화

	//std::cout << "detectron2::3" << std::endl;
	//text 출력
	/*for (auto pair : mapInstances) {
		int sid = pair.first;
		int c = pair.second.first;
		auto pt = pair.second.second / c;
		
		cv::Mat sinfo = sinfos[sid];
		int label = (int)sinfo.at<float>(1);
		bool isthing = sinfo.at<float>(3) == 1.0 ? true : false;
		std::vector<std::string> vecStr = SemanticSLAM::SemanticProcessor::vecStrCOCOLabels;
		if(isthing){
			vecStr = SemanticSLAM ::SemanticProcessor::vecStrCOCO_ObjLabels;
			cv::putText(img, vecStr[label], pt, 2, 1.2, cv::Scalar(255, 255, 255));
		}
	}*/
	cv::addWeighted(img, 0.5, inscolor, 0.5, 0.0, img);

	//std::cout << "detectron2::4" << std::endl;

	auto pPrevBF = pObjDevice->mpPrevBF;
	//MappingFromSeg(pNewBF, pPrevBF, SLAM, User, pObjDevice, mapName);
	
	{
		pNewBF->mbInitialized = true;
		pNewBF->mpPrevBF = pObjDevice->mpCurrBF;
		pObjDevice->mpCurrBF = pNewBF;
	}

	//여기서 이닛되고 시스템 변경하기.

	//std::cout << "detectron2::5" << std::endl;

	//맵포인트 갱신
	//ObjSystem->UpdateMapPoint(pNewBF); 

	//std::cout << "detectron2::6" << std::endl;
	//for (int i = 0; i < pKF->N; i++) {
	//	auto pMPi = pKF->mvpMapPoints.get(i);
	//	if (!pMPi || pMPi->isBad())
	//		continue;
	//	int label = pMPi->mpConfLabel->label;
	//	auto color = SemanticSLAM::SemanticProcessor::SemanticColors[label];
	//	cv::circle(img, pKF->mvKeys[i].pt, 5, color, -1);
	//}
	
	if (bVis) {
		SLAM->VisualizeImage(mapName, img, 2);
		//SLAM->VisualizeImage(mapName, inscolor, 1);
	}

	User->mnUsed--;

	/*if (bSave) {
		ss.str("");
		ss << "../res/images/" << id << "_detectron.png";
		cv::imwrite(ss.str(), segcolor);
	}*/
}

void Segmentation(EdgeSLAM::SLAM* SLAM, std::string user, int id) {
	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	if (!User->KeyFrames.Count(id))
		return;
	auto pKF = User->KeyFrames.Get(id);
	if (!pKF) {
		return;
	}
	if (!AllBoxFrames.Count(pKF))
	{
		return;
	}
	if (!User->ImageDatas.Count(id)) {
		std::cout << "image error" << std::endl;
		return;
	}
	
	if (!User->DepthDatas.Count(id))
	{
		std::cout << "depth error" << std::endl;
		return;
	}
	cv::Mat encoded = User->ImageDatas.Get(id);
	cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
	if (image.empty())
	{
		std::cout << "decoding error" << std::endl;
		return;
	}
	cv::Mat dencoded = User->DepthDatas.Get(id);
	cv::Mat depthimg = cv::imdecode(dencoded, cv::IMREAD_ANYDEPTH); //16U, ushort
	if (depthimg.empty())
	{
		std::cout << "depth decoding error" << std::endl;
		return;
	}
	cv::Mat imDepth;
	depthimg.convertTo(imDepth, CV_32F, User->mpCamera->mDepthMapFactor);
	
	User->mnUsed++;

	std::stringstream ss;
	ss << "/Download?keyword=Segmentation" << "&id=" << id << "&src=" << user;
	WebAPI API("143.248.6.143", 35005);
	auto res = API.Send(ss.str(), "");
	cv::Mat temp = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());
	cv::Mat labeled = cv::imdecode(temp, cv::IMREAD_GRAYSCALE);
	auto pBF = AllBoxFrames.Get(pKF);

	int w = labeled.cols;
	int h = labeled.rows;
	//세그멘테이션 시각화
	cv::Mat segcolor = cv::Mat::zeros(h, w, CV_8UC3);
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			int label = labeled.at<uchar>(y, x) + 1;
			segcolor.at<cv::Vec3b>(y, x) = SemanticSLAM::SemanticProcessor::SemanticColors[label];
		}
	}
	//바운딩 박스 시각화
	for (int i = 0; i < pBF->mvpBBs.size(); i++)
		cv::rectangle(image, pBF->mvpBBs[i]->mRect, cv::Scalar(255, 255, 255), 2);

	//박스 프레임에 이미지, 뎁스, 세그멘테이션 연결
	pBF->img = image.clone();
	pBF->depth = imDepth.clone();
	pBF->labeled = labeled.clone();

	//포인트 생성
	cv::Mat gray;
	cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAY		
	EdgeSLAM::FeatureTracker* mpTracker = new EdgeSLAM::FlannFeatureTracker(200);
	std::vector<cv::KeyPoint> vecTotalKPs;
	for (int i = 0; i < pBF->mvpBBs.size(); i++) {
		auto pBBox = pBF->mvpBBs[i];
		/*if (pBBox->mvKeyPoints.size() > 20)
			continue;*/
		cv::Mat mask = cv::Mat::zeros(h, w, CV_8UC1);
		cv::rectangle(mask, pBBox->mRect, cv::Scalar(255, 255, 255), -1);

		if (pBBox->mRect.width < 17 || pBBox->mRect.height < 17)
			continue;

		std::vector<cv::KeyPoint> tempKPs;
		cv::Mat tempDesc;
		//detector->detect(gray, tempKPs, mask);
		mpTracker->detectAndCompute(gray(pBBox->mRect), cv::Mat(), tempKPs, tempDesc);
		
		//박스 프레임에 추가
		//키포인트
		//뎁스
		//mvuRight
		//desc 등

		for (int j = 0; j < tempKPs.size(); j++) {
			tempKPs[j].pt += cv::Point2f(pBBox->mRect.x, pBBox->mRect.y);
			pBF->AddData(tempKPs[j], tempDesc.row(j));
			//cv::circle(image, tempKPs[j].pt, 3, cv::Scalar(0, 255, 0), -1);
			//vecTotalKPs.push_back(tempKPs[j]);
		}
	}
	pBF->SetPose(pKF->GetPose());
	pBF->Init();
	delete mpTracker;

	//바운딩 박스에 정보 연결

	//키포인트와 맵포인트 연결
	bool bDepthSensor = pKF->mpCamera->mCamSensor == BaseSLAM::CameraSensor::RGBD;

	if(false){
		std::vector<std::pair<cv::Mat, cv::Mat>> tempObjXs;
		std::vector<int> vecOctaves;
		std::map<int, cv::Mat> contentDatas;
		std::vector<float> vecCounts(pBF->mvpBBs.size(),0);
		std::vector<cv::Mat> vecXavgs(pBF->mvpBBs.size());
		for (int i = 0; i < pBF->mvpBBs.size(); i++) {
			vecXavgs[i] = cv::Mat::zeros(3, 1, CV_32FC1);
		}
	}
	if(false)
	for (int k = 0, kend = pKF->N; k < kend; k++) {

		auto pt = pKF->mvKeys[k].pt;
		int label = labeled.at<uchar>(pt) + 1;
		if (label == (int)StructureLabel::FLOOR)
			continue;
		if (label == (int)StructureLabel::WALL)
			continue;
		if (label == (int)StructureLabel::CEIL)
			continue;
		if (label == (int)StructureLabel::BUILDING)
			continue;
		//테이블 추가

		//박스에 데이터 추가
		for (int i = 0; i < pBF->mvpBBs.size(); i++) {
			auto pBBox = pBF->mvpBBs[i];
			if (!pBBox->mRect.contains(pt))
				continue;
			auto pMP = pKF->GetMapPoint(k);
			if (!pMP || pMP->isBad()) {
				pMP = nullptr;
				cv::circle(image, pKF->mvKeys[k].pt, 3, cv::Scalar(0, 0, 255), -1);
			}
			else {
				cv::circle(image, pKF->mvKeys[k].pt, 3, cv::Scalar(255, 0, 0), -1);
			}

			size_t idx = pBBox->mvKeyDatas.size();
			cv::Mat row = pKF->mDescriptors.row(k);
			pBBox->mvIDXs.push_back(k);
			pBBox->mapIDXs[k] = idx;
			pBBox->mvKeyDatas.push_back(pKF->mvKeys[k]);
			pBBox->mvKeyDataUns.push_back(pKF->mvKeysUn[k]);
			pBBox->mDescriptors.push_back(row.clone());
			pBBox->mvpMapPoints.push_back(pMP);
			pBBox->mpWorldPose->SetPose(pKF->GetPose());
			if (bDepthSensor) {
				pBBox->mvDepth.push_back(pKF->mvDepth[k]);

				/*if (pKF->mvDepth[k] > 0) {
					cv::Mat Xw, Xo;
					pKF->UnprojectStereo(k, Xw, Xo, Rwc, twc);
					pBBox->mvWorld.push_back(Xw);
					pBBox->mvObject.push_back(Xo);
					vecXavgs[i] += Xw;
					vecCounts[i]++;
					tempObjXs.push_back(std::make_pair(Xw, row));
					vecOctaves.push_back(pKF->mvKeysUn[k].octave);
				}
				else {
					pBBox->mvWorld.push_back(cv::Mat());
					pBBox->mvObject.push_back(cv::Mat());
				}*/
			}
		}
	}

	//객체 맵 생성
	{
		ObjectSLAM::ObjectInitializer* mpObjInit = new ObjectSLAM::ObjectInitializer();
		int nid = 0;
		for (int i = 0; i < pBF->mvpBBs.size(); i++) {
			ObjectSLAM::ObjectMap* pNewMap = mpObjInit->StereoInitialization(pBF->mvpBBs[i]);
			//bool b = mpObjInit->StereoInitialization(pBF->mvpBoundingBoxs[i], pNewMap);
			if (pNewMap) {
				ObjSystem->ObjectMaps.Update(nid++, pNewMap);
			}
		}
		delete mpObjInit;
	}

	//레퍼런스 프레임과 비교

	if(false)
	if (ObjSystem->ObjectMaps.Size() == 0) {
		ObjectSLAM::ObjectInitializer* mpObjInit = new ObjectSLAM::ObjectInitializer();
		int nid = 0;
		for (int i = 0; i < pBF->mvpBBs.size(); i++) {
			ObjectSLAM::ObjectMap* pNewMap = mpObjInit->StereoInitialization(pBF->mvpBBs[i]);
			//bool b = mpObjInit->StereoInitialization(pBF->mvpBoundingBoxs[i], pNewMap);
			if (pNewMap) {
				ObjSystem->ObjectMaps.Update(nid++, pNewMap);
			}
		}
	} else{
		//매핑
		//바운딩 박스 연결
		//새로운 맵포인트 생성 등
		//겹치는 맵 연결도 해야 함.


		//맵과 바운딩 박스 연결
		//맵의 3차원 위치와 바운딩 박스.
		cv::Mat Rcw = pKF->GetRotation();
		cv::Mat tcw = pKF->GetTranslation();
		cv::Mat K = pKF->K.clone();
		auto vecMaps = ObjSystem->ObjectMaps.Get();
		for (auto iter = vecMaps.begin(), iend = vecMaps.end(); iter != iend; iter++) {
			auto pObjMap = iter->second;
			cv::Mat Tobj = pObjMap->mpWorldPose->GetPose();
			cv::Mat Robj = Tobj.rowRange(0, 3).colRange(0,3);
			cv::Mat tobj = Tobj.rowRange(0, 3).col(3);
			cv::Mat proj = K* (Rcw*tobj+tcw);

			float d = proj.at<float>(2);
			cv::Point2f pt(proj.at<float>(0) / d, proj.at<float>(1) / d);
			if (pt.x <= 0.0 || pt.x >= w || pt.y <= 0.0 || pt.y >= h || d < 0.0)
				continue;

			//박스에 프로젝션
			for (int i = 0; i < pBF->mvpBBs.size(); i++) {
				auto pNeighBB = pBF->mvpBBs[i];
				if (pNeighBB->mRect.contains(pt)) {
					//매칭 
				}
			}

			auto pBB = pObjMap->vecBoundingBoxes.get(0);
			if (!pBB)
				continue;
			std::vector<std::pair<int, int>> vecMatches;
			int Na = ObjectSLAM::ObjectMatcher::SearchFrameByProjection(pBB, pBF, vecMatches);
			for (int i = 0, N = vecMatches.size(); i < N; i++) {
				int idx1 = vecMatches[i].first;
				int idx2 = vecMatches[i].second;

				cv::Mat X = pBB->mvWorld[idx1];
				cv::Mat proj = K * (Rcw * X + tcw);
				float d = proj.at<float>(2);
				cv::Point2f pt(proj.at<float>(0) / d, proj.at<float>(1) / d);
				cv::circle(image, pt, 2, cv::Scalar(255, 255, 0), -1);
				//cv::line(refimage, pt, pRefKF->mvKeys[idx2].pt, cv::Scalar(0, 0, 255), 2);
			}
			/*for (int i = 0, N = pBB->mvWorld.size(); i < N; i++) {
				if (pBB->mvDepth[i] < 0)
					continue;
				
				cv::Mat X = pBB->mvWorld[i].clone();
				cv::Mat proj = K * (Rcw * X + tcw);
				float d = proj.at<float>(2);

				cv::Mat proj2 = Rcw * X - tobj;
				float d2 = proj2.at<float>(2);

				cv::Point2f pt(proj.at<float>(0) / d, proj.at<float>(1) / d);

				
				cv::circle(image, pt, 4, cv::Scalar(255, 255, 0), 1);
				if (d2 < 0.0)
					cv::circle(image, pt, 3, cv::Scalar(255, 0, 255), -1);

			}
			cv::circle(image, pt, 15, cv::Scalar(255, 255, 255), 2);*/
		}
	}


	
	/*for (int i = 0; i < pBF->mvpBoundingBoxs.size(); i++) {
		cv::Mat X = vecXavgs[i];
		float n = vecCounts[i];
		if (n > 0) {
			X /= n;
			contentDatas[pBF->mvpBoundingBoxs[i]->mnLabel] = X;
		}
	}
	SLAM->TemporalDatas2.Update("objnode", contentDatas);*/

	//이전 프레임에서 비교
	auto pRefKF = pKF->GetParent();
	if (pRefKF) {
		cv::Mat K = pKF->K;
		cv::Mat Rcw = pKF->GetRotation();
		cv::Mat tcw = pKF->GetTranslation();

		if (AllBoxFrames.Count(pRefKF))
		{

			auto pRefBF = AllBoxFrames.Get(pRefKF);
			for (int j = 0; j < pRefBF->mvpBBs.size(); j++){
				auto pRefBox = pRefBF->mvpBBs[j];

				for (int i = 0; i < pRefBox->mvWorld.size(); i++) {
					if (pRefBox->mvDepth[i] < 0.0)
						continue;
					cv::Mat X = pRefBox->mvWorld[i];
					cv::Mat proj = K * (Rcw * X + tcw);
					float d = proj.at<float>(2);
					cv::Point2f pt(proj.at<float>(0) / d, proj.at<float>(1) / d);
					cv::circle(image, pt, 2, cv::Scalar(255, 255, 0), -1);
				}

				std::vector<std::pair<int, int>> vecMatches;
				int Na = ObjectSLAM::ObjectMatcher::SearchFrameByProjection(pRefBox, pBF, vecMatches, 5);
				for (int i = 0, N = vecMatches.size(); i < N; i++) {
					int idx1 = vecMatches[i].first;
					int idx2 = vecMatches[i].second;

					cv::Mat X = pRefBox->mvWorld[idx1];
					cv::Mat proj = K * (Rcw * X + tcw);
					float d = proj.at<float>(2);
					cv::Point2f pt(proj.at<float>(0) / d, proj.at<float>(1) / d);
					cv::circle(image, pt, 2, cv::Scalar(255, 255, 0), -1);
					cv::line(image, pt, pBF->mvKeyDatas[idx2].pt, cv::Scalar(0, 0, 255), 2);
				}
			}
		}
		//int refId = pRefKF->mnFrameId;
		//cv::Mat Rcw = pRefKF->GetRotation();
		//cv::Mat tcw = pRefKF->GetTranslation();

		//cv::Mat K = pKF->K.clone();
		//cv::Mat encoded = User->ImageDatas.Get(refId);
		//cv::Mat refimage = cv::imdecode(encoded, cv::IMREAD_COLOR);
		//if (!refimage.empty())
		//{
		//	//처리한 결과과 있는지도 확인하기
		//	if (AllBoxFrames.Count(pRefKF))
		//	{
		//		auto pRefBF = AllBoxFrames.Get(pRefKF);
		//		for (int i = 0; i < pRefBF->mvpBoundingBoxs.size(); i++){
		//			cv::rectangle(refimage, pRefBF->mvpBoundingBoxs[i]->mRect, cv::Scalar(255, 255, 255), 2);
		//			auto pRefBox = pRefBF->mvpBoundingBoxs[i];
		//			for (int j = 0, Nj = pRefBox->mvKPs.size(); j < Nj; j++)
		//				cv::circle(refimage, pRefBox->mvKPs[j].pt, 3, cv::Scalar(255, 0, 0), -1);
		//		}
		//	}

		//	//옵티컬 플로우
		//	//마스킹 포인트 검출
		//	cv::Mat gray, grayRef;
		//	cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAY		
		//	cv::cvtColor(refimage, grayRef, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAYs

		//	//레퍼런스 프레임 바운딩 박스 시각화
		//	cv::Mat mask = cv::Mat::zeros(h, w, CV_8UC1);
		//	for (int i = 0; i < pBF->mvpBoundingBoxs.size(); i++) {
		//		auto pBBox = pBF->mvpBoundingBoxs[i];
		//		cv::rectangle(mask, pBBox->mRect, cv::Scalar(255, 255, 255), -1);
		//	}

		//	//std::vector<cv::Point2f> corners1, corners2;
		//	//std::vector<uchar> features_found;
		//	//int win_size = 10;
		//	//cv::goodFeaturesToTrack(gray, corners1, 50 * pBF->mvpBoundingBoxs.size(), 0.3, 7, mask, 7);

		//	//cv::calcOpticalFlowPyrLK(
		//	//	gray,                         // Previous image
		//	//	grayRef,                         // Next image
		//	//	corners1,                     // Previous set of corners (from imgA)
		//	//	corners2,                     // Next set of corners (from imgB)
		//	//	features_found,               // Output vector, each is 1 for tracked
		//	//	cv::noArray(),                // Output vector, lists errors (optional)
		//	//	cv::Size(win_size * 2 + 1, win_size * 2 + 1),  // Search window size
		//	//	5,                            // Maximum pyramid level to construct
		//	//	cv::TermCriteria(
		//	//		cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
		//	//		20,                         // Maximum number of iterations
		//	//		0.3                         // Minimum change per iteration
		//	//	)
		//	//);

		//	//for (int i = 0; i < static_cast<int>(corners1.size()); ++i) {
		//	//	if (!features_found[i]) {
		//	//		continue;
		//	//	}

		//	//	line(
		//	//		refimage,                         // Draw onto this image
		//	//		corners1[i],                 // Starting here
		//	//		corners2[i],                 // Ending here
		//	//		cv::Scalar(255, 255, 0),       // This color
		//	//		3,                           // This many pixels wide
		//	//		cv::LINE_AA                  // Draw line in this style
		//	//	);
		//	//}

		//	////매칭
		//	std::vector<std::pair<int, int>> vecMatches;
		//	EdgeSLAM::SearchPoints::SearchFrameByProjection(tempObjXs, vecOctaves, pRefKF, vecMatches, 100, 50, 2);

		//	////시각화
		//	for (int i = 0; i < tempObjXs.size(); i++) {
		//		cv::Mat X = tempObjXs[i].first.clone();
		//		cv::Mat proj = K * (Rcw * X + tcw);
		//		float d = proj.at<float>(2);
		//		cv::Point2f pt(proj.at<float>(0) / d, proj.at<float>(1) / d);
		//		cv::circle(refimage, pt, 2, cv::Scalar(0, 255, 255), -1);
		//	}
		//	for (int i = 0, N = vecMatches.size(); i < N; i++) {
		//		int idx1 = vecMatches[i].first;
		//		int idx2 = vecMatches[i].second;

		//		cv::Mat X = tempObjXs[idx1].first.clone();
		//		cv::Mat proj = K * (Rcw * X + tcw);
		//		float d = proj.at<float>(2);
		//		cv::Point2f pt(proj.at<float>(0) / d, proj.at<float>(1) / d);
		//		cv::circle(refimage, pt, 2, cv::Scalar(255, 255, 0), -1);
		//		//cv::line(refimage, pt, pRefKF->mvKeys[idx2].pt, cv::Scalar(0, 0, 255), 2);
		//	}

		//	SLAM->VisualizeImage(User->mapName, refimage, 2);
		//}
	}

	//포인트 생성
	//cv::Ptr<cv::ORB> detector = cv::ORB::create(100);
	//EdgeSLAM::FeatureTracker* mpTracker = new EdgeSLAM::FlannFeatureTracker(200);
	//std::vector<cv::KeyPoint> vecTotalKPs;
	//for (int i = 0; i < pBF->mvpBoundingBoxs.size(); i++) {
	//	auto pBBox = pBF->mvpBoundingBoxs[i];
	//	/*if (pBBox->mvKeyPoints.size() > 20)
	//		continue;*/
	//	cv::Mat mask = cv::Mat::zeros(h, w, CV_8UC1);
	//	cv::rectangle(mask, pBBox->mRect, cv::Scalar(255, 255, 255), -1);

	//	if (pBBox->mRect.width < 17 || pBBox->mRect.height < 17)
	//		continue;

	//	std::vector<cv::KeyPoint> tempKPs;
	//	cv::Mat tempDesc;
	//	//detector->detect(gray, tempKPs, mask);
	//	mpTracker->detectAndCompute(gray(pBBox->mRect), cv::Mat(), tempKPs, tempDesc);
	//	
	//	for (int j = 0; j < tempKPs.size(); j++) {

	//		cv::circle(image, tempKPs[j].pt+cv::Point2f(pBBox->mRect.x,pBBox->mRect.y), 3, cv::Scalar(0, 255, 0), -1);
	//		vecTotalKPs.push_back(tempKPs[j]);
	//	}
	//}
	//delete mpTracker;
	// 
	//cv::Mat desc;
	//SLAM->mpFeatureTracker->computeDescriptor(gray, vecTotalKPs, desc);

	if (User->GetVisID() == 0) {
		SLAM->VisualizeImage(User->mapName, segcolor, 1);
		SLAM->VisualizeImage(User->mapName, image, 0);
		/*std::stringstream ss;
		ss << "../res/images/" <<id << "_mask.png";
		cv::imwrite(ss.str(), segcolor);*/
	}

	User->mnUsed--;
}

//void TrackObject2(EdgeSLAM::User* User, int id, std::map<ObjectSLAM::GlobalInstance*, cv::Rect>& _map) {
void TrackObject2(EdgeSLAM::User* User, int id, std::promise<std::map<ObjectSLAM::GlobalInstance*, cv::Rect>>* _map) {
	std::string keyprocess = "TrackObject ===";
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

	bool bShow = false;
	std::map<ObjectSLAM::GlobalInstance*, cv::Rect> mapGlobalRects;

	if (bSemantic && ObjSystem->MapObjectDevices.Count(User)) {
		auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);

		auto pCurrBF = pObjDevice->mpCurrBF;
		if (!User->mbMapping)
		{
			auto pKF = User->mpRefKF;
			if (pKF)
			{
				auto vpNeighBFs = ObjSystem->GetConnectedBoxFrames(pKF, 20);
				if (vpNeighBFs.size() > 0)
				{
					pCurrBF = vpNeighBFs[0];
				}
				if (bShow && !pCurrBF)
					std::cout << vpNeighBFs.size() << "???????????????????????" << std::endl;
			}
		}
		if (pCurrBF) {
			std::set<ObjectSLAM::GlobalInstance*> spGlobals;
			pCurrBF->GetNeighGlobalInstnace(spGlobals);
			//auto pKF = User->mpRefKF;

			for (auto pG : spGlobals)
			{
				if (!pG)
					continue;

				//센터와 평균 렉트를 이용하자.
				int n = 0;
				int rw = 0;
				int rh = 0;
				
				auto mapConnected = pG->mapConnected.Get();
				if (mapConnected.size() < 2)
					continue;
				for (auto pair1 : mapConnected)
				{
					auto pBF1 = pair1.first;
					int id1 = pair1.second;
					if (!pBF1->mapMasks.Count("yoloseg")) {
						std::stringstream ss;
						ss << "Global::Error::Mask," << pBF1->mnId << "," << id1;
						ObjSystem->vecObjectAssoRes.push_back(ss.str());
						continue;
					}
					auto pTempMask = pBF1->mapMasks.Get("yoloseg");
					if (!pTempMask->FrameInstances.Count(id1)) {
						std::stringstream ss;
						ss << "Global::Error::Frame," << pBF1->mnId << "," << id1;
						ObjSystem->vecObjectAssoRes.push_back(ss.str());
						continue;
					}
					auto pIns1 = pTempMask->FrameInstances.Get(id1);
					if (!pIns1) {
						std::stringstream ss;
						ss << "Global::Error::Instance," << pBF1->mnId << "," << id1;
						ObjSystem->vecObjectAssoRes.push_back(ss.str());
						continue;
					}
					rw += pIns1->rect.width;
					rh += pIns1->rect.height;
					n++;
				}

				if (n > 0)
				{
					rw /= n;
					rh /= n;
					//int rx = pt.x - rw / 2;
					//int ry = pt.y - rh / 2;
					auto rect = cv::Rect(0,0, rw, rh);
					mapGlobalRects[pG] = rect;
				}
			}

		}
	}

	_map->set_value(mapGlobalRects);

	if (bShow) {
		std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		auto du_seg = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
		std::cout << keyprocess << "== " << User ->strDeviceName<<", " << id << " == " << du_seg << "== " << mapGlobalRects.size() << std::endl;
	}
}

void TrackObject(EdgeSLAM::User* User,int id, EdgeSLAM::Frame* frame, const cv::Mat& gray, std::promise<int>* refid) {

	/*
		mpRef는 이전이라고 하더라도 아직 세그멘테이션이 안된 경우가 있음.
		세그멘테이션이 없는 경우에는 이전 키프레임에 접근할 수 있어야 함.
		인스턴스 센터로이드 측정 결과 확인하기.

		샘2가 안된 경우도 있음. 필요 없는건지 아직 안온건지 확인이 필요함.
	*/

	bool bShow = false;
	std::string keyprocess = "TrackObject ===";
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

	//refid = -1;
	int rid = -1;
	if (bSemantic && ObjSystem->MapObjectDevices.Count(User)) {
		auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);

		auto pCurrBF = pObjDevice->mpCurrBF;
		if (pCurrBF) {
			std::set<ObjectSLAM::GlobalInstance*> spGlobals;
			pCurrBF->GetNeighGlobalInstnace(spGlobals);
			auto pKF = User->mpRefKF;

			for (auto pG : spGlobals)
			{
				if (!pG)
					continue;
				auto mapConnected = pG->mapConnected.Get();
				if (mapConnected.size() < 2)
					continue;
				for (auto pair : mapConnected)
				{
					auto pPrevBF = pair.first;
					auto idx = pair.second;
				}
			}

		}

		auto pKF = User->mpRefKF;
		//pKF
		if (false) {

			if (id == pKF->mnFrameId)
			{
				std::cout << "object matching test = " << id << " " << std::endl;
			}

			if (ObjSystem->MapKeyFrameNBoxFrame.Count(pKF->mnFrameId)) {
				auto pBF = ObjSystem->MapKeyFrameNBoxFrame.Get(pKF->mnFrameId);
				


				bool bProcess = false; //true여야 optical flow matching test

				{
					//static object test
					auto vpNeighBFs = ObjSystem->GetConnectedBoxFrames(pKF, 10);
					auto vpNeighKFs = pKF->GetBestCovisibilityKeyFrames(10);

					std::set<ObjectSLAM::FrameInstance*> spNeighLocalIns;
					std::set<ObjectSLAM::GlobalInstance*> spNeighGlobalIns;
					std::set<ObjectSLAM::BoxFrame*> spNeighBFs;

					for (auto pBF : vpNeighBFs) { 
				     		if (pBF->mapMasks.Count("yoloseg"))
						{
							spNeighBFs.insert(pBF);
							auto mapIns = pBF->mapMasks.Get("yoloseg")->MapInstances.Get();

							for (auto pair : mapIns)
							{
								
								auto id = pair.first;
								auto pIns = pair.second;
								if (pIns)
								{
									if (!spNeighGlobalIns.count(pIns))
										spNeighGlobalIns.insert(pIns);
								}
							}
						}
					}

					/*for (auto pGlobal1 : spNeighGlobalIns)
					{
						auto mapLocalIns1 = pGlobal1->mapConnected.Get();
						for (auto pGlobal2 : spNeighGlobalIns)
						{
							if (pGlobal1 == pGlobal2)
								continue;
							auto mapLocalIns2 = pGlobal2->mapConnected.Get();

							for (auto pair1 : mapLocalIns1)
							{
								auto pBF1 = pair1.first;
								int id1 = pair1.second;
								for (auto pair2 : mapLocalIns2)
								{
									auto pBF2 = pair2.first;
									int id2 = pair2.second;

									if (pBF1 == pBF2)
										continue;
									auto pIns1 = pBF1->mapMasks.Get("yoloseg")->instance.Get(id1);
									auto pIns2 = pBF2->mapMasks.Get("yoloseg")->instance.Get(id2);
									float val = ObjectSLAM::InstanceSim::ComputeSimFromPartialMP(pIns1, pIns2);
									if (val > 0.5)
										std::cout << "test val = " << val << std::endl;
								}
							}
						}
					}*/

				}


				//if (pBF->mapMasks.Count("yoloseg"))
				//{
				//}
				//else {
				//	pBF = pBF->mpPrevBF;
				//	pKF = pBF->mpRefKF;
				//	if (pBF)
				//	{
				//		if (pBF->mapMasks.Count("yoloseg"))
				//		{
				//			
				//		}
				//		else {
				//			std::cout << "track object : fail prev segmentation" << std::endl;
				//			bProcess = false;
				//		}
				//	}
				//	else {
				//		std::cout << "track object : fail prev box frame" << std::endl;
				//		bProcess = false;
				//	}
				//	if (!pKF)
				//	{
				//		bProcess = false;
				//	}
				//}
				
				//무조건 이전 프레임과 비교
				pBF = pBF->mpPrevBF;
				
				if (pBF)
				{
					if (pBF->mapMasks.Count("yoloseg"))
					{

					}
					else {
						//std::cout << "track object : fail prev segmentation" << std::endl;
						bProcess = false;
					}

					pKF = pBF->mpRefKF;
					if (!pKF)
					{
						bProcess = false;
					}
				}
				else {
					//std::cout << "track object : fail prev box frame" << std::endl;
					bProcess = false;
				}

				if (bProcess) {
					auto pCurrSeg = pBF->mapMasks.Get("yoloseg");
					bool bSam2 = pBF->mapMasks.Count("sam2");

					int n = pCurrSeg->FrameInstances.Size();
					//std::cout << "track object :: instance = " << n << " " << bSam2 << std::endl;

					//asdf
					ObjectSLAM::ObjectMatchingInfo* pMatches = new ObjectSLAM::ObjectMatchingInfo(id, pKF->mnFrameId);
					auto nres = ObjectSLAM::ObjectMatcher::SearchInstance(frame, pBF, gray, pBF->gray, pMatches);
					ObjSystem->SetMatchInfo(id, pKF->mnFrameId, pMatches);

					//asdf
					/*ObjectSLAM::ObjectMatchingInfo* pMatches = new ObjectSLAM::ObjectMatchingInfo(id, pKF->mnFrameId);
					auto nres = ObjectSLAM::ObjectMatcher::SearchByOpticalFlow(frame, pKF, gray, pBF->gray, pMatches);
					ObjSystem->SetMatchInfo(id, pKF->mnFrameId, pMatches);*/

					rid = pKF->mnFrameId;
				}
			}
			else {
				//std::cout << "tarck object : fail find reference box frame" << std::endl;
			}
		}
		else
		{
			//std::cout << "tarck object : fail find reference keyframe" << std::endl;
		}
	}
	refid->set_value(rid);

	if (bShow) {
		std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		auto du_seg = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
		std::cout << keyprocess << "== " << id << " == " << du_seg << " " << std::endl;
	}
}

void TrackPLP(ThreadPool::ThreadPool* POOL, EdgeSLAM::SLAM* eSLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {
	//StructOptimization::StructMap* tempMap = (StructOptimization::StructMap*)SLAM->GetMap("testmap");
	//auto device = tempMap->GetDevice(src);
	//device->Acquire();

	//WebAPI API(ip, port);
	//std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
	//std::stringstream ss;
	//ss << "/Download?keyword=" << "SImage" << "&id=" << id << "&src=" << src;
	//auto res = API.Send(ss.str(), "");
	//std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();
	//cv::Mat temp = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());
	//cv::Mat img = cv::imdecode(temp, cv::IMREAD_COLOR);
	//cv::Mat visImg = img.clone();

	//cv::Mat gray;
	//cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAY
	//StructOptimization::SimpleFrame* pNewSF = new StructOptimization::SimpleFrame(id, device, gray);
	//pNewSF->mnProgress++;
	//bool bNeedNewKF = false;
	//StructOptimization::StructTracker::Track(POOL, SLAM, device, pNewSF, tempMap, bNeedNewKF);

	////prev
	//pNewSF->mnProgress--;
	//device->Release();
}
void TrackStereo(ThreadPool::ThreadPool* POOL, EdgeSLAM::SLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {
	if (!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++;

	int nVisID = User->GetVisID() + 4;
	std::string mapName = User->mapName;

	WebAPI API(ip, port);
	std::stringstream ss;
	ss << "/Download?keyword=" << "LImage" << "&id=" << id << "&src=" << src;
	auto res = API.Send(ss.str(), "");
	cv::Mat temp = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());
	cv::Mat img1 = cv::imdecode(temp, cv::IMREAD_COLOR);
	cv::Mat visImg1 = img1.clone();

	ss.str("");
	ss << "/Download?keyword=" << "RImage" << "&id=" << id << "&src=" << src;
	auto res2 = API.Send(ss.str(), "");
	cv::Mat temp2 = cv::Mat(res2.size(), 1, CV_8UC1, (void*)res2.data());
	cv::Mat img2 = cv::imdecode(temp2, cv::IMREAD_COLOR);
	cv::Mat visImg2 = img2.clone();

	User->ImageDatas.Update(id, temp.clone());

	cv::Mat gray1, gray2;
	cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAY
	cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAY
	EdgeSLAM::Frame* frame = new EdgeSLAM::Frame(gray1, gray2, User->mpCamera, id, 0.0, frame_ts);
	frame->mnShared++;

	EdgeSLAM::Tracker::Track(POOL, SLAM, id, src, frame, frame_ts);

	auto pRefKF = User->mpRefKF;
	auto trackStat = User->GetState();
	bool bTrackSuccess = trackStat == EdgeSLAM::UserState::Success && frame->mvpMapPoints.size() == frame->N;
	bool bCommuTest = User->mbCommuTest;
	int nContentKFs = User->mnContentKFs;
	bool bMapping = User->mbMapping;

	for (int i = 0; i < frame->mvKeys.size(); i++)
	{
		auto pt = frame->mvKeys[i].pt;
		cv::circle(visImg1, pt, 2, cv::Scalar(0, 255, 0), -1);
	}
	for (int i = 0; i < frame->mvRightKeys.size(); i++)
	{
		auto pt = frame->mvRightKeys[i].pt;
		cv::circle(visImg2, pt, 2, cv::Scalar(0, 255, 0), -1);
	}

	if (User->mbNewKF) {
		User->mbNewKF = false;
		std::string nsrc = src + ".LImage";
		Utils::SendReqMessage("reqdistdepth", src, id);
		Utils::SendReqMessage("RequestObjectDetection", nsrc, id);
		Utils::SendReqMessage("RequestSegmentation", nsrc, id);
	}
	
	/*if (bTrackSuccess) {

		for (int i = 0, N = frame->mvpMapPoints.size(); i < N; i++) {
			auto pMPi = frame->mvpMapPoints[i];
			if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i])
				continue;
			auto pt = frame->mvKeys[i].pt;
			cv::circle(visImg1, pt, 3, cv::Scalar(0, 255, 0), 1);
		}
	}
	else {
		cv::Mat failImg = cv::Mat::zeros(img1.size(), CV_8UC3);
		failImg = cv::Scalar(0, 0, 255);
		cv::addWeighted(img1, 0.7, failImg, 0.3, 0.0, visImg1);
	}*/

	SLAM->VisualizeImage(mapName, visImg1, nVisID);
	SLAM->VisualizeImage(mapName, visImg2, nVisID+1);
	frame->mnShared--;
	User->mnUsed--;
}
void TrackRGBD(ThreadPool::ThreadPool* POOL, EdgeSLAM::SLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {
	if (!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++;

	int nVisID = User->GetVisID() + 4;
	std::string mapName = User->mapName;


	WebAPI API(ip, port);
	std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss;
	ss << "/Download?keyword=" << "DImage" << "&id=" << id << "&src=" << src;
	auto res = API.Send(ss.str(), "");
	std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();
	cv::Mat temp = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());
	cv::Mat img = cv::imdecode(temp, cv::IMREAD_COLOR);
	cv::Mat visImg = img.clone();

	//인코딩 정보 저장
	User->ImageDatas.Update(id, temp.clone());

	//depth 다운로드
	ss.str("");
	ss << "/Download?keyword=" << "DDepth" << "&id=" << id << "&src=" << src;
	auto res2 = API.Send(ss.str(), "");
	cv::Mat tempdepth = cv::Mat(res2.size(), 1, CV_8UC1, (void*)res2.data());
	cv::Mat depthsrc = cv::imdecode(tempdepth, cv::IMREAD_ANYDEPTH); //16U, ushort
	User->DepthDatas.Update(id, tempdepth.clone());

	//frame 생성
	cv::Mat imDepth;
	depthsrc.convertTo(imDepth, CV_32F, User->mpCamera->mDepthMapFactor);

	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAY
	EdgeSLAM::Frame* frame = new EdgeSLAM::Frame(gray, imDepth, User->mpCamera, id, frame_ts);
	frame->mnShared++;
	if (frame->N > 3000) {
		std::cout << "before tracking " << id << " == " << frame->N << std::endl;
	}
	std::chrono::high_resolution_clock::time_point t_track_start = std::chrono::high_resolution_clock::now();
	EdgeSLAM::Tracker::Track(POOL, SLAM, id, src, frame, frame_ts);
	std::chrono::high_resolution_clock::time_point t_track_end = std::chrono::high_resolution_clock::now();

	auto pRefKF = User->mpRefKF;
	auto trackStat = User->GetState();
	bool bTrackSuccess = trackStat == EdgeSLAM::UserState::Success && frame->mvpMapPoints.size() == frame->N;
	bool bCommuTest = User->mbCommuTest;
	int nContentKFs = User->mnContentKFs;
	bool bMapping = User->mbMapping;

	if (User->mbNewKF) {
		User->mbNewKF = false;
		std::string nsrc = src + ".DImage";
		Utils::SendReqMessage("RequestObjectDetection", nsrc, id);
		Utils::SendReqMessage("RequestSegmentation", nsrc, id);
	}
	if (frame->N > 3000) {
		std::cout << "after tracking " << id << " == " << frame->N << std::endl;
	}
	if (bTrackSuccess) {
		
		for (int i = 0, N = frame->mvpMapPoints.size(); i < N; i++) {
			auto pMPi = frame->mvpMapPoints[i];
			if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i])
				continue;
			auto pt = frame->mvKeys[i].pt;
			cv::circle(visImg, pt, 3, cv::Scalar(0, 255, 0), 1);
		}

	}
	else {
		cv::Mat failImg = cv::Mat::zeros(img.size(), CV_8UC3);
		failImg = cv::Scalar(0, 0, 255);
		cv::addWeighted(img, 0.7, failImg, 0.3, 0.0, visImg);
	}
	//depth 시각화
	SLAM->VisualizeImage(mapName, visImg, nVisID);
	{
		//16short -> float 변환 후 노말라이즈
		cv::Mat depth;
		cv::normalize(depthsrc, depth, 0, 255, cv::NORM_MINMAX, CV_8UC1);
		cv::cvtColor(depth, depth, cv::COLOR_GRAY2BGR);
		std::string mapName = User->mapName;

		SLAM->VisualizeImage(mapName, depth, nVisID+1);

		cv::Mat blended2;
		cv::addWeighted(img, 0.7, depth, 0.3, 0.0, blended2);
		SLAM->VisualizeImage(mapName, blended2, nVisID + 2);
	}
	frame->mnShared--;
	User->mnUsed--;
}

void Track(ThreadPool::ThreadPool* POOL, EdgeSLAM::SLAM* SLAM, std::string src, std::string url, int id, double received_ts, double frame_ts) {
	if (!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++;
	//image download
	WebAPI API(ip, port);
	std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
	auto res = API.Send(url, "");
	std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();
	int n2 = res.size();

	//std::cout << "id = " << id << " " <<std::fixed<<std::setprecision(9)<< frame_ts << std::endl;

	cv::Mat temp = cv::Mat(n2, 1, CV_8UC1, (void*)res.data());
	cv::Mat img = cv::imdecode(temp, cv::IMREAD_COLOR);
	cv::Mat visImg = img.clone();

	//인코딩 정보 저장
	User->ImageDatas.Update(id, temp.clone());

	cv::Mat K = User->GetCameraMatrix();
	cv::Mat Kinv = User->GetCameraInverseMatrix();
	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAYs
	EdgeSLAM::Frame* frame = new EdgeSLAM::Frame(gray, User->mpCamera, id, frame_ts);
	frame->mnShared++;
	//auto frame = std::make_shared<EdgeSLAM::Frame>(img, User->mpCamera, id, received_ts);
	int nVisID = User->GetVisID() + 4;
	std::string mapName = User->mapName;
	//User->mnUsed--;

	{
		std::stringstream ss;
		// "num,source,id,quality,download,processing,upload,size\n";
		//ss << N << "," << src << "," << id << "," << User->mnQuality << "," << du_download << "," << du_track << "," << du_upload <<","<<du_graph <<","<<du_preprocessing<<"," << du_total << "," << totaldata.rows * sizeof(float) << std::endl;
		ss << "up," <<frame->mnFrameID<<"," << n2 << std::endl;
		SLAM->EvaluationLatency.push_back(ss.str());
	}

	auto pPrevKF = User->mpRefKF;
	//필요없는데 퓨처, 프로마이즈 참조용.
	/*std::promise<int> prefid;
	std::future<int> frefid = prefid.get_future();
	int refid;
	POOL->EnqueueJob(TrackObject, User, id, frame, gray, &prefid);*/
	std::promise<std::map<ObjectSLAM::GlobalInstance*, cv::Rect>> prefid;
	std::future<std::map<ObjectSLAM::GlobalInstance*, cv::Rect>> frefid = prefid.get_future();
	POOL->EnqueueJob(TrackObject2, User, id, &prefid);

	std::chrono::high_resolution_clock::time_point t_track_start = std::chrono::high_resolution_clock::now();
	EdgeSLAM::Tracker::Track(POOL, SLAM, id, src, frame, frame_ts);
	std::chrono::high_resolution_clock::time_point t_track_end = std::chrono::high_resolution_clock::now();
	


	//User->mnUsed++;
	//다이나믹 맵포인트 관리
	auto pRefKF = User->mpRefKF;
	auto trackStat = User->GetState();
	bool bTrackSuccess = trackStat == EdgeSLAM::UserState::Success && frame->mvpMapPoints.size() == frame->N;
	bool bCommuTest = User->mbCommuTest;
	int nContentKFs = User->mnContentKFs;
	bool bMapping = User->mbMapping;

	//std::cout << "object tracking = " <<id<<" " << refid << std::endl;
	cv::Mat ptdata = cv::Mat::zeros(0, 1, CV_32FC1);
	
	//SemanticSLAM::DynamicTrackingProcessor::ObjectTracking(POOL, SLAM, src, frame, img.clone(), id);
	/////////////////////로컬 그래프 정보 전송
	//트래킹 후에 데이터 전송
	cv::Mat Pcw = frame->GetPose();
	cv::Mat Rcw = Pcw.rowRange(0, 3).colRange(0, 3);
	cv::Mat tcw = Pcw.rowRange(0, 3).col(3);
	cv::Mat Tinv = frame->GetPoseInverse();
	cv::Mat Ow = frame->GetCameraCenter();
	//전송 수정 테스트
	//int nQueueKFs = 8;
	//cv::Mat totaldata = cv::Mat::zeros(2, 1, CV_32FC1); //파싱 아이디, 전체크기는 자기자신 포함. 1 + Nkf + Nmp

	//User->mbNewKF = true;

	bool bTrackObject = false;
	cv::Mat img2;
	std::vector<std::pair<cv::Point2f, cv::Point2f>> vecPairMatches;

	//std::map<ObjectSLAM::GlobalInstance*, cv::Rect> mapRects;
	//TrackObject2(User, id, mapRects);

	//global instance test
	frefid.wait();
	auto mapRects = frefid.get();
	if (mapRects.size() > 0)
	{
		//rect x,y 수정
		//시각화
		for (auto pair : mapRects)
		{
			auto pG = pair.first;
			auto rect = pair.second;

			auto pt = pG->ProjectPoint(Pcw, K);
			if (pt.x == -1)
				continue;
			rect.x = pt.x - rect.width / 2;
			rect.y = pt.y - rect.height / 2;

			cv::rectangle(visImg, rect, cv::Scalar(255, 0, 255), 2);
		}
		if (false && User->mbSave)
		{
			//기록
			std::stringstream ss;
			ss.str("");
			ss << "../res/device/" << User->strDeviceName<<"_"<<id << ".png";
			cv::imwrite(ss.str(), visImg);
		}
	}

	if (User->mbNewKF) {
		User->mbNewKF = false;

		std::string tsrc = src + ".Image";
		//시멘틱 슬램
		if (bSemantic && ObjSystem->MapObjectDevices.Count(User)) {
			auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
			 
			////sam2
			int refid = 0;
			if (refid > 0 && ObjSystem->MapKeyFrameNBoxFrame.Count(refid)) {
				auto pBF = ObjSystem->MapKeyFrameNBoxFrame.Get(refid);

				auto pMatches = ObjSystem->GetMatchInfo(id, refid);
				if (pMatches) {

					for (int i = 0; i < pMatches->vecFounds.size(); i++) {
						if (!pMatches->vecFounds[i])
							continue;
						auto pt1 = pMatches->vecTargetCorners[i];
						auto pt2 = pMatches->vecReferenceCorners[i];
						vecPairMatches.push_back(std::make_pair(pt1, pt2));
					}

					//시각화

					img2 = pBF->img.clone();
					
					if (pBF->mapMasks.Count("yoloseg"))
					{
						auto pSeg = pBF->mapMasks.Get("yoloseg");

						auto pRefKF = pBF->mpRefKF;
						for (int i = 0; i < pRefKF->N; i++)
						{
							auto pMPi = pRefKF->mvpMapPoints.get(i);
							if (!pMPi || pMPi->isBad())
								continue;
							auto pt = pRefKF->mvKeys[i].pt;
							cv::circle(img2, pt, 3, cv::Scalar(0, 255, 0), 2);
						}

						auto pSegIns = pSeg->FrameInstances.Get();
						for (auto pair : pSegIns)
						{
							auto pt = pair.second->pt;
							cv::circle(img2, pt, 5, cv::Scalar(0, 255, 255), 2);
						}
					}

					bTrackObject = true;
					
				}
			}

			if (ptdata.rows > 0) {
				//reqest
				auto sam2key = "reqsam2";
				//auto du_upload = Utils::SendData(sam2key, tsrc, ptdata, id, received_ts);
			}
			////sam2

			//Utils::SendReqMessage("reqdistdepth", tsrc, id);
			Utils::SendReqMessage("reqdetectron", tsrc, id);
			//Utils::SendReqMessage("reqsam2", tsrc, id);

			if (pPrevKF) {
				std::stringstream aass2;
				aass2 << tsrc << "," << id << "," << pPrevKF->sourceName << ".Image," << pPrevKF->mnFrameId;
				std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
				Utils::SendReqMessage("reqraft", aass2.str(), id, t_start.time_since_epoch().count());
			}

			//Utils::SendReqMessage("RequestSegmentation", tsrc, id);
			std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

			auto pNewBF = new ObjectSLAM::BoxFrame(id, img.cols, img.rows, User, (BaseSLAM::AbstractPose*)pRefKF->mpCamPose);
			pNewBF->t_start = start;
			pNewBF->Copy(frame);
			pNewBF->mpRefKF = pRefKF;
			pNewBF->img = img.clone();
			pNewBF->gray = gray.clone();
			pNewBF->mpPrevBF;

			/*if(refid > 0)
				ObjectSLAM::ObjectMapper::ObjectMatchingPropagation(ObjSystem, src, refid, pRefKF, pNewBF);*/

			//갱신 부분을 초기화루로 변경한다면?
			//CurrBF를 reference로 이용함.
			pNewBF->mpPrevBF = pObjDevice->mpCurrBF;
			pObjDevice->mpPrevBF = pObjDevice->mpCurrBF;
			pObjDevice->mpCurrBF = pNewBF;

			ObjSystem->MapKeyFrameNBoxFrame.Update(id, pNewBF);
			std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
			Utils::SendReqMessage("reqyoloseg", tsrc, id, t_start.time_since_epoch().count());
			ObjSystem->MapTimeStampForKF.Update(pRefKF->mnId, t_start.time_since_epoch().count());
		}
	}

	if(bTrackSuccess){
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		long long ts = start.time_since_epoch().count();
		
		std::string keyword = "ReferenceFrame";
		cv::Mat totaldata = cv::Mat::zeros(0, 0, CV_32FC1);
		
		if(User->mbBaseLocalMap)
		{
			cv::Mat localmapdata;
			EdgeSLAM::Tracker::GenerateLocalMapForSync(SLAM, User, localmapdata, id, ts);
			totaldata.push_back(localmapdata);
			//keyword = "UpdatedLocalMap";
		}

		cv::Mat kfdata, contentdata, planedata;
		EdgeSLAM::Tracker::GenerateReferenceFrameForSync(SLAM, User, frame, kfdata, id, ts);
		SemanticSLAM::ContentProcessor::GenerateGraphDataForSync(SLAM, User, contentdata, id, ts);
		SemanticSLAM::PlaneEstimator::GeneratePlaneDataForSync(SLAM, User, planedata, id, ts);

		totaldata.push_back(kfdata);
		totaldata.push_back(contentdata);
		totaldata.push_back(planedata);
		long long ts_end_graph = start.time_since_epoch().count();
		auto du_upload =  Utils::SendData(keyword, src, totaldata, id, received_ts);

		if (bSaveLatency) {
			auto du_track = std::chrono::duration_cast<std::chrono::milliseconds>(t_track_end - t_track_start).count();
			auto du_download = std::chrono::duration_cast<std::chrono::milliseconds>(t_down_end - t_down_start).count();
			auto du_graph = (ts_end_graph - ts)/1000000.0;
			auto du_total = (ts_end_graph - t_down_start.time_since_epoch().count()) / 1000000.0;
			auto du_preprocessing = std::chrono::duration_cast<std::chrono::milliseconds>(t_track_start - t_down_end).count();

			int N = SLAM->GetConnectedDevice();
			{
				std::stringstream ss;
				// "num,source,id,quality,download,processing,upload,size\n";
				//ss << N << "," << src << "," << id << "," << User->mnQuality << "," << du_download << "," << du_track << "," << du_upload <<","<<du_graph <<","<<du_preprocessing<<"," << du_total << "," << totaldata.rows * sizeof(float) << std::endl;
				ss << "down," <<frame->mnFrameID<<"," << kfdata.rows * 4 << std::endl;
				SLAM->EvaluationLatency.push_back(ss.str());
			}
		}
		if (bSaveVOLatency)  
		{
			int N = SLAM->GetConnectedDevice();
			std::stringstream ss;
			ss << N << "," << src << "," << "indirect,upload," << id << "," << totaldata.rows * sizeof(float) << "," << du_upload << std::endl;
			SLAM->EvaluationVirtualObjectLatency.push_back(ss.str());
		}
	
		////시각화 및 칼만필터 적용
		//visualize
		for (int i = 0, N = frame->mvpMapPoints.size(); i < N; i++) {
			auto pt = frame->mvKeys[i].pt;
			auto pMPi = frame->mvpMapPoints[i];
			if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i]){
				//cv::circle(visImg, pt, 2, cv::Scalar(0, 0, 0), 1);
				continue;
			}
			else {
				if (pMPi->mnLabelID == (int)StructureLabel::FLOOR || pMPi->mnLabelID == (int)StructureLabel::CEIL)
					cv::circle(visImg, pt, 3, cv::Scalar(255, 0, 0), 1);
				else
					cv::circle(visImg, pt, 3, cv::Scalar(0, 255, 0), 1);
			}
		}
		
		if (bTrackObject) {
			cv::Mat resImage;
			//SLAM->VisualizeMatchingImage(resImage, visImg, img2, vecPairMatches, User->mapName, nVisID + 1);

			/*std::stringstream ss;
			ss.str("");
			ss << "../res/asso/" << id << "_0track.png";
			cv::imwrite(ss.str(), resImage);*/
		}
	}

	frame->mnShared--;
	User->mnUsed--;
	
	//그리드 시각화
	if (bCommuTest && pRefKF) {

		std::vector<EdgeSLAM::KeyFrame*> vpLocalKFs = pRefKF->GetBestCovisibilityKeyFrames(nContentKFs);
		vpLocalKFs.push_back(pRefKF);
		std::set<SemanticSLAM::Grid*> setGrids;
		std::vector<std::vector<std::pair<cv::Point2f, bool>>> vecProjectedCorners;

		/*cv::Mat Pcw = frame->GetPose();
		cv::Mat Rcw = Pcw.rowRange(0, 3).colRange(0, 3);
		cv::Mat tcw = Pcw.rowRange(0, 3).col(3);*/

		cv::Mat gridImg = img.clone();
		//light test
		cv::Mat K2 = cv::Mat::eye(4, 4, CV_32FC1);
		K.copyTo(K2.colRange(0, 3).rowRange(0, 3));
		K2 = K2 * Pcw;
		//light test

		for (auto iter = vpLocalKFs.begin(), iend = vpLocalKFs.end(); iter != iend; iter++) {
			auto pKFi = *iter;
			if (SemanticSLAM::GridProcessor::GlobalKeyFrameNGrids.Count(pKFi)) {
				auto spGrids = SemanticSLAM::GridProcessor::GlobalKeyFrameNGrids.Get(pKFi);
				for (auto jter = spGrids.begin(), jend = spGrids.end(); jter != jend; jter++) {
					auto pTempGrid = *jter;
					if (setGrids.count(pTempGrid))
						continue;
					setGrids.insert(pTempGrid);
					int xidx, yidx, zidx;
					SemanticSLAM::GridProcessor::ConvertIndex(pTempGrid->pos, xidx, yidx, zidx);
					/*if (pTempGrid->mbFloor) {
						auto corners = SemanticSLAM::GridProcessor::ProjectedVoxelCorners(xidx, yidx, zidx, 0.1, K, Rcw, tcw);
						vecProjectedCorners.push_back(corners);
					}
					if (pTempGrid->mbCeil) {
						auto corners = SemanticSLAM::GridProcessor::ProjectedVoxelCorners(xidx, yidx, zidx, 0.1, K, Rcw, tcw,3);
						vecProjectedCorners.push_back(corners);
					}*/
					if (pTempGrid->mbFloor) {
						auto corners = SemanticSLAM::GridProcessor::ProjectedGridCorners(xidx, yidx, zidx, 0.1, K, Rcw, tcw);
						vecProjectedCorners.push_back(corners);
					}
					if (pTempGrid->mbCeil) {
						auto corners = SemanticSLAM::GridProcessor::ProjectedGridCorners(xidx, yidx, zidx, 0.1, K, Rcw, tcw,3);
						vecProjectedCorners.push_back(corners);
						if (pTempGrid->mpLightNode) {
							auto pGraph = pTempGrid->mpLightNode;
							cv::Mat temp = K2 * pGraph->pos;
							float depth = temp.at<float>(2);
							cv::Point2f pt(temp.at<float>(0) / depth, temp.at<float>(1) / depth);
							cv::circle(gridImg, pt, 10, pGraph->color, -1);
						}
						//mapLight[lightid++] = K2 * pTempGrid->matLight.t();
					}
					if (pTempGrid->mbWall) {
						auto corners2 = SemanticSLAM::GridProcessor::ProjectedGridCorners(xidx, yidx, zidx, 0.1, K, Rcw, tcw,2);
						vecProjectedCorners.push_back(corners2);
					}
				}//for jter
			}//if
		}//iter

		//그리드 시각화
		//cv::Mat gridImage = cv::Mat::zeros(640, 360, CV_8UC3);
		
		////light 시각화
		//{
		//	cv::RNG rng(12345);
		//	for (auto iter = mapLight.begin(), ien d = mapLight.end(); iter != iend; iter++) {
		//		cv::Mat tempLight = iter->second;
		//		cv::Vec3b color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		//		for (int i = 0; i < tempLight.cols; i++) {
		//			cv::Mat temp = tempLight.col(i);
		//			float depth = temp.at<float>(2);
		//			cv::Point2f pt(temp.at<float>(0) / depth, temp.at<float>(1) / depth);
		//			//visImg.at<cv::Vec3b>(pt) = color;
		//			cv::circle(gridImg, pt, 1, color, -1);
		//		}
		//	}
		//}
		//그리드 시각화
		//SemanticSLAM::GridProcessor::DrawGrids(vecProjectedCorners, gridImg);
		//SLAM->VisualizeImage(mapName, gridImg, nVisID+3);

		if(User->mbSave)
		{
			std::stringstream ss;
			ss << "../res/images/" << frame->mnFrameID << "_grid.png";
			cv::imwrite(ss.str(), gridImg.clone());
		}

		/*for (int i = 0, iend = vecProjectedCorners.size(); i < iend; i++) {
			auto pair1 = vecProjectedCorners[i][0];
			auto pair2 = vecProjectedCorners[i][1];
			auto pair3 = vecProjectedCorners[i][2];
			auto pair4 = vecProjectedCorners[i][3];

			auto pt1 = pair1.first;
			auto pt2 = pair2.first;
			auto pt3 = pair3.first;
			auto pt4 = pair4.first;

			auto b1 = pair1.second;
			auto b2 = pair2.second;
			auto b3 = pair3.second;
			auto b4 = pair4.second;

			if (b1 && b2)
				cv::line(visImg, pt1, pt2, cv::Scalar(255, 255, 0));
			if (b1 && b3)
				cv::line(visImg, pt1, pt3, cv::Scalar(255, 255, 0));
			if (b4 && b2)
				cv::line(visImg, pt4, pt2, cv::Scalar(255, 255, 0));
			if (b4 && b3)
				cv::line(visImg, pt4, pt3, cv::Scalar(255, 255, 0));
		}*/
	}
	//그리드 시각화

	/*{
		std::stringstream ss;
		ss << "../res/images/" << frame->mnFrameID << "_tracking.png";
		cv::imwrite(ss.str(), visImg);
	}*/
	if(nVisID < 8)
		SLAM->VisualizeImage(mapName, visImg, nVisID);
	////시각화 및 칼만필터 적용
}

void TrackWithObjects(ThreadPool::ThreadPool* POOL, EdgeSLAM::SLAM* SLAM, std::string src, std::string url, int id, double ts) {
	if (!SLAM->CheckUser(src)) {
		return;
	}
	auto User = SLAM->GetUser(src);
	if (!User)
		return;
	User->mnUsed++;
	//image download
	WebAPI API(ip, port);
	auto res = API.Send(url, "");
	int n2 = res.size();
	cv::Mat temp = cv::Mat(n2, 1, CV_8UC1, (void*)res.data());
	cv::Mat img = cv::imdecode(temp, cv::IMREAD_COLOR);
	cv::Mat visImg = img.clone();

	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAYs
	cv::Mat K = User->GetCameraMatrix();
	EdgeSLAM::Frame* frame = new EdgeSLAM::Frame(gray, User->mpCamera, id, ts);
	//auto frame = std::make_shared<EdgeSLAM::Frame>(img, User->mpCamera, id, ts);
	//std::shared_ptr<EdgeSLAM::Frame> spFrame(new EdgeSLAM::Frame(img, User->mpCamera, id, ts));
	auto vecObjectTrackingRes = User->mapObjectTrackingResult.Get();
	int nVisID = User->GetVisID() + 4;
	std::string mapName = User->mapName;
	//User->mnUsed--;
	
	//pUser->ImageDatas.Update(id, temp);
	//POOL->EnqueueJob(SLAM->Track, id, src, img.clone(), ts);
	//object tracking
	//POOL->EnqueueJob(SemanticSLAM::DynamicTrackingProcessor::ObjectTracking, POOL, SLAM, src, frame, img.clone(), id);
	
	
	std::chrono::high_resolution_clock::time_point t_track_start = std::chrono::high_resolution_clock::now();
	EdgeSLAM::Tracker::Track(POOL, SLAM, id, src, frame, ts);
	std::chrono::high_resolution_clock::time_point t_track_end = std::chrono::high_resolution_clock::now();
	auto du_track = std::chrono::duration_cast<std::chrono::milliseconds>(t_track_end - t_track_start).count();
		
	//User->mnUsed++;
	//다이나믹 맵포인트 관리
	auto pRefKF = User->mpRefKF;
	if(User->GetState() == EdgeSLAM::UserState::Success && frame->mvpMapPoints.size()> 0)
	{
		cv::Mat Pcw = frame->GetPose();
		cv::Mat Rcw = Pcw.rowRange(0, 3).colRange(0, 3);
		cv::Mat tcw = Pcw.rowRange(0, 3).col(3);

		int nTemp = 0;
		int nTemp2 = 0;
		int n3 = 0;
		for (int i = 0, N = frame->N; i < N; i++) {
			auto pMPi = frame->mvpMapPoints[i];
			if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i]) {
				continue;
			}

			if (pMPi->mpObjectPoint) {
				nTemp++;
				cv::circle(visImg, frame->mvKeys[i].pt, 5, cv::Scalar(0, 255, 255), 2);
			}

			//if (pMPi->mpSemanticLabel) {
			//	int n = pMPi->mpSemanticLabel->LabelCount.Count((int)StructureLabel::PERSON);
			//	if (n > 0) {
			//		nTemp++;
			//		cv::circle(visImg, frame->mvKeys[i].pt, 5, cv::Scalar(0, 255, 255), 2);
			//	}
			//}
			//if (pMPi->mnLabelID == (int)StructureLabel::PERSON) {
			//	nTemp2++;
			//	cv::circle(visImg, frame->mvKeys[i].pt, 5, cv::Scalar(0, 255, 0), 2);

			//	if (!pMPi->mpConfidence)
			//		pMPi->mpConfidence = new Confidence();
			//	cv::Mat Xw = pMPi->GetWorldPos();
			//	cv::Mat p2 = K * (Rcw * Xw + tcw);
			//	float d2 = p2.at<float>(2);

			//	cv::Point2f pt2(p2.at<float>(0) / d2, p2.at<float>(1) / d2);    //슬램
			//	auto dist = pMPi->mpConfidence->CalcConfidence(pt2, frame->mvKeysUn[i].pt);
			//	if (dist > 5.0) {
			//		cv::circle(visImg, frame->mvKeys[i].pt, 5, cv::Scalar(0, 0, 255), 2);
			//		n3++;
			//	}

			//}
			 
			//if (pMPi->BoxObservations() > 1) {
			//	nTemp++;
			//	//cv::circle(visImg, frame->mvKeys[i].pt, 9, cv::Scalar(255, 255, 0), 2);

			//	auto Boxes = pMPi->GetBoxObservations();
			//	bool bCalcConf = false;
			//	for (auto bter = Boxes.begin(), bend = Boxes.end(); bter != bend; bter++) {
			//		auto pBox = bter->first;
			//		int label = pBox->label;
			//		if (label == (int)MovingObjectLabel::PERSON) {
			//			nTemp2++;
			//			cv::circle(visImg, frame->mvKeys[i].pt, 5, cv::Scalar(0, 255, 255), 2);
			//			bCalcConf = true;
			//			break;
			//		}
			//	}
			//	if (bCalcConf) {
			//		if (!pMPi->mpConfidence)
			//			pMPi->mpConfidence = new Confidence();
			//		cv::Mat Xw = pMPi->GetWorldPos();
			//		cv::Mat p2 = K * (Rcw * Xw + tcw);
			//		float d2 = p2.at<float>(2);

			//		cv::Point2f pt2(p2.at<float>(0) / d2, p2.at<float>(1) / d2);    //슬램
			//		auto dist = pMPi->mpConfidence->CalcConfidence(pt2, frame->mvKeysUn[i].pt);
			//		if (dist > 5.0) {
			//			cv::circle(visImg, frame->mvKeys[i].pt, 5, cv::Scalar(0, 0, 255), 2);
			//			n3++;
			//		}
			//	}
			//}
		}
		
		//std::cout << "persond dynamic test =" << nTemp << " " << nTemp2 <<" "<<n3 << std::endl;
		//{
		//	//연결된 그리드 확인하기
		//	if (pRefKF && SemanticSLAM::GridProcessor::GlobalKeyFrameNGrids.Count(pRefKF)) {
		//		std::cout << "Grid test = " << SemanticSLAM::GridProcessor::GlobalKeyFrameNGrids.Get(pRefKF).size() << std::endl;
		//	}
		//}
	}
	
	//if (pRefKF && SemanticSLAM::SemanticProcessor::GraphKFAndPerson.Count(pRefKF)) {
	//	int nTemp = 0;
	//	int nTemp2 = 0;
	//	int nTemp3 = 0;
	//	auto pBBox = SemanticSLAM::SemanticProcessor::GraphKFAndPerson.Get(pRefKF);
	//	for (int i = 0, N = frame->N; i < N; i++) {
	//		auto pMPi = frame->mvpMapPoints[i];
	//		if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i]) {
	//			continue;
	//		}
	//		if (pMPi->BoxObservations() > 1) {
	//			nTemp2++;
	//			cv::circle(visImg, frame->mvKeys[i].pt, 5, cv::Scalar(0, 255, 255), 2);
	//		}
	//		if (pMPi->IsInBox(pBBox)) {
	//			nTemp++;
	//			cv::circle(visImg, frame->mvKeys[i].pt, 5, cv::Scalar(0,0, 255), 2);
	//		}
	//	}
	//	std::cout << "persond dynamic test =" <<nTemp<<" "<<nTemp2 << std::endl;
	//	/*for (int i = 0; i < pBBox->mvKeys.size(); i++) {

	//	}*/
	//}
	bool bCommuTest = User->mbCommuTest;
	int nContentKFs = User->mnContentKFs;
	User->mnUsed--;

	//SemanticSLAM::DynamicTrackingProcessor::ObjectTracking(POOL, SLAM, src, frame, img.clone(), id);
	/////////////////////로컬 그래프 정보 전송
	//트래킹 후에 데이터 전송
	cv::Mat Pcw = frame->GetPose();
	cv::Mat Rcw = Pcw.rowRange(0, 3).colRange(0, 3);
	cv::Mat tcw = Pcw.rowRange(0, 3).col(3);

	//오브젝트 정보도 같이 추가하기
	cv::Mat data = cv::Mat::zeros(0, 1, CV_32FC1);
	{
		//트래킹
		cv::Mat trackData = cv::Mat::zeros(14, 1, CV_32FC1); //inlier, pose + point2f, octave, angle, point3f
		int nTrackData = 2;
		trackData.at<float>(nTrackData++) = Pcw.at<float>(0, 0);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(0, 1);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(0, 2);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(1, 0);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(1, 1);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(1, 2);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(2, 0);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(2, 1);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(2, 2);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(0, 3);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(1, 3);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(2, 3);

		int nTrack = 0;
		for (int i = 0, N = frame->mvpMapPoints.size(); i < N; i++)
		{
			if (frame->mvpMapPoints[i])
			{
				if (!frame->mvbOutliers[i] && !frame->mvpMapPoints[i]->isBad())
				{
					int nDataIdx = 0;
					auto kp = frame->mvKeys[i];
					auto mp = frame->mvpMapPoints[i]->GetWorldPos();
					int octave = kp.octave;
					cv::Mat temp = cv::Mat::zeros(9, 1, CV_32FC1);
					temp.at<float>(nDataIdx++) = kp.pt.x;
					temp.at<float>(nDataIdx++) = kp.pt.y;
					temp.at<float>(nDataIdx++) = (float)kp.octave;
					temp.at<float>(nDataIdx++) = kp.angle;
					temp.at<float>(nDataIdx++) = (float)frame->mvpMapPoints[i]->mnId;
					temp.at<float>(nDataIdx++) = frame->mvpMapPoints[i]->mnPlaneID;// label or plane
					temp.at<float>(nDataIdx++) = mp.at<float>(0);
					temp.at<float>(nDataIdx++) = mp.at<float>(1);
					temp.at<float>(nDataIdx++) = mp.at<float>(2);
					trackData.push_back(temp);
					nTrack++;
				}
			}
		}
		if (nTrack == 0)
			trackData.push_back(cv::Mat::zeros(4500, 1, CV_32FC1));
		trackData.at<float>(0) = trackData.rows;
		trackData.at<float>(1) = (float)nTrack;
		data.push_back(trackData);
	}

	cv::Mat Rwc = Rcw.t();
	cv::Mat twc = -Rwc * tcw;

	std::map<int, cv::Mat> a;
	if (SLAM->TemporalDatas2.Count("dynamic")) {
		a = SLAM->TemporalDatas2.Get("dynamic");
	}
	
	for (auto iter = vecObjectTrackingRes.begin(), iend = vecObjectTrackingRes.end(); iter != iend; iter++) {
		auto pTrackRes = iter->second;
		if (pTrackRes->mState == EdgeSLAM::ObjectTrackingState::Success) {
			
			/*cv::Mat Rcw = Pcw.rowRange(0, 3).colRange(0, 3);
			cv::Mat tcw = Pcw.rowRange(0, 3).col(3);
			cv::Mat Rwc = Rcw.t();
			cv::Mat twc = -Rwc * tcw;*/
			
			cv::Mat Pco = pTrackRes->Pose.clone();
			cv::Mat Rco = Pco.rowRange(0, 3).colRange(0, 3);
			cv::Mat tco = Pco.rowRange(0, 3).col(3);

			cv::Mat Rwo = Rwc * Rco;
			cv::Mat two = Rwc * tco + twc;
			
			cv::Mat Pwo(4, 4, CV_32FC1);
			Rco.copyTo(Pwo.rowRange(0, 3).colRange(0, 3));
			tco.copyTo(Pwo.rowRange(0, 3).col(3));
			//cv::Mat Pwo = pTrackRes->mpObject->GetWorldPose();

			//컨피던스 계산
			if (pTrackRes->mpLastFrame) {
				SemanticSLAM::DynamicTrackingProcessor::UpdateConfidence(pTrackRes->mpLastFrame, Pcw, Pco, Pwo, pTrackRes->mpObject->GetOrigin(), K);
			}

			{
				SemanticSLAM::Plane* Floor = nullptr;
				if (SemanticSLAM::PlaneEstimator::GlobalFloor)
				{
					Floor = SemanticSLAM::PlaneEstimator::GlobalFloor;
				}
				cv::Mat Ow = pTrackRes->mpObject->GetOrigin();
				auto pTrackFrame = pTrackRes->mpLastFrame;
				auto pObj = pTrackRes->mpObject;
				if (pObj && pTrackFrame) {
					int Nconf = 0;
					auto allMPs = pObj->mspMPs.ConvertVector();
					std::vector<float> allDists(allMPs.size(), 0);
					std::vector<bool> allInliers(allMPs.size(), false);
					std::vector<float> allDists2(allMPs.size(), false);
					for (int i = 0, N = allMPs.size(); i < N; i++) {
						auto pMPi = allMPs[i];
						if (!pMPi || pMPi->isBad())
							continue;
						if (!pMPi->mpConfidence)
							pMPi->mpConfidence = new Confidence();
						cv::Mat Xo = pMPi->GetWorldPos() - Ow;
						cv::Mat Xw = pMPi->GetWorldPos();

						cv::Mat Xw2 = Rwo * Xo + two;
						////cv::Mat d1 = Xw2 - pMPi->GetWorldPos();
						//
						cv::Mat p1 = K * (Rco * Xo + tco);
						cv::Mat p2 = K * (Rcw * Xw + tcw);
						  
						float d1 = p1.at<float>(2);
						float d2 = p2.at<float>(2);

						cv::Point2f pt1(p1.at<float>(0) / d1, p1.at<float>(1) / d1);    //오브젝트
						cv::Point2f pt2(p2.at<float>(0) / d2, p2.at<float>(1) / d2);    //슬램

						allDists[i] = pMPi->mpConfidence->CalcConfidence(pt2, pt1);
						if (allDists[i] < 10.0){
							allInliers[i] = true;
							Nconf++;
							cv::circle(visImg, pt1, 8, cv::Scalar(0, 255, 0), -1);
						}
						else {
							cv::circle(visImg, pt1, 8, cv::Scalar(0, 0, 255), -1);
						}
						if (Floor) {
							float dist3 = Floor->Distacne(pMPi->GetWorldPos());
							allDists[2] = pMPi->mpConfidence->CalcConfidence(abs(dist3));
						}
						//std::cout << "Conf inlier = " << N << " " << allMPs.size() << std::endl;
					}
				}
			}

			cv::Mat origin = cv::Mat::zeros(3, 1, CV_32FC1);
			cv::Mat OBJc = tco;
			/*cv::Mat proj = k * OBJc;
			float depth = proj.at<float>(2);
			cv::Point2f pt(proj.at<float>(0) / depth, proj.at<float>(1) / depth);
			cv::circle(img, pt, 80, cv::Scalar(255, 255, 0), 3);*/

			//world에서 표현
			cv::Mat OBJw = Rwc * OBJc + twc;
			a[nVisID] = OBJw;

			{
				cv::Mat _ori = pTrackRes->mpObject->GetOrigin();
				//오브젝트 데이터 전송
				cv::Mat objData = cv::Mat::zeros(18, 1, CV_32FC1);
				int objIdx = 6;
				objData.at<float>(objIdx++) = Pwo.at<float>(0, 0);
				objData.at<float>(objIdx++) = Pwo.at<float>(0, 1);
				objData.at<float>(objIdx++) = Pwo.at<float>(0, 2);
				objData.at<float>(objIdx++) = Pwo.at<float>(1, 0);
				objData.at<float>(objIdx++) = Pwo.at<float>(1, 1);
				objData.at<float>(objIdx++) = Pwo.at<float>(1, 2);
				objData.at<float>(objIdx++) = Pwo.at<float>(2, 0);
				objData.at<float>(objIdx++) = Pwo.at<float>(2, 1);
				objData.at<float>(objIdx++) = Pwo.at<float>(2, 2);
				objData.at<float>(objIdx++) = Pwo.at<float>(0, 3);
				objData.at<float>(objIdx++) = Pwo.at<float>(1, 3);
				objData.at<float>(objIdx++) = Pwo.at<float>(2, 3);

				float nObj = 0;
				auto pTrackFrame = pTrackRes->mpLastFrame;
				for (int i = 0, N = pTrackFrame->mvImagePoints.size(); i < N; i++) {
					auto pMPi = pTrackFrame->mvpMapPoints[i];
					if (!pMPi || pMPi->isBad())
						continue;
					nObj++;
					auto objPt = pMPi->GetWorldPos() - _ori;
					auto imgPt = cv::Mat(pTrackFrame->mvImagePoints[i]);
					objData.push_back(imgPt);
					objData.push_back(objPt);
				}
				objData.at<float>(0) = data.rows+objData.rows;
				objData.at<float>(1) = nObj;
				objData.at<float>(2) = pTrackRes->mpObject->mnId;
				objData.at<float>(3) = 0.05;
				objData.at<float>(4) = 0.05;
				objData.at<float>(5) = 0.05;
				data.push_back(objData);
			}
		}//if
	}
	//test
	//float next = data.at<float>(0);
	//float n = data.at<float>(1);
	//if (next < data.rows) {
	//	float nnext = data.at<float>(next);
	//	float n2 = data.at<float>(next + 1);
	//	//std::cout << next << " " << nnext <<" "<<n2 << " " << data.rows << std::endl;
	//}

	//평면

	//가상 객체
	POOL->EnqueueJob(EdgeSLAM::Tracker::SendDeviceTrackingData, SLAM, src, data, id, ts);
	POOL->EnqueueJob(SemanticSLAM::ContentProcessor::ShareContent, SLAM, src, id);
	POOL->EnqueueJob(SemanticSLAM::PlaneEstimator::UpdateLocalMapPlanes, SLAM, src, id); //에러 확인하기
	/////////////////////로컬 그래프 정보 전송
	
	SLAM->TemporalDatas2.Update("dynamic", a);

	////시각화 및 칼만필터 적용
	//visualize
	for (int i = 0, N = frame->mvpMapPoints.size(); i < N; i++) {
		auto pt = frame->mvKeys[i].pt;
		auto pMPi = frame->mvpMapPoints[i];
		if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i]){
			cv::circle(visImg, pt, 2, cv::Scalar(0, 0, 0), -1);
		}else
			cv::circle(visImg, pt, 3, cv::Scalar(255, 0, 0), -1);
	}
	//std::cout << "Confidence check start" << std::endl;
	for (auto iter = vecObjectTrackingRes.begin(), iend = vecObjectTrackingRes.end(); iter != iend; iter++) {
		auto pTrackRes = iter->second;
		if (pTrackRes->mState == EdgeSLAM::ObjectTrackingState::Success) {
		
			SemanticSLAM::DynamicTrackingProcessor::drawBoundingBox(visImg, pTrackRes->Pose, K, pTrackRes->mpObject->radius_x, pTrackRes->mpObject->radius_y, pTrackRes->mpObject->radius_z);

			auto pTrackFrame = pTrackRes->mpLastFrame;
			cv::Mat Pwo;
			SemanticSLAM::DynamicTrackingProcessor::UpdateKalmanFilter(pTrackRes->mpObject, pTrackFrame->mvImagePoints.size(), Pcw, pTrackRes->Pose, Pwo);

			for (int i = 0, N = pTrackFrame->mvImagePoints.size(); i < N; i++) {
				auto pMPi = pTrackFrame->mvpMapPoints[i];
				if (!pMPi || pMPi->isBad())
					continue;
				
				auto pt = pTrackFrame->mvImagePoints[i];
				cv::circle(visImg, pt, 3, cv::Scalar(255, 255, 0), -1);

				/*if (pMPi->mpConfidence) {
					std::cout <<pMPi->mnId<<" = " << pMPi->mpConfidence->d1 << " " << pMPi->mpConfidence->d3 << std::endl;
					if(pMPi->mpConfidence->d3 < 0.01)
						cv::circle(visImg, pt, 6, cv::Scalar(0, 255, 255), -1);
					else
						cv::circle(visImg, pt, 6, cv::Scalar(0, 255, 0), -1);
					if(pMPi->mpConfidence->d1 > 10.0)
						cv::circle(visImg, pt, 3, cv::Scalar(0, 255, 255), -1);
					else
						cv::circle(visImg, pt, 3, cv::Scalar(0, 0, 255), -1);
				}*/
			}
		}
	}
	
	//그리드 시각화
	if (bCommuTest && pRefKF) {
		
		std::vector<EdgeSLAM::KeyFrame*> vpLocalKFs = pRefKF->GetBestCovisibilityKeyFrames(nContentKFs);
		vpLocalKFs.push_back(pRefKF);
		std::set<SemanticSLAM::Grid*> setGrids;
		std::vector<std::vector<std::pair<cv::Point2f, bool>>> vecProjectedCorners;

		cv::Mat Pcw = frame->GetPose();
		cv::Mat Rcw = Pcw.rowRange(0, 3).colRange(0, 3);
		cv::Mat tcw = Pcw.rowRange(0, 3).col(3);

		for (auto iter = vpLocalKFs.begin(), iend = vpLocalKFs.end(); iter != iend; iter++) {
			auto pKFi = *iter;
			if (SemanticSLAM::GridProcessor::GlobalKeyFrameNGrids.Count(pKFi)) {
				auto spGrids = SemanticSLAM::GridProcessor::GlobalKeyFrameNGrids.Get(pKFi);
				for (auto jter = spGrids.begin(), jend = spGrids.end(); jter != jend; jter++) {
					auto pTempGrid = *jter;
					if (setGrids.count(pTempGrid))
						continue;
					setGrids.insert(pTempGrid);
					int xidx, yidx, zidx;
					SemanticSLAM::GridProcessor::ConvertIndex(pTempGrid->pos, xidx, yidx, zidx);
					auto corners = SemanticSLAM::GridProcessor::ProjectedGridCorners(xidx, yidx, zidx, 0.1, K, Rcw, tcw);
					vecProjectedCorners.push_back(corners);
				}//for jter
			}//if
		}//iter
		
		//그리드 시각화
		//cv::Mat gridImage = cv::Mat::zeros(640, 360, CV_8UC3);
		for (int i = 0, iend = vecProjectedCorners.size(); i < iend; i++) {
			auto pair1 = vecProjectedCorners[i][0];
			auto pair2 = vecProjectedCorners[i][1];
			auto pair3 = vecProjectedCorners[i][2];
			auto pair4 = vecProjectedCorners[i][3];

			auto pt1 = pair1.first;
			auto pt2 = pair2.first;
			auto pt3 = pair3.first;
			auto pt4 = pair4.first;

			auto b1 = pair1.second;
			auto b2 = pair2.second;
			auto b3 = pair3.second;
			auto b4 = pair4.second;

			if(b1 && b2)
			cv::line(visImg, pt1, pt2, cv::Scalar(255, 255, 0));
			if (b1 && b3)
			cv::line(visImg, pt1, pt3, cv::Scalar(255, 255, 0));
			if (b4 && b2)
			cv::line(visImg, pt4, pt2, cv::Scalar(255, 255, 0));
			if (b4 && b3)
			cv::line(visImg, pt4, pt3, cv::Scalar(255, 255, 0));
		}
		//cv::imshow("asdfasdfasdf", gridImage);
		//cv::waitKey(10);
		////그리드 시각화
	}
	//그리드 시각화

	SLAM->VisualizeImage(mapName, visImg, nVisID);
	////시각화 및 칼만필터 적용


	//포즈와 실행 시간 전송
	{
		//트래킹
		float t_track = du_track / 1000.0;
		cv::Mat trackData = cv::Mat::zeros(13, 1, CV_32FC1); //inlier, pose + point2f, octave, angle, point3f
		int nTrackData = 0;
		trackData.at<float>(nTrackData++) = Pcw.at<float>(0, 0);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(0, 1);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(0, 2);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(1, 0);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(1, 1);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(1, 2);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(2, 0);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(2, 1);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(2, 2);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(0, 3);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(1, 3);
		trackData.at<float>(nTrackData++) = Pcw.at<float>(2, 3);
		trackData.at<float>(nTrackData++) = t_track;
		POOL->EnqueueJob(SendWiseUITest, src,id, trackData,ts);
	}
}

void parsing(char* argv[], int& index) {
	std::string strTimeStamp = Utils::GetTimeStamp("_").str();
	std::string keyword = (argv[index++]);
	if (keyword == "--latency") {
		bSaveLatency = true;
		latencyPath = argv[index++];
		std::stringstream ss_file;
		ss_file << latencyPath << "_" << strTimeStamp << ".csv";
		latencyPath = ss_file.str();
		std::cout << "latency path = " << latencyPath << std::endl;
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
		deepkey1 = std::string(argv[index++])+"_"+strNumDevice;
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
	//system("mode con cols=95 lines=33 | title Offloading Server = SLAM");

	ip = argv[1];
	port = atoi(argv[2]);

	//argv parsing
	parser(argc, argv);
	/*int nIndexARGV = 3;
	if (std::string(argv[nIndexARGV++]) == "--latency") {
		bSaveLatency = true;
	}
	nNetworkSimul = atoi(argv[nIndexARGV++]);*/

	WebAPI::ip = ip;
	WebAPI::port = port;
	WebAPI::Connect();
	//for (int i = 0; i < 100; i++) {
	//	{
	//		std::stringstream ss;
	//		ss << "../bin/marker/marker" << i + 1 << ".png";
	//		cv::Mat markerImage;
	//		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	//		cv::aruco::drawMarker(dictionary, i+1, 200, markerImage, 1);
	//		cv::imwrite(ss.str(), markerImage);
	//		//cv::imshow("marker2", markerImage);
	//	}
	//}
	//cv::waitKey(0);

	//system("title Offloading Server = SLAM");
	/*EdgeSLAM::FeatureDetector* detector;
	detector = new EdgeSLAM::ORBDetector();

	EdgeSLAM::FeatureTracker* tracker;serveraddr
	tracker = new EdgeSLAM::FlannFeatureTracker();

	EdgeSLAM::Frame::tracker = tracker;
	std::cout << EdgeSLAM::Frame::tracker->detector->mnScaleLevels << std::endl;*/

	/*WSAData wsaData;
	int code = WSAStartup(MAKEWORD(1, 1), &wsaData);
	if (code == -1)
		return -1;
	sock = socket(AF_INET, SOCK_DGRAM, 0);*/

	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(35001); // 포트 번호
	//serveraddr.sin_addr.s_addr = inet_addr("143.248.6.143");
	std::string addr=ip;
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
	SemanticSLAM::SemanticProcessor* SemanticProcessor = new SemanticSLAM::SemanticProcessor();
	SemanticSLAM::SemanticProcessor::Init(SLAM);
	SemanticSLAM::GridProcessor::Init(5, 480, 640);
	SemanticSLAM::PlaneEstimator::Init();
	SemanticSLAM::ContentProcessor::mbSaveLatency = bSaveVOLatency;
	SemanticSLAM::LineProcessor::Init();
	
	////ObjectSLAM Initialization
	ObjSystem = new ObjectSLAM::ObjectSLAM();
	ObjectSLAM::InstanceLinker::SetSystem(ObjSystem);
	ObjectSLAM::BoxFrame::ObjSystem = ObjSystem;
	ObjectSLAM::GlobalInstance::ObjSystem = ObjSystem;
	ObjSystem->MapLatency.Update("assoseg", std::vector<double>());
	ObjSystem->MapLatency.Update("assosam", std::vector<double>());
	if (bSaveSegLatency) {
		ObjSystem->mbSaveLatency = true;
		ObjSystem->MapLatency.Update(deepkey1, std::vector<double>());
		ObjSystem->MapLatency.Update(deepkey2, std::vector<double>());
		ObjSystem->MapLatency.Update(deepkey3, std::vector<double>());
	}

	//StructOptimization::StructMapper::EdgeSLAMSystem = SLAM;
	//StructOptimization::StructMapper::BaseSLAMSystem = BaseSLAMSystem;
	SemanticSLAM::SemanticProcessor::BaseSLAMSystem = BaseSLAMSystem;

	//UVR_SLAM::System* SLAM_SYSTEM = new UVR_SLAM::System(strParamPath);
	//bool bSLAM_Running = false;
	//SLAM_SYSTEM->mptMappingServer = new std::thread(&UVR_SLAM::MappingServer::RunWithMappingServer, SLAM_SYSTEM->mpMappingServer);
	//SLAM_SYSTEM->mptLocalMapper = new std::thread(&UVR_SLAM::ServerMapper::RunWithMappingServer, SLAM_SYSTEM->mpServerMapper);
	//SLAM_SYSTEM->mptLoopCloser = new std::thread(&UVR_SLAM::LoopCloser::RunWithMappingServer, SLAM_SYSTEM->mpLoopCloser);
	//SLAM_SYSTEM->mptMapOptimizer = new std::thread(&UVR_SLAM::ServerMapOptimizer::RunWithMappingServer, SLAM_SYSTEM->mpServerMapOptimizer);
	////Run SLAM system
	
	//////////////Connect to echo server
	////Facade Connect
	{
		WebAPI* mpAPI = new WebAPI(ip, port);
		std::vector<std::string> sendKeywords;
		std::vector<std::string> pairKeywords;
		sendKeywords.push_back("ReferenceFrame");			pairKeywords.push_back("NONE");
		sendKeywords.push_back("RPlaneEstimation");		pairKeywords.push_back("NONE");
		sendKeywords.push_back("RequestSegmentation");		pairKeywords.push_back("Segmentation");
		sendKeywords.push_back("RequestObjectDetection");	pairKeywords.push_back("ObjectDetection");
		sendKeywords.push_back("RequestDepth");				pairKeywords.push_back("Depth");
		sendKeywords.push_back("reqdistdepth");				pairKeywords.push_back("distdepth");
		sendKeywords.push_back("reqdetectron");				pairKeywords.push_back("detectron");
		sendKeywords.push_back("reqyoloseg");				pairKeywords.push_back("yoloseg");
		sendKeywords.push_back("reqsam2");				pairKeywords.push_back("sam2");
		sendKeywords.push_back("reqraft");				pairKeywords.push_back("raft");
		//sendKeywords.push_back("MappingResult");			pairKeywords.push_back("NONE");
		sendKeywords.push_back("Content");					pairKeywords.push_back("ContentGeneration");
		////평면 정보 공유
		sendKeywords.push_back("PlaneLine");					pairKeywords.push_back("NONE");
		////슬램을 통해 가상 포인트 공유.
		sendKeywords.push_back("LocalContent");					pairKeywords.push_back("NONE");

		////실험용
		//sendKeywords.push_back("ContentGeneration");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("VO.MOVE");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("VO.MARKER.CREATED");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("TouchB");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("dr");					pairKeywords.push_back("NONE");
		//sendKeywords.push_back("ir");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("MarkerDist");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("ReqRecon");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("ReqSuperPoint");					pairKeywords.push_back("NONE");

		sendKeywords.push_back("NewKeyFrame");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("UpdatedLocalMap");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("NewFrameForRecon");					pairKeywords.push_back("NONE");


		//키프레임 기반 시멘틱 정보를 공유
		sendKeywords.push_back("ArUcoMarkerDetection");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("ArUcoMarkerDetection2");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("GetCloudAnchor");					pairKeywords.push_back("NONE");
		sendKeywords.push_back("ShareSemanticInfo");					pairKeywords.push_back("NONE");

		sendKeywords.push_back("WiseUITest");					pairKeywords.push_back("NONE");
		
		//sendKeywords.push_back("MarkerRegistration");					pairKeywords.push_back("NONE");
		//sendKeywords.push_back("LocalMap");
		//sendKeywords.push_back("LocalMapScales");
		//sendKeywords.push_back("LocalMapAngles");
		//sendKeywords.push_back("LocalMapPoints");
		//sendKeywords.push_back("LocalMapPointIDs");
		//sendKeywords.push_back("ReferenceFrameDesc");		pairKeywords.push_back("NONE");
		//sendKeywords.push_back("RequestFeature");

		time_t timer = 0;
		struct tm t;
 		timer = time(NULL); // 1970년 1월 1일 0시 0분 0초부터 시작하여 현재까지의 초

		for (int i = 0, iend = sendKeywords.size(); i < iend; i++) {
			
			localtime_s(&t, &timer);

			std::stringstream ss;
			ss << "{\"src\":\""<<"SLAMServer"<<"\","<<"\"keyword\":\"" << sendKeywords[i] << "\",\"type1\":\"server\",\"type2\":\"" << pairKeywords[i] << "\""<<",\"capacity\":"<<300<<"}"; //test\"}";
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
		receivedKeywords.push_back("DeviceConnect");
		receivedKeywords.push_back("DeviceDisconnect");
		receivedKeywords.push_back("ObjectDetection");
		receivedKeywords.push_back("Segmentation");
		receivedKeywords.push_back("distdepth");
		receivedKeywords.push_back("detectron");
		receivedKeywords.push_back("sam2");
		receivedKeywords.push_back("raft");
		receivedKeywords.push_back("yoloseg");
		receivedKeywords.push_back("yolosegc");
		receivedKeywords.push_back("sam2c");
				
		receivedKeywords.push_back("DevicePosition");
		//receivedKeywords.push_back("RPlaneEstimation");
		//receivedKeywords.push_back("ReferenceFrame");
		//receivedKeywords.push_back("Keypoints");
		//receivedKeywords.push_back("FeatureDetection");

		receivedKeywords.push_back("VO.CREATE");
		receivedKeywords.push_back("VO.MANIPULATE");
		receivedKeywords.push_back("VO.REQMOVE");
		receivedKeywords.push_back("VO.DRAW");
		receivedKeywords.push_back("VO.MARKER.CREATE");
		
		///////////test
		receivedKeywords.push_back("TestRef");
		receivedKeywords.push_back("TestInter");
		receivedKeywords.push_back("TouchA");
		///////////test

		//dslam
		receivedKeywords.push_back("DImage");
		receivedKeywords.push_back("DDepth");
		//dslam

		//stereo
		receivedKeywords.push_back("LImage");
		receivedKeywords.push_back("RImage");
		//stereo

		//structoptimization
		receivedKeywords.push_back("SImage");
		receivedKeywords.push_back("SDepth");
		//structoptimization

		/////동기화 테스트
		receivedKeywords.push_back("ds");
		receivedKeywords.push_back("is");
		/////동기화 테스트

		////ARFoundation 통합용
		receivedKeywords.push_back("SetCloudAnchor");
		receivedKeywords.push_back("DevicePose");
		receivedKeywords.push_back("MarkerResults");
		receivedKeywords.push_back("ARFoundationMPs");
		receivedKeywords.push_back("VO.Generation.Draw");
		receivedKeywords.push_back("Recon");
		////ARFoundation 통합용

		////EdgeBase
		receivedKeywords.push_back("ReqUpdateLocalMap");

		///OXR
		receivedKeywords.push_back("OXR::POSE");
		receivedKeywords.push_back("OXR::IMAGE");

		//Simul
		receivedKeywords.push_back("TrafficTest");
		receivedKeywords.push_back("SimImage");
		receivedKeywords.push_back("SimDeviceConnect");
		receivedKeywords.push_back("SimDeviceDisconnect");

		receivedKeywords.push_back("DevicePoseForAlign");

		std::stringstream ss;
		for (int i = 0, iend = receivedKeywords.size(); i < iend; i++) {
			ss << "{\"type1\":" << "\"connect\"" << ",\"type2\":" <<   "\"all\"" << ",\"src\":" << "\"SLAMServer\"" << ",\"keyword\":\"" << receivedKeywords[i] << "\"" << "}";
			std::string temp = ss.str();
			auto buf = temp.c_str();
			int retval;
			retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
			ss.str("");
			//std::cout << temp << std::endl;
		}
		delete mpAPI;
	}
	//////////////Connect to echo server
	/*{
		cv::Mat img;
		std::vector<cv::Mat> touchInfo(2);
		touchInfo[0] = cv::Mat::zeros(640, 480, CV_32SC1);
		touchInfo[1] = cv::Mat::zeros(640, 480, CV_32SC1);
		cv::circle(touchInfo[0], cv::Point2f(300, 300), 20, cv::Scalar(2), -1);
		cv::circle(touchInfo[1], cv::Point2f(300, 300), 20, cv::Scalar(100), -1);
		cv::merge(touchInfo, img);

		auto val = img.at<cv::Vec2i>(295, 295);
		if (val.val[0] == 0) {
			std::cout << "a;dlkfja;sldfja;lskfjas;ldfkj" << std::endl;
		}
		else if (val.val[0] == 2) {
			std::cout << "selection tes t= " << val.val[1] << std::endl;
		}
		std::cout << img.type() <<" "<<CV_32SC2<< " " << img.at<cv::Vec2i>(295, 295) << std::endl;
		std::cout << "asdf;ja;sdlfkjasdf" << std::endl;
	}*/

	////////Simulator
	////쓰레드로
	//{
	//	////CPU 사용량 체크
	//	cpuinit();

	//	////basic parallel test1
	//	int nServer = 8; //8,5
	//	//POOL->EnqueueJob(BasicSimulator, nServer);
	//	//POOL->EnqueueJob(BasicFastSimulator, POOL, nServer);

	//	////basic sequantial test
	//	//POOL->EnqueueJob(BasicSimulator2);
	//	//POOL->EnqueueJob(BasicFastSeqSimulator, POOL, nServer);

	//	////제안하는 시스템 패러렐 테스트
	//	std::string src = "Simulator1";
	//	std::string sKeyword1 = "TestParallelData";
	//	std::string sKeyword2 = "TestSequantialData";
	//	std::string keyword = sKeyword1;
	//	SimulatorSendConnect(src, keyword, 2);

	//	SimulatorReceiveConnect(src, "ResParallelTask1","partial");
	//	//SimulatorReceiveConnect(src, "ResParallelTask2","partial");
	//	//SimulatorReceiveConnect(src, "ResParallelTask3", "partial");
	//	//SimulatorReceiveConnect(src, "ResParallelTask4", "partial");
	//	//SimulatorReceiveConnect(src, "ResParallelTask5", "partial");
	//	//SimulatorReceiveConnect(src, "ResParallelTask6", "partial");
	//	//SimulatorReceiveConnect(src, "ResParallelTask7", "partial");
	//	//SimulatorReceiveConnect(src, "ResParallelTask8", "partial");
	//	
	//	//SimulatorReceiveConnect(src, "ResSequantialTask1", "seq");
	//	//SimulatorReceiveConnect(src, "ResSequantialTask2", "seq");
	//	//SimulatorReceiveConnect(src, "ResSequantialTask3", "seq");
	//	//SimulatorReceiveConnect(src, "ResSequantialTask5", "seq");
	//	//SimulatorReceiveConnect(src, "ResSequantialTask4", "seq"); ////end

	//	POOL->EnqueueJob(TestDataGenerator, keyword, src);

	//	//std::thread t2(SimulatorThread2);
	//	//t2.join();
	//	////t1.join();
	//	//
	//	//return 0;
	//}
	////////Simulator


	//SLAM->InitProcessingTime();
	//SLAM->SaveProcessingTime();
	//SLAM->LoadProcessingTime();
	std::map<int, float> segDatas;
	std::map<int, float> objDatas;

	
	////receive data
	while (true) {
		
		addrlen = sizeof(clientaddr);
		int retval = recvfrom(sock, rbuf, 1024, 0, (sockaddr*)&clientaddr, &addrlen);
		rbuf[retval] = '\0';

		//std::cout << "noti size = " << retval << std::endl;
		//std::cout << rbuf <<" "<<retval<< std::endl;

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
				POOL->EnqueueJob(Track, POOL, SLAM, src, ss.str(), id, ts,ts2);
				//POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::ObjectTracking, SLAM, src, id);
			}
			else if (keyword == "raft") {
				/*std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
				auto du_latency = (t_start.time_since_epoch().count() - (long long)ts) / 1000000.0;
				std::cout << "test raft latency = " << du_latency << " " << (long long)ts << std::endl;*/
				POOL->EnqueueJob(raft, SLAM, src, id, ts);
			}
			else if (keyword == "sam2c") {
				/*std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
				auto du_latency = (t_start.time_since_epoch().count() - (long long)ts) / 1000000.0;
				std::cout << "test sam2 latency = " << du_latency <<" "<< (long long)ts << std::endl;*/
				POOL->EnqueueJob(sam2, SLAM, src, id, ts, false);
			}
			else if (keyword == "yolosegc") {
				/*std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
				auto du_latency = (t_start.time_since_epoch().count() - (long long)ts) / 1000000.0;
				std::cout << "test yoloseg latency = " << du_latency << " " << (long long)ts << std::endl;*/
				POOL->EnqueueJob(yoloseg, SLAM, keyword, src, id, ts, false);
			}
			else if (keyword == "detectron") {
				/*if (!ObjSystem->MapObjectDevices.Count(User))
					return;
				auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
				User->mnUsed++;

				auto pNewBF = pObjDevice->mpCurrBF;
				auto pKF = pNewBF->mpRefKF;*/
				
				//POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::PanopticSegmentation, SLAM, src, id);
				//POOL->EnqueueJob(detectron3, SLAM, keyword, src, id);
			}
			else if(keyword=="distdepth")
			{ }
			else if (keyword == "SImage") {
				POOL->EnqueueJob(TrackPLP, POOL, SLAM, src, ss.str(), id, ts, ts2);
			}
			else if (keyword == "DDepth") {
				POOL->EnqueueJob(TrackRGBD, POOL, SLAM, src, ss.str(), id, ts, ts2);
			}
			else if (keyword == "RImage") {
				//std::cout << keyword << ", " << id << std::endl;
				POOL->EnqueueJob(TrackStereo, POOL, SLAM, src, ss.str(), id, ts, ts2);
			}
			else if(keyword == "SimImage") {
				POOL->EnqueueJob(SimTrack, POOL, SLAM, src, ss.str(), id, ts,ts2);
			}
			else if (keyword == "TrafficTest") {
				std::cout << "test traffic " << id << std::endl;
			}
			else if (keyword == "TestImage") {
				POOL->EnqueueJob(SemanticSLAM::CoordinateIntegration::TestImageReturn, SLAM, src, id);
			}
			else if (keyword == "DevicePose") {
				POOL->EnqueueJob(SemanticSLAM::CoordinateIntegration::DownloadPose, SLAM, src, id);
			}
			else if (keyword == "DevicePoseForAlign") {
				POOL->EnqueueJob(DeviceCoordAlignTest, POOL, SLAM, src, ss.str(), id, ts);
			}
			else if (keyword == "ARFoundationMPs") {
				POOL->EnqueueJob(SemanticSLAM::CoordinateIntegration::Process, SLAM, src, id);
			}
			else if (keyword == "MarkerResults") {
				POOL->EnqueueJob(SemanticSLAM::MarkerProcessor::MarkerEventDetect, SLAM, src, id);
				//POOL->EnqueueJob(SemanticSLAM::MarkerProcessor::MarkerRegistration, SLAM, src, id, 0.18, 0.07);
			}
			else if (keyword == "OXR::IMAGE") {
				SLAM->TrackOXR(id, src);
			}
			else if (keyword == "ResParallelTask1") {
				Evaluation(keyword, "partial", id);
				PrintCPU();
			}
			else if (keyword == "ResParallelTask2") {
				Evaluation(keyword, "partial", id);
			}
			else if (keyword == "ResParallelTask3") {
				Evaluation(keyword, "partial", id);
			}
			else if (keyword == "ResParallelTask4") {
				Evaluation(keyword, "partial", id);
			}
			else if (keyword == "ResParallelTask5") {
				Evaluation(keyword, "partial", id);
			}
			else if (keyword == "ResParallelTask6") {
				Evaluation(keyword, "partial", id);
			}
			else if (keyword == "ResParallelTask7") {
				Evaluation(keyword, "partial", id);
			}
			else if (keyword == "ResParallelTask8") {
				Evaluation(keyword, "partial", id);
			}
			else if (keyword == "ResSequantialTask1") {
				PrintCPU();
				Evaluation(keyword, "seq", id);
			}
			else if (keyword == "ResSequantialTask2") {
				Evaluation(keyword, "seq", id);
			}
			else if (keyword == "ResSequantialTask3") {
				Evaluation(keyword, "seq", id);
			}
			else if (keyword == "ResSequantialTask5") {
				Evaluation(keyword, "seq", id);
			}
			else if (keyword == "ResSequantialTask4") {

			}
			else if (keyword == "ds") {
				POOL->EnqueueJob(SemanticSLAM::ContentProcessor::DirectTest, SLAM, src, id);
			}
			/*else if (keyword == "is") {
				POOL->EnqueueJob(SemanticSLAM::ContentProcessor::IndirectTest, SLAM, src, id);
			}*/
			else if (keyword == "Gyro") {
				SLAM->UpdateDeviceGyroSensor(src, id);
				//cv::Ptr<cv::StereoSGBM> ptr = cv::StereoSGBM::create();
			} 
			else if (keyword == "DevicePosition") {
				SLAM->UpdateDevicePosition(src, id, ts);
			}
			else if(keyword == "ReqUpdateLocalMap") {
				//POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::SendLocalMap, SLAM, src, id);
			}
			else if (keyword == "ReferenceFrame") {
				////테스트 위치
				////POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::ObjectMapping, SLAM, src, id);

				////POOL->EnqueueJob(SemanticSLAM::ContentProcessor::IndirectSend, SLAM, src, id);
				//POOL->EnqueueJob(SemanticSLAM::ContentProcessor::ShareContent, SLAM, src, id);
				//POOL->EnqueueJob(SemanticSLAM::PlaneEstimator::UpdateLocalMapPlan es, SLAM, src, id); //에러 확인하기

				//////키프레임의 이미지와 수퍼포인트 정보를 전달.
				//POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::ShareSemanticInfo, SLAM, src, id);
				//POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::MatchingSuperPoint, SLAM, src, id);

				//POOL->EnqueueJob(SemanticSLAM::ContentProcessor::ShareAnchor, SLAM, src, id);
				//////마커 테스트
				//////이 과정은 마커로 일관성 측정시 다시 켜기
				////POOL->EnqueueJob(SemanticSLAM::MarkerProcessor::MarkerTest, SLAM, src, id, 0.18, 0.07);
				////POOL->EnqueueJob(SemanticSLAM::MarkerProcessor::MarkerRegistration, SLAM, src, id, 0.18, 0.07);
				//
				//////POOL->EnqueueJob(SemanticSLAM::MarkerProcessor::DynamicObjectRegTest, SLAM, src, id, 0.10, 0.07);
				////////POOL->EnqueueJob(SemanticSLAM::MarkerProcessor::DynamicObjectVisTest, SLAM, src, id);
				//////POOL->EnqueueJob(SemanticSLAM::MarkerProcessor::CalculateInconsistency, SLAM, src, id, 0.10, 0.07);

				////MarkerTest
			}
			else if (keyword == "DeviceDisconnect" || keyword == "SimDeviceDisconnect") {

				//std::cout << "Disconnect::start" << src << std::endl;
				
				//SemanticSLAM::MarkerProcessor::SaveInconsistency();
				//SemanticSLAM::ContentProcessor::SaveLatency();

				bool bSimul = keyword == "SimDeviceDisconnect";

				if (SLAM->CheckUser(src)) {
					auto pUser = SLAM->GetUser(src);
					pUser->mnUsed++;

					bool bGBA =  pUser->mbPlaneGBA;
					bool bReset = pUser->mbResetAR;
					//SLAM->SaveTrajectory(src);
					

					std::string tempMapName = pUser->mapName;     
					pUser->mnUsed--;
					SLAM->RemoveUser(src);
					if (keyword == "DeviceDisconnect" && !SLAM->CheckUser(src)) {
						SLAM->UpdateUserVisID();
					}
					
					//SLAM->CreateUser(user, mapName, w, h, fx, fy, cx, cy, d1, d2, d3, d4, bMapping);

					if (bReset) {
						SemanticSLAM::ContentProcessor::ResetContent(SLAM);
					}
					
					if(SLAM->CountUser()==0 && bGBA && SLAM->CheckMap(tempMapName)){
						//SemanticSLAM::PlaneEstimator::SaveNormal();
						auto pMap = SLAM->GetMap(tempMapName);
						EdgeSLAM::PlanarOptimizer::GlobalBundleAdjustemnt(SLAM, pMap);
						std::cout << "Map Update With Plane" << std::endl;
						SemanticSLAM::SemanticProcessor::MapUpdateWithPlane(SLAM, pMap);
						SemanticSLAM::GridProcessor::UpdateGridAndRegion();
						std::cout << "Update Graph Node" << std::endl;
						//SemanticSLAM::GridProcessor::UpdateSemanticGraphNode(SLAM);

						//SemanticSLAM::DepthProcessor::PlanarDepthMapUpdate(SLAM);
						//SemanticSLAM::SemanticProcessor::ObjectMapUpdateWithPlane(SLAM, pMap);
					}
					
					if (SLAM->GetConnectedDevice() == 0) {
						//std::cout << "save start = " << src << std::endl;

						if (bSaveSegLatency)
						{
							/*std::stringstream ss;
							ss << "../res/" << deepkey1 << ".csv";
							*/
							ObjSystem->SaveObjectAsso();
							ObjSystem->SaveLatency(deepkey1);
							ObjSystem->SaveLatency(deepkey2);
							ObjSystem->SaveLatency(deepkey3);
						}
						
						if (bSaveLatency){
							std::cout << "save start" << std::endl;
							//SLAM->Save(latencyPath, SLAM->EvaluationLatency, "num,source,id,quality,download,processing,upload,graph,preprocessing,total,size");
							SLAM->Save(latencyPath, SLAM->EvaluationLatency, "type,id,size");
							std::cout << "save end" << std::endl;
						}
						if (bSaveVOLatency)
							SLAM->Save(volatencyPath, SLAM->EvaluationVirtualObjectLatency, "num,source,method,type,id,size,latency");
						if (bSaveTrajectory) {
							std::cout << "save start" << std::endl;
							std::cout << trajectoryPath << std::endl;
							SLAM->SaveTrajectory(trajectoryPath, tempMapName);
							std::cout << "save end" << std::endl;
							return 0;
						}
						//SLAM->SaveProcessingTime();
						//std::cout << "save end = " << src << std::endl;
					}
					/*if ((!bSimul || src == "eCARSimulator_0")) {
						
					}*/
				}
				//std::cout << "Disconnect end = " << src << std::endl;
				//cv::circle(cv::Mat(), cv::Point(), 5, cv::Vec2i());
			}
			else if (keyword == "DeviceConnect" || keyword == "SimDeviceConnect") {

				WebAPI API(ip,port);
				auto res = API.Send(ss.str(), "");
				int nInt = 20;
				int nByte = 10;
				cv::Mat f = cv::Mat::zeros(nInt, 1, CV_32FC1);
				std::memcpy(f.data, res.data(), nInt * sizeof(float));
				int nSubStr = nInt * 4 + nByte; //문자열의 길이를 제거한 것. 11*4+3(bool)
				auto vstr = split(res.substr(nSubStr, res.size() - nSubStr), ',');
				std::string user = vstr[0];
				const std::string mapName = vstr[1];
				/*int nidx = 0;
				int w = (int)f.at<float>(nidx++);
				int h = (int)f.at<float>(nidx++);
				float fx = f.at<float>(nidx++);
				float fy = f.at<float>(nidx++);
				float cx = f.at<float>(nidx++);
				float cy = f.at<float>(nidx++);
				float d1 = f.at<float>(nidx++);
				float d2 = f.at<float>(nidx++);
				float d3 = f.at<float>(nidx++);
				float d4 = f.at<float>(nidx++);
				float d5 = f.at<float>(nidx++);
				int quality = (int)f.at<float>(nidx++);
				int nskip = (int)f.at<float>(nidx++);
				int nContentKFs = (int)f.at<float>(nidx++);*/
								
				int nbFlagIdx = nInt * 4;
				bool bMapping = (int)res[nbFlagIdx] ? true : false;
				bool bTracking = (int)res[nbFlagIdx+1] ? true : false;
				bool bIMU = (int)res[nbFlagIdx+2] ? true : false;
				bool bResetVO = (int)res[nbFlagIdx+3] ? true : false;
				bool bPlaneGBA = (int)res[nbFlagIdx+4] ? true : false;
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
				
				std::cout << "connect " << f.at<float>(0) << " " << f.at<float>(1) << std::endl;
				//depth 정보
				
				//std::cout << f.t() << ", " << src << std::endl;
				//std::cout << bEdgeBase <<" "<< bKFMethod<<" || " << " " << bMapping << " " << bTracking << " " << bIMU << " " << bSave << " " << bAsyncTest << "=" << vstr[0] << " " << vstr[1] << std::endl;
				//std::cout << "bCommuTest = " << bCommuTest << std::endl;
				//std::cout << "Content KFs = " << nContentKFs << std::endl;

				bool bSimulation = false;
				//
				if (keyword == "SimDeviceConnect") {
					user = src;
					bSimulation = true;
					//bMapping = false;
				}

				if (!SLAM->CheckMap(mapName)) {
					std::cout << "Create Map : " << mapName << std::endl;
					SLAM->CreateMap(mapName, (int)f.at<float>(11));
					std::cout << "Map Initialization Success : " << mapName << std::endl;
					SLAM->InitVisualizer(user, mapName, (int)f.at<float>(0), (int)f.at<float>(1));
					std::cout << "Visualizer Initialization Success" << std::endl;
				}
				if (!SLAM->CheckUser(user)) {
					std::cout << "Create User" << std::endl;
					SLAM->CreateUser(user, mapName, f, vbFlags);
					if(keyword =="DeviceConnect")
						SLAM->SetUserVisID(SLAM->GetUser(user));
					//오브젝트 슬램 확장
					auto p1 = SLAM->GetUser(user);
					auto p2 = new ObjectSLAM::ObjectDevice();
					ObjSystem->MapObjectDevices.Update(p1, p2);
				}


				{
					//BaseSLAM + StructOptimization
					//파라메터 관련 수정 필요함.
					//if (!StructSLAMSystem->isMapInSystem(mapName)) {
					//	//auto newMap = new BaseSLAM::AbstractMap(mapName);
					//	auto newMap = new StructOptimization::StructMap(mapName);
					//	StructSLAMSystem->AddMap(newMap);
					//}
					//auto tempMap = StructSLAMSystem->GetMap(mapName);
					//if (!tempMap->isDeviceInMap(user)) {
					//	{
					//		f.at<float>(14) = 40.0;
					//		f.at<float>(15) = 40.0;
					//		f.at<float>(16) = 5000.0;
					//	}
					//	auto newUser = new BaseSLAM::BaseDevice(user, mapName, f, vbFlags);
					//	tempMap->AddDevice(newUser);
					//}
				}

				////트래픽 저장용
				//{
				//	std::stringstream ssfile1;
				//	ssfile1 << "../bin/normal/base.txt";
				//	std::ofstream f1;
				//	f1.open(ssfile1.str().c_str());
				//	f1.close();
				//}
				//{
				//	std::stringstream ssfile1;
				//	ssfile1 << "../bin/normal/proposed_"<<quality<<".txt";
				//	std::ofstream f1;
				//	f1.open(ssfile1.str().c_str());
				//	f1.close();
				//}
				//트래픽 저장용

				//if (!SLAM_SYSTEM->GetMap(mapName)) {
				//	auto pNewMap = new UVR_SLAM::ServerMap(SLAM_SYSTEM);
				//	SLAM_SYSTEM->AddMap(mapName, pNewMap);
				//}

				//if (!SLAM_SYSTEM->GetUser(user)) {
				//	auto pNewUser = new UVR_SLAM::User(user, mapName, w, h, fx, fy, cx, cy, d1, d2, d3, d4, bMapping);
				//	SLAM_SYSTEM->AddUser(user, pNewUser);
				//}
				//auto pMap = SLAM_SYSTEM->GetMap(mapName);
				//auto pUser = SLAM_SYSTEM->GetUser(user);
				//pUser->mpServerMap = pMap;
				//SLAM_SYSTEM->mpVisualizer->AddUser(pUser);
				//	
				/////////매핑 서버 슬램 쓰레드
				//if (!bSLAM_Running) {
				//	SLAM_SYSTEM->ModuleInit();
				//	SLAM_SYSTEM->mpVisualizer->Init(w, h);
				//	SLAM_SYSTEM->mptVisualizer = new std::thread(&UVR_SLAM::Visualizer::RunWithMappingServer, SLAM_SYSTEM->mpVisualizer);
				//	bSLAM_Running = true;
				//	SLAM_SYSTEM->mpVisualizer->SetServerMap(SLAM_SYSTEM->GetMap(mapName));
				//}
			}
			else if (keyword == "Descriptors") {
				/*auto pUser = SLAM_SYSTEM->GetUser(src);
				if (!pUser) {
					std::cout << "user error" << std::endl;
					continue;
				}
				auto pFrame = pUser->mapTempFrames[id];
				pFrame->TurnOnFlag(UVR_SLAM::FLAG_DESC);*/

				//////Set descriptors
				//auto res = mpAPI->Send(ss.str(), "");
				//int n = pFrame->mvPts.size();
				//pFrame->matDescriptor = cv::Mat::zeros(n, 256, CV_32FC1);
				//std::memcpy(pFrame->matDescriptor.data, res.data(), n * 256 * sizeof(float));
				//////Set descriptors
			}
			else if (keyword == "Keypoints") {
				POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::DownloadSuperPoint, SLAM, src, id);
				//SLAM->DownloadKeyPoints(src, id);

				//auto pUser = SLAM_SYSTEM->GetUser(src);
				//if (!pUser) {
				//	std::cout << "user error" << std::endl;
				//	continue;
				//}
				//////set points
				//auto res = mpAPI->Send(ss.str(), "");
				//int n = res.size() / 8;

				//auto pFrame = pUser->mapTempFrames[id];
				//pFrame->TurnOnFlag(UVR_SLAM::FLAG_KP);
				//cv::Mat seg = cv::Mat::zeros(n, 2, CV_32FC1);
				//std::memcpy(seg.data, res.data(), res.size());
				//for (int i = 0; i < n; i++) {
				//	cv::Point2f pt(seg.at<float>(2 * i), seg.at<float>(2 * i + 1));
				//	pFrame->mvPts.push_back(pt);
				//}
				//if (!(pUser->D.at<float>(0) == 0.0 && pUser->D.at<float>(1) == 0.0)) {
				//	int N = pFrame->mvPts.size();
				//	cv::Mat mat(N, 2, CV_32F);

				//	for (int i = 0; i<N; i++)
				//	{
				//		mat.at<float>(i, 0) = pFrame->mvPts[i].x;
				//		mat.at<float>(i, 1) = pFrame->mvPts[i].y;
				//	}

				//	// Undistort points
				//	mat = mat.reshape(2);
				//	cv::undistortPoints(mat, mat, pUser->K, pUser->D, cv::Mat(), pUser->K);
				//	mat = mat.reshape(1);
				//	for (int i = 0; i<N; i++)
				//	{
				//		pFrame->mvPts[i].x = mat.at<float>(i, 0);
				//		pFrame->mvPts[i].y = mat.at<float>(i, 1);
				//	}
				//}

				//pFrame->SetGrids();
				//pFrame->SetMapPoints(pFrame->mvPts.size());
				//////set points
				//
				//auto trackingState = SLAM_SYSTEM->mpMappingServer->CalcTrackingState(pFrame);
				//std::string type2 = "";
				//bool bRequestMatching = false;
				//switch (trackingState) {
				//case UVR_SLAM::TrackingMethodState::Initialization:
				//	type2 = "Initialization";
				//	bRequestMatching = true;
				//	break;
				//case UVR_SLAM::TrackingMethodState::PoseEstimation:
				//	type2 = "PoseEstimation";
				//	bRequestMatching = true;
				//	break;
				//case UVR_SLAM::TrackingMethodState::InitLocalization:
				//	type2 = "InitLocalization";
				//	bRequestMatching = true;
				//	break;
				//case UVR_SLAM::TrackingMethodState::SubLocalization:
				//	type2 = "SubLocalization";
				//	bRequestMatching = true;
				//	break;
				//default:
				//	type2 = "Parsing Error";
				//	break;
				//}
				////std::cout << "Tracking Method = " << type2 << std::endl;

				//if (bRequestMatching && !pUser->mbProgress) {
				//	pUser->mbProgress = true;
				//	pUser->mnPrevFrameID = pUser->mnCurrFrameID.load();
				//	pUser->mnCurrFrameID = id;
				//	pUser->mpTargetFrame = pFrame;
				//	WebAPI* mpAPI = new WebAPI("143.248.6.143", 35005);
				//	std::stringstream ss;
				//	//ss << "/Store?keyword=Matching&id=" << id << "&id2=" << pUser->mnReferenceFrameID << "&src=" << src << "&type2=" << type2;
				//	ss << "/Store?keyword=Matching&id=" << pUser->mnReferenceFrameID << "&id2=" << id << "&src=" << src << "&type2=" << type2;
				//	auto res = mpAPI->Send(ss.str(), "");//std::to_string(targetid)
				//}

				//////Insert Frame
				////SLAM_SYSTSEM->mpMappingServer->InsertFrame(pFrame);
				//////frame

				//////feature matching request
				//int fsize = pUser->vecTempFrameIDs.size();
				//if (fsize > 0 && fsize % 3 == 0) {
				//	int targetid = pUser->vecTempFrameIDs[fsize - 3];
				//	WebAPI* mpAPI = new WebAPI("143.248.6.143", 35005);
				//	std::stringstream ss;
				//	ss << "/Store?keyword=Matching&id=" << id <<"&id2="<<targetid<< "&src="<<src<<"&type2="<<type2;
				//	auto res = mpAPI->Send(ss.str(), "");//std::to_string(targetid)
				//}
				//pUser->vecTempFrameIDs.push_back(id);
				//////feature matching request
			}
			else if (keyword == "Segmentation") {
				//POOL->EnqueueJob(Segmentation, SLAM, src, id);
				POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::Segmentation, SLAM, src, id);
				
				//POOL->EnqueueJob(SemanticSLAM::PlaneEstimator::PlaneEstimation, SLAM, src, id);
				//POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::DenseOpticalFlow, SLAM, src, id);
			}
			else if (keyword == "RPlaneEstimation") {
				//POOL->EnqueueJob(SemanticSLAM::PlaneEstimator::PlaneEstimation, SLAM, src, id);
			}
			else if (keyword == "ObjectDetection") {
				//POOL->EnqueueJob(ObjectDetection3, SLAM, src, id);
				//POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::ObjectDetection, SLAM, src, id);
			}
			else if (keyword == "Recon") {
				POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::SimpleRecon, SLAM, src, id);
				/*auto res = POOL->EnqueueJob(SemanticSLAM::ContentProcessor::testMethod);
				auto var = res.get();
				auto temp = std::get<0>(var);*/
			}
			else if (keyword == "Depth") {
				//SLAM->ProcessDepthEstimation(src, id);
				POOL->EnqueueJob(SemanticSLAM::DepthProcessor::DepthProcessing, SLAM, src, id);
			}
			else if (keyword == "VO.CREATE") {
				POOL->EnqueueJob(SemanticSLAM::ContentProcessor::ContentProcess, SLAM, src, id,keyword, ts,0);
			}
			else if (keyword == "VO.DRAW") {
				POOL->EnqueueJob(SemanticSLAM::ContentProcessor::DrawContentProcess, SLAM, src, id, keyword, 0);
			}
			else if (keyword == "SetCloudAnchor") {
				POOL->EnqueueJob(SemanticSLAM::ContentProcessor::SetAnchor, SLAM, src, id, keyword, 0);
			}
			else if (keyword == "VO.Generation.Draw") {
				POOL->EnqueueJob(SemanticSLAM::ContentProcessor::ContentProcess, SLAM, src, id,keyword,ts, 200);//마커 아이디랑 같이 씀. 일단은, 마커는 100까지, 컨텐츠 보낼 때 데이터 추가 해야 함.
			}
			else if (keyword == "VO.MANIPULATE") {
				POOL->EnqueueJob(SemanticSLAM::ContentProcessor::UpdateProcess, SLAM, src, id);
			}
			else if (keyword == "VO.REQMOVE") {
				POOL->EnqueueJob(SemanticSLAM::ContentProcessor::MovingObjectSync, SLAM, src, id);
			}
			else if (keyword == "VO.MARKER.CREATE") {
				//POOL->EnqueueJob(SemanticSLAM::ContentProcessor::MovingObjectSync, SLAM, src, id);
				POOL->EnqueueJob(SemanticSLAM::MarkerProcessor::MarkerRegistration,SLAM, keyword, src, id);
			}
			else if (keyword == "TestRef" || keyword == "TestInter") {
				POOL->EnqueueJob(SemanticSLAM::ContentProcessor::UpdateLatency, keyword, src, id);
			}
			else if (keyword == "TouchA") {
				POOL->EnqueueJob(SemanticSLAM::ContentProcessor::TestTouch, src, id);
			}
			else if (keyword == "Matches") {
				//auto pUser = SLAM_SYSTEM->GetUser(src);
				//if (!pUser) {
				//	std::cout << "user error" << std::endl;
				//	continue;
				//}
				//auto pMap = pUser->mpServerMap;
				//std::string type = document["type2"].GetString();
				////std::cout << "Matches = " << type << std::endl;
				//				
				//////matching test
				//////find f
				//////usac param
				//cv::UsacParams uparam = cv::UsacParams();
				//uparam.isParallel = true;
				//uparam.score = cv::SCORE_METHOD_MAGSAC;
				//uparam.loMethod = cv::LOCAL_OPTIM_SIGMA;
				//////usac param

				//auto res = mpAPI->Send(ss.str(), "");
				//std::vector<cv::Point2f> pts1, pts2;
				//auto pFrame1 = pUser->mapTempFrames[id];
				//auto pFrame2 = pUser->mapTempFrames[id2];
				//
				//cv::Mat matches = cv::Mat::zeros(pFrame1->mvPts.size(), 1, CV_32SC1);
				//std::memcpy(matches.data, res.data(), matches.rows * sizeof(int));
				////cv::Mat img = pFrame1->matFrame.clone();
				////for (size_t i = 0, iend = matches.rows; i < iend; i++) {
				////	int idx1 = i;
				////	int idx2 = matches.at<int>(i);
				////	if (idx2 == -1)
				////		continue;
				////	auto pt1 = pFrame1->mvPts[idx1];
				////	auto pt2 = pFrame2->mvPts[idx2];
				////	pts1.push_back(pt1);
				////	pts2.push_back(pt2);
				////	//cv::circle(img, pt1, 3, cv::Scalar(255, 0, 255), -1);
				////	cv::line(img, pt1, pt2, cv::Scalar(255, 255, 0), 2);
				////}
				//
				//
				//pool->EnqueueJob(UVR_SLAM::MappingServer::DeviceLocalize, type, pool, SLAM_SYSTEM, pUser, id, id2, matches);


				//std::cout << "?????????" << std::endl;
				////cv::Mat F = cv::findFundamentalMat(pts1, pts2, cv::Mat(), uparam);
				//cv::sfm::
				//cv::sfm::reconstruct();
				//std::cout << "F = " << F << std::endl;

				/*{
					std::stringstream ss;
					ss << "img=match=" << src;
					cv::imshow(ss.str(), img); cv::waitKey(1);
				}*/

				////matching test
				/*int id2 = document["id2"].GetInt();

				WebAPI* mpAPI = new WebAPI("143.248.6.143", 35005);
				{
					std::stringstream ss;
					ss << "/Load?keyword=Matches" << "&id=" << id<<"&id2="<<id2;
					auto res = mpAPI->Send(ss.str(), "");
				}*/


			}
			////data processing
			//delete mpAPI;
		}

	}

	return 0;
}

