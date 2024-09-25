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
#include <ObjectSLAM.h>
#include <ObjectMap.h>
#include <ObjectPoint.h>
#include <ObjectInitializer.h>
#include <ObjectMatcher.h>
#include <ObjectMapper.h>
#include <InstanceLinker.h>
#include <ObjectMatchingInfo.h>

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
bool bSaveLatency = false;
std::string latencyPath;
bool bSaveVOLatency = false;
std::string volatencyPath;
bool bSaveTrajectory = false;
std::string trajectoryPath;

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
ObjectSLAM::ObjectSLAM *ObjSystem = new ObjectSLAM::ObjectSLAM();
ConcurrentMap<EdgeSLAM::KeyFrame*, ObjectSLAM::BoxFrame*> AllBoxFrames;
void ObjectDetection2(EdgeSLAM::SLAM* SLAM, std::string user, int id) {
	////전처리	
	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	/*if (!User->KeyFrames.Count(id))
		return;
	auto pKF = User->KeyFrames.Get(id);
	if (!pKF) {
		return;
	}*/
	if (!ObjSystem->MapObjectDevices.Count(User))
		return;
	if (!ObjSystem->MapKeyFrameNBoxFrame.Count(id))
	{
		std::cout << "find keyframe failed " << id << std::endl;
		return;
	}

	auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
	User->mnUsed++;
	auto pNewBF = ObjSystem->MapKeyFrameNBoxFrame.Get(id);//pObjDevice->mpCurrBF;
	
	auto pKF = pNewBF->mpRefKF;
	
	{ 
		std::unique_lock<std::mutex> lock(ObjSystem->mMutexMatches);
		for (auto ppair : ObjSystem->MapMatches) {
			auto pair = ppair.first;
			auto pMatch = ppair.second;
			if (pair.second == id) {
				std::cout << "match = " << pair.first << " " << pair.second << std::endl;
			} 
		}
		/*auto pair = std::make_pair(id, pKF->mnFrameId);
		if (ObjSystem->MapMatches.count(pair)) {
			std::cout << "asdf = " << pair.first<<" "<<pair.second << std::endl;
		}
		else {
			std::cout << "not asdf = " << pair.first << " " << pair.second << std::endl;
		}*/
	}

	/*auto pNewBF = pObjDevice->mpCurrBF;
	auto pKF = pNewBF->mpRefKF;*/

	std::string mapName = User->mapName;
	bool mbSave = User->mbSave;

	std::stringstream ss;
	ss << "/Download?keyword=ObjectDetection" << "&id=" << id << "&src=" << user;
	WebAPI API("143.248.6.143", 35005);
	auto res = API.Send(ss.str(), "");

	cv::Mat data = cv::Mat(res.size() / 24, 6, CV_32FC1, (void*)res.data());

	cv::Mat K = pKF->K.clone();
	cv::Mat R = pKF->GetRotation();
	cv::Mat KR = K * R;

	cv::Mat gray;
	cv::cvtColor(pNewBF->img, gray, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAY
	////전처리	

	//박스 프레임 생성
	//auto pNewBF = new ObjectSLAM::BoxFrame(id, gray.cols, gray.rows, User, (BaseSLAM::AbstractPose*)pKF->mpCamPose);

	//바운딩 박스 생성
	cv::Mat image = pNewBF->img.clone();
	for (int j = 0; j < data.rows; j++) {
		int label = (int)data.at<float>(j, 0) + 1;
		float conf = data.at<float>(j, 1);
		cv::Point2f left(data.at<float>(j, 2), data.at<float>(j, 3));
		cv::Point2f right(data.at<float>(j, 4), data.at<float>(j, 5));
		cv::Rect rect(left, right);

		auto pBox = new ObjectSLAM::BoundingBox(pNewBF, pKF->fx, pKF->fy, pKF->cx, pKF->cy, label, conf, left, right, User);
		pBox->SetPose(pNewBF->GetPose());
		pNewBF->mvpBBs.push_back(pBox);

		auto pair = std::make_pair(rect, std::vector<cv::Point2f>());
		
		//객체 정보
		cv::rectangle(image, left, right, cv::Scalar(255, 255, 255), 2);
		cv::Point2f pt(left.x + (right.x - left.x) / 2, left.y + 20);
		cv::putText(image, SemanticSLAM::SemanticProcessor::vecStrObjectLabels[label - 1], pt, 2, 1.2, cv::Scalar(255, 255, 255));
	}
	pNewBF->mbYolo = true;
	User->mnUsed--;
	
	//pNewBF->img = image.clone();
	
	//테스트 필요함.
	//pNewBF->BaseObjectRegistration(pKF);

	SLAM->VisualizeImage(User->mapName, image, 0);
	//여기까지가 오브젝트 디텍션.
	return;
	auto pPrevBF = pObjDevice->mpPrevBF;
	
	//이전 프레임과 매칭
	std::vector<uchar> features_found;
	std::vector<cv::Point2f> tempMatchingPoints;
	std::vector<std::pair<cv::Point2f, cv::Point2f>> vecPairVisualizedMatches;
	std::vector<cv::Point2f> currCorners, prevCorners;
	if (pPrevBF) {
		//tracking
		currCorners.clear();
		prevCorners.clear();
		
		cv::Mat prevGray = pPrevBF->gray.clone();
		cv::Mat prev = pPrevBF->img.clone();

		//코너 포인트 생성

		pPrevBF->ConvertBoxToFrame(prev.cols, prev.rows);
		std::vector<std::pair<int, int>> vecPairPointIdxInBox;//에러 처리 = pPrevBF->mvPairFrameAndBox;
		for (int i = 0, N = pPrevBF->mvKeyDatas.size(); i < N; i++) {
			prevCorners.push_back(pPrevBF->mvKeyDatas[i].pt);
		}

		cv::Mat used = cv::Mat::zeros(prevGray.size(), CV_8UC1);
		
		std::vector<cv::KeyPoint> tempMatchingPrevKP, tempMatchingCurrKP;
		cv::Mat tempPrevDesc = cv::Mat::zeros(0, 32, CV_8UC1);
		/*for (int i = 0; i < pPrevBF->mvpBBs.size(); i++) {
			auto pPrevBB = pPrevBF->mvpBBs[i];
			for (int j = 0; j < pPrevBB->N; j++) {
				auto kp = pPrevBB->mvKeyDatas[j];
				auto pt = pPrevBB->mvKeyDatas[j].pt;
				if (used.at<uchar>(pt))
					continue;
				used.at<uchar>(pt)++; `
				prevCorners.push_back(pt);
				vecPairPointIdxInBox.push_back(std::make_pair(i, j));
			}
		}*/
		
		int win_size = 10;
		cv::calcOpticalFlowPyrLK(
			prevGray,                         // Previous image
			gray,                         // Next image
			prevCorners,                     // Previous set of corners (from imgA)
			currCorners,                     // Next set of corners (from imgB)
			features_found,               // Output vector, each is 1 for tracked
			cv::noArray(),                // Output vector, lists errors (optional)
			cv::Size(win_size * 2 + 1, win_size * 2 + 1),  // Search window size
			5,                            // Maximum pyramid level to construct
			cv::TermCriteria(
				cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
				20,                         // Maximum number of iterations
				0.3                         // Minimum change per iteration
			)
		);
		
		//epipolar constraints
		cv::Mat T1 = pPrevBF->GetPose();
		cv::Mat T2 = pNewBF->GetPose();
		cv::Mat R1 = T1.rowRange(0, 3).colRange(0, 3);
		cv::Mat t1 = T1.rowRange(0, 3).col(3);
		cv::Mat R2 = T2.rowRange(0, 3).colRange(0, 3);
		cv::Mat t2 = T2.rowRange(0, 3).col(3);
		cv::Mat F12 = CommonUtils::Geometry::ComputeF12(R1, t1, R2, t2, pPrevBF->K, pNewBF->K);
		//std::cout <<"Obj::F=" << F12 << std::endl;
		//박스와 박스 나누기
	
		//현재 프레임에 매칭 정보 이동

		//매칭 정보 박스에 전파
		//std::vector<std::pair<int, int>> vecCurrPairPointIdxInBox(vecPairPointIdxInBox.size(), std::make_pair(-1,-1));
		std::vector<int> tempIDXs;
		for (int i = 0; i < static_cast<int>(prevCorners.size()); ++i) {
			if (!features_found[i]) {
				continue;
			}
			auto pt = currCorners[i];
			//디스크립터 계산 가능한 영역 안의 키포인트 검출
			if (pt.x < 20 || pt.x >= gray.cols - 20 || pt.y < 20 || pt.y >= gray.rows - 20)
				continue;

			auto prevPair = vecPairPointIdxInBox[i];
			int prevId = prevPair.first;
			int prevIdx = prevPair.second;

			auto kp = pPrevBF->mvpBBs[prevId]->mvKeyDatas[prevIdx];
			auto op = pPrevBF->mvpBBs[prevId]->mvpMapDatas.get(prevIdx);
			
			//epipolar
			if (!CommonUtils::Geometry::CheckDistEpipolarLine(prevCorners[i], pt, F12, pKF->mvLevelSigma2[kp.octave]))
				continue;

			tempMatchingPrevKP.push_back(kp);
			tempPrevDesc.push_back(pPrevBF->mvpBBs[prevId]->mDescriptors.row(prevIdx));
			
			kp.pt = pt;
			tempMatchingCurrKP.push_back(kp);
			
			tempIDXs.push_back(i);

			//디스크립터 가능한 포인트만 추가하자.
			//for (int j = 0; j < pNewBF->mvpBBs.size(); j++) {
			//	auto pCurrBB = pNewBF->mvpBBs[j];
			//	if (pCurrBB->mRect.contains(pt)) {
			//		//auto pair1 = std::make_pair()
			//	}
			//}
		}
		
		cv::Mat tempCurrDesc;
		SLAM->mpFeatureTracker->computeDescriptor(gray, tempMatchingCurrKP, tempCurrDesc);

		//calculate descriptor distance
		std::vector<bool> vbInliers(tempMatchingPrevKP.size(), false);
		std::map<std::pair<int, int>, int> mapCountMatchBoxAndBox;
		std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> mapMatchBoxAndBox;
		std::map<int, int> mapCurrBoxIndex;
		for (int i = 0; i < tempMatchingPrevKP.size(); i++) {
			int idx = tempIDXs[i];
			cv::Mat d1 = tempPrevDesc.row(i);
			cv::Mat d2 = tempCurrDesc.row(i);
			const int dist = Utils::CalcBinaryDescriptor(d1, d2);

			if (dist < 100) {
				vbInliers[i] = true;
				//vecMatches.push_back(std::make_pair(tempMatchingPrevKP[i].pt, tempMatchingCurrKP[i].pt));
				//check current bounding box
				//check previous bounding box
				int prevBoxIdx = vecPairPointIdxInBox[idx].first;
				int prevPointIdx = vecPairPointIdxInBox[idx].second;
				auto kp2 = tempMatchingCurrKP[i];
				cv::Mat d2 = tempCurrDesc.row(i);

				for (int j = 0; j < pNewBF->mvpBBs.size(); j++) {
					auto pCurrBB = pNewBF->mvpBBs[j];
					if (pCurrBB->mRect.contains(kp2.pt)) {
						//auto pair1 = std::make_pair()
						auto tPair = std::make_pair(prevBoxIdx, j);
						mapCountMatchBoxAndBox[tPair]++;
						mapMatchBoxAndBox[tPair].push_back(std::make_pair(prevPointIdx, mapCurrBoxIndex[j]++));
						pCurrBB->AddData(kp2, d2);
						auto diffPt = cv::Point2i(kp2.pt) - pCurrBB->mRect.tl();
						pCurrBB->mUsed.at<uchar>(diffPt)++;
					}
				}
			}
		}
		
		//박스와 박스 매칭
		std::vector<std::pair<int, int>> vecMaxCountBoxAndBox;
		for (int i = 0; i < pPrevBF->mvpBBs.size(); i++) {
			std::pair<int, int> max_idx = std::make_pair(-1, -1);
			int max_val = -1;
			for (int j = 0; j < pNewBF->mvpBBs.size(); j++) {
				auto pair = std::make_pair(i, j);
				int val = mapCountMatchBoxAndBox[pair];
				if (val > max_val) {
					max_val = val;
					max_idx = pair;
				}
			}
			if(max_val > 20){
				vecMaxCountBoxAndBox.push_back(max_idx);
			}
		}
		//std::cout << pKF->mpCamPose == pNewBF->mpPose << std::endl;
		//std::cout << "1" << std::endl;
		//triangulation
		for (auto pair : vecMaxCountBoxAndBox) {
			auto vecMatches = mapMatchBoxAndBox[pair];
			auto pB1 = pPrevBF->mvpBBs[pair.first];
			auto pB2 = pNewBF->mvpBBs[pair.second];

			std::vector<std::pair<bool, cv::Mat>> vecTriangulated;
			int a = ObjectSLAM::ObjectMapper::TwoViewTriangulation(pB1, pB2, vecMatches, vecTriangulated);
			
			for (int i = 0; i < vecMatches.size(); i++) {
				auto pair = vecTriangulated[i];
				if (!pair.first)
					continue;
				cv::Mat X3D = pair.second;

				int i1 = vecMatches[i].first;
				int i2 = vecMatches[i].second;
				auto pNewOP = new ObjectSLAM::ObjectPoint(pB1);
				pNewOP->SetData(X3D);
				pNewOP->SetDescriptor(pB1->mDescriptors.row(i1).clone());
				pB1->mvpMapDatas.update(i1, pNewOP);
				pB2->mvpMapDatas.update(i2, pNewOP);
				pNewOP->AddObservation(pB1, i1);
			}

			//pB2->Init();
			if (!pB1->mpMap) {
				/*
				std::cout << "tri = " << a << " " << vecMatches.size() << std::endl;
				*/
				//오브젝트 맵 생성
				//일단 맵포인트만 생성하고 추가하기
				if (a > 20) {
					auto pNewMap =new ObjectSLAM::ObjectMap();
					pB1->mpMap = pNewMap;
					pB2->mpMap = pNewMap;
				}
			}
			else {
				//오브젝트 포즈 추적 및 맵포인트 매칭 성능 확인
				
				//일단 매칭 되면 넘기기
				//오브젝트 맵 추가하기

				auto mvpOPs = pB1->mvpMapDatas.get();
				int a = 0;
				for (int i = 0; i < vecMatches.size(); i++) {
					int i1 = vecMatches[i].first;
					int i2 = vecMatches[i].second;
					auto pOP = mvpOPs[i1];
					if (pOP) {
						pB2->mvpMapDatas.update(i2, pOP);
						a++;
						vecPairVisualizedMatches.push_back(std::make_pair(pB1->mvKeyDatas[i1].pt, pB2->mvKeyDatas[i2].pt));
					}
				}
				if (a > 10) {
					pB2->mpMap = pB1->mpMap;
				}
				//std::cout << "tracking = " <<a++<< std::endl;
			}
			//pB2->StereoDataInit(pB2->mvKeyDatas.size());
			//std::cout << vecMatches.size() << " " << std::endl;
			//std::cout << pair.first<<" "<<pair.second << std::endl;
			
			//맵 포인트 생성
		}
		//std::cout << "2" << std::endl;
		//for (auto pBB : pNewBF->mvpBBs) {
		//	pBB->Init();
		//	//pBB->StereoDataInit(pBB->N);
		//	//i++;
		//	//if (pBB->N != mapMatchBoxAndBox[i])
		//	//	std::cout << "err = " << pBB->N << " " << mapCountMatchBoxAndBox[i] << std::endl;
		//}

		//visualization

		//mapping

		cv::Mat resImage;
		SLAM->VisualizeMatchingImage(resImage, prev, image, vecPairVisualizedMatches, User->mapName, 2);
		if (User->mbSave) {
			std::stringstream ss;
			ss << "../res/images/" << id << "_yolo.png";
			cv::imwrite(ss.str(), resImage);
		}
	}
	
	//add current keypoint that not occupied in current key frame
	auto vpBBs = pNewBF->mvpBBs; 
	for (int i = 0; i < pKF->N; i++)
	{
		auto kp = pKF->mvKeys[i];
		auto pt = kp.pt;
		//std::cout << i <<" "<<pKF->N << std::endl;
		//pNewBF->mUsed.at<uchar>(pt)++;
		for (auto pBBi : vpBBs) {
			if (pBBi->mvKeyDatas.size() > 100)
				continue;
			if (!pBBi->mRect.contains(pt))
				continue;
			
			auto diff = cv::Point2i(pt) - pBBi->mRect.tl();
			if (diff.x < 0.0 || diff.y < 0.0 || diff.x >= pBBi->mRect.width || diff.y >= pBBi->mRect.height)
				continue;
			if (pBBi->mUsed.at<uchar>(diff))
				continue;
			pBBi->mUsed.at<uchar>(diff)++;
			pBBi->AddData(pKF->mvKeys[i], pKF->mDescriptors.row(i));
		}
	}
	
	//box initialization
	/*for (auto pBB : pNewBF->mvpBBs) {
		pBB->Init();
	}*/

	//업데이트
	//pObjDevice->imPrev = image.clone();
	//pObjDevice->imPrevGray = gray.clone();
	//pObjDevice->mpPrevBF = pNewBF;
	
}
void ObjectDetection(EdgeSLAM::SLAM* SLAM, std::string user, int id) {
	//auto User = SLAM->GetUser(user);
	//if (!User)
	//	return;
	//if (!User->KeyFrames.Count(id))
	//	return;
	//auto pKF = User->KeyFrames.Get(id);
	//if (!pKF) {
	//	return;
	//}

	//cv::Mat encoded = User->ImageDatas.Get(id);
	//cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
	//if (image.empty())
	//{
	//	std::cout << "decoding error" << std::endl;
	//	return;
	//}

	//User->mnUsed++;
	////StructOptimization::StructMap* tempMap = (StructOptimization::StructMap*)BaseSLAMSystem->GetMap(User->mapName);
	////auto device = tempMap->GetDevice(user);

	//std::stringstream ss;
	//ss << "/Download?keyword=ObjectDetection" << "&id=" << id << "&src=" << user;
	//WebAPI API("143.248.6.143", 35005);
	//auto res = API.Send(ss.str(), "");

	//cv::Mat data = cv::Mat(res.size()/24, 6, CV_32FC1, (void*)res.data());

	//cv::Mat K = pKF->K.clone();
	//cv::Mat R = pKF->GetRotation();
	//cv::Mat KR = K * R;

	//cv::Mat gray;
	//cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);//COLOR_BGR2GRAY		

	////이전 프레임에서 옵8티컬 플로우 트래킹
	//std::vector<uchar> features_found;
	//std::vector<cv::Point2f> tempMatchingPoints;
	//if (bPrev) {
	//	//tracking
	//	currCorners.clear();

	//	//new bf
	//	prevCorners.clear();
	//	for (auto kp : pPrevBF->mvKeyDatas) {
	//		prevCorners.push_back(kp.pt);
	//	}

	//	int win_size = 10;
	//	cv::calcOpticalFlowPyrLK(
	//		prevGray,                         // Previous image
	//		gray,                         // Next image
	//		prevCorners,                     // Previous set of corners (from imgA)
	//		tempMatchingPoints,                     // Next set of corners (from imgB)
	//		features_found,               // Output vector, each is 1 for tracked
	//		cv::noArray(),                // Output vector, lists errors (optional)
	//		cv::Size(win_size * 2 + 1, win_size * 2 + 1),  // Search window size
	//		5,                            // Maximum pyramid level to construct
	//		cv::TermCriteria(
	//			cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
	//			20,                         // Maximum number of iterations
	//			0.3                         // Minimum change per iteration
	//		)
	//	);

	//}
	//
	//auto pNewBF = new ObjectSLAM::BoxFrame(id, gray.cols, gray.rows, User, (BaseSLAM::AbstractPose*)pKF->mpCamPose);
	////auto pNewBF = new ObjectSLAM::NewBoxFrame(gray.cols, gray.rows);
	//std::vector<std::pair<cv::Rect, std::vector<cv::Point2f>>> vecPairRectAndPoints;
	//std::vector<std::vector<cv::KeyPoint>> vecRectKPs;
	//std::vector<cv::Mat> vecRectDescs;
	//for (int j = 0; j < data.rows; j++) {
	//	int label = (int)data.at<float>(j, 0) + 1;
	//	float conf = data.at<float>(j, 1);
	//	cv::Point2f left(data.at<float>(j, 2), data.at<float>(j, 3));
	//	cv::Point2f right(data.at<float>(j, 4), data.at<float>(j, 5));
	//	cv::Rect rect(left, right);
	//	
	//	auto pBox = new ObjectSLAM::BoundingBox(pNewBF, pKF->fx, pKF->fy, pKF->cx, pKF->cy, label, conf, left, right, User);
	//	pNewBF->mvpBBs.push_back(pBox);

	//	auto pair = std::make_pair(rect, std::vector<cv::Point2f>());
	//	vecPairRectAndPoints.push_back(pair);
	//	vecRectKPs.push_back(std::vector<cv::KeyPoint>());
	//	vecRectDescs.push_back(cv::Mat::zeros(0, 32, CV_8UC1));
	//	//VP 및 객체 정보 시각화
	//	///*auto C = left + right;
	//	//C /= 2;
	//	//cv::Mat x = KR.col(0);
	//	//float d1 = x.at<float>(2);
	//	//auto VP1 = cv::Point(x.at<float>(0) / d1, x.at<float>(1) / d1);
	//	//cv::Mat y = KR.col(1);
	//	//float d2 = y.at<float>(2);
	//	//auto VP2 = cv::Point(y.at<float>(0) / d2, y.at<float>(1) / d2);
	//	//cv::Mat z = KR.col(2);
	//	//float d3 = z.at<float>(2);
	//	//auto VP3 = cv::Point(z.at<float>(0) / d3, z.at<float>(1) / d3);
	//	//cv::line(image, VP1, C, cv::Scalar(0, 0, 255), 2);
	//	//cv::line(image, VP2, C, cv::Scalar(0, 255, 0), 2);
	//	//cv::line(image, VP3, C, cv::Scalar(255, 0, 0), 2);
	//	//*/
	//	//객체 정보
	//	cv::rectangle(image, left, right, cv::Scalar(255, 255, 255), 2);
	//	cv::Point2f pt(left.x + (right.x-left.x) / 2, left.y + 20);
	//	cv::putText(image, SemanticSLAM::SemanticProcessor::vecStrObjectLabels[label-1], pt, 2, 1.2, cv::Scalar(255, 255, 255));
	//}

	////포인트 연결

	////cv::Mat desc;
	////SLAM->mpFeatureTracker->computeDescriptor(gray, vecTotalKPs, desc);
	//
	////옵티컬 플로우에 키포인트가 있는지 체크
	//cv::Mat used(gray.size(), CV_8UC1);
	//auto vpBBs = pNewBF->mvpBBs;
	//for (int i = 0; i < pKF->N; i++)
	//{
	//	auto pt = pKF->mvKeys[i].pt;
	//	used.at<uchar>(pt)++;
	//	//pNewBF->mUsed.at<uchar>(pt)++;
	//	for (auto pBBi : vpBBs) {
	//		if (!pBBi->mRect.contains(pt))
	//			continue;
	//		pBBi->AddData(pKF->mvKeys[i], pKF->mDescriptors.row(i));
	//	}

	//	for (int j = 0; j < vecPairRectAndPoints.size(); j++) {
	//		auto rect = vecPairRectAndPoints[j].first;
	//		auto corners = vecPairRectAndPoints[j].second;
	//		if (rect.contains(pt)) {
	//			cv::circle(image, pt, 2, cv::Scalar(255, 255, 0), -1);
	//			corners.push_back(pt);
	//			vecRectKPs[j].push_back(pKF->mvKeys[i]);
	//			vecRectDescs[j].push_back(pKF->mDescriptors.row(i));
	//		}
	//		vecPairRectAndPoints[j].second = corners;
	//	}
	//}

	////if(bPrev){
	////	//갱신
	////	//옵티컬 플로우 데이터를 추가하기 위해
	////	//옵티컬 플로우에 키포인트를 더해서 디스크립터 계산한 후 디스크립터 매칭이 가능한지 확인.
	////	//해당 위치에 포인트가 이미 있는지 체크하기

	////	int nres = 0;
	////	std::vector<int> vecResRect(vecPairRectAndPoints.size(),0);
	////	std::vector<std::vector<cv::Point2f>> vecRectPoints(vecPairRectAndPoints.size());

	////	cv::Mat res = cv::Mat::zeros(image.rows * 2, image.cols, CV_8UC3);
	////	
	////	std::vector<std::pair<cv::Point2f, cv::Point2f>> vecMatches;

	////	std::vector<cv::KeyPoint> tempMatchingPrevKP, tempMatchingCurrKP;
	////	cv::Mat tempPrevDesc = cv::Mat::zeros(0, 32, CV_8UC1);

	////	std::vector<cv::KeyPoint> vecOptKPs;
	////	for (int i = 0; i < static_cast<int>(prevCorners.size()); ++i) {
	////		if (!features_found[i]) {
	////			continue;
	////		}
	////		auto pt = tempMatchingPoints[i];
	////		if (pt.x < 20 || pt.x >= gray.cols - 20 || pt.y < 20 || pt.y >= gray.rows - 20)
	////			continue;

	////		vecMatches.push_back(std::make_pair(prevCorners[i], pt));
	////		
	////		//new kp
	////		auto kp = pPrevBF->mvKeyDatas[i];
	////		tempMatchingPrevKP.push_back(kp);
	////		kp.pt = pt;

	////		tempMatchingCurrKP.push_back(kp);
	////		tempPrevDesc.push_back(pPrevBF->mDescriptors.row(i));
	////		//new kp

	////		for (int j = 0; j < vecPairRectAndPoints.size(); j++) {
	////			auto rect = vecPairRectAndPoints[j].first;
	////			if (rect.contains(pt)) {
	////				vecResRect[j]++;
	////				vecRectPoints[j].push_back(pt);
	////			}
	////		}
	////		nres++;
	////		
	////		if (pt.x < 0 || pt.y < 0 || pt.x >= gray.cols-1 || pt.y >= gray.rows-1)
	////			continue;
	////		//if (used.at<uchar>(pt) > 0)
	////		//{
	////		//	
	////		//	line(
	////		//		image,                         // Draw onto this image
	////		//		prevCorners[i],                 // Starting here
	////		//		tempMatchingPoints[i],                 // Ending here
	////		//		cv::Scalar(0, 255, 255),       // This color
	////		//		3,                           // This many pixels wide
	////		//		cv::LINE_AA                  // Draw line in this style
	////		//	);
	////		//}
	////		
	////		currCorners.push_back(pt);
	////	}

	////	//compute descriptor from optical flow
	////	cv::Mat tempCurrDesc;
	////	SLAM->mpFeatureTracker->computeDescriptor(gray, tempMatchingCurrKP, tempCurrDesc);

	////	//calculate descriptor distance
	////	vecMatches.clear();
	////	for (int i = 0; i < tempMatchingPrevKP.size(); i++) {
	////		cv::Mat d1 = tempPrevDesc.row(i);
	////		cv::Mat d2 = tempCurrDesc.row(i);
	////		const int dist = Utils::CalcBinaryDescriptor(d1,d2);

	////		if (dist < 100) {
	////			vecMatches.push_back(std::make_pair(tempMatchingPrevKP[i].pt, tempMatchingCurrKP[i].pt));

	////			//여기에 추가하고
	////			//렉트 확인하기
	////			auto kp = tempMatchingCurrKP[i];
	////			const cv::Mat d = tempCurrDesc.row(i);

	////			if (pNewBF->mUsed.at<uchar>(kp.pt))
	////				continue;
	////			{
	////				pNewBF->AddData(kp, d);
	////				/*pNewBF->mUsed.at<uchar>(kp.pt)++;
	////				pNewBF->mvKeys.push_back(kp);
	////				pNewBF->mDesc.push_back(d);*/

	////				//바운딩 박스에 데이터 추가
	////				for (auto pBB : vpBBs) {
	////					
	////					if (pBB->mRect.contains(kp.pt)) {
	////						pBB->AddData(kp, d);
	////					}
	////				}
	////			}

	////		}
	////	}
	////	
	////	pNewBF->Init();
	////	for (auto pBB : pNewBF->mvpBBs) {
	////		pBB->Init();
	////	}
	////	
	////	//오브젝트 맵 테스트
	////	

	////	//플로우 매칭 테스트
	////	auto vpPrevBBs = pPrevBF->mvpBBs;
	////	std::vector<int> vMatchedDistance(vpBBs.size(), INT_MAX);
	////	std::vector<int> vMatched1(vpPrevBBs.size(), -1);
	////	std::vector<int> vMatched2(vpBBs.size(), -1);
	////	std::vector<int> vAlreadyMatches(vpBBs.size(), false);

	////	vecMatches.clear();
	////	/*std::vector<std::pair<int, int>> vecPairMatches;
	////	int n = ObjectSLAM::ObjectMatcher::SearchFrameAndFrame(pPrevBF, pNewBF, vecPairMatches, 80, 80, 0.8);
	////	for (auto pair : vecPairMatches) {
	////		int i1 = pair.first;
	////		int i2 = pair.second;
	////		vecMatches.push_back(std::make_pair(pPrevBF->mvKeyDatas[i1].pt, pNewBF->mvKeyDatas[i2].pt));
	////	}*/

	////	for (auto pBBi : vpPrevBBs) {
	////		
	////		std::vector<std::pair<int, int>> bestMatches;
	////		ObjectSLAM::BoundingBox* pBestBox = nullptr;
	////		int bestdist = 0;
	////		int bestidx; -1;
	////		std::cout << "match start" << std::endl;
	////		for (int j = 0; j < vpBBs.size(); j++) {

	////			if (vAlreadyMatches[j])
	////				continue;
	////			auto pBBj = vpBBs[j];
	////			if (pBBi->mnLabel != pBBj->mnLabel)
	////				continue;
	////			std::vector<std::pair<int, int>> tempMatches;
	////			int n = ObjectSLAM::ObjectMatcher::SearchBoxAndBox(pBBi, pBBj, tempMatches, 30.0, 100, 0.8);
	////			
	////			if (n > bestdist) {
	////				bestdist = n;
	////				bestidx = j; 
	////				pBestBox = pBBj;
	////				bestMatches = tempMatches;
	////			}
	////		}
	////		if (bestMatches.size() > 10) {
	////			std::cout << "best match = " << bestMatches.size() << std::endl;
	////			vAlreadyMatches[bestidx] = true;
	////			//시각화 
	////			for (auto pair : bestMatches) {
	////				int i1 = pair.first;
	////				int i2 = pair.second;
	////				vecMatches.push_back(std::make_pair(pBBi->mvKeyDatas[i1].pt, pBestBox->mvKeyDatas[i2].pt));
	////			}
	////		}
	////		else {
	////			std::cout << "match fail" << std::endl;
	////		}
	////	}

	////	cv::Mat resImage;
	////	SLAM->VisualizeMatchingImage(resImage, prev, image, vecMatches, User->mapName, 2);

	////	//이전 포인트 벡터 갱신
	////	prevCorners.clear();
	////	prevCorners = currCorners;

	////	prev = image.clone();
	////	prevGray = gray.clone();



	////	for (int j = 0; j < vecPairRectAndPoints.size(); j++) {
	////		auto rect = vecPairRectAndPoints[j].first;
	////		auto corners = vecPairRectAndPoints[j].second;
	////		if(vecResRect[j] < 10)
	////			prevCorners.insert(prevCorners.end(), corners.begin(), corners.end());
	////	}
	////	if (User->mbSave) {
	////		std::stringstream ss;
	////		ss << "../res/images/" << id << "_yolo.png";
	////		cv::imwrite(ss.str(), resImage);
	////	}

	////}
	////if (!bPrev) {
	////	//init 초기화
	////	bPrev = true;
	////	prev = image.clone();
	////	prevGray = gray.clone();
	////	prevCorners.clear();
	////	for (auto pair : vecPairRectAndPoints) {
	////		auto corners = pair.second;
	////		prevCorners.insert(prevCorners.end(), corners.begin(), corners.end());
	////	}
	////	if (prevCorners.size() > 20)
	////		bPrev = true;
	////}
	///*auto vpBBs = pNewBF->mvpBBs.get();
	//for (auto pBB : vpBBs) {
	//	pNewBF->mvKeys.insert(pNewBF->mvKeys.end(), pBB->mvKPs.begin(), pBB->mvKPs.end());
	//	pNewBF->mDesc.push_back(pBB->desc);
	//}*/
	//
	//for (auto pBB : vpBBs) {
	//	for (int i = 0; i < pBB->mvKeyDatas.size(); i++) {
	//		auto kp = pBB->mvKeyDatas[i];
	//		if (pNewBF->mUsed.at<uchar>(kp.pt))
	//			continue;
	//		const cv::Mat d = pBB->mDescriptors.row(i);
	//		pNewBF->AddData(kp, d);
	//		/*pNewBF->mvKeys.push_back(kp);
	//		pNewBF->mDesc.push_back(d);*/
	//	}
	//}
	
	//pPrevBF = pNewBF;

	////AllBoxFrames.Update(pKF, pNewBF);
	////SLAM->VisualizeImage(User->mapName, image, 3);
	///*if (User->mbSave) {
	//	std::stringstream ss;
	//	ss << "../res/images/" << id << "_yolo.png";
	//	cv::imwrite(ss.str(), image);
	//} */
	//User->mnUsed--;
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

void MappingFromSeg(ObjectSLAM::BoxFrame* pNewBF, ObjectSLAM::BoxFrame* pPrevBF, EdgeSLAM::SLAM* SLAM, EdgeSLAM::User* User, ObjectSLAM::ObjectDevice* pObjDevice, const std::string& mapName) {

	//MatchingTest(pNewBF, SLAM, User, pObjDevice, mapName);

	auto pKF = pNewBF->mpRefKF;

	{
		auto vpBFs = ObjSystem->GetConnectedBoxFrames(pKF, 10);

		////현재 프레임에 없는 것 체크하는 코드
		////new bf
		//std::set<int> setLabels;
		//for (auto pair : pNewBF->mmpBBs) {
		//	setLabels.insert(pair.second->mpConfLabel->label);
		//}
		////label, local BF
		//std::map<int, ObjectSLAM::BoxFrame*> mmpLabelNFrame;
		//
		//for (auto pBF : vpBFs) {
		//	for (auto pair : pBF->mmpBBs) {
		//		auto pIns = pair.second;
		//		int label = pIns->mpConfLabel->label;
		//		float conf = pIns->mpConfLabel->maxConf;
		//		if (!pIns->mbIsthing || pIns->isTable())
		//			continue;

		//		if(!setLabels.count(label) && !mmpLabelNFrame.count(label)){
		//			mmpLabelNFrame[label] = pBF;
		//			std::cout << "missing label = " << pIns->mStrLabel <<" "<<label <<" "<<conf << std::endl;
		//		}
		//	}
		//}
		//std::cout << "asdf test = " << mmpLabelNFrame.size() << std::endl;
		////현재 프레임에 없는 것 체크하는 코드
		std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

		//BF에서 인스턴스와 레이블 별로 나누기.

		bool bSave = User->mbSave;

		bool bVis = true;

		for (auto pBF : vpBFs) {

			if (!pBF)
				continue;
			
			//std::vector<cv::Point2f> vecDensePrevPoints,vecDenseCurrPoints;
			//pBF->MatchingFrameWithDenseOF(pNewBF, vecDensePrevPoints, vecDenseCurrPoints, 4);
			{
				std::vector<std::pair<int, int>> vecMatches;
				std::vector<std::pair<cv::Point2f, cv::Point2f>> vecMatchPoints;
				
				pBF->MatchingWithFrame(pNewBF, vecMatches);
				
				for (auto pair : vecMatches) {
					int idx1 = pair.first;
					int idx2 = pair.second;

					auto pt1 = pBF->mvKeyDatas[idx1].pt;
					auto pt2 = pNewBF->mvKeyDatas[idx2].pt;
					vecMatchPoints.push_back(std::make_pair(pt2, pt1));
				}
				
				//맵포인트 연결
				auto pKF2 = pBF->mpRefKF;
				/*for (int i = 0; i < pKF->N; i++) {
					auto pMPi = pKF->mvpMapPoints.get(i);
					if (!pMPi || pMPi->isBad())
						continue;
					if (!pMPi->IsInKeyFrame(pKF2))
						continue;
					auto idx = pMPi->GetIndexInKeyFrame(pKF2);
					if (idx < 0)
						continue;
					vecMatches.push_back(std::make_pair(idx, i));

					cv::Point2f pt1 = pKF->mvKeys[i].pt;
					cv::Point2f pt2 = pKF2->mvKeys[idx].pt;
					vecMatchPoints.push_back(std::make_pair(pt1, pt2));
				}*/

				for (auto pair : vecMatches) {
					int idx1 = pair.first;
					int idx2 = pair.second;

					//매칭 된 컨피던스 연결
					auto pRefConf = pBF->mvpConfLabels[idx1];
					if (pRefConf)
						pNewBF->mvpConfLabels[idx2] = pRefConf;
				}

				//매칭 정보 등록
				/*{
					std::unique_lock<std::mutex> lock(ObjectSLAM::ObjectMapper::mMutexMatches);
					ObjectSLAM::ObjectMapper::mapKFsMatches[std::make_pair(pKF2->mnId, pKF->mnId)] = vecMatches;
				}*/

				//인스턴스 연결
				ObjectSLAM::InstanceLinker::computeSim(pBF, pNewBF, vecMatches);


				//std::cout << "matching test = " << vecMatchPoints.size() << std::endl;
				cv::Mat resImage;
				SLAM->VisualizeMatchingImage(resImage, pNewBF->img, pBF->img, vecMatchPoints, User->mapName, 2);
				/*
				std::stringstream ss;
				ss << "../res/matches/kf_" << pNewBF->mnId << "_" << pBF->mnId << ".png";
				cv::imwrite(ss.str(), resImage);*/
			}

			{
				//auto pKF2 = pBF->mpRefKF;
				//auto pair = std::make_pair(pKF->mnId, pKF2->mnId);
				//std::vector<std::pair<size_t, size_t>> vecMatches;
				//SLAM->mpLocalMapper->GetMatches(pKF->mnId, pKF2->mnId, vecMatches);
				{
					/*std::vector<std::pair<cv::Point2f, cv::Point2f>> vecPairMatches;
					for (auto pair : vecMatches) {
						int idx1 = pair.first;
						int idx2 = pair.second;

						cv::Point2f pt1 = pKF->mvKeys[idx1].pt;
						cv::Point2f pt2 = pKF2->mvKeys[idx2].pt;
						vecPairMatches.push_back(std::make_pair(pt1, pt2));
					}*/

					/*for (int i = 0; i < pKF->N; i++) {
						auto pMPi = pKF->mvpMapPoints.get(i);
						if (!pMPi || pMPi->isBad())
							continue;
						if (!pMPi->IsInKeyFrame(pKF2))
							continue;
						auto idx = pMPi->GetIndexInKeyFrame(pKF2);
						if (idx < 0)
							continue;

						vecMatches.push_back(std::make_pair(i, idx));

						cv::Point2f pt1 = pKF->mvKeys[i].pt;
						cv::Point2f pt2 = pKF2->mvKeys[idx].pt;
						vecPairMatches.push_back(std::make_pair(pt1, pt2));
					}*/

					//confidence propagation
					//std::vector<bool> vbMatches(pKF->N, false);
					//for (auto pair : vecMatches) {
					//	int idx1 = pair.first;
					//	int idx2 = pair.second;

					//	vbMatches[idx1] = true;

					//	//매칭 된 컨피던스 연결
					//	auto pRefConf = pBF->mvpConfLabels[idx2];
					//	if (pRefConf)
					//		pNewBF->mvpConfLabels[idx1] = pRefConf;
					//}
					 
					//cv::Mat resImage;
					//SLAM->VisualizeMatchingImage(resImage, pNewBF->img, pBF->img, vecPairMatches, User->mapName, 2);
					/*
					std::stringstream ss;
					ss << "../res/matches/kf_" << pNewBF->mnId << "_"<<pBF->mnId<<".png";
					cv::imwrite(ss.str(), resImage);*/
				}
				
			}

			break;

			if(false)
			{
				std::vector<int> idxs;
				//std::vector<std::pair<int, int>> vecPairPointIdxInBox;
				std::vector<std::pair<int, int>> vecPairMatchInstance;

				std::set<std::pair<int, int>> vecAssignments;
				std::map<int, int> mapNewLinkIDs;
				std::map < std::pair<int, int>, std::pair<int, int>> mapChangedInstance;
				//std::cout << "test=1" << std::endl;
				std::vector<std::pair<cv::Point2f, cv::Point2f>> vecPairVisualizedMatches;
				pBF->MatchingWithFrame(pNewBF, idxs, vecPairMatchInstance, vecPairVisualizedMatches);
				//std::cout << "test=2" << std::endl;
				//ObjectSLAM::InstanceLinker::computeFromOF(pPrevBF, pNewBF, vecPairMatchInstance, mapNewLinkIDs);
				ObjectSLAM::InstanceLinker::computeFromOF(pBF, pNewBF, vecPairMatchInstance, mapNewLinkIDs, mapChangedInstance);
				//std::cout << "test=3" << std::endl;
				//changed pid,cid에 속하는 애들의 pid, idx에 데이터를 지우고
				//새로운 인스턴스에 추가 해야 함.
				//인스턴스 연결
				pBF->UpdateInstances(pNewBF, mapChangedInstance);//mapNewLinkIDs
				//std::cout << "test=4" << std::endl;
				//조건에 따라서 새로운 인스턴스 생성 및 키포인트에 컨피던스 전파
				pBF->UpdateInstanceKeyPoints(vecPairMatchInstance, idxs, mapChangedInstance);
				//std::cout << "test=5" << std::endl;
				pNewBF->UpdateInstanceKeyPoints(vecPairVisualizedMatches, vecPairMatchInstance, mapChangedInstance);
				//std::cout << "test=6" << std::endl;

				if (bVis) {
					bVis = false;
					cv::Mat resImage;
					SLAM->VisualizeMatchingImage(resImage, pBF->img, pNewBF->img, vecPairVisualizedMatches, mapName, 2);

					cv::Mat image = pNewBF->img.clone();
					cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC3);
					cv::Mat labeled = cv::Mat::zeros(image.size(), CV_8UC1);
					
					for (int x = 0; x < image.cols; x++) {
						for (int y = 0; y < image.rows; y++) {
							auto pt = cv::Point(x, y);
							int sid = pNewBF->GetInstance(pt);
							
							int label = pNewBF->mmpBBs[sid]->mpConfLabel->label;
							mask.at<cv::Vec3b>(pt) = SemanticSLAM::SemanticProcessor::SemanticColors[label];
							labeled.at<uchar>(pt) = label;
						}
					}
					cv::addWeighted(image, 0.3, mask, 0.7, 0.0, image);
					for (int i = 0; i < vecPairVisualizedMatches.size(); i++) {
						
						auto prevpt = vecPairVisualizedMatches[i].first;
						int pid = pBF->GetInstance(prevpt);
						auto prevIns = pBF->mmpBBs[pid];

						auto pt = vecPairVisualizedMatches[i].second;
						int sid = pNewBF->GetInstance(pt);
						auto pIns = pNewBF->mmpBBs[sid];
						if (pIns->isFloor())
							continue;
						if (pIns->isTable() && !prevIns->isTable())
							cv::circle(image, pt, 3, cv::Scalar(0,0,0), 1);
						if(!pIns->isTable())
							cv::circle(image, pt, 3, cv::Scalar(255, 255, 255), 1);
					}
					SLAM->VisualizeImage(mapName, image, 1);
					std::stringstream ss;
					ss << "../res/images/" << pNewBF->mnId << "_ins.png";
					cv::imwrite(ss.str(), image);
				}

				//break;

				//if (mapChangedInstance.size() > 0) {//
				//	cv::Mat image = pBF->img.clone();
				//	cv::Mat image2 = pNewBF->img.clone();
				//	for(auto pair : mapChangedInstance){
				//		int pid = pair.first.first;
				//		int newpid = pair.second.first;
				//		int newcid = pair.second.second;
				//		for (int x   = 0; x < image.cols; x++) {
				//			for (int y = 0; y < image.rows; y++) {
				//				auto pt = cv::Point(x, y);
				//				int sid = pBF->seg.at<uchar>(pt);
				//				if (sid == newpid)
				//				{
				//					image.at<cv::Vec3b>(pt) = cv::Vec3b(255, 0, 0);
				//				}
				//			}
				//		}
				//		for (int x = 0; x < image.cols; x++) {
				//			for (int y = 0; y < image.rows; y++) {
				//				auto pt = cv::Point(x, y);
				//				int sid = pNewBF->seg.at<uchar>(pt);
				//				if (sid == newcid)
				//				{
				//					image2.at<cv::Vec3b>(pt) = cv::Vec3b(255, 0, 0);
				//				}
				//			}
				//		}
				//		for (int i = 0; i < pBF->mvKeyDatas.size(); i++) {
				//			auto pt = pBF->mvKeyDatas[i].pt;
				//			int sid = pBF->seg.at<uchar>(pt);
				//			if (sid == newpid)
				//			{
				//				cv::circle(image, pt, 3, cv::Scalar(0, 0, 255), -1);
				//			}
				//		}
				//		for (int i = 0; i < pNewBF->mvKeyDatas.size(); i++) {
				//			auto pt = pNewBF->mvKeyDatas[i].pt;
				//			int sid = pNewBF->seg.at<uchar>(pt);
				//			if (sid == newcid)
				//			{
				//				cv::circle(image2, pt, 3, cv::Scalar(0, 0, 255), -1);
				//			}
				//		}
				//		break;
				//	}
				//	SLAM->VisualizeImage(mapName, image, 2);
				//	SLAM->VisualizeImage(mapName, image2, 3);
				//	std::stringstream ss;
				//	ss << "../res/images/" << pBF->mnId << "_dense.png";
				//	cv::imwrite(ss.str(), image);
				//	break;
				//}
			}
			
			//if(false)
			//{
			//	std::vector<int> idxs;
			//	std::vector<std::pair<int, int>> avecPairMatchInstance;
			//	std::set<std::pair<int, int>> vecAssignments;
			//	std::map<int, int> mapNewLinkIDs;
			//	std::map < std::pair<int, int>, std::pair<int, int>> mapChangedInstance;
			//	if (pBF->seg.rows == 0 || pBF->seg.cols == 0)
			//	{
			//		std::cout << "?????????asdfasdfads" <<" "<<pBF->mnId<<" "<< pBF->mpRefKF->mnFrameId << std::endl;
			//		continue;
			//	}
			//	pNewBF->MatchingWithFrame(pBF, idxs, avecPairMatchInstance);
			//	ObjectSLAM::InstanceLinker::computeFromOF(pNewBF, pBF, avecPairMatchInstance, mapNewLinkIDs, mapChangedInstance);
			//	std::cout << "new bf = " << mapChangedInstance.size() << std::endl;
			//	//인스턴스 연결
			//	pNewBF->UpdateInstances(pBF, mapChangedInstance);//mapNewLinkIDs
			//	//조건에 따라서 새로운 인스턴스 생성 및 키포인트에 컨피던스 전파
			//	pNewBF->UpdateInstanceKeyPoints(avecPairMatchInstance, idxs, mapChangedInstance);
			//}

			/*if (bSave && mapChangedInstance.size() > 0) {
				cv::Mat image = pBF->img.clone();

				for (auto pair : pBF->mmpBBs) {
					auto sid = pair.first;
					auto pIns = pair.second;

					for (int i = 0; i < pIns->N; i++)
					{
						if (!pIns->mvbInlierKPs.get(i))
							continue;
						cv::circle(image, pIns->mvKeyDatas[i].pt, 3, SemanticSLAM::SemanticProcessor::SemanticColors[pIns->mnLabel], -1);
					}
				}

				std::stringstream ss;
				ss << "../res/images/" << pBF->mnId << "_ins.png";
				cv::imwrite(ss.str(), image);
			}*/
		}
		
		//비어있는 컨피던스 생성 및 전파
		for (int i = 0; i < pNewBF->N; i++) {
			auto pConf = pNewBF->mvpConfLabels[i];
			if (!pConf)
			{
				pConf = new EdgeSLAM::SemanticConfLabel();
				pNewBF->mvpConfLabels[i] = pConf;
			}
			auto pt = pKF->mvKeys[i].pt;
			int sid = pNewBF->GetInstance(pt);
			auto pIns = pNewBF->mmpBBs[sid];
			pConf->Update(pIns->mpConfLabel->label, pIns->mpConfLabel->maxConf, pIns->mbIsthing);
		}
		
		//save 용
		if (bSave && pNewBF) {

			cv::Mat image = pNewBF->img.clone();

			cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC3);
			cv::Mat labeled = cv::Mat::zeros(image.size(), CV_8UC1);

			for (int x = 0; x < image.cols; x++) {
				for (int y = 0; y < image.rows; y++) {
					auto pt = cv::Point(x, y);
					int sid = pNewBF->GetInstance(pt);
					int label = pNewBF->mmpBBs[sid]->mpConfLabel->label;
					mask.at<cv::Vec3b>(pt) = SemanticSLAM::SemanticProcessor::SemanticColors[label];
					labeled.at<uchar>(pt) = label;
				}
			}
			cv::addWeighted(image, 0.3, mask, 0.7, 0.0, image);

			for (int i = 0; i < pNewBF->N; i++) {
				auto pConf = pNewBF->mvpConfLabels[i];
				//auto sid = pNewBF->mvnInsIDs[i];
				cv::circle(image, pNewBF->mvKeyDatas[i].pt, 3, SemanticSLAM::SemanticProcessor::SemanticColors[pConf->label], -1);
			}

			/*for (auto pair : pNewBF->mmpBBs) {
				auto sid = pair.first;
				auto pIns = pair.second;

				for (int i = 0; i < pIns->N; i++)
				{
					cv::circle(image, pIns->mvKeyDatas[i].pt, 3, SemanticSLAM::SemanticProcessor::SemanticColors[sid], -1);
				}
			}*/
			
			//cv::Mat edgeImg;
			//cv::cvtCol r(pNewBF->edge, edgeImg, cv::COLOR_GRAY2BGR);
			////pNewBF->edge.convertTo(edgeImg, CV_8UC3);
			SLAM->VisualizeImage(mapName, image, 1);
			
			

			/*{
				std::stringstream ss;
				ss << "../res/images/" << pNewBF->mnId << "_label.png";
				cv::imwrite(ss.str(), labeled);
			}*/
		}

		std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		auto du = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
		std::cout << "matching test = " << vpBFs.size() << " " << du << std::endl;
	}
	/*{
		for (int i = 0; i < pKF->N; i++)
		{
			auto kp = pKF->mvKeys[i];
			auto pt = kp.pt;
			int sid = pNewBF->seg.at<uchar>(pt);

			auto pIns = pNewBF->mmpBBs[sid];
			if (pIns->mvKeyDatas.size() > 100)
				continue;
			pIns->AddData(pKF->mvKeys[i], pKF->mDescriptors.row(i));
		}
	}*/
	return;

}

void sam2(EdgeSLAM::SLAM* SLAM, std::string user, int id){
	
	auto User = SLAM->GetUser(user);
	if (!User)
		return;
	User->mnUsed++;
	if (!ObjSystem->MapObjectDevices.Count(User)) {
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
	ss << "/Download?keyword=sam2" << "&id=" << id << "&src=" << user;
	WebAPI API("143.248.6.143", 35005);
	auto res = API.Send(ss.str(), "");

	int n2 = res.size();

	cv::Mat temp = cv::Mat::zeros(n2, 1, CV_8UC1);
	std::memcpy(temp.data, res.data(), res.size());
	cv::Mat labeled = cv::imdecode(temp, cv::IMREAD_GRAYSCALE);
	pNewBF->mbSam2 = true;

	int w = labeled.cols;
	int h = labeled.rows;
	cv::Mat segcolor = cv::Mat::zeros(h, w, CV_8UC3);
	cv::Mat inscolor = cv::Mat::zeros(h, w, CV_8UC3);
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			int label = labeled.at<uchar>(y, x);
			segcolor.at<cv::Vec3b>(y, x) = SemanticSLAM::SemanticProcessor::SemanticColors[label];
		}
	}
	if (bVis) {
		//SLAM->VisualizeImage(mapName, img, 0);
		SLAM->VisualizeImage(mapName, segcolor, 1);
	}
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

void TrackObject(EdgeSLAM::User* User,int id, EdgeSLAM::Frame* frame, const cv::Mat& gray, std::promise<int>* refid) {

	//refid = -1;
	int rid = -1;
	if (ObjSystem->MapObjectDevices.Count(User)) {
		auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
		auto pKF = User->mpRefKF;
		if (pKF) {

			if (id == pKF->mnFrameId)
			{
				std::cout << "object matching test = " << id << " " << std::endl;
			}

			if (ObjSystem->MapKeyFrameNBoxFrame.Count(pKF->mnFrameId)) {
				 auto pBF = ObjSystem->MapKeyFrameNBoxFrame.Get(pKF->mnFrameId);
				ObjectSLAM::ObjectMatchingInfo* pMatches = new ObjectSLAM::ObjectMatchingInfo(id, pKF->mnFrameId);
				auto nres = ObjectSLAM::ObjectMatcher::SearchByOpticalFlow(frame, pKF, gray, pBF->gray, pMatches);
				ObjSystem->SetMatchInfo(id, pKF->mnFrameId, pMatches);
				rid = pKF->mnFrameId;
			}
		}
	}
	refid->set_value(rid);
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

	// 세그멘테이션 결과 확인하기
	if(false){
	//if (User->mpRefKF) {
		std::cout << "reference = " << User->mpRefKF->mnFrameId << " " << frame->mnFrameID << std::endl;
		std::chrono::high_resolution_clock::time_point t_seg_start = std::chrono::high_resolution_clock::now();
		long long ts = t_seg_start.time_since_epoch().count();
		int c = 0;
		//복원 후 매칭까지
		/*for (int i = 0; i < User->mpRefKF->N; i++) {
			int label = User->mpRefKF->mvLabel[i];
			if (label == (int)StructureLabel::FLOOR || label == (int)StructureLabel::CEIL)
				c++;

		}*/

		SLAM->mpLocalMapper->CreateNewMapPointsWithPlane(SLAM, User->mapName, User->GetMap(), User->mpRefKF, ts, 2.5);
		std::vector<std::pair<int, int>> vecRes;
		frame->reset_map_points();
		frame->SetPose(User->PredictPose());
		int a = EdgeSLAM::SearchPoints::SearchKeyByPlane(User->mpRefKF, frame, vecRes);

		SemanticSLAM::Plane* floor = nullptr;
		SemanticSLAM::Plane* ceil = nullptr;
		if (SemanticSLAM::PlaneEstimator::GlobalFloor){
			if (SemanticSLAM::PlaneEstimator::GlobalCeil) {
				floor = SemanticSLAM::PlaneEstimator::GlobalFloor;
				ceil  = SemanticSLAM::PlaneEstimator::GlobalCeil;
				//cv::Mat fcam = 
			}
		}
		int b = EdgeSLAM::Optimizer::PoseOptimization(frame);
		std::chrono::high_resolution_clock::time_point t_seg_end = std::chrono::high_resolution_clock::now();
		auto du_seg = std::chrono::duration_cast<std::chrono::milliseconds>(t_seg_end - t_seg_start).count();
		cv::Mat vis = img.clone();
		
		for(int i = 0; i < frame->N; i++){
			auto pMPi = frame->mvpMapPoints[i];
			if (!pMPi || frame->mvbOutliers[i])
				continue;
			auto pt = frame->mvKeys[i].pt;
			cv::circle(vis, pt, 2, cv::Scalar(255, 0, 0), 2);
		}
		
		SLAM->VisualizeImage(User->mapName, vis, 3);
		std::cout << "ref = " << a <<", "<<b << " :: " << du_seg << std::endl;
	}
	// 세그멘테이션 결과 확인하기

	
	std::promise<int> prefid;
	std::future<int> frefid = prefid.get_future();
	int refid;
	POOL->EnqueueJob(TrackObject, User, id, frame, gray, &prefid);

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

	///LINE TRACKING
	//SemanticSLAM::LineFrame* pLF = nullptr;
	//if (bTrackSuccess) {
	//	pLF = new SemanticSLAM::LineFrame(frame, User->mpCamera,frame->mpCamPose);
	//	SemanticSLAM::LineProcessor::LineDetectonAndCompute(pLF, img);
	//	if (!SemanticSLAM::LineProcessor::PrevLineFrames.Count(User)) {
	//		SemanticSLAM::LineProcessor::PrevLineFrames.Update(User, pLF);
	//	}
	//	else {
	//		auto pPrevLF = SemanticSLAM::LineProcessor::PrevLineFrames.Get(User);
	//		bool mbInitLineMap = SemanticSLAM::LineProcessor::mbInitMap;
	//		if (mbInitLineMap) {
	//			std::vector<std::pair<int, int>> matches;
	//			SemanticSLAM::LineProcessor::LineMatchingWithFrame(pPrevLF, pLF, matches);

	//			std::vector<SemanticSLAM::LineFrame*> vpLocalKFs;
	//			std::vector<SemanticSLAM::MapLine*> vpLocalMPs;
	//			SemanticSLAM::LineProcessor::UpdateLocalMap(pLF, vpLocalKFs, vpLocalMPs);
	//			int Nline = SemanticSLAM::LineProcessor::LineMatchingWithLocalMapLines(pLF, vpLocalMPs);
	//			//std::cout << "Line Matching = " << Nline << std::endl;
	//			cv::Mat lsd_outImg = img.clone();
	//			
	//			cv::Mat Tcw1 = frame->GetPose();
	//			cv::Mat Kline = pLF->Kline.clone();
	//			auto pPlucker = new SemanticSLAM::PluckerCoordinate();
	//			cv::Mat projMat1 = pPlucker->PlukerProjectionMatrix(Tcw1);
	//			auto vecMLs = pLF->mvpMapLines.get();

	//			//평면 속하는 라인 찾기
	//			SemanticSLAM::Plane* Floor = nullptr;
	//			std::vector<bool> vecPlanarLines(vecMLs.size(),false);
	//			if (SemanticSLAM::PlaneEstimator::GlobalFloor)
	//			{
	//				Floor = SemanticSLAM::PlaneEstimator::GlobalFloor;
	//				cv::Mat normal = Floor->normal.clone();
	//				if (normal.rows > 0)
	//				{
	//					//std::cout << normal << std::endl;
	//					for (int i = 0, iend = matches.size(); i < iend; i++) {
	//						int idx = matches[i].second;
	//						auto pML = vecMLs[idx];
	//						if (!pML || pML->isBad()) {
	//							continue;
	//						}
	//						auto d = pML->GetPlucker().rowRange(3, 6);
	//						bool b = pPlucker->CheckLineOnPlane(normal, d);
	//						vecPlanarLines[idx] = false;//b;
	//					}
	//				}
	//			}
	//			cv::Mat en_prev = User->ImageDatas.Get(pPrevLF->mnReferenceID);
	//			cv::Mat prevImg = cv::imdecode(en_prev, cv::IMREAD_COLOR);
	//			for (int i = 0, iend = matches.size(); i < iend; i++) {

	//				int idx1 = matches[i].first;
	//				auto keyline1 = pPrevLF->mvKeyLines[idx1];
	//				
	//				int idx2 = matches[i].second;
	//				auto keyline2 = pLF->mvKeyLines[idx2];
	//				
	//				auto pML = vecMLs[idx2];
	//				//if (!pML || pML->isBad())
	//				//	continue;

	//				auto s1 = keyline1.getStartPoint();
	//				auto e1 = keyline1.getEndPoint();
	//				auto s2 = keyline2.getStartPoint();
	//				auto e2 = keyline2.getEndPoint();

	//				cv::line(prevImg, s1, e1, cv::Scalar(0, 255, 0), 2);
	//				cv::line(prevImg, s1, s2, cv::Scalar(0, 255, 255), 1);
	//				cv::line(prevImg, e1, e2, cv::Scalar(0, 255, 255), 1);

	//				/*if (pML && !pML->isBad()) {
	//					cv::Mat line = Kline* projMat1* pML->GetPlucker();
	//					auto m1 = -line.at<float>(0) / line.at<float>(1);
	//					bool bSlopeOpt1 = abs(m1) > 1.0;
	//					float val1;
	//					if (bSlopeOpt1)
	//						val1 = 480.0;
	//					else
	//						val1 = 640.0;
	//					auto pt11 = pPlucker->CalcLinePoint(0.0, line, bSlopeOpt1);
	//					auto pt12 = pPlucker->CalcLinePoint(val1, line, bSlopeOpt1);
	//					cv::line(lsd_outImg, pt11, pt12, cv::Scalar(0, 0, 255), 2);
	//				}*/
	//				if(vecPlanarLines[idx2])
	//					cv::line(lsd_outImg, s2, e2, cv::Scalar(0, 255, 0), 2);
	//				else
	//					cv::line(lsd_outImg, s2, e2, cv::Scalar(255, 0, 0), 2);
	//			}
	//			for (int i = 0; i < pLF->mvKeyLines.size(); i++) {
	//				auto pML = pLF->mvpMapLines.get(i);
	//				if (!pML || pML->isBad() || pLF->mvbOutliers.get(i))
	//					continue;
	//				auto keyline2 = pLF->mvKeyLines[i];
	//				auto s2 = keyline2.getStartPoint();
	//				auto e2 = keyline2.getEndPoint();
	//				cv::line(lsd_outImg, s2, e2, cv::Scalar(255, 0, 0), 2);
	//			}
	//			{
	//				std::stringstream ss;
	//				ss << "../res/images/" << frame->mnFrameID << "_line1.png";
	//				cv::imwrite(ss.str(), prevImg);
	//				ss.str("");
	//				ss << "../res/images/" << frame->mnFrameID << "_line2.png";
	//				cv::imwrite(ss.str(), lsd_outImg);
	//			}
	//			SLAM->VisualizeImage(mapName, lsd_outImg, nVisID + 1);
	//			SLAM->VisualizeImage(mapName, prevImg, nVisID + 2);
	//			//delete pPrevLF;
	//			
	//		}
	//		else {
	//			
	//			if (bTrackSuccess && !SemanticSLAM::LineProcessor::mbLineInitDoing) {
	//				//initialization
	//				SemanticSLAM::LineProcessor::mbLineInitDoing = true;
	//				std::cout << "Line Initialization Start" << std::endl;
	//				cv::Mat invK = User->mpCamera->Kinv.clone();
	//				cv::Mat Kline = User->mpCamera->Kfluker.clone();
	//				pLF->mfMedianDepth = User->mpRefKF->ComputeSceneMedianDepth(2);
	//				if (SemanticSLAM::LineProcessor::Initalization(pLF) == EdgeSLAM::MapState::Initialized) {
	//					SemanticSLAM::LineProcessor::mbInitMap = true;
	//					std::cout << "Line Initialization Success" << std::endl;
	//				}
	//				else {
	//					std::cout << "Line Initialization Failed" << std::endl;
	//				}
	//				SemanticSLAM::LineProcessor::mbLineInitDoing = false;
	//			}
	//		}//if
	//		SemanticSLAM::LineProcessor::PrevLineFrames.Update(User, pLF);
	//	}
	//}
	///LINE TRACKING

	//인스턴스 전파 테스트

	frefid.wait();
	refid = frefid.get();
	//std::cout << "object tracking = " <<id<<" " << refid << std::endl;
	cv::Mat ptdata = cv::Mat::zeros(0, 1, CV_32FC1);
	{
		if (ObjSystem->MapObjectDevices.Count(User)) {

			
			//auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
			//auto pKF = User->mpRefKF;
			//if (pKF) {

			//	if (id == pKF->mnFrameId)
			//	{
			//		std::cout << "object matching test = " << id << " " << std::endl;
			//	}

			//	if (ObjSystem->MapKeyFrameNBoxFrame.Count(pKF->mnFrameId)) {
			//		auto pBF = ObjSystem->MapKeyFrameNBoxFrame.Get(pKF->mnFrameId);
			//		std::vector<uchar> vecFounds;
			//		std::vector<cv::Point2f> vecCorners;
			//		std::vector<std::pair<cv::Point2f, cv::Point2f>> vecPairMatches;
			//		//auto nres = EdgeSLAM::SearchPoints::SearchByOpticalFlow(frame, User->mpRefKF, gray, pBF->gray, vecFounds, vecCorners);

			//		ObjectSLAM::ObjectMatchingInfo* pMatches = new ObjectSLAM::ObjectMatchingInfo(id, pKF->mnFrameId);
			//		auto nres = ObjectSLAM::ObjectMatcher::SearchByOpticalFlow(frame, pKF, gray, pBF->gray, pMatches);
			//		ObjSystem->SetMatchInfo(id, pKF->mnFrameId, pMatches);
			//		
			//		
			//		//BF에 객체가 있는지 체크
			//		//객체 안에 포인트 매칭 체크
			//		
			//		std::map<int, std::vector<cv::Point2f>> mapTemp;

			//		//시각화
			//		for (int i = 0; i < pMatches->vecFounds.size(); i++)
			//		{
			//			if (!pMatches->vecFounds[i])
			//				continue;
			//			auto pt = pMatches->vecReferenceCorners[i];
			//			auto pt1 = pMatches->vecTargetCorners[i];
			//			int j = 0;
			//			for (auto pRect : pBF->mvpBBs) {
			//				j++;
			//				if(pRect->mRect.contains(pt)){
			//					vecPairMatches.push_back(std::make_pair(pMatches->vecTargetCorners[i], pMatches->vecReferenceCorners[i]));
			//					mapTemp[j].push_back(pt1);
			//					break;
			//				}
			//			}
			//			//vecPairMatches.push_back(std::make_pair(pMatches->vecTargetCorners[i], pMatches->vecReferenceCorners[i]));
			//		}
			//		
			//		//평균 위치 계산.
			//		
			//		for (auto pair : mapTemp) {
			//			auto vecPTs = pair.second;
			//			int tempN = vecPTs.size();
			//			cv::Point2f avgPt(0, 0);
			//			for (auto apt : vecPTs) {
			//				avgPt.x += apt.x;
			//				avgPt.y += apt.y;
			//			}
			//			avgPt.x /= tempN;
			//			avgPt.y /= tempN;

			//			cv::Mat temp = cv::Mat::zeros(2, 1, CV_32FC1);
			//			temp.at<float>(0) = avgPt.x;
			//			temp.at<float>(1) = avgPt.y;
			//			ptdata.push_back(temp);
			//		}

			//		

			//		cv::Mat resImage;
			//		SLAM->VisualizeMatchingImage(resImage, img, pBF->img, vecPairMatches, User->mapName, nVisID + 1);

			//		//auto du_upload =  Utils::SendData(keyword, src, totaldata, id, received_ts);

			//	}
			//}


			//auto pBF = pObjDevice->mpCurrBF;
			//if (pBF) { 
			//	if (!pBF->mbInitialized)
			//		std::cout << "frame init???" << std::endl;
			//	
			//	std::vector<int> vecIns;
			//	std::vector<cv::Point2f> vecCorners;
			//	std::vector<std::pair<cv::Point2f, cv::Point2f>> vecPairMatches;
			//	std::map<int, int> mapPair;
			//	
			//	pBF->MatchingWithFrame(frame, gray, vecIns, mapPair, vecCorners);
 
			//	//시각화
			//	cv::Mat objImg = visImg.clone();
			//	for (int i = 0, N = frame->N; i < N; i+=3) {
			//		auto pt = frame->mvKeys[i].pt;

			//		int sid = vecIns[i];
			//		if (sid < 0)
			//			continue;
			//		
			//		cv::circle(objImg, pt, 3, SemanticSLAM::SemanticProcessor::SemanticColors[mapPair[sid]], -1);
			//		vecPairMatches.push_back(std::make_pair(pt, vecCorners[i]));
			//	}
			//	SLAM->VisualizeImage(mapName, objImg, nVisID+1);
			//	
			//	///*{
			//	//	std::stringstream ss;
			//	//	ss << "../res/images/" << id << "_track.png";
			//	//	cv::imwrite(ss.str(), objImg);
			//	//}
			//	//{
			//	//	cv::Mat resImage;
			//	//	SLAM->VisualizeMatchingImage(resImage, img, pBF->img, vecPairMatches, User->mapName, -1);
			//	//	std::stringstream ss;
			//	//	ss << "../res/matches/track_" << id << "_"<<pBF->mnId<<".png";
			//	//	cv::imwrite(ss.str(), resImage);
			//	//}*/
			//}
		}
	}

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
	if (User->mbNewKF) {
		User->mbNewKF = false;

		std::string tsrc = src + ".Image";
		if (ObjSystem->MapObjectDevices.Count(User)) {
			auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);

			////sam2
			if (refid > 0 && ObjSystem->MapKeyFrameNBoxFrame.Count(refid)) {
				auto pBF = ObjSystem->MapKeyFrameNBoxFrame.Get(refid);
				if (pBF->mbInitialized)
				{
					auto pMatches = ObjSystem->GetMatchInfo(id, refid);
					if (pMatches) {
						//pBF의 mask에 매칭 얻기.

						std::map<int, std::vector<cv::Point2f>> mapTemp;

						//시각화
						for (int i = 0; i < pMatches->vecFounds.size(); i++)
						{
							if (!pMatches->vecFounds[i])
								continue;
							auto ptref = pMatches->vecReferenceCorners[i];
							auto pttarget = pMatches->vecTargetCorners[i];

							int sid = pBF->GetInstance(ptref);
							if (!pBF->mmpBBs.count(sid))
								continue;
							auto pIns = pBF->mmpBBs[sid];
							if (!pIns)
								continue;
							if (pIns->isTable() || pIns->isFloor()) {
								continue;
							}
							mapTemp[sid].push_back(pttarget);
						}

						//평균 위치 계산.

						for (auto pair : mapTemp) {
							auto vecPTs = pair.second;
							int tempN = vecPTs.size();
							cv::Point2f avgPt(0, 0);
							for (auto apt : vecPTs) {
								avgPt.x += apt.x;
								avgPt.y += apt.y;
							}
							avgPt.x /= tempN;
							avgPt.y /= tempN;

							cv::Mat temp = cv::Mat::zeros(2, 1, CV_32FC1);
							temp.at<float>(0) = avgPt.x;
							temp.at<float>(1) = avgPt.y;
							ptdata.push_back(temp);
						}

					}
				}
			}

			if (ptdata.rows > 0) {
				std::cout << id << " " << ptdata.rows << " " << std::endl;
				//reqest
				auto sam2key = "reqsam2";
				auto du_upload = Utils::SendData(sam2key, tsrc, ptdata, id, received_ts);
				
			}
			////sam2

			//Utils::SendReqMessage("reqdistdepth", tsrc, id);
			Utils::SendReqMessage("reqdetectron", tsrc, id);
			//Utils::SendReqMessage("reqsam2", tsrc, id);
			//Utils::SendReqMessage("reqcotrack", tsrc, id);
			
			//Utils::SendReqMessage("RequestSegmentation", tsrc, id);
			std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

			auto pNewBF = new ObjectSLAM::BoxFrame(id, img.cols, img.rows, User, (BaseSLAM::AbstractPose*)pRefKF->mpCamPose);
			pNewBF->t_start = start;
			pNewBF->Copy(frame);
			pNewBF->mpRefKF = pRefKF;
			pNewBF->img = img.clone();
			pNewBF->gray = gray.clone();

			/*if(refid > 0)
				ObjectSLAM::ObjectMapper::ObjectMatchingPropagation(ObjSystem, src, refid, pRefKF, pNewBF);*/

			//갱신 부분을 초기화루로 변경한다면?
			//CurrBF를 reference로 이용함.
			//pObjDevice->mpPrevBF = pObjDevice->mpCurrBF;
			//pObjDevice->mpCurrBF = pNewBF;

			ObjSystem->MapKeyFrameNBoxFrame.Update(id, pNewBF);
			Utils::SendReqMessage("RequestObjectDetection", tsrc, id);
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
		
		if(false){
		////if (pRefKF && planedata.at<float>(0) >= 8.0) {
		//	////라인, 평면, 포인트 최적화 중
		//	auto pLF = new SemanticSLAM::LineFrame(frame, User->mpCamera, frame->mpCamPose);
		//	SemanticSLAM::LineProcessor::LineDetectonAndCompute(pLF, img);

		//	int Nplane = (int)planedata.at<float>(2);
		//	int N = (int)planedata.at<float>(2);
		//	cv::Mat tempa = planedata.rowRange(3, planedata.rows);
		//	cv::Mat P = tempa.reshape(1, N).colRange(1,5);
		//	cv::Mat reconed, vis_recon, depthImg;
		//	
		//	cv::Mat pmask = img.clone();
		//	//std::cout << "StructTest::Strat" << std::endl;
		//	StructOptimization::SimpleRecon::SimpleReconstruction(vis_recon, reconed, depthImg, img, User->matXcam, frame->GetPoseInverse().rowRange(0,3), frame->GetCameraCenter(), P);
		//	
		//	StructOptimization::StructMap* tempMap = (StructOptimization::StructMap*)BaseSLAMSystem->GetMap(User->mapName);
		//	auto device = tempMap->GetDevice(src);
		//	//임시로 뎁스에 마스크 이미지 넣음.
		//	StructOptimization::SimpleFrame* pSF = new StructOptimization::SimpleFrame(id, device, reconed, vis_recon, P);
		//	//std::cout << "StructTest::Classify" << std::endl;
		//	StructOptimization::SimpleRecon::ClassipyPointAndLine(pSF, reconed, depthImg, frame, pLF);

		//	///이 밑부터는 매핑으로
		//	//std::cout << "StructTest::Matching" << std::endl;
		//	StructOptimization::SimpleRecon::ProjectPointsAndLines(pmask, pSF);

		//	//StructOptimization::SimpleRecon::ProjectPlane(reconed, pSF, P);
		//	//std::cout << "StructTest::Triangulation" << std::endl;
		//	//sStructOptimization::SimpleRecon::SimpleTriangulation(tempMap, pSF, depthImg, P);
		//	
		//	//시각화
		//	{
		//		auto vecAllPoints = tempMap->GetAllData<StructOptimization::StructPoint>();
		//		cv::Mat matPointData;
		//		for (int i = 0, iend = vecAllPoints.size(); i < iend; i++) {
		//			auto pMP = vecAllPoints[i];
		//			if (!pMP || pMP->isBad())
		//				continue;
		//			StructOptimization::FramePointObservation* pO = pMP;
		//			if (pO->Observations() < 2)
		//				continue;
		//			auto endPt = pMP->GetData();
		//			matPointData.push_back(endPt.t());
		//		}
		//		//std::cout << "ALL LINE = " << vecAllLines.size() <<" "<<matLineData.rows << std::endl;
		//		SLAM->RegistVisualData(User->mapName, "MapPoints", matPointData);
		//	}
		//	//auto vecAllLines = tempMap->GetAllData< StructOptimization::StructLine>();
		//	//cv::Mat matLineData;
		//	//for (int i = 0, iend = vecAllLines.size(); i < iend; i++) {
		//	//	auto pML = vecAllLines[i];
		//	//	if (!pML || pML->isBad())
		//	//		continue;
		//	//	StructOptimization::FrameLineObservation* pO = pML;
		//	//	if (pO->Observations() < 2)
		//	//		continue;
		//	//	auto endPt = pML->GetEndPoints();
		//	//	matLineData.push_back(endPt);
		//	//}
		//	////std::cout << "ALL LINE = " << vecAllLines.size() <<" "<<matLineData.rows << std::endl;
		//	//SLAM->RegistVisualData(User->mapName, "MapLines", matLineData);

		//	
		//	//StructOptimization::StructMapper::bLineOptimization = false;
		//	//StructOptimization::StructMapper::bPlaneOptimization = true;
		//	//POOL->EnqueueJob(StructOptimization::StructMapper::ProcessMapping, tempMap, pSF, reconed, depthImg, P);
		//	
		//	cv::Mat blended2;
		//	cv::Mat vis1 = img.clone();
		//	cv::addWeighted(vis1, 0.7, vis_recon, 0.3, 0.0, blended2);

		//	SLAM->VisualizeImage(mapName, blended2, nVisID + 3);

		//	{
		//		std::stringstream ss;
		//		ss << "../res/images/" << frame->mnFrameID << "_candidate_wall_line.png";
		//		cv::imwrite(ss.str(), blended2);
		//		//ss.str("");
		//		//ss << "../res/images/" << frame->mnFrameID << "_recon.png";
		//		//cv::imwrite(ss.str(), blended);
		//	}
		}

		if (!bMapping && bCommuTest && pRefKF && planedata.at<float>(0) >= 8.0) {
			//{
			//	//데이터 복원 테스트
			//	//2 : size
			//	//3부터 id와 파라메터
			//	int startidx = 3;
			//	int Nplane = (int)planedata.at<float>(2);
			//	int N = (int)planedata.at<float>(2);
			//	cv::Mat tempa = planedata.rowRange(3, planedata.rows);
			//	cv::Mat P = tempa.reshape(1, N).colRange(1,5);
			//	//std::cout << "asdfasdf a;sdlkfjasl;dfjadls;fkjasdf    " << P<< std::endl;
			//	cv::Mat Kinv2 = cv::Mat::eye(4, 4, CV_32FC1);
			//	Kinv.copyTo(Kinv2.colRange(0, 3).rowRange(0, 3));
			//	//std::cout << Kinv2 << " " << Tinv << std::endl;

			//	cv::Mat A = Tinv * Kinv2;
			//	int idx = 0;
			//	int inc = 1;
			//	cv::Mat Temp = cv::Mat::ones(4, 640 * 480, CV_32FC1);
			//	std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();
			//	for (int x = 0; x < 640; x += inc) {
			//		for (int y = 0; y < 480; y += inc) {
			//			cv::Mat X = (cv::Mat_<float>(4, 1) << x, y, 1, 1);
			//			X.copyTo(Temp.col(idx));
			//			idx++;
			//		}
			//	}
			//	//P = Np*4(normal + dist), P.col(3)(Np*1)
			//	//B = 4 X N 
			//	cv::Mat B = A * Temp;
			//	cv::Mat temp2 = cv::Mat::ones(1, 640 * 480, CV_32FC1);
			//	//Ow.push_back(cv::Mat::ones(1, 1, CV_32FC1));
			//	cv::Mat Ow2 = Ow * temp2;

			//	//3*N
			//	cv::Mat dirs = B.rowRange(0,3) - Ow2;

			//	//Np*N
			//	cv::Mat normals = -P.colRange(0, 3)* dirs;
			//	
			//	//P x O = Np*3 3 1 =
			//	cv::Mat dists = P.col(3);
			//	cv::Mat normals2 = P.colRange(0, 3) * Ow;
			//	normals2 += dists;
			//	//std::cout <<"test=" << P << std::endl << normals2 << std::endl;
			//	
			//	cv::Mat vis2 = cv::Mat::zeros(480, 640, CV_8UC3);
			//	idx = 0;

			//	std::map<int, cv::Mat> mapDatas;
			//	std::set<SemanticSLAM::Region*> setRegions;
			//	for (int x = 0; x < 640; x += inc) {
			//		for (int y = 0; y < 480; y += inc) {
			//			cv::Mat res = normals.col(idx);
			//			cv::Mat dir = dirs.col(idx);
			//			float min_depth = FLT_MAX;
			//			int min_id = -1;
			//			for (int i = 0; i < N; i++) {
			//				float depth = normals2.at<float>(i)/res.at<float>(i);
			//				if (depth > 0.0){
			//					if(depth < min_depth) {
			//						min_id = i;
			//						min_depth = depth;
			//					}
			//				}
			//			}
			//			if (min_depth < 1.0) {
			//				if (min_id == 0) {
			//					vis2.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
			//					//region test
			//					cv::Mat Xp = Ow + dir * min_depth;
			//					int xidx, yidx, zidx;
			//					SemanticSLAM::GridProcessor::ConvertIndex(Xp, xidx, yidx, zidx);
			//					auto pRegion = SemanticSLAM::GridProcessor::SearchRegion(xidx, zidx);
			//					if (pRegion && !setRegions.count(pRegion)) {
			//						setRegions.insert(pRegion);
			//					}
			//				}
			//				if (min_id == 1)
			//					vis2.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
			//				if (min_id == 2){
			//					vis2.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);
			//				}
			//			}
			//			idx++;
			//		}
			//	}

			//	//light
			//	auto lightPts = SemanticSLAM::SemanticProcessor::LightPTs.Get();
			//	for (auto iter = lightPts.begin(), iend = lightPts.end(); iter != iend; iter++) {
			//		cv::Mat X = iter->second;
			//		cv::Mat Xcam = K*(Rcw* X + tcw);
			//		float depth = Xcam.at<float>(2);
			//		if (depth < 0)
			//			continue;
			//		cv::Point2f pt(Xcam.at<float>(0) / depth, Xcam.at<float>(1) / depth);
			//		if (pt.x <= 0 || pt.x >= 640 || pt.y <= 0 || pt.y >= 480)
			//			continue;
			//		vis2.at<cv::Vec3b>(pt) = cv::Vec3b(0, 0, 0);
			//	}

			//	//region test
			//	cv::RNG rng(12345);
			//	for (auto iter` = setRegions.begin(), iend = setRegions.end(); iter != iend; iter++) {
			//		auto pRegion = *iter;
			//		//std::cout < < "region test = " << pRegion->ConnectedMPs.Size() << std::endl;
			//		auto color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			//		int nOpt = 0;
			//		auto vecMPs = pRegion->ConnectedMPs.ConvertVector();
			//		for (int i = 0, mend = vecMPs.size(); i < mend; i++) {
			//			auto pMPi = vecMPs[i];
			//			if (!pMPi || pMPi->isBad())
			//				continue;
			//			if (pMPi->Observations() < 3)
			//				nOpt++;
			//		}
			//	}
			//	//region test

			//	//blending
			//	cv::Mat vis1 = img.clone();
			//	cv::Mat blended;
			//	cv::addWeighted(vis1, 0.5, vis2, 0.5, 0.0, blended);
			//	SLAM->VisualizeImage(mapName, blended, nVisID+1);
			//	SLAM->VisualizeImage(mapName, vis1, nVisID + 2);

			//	{
			//		//std::stringstream ss;
			//		//ss << "../res/images/" << frame->mnFrameID << "_ori.png";
			//		//cv::imwrite(ss.str(), vis1);
			//		//ss.str("");
			//		//ss << "../res/images/" << frame->mnFrameID << "_recon.png";
			//		//cv::imwrite(ss.str(), blended);
			//	}
			//}

		}
	}

	if(false)
	{
		//좌표계 정합 테스트
		if (User->MapDeviceTrajectories.Count(id))
		{
			cv::Mat F = cv::Mat::eye(4, 4, CV_32FC1);
			F.at<float>(1, 1) = -1.0;
			float s = User->ScaleFactor;
			//std::cout << "scale = " << s << std::endl;
			cv::Mat Tcoord = User->Tcoord.clone();
			//Tcoord = Tcoord.inv();
			//Tcoord = F * Tcoord * F;
			cv::Mat tempImg = img.clone();
			
			cv::Mat Twc = User->MapDeviceTrajectories.Get(id).clone();
			Twc = F * Twc * F;
			cv::Mat Tcw = Twc.inv();
			cv::Mat Ts = frame->GetPose();

			if (User->MapAlignedDeviceTrajectories.Count(id)) {
				cv::Mat Tawc = User->MapDeviceTrajectories.Get(id).clone();
				cv::Mat Tacw = Tawc.inv();
				for (int i = 0, N = frame->mvpMapPoints.size(); i < N; i++) {
					auto pMPi = frame->mvpMapPoints[i];
					if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i])
						continue;
					cv::Mat X = pMPi->GetWorldPos();
					X.push_back(cv::Mat::ones(1, 1, CV_32FC1));
					cv::Mat Xcam = Ts * X;
					Xcam = Xcam.rowRange(0, 3);
					cv::Mat proj = K * Xcam;
					float depth = proj.at<float>(2);
					cv::Point2f pt2(proj.at<float>(0) / depth, proj.at<float>(1) / depth);
					cv::circle(tempImg, pt2, 3, cv::Scalar(255, 0, 0), -1);
					cv::line(tempImg, frame->mvKeys[i].pt, pt2, cv::Scalar(0, 255, 0));
					
				}
			}

			std::map<int, cv::Mat> mapDatas;
			if (SLAM->TemporalDatas2.Count("hcaaa")) {
				mapDatas = SLAM->TemporalDatas2.Get("hcaaa");
			}

			for (int i = 0, N = frame->mvpMapPoints.size(); i < N; i++) {
				auto pMPi = frame->mvpMapPoints[i];
				if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i])
					continue;
				if (mapDatas.count(pMPi->mnId)) {
					cv::Mat X = mapDatas[pMPi->mnId];
					cv::Mat Xcam = F*Tcw * X;
					Xcam = Xcam.rowRange(0, 3);
					cv::Mat proj = K * Xcam;
					float depth = proj.at<float>(2);
					cv::Point2f pt2(proj.at<float>(0) / depth, proj.at<float>(1) / depth);
					//cv::circle(tempImg, pt2, 3, cv::Scalar(255, 0, 0), -1);
					//cv::line(tempImg, frame->mvKeys[i].pt, pt2, cv::Scalar(0, 255, 0));
				}
			}

			for (int i = 0, N = frame->mvpMapPoints.size(); i < N; i++) {
				auto pMPi = frame->mvpMapPoints[i];
				if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i])
					continue;
				cv::Mat X = pMPi->GetWorldPos();
				//X.at<float>(1) = -X.at<float>(1);
				X.push_back(cv::Mat::ones(1, 1, CV_32FC1));
				
				cv::Mat Xnew = Twc*s*F*Ts*X;
				mapDatas[pMPi->mnId] = Xnew;

				////cv::Mat Xcoord = s*Tcoord * X;
				////cv::Mat Xcam = s*T * X;
				
			}

			SLAM->TemporalDatas2.Update("hcaaa", mapDatas);
			SLAM->VisualizeImage(mapName, tempImg, nVisID + 1);
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
		sendKeywords.push_back("reqsam2");				pairKeywords.push_back("sam2");
		sendKeywords.push_back("reqcotrack");				pairKeywords.push_back("cotrack");
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
		receivedKeywords.push_back("cotrack");
				
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
			else if (keyword == "sam2") {
				POOL->EnqueueJob(sam2, SLAM, src, id);
			}
			else if (keyword == "detectron") {
				/*if (!ObjSystem->MapObjectDevices.Count(User))
					return;
				auto pObjDevice = ObjSystem->MapObjectDevices.Get(User);
				User->mnUsed++;

				auto pNewBF = pObjDevice->mpCurrBF;
				auto pKF = pNewBF->mpRefKF;*/
				
				//POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::PanopticSegmentation, SLAM, src, id);
				POOL->EnqueueJob(detectron2, SLAM, src, id);
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
				POOL->EnqueueJob(ObjectDetection2, SLAM, src, id);
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

