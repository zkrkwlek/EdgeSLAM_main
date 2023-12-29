// UdpServer.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"

#pragma comment(lib, "ws2_32")
//#include <WinSock2.h>
#include <WS2tcpip.h>
#include <iostream>
#include <sstream>

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
#include <SemanticLabel.h>
#include <rapidjson\document.h>
#include <SLAM.h>
#include <Map.h>
#include <Frame.h>
#include <ObjectFrame.h>
#include <Tracker.h>
#include <User.h>
#include <SemanticProcessor.h>
#include <GridProcessor.h>
#include <DynamicTrackingProcessor.h>
#include <PlaneEstimator.h>
#include <ContentProcessor.h>
#include <MarkerProcessor.h>
#include <CoordinateIntegration.h>
#include <PlanarOptimizer.h>
#include <Node.h>
#include <Confidence.h>
#include <LabelInfo.h>
#include <ConcurrentMap.h>

#include <ThreadPool.h>
#include <WebAPI.h>
//#include <FeatureDetector.h>
//#include <FeatureTracker.h>
//#include <Frame.h>
//#include <ORBDetector.h>

#include <opencv2/aruco.hpp>

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
			ss << "/Store?keyword=" << keyword << "&id=" << ++nID << "&src=" << src;
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
			ss << "/Store?keyword="<< sKeyword1 <<"&id=" << ++nID << "&src="<<SOURCE;
			auto res = API.Send(ss.str(), data.data, sizeof(unsigned char)*data.rows);
			LatencyMapStart.Update(nID, start);
		}

		{
			std::stringstream ss;
			ss << "/Store?keyword=" << sKeyword2 << "&id=" << ++nID << "&src=" << SOURCE;
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
		ss << "/Store?keyword=WiseUITest&id=" << id << "&src=" << src << "&ts=" << std::fixed << std::setprecision(6) << ts << "&type2=" << src;
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
	EdgeSLAM::Tracker::TrackSimulation(POOL, SLAM, id, src, frame, img, frame_ts);
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
	cv::Mat temp = cv::Mat(n2, 1, CV_8UC1, (void*)res.data());
	cv::Mat img = cv::imdecode(temp, cv::IMREAD_COLOR);
	cv::Mat visImg = img.clone();

	cv::Mat K = User->GetCameraMatrix();
	EdgeSLAM::Frame* frame = new EdgeSLAM::Frame(img, User->mpCamera, id, frame_ts);
	frame->mnShared++;
	//auto frame = std::make_shared<EdgeSLAM::Frame>(img, User->mpCamera, id, received_ts);
	int nVisID = User->GetVisID() + 4;
	std::string mapName = User->mapName;
	//User->mnUsed--;

	std::chrono::high_resolution_clock::time_point t_track_start = std::chrono::high_resolution_clock::now();
	EdgeSLAM::Tracker::Track(POOL, SLAM, id, src, frame, img, frame_ts);
	std::chrono::high_resolution_clock::time_point t_track_end = std::chrono::high_resolution_clock::now();
	
	//User->mnUsed++;
	//다이나믹 맵포인트 관리
	auto pRefKF = User->mpRefKF;
	auto trackStat = User->GetState();
	bool bTrackSuccess = trackStat == EdgeSLAM::UserState::Success && frame->mvpMapPoints.size() > 0;
	bool bCommuTest = User->mbCommuTest;
	int nContentKFs = User->mnContentKFs;
	
	//SemanticSLAM::DynamicTrackingProcessor::ObjectTracking(POOL, SLAM, src, frame, img.clone(), id);
	/////////////////////로컬 그래프 정보 전송
	//트래킹 후에 데이터 전송
	cv::Mat Pcw = frame->GetPose();
	cv::Mat Rcw = Pcw.rowRange(0, 3).colRange(0, 3);
	cv::Mat tcw = Pcw.rowRange(0, 3).col(3);

	//전송 수정 테스트
	//int nQueueKFs = 8;
	//cv::Mat totaldata = cv::Mat::zeros(2, 1, CV_32FC1); //파싱 아이디, 전체크기는 자기자신 포함. 1 + Nkf + Nmp

	if (User->mbNewKF) {
		Utils::SendReqMessage("RequestObjectDetection", src, id);
		Utils::SendReqMessage("RequestSegmentation", src, id);
		User->mbNewKF = false;
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
				ss << N << "," << src << "," << id << "," << User->mnQuality << "," << du_download << "," << du_track << "," << du_upload <<","<<du_graph <<","<<du_preprocessing<<"," << du_total << "," << totaldata.rows * sizeof(float) << std::endl;
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
			auto pMPi = frame->mvpMapPoints[i];
			if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i])
				continue;
			auto pt = frame->mvKeys[i].pt;
			cv::circle(visImg, pt, 3, cv::Scalar(255, 0, 0), -1);
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
					auto corners = SemanticSLAM::GridProcessor::ProjectdGrid(xidx, yidx, zidx, 0.1, K, Rcw, tcw);
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

			if (b1 && b2)
				cv::line(visImg, pt1, pt2, cv::Scalar(255, 255, 0));
			if (b1 && b3)
				cv::line(visImg, pt1, pt3, cv::Scalar(255, 255, 0));
			if (b4 && b2)
				cv::line(visImg, pt4, pt2, cv::Scalar(255, 255, 0));
			if (b4 && b3)
				cv::line(visImg, pt4, pt3, cv::Scalar(255, 255, 0));
		}
	}
	//그리드 시각화

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

	cv::Mat K = User->GetCameraMatrix();
	EdgeSLAM::Frame* frame = new EdgeSLAM::Frame(img, User->mpCamera, id, ts);
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
	EdgeSLAM::Tracker::Track(POOL, SLAM, id, src, frame, img, ts);
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
		auto pMPi = frame->mvpMapPoints[i];
		if (!pMPi || pMPi->isBad() || frame->mvbOutliers[i])
			continue;
		auto pt = frame->mvKeys[i].pt;
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
					auto corners = SemanticSLAM::GridProcessor::ProjectdGrid(xidx, yidx, zidx, 0.1, K, Rcw, tcw);
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
		trajectoryPath = argv[index++];
		std::stringstream ss_file;
		ss_file << trajectoryPath << "_" << strTimeStamp << ".txt";
		trajectoryPath = ss_file.str();
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
	SemanticSLAM::PlaneEstimator::Init();
	SemanticSLAM::ContentProcessor::mbSaveLatency = bSaveVOLatency;
	
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
		//sendKeywords.push_back("RequestDepth");				pairKeywords.push_back("Depth");
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
		receivedKeywords.push_back("Gyro");
		receivedKeywords.push_back("Accelerometer");
		receivedKeywords.push_back("DeviceConnect");
		receivedKeywords.push_back("DeviceDisconnect");
		receivedKeywords.push_back("ObjectDetection");
		receivedKeywords.push_back("Segmentation");
		receivedKeywords.push_back("Depth");
				
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
				ss << "/Load?keyword=" << keyword << "&id=" << id << "&id2=" << id2 << "&src=" << src;
			}
			else
				ss << "/Load?keyword=" << keyword << "&id=" << id << "&src=" << src;

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
				//POOL->EnqueueJob(DeviceCoordAlignTest, POOL, SLAM, src, ss.str(), id, ts);
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
						SemanticSLAM::SemanticProcessor::MapUpdateWithPlane(SLAM, pMap);
						//SemanticSLAM::SemanticProcessor::ObjectMapUpdateWithPlane(SLAM, pMap);
					}
					if (SLAM->GetConnectedDevice() == 0) {
						//std::cout << "save start = " << src << std::endl;
						if (bSaveLatency){
							std::cout << "save start" << std::endl;
							SLAM->Save(latencyPath, SLAM->EvaluationLatency, "num,source,id,quality,download,processing,upload,graph,preprocessing,total,size");
							std::cout << "save end" << std::endl;
						}
						if (bSaveVOLatency)
							SLAM->Save(volatencyPath, SLAM->EvaluationVirtualObjectLatency, "num,source,method,type,id,size,latency");
						if (bSaveTrajectory) {
							SLAM->SaveTrajectory(trajectoryPath, tempMapName);
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
				std::memcpy(f.data, res.c_str(), nInt * sizeof(float));
				int nSubStr = nInt * 4 + nByte; //문자열의 길이를 제거한 것. 11*4+3(bool)
				auto vstr = split(res.substr(nSubStr, res.size() - nSubStr), ',');
				std::string user = vstr[0];
				std::string mapName = vstr[1];
				int nidx = 0;
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
				int nContentKFs = (int)f.at<float>(nidx++);
				int nbFlagIdx = nInt * 4;
				
				bool bMapping = (int)res[nbFlagIdx] ? true : false;
				bool bTracking = (int)res[nbFlagIdx+1] ? true : false;
				bool bIMU = (int)res[nbFlagIdx+2] ? true : false;
				bool bResetVO = (int)res[nbFlagIdx+3] ? true : false;
				bool bPlaneGBA = (int)res[nbFlagIdx+4] ? true : false;
				bool bEdgeBase = (int)res[nbFlagIdx + 5] ? true : false;
				bool bKFMethod = (int)res[nbFlagIdx + 6] ? true : false;
				bool bCommuTest = (int)res[nbFlagIdx + 7] ? true : false;	//그리드와 키프레임
				bool bContentSyncTest = (int)res[nbFlagIdx + 8] ? true : false; //다이렉트와 인다이렉트
				std::vector<bool> vbFlags(10, false);
				vbFlags[0] = bMapping;
				vbFlags[1] = bTracking;
				vbFlags[2] = bIMU;
				vbFlags[3] = bResetVO;
				vbFlags[4] = bPlaneGBA;
				vbFlags[5] = bEdgeBase;
				vbFlags[6] = bCommuTest;
				vbFlags[7] = bContentSyncTest;
				
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
					std::cout << "Create Map" << std::endl;
					SLAM->CreateMap(mapName, quality);
					SLAM->InitVisualizer(user, mapName, w, h);
				}
				if (!SLAM->CheckUser(user)) {
					std::cout << "Create User" << std::endl;
					SLAM->CreateUser(user, mapName, w, h, fx, fy, cx, cy, d1, d2, d3, d4, d5, quality, nskip, nContentKFs, vbFlags);
					if(keyword =="DeviceConnect")
						SLAM->SetUserVisID(SLAM->GetUser(user));
				}

				//트래픽 저장용
				{
					std::stringstream ssfile1;
					ssfile1 << "../bin/normal/base.txt";
					std::ofstream f1;
					f1.open(ssfile1.str().c_str());
					f1.close();
				}
				{
					std::stringstream ssfile1;
					ssfile1 << "../bin/normal/proposed_"<<quality<<".txt";
					std::ofstream f1;
					f1.open(ssfile1.str().c_str());
					f1.close();
				}
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
				
				POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::Segmentation, SLAM, src, id);
				//POOL->EnqueueJob(SemanticSLAM::PlaneEstimator::PlaneEstimation, SLAM, src, id);

				////세그멘테이션 레이턴시 계산
				/*if (SLAM->RequestTime.Count(id)) {
					std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
					auto start = SLAM->RequestTime.Get(id);
					auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
					float t_test1 = du_test1 / 1000.0;
					std::cout << "seg time latency = " << t_test1 << std::endl;
					segDatas[id] = t_test1;
				}
				int count = segDatas.size();
				float sum = 0.0;
				for (auto iter = segDatas.begin(), iend = segDatas.end(); iter != iend; iter++) {
					iter->second;
					sum += iter->second;
				}
				std::cout << "AVG SEG TIME = " << sum / count << "  || " << count << std::endl;*/
				////세그멘테이션 레이턴시 계산

				//POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::DenseOpticalFlow, SLAM, src, id);
			}
			else if (keyword == "RPlaneEstimation") {
				//POOL->EnqueueJob(SemanticSLAM::PlaneEstimator::PlaneEstimation, SLAM, src, id);
			}
			else if (keyword == "ObjectDetection") {
			
				POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::ObjectDetection, SLAM, src, id);
				
				/*if (SLAM->RequestTime.Count(id)) {
					std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
					auto start = SLAM->RequestTime.Get(id);
					auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
					float t_test1 = du_test1 / 1000.0;
					std::cout << "obj time latency = " << t_test1 << std::endl;
					objDatas[id] = t_test1;
				}
				else {
					objDatas[id] = 0.0;
				}
				int count = objDatas.size();
				float sum = 0.0;
				for (auto iter = objDatas.begin(), iend = objDatas.end(); iter != iend; iter++) {
					iter->second;
					sum += iter->second;
				}
				std::cout << "AVG OBJ TIME = " << sum / count << "  || " << count << std::endl;*/

			}
			else if (keyword == "Recon") {
				POOL->EnqueueJob(SemanticSLAM::SemanticProcessor::SimpleRecon, SLAM, src, id);
				/*auto res = POOL->EnqueueJob(SemanticSLAM::ContentProcessor::testMethod);
				auto var = res.get();
				auto temp = std::get<0>(var);*/
			}
			else if (keyword == "Depth") {
				SLAM->ProcessDepthEstimation(src, id);
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

