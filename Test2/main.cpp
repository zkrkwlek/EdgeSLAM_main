#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <random>
#include <iostream>
#include <fstream>
#include <chrono>
#include <WebApi.h>
#include "LoadDataset.h"
#include "ThreadPool.h"
#include "rapidjson/document.h"

#include <thread>
#include <Utils.h>
#include <ConcurrentMap.h>
#include <ConcurrentVector.h>
#include <ConcurrentDeque.h>

#pragma comment(lib, "ws2_32")
#include <WS2tcpip.h>

//argv
std::string src;
std::string datakeyword;
std::string rgbfile = "rgb.txt";
bool bMapping = false;
bool bTracking = false;
bool bSyncLocalMap = false;
bool bGenVO = false;
bool bVOTest = false;
bool bSaveLatency = false;
std::string latencyPath;
bool bSaveTrajectory = false;
std::string trajectoryPath;
bool bCaptureDepth = false;
bool bGridCommu = false;
bool bPlaneGBA = false;
bool bPlay = false; //종료 없이 재생
bool mbDelay = true;
int nVODataSize = 17;

//다른 파라메터
std::vector<int> depthParam(2);

void parsing(char* argv[], int& index) {
	std::string ts = Utils::GetTimeStamp("_").str();
	std::string keyword = (argv[index++]);
	if (keyword == "--latency") {
		bSaveLatency = true;
		latencyPath = argv[index++];
		std::stringstream ss_file;
		ss_file << latencyPath << "_" << ts << ".csv";
		latencyPath = ss_file.str();
		std::cout << "latency path = " << latencyPath << std::endl;
	}
	else if (keyword == "--mapping") {
		bMapping = true;
		std::cout << "mapping ";
	}
	else if (keyword == "--tracking") {
		bTracking = true;
		std::cout << "tracking ";
	}
	else if (keyword == "--localmap") {
		bSyncLocalMap = true;
		std::cout << "sync_localmap ";
	}
	else if (keyword == "--gen_vo") {
		bGenVO = true;
	}
	else if (keyword == "--test_direct_vo") {
		bVOTest = true;
		nVODataSize = 2500;
		std::cout << "test_direct_vo ";
	}
	else if (keyword == "--trajectory") {
		bSaveTrajectory = true;
		trajectoryPath = argv[index++];
		std::stringstream ss_file;
		ss_file << trajectoryPath << "_" << ts << ".txt";
		trajectoryPath = ss_file.str();
		std::cout << "trajectory path = " << trajectoryPath << std::endl;
	}
	else if (keyword == "--no_delay") {
		mbDelay = false;
	}
	else if (keyword == "--depth") {
		bCaptureDepth = true;
	}
	else if (keyword == "--grid") {
		bGridCommu = true;
		//std::cout << "grid test = " << std::endl;
	}
	else if (keyword == "--pgba") {
		bPlaneGBA = true;
	}
	else if (keyword == "--vo") {
		bPlay = true;
	}
	else if (keyword == "--rgb_file")
	{
		rgbfile = argv[index++];
	}
}

void parser(int argc, char* argv[], int index) {
	std::cout << "start parsing" << std::endl;
	for (; index < argc;) {
		parsing(argv, index);
	}
}



int main(int argc, char* argv[]) {
	
	std::string ip;
	int port;
	ip = argv[1];
	port = atoi(argv[2]);
	std::string paramPath = argv[3];
	std::string dir = argv[4];//"E:/SLAM_DATASET/TUM/rgbd_dataset_freiburg2_desk_with_person/";

	//통신 관련
	WSAData wsaData;
	int code = WSAStartup(MAKEWORD(2, 2), &wsaData);
	int sock = socket(AF_INET, SOCK_DGRAM, 17);
	sockaddr_in serveraddr;

	WebAPI::ip = ip;
	WebAPI::port = port;
	WebAPI API(ip, port);

	int nTimeoutValue = INT_MAX;
	int aaa = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&nTimeoutValue, sizeof(nTimeoutValue));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(35001); // 포트 번호
	//serveraddr.sin_addr.s_addr = inet_addr("143.248.6.143");
	std::string addr = ip;
	inet_pton(AF_INET, addr.c_str(), &serveraddr.sin_addr.s_addr);
	
	int idxArgv = 5;

	//Image, SimImage
	datakeyword = argv[idxArgv++];
	//사용자 이름
	src = argv[idxArgv++];
	//맵 이름
	std::string mapname = argv[idxArgv++];
	//압축 퀄리티
	int quality = atoi(argv[idxArgv++]);
	//스킵 프레임
	int nskip = atoi(argv[idxArgv++]);

	//여기서부터 파싱 시작
	parser(argc, argv, idxArgv);

	std::vector<int> tempParam(2);
	tempParam[0] = cv::IMWRITE_JPEG_QUALITY;
	tempParam[1] = quality;//default(95) 0-100

	depthParam[0] = cv::IMWRITE_PNG_COMPRESSION;
	depthParam[1] = 1;

	//서버와 연결
	//키워드 전송
	std::vector<std::string> sendKeywords;
	std::vector<std::string> pairKeywords;
	std::vector<std::string> receivedKeywords;
	sendKeywords.push_back("DeviceConnect");			pairKeywords.push_back("NONE");
	sendKeywords.push_back("DeviceDisconnect");		pairKeywords.push_back("NONE");
	sendKeywords.push_back("TUM");			pairKeywords.push_back("NONE");
	sendKeywords.push_back("tum_color");				pairKeywords.push_back("NONE");
	sendKeywords.push_back("tum_depth");				pairKeywords.push_back("NONE");
	sendKeywords.push_back("DImage");				pairKeywords.push_back("NONE");
	sendKeywords.push_back("DDepth");				pairKeywords.push_back("NONE");
	sendKeywords.push_back(datakeyword);				pairKeywords.push_back("NONE");
	
	for (int i = 0, iend = sendKeywords.size(); i < iend; i++) {
		std::stringstream ss;
		ss << "{\"src\":\"" << src << "\"," << "\"keyword\":\"" << sendKeywords[i] << "\",\"type1\":\"server\",\"type2\":\"" << pairKeywords[i] << "\"" << ",\"capacity\":" << 300 << "}"; //test\"}";
		auto res = API.Send("/Connect", ss.str());
	}

	for (int i = 0, iend = receivedKeywords.size(); i < iend; i++) {
		std::stringstream ss;
		ss << "{\"type1\":" << "\"connect\"" << ",\"type2\":" << "\"single\"" << ",\"src\":\"" << src << "\",\"keyword\":\"" << receivedKeywords[i] << "\"" << "}";
		std::string temp = ss.str();
		auto buf = temp.c_str();
		int retval;
		retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
	}

	//서버에 올려놓은 데이터셋
	std::string keydataset = "TUM";
	std::string scene_id = "2_desk_with_person";
	std::string src_cam = keydataset + "2.intrinsic";
	std::string srcc = keydataset + (scene_id)+".color";
	std::string srcd = keydataset + (scene_id)+".depth";
	std::string srct = keydataset + (scene_id)+".ts";
	{
		std::stringstream ss;
		ss << "/Load?keyword=" << keydataset << "&src=" << srcc;
		auto res = API.Send(ss.str(), "");
	}
	{
		std::stringstream ss;
		ss << "/Load?keyword=" << keydataset << "&src=" << srcd;
		auto res = API.Send(ss.str(), "");
	}
	{
		std::stringstream ss;
		ss << "/Load?keyword=" << keydataset << "&src=" << srct;
		auto res = API.Send(ss.str(), "");
	}
	{
		std::stringstream ss;
		ss << "/Load?keyword=" << keydataset << "&src=" << src_cam;
		auto res = API.Send(ss.str(), "");
	}

	cv::Mat cam_data;
	{
		std::stringstream ss;
		ss << "/Download?keyword="<<keydataset << "&id=0"<< "&src=" << src_cam;
		auto res = API.Send(ss.str(), "");
		cam_data = cv::Mat(9, 1, CV_32FC1, (void*)res.data());
	}
	
	cv::Mat ids;
	{
		std::stringstream ss;
		ss << "/Get?keyword=" << keydataset<< "&src=" << srcc;
		auto res = API.Send(ss.str(), "");
		ids = cv::Mat(res.size()/4, 1, CV_32SC1, (void*)res.data());
	}
	
	{
		//device connect
		int fid = 0;
		int nInt = 20;
		int nByte = 10;

		cv::Mat temp1 = cv::Mat::zeros(nInt * 4 + nByte, 1, CV_8UC1);
		//11,12,13
		//14,15,16 뎁스 관련. mbf, mthdepth, mdepthmapfactor
		cv::Mat temp2 = (cv::Mat_<float>(17, 1) << 640, 480, 0.0,0.0,0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, quality, nskip, 10, 40.0,40.0,5208.0);
		cam_data.copyTo(temp2.rowRange(2, 11));
		std::memcpy(temp1.data, temp2.data, nInt*4);

		int nbFlagIdx = nInt * 4;
		//매핑
		temp1.at<uchar>(nbFlagIdx) = bMapping ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 1) = bTracking ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 4) = bPlaneGBA ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 5) = bSyncLocalMap ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 7) = bGridCommu ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 8) = bVOTest ? 1 : 0;
		std::string strtemp = src + "," + mapname;
		cv::Mat temp3(strtemp.length(), 1, CV_8UC1, (void*)strtemp.c_str());
		temp1.push_back(temp3);
		std::stringstream ss;
		ss << "/Upload?keyword=DeviceConnect&id=" << fid << "&src=" << src;// << "&type2=" << user->userName;
		auto res = API.Send(ss.str(), temp1.data, temp1.rows);
	}
	
	for (int i = 0; i < ids.rows; i++) {
		int id = i + 1;
		//std::cout << id << std::endl;

		if (id % nskip == 0)
			continue;

		std::stringstream ss;
		ss << "/Download?keyword="<<keydataset<<""<<"&id="<<id<<"&src="<<srcc;// << " & type2 = " << user->userName;
		auto res = API.Send(ss.str(), "");
		cv::Mat cdata = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());
		ss.str("");
		ss << "/Download?keyword=" << keydataset << "" << "&id=" << id << "&src=" << srcd;
		auto dres = API.Send(ss.str(), "");
		cv::Mat ddata = cv::Mat(dres.size(), 1, CV_8UC1, (void*)dres.data());

		{
			ss.str("");
			ss << "/Upload?keyword=DImage&id=" << id << "&src=" << src;// << "&type2=" << user->userName;
			API.Send(ss.str(), cdata.data, cdata.rows);
		}
		{
			ss.str("");
			ss << "/Upload?keyword=DDepth&id=" << id << "&src=" << src;// << "&type2=" << user->userName;
			API.Send(ss.str(), ddata.data, ddata.rows);
		}

	}
	
	{
		std::stringstream ss;
		cv::Mat temp = cv::Mat::zeros(1000, 1, CV_32FC1);
		ss << "/Upload?keyword=DeviceDisconnect&id=" << ids.size() << "&src=" << src;// << "&type2=" << user->userName;
		auto res = API.Send(ss.str(), temp.data, temp.rows);
	}

}
