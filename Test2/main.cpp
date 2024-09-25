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
bool bPLP = false;
bool bPlay = false; //종료 없이 재생
bool mbDelay = true;
int nVODataSize = 17;
int nCamType = 0;// 0 : mono, 1 : stereo, 2 : rgbd
int nIMUType = 0;// 0 : NONE, 1 : gyro, 2 : gyro + acc
int nFeature = 1000;
bool bSave = false;
std::string keydataset = "";
std::string scene_id = "";
std::string src_cam = "";

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
	else if (keyword == "--plp") {
		bPLP = true;
	}
	else if (keyword == "--dataset")
	{
		keydataset= argv[index++];
		scene_id= argv[index++];
		src_cam = argv[index++];
	}
	else if (keyword == "--features") 
	{
		nFeature = atoi(argv[index++]);
	}
	else if (keyword == "--CAM") {
		nCamType = atoi(argv[index++]);
	}
	else if (keyword == "--IMU") {
		nIMUType = atoi(argv[index++]);
	}
	else if (keyword == "--SAVE") {
		bSave = true;
	}
}

void parser(int argc, char* argv[], int index) {
	std::cout << "start parsing" << std::endl;
	for (; index < argc;) {
		parsing(argv, index);
	}
}

void ReceiveData(SOCKET sock) {

	sockaddr_in clientaddr;
	int addrlen;
	char rbuf[1024];

	WebAPI API("asdf", 123);
	
	while (true) {
		addrlen = sizeof(clientaddr);
		
		int retval = recvfrom(sock, rbuf, 1024, 0, (sockaddr*)&clientaddr, &addrlen);
		if (retval < 0)
			continue;
		rbuf[retval] = '\0';
		
		rapidjson::Document document;
		if (document.Parse(rbuf).HasParseError()) {
			std::cout << "JSON parsing error" << std::endl;
			continue;
		}

		if (document["keyword"].IsString() && document["id"].IsInt()) {
			
			int id = document["id"].GetInt();
			double ts = std::stod(document["ts"].GetString());
			std::string keyword = document["keyword"].GetString();
			std::string src = document["src"].GetString();
			std::string type2 = document["type2"].GetString();

			int id2 = -1;
			std::stringstream ss;
			if (document.HasMember("id2")) {
				id2 = document["id2"].GetInt();
				ss << "/Download?keyword=" << keyword << "&id=" << id << "&id2=" << id2 << "&src=" << src;
			}
			else
				ss << "/Download?keyword=" << keyword << "&id=" << id << "&src=" << src;

			std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
			auto res = API.Send(ss.str(), "");
			std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();
			auto du_down = std::chrono::duration_cast<std::chrono::milliseconds>(t_down_end - t_down_start).count();
			auto du_latency = (t_down_end.time_since_epoch().count() - (long long)ts) / 1000000.0;
			std::cout << "device = " << keyword << " " << du_down << " " << du_latency << std::endl;
		}
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
	WebAPI::Connect();
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

	//LoadDataset* dataset = (LoadDataset*)new TumDataset(dir, paramPath, rgbfile);

	std::vector<int> tempParam(2);
	tempParam[0] = cv::IMWRITE_JPEG_QUALITY;
	tempParam[1] = quality;//default(95) 0-100

	depthParam[0] = cv::IMWRITE_PNG_COMPRESSION;
	depthParam[1] = 1;

	std::stringstream ssdata;
	ssdata << keydataset << scene_id << "_" << quality<<".";
	std::string srcc;
	std::string srcd;
	std::string itype1 = "";
	std::string itype2 = "";
	double dCamDelay = 1e3;
	if (keydataset == "TUM") {
		srcc = ssdata.str() + "color";
		srcd = ssdata.str() + "depth";
		itype1 = "DImage";
		itype2 = "DDepth";
	}
	if (keydataset == "KITTI" || keydataset =="EuroC") {
		srcc = ssdata.str() + "left";
		srcd = ssdata.str() + "right";
		itype1 = "LImage";
		itype2 = "RImage";
		//dCamDelay = 1e6;
	}
	if (nCamType == 0)
	{
		itype1 = "Image";
	}

	//서버와 연결
	//키워드 전송
	std::vector<std::string> sendKeywords;
	std::vector<std::string> pairKeywords;
	std::vector<std::string> receivedKeywords;
	sendKeywords.push_back("DeviceConnect");			pairKeywords.push_back("NONE");
	sendKeywords.push_back("DeviceDisconnect");		pairKeywords.push_back("NONE");
	
	/*
	sendKeywords.push_back("TUM");			pairKeywords.push_back("NONE");
	*/
	sendKeywords.push_back(datakeyword);				pairKeywords.push_back("NONE");
	sendKeywords.push_back(keydataset);					pairKeywords.push_back("NONE");
	sendKeywords.push_back(itype1);						pairKeywords.push_back("NONE");
	if (nCamType > 0) {
		sendKeywords.push_back(itype2);						pairKeywords.push_back("NONE");
	}
	receivedKeywords.push_back("ReferenceFrame");

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
	/*std::string keydataset = "TUM";
	std::string scene_id = "2_desk_with_person";
	std::string src_cam = keydataset + "2.intrinsic";*/
	//keydataset = "BONN";
	//scene_id = "placing_nonobstructing_box";
	/*std::string keydataset = "NUIM";
	std::string scene_id = "office_1";*/
	//src_cam = keydataset + ".intrinsic";

	
	std::string srct = keydataset + (scene_id)+".ts";
	{
		std::stringstream ss;
		ss << "/Load?keyword=" << keydataset << "&src=" << src_cam;
		auto res = API.Send(ss.str(), "");
		ss.str("");
	}
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
	

	double tframe = 20.0;
	cv::Mat cam_data;
	bool basdf = true;
	//while(basdf)
	//{
		std::stringstream ss;
		ss << "/Download?keyword="<<keydataset << "&id=0"<< "&src=" << src_cam;
		auto res2 = API.Send(ss.str(), "");
		std::cout << res2.size() << std::endl;
		cam_data = cv::Mat(res2.size()/4, 1, CV_32FC1, (void*)res2.data());
		tframe = ((double)1.0) / (double)cam_data.at<float>(11);
		if (tframe < 1.0)
		{
			basdf = false;
			std::cout << "success = " << cam_data.t() <<" "<<cam_data.rows << std::endl;
		}
		else {
			std::cout << "fail = " << cam_data.t() << std::endl;
		}
	//}
	
	cv::Mat ids;
	{
		std::stringstream ss;
		ss << "/Get?keyword=" << keydataset<< "&src=" << srct;
		auto res = API.Send(ss.str(), "");
		ids = cv::Mat(res.size()/4, 1, CV_32SC1, (void*)res.data());
	}
	
	{
		//device connect
		int fid = 0;
		int nInt = 20;
		int nByte = 10;

		
		//11,12,13 : 퀄리티, 스킵, ?
		//14,15,16 뎁스 관련. mbf, mthdepth, mdepthmapfactor
		//17, 18 : 카메라 : 0 모노, 1 : 스테레오 2: 뎁스
		cv::Mat temp2 = (cv::Mat_<float>(nInt, 1) << cam_data.at<float>(0), cam_data.at<float>(1),
			//dataset->fx,dataset->fy,dataset->cx,dataset->cy, 
			//dataset->d1, dataset->d2, dataset->d3, dataset->d4, dataset->d5, 
			cam_data.at<float>(2), cam_data.at<float>(3), cam_data.at<float>(4), cam_data.at<float>(5),
			cam_data.at<float>(6), cam_data.at<float>(7), cam_data.at<float>(8), cam_data.at<float>(9), cam_data.at<float>(10),
			quality, nskip, cam_data.at<float>(11), cam_data.at<float>(12), cam_data.at<float>(13), 5000.0,
			nCamType,nIMUType,nFeature);
		//cam_data.rowRange(0,11).copyTo(temp2.rowRange(0 , 11));

		//cv::Mat temp1 = cv::Mat::zeros(nInt * 4, 1, CV_8UC1);
		//std::memcpy(temp1.data, temp2.data, nInt*sizeof(int));
		cv::Mat temp1(temp2.rows * 4, 1, CV_8UC1, temp2.data);

		int nbFlagIdx = 0;// nInt * 4;
		cv::Mat tempb = cv::Mat::zeros(nByte, 1, CV_8UC1);
		//매핑
		tempb.at<uchar>(nbFlagIdx) = bMapping ? 1 : 0;
		tempb.at<uchar>(nbFlagIdx + 1) = bTracking ? 1 : 0;
		tempb.at<uchar>(nbFlagIdx + 4) = bPlaneGBA ? 1 : 0;
		tempb.at<uchar>(nbFlagIdx + 5) = bSyncLocalMap ? 1 : 0;
		tempb.at<uchar>(nbFlagIdx + 7) = bGridCommu ? 1 : 0;
		tempb.at<uchar>(nbFlagIdx + 8) = bSave? 1 : 0;
		tempb.at<uchar>(nbFlagIdx + 9) = bPLP? 1 : 0;
		std::string strtemp = src + "," + mapname;
		cv::Mat temp3(strtemp.length(), 1, CV_8UC1, (void*)strtemp.c_str());
		temp1.push_back(tempb);
		temp1.push_back(temp3);
		std::stringstream ss;

		ss << "/Upload?keyword=DeviceConnect&id=" << fid << "&src=" << src;// << "&type2=" << user->userName;
		auto res = API.Send(ss.str(), temp1.data, temp1.rows);

		cv::Mat temp4(20, 1, CV_32FC1, temp1.data);
		//std::cout << "?? = " << temp2.t() << std::endl;
		//std::cout << "aa = " << temp4.rowRange(0, 11).t() << std::endl;
	}
	
	//리턴 데이터 체크용
	/*ThreadPool::ThreadPool* POOL = new ThreadPool::ThreadPool(1);
	POOL->EnqueueJob(ReceiveData, sock);*/

	//int fps = 30;
	//std::cout <<"commu " << itype1 << " " << itype2 <<" "<<tframe<<" "<<nCamType <<" "<<ids.rows << std::endl;
	
	for (int i = 0; i < ids.rows; i++) {
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

		int id = i;
		//std::cout << id << std::endl;

		if (id % nskip == 0) {
			std::stringstream ss;
			ss << "/Download?keyword=" << keydataset << "" << "&id=" << id << "&src=" << srcc;// << " & type2 = " << user->userName;
			auto res = API.Send(ss.str(), "");
			cv::Mat cdata = cv::Mat(res.size(), 1, CV_8UC1, (void*)res.data());
			
			/*std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
			auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t1).count();
			std::cout << "download test = " << du_test1 << std::endl;*/

			//tx
			double ts;
			ss.str("");
			ss << "/Download?keyword=" << keydataset << "" << "&id=" << id << "&src=" << srct;// << " & type2 = " << user->userName;
			auto res2 = API.Send(ss.str(), "");
			ts = atof(res2.c_str());
			
			ss.str("");
			std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
			ss << "/Upload?keyword=" << itype1 << "&id=" << id << "&src=" << src << "&ts=" << t_start.time_since_epoch().count() << "&ts2="<<res2;;// << " & type2 = " << user->userName;
			auto res3 = API.Send(ss.str(), cdata.data, cdata.rows);
			
			if (nCamType > 0) {
				ss.str("");
				ss << "/Download?keyword=" << keydataset << "" << "&id=" << id << "&src=" << srcd;
				auto dres = API.Send(ss.str(), "");
				cv::Mat ddata = cv::Mat(dres.size(), 1, CV_8UC1, (void*)dres.data());
				{
					ss.str("");
					ss << "/Upload?keyword=" << itype2 << "&id=" << id << "&src=" << src;// << "&type2=" << user->userName;
					API.Send(ss.str(), ddata.data, ddata.rows);
				}
				/*cv::Mat img2 = cv::imdecode(ddata, cv::IMREAD_COLOR);
				cv::Mat visImg2 = img2.clone();
				cv::imshow("Asdf", visImg2);
				cv::waitKey(1);*/
			}
		}

		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
		
		if (ttrack < tframe) {
			auto diff = (tframe - ttrack) * dCamDelay;
			long long delay = (long long)diff;
			std::this_thread::sleep_for(std::chrono::milliseconds(delay));
			//std::cout << "delay = " << delay <<" = "<<tframe<<", "<<ttrack << std::endl;
		}

	}
	
	{
		std::stringstream ss;
		cv::Mat temp = cv::Mat::zeros(1000, 1, CV_32FC1);
		ss << "/Upload?keyword=DeviceDisconnect&id=" << ids.rows << "&src=" << src;// << "&type2=" << user->userName;
		auto res = API.Send(ss.str(), temp.data, temp.rows);
	}
	WebAPI::Close();

}
