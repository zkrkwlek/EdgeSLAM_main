
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <WebApi.h>
#include "LoadDataset.h"
#include "rapidjson/document.h"
#include "ThreadPool.h"

#include <Utils.h>
//#include <Frame.h>
//#include <Camera.h>
//#include <FeatureTracker.h>
//#include <SearchPoints.h>
#include <thread> 

float CalculateDescDistance(cv::Mat a, cv::Mat b) {

	if (a.type() == CV_8UC1) {
		const int* pa = a.ptr<int32_t>();
		const int* pb = b.ptr<int32_t>();

		int dist = 0;

		for (int i = 0; i < 8; i++, pa++, pb++)
		{
			unsigned  int v = *pa ^ *pb;
			v = v - ((v >> 1) & 0x55555555);
			v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
			dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
		}
		return (float)dist;
	}
	if (a.type() == CV_32FC1) {
		float dist = (float)cv::norm(a, b, cv::NORM_L2);
		return dist;
	}
	return 0.0;
}

int nLastID = -1;
float fx, fy, cx, cy;
int w, h;
void LoadParameter(std::string path) {
	cv::FileStorage fs(path, cv::FileStorage::READ);
	fs["Image.width"] >> w;
	fs["Image.height"] >> h;

	fs["Camera.fx"] >> fx;
	fs["Camera.fy"] >> fy;
	fs["Camera.cx"] >> cx;
	fs["Camera.cy"] >> cy;


	//Pluker Line Coordinate에 이용함.
	//mK2 = (cv::Mat_<float>(3, 3) << fx, 0, 0, 0, fy, 0, -fy*cx, -fx*cy, fx*fy);

	fs.release();
}

void SendImage() {

}

#pragma comment(lib, "ws2_32")
#include <WS2tcpip.h>

bool bDone = true;
int nReID;
std::vector<double> vecTSs;
float nTotal = 0.0f;
float nSize = 0;
std::vector<cv::Mat> vecR, vecT;

int nSendID = -1;
std::vector<int> param = std::vector<int>(2);
std::vector<int> param2 = std::vector<int>(2);

void SendData(WebAPI* api, const cv::Mat& img, std::string src, int fid) {
	std::vector<uchar> buffer;
	cv::imencode(".jpg", img, buffer, param);
	cv::Mat encoded(buffer);
	std::stringstream ss;
	ss << "/Upload?keyword=SimImage&id=" << fid << "&src=" << src;// << "&type2=" << user->userName;
	auto res = api->Send(ss.str(), encoded.data, encoded.rows);
	nSendID = fid;
}

void ReceiveData(SOCKET sock) {
	
	sockaddr_in clientaddr;
	int addrlen;
	char rbuf[1024];

	WebAPI API("asdf",123);

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

		if (document["keyword"].IsString() && document["id"].IsInt()) {
			int id = document["id"].GetInt();
			double ts = std::stod(document["ts"].GetString());
			std::string keyword = document["keyword"].GetString();
			std::string src = document["src"].GetString();
			std::string type2 = document["type2"].GetString();
			
			//////load data with keyword
			int id2 = -1;
			std::stringstream ss;
			if (document.HasMember("id2")) {
				id2 = document["id2"].GetInt();
				ss << "/Download?keyword=" << keyword << "&id=" << id << "&id2=" << id2 << "&src=" << src;
			}
			else
				ss << "/Download?keyword=" << keyword << "&id=" << id << "&src=" << src;
			
			auto res = API.Send(ss.str(), "");
			
			if (keyword == "WiseUITest") {
				cv::Mat temp = cv::Mat(res.size()/4, 1, CV_32FC1, (void*)res.data());
				cv::Mat R = cv::Mat(3, 3, CV_32FC1, (void*)res.data());
				cv::Mat t = cv::Mat(3, 1, CV_32FC1,(void*)(res.data()+36));
				//temp.rowRange(9, 12).copyTo(t);
				float tt = temp.at<float>(12);
				//std::cout << temp.size() << std::endl;
				//std::cout << R << t.t() <<tt<< std::endl;
				vecR.push_back(R.clone());
				vecT.push_back(t.clone());
				nTotal += tt*1000.0;
				nSize++;
			}
			nReID = id;
						
			if (id == nLastID)
			{
				break;
			} 
		}
	}//wihle

	bDone = false;
}

WSAData wsaData;
int code = WSAStartup(MAKEWORD(2, 2), &wsaData);
int sock = socket(AF_INET, SOCK_DGRAM, 17);
sockaddr_in serveraddr;

std::string keydataset = "";
std::string scene_id = "";
std::string src_cam = "";

void parsing(char* argv[], int& index) {
	std::string ts = Utils::GetTimeStamp("_").str();
	std::string keyword = (argv[index++]);

	std::cout << keyword << std::endl;

	if (keyword == "--dataset")
	{
		keydataset = argv[index++];
		scene_id = argv[index++];
		src_cam = argv[index++];
		std::cout << keydataset << " " << src_cam << std::endl;
	}
	else if (keyword == "--load")
	{

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
		
	int fps = 30;
	double tframe = ((double)1.0) / fps;

	//데이터 폴더
	/*LoadParameter(paramPath);
	std::cout << fx << " " << fy << " " << cx << std::endl;*/
	std::cout << ip << " " << port << std::endl;
	
	std::string dir = argv[4];//"E:/SLAM_DATASET/TUM/rgbd_dataset_freiburg2_desk_with_person/";
	std::string res_path = argv[5];
	parser(argc, argv, 6);

	WebAPI::ip = ip;
	WebAPI::port = port;
	WebAPI::Connect();
	WebAPI API(ip, port);
		
	param[0] = cv::IMWRITE_JPEG_QUALITY;
	param[1] = 70;//default(95) 0-100
	
	param2[0] = cv::IMWRITE_PNG_COMPRESSION;
	param2[1] = 1;//default(95) 0-100

	int nSkip = 4;
	int fid = 0;
	

	std::string src = "WiseUI";
	bool bMapping = true;
	bool bTracking = true;
	 
	//std::string dir = "E:/SLAM_DATASET/Hololens2/oxrst_room1_r2/pinhole_projection/";
	//LoadDataset* dataset = (LoadDataset*)new Hololens2Dataset(dir);

	int nTimeoutValue = INT_MAX;
	int aaa = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&nTimeoutValue, sizeof(nTimeoutValue));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(35001); // 포트 번호
	//serveraddr.sin_addr.s_addr = inet_addr("143.248.6.143");
	std::string addr = ip;
	inet_pton(AF_INET, addr.c_str(), &serveraddr.sin_addr.s_addr);

	std::vector<std::string> sendKeywords;
	std::vector<std::string> pairKeywords;

	sendKeywords.push_back("DeviceConnect");			pairKeywords.push_back("NONE");
	sendKeywords.push_back("DeviceDisconnect");			pairKeywords.push_back("NONE");
	sendKeywords.push_back(keydataset);					pairKeywords.push_back("NONE");
	sendKeywords.push_back("SimImage");			pairKeywords.push_back("NONE");

	for (int i = 0, iend = sendKeywords.size(); i < iend; i++) {
		std::stringstream ss;
		ss << "{\"src\":\"" << src << "\"," << "\"keyword\":\"" << sendKeywords[i] << "\",\"type1\":\"server\",\"type2\":\"" << pairKeywords[i] << "\"" << ",\"capacity\":" << 300 << "}"; //test\"}";
		auto res = API.Send("/Connect", ss.str());
	}
	
	std::vector<std::string> receivedKeywords;
	receivedKeywords.push_back("WiseUITest");
	
	for (int i = 0, iend = receivedKeywords.size(); i < iend; i++) {
		std::stringstream ss;
		ss << "{\"type1\":" << "\"connect\"" << ",\"type2\":" << "\"single\"" << ",\"src\":\"" << src << "\",\"keyword\":\"" << receivedKeywords[i] << "\"" << "}";
		std::string temp = ss.str();
		auto buf = temp.c_str();
		int retval;
		retval = sendto(sock, buf, strlen(buf), 0, (sockaddr*)&serveraddr, sizeof(serveraddr));
	}
	
	bool bInit = false;
	if (!bInit) {
		
		{
			//서버에 데이터 올리기
			std::stringstream ss;
			ss << "/Load?keyword=" << keydataset << "&src=" << src_cam;
			auto res = API.Send(ss.str(), "");
			ss.str("");
		}

		{
			bool basdf = true;
			cv::Mat cam_data;
			{
				std::stringstream ss;
				ss << "/Download?keyword=" << keydataset << "&id=0" << "&src=" << src_cam;
				auto res2 = API.Send(ss.str(), "");
				
				cam_data = cv::Mat(res2.size() / 4, 1, CV_32FC1, (void*)res2.data());
				tframe = ((double)1.0) / (double)cam_data.at<float>(11);
				if (tframe < 1.0)
				{
					basdf = false;
					std::cout << "success = " << cam_data.t() << " " << cam_data.rows << std::endl;
				}
				else {
					std::cout << "fail = " << cam_data.t() << std::endl;
				}
			}
			cv::Mat a = cv::Mat::zeros(300, 300, CV_32FC1);
			
			int nInt = 20;
			int nByte = 10;
			int quality = 70;
			
			//11,12,13 : 퀄리티, 스킵, ?
			//14,15,16 뎁스 관련. mbf, mthdepth, mdepthmapfactor
			//17, 18 : 카메라 : 0 모노, 1 : 스테레오 2: 뎁스
			cv::Mat temp2 = (cv::Mat_<float>(nInt, 1) << cam_data.at<float>(0), cam_data.at<float>(1),
				//dataset->fx,dataset->fy,dataset->cx,dataset->cy, 
				//dataset->d1, dataset->d2, dataset->d3, dataset->d4, dataset->d5, 
				cam_data.at<float>(2), cam_data.at<float>(3), cam_data.at<float>(4), cam_data.at<float>(5),
				cam_data.at<float>(6), cam_data.at<float>(7), cam_data.at<float>(8), cam_data.at<float>(9), cam_data.at<float>(10),
				quality, nSkip, cam_data.at<float>(11), cam_data.at<float>(12), cam_data.at<float>(13), 5000.0,
				0, 0, 2000);

			cv::Mat temp1(temp2.rows * 4, 1, CV_8UC1, temp2.data);

			int nbFlagIdx = 0;// nInt * 4;
			cv::Mat tempb = cv::Mat::zeros(nByte, 1, CV_8UC1);
			//매핑
			bool bPlaneGBA = false;
			bool bSyncLocalMap = false;
			bool bGridCommu = false;
			bool bSave = false;
			bool bPLP = false;
			//매핑
			tempb.at<uchar>(nbFlagIdx) = bMapping ? 1 : 0;
			tempb.at<uchar>(nbFlagIdx + 1) = bTracking ? 1 : 0;
			tempb.at<uchar>(nbFlagIdx + 4) = bPlaneGBA ? 1 : 0;
			tempb.at<uchar>(nbFlagIdx + 5) = bSyncLocalMap ? 1 : 0;
			tempb.at<uchar>(nbFlagIdx + 7) = bGridCommu ? 1 : 0;
			tempb.at<uchar>(nbFlagIdx + 8) = bSave ? 1 : 0;
			tempb.at<uchar>(nbFlagIdx + 9) = bPLP ? 1 : 0;
			std::string strtemp = src + ",wtestmap";
			cv::Mat temp3(strtemp.length(), 1, CV_8UC1, (void*)strtemp.c_str());
			temp1.push_back(tempb);
			temp1.push_back(temp3);

			std::cout << temp2.t() << std::endl;
			std::cout << strtemp << std::endl;

			////매핑
			//temp1.at<uchar>(52) = bMapping ? 1 : 0;
			////트래킹
			//temp1.at<uchar>(53) = bTracking ? 1 : 0;
			/*std::string strtemp = src + ",WiseUIMAP";
			cv::Mat temp3(strtemp.length(), 1, CV_8UC1, (void*)strtemp.c_str());*/
			//temp1.push_back(temp3);
			std::stringstream ss;
			ss << "/Upload?keyword=DeviceConnect&id=" << fid << "&src=" << src;// << "&type2=" << user->userName;
			auto res = API.Send(ss.str(), temp1.data, temp1.rows);

			//return -1;
		}
		bInit = true;
	}
	
	////키우더ㅡ 전송
	//나중에 커넥트도 추가해야 함
	
	
	LoadDataset* dataset = (LoadDataset*)new TumDataset(dir);
	
	ThreadPool::ThreadPool* POOL = new ThreadPool::ThreadPool(2);
	POOL->EnqueueJob(ReceiveData, sock);
		
	while (true) {
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
		double tts;
		cv::Mat img;
		cv::Mat depth;
		bool bimg = dataset->GrabImage(img, tts);
		bool bdepth = dataset->GrabDepth(depth);

		if(bimg)
		{
			++fid;
			if (fid % nSkip != 0) {
				continue;
			}
			POOL->EnqueueJob(SendData, &API, img, src, fid);
			vecTSs.push_back(tts);
			cv::imshow("image", img);
		}
		//if (bdepth) {
		//	if (fid % nSkip != 0) {
		//		continue;
		//	}
		//	std::vector<uchar> buffer;
		//	cv::imencode(".png", depth, buffer, param2);
		//	cv::Mat encoded(buffer);
		//	
		//	////https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
		//	////float factor = 5000;
		//	////CV_16UC1
		//	////std::cout << depth.type() <<" "<<CV_16SC1 << " " << CV_16UC1 << " " << depth.at<ushort>(200, 200)/ factor << " " << depth.at<short>(200, 200) << std::endl;
		//	cv::imshow("depth", depth);
		//}
		if (!bimg){
			nLastID = nSendID;
			break;
		}
		
		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
		if (ttrack < tframe) {
			auto diff = (tframe - ttrack) * 1e3;
			long long delay = (long long)diff;
			std::this_thread::sleep_for(std::chrono::milliseconds(delay));
			//std::cout << "delay = " << delay << std::endl;
		}
		cv::waitKey(1);
	}
	std::cout << "wait return value" << std::endl;
	while (bDone){
		//continue;
	}

	float avgTime = nTotal / nSize;
	std::cout << "AVG Tracking time(ms) = " << nTotal / nSize << std::endl;
	std::cout << "FPS : " << 1000.0 / avgTime << std::endl;

	{
		//std::cout << vecR.size() << " " << vecTSs.size() << " " << vecT.size() << std::endl;
		std::stringstream ss;
		ss << res_path;
		std::ofstream f;
		f.open(ss.str().c_str());
		f << std::fixed;
		for (int i = 0; i < vecR.size(); i ++) {

			cv::Mat R = vecR[i];
			cv::Mat t = vecT[i];
			double  ts = vecTSs[i];
			R = R.t(); //inverse
			t = -R * t;  //camera center
			std::vector<float> q = Utils::toQuaternion(R);
			f << std::setprecision(6) << ts << std::setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
				<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;
		}
		f.close();
	}

	std::cout << "done " << std::endl;
	//다 끝나면
	{ 
		std::stringstream ss;
		cv::Mat temp = cv::Mat::zeros(1000, 1, CV_32FC1);
		ss << "/Upload?keyword=DeviceDisconnect&id=" << fid << "&src=" << src;// << "&type2=" << user->userName;
		auto res = API.Send(ss.str(), temp.data, temp.rows);
	}

	return 0;

	return 0;

}