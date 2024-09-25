#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <random>
#include <iostream>
#include <fstream>
#include <chrono>
#include "LoadDataset.h"
#include "ThreadPool.h"
#include "rapidjson/document.h"

#include <thread>
#include <Utils.h>
#include <ConcurrentMap.h>
#include <ConcurrentVector.h>
#include <ConcurrentDeque.h>

#include <sio_client.h>

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

void OnTestMessage(sio::event& e)
{
	std::cout << "???????" << std::endl;
}
std::mutex mMutexDownload;
void OnMessage3(const std::string& name, sio::message::ptr const& message, bool need_ack, sio::message::list& ack_message) {
	//std::unique_lock<std::mutex> lock(mMutexDownload);
	std::cout << "download = " << std::endl;
}
void OnMessage2(const sio::message::list & li) {
	//std::cout << "download " <<li.size()<< std::endl;
	std::cout << "asdfasdfasdf" << std::endl;
}

void OnError(const sio::message::ptr& e) {
	std::cout << "err" << std::endl;
}
std::chrono::high_resolution_clock::time_point start;
void OnMessage(sio::event& e)
{

	//std::unique_lock<std::mutex> lock(mMutexDownload);
	//std::unique_lock<std::mutex> lock(mMutexDownload);

	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	std::cout << e.get_message()->get_string().size() <<" "<<du_test1 << std::endl;

	//std::cout << "download = " <<e.get_message() << std::endl;
	//const std::shared_ptr<const std::string> msg = e.get_message()->get_binary();
	
	//const std::string* data = msg.get();
	//cv::Mat bdata = cv::Mat(data->size(), 1, CV_8UC1, (void*)data);
	//std::cout << bdata.rows << std::endl;
	 
	//std::cout << "???????" << std::endl;
}

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
		keydataset = argv[index++];
		scene_id = argv[index++];
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

int main(int argc, char* argv[]) {

	std::string ip;
	int port;
	ip = argv[1];
	port = atoi(argv[2]);
	std::string paramPath = argv[3];
	std::string dir = argv[4];//"E:/SLAM_DATASET/TUM/rgbd_dataset_freiburg2_desk_with_person/";

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
	ssdata << keydataset << scene_id << "_" << quality << ".";
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
	if (keydataset == "KITTI" || keydataset == "EuroC") {
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

	sio::client* pIO = new sio::client();

	pIO->connect("http://143.248.6.25:35005");
	//pIO->socket()->on_error()

	while (!pIO->opened())
	{
		//std::cout << pIO->opened() << std::endl;
	}
	
	//pIO->socket()->on_error(&OnError);
	pIO->socket()->on("download data", &OnMessage);
	/*pIO->socket()->on("download data", [&](sio::event& ev)
		{
			std::cout << "download " << std::endl;
		});*/


	double tframe = 1.0 / 30.0;

	auto mkeyword = sio::string_message::create(keydataset);
	auto msrc = sio::string_message::create(srcc);
	cv::Mat datat = cv::Mat::ones(50000, 1, CV_8UC1);
	cv::randu(datat, cv::Scalar(100), cv::Scalar(200));
	std::shared_ptr<std::string> pstr(new std::string((char*)datat.data));
	auto mpstr = sio::binary_message::create(pstr);

	for (int i = 0; i < 1000; i++) {
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
		
		if (i % nskip == 0) {
			start = t1;
			sio::message::list li;
			li.push(mkeyword);
			li.push(msrc);
			li.push(sio::int_message::create(i));
			li.push(mpstr);
			pIO->socket()->emit("download", li);
			//std::cout << "req " << i << std::endl;
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

}
