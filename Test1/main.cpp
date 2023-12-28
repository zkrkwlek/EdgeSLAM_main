 
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

#include <./EdgeDeviceSLAM/include/Tracker.h>
#include <./EdgeDeviceSLAM/include/SearchPoints.h>
#include <./EdgeDeviceSLAM/include/RefFrame.h>
#include <./EdgeDeviceSLAM/include/Frame.h>
#include <./EdgeDeviceSLAM/include/Map.h>
#include <./EdgeDeviceSLAM/include/MapPoint.h>
#include <./EdgeDeviceSLAM/include/ORBDetector.h>
#include <./EdgeDeviceSLAM/include/Camera.h>
#include <./EdgeDeviceSLAM/include/CameraPose.h>
#include <./EdgeDeviceSLAM/include/MotionModel.h>
#include <./EdgeDeviceSLAM/include/Converter.h>

//#include <Camera.h>
//#include <FeatureTracker.h>
//#include <SearchPoints.h>
#include <thread>
#include <Utils.h>
#include <ConcurrentMap.h>
#include <ConcurrentVector.h>
#include <ConcurrentDeque.h>

float CalculateDescDistance(cv::Mat a, cv::Mat b) {
	
	if (a.type() == CV_8UC1) {
		const int *pa = a.ptr<int32_t>();
		const int *pb = b.ptr<int32_t>();

		int dist = 0;

		for (int i = 0; i<8; i++, pa++, pb++)
		{
			unsigned  int v = *pa ^ *pb;
			v = v - ((v >> 1) & 0x55555555);
			v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
			dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
		}
		return (float)dist;
	}
	if(a.type() == CV_32FC1){
		float dist = (float)cv::norm(a, b, cv::NORM_L2);
		return dist;
	}
	return 0.0;
}

float fx, fy, cx, cy;
float d1, d2, d3, d4, d5;
int nFeatures;
int nLevels;
float fScaleFactor;
int w, h;
void LoadParameter(std::string path) {
	cv::FileStorage fs(path, cv::FileStorage::READ);
	fs["Image.width"] >> w;
	fs["Image.height"] >> h;

	fs["Camera.fx"] >> fx;
	fs["Camera.fy"] >> fy;
	fs["Camera.cx"] >> cx;
	fs["Camera.cy"] >> cy;

	fs["Camera.k1"] >> d1;
	fs["Camera.k2"] >> d2;
	fs["Camera.p1"] >> d3;
	fs["Camera.p2"] >> d4;
	fs["Camera.k3"] >> d5;
		
	fs["ORBextractor.nFeatures"] >> nFeatures;
	fs["ORBextractor.nLevels"] >> nLevels;
	fs["ORBextractor.scaleFactor"] >> fScaleFactor;

	//Pluker Line Coordinate에 이용함.
	//mK2 = (cv::Mat_<float>(3, 3) << fx, 0, 0, 0, fy, 0, -fy*cx, -fx*cy, fx*fy);

	fs.release();
}

//argv
std::string datakeyword;
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

bool mbDelay = true;
bool mbSuccessInitialization = false;
std::chrono::high_resolution_clock::time_point time_initialization;
float mfInitTime;

std::atomic<int> nMatch = -1;
bool bSetReferenceFrame = false;
bool bSetLocalMap = false;

std::vector<int> depthParam(2);

EdgeDeviceSLAM::Frame* pCurrFrame = nullptr;
EdgeDeviceSLAM::Frame* pPrevFrame = nullptr;
EdgeDeviceSLAM::Camera* pCamera;
EdgeDeviceSLAM::ORBDetector* pDetector;
EdgeDeviceSLAM::MotionModel* pMotionModel;
EdgeDeviceSLAM::CameraPose* pCameraPose;
EdgeDeviceSLAM::Tracker* pTracker;
EdgeDeviceSLAM::Map* pMap;

EdgeDeviceSLAM::ORBDetector* EdgeDeviceSLAM::Tracker::Detector;
EdgeDeviceSLAM::ORBDetector* EdgeDeviceSLAM::SearchPoints::Detector;
EdgeDeviceSLAM::ORBDetector* EdgeDeviceSLAM::MapPoint::Detector;
EdgeDeviceSLAM::ORBDetector* EdgeDeviceSLAM::RefFrame::Detector;
EdgeDeviceSLAM::ORBDetector* EdgeDeviceSLAM::Frame::detector;

//ConcurrentMap<int, std::chrono::high_resolution_clock::time_point> mapVOSendTimes, mapImageSendTimes;
ConcurrentMap<int, cv::Mat> mapSendedImages; //서버에 전송한 이미지의 흑백 버전 저장
ConcurrentDeque<EdgeDeviceSLAM::RefFrame*> dequeRefFrames;//??
ConcurrentVector<std::string> EvaluationLatency;
std::vector<cv::Mat> vecTrajectories;
std::vector<double> vecTimestamps;
int nVODataSize = 17;

//전송 이미지 저장
void StoreImage(int id, const cv::Mat& img) {

	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);//COLOR_BGRA2GRAY, COLOR_RGBA2GRAY
	mapSendedImages.Update(id, gray.clone());

}
void EraseImage(int id) {
	if (mapSendedImages.Count(id))
		mapSendedImages.Erase(id);
}

//그래프 복원
int nQueueKeyFrame = 10;
void UpdateReferenceFrameFromEdgeServer(int fid, float* data) {
	int N = (int)data[2];
	
	if (N < 30)
		return;
	if (!mapSendedImages.Count(fid))
		//return dequeRefFrames.size();
		return;

	cv::Mat img = mapSendedImages.Get(fid);

	//맵포인트 갱신
	int Nmp = (int)data[15 + N * 5];
	int nMPidx = 16 + N * 5;
	
	if (!bSyncLocalMap) {
		for (int i = 0; i < Nmp; i++) {
			int id = (int)data[nMPidx++];
			int label = (int)data[nMPidx++];
			float x = data[nMPidx++];
			float y = data[nMPidx++];
			float z = data[nMPidx++];

			EdgeDeviceSLAM::MapPoint* pMP = nullptr;
			if (pMap->MapPoints.Count(id)) {
				pMP = pMap->MapPoints.Get(id);
				pMP->SetWorldPos(x, y, z);
			}
			else {
				pMP = new EdgeDeviceSLAM::MapPoint(id, x, y, z,pMap);
				pMap->MapPoints.Update(id, pMP);
			}
		}
	}

	//레퍼런스 프레임
	auto pRefFrame = new EdgeDeviceSLAM::RefFrame(pCamera, data + 2, pMap);
	pDetector->Compute(img, cv::Mat(), pRefFrame->mvKeys, pRefFrame->mDescriptors);

	int nRefMatches = pRefFrame->mvpMapPoints.size();
	pRefFrame->UpdateMapPoints();

	//로컬 맵 갱신
	if (!bSyncLocalMap) {

		dequeRefFrames.push_back(pRefFrame);
		////local map 갱신
		std::set<EdgeDeviceSLAM::MapPoint*> spMPs;

		////일정 레퍼런스 프레임이 생기면 디큐의 처음 레퍼런스 프레임 제거
		//옵저베이션 제거
		EdgeDeviceSLAM::RefFrame* firstRef = nullptr;
		if (dequeRefFrames.size() >= nQueueKeyFrame) {
			firstRef = dequeRefFrames.front();
			dequeRefFrames.pop_front();
			////delete ref frame
			if (firstRef) {
				auto vpMPs = firstRef->mvpMapPoints;
				for (int i = 0; i < firstRef->N; i++) {
					auto pMP = vpMPs[i];
					if (!pMP || pMP->isBad()) {
						continue;
					}
					pMP->EraseObservation(firstRef);
				}
				//delete firstRef;
			}
		}

		////로컬맵 생성
		std::vector<EdgeDeviceSLAM::MapPoint*> vecMPs;
		auto vecRefFrames = dequeRefFrames.get();
		for (int i = 0; i < vecRefFrames.size(); i++) {
			auto ref = vecRefFrames[i];
			if (!ref)
				continue;
			auto vpMPs = ref->mvpMapPoints;
			for (int j = 0; j < ref->N; j++) {
				auto pMP = vpMPs[j];
				if (!pMP || pMP->isBad() || spMPs.count(pMP)) {
					continue;
				}
				if (pRefFrame->is_in_frustum(pMP, 0.5)) {
					vecMPs.push_back(pMP);
					spMPs.insert(pMP);
				}
			}
		}
		pMap->LocalMapPoints.Copy(vecMPs);
		if (!bSetLocalMap) {
			bSetLocalMap = true;
		}
	}

	pMap->ReferenceFrame.Update(pRefFrame);
	if (!bSetReferenceFrame) {
		bSetReferenceFrame = true;
	}
	//std::cout << "Received MPs = " << N <<", "<< nRefMatches << std::endl;
}
void UpdateLocalMapFromEdgeServer(float* data) {
	int nSize = (int)data[0];
	if (nSize == 2) {
		return;
	}

	int nLocalMap = (int)data[2];
	int nObsIdx = 3;
	int nUpdatedMPs = (int)data[nObsIdx + nLocalMap];
	int nUpdatedIdx = nObsIdx + nLocalMap + 1;

	std::vector<EdgeDeviceSLAM::MapPoint*> vpMPs;// = std::vector<EdgeSLAM::MapPoint*>(n, static_cast<EdgeSLAM::MapPoint*>(nullptr));

	for (int i = 0; i < nUpdatedMPs; i++) {

		int id = (int)data[nUpdatedIdx++];
		float minDist = data[nUpdatedIdx++];
		float maxDist = data[nUpdatedIdx++];

		//Xw
		float x = data[nUpdatedIdx++];
		float y = data[nUpdatedIdx++];
		float z = data[nUpdatedIdx++];
		cv::Mat X = (cv::Mat_<float>(3, 1) << x, y, z);

		//normal
		float nx = data[nUpdatedIdx++];
		float ny = data[nUpdatedIdx++];
		float nz = data[nUpdatedIdx++];
		cv::Mat norm = (cv::Mat_<float>(3, 1) << nx, ny, nz);

		//desc
		void* ptrdesc = data + nUpdatedIdx;
		nUpdatedIdx += 8;
		cv::Mat desc(1, 32, CV_8UC1, ptrdesc);

		EdgeDeviceSLAM::MapPoint* pMP = nullptr;
		if (pMap->MapPoints.Count(id)) {
			pMP = pMap->MapPoints.Get(id);
			pMP->SetWorldPos(X.at<float>(0), X.at<float>(1), X.at<float>(2));
		}
		else {
			pMP = new EdgeDeviceSLAM::MapPoint(id, X.at<float>(0), X.at<float>(1), X.at<float>(2),pMap);
			pMap->MapPoints.Update(id, pMP);
		}
		pMP->SetMapPointInfo(minDist, maxDist, norm);
		pMP->SetDescriptor(desc);

	}

	int nError = 0;
	//add not updated mps
	for (int i = 0; i < nLocalMap; i++) {
		int id = (int)data[nObsIdx++];

		EdgeDeviceSLAM::MapPoint* pMP = nullptr;
		if (pMap->MapPoints.Count(id)) {
			pMP = pMap->MapPoints.Get(id);
			vpMPs.push_back(pMP);
		}
		else
			nError++;
	}
	pMap->LocalMapPoints.Copy(vpMPs);
	if (!bSetLocalMap) {
		bSetLocalMap = true;
	}

	//bReqLocalMap = false;
	//nLastKeyFrameId = id;
}
void ParsingData(int fid, float* totalData, int totalLength) {
	int nextid = 0;
	int size = (int)totalData[nextid];
	int pid = (int)totalData[nextid + 1];
	//std::cout << pid << " " << nextid << " " << size << " " << totalLength << std::endl;
	while (nextid < totalLength)
	{
		if (pid == 1)
		{
			//맵포인트 생성
			//int Nmp = (int)totalData[nextid + 2];
			//visualization
			//float[] points = new float[Nmp * 3];
			//GCHandle handle2 = GCHandle.Alloc(points, GCHandleType.Pinned);
			//IntPtr ptr2 = handle2.AddrOfPinnedObject();
			//mTracker.CreateKeyFrame(id, ptr, nextid, ptr2);
			//mPointCloudManager.UpdateMPs(ref totalData, ref points, Nmp, nextid);
			//handle2.Free();
			//EraseImage(id);
			//update keyframe
			UpdateReferenceFrameFromEdgeServer(fid, totalData + nextid);
			EraseImage(fid);
		}
		else if (pid == 2)
		{
			//update local map
			//mTracker.UpdateData(id, ptr + nextid);
			UpdateLocalMapFromEdgeServer(totalData + nextid);
		}
		else if (pid == 3)
		{
			//mContentManager.UpdateVirtualFrame(id, ref totalData, nextid, receivedTime, mExParam.bEdgeBase);
		}
		else if (pid == 4)
		{
			//mPlaneManager.UpdateLocalPlane(id, ref totalData, nextid);
		}
		//std::cout << size << " " << nextid << " " << totalLength << std::endl;
		//갱신
		size = (int)totalData[nextid];
		nextid += size;
		if (nextid < totalLength)
			pid = (int)totalData[nextid + 1];

	}

}

int Tracking(const cv::Mat& img, int id, double ts) {

	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

	if (pPrevFrame)
		delete pPrevFrame;
	pPrevFrame = pCurrFrame;
	pCurrFrame = new EdgeDeviceSLAM::Frame(gray, pCamera, id);

	bool bTrack = false;
	
	if (!bSetReferenceFrame || !bSetLocalMap)
		return -1;
	if (pTracker->mTrackState == EdgeDeviceSLAM::TrackingState::NotEstimated || pTracker->mTrackState == EdgeDeviceSLAM::TrackingState::Failed) {

		EdgeDeviceSLAM::RefFrame* rf = pMap->ReferenceFrame.Get();
		if (rf) {
			nMatch = pTracker->TrackWithReferenceFrame(rf, pCurrFrame, 100.0, 50.0);
			bTrack = nMatch >= 10;
			if (bTrack) {
				pTracker->mnLastRelocFrameId = pCurrFrame->mnFrameID;
			}
		}
	}

	if (pTracker->mTrackState == EdgeDeviceSLAM::TrackingState::Success) {
		
		pCurrFrame->SetPose(pMotionModel->predict());
		nMatch = pTracker->TrackWithPrevFrame(pPrevFrame, pCurrFrame, 100.0, 50.0);
		bTrack = nMatch >= 10;
		if (!bTrack) {
			EdgeDeviceSLAM::RefFrame* rf = pMap->ReferenceFrame.Get();
			if (rf) {
				nMatch = pTracker->TrackWithReferenceFrame(rf, pCurrFrame, 100.0, 50.0);
				bTrack = nMatch >= 10;
			}
		}
	}
	if (bTrack) {
		auto vecLocalMPs = pMap->LocalMapPoints.get();
		nMatch = 4;
		//WriteLog("Localization::TrackLocalMap::Start");
		nMatch = pTracker->TrackWithLocalMap(pCurrFrame, vecLocalMPs, 100.0, 50.0);
		//WriteLog("Localization::TrackLocalMap::End");
		if (pCurrFrame->mnFrameID < pTracker->mnLastRelocFrameId + 30 && nMatch < 30) {
			bTrack = false;
		}
		else if (nMatch < 30) {
			bTrack = false;
		}
		else {
			bTrack = true;
		}
	}
	
	if (bTrack) {
		pTracker->mTrackState = EdgeDeviceSLAM::TrackingState::Success;
		cv::Mat T = pCurrFrame->GetPose();
		pCameraPose->SetPose(T);
		pMotionModel->update(T);
		
		////유니티에 카메라 포즈 복사
		////R과 카메라 센터
		//cv::Mat t = T.col(3).rowRange(0, 3).t();
		cv::Mat P = cv::Mat::zeros(4, 3, CV_32FC1);
		T.rowRange(0, 3).colRange(0, 3).copyTo(P.rowRange(0, 3));
		//센터 또는 t임.
		cv::Mat Ow = pCameraPose->GetCenter().t();
		//cv::Mat Ow = T.col(3).rowRange(0,3).t();
		Ow.copyTo(P.row(3));
		vecTrajectories.push_back(T);
		vecTimestamps.push_back(ts);
	}
	else {
		pTracker->mTrackState = EdgeDeviceSLAM::TrackingState::Failed;
		pCameraPose->Init();
		pMotionModel->reset();
	}

	bool bres = bTrack;
	//std::cout << "tracking test = " << nMatch << std::endl;
	return -1;
}

int nLastID = -1;
int nSendID = -1;
bool bDone = true;
int nReID;
int nVoID = 1;

void SendVirtualObject(WebAPI* api, std::string src) {
	
	int nSendID = nVoID;
	cv::Mat vo = cv::Mat::zeros(nVODataSize, 1, CV_32FC1);
	vo.at<float>(0) = 17;
	std::chrono::high_resolution_clock::time_point t_up_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss;
	ss << "/Store?keyword=" << "VO.CREATE" << "&id=" << nSendID << "&src=" << src<<"&ts="<<t_up_start.time_since_epoch().count();// << " & type2 = " << user->userName;
	api->Send(ss.str(), vo.data, vo.rows * sizeof(float));
	std::chrono::high_resolution_clock::time_point t_up_end = std::chrono::high_resolution_clock::now();

	//latency start = up_end
	//mapVOSendTimes.Update(nSendID, t_start);

	auto du_up = std::chrono::duration_cast<std::chrono::milliseconds>(t_up_end - t_up_start).count();
	ss.str("");
	//num,source,method,type,id,quality,size,latency
	ss << "1," << src << "," << "VO" << ",upload," << nSendID << "," << "0" << "," <<vo.rows*sizeof(float) << "," << du_up << std::endl;
	EvaluationLatency.push_back(ss.str());
	nVoID++;
}

void SendImageData(WebAPI* api, const cv::Mat& img, std::string src, std::string keyword, int fid, std::string encoding_type, std::vector<int> param, double timestamp) {
	std::vector<uchar> buffer;
	cv::imencode(encoding_type, img, buffer, param);
	cv::Mat encoded(buffer);
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss;
	ss << "/Store?keyword="<< keyword <<"&id=" << fid << "&src=" << src<<"&ts="<<t_start.time_since_epoch().count()<<"&ts2="<< std::setprecision(16) <<timestamp;
	auto res = api->Send(ss.str(), encoded.data, encoded.rows);
	nSendID = fid;
	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
	ss.str("");
	ss <<"1," <<src << "," << keyword << ",upload," << fid << "," << param[1] << "," << encoded.rows << "," << duration << std::endl;
	EvaluationLatency.push_back(ss.str());
	//mapImageSendTimes.Update(fid, t_start);
}

#pragma comment(lib, "ws2_32")
#include <WS2tcpip.h>

void ReceiveData(SOCKET sock) {

	sockaddr_in clientaddr;
	int addrlen;
	char rbuf[1024];

	WebAPI API("asdf", 123);

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
				ss << "/Load?keyword=" << keyword << "&id=" << id << "&id2=" << id2 << "&src=" << src;
			}
			else
				ss << "/Load?keyword=" << keyword << "&id=" << id << "&src=" << src;

			/*if (!mbSuccessInitialization) {
				mbSuccessInitialization = true;
				mfInitTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_initialization).count();
				std::cout << "Initialization time = " << mfInitTime << std::endl;
			}*/

			//std::cout << "receive data start" << std::endl;
			//레퍼런스 프레임 복원.
			//알고리즘은 서버 따라서
			//download time
			std::chrono::high_resolution_clock::time_point t_down_start = std::chrono::high_resolution_clock::now();
			auto res = API.Send(ss.str(), "");
			std::chrono::high_resolution_clock::time_point t_down_end = std::chrono::high_resolution_clock::now();
			auto du_down = std::chrono::duration_cast<std::chrono::milliseconds>(t_down_end - t_down_start).count();
			
			//latency
			auto du_latency = (t_down_end.time_since_epoch().count() - (long long)ts) / 1000000.0;

			ss.str("");
			//num,source,method,type,id,quality,size,latency
			ss << "1," << src << "," << keyword <<",download," << id << "," << "0" << "," << res.size() << "," << du_down << std::endl;
			EvaluationLatency.push_back(ss.str());

			ss.str("");
			ss << "1," << src << "," << keyword << ",latency," << id << "," << "0" << "," << res.size() << "," << du_latency << std::endl;
			EvaluationLatency.push_back(ss.str());

			if (keyword == "ReferenceFrame") {
				//if (mapImageSendTimes.Count(id)) {
				//	auto t_start = mapImageSendTimes.Get(id);
				//	
				//	//std::cout << "receive image = " <<id<<", " << du_down << std::endl;
				//	std::stringstream ss;
				//	ss << src << "," << keyword << ",latency," << id << "," << "0" << "," << res.size() << "," << du_latency << std::endl;
				//	EvaluationLatency.push_back(ss.str());
				//}
				//std::cout << "parsing start" << std::endl;
				ParsingData(id, (float*)res.data(), res.size() / 4);
				//std::cout << "parsing end" << std::endl;
			}
			else if (keyword == "dr") {
				//if (mapVOSendTimes.Count(id)) {
				//	auto t_start = mapVOSendTimes.Get(id);
				//	auto du_latency = std::chrono::duration_cast<std::chrono::milliseconds>(t_down_start - t_start).count();
				//	//std::cout << "receive vo = " <<id<<", " << du_down << std::endl;
				//	std::stringstream ss;
				//	ss << src << "," << keyword << ",latency," << id << "," << "0" << "," << res.size() << "," << du_latency << std::endl;
				//	EvaluationLatency.push_back(ss.str());
				//}
			}

			//std::cout << "receive data = " << keyword << " " << res.size() << std::endl;
			//if (keyword == "WiseUITest") {
			//	cv::Mat temp = cv::Mat(res.size() / 4, 1, CV_32FC1, (void*)res.data());
			//	cv::Mat R = cv::Mat(3, 3, CV_32FC1, (void*)res.data());
			//	cv::Mat t = cv::Mat(3, 1, CV_32FC1, (void*)(res.data() + 36));
			//	//temp.rowRange(9, 12).copyTo(t);
			//	float tt = temp.at<float>(12);
			//	//std::cout << temp.size() << std::endl;
			//	//std::cout << R << t.t() <<tt<< std::endl;
			//	vecR.push_back(R.clone());
			//	vecT.push_back(t.clone());
			//	nTotal += tt * 1000.0;
			//	nSize++;
			//}
			nReID = id;

			if (id == nLastID)
			{
				break;
			}
		}
	}//wihle

	bDone = false;
}

void SaveTrajectory() {
	if (!bSaveTrajectory)
		return;
	std::ofstream f;
	f.open(trajectoryPath.c_str());
	f << std::fixed;
	for (int i = 0; i < vecTrajectories.size(); i ++) {
		cv::Mat R = vecTrajectories[i].rowRange(0, 3).colRange(0, 3);
		cv::Mat t = vecTrajectories[i].rowRange(0, 3).col(3);
		R = R.t(); //inverse
		t = -R * t;  //camera center
		std::vector<float> q = EdgeDeviceSLAM::Converter::toQuaternion(R);
		f << std::setprecision(6) << vecTimestamps[i] << std::setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
			<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;
	}
	f.close();
	std::cout << "save trajectory = " << trajectoryPath << std::endl;
}

void SaveLatency() {
	if (!bSaveLatency)
		return;
	std::ofstream file;
	file.open(latencyPath, std::ios::app);
	std::stringstream ss;
	auto vecDatas = EvaluationLatency.get();
	std::string strHeader = "num,source,method,type,id,quality,size,latency";
	//file.write(strHeader.c_str(),strHeader.size());
	ss << strHeader << std::endl;
	for (int i = 0, N = vecDatas.size(); i < N; i++) {
		ss << vecDatas[i];
	}
	file.write(ss.str().c_str(), ss.str().size());
	file.close();
	EvaluationLatency.Clear();
}

void parsing(char* argv[], int& index) {
	std::string ts = Utils::GetTimeStamp("_").str();
	std::string keyword = (argv[index++]);
	if (keyword == "--latency") {
		bSaveLatency = true;
		latencyPath = argv[index++];
		std::stringstream ss_file;
		ss_file << latencyPath << "_" <<ts<< ".csv";
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
}

void parser(int argc, char* argv[], int index) {
	std::cout << "start parsing" << std::endl;
	for (;index < argc;) {
		parsing(argv, index);
	}
}

int main(int argc, char* argv[]) {
	if(false)
	{
		cv::VideoCapture cap2("./aaa.mp4"); 
		cv::VideoCapture cap("./aaa.mp4");
		
		// 동영상의 오디오는 출력하지 않음
		
		if (!cap.isOpened()) {
			//cerr << "Camera open failed!" << endl;
			return -1;
		}

		std::cout << "Frame width: " << cvRound(cap.get(cv::CAP_PROP_FRAME_WIDTH)) << std::endl;
		std::cout << "Frame height: " << cvRound(cap.get(cv::CAP_PROP_FRAME_HEIGHT)) << std::endl;
		std::cout << "Frame count: " << cvRound(cap.get(cv::CAP_PROP_FRAME_COUNT)) << std::endl;	// 동영상파일 전체 프레임 수
		
		double fps2 = cap.get(cv::CAP_PROP_FPS);
		std::cout << "FPS: " << fps2 << std::endl;
		int delay = cvRound(1000 / fps2);
		// 동영상 파일의 FPS값을 이용하여 매 프레임 사이의 시간간격을 ms 단위로 구함
		// 초당 30프레임을 재생하는 동영상의 경우 딜레이는 33ms 가 됨

		cv::Mat frame, inversed;
		cv::Mat frame2;
		while (true) {
			cap >> frame;
			
			if (frame.empty())
				continue;
			inversed = ~frame;	// 반전

			imshow("frame", frame);
			imshow("inversed", inversed);
			std::cout << frame.size() << " " << frame.channels() << std::endl;
			if (cv::waitKey(delay) == 27)	// ESC key delay 값을 10ms으로 줄이면 영상이 빨라짐
				break;
		}
		cv::destroyAllWindows();

		return 0;
	}
	
	std::cout << Utils::GetTimeStamp("_").str() << std::endl;
	WSAData wsaData;
	int code = WSAStartup(MAKEWORD(2, 2), &wsaData);
	int sock = socket(AF_INET, SOCK_DGRAM, 17);
	sockaddr_in serveraddr;
	
	std::string ip;
	int port;
	ip = argv[1];
	port = atoi(argv[2]);
	std::string paramPath = argv[3];

	std::string dir = argv[4];//"E:/SLAM_DATASET/TUM/rgbd_dataset_freiburg2_desk_with_person/";
	LoadDataset* dataset = (LoadDataset*)new TumDataset(dir);

	WebAPI::ip = ip;
	WebAPI::port = port;

	int fps = 30;
	double tframe = ((double)1.0) / fps;

	//데이터 폴더
	LoadParameter(paramPath);

	//슬램 실행
	pDetector = new EdgeDeviceSLAM::ORBDetector(nFeatures, fScaleFactor, nLevels);
	pCamera = new EdgeDeviceSLAM::Camera(w, h, fx, fy, cx, cy, d1, d2, d3, d4);

	EdgeDeviceSLAM::Tracker::Detector = pDetector;
	EdgeDeviceSLAM::SearchPoints::Detector = pDetector;
	EdgeDeviceSLAM::RefFrame::Detector = pDetector;
	EdgeDeviceSLAM::Frame::detector = pDetector;
	EdgeDeviceSLAM::MapPoint::Detector = pDetector;

	pTracker = new EdgeDeviceSLAM::Tracker();
	pMap = new EdgeDeviceSLAM::Map();

	EdgeDeviceSLAM::RefFrame::nId = 0;
	pCameraPose = new EdgeDeviceSLAM::CameraPose();
	pMotionModel = new EdgeDeviceSLAM::MotionModel();

	pTracker->mTrackState = EdgeDeviceSLAM::TrackingState::NotEstimated;

	WebAPI API(ip, port);
	int nTimeoutValue = INT_MAX;
	int aaa = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&nTimeoutValue, sizeof(nTimeoutValue));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(35001); // 포트 번호
	//serveraddr.sin_addr.s_addr = inet_addr("143.248.6.143");
	std::string addr = ip;
	inet_pton(AF_INET, addr.c_str(), &serveraddr.sin_addr.s_addr);

	//배치파일에서 개별 전송하도록 변경
	{
		int idxArgv = 5;
		
		//Image, SimImage
		datakeyword = argv[idxArgv++];
		//사용자 이름
		std::string src = argv[idxArgv++]; 
		//맵 이름
		std::string mapname = argv[idxArgv++];
		//압축 퀄리티
		int quality = atoi(argv[idxArgv++]);
		//스킵 프레임
		int nskip = atoi(argv[idxArgv++]);
		
		//여기서부터 파싱 시작
		parser(argc, argv, idxArgv);
		//레이턴시 파일 저장
		//std::string latency_path = argv[idxArgv++];
		////포즈 정보 기록
		//traj_path = argv[idxArgv++];
		////데이터 타입
		////매핑
		//bMapping = atoi(argv[idxArgv++]) ? true : false;
		////트래킹
		//bTracking = atoi(argv[idxArgv++]) ? true : false;
		////로컬 맵 통신
		//bSyncLocalMap = atoi(argv[idxArgv++]) ? true : false;
		////가상 객체 통신
		//bVOTest = atoi(argv[idxArgv++]) ? true : false;

		std::vector<int> tempParam(2);
		tempParam[0] = cv::IMWRITE_JPEG_QUALITY;
		tempParam[1] = quality;//default(95) 0-100

		depthParam[0] = cv::IMWRITE_PNG_COMPRESSION;
		depthParam[1] = 1;

		//키워드 전송
		std::vector<std::string> sendKeywords;
		std::vector<std::string> pairKeywords;
		std::vector<std::string> receivedKeywords;
		sendKeywords.push_back("SimDeviceConnect");			pairKeywords.push_back("NONE");
		sendKeywords.push_back("SimDeviceDisconnect");		pairKeywords.push_back("NONE");
		sendKeywords.push_back("VO.CREATE");				pairKeywords.push_back("NONE");
		sendKeywords.push_back(datakeyword);				pairKeywords.push_back("NONE");
		if (bCaptureDepth) {
			sendKeywords.push_back("Depth");				pairKeywords.push_back("NONE");
		}
		receivedKeywords.push_back("ReferenceFrame");
		receivedKeywords.push_back("dr");

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

		//초기 딜레이
		if (mbDelay) {
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_int_distribution<long long> dist(/* 평균 = */ 0, /* 표준 편차 = */ 30);

			long long mnDelayValue = dist(gen) * 100;
			std::cout << "Delay Start " << mnDelayValue << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds((long long)mnDelayValue));
			std::cout << "Delay End" << std::endl;
		}

		//device connect
		int fid = 0;
		int nInt = 20;
		int nByte = 10;

		cv::Mat temp1 = cv::Mat::zeros(nInt * 4 + nByte, 1, CV_8UC1);
		cv::Mat temp2 = (cv::Mat_<float>(14, 1) << w, h, fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0, 0.0, quality, nskip, 10);
		std::memcpy(temp1.data, temp2.data, 56);

		int nbFlagIdx = nInt * 4;
		//매핑
		temp1.at<uchar>(nbFlagIdx) = bMapping ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 1) = bTracking ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 5) = bSyncLocalMap ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 8) = bVOTest ? 1 : 0;
		std::string strtemp = src + ","+mapname;
		cv::Mat temp3(strtemp.length(), 1, CV_8UC1, (void*)strtemp.c_str());
		temp1.push_back(temp3);
		std::stringstream ss;
		ss << "/Store?keyword=SimDeviceConnect&id=" << fid << "&src=" << src;// << "&type2=" << user->userName;
		auto res = API.Send(ss.str(), temp1.data, temp1.rows);

		//이미지 전송
		ThreadPool::ThreadPool* POOL = new ThreadPool::ThreadPool(3);
		POOL->EnqueueJob(ReceiveData, sock);

		//처음 시작 시간 설정
		//time_initialization = std::chrono::high_resolution_clock::now();

		while (true) {
			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			double ts_image,ts_depth;
			cv::Mat img;
			cv::Mat depth;
			bool bimg = dataset->GrabImage(img, ts_image);

			if (bimg)
			{
				++fid;

				//tracking
				if(bTracking){
					Tracking(img,fid, ts_image);
				}
				if(bGenVO && nVoID <= 300)
					POOL->EnqueueJob(SendVirtualObject, &API, src);
				if (fid % nskip == 0) {
					StoreImage(fid, img);
					POOL->EnqueueJob(SendImageData, &API, img, src, datakeyword, fid, ".jpg", tempParam, ts_image);
				}
				//vecTSs.push_back(tts);
				cv::imshow("image", img);
			}else{
				std::cout<< std::fixed << std::cout.precision(9) << " track = " << ts_image << std::endl;
			}

			if (bCaptureDepth) {
				//capture depth
				bool bdepth = dataset->GrabDepth(depth, ts_depth);
				if (bdepth) {
					//std::cout << "depth type = " << depth.type() << " " << CV_16SC1 << " " << CV_16UC1 <<" "<<CV_16SC2 <<" "<<CV_16UC2 << std::endl;
					//send
					POOL->EnqueueJob(SendImageData, &API, depth, src, "Depth", fid, ".png", depthParam, ts_depth);
					cv::imshow("depth", depth);
				}
				
			}


			if (!bimg) {
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

		std::cout << "done " << std::endl;
		//device disconnect
		{
			std::stringstream ss;
			cv::Mat temp = cv::Mat::zeros(1000, 1, CV_32FC1);
			ss << "/Store?keyword=SimDeviceDisconnect&id=" << fid << "&src=" << src;// << "&type2=" << user->userName;
			auto res = API.Send(ss.str(), temp.data, temp.rows);

			SaveLatency();
			SaveTrajectory();
			/*{
				std::ofstream file;
				file.open("../res/initialization.csv", std::ios::app);
				std::stringstream ss;
				ss << src << "," << mfInitTime<<std::endl;
				file.write(ss.str().c_str(), ss.str().size());
				file.close();
			}*/
		}
	}
	return 0;


	/// 시뮬레이터를 여기서 설정하던 것.
	//매퍼 설정
	//퀄리티
	//매핑
	//베이스 로컬 맵
	{
		int idxArgv = 6;
		int baseQuality = atoi(argv[idxArgv++]);
		bool baseMapping = atoi(argv[idxArgv++]) ? true : false;
		bool baseSyncLocalMap = atoi(argv[idxArgv++]) ? true : false;
		int Nclient = atoi(argv[idxArgv++]);
		int  clientQualtiy = atoi(argv[idxArgv++]);

		std::cout << baseQuality << " " << baseMapping << " " << baseSyncLocalMap << " " << Nclient << " " << clientQualtiy << std::endl;

		//클라이언트 개수

		std::string src = "eCARSimulator";
		std::vector<std::string> vecSims;
		std::vector<bool> vecMappings, vecBoolBases;

		std::vector<std::vector<int>> params;

		//baseline test
		{
			std::vector<int> tempParam(2);
			tempParam[0] = cv::IMWRITE_JPEG_QUALITY;
			tempParam[1] = baseQuality;//default(95) 0-100

			std::stringstream ss;
			ss << src << "_" << 0;
			vecSims.push_back(ss.str());
			params.push_back(tempParam);
			vecMappings.push_back(baseMapping);
			vecBoolBases.push_back(baseSyncLocalMap);
		}
		{
			for (int i = 0; i < Nclient; i++) {
				std::stringstream ss;
				ss << src << "_" << i + 1;
				vecSims.push_back(ss.str());
			}

			for (int i = 0; i < Nclient; i++) {
				std::vector<int> tempParam(2);
				tempParam[0] = cv::IMWRITE_JPEG_QUALITY;
				tempParam[1] = clientQualtiy;//default(95) 0-100
				params.push_back(tempParam);
			}

			for (int i = 0; i < Nclient; i++)
				vecMappings.push_back(false);

			for (int i = 0; i < Nclient; i++)
				vecBoolBases.push_back(false);
		}

		bool bTracking = true;
		int nSkip = 3;
		int fid = 0;

		//for (int s = 0; s < vecSims.size(); s++) {
		//	for (int i = 0, iend = sendKeywords.size(); i < iend; i++) {
		//		std::stringstream ss;
		//		ss << "{\"src\":\"" << vecSims[s] << "\"," << "\"keyword\":\"" << sendKeywords[i] << "\",\"type1\":\"server\",\"type2\":\"" << pairKeywords[i] << "\"" << ",\"capacity\":" << 300 << "}"; //test\"}";
		//		auto res = API.Send("/Connect", ss.str());
		//	}
		//}

		bool bInit = false;
		if (!bInit) {

			{
				int nInt = 20;
				int nByte = 10;

				for (int i = 0; i < vecSims.size(); i++) {
					cv::Mat temp1 = cv::Mat::zeros(nInt * 4 + nByte, 1, CV_8UC1);
					cv::Mat temp2 = (cv::Mat_<float>(14, 1) << w, h, fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0, 0.0, params[i][1], nSkip, 10);
					std::memcpy(temp1.data, temp2.data, 56);

					int nbFlagIdx = nInt * 4;
					//매핑
					temp1.at<uchar>(nbFlagIdx) = vecMappings[i] ? 1 : 0;
					//트래킹
					temp1.at<uchar>(nbFlagIdx + 1) = bTracking ? 1 : 0;
					temp1.at<uchar>(nbFlagIdx + 5) = vecBoolBases[i] ? 1 : 0;
					std::string strtemp = vecSims[i] + ",WiseUIMAP";
					cv::Mat temp3(strtemp.length(), 1, CV_8UC1, (void*)strtemp.c_str());

					temp1.push_back(temp3);
					std::stringstream ss;
					ss << "/Store?keyword=SimDeviceConnect&id=" << fid << "&src=" << vecSims[i];// << "&type2=" << user->userName;
					auto res = API.Send(ss.str(), temp1.data, temp1.rows);
				}
			}
			bInit = true;
		}

		////키우더ㅡ 전송
		//나중에 커넥트도 추가해야 함



		ThreadPool::ThreadPool* POOL = new ThreadPool::ThreadPool(11);
		while (true) {
			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			double tts;
			cv::Mat img;
			cv::Mat depth;
			bool bimg = dataset->GrabImage(img, tts);
			bool bdepth = dataset->GrabDepth(depth);

			if (bimg)
			{
				++fid;
				if (fid % nSkip != 0) {
					continue;
				}
				for (int i = 0; i < vecSims.size(); i++) {
					//POOL->EnqueueJob(SendImageData, &API, img, vecSims[i], fid, params[i],tts);
				}

				//vecTSs.push_back(tts);
				cv::imshow("image", img);
			}

			if (!bimg) {
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

		std::cout << "done " << std::endl;
		//다 끝나면
		for (int i = 0; i < vecSims.size(); i++)
		{
			std::stringstream ss;
			cv::Mat temp = cv::Mat::zeros(1000, 1, CV_32FC1);
			ss << "/Store?keyword=SimDeviceDisconnect&id=" << fid << "&src=" << vecSims[i];// << "&type2=" << user->userName;
			auto res = API.Send(ss.str(), temp.data, temp.rows);
		}

	}
	

	return 0;


	{
		std::vector<int> param;

		std::string dir = "E:/SLAM_DATASET/Hololens2/oxrst_room1_r2/pinhole_projection/";
		//Hololens2Dataset dataset(dir);
		LoadDataset* dataset = (LoadDataset*)new Hololens2Dataset(dir);


		////키우더ㅡ 전송
		//나중에 커넥트도 추가해야 함
		WebAPI API("143.248.6.143", 35005);
		std::vector<std::string> sendKeywords;
		std::vector<std::string> pairKeywords;

		sendKeywords.push_back("DeviceConnect");			pairKeywords.push_back("NONE");
		sendKeywords.push_back("OXR::POSE");			pairKeywords.push_back("NONE");
		sendKeywords.push_back("OXR::IMAGE");			pairKeywords.push_back("NONE");

		for (int i = 0, iend = sendKeywords.size(); i < iend; i++) {

			std::stringstream ss;
			ss << "{\"src\":\"" << "SLAMServer" << "\"," << "\"keyword\":\"" << sendKeywords[i] << "\",\"type1\":\"server\",\"type2\":\"" << pairKeywords[i] << "\"" << ",\"capacity\":" << 300 << "}"; //test\"}";
			auto res = API.Send("/Connect", ss.str());
		}

		cv::Mat F = cv::Mat::eye(4, 4, CV_32FC1);
		F.at<float>(1, 1) = -1;
		F.at<float>(2, 2) = -1;

		int nSkip = 4;
		int id = 0;
		bool bInit = false;

		std::string src = "PC";
		bool bMapping = true;
		bool bTracking = false;

		while (true) {
			cv::Mat img;
			cv::Mat depth;
			cv::Mat T;
			bool bimg = dataset->GrabImage(img, T);

			T = F * T * F;

			cv::Mat R = T.rowRange(0, 3).colRange(0, 3);
			cv::Mat t = T.rowRange(0, 3).col(3);
			R = R.t();
			t = -R * t;
			R.copyTo(T.rowRange(0, 3).colRange(0, 3));
			t.copyTo(T.rowRange(0, 3).col(3));

			bool bdepth = dataset->GrabDepth(depth);
			if (bdepth) {
				cv::imshow("depth", depth);
			}
			if (bimg) {

				cv::Mat K = dataset->GetCameraParam();
				float fx = K.at<float>(0, 0);
				float fy = K.at<float>(1, 1);
				float cx = K.at<float>(0, 2);
				float cy = K.at<float>(1, 2);

				if (!bInit) {

					{
						cv::Mat temp1 = cv::Mat::zeros(62, 1, CV_8UC1);
						cv::Mat temp2 = (cv::Mat_<float>(13, 1) << dataset->width, dataset->height, fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0, 0.0, 100, nSkip);
						std::memcpy(temp1.data, temp2.data, 52);
						//매핑
						temp1.at<uchar>(52) = bMapping ? 1 : 0;
						//트래킹
						temp1.at<uchar>(53) = bTracking ? 1 : 0;
						std::string strtemp = src + ",OXRMAP";
						cv::Mat temp3(strtemp.length(), 1, CV_8UC1, (void*)strtemp.c_str());

						temp1.push_back(temp3);
						std::stringstream ss;
						ss << "/Store?keyword=DeviceConnect&id=" << id << "&src=" << src;// << "&type2=" << user->userName;
						auto res = API.Send(ss.str(), temp1.data, temp1.rows);
					}

					bInit = true;
				}

				++id;
				if (id % nSkip != 0) {
					continue;
				}

				/*auto mpCamera = new EdgeSLAM::Camera(dataset->width, dataset->height,fx, fy, cx, cy, 0.0,0.0,0.0,0.0,0.0);
				auto mpFrame = new EdgeSLAM::Frame(img, mpCamera, 0, 0);
				mpFrame->SetPose(T);

				for (int i = 0; i < mpFrame->mvKeys.size(); i++)
					cv::circle(img, mpFrame->mvKeys[i].pt, 3, -1);*/

				std::stringstream ss;
				ss << "/Store?keyword=OXR::POSE&id=" << ++id << "&src=" << src;// << "&type2=" << user->userName;
				auto res = API.Send(ss.str(), T.data, sizeof(float) * 16);

				{
					std::vector<uchar> buffer;
					cv::imencode(".jpg", img, buffer, param);
					cv::Mat encoded(buffer);

					std::stringstream ss;
					ss << "/Store?keyword=OXR::IMAGE&id=" << id << "&src=" << src;// << "&type2=" << user->userName;
					auto res = API.Send(ss.str(), encoded.data, encoded.rows);
				}

				cv::imshow("image", img);
				//std::cout << T << std::endl;
				cv::waitKey(0);
			}
			else {
				std::cout << "fail load image" << std::endl;
			}

		}

		return 0;
		{
			std::stringstream ss;
			ss << "E:/SLAM_DATASET/ICL_NUIM/living_room_traj2_frei_png/rgb.txt";
			std::ifstream ifs;
			ifs.open(ss.str());

			std::stringstream ss2;
			ss2 << "E:/SLAM_DATASET/ICL_NUIM/living_room_traj2_frei_png/rgb2.txt";
			std::ofstream of;
			of.open(ss2.str());

			char line[1000] = { 0, };
			ifs.getline(line, 1000);
			of << line << std::endl;
			ifs.getline(line, 1000);
			of << line << std::endl;
			ifs.getline(line, 1000);
			of << line << std::endl;

			int idx = 0;
			//1부터 할지 0부터 할지
			while (!ifs.eof()) {
				std::string a, b;
				std::stringstream ss;
				ifs >> a >> b;

				std::cout << idx << " " << b << std::endl;
				of << idx << " " << b << std::endl;
				idx++;
				//ss << path << b;
			}
			of.close();

			return 0;
		}



		std::string path = "E:/SLAM_DATASET/TUM/rgbd_dataset_freiburg3_long_office_household/";
		std::stringstream ss;
		ss << "E:/SLAM_DATASET/TUM/rgbd_dataset_freiburg3_long_office_household/rgb.txt";
		std::cout << ss.str() << std::endl;

		std::ifstream ifs;
		ifs.open(ss.str());

		char line[1000] = { 0, };
		ifs.getline(line, 1000);
		ifs.getline(line, 1000);
		ifs.getline(line, 1000);



		float total = 640 * 480;

		cv::Ptr<cv::Feature2D> ORB = cv::ORB::create(1000);
		cv::Ptr<cv::Feature2D> SIFT = cv::SIFT::create(1000);
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);

		//디스크립터차이
		std::map<int, std::vector<float>> mapORBDescriptorDistance, mapSIFTDescriptorDistance;
		////압축률
		std::map<int, std::vector<double>> mapCompressionRatio, mapCompressionSize, mapPixelDistance;
		////인코딩 디코딩 시간
		std::map<int, std::vector<double>> mapEncodingTime, mapDecodingTime;

		////SuperPoint
		//{
		//	WebAPI* mpAPI = new WebAPI("143.248.6.143", 35005);
		//	std::vector<std::string> sendKeywords;
		//	std::vector<std::string> pairKeywords;
		//	sendKeywords.push_back("SuperPointTest");		pairKeywords.push_back("SuperPointTest2");

		//	for (int i = 0, iend = sendKeywords.size(); i < iend; i++) {
		//		std::stringstream ss;
		//		ss << "{\"keyword\":\"" << sendKeywords[i] << "\",\"type1\":\"Server\",\"type2\":\"" << pairKeywords[i] << "\"}"; //test\"}";
		//		std::cout << ss.str() << std::endl;
		//		mpAPI->Send("/Connect", ss.str());
		//		std::cout << "asdfasdf" << std::endl;
		//	}
		//	delete mpAPI;
		//}

		{
			std::map<int, std::vector<double>> mapEncodingTime, mapDecodingTime, mapCompressionSize;
			std::vector<int> param = std::vector<int>(2);
			param[0] = cv::IMWRITE_PNG_COMPRESSION;
			////PNG
			int nTotal = 0;
			while (!ifs.eof()) {
				if (nTotal == 2500)
					break;
				nTotal++;

				std::string a, b;
				std::stringstream ss;
				ifs >> a >> b;
				ss << path << b;
				std::cout << nTotal << " " << ss.str() << std::endl;
				cv::Mat img = cv::imread(ss.str());
				if (img.empty())
					continue;

				for (int i = 1; i < 10; i++) {
					param[1] = i;//0 - 9
					std::vector<uchar> buffer;

					std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
					cv::imencode(".png", img, buffer, param);
					std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
					auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
					double t_test1 = du_test1 / 1000.0;
					mapEncodingTime[i].push_back(t_test1);

					//decoding
					t1 = std::chrono::high_resolution_clock::now();
					cv::Mat jpeg = cv::imdecode(cv::Mat(buffer), cv::IMREAD_COLOR);
					t2 = std::chrono::high_resolution_clock::now();
					du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
					t_test1 = du_test1 / 1000.0;
					mapDecodingTime[i].push_back(t_test1);

					//압축 크기, 압축률
					mapCompressionSize[i].push_back(buffer.size());
				}


			}
			return 0;
			std::stringstream ssfile;
			ssfile << "../bin/PNG_EXPERIMENT.txt";
			std::ofstream f;
			f.open(ssfile.str().c_str());
			f << "Quality AVG Ratio Encoding Decoding" << std::endl;
			for (int i = 1; i < 10; i++) {

				//압축률
				cv::Mat csize(mapCompressionSize[i]);
				double avgCompressed = (640.0 * 480.0 * 3 - cv::mean(csize).val[0]) / (640.0 * 480 * 3) * 100.0;

				////인코딩 시간
				cv::Mat tencode(mapEncodingTime[i]);
				double ate = cv::mean(tencode).val[0];
				////디코딩 시간
				cv::Mat tdecode(mapDecodingTime[i]);
				double atd = cv::mean(tdecode).val[0];
				f << i << " " << cv::mean(csize).val[0] << " " << avgCompressed << " " << ate << " " << atd << std::endl;
				/*f << "Quality = " << i << std::endl;
				f << "Average compression size = " << cv::mean(csize).val[0] << std::endl;
				f << "압축률 = " << avgCompressed << " %" << std::endl;
				f << "Encoding Time = " << ate << " ms" << std::endl << "Decoding Time =" << atd << " ms" << std::endl;
				f << std::endl;*/
			}
			f.close();
			return 0;
		}

		int nTotal = 0;
		while (!ifs.eof()) {
			if (nTotal == 2500)
				break;
			nTotal++;
			/*ifs.getline(line, 1000);
			std::cout <<"a "<< line << std::endl;*/
			std::string a, b;
			std::stringstream ss;
			ifs >> a >> b;
			ss << path << b;
			std::cout << nTotal << " " << ss.str() << std::endl;
			cv::Mat img = cv::imread(ss.str());
			if (img.empty())
				continue;

			//{
			//	std::vector<uchar> buffer;
			//	cv::imencode(".png", img, buffer, param);
			//	WebAPI* mpAPI = new WebAPI("143.248.6.143", 35005);
			//	std::stringstream ss;
			//	ss << "/Store?keyword=SuperPointTest&id=" << nTotal << "&src=TestServer";// << "&type2=" << user->userName;
			//	auto res = mpAPI->Send(ss.str(), cv::Mat(buffer).data, buffer.size());
			//	delete mpAPI;
			//	continue;
			//}

			std::vector<cv::KeyPoint> vecORBKPs, vecSIFTKPs;
			cv::Mat oriGray, oriORBDesc, oriSIFTDesc;
			cv::cvtColor(img, oriGray, cv::COLOR_BGR2GRAY);
			ORB->detectAndCompute(oriGray, cv::Mat(), vecORBKPs, oriORBDesc);
			SIFT->detectAndCompute(oriGray, cv::Mat(), vecSIFTKPs, oriSIFTDesc);

			for (int i = 100; i >= 0; i -= 10) {
				//encoding
				if (i == 0)
					i = 1;
				param[1] = i;
				std::vector<uchar> buffer;
				std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
				cv::imencode(".jpg", img, buffer, param);
				std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
				auto du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
				double t_test1 = du_test1 / 1000.0;
				mapEncodingTime[i].push_back(t_test1);

				//decoding
				t1 = std::chrono::high_resolution_clock::now();
				cv::Mat jpeg = cv::imdecode(cv::Mat(buffer), cv::IMREAD_COLOR);
				t2 = std::chrono::high_resolution_clock::now();
				du_test1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
				t_test1 = du_test1 / 1000.0;
				mapDecodingTime[i].push_back(t_test1);

				//압축 크기, 압축률
				mapCompressionSize[i].push_back(buffer.size());

				//픽셀차이
				double diff = sum(abs(img - jpeg)).val[0];
				mapPixelDistance[i].push_back(diff);

				//디스크립터 계산
				cv::Mat jpegORBDesc, jpegSIFTDesc;
				cv::Mat jpegGray;
				cv::cvtColor(jpeg, jpegGray, cv::COLOR_BGR2GRAY);
				ORB->compute(jpegGray, vecORBKPs, jpegORBDesc);
				for (int j = 0, jend = jpegORBDesc.rows; j < jend; j++) {
					cv::Mat row1 = oriORBDesc.row(j);
					cv::Mat row2 = jpegORBDesc.row(j);
					//sumDesc += CalculateDescDistance(row1, row2);
					mapORBDescriptorDistance[i].push_back(CalculateDescDistance(row1, row2));
				}
				SIFT->compute(jpegGray, vecSIFTKPs, jpegSIFTDesc);
				for (int j = 0, jend = jpegSIFTDesc.rows; j < jend; j++) {
					cv::Mat row1 = oriSIFTDesc.row(j);
					cv::Mat row2 = jpegSIFTDesc.row(j);
					//sumDesc += CalculateDescDistance(row1, row2);
					mapSIFTDescriptorDistance[i].push_back(CalculateDescDistance(row1, row2));
				}
			}

		}

		std::stringstream ssfile;
		ssfile << "../bin/JpegExperiments.txt";
		std::ofstream f;
		f.open(ssfile.str().c_str());

		for (int i = 100; i >= 0; i -= 10) {

			if (i == 0)
				i = 1;

			//압축률
			cv::Mat csize(mapCompressionSize[i]);
			double avgCompressed = (640.0 * 480.0 * 3 - cv::mean(csize).val[0]) / (640.0 * 480 * 3) * 100.0;
			////픽셀 디스턴스
			cv::Mat pixelDist(mapPixelDistance[i]);
			double avgDiff = cv::mean(pixelDist).val[0] / (640.0 * 480 * 3);
			////인코딩 시간
			cv::Mat tencode(mapEncodingTime[i]);
			double ate = cv::mean(tencode).val[0];
			////디코딩 시간
			cv::Mat tdecode(mapDecodingTime[i]);
			double atd = cv::mean(tdecode).val[0];
			////디스크립터시간
			cv::Mat descORBDist(mapORBDescriptorDistance[i]);
			float avgORBDescDist = cv::mean(descORBDist).val[0];
			cv::Mat descSIFTDist(mapSIFTDescriptorDistance[i]);
			float avgSIFTDescDist = cv::mean(descSIFTDist).val[0];

			f << "Quality = " << i << std::endl;
			f << "Average compression size = " << cv::mean(csize).val[0] << std::endl;
			f << "압축률 = " << avgCompressed << " %" << std::endl;
			f << "Pixel Distance = " << avgDiff << std::endl;
			f << "Descriptor Distance(ORB) = " << avgORBDescDist << std::endl;
			f << "Descriptor Distance(SIFT) = " << avgSIFTDescDist << std::endl;
			f << "Descriptor Distance(SuperPoint) = " << std::endl;
			f << "Encoding Time = " << ate << " ms" << std::endl << "Decoding Time =" << atd << " ms" << std::endl;

			f << std::endl;
		}
		f.close();

		////매칭 결과 측정
		/*std::vector<double> m0, m1, m2, m3, m4, m5;
		for (int i = 0, iend = d1.size() - 10; i < iend; i++) {
			int i2 = i + 10;
			int n0 = 0;
			int n1 = 0;
			int n2 = 0;
			int n3 = 0;
			int n4 = 0;
			int n5 = 0;

			{
				std::vector<std::vector<cv::DMatch>> matches;
				matcher->knnMatch(desc_ori[i], desc_ori[i2], matches, 2);
				for (int j = 0, jend = matches.size(); j < jend; j++) {
					if (matches[j][0].distance < matches[j][1].distance*0.8) {
						n0++;
					}
				}
			}
			{
				std::vector<std::vector<cv::DMatch>> matches;
				matcher->knnMatch(desc_ori[i], desc_j1[i2], matches, 2);
				for (int j = 0, jend = matches.size(); j < jend; j++) {
					if (matches[j][0].distance < matches[j][1].distance*0.8) {
						n1++;
					}
				}
			}
			{
				std::vector<std::vector<cv::DMatch>> matches;
				matcher->knnMatch(desc_ori[i], desc_j2[i2], matches, 2);
				for (int j = 0, jend = matches.size(); j < jend; j++) {
					if (matches[j][0].distance < matches[j][1].distance*0.8) {
						n2++;
					}
				}
			}
			{
				std::vector<std::vector<cv::DMatch>> matches;
				matcher->knnMatch(desc_ori[i], desc_j3[i2], matches, 2);
				for (int j = 0, jend = matches.size(); j < jend; j++) {
					if (matches[j][0].distance < matches[j][1].distance*0.8) {
						n3++;
					}
				}
			}
			{
				std::vector<std::vector<cv::DMatch>> matches;
				matcher->knnMatch(desc_ori[i], desc_j4[i2], matches, 2);
				for (int j = 0, jend = matches.size(); j < jend; j++) {
					if (matches[j][0].distance < matches[j][1].distance*0.8) {
						n4++;
					}
				}
			}
			{
				std::vector<std::vector<cv::DMatch>> matches;
				matcher->knnMatch(desc_ori[i], desc_p1[i2], matches, 2);
				for (int j = 0, jend = matches.size(); j < jend; j++) {
					if (matches[j][0].distance < matches[j][1].distance*0.8) {
						n5++;
					}
				}
			}
			m0.push_back(n0);
			m1.push_back(n1);
			m2.push_back(n2);
			m3.push_back(n3);
			m4.push_back(n4);
			m5.push_back(n5);
		}*/
		////매칭 결과 측정
	}
	

	return 0;
}