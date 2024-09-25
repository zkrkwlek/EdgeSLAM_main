 
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
bool bSave = false;
int nCamType = 0;// 0 : mono, 1 : stereo, 2 : rgbd
int nIMUType = 0;// 0 : NONE, 1 : gyro, 2 : gyro + acc
int nFeature = 1000;
std::string keydataset = "";
std::string scene_id = "";
std::string src_cam = "";


bool bPlay = false; //종료 없이 재생
double ts_last;
cv::Mat img_last;
cv::Mat depth_last;

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

//시각화 윈도우
std::string strWindowName = "CLIENT::Display";
cv::Mat MatDisplay;
cv::Mat MatImage;
std::mutex MutexDisplay;

int nLastID = -1;
int nSendID = -1;
bool bDone = true;
int nReID;
int nVoID = 1;
int nVOID = 1;
cv::Mat MatVO = cv::Mat::ones(4, 1, CV_32FC1)*-10000;
std::mutex MutexVO;

//카메라 자세
cv::Mat Pose = cv::Mat::eye( 4, 4, CV_32FC1);
cv::Mat Kinv;
cv::Mat plane = cv::Mat::zeros(4, 1, CV_32FC1);
//평면 정보

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	//float* tempData = (float*)userdata;
	WebAPI* API = (WebAPI*)userdata;
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		//가상 객체 생성
		cv::Mat Tinv = pCameraPose->GetInversePose();
		cv::Mat O = pCameraPose->GetCenter();
		cv::Mat normal = plane.rowRange(0, 3).clone();
		cv::Mat x3D = (cv::Mat_<float>(3, 1) << x, y, 1.0);
		cv::Mat Xw = Kinv * x3D;
		Xw.push_back(cv::Mat::ones(1, 1, CV_32FC1)); //3x1->4x1
		Xw = Tinv * Xw; // 4x4 x 4 x 1
		Xw = Xw.rowRange(0, 3) / Xw.at<float>(3); // 4x1 -> 3x1
		cv::Mat dir = Xw - O; //3x1
		float dist = plane.at<float>(3);
		float a = -normal.dot(dir);
		if (std::abs(a) < 0.000001)
			return;
		float d = (normal.dot(O) + dist) / a;
		if (d < 0.0)
			return;
		cv::Mat Xp = O + dir * d;
		Xp.at<float>(1) *= -1.0;
		//메세지 전송
		
		cv::Mat vo = cv::Mat::zeros(nVODataSize, 1, CV_32FC1);
		vo.at<float>(0) = 17;
		Xp.copyTo(vo.rowRange(3, 6));
		std::chrono::high_resolution_clock::time_point t_up_start = std::chrono::high_resolution_clock::now();
		std::stringstream ss;
		ss << "/Upload?keyword=" << "VO.CREATE" << "&id=" << nVOID++ << "&src=" << src << "&ts=" << t_up_start.time_since_epoch().count();// << " & type2 = " << user->userName;
		API->Send(ss.str(), vo.data, vo.rows * sizeof(float));

		////가상 객체 등록
		//tempData[2] = (float)x - tempData[0];
		//tempData[3] = (float)y;

		//////button interface
		//if (tempData[2] < 50.0 && y < 50) {
		//	bSaveMap = !bSaveMap;
		//}
		//else if (tempData[2] < 50 && (y >= 50 && y < 100)) {
		//	//bShowOnlyTrajectory = !bShowOnlyTrajectory;
		//	bLoadMap = !bLoadMap;
		//}
		//////button interface
	}
}

void Visualize() {

	//윈도우 등록
	MatDisplay = cv::Mat::zeros(h, w, CV_8UC3);
	std::stringstream ss;
	ss << strWindowName << "::" << src;
	strWindowName = ss.str();
	cv::imshow(strWindowName, MatDisplay);
	//cv::moveWindow(strWindowName, mnDisplayX, mnDisplayY);
	//평면과 카메라 자세 등이 가야 함.
	/*mapControlData[8] = -90.0;
	mapControlData[0] = mnWidth;
	mapControlData[1] = mnVisScale;*/
	WebAPI API("asdf", 123);
	cv::setMouseCallback(strWindowName, CallBackFunc, (void*)(&API));

	cv::Mat K = pCamera->K.clone();
	while (bDone) {
		//cv::imshow(MatDisplay)

		cv::Mat T = pCameraPose->GetPose();
		cv::Mat X;
		cv::Mat vis;
		{
			std::unique_lock<std::mutex> lock(MutexVO);
			X = MatVO.clone();
		}
		{
			std::unique_lock<std::mutex> lock(MutexDisplay);
			vis = MatImage.clone();
		}
		cv::Mat temp = T * X;
		temp = temp.rowRange(0, 3);
		temp = K * temp;
		float d = temp.at<float>(2);
		cv::Point2f pt(temp.at<float>(0) / d, temp.at<float>(1) / d);
		cv::circle(vis, pt, 5, cv::Scalar(255, 0, 0), -1);
		{
			std::unique_lock<std::mutex> lock(MutexDisplay);
			MatDisplay = vis.clone();
		}
		cv::imshow(strWindowName, MatDisplay);
		cv::waitKey(20);
	}
}

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

void UpdateLocalPlane(float* data) {
	int nSize = (int)data[0];
	if (nSize == 2) {
		return;
	}
	int N = (int)data[2];
	int idx = 3;
	for (int i = 0; i < N; i++) {
		int id = (int)data[idx];
		float nx = data[idx + 1];
		float ny = data[idx + 2];
		float nz = data[idx + 3];
		float d  = data[idx + 4];
		idx += 5;
		plane.at<float>(0) = nx;
		plane.at<float>(1) = ny;
		plane.at<float>(2) = nz;
		plane.at<float>(3) = d;
		break;
	}
}

void UpdateLocalContent(float* data) {
	int Nconnect = (int)data[2];
	int Ncontent = (int)data[3 + Nconnect];
	int cidx = 4 + Nconnect; //vo idx

	/*{
		std::unique_lock<std::mutex> lock(MutexVO);
		MatVO.at<float>(0) = -10000;
		MatVO.at<float>(1) = -10000;
		MatVO.at<float>(2) = -10000;
	}*/

	for (int j = 0; j < Ncontent; j++)
	{
		int len = (int)data[cidx];
		cv::Mat X = cv::Mat::ones(4, 1, CV_32FC1);
		X.at<float>(0) = data[cidx + 3];
		X.at<float>(1) = -data[cidx + 4];
		X.at<float>(2) = data[cidx + 5];
		cidx += len;
		
		{
			std::unique_lock<std::mutex> lock(MutexVO);
			MatVO = X.clone();
		}
	}
	
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
			UpdateLocalContent(totalData + nextid);
			//mContentManager.UpdateVirtualFrame(id, ref totalData, nextid, receivedTime, mExParam.bEdgeBase);
		}
		else if (pid == 4)
		{
			UpdateLocalPlane(totalData + nextid);
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
		//P.copyTo(Pose.rowRange(0, 4).colRange(0, 3));
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

void SendVirtualObject(WebAPI* api, std::string src) {
	
	int nSendID = nVoID;
	cv::Mat vo = cv::Mat::zeros(nVODataSize, 1, CV_32FC1);
	vo.at<float>(0) = 17;
	std::chrono::high_resolution_clock::time_point t_up_start = std::chrono::high_resolution_clock::now();
	std::stringstream ss;
	ss << "/Upload?keyword=" << "VO.CREATE" << "&id=" << nSendID << "&src=" << src<<"&ts="<<t_up_start.time_since_epoch().count();// << " & type2 = " << user->userName;
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
	ss << "/Upload?keyword="<< keyword <<"&id=" << fid << "&src=" << src<<"&ts="<<t_start.time_since_epoch().count()<<"&ts2="<< std::setprecision(16) <<timestamp;
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
				ss << "/Download?keyword=" << keyword << "&id=" << id << "&id2=" << id2 << "&src=" << src;
			}
			else
				ss << "/Download?keyword=" << keyword << "&id=" << id << "&src=" << src;

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
		src_cam += ".intrinsic";
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
	for (;index < argc;) {
		parsing(argv, index);
	}
}

int main(int argc, char* argv[]) {
	
	//평면 테스트
	if(false)
	{
		long long total1 = 0.0;
		long long total2 = 0.0;
		long long total3 = 0.0;
		for (int aa = 0; aa < 100; aa++) {
			cv::Mat R = cv::Mat::eye(3, 3, CV_32FC1);
			cv::Mat t = cv::Mat::ones(3, 1, CV_32FC1);
			cv::Mat K = cv::Mat::eye(3, 3, CV_32FC1);
			K.at<float>(0, 0) = 480;
			K.at<float>(1, 1) = 480;
			K.at<float>(0, 2) = 320;
			K.at<float>(1, 2) = 240;
			cv::Mat Kinv = K.inv();
			cv::Mat P = cv::Mat::eye(3, 4, CV_32FC1);
			P.at<float>(0, 3) = 2;
			P.at<float>(1, 3) = 3;
			P.at<float>(2, 3) = 4;

			cv::Mat O = cv::Mat::ones(3, 1, CV_32FC1);

			cv::Mat n = cv::Mat::zeros(3, 1, CV_32FC1);
			n.at<float>(1) = -1;
			float p = 3.0;

			int inc = 1;

			cv::Mat A = R * Kinv;
			cv::Mat T = cv::Mat::eye(4, 4, CV_32FC1);
			cv::Mat Kinv2 = cv::Mat::eye(4, 4, CV_32FC1);
			Kinv2.at<float>(3, 3) = 0.0;
			A = T * Kinv2;

			int idx = 0;
			cv::Mat Temp = cv::Mat::ones(4, 640 * 480, CV_32FC1);
			std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();
			for (int x = 0; x < 640; x += inc) {
				for (int y = 0; y < 480; y += inc) {
					cv::Mat X = (cv::Mat_<float>(4, 1) << x, y, 1, 1);
					X.copyTo(Temp.col(idx));
					idx++;
				}
			}
			cv::Mat B = A * Temp;
			std::chrono::high_resolution_clock::time_point t6 = std::chrono::high_resolution_clock::now();


			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			idx = 0;
			for (int x = 0; x < 640; x += inc) {
				for (int y = 0; y < 480; y += inc) {
					//cv::Mat X = (cv::Mat_<float>(4, 1) << x, y, 1, 1);
					//cv::Mat dir =  (A * X);
					//dir = dir.rowRange(0, 3);
					cv::Mat dir = B.col(idx++).rowRange(0, 3);
					for (int i = 0; i < 3; i++) {
						float a1 = n.dot(dir) + p;
						float a2 = n.dot(O);
						float depth = a1 / a2;
					}

					//std::cout <<"ㅁ=" << n.dot(dir) + p << std::endl;
					//std::cout <<"ㅠ=" << n.dot(O) << std::endl;
					//float depth = (n.dot(dir) + p) / (n.dot(0));
				}
			}

			std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

			std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
			idx = 0;
			for (int x = 0; x < 640; x += inc) {
				for (int y = 0; y < 480; y += inc) {
					/*cv::Mat X = (cv::Mat_<float>(4, 1) << x, y,1, 1);
					cv::Mat dir = (A * X);
					dir = dir.rowRange(0, 3);*/
					cv::Mat dir = B.col(idx++).rowRange(0, 3);
					for (int i = 0; i < 3; i++) {
						float depth = (O.at<float>(1) + p) / dir.at<float>(1);
					}
				}
			}
			std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();

			auto d1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
			auto d2 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
			auto d3 = std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count();
			std::cout <<aa<< " = test = " << d1 << " " << d2 << " " << d3 << std::endl;
			total1 += d1;
			total2 += d2;
			total3 += d3;
		}
		std::cout << " test = " << ((double)total1) / 100 << " " << ((double)total2) / 100 << " " << ((double)total3) / 100 << std::endl;
	}
	
	
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
	
	WebAPI::ip = ip;
	WebAPI::port = port;
	WebAPI API(ip, port);

	int fps = 30;
	double tframe = ((double)1.0) / fps;

	//데이터 폴더
	LoadParameter(paramPath);

	//슬램 실행
	pDetector = new EdgeDeviceSLAM::ORBDetector(nFeatures, fScaleFactor, nLevels);
	pCamera = new EdgeDeviceSLAM::Camera(w, h, fx, fy, cx, cy, d1, d2, d3, d4);
	Kinv = pCamera->Kinv.clone();

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
		src = argv[idxArgv++]; 
		//맵 이름
		std::string mapname = argv[idxArgv++];
		//압축 퀄리티
		int quality = atoi(argv[idxArgv++]);
		//스킵 프레임
		int nskip = atoi(argv[idxArgv++]);
		
		//여기서부터 파싱 시작
		parser(argc, argv, idxArgv);
		LoadDataset* dataset = (LoadDataset*)new TumDataset(dir, paramPath, rgbfile);

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

		//bf, th, depthfactor 항상 확인
		cv::Mat temp1 = cv::Mat::zeros(nInt * 4 + nByte, 1, CV_8UC1);
		cv::Mat temp2 = (cv::Mat_<float>(20, 1) << w, h, fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0, 0.0, 
			quality, nskip, 30, 
			40.0,40.0,5000.0,
			0,0,0);
		std::memcpy(temp1.data, temp2.data, nInt * 4);

		int nbFlagIdx = nInt * 4;
		//매핑
		temp1.at<uchar>(nbFlagIdx) = bMapping ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 1) = bTracking ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 4) = bPlaneGBA ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 5) = bSyncLocalMap ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 7) = bGridCommu ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 8) = bVOTest ? 1 : 0;
		temp1.at<uchar>(nbFlagIdx + 9) = bPLP?1:0; //PLP test
		std::string strtemp = src + ","+mapname;
		cv::Mat temp3(strtemp.length(), 1, CV_8UC1, (void*)strtemp.c_str());
		temp1.push_back(temp3);
		std::stringstream ss;
		ss << "/Upload?keyword=SimDeviceConnect&id=" << fid << "&src=" << src;// << "&type2=" << user->userName;
		auto res = API.Send(ss.str(), temp1.data, temp1.rows);

		//이미지 전송
		ThreadPool::ThreadPool* POOL = new ThreadPool::ThreadPool(3);
		POOL->EnqueueJob(ReceiveData, sock);
		POOL->EnqueueJob(Visualize);
		//처음 시작 시간 설정
		//time_initialization = std::chrono::high_resolution_clock::now();

		while (true) {
			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			double ts_image,ts_depth;
			cv::Mat img;
			cv::Mat depth;
			bool bimg = dataset->GrabImage(img, ts_image);

			if (bPlay && bimg) {
				ts_last = ts_image;
				img_last = img.clone();
			}
			if (bPlay && !bimg) {
				bimg = true;
				img = img_last.clone();
				ts_image = ts_last;
			}

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
				//cv::imshow(strWindowName, img);
				{
					std::unique_lock<std::mutex> lock(MutexDisplay);
					MatImage = img.clone();
				}
			}else{
				std::cout<< std::fixed << std::cout.precision(9) << " track = " << ts_image << std::endl;
			}

			if (bCaptureDepth) {
				//capture depth
				bool bdepth = dataset->GrabDepth(depth, ts_depth);
				if (bdepth && fid % nskip == 0) {
					//std::cout << "depth type = " << depth.type() << " " << CV_16SC1 << " " << CV_16UC1 <<" "<<CV_16SC2 <<" "<<CV_16UC2 << std::endl;
					//send
					POOL->EnqueueJob(SendImageData, &API, depth, src, "Depth", fid, ".png", depthParam, ts_depth);
					//cv::imshow("depth", depth);
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
			//cv::waitKey(1);
		}

		std::cout << "done " << std::endl;
		//device disconnect
		{
			std::stringstream ss;
			cv::Mat temp = cv::Mat::zeros(1000, 1, CV_32FC1);
			ss << "/Upload?keyword=SimDeviceDisconnect&id=" << fid << "&src=" << src;// << "&type2=" << user->userName;
			auto res = API.Send(ss.str(), temp.data, temp.rows);

			SaveLatency();
			SaveTrajectory();
			
		}
	}
	return 0;
}