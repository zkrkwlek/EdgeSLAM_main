#include <vector>
#include <sstream>
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <curl/curl.h>

using json = nlohmann::json;

size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

int main() {
    // Server information
    const char* server_ip = "143.248.6.143";
    const int server_port = 35001;

    // Create a UDP socket
    int client_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (client_socket == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    sockaddr_in server_address;
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(server_port);
    inet_pton(AF_INET, server_ip, &(server_address.sin_addr));

    //CURL
    CURL* curl;
    CURLcode res;
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, "http://143.248.6.143:35005/Connect");
    curl_easy_setopt(curl, CURLOPT_POST, 1);
    struct curl_slist *list = NULL;
    list = curl_slist_append(list, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, list);
    // POST 데이터 설정
    std::string post_data = R"({"src":"depth_receiver","keyword":"testmethod","type1":"server","type2":"NONE","capacity":30})";
    {
        //std::stringstream ss;
        //ss << "{\"src\":\""<<"depth_receiver"<<"\","<<"\"keyword\":\"" << "test_method" << "\",\"type1\":\"server\",\"type2\":\"" << "NONE" << "\""<<",\"capacity\":"<<300<<"}"; //test\"}";
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data.c_str());

    }
    
    res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    //udp
    std::vector<std::string> receivedKeywords;
    receivedKeywords.push_back("Depth");
    receivedKeywords.push_back("ObjectDetection");
    receivedKeywords.push_back("Segmentation");

    std::string src = "depth_receiver";
    for (int i = 0, iend = receivedKeywords.size(); i < iend; i++) {
        std::stringstream ss;
        ss << "{\"type1\":" << "\"connect\"" << ",\"type2\":" << "\"all\"" << ",\"src\":\"" << src << "\",\"keyword\":\"" << receivedKeywords[i] << "\"" << "}";
        std::string temp = ss.str();
        auto buf = temp.c_str();
        int retval;
        retval = sendto(client_socket, buf, strlen(buf), 0, (sockaddr*)&server_address, sizeof(server_address));
    }
        
    sockaddr_in clientaddr;
	socklen_t addrlen = sizeof(clientaddr);
	char rbuf[1024];
    while(true){
        addrlen = sizeof(clientaddr);
		ssize_t retval = recvfrom(client_socket, rbuf, 1024, 0, (sockaddr*)&clientaddr, &addrlen);
		if(retval == -1){
            std::cout<<"error"<<std::endl;
            continue;
        }
        rbuf[retval] = '\0';

        auto jsonData = json::parse(rbuf);
        std::string keyword = jsonData["keyword"];
        int id = jsonData["id"];
        std::string src = jsonData["src"];

        //CURL
        //CURL* curl;
        //CURLcode res;
        curl = curl_easy_init();

        std::string url = "http://143.248.6.143:35005/Load";
        std::stringstream ss;
        ss << "http://143.248.6.143:35005/Load?keyword=" << keyword << "&id=" << id << "&src=" << src;
        //ss << "keyword=" << keyword << "&id=" << id << "&src=" << src;

        curl_easy_setopt(curl, CURLOPT_URL,ss.str().c_str());
        //curl_easy_setopt(curl, CURLOPT_URL, ss.str());
        //curl_easy_setopt(curl, CURLOPT_POST, 1);
        //struct curl_slist *list = NULL;
        //list = curl_slist_append(list, "Content-Type: application/json");
        //curl_easy_setopt(curl, CURLOPT_HTTPHEADER, list);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "");

        std::string response_data;
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
         
        cv::Mat temp = cv::Mat(response_data.size(), 1, CV_8UC1, (void*)response_data.data());

        if(keyword == "Depth")
        {
            cv::Mat depth = cv::imdecode(temp, cv::IMREAD_ANYDEPTH);
            cv::imwrite("./depth.png",depth);
            cv::waitKey(1);
        }
        if(keywro)
        
        //std::cout<<keyword<<" "<<id<<" "<<src<<"="<<response_data.size()<<std::endl;
        //std::cout<<ss.str()<<std::endl;
    }
/*
    try {
        // Get user input for the message to send
        std::cout << "Enter message to send: ";
        std::string message;
        std::getline(std::cin, message);

        // Send the message to the server
        sendto(client_socket, message.c_str(), message.size(), 0,
               (struct sockaddr*)&server_address, sizeof(server_address));

        // Receive the response from the server
        char buffer[1024];
        sockaddr_in sender_address;
        socklen_t sender_address_len = sizeof(sender_address);
        ssize_t received_bytes = recvfrom(client_socket, buffer, sizeof(buffer), 0,
                                          (struct sockaddr*)&sender_address, &sender_address_len);
        if (received_bytes == -1) {
            throw std::runtime_error("Error receiving data");
        }

        // Print the response
        buffer[received_bytes] = '\0';
        std::cout << "Received response from " << inet_ntoa(sender_address.sin_addr) << ": "
                  << buffer << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
*/
    // Close the socket
    //close(client_socket);

    return 0;
}
