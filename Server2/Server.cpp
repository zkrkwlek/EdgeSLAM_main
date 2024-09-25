#include <sio_client.h>
#include <cstdio>
#include <iostream>

void OnMessage(sio::event&)
{
	std::cout << "???????" << std::endl;
}

int main(int argc, char* argv[])
{


	sio::client* pIO = new sio::client();
	
	pIO->connect("http://143.248.6.25:35005");
	pIO->socket()->on("my response", &OnMessage);

	int i = 0;
	while(true){
		/*if (i == 500){
			break;
		}
		std::cout << i << std::endl;
		i++;*/
	}
}