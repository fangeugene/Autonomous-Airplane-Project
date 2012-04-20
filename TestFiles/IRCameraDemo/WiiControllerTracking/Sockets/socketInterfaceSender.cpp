#include "socketInterfaceSender.h"

UDPSocket sender;
string buf;
char* outputString = "HEAD%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,TAIL";
vector<string> vectbuf, vect1, vect2;
int i,j;

void parse(string buf, vector<string> &vect) {
  int front, back;
  // get data segment
  front = buf.find("HEAD") + 4;
  back = buf.find("TAIL", front);
  string data = buf.substr(front, back-front-1);
  
  boost::split(vect, data, boost::is_any_of(","));
}

void senderInit() {
  // Initialize sockets
  if(! (sender.create())) {
		cout << "Error creating socket!" << endl;
		exit(-1);
	}
	if(! (sender.bind(SEND_PORT))) {
		cout << "Client: error connecting to ip: " << IP << "  port: " << SEND_PORT << endl;
		exit(-1);
	}
	if(! (sender.setDestination(IP, SEND_PORT))) {
		cout << "error setting destination" << endl;
		exit(-1);
	}
	//sender.set_non_blocking(true);
	sender.set_timeout(0);    
}

void sendDeviceState (double p0, double p1, double p2, double m0, double m1, double m2, double m3, double m4, double m5, double m6, double m7, double m8) {
	char buf[1024];
	memset(&buf, 0, sizeof(buf));
	sprintf((char*)&buf, outputString, p0, p1, p2, m0, m1, m2, m3, m4, m5, m6, m7, m8);
	cout << "sending: " << buf << endl;
	sender.send(string(buf));
}
