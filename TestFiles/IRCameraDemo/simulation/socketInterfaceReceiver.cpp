#include "socketInterfaceReceiver.h"

UDPSocket receiver = UDPSocket();
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

void receiverInit() {
  // Initialize sockets
  if(! (receiver.create())) {
		cout << "Error creating socket!" << endl;
		exit(-1);
	}
	if(! (receiver.bind(RECEIVE_PORT))) {
		cout << "Client: error connecting port: " << RECEIVE_PORT << endl;
		exit(-1);
	}
  //receiver.set_non_blocking(true);
  receiver.set_timeout(0);             // using recv()    
}

bool getDeviceState (Vector3d& start_proxy_pos, Matrix3d& start_proxy_rot) {
  if (receiver.recv(buf)) {
    while (receiver.recv(buf));
    
    parse(buf, vect1);

  	for(int i=0; i<3; i++)
   		start_proxy_pos(i) = boost::lexical_cast<double>(vect1[i].c_str());
  	
  	for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
          start_proxy_rot(j,i) = boost::lexical_cast<double>(vect1[3+i*3+j].c_str());
    
    return true;
  }
  return false;
}
