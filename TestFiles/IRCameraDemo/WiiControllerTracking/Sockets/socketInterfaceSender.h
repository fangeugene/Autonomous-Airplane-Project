#include <stdlib.h>
#include <iostream>
#include <cstring>
#include <string>
#include <cstdio>
#include "../utils/UDPSocket.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>

#define IP "10.10.67.101"
#define SEND_PORT 9000

using namespace std;

void parse(string buf, vector<string> &vect);

void senderInit();

void sendDeviceState (double p0, double p1, double p2, double m0, double m1, double m2, double m3, double m4, double m5, double m6, double m7, double m8);
