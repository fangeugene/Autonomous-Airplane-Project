#include <stdlib.h>
#include <iostream>
#include <cstring>
#include <string>
#include <cstdio>
#include "../utils/UDPSocket.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <Eigen/Core>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

#define RECEIVE_PORT 9000

using namespace std;

void parse(string buf, vector<string> &vect);

void receiverInit();

bool getDeviceState (Vector3d& start_proxy_pos, Matrix3d& start_proxy_rot);
