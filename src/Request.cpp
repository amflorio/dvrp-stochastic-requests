#include "Request.h"

using namespace std;

ostream& operator<<(ostream& os, const Request& r) {
    os<<"["<<r.u<<","<<r.i<<","<<r.d<<"]";
    return os;
}

