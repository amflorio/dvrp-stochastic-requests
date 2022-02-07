#include "Decision.h"

using namespace std;

ostream& operator<<(ostream& os, const Decision& d) {
    os<<"["<<d.accept<<","<<d.assign<<"]";
    return os;
}

