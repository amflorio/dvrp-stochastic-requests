#ifndef REQUEST_H
#define REQUEST_H

#include <iostream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>

class Request {
    friend class boost::serialization::access;
    private:
        template<class Archive> void serialize(Archive& ar, 
                const unsigned int version) {
            ar & u;
            ar & i;
            ar & d;
        }
        Request() : u{0}, i{0}, d{0} {}
    public:
        double u;               // arrival time
        int i;                  // node
        double d;               // duration
        Request(double u_, int i_, double d_) : u{u_}, i{i_}, d{d_} {}
};

std::ostream& operator<<(std::ostream& os, const Request& r);

#endif

