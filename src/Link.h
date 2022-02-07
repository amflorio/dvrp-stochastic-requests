#ifndef LINK_H
#define LINK_H

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>

class Link {
    friend class boost::serialization::access;
    friend class PlannedLink;
    private:
        template<class Archive> void serialize(Archive& ar,
                const unsigned int version) {
            ar & idx;
            ar & s;
            ar & t;
            ar & d;
        }
        Link() {}
    public:
        int idx=-1;         // link index in the set of links
        int s=-1;           // index of the origin node
        int t=-1;           // index of the destination node
        double d=-1;        // link distance
        Link(int idx_, int s_, int t_, double d_)
                : idx{idx_}, s{s_}, t{t_}, d{d_} {}
};

#endif

