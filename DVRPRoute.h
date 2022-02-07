#ifndef DVRPROUTE_H
#define DVRPROUTE_H

#include <vector>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>

class DVRPRoute {
    friend std::ostream& operator<<(std::ostream& os, const DVRPRoute& r);
    friend class boost::serialization::access;
    private:
        template<class Archive> void serialize(Archive& ar, 
                const unsigned int version) {
            ar & cost;
            ar & nodes;
            ar & redcost;
        }
        DVRPRoute() {}
    public:
	double cost;
	std::vector<int> nodes;
	double redcost;
	DVRPRoute(double c, std::vector<int> nds, double rc) : cost{c},
		nodes{std::move(nds)}, redcost{rc} {}
	bool elementary() const;
	int visits(int i) const;
};

std::ostream& operator<<(std::ostream& os, const DVRPRoute& r);

#endif

