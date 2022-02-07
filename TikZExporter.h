#ifndef TIKZEXPORTER_H
#define TIKZEXPORTER_H

#include <fstream>
#include <string>
#include "Instance.h"
#include "OfflinePlan.h"
#include "State.h"

class TikZExporter {
    private:
        const bool BG_BLACK=true;
        const Instance& I;
        std::ofstream of;
        std::streambuf* buf;
        std::string prefix;
        std::string texPolicy;
        std::vector<std::tuple<double,State,bool,std::vector<Request>,int,
                double,double>> states;
        void baseNetwork();
        void exportState(double u, const State& s, bool dynreq,
                const std::vector<Request>& served, int accepted,
                double avgDecTime, double maxDecTime,
                const std::vector<Request>& past);
        std::pair<double, double> outerRectangle();
        std::pair<double, double> vehicleCoords(const PlannedRoute& r, double u)
                const;
    public:
        TikZExporter(const Instance& i);
        TikZExporter(const Instance& i, std::string prefix);
        void addState(double u, State s, bool dynreq,
                const std::vector<Request>& served, int accepted,
                double avgDecTime, double maxDecTime);
        void exportAll();
        void exportNetwork();
        void exportOfflinePlan(const OfflinePlan& plan);
        void setTexPolicy(std::string s) {texPolicy=s;}
        double timeLastState() const {return std::get<0>(states.back());}
};

#endif

