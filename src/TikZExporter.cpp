#include <iostream>
#include "TikZExporter.h"

using namespace std;

TikZExporter::TikZExporter(const Instance& i) : I{i}, buf{cout.rdbuf()} {}

TikZExporter::TikZExporter(const Instance& i,string pref) : I{i},prefix{pref} {}

void TikZExporter::addState(double u, State s, bool dynreq,
        const std::vector<Request>& served, int accepted, double avgDecTime,
        double maxDecTime) {
    assert(states.empty()||u>=get<0>(states.back()));
    assert(u==s.req.u||!dynreq);
    states.emplace_back(u, s, dynreq, served, accepted, avgDecTime, maxDecTime);
}

void TikZExporter::baseNetwork() {
    std::ostream os(buf);
    // draw every node
    for (int i=0; i<I.V; ++i) {
        /*
        os<<"\\filldraw [gray] ("<<I.xcoords[i]<<","<<I.ycoords[i]
            <<") circle (2pt) node[below] {"<<i<<"};"<<endl;
        */
        /*
        string color=I.type[i]==I.NC?"gray":I.type[i]==I.C||I.type[i]==I.C1
                ?"red":"green";
        os<<"\\filldraw ["<<color<<"] ("<<I.xcoords[i]<<","<<I.ycoords[i]
                <<") circle (1.4pt);"<<endl;
        */
    }
    // draw every link
    for (int i=0; i<I.A; ++i) {
        int s=I.links_[i].s;
        int t=I.links_[i].t;
        os<<"\\draw [Snow3] ("<<I.xcoords[s]<<","<<I.ycoords[s]<<") -- ("
                <<I.xcoords[t]<<","<<I.ycoords[t]<<");"<<endl;
    }
    // draw big circle for each cluster
    // first find scaling factor
    double scaling=I.distanceLL(I.lats[0],I.lons[0],I.lats[I.V-1],I.lons[I.V-1])
            /I.distanceXY(I.xcoords[0],I.ycoords[0],I.xcoords[I.V-1],
            I.ycoords[I.V-1]);
    for (const auto& cluster : I.clusters) {
        auto xy=I.getXYCoords(get<0>(cluster), get<1>(cluster));
        os<<"\\draw [draw=orange,fill=orange,fill opacity=0.2] ("<<xy.first<<","
                <<xy.second<<") circle ("<<get<2>(cluster)/scaling<<");"<<endl;
    }
    // identify the depot
    os<<"\\filldraw [blue] ("<<I.xcoords[I.dep]<<","<<I.ycoords[I.dep]
            <<") +(-14pt,-14pt) rectangle +(14pt,14pt);"<<endl;
    /*
    os<<"\\filldraw [blue] ("<<I.xcoords[I.dep]<<","<<I.ycoords[I.dep]
            <<") circle (14pt);"<<endl;
    */
    /* plot some random dynamic requests as well
    {
        auto reqs=I.sampleDynRequests(0);
        for (const auto& req : reqs) {
            os<<"\\draw [draw=cyan,fill=cyan] ("<<I.xcoords[req.i]<<","
                    <<I.ycoords[req.i]<<") circle (3pt);"<<endl;
        }
    }
    */
}

//, of(filename), buf{of.rdbuf()} {
    //assert(of.is_open());

void TikZExporter::exportAll() {
    cout<<"TikZExporter: exporting all states to .TeX files (prefix `"<<prefix
            <<"') ..."<<endl;
    // exporting base network
    string bnfile=prefix+"_"+I.network+"-network.tex";
    of=ofstream(bnfile);
    assert(of.good());
    buf=of.rdbuf();
    exportNetwork();
    vector<Request> past;
    for (int i=0; i<states.size(); ++i) {
        /*
        if (i==100)
            break;
        */
        const auto& st=states[i];
        if (get<2>(st)) {
            cout<<get<0>(st)<<": ["<<get<1>(st).vehics.size()<<";"
                <<get<1>(st).req.u<<","<<get<1>(st).req.i<<","<<get<1>(st).req.d
                <<"], "<<get<2>(st)<<", "<<get<3>(st).size()<<endl;
        } else {
            cout<<get<0>(st)<<": ["<<get<1>(st).vehics.size()<<";-,-,-], "
                    <<get<2>(st)<<", "<<get<3>(st).size()<<endl;
        }
        auto istr=to_string(i);
        string istr0s=string(5-istr.length(), '0')+istr;
        string filename=prefix+"_"+istr0s+(get<2>(st)?"_dyn":"")+".tex";
        of=ofstream(filename);
        assert(of.good());
        buf=of.rdbuf();
        exportState(get<0>(st), get<1>(st), get<2>(st), get<3>(st), get<4>(st),
                get<5>(st), get<6>(st), past);
        if (get<2>(st))
            past.push_back(get<1>(st).req);
    }
}

void TikZExporter::exportNetwork() {
    std::ostream os(buf);
    os<<setprecision(15);
    os<<"\\PassOptionsToPackage{usenames,x11names}{xcolor}"<<endl;
    os<<"\\documentclass[tikz]{standalone}"<<endl;
    os<<"\\tikzset{>=latex}"<<endl;
    os<<"\\begin{document}"<<endl;
    os<<"\\begin{tikzpicture}"<<endl;
    baseNetwork();
    outerRectangle();
    os<<"\\end{tikzpicture}"<<endl;
    os<<"\\end{document}"<<endl;
}

void TikZExporter::exportOfflinePlan(const OfflinePlan& plan) {
    std::ostream os(buf);
    os<<setprecision(15);
    os<<"\\PassOptionsToPackage{usenames,x11names}{xcolor}"<<endl;
    os<<"\\documentclass[tikz]{standalone}"<<endl;
    os<<"\\tikzset{>=latex}"<<endl;
    os<<"\\begin{document}"<<endl;
    os<<"\\begin{tikzpicture}"<<endl;
    auto posRect=outerRectangle();
    os<<"\\node[anchor=south west,inner sep=0] at ("<<posRect.first<<","
            <<posRect.second<<") {\\includegraphics{"<<I.network<<"-network.pdf"
            <<"}};"<<endl;
    os<<"\\begin{scope}[transparency group]"<<endl;
    os<<"\\begin{scope}[blend mode=multiply]"<<endl;
    const vector<string> colors {"teal", "blue", "violet", "lime", "magenta",
            "yellow", "orange", "olive", "darkgray", "cyan"};
    for (int k=0; k<plan.routes.size(); ++k) {
        for (int lidx=0; lidx<plan.routes[k].links.size(); ++lidx) {
            const auto& l=plan.routes[k].links[lidx];
            string fmt="dotted,line width=10,opacity=0.5,"+colors[k];
            if (lidx%7==0&&l.l.d>=150)
                fmt+=",->";
            os<<"\\draw ["<<fmt<<"] ("<<I.xcoords[l.l.s]<<","<<I.ycoords[l.l.s]
                <<") -- ("<<I.xcoords[l.l.t]<<","<<I.ycoords[l.l.t]<<");"<<endl;
        }
    }
    os<<"\\end{scope}"<<endl;
    os<<"\\end{scope}"<<endl;
    for (const auto& r : plan.routes)
        for (const auto& l : r.links)
            for (const auto& r : l.reqs)
                os<<"\\filldraw [red] ("<<I.xcoords[r.i]<<","<<I.ycoords[r.i]
                        <<") circle (9pt);"<<endl;
    os<<"\\end{tikzpicture}"<<endl;
    os<<"\\end{document}"<<endl;
}

void TikZExporter::exportState(double u, const State& s, bool dynreq,
        const vector<Request>& served, int accepted, double avgDecTime,
        double maxDecTime, const vector<Request>& past) {
    std::ostream os(buf);
    os<<setprecision(15);
    os<<"\\PassOptionsToPackage{usenames,x11names}{xcolor}"<<endl;
    os<<"\\documentclass[tikz]{standalone}"<<endl;
    os<<"\\tikzset{>=latex}"<<endl;
    os<<"\\begin{document}"<<endl;
    os<<"\\begin{tikzpicture}"<<endl;
    auto posRect=outerRectangle();
    os<<"\\node[anchor=south west,inner sep=0] at ("<<posRect.first<<","
            <<posRect.second<<") {\\includegraphics{"<<I.network<<"-network.pdf"
            <<"}};"<<endl;
    os<<setprecision(1);
    os<<fixed;
    os<<"\\node[red!75,scale=8,align=left,font=\\ttfamily] at (100,60) {Policy: "<<texPolicy<<"\\\\Remaining time: "<<(int)(I.period()-u+0.5)<<" min\\\\Requests accepted: "<<accepted<<"\\\\Max decision time: "<<maxDecTime<<" s\\\\Avg decision time: "<<avgDecTime<<" s};"<<endl;
    os<<setprecision(15);
    os<<defaultfloat;
    os<<"\\begin{scope}[transparency group]"<<endl;
    os<<"\\begin{scope}[blend mode=multiply]"<<endl;
    vector<string> colors;
    if (!BG_BLACK)
        colors={"blue", "lime", "darkgray", "orange", "cyan",
            "yellow", "magenta", "olive", "violet", "teal"};
    else
        colors={"yellow", "magenta", "white", "orange", "cyan",
            "blue", "pink", "olive", "violet", "teal"};
    for (int k=0; k<s.vehics.size(); ++k) {
        double t=s.vehics[k].start;
        for (const auto& l : s.vehics[k].links) {
            t+=l.l.d/I.speed();         // arrival time at endpoint
            if (t>=u) {
                string fmt="dotted,line width=10,opacity=0.5,"+colors[k];
                os<<"\\draw ["<<fmt<<"] ("<<I.xcoords[l.l.s]<<","
                        <<I.ycoords[l.l.s]<<") -- ("<<I.xcoords[l.l.t]<<","
                        <<I.ycoords[l.l.t]<<");"<<endl;
            }
            for (const auto& r : l.reqs)
                t+=r.d;
        }
    }
    os<<"\\end{scope}"<<endl;
    os<<"\\end{scope}"<<endl;
    // plot each vehicle
    for (int k=0; k<s.vehics.size(); ++k) {
        auto vcoords=vehicleCoords(s.vehics[k], u);
        os<<"\\filldraw [black] ("<<to_string(vcoords.first)<<","
                <<to_string(vcoords.second)
                <<") +(-16pt,-16pt) rectangle +(16pt,16pt);"<<endl;
        os<<"\\filldraw ["<<colors[k]<<"] ("<<to_string(vcoords.first)<<","
                <<to_string(vcoords.second)
                <<") +(-14pt,-14pt) rectangle +(14pt,14pt);"<<endl;
    }
    // plot all scheduled and served requests of currently active routes
    const string colplanned="red";
    const string colserved="green";
    for (int k=0; k<s.vehics.size(); ++k) {
        double t=s.vehics[k].start;
        for (const auto& l : s.vehics[k].links) {
            t+=l.l.d/I.speed();
            for (const auto& r : l.reqs)
                t+=r.d;
            string rcolor=t<u?colserved:colplanned;
            for (const auto& r : l.reqs) {
                os<<"\\filldraw ["<<rcolor<<"] ("<<I.xcoords[r.i]<<","
                        <<I.ycoords[r.i]<<") circle (9pt);"<<endl;
            }
        }
    }
    // plot all served requests of routes already finished
    for (const auto& r : served) {
        os<<"\\filldraw ["<<colserved<<"] ("<<I.xcoords[r.i]<<","
                <<I.ycoords[r.i]<<") circle (9pt);"<<endl;
    }
    // plot current request
    if (dynreq) {
        os<<"\\filldraw [red] ("<<I.xcoords[s.req.i]<<","
                <<I.ycoords[s.req.i]<<") circle (24pt);"<<endl;
        os<<"\\filldraw [white] ("<<I.xcoords[s.req.i]<<","
                <<I.ycoords[s.req.i]<<") circle (20pt);"<<endl;
        os<<"\\filldraw [red] ("<<I.xcoords[s.req.i]<<","
                <<I.ycoords[s.req.i]<<") circle (16pt);"<<endl;
        os<<"\\filldraw [white] ("<<I.xcoords[s.req.i]<<","
                <<I.ycoords[s.req.i]<<") circle (12pt);"<<endl;
        os<<"\\filldraw [red] ("<<I.xcoords[s.req.i]<<","
                <<I.ycoords[s.req.i]<<") circle (8pt);"<<endl;
        os<<"\\filldraw [white] ("<<I.xcoords[s.req.i]<<","
                <<I.ycoords[s.req.i]<<") circle (4pt);"<<endl;
    }
    for (const auto& req : past) {
        double width=max(0.0,(16-(u-req.u))/5);
        if (width>0.001) {
            os<<"\\draw [line width="<<width<<",red] ("<<I.xcoords[req.i]<<","
                    <<I.ycoords[req.i]<<") circle (24pt);"<<endl;
            os<<"\\draw [line width="<<width<<",white] ("<<I.xcoords[req.i]<<","
                    <<I.ycoords[req.i]<<") circle (20pt);"<<endl;
            os<<"\\draw [line width="<<width<<",red] ("<<I.xcoords[req.i]<<","
                    <<I.ycoords[req.i]<<") circle (16pt);"<<endl;
            os<<"\\draw [line width="<<width<<",white] ("<<I.xcoords[req.i]<<","
                    <<I.ycoords[req.i]<<") circle (12pt);"<<endl;
            os<<"\\draw [line width="<<width<<",red] ("<<I.xcoords[req.i]<<","
                    <<I.ycoords[req.i]<<") circle (8pt);"<<endl;
            os<<"\\draw [line width="<<width<<",white] ("<<I.xcoords[req.i]<<","
                    <<I.ycoords[req.i]<<") circle (4pt);"<<endl;
        }
    }
    os<<"\\end{tikzpicture}"<<endl;
    os<<"\\end{document}"<<endl;
}

pair<double, double> TikZExporter::outerRectangle() {
    std::ostream os(buf);
    double minX=numeric_limits<double>::max();
    double minY=numeric_limits<double>::max();
    double maxX=numeric_limits<double>::min();
    double maxY=numeric_limits<double>::min();
    for (int i=0; i<I.V; ++i) {
        minX=min(minX, I.xcoords[i]);
        minY=min(minY, I.ycoords[i]);
        maxX=max(maxX, I.xcoords[i]);
        maxY=max(maxY, I.ycoords[i]);
    }
    minX=(int)(minX-2.5);
    minY=(int)(minY-2.5);
    maxX=(int)(maxX+2.5);
    maxY=(int)(maxY+2.5);
    /*
    os<<"\\draw [line width=0mm] ("<<minX<<","<<minY<<") rectangle ("<<maxX<<","
            <<maxY<<");"<<endl;
    */
    // MANUAL outer rectangle for video generation
    os<<"\\draw [line width=0mm] (78,-45) rectangle (190,81);"<<endl;
    return {minX, minY};
}

pair<double, double> TikZExporter::vehicleCoords(const PlannedRoute& r,
        double u) const {
    if (r.links.size()==0)      // vehicle at the depot?
        return {I.xcoords[I.dep], I.ycoords[I.dep]};
    double t=r.start;
    assert(u>=t);
    for (const auto& l : r.links) {
        double newt=t+l.l.d/I.speed();
        if (newt>u) {
            double p=(u-t)/(newt-t);
            assert(p>=0&&p<=1);
            return {(1-p)*I.xcoords[l.l.s]+p*I.xcoords[l.l.t],
                    (1-p)*I.ycoords[l.l.s]+p*I.ycoords[l.l.t]};
        } else {
            for (const auto& r : l.reqs)
                newt+=r.d;
            if (newt>u)
                return {I.xcoords[l.l.t], I.ycoords[l.l.t]};
        }
        t=newt;
    }
    assert(false);
}

