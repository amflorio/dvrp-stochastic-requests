SYSTEM = x86-64_osx
LIBFORMAT = static_pic

CPLEXDIR = /Applications/CPLEX_Studio1210/cplex
CONCERTDIR = /Applications/CPLEX_Studio1210/concert

CCC = clang++ -O2 -Wall -std=c++11
CCOPT = -m64 -fPIC -fexceptions -DIL_STD -stdlib=libc++ -c

CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CCLNDIRS  = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR)
CCLNFLAGS = -lconcert -lilocplex -lcplex -m64 -lm -lpthread -framework CoreFoundation -framework IOKit -stdlib=libc++ -lboost_serialization

CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include

CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR)

SOURCES=Data.cpp Decision.cpp Dijkstra.cpp DVRPData.cpp DVRPLabel.cpp DVRPLinearSolver.cpp DVRPPricing.cpp DVRPRoute.cpp DVRPSolution.cpp DVRPSolver.cpp GreedyPolicy.cpp Instance.cpp MaxFlowSolver.cpp MinCutSolver.cpp mPGreedyPolicy.cpp MPotentialPlanner.cpp MyopicPlanner.cpp OfflinePlan.cpp OfflinePlanner.cpp OnlinePolicy.cpp Path.cpp PFAPolicy.cpp PGreedyPolicy.cpp PlannedRoute.cpp PolicySimulator.cpp PotentialPlanner.cpp Request.cpp RolloutPolicy.cpp RouteReoptimizer.cpp SimResults.cpp TikZExporter.cpp main.cpp
OBJECTS=$(SOURCES:.cpp=.o)
	EXECUTABLE=dvrpsr

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CCC) $(CCLNDIRS) $(CCLNFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CCC) $(CCFLAGS) $< -o $@

clean:
	/bin/rm -rf *.o

