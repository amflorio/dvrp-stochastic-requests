#include <chrono>

class Stopwatch {
    using clock=std::chrono::high_resolution_clock;
    private:
        clock::time_point begin_time_{clock::now()},
            stop_time_{clock::time_point::min()};
        bool is_running() const {
            return stop_time_ == clock::time_point::min();
        }
        clock::time_point end_time() const {
            return is_running() ? clock::now() : stop_time_;
        }
    public:
        void stop() {
            if (is_running())
                stop_time_ = clock::now();
        }
        clock::duration elapsed() const {
            return end_time() - begin_time_;
        }
        double elapsedSeconds() const {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                    elapsed()).count()/1000.0;
        }
};

