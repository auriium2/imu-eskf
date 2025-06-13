// watchdog.hpp
#pragma once

#include <chrono>
#include <cstdio>
#include <string>

class Watchdog {
public:
    Watchdog(const std::string& name, bool auto_print = true)
        : _name(name), _start(std::chrono::high_resolution_clock::now()), _auto(auto_print)
    {}

    void stop() {
        if (!_stopped) {
            auto end = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(end - _start).count();
            std::printf("[Watchdog] %s: %.3f ms\n", _name.c_str(), ms);
            _stopped = true;
        }
    }

    ~Watchdog() {
        if (_auto) stop();
    }

private:
    std::string _name;
    std::chrono::high_resolution_clock::time_point _start;
    bool _auto;
    bool _stopped = false;
};
