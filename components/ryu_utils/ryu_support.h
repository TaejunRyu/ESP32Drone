#include <chrono>

class PerfMonitor {
    using clock = std::chrono::high_resolution_clock;
    clock::time_point start_time;
public:
    // 측정 시작
    void start() noexcept {
        start_time = clock::now();
    }
    [[nodiscard]] int64_t stop() const noexcept {
        const auto end_time = clock::now(); // const로 명시
        const auto diff = end_time - start_time; // 연산을 분리하여 IntelliSense 추론 도움
        return std::chrono::duration_cast<std::chrono::microseconds>(diff).count();
    }
};
