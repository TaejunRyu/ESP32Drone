#pragma once

#include "ryu_common_std.h"
#include "ryu_paramtable_ryu.h"

/**
 * [C++23] 해시 함수: constexpr을 붙여 컴파일 타임에도 사용 가능하게 함
 */
[[nodiscard]] constexpr uint32_t hash_string(std::string_view str) noexcept {
    uint32_t hash = 5381;
    for (char c : str) {
        // hash * 33 + c 최적화 연산
        hash = ((hash << 5) + hash) + static_cast<uint32_t>(c);
    }
    return hash;
}

// 해시 테이블 크기 정의 (C++23 스타일)
constexpr size_t HASH_SIZE = 250;
inline std::array<int16_t, HASH_SIZE> hash_table; // 인덱스 저장 (-1은 빈 칸)

/**
 * 해시맵 초기화 (부팅 시 1회 실행)
 */
void init_param_hash() {
    hash_table.fill(-1); // std::array 초기화
    
    for (int i = 0; i < static_cast<int>(ParamTable::praram_count); ++i) {
        // std::string_view 덕분에 .data() 없이 직접 hash_string에 전달 가능
        uint32_t h = hash_string(ParamTable::drone_params[i].name) % HASH_SIZE;
        
        while (hash_table[h] != -1) [[unlikely]] { // 충돌 처리
            h = (h + 1) % HASH_SIZE;
        }
        hash_table[h] = static_cast<int16_t>(i);
    }
}

/**
 * 이름으로 파라미터 인덱스 찾기 (QGC 요청 처리용)
 * [[nodiscard]]: 결과값을 반드시 확인하도록 강제
 */
[[nodiscard]] int16_t find_param_index(std::string_view id) noexcept {
    if (id.empty()) return -1;

    uint32_t h = hash_string(id) % HASH_SIZE;
    const uint32_t start_h = h;
    
    while (hash_table[h] != -1) {
        const int16_t idx = hash_table[h];
        // std::string_view의 == 연산자는 strncmp보다 안전하고 최적화되어 있음
        if (ParamTable::drone_params[idx].name == id) [[likely]] {
            return idx; // 찾음!
        }
        
        h = (h + 1) % HASH_SIZE;
        if (h == start_h) [[unlikely]] break; // 한 바퀴 다 돎
    }
    return -1; // 없음
}
