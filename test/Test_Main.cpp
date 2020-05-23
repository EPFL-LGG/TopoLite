//
// Created by ziqwang on 2019-12-13.
//

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <limits>

TEST_CASE("numerical limit"){
    REQUIRE(std::numeric_limits<int>::max() == 0x7fffffff);
    REQUIRE(std::numeric_limits<long long>::max() == 0x7fffffffffffffff);
    REQUIRE(std::numeric_limits<double>::digits >= 53);
}