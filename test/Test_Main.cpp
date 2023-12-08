//
// Created by ziqwang on 2019-12-13.
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_all.hpp>
#include <limits>

TEST_CASE("man"){
    REQUIRE(std::numeric_limits<int>::max() == 0x7fffffff);
    REQUIRE(std::numeric_limits<long long>::max() == 0x7fffffffffffffff);
    REQUIRE(std::numeric_limits<double>::digits >= 53);
}