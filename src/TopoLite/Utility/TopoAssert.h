//
// Created by ziqwang on 2019-11-25.
//
#define TopoASSERT(condition) {if(!(condition)){ std::cerr << "ASSERT FAILED: " << #condition << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl; } }