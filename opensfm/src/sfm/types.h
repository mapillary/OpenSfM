#pragma once

#include <opencv2/features2d/features2d.hpp>

using TrackId = std::string;
using ShotId = std::string;

struct HashPair { 
    template <class T1, class T2> 
    size_t operator()(const std::pair<T1, T2>& p) const
    { 
        auto hash1 = std::hash<T1>{}(p.first); 
        auto hash2 = std::hash<T2>{}(p.second); 
        return hash1 ^ hash2; 
    } 
}; 