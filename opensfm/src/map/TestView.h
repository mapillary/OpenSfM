// #include <pybind11/pybind11.h>
#include <vector>
#include <unordered_map>
namespace map
{
    


struct TestShot
{
    TestShot(std::string s):shot_id(s){}
    std::string shot_id;
};


class TestView
{
public:
    TestView()
    {
        for (auto i = 0; i < 10; ++i)
        {
            auto shot_id = "shot"+std::to_string(i);
            test_vector.push_back(shot_id);
            test_map.emplace(shot_id, TestShot(shot_id));
        }
    }
// private:
std::vector<std::string> test_vector;
std::unordered_map<std::string, TestShot> test_map;
};
} // namespace map