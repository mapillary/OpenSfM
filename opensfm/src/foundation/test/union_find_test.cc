#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <foundation/union_find.h>

class UnionFindFixture : public ::testing::Test {
 public:
  UnionFindFixture() {
    for (int i = 0; i < count; ++i) {
      elements.emplace_back(std::unique_ptr<UnionFindElement<int>>(
        new UnionFindElement<int>(i)));
    }
  }

  static constexpr int count = 10;
  std::vector<std::unique_ptr<UnionFindElement<int>>> elements;
};

TEST_F(UnionFindFixture, IsCorrect) {
  // Union 0, 1, 2, 3, 4
  Union(elements[0].get(), elements[1].get());
  Union(elements[1].get(), elements[2].get());
  Union(elements[4].get(), elements[3].get());
  Union(elements[3].get(), elements[2].get());

  // Union of 5, 6
  Union(elements[5].get(), elements[6].get());

  // Union of 7

  // Union of 8, 9
  Union(elements[8].get(), elements[9].get());

  // Get clusters
  const auto clusters = GetUnionFindClusters(&elements);
  ASSERT_EQ(1, clusters[0].size());
  ASSERT_EQ(2, clusters[1].size());
  ASSERT_EQ(2, clusters[2].size());
  ASSERT_EQ(5, clusters[3].size());
  ASSERT_EQ(4, clusters.size());
}
