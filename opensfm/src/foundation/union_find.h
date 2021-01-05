#pragma once

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

template <class T>
struct UnionFindElement {
  T data;
  UnionFindElement<T>* parent;
  int rank;

  explicit UnionFindElement(const T& data)
      : data(data), parent(this), rank(0) {}
};

template <class T>
UnionFindElement<T>* Find(UnionFindElement<T>* e) {
  if (e->parent != e) {
    e->parent = Find(e->parent);
  }
  return e->parent;
}

template <class T>
void Union(UnionFindElement<T>* e1, UnionFindElement<T>* e2) {
  const auto root_e1 = Find(e1);
  const auto root_e2 = Find(e2);
  if (root_e1 != root_e2) {
    if (root_e1->rank < root_e2->rank) {
      root_e1->parent = root_e2;
    } else {
      root_e2->parent = root_e1;
      if (root_e1->rank == root_e2->rank) {
        ++(root_e1->rank);
      }
    }
  }
}

template <class T>
std::vector<std::vector<UnionFindElement<T>*>> GetUnionFindClusters(
    std::vector<std::unique_ptr<UnionFindElement<T>>>* elements) {
  std::unordered_map<UnionFindElement<T>*, std::vector<UnionFindElement<T>*>>
      aggregations;
  for (const auto& e : *elements) {
    aggregations[Find(e.get())].emplace_back(e.get());
  }

  std::vector<std::vector<UnionFindElement<T>*>> clusters;
  clusters.reserve(aggregations.size());
  for (const auto agg : aggregations) {
    clusters.emplace_back(agg.second);
  }

  // For deterministic ordering
  std::sort(clusters.begin(), clusters.end(),
            [](const std::vector<UnionFindElement<T>*>& v1,
               const std::vector<UnionFindElement<T>*>& v2) {
              return v1.size() < v2.size();
            });

  return clusters;
}
