#pragma once

template <class T>
class RandomSamplesGenerator {
 public:
  using RAND_GEN = T;
  using DISTRIBUTION =
      std::uniform_int_distribution<typename RAND_GEN::result_type>;

  RandomSamplesGenerator(int seed = 42) : generator_(seed) {}

  template <class MODEL>
  std::vector<typename MODEL::Data> GetRandomSamples(
      const std::vector<typename MODEL::Data>& samples, int size) {
    const auto random_sample_indices =
        GenerateOneSample(size, samples.size() - 1);
    std::vector<typename MODEL::Data> random_samples;
    std::for_each(random_sample_indices.begin(), random_sample_indices.end(),
                  [&random_samples, &samples](const int idx) {
                    random_samples.push_back(samples[idx]);
                  });
    return random_samples;
  }

 private:
  std::vector<int> GenerateOneSample(int size, int range_max) {
    std::vector<int> indices(size);
    DISTRIBUTION distribution(0, range_max);
    for (int i = 0; i < size; ++i) {
      do {
        indices[i] = distribution(generator_);
      } while (std::find(indices.begin(), indices.begin() + i, indices[i]) !=
               (indices.begin() + i));
    }
    return indices;
  }

  RAND_GEN generator_;
};
