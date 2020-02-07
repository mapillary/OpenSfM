#pragma once

template<class T = std::mt19937>
class RandomSamplesGenerator{
  public:
    using RAND_GEN = T;
    using DISTRIBUTION  = std::uniform_int_distribution<RAND_GEN::result_type>;

    RandomSamplesGenerator(int seed = 42): generator_(seed){}
    std::vector<int> Generate(int size, int range_max){
      return GenerateOneSample(size, range_max);
    }

  private:
    std::vector<int> GenerateOneSample(int size, int range_max){
      std::vector<int> indices(size);
      DISTRIBUTION distribution(0, range_max);
      for(int i = 0; i < size; ++i){
        do {
          indices[i] = distribution(generator_);
        }
        while(std::find(indices.begin(), indices.begin()+i, indices[i]) != (indices.begin()+i));
      }
      return indices;
    }

    RAND_GEN generator_;
};