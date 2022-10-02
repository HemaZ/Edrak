#include "DBoW3.h"
#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/Images/Features.hpp"
#include <iostream>

void PrintUsage() {
  std::cout << "How to use: ./TrainDBow3 ImagesFolderPath/*.png OutputDBPath"
            << std::endl;
}

vector<cv::Mat> ExtractFeatures(const Edrak::IO::Reader &imgsReader) {
  vector<cv::Mat> features;
  Edrak::Images::Features::KeyPoints::KeyPoints kps;
  cv::Mat image;
  while (imgsReader.NextFrame(image)) {
    cv::Mat descriptors;
    Edrak::Images::Features::ORB(image, kps, descriptors);
    features.push_back(descriptors);
  }
  std::cout << "Processed " << features.size() << " Images" << std::endl;
  return features;
}

void CreateVocab(const vector<cv::Mat> &features, const std::string &filePath) {
  // branching factor and depth levels
  const int k = 9;
  const int L = 3;
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;
  DBoW3::Vocabulary voc(k, L, weight, score);
  voc.create(features);
  std::cout << "Vocabulary information: " << std::endl << voc << std::endl;
  // save the vocabulary to disk
  voc.save(filePath);
  std::cout << "Vocabulary saved to " << filePath << std::endl;
}

int main(int argc, char const *argv[]) {
  if (argc < 3) {
    PrintUsage();
    return -1;
  }
  Edrak::IO::Reader imgsReader(argv[1]);
  auto orbFeatures = ExtractFeatures(imgsReader);
  CreateVocab(orbFeatures, argv[2]);
  return 0;
}
