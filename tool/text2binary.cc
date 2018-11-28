//
// Created by ichigoi7e on 12/07/2018.
//

#include <time.h>
#include "ORBVocabulary.h"

using namespace std;

bool load_as_text(ORB_SLAM2::ORBVocabulary* voc, const std::string infile);
void save_as_binary(ORB_SLAM2::ORBVocabulary* voc, const std::string outfile);
void load_as_binary(ORB_SLAM2::ORBVocabulary* voc, const std::string infile);

int main(int argc, char **argv) {
  cout << "BoW load/save benchmark" << endl;
  ORB_SLAM2::ORBVocabulary* voc = new ORB_SLAM2::ORBVocabulary();
  load_as_text(voc, "Vocabulary/ORBvoc.txt");
  save_as_binary(voc, "Vocabulary/ORBvoc.bin");
  load_as_binary(voc, "Vocabulary/ORBvoc.bin");
  return 0;
}

bool load_as_text(ORB_SLAM2::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  bool res = voc->loadFromTextFile(infile);
  printf("Loading from text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  return res;
}

void save_as_binary(ORB_SLAM2::ORBVocabulary* voc, const std::string outfile) {
  voc->saveToBinaryFile(outfile);
}

void load_as_binary(ORB_SLAM2::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->loadFromBinaryFile(infile);
  printf("Loading from binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}