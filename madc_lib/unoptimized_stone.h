
#ifndef STONE_H
#define STONE_H

struct idxXYZ;
 
#include "Eigen/Dense"
#include "Eigen/Eigenvalues"
#include <iostream>
#include <vector>

#define epsilon 0.35

#define my_MIN(a,b) (((a)<(b))?(a):(b))
#define my_MAX(a,b) (((a)>(b))?(a):(b))



class Stone
{

 private:
  int id;
  Stone* parent;

  int n;
  double runningSum[3];
  double runningCov[3][3];

  double*  mu;
  double** cov;

  Stone* findRoot();

  
 public:
  
  Stone* next;
  int offset_signature;
  void resetParent();
  
  double x_lower_bound;
  double x_upper_bound;

  double z_lower_bound;
  double z_upper_bound;

  
  bool clean_mu_cov;
  Eigen::Matrix4d characteristicMatrix;
  Eigen::Matrix4d inverse_characteristicMatrix;

  bool processed;
  int getId();
  void print();
  void setParent(Stone*);
  void addMember( idxXYZ & tuple);
  Stone(int);
  double*  getMu();
  double** getSigma();

  Stone* findRoot(int& depth);

  static void linkTwoStones(Stone* s1, Stone* s2, int& sumOfDepths);
  static bool areTwoStonesLinked(Stone* s1, Stone* s2, int& sumOfDepths);
  static bool areTwoStonesLinked(Stone* s1, Stone* s2, std::vector<int> *);
  static bool compareTwoStonesV2(Stone& s1, Stone& s2);
  static bool compareTwoObjectRepresentatives(Stone* s1, Stone* s2);

  
  void* getCharacteristicMatrix();
  
};

#endif
