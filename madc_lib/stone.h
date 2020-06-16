#ifndef STONE_H
#define STONE_H

struct idxXYZ;
 
#include <iostream>
#include <vector>
#include <sstream>

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>
#include <gsl/gsl_blas.h>


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

  gsl_matrix *characteristic_matrix_inverse;
  gsl_matrix *characteristic_matrix;


  
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
  std::string serializeTheStone();
};

#endif
