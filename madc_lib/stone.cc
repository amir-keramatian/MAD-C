#include "stone.h"
#include "idxXYZ.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <limits.h>

using namespace std;

Stone::Stone(int gid) : id(gid), n(0) {

  parent = this;
  next   = this;
  processed = false;

  x_lower_bound = INT_MAX;
  x_upper_bound = -INT_MAX;

  z_lower_bound = INT_MAX;
  z_upper_bound = -INT_MAX;

  mu  = new double[3];
  cov = new double*[3];
  clean_mu_cov = false;
  
  for (int i = 0; i < 3; ++i)
    {
      runningSum[i] = 0;

      cov[i] = new double[3];
      
      for (int j = 0; j < 3; ++j)
	{
	  runningCov[i][j] = 0;
	}
    }
  
}

int Stone::getId(){
  return id;
}

void Stone::setParent(Stone* gParent){
  parent = gParent;
}

Stone* Stone::findRoot(){
  if (this == parent)
    return this;
  
  //path shortcuting:
  this->parent = parent->findRoot();

  return this->parent;
}


void Stone::addMember(struct idxXYZ & tuple)
{
  clean_mu_cov = false;
  
  if (tuple.clusterIDX != id || tuple.clusterIDX == 0){
    printf("Error in initializing the stones...\n");

    exit(EXIT_FAILURE);
  }

  if (tuple.x < x_lower_bound)
    x_lower_bound = tuple.x;
  if (tuple.x > x_upper_bound)
    x_upper_bound = tuple.x;


  if (tuple.z < z_lower_bound)
    z_lower_bound = tuple.z;
  if (tuple.z > z_upper_bound)
    z_upper_bound = tuple.z;
  
  
  n++;

  double dataPoint[3] = {tuple.x, tuple.y, tuple.z};
  for (int i = 0; i < 3; ++i)
    {
      runningSum[i] += dataPoint[i];

      for (int j = 0; j < 3; ++j)
	{
	  runningCov[i][j] += dataPoint[i] * dataPoint[j];
	}
    }

}

void Stone::print(){

  double*  mou    = getMu();
  double** sigma = getSigma();

  std::cout << "cluster " << id << endl;
  std::cout << "mu:" << endl;

  
  for (int i = 0; i < 3; ++i)
    {
      std::cout << mou[i] << " ";
    }

  std::cout << endl;
  std::cout << "sigma:" << endl;
  
  for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
	{
	  std::cout << sigma[i][j] << "\t";
	}
      cout << endl;
    }
  cout << endl << endl;
}

double* Stone::getMu(){

  if (clean_mu_cov == true)
    return mu;
  
  if ( n == 0){
    printf("division by zero in getMu()\n");
    exit(EXIT_FAILURE);
  }

  mu[0] = runningSum[0]/n;
  mu[1] = runningSum[1]/n;
  mu[2] = runningSum[2]/n;

  //  clean_mu_cov = true;
  return mu;
}

double** Stone::getSigma(){

  if(clean_mu_cov == true)
    return cov;

  getMu();
  
  if ( n == 0){
    printf("division by zero in getSigma()\n");
    exit(EXIT_FAILURE);
  }
  
  for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
	{
	  cov[i][j] = 1.6*1.6*(runningCov[i][j]/n - mu[i]*mu[j]); //TODO: 1.05 -> 1.5
	}
      cov[i][i] += 0.01;
    }


  gsl_matrix* covariance_matrix = gsl_matrix_alloc(3,3);
  gsl_matrix* lambda_matrix = gsl_matrix_alloc(3,3);
  gsl_matrix* lambda_inverse_matrix = gsl_matrix_alloc(3,3);
  gsl_matrix* product_matrix = gsl_matrix_alloc(3,3);
  gsl_matrix* product_intermediate_matrix = gsl_matrix_alloc(3,3);

  gsl_matrix* T_matrix = gsl_matrix_alloc(4,4);
  gsl_matrix* Sigma_inverse_four_by_four = gsl_matrix_alloc(4,4);
  gsl_matrix* characteristic_intermediate = gsl_matrix_alloc(4,4);
  characteristic_matrix = gsl_matrix_alloc(4,4);
  
  

  for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
	{
	  gsl_matrix_set(T_matrix, i, j, 0);
	  gsl_matrix_set(Sigma_inverse_four_by_four, i, j, 0);
	  gsl_matrix_set(characteristic_intermediate, i, j, 0);
	  gsl_matrix_set(characteristic_matrix, i, j, 0);
	}
      gsl_matrix_set(T_matrix, i, i, 1);
    }
  
  gsl_matrix_set(T_matrix, 3, 0, -mu[0]);
  gsl_matrix_set(T_matrix, 3, 1, -mu[1]);
  gsl_matrix_set(T_matrix, 3, 2, -mu[2]);
  //  gsl_matrix_set(T_matrix, 3, 3, 1);

  for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
	{
	  gsl_matrix_set(covariance_matrix, i, j, cov[i][j]);
	  gsl_matrix_set(lambda_matrix, i, j, 0);
	  gsl_matrix_set(lambda_inverse_matrix, i, j, 0);
	  gsl_matrix_set(product_matrix, i, j, 0);
	  gsl_matrix_set(product_intermediate_matrix, i, j, 0);
	}
    }
  

  gsl_vector *eval = gsl_vector_alloc(3);
  gsl_matrix *evec = gsl_matrix_alloc(3,3);

  gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc(3);

  gsl_eigen_symmv(covariance_matrix, eval, evec, w);

  gsl_eigen_symmv_free(w);

  {
    int i;

    for (i = 0; i < 3; i++)
      {
	double eval_i
	  = gsl_vector_get (eval, i);
	gsl_vector_view evec_i
	  = gsl_matrix_column (evec, i);


	/********************aura********************/
	
	eval_i = (sqrt(eval_i) + epsilon/2)*(sqrt(eval_i) + epsilon/2);
	
	gsl_matrix_set(lambda_matrix, i, i, eval_i);
	gsl_matrix_set(lambda_inverse_matrix, i, i, (1/eval_i));

      }
  }

  /*
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
    1.0, evec, lambda_matrix,
    0.0, product_intermediate_matrix);
    gsl_blas_dgemm(CblasNoTrans, CblasTrans,
    1.0, product_intermediate_matrix, evec,
    0.0, product_matrix);*/

  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
		 1.0, evec, lambda_inverse_matrix,
		 0.0, product_intermediate_matrix);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans,
  		 1.0, product_intermediate_matrix, evec,
  		 0.0, product_matrix);

  /*product_matrix contains sigma inverse */

  for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
	{
	  gsl_matrix_set(Sigma_inverse_four_by_four, i, j,
			 gsl_matrix_get(product_matrix, i, j)
			 );
	}
    }
  gsl_matrix_set(Sigma_inverse_four_by_four, 3, 3, -1);


  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
		 1.0, T_matrix, Sigma_inverse_four_by_four,
		 0.0, characteristic_intermediate);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans,
  		 1.0, characteristic_intermediate, T_matrix,
  		 0.0, characteristic_matrix); /* hoora */
  
  gsl_vector_free (eval);
  gsl_matrix_free (evec);


  gsl_permutation *p = gsl_permutation_alloc(4);
  int s;
  gsl_linalg_LU_decomp(characteristic_matrix, p, &s);

  characteristic_matrix_inverse = gsl_matrix_alloc(4, 4);
  gsl_linalg_LU_invert(characteristic_matrix, p, characteristic_matrix_inverse);

  


  /*
    for (int i = 0; i < 3; ++i)
    {
    for (int j = 0; j < 3; ++j)
    {
    cov[i][j] = gsl_matrix_get(product_matrix, i, j);
    }
    }
  */

  
  //  clean_mu_cov = true;
  return cov;
}

bool Stone::compareTwoObjectRepresentatives(Stone* s1, Stone* s2){
  
  if ( (s1->x_upper_bound < s2->x_lower_bound) ||
       (s2->x_upper_bound < s1->x_lower_bound) ){
    return false;
  }
    
  if ( (s1->z_upper_bound < s2->z_lower_bound) ||
       (s2->z_upper_bound < s1->z_lower_bound) ){
    return false;
  }
  
  Stone* thisEllipsoid = s1;

  do{
    Stone* thatEllipsoid = s2;
    thisEllipsoid = thisEllipsoid->next;
    do{
      thatEllipsoid = thatEllipsoid->next;
      if ((compareTwoStonesV2(*thisEllipsoid, *thatEllipsoid)==true))
	return true;
    } while(thatEllipsoid != s2);

  } while(thisEllipsoid != s1);

  return false;
}


void* Stone::getCharacteristicMatrix(){

  if (clean_mu_cov == true){
    //std::cout << "avoided!" << std::endl;
    return (void*) characteristic_matrix;
  }

  x_lower_bound -= epsilon/2;
  x_upper_bound += epsilon/2;

  z_lower_bound -= epsilon/2;
  z_upper_bound += epsilon/2;
  
  
  getMu();
  getSigma();
        
  clean_mu_cov = true;
  
  return (void*) characteristic_matrix;
}



bool Stone::compareTwoStonesV2(Stone& s1, Stone& s2){
  
  gsl_matrix* A = ((gsl_matrix*)s1.getCharacteristicMatrix());
  gsl_matrix* B = ((gsl_matrix*)s2.getCharacteristicMatrix());  

  if (s1.offset_signature == s2.offset_signature){
    return false;
  }
  if ( (s1.x_upper_bound < s2.x_lower_bound) ||
       (s2.x_upper_bound < s1.x_lower_bound) ){
    return false;
  }
  if ( (s1.z_upper_bound < s2.z_lower_bound) ||
       (s2.z_upper_bound < s1.z_lower_bound) ){
    return false;
  }
  
  bool overlapOrTouch = false;

  gsl_matrix* decisionMatrix = gsl_matrix_alloc(4,4);

  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
		 1.0, s1.characteristic_matrix_inverse, s2.characteristic_matrix,
		 0.0, decisionMatrix);

  /*
    cout << "another way to display the matrix" << endl;
    for (int i = 0; i < 4; ++i)
    {
    cout << "\t";
    for (int j = 0; j < 4; ++j)
    {
    cout << gsl_matrix_get(decisionMatrix, i, j) << "\t";
    }
    cout << endl;
    }
    cout << endl;
  */

  gsl_vector_complex *eval = gsl_vector_complex_alloc(4);
  gsl_matrix_complex *evec = gsl_matrix_complex_alloc(4,4);

  gsl_eigen_nonsymmv_workspace *w = gsl_eigen_nonsymmv_alloc(4);

  gsl_eigen_nonsymmv(decisionMatrix, eval, evec, w);

  gsl_eigen_nonsymmv_free(w);

  //gsl_eigen_nonsymmv_sort(eval, evec, GSL_EIGEN_SORT_ABS_DESC);

  {
    int i, j;

    for (i = 0; i < 4; i++) //iterate over eigen vectors/values
      {
	gsl_complex eval_i = gsl_vector_complex_get (eval, i);
	gsl_vector_complex_view evec_i = gsl_matrix_complex_column (evec, i);

	//printf ("eigenvalue = %g + %gi\n", GSL_REAL(eval_i), GSL_IMAG(eval_i));
	//printf ("eigenvector = \n");
	
	gsl_matrix* a_vector = gsl_matrix_alloc(4,1);
	gsl_matrix* intermediate_row_vector = gsl_matrix_alloc(1,4);
	gsl_matrix* final = gsl_matrix_alloc(1,1);
	bool admissible = true;
	gsl_complex fourthDimension = gsl_vector_complex_get(&evec_i.vector, 3);


	if ((GSL_REAL(fourthDimension) == 0) && (GSL_IMAG(fourthDimension) == 0)){
	  admissible = false;
	  //cout << "don't do it" << endl;
	}

	if (admissible) {
	  for (j = 0; j < 4; ++j)
	    {
	    
	      gsl_complex z =
		gsl_vector_complex_get(&evec_i.vector, j);
	      //printf("%g + %gi\n", GSL_REAL(z), GSL_IMAG(z));

	      z = gsl_complex_div(z, fourthDimension);
	    
	      //#	      a(j) = GSL_REAL(z);

	      gsl_matrix_set(a_vector, j, 0, GSL_REAL(z));
	      
	      //a(j).imag() = 0;//GSL_IMAG(z);

	    } //end for j = 0; j < 4; j++
	
	  //                         cout << "X' A X = " <<  (a.transpose()*A)*(a) << " inside decision: " << (( (a.transpose()*A)*(a))  <= 0.00001) << endl;
	  //                         cout << "X' B X = " <<  (a.transpose()*B)*(a) << " inside decision: " << (( (a.transpose()*B)*(a))  <= 0.00001) << endl;

	  //#	  if (    ( ((a.transpose()*A)*(a))  <= 0.00001 ) &&
	  //#	  (( (a.transpose()*B)*(a))  <= 0.00001 )        ) {


	  gsl_blas_dgemm(CblasTrans, CblasNoTrans,
			 1.0, a_vector, s1.characteristic_matrix,
			 0.0, intermediate_row_vector);

	  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
			 1.0, intermediate_row_vector, a_vector,
			 0.0, final);

	  double decision_1 = gsl_matrix_get(final, 0, 0);


	  gsl_blas_dgemm(CblasTrans, CblasNoTrans,
			 1.0, a_vector, s2.characteristic_matrix,
			 0.0, intermediate_row_vector);

	  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
			 1.0, intermediate_row_vector, a_vector,
			 0.0, final);

	  double decision_2 = gsl_matrix_get(final, 0, 0);


	  if (    ( decision_1  <= 0.00001 ) &&
	  	  ( decision_2  <= 0.00001 )        ) {

	    overlapOrTouch = true;
	    break;
	  }
	  
	} // end if admissible
      } //end for i = 0; i < 4; i++
  }

  gsl_vector_complex_free(eval);
  gsl_matrix_complex_free(evec);

  //cout << overlapOrTouch << endl;
  return overlapOrTouch;
}

void Stone::resetParent(){
  this->parent = this;
}

Stone* Stone::findRoot(int& depth){
  if (this == parent)
    return this;

  ++depth;
  
  //path shortcuting:
  this->parent = parent->findRoot(depth);

  return this->parent;
}

void Stone::linkTwoStones(Stone* s1, Stone* s2, int& sumOfDepths){

  Stone *parent_s1, *parent_s2;

  if (areTwoStonesLinked(s1, s2, sumOfDepths)==true){
    return; //do nothing because they are already linked
  }
  else{
    /* OPTIMIZE WITH RANK ... BLAH BLAH*/
    parent_s1 = s1->findRoot();
    parent_s2 = s2->findRoot();

    Stone* newRoot = parent_s2;
    Stone* child = parent_s1;
    
    //    parent_s1->setParent(parent_s2);
    child->setParent(newRoot);

    /* expand bounding boxes */
    newRoot->x_lower_bound =
      my_MIN(parent_s1->x_lower_bound, parent_s2->x_lower_bound);
    newRoot->x_upper_bound =
      my_MAX(parent_s1->x_upper_bound, parent_s2->x_upper_bound);

    newRoot->z_lower_bound =
      my_MIN(parent_s1->z_lower_bound, parent_s2->z_lower_bound);
    newRoot->z_upper_bound =
      my_MAX(parent_s1->z_upper_bound, parent_s2->z_upper_bound);


    /* CIRCULAR LINKED LIST */
    Stone* temp = s1->next;
    s1->next = s2->next;
    s2->next = temp;

  }
}

bool Stone::areTwoStonesLinked(Stone* s1, Stone* s2, int& sumOfDepths){
  int depth_s1 = 0;
  int depth_s2 = 0;

  Stone *parent_s1, *parent_s2;

  parent_s1 = s1->findRoot(depth_s1);
  parent_s2 = s2->findRoot(depth_s2);

  sumOfDepths = depth_s1 + depth_s2;

  return (parent_s1 == parent_s2);  
}

bool Stone::areTwoStonesLinked(Stone* s1, Stone* s2, std::vector<int> * cost_vector){
  int depth_s1 = 0;
  int depth_s2 = 0;

  Stone *parent_s1, *parent_s2;

  parent_s1 = s1->findRoot(depth_s1);
  parent_s2 = s2->findRoot(depth_s2);

  int sum = depth_s1 + depth_s2;
  //##cost_vector->push_back(sum);

  return (parent_s1 == parent_s2);  
}

std::string Stone::serializeTheStone(){

  if (clean_mu_cov == false){
    std::cout << "unprocessed stone can not be serialized" << std::endl;
    exit(EXIT_FAILURE);
  }
  
  std::string result;
  std::ostringstream strs;

  strs << offset_signature << "," << id << ",";
  
  for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
	{
	  strs << gsl_matrix_get(characteristic_matrix, i, j) << ",";
	}
    }

  for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
	{
	  strs << gsl_matrix_get(characteristic_matrix_inverse, i, j) << ",";
	}
    }
  
  strs << this->x_lower_bound << ","
       << this->x_upper_bound << ","
       << this->z_lower_bound << ","
       << this->z_upper_bound << ",";

  result = strs.str();

  return result;
}
