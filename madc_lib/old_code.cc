bool Stone::compareTwoStones(Stone& s1, Stone& s2){

  bool overlapOrTouch = false;

  
  double** sigma_1 = s1.getSigma();
  double** sigma_2 = s2.getSigma();
  double* mu_1 = s1.getMu();
  double* mu_2 = s2.getMu();
  
  Matrix3d sigma_1Mat;

  for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
	{
	  sigma_1Mat(i,j) = sigma_1[i][j];
	}
    }

  //cout << "sigma_1Mat is" << endl << sigma_1Mat << endl;

  Matrix3d sigma_1InvMat = sigma_1Mat.inverse();

  Matrix4d A = Matrix4d::Zero(4,4);
  
  for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
  	{
  	  A(i,j) = sigma_1InvMat(i,j);
	}
    }
  A(3,3) = -1;

  
  Matrix4d T = Matrix4d::Identity(4,4);
  T(3,0) = -1*(mu_2[0] - mu_1[0]);
  T(3,1) = -1*(mu_2[1] - mu_1[1]);
  T(3,2) = -1*(mu_2[2] - mu_1[2]);

  Matrix3d sigma_2Mat;

  for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
  	{
  	  sigma_2Mat(i,j) = sigma_2[i][j];
  	}
    }

  Matrix3d sigma_2InvMat = sigma_2Mat.inverse();

  Matrix4d B = Matrix4d::Zero(4,4);
  
  for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
  	{
  	  B(i,j) = sigma_2InvMat(i,j);
  	}
    }
  B(3,3) = -1;

  B = T * B * T.transpose();

  //cout << "A = " << endl << A << endl << "B = " << endl << B << endl;

  Matrix4cd tokhmiTakhayoli = A.inverse() * B;

  //cout << "Decision Matrix = " << endl << tokhmiTakhayoli << endl;


  /****************************************************************************************************/

  gsl_matrix* decisionMatrix = gsl_matrix_alloc(4,4);


  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
      {
	gsl_matrix_set(decisionMatrix, i, j, tokhmiTakhayoli(i,j).real());
      }
  }

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
	
	Vector4d a;
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
	    
	      a(j) = GSL_REAL(z);
	      //a(j).imag() = 0;//GSL_IMAG(z);

	    } //end for j = 0; j < 4; j++
	
	  //                         cout << "X' A X = " <<  (a.transpose()*A)*(a) << " inside decision: " << (( (a.transpose()*A)*(a))  <= 0.00001) << endl;
	  //                         cout << "X' B X = " <<  (a.transpose()*B)*(a) << " inside decision: " << (( (a.transpose()*B)*(a))  <= 0.00001) << endl;

	  if (    ( ((a.transpose()*A)*(a))  <= 0.00001 ) && (( (a.transpose()*B)*(a))  <= 0.00001 )        ) {
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
