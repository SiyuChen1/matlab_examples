/*


 dval : returns 
 

 Usage
 -------


 v            = dval(x , y , [w] , [sigma] );

 Inputs
 -------

 x             Source data (d x Nx)
 y             Target data (d x Ny)
 w             Weigths (1 x Nx) ( default w = ones(1 , Nx) ) 
 sigma         Noise Standard deviation of the kernel (default sigma = 1)


 Ouputs
 -------
 v             density (1 x Ny)


 To compile
 ----------

 mex dval.c


 mex  -f mexopts_intel10.bat -output dval.dll dval.c


Example 1 
---------


d       = 10;
Nx      = 100;
Ny      = 10000;
x       = randn(d , Nx);
y       = randn(d , Ny);
v       = dval(x , y);



Example 2 
---------


d       = 10;
Nx      = 100;
Ny      = 10000;
x       = randn(d , Nx);
y       = randn(d , Ny);
u       = rand(1 , Nx);
sigma   = 2;
v       = dval(x , y , u , sigma);

 
Example 3 
---------


d       = 2;
R       = [2 , 0.4 ; 0.4  3];
Nx      = 100;
sigma   = 3;
vect    = (-5:0.3:5);
Ny      = length(vect);
w       = (1/Nx)*ones(1 , Nx);
  
x       = (chol(R)'*randn(d , Nx));

[X , Y] = meshgrid(vect);
y       = [X(:) , Y(:)]';

v       = dval(x , y , w , sigma);

densite = reshape( v , Ny , Ny);

figure
set(gcf , 'renderer' , 'opengl');
surfc(X , Y , densite)
shading interp
lighting phong

light
alpha(0.5);
hold on
plot(x(1 , :) , x(2 , :) , 'r+' , 'markersize' , 10);
hold off
view(2)
colorbar


  
  Author : Sébastien PARIS : sebastien.paris@lis-lab.fr
  -------

*/


#include <math.h>
#include "mex.h"


/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/


void dval(double * , double * , double * , double  , double * , int  , int  , int );

/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/




void mexFunction( int nlhs, mxArray *plhs[] , int nrhs, const mxArray *prhs[] )

{
	double *x , *y , *w;
	double sigma = 1.0;
	double *v;
	const mwIndex  *dimsx , *dimsy;	
	mwIndex *dimsv;
	int numdimsx , numdimsy ;
	int numdimsv;
	int i , d , Nx , Ny;
	
	/*--------------------------------------------------------------------------------*/
	/*--------------------------------------------------------------------------------*/
	/* -------------------------- Parse INPUT  -------------------------------------- */
	/*--------------------------------------------------------------------------------*/	
	/*--------------------------------------------------------------------------------*/

	if ((nrhs < 2))
	{
			mexErrMsgTxt("Usage : v = dval(x , y , [w] , [sigma]);"); 
	}
	
	/* ----- Input 1 ----- */
	
	x           = mxGetPr(prhs[0]);
	numdimsx    = mxGetNumberOfDimensions(prhs[0]);
	dimsx       = mxGetDimensions(prhs[0]);
	d           = dimsx[0];
	Nx          = dimsx[1];
	
	
	/* ----- Input 2 ----- */
	
	
	y           = mxGetPr(prhs[1]);
    numdimsy    = mxGetNumberOfDimensions(prhs[1]);
	dimsy       = mxGetDimensions(prhs[1]);
	Ny          = dimsy[1];
	
	/* ----- Input 3 ----- */
	
	if ((nrhs < 3) || mxIsEmpty(prhs[2]))
	{	
		w    = (double *)mxMalloc(Nx*sizeof(double));	
		for (i = 0 ; i < Nx ; i++)
		{
			w[i] = 1.0;	
		}
	}
	else
	{	
		w           = mxGetPr(prhs[2]);	
	}

	/* ----- Input 4 ----- */
	
	if (nrhs == 4)
	{				
		sigma       = (double)mxGetScalar(prhs[3]);	
	}
	
	/*--------------------------------------------------------------------------------*/
	/*---------------------------------------,----------------------------------------*/
	/* -------------------------- Parse OUTPUT  ------------------------------------- */
	/*--------------------------------------------------------------------------------*/
	/*--------------------------------------------------------------------------------*/
	
	/* ----- output 1 ----- */
	
	numdimsv       = 2;
	dimsv          = (mwIndex *)mxMalloc(numdimsv*sizeof(mwIndex));
	dimsv[0]       = 1;
	dimsv[1]       = Ny;
	
	plhs[0]        = mxCreateNumericArray(numdimsv , dimsv , mxDOUBLE_CLASS, mxREAL);
	v              = mxGetPr(plhs[0]);
	
	
	/*---------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------*/
	/* ----------------------- MAIN CALL  -------------------------------------------- */
	/*---------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------*/	
	/*---------------------------------------------------------------------------------*/
	
	
	dval(x , y , w , sigma , 
		 v , 
		 d , Nx , Ny);
	
	
	/*-----------------------------------------------*/
	/*-----------------------------------------------*/
	/* ------------ END of Mex File ---------------- */
	/*-----------------------------------------------*/
	/*-----------------------------------------------*/
	
	mxFree(dimsv);
	
	if ((nrhs < 3) || mxIsEmpty(prhs[2]))
	{
		mxFree(w);
	}
}

/*----------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------*/



void dval(double *x , double *y , double *w , double sigma , 
		  double *v , 
		  int d , int Nx , int Ny )

{
	int i , j , l , id , jd;
	register double temp , res ;
	double tempv;
	double cte = -1.0/(sigma*sigma) ;
	
	for (i = 0; i < Ny ; i++) 
	{
		id    = i*d;	
		tempv = 0.0;
		for (j = 0; j < Nx ; j++) 
		{
			jd  = j*d;
			res = 0.0;
			for (l = 0 ; l < d ; l++)
			{
				temp  = (y[l + id] - x[l + jd]);	
				res  +=temp*temp;
			}
			tempv += w[j]*exp(cte*res);	
		}
		v[i] = tempv;
	}
}



