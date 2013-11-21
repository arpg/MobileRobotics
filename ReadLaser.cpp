#include "mex.h"
#include <math.h>

double RayCast( 
        unsigned char *img, 
        int w,
        int h,
        double *pose, 
        double th,
        double max_range, 
        double mpp
        )
{
    double theta = pose[2] + th;
    double ray[2] = { cos(theta), sin(theta) };
    for( double range = 1.0; range < max_range/mpp; range+=0.5 ){
        double pt[2] = { range*ray[0] + pose[0], range*ray[1] + pose[1] };
        if( pt[0] <= 0 || pt[0] >= w || pt[1] <= 0 || pt[1] >= h ){
            continue;
        }
        int pr[2] = { round(pt[0]), round(pt[1]) };
        if( img[ pr[0]*h + pr[1] ] < 200 ){ // because matlab images are transposed!
            double z = sqrt( (pt[0]-pose[0])*(pt[0]-pose[0])
                            +(pt[1]-pose[1])*(pt[1]-pose[1]) ) * mpp;
            return z;
        }
    }
    return max_range;
}


// function z = ReadLaser( img, mpp, left, top, pose, fov, n_beams, dMaxRange )
void mexFunction(int nlhs, mxArray *plhs[ ],int nrhs, const mxArray *prhs[ ]) 
{

    // 1) get map: check data has the rigth shape
    if( !mxIsUint8(prhs[0]) ){
        mexErrMsgTxt("First param should be a uint8 image");
    }
    unsigned char *img = (unsigned char*)mxGetData( prhs[0] );
    size_t width  = mxGetN( prhs[0] );
    size_t height = mxGetM( prhs[0] );

    // 2) get map meters per pixel:
    double mpp = *(double *)mxGetPr( prhs[1] );

    // 3) get left coord of map in meters:
    double left = *(double *)mxGetPr( prhs[2] );

    // 4) get top coord of map in meters:
    double top = *(double *)mxGetPr( prhs[3] );

    // 5) get robot pose
    double* xwr = (double *)mxGetPr( prhs[4] );

    // 6) get max range
    double max_range = *(double *)mxGetPr( prhs[5] );

    // 7) get fov, in radians
    double fov = *(double *)mxGetPr( prhs[6] );

    // 8) get number of beams
    int n_beams = (*mxGetPr( prhs[7] ));

    plhs[0] = mxCreateDoubleMatrix( n_beams, 1, mxREAL );
    double *z = (double *)mxGetPr( plhs[0] );
    double dth = fov/(double)(n_beams-1);
    int ii=0;
    double pose[3] = {(xwr[0]-left)/mpp, (xwr[1]-top)/mpp, xwr[2]};
    for( double th = -fov/2.0; th < fov/2.0+dth; th+=dth ){
        
        z[ii++] = RayCast( img, width, height, pose, th, max_range, mpp );
    }
}
