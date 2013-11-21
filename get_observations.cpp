#include <string.h>
#include <math.h>
#include <map>
#include <iostream>

#include "mex.h"
#include "matrix.h"

#define NUM_READINGS 180

/*************************************************************/
bool isOccupied( double *imgData, int x, int y, int width, int height )
{
    if( !( x >= 0 && x < width && y >= 0 && y < height ) )
        return true;
    else if( imgData[y*width + x] <= 250 )
        return true;
    else 
        return false;
}

double ray_cast(  double *imgData, double *pose, double max_range, int dir, int width, int height )
{
    /* dir is the iteration num for the laser measurement */
    double theta = pose[2] + (dir-NUM_READINGS/2)*((22/7.0)/180.0);
    int x = (int)(pose[1]);
    int y = (int)(pose[0]);
    
    double range = 0.5;
    
    while( x >= 0 && x < width && y >= 0 && y < height )
    {
        x = (int)(pose[1] + range * cos(theta));
        y = (int)(pose[0] + range * sin(theta));
        range += 0.5;
        
        if( isOccupied( imgData, x, y, width, height ) )
        {
            break;
        }
    }
    return sqrt( (pose[1]-x)*(pose[1]-x) + (pose[0]-y)*(pose[0]-y) );
}

void getMeasurements(
        double *imgData,
        double *pose,
        double max_range,
        std::map<double,double> &measurements,
        int width,
        int height
        )
{
    for( int ii = 0; ii < NUM_READINGS; ii++ ){
        double range = ray_cast( imgData, pose, max_range, ii, width, height );
        measurements[ (ii-NUM_READINGS/2) * ((22/7.0)/180.0)] = range;
    }    
}

/*************************************************************/
void mexFunction(int nlhs, mxArray *plhs[ ],int nrhs, const mxArray *prhs[ ]) 
{
    /* Read the input */    
    double *imgData = mxGetPr( prhs[0] );
    
    double *dWidth = mxGetPr( prhs[1] );
    int width = (int) (*dWidth);    
    
    double *dHeight = mxGetPr( prhs[2] );
    int height = (int)(*dHeight);
    
    double *pose = mxGetPr( prhs[3] );    
    double max_range = (double) (*(mxGetPr(prhs[4])));
 
        
    std::map<double,double> measurements;
    getMeasurements( imgData, pose, max_range, measurements, width, height );
    plhs[0] = mxCreateDoubleMatrix( measurements.size(), 2, mxREAL );
    double *output = mxGetPr( plhs[0] );

    int ii = 0;
    for( std::map<double,double>::iterator it = measurements.begin();
        it != measurements.end(); it++, ii++ )
    {
        output[ ii ] = it->first;
        output[ ii + measurements.size() ] = it->second;
    }
}
