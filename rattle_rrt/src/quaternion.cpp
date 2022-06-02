/*Copyright (c) 2007 Jernej Barbic

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

Except as contained in this notice, the name(s) of the above 
copyright holders shall not be used in advertising or otherwise 
to promote the sale, use or other dealings in this Software 
without prior written authorization.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/

#include "rbd/quaternion.h"

// transforms the given matrix (assumed orthogonal) into one of the two corresponding quaternions
// matrix is assumed to be in row-major order
// this implementation follows Baraff & Witkin SIGGRAPH 2003 course notes

template <typename real>
Quaternion<real> Quaternion<real>::Matrix2Quaternion(real * R) {
/* (0,0) 0  (0,1) 1  (0,2) 2
   (1,0) 3  (1,1) 4  (1,2) 5
   (2,0) 6  (2,1) 7  (2,2) 8
*/

  Quaternion<real> q;
  real tr, u;
  tr = R[0] + R[4] + R[8];

  if (tr >= 0) {
    u = (real)sqrt(tr + 1);
    q.s = (real)0.5 * u;
    u = (real)0.5 / u;
    q.x = (R[7] - R[5]) * u;
    q.y = (R[2] - R[6]) * u;
    q.z = (R[3] - R[1]) * u;
  } else {
    int i = 0;
    if (R[4] > R[0])
      i = 1;

    if (R[8] > R[3*i+i])
      i = 2;

    switch (i) {
      case 0:
        u = (real)sqrt((R[0] - (R[4] + R[8])) + 1);
        q.x = 0.5f * u;
        u = 0.5f / u;
        q.y = (R[3] + R[1]) * u;
        q.z = (R[2] + R[6]) * u;
        q.s = (R[7] - R[5]) * u;
      break;

      case 1:
        u = (real)sqrt((R[4] - (R[8] + R[0])) + 1);
        q.y = 0.5f * u;
        u = 0.5f / u;
        q.z = (R[7] + R[5]) * u;
        q.x = (R[3] + R[1]) * u;
        q.s = (R[2] - R[6]) * u;
      break;

      case 2:
        u = (real)sqrt((R[8] - (R[0] + R[4])) + 1);
        q.z = 0.5f * u;

        u = 0.5f / u;
        q.x = (R[2] + R[6]) * u;
        q.y = (R[7] + R[5]) * u;
        q.s = (R[3] - R[1]) * u;
      break;
    }
  }

  return q;
}

template Quaternion<double> Quaternion<double>::Matrix2Quaternion(double * R);
template Quaternion<float> Quaternion<float>::Matrix2Quaternion(float * R);
