/* matrix_test.cpp
 *
 * unit tests for the matrix class
 *
 * Author: Austin Hendrix
 */

#include <iostream>
#include "matrix.h"

using namespace std;

int main(int argc, char ** argv) {
   matrix<3, 3> a = I<3>();
   matrix<3, 3> b = I<3>();
   matrix<3, 3> c = a + b;
   matrix<3, 3> d;

   d.data[0][0] = 1.0;
   d.data[0][1] = 2.0;
   d.data[0][2] = 3.0;
   d.data[1][0] = 4.0;
   d.data[1][1] = 1.0;
   d.data[1][2] = 2.0;
   d.data[2][0] = 3.0;
   d.data[2][1] = 4.0;
   d.data[2][2] = 1.0;

   cout << a << endl;
   cout << a.T() << endl;
   cout << c << endl;
   cout << invert(c) << endl;
   cout << invert(c) * c << endl;
   cout << d << endl;

   cout << invert(d) << endl;
   /* ought to be:
    * [ [ 1, 0, 0]
    *   [ 0.571, -0.142, 0]
    *   [ 0.361, 0.056, -0.194] ]
    */
   cout << invert(d) * d << endl;
}
