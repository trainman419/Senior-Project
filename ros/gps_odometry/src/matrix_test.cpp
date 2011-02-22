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

   cout << a << endl;
   cout << a.T() << endl;
   cout << c << endl;
   cout << invert(c) << endl;
   cout << invert(c) * c << endl;
}
