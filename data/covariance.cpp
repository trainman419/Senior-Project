/*
 * explore the relationship between covariances of a 2D ellipsoidal distribution
 *
 * Author: Austin Hendrix
 */

#include <iostream>
#include <vector>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

using namespace std;

double mean(vector<double> * v) {
   double tot = 0.0;
   for(int i=0; i<v->size(); i++) {
      tot += v->at(i);
   }
   return tot/v->size();
}

double cov(vector<double> * a, vector<double> * b) {
   double s = 0.0;
   assert(a->size() == b->size());
   double ma = mean(a);
   double mb = mean(b);
   for( int i=0; i<a->size(); i++ ) {
      s += (a->at(i) - ma) * (b->at(i) - mb);
   }
   return s/a->size();
}

inline double rand_g() {
   return ((double)rand())/((double)RAND_MAX); // [0, 1]
}

double g1(double dev) {
   double r = 0.0;
   for( int i=0; i<12; i++ ) {
      r += rand_g();
   }
   r -= 6.0;
   r *= dev;
   return r;
}

vector<double> * gauss(int c, double dev) {
   vector<double> * res = new vector<double>(c);
   for( int i=0; i<c; i++ ) {
      (*res)[i] = g1(dev);
   }
   return res;
}

vector<double> * mult(vector<double> * a, double b) {
   vector<double> * res = new vector<double>(a->size());
   for(int i=0; i<a->size(); i++) {
      (*res)[i] = a->at(i) * b;
   }
   return res;
}

vector<double> * add(vector<double> * a, vector<double> * b) {
   vector<double> * res = new vector<double>(a->size());
   assert(a->size() == b->size());
   for( int i=0; i<a->size(); i++ ) {
      (*res)[i] = a->at(i) + b->at(i);
   }
   return res;
}

vector<double> * sub(vector<double> * a, vector<double> * b) {
   vector<double> * res = new vector<double>(a->size());
   assert(a->size() == b->size());
   for( int i=0; i<a->size(); i++ ) {
      (*res)[i] = a->at(i) - b->at(i);
   }
   return res;
}



int main(int argc, char ** argv) {
   // seed the random number generator
   srand(time(0));

   vector<double> * a = gauss(1000000, 1);
   vector<double> * b = gauss(1000000, 10);

   double var_a = cov(a, a);
   double var_b = cov(b, b);

   printf("a: % 2.4f % 2.4lf\n", mean(a), var_a);
   printf("b: % 2.4f % 2.4lf\n", mean(b), var_b);

   //*
   for( double t=-M_PI; t <= M_PI; t += M_PI/128) {
      vector<double> * ca = mult(a, cos(t));
      vector<double> * sa = mult(a, sin(t));
      vector<double> * cb = mult(b, cos(t));
      vector<double> * sb = mult(b, sin(t));

      vector<double> * x = sub(ca, sb);
      vector<double> * y = add(cb, sa);

      /*
      printf("% 7.4lf: % 9.4lf(% 9.4lf) % 9.4lf(% 9.4lf) % 9.4lf(% 9.4lf)\n", t,
            cov(x, x), var_a*cos(t)*cos(t) + var_b*sin(t)*sin(t), 
            cov(y, y), var_b*cos(t)*cos(t) + var_a*sin(t)*sin(t),
            cov(x, y), var_a*sin(t)*cos(t) - var_b*sin(t)*cos(t));
            */
      double cxx = cov(x, x);
      double cyy = cov(y, y);
      double cxy = cov(x, y);
      double p_cxx = var_a*cos(t)*cos(t) + var_b*sin(t)*sin(t); 
      double p_cyy = var_b*cos(t)*cos(t) + var_a*sin(t)*sin(t);
      double p_cxy = var_a*sin(t)*cos(t) - var_b*sin(t)*cos(t);

      printf("% 7.4f: % 9.4lf % 9.4lf % 9.4lf\n", t,
            (cxx - p_cxx)/cxx, (cyy - p_cyy)/cyy, (cxy - p_cxy)/cxy);

      delete ca;
      delete sa;
      delete cb;
      delete sb;
      delete x;
      delete y;
   }
   //*/
}
