/* matrix.h
 *
 * A simple matrix library for representing position and covariance
 *
 * Author: Austin Hendrix
 */

#include <assert.h>
#include <iostream>

template<int N, int M>
class matrix;

template<int N, int M>
std::ostream & operator<<( std::ostream &, matrix<N, M> m);

template<int N, int M>
class matrix {
   public:
      matrix() {} // explicitly doesn't initialize matrix
      matrix(double); // explicitly initalize every element to init
      ~matrix() {}
      // default copy constructor is ok

      // addition
      matrix operator+(matrix &o);

      // matrix multiplication
      //  this is defined as a friend function rather than a member because
      //  templated friend classes aren't allowed.
      template<int K, int L, int J> 
         friend matrix<K, J> operator*(matrix<K, L> a, matrix<L, J> o);

      // matrix transpose
      matrix<M, N> T();

      // inversion and determinants are defined as friends so that the 
      // compiler checks that they're only operating on square matrices

      // inversion
      template<int K> friend matrix<K, K> invert(matrix<K, K> &m);
      // determinant
      template<int K> friend double det(matrix<K, K> &m);

      // identity matrix
      template<int K> friend matrix<K, K> I();

      // print matrix
      friend std::ostream & operator<<<>( std::ostream &, matrix<N, M> m);
   private:
      double data[N][M];

};

// constructor. initialize to 0
template<int N, int M> matrix<N, M>::matrix(double init) {
   for( int i=0; i<N; i++ )
      for( int j=0; j<M; j++ )
         data[i][j] = init;
}

// add matrices
template<int N, int M> matrix<N, M> matrix<N, M>::operator+(matrix<N, M> &o) {
   matrix<N, M> res; // no initialization needed
   for( int i=0; i<N; i++ ) {
      for( int j=0; j<M; j++ ) {
         res.data[i][j] = data[i][j] + o.data[i][j];
      }
   }

   return res;
}

// multiply two matrices
template<int N, int M, int K>
matrix<N, K> operator*(matrix<N, M> a, matrix<M, K> o) {
   matrix<N, K> res(0.0);

   for( int i=0; i<N; i++ ) {
      for( int j=0; j<K; j++ ) {
         
         for( int m=0; m<M; m++ ) {
            res.data[i][j] += a.data[i][m] * o.data[m][j];
         }
      }
   }

   return res;
}

// transpose a matrix
template<int N, int M>
matrix<M, N> matrix<N, M>::T() {
   matrix<M, N> ret;

   for( int i=0; i<N; i++ ) {
      for( int j=0; j<M; j++ ) {
         ret.data[j][i] = data[i][j];
      }
   }

   return ret;
}

// inversion
template<int K>
matrix<K, K> invert(matrix<K, K> & m) {
   matrix<K, K*2> tmp;
   matrix<K, K> res;

   // build temporary matrix
   //  first half; input matrix
   for( int i=0; i<K; i++ ) {
      for( int j=0; j<K; j++ ) {
         tmp.data[i][j] = m.data[i][j];
      }
   }
   // second half; identity matrix
   for( int i=0; i<K; i++ ) {
      for( int j=0; j<K; j++ ) {
         tmp.data[i][j+K] = (i==j)?1.0:0.0;
      }
   }

   // do gaussian elimination
   for( int i=0; i<K; i++ ) {
      // per row:
      
      // cancel inital values to 0
      for( int j=0; j<i; j++ ) {
         if( tmp.data[i][j] != 0.0 ) {
            double m = tmp.data[i][j] / tmp.data[j-1][j];
            for( int k=j; k<2*K; k++ ) { // we should be able to start at k=j
               tmp.data[i][k] -= tmp.data[j-1][j]*m;
            }
         }
      }

      // divide row so that initial constant is 1
      double s = tmp.data[i][i];
      // avoid divide by 0 (will happen is matrix isn't invertible)
      // TODO: do something better than an assert here
      assert( s != 0.0 );
      for( int j=0; j<K*2; j++ ) {
         tmp.data[i][j] /= s;
      }
   }

   // build result matrix
   for( int i=0; i<K; i++ ) {
      for( int j=0; j<K; j++ ) {
         res.data[i][j] = tmp.data[i][j+K];
      }
   }

   return res;
}

// generate and identity matrix of the requested size
template<int K>
matrix<K, K> I() {
   matrix<K, K> ret(0.0);
   // initialization to 0 and then setting the diagonal has complexity 
   //  O(K*K + K) ~= O(K*K) which is acceptable for now
   //  if this is really an issue, fix it then
   for( int i=0; i<K; i++ ) {
      ret.data[i][i] = 1.0;
   }

   return ret;
}

// output function
template<int N, int M> 
std::ostream & operator<<(std::ostream & out, matrix<N, M> m) {
   out << "[ ";
   for( int i=0; i<N; i++ ) {
      out << "[ ";
      for( int j=0; j<M; j++ ) {
         out << m.data[i][j];
         if( j < M-1 ) 
            out << ", ";
      }
      out << " ]";
      if( i < N-1) out << std::endl;
   }
   out << " ]" << std::endl;
   return out;
}
