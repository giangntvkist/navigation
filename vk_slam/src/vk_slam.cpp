#include "vk_slam/vk_slam.hpp"
#include "vk_slam/optimization.cpp"
typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> Trip;

// void insertCoefficient(int id, int i, int j, double w, std::vector<T>& coeffs, Eigen::VectorXd& b, const Eigen::VectorXd& boundary) {
//   int n = int(boundary.size());
//   int id1 = i+j*n;

//         if(i==-1 || i==n) b(id) -= w * boundary(j); // constrained coefficient
//   else  if(j==-1 || j==n) b(id) -= w * boundary(i); // constrained coefficient
//   else  coeffs.push_back(T(id,id1,w));              // unknown coefficient
// }

// void buildProblem(std::vector<T>& coefficients, Eigen::VectorXd& b, int n) {
//   b.setZero();
//   Eigen::ArrayXd boundary = Eigen::ArrayXd::LinSpaced(n, 0,M_PI).sin().pow(2);
//   for(int j=0; j<n; ++j)
//   {
//     for(int i=0; i<n; ++i)
//     {
//       int id = i+j*n;
//       insertCoefficient(id, i-1,j, -1, coefficients, b, boundary);
//       insertCoefficient(id, i+1,j, -1, coefficients, b, boundary);
//       insertCoefficient(id, i,j-1, -1, coefficients, b, boundary);
//       insertCoefficient(id, i,j+1, -1, coefficients, b, boundary);
//       insertCoefficient(id, i,j,    4, coefficients, b, boundary);
//     }
//   }
// }
// void solve_LES(const SparseMatrix<double> &A, const Matrix<double,Dynamic,1> &b, Matrix<double,Dynamic,1> &x) {
//     SparseLU<SparseMatrix<double>, COLAMDOrdering<int> > solver;
//     solver.compute(A);
//     if (solver.info() != Success) std::cout << "Decomposition failed" << std::endl;
//     x = solver.solve(b);
//     if (solver.info() != Success) std::cout << "solver failed" << std::endl;
// }
int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_slam");
    ROS_INFO("Running vk_slam node!");
    ros::NodeHandle nh;


    //  int n = 4;
    // std::vector<Triplet<double>> coefficients;
    // Eigen::Matrix<double,Dynamic,1> b(n), x(n);

    // buildProblem(coefficients, b, n);
    
    // Eigen::SparseMatrix<double> A(n,n);
    // A.setFromTriplets(coefficients.begin(), coefficients.end());
    // cout << A << endl;
    // solve_LES(A,b,x);
    // cout << x << endl;


std::vector<Trip> trp, tmp;

    // I subtracted 1 from the indices so that the output matches your question
    trp.push_back(Trip(1-1,1-1,3));
    trp.push_back(Trip(1-1,3-1,4));
    trp.push_back(Trip(2-1,3-1,1));
    trp.push_back(Trip(3-1,2-1,2));
    trp.push_back(Trip(3-1,4-1,5));
    trp.push_back(Trip(4-1,1-1,4));
    trp.push_back(Trip(6-1,1-1,0));

    int rows, cols;
    rows = cols = 6;
    SparseMatrix<int> A(rows,cols);

    A.setFromTriplets(trp.begin(), trp.end());
    cout << "Matrix from triplets:" << endl;
    cout << A << endl;            

    cout << endl << "Triplets:" << endl;
    cout << "Row\tCol\tVal" <<endl;
    for (int k=0; k < A.outerSize(); ++k)
    {
        for (SparseMatrix<int>::InnerIterator it(A,k); it; ++it)
        {
            cout << 1+it.row() << "\t"; // row index
            cout << 1+it.col() << "\t"; // col index (here it is equal to k)
            cout << it.value() << endl;
        }
    }
    Eigen::MatrixXd B;
    B.setIdentity(3,3);
    
    cout << SparseMatrix<int>(A.block(0,0,3,3)) << endl;
    cout << B << endl;
        

    // ros::Rate rate(map_update_interval);
    // while(ros::ok()) {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    return 0;
}