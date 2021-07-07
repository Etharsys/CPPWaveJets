#include "wavejet.hpp"



using namespace std;
using namespace Eigen;


void Wavejet::compute_wavejets()
{
    //cout << "\nNEIGHBORS : \n" << _neighbors << endl;
    set_lines_cols();
    //cout << "\nnneigh & ncolphi : " << _nneigh << ", " << _ncolPhi << endl;
    auto svdV = neighbours_principal_vector();
    //cout << "\nSVD : \n" << svdV << endl;

    auto coords = neighbours_coords(svdV);
    //cout << "\nCOORDS : \n" << coords << endl;
    switch_polar_coords(coords);
    //cout << "\nPOLAR radius : \n" << _radius.adjoint() << endl;
    //cout << "\nPOLAR thetas : \n" << _theta.adjoint()  << endl;
    //cout << "\nPOLAR z      : \n" << _z.adjoint()      << endl;

    compute_phi();
    compute_a();
    //cout << "A(n) coefficients : " << _an.transpose() << endl;
}


void Wavejet::display_svdV(cv::viz::Viz3d& cam)
{
    using namespace cv::viz;

    cv::Point3d ori { _p(0) , _p(1) , _p(2) };
    cv::Point3d t1  { _t1(0) * 10, _t1(1) * 10, _t1(2) * 10};
    cv::Point3d t2  { _t2(0) * 10, _t2(1) * 10, _t2(2) * 10};
    cv::Point3d nm  { _normal(0) * 10, _normal(1) * 10, _normal(2) * 10};

    cout << "t1 : " << t1 << endl;
    cout << "t2 : " << t2 << endl;

    cam.showWidget("origin", WCloud(vector {ori} , Color::green()));
    cam.showWidget("t1"    , WLine (ori, t1 + ori, Color::blue ()));
    cam.showWidget("t2"    , WLine (ori, t2 + ori, Color::blue ()));
    cam.showWidget("normal", WLine (ori, nm + ori, Color::red  ()));
}

cv::Affine3d Wavejet::get_svd()
{
    auto affine3d = cv::Affine3d();
    for (u_int i = 0; i < 3; ++i)
    {
        affine3d.matrix(i, 0) = _t1(i);
        affine3d.matrix(i, 1) = _t2(i);
        affine3d.matrix(i, 2) = _normal(i);   
    }
    //cout << "opencv : \n" << affine3d.matrix << endl;

    return affine3d.translate(cv::Point3d {_p(0), _p(1), _p(2)} );
}


void Wavejet::display(cv::viz::Viz3d& cam, 
                      const cv::Affine3d& transform, 
                      u_int idx)
{
    WavejetDisplay wd { _phi, _order };
    wd.compute_and_find_point_position();
    wd.display(cam, transform, _nr, idx);
}

void Wavejet::display(cv::viz::Viz3d& cam, u_int idx)
{
    display(cam, cv::Affine3d(), idx);
}


MatrixXd Wavejet::list_to_matrixXd(const Neighbors& neighbors)
{
    u_int i = 0;
    MatrixXd mat { neighbors.size(), 3 };
    for (const auto& p : neighbors)
    {
        if (p.x == _p(0) && p.y == _p(1) && p.z == _p(2))
        {
            continue;
        }
        mat(i, 0) = p.x;
        mat(i, 1) = p.y;
        mat(i, 2) = p.z;
        i++;
    }
    return mat;
}


void Wavejet::set_lines_cols()
{
    _nneigh  = _neighbors.col(0).size();
    _ncolPhi = (pow(_order, 2) / 2.) + (3. * _order / 2) + 1;
}


Eigen::Matrix3d Wavejet::neighbours_principal_vector()
{
    Vector3d average_p { _neighbors.col(0).mean(), 
                         _neighbors.col(1).mean(),
                         _neighbors.col(2).mean() };
    Matrix3d c_matrix = 1./_nneigh * 
                        (_neighbors.adjoint() * _neighbors) -
                        ( average_p *  average_p.adjoint());
    JacobiSVD<MatrixXd> svd { c_matrix, ComputeFullV };
    Matrix3d v = svd.matrixV();
    _t1        = v.col(0);
    _t2        = v.col(1);
    _normal    = v.col(2);
    return v;
}


MatrixXd Wavejet::neighbours_coords(const Eigen::Matrix3d& neighbors_principal_vectors)
{
    MatrixXd repetition { _nneigh, 3 };
    for (u_int i = 0; i < _nneigh; ++i)
    {
        repetition.row(i) = _p; // _nneigh * _p (as colon)
    }
    return (_neighbors - repetition) * neighbors_principal_vectors;
}


void Wavejet::switch_polar_coords(const MatrixXd& locneighbors)
{
    _theta  = VectorXd { _nneigh };
    _radius = VectorXd { _nneigh };
    _z      = VectorXd { _nneigh };
    for (u_int i = 0; i < _nneigh; ++i)
    {
        _radius(i) = sqrt( pow(locneighbors(i, 0), 2) + pow(locneighbors(i, 1), 2) );
        _theta(i)  = atan2(locneighbors(i, 1), locneighbors(i, 0));
    }
    _z = locneighbors.col(2);
}

void Wavejet::compute_phi()
{
    MatrixXcd M       { _nneigh , _ncolPhi };
    MatrixXd  indices { _ncolPhi, 2 };
    unsigned int idx   = 0;
    auto norm_radius = _radius / _nr;
    _z = _z / _nr;
    auto w = exp(- norm_radius.array().square() / 18); // 18 ?
    for (u_int k = 0; k <= _order; ++k) 
    {
        auto rk = norm_radius.array().pow(k);
        for (int n = -int(k); n <= int(k); n+=2) 
        {
            VectorXd  n_thet = double(n) * _theta;
            VectorXcd e { n_thet.size() };
            e.real() << n_thet.array().cos();
            e.imag() << n_thet.array().sin();

            M.col(idx) = rk.array() * e.array() * w.array();
            indices.row(idx) = Vector2d { k, n };

            idx++;
        }
    }
    MatrixXcd b = _z.array() * w.array();
    VectorXcd phi = (M.adjoint() * M).inverse() * M.adjoint() * b;
    /*FullPivLU<MatrixXcd> lu_decomp(M);
    cout << "rank(M) = " << lu_decomp.rank() << endl << endl;
    cout << "idx = " << indices << endl;*/
    
    idx = 0;
    for (u_int k = 0; k <= _order; ++k) 
    {
        for (int n = -int(k); n <= int(k); n+=2) 
        {
            _phi.set(k, n, phi(idx));
            idx++;
        }
    }
}


void Wavejet::compute_a()
{
    u_int idx = 0;
    _an = MatrixXd::Zero(_order+1, 1);
    for (u_int k = 0; k < _order; ++k)
    {
        for (int n = -int(k); n <= int(k); n+=2)
        {
            if (n >= -1)
            {
                _an(n+1) += _phi.at(k, n) / double(k+2);
            }
            idx++;
        }
    }
    _an(0) = _an(0).real();
    _an(1) = complex { _an(1).real(), abs(_an(1).imag()) };
}