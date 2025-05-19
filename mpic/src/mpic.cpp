
#include "mpic/mpic.h"

MatrixXd kronProduct(MatrixXd S, MatrixXd X)
{
    MatrixXd P(S.rows() * X.rows(), S.cols() * X.cols());
    P.setZero();
    for (int i = 0; i < S.rows(); i++)
    {
        for (int j = 0; j < S.cols(); j++)
        {
            P.block(i * X.rows(), j * X.cols(), X.rows(), X.cols()) = S(i, j) * X;
        }
    }
    return P;
}
MatrixXd diagMat(MatrixXd diag, int shift)
{
    MatrixXd M(diag.size() + std::abs(shift), diag.size() + std::abs(shift));
    M.setZero();

    MatrixXd diag_res;
    if (diag.cols() < diag.rows())
        diag_res = diag.transpose();
    else
        diag_res = diag;

    for (int i = 0; i < diag.size(); i++)
    {
        if (shift < 0)
            M(i - shift, i) = diag_res(0, i);
        else
            M(i, i + shift) = diag_res(0, i);
    }
    return M;
}
MatrixXd powMat(MatrixXd M, float pow)
{
    MatrixXd P = MatrixXd::Identity(M.rows(), M.cols());
    for (int i = 1; i <= pow; i++)
    {
        P = P * M;
    }
    return P;
}
/*--------------------MPIC--------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
// 状态变量的维度、控制变量的维度和预测时域的长度
MPIC::MPIC(int nx, int nu, int N)
    : _nx(nx), _nu(nu), _N(N)
{
    _selectX = MatrixXd::Zero(_nx, 1);
    _selectu = MatrixXd::Zero(_nu, 1);
    _Xmax = MatrixXd::Zero(_nx, 1);
    _Xmin = MatrixXd::Zero(_nx, 1);
    _umin = MatrixXd::Zero(_nu, 1);
    _umax = MatrixXd::Zero(_nu, 1);

    _A = MatrixXd::Zero(_nx, _nx);
    _B = MatrixXd::Zero(_nx, _nu);
    _K = MatrixXd::Zero(_nu, _nx);

    _nSlack = 1;
    _costSlack.push_back(1e16);
    //    _costSlack.push_back(1e16);
    //    _costSlack.push_back(1e16);
    //    _costSlack.push_back(8e18);
    //    _costSlack.push_back(8e18);
    //    _costSlack.push_back(8e18);

    QProblem qpb(_N * _nu + _nSlack, 2 * _N * (_nx + _nu) + _nSlack);
    _qpb = qpb;
    Options options;
    options.setToMPC();
    options.printLevel = PL_NONE;
    //    options.enableNZCTests = BT_TRUE;
    //    options.enableFlippingBounds = BT_TRUE;
    //    options.epsFlipping = 1e-6;
    //    options.initialStatusBounds = ST_LOWER;
    //    options.epsRegularisation = 1e-6;
    _qpb.setOptions(options);

    _uOpt = MatrixXd::Zero(_nu, 1);
    _Xmodel = MatrixXd::Zero(_nx, 1);
    _duOpt = MatrixXd::Zero(_nu, 1);
    _qpSolution = new real_t[_nu * _N + _nSlack];

    _valueSlack = std::vector<double>(_nSlack);

    _QPfail = false;
}

MPIC::~MPIC()
{
    delete[] _qpSolution;
}
/*--------------------------------------------------------------------------------*/
void MPIC::setSystem(MatrixXd A, MatrixXd B, MatrixXd K)
{
    _A = A;
    _B = B;
    _K = K;
}
/*--------------------------------------------------------------------------------*/
void MPIC::updateK(MatrixXd K)
{
    _K = K;
}
/*--------------------------------------------------------------------------------*/
void MPIC::computeQP()
{
    /* weight */
    // TODO: R 权重修改
    MatrixXd R = MatrixXd::Identity(_nu, _nu);
    MatrixXd Q = _K.transpose() * R * _K;
    MatrixXd S = _K.transpose() * R;

    /* computing QP matrices */

    MatrixXd IN = MatrixXd::Identity(_N, _N);
    MatrixXd Qbar = kronProduct(IN, Q);
    Qbar.block(0, 0, _nx, _nx) = MatrixXd::Zero(_nx, _nx);
    MatrixXd Rbar = kronProduct(IN, R);
    MatrixXd Sbar = kronProduct(IN, S);
    MatrixXd Cbar_tmp = kronProduct(IN, _B);
    for (int i = 1; i < _N; i++)
    {
        Cbar_tmp = Cbar_tmp + kronProduct(diagMat(MatrixXd::Ones(1, _N - i), -i), powMat(_A, i) * _B);
    }
    MatrixXd Cbar = MatrixXd::Zero(Cbar_tmp.rows(), Cbar_tmp.cols());
    Cbar.bottomRightCorner(Cbar.rows() - _nx, Cbar.cols() - _nu) = Cbar_tmp.block(0, 0, Cbar.rows() - _nx, Cbar.cols() - _nu);

    MatrixXd Abar = MatrixXd::Zero(_nx * _N, _nx);
    for (int i = 0; i < _N; i++)
    {
        Abar.block(_nx * i, 0, _nx, _nx) = powMat(_A, i);
    }

    MatrixXd Cw(_N * (_nx + _nu), _nu * _N);
    Cw.setZero();

    Cw.block(0, 0, _N * _nx, Cw.cols()) = Cbar * kronProduct(MatrixXd::Ones(_N, _N).triangularView<StrictlyLower>(), MatrixXd::Identity(_nu, _nu));
    Cw.block(_nx * _N, 0, _N * _nu, Cw.cols()) = kronProduct(MatrixXd::Ones(_N, _N).triangularView<StrictlyLower>(), MatrixXd::Identity(_nu, _nu));

    MatrixXd Aw(_N * (_nx + _nu), _nu + _nx);
    Aw.setZero();
    Aw.block(0, 0, _N * _nx, _nx) = Abar;
    Aw.block(0, _nx, _N * _nx, _nu) = Cbar * kronProduct(MatrixXd::Ones(_N, 1), MatrixXd::Identity(_nu, _nu));
    Aw.block(_N * _nx, 0, _N * _nu, _nx) = MatrixXd::Zero(_N * _nu, _nx);
    Aw.block(_N * _nx, _nx, _N * _nu, _nu) = kronProduct(MatrixXd::Ones(_N, 1), MatrixXd::Identity(_nu, _nu));

    MatrixXd Phi(_N * (_nu + _nx), _N * (3 * _nu + 2 * _nx));
    Phi.setZero();
    Phi.block(0, 0, _N * (_nu + _nx), _N * (_nu + _nx)) = MatrixXd::Identity(_N * (_nu + _nx), _N * (_nu + _nx));
    Phi.block(0, _N * (_nu + _nx), _N * (_nu + _nx), _N * _nu) = MatrixXd::Identity(_N * (_nu + _nx), _N * _nu);
    Phi.block(0, _N * (2 * _nu + _nx), _N * (_nu + _nx), _N * (_nu + _nx)) = -MatrixXd::Identity(_N * (_nu + _nx), _N * (_nu + _nx));

    MatrixXd Psi(_N * (3 * _nu + 2 * _nx), _N * _nu + (_N + 1) * (_nu + _nx));
    Psi.setZero();
    Psi.block(0, 0, _N * _nu, _N * _nu) = MatrixXd::Identity(_N * _nu, _N * _nu);
    Psi.block(_N * _nu, 0, Cw.rows(), Cw.cols()) = Cw;
    Psi.block(_N * _nu, Cw.cols(), Aw.rows(), Aw.cols()) = Aw;
    Psi.block(_N * (2 * _nu + _nx), _N * _nu + _nx + _nu, _N * (_nu + _nx), _N * (_nu + _nx)) = MatrixXd::Identity(_N * (_nx + _nu), _N * (_nx + _nu));

    MatrixXd Q_(_N * (_nx + _nu), _N * (_nx + _nu));
    Q_.setZero();
    Q_.block(0, 0, _N * _nu, _N * _nu) = Rbar;
    Q_.block(_N * _nu, 0, _N * _nx, _N * _nu) = Sbar;
    Q_.block(0, _N * _nu, _N * _nu, _N * _nx) = Sbar.transpose();
    Q_.block(_N * _nu, _N * _nu, _N * _nx, _N * _nx) = Qbar;

    // 22
    MatrixXd W_ = Psi.transpose() * Phi.transpose() * Q_ * Phi * Psi;
    MatrixXd H_tmp = W_.block(0, 0, _N * _nu, _N * _nu) / 2 + W_.block(0, 0, _N * _nu, _N * _nu).transpose() / 2;
    _F = W_.block(_N * _nu, 0, (_N + 1) * (_nu + _nx), _N * _nu);

    // softening constraints
    _H = Eigen::MatrixXd(H_tmp.rows() + _nSlack, H_tmp.cols() + _nSlack);
    for (int i = 0; i < _nSlack; i++)
        _H(i, i) = _costSlack[i];
    _H.topRightCorner(_nSlack, H_tmp.cols()) = Eigen::MatrixXd::Zero(_nSlack, H_tmp.cols());
    _H.bottomLeftCorner(H_tmp.rows(), _nSlack) = Eigen::MatrixXd::Zero(H_tmp.rows(), _nSlack);
    _H.bottomRightCorner(H_tmp.rows(), H_tmp.cols()) = H_tmp;

    _H = (_H + _H.transpose()) / 2;

    /* Constraints */
    MatrixXd Ibar_tmp(_N * (_nu + _nx), _N * (_nu + _nx));
    Ibar_tmp.setZero();
    Ibar_tmp.block(0, 0, _nx, _nx) = MatrixXd::Zero(_nx, _nx);
    Ibar_tmp.block(_nx, _nx, (_N - 1) * _nx, (_N - 1) * _nx) = kronProduct(MatrixXd::Identity(_N - 1, _N - 1), diagMat(_selectX, 0));
    Ibar_tmp.block(_N * _nx, _N * _nx, _N * _nu, _N * _nu) = kronProduct(MatrixXd::Identity(_N, _N), diagMat(_selectu, 0));

    MatrixXd Ibar(2 * _N * (_nu + _nx), _N * (_nu + _nx));
    MatrixXd Ibar_sel(2 * _N * (_nu + _nx), 2 * _N * (_nu + _nx));
    Ibar << Ibar_tmp, -Ibar_tmp;
    Ibar_sel.topLeftCorner(Ibar_tmp.rows(), Ibar_tmp.cols()) = Ibar_tmp;
    Ibar_sel.topRightCorner(Ibar_tmp.rows(), Ibar_tmp.cols()) = Eigen::MatrixXd::Zero(Ibar_tmp.rows(), Ibar_tmp.cols());
    Ibar_sel.bottomLeftCorner(Ibar_tmp.rows(), Ibar_tmp.cols()) = Eigen::MatrixXd::Zero(Ibar_tmp.rows(), Ibar_tmp.cols());
    Ibar_sel.bottomRightCorner(Ibar_tmp.rows(), Ibar_tmp.cols()) = Ibar_tmp;

    Eigen::MatrixXd Gp = Ibar * Cw;

    _Gp = Eigen::MatrixXd(Gp.rows() + _nSlack, Gp.cols() + _nSlack);
    _Gp.topLeftCorner(_nSlack, _nSlack) = -Eigen::MatrixXd::Identity(_nSlack, _nSlack);
    _Gp.topRightCorner(_nSlack, Gp.cols()) = Eigen::MatrixXd::Zero(_nSlack, Gp.cols());
    Eigen::MatrixXd M = Ibar_sel * kronProduct(Eigen::MatrixXd::Ones(Gp.rows() / _nSlack, 1), -Eigen::MatrixXd::Identity(_nSlack, _nSlack));
    _Gp.bottomLeftCorner(Gp.rows(), _nSlack) = M;
    _Gp.bottomRightCorner(Gp.rows(), Gp.cols()) = Gp;

    _Sp = -Ibar * Aw;

    _Wp_sup = MatrixXd::Zero(_N * (_nu + _nx), 1);
    _Wp_inf = MatrixXd::Zero(_N * (_nu + _nx), 1);

    _Wp_inf.block(0, 0, _N * _nx, 1) = kronProduct(MatrixXd::Ones(_N, 1), _Xmin);
    _Wp_inf.block(_N * _nx, 0, _N * _nu, 1) = kronProduct(MatrixXd::Ones(_N, 1), _umin);
    _Wp_inf = Ibar_tmp * _Wp_inf;
    _Wp_sup.block(0, 0, _N * _nx, 1) = kronProduct(MatrixXd::Ones(_N, 1), _Xmax);
    _Wp_sup.block(_N * _nx, 0, _N * _nu, 1) = kronProduct(MatrixXd::Ones(_N, 1), _umax);
    _Wp_sup = Ibar_tmp * _Wp_sup;

    _lb = MatrixXd::Zero(_Gp.rows() + _nSlack, 1);
    _ub = MatrixXd::Zero(_Gp.rows() + _nSlack, 1);
    _lb << 0.0 * MatrixXd::Ones(_nSlack, 1), -1 * MatrixXd::Ones(_Gp.rows(), 1);
    _ub << 10.0 * MatrixXd::Ones(_nSlack, 1), 1 * MatrixXd::Ones(_Gp.rows(), 1);

    _g_tmp = MatrixXd::Zero(1, (_N + 1) * (_nx + _nu));
    _g = MatrixXd::Zero(1, _N * _nu + _nSlack);
    _W = MatrixXd::Zero(2 * _Wp_sup.rows(), 1);
    _ubA = MatrixXd::Zero(_W.rows() + _nSlack, 1);
}
/*--------------------------------------------------------------------------------*/
void MPIC::firstSolveMPIC(MatrixXd X0, MatrixXd r0)
{

    _QPfail = false;
    _g_tmp << X0.transpose(), _uOpt.transpose(), r0.transpose();
    _g << 0.0 * MatrixXd::Ones(1, _nSlack), _g_tmp * _F;

    _W << _Wp_sup, -_Wp_inf;
    _ubA << 0.0 * MatrixXd::Ones(_nSlack, 1), _W + _Sp * _g_tmp.block(0, 0, 1, _nx + _nu).transpose();

    //    _lb << 0.0,-100*MatrixXd::Ones(_Gp.rows(),1);
    //    _ub << 10.0,100*MatrixXd::Ones(_Gp.rows(),1);

    int_t nWSR = 10000;

    try
    {
        //        if(_qpb.init( _H.data(),_g.data(),_Gp.data(),_lb.data(),_ub.data(),nullptr,_ubA.data(), nWSR ) != SUCCESSFUL_RETURN) throw std::string("initial QP not solved");
        if (_qpb.init(_H.data(), _g.data(), _Gp.data(), nullptr, nullptr, nullptr, _ubA.data(), nWSR) != SUCCESSFUL_RETURN)
            throw std::string("initial QP not solved");
    }
    catch (std::string e)
    {
        std::cerr << e << std::endl;
        _qpb.printProperties();
        _QPfail = true;
    }

    //    _qpb.printProperties();
    _duOpt.setZero();

    if (!_QPfail)
    {
        _qpb.getPrimalSolution(_qpSolution);
        for (int i = 0; i < _nSlack; i++)
            _valueSlack[i] = _qpSolution[i];
        for (int i = 0; i < _nu; i++)
            _duOpt(i, 0) = _qpSolution[i + _nSlack];
    }
    _uOpt += _duOpt;
    _Xmodel = _A * X0 + _B * _uOpt;
}
/*--------------------------------------------------------------------------------*/
void MPIC::updateSolveMPIC(MatrixXd X, MatrixXd rk)
{
    // softening constraints

    _QPfail = false;
    _g_tmp << X.transpose(), _uOpt.transpose(), rk.transpose();
    _g << 0.0 * MatrixXd::Ones(1, _nSlack), _g_tmp * _F;

    _W << _Wp_sup, -_Wp_inf;
    _ubA << 0.0 * MatrixXd::Ones(_nSlack, 1), _W + _Sp * _g_tmp.block(0, 0, 1, _nx + _nu).transpose();

    _lb << 0.0 * MatrixXd::Ones(_nSlack, 1), -0.1 * MatrixXd::Ones(_Gp.rows(), 1);
    _ub << 10.0 * MatrixXd::Ones(_nSlack, 1), 0.1 * MatrixXd::Ones(_Gp.rows(), 1);

    int_t nWSR = 1000000000;

    try
    {
        //        if(_qpb.hotstart(_g.data(),_lb.data(),_ub.data(),nullptr,_ubA.data(), nWSR )!= SUCCESSFUL_RETURN) throw std::string("hotstart QP not solved");
        //        if(_qpb.hotstart(_g.data(),nullptr,nullptr,nullptr,_ubA.data(), nWSR )!= SUCCESSFUL_RETURN) throw std::string("hotstart QP not solved");
        // if(_qpb.init( _H.data(),_g.data(),_Gp.data(),_lb.data(),_ub.data(),nullptr,_ubA.data(), nWSR ) != SUCCESSFUL_RETURN) throw std::string("QP not solved");
        if (_qpb.init(_H.data(), _g.data(), _Gp.data(), nullptr, nullptr, nullptr, _ubA.data(), nWSR) != SUCCESSFUL_RETURN)
            throw std::string("QP not solved");
    }
    catch (std::string e)
    {
        std::cerr << e << std::endl;
        _qpb.printProperties();
        _QPfail = true;
    }
    _duOpt.setZero();
    if (!_QPfail)
    {
        _qpb.getPrimalSolution(_qpSolution);
        for (int i = 0; i < _nSlack; i++)
            _valueSlack[i] = _qpSolution[i];
        for (int i = 0; i < _nu; i++)
            _duOpt(i) = _qpSolution[i + _nSlack];
    }
    _uOpt += _duOpt;
    _Xmodel = _A * _Xmodel + _B * _uOpt;
    std ::cout << "x model: \n"
               << _Xmodel << std::endl;
}
/*--------------------------------------------------------------------------------*/
MatrixXd MPIC::getuOpt()
{
    return _uOpt;
}
/*--------------------------------------------------------------------------------*/
void MPIC::addConstraintsX(MatrixXd selectX, MatrixXd Xmax, MatrixXd Xmin)
{
    _selectX = selectX;
    _Xmax = Xmax;
    _Xmin = Xmin;
}
/*--------------------------------------------------------------------------------*/
void MPIC::addConstraintsU(MatrixXd selectu, MatrixXd umax, MatrixXd umin)
{
    _selectu = selectu;
    _umax = umax;
    _umin = umin;
}
/*--------------------------------------------------------------------------------*/
void MPIC::setHorizon(int N)
{
    _N = N;
}
/*--------------------------------------------------------------------------------*/
void MPIC::setTimeStep(double Ts)
{
    _Ts = Ts;
}

/*--------------------------------------------------------------------------------*/
int MPIC::getHorizon()
{
    return _N;
}

/*--------------------------------------------------------------------------------*/
int MPIC::getDimU()
{
    return _nu;
}
/*--------------------------------------------------------------------------------*/
int MPIC::getDimX()
{
    return _nx;
}
/*--------------------------------------------------------------------------------*/
double MPIC::getSlack(int j)
{
    return _valueSlack[j];
}
/*--------------------------------------------------------------------------------*/
MatrixXd MPIC::getXmodel()
{
    return _Xmodel;
}

MPIController::MPIController(double sampling_time)
{
    _Ts = sampling_time;
}

void MPIController::on_init()
{
    // definition of the parameters that need to be queried from the
    // controller configuration file with default values
    std::vector<std::string> joints(1, "joint1");
    std::vector<std::double_t> stiffness;
    double damping_ratio = 1;
    std::vector<double> mass;
    // int control_horizon = 1;
    double sampling_time = _Ts;
    // auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    // auto_declare<std::vector<double>>("stiffness", std::vector<double>());
    // auto_declare<double>("damping_ratio", 1);
    // auto_declare<std::vector<double>>("mass", std::vector<double>());
    // auto_declare<int>("control_horizon", 1);
    // auto_declare<double>("sampling_time", 0);
}

void MPIController::on_configure()
{
    // getting the names of the joints to be controlled
    joint_names_.push_back("joint1");

    if (joint_names_.empty())
    {
        std::cout << "'joints' parameter was empty" << std::endl;
    }

    // getting the impedance parameters
    std::vector<std::double_t> stiffness;
    std::vector<double> mass;
    double damping_ratio = 0.7;
    int control_horizon = 5;

    // k
    if (stiffness.empty())
        stiffness.resize(joint_names_.size(), 10.0);
    // m
    if (mass.empty())
        mass.resize(joint_names_.size(), 1.0);
    if (damping_ratio < 0)
        damping_ratio = 0.5;

    stiffnessMat_ = Eigen::MatrixXd::Identity(joint_names_.size(), joint_names_.size());
    massMat_ = Eigen::MatrixXd::Identity(joint_names_.size(), joint_names_.size());
    dampingMat_ = Eigen::MatrixXd::Identity(joint_names_.size(), joint_names_.size());

    for (int i = 0; i < joint_names_.size(); i++)
    {
        stiffnessMat_(i, i) = stiffness[i];
        massMat_(i, i) = mass[i];
        // TODO: 阻尼根据前三个参数计算
        dampingMat_(i, i) = 2 * damping_ratio * sqrt(stiffnessMat_(i, i) / massMat_(i, i));
        // dampingMat_(i, i) = 35;
        cout << "---------------\n"
             << " dampingMat_: \n"
             << dampingMat_(i, i) << endl;
    }
    // TODO: u是关节数目？
    int nu = joint_names_.size();
    int nx = 3 * joint_names_.size();
    int N = control_horizon;

    Eigen::MatrixXd K(nu, nx);
    K.block(0, 0, nu, nu) = massMat_.block(0, 0, nu, nu).inverse() * dampingMat_.block(0, 0, nu, nu);
    K.block(0, nu, nu, nu) = massMat_.block(0, 0, nu, nu).inverse() * stiffnessMat_.block(0, 0, nu, nu);
    // TODO: 与论文给出公式11不一样
    K.block(0, 2 * nu, nu, nu) = massMat_.block(0, 0, nu, nu).inverse() * Eigen::MatrixXd::Identity(nu, nu);
    cout << "---------------\n"
         << " K: \n"
         << K << endl;
    // TODO: A B 的离散化？
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx, nx);
    A.block(nu, 0, nu, nu) = _Ts * Eigen::MatrixXd::Identity(nu, nu);
    cout << "---------------\n"
         << " A: \n"
         << A << endl;
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx, nu);
    B.block(0, 0, nu, nu) = _Ts * Eigen::MatrixXd::Identity(nu, nu);
    cout << "---------------\n"
         << " B: \n"
         << B << endl;
    Eigen::MatrixXd selectX = Eigen::MatrixXd::Zero(nx, 1);
    Eigen::MatrixXd selectu = Eigen::MatrixXd::Zero(nu, 1);

    Eigen::MatrixXd Xmax = Eigen::MatrixXd::Zero(nx, 1);
    Eigen::MatrixXd Xmin = Eigen::MatrixXd::Zero(nx, 1);
    Eigen::MatrixXd umax = Eigen::MatrixXd::Zero(nu, 1);
    Eigen::MatrixXd umin = Eigen::MatrixXd::Zero(nu, 1);

    for (auto i = 0ul; i < joint_names_.size(); i++)
    {
        // TODO: force limits
        std::vector<double> plimits = {};
        std::vector<double> vlimits = {};
        std::vector<double> alimits = {};

        if (!vlimits.empty())
        {
            Xmax(i, 0) = vlimits[1];
            Xmin(i, 0) = vlimits[0];
        }
        if (!plimits.empty())
        {
            selectX(nu + i, 0) = 1;
            Xmax(nu + i, 0) = plimits[1];
            Xmin(nu + i, 0) = plimits[0];
        }
        if (!alimits.empty())
        {
            selectu(i, 0) = 1;
            umax(i, 0) = alimits[1];
            umin(i, 0) = alimits[0];
        }
    }

    mpic_ = new MPIC(nx, nu, N);
    mpic_->setTimeStep(_Ts);
    mpic_->setSystem(A, B, K);
    mpic_->addConstraintsX(selectX, Xmax, Xmin);
    mpic_->addConstraintsU(selectu, umax, umin);
    mpic_->computeQP();

    stateX_ = Eigen::VectorXd::Zero(mpic_->getDimX());
    rk_ = MatrixXd::Zero(mpic_->getHorizon() * (mpic_->getDimU() + mpic_->getDimX()), 1);
    // ur H*1
    rku_tmp_ = MatrixXd::Zero(mpic_->getDimU() * mpic_->getHorizon(), 1);
    // xr H*3
    rkX_tmp_ = MatrixXd::Zero(mpic_->getDimX() * mpic_->getHorizon(), 1);
    uOpt_ = MatrixXd::Zero(mpic_->getDimU(), 1);

    solved_first_ = false;
    std::cout << "configure successful" << std::endl;
}

// main control loop function getting the state interface and writing to the command interface
void MPIController::update()
{
    cerr << "update" << endl;
    double vout = 0;
    double xref = 0;
    boost::shared_ptr<nav_msgs::Odometry const> shared_ptr;
    nav_msgs::Odometry odom;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 50); // 指定范围为1到10
    int random_number = dis(gen);

    int i = 0;
    while (1)
    
        random_number = dis(gen);
        cout << "--------------------------------------" << endl;
        i += 1;
        // TODO: updating reference for prediction
        for (int i = 0; i < mpic_->getHorizon(); i++)
        {
            rku_tmp_(i) = 0;
            // rkX_tmp_ : x',x,f;x' x f;
            rkX_tmp_(i) = 0.3;
            xref = xref + 0.3 * _Ts * i;
            rkX_tmp_(i + 1) = xref;
            rkX_tmp_(i + 2) = 0;
        }
        std::cout << "x ref: \n"
                  << xref << std::endl;
        rk_ << rku_tmp_, rkX_tmp_;

        // Model Predictive Impedance Control loop
        Eigen::VectorXd q = Eigen::VectorXd::Zero(joint_names_.size());
        Eigen::VectorXd qv = Eigen::VectorXd::Zero(joint_names_.size());
        Eigen::VectorXd te = Eigen::VectorXd::Zero(joint_names_.size());

        shared_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", nh, ros::Duration(10));
        odom = *shared_ptr;

        for (auto index = 0ul; index < joint_names_.size(); ++index)
        {
            // the stats are given in the same order as defines in state_interface_configuration
            // TODO:  机器人位置，速度，力
            q(index) = -odom.pose.pose.position.x;
            qv(index) = odom.twist.twist.linear.x;

            te(index) = 0;
            // te(index) = 10 + i / 10;
            // te(index) = random_number;
        }

        stateX_.head(mpic_->getDimU()) = qv.head(mpic_->getDimU());
        stateX_.segment(mpic_->getDimU(), mpic_->getDimU()) = q.head(mpic_->getDimU());
        stateX_.tail(mpic_->getDimU()) = te.head(mpic_->getDimU());
        std::cout << fixed << setprecision(3) << "state -x:\n " << stateX_ << std::endl;
        if (!solved_first_)
        {
            mpic_->firstSolveMPIC(stateX_, rk_);
            solved_first_ = true;
        }
        else
            mpic_->updateSolveMPIC(stateX_, rk_);

        if (mpic_->_QPfail)
            std::cerr << "MPIC: failed solving QP" << std::endl;
        else
            uOpt_ = mpic_->getuOpt();

        // TODO:u_opt输出 
        for (auto index = 0ul; index < joint_names_.size(); ++index)
        {
            geometry_msgs::Twist msg;
            vout += uOpt_(index) * _Ts;
            std::cout << "vout: \n"
                      << vout << std::endl;
            msg.linear.x = vout; // Set the linear velocity
            msg.linear.y = 0;    // Set the linear velocity
            msg.angular.z = 0;   // Set the angular velocity
            pub.publish(msg);
            ros::Duration(_Ts).sleep();
            // msg.linear.x = 0;
            // pub.publish(msg);
            ros::spinOnce();
        }
        std::ofstream file("/home/nianfei/work_dict/dog_ws/src/mpic/src/data.txt", std::ios::app);
        if (file.is_open())
        {
            // 写入数据到文件
            file << i << " " << 0.3 << " " << vout << " " << 10 + i / 10 << std::endl;

            // 关闭文件
            file.close();
        }
        else
        {
            std::cout << "无法打开文件" << std::endl;
        }
    }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpic");
    double ts = 0.01;
    MPIController mpi_ctrl(ts);
    mpi_ctrl.on_init();
    mpi_ctrl.on_configure();
    mpi_ctrl.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(int(ts * 1000)));
}