#include "nmpc.hpp"
#include "unsupported/Eigen/KroneckerProduct"

MPC::MPC(int N, double dT, double maxWheelSpeed, double wheelWidth, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd Qf)
{
    this->N = N;
    this->dT = dT;
    this->Q = Q;
    this->R = R;
    this->Qf = Qf;

    // check if dummy data is needed
    x_dummy = Eigen::VectorXd::Zero(N);
    y_dummy = Eigen::VectorXd::Zero(N);
    theta_dummy = Eigen::VectorXd::Zero(N);
    v_dummy = Eigen::VectorXd::Zero(N);
    w_dummy = Eigen::VectorXd::Zero(N);

    x_err_ref = Eigen::VectorXd::Zero(N);
    y_err_ref = Eigen::VectorXd::Zero(N);
    theta_err_ref = Eigen::VectorXd::Zero(N);

    A_hat = Eigen::MatrixXd::Zero(3, 3);
    B_hat = Eigen::MatrixXd::Zero(3, 2);

    A_ba = Eigen::MatrixXd::Zero(N*3, 3);
    B_ba = Eigen::MatrixXd::Zero(N*3, 2*N);

    // i=0 : A_hat^0
    for (int i = 0; i < N+1; i++)
    {
        A_hat_power.push_back(Eigen::MatrixXd::Identity(3, 3));
    }

    data = (OSQPData *)c_malloc(sizeof(OSQPData));
    if (data == nullptr)
    {
        std::cerr << "c_malloc failed to allocate memory for data" << std::endl;
        return;
    }
    data->A = nullptr;
    data->P = nullptr;
    data->q = nullptr;
    data->l = nullptr;
    data->u = nullptr;

    settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    if (settings == nullptr)
    {
        std::cerr << "c_malloc failed to allocate memory for settings" << std::endl;
        return;
    }
    osqp_set_default_settings(settings);
    settings->verbose = false;

    data->n = 2 * N;
    data->m = 2 * N;

    Eigen::MatrixXd tempP = Eigen::MatrixXd::Identity(2 * N, 2 * N);
    createOsqpSparseMatrix(tempP, data->P);
    data->q = (c_float *)c_malloc(data->n * sizeof(c_float));

    Eigen::MatrixXd a0 = Eigen::MatrixXd::Zero(2, 2);
    a0 << 1, wheelWidth/2, 1, -wheelWidth/2;
    // a0 << 1, 0, 1, 0;
    Eigen::MatrixXd A0 = Eigen::kroneckerProduct(
        Eigen::MatrixXd::Identity(N, N),
        a0
    );
    Eigen::SparseMatrix<double> A0_sparse = A0.sparseView();
    createOsqpSparseMatrix(A0, data->A);
    data->l = (c_float *)c_malloc(data->m * sizeof(c_float));
    for (int i = 0; i < N; i++)
    {
        data->l[i*2] = -maxWheelSpeed;
        data->l[i*2+1] = -maxWheelSpeed;
    }
    data->u = (c_float *)c_malloc(data->m * sizeof(c_float));
    for (int i = 0; i < N; i++)
    {
        data->u[i*2] = maxWheelSpeed;
        data->u[i*2+1] = maxWheelSpeed;
    }

    c_int ret = osqp_setup(&work, data, settings);

    x_ref = Eigen::VectorXd();
    y_ref = Eigen::VectorXd();
    theta_ref = Eigen::VectorXd();
    v_ref = Eigen::VectorXd();
    w_ref = Eigen::VectorXd();
    enableControl = false;
    current_iter = 0;
}

MPC::~MPC()
{
    if (data != nullptr)
    {
        if (data->A != nullptr)
        {
            csc_spfree(data->A);
        }
        if (data->P != nullptr)
        {
            csc_spfree(data->P);
        }
        if (data->q != nullptr)
        {
            c_free(data->q);
        }
        if (data->l != nullptr)
        {
            c_free(data->l);
        }
        if (data->u != nullptr)
        {
            c_free(data->u);
        }
        c_free(data);
    }

    if (settings != nullptr)
    {
        c_free(settings);
    }

    if (work != nullptr)
    {
        osqp_cleanup(work);
    }
}

void MPC::setTrackReference(const Eigen::VectorXd &x, 
    const Eigen::VectorXd &y, 
    const Eigen::VectorXd &theta,
    const Eigen::VectorXd &v,
    const Eigen::VectorXd &w)
{
    x_ref = x;
    y_ref = y;
    theta_ref = theta;
    v_ref = v;
    w_ref = w;
    enableControl = true;
    current_iter = 0;
}

bool MPC::update(const Eigen::VectorXd &state, Eigen::VectorXd &control)
{
    static int waitCount = 0;
    // Check if control is enabled
    if (!enableControl)
    {
        control = Eigen::VectorXd::Zero(2);
        return false;
    }
    
    
    for (int i = 0; i < N; i++)
    {
        int indx = current_iter + i;
        if (indx >= x_ref.size()) indx = x_ref.size() - 1;
        x_dummy(i) = x_ref(indx);
        y_dummy(i) = y_ref(indx);
        theta_dummy(i) = theta_ref(indx);
        v_dummy(i) = v_ref(indx);
        w_dummy(i) = w_ref(indx);
    }

    for (int i = 0; i < N; i++)
    {
        x_err_ref(i) = x_dummy(i) - x_dummy(0);
        y_err_ref(i) = y_dummy(i) - y_dummy(0);
        theta_err_ref(i) = theta_dummy(i) - theta_dummy(0);
        if (theta_err_ref(i) > M_PI)
        {
            theta_err_ref(i) -= 2 * M_PI;
        }
        if (theta_err_ref(i) < -M_PI)
        {
            theta_err_ref(i) += 2 * M_PI;
        }
    }

    /// update osqp data
    setup(state);
    // solve osqp
    control = Eigen::VectorXd::Zero(2);
    if (osqp_solve(work) == 0)
    {
        control(0) = work->solution[0].x[0];
        control(1) = work->solution[0].x[1];
    }
    
    // update current iteration
    waitCount++;
    if (waitCount >= 1)
    {
        waitCount = 0;
        current_iter++;
        if (current_iter >= x_ref.size())
        {
            enableControl = false;
            current_iter = 0;
            return true;
        }
    }
    
    
    return false;
}

void MPC::abort()
{
    x_ref = Eigen::VectorXd();
    y_ref = Eigen::VectorXd();
    theta_ref = Eigen::VectorXd();
    v_ref = Eigen::VectorXd();
    w_ref = Eigen::VectorXd();
    enableControl = false;
    current_iter = 0;
}

void MPC::setup(const Eigen::VectorXd &state)
{
    
    A_hat   <<  1, 0, -state(3) * dT * sin(state(2)),
                0, 1, state(3) * dT * cos(state(2)),
                0, 0, 1;
    
    B_hat   <<  dT * cos(state(2)), 0,
                dT * sin(state(2)), 0,
                0, dT;
    
    // A_hat_power.push_back(Eigen::MatrixXd::Identity(3, 3)); // A_hat^0
    for (int i = 1; i < N+1; i++)
    {
        A_hat_power.at(1) = (A_hat_power[i-1] * A_hat);
    }

    

    for (int i = 0; i < N; i++)
    {
        A_ba.block(i*3, 0, 3, 3) = A_hat_power[i+1];
        for (int j = 0; j < i+1; j++)
        {
            B_ba.block(i*3, j*2, 3, 2) = A_hat_power[i-j] * B_hat;
        }
    }

    // 3N,3N
    Eigen::MatrixXd Q_bar = Eigen::kroneckerProduct(
        Eigen::MatrixXd::Identity(N, N),
        Q
    );
    Q_bar.block((N-1)*3,(N-1)*3,3,3) = Qf;
    // 2N,2N
    Eigen::MatrixXd R_bar = Eigen::kroneckerProduct(
        Eigen::MatrixXd::Identity(N, N),
        R
    );
    // std::cout << "A_ba: " << A_ba << std::endl;
    // std::cout << "B_ba: " << B_ba << std::endl;
    // std::cout << "Q_bar: " << Q_bar << std::endl;
    // std::cout << "R_bar: " << R_bar << std::endl;

    Eigen::MatrixXd e0 = Eigen::MatrixXd::Zero(3, 1);
    e0 << state(0) - x_dummy(0),
          state(1) - y_dummy(0),
          state(2) - theta_dummy(0);
    // e0 << state(0), state(1), state(2);
    // std::cout << x_dummy(0) << " " << y_dummy(0) << " " << theta_dummy(0) << std::endl;
    Eigen::MatrixXd sref = Eigen::MatrixXd::Zero(N*3, 1);
    for (int i = 0; i < N; i++)
    {
        sref.block(i*3, 0, 3, 1) << x_err_ref(i), y_err_ref(i), theta_err_ref(i);
    }
    // std::cout << "e0: " << e0 << std::endl;
    // std::cout << "sref: " << sref << std::endl;
    // 3N,1
    Eigen::MatrixXd E = A_ba * e0 - sref;
    // 2N,2N
    Eigen::MatrixXd P = 2 * (B_ba.transpose() * Q_bar * B_ba + R_bar);
    // 2N,1
    Eigen::MatrixXd q = 2 * B_ba.transpose() * Q_bar * E;
    // std::cout << "E: " << E << std::endl;
    // std::cout << "P: " << P << std::endl;
    // std::cout << "q: " << q << std::endl;
    // update data
    createOsqpSparseMatrix(P, data->P);
    data->P = csc_to_triu(data->P);

    for (int i = 0; i < q.size(); i++) {
        data->q[i] = q(i);
    }

    osqp_cleanup(work);
    osqp_setup(&work, data, settings);
}

bool MPC::createOsqpSparseMatrix(const Eigen::MatrixXd &matrix, csc *&osqp_matrix)
{
    Eigen::SparseMatrix<double> sparse_matrix = matrix.sparseView();
    sparse_matrix.makeCompressed();
    c_int rows = sparse_matrix.rows();
    c_int cols = sparse_matrix.cols();
    c_int nnz = sparse_matrix.nonZeros();

    const int* outerIndexPtr = sparse_matrix.outerIndexPtr();
    const int* innerNonZerosPtr = sparse_matrix.innerNonZeroPtr();

    if (osqp_matrix != nullptr)
    {
        csc_spfree(osqp_matrix);
    }

    osqp_matrix = csc_spalloc(rows, cols, nnz, 1, 0);

    int innerOsqpPosition = 0;
    for (int k = 0; k < cols; k++)
    {
        if (sparse_matrix.isCompressed())
        {
            osqp_matrix->p[k] = static_cast<c_int>(outerIndexPtr[k]);
        } 
        else
        {
            if (k == 0)
            {
                osqp_matrix->p[k] = 0;
            } 
            else
            {
                osqp_matrix->p[k] = osqp_matrix->p[k - 1] + innerNonZerosPtr[k - 1];
            }
        }
        for (typename Eigen::SparseMatrix<double>::InnerIterator it(sparse_matrix, k);
             it;
             ++it)
        {
            osqp_matrix->i[innerOsqpPosition] = static_cast<c_int>(it.row());
            osqp_matrix->x[innerOsqpPosition] = static_cast<c_float>(it.value());
            innerOsqpPosition++;
        }
    }
    osqp_matrix->p[static_cast<int>(cols)] = static_cast<c_int>(innerOsqpPosition);

    return false;
}
