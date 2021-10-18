/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
*
* Modifications: Jeremie Deray
******************************************************************************************** */

#include "RF2O.h"

void Blitz::RF2O::Initialize(Blitz::LaserScan& scan, GeometryMsgs::Pose& initial_robot_pose)
{
    cout << "[rf2o] Got first Laser Scan .... Configuring node" << endl;

    width = scan.points.size();
    cols = width;
    fovh = 359.0f;
    ctf_levels = 5;
    iter_irls  = 5;

    Pose3d robot_initial_pose = Pose3d::Identity();

    robot_initial_pose = Eigen::Quaterniond(initial_robot_pose.orientation.w,
                                            initial_robot_pose.orientation.x,
                                            initial_robot_pose.orientation.y,
                                            initial_robot_pose.orientation.z);

    robot_initial_pose.translation()(0) = initial_robot_pose.position.x;
    robot_initial_pose.translation()(1) = initial_robot_pose.position.y;

    cout << "[rf2o] Setting origin at:\n" << robot_initial_pose.matrix() << endl;

    laser_pose_    = robot_initial_pose * laser_pose_on_robot_;
    laser_oldpose_ = laser_oldpose_;
    range_wf = Eigen::MatrixXf::Constant(1, width, 1);

    transformations.resize(ctf_levels);
    for (unsigned int i = 0; i < ctf_levels; i++)
    transformations[i].resize(3, 3);

    unsigned int s, cols_i;
    const unsigned int pyr_levels = std::round(std::log2(round(float(width) / float(cols)))) + ctf_levels;
    range.resize(pyr_levels);
    range_old.resize(pyr_levels);
    range_inter.resize(pyr_levels);
    xx.resize(pyr_levels);
    xx_inter.resize(pyr_levels);
    xx_old.resize(pyr_levels);
    yy.resize(pyr_levels);
    yy_inter.resize(pyr_levels);
    yy_old.resize(pyr_levels);
    range_warped.resize(pyr_levels);
    xx_warped.resize(pyr_levels);
    yy_warped.resize(pyr_levels);

    for (unsigned int i = 0; i < pyr_levels; i++)
    {
        s = std::pow(2.f, int(i));
        cols_i = std::ceil(float(width) / float(s));

        range[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
        range_old[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
        range_inter[i].resize(1, cols_i);

        xx[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
        xx_old[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);

        yy[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
        yy_old[i] = Eigen::MatrixXf::Constant(1, cols_i, 0.f);

        xx_inter[i].resize(1, cols_i);
        yy_inter[i].resize(1, cols_i);

        if (cols_i <= cols)
        {
            range_warped[i].resize(1, cols_i);
            xx_warped[i].resize(1, cols_i);
            yy_warped[i].resize(1, cols_i);
        }
    }

    dt.resize(1, cols);
    dtita.resize(1, cols);
    normx.resize(1, cols);
    normy.resize(1, cols);
    norm_ang.resize(1, cols);
    weights.resize(1, cols);

    null    = Eigen::MatrixXi::Constant(1, cols, 0);
    cov_odo = IncrementCov::Zero();

    fps = 1.f;
    num_valid_range = 0;

    g_mask[0] = 1.f / 16.f;
    g_mask[1] = 0.25f;
    g_mask[2] = 6.f / 16.f;
    g_mask[3] = g_mask[1];
    g_mask[4] = g_mask[0];

    kai_abs_     = MatrixS31::Zero();
    kai_loc_old_ = MatrixS31::Zero();

    module_initialized = true;
    last_odom_time = chrono::high_resolution_clock::now();
}

Pose3d& Blitz::RF2O::GetIncrement()
{
    return robot_pose_;
}

IncrementCov& Blitz::RF2O::GetIncrementCovariance()
{
  return cov_odo;
}

Pose3d& Blitz::RF2O::GetPose()
{
  return robot_pose_;
}

bool Blitz::RF2O::OdometryCalc(LaserScan& scan)
{
    range_wf = Eigen::Map<const Eigen::MatrixXf>(scan.GetData().data(), width, 1);

    Time start = chrono::high_resolution_clock::now();

    CreateImagePyramid();

    //Coarse-to-fine scheme
    for (unsigned int i=0; i<ctf_levels; i++)
    {
        //Previous computations
        transformations[i].setIdentity();

        level = i;
        unsigned int s = std::pow(2.f,int(ctf_levels-(i+1)));
        cols_i = std::ceil(float(cols)/float(s));
        image_level = ctf_levels - i + std::round(std::log2(std::round(float(width)/float(cols)))) - 1;

        //1. Perform warping
        if (i == 0)
        {
            range_warped[image_level] = range[image_level];
            xx_warped[image_level]    = xx[image_level];
            yy_warped[image_level]    = yy[image_level];
        }
        else
        {
            PerformWarping();
        }

        CalculateCoord();
        FindNullPoints();
        CalculaterangeDerivativesSurface();
        //computeNormals();
        ComputeWeights();

        if (num_valid_range > 3)
        {
            SolveSystemNonLinear();
            //solveSystemOneLevel();
        }
        else
        {
            continue;
        }

        if (!FilterLevelSolution())
        {
            return false;
        }
    }

    m_runtime = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start);

    //cout << "[rf2o] execution time (ms): " << m_runtime.count() << endl;

    PoseUpdate();

    return true;
}

void Blitz::RF2O::CreateImagePyramid()
{
    const float max_range_dif = 0.3f;

    range_old.swap(range);
    xx_old.swap(xx);
    yy_old.swap(yy);

    unsigned int pyr_levels = std::round(std::log2(std::round(float(width)/float(cols)))) + ctf_levels;

    for (unsigned int i = 0; i<pyr_levels; i++)
    {
        unsigned int s = std::pow(2.f,int(i));
        cols_i = std::ceil(float(width)/float(s));

        const unsigned int i_1 = i-1;
        if (i == 0)
        {
            for (unsigned int u = 0; u < cols_i; u++)
            {
                const float dcenter = range_wf(u);

                if ((u>1)&&(u<cols_i-2))
                {
                    if (dcenter > 0.f)
                    {
                        float sum = 0.f;
                        float weight = 0.f;

                        for (int l=-2; l<3; l++)
                        {
                            const float abs_dif = std::abs(range_wf(u+l)-dcenter);
                            if (abs_dif < max_range_dif)
                            {
                                const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                                weight += aux_w;
                                sum += aux_w*range_wf(u+l);
                            }
                        }
                        range[i](u) = sum/weight;
                    }
                    else
                    {
                        range[i](u) = 0.f;
                    }
                }
                else
                {
                    if (dcenter > 0.f)
                    {
                        float sum = 0.f;
                        float weight = 0.f;

                        for (int l=-2; l<3; l++)
                        {
                            const int indu = u+l;
                            if ((indu>=0)&&(indu<cols_i))
                            {
                                const float abs_dif = std::abs(range_wf(indu)-dcenter);
                                if (abs_dif < max_range_dif)
                                {
                                    const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                                    weight += aux_w;
                                    sum += aux_w*range_wf(indu);
                                }
                            }
                        }
                        range[i](u) = sum/weight;
                    }
                    else
                    {
                        range[i](u) = 0.f;
                    }
                }
            }
        }
        else
        {
            for (unsigned int u = 0; u < cols_i; u++)
            {
                const int u2 = 2*u;
                const float dcenter = range[i_1](u2);

                if ((u>0)&&(u<cols_i-1))
                {
                    if (dcenter > 0.f)
                    {
                        float sum = 0.f;
                        float weight = 0.f;

                        for (int l=-2; l<3; l++)
                        {
                            const float abs_dif = std::abs(range[i_1](u2+l)-dcenter);
                            if (abs_dif < max_range_dif)
                            {
                                const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                                weight += aux_w;
                                sum += aux_w*range[i_1](u2+l);
                            }
                        }
                        range[i](u) = sum/weight;
                    }
                    else
                    {
                        range[i](u) = 0.f;
                    }
                }

                //Boundary
                else
                {
                    if (dcenter > 0.f)
                    {
                        float sum = 0.f;
                        float weight = 0.f;
                        const unsigned int cols_i2 = range[i_1].cols();


                        for (int l=-2; l<3; l++)
                        {
                            const int indu = u2+l;
                            if ((indu>=0)&&(indu<cols_i2))
                            {
                                const float abs_dif = std::abs(range[i_1](indu)-dcenter);
                                if (abs_dif < max_range_dif)
                                {
                                    const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
                                    weight += aux_w;
                                    sum += aux_w*range[i_1](indu);
                                }
                            }
                        }
                        range[i](u) = sum/weight;
                    }
                    else
                    {
                        range[i](u) = 0.f;
                    }
                }
            }
        }
        for (unsigned int u = 0; u < cols_i; u++)
        {
            if (range[i](u) > 0.f)
            {
                const float tita = -0.5*fovh + float(u)*fovh/float(cols_i-1);
                xx[i](u) = range[i](u)*std::cos(tita);
                yy[i](u) = range[i](u)*std::sin(tita);
            }
            else
            {
                xx[i](u) = 0.f;
                yy[i](u) = 0.f;
            }
        }
    }
}

void Blitz::RF2O::CalculateCoord()
{
  for (unsigned int u = 0; u < cols_i; u++)
  {
    if ((range_old[image_level](u) == 0.f) || (range_warped[image_level](u) == 0.f))
    {
      range_inter[image_level](u) = 0.f;
      xx_inter[image_level](u)    = 0.f;
      yy_inter[image_level](u)    = 0.f;
    }
    else
    {
      range_inter[image_level](u) = 0.5f*(range_old[image_level](u) + range_warped[image_level](u));
      xx_inter[image_level](u)    = 0.5f*(xx_old[image_level](u)    + xx_warped[image_level](u));
      yy_inter[image_level](u)    = 0.5f*(yy_old[image_level](u)    + yy_warped[image_level](u));
    }
  }
}

void Blitz::RF2O::CalculaterangeDerivativesSurface()
{
    rtita = Eigen::MatrixXf::Constant(1, cols_i, 1.f);

    for (unsigned int u = 0; u < cols_i-1; u++)
    {
        float dista = xx_inter[image_level](u+1) - xx_inter[image_level](u);
        dista *= dista;
        float distb = yy_inter[image_level](u+1) - yy_inter[image_level](u);
        distb *= distb;
        const float dist = dista + distb;

        if (dist  > 0.f) {
            rtita(u) = std::sqrt(dist);
        }
    }

    for (unsigned int u = 1; u < cols_i-1; u++)
    {
        dtita(u) = (rtita(u-1)*(range_inter[image_level](u+1)-
                                range_inter[image_level](u)) + rtita(u)*(range_inter[image_level](u) -
                                range_inter[image_level](u-1)))/(rtita(u)+rtita(u-1));
    }


    dtita(0) = dtita(1);
    dtita(cols_i-1) = dtita(cols_i-2);

    for (unsigned int u = 0; u < cols_i; u++) {
        dt(u) = fps*(range_warped[image_level](u) - range_old[image_level](u));
    }
}

void Blitz::RF2O::ComputeNormals()
{
    normx.setConstant(1, cols, 0.f);
    normy.setConstant(1, cols, 0.f);
    norm_ang.setConstant(1, cols, 0.f);

    const float incr_tita = fovh/float(cols_i-1);
    for (unsigned int u=0; u<cols_i; u++)
    {
        if (null(u) == 0.f)
        {
            const float tita = -0.5f*fovh + float(u)*incr_tita;
            const float alfa = -std::atan2(2.f*dtita(u), 2.f*range[image_level](u)*incr_tita);
            norm_ang(u) = tita + alfa;
            if (norm_ang(u) < -M_PI)
            norm_ang(u) += 2.f*M_PI;
            else if (norm_ang(u) < 0.f)
            norm_ang(u) += M_PI;
            else if (norm_ang(u) > M_PI)
            norm_ang(u) -= M_PI;

            normx(u) = std::cos(tita + alfa);
            normy(u) = std::sin(tita + alfa);
        }
    }
}

void Blitz::RF2O::ComputeWeights()
{
    weights.setConstant(1, cols, 0.f);

    const float kdtita = 1.f;
    const float kdt = kdtita / (fps*fps);
    const float k2d = 0.2f;

    for (unsigned int u = 1; u < cols_i-1; u++)
    {
        if (null(u) == 0)
        {
            const float ini_dtita = range_old[image_level](u+1) - range_old[image_level](u-1);
            const float final_dtita = range_warped[image_level](u+1) - range_warped[image_level](u-1);

            const float dtitat = ini_dtita - final_dtita;
            const float dtita2 = dtita(u+1) - dtita(u-1);

            const float w_der = kdt*(dt(u)*dt(u)) +
                                kdtita*(dtita(u)*dtita(u)) +
                                k2d*(std::abs(dtitat) + std::abs(dtita2));

            weights(u) = std::sqrt(1.f/w_der);
        }
    }


    const float inv_max = 1.f / weights.maxCoeff();
    weights = inv_max*weights;
}

void Blitz::RF2O::FindNullPoints()
{
    num_valid_range = 0;
    
    for (unsigned int u = 1; u < cols_i-1; u++)
    {
        if (range_inter[image_level](u) == 0.f)
        {
            null(u) = 1;
        }
        else
        {
            num_valid_range++;
            null(u) = 0;
        }
    }
}

void Blitz::RF2O::SolveSystemOneLevel()
{
    A.resize(num_valid_range, 3);
    B.resize(num_valid_range, 1);

    unsigned int cont = 0;
    const float kdtita = (cols_i-1)/fovh;

    for (unsigned int u = 1; u < cols_i-1; u++)
    {
        if (null(u) == 0)
        {
            // Precomputed expressions
            const float tw = weights(u);
            const float tita = -0.5*fovh + u/kdtita;

            //Fill the matrix A
            A(cont, 0) = tw*(std::cos(tita) + dtita(u)*kdtita*std::sin(tita)/range_inter[image_level](u));
            A(cont, 1) = tw*(std::sin(tita) - dtita(u)*kdtita*std::cos(tita)/range_inter[image_level](u));
            A(cont, 2) = tw*(-yy[image_level](u)*std::cos(tita) + xx[image_level](u)*std::sin(tita) - dtita(u)*kdtita);
            B(cont, 0) = tw*(-dt(u));

            cont++;
        }
    }
    
    Eigen::MatrixXf AtA, AtB;
    AtA = A.transpose()*A;
    AtB = A.transpose()*B;
    Var = AtA.ldlt().solve(AtB);

    Eigen::MatrixXf res(num_valid_range,1);
    res = A*Var - B;
    cov_odo = (1.f/float(num_valid_range-3))*AtA.inverse()*res.squaredNorm();

    kai_loc_level_ = Var;
}

void Blitz::RF2O::SolveSystemNonLinear()
{
    A.resize(num_valid_range, 3); Aw.resize(num_valid_range, 3);
    B.resize(num_valid_range, 1); Bw.resize(num_valid_range, 1);
    unsigned int cont = 0;
    const float kdtita = float(cols_i-1)/fovh;

    for (unsigned int u = 1; u < cols_i-1; u++)
    {
        if (null(u) == 0)
        {
            // Precomputed expressions
            const float tw = weights(u);
            const float tita = -0.5*fovh + u/kdtita;

            //Fill the matrix A
            A(cont, 0) = tw*(std::cos(tita) + dtita(u)*kdtita*std::sin(tita)/range_inter[image_level](u));
            A(cont, 1) = tw*(std::sin(tita) - dtita(u)*kdtita*std::cos(tita)/range_inter[image_level](u));
            A(cont, 2) = tw*(-yy[image_level](u)*std::cos(tita) + xx[image_level](u)*std::sin(tita) - dtita(u)*kdtita);
            B(cont, 0) = tw*(-dt(u));

            cont++;
        }
    }
    
    Eigen::MatrixXf AtA, AtB;
    AtA = A.transpose()*A;
    AtB = A.transpose()*B;
    Var = AtA.ldlt().solve(AtB);

    Eigen::MatrixXf res(num_valid_range,1);
    res = A*Var - B;

    float aver_dt = 0.f, aver_res = 0.f; unsigned int ind = 0;
    for (unsigned int u = 1; u < cols_i-1; u++)
    {
        if (null(u) == 0)
        {
            aver_dt  += std::abs(dt(u));
            aver_res += std::abs(res(ind++));
        }
    }
    aver_dt /= cont; aver_res /= cont;

    const float k = 10.f/aver_dt;
    for (unsigned int i=1; i<=iter_irls; i++)
    {
        cont = 0;

        for (unsigned int u = 1; u < cols_i-1; u++)
        {
            if (null(u) == 0)
            {
                const float res_weight = std::sqrt(1.f/(1.f + ((k*res(cont))*(k*res(cont)))));

                //Fill the matrix Aw
                Aw(cont,0) = res_weight*A(cont,0);
                Aw(cont,1) = res_weight*A(cont,1);
                Aw(cont,2) = res_weight*A(cont,2);
                Bw(cont)   = res_weight*B(cont);
                cont++;
            }
        }
        
        AtA = Aw.transpose()*Aw;
        AtB = Aw.transpose()*Bw;
        Var = AtA.ldlt().solve(AtB);
        res = A*Var - B;
    }

    cov_odo = (1.f/float(num_valid_range-3))*AtA.inverse()*res.squaredNorm();
    kai_loc_level_ = Var;

    //cout << "[rf2o] COV_ODO:\n" << cov_odo << endl;
}

void Blitz::RF2O::Reset(Pose3d& ini_pose)
{
    laser_pose_    = ini_pose;
    laser_oldpose_ = ini_pose;

    CreateImagePyramid();
}

void Blitz::RF2O::PerformWarping()
{
    Eigen::Matrix3f acu_trans;
    acu_trans.setIdentity();
    for (unsigned int i=1; i<=level; i++)
    {
        acu_trans = transformations[i-1]*acu_trans;
    }

    Eigen::MatrixXf wacu = Eigen::MatrixXf::Constant(1, cols_i, 0.f);
    range_warped[image_level].setConstant(1, cols_i, 0.f);
    const float cols_lim = float(cols_i-1);
    const float kdtita = cols_lim / fovh;

    for (unsigned int j = 0; j < cols_i; j++)
    {
        if (range[image_level](j) > 0.f)
        {
            //Transform point to the warped reference frame
            const float x_w = acu_trans(0,0)*xx[image_level](j) + acu_trans(0,1)*yy[image_level](j) + acu_trans(0,2);
            const float y_w = acu_trans(1,0)*xx[image_level](j) + acu_trans(1,1)*yy[image_level](j) + acu_trans(1,2);
            const float tita_w = std::atan2(y_w, x_w);
            const float range_w = std::sqrt(x_w*x_w + y_w*y_w);

            //Calculate warping
            const float uwarp = kdtita*(tita_w + 0.5*fovh);

            //The warped pixel (which is not integer in general) contributes to all the surrounding ones
            if (( uwarp >= 0.f ) && ( uwarp < cols_lim ))
            {
                const int uwarp_l = uwarp;
                const int uwarp_r = uwarp_l + 1;
                const float delta_r = float(uwarp_r) - uwarp;
                const float delta_l = uwarp - float(uwarp_l);

                //Very close pixel
                if (std::abs(std::round(uwarp) - uwarp) < 0.05f)
                {
                    range_warped[image_level](round(uwarp)) += range_w;
                    wacu(std::round(uwarp)) += 1.f;
                }
                else
                {
                    const float w_r = delta_l*delta_l;
                    range_warped[image_level](uwarp_r) += w_r*range_w;
                    wacu(uwarp_r) += w_r;

                    const float w_l = delta_r*delta_r;
                    range_warped[image_level](uwarp_l) += w_l*range_w;
                    wacu(uwarp_l) += w_l;
                }
            }
        }
    }

    for (unsigned int u = 0; u<cols_i; u++)
    {
        if (wacu(u) > 0.f)
        {
            const float tita = -0.5f*fovh + float(u)/kdtita;
            range_warped[image_level](u) /= wacu(u);
            xx_warped[image_level](u) = range_warped[image_level](u)*std::cos(tita);
            yy_warped[image_level](u) = range_warped[image_level](u)*std::sin(tita);
        }
        else
        {
            range_warped[image_level](u) = 0.f;
            xx_warped[image_level](u) = 0.f;
            yy_warped[image_level](u) = 0.f;
        }
    }
}

bool Blitz::RF2O::FilterLevelSolution()
{
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(cov_odo);
    if (eigensolver.info() != Eigen::Success)
    {
        cout << "[rf2o] ERROR: Eigensolver couldn't find a solution. Pose is not updated" << endl;
        return false;
    }

    Eigen::Matrix<float,3,3> Bii;
    Eigen::Matrix<float,3,1> kai_b;
    Bii = eigensolver.eigenvectors();

    kai_b = Bii.colPivHouseholderQr().solve(kai_loc_level_);

    assert((kai_loc_level_).isApprox(Bii*kai_b, 1e-5) && "Ax=b has no solution." && __LINE__);

    MatrixS31 kai_loc_sub;

    Eigen::Matrix3f acu_trans;
    acu_trans.setIdentity();
    for (unsigned int i=0; i<level; i++)
    {
        acu_trans = transformations[i]*acu_trans;
    }

    kai_loc_sub(0) = -fps*acu_trans(0,2);
    kai_loc_sub(1) = -fps*acu_trans(1,2);
    if (acu_trans(0,0) > 1.f)
    {
        kai_loc_sub(2) = 0.f;
    }
    else
    {
        kai_loc_sub(2) = -fps*std::acos(acu_trans(0,0))*sign(acu_trans(1,0));
    }
    kai_loc_sub += kai_loc_old_;

    Eigen::Matrix<float,3,1> kai_b_old;
    kai_b_old = Bii.colPivHouseholderQr().solve(kai_loc_sub);

    assert((kai_loc_sub).isApprox(Bii*kai_b_old, 1e-5) && "Ax=b has no solution." && __LINE__);

    const float cf = 15e3f*std::exp(-float(int(level))),
                df = 0.05f*std::exp(-float(int(level)));

    Eigen::Matrix<float,3,1> kai_b_fil;
    for (unsigned int i=0; i<3; i++)
    {
        kai_b_fil(i) = (kai_b(i) + (cf*eigensolver.eigenvalues()(i,0) + df)*kai_b_old(i))/(1.f + cf*eigensolver.eigenvalues()(i,0) + df);
    }

    Eigen::Matrix<float, 3, 1> kai_loc_fil = Bii.inverse().colPivHouseholderQr().solve(kai_b_fil);

    assert((kai_b_fil).isApprox(Bii.inverse()*kai_loc_fil, 1e-5) && "Ax=b has no solution." && __LINE__);

    const float incrx = kai_loc_fil(0)/fps;
    const float incry = kai_loc_fil(1)/fps;
    const float rot   = kai_loc_fil(2)/fps;

    transformations[level](0,0) = std::cos(rot);
    transformations[level](0,1) = -std::sin(rot);
    transformations[level](1,0) = std::sin(rot);
    transformations[level](1,1) = std::cos(rot);
    transformations[level](0,2) = incrx;
    transformations[level](1,2) = incry;

    return true;
}

void Blitz::RF2O::PoseUpdate()
{
    // Compute Transformations
    Eigen::Matrix3f acu_trans;
    acu_trans.setIdentity();
    for (unsigned int i=1; i <= ctf_levels; i++)
    {
        acu_trans = transformations[i-1]*acu_trans;
    }

    // Compute kai_loc & kai_abs
    kai_loc_(0) = fps*acu_trans(0,2);
    kai_loc_(1) = fps*acu_trans(1,2);
    if (acu_trans(0,0) > 1.f)
        kai_loc_(2) = 0.f;
    else
    {
        kai_loc_(2) = fps*std::acos(acu_trans(0,0))*sign(acu_trans(1,0));
    }
    float phi = getYaw(laser_pose_.rotation());
    kai_abs_(0) = kai_loc_(0)*std::cos(phi) - kai_loc_(1)*std::sin(phi);
    kai_abs_(1) = kai_loc_(0)*std::sin(phi) + kai_loc_(1)*std::cos(phi);
    kai_abs_(2) = kai_loc_(2);

    // Update Poses
    laser_oldpose_ = laser_pose_;
    Pose3d pose_aux_2D = Pose3d::Identity();
    pose_aux_2D = matrixYaw(double(kai_loc_(2)/fps));
    pose_aux_2D.translation()(0) = acu_trans(0,2);
    pose_aux_2D.translation()(1) = acu_trans(1,2);
    laser_pose_ = laser_pose_ * pose_aux_2D;
    last_increment_ = pose_aux_2D;

    // Compute kai_loc_old
    phi = getYaw(laser_pose_.rotation());
    kai_loc_old_(0) =  kai_abs_(0)*std::cos(phi) + kai_abs_(1)*std::sin(phi);
    kai_loc_old_(1) = -kai_abs_(0)*std::sin(phi) + kai_abs_(1)*std::cos(phi);
    kai_loc_old_(2) =  kai_abs_(2);

    cout << "[rf2o] LASERodom = [" << laser_pose_.translation()(0) << laser_pose_.translation()(1) << getYaw(laser_pose_.rotation()) << "]" << endl;

    //Compose Transformations
    robot_pose_ = laser_pose_ * laser_pose_on_robot_inv_;

    cout << "BASEodom = [" << robot_pose_.translation()(0) << robot_pose_.translation()(1) << getYaw(robot_pose_.rotation()) << "]" << endl;

    // Estimate linear/angular speeds (mandatory for base_local_planner)
    // last_scan -> the last scan received
    // last_odom_time -> The time of the previous scan lasser used to estimate the pose
    //-------------------------------------------------------------------------------------
    std::chrono::duration<double, std::milli> dur = current_scan_time - last_odom_time;
    double time_inc_sec = dur.count() / 1000.0;
    last_odom_time = current_scan_time;
    lin_speed = acu_trans(0,2) / time_inc_sec;
    //double lin_speed = sqrt( mrpt::math::square(robot_oldpose.x()-robot_pose.x()) + mrpt::math::square(robot_oldpose.y()-robot_pose.y()) )/time_inc_sec;

    double ang_inc = getYaw(robot_pose_.rotation()) - getYaw(robot_oldpose_.rotation());

    if (ang_inc > 3.14159)
    {
        ang_inc -= 2*3.14159;
    }  
    if (ang_inc < -3.14159)
    {
        ang_inc += 2*3.14159;
    }
    ang_speed = ang_inc/time_inc_sec;
    robot_oldpose_ = robot_pose_;
}

GeometryMsgs::Pose Blitz::RF2O::GetPoseFromLaser(LaserScan* new_scan)
{
    last_scan = *new_scan;
    //current_scan_time = last_scan.header.stamp;

    if (!first_laser_scan)
    {
        //copy laser scan to internal variable
        for (unsigned int i = 0; i<width; i++)
        {
            range_wf(i) = new_scan->points[i].distance;
        }

    }
    else
    {
        Initialize(last_scan, initial_robot_pose);
        first_laser_scan = false;
        return GeometryMsgs::Pose();
    }

    OdometryCalc(last_scan);

    GeometryMsgs::Pose pose;

    pose.position.x = robot_pose_.translation()(0);
    pose.position.y = robot_pose_.translation()(1);
    pose.orientation.z = getYaw(robot_pose_.rotation());
    
    return pose;
}