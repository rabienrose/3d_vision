#include <iostream>
#include <imu_tools.h>
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include<Eigen/StdVector>
#include "configparam.h"
#include "g2otypes.h"


#include <memory>
#include "Converter.h"

namespace ORB_SLAM2
{
using namespace std;
using namespace Eigen;

cv::Mat SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void CalAccBias(const vector<cv::Mat>& vTwc, const vector<IMUPreintegrator>& vImuPreInt, 
                double& sstar, cv::Mat& gwstar, cv::Mat Tbc, Eigen::Matrix3d& Rwi_refined, Eigen::Vector3d& bias_a){
    int N= vTwc.size();
    cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
    cv::Mat pbc = Tbc.rowRange(0,3).col(3);
    cv::Mat Rcb = Rbc.t();
    cv::Mat pcb = -Rcb*pbc;
    cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
    gI.at<float>(2) = 1;
    // Normalized approx. gravity vecotr in world frame
    cv::Mat gwn = gwstar/cv::norm(gwstar);
    // Debug log
    //cout<<"gw normalized: "<<gwn<<endl;

    // vhat = (gI x gw) / |gI x gw|
    cv::Mat gIxgwn = gI.cross(gwn);
    double normgIxgwn = cv::norm(gIxgwn);
    cv::Mat vhat = gIxgwn/normgIxgwn;
    double theta = std::atan2(normgIxgwn,gI.dot(gwn));
    // Debug log
    //cout<<"vhat: "<<vhat<<", theta: "<<theta*180.0/M_PI<<endl;

    Eigen::Vector3d vhateig = Converter::toVector3d(vhat);
    Eigen::Matrix3d RWIeig = Sophus::SO3::exp(vhateig*theta).matrix();
    cv::Mat Rwi = Converter::toCvMat(RWIeig);
    cv::Mat GI = gI*9.8;//9.8012;
    // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
    cv::Mat C = cv::Mat::zeros(3*(N-2),6,CV_32F);
    cv::Mat D = cv::Mat::zeros(3*(N-2),1,CV_32F);

    for(int i=0; i<N-2; i++)
    {
        // Delta time between frames
        double dt12 = vImuPreInt[i+1].getDeltaTime();
        double dt23 = vImuPreInt[i+2].getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(vImuPreInt[i+1].getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(vImuPreInt[i+1].getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(vImuPreInt[i+2].getDeltaP());
        cv::Mat Jpba12 = Converter::toCvMat(vImuPreInt[i+1].getJPBiasa());
        cv::Mat Jvba12 = Converter::toCvMat(vImuPreInt[i+1].getJVBiasa());
        cv::Mat Jpba23 = Converter::toCvMat(vImuPreInt[i+2].getJPBiasa());
        // Pose of camera in world frame
        cv::Mat Twc1 = vTwc[i].clone();//pKF1->GetPoseInverse();
        cv::Mat Twc2 = vTwc[i+1].clone();//pKF2->GetPoseInverse();
        cv::Mat Twc3 = vTwc[i+2].clone();//pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
        cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
        cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);
        // Stack to C/D matrix
        // lambda*s + phi*dthetaxy + zeta*ba = psi
        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        cv::Mat phi = - 0.5*(dt12*dt12*dt23 + dt12*dt23*dt23)*Rwi*SkewSymmetricMatrix(GI);  // note: this has a '-', different to paper
        cv::Mat zeta = Rc2*Rcb*Jpba23*dt12 + Rc1*Rcb*Jvba12*dt12*dt23 - Rc1*Rcb*Jpba12*dt23;
        cv::Mat psi = (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - (Rc2-Rc3)*pcb*dt12
                     - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23); // note:  - paper
        lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
        phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
        zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
        psi.copyTo(D.rowRange(3*i+0,3*i+3));

        // Debug log
        //cout<<"iter "<<i<<endl;
    }

    // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
    // C = u*w*vt, u*w*vt*x=D
    // Then x = vt'*winv*u'*D
    cv::Mat w2,u2,vt2;
    // Note w2 is 6x1 vector by SVDecomp()
    // C is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(C,w2,u2,vt2,cv::SVD::MODIFY_A);
    // Debug log
    //cout<<"u2:"<<endl<<u2<<endl;
    //cout<<"vt2:"<<endl<<vt2<<endl;
    //cout<<"w2:"<<endl<<w2<<endl;

    // Compute winv
    cv::Mat w2inv=cv::Mat::eye(6,6,CV_32F);
    for(int i=0;i<6;i++)
    {
        if(fabs(w2.at<float>(i))<1e-10)
        {
            w2.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
        }

        w2inv.at<float>(i,i) = 1./w2.at<float>(i);
    }
    // Then y = vt'*winv*u'*D
    cv::Mat y = vt2.t()*w2inv*u2.t()*D;

    double s_ = y.at<float>(0);
    sstar=s_;
    cv::Mat dthetaxy = y.rowRange(1,3);
    cv::Mat dbiasa_ = y.rowRange(3,6);
    
    //cv::Mat dbiasa_=cv::Mat::zeros(3,1, CV_64FC1);
    Vector3d dbiasa_eig = Converter::toVector3d(dbiasa_);
    bias_a=dbiasa_eig;
    // dtheta = [dx;dy;0]
    cv::Mat dtheta = cv::Mat::zeros(3,1,CV_32F);
    dthetaxy.copyTo(dtheta.rowRange(0,2));
    Eigen::Vector3d dthetaeig = Converter::toVector3d(dtheta);
    // Rwi_ = Rwi*exp(dtheta)
    Eigen::Matrix3d Rwieig_ = RWIeig*Sophus::SO3::exp(dthetaeig).matrix();
    Rwi_refined=Rwieig_;
}

void CalGravityAndScale(const vector<cv::Mat>& vTwc, const vector<IMUPreintegrator>& vImuPreInt, cv::Mat Tbc,
    double& sstar, cv::Mat& gwstar
){
    int N= vTwc.size();
    cv::Mat A = cv::Mat::zeros(3*(N-2),4,CV_32F);
    cv::Mat B = cv::Mat::zeros(3*(N-2),1,CV_32F);
    cv::Mat I3 = cv::Mat::eye(3,3,CV_32F);
    cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
    cv::Mat pbc = Tbc.rowRange(0,3).col(3);
    cv::Mat Rcb = Rbc.t();
    cv::Mat pcb = -Rcb*pbc;

    for(int i=0; i<N-2; i++)
    {
        double dt12 = vImuPreInt[i+1].getDeltaTime();
        double dt23 = vImuPreInt[i+2].getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(vImuPreInt[i+1].getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(vImuPreInt[i+1].getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(vImuPreInt[i+2].getDeltaP());

        // Pose of camera in world frame
        cv::Mat Twc1 = vTwc[i].clone();//pKF1->GetPoseInverse();
        cv::Mat Twc2 = vTwc[i+1].clone();//pKF2->GetPoseInverse();
        cv::Mat Twc3 = vTwc[i+2].clone();//pKF3->GetPoseInverse();
        // Position of camera center
        cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
        cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
        cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
        // Rotation of camera, Rwc
        cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
        cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
        cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);

        // Stack to A/B matrix
        // lambda*s + beta*g = gamma
        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        cv::Mat beta = 0.5*I3*(dt12*dt12*dt23 + dt12*dt23*dt23);
        cv::Mat gamma = (Rc3-Rc2)*pcb*dt12 + (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt12*dt23;
        lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));
        beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));
        gamma.copyTo(B.rowRange(3*i+0,3*i+3));
        // Tested the formulation in paper, -gamma. Then the scale and gravity vector is -xx

        // Debug log
        //cout<<"iter "<<i<<endl;
    }
    // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
    // A = u*w*vt, u*w*vt*x=B
    // Then x = vt'*winv*u'*B
    cv::Mat w,u,vt;
    // Note w is 4x1 vector by SVDecomp()
    // A is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A);
    // Debug log
    //cout<<"u:"<<endl<<u<<endl;
    //cout<<"vt:"<<endl<<vt<<endl;
    //cout<<"w:"<<endl<<w<<endl;

    // Compute winv
    cv::Mat winv=cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<4;i++)
    {
        if(fabs(w.at<float>(i))<1e-10)
        {
            w.at<float>(i) += 1e-10;
            // Test log
            cerr<<"w(i) < 1e-10, w="<<endl<<w<<endl;
        }

        winv.at<float>(i,i) = 1./w.at<float>(i);
    }
    // Then x = vt'*winv*u'*B
    cv::Mat x = vt.t()*winv*u.t()*B;

    // x=[s,gw] 4x1 vector
    sstar = x.at<float>(0);    // scale should be positive
    gwstar = x.rowRange(1,4);   // gravity should be about ~9.8
}

void GlobalBundleAdjustmentNavStatePRV(std::vector<IMUPreintegrator>& preints, std::vector<NavState>& states, Matrix4d Tbc,
                                       std::vector<std::vector<MP_INFO>> mp_infos, float fx, float fy, float cx, float cy,
                                       std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& mp_posis,
                                       const cv::Mat& gw, int nIterations)
{
    int nlevels=8;
    float scaleFactor=1.2;
    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }
    
    Matrix3d Rbc = Tbc.topLeftCorner(3,3);
    Vector3d Pbc = Tbc.topRightCorner(3,1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(mp_posis.size());

    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(
            g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));
    optimizer.setAlgorithm(solver);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<preints.size(); i++)
    {
        g2o::VertexNavStatePR * vNSPR = new g2o::VertexNavStatePR();
        vNSPR->setEstimate(states[i]);
        vNSPR->setId(i*3);
        vNSPR->setFixed(i==0);
        optimizer.addVertex(vNSPR);
        // V
        g2o::VertexNavStateV * vNSV = new g2o::VertexNavStateV();
        vNSV->setEstimate(states[i]);
        vNSV->setId(i*3+1);
        vNSV->setFixed(false);
        optimizer.addVertex(vNSV);
        // Bias
        g2o::VertexNavStateBias * vNSBias = new g2o::VertexNavStateBias();
        vNSBias->setEstimate(states[i]);
        vNSBias->setId(i*3+2);
        vNSBias->setFixed(i==0);
        optimizer.addVertex(vNSBias);

        if(i*3+2>maxKFid)
            maxKFid=i*3+2;
    }

    // Add NavState PRV/Bias edges
    const float thHuberNavStatePRV = sqrt(100*21.666);
    const float thHuberNavStateBias = sqrt(100*16.812);
    // Inverse covariance of bias random walk

    Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
    InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
    InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE

    std::vector<g2o::EdgeNavStatePRV*> prvEdges;
    for(size_t i=1; i<preints.size(); i++)
    {
        // PVR edge
        {
            // PR0, PR1, V0, V1, B0
            g2o::EdgeNavStatePRV * epvr = new g2o::EdgeNavStatePRV();
            epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*(i-1))));
            epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*i)));
            epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*(i-1)+1)));
            epvr->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*i+1)));
            epvr->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*(i-1)+2)));
            epvr->setMeasurement(preints[i]);

            //Matrix9d InvCovPVR = pKF1->GetIMUPreInt().getCovPVPhi().inverse();
            Matrix9d CovPRV = preints[i].getCovPVPhi();
            CovPRV.col(3).swap(CovPRV.col(6));
            CovPRV.col(4).swap(CovPRV.col(7));
            CovPRV.col(5).swap(CovPRV.col(8));
            CovPRV.row(3).swap(CovPRV.row(6));
            CovPRV.row(4).swap(CovPRV.row(7));
            CovPRV.row(5).swap(CovPRV.row(8));
            epvr->setInformation(CovPRV.inverse());

            epvr->SetParams(GravityVec);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            epvr->setRobustKernel(rk);
            rk->setDelta(thHuberNavStatePRV);
            prvEdges.push_back(epvr);
            optimizer.addEdge(epvr);
        }
        // Bias edge
        {
            g2o::EdgeNavStateBias * ebias = new g2o::EdgeNavStateBias();
            ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*(i-1)+2)));
            ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*i+2)));
            ebias->setMeasurement(preints[i]);

            ebias->setInformation(InvCovBgaRW/preints[i].getDeltaTime());

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            ebias->setRobustKernel(rk);
            rk->setDelta(thHuberNavStateBias);

            optimizer.addEdge(ebias);
        }

    }

    const float thHuber2D = sqrt(5.99);
    // Set MapPoint vertices
    std::vector<g2o::EdgeNavStatePRPointXYZ*> project_edges;
    for(size_t i=0; i<mp_infos.size(); i++)
    {
        //std::cout<<"i: "<<i<<std::endl;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(mp_posis[i]);
        const int id = i+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        int nEdges = 0;
        //SET EDGES
        
        for(int j=0; j<mp_infos[i].size(); j++)
        {
            
            nEdges++;
            Eigen::Matrix<double,2,1> obs;
            obs << mp_infos[i][j].u, mp_infos[i][j].v;

            g2o::EdgeNavStatePRPointXYZ* e = new g2o::EdgeNavStatePRPointXYZ();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*mp_infos[i][j].frame_id)));
            e->setMeasurement(obs);
            //std::cout<<"j: "<<mp_infos[i][j].octove<<std::endl;
            const float &invSigma2 = mvInvLevelSigma2[mp_infos[i][j].octove];
            
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
            //e->setInformation(Eigen::Matrix2d::Identity());

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);

            e->SetParams(fx,fy,cx,cy,Rbc,Pbc);
            project_edges.push_back(e);

            optimizer.addEdge(e);
            
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }
    
    float totall_pvr_error=0;
    for(int i=0; i<prvEdges.size(); i++){
        prvEdges[i]->computeError();
        totall_pvr_error = totall_pvr_error+sqrt(prvEdges[i]->chi2())/prvEdges.size();
    }
    std::cout<<"totall_pvr_error before: "<<totall_pvr_error<<std::endl;
    
    float totall_proj_error=0;
    for(int i=0; i<project_edges.size(); i++){
        project_edges[i]->computeError();
        totall_proj_error = totall_proj_error+sqrt(project_edges[i]->chi2())/project_edges.size();
    }
    std::cout<<"totall_proj_error before: "<<totall_proj_error<<std::endl;
    
    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    
    totall_pvr_error=0;
    for(int i=0; i<prvEdges.size(); i++){
        prvEdges[i]->computeError();
        totall_pvr_error = totall_pvr_error+sqrt(prvEdges[i]->chi2())/prvEdges.size();
    }
    std::cout<<"totall_pvr_error after: "<<totall_pvr_error<<std::endl;
    
    
    totall_proj_error=0;
    for(int i=0; i<project_edges.size(); i++){
        project_edges[i]->computeError();
        totall_proj_error = totall_proj_error+sqrt(project_edges[i]->chi2())/project_edges.size();
    }
    std::cout<<"totall_proj_error after: "<<totall_proj_error<<std::endl;
    
    

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<states.size(); i++)
    {
        g2o::VertexNavStatePR* vNSPR = static_cast<g2o::VertexNavStatePR*>(optimizer.vertex(3*i));
        g2o::VertexNavStateV* vNSV = static_cast<g2o::VertexNavStateV*>(optimizer.vertex(3*i+1));
        g2o::VertexNavStateBias* vNSBias = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(3*i+2));
        const NavState& nspr = vNSPR->estimate();
        const NavState& nsv = vNSV->estimate();
        const NavState& nsbias = vNSBias->estimate();
        NavState& ns_recov = states[i];
        ns_recov.Set_Pos(nspr.Get_P());
        ns_recov.Set_Rot(nspr.Get_R());
        ns_recov.Set_Vel(nsv.Get_V());
        ns_recov.Set_BiasGyr(nsbias.Get_dBias_Gyr()+ nsbias.Get_dBias_Gyr());
        ns_recov.Set_BiasAcc(nsbias.Get_dBias_Acc()+ nsbias.Get_dBias_Acc());
    }

    //Points
    for(size_t i=0; i<mp_posis.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;
        
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+maxKFid+1));
        mp_posis[i]=vPoint->estimate();
    }
}

Eigen::Vector3d OptimizeInitialGyroBias(const vector<cv::Mat>& vTwc, const vector<IMUPreintegrator>& vImuPreInt, Matrix4d Tbc)
{
    int N = vTwc.size(); if(vTwc.size()!=vImuPreInt.size()) cerr<<"vTwc.size()!=vImuPreInt.size()"<<endl;
    
    Matrix3d Rcb = Tbc.topLeftCorner(3,3).transpose();
    
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(
            g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // Add vertex of gyro bias, to optimizer graph
    g2o::VertexGyrBias * vBiasg = new g2o::VertexGyrBias();
    vBiasg->setEstimate(Eigen::Vector3d::Zero());
    vBiasg->setId(0);
    optimizer.addVertex(vBiasg);

    // Add unary edges for gyro bias vertex
    //for(std::vector<KeyFrame*>::const_iterator lit=vpKFs.begin(), lend=vpKFs.end(); lit!=lend; lit++)
    for(int i=0; i<N; i++)
    {
        // Ignore the first KF
        if(i==0)
            continue;

        const cv::Mat& Twi = vTwc[i-1];    // pose of previous KF
        Matrix3d Rwci = Converter::toMatrix3d(Twi.rowRange(0,3).colRange(0,3));
        //Matrix3d Rwci = Twi.rotation_matrix();
        const cv::Mat& Twj = vTwc[i];        // pose of this KF
        Matrix3d Rwcj = Converter::toMatrix3d(Twj.rowRange(0,3).colRange(0,3));
        //Matrix3d Rwcj =Twj.rotation_matrix();

        const IMUPreintegrator& imupreint = vImuPreInt[i];

        g2o::EdgeGyrBias * eBiasg = new g2o::EdgeGyrBias();
        eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        // measurement is not used in EdgeGyrBias
        eBiasg->dRbij = imupreint.getDeltaR();
        eBiasg->J_dR_bg = imupreint.getJRBiasg();
        eBiasg->Rwbi = Rwci*Rcb;
        eBiasg->Rwbj = Rwcj*Rcb;
        //eBiasg->setInformation(Eigen::Matrix3d::Identity());
        eBiasg->setInformation(imupreint.getCovPVPhi().bottomRightCorner(3,3).inverse());
        optimizer.addEdge(eBiasg);
    }

    // It's actualy a linear estimator, so 1 iteration is enough.
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(1);

    g2o::VertexGyrBias * vBgEst = static_cast<g2o::VertexGyrBias*>(optimizer.vertex(0));

    return vBgEst->estimate();
}
}
