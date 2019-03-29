#include <string>
#include <fstream>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <math.h>
#include "imu_tools.h"
#include "NavState.h"
#include "Converter.h"

#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "test_imu_tool/visual_tool.h"
#include "read_write_data_lib/read_write.h"
#include "visualization/common-rviz-visualization.h"
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "opencv2/opencv.hpp"

// Eigen::Matrix4d align_traj(std::vector<Eigen::Vector3d>& traj1, std::vector<Eigen::Vector3d>& traj2){
//     Eigen::Vector3d p1=Eigen::Vector3d::Zero();
//     Eigen::Vector3d p2=Eigen::Vector3d::Zero();     // center of mass
//     int N = traj1.size();
//     for (int i=0; i<traj1.size(); i++){
//         p1 += traj1[i];
//         p2 += traj2[i];
//     }
//     p1 = p1 /  N;
//     p2 = p2 / N;
//     
//     std::vector<Eigen::Vector3d> q1;
//     std::vector<Eigen::Vector3d> q2;
//     for (int i=0; i<traj1.size(); i++){
//         q1.push_back(traj1[i]-p1);
//         q2.push_back(traj2[i]-p2);
//     }
// 
//     // compute q1*q2^T
//     Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
//     for ( int i=0; i<N; i++ )
//     {
//         W += q1[i] * q2[i].transpose();
//     }
//     std::cout<<"W="<<W<<std::endl;
// 
//     // SVD on W
//     Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
//     Eigen::Matrix3d U = svd.matrixU();
//     Eigen::Matrix3d V = svd.matrixV();
//     
//     if (U.determinant() * V.determinant() < 0)
//      {
//         for (int x = 0; x < 3; ++x)
//         {
//             U(x, 2) *= -1;
//         }
//     }
//     
//     std::cout<<"U="<<U<<std::endl;
//     std::cout<<"V="<<V<<std::endl;
// 
//     Eigen::Matrix3d R_ = U* ( V.transpose() );
//     Eigen::Vector3d t_ = p1 - R_ * p2;
//     Eigen::Matrix4d re = Eigen::Matrix4d::Identity();
//     re.block(0,0,3,3)=R_;
//     re.block(0,3,3,1)=t_;
//     
//     return re;
// }

namespace g2o {
    class EdgePosePre : public BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePosePre(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        const g2o::VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        SE3Quat C(_measurement);
        SE3Quat qure_rot=v2->estimate();
        //qure_rot.setTranslation(Eigen::Vector3d::Zero());
        SE3Quat error_= v1->estimate().inverse()*qure_rot*C;
        _error = error_.log();
        }
    };
    
    class EdgePosiPre : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePosiPre(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        _error= v1->estimate().inverse().translation()-_measurement;
        }
    };
}

void doOpti(std::vector<Eigen::Matrix4d> lidar_poses,
    std::vector<std::vector<ORB_SLAM2::MP_INFO>> mp_infos, std::vector<Eigen::Vector3d>& mp_posis,
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& pose_vec, Eigen::Matrix3d cam_inter
){
//     std::vector<Eigen::Vector3d> traj_lidar;
//     std::vector<Eigen::Vector3d> traj_cam;
//     for(int i=0; i<pose_vec.size(); i++){
//         Eigen::Vector3d cam_posi=pose_vec[i].block(0,3,3,1);
//         traj_lidar.push_back(lidar_posis[i]);
//         traj_cam.push_back(cam_posi);
//         //std::cout<<lidar_posis[i].transpose()<<std::endl;
//         //std::cout<<cam_posi.transpose()<<std::endl;
//         
//     }
//     Eigen::Matrix4d T_lidarw_camw = align_traj(traj_lidar, traj_cam);
//     
//     for(int i=0; i<pose_vec.size(); i++){
//         pose_vec[i]=T_lidarw_camw*pose_vec[i];
//     }
//     
//     for(int i=0; i<mp_posis.size(); i++){
//         Eigen::Vector4d posi;
//         posi(0)=mp_posis[i](0);
//         posi(1)=mp_posis[i](1);
//         posi(2)=mp_posis[i](2);
//         posi(3)=1;
//         posi=T_lidarw_camw*posi;
//         mp_posis[i](0)=posi(0);
//         mp_posis[i](1)=posi(1);
//         mp_posis[i](2)=posi(2);
//     }
    
    std::vector<cv::Mat> proj_cv_mats;
    Eigen::Matrix4d temp_rot=Eigen::Matrix4d::Identity();
    for(int i=0; i<lidar_poses.size(); i++){
//         temp_rot(0,0)=0.999163;
//         temp_rot(0,1)=0.0252427;
//         temp_rot(0,2)=0.032173;
//         temp_rot(1,0)=-0.0158482;
//         temp_rot(1,1)=0.964287;
//         temp_rot(1,2)=-0.264385;
//         temp_rot(2,0)=-0.0376986;
//         temp_rot(2,1)=0.263654;
//         temp_rot(2,2)=0.96388;

        pose_vec[i]=lidar_poses[i];
        Eigen::Matrix<double,3,4> P_double_34=lidar_poses[i].inverse().block(0,0,3,4);
        P_double_34=cam_inter*P_double_34;
        cv::Mat cv_mat(3,4, CV_32FC1);
        for(int n=0; n<3; n++){
            for(int m=0; m<4; m++){
                cv_mat.at<float>(n,m)=P_double_34(n,m);
            }
        }
        proj_cv_mats.push_back(cv_mat);
    }

    for(int i=0; i<mp_infos.size(); i++){
        int last_id=mp_infos[i].size()-1;
        std::vector<cv::Point2f> pts1;
        cv::Point2f pt1( mp_infos[i][0].u, mp_infos[i][0].v);
        pts1.push_back(pt1);
        std::vector<cv::Point2f> pts2;
        cv::Point2f pt2( mp_infos[i][last_id].u, mp_infos[i][last_id].v);
        pts2.push_back(pt2);
        cv::Mat proj1=proj_cv_mats[mp_infos[i][0].frame_id];
        cv::Mat proj2=proj_cv_mats[mp_infos[i][last_id].frame_id];
        cv::Mat out_posi;
        cv::triangulatePoints(proj1, proj2, pts1, pts2, out_posi);
        mp_posis[i](0)=out_posi.at<float>(0)/out_posi.at<float>(3);
        mp_posis[i](1)=out_posi.at<float>(1)/out_posi.at<float>(3);
        mp_posis[i](2)=out_posi.at<float>(2)/out_posi.at<float>(3);
    }
    
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
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(
            g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));
    optimizer.setAlgorithm(solver);
    std::vector<g2o::VertexSE3Expmap*> kf_verts;
    long unsigned int maxKFid = 0;
    for(int i=0; i<pose_vec.size(); i++){
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        Eigen::Matrix<double,3,3> R=pose_vec[i].block(0,0,3,3);
        Eigen::Matrix<double,3,1> t=pose_vec[i].block(0,3,3,1);
        vSE3->setEstimate(g2o::SE3Quat(R,t).inverse());
        vSE3->setId(i);
        kf_verts.push_back(vSE3);
        optimizer.addVertex(vSE3);
        if(i>maxKFid)
            maxKFid=i;
    }
    
    g2o::VertexSE3Expmap* cam_lidar_offset = new g2o::VertexSE3Expmap();
    Eigen::Matrix<double,3,3> R=Eigen::Matrix<double,3,3>::Identity();
    Eigen::Matrix<double,3,1> t=Eigen::Matrix<double,3,1>::Zero();
    cam_lidar_offset->setEstimate(g2o::SE3Quat(R,t));
    cam_lidar_offset->setId(maxKFid+1);
    //cam_lidar_offset->setFixed(true);
    //optimizer.addVertex(cam_lidar_offset);

//     std::vector<g2o::VertexSE3Expmap*> lidar_verts;
//     long unsigned int maxLidarid = 0;
//     for(int i=0; i<pose_vec.size(); i++){
//         g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
//         Eigen::Matrix<double,3,3> R(lidar_dirs[i]);
//         Eigen::Matrix<double,3,1> t=lidar_posis[i];
//         vSE3->setEstimate(g2o::SE3Quat(R,t).inverse());
//         vSE3->setId(i+maxKFid+1);
//         vSE3->setFixed(true);
//         lidar_verts.push_back(vSE3);
//         optimizer.addVertex(vSE3);
//         if(i>maxLidarid)
//             maxLidarid=i;
//     }

//     std::vector<g2o::EdgeSE3Expmap*> lidar_edges;
//     for(int i=0; i<pose_vec.size(); i++){
//         g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();
//         e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(kf_verts[i]));
//         e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(lidar_verts[i]));
//         Eigen::Matrix<double,3,3> R(lidar_dirs[i]);
//         Eigen::Matrix<double,3,1> t=lidar_posis[i];
//         e->setMeasurement(g2o::SE3Quat(Eigen::Matrix<double,3,3>::Identity(),Eigen::Matrix<double,3,1>::Zero()));
//         Eigen::Matrix<double, 6, 6> infor_lidar = Eigen::Matrix<double, 6, 6>::Identity();
//         infor_lidar(0,0)=0;
//         infor_lidar(1,2)=0;
//         infor_lidar(3,3)=0;
//         e->setInformation(infor_lidar);
//         optimizer.addEdge(e);
//         lidar_edges.push_back(e);
//     }
    
    std::vector<g2o::EdgePosiPre*> lidar_edges;
    for(int i=0; i<pose_vec.size(); i++){
        g2o::EdgePosiPre* e = new g2o::EdgePosiPre();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(kf_verts[i]));
        //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(cam_lidar_offset));
        //Eigen::Matrix<double,3,3> R=lidar_poses[i].block(0,0,3,3);
        //Eigen::Matrix<double,3,1> t=lidar_poses[i].block(0,3,3,1);
        e->setMeasurement(lidar_poses[i].block(0,3,3,1));
        //e->setMeasurement(g2o::SE3Quat(R,t).inverse());
        //e->setInformation(Eigen::Matrix<double, 6, 6>::Identity()*1000000);
        e->setInformation(Eigen::Matrix<double, 3, 3>::Identity()*1000000);
        optimizer.addEdge(e);
        lidar_edges.push_back(e);
        
    }
    
    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    
    std::vector<g2o::VertexSBAPointXYZ*> mp_verts;
    std::vector<g2o::EdgeSE3ProjectXYZ*> proj_edges;
    for(int i=0; i<mp_infos.size(); i++){
        for(int j=0; j<mp_infos[i].size(); j++){
            if(j==0){
                g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
                vPoint->setEstimate(mp_posis[mp_infos[i][j].mp_id]);
                vPoint->setMarginalized(true);
                const int id = i+maxKFid+1+2;
                vPoint->setId(id);
                optimizer.addVertex(vPoint);
                mp_verts.push_back(vPoint);
            }
            Eigen::Matrix<double,2,1> obs;
            obs << mp_infos[i][j].u, mp_infos[i][j].v;

            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(mp_verts.back()));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(kf_verts[mp_infos[i][j].frame_id]));
            e->setMeasurement(obs);
            const float &invSigma2 = mvInvLevelSigma2[mp_infos[i][j].octove];
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);
            e->fx = cam_inter(0,0);
            e->fy = cam_inter(1,1);
            e->cx = cam_inter(0,2);
            e->cy = cam_inter(1,2);

            optimizer.addEdge(e);
            proj_edges.push_back(e);
        }
    }
    
    float avg_error=0;
    for(int i=0; i<lidar_edges.size(); i++){
        lidar_edges[i]->computeError();
        avg_error=avg_error+sqrt(lidar_edges[i]->chi2())/lidar_edges.size();
    }
    std::cout<<"lidar edge err before: "<<avg_error<<std::endl;
    avg_error=0;
    for(int i=0; i<proj_edges.size(); i++){
        proj_edges[i]->computeError();
        avg_error=avg_error+sqrt(proj_edges[i]->chi2())/proj_edges.size();
    }
    std::cout<<"project edge err before: "<<avg_error<<std::endl;
    clock_t time;
    time = clock();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    time = clock() - time;
    //std::cout<<"cam after: "<<vCam->estimate().transpose()<<std::endl;
    std::cout<<"opt time: "<<((float)time)/CLOCKS_PER_SEC<<std::endl;

    avg_error=0;
    for(int i=0; i<lidar_edges.size(); i++){
        lidar_edges[i]->computeError();
        avg_error=avg_error+sqrt(lidar_edges[i]->chi2())/lidar_edges.size();
    }
    std::cout<<"lidar edge err after: "<<avg_error<<std::endl;
    avg_error=0;
    for(int i=0; i<proj_edges.size(); i++){
        proj_edges[i]->computeError();
        avg_error=avg_error+sqrt(proj_edges[i]->chi2())/proj_edges.size();
    }
    std::cout<<"project edge err after: "<<avg_error<<std::endl;
    
    for(int i=0; i<mp_verts.size(); i++){
        mp_posis[i]=mp_verts[i]->estimate();
    }
    
    //std::cout<<cam_lidar_offset->estimate()<<std::endl;
    
    for(int i=0; i<kf_verts.size(); i++){
        pose_vec[i]=kf_verts[i]->estimate().inverse().to_homogeneous_matrix();
    }
    
}

int main(int argc, char* argv[]) {
    std::string res_root=argv[0];
    visualization::RVizVisualizationSink::init();
    std::string save_addr;
    std::string line;
    
    std::string cam_addr=res_root+"/camera_config.txt";
    Eigen::Matrix3d cam_inter;
    Eigen::Vector4d cam_distort;
    Eigen::Matrix4d Tbc;
    CHAMO::read_cam_info(cam_addr, cam_inter, cam_distort, Tbc);
    
    std::string img_time_addr=res_root+"/camera_1_image_time.txt";
    std::string pose_addr=res_root+"/traj.txt";
    std::map<double, int> pose_list;
    std::map<int, int> frame_ids;
    std::vector<double> img_times;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> pose_vec;
    CHAMO::read_pose_list(pose_list, frame_ids, pose_vec, img_times, pose_addr, img_time_addr);

    std::string posi_addr=res_root+"/posi.txt";
    std::vector<Eigen::Vector3d> mp_posis;
    CHAMO::read_mp_posi(posi_addr, mp_posis);
    
    std::string kp_addr=res_root+"/kps.txt";
    std::vector<Eigen::Vector2f> kp_uvs;
    std::vector<int> kp_frameids;
    std::vector<int> kp_octoves;
    CHAMO::read_kp_info(kp_addr, kp_uvs, kp_frameids, kp_octoves);
    
    std::string track_addr=res_root+"/track.txt";
    std::vector<std::vector<int>> tracks;
    CHAMO::read_track_info(track_addr, tracks);
    
    std::vector<Eigen::Vector3d> lidar_posis;
    std::vector<Eigen::Quaterniond> lidar_dirs;
    std::vector<double> lidar_time;
    //std::string lidar_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/old/wayz_2018_11_26.bag_trajectory.txt";
    std::string lidar_addr=res_root+"/wayz_2018_11_26.bag_trajectory.txt";
    CHAMO::read_lidar_pose(lidar_addr, lidar_dirs, lidar_posis, lidar_time);
    
    std::vector<std::vector<ORB_SLAM2::MP_INFO>> mp_infos;
    for(int i=0; i<tracks.size(); i++){
        std::vector<ORB_SLAM2::MP_INFO> track_info;
        for(int j=0; j<tracks[i].size(); j++){
            int kp_id=tracks[i][j];
            ORB_SLAM2::MP_INFO info;
            info.u=kp_uvs[kp_id].x();
            info.v=kp_uvs[kp_id].y();
            info.octove=kp_octoves[kp_id];
            info.frame_id=frame_ids[kp_frameids[kp_id]];
            info.mp_id=i;
            track_info.push_back(info);
        }
        mp_infos.push_back(track_info);
    }

    std::vector<Eigen::Matrix4d> lidar_align;
    Eigen::Matrix4d temp_rot=Eigen::Matrix4d::Identity();
    temp_rot(0,0)=0.00650026;
    temp_rot(0,1)=-0.999966;
    temp_rot(0,2)=0.00511285;
    temp_rot(0,3)=-0.179085;
    temp_rot(1,0)=0.429883;
    temp_rot(1,1)=-0.00182202;
    temp_rot(1,2)=-0.902883;
    temp_rot(1,3)=-0.188234;
    temp_rot(2,0)=0.902861;
    temp_rot(2,1)=0.0080669;
    temp_rot(2,2)=0.429856;
    temp_rot(2,3)=0.0180092;
    for(int i=0; i<img_times.size(); i++){
        int corres_lidar=-1;
        for(int j=0; j<lidar_time.size(); j++){
            if(fabs(lidar_time[j]-img_times[i])<0.05){
                corres_lidar=j;
                break;
            }
        }
        if(corres_lidar>0){
            Eigen::Matrix<double,4,4> P_double=Eigen::Matrix<double,4,4>::Identity();
            P_double.block(0,3,3,1)=lidar_posis[corres_lidar];
            Eigen::Matrix3d rot(lidar_dirs[corres_lidar]);
            P_double.block(0,0,3,3)=rot;
            P_double=temp_rot*P_double.inverse();
            lidar_align.push_back(P_double.inverse());
        }else{
            std::cout<<"error in lidar "<<std::endl;
        }
    }
    
//     for(int i=0; i<mp_infos.size(); i++){
//         for(int j=0; j<mp_infos[i].size(); i++){
//             Eigen::Vector3d mp_posi = mp_posis[mp_infos[i][j].mp_id];
//             double t_u = mp_infos[i][j].u;
//             double t_v = mp_infos[i][j].v;
//             Eigen::Matrix4d proj_pose = pose_vec[mp_infos[i][j].frame_id].inverse();
//             Eigen::Vector4d mp_homo;
//             mp_homo.block(0,0,3,1)=mp_posi;
//             mp_homo(3)=1;
//             double fx = cam_inter(0,0);
//             double fy = cam_inter(1,1);
//             double cx = cam_inter(0,2);
//             double cy = cam_inter(1,2);
//             //std::cout<<t_u<<":"<<t_v<<std::endl;
//             Eigen::Vector4d mp_cam = proj_pose*mp_homo;
//             //std::cout<<mp_cam.transpose()<<std::endl;
//             double u = mp_cam(0)*fx/mp_cam(2)+cx;
//             double v = mp_cam(1)*fy/mp_cam(2)+cy;
//             double err=sqrt((u-t_u)*(u-t_u)+(v-t_v)*(v-t_v));
//             //std::cout<<u<<":"<<v<<std::endl;
//             //std::cout<<err<<std::endl;
//         }
//     }
    doOpti(lidar_align, mp_infos, mp_posis, pose_vec, cam_inter);
    
    show_pose_as_marker(pose_vec, "pose_cam");
    Eigen::Matrix3d Rwi_=Eigen::Matrix3d::Identity();
    show_mp_as_cloud(mp_posis, Rwi_, "chamo_target");
    show_mp_as_cloud(lidar_posis, Rwi_, "/chamo/gps");
    std::string posi_out_addr=res_root+"/posi_alin.txt";
    std::ofstream f;
    f.open(posi_out_addr.c_str());
    for(int i=0; i<mp_posis.size(); i++){
        f<<mp_posis[i](0)<<","<<mp_posis[i](1)<<","<<mp_posis[i](2)<<std::endl;
    }
    f.close();

    //ros::spin();
        
}