#include <string>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include <math.h>

#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "imu_tools.h"
#include "read_write_data_lib/read_write.h"
#include "opencv2/opencv.hpp"
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

namespace OptimizerTool
{
    void optimize_true_pose(std::vector<Eigen::Matrix4d> lidar_poses,
        std::vector<std::vector<orb_slam::MP_INFO>> mp_infos, std::vector<Eigen::Vector3d>& mp_posis,
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& pose_vec, Eigen::Matrix3d cam_inter
    ){

        std::vector<cv::Mat> proj_cv_mats;
        for(int i=0; i<lidar_poses.size(); i++){
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
        std::cout<<"procece lidar: "<<proj_cv_mats.size()<<std::endl;
        std::cout<<"mp_infos count: "<<mp_infos.size()<<std::endl;
        std::cout<<"mp_posis count: "<<mp_posis.size()<<std::endl;

        for(int i=0; i<mp_infos.size(); i++){
    //         std::cout<<i<<"/"<<mp_infos.size()<<std::endl;
            int last_id=mp_infos[i].size()-1;
            std::vector<cv::Point2f> pts1;
            cv::Point2f pt1( mp_infos[i][0].u, mp_infos[i][0].v);
            pts1.push_back(pt1);
            std::vector<cv::Point2f> pts2;
            cv::Point2f pt2( mp_infos[i][last_id].u, mp_infos[i][last_id].v);
            pts2.push_back(pt2);
            if(proj_cv_mats.size()<=mp_infos[i][0].frame_id){
                std::cout<<"proj_cv_mats index wrong"<<std::endl;
            }
            if(proj_cv_mats.size()<=mp_infos[i][last_id].frame_id){
                std::cout<<"proj_cv_mats index wrong"<<std::endl;
            }
            cv::Mat proj1=proj_cv_mats[mp_infos[i][0].frame_id];
            cv::Mat proj2=proj_cv_mats[mp_infos[i][last_id].frame_id];
            cv::Mat out_posi;
            cv::triangulatePoints(proj1, proj2, pts1, pts2, out_posi);
            mp_posis[i](0)=out_posi.at<float>(0)/out_posi.at<float>(3);
            mp_posis[i](1)=out_posi.at<float>(1)/out_posi.at<float>(3);
            mp_posis[i](2)=out_posi.at<float>(2)/out_posi.at<float>(3);
            //std::cout<<mp_posis[i].transpose()<<std::endl;
        }
        std::cout<<"procece mp"<<std::endl;
        
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
            new g2o::BlockSolverX(
                new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()));
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
        std::cout<<"add vertice"<<std::endl;
        
        g2o::VertexSE3Expmap* cam_lidar_offset = new g2o::VertexSE3Expmap();
        Eigen::Matrix<double,3,3> R=Eigen::Matrix<double,3,3>::Identity();
        Eigen::Matrix<double,3,1> t=Eigen::Matrix<double,3,1>::Zero();
        cam_lidar_offset->setEstimate(g2o::SE3Quat(R,t));
        cam_lidar_offset->setId(maxKFid+1);
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
            e->setInformation(Eigen::Matrix<double, 3, 3>::Identity()*100000000);
            optimizer.addEdge(e);
            lidar_edges.push_back(e);
            
        }
        std::cout<<"add lidar edge"<<std::endl;
        
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
                
//                 if(i==0){
//                     std::cout<<mp_verts.back()->estimate()<<std::endl;
//                     e->computeError();
//                     std::cout<<sqrt(e->chi2())<<std::endl;
//                 }
                proj_edges.push_back(e);
            }
        }
        std::cout<<"add project edge"<<std::endl;
        
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
        for(int i=0; i<1; i++){
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
        }
        
        for(int i=0; i<mp_verts.size(); i++){
            mp_posis[i]=mp_verts[i]->estimate();
        }
        
        for(int i=0; i<kf_verts.size(); i++){
            pose_vec[i]=kf_verts[i]->estimate().inverse().to_homogeneous_matrix();
        }
    }
    
    void optimize_true_pose(std::string res_root){
        std::cout<<"start lidar opti"<<std::endl;
        std::string cam_addr=res_root+"/camera_config.txt";
        Eigen::Matrix3d cam_inter;
        Eigen::Vector4d cam_distort;
        Eigen::Matrix4d Tbc;
        CHAMO::read_cam_info(cam_addr, cam_inter, cam_distort, Tbc);
        std::cout<<"Tbc: "<<Tbc<<std::endl;
        
        std::string img_time_addr=res_root+"/image_time.txt";
        std::string pose_addr=res_root+"/traj.txt";
        std::map<double, int> pose_list;
        std::map<int, int> frame_ids;
        std::vector<double> img_times;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> pose_vec;
        CHAMO::read_pose_list(pose_list, frame_ids, pose_vec, img_times, pose_addr, img_time_addr);
        std::cout<<"pose_list: "<<img_times.size()<<std::endl;

        std::string posi_addr=res_root+"/posi.txt";
        std::vector<Eigen::Vector3d> mp_posis;
        CHAMO::read_mp_posi(posi_addr, mp_posis);
        std::cout<<"mp_posis: "<<mp_posis.size()<<std::endl;
        
        std::string kp_addr=res_root+"/kps.txt";
        std::vector<Eigen::Vector2f> kp_uvs;
        std::vector<int> kp_frameids;
        std::vector<int> kp_octoves;
        CHAMO::read_kp_info(kp_addr, kp_uvs, kp_frameids, kp_octoves);
        std::cout<<"kp_uvs: "<<kp_uvs.size()<<std::endl;
        
        std::string track_addr=res_root+"/track.txt";
        std::vector<std::vector<int>> tracks;
        CHAMO::read_track_info(track_addr, tracks);
        std::cout<<"tracks: "<<tracks.size()<<std::endl;
        
        std::vector<Eigen::Vector3d> lidar_posis;
        std::vector<Eigen::Quaterniond> lidar_dirs;
        std::vector<double> lidar_time;
        //std::string lidar_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/old/wayz_2018_11_26.bag_trajectory.txt";
        std::string lidar_addr=res_root+"/lidar_trajectory.txt";
        CHAMO::read_lidar_pose(lidar_addr, lidar_dirs, lidar_posis, lidar_time);
        std::cout<<"lidar_addr: "<<lidar_posis.size()<<std::endl;
        
        std::vector<std::vector<orb_slam::MP_INFO>> mp_infos;
        for(int i=0; i<tracks.size(); i++){
            std::vector<orb_slam::MP_INFO> track_info;
            for(int j=0; j<tracks[i].size(); j++){
                int kp_id=tracks[i][j];
                orb_slam::MP_INFO info;
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
        for(int i=0; i<img_times.size(); i++){
            int corres_lidar=-1;
            for(int j=0; j<lidar_time.size(); j++){
                if(fabs(lidar_time[j]-img_times[i])<0.01){
                    corres_lidar=j;
                    break;
                }
            }
            if(corres_lidar>=0){
                Eigen::Matrix<double,4,4> P_double=Eigen::Matrix<double,4,4>::Identity();
                
                P_double.block(0,3,3,1)=lidar_posis[corres_lidar];
                Eigen::Matrix3d rot(lidar_dirs[corres_lidar]);
                P_double.block(0,0,3,3)=rot;
                P_double=P_double.inverse();
                lidar_align.push_back(P_double.inverse());
//                 if(i<100){
//                     std::cout<<P_double<<std::endl;
//                 }
                
            }else{
                std::cout<<"error in lidar: "<<i<<std::endl;
                lidar_align.push_back(lidar_align.back());
                //we allow no lidar correspond to img, but every image must have a lidar attach to it.
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
        optimize_true_pose(lidar_align, mp_infos, mp_posis, pose_vec, cam_inter);
        std::string posi_out_addr=res_root+"/posi_alin.txt";
        std::ofstream f;
        f.open(posi_out_addr.c_str());
        for(int i=0; i<mp_posis.size(); i++){
            f<<mp_posis[i](0)<<","<<mp_posis[i](1)<<","<<mp_posis[i](2)<<std::endl;
        }
        f.close();
        std::string pose_out_addr=res_root+"/traj_alin.txt";
        f.open(pose_out_addr.c_str());
        for(int i=0; i<pose_vec.size(); i++){
            f<<"chamo.jpg"<<i<<pose_vec[i](0,0)<<pose_vec[i](0,1)<<pose_vec[i](0,2)<<pose_vec[i](0,3)<<pose_vec[i](1,0)<<pose_vec[i](1,1)<<pose_vec[i](1,2)<<pose_vec[i](1,3)<<pose_vec[i](2,0)<<pose_vec[i](2,1)<<pose_vec[i](2,2)<<pose_vec[i](2,3)<<std::endl;
        }
        f.close();
    }
}
