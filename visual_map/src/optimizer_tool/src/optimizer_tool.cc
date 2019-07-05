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
#include "test_imu_tool/visual_tool.h"
#include "optimizer_tool/optimizer_tool.h"
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

DECLARE_int32(opti_count);
DECLARE_double(gps_weight);

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
    void findFramePoseByName(std::vector<std::string>& names, int& re_id, std::string query_name){
        re_id=-1;
        for(int i=0; i<names.size(); i++){
            if(names[i]==query_name){
                re_id=i;
                return;
            }
        }
        return;
    };
    void optimize_true_pose(std::vector<Eigen::Matrix4d>& poses_alin, std::vector<Eigen::Vector3d>& gps_alin, std::vector<int>& gps_inlers,
        std::vector<std::vector<orb_slam::MP_INFO>>& mp_infos, Eigen::Matrix3d cam_inter,
        std::vector<Eigen::Matrix4d>& poses_out, std::vector<Eigen::Vector3d>& mp_posis_out
    ){

        std::vector<cv::Mat> proj_cv_mats;
        for(int i=0; i<poses_alin.size(); i++){
            Eigen::Matrix<double,3,4> P_double_34=poses_alin[i].inverse().block(0,0,3,4);
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
        std::cout<<"mp_posis count: "<<mp_posis_out.size()<<std::endl;
        mp_posis_out.resize(mp_infos.size());
        
        std::vector<bool> mp_mask;
        mp_mask.resize(mp_infos.size());
        for(int i=0; i<mp_infos.size(); i++){
            if(mp_infos.size()>5){
                mp_mask[i]=true;
                int last_id=mp_infos[i].size()-1;
                std::vector<cv::Point2f> pts1;
                cv::Point2f pt1( mp_infos[i][0].u, mp_infos[i][0].v);
                pts1.push_back(pt1);
                std::vector<cv::Point2f> pts2;
                cv::Point2f pt2( mp_infos[i][last_id].u, mp_infos[i][last_id].v);
                pts2.push_back(pt2);
                if(proj_cv_mats.size()<=mp_infos[i][0].frame_id){
                    std::cout<<"proj_cv_mats index wrong: "<<mp_infos[i][0].frame_id<<std::endl;
                }
                if(proj_cv_mats.size()<=mp_infos[i][last_id].frame_id){
                    std::cout<<"proj_cv_mats index wrong: "<<mp_infos[i][last_id].frame_id<<std::endl;
                }
                cv::Mat proj1=proj_cv_mats[mp_infos[i][0].frame_id];
                cv::Mat proj2=proj_cv_mats[mp_infos[i][last_id].frame_id];
                cv::Mat out_posi;
                cv::triangulatePoints(proj1, proj2, pts1, pts2, out_posi);
    //             if(cv::norm(out_posi)>100){
    //                 std::cout<<out_posi.t()<<std::endl;
    //             }
                mp_posis_out[i](0)=out_posi.at<float>(0)/out_posi.at<float>(3);
                mp_posis_out[i](1)=out_posi.at<float>(1)/out_posi.at<float>(3);
                mp_posis_out[i](2)=out_posi.at<float>(2)/out_posi.at<float>(3);
            }else{
                mp_mask[i]=false;
                mp_posis_out[i](0)=1000000;
                mp_posis_out[i](1)=1000000;
                mp_posis_out[i](2)=1000000;
            }
        }
//         int temp_count=0;
//         double err_total=0;
//         for(int i=0; i<mp_infos.size(); i++){
//             for(int j=0; j<mp_infos[i].size(); i++){
//                 Eigen::Vector3d mp_posi = mp_posis_out[mp_infos[i][j].mp_id];
//                 double t_u = mp_infos[i][j].u;
//                 double t_v = mp_infos[i][j].v;
//                 Eigen::Matrix4d proj_pose = poses_alin[mp_infos[i][j].frame_id].inverse();
//                 Eigen::Vector4d mp_homo;
//                 mp_homo.block(0,0,3,1)=mp_posi;
//                 mp_homo(3)=1;
//                 double fx = cam_inter(0,0);
//                 double fy = cam_inter(1,1);
//                 double cx = cam_inter(0,2);
//                 double cy = cam_inter(1,2);
//                 //std::cout<<t_u<<":"<<t_v<<std::endl;
//                 Eigen::Vector4d mp_cam = proj_pose*mp_homo;
//                 //std::cout<<mp_cam.transpose()<<std::endl;
//                 double u = mp_cam(0)*fx/mp_cam(2)+cx;
//                 double v = mp_cam(1)*fy/mp_cam(2)+cy;
//                 double err=sqrt((u-t_u)*(u-t_u)+(v-t_v)*(v-t_v));
//                 err_total=err_total+err;
//                 //std::cout<<u<<":"<<v<<std::endl;
//                 //std::cout<<err<<std::endl;
//                 temp_count++;
//             }
//         }
//         
//         std::cout<<"err_total: "<<err_total/temp_count<<std::endl;
        
        Eigen::Matrix3d Rwi_=Eigen::Matrix3d::Identity();
        show_mp_as_cloud(mp_posis_out, Rwi_, "/temp_mp");
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
        
        //add all vertice, number equal all poses 
        std::vector<g2o::VertexSE3Expmap*> kf_verts;
        long unsigned int maxKFid = 0;
        for(int i=0; i<poses_alin.size(); i++){
            g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
            Eigen::Matrix<double,3,3> R=poses_alin[i].block(0,0,3,3);
            Eigen::Matrix<double,3,1> t=poses_alin[i].block(0,3,3,1);
            vSE3->setEstimate(g2o::SE3Quat(R,t).inverse());
            vSE3->setId(i);
            kf_verts.push_back(vSE3);
            optimizer.addVertex(vSE3);
            if(i>maxKFid)
                maxKFid=i;
        }
        std::cout<<"add vertice"<<std::endl;
        
        //add gps edge, leave vertice without gps empty 
        std::vector<g2o::EdgePosiPre*> lidar_edges;
        for(int i=0; i<poses_alin.size(); i++){
            if(gps_inlers[i]==1){
                g2o::EdgePosiPre* e = new g2o::EdgePosiPre();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(kf_verts[i]));
                e->setMeasurement(gps_alin[i]);
                e->setInformation(Eigen::Matrix<double, 3, 3>::Identity()*FLAGS_gps_weight);
                optimizer.addEdge(e);
                lidar_edges.push_back(e);
            }
        }
        std::cout<<"add lidar edge"<<std::endl;
        
        const float thHuber2D = sqrt(5.99);
        const float thHuber3D = sqrt(7.815);
        
        //add projection edge, number equal to mappoints*track
        //add mappoint vertice
        std::vector<g2o::VertexSBAPointXYZ*> mp_verts;
        std::vector<g2o::EdgeSE3ProjectXYZ*> proj_edges;
        for(int i=0; i<mp_infos.size(); i++){
            if(mp_mask[i]==false){
                mp_verts.push_back(0);
                continue;
            }
            for(int j=0; j<mp_infos[i].size(); j++){
                if(j==0){
                    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
                    //std::cout<<mp_posis_out[mp_infos[i][j].mp_id].transpose()<<std::endl;
                    vPoint->setEstimate(mp_posis_out[mp_infos[i][j].mp_id]);
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
                
//                 Eigen::Vector3d mp_posi = mp_verts.back()->estimate();
//                 double t_u = obs(0);
//                 double t_v = obs(1);
//                 Eigen::Matrix4d proj_pose = kf_verts[mp_infos[i][j].frame_id]->estimate().to_homogeneous_matrix();
//                 Eigen::Vector4d mp_homo;
//                 mp_homo.block(0,0,3,1)=mp_posi;
//                 mp_homo(3)=1;
//                 double fx = cam_inter(0,0);
//                 double fy = cam_inter(1,1);
//                 double cx = cam_inter(0,2);
//                 double cy = cam_inter(1,2);
//                 //std::cout<<t_u<<":"<<t_v<<std::endl;
//                 Eigen::Vector4d mp_cam = proj_pose*mp_homo;
//                 //std::cout<<mp_cam.transpose()<<std::endl;
//                 double u = mp_cam(0)*fx/mp_cam(2)+cx;
//                 double v = mp_cam(1)*fy/mp_cam(2)+cy;
//                 double err=sqrt((u-t_u)*(u-t_u)+(v-t_v)*(v-t_v));
//                 std::cout<<"err: "<<err<<std::endl;
                
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
                
                if(true){
                    //std::cout<<mp_verts.back()->estimate().transpose()<<std::endl;
                    e->computeError();
                    //std::cout<<sqrt(e->chi2())<<std::endl;
                }
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
            //std::cout<<"avg_error: "<<avg_error<<"||"<<sqrt(proj_edges[i]->chi2())<<std::endl;
            if(avg_error<0){
                return;
            }
            
        }
        std::cout<<"project edge err before: "<<avg_error<<std::endl;
        clock_t time;
        time = clock();
        optimizer.initializeOptimization();
        for(int i=0; i<1; i++){
            optimizer.optimize(FLAGS_opti_count);
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
            if(mp_verts[i]!=0){
                mp_posis_out[i]=mp_verts[i]->estimate();
            }
        }
        poses_out.resize(kf_verts.size());
        for(int i=0; i<kf_verts.size(); i++){
            poses_out[i]=kf_verts[i]->estimate().inverse().to_homogeneous_matrix();
        }
    }
    
    void optimize_gps_pose(std::string map_addr, std::string map_name){
        std::cout<<"start lidar opti"<<std::endl;
        
        vm::VisualMap map;
        vm::loader_visual_map(map, map_addr+"/"+map_name);
        map.ComputeUniqueId();
        std::vector<Eigen::Matrix4d> poses_alin;
        for(int i=0; i<map.frames.size(); i++){
            poses_alin.push_back(map.frames[i]->getPose());
        }
        
        std::vector<int> gps_inliers;
        std::vector<Eigen::Vector3d> gps_alins;
        for(int i=0; i<map.frames.size(); i++){
            if(map.frames[i]->gps_accu<30){
                gps_inliers.push_back(1);
            }else{
                gps_inliers.push_back(0);
            }
            gps_alins.push_back(map.frames[i]->gps_position);
        }
        
        std::vector<std::vector<orb_slam::MP_INFO>> mp_infos;
        for(int i=0; i<map.mappoints.size(); i++){
            std::vector<orb_slam::MP_INFO> track_info;
            for(int j=0; j<map.mappoints[i]->track.size(); j++){
                orb_slam::MP_INFO info;
                map.mappoints[i]->track[j].getUV(info.u, info.v, info.octove);
                info.frame_id=map.mappoints[i]->track[j].frame->id;
                info.mp_id=map.mappoints[i]->id;
                track_info.push_back(info);
            }
            mp_infos.push_back(track_info);
        }
        Eigen::Matrix3d cam_inter;
        if(map.frames.size()>0){
            cam_inter=map.frames[0]->getKMat();
        }else{
            std::cout<<"[optimize_gps_pose][error]frame is empty"<<std::endl;
            exit(0);
        }
        
        std::vector<Eigen::Matrix4d> poses_out;
        std::vector<Eigen::Vector3d> mp_posis_out;
        optimize_true_pose(poses_alin, gps_alins, gps_inliers, mp_infos, cam_inter, poses_out, mp_posis_out);
        for(int i=0; i<poses_out.size(); i++){
            map.frames[i]->setPose(poses_out[i]);
        }
        for(int i=0; i<mp_posis_out.size(); i++){
            map.mappoints[i]->position=mp_posis_out[i];
        }
        vm::save_visual_map(map, map_addr+"/opti_"+map_name);
    }
}
