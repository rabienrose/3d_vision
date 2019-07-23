#include <string>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include <math.h>

#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_seven_dof_expmap.h"
#include "imu_tools.h"
#include "read_write_data_lib/read_write.h"
#include "opencv2/opencv.hpp"
#include "test_imu_tool/visual_tool.h"
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_int32(opti_count, 100, "How many of the iteration of optimization");
DEFINE_double(gps_weight, 0.0001, "The weight of GPS impact in optimization");
DEFINE_double(t_c_g_x, 0, "gps position in camera coordinate");
DEFINE_double(t_c_g_y, 0, "gps position in camera coordinate");
DEFINE_double(t_c_g_z, 0, "gps position in camera coordinate");

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
    
    class EdgePosiPreSim3 : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSim3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePosiPreSim3(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  {
        const g2o::VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
        _error= v1->estimate().inverse().translation()-_measurement;
//         std::cout<<v1->estimate().inverse().translation().transpose()<<std::endl;
//         std::cout<<_measurement.transpose()<<std::endl;
        }
    };
}


namespace OptimizerTool
{

    void optimize_sim3_graph( std::vector<Eigen::Vector3d>& gps_alin, std::vector<int>& gps_inlers,
        std::vector<Eigen::Matrix4d>& poses_out, std::vector<Eigen::Matrix4d>& poses_in,
        std::vector<Eigen::Matrix4d>& T_1_to_2_list, std::vector<double>& scale_1_to_2_list, 
        std::vector<int>& graph_v1_list, std::vector<int>& graph_v2_list, 
        std::vector<double>& graph_weight, 
        bool input_is_sim
    ){
        CHECK_EQ(gps_alin.size(), gps_inlers.size());
        CHECK_EQ(gps_alin.size(), poses_in.size());
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            new g2o::BlockSolver_7_3(
                new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>()));

        solver->setUserLambdaInit(1e-16);
        optimizer.setAlgorithm(solver);
        
        std::vector<g2o::VertexSim3Expmap*> v_sim3_list;
        std::vector<g2o::Sim3> sim3_list;
        
        for(int i=0; i<poses_in.size(); i++){
            g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();
            
            Eigen::Matrix4d pose_inv=poses_in[i].inverse();

            Eigen::Matrix<double,3,3> Rcw = pose_inv.block(0,0,3,3);
            Eigen::Matrix<double,3,1> tcw = pose_inv.block(0,3,3,1);
            g2o::Sim3 Siw;
            double scale=Rcw.block(0,0,3,1).norm();
            Rcw=Rcw/scale;
            Siw=g2o::Sim3(Rcw,tcw,scale);
            
            VSim3->setEstimate(Siw);
            if(i==0){
                VSim3->setFixed(true);
            }else{
                VSim3->setFixed(false);
            }
            
            VSim3->setId(i);
            VSim3->setMarginalized(false);
            VSim3->_fix_scale = false;

            optimizer.addVertex(VSim3);
            v_sim3_list.push_back(VSim3);
            sim3_list.push_back(Siw);
        }
        std::cout<<"add sim3 vertices"<<std::endl;
        
        std::vector<g2o::EdgePosiPreSim3*> gps_edges;
        for(int i=0; i<poses_in.size(); i++){
            if(gps_inlers[i]==1){
                g2o::EdgePosiPreSim3* e = new g2o::EdgePosiPreSim3();
                //std::cout<<v_sim3_list[i]->estimate().inverse().translation().transpose()<<std::endl;
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_sim3_list[i]));
                e->setMeasurement(gps_alin[i]);
                e->setInformation(Eigen::Matrix<double, 3, 3>::Identity()*FLAGS_gps_weight);
                optimizer.addEdge(e);
                gps_edges.push_back(e);
                e->computeError();
            }
        }
        std::cout<<"add gps edge: "<<gps_edges.size()<<std::endl;
        
        std::vector<g2o::EdgeSim3*> sim3_edge_list;
        for(int i=0; i<T_1_to_2_list.size(); i++){
            g2o::Sim3 Sji(T_1_to_2_list[i].block(0,0,3,3),T_1_to_2_list[i].block(0,3,3,1),scale_1_to_2_list[i]);
            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            //std::cout<<"asdfasdf"<<std::endl;
            //std::cout<<"v1: "<<v_sim3_list[graph_v1_list[i]]->estimate().inverse().translation().transpose()<<std::endl;
            //std::cout<<"obs: "<<(v_sim3_list[graph_v2_list[i]]->estimate().inverse().rotation().toRotationMatrix()*Sji.translation()+v_sim3_list[graph_v2_list[i]]->estimate().inverse().translation()).transpose()<<std::endl;
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_sim3_list[graph_v1_list[i]]));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_sim3_list[graph_v2_list[i]]));
            e->setMeasurement(Sji);
            Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
            if(graph_weight[i]>0){
                e->information() = matLambda*graph_weight[i];
            }else{
                e->information() = matLambda*1;
            }
            
//             if(input_is_sim || graph_weight[i]>0){
                optimizer.addEdge(e);
                sim3_edge_list.push_back(e);
//             }else{
//                 e->computeError();
//                 if(sqrt(e->chi2())<2){
//                     optimizer.addEdge(e);
//                     sim3_edge_list.push_back(e);
//                 }
//             }
        }
        std::cout<<"add sim3 edge"<<std::endl;
        
        float avg_error=0;
        for(int i=0; i<gps_edges.size(); i++){
            gps_edges[i]->computeError();
            avg_error=avg_error+sqrt(gps_edges[i]->chi2())/gps_edges.size();
        }
        std::cout<<"gps edge err before: "<<avg_error<<std::endl;
        avg_error=0;
        for(int i=0; i<sim3_edge_list.size(); i++){
            sim3_edge_list[i]->computeError();
            avg_error=avg_error+sqrt(sim3_edge_list[i]->chi2())/sim3_edge_list.size();
//             if(sqrt(sim3_edge_list[i]->chi2())>1){
//                 std::cout<<"avg_error: "<<avg_error<<"||"<<sqrt(sim3_edge_list[i]->chi2())<<std::endl;
//             }
            if(avg_error<0){
                return;
            }
        }
        std::cout<<"sim3 edge err before: "<<avg_error<<std::endl;
        
        optimizer.initializeOptimization();
        optimizer.computeInitialGuess();
        optimizer.optimize(FLAGS_opti_count);
        
        avg_error=0;
        for(int i=0; i<gps_edges.size(); i++){
            gps_edges[i]->computeError();
            avg_error=avg_error+sqrt(gps_edges[i]->chi2())/gps_edges.size();
            
        }
        std::cout<<"gps edge err after: "<<avg_error<<std::endl;
        avg_error=0;
        for(int i=0; i<sim3_edge_list.size(); i++){
            sim3_edge_list[i]->computeError();
            avg_error=avg_error+sqrt(sim3_edge_list[i]->chi2())/sim3_edge_list.size();
            if(avg_error<0){
                return;
            }
            
        }
        std::cout<<"sim3 edge err after: "<<avg_error<<std::endl;
        poses_out.resize(sim3_list.size());
        for(int i=0; i<sim3_list.size(); i++){
            g2o::Sim3 CorrectedSiw =  v_sim3_list[i]->estimate();
            Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = CorrectedSiw.translation();
            double s = CorrectedSiw.scale();
            Eigen::Matrix4d Tiw=Eigen::Matrix4d::Identity();
            if(input_is_sim){
                Tiw.block(0,0,3,3)=eigR*s;
                Tiw.block(0,3,3,1)=eigt;
            }else{
                eigt *=(1./s); //[R t/s;0 1]
                Tiw.block(0,0,3,3)=eigR;
                Tiw.block(0,3,3,1)=eigt;
            }
            poses_out[i]=Tiw.inverse();
        }
    }

    void optimize_sim3_graph(std::string map_addr, std::string map_name){
        vm::VisualMap map;
        std::cout<<"pose graph: "<<map_addr+"/"+map_name<<std::endl;
        vm::loader_visual_map(map, map_addr+"/"+map_name);
        map.ComputeUniqueId();
        std::vector<Eigen::Matrix4d> poses_out;
        std::vector<Eigen::Matrix4d> poses_in;
        std::vector<Eigen::Matrix4d> T_2_1_list;
        std::vector<double> scale_1_to_2_list;
        std::vector<int> graph_v1_list;
        std::vector<int> graph_v2_list;
        std::vector<int> gps_inliers;
        std::vector<Eigen::Vector3d> gps_alins;
        for(int i=0; i<map.frames.size(); i++){
            if(map.frames[i]->gps_accu<10){
                gps_inliers.push_back(1);
            }else{
                gps_inliers.push_back(0);
            }
            gps_alins.push_back(map.frames[i]->gps_position);
            //std::cout<<map.frames[i]->gps_position.transpose()<<std::endl;
            //std::cout<<map.frames[i]->position.transpose()<<std::endl;
            poses_in.push_back(map.frames[i]->getPose());
        }
        scale_1_to_2_list=map.pose_graph_e_scale;
        for(int i=0; i<map.pose_graph_e_rot.size(); i++){
            Eigen::Matrix4d T_temp=Eigen::Matrix4d::Identity();
            T_temp.block(0,0,3,3)=map.pose_graph_e_rot[i];
            T_temp.block(0,3,3,1)=map.pose_graph_e_posi[i];
            T_2_1_list.push_back(T_temp);
            graph_v1_list.push_back(map.pose_graph_v1[i]->id);
            graph_v2_list.push_back(map.pose_graph_v2[i]->id);
        }
        optimize_sim3_graph(gps_alins, gps_inliers, poses_out, poses_in, T_2_1_list, scale_1_to_2_list, graph_v1_list, graph_v2_list, map.pose_graph_weight, false);
        //std::vector<Eigen::Matrix4d> poses_out2;
        //optimize_sim3_graph(gps_alins, gps_inliers, poses_out2, poses_out, T_2_1_list, scale_1_to_2_list, graph_v1_list, graph_v2_list, map.pose_graph_weight, false);
        for(int i=0; i<map.frames.size(); i++){
            map.frames[i]->setPose(poses_out[i]);
        }
        vm::save_visual_map(map, map_addr+"/graph_"+map_name);
    }
}
