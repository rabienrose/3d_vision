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

    void optimize_sim3_graph( std::vector<Eigen::Vector3d>& gps_alin, std::vector<int>& gps_inlers,
        std::vector<Eigen::Matrix4d>& poses_out, std::vector<Eigen::Matrix4d>& poses_in, std::vector<std::pair<int, int>> edge_ids
    ){
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
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            VSim3->setEstimate(Siw);
            VSim3->setFixed(false);
            VSim3->setId(i);
            VSim3->setMarginalized(false);
            VSim3->_fix_scale = false;

            optimizer.addVertex(VSim3);
            v_sim3_list.push_back(VSim3);
            sim3_list.push_back(Siw);
        }
        std::cout<<"add sim3 vertices"<<std::endl;
        
        std::vector<g2o::EdgePosiPre*> gps_edges;
        for(int i=0; i<poses_in.size(); i++){
            if(gps_inlers[i]==1){
                g2o::EdgePosiPre* e = new g2o::EdgePosiPre();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_sim3_list[i]));
                e->setMeasurement(gps_alin[i]);
                e->setInformation(Eigen::Matrix<double, 3, 3>::Identity()*1);
                optimizer.addEdge(e);
                gps_edges.push_back(e);
            }
        }
        std::cout<<"add gps edge"<<std::endl;
        
        
        std::vector<g2o::EdgeSim3*> sim3_edge_list;
        for(int i=0; i<edge_ids.size(); i++){
            g2o::Sim3 Siw = sim3_list[edge_ids[i].first];
            g2o::Sim3 Swi = Siw.inverse();
            
            g2o::Sim3 Sjw = sim3_list[edge_ids[i].second];
            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_sim3_list[edge_ids[i].second]));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_sim3_list[edge_ids[i].first]));
            e->setMeasurement(Sji);
            Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
            e->information() = matLambda;

            optimizer.addEdge(e);

            sim3_edge_list.push_back(e);
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
            //std::cout<<"avg_error: "<<avg_error<<"||"<<sqrt(proj_edges[i]->chi2())<<std::endl;
            if(avg_error<0){
                return;
            }
            
        }
        std::cout<<"sim3 edge err before: "<<avg_error<<std::endl;
        
        optimizer.initializeOptimization();
        optimizer.optimize(20);
        
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
            //std::cout<<"avg_error: "<<avg_error<<"||"<<sqrt(proj_edges[i]->chi2())<<std::endl;
            if(avg_error<0){
                return;
            }
            
        }
        std::cout<<"sim3 edge err after: "<<avg_error<<std::endl;
        
    }

    void optimize_sim3_graph(std::string res_root){
        
    }
}
