#include "orb_slam_lib/sim3_match.h"
#include <Eigen/Dense>
#include <vector>
namespace orb_slam
{
    
    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
        cv::reduce(P,C,1,cv::REDUCE_SUM);
        C = C/P.cols;

        for(int i=0; i<P.cols; i++)
        {
            Pr.col(i)=P.col(i)-C;
        }
    }
    
    void ComputeSim3(cv::Mat &P1, cv::Mat &P2, cv::Mat& mT12i)
    {
        cv::Mat mR12i;
        cv::Mat mt12i;
        float ms12i;
        std::vector<bool> mvbInliersi;
        int mnInliersi;

        cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
        cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
        cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
        cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

        ComputeCentroid(P1,Pr1,O1);
        ComputeCentroid(P2,Pr2,O2);

        // Step 2: Compute M matrix

        cv::Mat M = Pr2*Pr1.t();

        // Step 3: Compute N matrix

        double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

        cv::Mat N(4,4,P1.type());

        N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
        N12 = M.at<float>(1,2)-M.at<float>(2,1);
        N13 = M.at<float>(2,0)-M.at<float>(0,2);
        N14 = M.at<float>(0,1)-M.at<float>(1,0);
        N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
        N23 = M.at<float>(0,1)+M.at<float>(1,0);
        N24 = M.at<float>(2,0)+M.at<float>(0,2);
        N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
        N34 = M.at<float>(1,2)+M.at<float>(2,1);
        N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

        N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                    N12, N22, N23, N24,
                                    N13, N23, N33, N34,
                                    N14, N24, N34, N44);


        // Step 4: Eigenvector of the highest eigenvalue

        cv::Mat eval, evec;

        cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

        cv::Mat vec(1,3,evec.type());
        (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

        // Rotation angle. sin is the norm of the imaginary part, cos is the real part
        double ang=atan2(norm(vec),evec.at<float>(0,0));

        vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

        mR12i.create(3,3,P1.type());

        cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

        // Step 5: Rotate set 2

        cv::Mat P3 = mR12i*Pr2;

        // Step 6: Scale

        double nom = Pr1.dot(P3);
        cv::Mat aux_P3(P3.size(),P3.type());
        aux_P3=P3;
        cv::pow(P3,2,aux_P3);
        double den = 0;

        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }

        ms12i = nom/den;

        // Step 7: Translation

        mt12i.create(1,3,P1.type());
        mt12i = O1 - ms12i*mR12i*O2;

        // Step 8: Transformation

        // Step 8.1 T12
        mT12i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sR = ms12i*mR12i;

        sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
        mt12i.copyTo(mT12i.rowRange(0,3).col(3));
    }
    
    void ComputeSim3(std::vector<Eigen::Vector3d>& P1, std::vector<Eigen::Vector3d>& P2, Eigen::Matrix4d& T12i_eig){
        cv::Mat p1mat(3, P1.size(), CV_32FC1);
        cv::Mat p2mat(3, P2.size(), CV_32FC1);
        for(int i=0; i<P1.size(); i++){
            p1mat.at<float>(0, i)=P1[i](0);
            p1mat.at<float>(1, i)=P1[i](1);
            p1mat.at<float>(2, i)=P1[i](2);
            p2mat.at<float>(0, i)=P2[i](0);
            p2mat.at<float>(1, i)=P2[i](1);
            p2mat.at<float>(2, i)=P2[i](2);
        }
        cv::Mat pose_mat;
        ComputeSim3(p1mat, p2mat, pose_mat);
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                T12i_eig(i, j)=pose_mat.at<float>(i, j);
            }
        }
    }
}
