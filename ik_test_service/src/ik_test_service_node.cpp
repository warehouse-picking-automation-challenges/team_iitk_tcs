#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>
#include <stdlib.h>

#include "ik_test_service/PoseJoint7dof.h"
#include "ik_test_service/wam_ik_f7.h"
#include <std_msgs/Int16.h>

#define PRE_DEFINED_ORIENTATION
//#define DEBUG

bool getCentroidIKCallback(ik_test_service::PoseJoint7dof::Request &req, ik_test_service::PoseJoint7dof::Response &res)
{
    std::cout << "Received IK Fast for centroid service" << std::endl;
    geometry_msgs::Pose trg_pose;
    trg_pose = req.pose;

    KDL::Rotation target_RMat;
    target_RMat = KDL::Rotation::Quaternion(trg_pose.orientation.x, trg_pose.orientation.y, trg_pose.orientation.z, trg_pose.orientation.w);

    IkReal eerot[9],eetrans[3];

    eerot[0] = target_RMat.data[0]; eerot[1] = target_RMat.data[1]; eerot[2] = target_RMat.data[2]; eetrans[0] = trg_pose.position.x;
    eerot[3] = target_RMat.data[3]; eerot[4] = target_RMat.data[4]; eerot[5] = target_RMat.data[5]; eetrans[1] = trg_pose.position.y;
    eerot[6] = target_RMat.data[6]; eerot[7] = target_RMat.data[7]; eerot[8] = target_RMat.data[8]; eetrans[2] = trg_pose.position.z;

#ifdef PRE_DEFINED_ORIENTATION
    IkReal *jts = (IkReal*) malloc(sizeof(IkReal)*7);

//    double binA[7] = {0.201536, 0.608054, 0.225233, 0.519082, -0.398282, -0.0337634, 0.324263};
//    double binB[7] = {-0.0933172, 0.543762, 0.187557, 0.533825, -0.170715, 0.0334471, 0.18268};
//    double binC[7] = {-0.376848, 0.606017, 0.097017, 0.525985, -0.146044, -0.0986018, 0.304639};

//    double binD[7] = {-0.0120162, 0.500404, 0.477696, 1.05342, -0.378514, -0.0323401, 0.315427};
//    double binE[7] = {0.0140615, 0.376002, -0.0875296, 1.38007, -0.285447, -0.367207, 0.419918};
//    double binF[7] = {-0.209023, 0.454737, -0.233853, 1.25411, -0.297149, -0.352183, 0.60938};

//    double binG[7] = {0.633498, 0.674979, -0.565362, 1.60829, 0.00181864, -0.706817, 0.514135};
//    double binH[7] = {-0.097773, 0.545853, 0.0223955, 1.86779, -0.18811, -0.942766, 0.354881};
//    double binI[7] = {-0.572029, 0.627466, 0.108238, 1.64162, -0.519181, -0.859978, 0.466256};

//    double binJ[7] = {0.369032, 0.923674, -0.108146, 1.83549, 0.0536103, -1.17516, 0.181858};
//    double binK[7] = {-0.0212931, 0.827155, -0.047619, 2.11911, -0.0483125, -1.33496, 0.117129};
//    double binL[7] = {-0.428127, 0.893944, -0.0826491, 1.90597, -0.345857, -1.26229, 0.245457};

    double binA[7] = {};
    double binB[7] = {};
    double binC[7] = {};

//    double binD[7] = {1.844, 0.300, 0.053, 1.004, -0.169, -0.039, 0.896};
//    double binE[7] = {1.353, 0.198, 0.195, 1.115, -0.142, 0.014, 0.897};
//    double binF[7] = {1.216, 0.310, -0.208, 1.078, -0.006, -0.170, 0.899};

//    double binG[7] = {1.603, 0.280, 0.389, 1.577, -0.161, -0.276, 0.891};
//    double binH[7] = {1.566, 0.187, -0.109, 1.643, -0.060, -0.143, 0.900};
//    double binI[7] = {1.292, 0.329, -0.297, 1.551, -0.002, -0.288, 0.902};

    double binD[7] = {1.306, -0.174,  0.593,  2.209, -0.047, -0.555,  0.066};
    double binE[7] = {1.551, -0.394,  0.047,  2.226, -0.067, -0.272, -0.068};
    double binF[7] = {1.730, -0.136, -0.447,  2.308,  0.161, -0.762, -0.310};

    double binG[7] = {1.306,  0.008,  0.641,  2.598, -0.060, -0.910,  0.001};
    double binH[7] = {1.601, -0.240,  0.007,  2.622, -0.026, -0.628, -0.113};
    double binI[7] = {1.737,  0.007, -0.455,  2.725,  0.187, -1.117, -0.248};

    double binJ[7] = {};
    double binK[7] = {};
    double binL[7] = {};

    int bin_num = req.bin_num.data;

    switch(bin_num)
    {
    case 0: for(int i = 0; i < 7; i++)
                jts[i] = binA[i];
            std::cout << "Computin IK for Bin A" << std::endl;
                break;
    case 1: for(int i = 0; i < 7; i++)
                jts[i] = binB[i];
            std::cout << "Computin IK for Bin B" << std::endl;
                break;
    case 2: for(int i = 0; i < 7; i++)
                jts[i] = binC[i];
            std::cout << "Computin IK for Bin C" << std::endl;
                break;
    case 3: for(int i = 0; i < 7; i++)
                jts[i] = binD[i];
            std::cout << "Computin IK for Bin D" << std::endl;
                break;
    case 4: for(int i = 0; i < 7; i++)
                jts[i] = binE[i];
            std::cout << "Computin IK for Bin E" << std::endl;
                break;
    case 5: for(int i = 0; i < 7; i++)
                jts[i] = binF[i];
            std::cout << "Computin IK for Bin F" << std::endl;
                break;
    case 6: for(int i = 0; i < 7; i++)
                jts[i] = binG[i];
            std::cout << "Computin IK for Bin G" << std::endl;
                break;
    case 7: for(int i = 0; i < 7; i++)
                jts[i] = binH[i];
            std::cout << "Computin IK for Bin H" << std::endl;
                break;
    case 8: for(int i = 0; i < 7; i++)
                jts[i] = binI[i];
            std::cout << "Computin IK for Bin I" << std::endl;
                break;
    case 9: for(int i = 0; i < 7; i++)
                jts[i] = binJ[i];
            std::cout << "Computin IK for Bin J" << std::endl;
                break;
    case 10: for(int i = 0; i < 7; i++)
                jts[i] = binK[i];
            std::cout << "Computin IK for Bin K" << std::endl;
                break;
    case 11: for(int i = 0; i < 7; i++)
                jts[i] = binL[i];
            std::cout << "Computin IK for Bin L" << std::endl;
                break;
    default: std::cout << "No Bin number input\nTaking Bin K default value";
             for(int i = 0; i < 7; i++)
                jts[i] = binK[i];
    }
    ComputeFk(jts, eetrans, eerot);
#endif

#ifdef DEBUG
    std::cout << "Orientation: \n" << eerot[0] << " " << eerot[1] << " " << eerot[2] << "\n"
                                   << eerot[3] << " " << eerot[4] << " " << eerot[5] << "\n"
                                   << eerot[6] << " " << eerot[7] << " " << eerot[8] << "\n";
    std::cout << "Suction Position: [" << trg_pose.position.x << "," << trg_pose.position.y << "," << trg_pose.position.z << "]" << std::endl;
#endif

//    double pipe_length = 0.16;//stow task pipe length
    double pipe_length = 0.43;// pick task pipe length
    geometry_msgs::Point pt_at_eef;
    tf::Vector3 z_vector(eerot[2], eerot[5], eerot[8]);
    z_vector.normalize();
    double x_offset = 0.00, y_offset = 0.00, z_offset = 0.00;
//    double x_offset = 0.0, y_offset = 0.00, z_offset = 0.0;
    pt_at_eef.x =  trg_pose.position.x + x_offset - pipe_length*z_vector.x();
    pt_at_eef.y =  trg_pose.position.y + y_offset - pipe_length*z_vector.y();
    pt_at_eef.z =  trg_pose.position.z + z_offset - pipe_length*z_vector.z();

#ifdef DEBUG
    std::cout << "Joint7 Position: [" << pt_at_eef.x << "," << pt_at_eef.y << "," << pt_at_eef.z << "]" << std::endl;
#endif

//    eetrans[0] = pt_at_eef.x+0.041; eetrans[1] = pt_at_eef.y; eetrans[2] = pt_at_eef.z+0.522;
    eetrans[0] = pt_at_eef.x+0.22; eetrans[1] = pt_at_eef.y+0.14; eetrans[2] = pt_at_eef.z+0.406;

    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());

    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = 0;
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        for( std::size_t j = 0; j < GetNumJoints(); ++j)
            res.joint_angles.data.push_back(0);
        fprintf(stderr,"Failed to get ik solution\n");
        return false;
    }
    else
    {
        printf("/*********************************************************/\n");
#ifdef DEBUG
        printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
#endif
        std::vector<IkReal> solvalues(GetNumJoints());

        double least_norm = 1000;
        int least_norm_idx;
        for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
            const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
#ifdef DEBUG
            printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
#endif
            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#ifdef DEBUG
            for( std::size_t j = 0; j < solvalues.size(); ++j)
                printf("%.3f, ", solvalues[j]);
            printf("\n");
#endif
            double sum = 0;
            double start_pos[7] = {-0.0574878, 0.594913, 0.0453384, 1.16046, -0.201394, -0.0951226, -0.341729};
            for( std::size_t j = 0; j < solvalues.size(); ++j)
                sum += (solvalues[j] - start_pos[j])*(solvalues[j] - start_pos[j]);
            double norm_sq_avg = sum/solvalues.size();

            if(norm_sq_avg<least_norm)
            {
                least_norm = norm_sq_avg;
                least_norm_idx = i;
            }
        }
#ifdef DEBUG
        std::cout << "Selected " << least_norm_idx << " ik solution" << std::endl;
#endif
//        copy the solution with minimum norm from home position
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(least_norm_idx);
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        std::cout << "Sent:[";
        for( std::size_t j = 0; j < solvalues.size(); ++j)
        {
            std::cout << solvalues[j] << " ";
            res.joint_angles.data.push_back(solvalues[j]);
        }
        std::cout << "]\nGot IK Solution\n";
        printf("/*********************************************************/\n");
        return true;
    }
}

bool getKinectIKCallback(ik_test_service::PoseJoint7dof::Request &req, ik_test_service::PoseJoint7dof::Response &res)
{
    std::cout << "Received IK Fast for kinect view service" << std::endl;
    geometry_msgs::Pose trg_pose;
    trg_pose = req.pose;

    KDL::Rotation target_RMat;
    target_RMat = KDL::Rotation::Quaternion(trg_pose.orientation.x, trg_pose.orientation.y, trg_pose.orientation.z, trg_pose.orientation.w);

    IkReal eerot[9],eetrans[3];

    eerot[0] = target_RMat.data[0]; eerot[1] = target_RMat.data[1]; eerot[2] = target_RMat.data[2]; eetrans[0] = trg_pose.position.x;
    eerot[3] = target_RMat.data[3]; eerot[4] = target_RMat.data[4]; eerot[5] = target_RMat.data[5]; eetrans[1] = trg_pose.position.y;
    eerot[6] = target_RMat.data[6]; eerot[7] = target_RMat.data[7]; eerot[8] = target_RMat.data[8]; eetrans[2] = trg_pose.position.z;

#ifdef PRE_DEFINED_ORIENTATION
    IkReal *jts = (IkReal*) malloc(sizeof(IkReal)*7);
    double p_len;
    double trg_reduce[3] = {0.0, 0.0, 0.0};
    double binA[7] = {1.5449, -0.0602461, -0.0980204, 1.73135, 0.0479962, -0.0612802, 0.414781};    trg_reduce[0]=0.0; trg_reduce[1]=0.0, trg_reduce[2]=0.15; p_len = 0.43;
    double binB[7] = {1.5449, -0.0602461, -0.0980204, 1.73135, 0.0479962, -0.0612802, 0.414781};    trg_reduce[0]=0.0; trg_reduce[1]=0.0, trg_reduce[2]=0.15; p_len = 0.43;
    double binC[7] = {1.5449, -0.0602461, -0.0980204, 1.73135, 0.0479962, -0.0612802, 0.414781};    trg_reduce[0]=0.0; trg_reduce[1]=0.0, trg_reduce[2]=0.15; p_len = 0.43;

    double binD[7] = {1.48088, -0.212993, -0.0015052, 2.11562, 0.0528195, 0.00126514, 0.314194};    trg_reduce[0]=0.0; trg_reduce[1]=0.0, trg_reduce[2]=0.12; p_len = 0.46;
    double binE[7] = {1.48088, -0.212993, -0.0015052, 2.11562, 0.0528195, 0.00126514, 0.314194};    trg_reduce[0]=0.0; trg_reduce[1]=0.0, trg_reduce[2]=0.12; p_len = 0.46;
    double binF[7] = {1.48088, -0.212993, -0.0015052, 2.11562, 0.0528195, 0.00126514, 0.314194};    trg_reduce[0]=0.0; trg_reduce[1]=0.0, trg_reduce[2]=0.12; p_len = 0.46;

    // row 3 kinect view angles decrease centroid z by 12cm, decrease centroid y by 23cm and
    // pipe length should be considered from the bending position

    double binG[7] = {1.5976, -0.741822, -0.0710181, 2.65285, -0.0514753, -0.071085, -0.143535};    trg_reduce[0]=0.0; trg_reduce[1]=0.23, trg_reduce[2]=0.12; p_len = 0.43;
    double binH[7] = {1.5976, -0.741822, -0.0710181, 2.65285, -0.0514753, -0.071085, -0.143535};    trg_reduce[0]=0.0; trg_reduce[1]=0.23, trg_reduce[2]=0.12; p_len = 0.43;
    double binI[7] = {1.5976, -0.741822, -0.0710181, 2.65285, -0.0514753, -0.071085, -0.143535};    trg_reduce[0]=0.0; trg_reduce[1]=0.23, trg_reduce[2]=0.12; p_len = 0.43;

    // row 4 kinect view angles decrease centroid z by -2cm, decrease centroid y by 43cm and
    // pipe length should be considered from the bending position
    double binJ[7] = {1.63983, -0.989893, -0.0660463, 2.98939, 0.0375588, -0.36523, -0.0490093}; trg_reduce[0]=0.0; trg_reduce[1]=0.43, trg_reduce[2]=-0.02; p_len = 0.43;
    double binK[7] = {1.63983, -0.989893, -0.0660463, 2.98939, 0.0375588, -0.36523, -0.0490093}; trg_reduce[0]=0.0; trg_reduce[1]=0.43, trg_reduce[2]=-0.02; p_len = 0.43;
    double binL[7] = {1.63983, -0.989893, -0.0660463, 2.98939, 0.0375588, -0.36523, -0.0490093}; trg_reduce[0]=0.0; trg_reduce[1]=0.43, trg_reduce[2]=-0.02; p_len = 0.43;

    int bin_num = req.bin_num.data;
    switch(bin_num)
    {
    case 0: for(int i = 0; i < 7; i++)
                jts[i] = binA[i];
            std::cout << "Computin IK for Bin A" << std::endl;
                break;
    case 1: for(int i = 0; i < 7; i++)
                jts[i] = binB[i];
            std::cout << "Computin IK for Bin B" << std::endl;
                break;
    case 2: for(int i = 0; i < 7; i++)
                jts[i] = binC[i];
            std::cout << "Computin IK for Bin C" << std::endl;
                break;
    case 3: for(int i = 0; i < 7; i++)
                jts[i] = binD[i];
            std::cout << "Computin IK for Bin D" << std::endl;
                break;
    case 4: for(int i = 0; i < 7; i++)
                jts[i] = binE[i];
            std::cout << "Computin IK for Bin E" << std::endl;
                break;
    case 5: for(int i = 0; i < 7; i++)
                jts[i] = binF[i];
            std::cout << "Computin IK for Bin F" << std::endl;
                break;
    case 6: for(int i = 0; i < 7; i++)
                jts[i] = binG[i];
            std::cout << "Computin IK for Bin G" << std::endl;
                break;
    case 7: for(int i = 0; i < 7; i++)
                jts[i] = binH[i];
            std::cout << "Computin IK for Bin H" << std::endl;
                break;
    case 8: for(int i = 0; i < 7; i++)
                jts[i] = binI[i];
            std::cout << "Computin IK for Bin I" << std::endl;
                break;
    case 9: for(int i = 0; i < 7; i++)
                jts[i] = binJ[i];
            std::cout << "Computin IK for Bin J" << std::endl;
                break;
    case 10: for(int i = 0; i < 7; i++)
                jts[i] = binK[i];
            std::cout << "Computin IK for Bin K" << std::endl;
                break;
    case 11: for(int i = 0; i < 7; i++)
                jts[i] = binL[i];
            std::cout << "Computin IK for Bin L" << std::endl;
                break;
    default: std::cout << "No Bin number input\nTaking Bin K default value";
             for(int i = 0; i < 7; i++)
                jts[i] = binK[i];
    }
    ComputeFk(jts, eetrans, eerot);
#endif

#ifdef DEBUG
    std::cout << "Orientation: \n" << eerot[0] << " " << eerot[1] << " " << eerot[2] << "\n"
                                   << eerot[3] << " " << eerot[4] << " " << eerot[5] << "\n"
                                   << eerot[6] << " " << eerot[7] << " " << eerot[8] << "\n";
    std::cout << "Suction Position: [" << trg_pose.position.x << "," << trg_pose.position.y << "," << trg_pose.position.z << "]" << std::endl;
#endif

    double pipe_length = p_len;
    geometry_msgs::Point pt_at_eef;
    tf::Vector3 z_vector(eerot[2], eerot[5], eerot[8]);
    z_vector.normalize();
    pt_at_eef.x =  trg_pose.position.x - trg_reduce[0] - pipe_length*z_vector.x();
    pt_at_eef.y =  trg_pose.position.y - trg_reduce[1] - pipe_length*z_vector.y();
    pt_at_eef.z =  trg_pose.position.z - trg_reduce[2] - pipe_length*z_vector.z();

#ifdef DEBUG
    std::cout << "Joint7 Position: [" << pt_at_eef.x << "," << pt_at_eef.y << "," << pt_at_eef.z << "]" << std::endl;
#endif

//    eetrans[0] = pt_at_eef.x+0.041; eetrans[1] = pt_at_eef.y; eetrans[2] = pt_at_eef.z+0.522;
    eetrans[0] = pt_at_eef.x+0.22; eetrans[1] = pt_at_eef.y+0.14; eetrans[2] = pt_at_eef.z+0.406;
    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());

    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = 0;
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    cout << "bSuccess " << bSuccess << endl;
    if( !bSuccess ) {
        for( std::size_t j = 0; j < GetNumJoints(); ++j)
//            res.joint_angles.data.push_back(0);
            res.joint_angles.data.push_back(jts[j]);
        if(bin_num == 0 ||bin_num == 1 || bin_num == 2 || bin_num == 3 || bin_num == 4 || bin_num == 5)
        {
            res.joint_angles.data[4] = 0.77;
            res.joint_angles.data[5] = 1.0;
        }
//        fprintf(stderr,"Failed to get ik solution\n");
        cout << "Generated default joint angles" << endl;
        return true;
    }
    else
    {
        printf("/*********************************************************/\n");
#ifdef DEBUG
        printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
#endif
        std::vector<IkReal> solvalues(GetNumJoints());

        double least_norm = 1000;
        int least_norm_idx;
        for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
            const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
#ifdef DEBUG
            printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
#endif
            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#ifdef DEBUG
            for( std::size_t j = 0; j < solvalues.size(); ++j)
                printf("%.3f, ", solvalues[j]);
            printf("\n");
#endif
            double sum = 0;
            double start_pos[7] = {1.51141, -0.75, -0.0137749, 2.0, -0.00166049, -0.252158, 0.658184};
            for( std::size_t j = 0; j < solvalues.size(); ++j)
                sum += (solvalues[j] - start_pos[j])*(solvalues[j] - start_pos[j]);
            double norm_sq_avg = sum/solvalues.size();

            if(norm_sq_avg<least_norm)
            {
                least_norm = norm_sq_avg;
                least_norm_idx = i;
            }
        }
#ifdef DEBUG
        std::cout << "Selected " << least_norm_idx << " ik solution" << std::endl;
#endif
//        copy the solution with minimum norm from home position
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(least_norm_idx);
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

//        To bend the pipe for kinect view
        solvalues[4] = 0.77;
        solvalues[5] = 1.0;
        std::cout << "Sent:[";
        for( std::size_t j = 0; j < solvalues.size(); ++j)
        {
            std::cout << solvalues[j] << " ";
            res.joint_angles.data.push_back(solvalues[j]);
        }

        std::cout << "]\nGot IK Solution\n";
        printf("/*********************************************************/\n");
        return true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ik_test_service");
    ros::NodeHandle node;

    ros::ServiceServer service_centroid_wam_ik;
    service_centroid_wam_ik = node.advertiseService("/ik_centre_joints", getCentroidIKCallback);
    ros::ServiceServer service_kinect_wam_ik;
    service_kinect_wam_ik = node.advertiseService("/ik_kinect_view_joints", getKinectIKCallback);
    ROS_INFO("Ready to get IK for WAM 7DOF.");

    while(ros::ok())
        ros::spinOnce();
    return 0;

}
