#ifndef TRAJECTORY_RPT_H
#define TRAJECTORY_RPT_H

#include "wam_ikfast_7dof_service/wam_ik_f7.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>

#define DEFAULT_OBJECT_POSITION


#define DEFAULT_ORIENTATION

//#define DEBUG

using namespace std;

#define NO_JOINTS 7
#define DEBUG 0

void obtainLeastDistJoints(int &samples, double recorded_jts[][7], IkReal* eerot, IkReal* eetrans, double &min_dist, geometry_msgs::Point &desired_eef, IkReal* min_jts,
                           double &length_x, double &length_z)
{
    IkReal *jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);
    for(int i=0; i<samples; i++)
    {
        for(int j=0; j<NO_JOINTS; j++)
            jts[j] = recorded_jts[i][j];

        ComputeFk(jts, eetrans, eerot);

        geometry_msgs::Point pt_eef;
        tf::Vector3 x_vector(eerot[0], eerot[3], eerot[6]), z_vector(eerot[2], eerot[5], eerot[8]);
        x_vector.normalize();
        z_vector.normalize();

        pt_eef.x =  desired_eef.x - length_z*z_vector.x() - length_x*x_vector.x();
        pt_eef.y =  desired_eef.y - length_z*z_vector.y() - length_x*x_vector.y();
        pt_eef.z =  desired_eef.z - length_z*z_vector.z() - length_x*x_vector.z();

        pt_eef.x = pt_eef.x+0.22; pt_eef.y = pt_eef.y+0.14; pt_eef.z = pt_eef.z+0.406;

        double dist = pow(eetrans[0]-pt_eef.x,2) + pow(eetrans[1]-pt_eef.y,2) + pow(eetrans[2]-pt_eef.z,2);
        if(dist<min_dist)
        {
            min_dist = dist;
            for(int j=0; j<NO_JOINTS; j++)
                min_jts[j] = jts[j];
        }
    }

    return;
}

bool computeMiddleIK(geometry_msgs::Point &point, int bin_idx, vector<double> &jt_angle)
{
    IkReal eerot[9],eetrans[3] = {0,0,0};
    IkReal *jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);
    double pipe_len_x = 0.06, pipe_len_z = 0.43;

    // compute forward kinematics for each sample joint angle and compare its eef_position with desired position
    // Choose the orientation matrix and joint angles which gives eef_position with least distance from desired position
    int n_samples = 1;
    double lst_dist = 10000.0;
    IkReal *lst_jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);

    if(bin_idx == 0 || bin_idx == 1 || bin_idx == 2)
    {
        double row1_binfront[][NO_JOINTS] = {{1.55297, -0.335114, 0.0259533, 1.5425, 0.164073, 0.284103, -0.134288},
                                             {1.55297, -0.257872, 0.0224412, 1.54225, 0.199022, 0.195701, -0.152268},
                                             {1.55297, -0.163254, 0.0254972, 1.48123, 0.22306, 0.157431, -0.16172},
                                             {1.55297, -0.0554405, 0.0327495, 1.36533, 0.216497, 0.173957, -0.170351},
                                             {1.55297, -0.324091, -0.139071, 1.37121, 0.411565, 0.564331, -0.358785},
                                             {1.55301, -0.266424, -0.119641, 1.38868, 0.355188, 0.469366, -0.353853},
                                             {1.55301, -0.170313, -0.0874384, 1.38817, 0.332178, 0.34989, -0.33988},
                                             {1.55301, -0.101976, -0.0892173, 1.38783, 0.331229, 0.247256, -0.345634},
                                             {1.55294, -0.32477, 0.127805, 1.42047, 0.184631, 0.42738, -0.105827},
                                             {1.55294, -0.249754, 0.111795, 1.42021, 0.189534, 0.34815, -0.12052},
                                             {1.55297, -0.162466, 0.120598, 1.41868, 0.189375, 0.251684, -0.162748},
                                             {1.55297, -0.0433587, 0.11289, 1.32306, 0.196176, 0.22132, -0.177132}
                                            };
        n_samples = 12;
        obtainLeastDistJoints(n_samples, row1_binfront, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_front[][NO_JOINTS] = {{1.49643, -0.202078, 0.043742, 1.58077, 0.0904574, 0.143277, 0.390533},
                                          {1.62766, 0.0131406, -0.161832, 1.41953, 0.0290191, 0.0593825, 0.0454132},
                                          {1.62007, -0.0288063, -0.0240376, 1.4508, 0.138295, 0.0903784, -0.0828124},
                                          {1.61744, -0.0298923, 0.0246762, 1.43904, 0.122086, 0.0904574, -0.0279466},
                                          {1.62255, 0.0148783, 0.104178, 1.40709, 0.11236, 0.0859504, -0.0512697}
                                        };
        n_samples = 5;
        obtainLeastDistJoints(n_samples, row1_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_itd1[][NO_JOINTS] = {{1.49629, 0.000977404, 0.0622149, 1.3776, 0.0958343, 0.19499, 0.260458},
                                         {1.62514, 0.143407, -0.148331, 1.25565, 0.0170794, 0.0880853, 0.0315427},
                                         {1.61992, 0.11156, -0.0415526, 1.28403, 0.100104, 0.0983645, -0.0184941},
                                         {1.61733, 0.121035, 0.0205254, 1.27491, 0.0818387, 0.102713, -0.0212682},
                                         {1.62255, 0.154837, 0.085249, 1.25241, 0.110067, 0.078913, -0.0584618}
                                        };
        n_samples = 5;
        obtainLeastDistJoints(n_samples, row1_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_inside[][NO_JOINTS] = {{1.49552, 0.245464, 0.0562397, 0.992145, 0.109593, 0.376853, 0.225422},
                                           {1.62503, 0.290316, -0.169449, 1.01848, 0.0165259, 0.161226, 0.0777779},
                                           {1.61882, 0.241174, -0.0476647, 1.04191, 0.0331308, 0.19665, 0.0687363},
                                           {1.61736, 0.25046, 0.0254972, 1.10966, 0.0592243, 0.117895, -0.00678116},
                                           {1.62255, 0.284153, 0.0848385, 1.09271, 0.0881644, 0.0957552, -0.064421}
                                          };
        n_samples = 5;
        obtainLeastDistJoints(n_samples, row1_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_itd3[][NO_JOINTS] = {{1.49505, 0.40079, 0.0686006, 0.711426, 0.0728246, 0.501074, 0.279672},
                                         {1.6163, 0.417949, -0.180989, 0.856302, 0.000395356, 0.179096, 0.119698},
                                         {1.61722, 0.406464, -0.0556012, 0.819657, 0.0139165, 0.198627, 0.0814767},
                                         {1.62394, 0.413279, 0.0291005, 0.859967, 0.0438054, 0.154189, -0.00339058},
                                         {1.62927, 0.446836, 0.0888524, 0.855706, 0.0489451, 0.129756, -0.0574344}
                                        };
        n_samples = 5;
        obtainLeastDistJoints(n_samples, row1_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_end[][NO_JOINTS] = {{1.49779, 0.601239, 0.174649, 0.524451, -0.14731, 0.380095, 0.372347}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }
    else if(bin_idx == 3 || bin_idx == 4 || bin_idx == 5)
    {
        double row2_binfront[][NO_JOINTS] = {{1.55305, -0.445479, 0.0447911, 2.02366, 0.0646011, 0.00956761, -0.106957},
                                             {1.55305, -0.355259, 0.0534118, 2.01778, 0.0746432, -0.0608848, -0.126171},
                                             {1.55305, -0.252442, 0.0667761, 1.9704, 0.112518, -0.0995506, -0.159871},
                                             {1.55305, -0.115795, 0.0764915, 1.90785, 0.126039, -0.149445, -0.176618},
                                             {1.55308, -0.471136, -0.0492155, 2.00875, 0.0664198, 0.0937784, -0.102745},
                                             {1.55305, -0.342146, -0.0225324, 2.00841, 0.113942, -0.0426194, -0.141993},
                                             {1.55305, -0.245953, -0.0244025, 1.99673, 0.133788, -0.140114, -0.148261},
                                             {1.55305, -0.108329, -0.0235358, 1.94006, 0.173403, -0.210883, -0.191105},
                                             {1.5666, -0.377685, 0.173919, 2.04002, 0.0919598, -0.0198469, 0.0717159},
                                             {1.56656, -0.288578, 0.171821, 2.0269, 0.107062, -0.0887179, 0.0265082},
                                             {1.56656, -0.192358, 0.170361, 1.95378, 0.107932, -0.0887969, 0.0170556},
                                             {1.56656, -0.0859029, 0.148331, 1.87708, 0.123351, -0.106272, 0.00256862}
                                            };
        n_samples = 12;
        obtainLeastDistJoints(n_samples, row2_binfront, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_front[][NO_JOINTS] = {{1.57799, -0.255835, -0.0899927, 2.21498, 0.0189771, -0.376063, 0.405945},
                                          {1.56671, 0.0222359, 0.0219394, 1.83387, 0.133472, -0.139798, -0.06925},
                                          {1.56674, -0.0242179, -0.0891261, 1.94884, 0.0634151, -0.265521, -0.0352415},
                                          {1.56587, 0.0228061, 0.1632, 1.86106, 0.00474427, -0.175538, 0.0537356},
                                          {1.56364, -5.43002e-05, -0.0383142, 1.8499, 0.123272, -0.148891, -0.0283576},
                                          {1.62817, 0.00605447, 0.0305145, 1.95199, 0.0568522, -0.301657, 0.049523}
                                         };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row2_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_itd1[][NO_JOINTS] = {{1.57803, -0.112917, -0.0719303, 2.12073, 0.00956761, -0.408244, 0.425158},
                                         {1.56671, 0.128773, 0.0243113, 1.72658, 0.142803, -0.181073, -0.0863057},
                                         {1.56671, 0.103496, -0.0857508, 1.8072, 0.0645221, -0.263307, -0.0361662},
                                         {1.56163, 0.179191, 0.144773, 1.66931, 0.03661, -0.183524, 0.0114047},
                                         {1.56367, 0.0948896, -0.0373563, 1.76041, 0.121849, -0.206139, -0.0373991},
                                         {1.62796, 0.102872, 0.0126346, 1.80473, 0.0989971, -0.280703, -0.00647293}
                                        };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row2_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_inside[][NO_JOINTS] = {{1.57803, 0.00469697, -0.0747582, 2.01105, 0.00759083, -0.40864, 0.430501},
                                           {1.56663, 0.265827, 0.0231253, 1.54958, 0.14557, -0.179571, -0.0970939},
                                           {1.56671, 0.233111, -0.0783616, 1.62321, 0.0773316, -0.247809, -0.0366799},
                                           {1.56185, 0.276062, 0.142401, 1.55222, 0.0398519, -0.179808, 0.010891},
                                           {1.56342, 0.216794, -0.0379949, 1.60318, 0.120267, -0.205032, -0.0580508},
                                           {1.62796, 0.228061, 0.00994344, 1.64468, 0.126039, -0.278172, -0.01572}
                                          };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row2_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_itd3[][NO_JOINTS] = {{1.57792, 0.197788, -0.0681901, 1.80013, -0.00205585, -0.387132, 0.438515},
                                         {1.56678, 0.389332, 0.0141398, 1.36874, 0.160673, -0.162096, -0.101923},
                                         {1.56671, 0.36039, -0.0800036, 1.44271, 0.077806, -0.206059, -0.0267137},
                                         {1.56744, 0.457181, 0.151387, 1.30738, 0.045545, -0.156877, -0.0229121},
                                         {1.56167, 0.376925, -0.0418263, 1.41603, 0.121216, -0.202027, -0.0658595},
                                         {1.62806, 0.358273, 0.023171, 1.48438, 0.127779, -0.276275, -0.0247615}
                                        };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row2_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_end[][NO_JOINTS] = {{1.57799, 0.386455, -0.0776318, 1.54992, 0.0250656, -0.371081, 0.433481}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }
    else if(bin_idx == 6 || bin_idx == 7 || bin_idx == 8)
    {
        double row3_binfront[][NO_JOINTS] = {{1.55305, -0.445479, 0.0447911, 2.02366, 0.0646011, 0.00956761, -0.106957},
                                             {1.55305, -0.355259, 0.0534118, 2.01778, 0.0746432, -0.0608848, -0.126171},
                                             {1.55305, -0.252442, 0.0667761, 1.9704, 0.112518, -0.0995506, -0.159871},
                                             {1.55305, -0.115795, 0.0764915, 1.90785, 0.126039, -0.149445, -0.176618},
                                             {1.55308, -0.471136, -0.0492155, 2.00875, 0.0664198, 0.0937784, -0.102745},
                                             {1.55305, -0.342146, -0.0225324, 2.00841, 0.113942, -0.0426194, -0.141993},
                                             {1.55305, -0.245953, -0.0244025, 1.99673, 0.133788, -0.140114, -0.148261},
                                             {1.55305, -0.108329, -0.0235358, 1.94006, 0.173403, -0.210883, -0.191105},
                                             {1.5666, -0.377685, 0.173919, 2.04002, 0.0919598, -0.0198469, 0.0717159},
                                             {1.56656, -0.288578, 0.171821, 2.0269, 0.107062, -0.0887179, 0.0265082},
                                             {1.56656, -0.192358, 0.170361, 1.95378, 0.107932, -0.0887969, 0.0170556},
                                             {1.56656, -0.0859029, 0.148331, 1.87708, 0.123351, -0.106272, 0.00256862}
                                            };
        n_samples = 12;
        obtainLeastDistJoints(n_samples, row3_binfront, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_front[][NO_JOINTS] = {{1.59731, -0.112754, -0.158958, 2.62328, -0.251367, -0.845034, 0.258198},
                                          {1.56671, 0.0222359, 0.0219394, 1.83387, 0.133472, -0.139798, -0.06925},
                                          {1.56674, -0.0242179, -0.0891261, 1.94884, 0.0634151, -0.265521, -0.0352415},
                                          {1.56587, 0.0228061, 0.1632, 1.86106, 0.00474427, -0.175538, 0.0537356},
                                          {1.56364, -5.43002e-05, -0.0383142, 1.8499, 0.123272, -0.148891, -0.0283576},
                                          {1.62817, 0.00605447, 0.0305145, 1.95199, 0.0568522, -0.301657, 0.049523}
                                         };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row3_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_itd1[][NO_JOINTS] = {{1.59735, 0.0344535, -0.13378, 2.50806, -0.211674, -0.879192, 0.262308},
                                         {1.56671, 0.128773, 0.0243113, 1.72658, 0.142803, -0.181073, -0.0863057},
                                         {1.56671, 0.103496, -0.0857508, 1.8072, 0.0645221, -0.263307, -0.0361662},
                                         {1.56163, 0.179191, 0.144773, 1.66931, 0.03661, -0.183524, 0.0114047},
                                         {1.56367, 0.0948896, -0.0373563, 1.76041, 0.121849, -0.206139, -0.0373991},
                                         {1.62796, 0.102872, 0.0126346, 1.80473, 0.0989971, -0.280703, -0.00647293}
                                        };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row3_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_inside[][NO_JOINTS] = {{1.59735, 0.183698, -0.123061, 2.39829, -0.17704, -0.948142, 0.168707},
                                           {1.56663, 0.265827, 0.0231253, 1.54958, 0.14557, -0.179571, -0.0970939},
                                           {1.56671, 0.233111, -0.0783616, 1.62321, 0.0773316, -0.247809, -0.0366799},
                                           {1.56185, 0.276062, 0.142401, 1.55222, 0.0398519, -0.179808, 0.010891},
                                           {1.56342, 0.216794, -0.0379949, 1.60318, 0.120267, -0.205032, -0.0580508},
                                           {1.62796, 0.228061, 0.00994344, 1.64468, 0.126039, -0.278172, -0.01572}
                                          };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row3_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_itd3[][NO_JOINTS] = {{1.59742, 0.347277, -0.0893541, 2.21498, -0.117579, -0.953994, -0.0101717},
                                         {1.56678, 0.389332, 0.0141398, 1.36874, 0.160673, -0.162096, -0.101923},
                                         {1.56671, 0.36039, -0.0800036, 1.44271, 0.077806, -0.206059, -0.0267137},
                                         {1.56744, 0.457181, 0.151387, 1.30738, 0.045545, -0.156877, -0.0229121},
                                         {1.56167, 0.376925, -0.0418263, 1.41603, 0.121216, -0.202027, -0.0658595},
                                         {1.62806, 0.358273, 0.023171, 1.48438, 0.127779, -0.276275, -0.0247615}
                                        };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row3_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_end[][NO_JOINTS] = {{1.59742, 0.51425, -0.101761, 1.93904, -0.141933, -0.94577, 0.160796}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }
    else if(bin_idx == 9 || bin_idx == 10 || bin_idx == 11)
    {
        double row4_binfront[][NO_JOINTS] = {{1.55305, -0.445479, 0.0447911, 2.02366, 0.0646011, 0.00956761, -0.106957},
                                             {1.55305, -0.355259, 0.0534118, 2.01778, 0.0746432, -0.0608848, -0.126171},
                                             {1.55305, -0.252442, 0.0667761, 1.9704, 0.112518, -0.0995506, -0.159871},
                                             {1.55305, -0.115795, 0.0764915, 1.90785, 0.126039, -0.149445, -0.176618},
                                             {1.55308, -0.471136, -0.0492155, 2.00875, 0.0664198, 0.0937784, -0.102745},
                                             {1.55305, -0.342146, -0.0225324, 2.00841, 0.113942, -0.0426194, -0.141993},
                                             {1.55305, -0.245953, -0.0244025, 1.99673, 0.133788, -0.140114, -0.148261},
                                             {1.55305, -0.108329, -0.0235358, 1.94006, 0.173403, -0.210883, -0.191105},
                                             {1.5666, -0.377685, 0.173919, 2.04002, 0.0919598, -0.0198469, 0.0717159},
                                             {1.56656, -0.288578, 0.171821, 2.0269, 0.107062, -0.0887179, 0.0265082},
                                             {1.56656, -0.192358, 0.170361, 1.95378, 0.107932, -0.0887969, 0.0170556},
                                             {1.56656, -0.0859029, 0.148331, 1.87708, 0.123351, -0.106272, 0.00256862}
                                            };
        n_samples = 12;
        obtainLeastDistJoints(n_samples, row4_binfront, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_front[][NO_JOINTS] = {{1.52196, 0.329114, -0.0356687, 2.61714, 0.214836, -1.49231, -0.36084},
                                          {1.56671, 0.0222359, 0.0219394, 1.83387, 0.133472, -0.139798, -0.06925},
                                          {1.56674, -0.0242179, -0.0891261, 1.94884, 0.0634151, -0.265521, -0.0352415},
                                          {1.56587, 0.0228061, 0.1632, 1.86106, 0.00474427, -0.175538, 0.0537356},
                                          {1.56364, -5.43002e-05, -0.0383142, 1.8499, 0.123272, -0.148891, -0.0283576},
                                          {1.62817, 0.00605447, 0.0305145, 1.95199, 0.0568522, -0.301657, 0.049523}
                                         };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row4_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_itd1[][NO_JOINTS] = {{1.52204, 0.445289, -0.0337986, 2.42616, 0.210725, -1.47729, -0.359915},
                                         {1.56671, 0.128773, 0.0243113, 1.72658, 0.142803, -0.181073, -0.0863057},
                                         {1.56671, 0.103496, -0.0857508, 1.8072, 0.0645221, -0.263307, -0.0361662},
                                         {1.56163, 0.179191, 0.144773, 1.66931, 0.03661, -0.183524, 0.0114047},
                                         {1.56367, 0.0948896, -0.0373563, 1.76041, 0.121849, -0.206139, -0.0373991},
                                         {1.62796, 0.102872, 0.0126346, 1.80473, 0.0989971, -0.280703, -0.00647293}
                                        };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row4_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_inside[][NO_JOINTS] = {{1.52229, 0.600995, -0.0311987, 2.1659, 0.225353, -1.42186, -0.366285},
                                           {1.56663, 0.265827, 0.0231253, 1.54958, 0.14557, -0.179571, -0.0970939},
                                           {1.56671, 0.233111, -0.0783616, 1.62321, 0.0773316, -0.247809, -0.0366799},
                                           {1.56185, 0.276062, 0.142401, 1.55222, 0.0398519, -0.179808, 0.010891},
                                           {1.56342, 0.216794, -0.0379949, 1.60318, 0.120267, -0.205032, -0.0580508},
                                           {1.62796, 0.228061, 0.00994344, 1.64468, 0.126039, -0.278172, -0.01572}
                                          };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row4_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_itd3[][NO_JOINTS] = {{1.52284, 0.803534, -0.0278234, 1.82569, 0.224325, -1.32958, -0.365463},
                                         {1.56678, 0.389332, 0.0141398, 1.36874, 0.160673, -0.162096, -0.101923},
                                         {1.56671, 0.36039, -0.0800036, 1.44271, 0.077806, -0.206059, -0.0267137},
                                         {1.56744, 0.457181, 0.151387, 1.30738, 0.045545, -0.156877, -0.0229121},
                                         {1.56167, 0.376925, -0.0418263, 1.41603, 0.121216, -0.202027, -0.0658595},
                                         {1.62806, 0.358273, 0.023171, 1.48438, 0.127779, -0.276275, -0.0247615}
                                        };
        n_samples = 6;
        obtainLeastDistJoints(n_samples, row4_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_end[][NO_JOINTS] = {{1.52288, 0.934018, -0.0382229, 1.53543, 0.190957, -1.23153, -0.342243}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }

    // Now with the joints giving least distance from desired point compute the orientation and make use of it to compute accurate IK
    ComputeFk(lst_jts, eetrans, eerot);

    IkReal eerot_t[9],eetrans_t[3] = {0,0,0};

    // Compute the eef position wrt robot eef tool from the desired eef position taking into consideration of pipe lengths
    geometry_msgs::Point pt_eef;
    tf::Vector3 x_vector(eerot[0], eerot[3], eerot[6]), z_vector(eerot[2], eerot[5], eerot[8]);
    x_vector.normalize();
    z_vector.normalize();

    pt_eef.x =  point.x - pipe_len_z*z_vector.x() - pipe_len_x*x_vector.x();
    pt_eef.y =  point.y - pipe_len_z*z_vector.y() - pipe_len_x*x_vector.y();
    pt_eef.z =  point.z - pipe_len_z*z_vector.z() - pipe_len_x*x_vector.z();

    eetrans[0] = pt_eef.x+0.22; eetrans[1] = pt_eef.y+0.14; eetrans[2] = pt_eef.z+0.406;

    // Compute the inverse kinematics
    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());

    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = 0;
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution");
        fprintf(stderr,"\nCopied default solution\n");
        for( std::size_t j = 0; j < GetNumJoints(); ++j)
            jt_angle[j] = lst_jts[j];// copy default joint angle values in case ik solution is not found

        return true;
//        return false;
    }
    else
    {
//#if(DEBUG)
//        printf("/*********************************************************/\n");
        printf("Success: Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
//#endif
        std::vector<IkReal> solvalues(GetNumJoints());

        double least_norm = 1000;
        int least_norm_idx;
        for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
            const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
#if(DEBUG)
            printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
#endif
            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#if(DEBUG)
            for( std::size_t j = 0; j < solvalues.size(); ++j)
                printf("%.3f, ", solvalues[j]);
            printf("\n");
#endif
            double sum = 0;

            for( std::size_t j = 0; j < solvalues.size(); ++j)
                sum += (solvalues[j] - lst_jts[j])*(solvalues[j] - lst_jts[j]);
            double norm_sq_avg = sum/solvalues.size();

            if(norm_sq_avg<least_norm)
            {
                least_norm = norm_sq_avg;
                least_norm_idx = i;
            }
        }
#if(DEBUG)
        std::cout << "Selected " << least_norm_idx << " ik solution" << std::endl;
#endif
//        copy the solution with minimum norm from home position
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(least_norm_idx);
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

        for( std::size_t j = 0; j < solvalues.size(); ++j)
        {
            jt_angle[j] = solvalues[j];
            jts[j] = solvalues[j];
        }
        ComputeFk(jts, eetrans_t, eerot_t);
        cout << "Desired EEF position: [" << point.x << " " << point.y << " " << point.z << "]" << endl;
        double eef[3];
        pt_eef.x =  point.x - pipe_len_z*z_vector.x() - pipe_len_x*x_vector.x();
        pt_eef.y =  point.y - pipe_len_z*z_vector.y() - pipe_len_x*x_vector.y();
        pt_eef.z =  point.z - pipe_len_z*z_vector.z() - pipe_len_x*x_vector.z();

        eetrans[0] = pt_eef.x+0.22; eetrans[1] = pt_eef.y+0.14; eetrans[2] = pt_eef.z+0.406;

        eef[0] = eetrans_t[0] - 0.22 + pipe_len_z*z_vector.x() + pipe_len_x*x_vector.x();
        eef[1] = eetrans_t[1] - 0.14 + pipe_len_z*z_vector.y() + pipe_len_x*x_vector.y();
        eef[2] = eetrans_t[2] - 0.406 + pipe_len_z*z_vector.z() + pipe_len_x*x_vector.z();
        cout << "Inverse kinematics EEF position: [" << eef[0] << " " << eef[1] << " " << eef[2] << "]" << endl;

#if(DEBUG)
        cout << "Sent:[";
        for( std::size_t j = 0; j < solvalues.size(); ++j)
        {
            cout << solvalues[j] << " ";
        }
        cout << "]\nGot IK Solution to reach centroid\n";
#endif
        printf("/*********************************************************/\n");


        return true;
    }
}

bool computeLeftIK(geometry_msgs::Point &point, int bin_idx, vector<double> &jt_angle)
{
    IkReal eerot[9],eetrans[3] = {0,0,0};
    IkReal *jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);
    double pipe_len_x = 0.11, pipe_len_z = 0.43;

    // compute forward kinematics for each sample joint angle and compare its eef_position with desired position
    // Choose the orientation matrix and joint angles which gives eef_position with least distance from desired position
    int n_samples = 1;
    double lst_dist = 10000.0;
    IkReal *lst_jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);

    if(bin_idx == 0 || bin_idx == 1 || bin_idx == 2)
    {
        double row1_front[][NO_JOINTS] = {{1.58168, -0.0876948, -0.115399, 1.69658, 0.0384286, -0.216655, 1.84746},
                                          {1.599,  0.104,  0.209,  1.335,  0.111,  0.121,  1.485},
                                          {1.631, -0.042,  0.060,  1.518,  0.246,  0.045,  1.380}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_itd1[][NO_JOINTS] = {{1.5815, 0.072165, -0.123518, 1.42788, -0.00616755, -0.0297308, 1.93222},
                                         {1.606,  0.191,  0.221,  1.295,  0.120,  0.036,  1.471},
                                         {1.620,  0.152,  0.058,  1.311,  0.609,  0.032,  0.963}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_inside[][NO_JOINTS] = {{1.58048, 0.25551, -0.136791, 1.12628, 0.0158142, 0.122086, 1.97065},
                                           {1.641,  0.473,  0.252,  0.823, -0.003,  0.225,  1.375},
                                           {1.608,  0.337,  0.118,  1.122,  0.688, -0.016,  0.811}
                                         };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_itd3[][NO_JOINTS] = {{1.57244, 0.415912, -0.133872, 0.842667, -0.0376379, 0.212543, 1.95667}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_end[][NO_JOINTS] = {{1.56119, 0.547889, -0.135742, 0.60703, -0.0210329, 0.274535, 1.97322}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }
    else if(bin_idx == 3 || bin_idx == 4 || bin_idx == 5)
    {
        double row2_front[][NO_JOINTS] = {{1.56444, -0.0866631, -0.0605729, 2.10786, 0.221795, -0.494906, 1.5422},
                                          {1.654,  0.127,  0.155,  1.678,  0.202, -0.023,  1.231},
                                          {1.666,  0.073, -0.116,  1.532,  0.615,  0.338,  0.960}
                                         };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_itd1[][NO_JOINTS] = {{1.5644, 0.107867, -0.0664568, 1.97568, 0.148258, -0.558796, 1.6057},
                                         {1.665,  0.276,  0.179,  1.635,  0.389, -0.231,  0.942},
                                         {1.689,  0.234, -0.019,  1.544,  0.254, -0.069,  1.281}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_inside[][NO_JOINTS] = {{1.56404, 0.242043, -0.0741197, 1.83302, 0.147231, -0.565359, 1.62142},
                                           {1.677,  0.474,  0.202,  1.373,  0.441, -0.259,  0.966},
                                           {1.687,  0.439, -0.001,  1.357,  0.438, -0.192,  1.122}
                                          };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_itd3[][NO_JOINTS] = {{1.56101, 0.384418, -0.0977467, 1.64281, 0.117342, -0.560615, 1.73248}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_end[][NO_JOINTS] = {{1.54468, 0.510096, -0.0817369, 1.47552, 0.0751967, -0.570815, 1.7592}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }
    else if(bin_idx == 6 || bin_idx == 7 || bin_idx == 8)
    {
        double row3_front[][NO_JOINTS] = {{1.58957, 0.0543274, -0.0117223, 2.47695, 0.150947, -0.886941, 1.26191}
                                         };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_itd1[][NO_JOINTS] = {{1.58953, 0.254206, -0.0102627, 2.25683, 0.153635, -0.885993, 1.24732}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_inside[][NO_JOINTS] = {{1.58095, 0.410591, -0.000136836, 2.06031, 0.182259, -0.859741, 1.24681}
                                          };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_itd3[][NO_JOINTS] = {{1.58146, 0.514467, 0.00396825, 1.87435, 0.205111, -0.836889, 1.19122}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_end[][NO_JOINTS] = {{1.58318, 0.590732, 0.0622149, 1.74294, 0.258484, -0.833173, 1.07132}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }
    else if(bin_idx == 9 || bin_idx == 10 || bin_idx == 11)
    {
        double row4_front[][NO_JOINTS] = {{0.51129, 0.834051, 1.04288, 2.66981, -0.395277, -1.25636, 0.874667}
                                         };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_itd1[][NO_JOINTS] = {{0.680138, 0.872469, 0.979246, 2.52059, -0.276196, -1.2755, 0.887202}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_inside[][NO_JOINTS] = {{0.835545, 0.900976, 0.931171, 2.32236, -0.148496, -1.27115, 0.880934}
                                          };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_itd3[][NO_JOINTS] = {{1.02857, 0.906651, 0.75707, 2.14365, -0.106983, -1.34808, 0.97022}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_end[][NO_JOINTS] = {{1.24256, 0.94572, 0.471219, 1.94372, -0.135528, -1.36113, 1.05129}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }

    // Now with the joints giving least distance from desired point compute the orientation and make use of it to compute accurate IK
    ComputeFk(lst_jts, eetrans, eerot);

    IkReal eerot_t[9],eetrans_t[3] = {0,0,0};

    // Compute the eef position wrt robot eef tool from the desired eef position taking into consideration of pipe lengths
    geometry_msgs::Point pt_eef;
    tf::Vector3 x_vector(eerot[0], eerot[3], eerot[6]), z_vector(eerot[2], eerot[5], eerot[8]);
    x_vector.normalize();
    z_vector.normalize();

    pt_eef.x =  point.x - pipe_len_z*z_vector.x() - pipe_len_x*x_vector.x();
    pt_eef.y =  point.y - pipe_len_z*z_vector.y() - pipe_len_x*x_vector.y();
    pt_eef.z =  point.z - pipe_len_z*z_vector.z() - pipe_len_x*x_vector.z();

    eetrans[0] = pt_eef.x+0.22; eetrans[1] = pt_eef.y+0.14; eetrans[2] = pt_eef.z+0.406;

    // Compute the inverse kinematics
    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());

    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = 0;
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution");
//        fprintf(stderr,"\nCopied default solution\n");
//        for( std::size_t j = 0; j < GetNumJoints(); ++j)
//            jt_angle[j] = lst_jts[j];// copy default joint angle values in case ik solution is not found
//        return true;
        return false;
    }
    else
    {
//#if(DEBUG)
//        printf("/*********************************************************/\n");
        printf("Success: Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
//#endif
        std::vector<IkReal> solvalues(GetNumJoints());

        double least_norm = 1000;
        int least_norm_idx;
        for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
            const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
#if(DEBUG)
            printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
#endif
            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#if(DEBUG)
            for( std::size_t j = 0; j < solvalues.size(); ++j)
                printf("%.3f, ", solvalues[j]);
            printf("\n");
#endif
            double sum = 0;

            for( std::size_t j = 0; j < solvalues.size(); ++j)
                sum += (solvalues[j] - lst_jts[j])*(solvalues[j] - lst_jts[j]);
            double norm_sq_avg = sum/solvalues.size();

            if(norm_sq_avg<least_norm)
            {
                least_norm = norm_sq_avg;
                least_norm_idx = i;
            }
        }
#if(DEBUG)
        std::cout << "Selected " << least_norm_idx << " ik solution" << std::endl;
#endif
//        copy the solution with minimum norm from home position
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(least_norm_idx);
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

        for( std::size_t j = 0; j < solvalues.size(); ++j)
        {
            jt_angle[j] = solvalues[j];
            jts[j] = solvalues[j];
        }
        ComputeFk(jts, eetrans_t, eerot_t);
        cout << "Desired EEF position: [" << point.x << " " << point.y << " " << point.z << "]" << endl;
        double eef[3];
        pt_eef.x =  point.x - pipe_len_z*z_vector.x() - pipe_len_x*x_vector.x();
        pt_eef.y =  point.y - pipe_len_z*z_vector.y() - pipe_len_x*x_vector.y();
        pt_eef.z =  point.z - pipe_len_z*z_vector.z() - pipe_len_x*x_vector.z();

        eetrans[0] = pt_eef.x+0.22; eetrans[1] = pt_eef.y+0.14; eetrans[2] = pt_eef.z+0.406;

        eef[0] = eetrans_t[0] - 0.22 + pipe_len_z*z_vector.x() + pipe_len_x*x_vector.x();
        eef[1] = eetrans_t[1] - 0.14 + pipe_len_z*z_vector.y() + pipe_len_x*x_vector.y();
        eef[2] = eetrans_t[2] - 0.406 + pipe_len_z*z_vector.z() + pipe_len_x*x_vector.z();
        cout << "Inverse kinematics EEF position: [" << eef[0] << " " << eef[1] << " " << eef[2] << "]" << endl;

#if(DEBUG)
        cout << "Sent:[";
        for( std::size_t j = 0; j < solvalues.size(); ++j)
        {
            cout << solvalues[j] << " ";
        }
        cout << "]\nGot IK Solution to reach centroid\n";
#endif
        printf("/*********************************************************/\n");

        return true;
    }
}

bool computeRightIK(geometry_msgs::Point &point, int bin_idx, vector<double> &jt_angle)
{
    IkReal eerot[9],eetrans[3] = {0,0,0};
    IkReal *jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);
    double pipe_len_x = 0.11, pipe_len_z = 0.43;

    // compute forward kinematics for each sample joint angle and compare its eef_position with desired position
    // Choose the orientation matrix and joint angles which gives eef_position with least distance from desired position
    int n_samples = 1;
    double lst_dist = 10000.0;
    IkReal *lst_jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);

    if(bin_idx == 0 || bin_idx == 1 || bin_idx == 2)
    {
        double row1_front[][NO_JOINTS] = { {1.47584, -0.0623638, 0.0520434, 1.62764, -0.300708, -0.0715594, -0.904052},
                                           {1.143,  0.089,  0.195,  1.468, -0.350, -0.091, -1.382},
                                           {1.578,  0.016, -0.038,  1.456, -1.041,  0.115, -0.640}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_itd1[][NO_JOINTS] = {{1.47511, 0.108981, 0.0667761, 1.4318, -0.356295, -0.0600941, -0.855145},
                                         {1.146,  0.247,  0.246,  1.253, -0.346, -0.018, -1.416},
                                         {1.575,  0.214, -0.067,  1.259, -0.653,  0.056, -0.993}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_inside[][NO_JOINTS] = { {1.47536, 0.225753, 0.0899015, 1.20435, -0.356848, 0.0552708, -0.827302},
                                            {1.155,  0.404,  0.322,  0.897, -0.261,  0.210, -1.458},
                                            {1.572,  0.418, -0.093,  0.992, -0.608,  0.064, -0.995}
                                         };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_itd3[][NO_JOINTS] = { {1.47861, 0.417216, 0.151751, 0.903344, -0.469841, 0.158617, -0.766374}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row1_end[][NO_JOINTS] = {{1.47317, 0.550333, 0.19823, 0.66106, -0.460115, 0.244093, -0.787745}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row1_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }
    else if(bin_idx == 3 || bin_idx == 4 || bin_idx == 5)
    {
        double row2_front[][NO_JOINTS] = { {1.5724, -0.0542731, -0.131408, 2.0771, -0.223218, -0.359537, -1.07163},
                                           {1.586,  0.071, -0.287,  1.937, -0.214, -0.397, -1.139},
                                           {1.581, -0.053, -0.089,  1.855,  0.059, -0.118, -1.500}
                                         };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_itd1[][NO_JOINTS] = {{1.5724, 0.117397, -0.137384, 1.90555, -0.219027, -0.377486, -1.07081},
                                         {1.586,  0.272, -0.270,  1.704, -0.127, -0.343, -1.147},
                                         {1.581,  0.159, -0.081,  1.852,  0.120, -0.464, -1.569}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_inside[][NO_JOINTS] = {{1.57248, 0.27837, -0.160965, 1.72462, -0.210013, -0.365941, -1.04748},
                                           {1.584,  0.492, -0.303,  1.289, -0.093, -0.147, -1.306},
                                           {1.581,  0.384, -0.092,  1.550,  0.205, -0.351, -1.708}
                                          };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_itd3[][NO_JOINTS] = {{1.56992, 0.419279, -0.163428, 1.50603, -0.192696, -0.324271, -1.04615}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row2_end[][NO_JOINTS] = {{1.56415, 0.543056, -0.15125, 1.23102, -0.22053, -0.136556, -0.983371}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row2_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }
    else if(bin_idx == 6 || bin_idx == 7 || bin_idx == 8)
    {
        double row3_front[][NO_JOINTS] = {{1.59753, 0.132248, -0.0261357, 2.53686, 0.001107, -1.01496, -1.58504}
                                         };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_itd1[][NO_JOINTS] = {{1.59746, 0.234142, -0.0174238, 2.36659, 0.0226934, -0.94751, -1.59347}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_inside[][NO_JOINTS] = {{1.59731, 0.361884, -0.0344371, 2.14629, 0.0337634, -0.820996, -1.63847}
                                          };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_itd3[][NO_JOINTS] = {{1.59724, 0.473471, -0.0375388, 1.90256, 0.0287819, -0.711324, -1.66087}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row3_end[][NO_JOINTS] = {{1.58362, 0.57675, -0.0644499, 1.65346, 0.0369262, -0.529382, -1.64155}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row3_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }
    else if(bin_idx == 9 || bin_idx == 10 || bin_idx == 11)
    {
        double row4_front[][NO_JOINTS] = {{-0.724952, -0.937683, 1.69353, 2.81238, -0.310829, -1.50844, -0.672054}
                                         };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_front, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_itd1[][NO_JOINTS] = {{-0.807312, -1.0345, 1.77627, 2.5632, -0.267498, -1.41545, -0.590372}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_itd1, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_inside[][NO_JOINTS] = {{-0.932733, -1.11362, 1.83438, 2.24805, -0.23152, -1.31353, -0.558829}
                                          };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_inside, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_itd3[][NO_JOINTS] = {{-0.954757, -1.16205, 1.8508, 2.03108, -0.210725, -1.11609, -0.575885}
                                        };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_itd3, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);

        double row4_end[][NO_JOINTS] = {{-1.05169, -1.20321, 1.84487, 1.86515, -0.274219, -1.13578, -0.531807}
                                       };
        n_samples = 1;
        obtainLeastDistJoints(n_samples, row4_end, eerot, eetrans, lst_dist, point, lst_jts, pipe_len_x, pipe_len_z);
    }

    // Now with the joints giving least distance from desired point compute the orientation and make use of it to compute accurate IK
    ComputeFk(lst_jts, eetrans, eerot);

    IkReal eerot_t[9],eetrans_t[3] = {0,0,0};

    // Compute the eef position wrt robot eef tool from the desired eef position taking into consideration of pipe lengths
    geometry_msgs::Point pt_eef;
    tf::Vector3 x_vector(eerot[0], eerot[3], eerot[6]), z_vector(eerot[2], eerot[5], eerot[8]);
    x_vector.normalize();
    z_vector.normalize();

    pt_eef.x =  point.x - pipe_len_z*z_vector.x() - pipe_len_x*x_vector.x();
    pt_eef.y =  point.y - pipe_len_z*z_vector.y() - pipe_len_x*x_vector.y();
    pt_eef.z =  point.z - pipe_len_z*z_vector.z() - pipe_len_x*x_vector.z();

    eetrans[0] = pt_eef.x+0.22; eetrans[1] = pt_eef.y+0.14; eetrans[2] = pt_eef.z+0.406;

    // Compute the inverse kinematics
    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());

    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = 0;
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution");
//        fprintf(stderr,"\nCopied default solution\n");
//        for( std::size_t j = 0; j < GetNumJoints(); ++j)
//            jt_angle[j] = lst_jts[j];// copy default joint angle values in case ik solution is not found
//        return true;
        return false;
    }
    else
    {
//#if(DEBUG)
//        printf("/*********************************************************/\n");
        printf("Success: Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
//#endif
        std::vector<IkReal> solvalues(GetNumJoints());

        double least_norm = 1000;
        int least_norm_idx;
        for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
            const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
#if(DEBUG)
            printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
#endif
            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#if(DEBUG)
            for( std::size_t j = 0; j < solvalues.size(); ++j)
                printf("%.3f, ", solvalues[j]);
            printf("\n");
#endif
            double sum = 0;

            for( std::size_t j = 0; j < solvalues.size(); ++j)
                sum += (solvalues[j] - lst_jts[j])*(solvalues[j] - lst_jts[j]);
            double norm_sq_avg = sum/solvalues.size();

            if(norm_sq_avg<least_norm)
            {
                least_norm = norm_sq_avg;
                least_norm_idx = i;
            }
        }
#if(DEBUG)
        std::cout << "Selected " << least_norm_idx << " ik solution" << std::endl;
#endif
//        copy the solution with minimum norm from home position
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(least_norm_idx);
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

        for( std::size_t j = 0; j < solvalues.size(); ++j)
        {
            jt_angle[j] = solvalues[j];
            jts[j] = solvalues[j];
        }
        ComputeFk(jts, eetrans_t, eerot_t);
        cout << "Desired EEF position: [" << point.x << " " << point.y << " " << point.z << "]" << endl;
        double eef[3];
        pt_eef.x =  point.x - pipe_len_z*z_vector.x() - pipe_len_x*x_vector.x();
        pt_eef.y =  point.y - pipe_len_z*z_vector.y() - pipe_len_x*x_vector.y();
        pt_eef.z =  point.z - pipe_len_z*z_vector.z() - pipe_len_x*x_vector.z();

        eetrans[0] = pt_eef.x+0.22; eetrans[1] = pt_eef.y+0.14; eetrans[2] = pt_eef.z+0.406;

        eef[0] = eetrans_t[0] - 0.22 + pipe_len_z*z_vector.x() + pipe_len_x*x_vector.x();
        eef[1] = eetrans_t[1] - 0.14 + pipe_len_z*z_vector.y() + pipe_len_x*x_vector.y();
        eef[2] = eetrans_t[2] - 0.406 + pipe_len_z*z_vector.z() + pipe_len_x*x_vector.z();
        cout << "Inverse kinematics EEF position: [" << eef[0] << " " << eef[1] << " " << eef[2] << "]" << endl;

#if(DEBUG)
        cout << "Sent:[";
        for( std::size_t j = 0; j < solvalues.size(); ++j)
        {
            cout << solvalues[j] << " ";
        }
        cout << "]\nGot IK Solution to reach centroid\n";
#endif
        printf("/*********************************************************/\n");

        return true;
    }
}

#endif
