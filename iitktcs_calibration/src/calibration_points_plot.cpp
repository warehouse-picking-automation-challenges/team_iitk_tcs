#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

using namespace std;

void addMarker(double *centroid, double *color, visualization_msgs::Marker &marker, std::string &frame_id, int id)
{
//    static int id = 0;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

//    id++;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"calibration_plot");
    ros::NodeHandle nh;
    ros::Publisher pub_marker = nh.advertise<visualization_msgs::MarkerArray>("/plot", 10);

    int n_pts = 12;
// ee link
    double el_pts[][3] = {{0.86811, -0.10467, -0.34042},
                          {0.872902, 0.0172405, -0.342456},
                          {0.872945, 0.132791, -0.332209},
                          {0.876369, 0.267534, -0.328459},
                          {0.83799, 0.25851, -0.215936},
                          {0.834617, 0.128392, -0.225121},
                          {0.836081, 0.0136406, -0.233054},
                          {0.831953, -0.110265, -0.233079},
                          {0.800218, -0.114954, -0.149218},
                          {0.805548, 0.00461507, -0.148496},
                          {0.804717, 0.117155, -0.137269},
                          {0.807238, 0.25102, -0.130119}
                         };

    double cl_pts[][3] = {{-0.0459189, -0.00351812, 0.859789},
                          {0.073095, -0.000331081, 0.862786},
                          {0.185133, 0.0100267, 0.859363},
                          {0.321841, 0.0100666, 0.862786},
                          {0.31823, 0.107406, 0.80133},
                          {0.187883, 0.101276, 0.79985},
                          {0.0736949, 0.0946028, 0.79985},
                          {-0.0520686, 0.091132, 0.795444},
                          {-0.0604677, 0.167428, 0.744511},
                          {0.0654697, 0.175421, 0.740375},
                          {0.176019, 0.174767, 0.749341},
                          {0.311278, 0.175831, 0.753906}
                         };

    std::cout << "Publishing Markers" << std::endl;
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker sphere_marker;

    double color_r[3] = {1.0,0.0,0.0};
    double color_g[3] = {1.0,1.0,0.0};

    cout << "ref color: red camera link" << endl;// camera_link
    cout << "ref color: yellow ee link" << endl;
    for(int j=0; j<n_pts; j++)
    {
        string frame_cl = "/camera_link";
        string frame_ee = "/ee_link";

        addMarker(cl_pts[j],color_r,sphere_marker,frame_cl, 2*j);
        markerArray.markers.push_back(sphere_marker);
        addMarker(el_pts[j],color_g,sphere_marker,frame_ee, 2*j+1);
        markerArray.markers.push_back(sphere_marker);
    }

    while(pub_marker.getNumSubscribers() < 1)
    {
        ros::Duration(3).sleep();
        cout << "Put subscriber for topic: /plot"  << endl;
    }

    ros::Rate r(1);
    while(ros::ok())
    {
        pub_marker.publish(markerArray);
        r.sleep();
    }

    return 0;
}

