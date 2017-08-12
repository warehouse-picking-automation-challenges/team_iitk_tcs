#include<ros/ros.h>
#include<ros/package.h>
#include<boost/filesystem.hpp>
#include<boost/algorithm/string.hpp>
#include<yaml-cpp/yaml.h>
#include<fstream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;

vector<std::string> model_names=
{
    "Toilet_Brush","Avery_Binder","Balloons","Band_Aid_Tape","Bath_Sponge","Black_Fashion_Gloves","Burts_Bees_Baby_Wipes"
    ,"Colgate_Toothbrush_4PK","Composition_Book","Crayons","Duct_Tape","Epsom_Salts","Expo_Eraser","Fiskars_Scissors"
    ,"Flashlight","Glue_Sticks","Hand_Weight","Hanes_Socks","Hinged_Ruled_Index_Cards","Ice_Cube_Tray","Irish_Spring_Soap"
    ,"Laugh_Out_Loud_Jokes","Marbles","Measuring_Spoons","Mesh_Cup","Mouse_Traps","Pie_Plates","Plastic_Wine_Glass"
    ,"Poland_Spring_Water","Reynolds_Wrap","Robots_DVD","Robots_Everywhere","Scotch_Sponges","Speed_Stick","White_Facecloth"
    ,"Table_Cloth","Tennis_Ball_Container","Ticonderoga_Pencils","Tissue_Box","Windex"
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"iitktcs_startup_kit_node");
    ros::NodeHandle nh;
    if (argc != 2)
    {
        cout << "Wrong usages : " << endl;
        cout << "===============" << endl;
        cout << "Correct way is : rosrun iitktcs_startup_kit iitktcs_startup_kit_node <folder_location>" << endl;
        return 1;
    }
    vector<string> current_object_names,current_object_path_names,current_object_path_names_in_lower,actual_folder_name;
    vector<int> current_object_ids;
    int known_items_size = model_names.size();
    actual_folder_name = model_names;
    for (int i=0; i<model_names.size(); i++)
    {
        boost::algorithm::to_lower(model_names[i]);
//        cout << model_names[i] << " ";
    }

    /*
     * Iterates through the folders within the folder (path provided in argument.)
     * and loads the object list and competition list set on ROS parameter server.
     */
    boost::filesystem::path file_path (argv[1]);
    try
    {
        if(boost::filesystem::exists(file_path))
        {
            if(boost::filesystem::is_directory(file_path))
            {
                cout << "Address (" << file_path << ") is a directory." << endl;
                boost::filesystem::directory_iterator end_itr;
                for (boost::filesystem::directory_iterator itr(file_path); itr!=end_itr; ++itr)
                {
                    if(boost::filesystem::is_directory(*itr))
                    {
                        string current_file_path = itr->path().string();
                        current_object_path_names.push_back(current_file_path);
                        string current_file = itr->path().filename().string();
                        boost::algorithm::to_lower(current_file);
                        current_object_names.push_back(current_file);
                    }
                }
                current_object_path_names_in_lower = current_object_path_names;
                sort(current_object_names.begin(),current_object_names.end());
                // Creating New Model Names.
                for (int i=0; i<current_object_names.size();i++)
                {
                    bool present = false;
//                    cout << current_object_names[i] << endl;
                    for (int j=0;j<model_names.size();j++)
                    {
                        if(current_object_names[i].compare(model_names[j].c_str()) == 0)
                        {
                            present = true;
                            break;
                        }
                    }
                    if(present == false)
                    {
                        model_names.push_back(current_object_names[i]);
                    }
                }
                // Creating Available IDs.
                for (int i=0; i<current_object_names.size();i++)
                {
                    for (int j=0; j<model_names.size();j++)
                    {
                        if(current_object_names[i].compare(model_names[j].c_str()) == 0)
                        {
                            current_object_ids.push_back(j+1);
                            break;
                        }
                    }
                }
                sort(current_object_ids.begin(),current_object_ids.end());
                YAML::Node object_names,competition_object_ids;
                for (int i=0; i<model_names.size(); i++)
                    object_names["/ARC17_OBJECT_NAMES"].push_back(model_names[i]);
                for (int i=0; i<current_object_ids.size(); i++)
                    competition_object_ids["/ARC17_COMPETITION_SET"].push_back(current_object_ids[i]);
                string path = ros::package::getPath("iitktcs_startup_kit");
                path.append("/parameters/objects_list.yaml");
                cout << "Path is : " << path << endl;
                ofstream object_yaml(path.c_str());
                object_yaml << object_names << endl << competition_object_ids;
                object_yaml.close();
                nh.setParam("/ARC17_OBJECT_NAMES",model_names);
                nh.setParam("/ARC17_COMPETITION_SET",current_object_ids);
//                for (int i=0; i<model_names.size(); i++)
//                    cout << "   " << model_names[i] << endl;
            }
            else
                cout << "Address (" << file_path << ") is not a directory." << endl;
        }
        else
            cout << "Address (" << file_path <<") provided does not exist." << endl;
    }
    catch (const boost::filesystem::filesystem_error& ex)
    {
        cout << ex.what() << endl;
    }

    /*
     * Code block to display the images in folder.
     */

    vector<string> new_objects_path;
    for(int i=0; i<current_object_path_names_in_lower.size(); i++)
        boost::algorithm::to_lower(current_object_path_names_in_lower[i]);
    // Previous.
//    for(int i=0; i<current_object_ids.size();i++)
//        cout << " " << current_object_ids[i];
//    for(int i=(current_object_ids.size()/2); i<current_object_ids.size();i++)
//    {
//        for(int j=0;j<current_object_path_names_in_lower.size();j++)
//        {
////            cout  << i << " " << model_names[current_object_ids[i]-1] << "     " << j << "    " << current_object_path_names_in_lower[j] << endl;
//            if(current_object_path_names_in_lower[j].find(model_names[current_object_ids[i]-1]) != std::string::npos)
//            {
//                new_objects_path.push_back(current_object_path_names[j]);
//                break;
//            }
//        }
//    }

    // New
    for(int i=0; i<current_object_ids.size();i++)
        cout << " " << current_object_ids[i];
    cout << endl << known_items_size << "\t" << model_names.size() << endl;
    for(int i=known_items_size; i<model_names.size();i++)
    {
        cout << i << endl;
        for(int j=0;j<current_object_path_names_in_lower.size();j++)
        {
            cout  << i << " " << model_names[i] << "     " << j << "    " << current_object_path_names_in_lower[j] << endl;
            if(current_object_path_names_in_lower[j].find(model_names[i]) != std::string::npos)
            {
                new_objects_path.push_back(current_object_path_names[j]);
                break;
            }
        }
    }

    cout << "Size is : " << new_objects_path.size() << endl;
    for(int i=0;i<new_objects_path.size();i++)
        cout<<new_objects_path[i] << endl;
    for(int i=0;i<new_objects_path.size();i++)
    {
        cout << "Path " << i+1 << ") is " << new_objects_path[i] << endl;
        if(boost::filesystem::is_directory(new_objects_path[i]))
        {
            boost::filesystem::directory_iterator end_itr;
            vector<string> path;
            for (boost::filesystem::directory_iterator itr(new_objects_path[i]); itr!=end_itr; ++itr)
            {
                if(boost::filesystem::is_regular_file(*itr) && itr->path().extension() == ".png")
                {
//                    cout << *itr << endl;
                    path.push_back(itr->path().string());
                }
            }
            cout << endl << endl << endl;
            cout << "Displaying images for " <<"\033[1;34m"<< model_names[40+i] <<"\033[0m"<< "." << endl;
            for (int j=0; j<path.size();j++)
            {
                cv::Mat image;
                image = cv::imread(path[j], CV_LOAD_IMAGE_COLOR);
                if(!image.data)
                    cout << "Image for " << path[j] << " not found." << endl;
                else
                {
                    cv::namedWindow(path[j], cv::WINDOW_NORMAL );
                    cv::imshow(path[j], image);
                    cv::waitKey(0);
                    cv::destroyWindow(path[j]);
                }
            }
            string input;
            cout << endl;
            cout << "All the images for " <<"\033[1;34m"<< model_names[40+i] <<"\033[0m"<< " have been displayed." << endl << endl;
            cout << "Move to "<< "\033[1;32m" << "'next object'" << "\033[0m" << " or repeat the images for " <<"\033[1;31m"<< "'same object'" <<"\033[0m"<<"?" << endl;
            cout << "Press "<< "\033[1;32m"<< "\t   'Y'\t" <<"\033[0m"<<"      to move to next object or " << "\033[1;31m"<< "  'any key'  " <<"\033[0m"<<" to repeat. : " ;
            cin >> input;
            if(input == "Yes" || input == "Y" || input == "y" || input == "yes" || input == "YES")
                cout << "Next item." << endl;
            else
            {
                cout << endl << "Showing images for " <<"\033[1;34m"<< model_names[40+i] <<"\033[0m"<< " again." << endl;
                i-=1;
            }
        }
        else
            cout << new_objects_path[i] << " is not a directory." << endl;
    }

    return 0;
}
