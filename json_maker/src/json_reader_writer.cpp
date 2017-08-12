#include <json_maker/json_helper.h>
#include <json_maker/write_pick_status.h>
#include <json_maker/write_stow_data.h>
#include <json_maker/stowToteContents.h>
#include <json_maker/getBinContents.h>
#if(ARC_2017)
#include <json_maker/get_target_bin_objects.h>
#include <json_maker/write_object_pick_status.h>
#else
#include <json_maker/get_bin_object.h>
#endif

#include <typeinfo>

class JSON_MAKER{


private:
protected:
public:

    vector< vector<string> > bin_contents;
#if(ARC_2017)
    vector< string > rp_items;
    vector< vector< string > > rp_items_all;
#if(PICKING_TASK)
    string task = "PICK";
#elif(STOWING_TASK)
    string task = "STOW";
#endif
#else
    vector<string> rp_items;
#endif
    vector<string> tote_contents;
    int iteration ;

#if(PICKING_TASK)
    vector<bool> rp_picked;
    ros::ServiceServer pick_obj_service;
    ros::ServiceServer pick_status_write_service;
#if(ARC_2017)
    bool pick_object_callback(json_maker::get_target_bin_objectsRequest &req, json_maker::get_target_bin_objectsResponse &res);
#else
    bool pick_object_callback(json_maker::get_bin_objectRequest &req,json_maker::get_bin_objectResponse &res);
#endif
#if(ARC_2017)
    bool pick_status_write_callback(json_maker::write_object_pick_statusRequest &req,json_maker::write_object_pick_statusResponse &res);
#else
    bool pick_status_write_callback(json_maker::write_pick_statusRequest &req,json_maker::write_pick_statusResponse &res);
#endif
#endif

#if(STOWING_TASK)
    vector<bool> st_picked;
    vector<int> st_bin;
    ros::ServiceServer write_stow_data_service;
    ros::ServiceServer tote_contents_service;
    ros::ServiceServer bin_contents_service;
    bool write_stow_data_service_callback(json_maker::write_stow_dataRequest &req,json_maker::write_stow_dataResponse &res);
    bool tote_contents_callback(json_maker::stowToteContents::Request &req, json_maker::stowToteContents::Response &res);
    bool get_bin_contents_callback(json_maker::getBinContentsRequest &req,json_maker::getBinContentsResponse &res);
#endif

    JSON_MAKER();
    ~JSON_MAKER();

};

JSON_MAKER::JSON_MAKER()
{

    iteration = 0;
    ros::NodeHandle nh;
    bin_contents = vector< vector<string> >(NO_BINS);

#if(PICKING_TASK)
    rp_items = vector<string>(NO_BINS);
    rp_items_all = vector<vector<string> >(NO_OF_BOXES);
    rp_picked = vector<bool>(NO_BINS, false);
    vector<int> available_object_ids;

    string package_path = ros::package::getPath("json_maker");
#if(ARC_2017)
    vector<string> path;
    path.push_back(package_path.append("/data/examples/arc_2017/item_location_file.json"));
    package_path = ros::package::getPath("json_maker");
    path.push_back(package_path.append("/data/examples/arc_2017/order_file.json"));
    package_path = ros::package::getPath("json_maker");
    path.push_back(package_path.append("/data/examples/arc_2017/box_sizes.json"));
#else
    string path = package_path.append("/data/examples/rp_input.json");
#endif
    pick_obj_service = nh.advertiseService("/iitktcs/pick_object_service",&JSON_MAKER::pick_object_callback,this);
    pick_status_write_service = nh.advertiseService("/iitktcs/pick_object_status_service",&JSON_MAKER::pick_status_write_callback,this);
#endif

#if(STOWING_TASK)
    st_picked =  vector<bool> (int(NO_BINS), false);
    st_bin = vector<int>(int(NO_BINS), -1);
    string package_path = ros::package::getPath("json_maker");
    string path = package_path.append("/data/examples/st_input.json");
//    string path = "/home/ilab/tcs/apc_ws/src/apc_project/json_maker/data/examples/st_input.json";
    write_stow_data_service = nh.advertiseService("/write_stow_data_service",&JSON_MAKER::write_stow_data_service_callback,this);
    tote_contents_service = nh.advertiseService("/tote_contents/data",&JSON_MAKER::tote_contents_callback,this);
    bin_contents_service = nh.advertiseService("/get_bin_contents",&JSON_MAKER::get_bin_contents_callback,this);
#endif
    nh.getParam("/ARC17_OBJECT_NAMES",model_names);
    nh.getParam("/ARC17_COMPETITION_SET",set_id);
    for (int i=0;i<set_id.size();i++)
    {
        cout << "Set provided "  << i+1 << " is : "<< model_names[set_id[i]-1] << endl;
        set_provided.push_back(model_names[set_id[i]-1]);
    }
    for (int i=0;i<model_names.size();i++)
        cout << "Model " << i+1 << ") is " << model_names[i] << endl;

    cout << "Parsing file: "<< endl << "1) " << path[0] << " , " << endl << "2) " << path[1] << " , " << endl << "3) " << path[2] << endl;
    readJsonFile(path, bin_contents, rp_items, tote_contents, rp_items_all);
//    vector<string> current_objects;
//    nh.getParam("Current_Objects",current_objects);
//    for (int i=0; i<current_objects.size();i++)
//    {
//        bool present = false;
//        cout << current_objects[i] << endl;
//        for (int j=0;j<model_names.size();j++)
//        {
//            if(current_objects[i].compare(model_names[j].c_str()) == 0)
//            {
//                present = true;
//                break;
//            }
//        }
//        if(present == false)
//        {
//            model_names.push_back(current_objects[i]);
//        }
//    }
//    for(int i=0;i<model_names.size();i++)
//        cout << " " << model_names[i];
//    for(int i=0;i<bin_contents.size();i++)
//    {
//        for(int j=0;j<bin_contents[i].size();j++)
//        {
//            for(int k=0;k<model_names.size();k++)
//            {
//                if(bin_contents[i][j].compare((model_names[k].c_str())) == 0)
//                {
//                    available_object_ids.push_back(k+1);
//                    break;
//                }
//            }
//        }
//    }
//    sort(available_object_ids.begin(),available_object_ids.end(),less<int>());
//    nh.setParam("/ARC17_OBJECT_NAMES",model_names);
//    nh.setParam("/ARC17_COMPETITION_SET",available_object_ids);
//    cout << "Parsed: " << endl << "1) " << path [0]  << " , " << endl << "2) " << path[1] << " , " << endl << "3) " << path[2] << endl;
}
#if(PICKING_TASK)
#if(ARC_2017)
bool JSON_MAKER::pick_object_callback(json_maker::get_target_bin_objectsRequest &req,json_maker::get_target_bin_objectsResponse &res)
{
    cout << "Called object pick service callback." << endl;
    vector<iitktcs_msgs_srvs::custom_ids> available, target;
    iitktcs_msgs_srvs::custom_ids available_row,target_row;
    for(int i=0; i < rp_items_all.size();i++)
        for(int j=0; j < rp_items_all[i].size();j++)
            boost::algorithm::to_lower(rp_items_all[i][j]);
    for(int i=0; i < rp_items_all.size();i++)
    {
        for(int j=0; j< rp_items_all[i].size();j++)
        {
            for(int k=0;k<model_names.size();k++)
            {
                cout << rp_items_all[i][j] << ":" << model_names[k].c_str() << endl;
                if(rp_items_all[i][j].compare((model_names[k].c_str())) == 0)
                {
                    target_row.ids.data.push_back(k+1);
                    break;
                }
            }
        }
        target.push_back(target_row);
        target_row.ids.data.clear();
    }
    for (int i=0; i < bin_contents.size(); i++)
        for (int j=0; j < bin_contents[i].size(); j++)
            boost::algorithm::to_lower(bin_contents[i][j]);
    for(int i=0; i < bin_contents.size();i++)
    {
        for(int j=0; j< bin_contents[i].size();j++)
        {
            for(int k=0;k<model_names.size();k++)
            {
                cout << bin_contents[i][j] << ":" << model_names[k].c_str() << endl;
                if(bin_contents[i][j].compare((model_names[k].c_str())) == 0)
                {
                    available_row.ids.data.push_back(k+1);
                    break;
                }
            }
        }
        available.push_back(available_row);
        available_row.ids.data.clear();
    }
    res.ids_target = target;
    res.ids_available = available;
    res.task.data = task;
    return true;
}
#else
bool JSON_MAKER::pick_object_callback(json_maker::get_bin_objectRequest &req,json_maker::get_bin_objectResponse &res)
{
    cout << "Called object pick service callback" << endl;
//    int n_bins_to_pick = 9;
//    int bin_map_array[n_bins_to_pick] = {1,4,0,3,2,5,7,6,8};

//    cout << "Iteration number \t" << iteration << endl;
//    if(iteration == n_bins_to_pick)
//        iteration = 0;

    int priority_obj_id = 0 ;

    int bin_num = req.bin_num.data;
    cout << "Requested for item to be picked in bin: " << bin_num << endl;

    for(int i=0;i<NO_CLASSES;i++)
    {
        cout << rp_items[bin_num] << "\t" << model_names[i] << endl;
//        if(rp_items[bin_map_array[iteration]].compare(model_names[i]) == 0)
        if(rp_items[bin_num].compare(model_names[i]) == 0)
        {
            priority_obj_id = i+1;
            cout << "priority_obejct id \t" << priority_obj_id << "\t" << endl;
            break;
        }
    }

//    int priority_bin_id= bin_map_array[iteration];

//    res.bin_num.data = priority_bin_id;
    res.obj_id.data = priority_obj_id;

//    iteration++;

    return true;
}
#endif
bool JSON_MAKER::pick_status_write_callback(json_maker::write_object_pick_statusRequest &req, json_maker::write_object_pick_statusResponse &res)
{
    cout << "IDs Available size is : " << req.ids_available.size() << endl;
    cout << "IDs Picked is : " << req.ids_picked.size() << endl;

    for (int i=0;i<req.ids_available.size();i++)
    {
        bin_contents[i].clear();
        for (int j=0;j<req.ids_available[i].ids.data.size();j++)
            bin_contents[i].push_back(model_names[req.ids_available[i].ids.data[j]-1]);
    }
    for (int i=0;i<req.ids_picked.size();i++)
    {
        rp_items_all[i].clear();
        for (int j=0;j<req.ids_picked[i].ids.data.size();j++)
            rp_items_all[i].push_back(model_names[req.ids_picked[i].ids.data[j]-1]);
    }
    string package_path = ros::package::getPath("json_maker");
    string rp_out_path = package_path.append("/data/examples/arc_2017/pick_status.json");
    writeRPJsonFile(rp_out_path,bin_contents,rp_items_all,rp_picked);
    return true;
}

#endif

#if(STOWING_TASK)
bool JSON_MAKER::write_stow_data_service_callback(json_maker::write_stow_dataRequest &req, json_maker::write_stow_dataResponse &res)
{

    // Compare object id returned by the services with the objects in the json and set the index accordingly.

    int obj_id_in_json;
    int obj_id = req.obj_id.data-1;
    bool found_id = false;
    // Run a for loop and the object returned from apc_controller lies in req.obj_id.data. Compare it with array in json file data
    // and store its index in obj_id_in_jsonvariable
    for(int i=0; i<tote_contents.size(); i++)
    {
        if(strcmp(tote_contents[i].c_str(),model_names[obj_id].c_str()) == 0)
        {
            obj_id_in_json = i;
            found_id = true;
            break;
        }
    }

    if(found_id)
    {
        st_picked[obj_id_in_json] = true;
        st_bin[obj_id_in_json] = req.bin_id.data;

        string package_path = ros::package::getPath("json_maker");
        string st_out_path = package_path.append("/data/examples/st_out1.json");
//        string st_out_path = "/home/ilab/tcs/apc_ws/src/apc_project/json_maker/data/examples/st_out1.json";
        writeSTJsonFile(st_out_path,bin_contents,tote_contents,st_picked,st_bin);
    }
    return true;
}

// Service call back function to send the tote contents object ids
bool JSON_MAKER::tote_contents_callback(json_maker::stowToteContents::Request &req, json_maker::stowToteContents::Response &res)
{
    cout << "Entered tote contents return call back " << endl;
    for(int i=0; i<this->tote_contents.size(); i++)
    {
        for(int j=0;j<NO_CLASSES;j++)
        {
            if(tote_contents[i].compare(model_names[j]) == 0)
            {
                int priority_obj_id = j+1;
                res.tote_contents.data.push_back(priority_obj_id);// push the object index
            }
        }
    }
    return true;
}



bool JSON_MAKER::get_bin_contents_callback(json_maker::getBinContentsRequest &req, json_maker::getBinContentsResponse &res)
{

    for(int i=3;i<12;i++)
    {
        for(int j=0;j<bin_contents[i].size();j++)
        {
            string obj_name = bin_contents[i][j];


            for(int k=0;k<NO_CLASSES;k++)
            {
                if(strcmp(obj_name.c_str(),model_names[k].c_str()) == 0)
                {
                    if(i==3)
                    {
                    res.bin_content_d.data.push_back(k);
                    break;
                    }
                    else if(i==4)
                    {
                    res.bin_content_e.data.push_back(k);
                    break;
                    }
                    else if(i==5)
                    {
                    res.bin_content_f.data.push_back(k);
                    break;
                    }
                    else if(i==6)
                    {
                    res.bin_content_g.data.push_back(k);
                    break;
                    }
                    else if(i==7)
                    {
                    res.bin_content_h.data.push_back(k);
                    break;
                    }
                    else if(i==8)
                    {
                    res.bin_content_i.data.push_back(k);
                    break;
                    }
                    else if(i==9)
                    {
                    res.bin_content_j.data.push_back(k);
                    break;
                    }
                    else if(i==10)
                    {
                    res.bin_content_k.data.push_back(k);
                    break;
                    }
                    else if(i==11)
                    {
                    res.bin_content_l.data.push_back(k);
                    break;
                    }

                }
            }
        }
    }

    return true;
}
#endif
JSON_MAKER::~JSON_MAKER()
{

}

int main(int argc, char **argv)
{

    ros::init(argc,argv,"json_reader_Writer");

    JSON_MAKER *json_maker = new JSON_MAKER();

    ros::spin();



    return 0;
}
