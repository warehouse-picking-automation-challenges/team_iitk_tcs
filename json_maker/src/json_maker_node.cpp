#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "json/json.h"
#include <std_msgs/String.h>
#include <ros/package.h>

using namespace std;

string bin_names[] = {"bin_A","bin_B","bin_C","bin_D","bin_E","bin_F",
                      "bin_G","bin_H","bin_I","bin_J","bin_K","bin_L"};

#define RP_TASK 1
#define ST_TASK 0
#define NO_BINS 12

#define PICKING_TASK    1
#define STOW_TASK   0

bool readJsonFile(string &file_path, vector< vector<string> > &rack_contents, vector<string> &rp_objects,
                  vector<string> &tote_items)
{
    std::ifstream ifile(file_path.c_str());
    Json::Reader reader;
    Json::Value root;
    if (ifile != NULL && reader.parse(ifile, root))
    {
        if(root.isMember("bin_contents"))
        {
            const Json::Value bin_contents = root["bin_contents"];

            for(int j=0; j<rack_contents.size(); j++)
            {
                string bin = bin_names[j];
                if(bin_contents.isMember(bin.c_str()))
                {
                    //                    cout << bin.c_str() << ":\n";
                    vector<string> object_names;
                    for (unsigned int i = 0; i < bin_contents[bin.c_str()].size(); i++)
                    {
                        //                        cout << bin_contents[bin.c_str()][i].asString() << endl;
                        object_names.push_back(bin_contents[bin.c_str()][i].asString());
                    }
                    //                    cout << endl;
                    rack_contents[j] = object_names;
                }
            }
        }

        if(root.isMember("work_order"))
        {
            const Json::Value order = root["work_order"];
            //            cout << "work order\n";

            for(int j=0; j<order.size(); j++)
            {
                string bin = order[j]["bin"].asString();
                string item = order[j]["item"].asString();

                for(int k=0; k<NO_BINS; k++)
                {
                    if(strcmp(bin.c_str(),bin_names[k].c_str()) == 0)
                    {
                        //                        cout << bin.c_str() << ": ";
                        //                        cout << item.c_str() << endl;
                        rp_objects[k] = item;
                    }
                }
            }
        }
        if(root.isMember("tote_contents"))
        {
            for(int i=0; i<root["tote_contents"].size(); i++)
            {
                tote_items.push_back(root["tote_contents"][i].asString());
            }
        }
        return true;
    }
    else
    {
        if(ifile == NULL)
            cerr << "Failed to open JSON file: " << file_path << endl;
        else
            cerr << "Failed to parse JSON file: " << file_path << endl;
        return false;
    }
}

// file_path: path to the output file
// bin_items: array of bins(12) with each containing array of names of items in the respective bin
// tote_items: array of items to be picked from tote
// picked: array of bool indicating whether tote object as indexed in tote_items is picked or not
// dest_bin: array of bin index in which the tote object is placed
bool writeSTJsonFile(string &file_path, vector< vector<string> > &bin_items, vector<string> &tote_items,
                     vector<bool> &picked, vector<int> &dest_bin)
{
    Json::Value root;
    vector<bool> bin_empty(12, true);// In case bin is empty, flag is required to write empty array to json file
    if(bin_items.size() == NO_BINS)
    {
        // First put all the existing items in the rack into root
        for(int i=0; i<NO_BINS; i++)
        {
            for(int j=0; j<bin_items[i].size(); j++)
            {
                root["bin_contents"][bin_names[i]].append(bin_items[i][j]);
                bin_empty[i] = false;
            }
        }

        if(tote_items.size() == dest_bin.size())
        {
            for(int i=0; i<tote_items.size(); i++)
            {
                // if the tote item is picked then push the item into bin contents else tote contents
                if(picked[i])
                {
                    int bin = dest_bin[i];
                    root["bin_contents"][bin_names[bin]].append(tote_items[i]);
                    bin_empty[bin] = false;
                }
                else
                {
                    root["tote_contents"].append(tote_items[i]);
                }
            }

            // put empty array if any bin content is empty
            for(int i=0; i<NO_BINS; i++)
            {
                if(bin_empty[i])
                    root["bin_contents"][bin_names[i]] = Json::Value (Json::arrayValue);
            }
        }
        else
        {
            cerr << "No. of tote_items and dest bin are not equal" << endl;
            return false;
        }
    }
    else
    {
        cerr << "No of bins given is less that 12" << endl;
        return false;
    }

    std::ofstream file_id;
    file_id.open(file_path.c_str());

    file_id << root;


    return true;
}

// file_path: path to the output file
// bin_items: array of bins(12) with each containing array of names of items in the respective bin
// pick_items: array of items to be picked from each bin. Indexed with the bin number.
// picked: array of bool indicating whether desired object from the respective bin is picked or not
bool writeRPJsonFile(string &file_path, vector< vector<string> > &bin_items, vector<string> &pick_items, vector<bool> &picked)
{
    Json::Value root;

    if(bin_items.size() == NO_BINS && pick_items.size() == NO_BINS)
    {
        int tote_size = 0;

        for(int i=0; i<NO_BINS; i++)
        {
            int bin_size = 0;
            bool item_found = false;// Because multiple copies of same item could be there
            for(int j=0; j<bin_items[i].size(); j++)
            {
                // if bin item is the to be picked item then put to tote if picked
                if(strcmp(bin_items[i][j].c_str(), pick_items[i].c_str()) == 0 && picked[i] && !item_found)
                {
                    item_found = true;
                    root["tote_contents"][tote_size] = bin_items[i][j];
                    tote_size++;
                    cout << "tote " << i << ": " << bin_items[i][j] << endl;
                }
                else
                {
                    root["bin_contents"][bin_names[i]][bin_size] = bin_items[i][j];
                    bin_size++;
                    cout << "bin " << i << ": " << bin_items[i][j] << endl;
                }
            }
            if(bin_size == 0)
                root["bin_contents"][bin_names[i]] = Json::Value (Json::arrayValue);

        }
    }
    else
    {
        cerr << "No of bins not equal to 12" << endl;
        return false;
    }

    std::ofstream file_id;
    file_id.open(file_path.c_str());

    file_id << root;


    return true;
}

void printJSONValue(const Json::Value &val)
{
    if( val.isString() ) {
        printf( "string(%s)", val.asString().c_str() );
    } else if( val.isBool() ) {
        printf( "bool(%d)", val.asBool() );
    } else if( val.isInt() ) {
        printf( "int(%d)", val.asInt() );
    } else if( val.isUInt() ) {
        printf( "uint(%u)", val.asUInt() );
    } else if( val.isDouble() ) {
        printf( "double(%f)", val.asDouble() );
    }
    else
    {
        printf( "unknown type=[%d]", val.type() );
    }
    getchar();
    return;
}

bool printJSONTree( const Json::Value &root, unsigned short depth /* = 0 */)
{
    depth += 1;
    printf( " {type=[%d], size=%d}", root.type(), root.size() );

    if( root.size() > 0 ) {
        printf("\n");
        for( Json::Value::const_iterator itr = root.begin() ; itr != root.end() ; itr++ ) {
            // Print depth.
            for( int tab = 0 ; tab < depth; tab++) {
                printf("-");
            }
            printf(" subvalue(");
            printJSONValue(itr.key());
            printf(") -");
            printJSONTree( *itr, depth);
        }
        return true;
    } else {
        printf(" ");
        printJSONValue(root);
        printf( "\n" );
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"json_maker");
    ros::NodeHandle node;


    vector< vector<string> > bin_contents(NO_BINS);
    vector<string> tote_contents;
    vector<string> rp_items(NO_BINS);
    // Read JSON File for picking task
#if(PICKING_TASK)
    string path = "/home/ravi/ros_projects/apc_ws/src/apc_demo/json_maker/data/examples/rp_input.json";
    readJsonFile(path, bin_contents, rp_items, tote_contents);
#elif(STOW_TASK)
    string path = "/home/ravi/ros_projects/apc_ws/src/apc_demo/json_maker/data/examples/stow_input.json";
    readJsonFile(path, bin_contents, rp_items, tote_contents);
#endif



    /*
//    ros::ServiceServer service_rp_read_file = node.advertiseService("/rack_pick/readfile", rpReadFileCallback);
//    ros::ServiceServer service_rp_send_data = node.advertiseService("/rack_pick/send_data", rpSendDataCallback);
//    ros::ServiceServer service_st_read_file = node.advertiseService("/stow_pick/readfile", stReadFileCallback);
//    ros::ServiceServer service_st_send_data = node.advertiseService("/stow_pick/send_data", stSendDataCallback);

//    Read either Rack pick or stow task ip Json File
    vector< vector<string> > bin_contents(NO_BINS);
    vector<string> tote_contents;
    vector<string> rp_items(NO_BINS);
#if(RP_TASK)
    string path = "/home/ravi/ros_projects/apc_ws/src/apc_demo/json_maker/data/examples/rp_input.json";
#elif(ST_TASK)
//    string path = "/home/ravi/ros_projects/apc_ws/src/apc_demo/json_maker/data/examples/st_input.json";
    string path = "/home/ravi/ros_projects/apc_ws/src/apc_demo/json_maker/data/examples/st_out.json";
#endif

    readJsonFile(path, bin_contents, rp_items, tote_contents);

    cout << "\nContents of " << endl;
    for(int i=0; i<NO_BINS; i++)
    {
        cout << bin_names[i] << "\n";
        for(int j=0; j<bin_contents[i].size(); j++)
            cout << bin_contents[i][j].c_str() << endl;
        cout << endl;
    }
    for(int i=0; i<NO_BINS; i++)
        cout << "Item to pick: " << rp_items[i] << endl;
    if(tote_contents.size()>0)
    {
        cout << "Tote contents:" << endl;
        for(int i=0; i<tote_contents.size(); i++)
            cout << tote_contents[i] << endl;
    }
    else
        cout << "No contents in tote" << endl;

//    write to op Json File
#if(RP_TASK)
    vector<bool> rp_picked(NO_BINS, false);
    for(int i=0; i<NO_BINS; i+=2)
        rp_picked[i] = true;

    string rp_out_path = "/home/ravi/ros_projects/apc_ws/src/apc_demo/json_maker/data/examples/rp_out.json";
    writeRPJsonFile(rp_out_path,bin_contents,rp_items,rp_picked);
#endif
#if(ST_TASK)
    vector<bool> st_picked(tote_contents.size(), false);
    vector<int> st_bin(tote_contents.size(), 0);
    for(int i=0; i<st_picked.size(); i+=2)
    {
        st_picked[i] = true;
        st_bin[i] = rand() % 12;
        cout << bin_names[st_bin[i]] << ":" << tote_contents[i] << endl;
    }

    string st_out_path = "/home/ravi/ros_projects/apc_ws/src/apc_demo/json_maker/data/examples/st_out1.json";
    writeSTJsonFile(st_out_path,bin_contents,tote_contents,st_picked,st_bin);
#endif

*/
    return 0;
}


