#ifndef JSON_HELPER_H
#define JSON_HELPER_H

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "json/json.h"
#include <std_msgs/String.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>

#define ARC_2017    1

using namespace std;
#if(ARC_2017)
#define NO_BINS 2
#define NO_OF_BOXES 5
#else
#define NO_BINS 12
#endif

#define PICKING_TASK    1
#define STOWING_TASK    0
#if(ARC_2017)
//#define NO_CLASSES 0
#else
#define NO_CLASSES 22
#endif

#if(ARC_2017)
string bin_names[] = {"A","B"};
vector<string> boxes;
vector<string> boxes_present;
#else
string bin_names[] = {"bin_A","bin_B","bin_C","bin_D","bin_E","bin_F",
                      "bin_G","bin_H","bin_I","bin_J","bin_K","bin_L"};
#endif

#if(ARC_2017)
vector<std::string> model_names, set_provided;
vector<int> set_id;
/*=
{
    "Toilet_Brush","Avery_Binder","Balloons","Band_Aid_Tape","Bath_Sponge","Black_Fashion_Gloves","Burts_Bees_Baby_Wipes"
    ,"Colgate_Toothbrush_4PK","Composition_Book","Crayons","Duct_Tape","Epsom_Salts","Expo_Eraser","Fiskars_Scissors"
    ,"Flashlight","Glue_Sticks","Hand_Weight","Hanes_Socks","Hinged_Ruled_Index_Cards","Ice_Cube_Tray","Irish_Spring_Soap"
    ,"Laugh_Out_Loud_Jokes","Marbles","Measuring_Spoons","Mesh_Cup","Mouse_Traps","Pie_Plates","Plastic_Wine_Glass"
    ,"Poland_Spring_Water","Reynolds_Wrap","Robots_DVD","Robots_Everywhere","Scotch_Sponges","Speed_Stick","White_Facecloth"
    ,"Table_Cloth","Tennis_Ball_Container","Ticonderoga_Pencils","Tissue_Box","Windex"
};*/
#else
std::string model_names[] = {
    "barbie_book","black_ball","brown_plastic_cup","camlin_color_pencils","care_mate_swipes",
    "cleaning_brush","cloth_clips","devi_coffee",
    "dove_soap","feeding_bottle","fevicol",
    "garnet_bulb","green_battery","nivea_deo",
    "patanjali_toothpaste","realsense_box","red_green_ball",
    "scissors","skipping_rope","socks","tissue_paper","cotton_balls"
};
#endif

#if(ARC_2017)
bool readJsonFile(vector<string> &file_path, vector<vector<string> > &rack_contents, vector<string> &rp_objects, vector<string> &tote_items, vector<vector<string> > &rp_items_all)
{
    // Model names in lower case.
    for (int i=0;i<model_names.size();i++)
        boost::algorithm::to_lower(model_names[i]);

    std::ifstream ifile1(file_path[0].c_str()),ifile2(file_path[1].c_str()),ifile3(file_path[2].c_str());

    cout << "Received file paths are : " << endl << file_path[0].c_str() << " , " << endl << file_path[1].c_str() << " and "  << endl << file_path[2].c_str() << endl;

    Json::Reader reader1;
    Json::Value root1,root2,root3;
    if ( ifile1 != NULL && reader1.parse(ifile1, root1) && ifile2 != NULL && reader1.parse(ifile2, root2) && ifile3 != NULL && reader1.parse(ifile3,root3))
    {
        cout << "Parsed file. " << endl;
        if(root1.isMember("bins"))
        {
            const Json::Value bins = root1["bins"];

            for(int i=0; i<bins.size(); i++)
            {
                string bin = bins[i]["bin_id"].asString();
                cout << "Bin ID is : " << bin << endl << endl;
                vector<string> object_names;
                for (int j=0; j<bins[i]["contents"].size(); j++)
                {
                    object_names.push_back(bins[i]["contents"][j].asString());
                    cout << "In bin " << bin << " object is : " << bins[i]["contents"][j].asString() << endl;
                }
                cout << endl;
                for (int i=0;i<object_names.size();i++)
                    boost::algorithm::to_lower(object_names[i]);
                rack_contents[i] = object_names;
                object_names.clear();
            }
        }
        else
            cerr << "Failed to parse JSON file: " << file_path[0] << endl;

        string bin[2] = {"A","B"};
        for (int i=0; i<rack_contents.size();i++)
            for (int j=0; j<rack_contents[i].size();j++)
                for(int k=0; k<set_provided.size();k++)
                {
                    boost::algorithm::to_lower(rack_contents[i][j]);
                    if(rack_contents[i][j].compare(set_provided[k])==0)
                        break;
                    else if(k == set_provided.size()-1)
                    {
                        cout << "Bin content \033[1;31m" << rack_contents[i][j] << "\033[0m in bin \033[1;31m" <<  bin[i] << "\033[0m does not match the list provided in the pen drive." << endl;
                    }
                }

        if(root1.isMember("tote"))
        {
            for(int i=0; i<root1["tote"]["contents"].size(); i++)
            {
                tote_items.push_back(root1["tote"]["contents"][i].asString());
                cout << "Tote content is : " << root1["tote"]["contents"][i].asString() << endl;
            }
            for (int i=0; i<tote_items.size(); i++)
            {
                boost::algorithm::to_lower(tote_items[i]);
                cout << "Tote content in lower case is : " << tote_items[i] << endl;
            }
            cout << endl;
        }
        else
            cerr << "Failed to parse JSON file: " << file_path[0] << endl;

        if(root3.isMember("boxes"))
        {
            for (int i=0; i<root3["boxes"].size();i++)
            {
                boxes.push_back(root3["boxes"][i]["size_id"].asString());
            }
            for (int i=0; i<boxes.size(); i++)
                boost::algorithm::to_lower(boxes[i]);
            for (int i=0; i<boxes.size(); i++)
                cout << "Box ID in lower case is : " << boxes[i] << endl;
        }
        else
           cerr << "Failed to parse JSON file: " << file_path[2] << endl;

        if(root2.isMember("orders"))
        {
            const Json::Value order = root2["orders"];
            cout << "Order is : \n";
            for(int i=0; i<order.size(); i++)
            {
                string box = order[i]["size_id"].asString();
                boost::algorithm::to_lower(box);
                cout << "Box is " << box << endl;
                vector<string> orders_to_pick;
                for(int j=0; j<order[i]["contents"].size(); j++)
                {
                    cout << box << " : ";
                    cout << order[i]["contents"][j].asString() << endl;
                    orders_to_pick.push_back(order[i]["contents"][j].asString());
                }
                for (int k=0;k<boxes.size();k++)
                {
                    if(strcmp(box.c_str(),boxes[k].c_str())==0)
                        rp_items_all[k] = orders_to_pick;
                }
            }
            for (int i=0;i<rp_items_all.size();i++)
            {
                cout << "Box is : " << boxes[i] << endl;
                if(rp_items_all[i].size()>0)
                {
                    for(int j=0;j<rp_items_all[i].size();j++)
                    {
                        cout << "Item from box " << boxes[i] << " is : " << rp_items_all[i][j] << endl;
                    }
                    boxes_present.push_back(boxes[i]);
                }
                cout << endl;
            }
        }
        else
            cerr << "Failed to parse JSON file: " << file_path[1] << endl;
        for(int i=0; i<rp_items_all.size(); i++)
            for(int j=0; j<rp_items_all[i].size(); j++)
                for (int k=0; k<set_provided.size(); k++)
                {
                    boost::algorithm::to_lower(rp_items_all[i][j]);
                    if(rp_items_all[i][j].compare(set_provided[k])==0)
                        break;
                    else if(k == set_provided.size()-1)
                    {
                        cout << "Order content \033[1;31m" << rp_items_all[i][j] << "\033[0m for box \033[1;31m" <<  i << "\033[0m does not match the list provided in the pen drive." << endl;
                    }
                }

        cout << "Parsed: " << endl << "1) " << file_path [0]  << " , " << endl << "2) " << file_path[1] << " , " << endl << "3) " << file_path[2] << endl;
        return true;
    }
    else
    {
        cout << "\033[1;31m" << "Failed to parse JSON files. Check the JSON files." << "\033[0m" << endl;
        if(ifile1 == NULL || ifile2 == NULL || ifile3 == NULL)
            cerr << "Failed to open JSON file: " << file_path[0] << " and " << file_path[1] << " and " << file_path[2] << endl;
        return false;
    }
}
#else
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
                                        cout << bin.c_str() << ":\n";
                    vector<string> object_names;
                    for (unsigned int i = 0; i < bin_contents[bin.c_str()].size(); i++)
                    {
                                                cout << bin_contents[bin.c_str()][i].asString() << endl;
                        object_names.push_back(bin_contents[bin.c_str()][i].asString());
                    }
                                        cout << endl;
                    rack_contents[j] = object_names;
                }
            }
        }

        if(root.isMember("work_order"))
        {
            const Json::Value order = root["work_order"];
                        cout << "work order\n";

            for(int j=0; j<order.size(); j++)
            {
                string bin = order[j]["bin"].asString();
                string item = order[j]["item"].asString();

                for(int k=0; k<NO_BINS; k++)
                {
                    if(strcmp(bin.c_str(),bin_names[k].c_str()) == 0)
                    {
                                                cout << bin.c_str() << ": ";
                                                cout << item.c_str() << endl;
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
#endif

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

    bool all_picked = true; // to check whether all tote contents are picked, tote is empty
    for(int i=0; i<picked.size(); i++)
    {
        if(!picked[i])
        {
            all_picked = false;
            break;
        }
    }
    if(all_picked)
        root["tote_contents"] = Json::Value (Json::arrayValue);

    std::ofstream file_id;
    file_id.open(file_path.c_str());

    if(!file_id.is_open())
    {
        cerr << "Error opening file in json" << endl;
        exit(0);
    }

    file_id << root;

    file_id.close();


    return true;
}

// file_path: path to the output file
// bin_items: array of bins(12) with each containing array of names of items in the respective bin
// pick_items: array of items to be picked from each bin. Indexed with the bin number.
// picked: array of bool indicating whether desired object from the respective bin is picked or not
#if(ARC_2017)
bool writeRPJsonFile(string &file_path, vector< vector<string> > &bin_items, vector< vector<string> > &pick_items, vector<bool> &picked)
{
    Json::Value root_write;
    int bin_items_counter=0,pick_items_counter=0,box_to_write=0;

    for (int i=0;i<bin_items.size();i++)
    {
        if(bin_items[i].size()>0)
        {
            for (int j=0;j<bin_items[i].size();j++)\
            {
                root_write["bins"][bin_items_counter]["bin_id"] = bin_names[i];
                root_write["bins"][bin_items_counter]["contents"][j] = bin_items[i][j];
            }
            bin_items_counter+=1;
        }
    }

    for (int i=0;i<boxes_present.size();i++)
    {
        boost::algorithm::to_upper(boxes_present[i]);
        cout << "Boxes are : " << boxes_present[i] << endl;
        root_write["boxes"][i]["size_id"] = boxes_present[i];
        root_write["boxes"][i]["contents"]= Json::arrayValue;
    }

    for (int i=0;i<pick_items.size();i++)
    {
        boost::algorithm::to_upper(boxes[i]);
        if(pick_items[i].size()>0)
        {
            for (int l=0;l<boxes_present.size();l++)
                if((boxes[pick_items_counter].compare(boxes_present[l]))==0)
                {
                    cout << "Box is : " << boxes_present[l] << endl;
                    box_to_write = l;
                    break;
                }
            for (int j=0;j<pick_items[i].size();j++)
            {
                root_write["boxes"][box_to_write]["contents"][j] = pick_items[i][j];
            }
        }
        pick_items_counter+=1;
    }
    root_write["tote"]["contents"]=Json::arrayValue;

    //cout << "Root is " << root_write;

    std::ofstream file_id;
    file_id.open(file_path.c_str());

    if(!file_id.is_open())
    {
        cerr << "Error opening file in json" << endl;
        exit(0);
    }
    file_id << root_write;
    file_id.close();

    return true;
}

#else
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
//                    cout << "tote " << i << ": " << bin_items[i][j] << endl;
                }
                else
                {
                    root["bin_contents"][bin_names[i]][bin_size] = bin_items[i][j];
                    bin_size++;
//                    cout << "bin " << i << ": " << bin_items[i][j] << endl;
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

    if(!file_id.is_open())
    {
        cerr << "Error opening file in json" << endl;
        exit(0);
    }

    file_id << root;

    file_id.close();


    return true;
}
#endif


#endif // JSON_HELPER_H
