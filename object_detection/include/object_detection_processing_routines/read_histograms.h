#ifndef READ_HISTOGRAMS_H
#define READ_HISTOGRAMS_H

//---READ HISTOGRAMS--
unsigned int read_histograms_accumulated(std::string hist_file_path, std::vector<cv::Mat>& histograms)
{
    unsigned int number_of_hists;

    FILE* output_file = fopen(hist_file_path.c_str(), "rb");

    int dims;
    int *hist_size;

    fread((void*)&number_of_hists, sizeof(int), 1, output_file);
    fread((void*)&dims, sizeof(int), 1, output_file);

    hist_size = new int[dims];
    fread(hist_size, sizeof(int), dims, output_file);

    printf("%d %d ", number_of_hists, dims);

    for (int i = 0; i < dims; i++)
        printf("%d ", hist_size[i]);
    printf("\n");

    int no_of_elements = 1.0;

    for (int i = 0; i < dims; i++)
        no_of_elements *= hist_size[i];

    for (int i = 0; i < number_of_hists; i++)
    {
        cv::Mat histogram;
        histogram.create(dims, hist_size, CV_32FC1);


        fread(histogram.data, histogram.step[dims - 1], no_of_elements, output_file);
        histograms.push_back(histogram);

        histogram.release();
    }

    fclose(output_file);

    return number_of_hists;
}

void read_all_hostograms(int model_no,std::vector<cv::Mat>& vector_histograms,std::string database_path)
{
    std::string model_depth_hist_file_path = "/image_database/depth histograms/DEPTH_HIST_";
    std::string model_color_hist_file_path = "/image_database/color histograms/COLOR_HIST_";

    std::stringstream model_depth_hist_file_name;
    model_depth_hist_file_name << database_path << model_depth_hist_file_path.c_str() << model_names[model_no] << ".txt";

    std::stringstream model_color_hist_file_name;
    model_color_hist_file_name << database_path << model_color_hist_file_path.c_str() << model_names[model_no] << ".txt";

    std::cout << model_depth_hist_file_name.str() << std::endl;
    std::cout << model_color_hist_file_name.str() << std::endl;

    read_histograms_accumulated(model_depth_hist_file_name.str().c_str(), vector_histograms);
    read_histograms_accumulated(model_color_hist_file_name.str().c_str(), vector_histograms);


    for (int i = 0; i < vector_histograms.size()-1; i++)
    {
        cv::normalize(vector_histograms[i], vector_histograms[i], 0.0, 1.0, cv::NORM_MINMAX);
    }
    for (int i = 0; i<vector_histograms[vector_histograms.size() - 1].cols;i++)
    cv::normalize(vector_histograms[vector_histograms.size() - 1].col(i), vector_histograms[vector_histograms.size() - 1].col(i), 0.0, 1.0, cv::NORM_MINMAX);

    std::cout << "HISTOGRAMS READ" << std::endl;

    //for (int i = 0; i < vector_histograms.size();i++)
    //std::cout << vector_histograms[i] << std::endl << std::endl;

}

#endif // READ_HISTOGRAMS_H
