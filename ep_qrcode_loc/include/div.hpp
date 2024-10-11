#pragma once

#include "utility_qloc.hpp"

// 数据分割
class Div : public ParamServer
{
private:
    std::mutex mtx;

    Logger *logger;
    std::stringstream stream;

    

public:
    Div(std::string csv_path)
    {
        // 读取文本文件中库位相关信息
        logger = &Logger::getInstance();
        logger->info("Div Start");
        std::string config_path;
        config_path = csv_path + "config.csv";
        std::ifstream config_ifs;
        config_ifs.open(config_path, std::ios::in);
        if (!config_ifs.is_open())
        {
            logger->info(config_path + "打开失败!");
        }
        else
        {
            logger->info(config_path + "打开成功!");
        }
        std::string config_buf;               // 将数据存放到c++ 中的字符串中
        while (std::getline(config_ifs, config_buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
        {
            std::stringstream line_ss;
            line_ss << config_buf;

            // 跳过空行
            if("" == config_buf)
            {
                continue;
            }

            std::string name;
            std::vector<uint32_t> pos;
            pos.resize(19);

            // 提取信息
            line_ss >> name;
            
            for(int i = 0; i < 19; i++)
            {
                line_ss >> pos[i];
            }

            std::string data_in_path = csv_path + name + ".csv";
            std::string data_out_path = csv_path +"output/" + name;

            csv_cut(data_in_path, pos[0], pos[1], data_out_path + "车-17列-出-1.csv");
            csv_cut(data_in_path, pos[2], pos[3], data_out_path + "车-16列-进-1.csv");
            csv_cut(data_in_path, pos[3], pos[4], data_out_path + "车-16列-出-1.csv");
            csv_cut(data_in_path, pos[5], pos[6], data_out_path + "车-17列-进-1.csv");

            csv_cut(data_in_path, pos[6], pos[7], data_out_path + "车-17列-出-2.csv");
            csv_cut(data_in_path, pos[8], pos[9], data_out_path + "车-16列-进-2.csv");
            csv_cut(data_in_path, pos[9], pos[10], data_out_path + "车-16列-出-2.csv");
            csv_cut(data_in_path, pos[11], pos[12], data_out_path + "车-17列-进-2.csv");

            csv_cut(data_in_path, pos[12], pos[13], data_out_path + "车-17列-出-3.csv");
            csv_cut(data_in_path, pos[14], pos[15], data_out_path + "车-16列-进-3.csv");
            csv_cut(data_in_path, pos[15], pos[16], data_out_path + "车-16列-出-3.csv");
            csv_cut(data_in_path, pos[17], pos[18], data_out_path + "车-17列-进-3.csv");

            std::list<std::string> path_list;

            path_list.clear();
            path_list.push_back(data_out_path + "车-16列-进-1.csv");
            path_list.push_back(data_out_path + "车-16列-进-2.csv");
            path_list.push_back(data_out_path + "车-16列-进-3.csv");
            csv_merget(path_list, data_out_path + "车-16列-进-123.csv");

            path_list.clear();
            path_list.push_back(data_out_path + "车-16列-出-1.csv");
            path_list.push_back(data_out_path + "车-16列-出-2.csv");
            path_list.push_back(data_out_path + "车-16列-出-3.csv");
            csv_merget(path_list, data_out_path + "车-16列-出-123.csv");

            path_list.clear();
            path_list.push_back(data_out_path + "车-17列-进-1.csv");
            path_list.push_back(data_out_path + "车-17列-进-2.csv");
            path_list.push_back(data_out_path + "车-17列-进-3.csv");
            csv_merget(path_list, data_out_path + "车-17列-进-123.csv");

            path_list.clear();
            path_list.push_back(data_out_path + "车-17列-出-1.csv");
            path_list.push_back(data_out_path + "车-17列-出-2.csv");
            path_list.push_back(data_out_path + "车-17列-出-3.csv");
            csv_merget(path_list, data_out_path + "车-17列-出-123.csv");



            path_list.clear();
            path_list.push_back(data_out_path + "车-16列-进-1.csv");
            path_list.push_back(data_out_path + "车-16列-出-1.csv");
            csv_merget(path_list, data_out_path + "车-16列-进出-1.csv");

            path_list.clear();
            path_list.push_back(data_out_path + "车-17列-进-1.csv");
            path_list.push_back(data_out_path + "车-17列-出-1.csv");
            csv_merget(path_list, data_out_path + "车-17列-进出-1.csv");

            path_list.clear();
            path_list.push_back(data_out_path + "车-16列-进-2.csv");
            path_list.push_back(data_out_path + "车-16列-出-2.csv");
            csv_merget(path_list, data_out_path + "车-16列-进出-2.csv");

            path_list.clear();
            path_list.push_back(data_out_path + "车-17列-进-2.csv");
            path_list.push_back(data_out_path + "车-17列-出-2.csv");
            csv_merget(path_list, data_out_path + "车-17列-进出-2.csv");

            path_list.clear();
            path_list.push_back(data_out_path + "车-16列-进-3.csv");
            path_list.push_back(data_out_path + "车-16列-出-3.csv");
            csv_merget(path_list, data_out_path + "车-16列-进出-3.csv");

            path_list.clear();
            path_list.push_back(data_out_path + "车-17列-进-3.csv");
            path_list.push_back(data_out_path + "车-17列-出-3.csv");
            csv_merget(path_list, data_out_path + "车-17列-进出-3.csv");

        }
        config_ifs.close();
        logger->info(config_path + "读取完毕!");
    }

    ~Div(){}

    void csv_cut(std::string source_path, uint32_t start_line, uint32_t stop_line, std::string des_path)
    {
        // 打开对应的数据文件
        std::ifstream data_ifs;
        data_ifs.open(source_path, std::ios::in);
        if (!data_ifs.is_open())
        {
            std::cout << source_path + "打开失败!" << std::endl;
        }
        else
        {
            std::cout << source_path + "打开成功!" << std::endl;
        }

        std::ofstream data_ofs;
        data_ofs.open(des_path, std::ios::out);
        if (!data_ofs.is_open())
        {
            std::cout << des_path + "打开失败!" << std::endl;
        }
        else
        {
            std::cout << des_path + "打开成功!" << std::endl;
        }

        // 第一行
        //data_ofs << ",,,,,实时-yaw,实时-x,实时-y,参考-x,参考-y,参考-yaw,,,,,,,,,," << std::endl;

        uint32_t line_num = 0;
        std::string data_buf;                    // 将数据存放到c++ 中的字符串中
        while (std::getline(data_ifs, data_buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
        {
            line_num++;
            if (line_num < start_line)
            {
                // 无用数据
            }
            else if (line_num < stop_line)
            {
                data_ofs << data_buf << std::endl;
            }
            else if (line_num == stop_line + 1)
            {
                data_ofs.close();
                std::cout << des_path + "关闭" << std::endl;
                break;
            }
        }
        data_ifs.close();
        std::cout << source_path + "关闭" << std::endl;
    }

    void csv_merget(std::list<std::string> source_path_list, std::string des_path)
    {
        std::ofstream data_ofs;
        data_ofs.open(des_path, std::ios::out);
        if (!data_ofs.is_open())
        {
            std::cout << des_path + "打开失败!" << std::endl;
        }
        else
        {
            std::cout << des_path + "打开成功!" << std::endl;
        }

        for(std::list<std::string>::iterator it = source_path_list.begin(); it != source_path_list.end(); it++)
        {
            std::ifstream data_ifs;
            data_ifs.open(*it, std::ios::in);
            if (!data_ifs.is_open())
            {
                std::cout << *it + "打开失败!" << std::endl;
            }
            else
            {
                std::cout << *it + "打开成功!" << std::endl;
            }

            std::string data_buf;                    // 将数据存放到c++ 中的字符串中
            while (std::getline(data_ifs, data_buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
            {
                // 跳过空行
                if("" == data_buf)
                {
                    continue;
                }

                data_ofs << data_buf << std::endl;
            }
            data_ifs.close();
            std::cout << *it + "关闭" << std::endl;
        }

        data_ofs.close();
        std::cout << des_path + "关闭" << std::endl;
    }
};
