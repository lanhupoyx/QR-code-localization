#pragma once

#include "utility_qloc.hpp"

// 线段对象，用于划定运行区域
class LineSegment
{
private:
    float x1_, y1_, x2_, y2_;
    float a_, b_;// y = ax+b
    float x_; // x = n
    float y_; // y = n

    Logger *logger;

public:
    LineSegment(float x1, float y1, float x2, float y2) : x1_(x1), y1_(y1), x2_(x2), y2_(y2)
    {
        logger = &Logger::getInstance();
        if (x1_ == x2_)
        {
            // logger->info("error: x1 == x2");
            x_ = x1_;
        }
        else if (y1_ == y2_)
        {
            y_ = y1_;
        }
        else
        {
            a_ = (y1_ - y2_) / (x1_ - x2_);
            b_ = y1_ - a_ * x1_;
        }
    }

    ~LineSegment() {}

    // 判定从该点出发的射线是否与本线段相交
    bool isCross(float x, float y)
    {
        // 射线从(x,y)出发，沿y轴方向射出
        if (x1_ == x2_) // 线段平行于y轴
        {
            if (x == x_)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (y1_ == y2_) // 线段平行于x轴
        {
            if (((x < x1_) && (x < x2_)) || ((x > x1_) && (x > x2_)))
            {
                return false;
            }
            else if (y > y_)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else // 线段倾斜
        {
            float y_cross = a_ * x + b_;
            if (y_cross < y)
            {
                return false;
            }
            if (((x < x1_) && (x < x2_)) || ((x > x1_) && (x > x2_)))
            {
                return false;
            }
            else
            {
                return true;
            }
        }
    }
};

// 区域对象，用于表示运行区域
class Polygon : public ParamServer
{
private:
    std::list<LineSegment> linelist;

public:
    Polygon()
    {
        int pointNum = enableArea.size() / 2;

        for (int i = 0; i < pointNum - 1; i++)
        {
            LineSegment line(enableArea[i * 2], enableArea[i * 2 + 1], enableArea[i * 2 + 2], enableArea[i * 2 + 3]);
            linelist.push_back(line);
        }
        LineSegment line2(enableArea[(pointNum-1) * 2], enableArea[(pointNum-1) * 2 + 1], enableArea[0], enableArea[1]);
        linelist.push_back(line2);
    }

    ~Polygon() {}

    // 判定该点是否为本区域内的点，区域内部点出发的射线与区域边相交的次数为奇数
    bool isInArea(float x, float y)
    {
        //return true;
        int corss_num = 0;
        for (std::list<LineSegment>::iterator it = linelist.begin(); it != linelist.end(); it++)
        {
            if (it->isCross(x, y))
            {
                corss_num++;
            }
        }
        if ((corss_num % 2) == 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};
