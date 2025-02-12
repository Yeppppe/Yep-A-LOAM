// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();    //记录当前时间为开始时间
    }

    double toc()
    {
        end = std::chrono::system_clock::now();             //记录当前时间为结束时间
        std::chrono::duration<double> elapsed_seconds = end - start;     //记录开始和结束时间的时间间隔，单位为秒
        return elapsed_seconds.count() * 1000;               //把秒转换为毫秒
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;    //私有变量记录开始和结束时间
};
