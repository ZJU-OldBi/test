#pragma once
#include <queue>

namespace Filter{
    class lpf
    {
    private:
        std::vector<double> x_buffer;
        std::vector<double> y_buffer;
        int order;
        std::vector<double> den, num;
        
    public:
        lpf();
        ~lpf();
        
        /*
        den(0) == 1
        order+1 == len(den) == len(num)
        */
        void init(int order,std::vector<double> den,std::vector<double> num);
        double filtrate(double value);
    };
    

    
}