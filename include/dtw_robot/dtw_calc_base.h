#pragma once

namespace dtw_robot
{
    class calc_base
    {
    public:
        virtual int op(int a, int b)=0;
    };
}