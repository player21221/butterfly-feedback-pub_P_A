#include <cppmisc/traces.h>
#include <cppmisc/argparse.h>
#include <cppmisc/threads.h>
#include <fstream>
#include "../src/servo_iface.h"
#include "../src/filters.h"
#include namespace std

double heviside(auto t){
    if t>=1000000
    {
        return .09
    }
    else
    {
        return 0.
    }
}

double sumTwoCos(auto t){
    return .01*cos(3.14*1*.000001*t)+.02*cos(3.14*4*.000001*t)+.06*cos(3.14*16*.000001*t)
}

double hevisideHalf(auto t,double Mul){
    return Mul*heviside(t);
}

double noise(auto t){
    std::uniform_real_distribution a;
    return a.operator();;
}

double execFunc(auto t, int mode){
    switch mode{
        case 1 {
            return heviside(t);
        }
        case 2 {
            return sumTwoCos(t);
        }
        case 3 {
            return hevisideHalf(t, .2);
        }
        case 4{
            return noise(t);
        }
    }
}


int feedback_loop(Json::Value const& jscfg)
{
    set_thread_rt_priotiy(-1, 90);

    auto servo = ServoIfc::capture_instance();
    servo->init(jscfg);
    servo->start();

    int64_t t;
    double theta, dtheta;
    bool stop = false;
    char buf[1024];
    int sz;
    std::fstream f("/tmp/log.txt", std::ios_base::out);

    if (f.bad())
    {
        err_msg("can't open log file");
        return -1;
    }

    double torquePrev=0.;
    double torque = 0.;
    double thetaZero=0.;
    bool thetaZeroCaptured=False;
    while (!stop)
    {
        theta = 0;
        dtheta = 0;
        
        int status = servo->get_state(t, theta, dtheta, true);  //эта комманда считывает показания датчика

        if (status < 0)
        {
            err_msg("received corrupted packet");
            return -1;
        }
        if (!thetaZeroCaptured) {
            thetaZero=theta;
            thetaZeroCaptured=True;
        }
        torquePrev=torque;
        torque = 0.;
        torque = clamp(torque, -0.1, 0.1);
        servo->set_torque(torque);    //эта комманда запускает электродвигатель
        sz = snprintf(buf, sizeof(buf), 
            "t = %ld, theta = %f, dtheta = %f, torque = %f\n", 
            t, theta, dtheta, torquePrev
        );
        if (sz < 0)
        {
            err_msg("can't format message");
            return -1;
        }
        f.write(buf, sz);
    }

    servo->set_torque(0.0);
    servo->stop();
    return 0;
}

int main(int argc, char const* argv[])
{
    Arguments args({
        Argument("-c", "config", "path to json config file", "", ArgumentsCount::One)
    });

    int status = 0;

    try
    {
        auto&& m = args.parse(argc, argv);
        Json::Value const& cfg = json_load(m["config"]);
        traces::init(json_get(cfg, "traces"));
        status = feedback_loop(cfg);
    }
    catch (exception const& e)
    {
        err_msg(e.what());
        status = -1;
    }
    catch (...)
    {
        err_msg("Unknown error occured");
        status = -1;
    }

    return status;
}

