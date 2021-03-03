#include <cppmisc/traces.h>
#include <cppmisc/argparse.h>
#include <cppmisc/threads.h>
#include "../src/servo_iface.h"
#include "../src/filters.h"


int feedback_loop(Json::Value const& jscfg)
{
    set_thread_rt_priotiy(-1, 90);

    auto servo = ServoIfc::capture_instance();
    servo->init(jscfg);
    servo->start();

    int64_t t;
    double theta, dtheta;
    Integrator I_theta;
    bool stop = false;

    while (!stop)
    {
        int status = servo->get_state(t, theta, dtheta, true);
        if (status < 0)
        {
            err_msg("received corrupted packed");
            return -1;
        }
        I_theta.update(t, theta);
        double torque = -0.2 * theta - 0.1 * dtheta - 0.1 * I_theta.value();
        torque = clamp(torque, -0.1, 0.1);
        servo->set_torque(torque);
        stop = fabs(theta) < 1e-4 && fabs(dtheta) < 1e-5;
        info_msg("servo state: ", theta, " ", dtheta, " ", stop);
    }

    servo->set_torque(0.0);
    servo->stop();
}

int main(int argc, char const* argv[])
{
    make_arg_list args({
        {{"-c", "--config"}, "config", "path to json config file", "", true}
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

