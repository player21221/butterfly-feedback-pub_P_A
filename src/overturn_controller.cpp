#include <cppmisc/traces.h>
#include <cppmisc/argparse.h>
#include <cppmisc/signals.h>
#include "butterfly.h"
#include "overturn_controller.h"

using namespace std;


int launch(Json::Value const& jscfg)
{
    Butterfly bfly;
    bfly.init(jscfg);
    bool stop = false;
    auto stop_handler = [&stop, &bfly]() { stop = true; bfly.stop(); };
    SysSignals::instance().set_sigint_handler(stop_handler);
    SysSignals::instance().set_sigterm_handler(stop_handler);

    // overeturn
    auto f = [](BflySignals& signals) {
        if (signals.t < 0.1)
            return true;

        if (!signals.ball_found)
            return false;

        signals.torque = get_torque(signals.theta, signals.phi, signals.dtheta, signals.dphi);
        signals.torque = clamp(signals.torque, -0.1, 0.1);
    
        info_msg("t=", signals.t, ",torque=", signals.torque, ",theta=", signals.theta, ",phi=", signals.phi, 
             ",dtheta=", signals.dtheta, ",dphi=", signals.dphi, ",x=", signals.x, ",y=", signals.y);

        return true;
    };

    bfly.start(f);
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
        launch(cfg);
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

