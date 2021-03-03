#include "butterfly.h"
#include "filters.h"

using namespace std;


Butterfly::Butterfly()
{
    m_theta = 0;
    m_dtheta = 0;
    m_x = 0;
    m_y = 0;
    m_vx = 0;
    m_vy = 0;
    m_phi = 0;
    m_dphi = 0;
    m_stop = false;
    m_ball_found = false;
}

Butterfly::~Butterfly()
{
}

void Butterfly::init(Json::Value const& cfg)
{
    info_msg("initializing hardware..");

    auto const& butcfg = json_get(cfg, "controller");

    m_servo = ServoIfc::capture_instance();
    m_servo->init(cfg);

    m_camera = Camera::capture_instance();
    m_camera->init(cfg);

    info_msg("done");
}

void wait_for_camera_ready()
{

}

void Butterfly::measure()
{
    int64_t t_servo;
    int status = m_servo->get_state(t_servo, m_theta, m_dtheta, true);
    if (status < 0)
        throw_runtime_error("servo disconnected");

    int64_t t_cam;
    status = m_camera->get(t_cam, m_x, m_y);

    switch (status)
    {
    case 1:
    {
        m_vx = m_diff_x.process(t_cam, m_x);
        m_vy = m_diff_y.process(t_cam, m_y);

        double alpha = atan2(m_x, m_y);
        double dalpha = (m_y * m_vx - m_x * m_vy) / (m_x * m_x + m_y * m_y);

        m_phi = m_theta + alpha;
        m_dphi = m_dtheta + dalpha;
        m_ball_found = true;
        break;
    }
    case 0:
    {
        break;
    }
    default:
    {
        if (m_ball_found)
            info_msg("ball was lost");
        m_ball_found = false;
        break;
    }
    }
}

void Butterfly::stop()
{
    m_stop = true;
}

void Butterfly::get_signals(int64_t const& t, BflySignals& signals)
{
    signals.t = t * 1e-6;
    signals.ball_found = m_ball_found;
    signals.theta = m_theta;
    signals.dtheta = m_dtheta;
    signals.phi = m_phi;
    signals.dphi = m_dphi;
    signals.x = m_x;
    signals.vx = m_vx;
    signals.y = m_y;
    signals.vy = m_vy;
    signals.torque = 0;
}

void Butterfly::start(callback_t const& cb)
{
    if (!m_camera || !m_servo)
        throw_runtime_error("Butterfly not initialized yet");

    m_camera->start();
    m_servo->start();

    int status;
    int64_t t, t0;
    t0 = epoch_usec();

    while (!m_stop)
    {
        t = epoch_usec();
        measure();

        BflySignals signals;
        get_signals(t - t0, signals);
        status = cb(signals);
        if (!status)
            m_stop = true;

        m_servo->set_torque(signals.torque);
    }

    m_servo->stop();
    m_camera->stop();

    info_msg("stopped");
}
