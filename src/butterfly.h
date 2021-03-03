#pragma once

#include <memory>
#include <stdexcept>
#include <cppmisc/json.h>
#include "filters.h"
#include "servo_iface.h"
#include "cam_iface.h"


struct BflySignals
{
    bool ball_found;
    double t;
    double theta;
    double dtheta;
    double phi;
    double dphi;
    double x;
    double vx;
    double y;
    double vy;
    double torque;
};

class Butterfly
{
private:
    std::shared_ptr<ServoIfc> m_servo;
    std::shared_ptr<Camera> m_camera;

    EulerDiff   m_diff_x;
    EulerDiff   m_diff_y;

    double      m_theta, m_dtheta;
    double      m_x, m_y;
    double      m_vx, m_vy;
    double      m_phi, m_dphi;
    bool        m_stop;
    bool        m_ball_found;

    void measure();
    void get_signals(int64_t const& t, BflySignals& signals);

public:
    typedef std::function<bool(BflySignals&)> callback_t;

    Butterfly();
    ~Butterfly();

    void init(Json::Value const& jscfg);
    void stop();
    void start(callback_t const& cb);    
};
