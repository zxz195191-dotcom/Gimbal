#include "load_compensation.h"
#include "yaw_load_cal.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void start(void) {
    YawCal_Init();
    assert(!YawCal_Start(0,12));
    g_yaw_cal.armed=1;
    assert(!YawCal_Start(0,NAN));
    assert(YawCal_Start(0,25));
}
static void scan(unsigned positional, float center) {
    YawCal_Init(); g_yaw_cal.armed=1; assert(YawCal_Start(0,center));
    float angle=center;
    for (uint32_t now=5; now<70000 && YawCal_Active(); now+=5) {
        float change=fmaxf(-.025f,fminf(.025f,g_yaw_cal.target_deg-angle));
        angle+=change;
        float speed=change*200;
        float f=positional ? .2f+.012f*(angle-(center-YAW_CAL_NODE_HALF_DEG)):.4f;
        float torque=.1f+(speed>0?f:(speed<0?-f:0));

        YawCalInput in={now,now,now,angle,(now%20)?0:20,torque,angle,1}; // Noisy velocity must not starve sampling.
        (void)YawCal_Update(&in);
    }
    if (!g_yaw_cal.ready) printf("failed scan state=%u error=%u lap=%u\n",g_yaw_cal.state,g_yaw_cal.error,g_yaw_cal.lap);
    assert(g_yaw_cal.ready && g_yaw_cal.state==YCAL_DONE);
    assert(!g_yaw_cal.use_table && g_yaw_cal.lap==1);
    assert(fabsf(g_yaw_cal.b_mean-.1f)<.05f);
    if (!positional) assert(fabsf(g_yaw_cal.f_mean-.4f)<.001f);
    for (unsigned l=0;l<YAW_CAL_LAPS;++l) for (unsigned d=0;d<2;++d) for(unsigned n=0;n<YAW_CAL_NODES;++n)
        assert(g_yaw_cal.counts[l][d][n]==1);
    assert(YawCal_Friction(center-28)==YawCal_Friction(center+28));
    assert(YawCal_Friction(center-5)>=0);
    float retained=YawCal_Friction(center-5);
    g_yaw_cal.armed=1;
    assert(YawCal_Start(70000,18.5f));
    YawCal_Stop(YCAL_CANCEL);
    assert(g_yaw_cal.ready && YawCal_Friction(center-5)==retained);
}
static void partial_scan(float span, unsigned expected, float friction) {
    g_yaw_cal.armed=1; assert(YawCal_Start(0,0));
    float command=0, actual=0;
    for(uint32_t now=5;now<30000 && YawCal_Active();now+=5) {
        command+=fmaxf(-.025f,fminf(.025f,g_yaw_cal.target_deg-command));
        float next=fmaxf(-span,fminf(span,command));
        float torque=next>actual?friction:(next<actual?-friction:0);
        YawCalInput in={now,now,now,next,0,torque,command,1};
        actual=next; (void)YawCal_Update(&in);
    }
    assert(g_yaw_cal.paired==expected);
    assert(g_yaw_cal.state==((expected>=YAW_CAL_MIN_PAIRS && friction>=0)?YCAL_DONE:YCAL_ABORT));
}
static void boundary_samples(void) {
    YawCal_Init(); g_yaw_cal.armed=1; assert(YawCal_Start(0,0));
    YawCalInput in={0,0,0,-25,0,.5f,-25,1};
    for(unsigned i=0;i<103;++i) {
        in.now_ms+=5; in.feedback_ms=in.now_ms; ++in.rx_count; (void)YawCal_Update(&in);
    }
    assert(g_yaw_cal.state==YCAL_FORWARD);
    in.angle_deg=18; in.command_deg=25; in.now_ms+=5; ++in.rx_count;
    (void)YawCal_Update(&in);
    assert(g_yaw_cal.state==YCAL_SETTLE_HIGH && !g_yaw_cal.counts[0][0][8]);
    in.angle_deg=21; in.now_ms+=5; ++in.rx_count; (void)YawCal_Update(&in);
    assert(g_yaw_cal.counts[0][0][8]==1); // Crossed after planned arrival.
    for(unsigned i=0;i<103;++i) { in.now_ms+=5; ++in.rx_count; (void)YawCal_Update(&in); }
    assert(g_yaw_cal.state==YCAL_REVERSE);
    in.angle_deg=23; in.command_deg=24; in.now_ms+=5; ++in.rx_count; (void)YawCal_Update(&in);
    assert(!g_yaw_cal.counts[0][1][8]); // Actual still positive after commanded reversal.
    in.angle_deg=22; in.torque_nm=-.5f; in.now_ms+=5; ++in.rx_count; (void)YawCal_Update(&in);
    in.angle_deg=19; in.now_ms+=5; // Duplicate CAN frame must not count.
    (void)YawCal_Update(&in); assert(!g_yaw_cal.counts[0][1][8]);
    ++in.rx_count; (void)YawCal_Update(&in);
    assert(g_yaw_cal.counts[0][1][8]==1 && fabsf(g_yaw_cal.negative[0][8]+.5f)<1e-6f);
    puts("PASS actual crossing: endpoint dwell, command reversal with old actual direction, duplicate frame");
}
int main(void) {
    LoadCompConfig c={2,.35f,1,1,1,.0174533f,.0523599f,.00174533f,300};
    LoadCompState s={0};
    assert(LoadComp_Update(&s,&c,.2f,0,.02f,0,0,.4f,5,1,1)==.2f);
    s.integral_enabled=1;
    for(unsigned n=0;n<59;++n) LoadComp_Update(&s,&c,0,0,.02f,0,0,.4f,5,1,1);
    assert(s.integral_nm==0);
    for(unsigned n=0;n<10000;++n) LoadComp_Update(&s,&c,0,0,.02f,0,0,.4f,5,1,1);
    assert(fabsf(s.integral_nm-.35f)<1e-5f);
    float old=s.integral_nm;
    LoadComp_Update(&s,&c,0,0,.02f,0,.1f,.4f,5,0,1);
    assert(s.integral_nm<old && old-s.integral_nm<=.00501f);
    LoadComp_Reset(&s);
    for(unsigned n=0;n<1000;++n) LoadComp_Update(&s,&c,0,0,.02f,0,0,10.1f,5,1,1);
    assert(s.integral_nm==0); // Anti-windup against PD+FF saturation.
    for(unsigned n=0;n<1000;++n) LoadComp_Update(&s,&c,0,.4f,0,0,.1f,0,5,0,1);
    old=s.friction_nm;
    LoadComp_Update(&s,&c,0,-.4f,0,0,-.1f,0,5,0,1);
    assert(old-s.friction_nm<=.00501f); // No sign-flip torque step.
    LoadComp_Update(&s,&c,0,0,0,0,0,0,5,1,0);
    assert(s.integral_nm==0 && s.friction_nm==0);
    scan(0,0); scan(1,0); scan(0,25); scan(1,-30);
    start(); YawCalInput in={5,5,1,41,0,0,12,1};
    assert(YawCal_Update(&in)==1 && YawCal_Active());
    start(); in=(YawCalInput){55,0,1,12,0,0,12,0};
    assert(YawCal_Update(&in)==-1 && g_yaw_cal.error==YCAL_FEEDBACK);
    start(); in=(YawCalInput){5,5,1,12,0,0,12,1}; assert(YawCal_Update(&in)==1);
    in=(YawCalInput){10,10,2,13,0,0,13,1};
    assert(YawCal_Update(&in)==1 && YawCal_Active());
    start(); in=(YawCalInput){5,5,1,12,20,1.6f,20,1};
    assert(YawCal_Update(&in)==1 && YawCal_Active());
    start(); in=(YawCalInput){75001,75001,1,12,0,0,12,1};
    assert(YawCal_Update(&in)==-1 && g_yaw_cal.error==YCAL_TIMEOUT);
    // Missing samples must not abort a pass, but must never publish a fabricated table.
    start();
    for(uint32_t now=5;now<10000 && YawCal_Active();now+=5) {
        in=(YawCalInput){now,now,now,25,0,0,g_yaw_cal.target_deg,1};
        (void)YawCal_Update(&in);
    }
    assert(g_yaw_cal.lap==YAW_CAL_LAPS && g_yaw_cal.error==YCAL_SAMPLES && !g_yaw_cal.ready);
    YawCal_Init(); partial_scan(6,3,.4f);
    float retained=YawCal_Friction(0); assert(fabsf(retained-.4f)<.001f);
    partial_scan(3,1,.8f); assert(YawCal_Friction(0)==retained && g_yaw_cal.ready);
    partial_scan(0,0,.8f); assert(YawCal_Friction(0)==retained && g_yaw_cal.ready);
    partial_scan(12,5,.6f); assert(fabsf(YawCal_Friction(0)-.6f)<.001f);
    partial_scan(6,3,-.4f); assert(g_yaw_cal.error==YCAL_SIGN && fabsf(YawCal_Friction(0)-.6f)<.001f);
    puts("PASS adoption: 3 pairs accepted, 1/0 pairs retain old model, 5 pairs update model");
    boundary_samples();
    puts("PASS shared load: defaults, dwell, limits, anti-windup, reversal; actual crossing F, faults/deadline");
}
