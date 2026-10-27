#define dm4310_send_mit unused_mock_mit
#include "scan_check.c"
#undef dm4310_send_mit
#include "yaw_load_cal.h"
#include "load_compensation.h"
#define RAD .017453292519943295f
extern LoadCompState g_load_comp[DM_NUM];
static float fc=.4f, inertia=.02f;
static unsigned frame_period=2, wire_frames;
static int frozen, broken, tx_fail;
static float wire_ff;
HAL_StatusTypeDef Can_Send_Std(CAN_HandleTypeDef *h,uint16_t id,uint8_t *d,uint8_t length) {
    (void)h; assert(length==8);
    if(tx_fail && id==DM_YAW_CAN_ID) return 1;
    float p=uint_to_float((d[0]<<8)|d[1],-12.5f,12.5f,16);
    float v=uint_to_float((d[2]<<4)|(d[3]>>4),-30,30,12);
    float k=uint_to_float(((d[3]&15)<<8)|d[4],0,500,12);
    float kd0=uint_to_float((d[5]<<4)|(d[6]>>4),0,5,12);
    float ff=uint_to_float(((d[6]&15)<<8)|d[7],-10,10,12);
    if(id==DM_YAW_CAN_ID) {
        yaw_p_des=p; yaw_v_des=v; yaw_kp=k; yaw_kd=kd0; yaw_ff=wire_ff=ff; ++wire_frames;
    } else { p_des=p; v_des=v; kp=k; kd=kd0; applied_ff=ff; }
    return HAL_OK;
}
static void plant(void) {
    ++tick;
    float t=yaw_enabled?yaw_kp*(yaw_p_des-yaw_position)+yaw_kd*(yaw_v_des-yaw_velocity)+yaw_ff:0;
    t=fmaxf(-10,fminf(10,t));
    float resistance=fabsf(yaw_velocity)<.0001f?fmaxf(-fc,fminf(fc,t)):
        (yaw_velocity>0?fc:-fc);
    float v=yaw_velocity+(t-resistance-.1f*yaw_velocity)/inertia*.001f;
    if(v*yaw_velocity<0 && fabsf(t)<fc) v=0;
    if(frozen || !yaw_enabled) v=0;
    yaw_velocity=v; yaw_position+=v*.001f;
    dm_motor[Pitch].last_rx_ms=tick; ++dm_motor[Pitch].rx_count;
    if(!broken && tick%frame_period==0) {
        dm_motor[Yaw].p_int=yaw_position;
        dm_motor[Yaw].v_int=yaw_velocity;
        dm_motor[Yaw].t_int=t;
        dm_motor[Yaw].last_rx_ms=tick; ++dm_motor[Yaw].rx_count;
    }
    Gimbal_Yaw_Test_Update(tick,GPIO_PIN_RESET);
}
static void run(unsigned ms) { while(ms--) plant(); }
static void init(void) {
    fc=.4f; inertia=.02f; frame_period=2; frozen=broken=tx_fail=0;
    ready(); Gimbal_Yaw_Enable(); run(100);
    assert(g_yaw_test.state==YAW_TEST_ACTIVE);
}
static void calibrate(void) {
    assert(Gimbal_Yaw_Cal_Start());
    uint32_t start=tick;
    while(YawCal_Active() && tick-start<76000) plant();
    printf("cal dir=%.0f state=%u error=%u pairs=%u F=%.5f duration=%lu ms\n",
        (double)GIMBAL_YAW_DIRECTION,g_yaw_cal.state,g_yaw_cal.error,g_yaw_cal.paired,
        (double)g_yaw_cal.f_mean,(unsigned long)(tick-start));
    assert(!YawCal_Active());
}
static void check_output(float speed,float sign) {
    float target=g_yaw_test.measured_relative_rad/RAD+sign*40;
    assert(Gimbal_Yaw_Set_Target_Speed_Deg(target,speed));
    run(1000);
    float expected=GIMBAL_YAW_DIRECTION*sign*fminf(1,YawCal_Friction(10000));
    assert(fabsf(wire_ff-expected)<.012f);
    assert(wire_frames>0);
    printf("MIT decoded speed=%.0f expected=%.5f actual=%.5f Nm\n",(double)speed,(double)expected,(double)wire_ff);
}
int main(void) {
    init(); calibrate();
    assert(g_yaw_cal.state==YCAL_DONE && g_yaw_cal.paired==9);
    assert(fabsf(YawCal_Friction(10000)-.4087f)<.04f);
    assert(!g_load_comp[Yaw].integral_enabled && !g_load_comp[Pitch].integral_enabled);
    check_output(5,1); check_output(20,-1);
    assert(Gimbal_Yaw_Set_Target_Speed_Deg(g_yaw_test.measured_relative_rad/RAD,5));
    run(1500); assert(fabsf(wire_ff)<.01f);
    fc=.6f; calibrate(); assert(g_yaw_cal.state==YCAL_DONE);
    assert(fabsf(YawCal_Friction(0)-.6087f)<.04f);
    float saved=YawCal_Friction(0);
    frozen=1; calibrate();
    assert(g_yaw_cal.state==YCAL_ABORT && g_yaw_cal.error==YCAL_SAMPLES);
    assert(g_yaw_test.state==YAW_TEST_ACTIVE && yaw_enabled && YawCal_Friction(0)==saved);
    char report[320]; assert(Gimbal_Load_Report_Line(report,sizeof(report)));
    assert(strstr(report,"HOLD") && strstr(report,"counts(+/-):") && strstr(report,"old_kept=1"));
    printf("failure report: %s",report);
    Gimbal_Load_Report_Advance(); assert(!Gimbal_Load_Report_Line(report,sizeof(report)));
    run(1000); assert(Gimbal_Load_Report_Line(report,sizeof(report))); Gimbal_Load_Report_Advance();
    run(1000); assert(Gimbal_Load_Report_Line(report,sizeof(report))); Gimbal_Load_Report_Advance();
    run(3000); assert(!Gimbal_Load_Telemetry_Paused());
    frozen=0; check_output(20,1); // Prior enabled FF restored, not merely old F retained.
    assert(Gimbal_Yaw_Cal_Start()); run(500); broken=1; run(60);
    assert(g_yaw_test.state==YAW_TEST_FAULT && !yaw_enabled && YawCal_Friction(0)==saved);
    init(); inertia=.08f; frame_period=17;
    assert(Gimbal_Yaw_Set_Kp(4)); calibrate();
    assert(g_yaw_cal.state==YCAL_DONE && g_yaw_cal.paired>=3);
    assert(fabsf(YawCal_Friction(0)-.4087f)<.09f);
    init(); fc=1.4f; calibrate();
    assert(g_yaw_cal.state==YCAL_DONE && YawCal_Friction(0)>1);
    check_output(20,1); // Model retained, actual MIT output respects existing 1 Nm cap.
    assert(Gimbal_Load_Report_Line(report,sizeof(report))); printf("success report: %s",report);
    init(); assert(Gimbal_Yaw_Cal_Start()); run(200); dm_motor[Yaw].state=8; plant();
    assert(!yaw_enabled && g_yaw_test.state==YAW_TEST_FAULT);
    init(); assert(Gimbal_Yaw_Cal_Start()); run(200); dm_motor[Yaw].t_mos=90; plant();
    assert(!yaw_enabled && g_yaw_test.state==YAW_TEST_FAULT);
    init(); assert(Gimbal_Yaw_Cal_Start()); tx_fail=1; run(10);
    assert(!yaw_enabled && g_yaw_test.state==YAW_TEST_FAULT);
    init(); assert(Gimbal_Yaw_Cal_Start()); run(100); Gimbal_Yaw_Cal_Stop();
    assert(!YawCal_Active() && !yaw_enabled && !Gimbal_Load_Telemetry_Paused());
    puts("PASS complete Yaw: real MIT bytes, plant, lag, sparse feedback, recalibration, retention, safety and reports");
}
