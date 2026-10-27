#include "scan_check.c"
#define RAD .017453292519943295f
static void refresh(float elevation,float roll,float yaw) {
    imu_valid=1; imu_auto=0;
    mock_pose(roll,elevation,yaw,0,0,0);
}
static void hold_steps(unsigned n,float elevation) {
    while(n--) { refresh(elevation,0,0); step(1,0); }
}
static float vertical_after(const float q[4],float d) {
    /* Independently compose q * joint quaternion, then rotate forward X. */
    const float h=d*PITCH_ENCODER_ROTATION_SIGN*.5f,c=cosf(h),s=sinf(h);
    const float w=q[0]*c-q[2]*s, x=q[1]*c-q[3]*s;
    const float y=q[0]*s+q[2]*c, z=q[1]*s+q[3]*c;
    return 2*(x*z-w*y);
}
static void geometry(void) {
    float e,d,g;
    reset();
    for(int yaw=-180;yaw<=180;yaw+=30)
    for(int roll=-60;roll<=60;roll+=15)
    for(int pitch=-40;pitch<=40;pitch+=10) {
        refresh(pitch*RAD,roll*RAD,yaw*RAD);
        const float goal=(pitch+2)*RAD;
        assert(Pitch_Inertial_Geometry(imu_sample.quaternion_wxyz,goal,&e,&d,&g));
        assert(fabsf(e-pitch*RAD)<2e-6f);
        assert(fabsf(vertical_after(imu_sample.quaternion_wxyz,d)-sinf(goal))<2e-6f);
        assert(fabsf(d)<5*RAD);
        const float fd=(asinf(vertical_after(imu_sample.quaternion_wxyz,.001f))-
                        asinf(vertical_after(imu_sample.quaternion_wxyz,-.001f)))/.002f;
        assert(fabsf(fd-g)<.001f);
    }
    refresh(10*RAD,90*RAD,30*RAD);
    assert(!Pitch_Inertial_Geometry(imu_sample.quaternion_wxyz,10*RAD,&e,&d,&g));
    refresh(85*RAD,0,0);
    assert(!Pitch_Inertial_Geometry(imu_sample.quaternion_wxyz,0,&e,&d,&g));
    float bad[4]={0}; assert(!Pitch_Inertial_Geometry(bad,0,&e,&d,&g));
    bad[0]=NAN; assert(!Pitch_Inertial_Geometry(bad,0,&e,&d,&g));
    puts("PASS quaternion geometry: yaw/roll sweep, composed joint rotation, finite-difference sign, singular/invalid rejection");
}
static void integration(void) {
    ready();
    assert(g_pitch_inertial.mode==PITCH_MODE_ENCODER);
    assert(!Gimbal_Pitch_Set_Mode(PITCH_MODE_INERTIAL));
    assert(g_pitch_inertial.reason==PI_IMU);
    position=20*RAD; hold_steps(10,-20*RAD);
    Gimbal_Yaw_Enable(); hold_steps(100,-20*RAD);
    assert(Gimbal_Yaw_Set_Mode(YAW_MODE_INERTIAL));
    const float scale=Pitch_FF_Get_Scale(),k=g_pitch_test.kp,damp=g_pitch_test.kd;
    const float origin=g_pitch_test.origin_rad;
    assert(Gimbal_Pitch_Set_Mode(PITCH_MODE_INERTIAL));
    assert(g_pitch_inertial.mode==PITCH_MODE_INERTIAL);
    assert(g_yaw_inertial.mode==YAW_MODE_INERTIAL);
    assert(g_pitch_test.command_relative_rad==g_pitch_test.measured_relative_rad);
    hold_steps(10,-20*RAD);
    assert(fabsf(g_pitch_test.target_relative_rad-20*RAD)<2e-6f);
    assert(kp==k && kd==damp); /* Encoder MIT PD retained, not zeroed like Yaw. */
    assert(Pitch_FF_Get_Scale()==scale && g_pitch_test.origin_rad==origin);
    assert(!Gimbal_Pitch_Set_Target_Deg(10));
    assert(!Gimbal_Pitch_Move_Relative_Speed_Deg(1,5));
    assert(!Gimbal_Pitch_Repeat_Start(10,30,5));
    assert(!Gimbal_Pitch_Scan_Start());
    assert(!Pitch_Cont_Start() && Pitch_FF_Get_Scale()==scale);
    /* A +2 deg upward base disturbance must command the correcting joint
     * direction. Frozen encoder checks mapping separately from plant dynamics. */
    const float before=g_pitch_test.command_relative_rad;
    hold_steps(5,-18*RAD);
    assert(fabsf(g_pitch_test.target_relative_rad-
        (20+2*PITCH_ENCODER_ROTATION_SIGN)*RAD)<3e-6f);
    assert(fabsf(g_pitch_test.command_relative_rad-before)<=30*RAD*.005f+1e-6f);
    assert(kp==k && kd==damp && g_yaw_inertial.mode==YAW_MODE_INERTIAL);
    assert(Gimbal_Pitch_Set_Mode(PITCH_MODE_ENCODER));
    assert(g_pitch_test.command_relative_rad==g_pitch_test.measured_relative_rad);
    assert(Gimbal_Pitch_Set_Target_Deg(21)); /* Original encoder API restored. */
    assert(Gimbal_Pitch_Set_Mode(PITCH_MODE_INERTIAL));
    Gimbal_Yaw_Test_Stop(); /* Isolate Pitch reference-loss handling from Yaw fault reporting. */
    imu_sample.reference_id++;
    hold_steps(5,-18*RAD);
    assert(g_pitch_inertial.mode==PITCH_MODE_ENCODER && g_pitch_inertial.reason==PI_REFERENCE);
    assert(g_pitch_test.state==YAW_TEST_ACTIVE && enabled && Pitch_FF_Get_Scale()==scale);
    /* Yaw has its own pre-existing reference-change fault. Re-enable independently. */
    Gimbal_Yaw_Test_Stop();
    refresh(-18*RAD,0,0); assert(Gimbal_Pitch_Set_Mode(PITCH_MODE_INERTIAL));
    imu_valid=0; step(1,0);
    for(int i=0;i<5;i++) step(1,0);
    assert(g_pitch_inertial.mode==PITCH_MODE_ENCODER && g_pitch_inertial.reason==PI_IMU);
    assert(g_pitch_test.state==YAW_TEST_ACTIVE && enabled && kp==k && kd==damp);
    assert(g_pitch_test.target_relative_rad==g_pitch_test.measured_relative_rad);
    assert(Pitch_FF_Get_Scale()==scale);
    /* Old but numerically finite samples must not be reused. */
    refresh(-18*RAD,0,0); assert(Gimbal_Pitch_Set_Mode(PITCH_MODE_INERTIAL));
    imu_sample.timestamp_ms=tick-6;
    for(int i=0;i<5;i++) step(1,0);
    assert(g_pitch_inertial.mode==PITCH_MODE_ENCODER && g_pitch_inertial.reason==PI_IMU);
    /* Bounds independent of quaternion zero offset. */
    position=40.9f*RAD; hold_steps(10,0);
    assert(Gimbal_Pitch_Set_Mode(PITCH_MODE_INERTIAL));
    hold_steps(10,3*PITCH_ENCODER_ROTATION_SIGN*RAD);
    assert(g_pitch_inertial.limited && g_pitch_test.target_relative_rad<=41*RAD);
    assert(g_pitch_test.command_relative_rad<=41*RAD+1e-6f);
    assert(Gimbal_Pitch_Set_Mode(PITCH_MODE_ENCODER));
    position=-99.9f*RAD; hold_steps(10,0);
    assert(Gimbal_Pitch_Set_Mode(PITCH_MODE_INERTIAL));
    hold_steps(10,-3*PITCH_ENCODER_ROTATION_SIGN*RAD);
    assert(g_pitch_inertial.limited && g_pitch_test.target_relative_rad>=-100*RAD);
    assert(g_pitch_test.command_relative_rad>=-100*RAD-1e-6f);
    /* Geometry loss falls back while gravity support stays enabled. */
    refresh(0,90*RAD,0);
    for(int i=0;i<5;i++) step(1,0);
    assert(g_pitch_inertial.mode==PITCH_MODE_ENCODER && g_pitch_inertial.reason==PI_GEOMETRY);
    assert(enabled && Pitch_FF_Get_Scale()==scale);
    Gimbal_Pitch_Probe(); char report[320];
    assert(Gimbal_Load_Report_Line(report,sizeof(report)) && strstr(report,"#PP,"));
    printf("status: %s",report); Gimbal_Load_Report_Advance();
    Gimbal_Pitch_Stop(); assert(g_pitch_inertial.mode==PITCH_MODE_ENCODER && !enabled);
    puts("PASS Pitch integration: independent modes, bumpless capture, offset/sign, original PD/FF, speed/bounds, stale/reference/geometry fallback and report");
}
static void dynamic_step(float base_up) {
    refresh(base_up-PITCH_ENCODER_ROTATION_SIGN*position,0,0);
    step(1,1);
}
static void disturbance(void) {
    if(PITCH_ENCODER_ROTATION_SIGN<0) return; /* Reverse mapping covered above. */
    ready();
    assert(Gimbal_Pitch_Set_Target_Speed_Deg(20,5));
    for(int i=0;i<6500;i++) dynamic_step(0);
    assert(Gimbal_Pitch_Set_Mode(PITCH_MODE_INERTIAL));
    for(int i=0;i<2500;i++) dynamic_step(0);
    const float start=position,initial_error=g_pitch_inertial.target_elevation_rad+position;
    for(int leg=0;leg<3;leg++) {
        const float base=(leg==0?2:(leg==1?-2:0))*RAD;
        for(int i=0;i<3500;i++) dynamic_step(base);
        const float error=g_pitch_inertial.target_elevation_rad-(base-position);
        printf("base=%.2f encoder_change=%.3f incremental_space_error=%.3f deg\n",
            (double)(base/RAD),(double)((position-start)/RAD),
            (double)((error-initial_error)/RAD));
        assert(fabsf((position-start)-base)<.5f*RAD);
        assert(g_pitch_inertial.mode==PITCH_MODE_INERTIAL && enabled);
        assert(kp==25 && kd==2 && Pitch_FF_Get_Scale()==1);
    }
    puts("PASS independent legacy plant: +/-2 deg base disturbance, original PD and saved FF retained (not a hardware accuracy prediction)");
}
int main(void) { geometry(); integration(); disturbance(); return 0; }
