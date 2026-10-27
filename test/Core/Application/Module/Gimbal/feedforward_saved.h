#pragma once
/* Paste the complete exported declarations below the corresponding #if.
 * Then set ONLY that axis's PRESENT flag to 1. No fabricated fallback data.
 * Pitch: logical joint N.m, indices 0..41 deg, relative to the SAME mechanical
 * lower stop at first enable. Never paste a table from a different zero,
 * installation, direction, or load. Yaw F is angle-independent logical N.m.
 */
#define PITCH_SAVED_PRESENT 1
#define YAW_SAVED_PRESENT 1
#if PITCH_SAVED_PRESENT
static const float pitch_saved_up_nm[] = {
 -1.709365845e-02f,
 8.681184053e-01f,
 1.076952338e+00f,
 1.107476115e+00f,
 1.164751530e+00f,
 1.135531425e+00f,
 1.112247705e+00f,
 1.192203164e+00f,
 1.159867764e+00f,
 1.111678958e+00f,
 1.189059019e+00f,
 1.146185279e+00f,
 1.147678256e+00f,
 1.200117826e+00f,
 1.159671664e+00f,
 1.157889724e+00f,
 1.203573346e+00f,
 1.138690591e+00f,
 1.142313004e+00f,
 1.177363396e+00f,
 1.134943247e+00f,
 1.136037111e+00f,
 1.156176805e+00f,
 1.150694370e+00f,
 1.194258332e+00f,
 1.302556753e+00f,
 1.198493719e+00f,
 1.166533351e+00f,
 1.228047252e+00f,
 1.097851157e+00f,
 1.050367355e+00f,
 1.086690903e+00f,
 1.046977043e+00f,
 1.058273792e+00f,
 1.147404790e+00f,
 1.158380985e+00f,
 1.150182724e+00f,
 1.243823647e+00f,
 1.231801391e+00f,
 1.247862816e+00f,
 1.340254307e+00f,
 1.249468684e+00f, /* 41 deg */
};
static const float pitch_saved_down_nm[] = {
 -9.523773193e-02f,
 2.331975698e-01f,
 5.592479110e-01f,
 5.763689876e-01f,
 5.591975451e-01f,
 5.801409483e-01f,
 5.763689876e-01f,
 5.725798011e-01f,
 6.131095290e-01f,
 5.813657045e-01f,
 5.664461851e-01f,
 5.940729380e-01f,
 5.838884115e-01f,
 5.841856003e-01f,
 5.741502643e-01f,
 5.326634049e-01f,
 5.641021729e-01f,
 5.923536420e-01f,
 5.426688194e-01f,
 5.410982370e-01f,
 5.836381912e-01f,
 5.289033055e-01f,
 5.565534234e-01f,
 5.362561941e-01f,
 4.568923712e-01f,
 4.126987457e-01f,
 4.894891679e-01f,
 3.927392662e-01f,
 3.690216243e-01f,
 4.264222085e-01f,
 4.670359194e-01f,
 4.664220810e-01f,
 5.009316802e-01f,
 4.597654939e-01f,
 4.099205136e-01f,
 4.207229912e-01f,
 3.890042603e-01f,
 3.451665044e-01f,
 3.548454344e-01f,
 3.377786875e-01f,
 3.437148333e-01f,
 5.966846943e-01f, /* 41 deg */
};
#endif
#if YAW_SAVED_PRESENT
static const float yaw_saved_f_nm = 4.560962522e-01f;
// static const float yaw_saved_f_nm = 4.390500784e-01f;
#endif
