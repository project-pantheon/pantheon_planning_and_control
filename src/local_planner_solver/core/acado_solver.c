/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 4 + 3];

acadoWorkspace.state[28] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[29] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[30] = acadoVariables.od[lRun1 * 15];
acadoWorkspace.state[31] = acadoVariables.od[lRun1 * 15 + 1];
acadoWorkspace.state[32] = acadoVariables.od[lRun1 * 15 + 2];
acadoWorkspace.state[33] = acadoVariables.od[lRun1 * 15 + 3];
acadoWorkspace.state[34] = acadoVariables.od[lRun1 * 15 + 4];
acadoWorkspace.state[35] = acadoVariables.od[lRun1 * 15 + 5];
acadoWorkspace.state[36] = acadoVariables.od[lRun1 * 15 + 6];
acadoWorkspace.state[37] = acadoVariables.od[lRun1 * 15 + 7];
acadoWorkspace.state[38] = acadoVariables.od[lRun1 * 15 + 8];
acadoWorkspace.state[39] = acadoVariables.od[lRun1 * 15 + 9];
acadoWorkspace.state[40] = acadoVariables.od[lRun1 * 15 + 10];
acadoWorkspace.state[41] = acadoVariables.od[lRun1 * 15 + 11];
acadoWorkspace.state[42] = acadoVariables.od[lRun1 * 15 + 12];
acadoWorkspace.state[43] = acadoVariables.od[lRun1 * 15 + 13];
acadoWorkspace.state[44] = acadoVariables.od[lRun1 * 15 + 14];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 4] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 4 + 4];
acadoWorkspace.d[lRun1 * 4 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 4 + 5];
acadoWorkspace.d[lRun1 * 4 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 4 + 6];
acadoWorkspace.d[lRun1 * 4 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 4 + 7];

acadoWorkspace.evGx[lRun1 * 16] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 16 + 1] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 16 + 2] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 16 + 3] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 16 + 4] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 16 + 5] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 16 + 6] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 16 + 7] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 16 + 8] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 16 + 9] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 16 + 10] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 16 + 11] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 16 + 12] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 16 + 13] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 16 + 14] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 16 + 15] = acadoWorkspace.state[19];

acadoWorkspace.evGu[lRun1 * 8] = acadoWorkspace.state[20];
acadoWorkspace.evGu[lRun1 * 8 + 1] = acadoWorkspace.state[21];
acadoWorkspace.evGu[lRun1 * 8 + 2] = acadoWorkspace.state[22];
acadoWorkspace.evGu[lRun1 * 8 + 3] = acadoWorkspace.state[23];
acadoWorkspace.evGu[lRun1 * 8 + 4] = acadoWorkspace.state[24];
acadoWorkspace.evGu[lRun1 * 8 + 5] = acadoWorkspace.state[25];
acadoWorkspace.evGu[lRun1 * 8 + 6] = acadoWorkspace.state[26];
acadoWorkspace.evGu[lRun1 * 8 + 7] = acadoWorkspace.state[27];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 49. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (sqrt(((((xd[0]-od[0])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[0])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[1])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[1])+(real_t)(5.0000000000000003e-02))))));
a[1] = (sqrt(((((xd[0]-od[2])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[2])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[3])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[3])+(real_t)(5.0000000000000003e-02))))));
a[2] = (sqrt(((((xd[0]-od[4])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[4])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[5])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[5])+(real_t)(5.0000000000000003e-02))))));
a[3] = (sqrt(((((xd[0]-od[6])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[6])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[7])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[7])+(real_t)(5.0000000000000003e-02))))));
a[4] = (sqrt(((((xd[0]-od[8])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[8])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[9])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[9])+(real_t)(5.0000000000000003e-02))))));
a[5] = (sqrt(((((xd[0]-od[10])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[10])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[11])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[11])+(real_t)(5.0000000000000003e-02))))));
a[6] = (sqrt(((((xd[0]-od[12])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[12])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[13])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[13])+(real_t)(5.0000000000000003e-02))))));
a[7] = (1.0/sqrt(((((xd[0]-od[0])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[0])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[1])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[1])+(real_t)(5.0000000000000003e-02))))));
a[8] = (a[7]*(real_t)(5.0000000000000000e-01));
a[9] = ((((xd[0]-od[0])+(real_t)(5.0000000000000003e-02))+((xd[0]-od[0])+(real_t)(5.0000000000000003e-02)))*a[8]);
a[10] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[11] = (a[10]*a[10]);
a[12] = ((((xd[1]-od[1])+(real_t)(5.0000000000000003e-02))+((xd[1]-od[1])+(real_t)(5.0000000000000003e-02)))*a[8]);
a[13] = (1.0/sqrt(((((xd[0]-od[2])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[2])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[3])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[3])+(real_t)(5.0000000000000003e-02))))));
a[14] = (a[13]*(real_t)(5.0000000000000000e-01));
a[15] = ((((xd[0]-od[2])+(real_t)(5.0000000000000003e-02))+((xd[0]-od[2])+(real_t)(5.0000000000000003e-02)))*a[14]);
a[16] = ((real_t)(1.0000000000000000e+00)/a[1]);
a[17] = (a[16]*a[16]);
a[18] = ((((xd[1]-od[3])+(real_t)(5.0000000000000003e-02))+((xd[1]-od[3])+(real_t)(5.0000000000000003e-02)))*a[14]);
a[19] = (1.0/sqrt(((((xd[0]-od[4])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[4])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[5])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[5])+(real_t)(5.0000000000000003e-02))))));
a[20] = (a[19]*(real_t)(5.0000000000000000e-01));
a[21] = ((((xd[0]-od[4])+(real_t)(5.0000000000000003e-02))+((xd[0]-od[4])+(real_t)(5.0000000000000003e-02)))*a[20]);
a[22] = ((real_t)(1.0000000000000000e+00)/a[2]);
a[23] = (a[22]*a[22]);
a[24] = ((((xd[1]-od[5])+(real_t)(5.0000000000000003e-02))+((xd[1]-od[5])+(real_t)(5.0000000000000003e-02)))*a[20]);
a[25] = (1.0/sqrt(((((xd[0]-od[6])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[6])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[7])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[7])+(real_t)(5.0000000000000003e-02))))));
a[26] = (a[25]*(real_t)(5.0000000000000000e-01));
a[27] = ((((xd[0]-od[6])+(real_t)(5.0000000000000003e-02))+((xd[0]-od[6])+(real_t)(5.0000000000000003e-02)))*a[26]);
a[28] = ((real_t)(1.0000000000000000e+00)/a[3]);
a[29] = (a[28]*a[28]);
a[30] = ((((xd[1]-od[7])+(real_t)(5.0000000000000003e-02))+((xd[1]-od[7])+(real_t)(5.0000000000000003e-02)))*a[26]);
a[31] = (1.0/sqrt(((((xd[0]-od[8])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[8])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[9])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[9])+(real_t)(5.0000000000000003e-02))))));
a[32] = (a[31]*(real_t)(5.0000000000000000e-01));
a[33] = ((((xd[0]-od[8])+(real_t)(5.0000000000000003e-02))+((xd[0]-od[8])+(real_t)(5.0000000000000003e-02)))*a[32]);
a[34] = ((real_t)(1.0000000000000000e+00)/a[4]);
a[35] = (a[34]*a[34]);
a[36] = ((((xd[1]-od[9])+(real_t)(5.0000000000000003e-02))+((xd[1]-od[9])+(real_t)(5.0000000000000003e-02)))*a[32]);
a[37] = (1.0/sqrt(((((xd[0]-od[10])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[10])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[11])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[11])+(real_t)(5.0000000000000003e-02))))));
a[38] = (a[37]*(real_t)(5.0000000000000000e-01));
a[39] = ((((xd[0]-od[10])+(real_t)(5.0000000000000003e-02))+((xd[0]-od[10])+(real_t)(5.0000000000000003e-02)))*a[38]);
a[40] = ((real_t)(1.0000000000000000e+00)/a[5]);
a[41] = (a[40]*a[40]);
a[42] = ((((xd[1]-od[11])+(real_t)(5.0000000000000003e-02))+((xd[1]-od[11])+(real_t)(5.0000000000000003e-02)))*a[38]);
a[43] = (1.0/sqrt(((((xd[0]-od[12])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[12])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[13])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[13])+(real_t)(5.0000000000000003e-02))))));
a[44] = (a[43]*(real_t)(5.0000000000000000e-01));
a[45] = ((((xd[0]-od[12])+(real_t)(5.0000000000000003e-02))+((xd[0]-od[12])+(real_t)(5.0000000000000003e-02)))*a[44]);
a[46] = ((real_t)(1.0000000000000000e+00)/a[6]);
a[47] = (a[46]*a[46]);
a[48] = ((((xd[1]-od[13])+(real_t)(5.0000000000000003e-02))+((xd[1]-od[13])+(real_t)(5.0000000000000003e-02)))*a[44]);

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = ((real_t)(1.0000000000000000e+00)/a[0]);
out[4] = ((real_t)(1.0000000000000000e+00)/a[1]);
out[5] = ((real_t)(1.0000000000000000e+00)/a[2]);
out[6] = ((real_t)(1.0000000000000000e+00)/a[3]);
out[7] = ((real_t)(1.0000000000000000e+00)/a[4]);
out[8] = ((real_t)(1.0000000000000000e+00)/a[5]);
out[9] = ((real_t)(1.0000000000000000e+00)/a[6]);
out[10] = u[1];
out[11] = u[0];
out[12] = (real_t)(1.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(1.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(1.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = ((real_t)(0.0000000000000000e+00)-(a[9]*a[11]));
out[25] = ((real_t)(0.0000000000000000e+00)-(a[12]*a[11]));
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = ((real_t)(0.0000000000000000e+00)-(a[15]*a[17]));
out[29] = ((real_t)(0.0000000000000000e+00)-(a[18]*a[17]));
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = ((real_t)(0.0000000000000000e+00)-(a[21]*a[23]));
out[33] = ((real_t)(0.0000000000000000e+00)-(a[24]*a[23]));
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = ((real_t)(0.0000000000000000e+00)-(a[27]*a[29]));
out[37] = ((real_t)(0.0000000000000000e+00)-(a[30]*a[29]));
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = ((real_t)(0.0000000000000000e+00)-(a[33]*a[35]));
out[41] = ((real_t)(0.0000000000000000e+00)-(a[36]*a[35]));
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = ((real_t)(0.0000000000000000e+00)-(a[39]*a[41]));
out[45] = ((real_t)(0.0000000000000000e+00)-(a[42]*a[41]));
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = ((real_t)(0.0000000000000000e+00)-(a[45]*a[47]));
out[49] = ((real_t)(0.0000000000000000e+00)-(a[48]*a[47]));
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[4]*tmpObjS[12] + tmpFx[8]*tmpObjS[24] + tmpFx[12]*tmpObjS[36] + tmpFx[16]*tmpObjS[48] + tmpFx[20]*tmpObjS[60] + tmpFx[24]*tmpObjS[72] + tmpFx[28]*tmpObjS[84] + tmpFx[32]*tmpObjS[96] + tmpFx[36]*tmpObjS[108] + tmpFx[40]*tmpObjS[120] + tmpFx[44]*tmpObjS[132];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[4]*tmpObjS[13] + tmpFx[8]*tmpObjS[25] + tmpFx[12]*tmpObjS[37] + tmpFx[16]*tmpObjS[49] + tmpFx[20]*tmpObjS[61] + tmpFx[24]*tmpObjS[73] + tmpFx[28]*tmpObjS[85] + tmpFx[32]*tmpObjS[97] + tmpFx[36]*tmpObjS[109] + tmpFx[40]*tmpObjS[121] + tmpFx[44]*tmpObjS[133];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[4]*tmpObjS[14] + tmpFx[8]*tmpObjS[26] + tmpFx[12]*tmpObjS[38] + tmpFx[16]*tmpObjS[50] + tmpFx[20]*tmpObjS[62] + tmpFx[24]*tmpObjS[74] + tmpFx[28]*tmpObjS[86] + tmpFx[32]*tmpObjS[98] + tmpFx[36]*tmpObjS[110] + tmpFx[40]*tmpObjS[122] + tmpFx[44]*tmpObjS[134];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[4]*tmpObjS[15] + tmpFx[8]*tmpObjS[27] + tmpFx[12]*tmpObjS[39] + tmpFx[16]*tmpObjS[51] + tmpFx[20]*tmpObjS[63] + tmpFx[24]*tmpObjS[75] + tmpFx[28]*tmpObjS[87] + tmpFx[32]*tmpObjS[99] + tmpFx[36]*tmpObjS[111] + tmpFx[40]*tmpObjS[123] + tmpFx[44]*tmpObjS[135];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[4]*tmpObjS[16] + tmpFx[8]*tmpObjS[28] + tmpFx[12]*tmpObjS[40] + tmpFx[16]*tmpObjS[52] + tmpFx[20]*tmpObjS[64] + tmpFx[24]*tmpObjS[76] + tmpFx[28]*tmpObjS[88] + tmpFx[32]*tmpObjS[100] + tmpFx[36]*tmpObjS[112] + tmpFx[40]*tmpObjS[124] + tmpFx[44]*tmpObjS[136];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[4]*tmpObjS[17] + tmpFx[8]*tmpObjS[29] + tmpFx[12]*tmpObjS[41] + tmpFx[16]*tmpObjS[53] + tmpFx[20]*tmpObjS[65] + tmpFx[24]*tmpObjS[77] + tmpFx[28]*tmpObjS[89] + tmpFx[32]*tmpObjS[101] + tmpFx[36]*tmpObjS[113] + tmpFx[40]*tmpObjS[125] + tmpFx[44]*tmpObjS[137];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[4]*tmpObjS[18] + tmpFx[8]*tmpObjS[30] + tmpFx[12]*tmpObjS[42] + tmpFx[16]*tmpObjS[54] + tmpFx[20]*tmpObjS[66] + tmpFx[24]*tmpObjS[78] + tmpFx[28]*tmpObjS[90] + tmpFx[32]*tmpObjS[102] + tmpFx[36]*tmpObjS[114] + tmpFx[40]*tmpObjS[126] + tmpFx[44]*tmpObjS[138];
tmpQ2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[4]*tmpObjS[19] + tmpFx[8]*tmpObjS[31] + tmpFx[12]*tmpObjS[43] + tmpFx[16]*tmpObjS[55] + tmpFx[20]*tmpObjS[67] + tmpFx[24]*tmpObjS[79] + tmpFx[28]*tmpObjS[91] + tmpFx[32]*tmpObjS[103] + tmpFx[36]*tmpObjS[115] + tmpFx[40]*tmpObjS[127] + tmpFx[44]*tmpObjS[139];
tmpQ2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[4]*tmpObjS[20] + tmpFx[8]*tmpObjS[32] + tmpFx[12]*tmpObjS[44] + tmpFx[16]*tmpObjS[56] + tmpFx[20]*tmpObjS[68] + tmpFx[24]*tmpObjS[80] + tmpFx[28]*tmpObjS[92] + tmpFx[32]*tmpObjS[104] + tmpFx[36]*tmpObjS[116] + tmpFx[40]*tmpObjS[128] + tmpFx[44]*tmpObjS[140];
tmpQ2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[4]*tmpObjS[21] + tmpFx[8]*tmpObjS[33] + tmpFx[12]*tmpObjS[45] + tmpFx[16]*tmpObjS[57] + tmpFx[20]*tmpObjS[69] + tmpFx[24]*tmpObjS[81] + tmpFx[28]*tmpObjS[93] + tmpFx[32]*tmpObjS[105] + tmpFx[36]*tmpObjS[117] + tmpFx[40]*tmpObjS[129] + tmpFx[44]*tmpObjS[141];
tmpQ2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[4]*tmpObjS[22] + tmpFx[8]*tmpObjS[34] + tmpFx[12]*tmpObjS[46] + tmpFx[16]*tmpObjS[58] + tmpFx[20]*tmpObjS[70] + tmpFx[24]*tmpObjS[82] + tmpFx[28]*tmpObjS[94] + tmpFx[32]*tmpObjS[106] + tmpFx[36]*tmpObjS[118] + tmpFx[40]*tmpObjS[130] + tmpFx[44]*tmpObjS[142];
tmpQ2[11] = + tmpFx[0]*tmpObjS[11] + tmpFx[4]*tmpObjS[23] + tmpFx[8]*tmpObjS[35] + tmpFx[12]*tmpObjS[47] + tmpFx[16]*tmpObjS[59] + tmpFx[20]*tmpObjS[71] + tmpFx[24]*tmpObjS[83] + tmpFx[28]*tmpObjS[95] + tmpFx[32]*tmpObjS[107] + tmpFx[36]*tmpObjS[119] + tmpFx[40]*tmpObjS[131] + tmpFx[44]*tmpObjS[143];
tmpQ2[12] = + tmpFx[1]*tmpObjS[0] + tmpFx[5]*tmpObjS[12] + tmpFx[9]*tmpObjS[24] + tmpFx[13]*tmpObjS[36] + tmpFx[17]*tmpObjS[48] + tmpFx[21]*tmpObjS[60] + tmpFx[25]*tmpObjS[72] + tmpFx[29]*tmpObjS[84] + tmpFx[33]*tmpObjS[96] + tmpFx[37]*tmpObjS[108] + tmpFx[41]*tmpObjS[120] + tmpFx[45]*tmpObjS[132];
tmpQ2[13] = + tmpFx[1]*tmpObjS[1] + tmpFx[5]*tmpObjS[13] + tmpFx[9]*tmpObjS[25] + tmpFx[13]*tmpObjS[37] + tmpFx[17]*tmpObjS[49] + tmpFx[21]*tmpObjS[61] + tmpFx[25]*tmpObjS[73] + tmpFx[29]*tmpObjS[85] + tmpFx[33]*tmpObjS[97] + tmpFx[37]*tmpObjS[109] + tmpFx[41]*tmpObjS[121] + tmpFx[45]*tmpObjS[133];
tmpQ2[14] = + tmpFx[1]*tmpObjS[2] + tmpFx[5]*tmpObjS[14] + tmpFx[9]*tmpObjS[26] + tmpFx[13]*tmpObjS[38] + tmpFx[17]*tmpObjS[50] + tmpFx[21]*tmpObjS[62] + tmpFx[25]*tmpObjS[74] + tmpFx[29]*tmpObjS[86] + tmpFx[33]*tmpObjS[98] + tmpFx[37]*tmpObjS[110] + tmpFx[41]*tmpObjS[122] + tmpFx[45]*tmpObjS[134];
tmpQ2[15] = + tmpFx[1]*tmpObjS[3] + tmpFx[5]*tmpObjS[15] + tmpFx[9]*tmpObjS[27] + tmpFx[13]*tmpObjS[39] + tmpFx[17]*tmpObjS[51] + tmpFx[21]*tmpObjS[63] + tmpFx[25]*tmpObjS[75] + tmpFx[29]*tmpObjS[87] + tmpFx[33]*tmpObjS[99] + tmpFx[37]*tmpObjS[111] + tmpFx[41]*tmpObjS[123] + tmpFx[45]*tmpObjS[135];
tmpQ2[16] = + tmpFx[1]*tmpObjS[4] + tmpFx[5]*tmpObjS[16] + tmpFx[9]*tmpObjS[28] + tmpFx[13]*tmpObjS[40] + tmpFx[17]*tmpObjS[52] + tmpFx[21]*tmpObjS[64] + tmpFx[25]*tmpObjS[76] + tmpFx[29]*tmpObjS[88] + tmpFx[33]*tmpObjS[100] + tmpFx[37]*tmpObjS[112] + tmpFx[41]*tmpObjS[124] + tmpFx[45]*tmpObjS[136];
tmpQ2[17] = + tmpFx[1]*tmpObjS[5] + tmpFx[5]*tmpObjS[17] + tmpFx[9]*tmpObjS[29] + tmpFx[13]*tmpObjS[41] + tmpFx[17]*tmpObjS[53] + tmpFx[21]*tmpObjS[65] + tmpFx[25]*tmpObjS[77] + tmpFx[29]*tmpObjS[89] + tmpFx[33]*tmpObjS[101] + tmpFx[37]*tmpObjS[113] + tmpFx[41]*tmpObjS[125] + tmpFx[45]*tmpObjS[137];
tmpQ2[18] = + tmpFx[1]*tmpObjS[6] + tmpFx[5]*tmpObjS[18] + tmpFx[9]*tmpObjS[30] + tmpFx[13]*tmpObjS[42] + tmpFx[17]*tmpObjS[54] + tmpFx[21]*tmpObjS[66] + tmpFx[25]*tmpObjS[78] + tmpFx[29]*tmpObjS[90] + tmpFx[33]*tmpObjS[102] + tmpFx[37]*tmpObjS[114] + tmpFx[41]*tmpObjS[126] + tmpFx[45]*tmpObjS[138];
tmpQ2[19] = + tmpFx[1]*tmpObjS[7] + tmpFx[5]*tmpObjS[19] + tmpFx[9]*tmpObjS[31] + tmpFx[13]*tmpObjS[43] + tmpFx[17]*tmpObjS[55] + tmpFx[21]*tmpObjS[67] + tmpFx[25]*tmpObjS[79] + tmpFx[29]*tmpObjS[91] + tmpFx[33]*tmpObjS[103] + tmpFx[37]*tmpObjS[115] + tmpFx[41]*tmpObjS[127] + tmpFx[45]*tmpObjS[139];
tmpQ2[20] = + tmpFx[1]*tmpObjS[8] + tmpFx[5]*tmpObjS[20] + tmpFx[9]*tmpObjS[32] + tmpFx[13]*tmpObjS[44] + tmpFx[17]*tmpObjS[56] + tmpFx[21]*tmpObjS[68] + tmpFx[25]*tmpObjS[80] + tmpFx[29]*tmpObjS[92] + tmpFx[33]*tmpObjS[104] + tmpFx[37]*tmpObjS[116] + tmpFx[41]*tmpObjS[128] + tmpFx[45]*tmpObjS[140];
tmpQ2[21] = + tmpFx[1]*tmpObjS[9] + tmpFx[5]*tmpObjS[21] + tmpFx[9]*tmpObjS[33] + tmpFx[13]*tmpObjS[45] + tmpFx[17]*tmpObjS[57] + tmpFx[21]*tmpObjS[69] + tmpFx[25]*tmpObjS[81] + tmpFx[29]*tmpObjS[93] + tmpFx[33]*tmpObjS[105] + tmpFx[37]*tmpObjS[117] + tmpFx[41]*tmpObjS[129] + tmpFx[45]*tmpObjS[141];
tmpQ2[22] = + tmpFx[1]*tmpObjS[10] + tmpFx[5]*tmpObjS[22] + tmpFx[9]*tmpObjS[34] + tmpFx[13]*tmpObjS[46] + tmpFx[17]*tmpObjS[58] + tmpFx[21]*tmpObjS[70] + tmpFx[25]*tmpObjS[82] + tmpFx[29]*tmpObjS[94] + tmpFx[33]*tmpObjS[106] + tmpFx[37]*tmpObjS[118] + tmpFx[41]*tmpObjS[130] + tmpFx[45]*tmpObjS[142];
tmpQ2[23] = + tmpFx[1]*tmpObjS[11] + tmpFx[5]*tmpObjS[23] + tmpFx[9]*tmpObjS[35] + tmpFx[13]*tmpObjS[47] + tmpFx[17]*tmpObjS[59] + tmpFx[21]*tmpObjS[71] + tmpFx[25]*tmpObjS[83] + tmpFx[29]*tmpObjS[95] + tmpFx[33]*tmpObjS[107] + tmpFx[37]*tmpObjS[119] + tmpFx[41]*tmpObjS[131] + tmpFx[45]*tmpObjS[143];
tmpQ2[24] = + tmpFx[2]*tmpObjS[0] + tmpFx[6]*tmpObjS[12] + tmpFx[10]*tmpObjS[24] + tmpFx[14]*tmpObjS[36] + tmpFx[18]*tmpObjS[48] + tmpFx[22]*tmpObjS[60] + tmpFx[26]*tmpObjS[72] + tmpFx[30]*tmpObjS[84] + tmpFx[34]*tmpObjS[96] + tmpFx[38]*tmpObjS[108] + tmpFx[42]*tmpObjS[120] + tmpFx[46]*tmpObjS[132];
tmpQ2[25] = + tmpFx[2]*tmpObjS[1] + tmpFx[6]*tmpObjS[13] + tmpFx[10]*tmpObjS[25] + tmpFx[14]*tmpObjS[37] + tmpFx[18]*tmpObjS[49] + tmpFx[22]*tmpObjS[61] + tmpFx[26]*tmpObjS[73] + tmpFx[30]*tmpObjS[85] + tmpFx[34]*tmpObjS[97] + tmpFx[38]*tmpObjS[109] + tmpFx[42]*tmpObjS[121] + tmpFx[46]*tmpObjS[133];
tmpQ2[26] = + tmpFx[2]*tmpObjS[2] + tmpFx[6]*tmpObjS[14] + tmpFx[10]*tmpObjS[26] + tmpFx[14]*tmpObjS[38] + tmpFx[18]*tmpObjS[50] + tmpFx[22]*tmpObjS[62] + tmpFx[26]*tmpObjS[74] + tmpFx[30]*tmpObjS[86] + tmpFx[34]*tmpObjS[98] + tmpFx[38]*tmpObjS[110] + tmpFx[42]*tmpObjS[122] + tmpFx[46]*tmpObjS[134];
tmpQ2[27] = + tmpFx[2]*tmpObjS[3] + tmpFx[6]*tmpObjS[15] + tmpFx[10]*tmpObjS[27] + tmpFx[14]*tmpObjS[39] + tmpFx[18]*tmpObjS[51] + tmpFx[22]*tmpObjS[63] + tmpFx[26]*tmpObjS[75] + tmpFx[30]*tmpObjS[87] + tmpFx[34]*tmpObjS[99] + tmpFx[38]*tmpObjS[111] + tmpFx[42]*tmpObjS[123] + tmpFx[46]*tmpObjS[135];
tmpQ2[28] = + tmpFx[2]*tmpObjS[4] + tmpFx[6]*tmpObjS[16] + tmpFx[10]*tmpObjS[28] + tmpFx[14]*tmpObjS[40] + tmpFx[18]*tmpObjS[52] + tmpFx[22]*tmpObjS[64] + tmpFx[26]*tmpObjS[76] + tmpFx[30]*tmpObjS[88] + tmpFx[34]*tmpObjS[100] + tmpFx[38]*tmpObjS[112] + tmpFx[42]*tmpObjS[124] + tmpFx[46]*tmpObjS[136];
tmpQ2[29] = + tmpFx[2]*tmpObjS[5] + tmpFx[6]*tmpObjS[17] + tmpFx[10]*tmpObjS[29] + tmpFx[14]*tmpObjS[41] + tmpFx[18]*tmpObjS[53] + tmpFx[22]*tmpObjS[65] + tmpFx[26]*tmpObjS[77] + tmpFx[30]*tmpObjS[89] + tmpFx[34]*tmpObjS[101] + tmpFx[38]*tmpObjS[113] + tmpFx[42]*tmpObjS[125] + tmpFx[46]*tmpObjS[137];
tmpQ2[30] = + tmpFx[2]*tmpObjS[6] + tmpFx[6]*tmpObjS[18] + tmpFx[10]*tmpObjS[30] + tmpFx[14]*tmpObjS[42] + tmpFx[18]*tmpObjS[54] + tmpFx[22]*tmpObjS[66] + tmpFx[26]*tmpObjS[78] + tmpFx[30]*tmpObjS[90] + tmpFx[34]*tmpObjS[102] + tmpFx[38]*tmpObjS[114] + tmpFx[42]*tmpObjS[126] + tmpFx[46]*tmpObjS[138];
tmpQ2[31] = + tmpFx[2]*tmpObjS[7] + tmpFx[6]*tmpObjS[19] + tmpFx[10]*tmpObjS[31] + tmpFx[14]*tmpObjS[43] + tmpFx[18]*tmpObjS[55] + tmpFx[22]*tmpObjS[67] + tmpFx[26]*tmpObjS[79] + tmpFx[30]*tmpObjS[91] + tmpFx[34]*tmpObjS[103] + tmpFx[38]*tmpObjS[115] + tmpFx[42]*tmpObjS[127] + tmpFx[46]*tmpObjS[139];
tmpQ2[32] = + tmpFx[2]*tmpObjS[8] + tmpFx[6]*tmpObjS[20] + tmpFx[10]*tmpObjS[32] + tmpFx[14]*tmpObjS[44] + tmpFx[18]*tmpObjS[56] + tmpFx[22]*tmpObjS[68] + tmpFx[26]*tmpObjS[80] + tmpFx[30]*tmpObjS[92] + tmpFx[34]*tmpObjS[104] + tmpFx[38]*tmpObjS[116] + tmpFx[42]*tmpObjS[128] + tmpFx[46]*tmpObjS[140];
tmpQ2[33] = + tmpFx[2]*tmpObjS[9] + tmpFx[6]*tmpObjS[21] + tmpFx[10]*tmpObjS[33] + tmpFx[14]*tmpObjS[45] + tmpFx[18]*tmpObjS[57] + tmpFx[22]*tmpObjS[69] + tmpFx[26]*tmpObjS[81] + tmpFx[30]*tmpObjS[93] + tmpFx[34]*tmpObjS[105] + tmpFx[38]*tmpObjS[117] + tmpFx[42]*tmpObjS[129] + tmpFx[46]*tmpObjS[141];
tmpQ2[34] = + tmpFx[2]*tmpObjS[10] + tmpFx[6]*tmpObjS[22] + tmpFx[10]*tmpObjS[34] + tmpFx[14]*tmpObjS[46] + tmpFx[18]*tmpObjS[58] + tmpFx[22]*tmpObjS[70] + tmpFx[26]*tmpObjS[82] + tmpFx[30]*tmpObjS[94] + tmpFx[34]*tmpObjS[106] + tmpFx[38]*tmpObjS[118] + tmpFx[42]*tmpObjS[130] + tmpFx[46]*tmpObjS[142];
tmpQ2[35] = + tmpFx[2]*tmpObjS[11] + tmpFx[6]*tmpObjS[23] + tmpFx[10]*tmpObjS[35] + tmpFx[14]*tmpObjS[47] + tmpFx[18]*tmpObjS[59] + tmpFx[22]*tmpObjS[71] + tmpFx[26]*tmpObjS[83] + tmpFx[30]*tmpObjS[95] + tmpFx[34]*tmpObjS[107] + tmpFx[38]*tmpObjS[119] + tmpFx[42]*tmpObjS[131] + tmpFx[46]*tmpObjS[143];
tmpQ2[36] = + tmpFx[3]*tmpObjS[0] + tmpFx[7]*tmpObjS[12] + tmpFx[11]*tmpObjS[24] + tmpFx[15]*tmpObjS[36] + tmpFx[19]*tmpObjS[48] + tmpFx[23]*tmpObjS[60] + tmpFx[27]*tmpObjS[72] + tmpFx[31]*tmpObjS[84] + tmpFx[35]*tmpObjS[96] + tmpFx[39]*tmpObjS[108] + tmpFx[43]*tmpObjS[120] + tmpFx[47]*tmpObjS[132];
tmpQ2[37] = + tmpFx[3]*tmpObjS[1] + tmpFx[7]*tmpObjS[13] + tmpFx[11]*tmpObjS[25] + tmpFx[15]*tmpObjS[37] + tmpFx[19]*tmpObjS[49] + tmpFx[23]*tmpObjS[61] + tmpFx[27]*tmpObjS[73] + tmpFx[31]*tmpObjS[85] + tmpFx[35]*tmpObjS[97] + tmpFx[39]*tmpObjS[109] + tmpFx[43]*tmpObjS[121] + tmpFx[47]*tmpObjS[133];
tmpQ2[38] = + tmpFx[3]*tmpObjS[2] + tmpFx[7]*tmpObjS[14] + tmpFx[11]*tmpObjS[26] + tmpFx[15]*tmpObjS[38] + tmpFx[19]*tmpObjS[50] + tmpFx[23]*tmpObjS[62] + tmpFx[27]*tmpObjS[74] + tmpFx[31]*tmpObjS[86] + tmpFx[35]*tmpObjS[98] + tmpFx[39]*tmpObjS[110] + tmpFx[43]*tmpObjS[122] + tmpFx[47]*tmpObjS[134];
tmpQ2[39] = + tmpFx[3]*tmpObjS[3] + tmpFx[7]*tmpObjS[15] + tmpFx[11]*tmpObjS[27] + tmpFx[15]*tmpObjS[39] + tmpFx[19]*tmpObjS[51] + tmpFx[23]*tmpObjS[63] + tmpFx[27]*tmpObjS[75] + tmpFx[31]*tmpObjS[87] + tmpFx[35]*tmpObjS[99] + tmpFx[39]*tmpObjS[111] + tmpFx[43]*tmpObjS[123] + tmpFx[47]*tmpObjS[135];
tmpQ2[40] = + tmpFx[3]*tmpObjS[4] + tmpFx[7]*tmpObjS[16] + tmpFx[11]*tmpObjS[28] + tmpFx[15]*tmpObjS[40] + tmpFx[19]*tmpObjS[52] + tmpFx[23]*tmpObjS[64] + tmpFx[27]*tmpObjS[76] + tmpFx[31]*tmpObjS[88] + tmpFx[35]*tmpObjS[100] + tmpFx[39]*tmpObjS[112] + tmpFx[43]*tmpObjS[124] + tmpFx[47]*tmpObjS[136];
tmpQ2[41] = + tmpFx[3]*tmpObjS[5] + tmpFx[7]*tmpObjS[17] + tmpFx[11]*tmpObjS[29] + tmpFx[15]*tmpObjS[41] + tmpFx[19]*tmpObjS[53] + tmpFx[23]*tmpObjS[65] + tmpFx[27]*tmpObjS[77] + tmpFx[31]*tmpObjS[89] + tmpFx[35]*tmpObjS[101] + tmpFx[39]*tmpObjS[113] + tmpFx[43]*tmpObjS[125] + tmpFx[47]*tmpObjS[137];
tmpQ2[42] = + tmpFx[3]*tmpObjS[6] + tmpFx[7]*tmpObjS[18] + tmpFx[11]*tmpObjS[30] + tmpFx[15]*tmpObjS[42] + tmpFx[19]*tmpObjS[54] + tmpFx[23]*tmpObjS[66] + tmpFx[27]*tmpObjS[78] + tmpFx[31]*tmpObjS[90] + tmpFx[35]*tmpObjS[102] + tmpFx[39]*tmpObjS[114] + tmpFx[43]*tmpObjS[126] + tmpFx[47]*tmpObjS[138];
tmpQ2[43] = + tmpFx[3]*tmpObjS[7] + tmpFx[7]*tmpObjS[19] + tmpFx[11]*tmpObjS[31] + tmpFx[15]*tmpObjS[43] + tmpFx[19]*tmpObjS[55] + tmpFx[23]*tmpObjS[67] + tmpFx[27]*tmpObjS[79] + tmpFx[31]*tmpObjS[91] + tmpFx[35]*tmpObjS[103] + tmpFx[39]*tmpObjS[115] + tmpFx[43]*tmpObjS[127] + tmpFx[47]*tmpObjS[139];
tmpQ2[44] = + tmpFx[3]*tmpObjS[8] + tmpFx[7]*tmpObjS[20] + tmpFx[11]*tmpObjS[32] + tmpFx[15]*tmpObjS[44] + tmpFx[19]*tmpObjS[56] + tmpFx[23]*tmpObjS[68] + tmpFx[27]*tmpObjS[80] + tmpFx[31]*tmpObjS[92] + tmpFx[35]*tmpObjS[104] + tmpFx[39]*tmpObjS[116] + tmpFx[43]*tmpObjS[128] + tmpFx[47]*tmpObjS[140];
tmpQ2[45] = + tmpFx[3]*tmpObjS[9] + tmpFx[7]*tmpObjS[21] + tmpFx[11]*tmpObjS[33] + tmpFx[15]*tmpObjS[45] + tmpFx[19]*tmpObjS[57] + tmpFx[23]*tmpObjS[69] + tmpFx[27]*tmpObjS[81] + tmpFx[31]*tmpObjS[93] + tmpFx[35]*tmpObjS[105] + tmpFx[39]*tmpObjS[117] + tmpFx[43]*tmpObjS[129] + tmpFx[47]*tmpObjS[141];
tmpQ2[46] = + tmpFx[3]*tmpObjS[10] + tmpFx[7]*tmpObjS[22] + tmpFx[11]*tmpObjS[34] + tmpFx[15]*tmpObjS[46] + tmpFx[19]*tmpObjS[58] + tmpFx[23]*tmpObjS[70] + tmpFx[27]*tmpObjS[82] + tmpFx[31]*tmpObjS[94] + tmpFx[35]*tmpObjS[106] + tmpFx[39]*tmpObjS[118] + tmpFx[43]*tmpObjS[130] + tmpFx[47]*tmpObjS[142];
tmpQ2[47] = + tmpFx[3]*tmpObjS[11] + tmpFx[7]*tmpObjS[23] + tmpFx[11]*tmpObjS[35] + tmpFx[15]*tmpObjS[47] + tmpFx[19]*tmpObjS[59] + tmpFx[23]*tmpObjS[71] + tmpFx[27]*tmpObjS[83] + tmpFx[31]*tmpObjS[95] + tmpFx[35]*tmpObjS[107] + tmpFx[39]*tmpObjS[119] + tmpFx[43]*tmpObjS[131] + tmpFx[47]*tmpObjS[143];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[4] + tmpQ2[2]*tmpFx[8] + tmpQ2[3]*tmpFx[12] + tmpQ2[4]*tmpFx[16] + tmpQ2[5]*tmpFx[20] + tmpQ2[6]*tmpFx[24] + tmpQ2[7]*tmpFx[28] + tmpQ2[8]*tmpFx[32] + tmpQ2[9]*tmpFx[36] + tmpQ2[10]*tmpFx[40] + tmpQ2[11]*tmpFx[44];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[5] + tmpQ2[2]*tmpFx[9] + tmpQ2[3]*tmpFx[13] + tmpQ2[4]*tmpFx[17] + tmpQ2[5]*tmpFx[21] + tmpQ2[6]*tmpFx[25] + tmpQ2[7]*tmpFx[29] + tmpQ2[8]*tmpFx[33] + tmpQ2[9]*tmpFx[37] + tmpQ2[10]*tmpFx[41] + tmpQ2[11]*tmpFx[45];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[6] + tmpQ2[2]*tmpFx[10] + tmpQ2[3]*tmpFx[14] + tmpQ2[4]*tmpFx[18] + tmpQ2[5]*tmpFx[22] + tmpQ2[6]*tmpFx[26] + tmpQ2[7]*tmpFx[30] + tmpQ2[8]*tmpFx[34] + tmpQ2[9]*tmpFx[38] + tmpQ2[10]*tmpFx[42] + tmpQ2[11]*tmpFx[46];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[7] + tmpQ2[2]*tmpFx[11] + tmpQ2[3]*tmpFx[15] + tmpQ2[4]*tmpFx[19] + tmpQ2[5]*tmpFx[23] + tmpQ2[6]*tmpFx[27] + tmpQ2[7]*tmpFx[31] + tmpQ2[8]*tmpFx[35] + tmpQ2[9]*tmpFx[39] + tmpQ2[10]*tmpFx[43] + tmpQ2[11]*tmpFx[47];
tmpQ1[4] = + tmpQ2[12]*tmpFx[0] + tmpQ2[13]*tmpFx[4] + tmpQ2[14]*tmpFx[8] + tmpQ2[15]*tmpFx[12] + tmpQ2[16]*tmpFx[16] + tmpQ2[17]*tmpFx[20] + tmpQ2[18]*tmpFx[24] + tmpQ2[19]*tmpFx[28] + tmpQ2[20]*tmpFx[32] + tmpQ2[21]*tmpFx[36] + tmpQ2[22]*tmpFx[40] + tmpQ2[23]*tmpFx[44];
tmpQ1[5] = + tmpQ2[12]*tmpFx[1] + tmpQ2[13]*tmpFx[5] + tmpQ2[14]*tmpFx[9] + tmpQ2[15]*tmpFx[13] + tmpQ2[16]*tmpFx[17] + tmpQ2[17]*tmpFx[21] + tmpQ2[18]*tmpFx[25] + tmpQ2[19]*tmpFx[29] + tmpQ2[20]*tmpFx[33] + tmpQ2[21]*tmpFx[37] + tmpQ2[22]*tmpFx[41] + tmpQ2[23]*tmpFx[45];
tmpQ1[6] = + tmpQ2[12]*tmpFx[2] + tmpQ2[13]*tmpFx[6] + tmpQ2[14]*tmpFx[10] + tmpQ2[15]*tmpFx[14] + tmpQ2[16]*tmpFx[18] + tmpQ2[17]*tmpFx[22] + tmpQ2[18]*tmpFx[26] + tmpQ2[19]*tmpFx[30] + tmpQ2[20]*tmpFx[34] + tmpQ2[21]*tmpFx[38] + tmpQ2[22]*tmpFx[42] + tmpQ2[23]*tmpFx[46];
tmpQ1[7] = + tmpQ2[12]*tmpFx[3] + tmpQ2[13]*tmpFx[7] + tmpQ2[14]*tmpFx[11] + tmpQ2[15]*tmpFx[15] + tmpQ2[16]*tmpFx[19] + tmpQ2[17]*tmpFx[23] + tmpQ2[18]*tmpFx[27] + tmpQ2[19]*tmpFx[31] + tmpQ2[20]*tmpFx[35] + tmpQ2[21]*tmpFx[39] + tmpQ2[22]*tmpFx[43] + tmpQ2[23]*tmpFx[47];
tmpQ1[8] = + tmpQ2[24]*tmpFx[0] + tmpQ2[25]*tmpFx[4] + tmpQ2[26]*tmpFx[8] + tmpQ2[27]*tmpFx[12] + tmpQ2[28]*tmpFx[16] + tmpQ2[29]*tmpFx[20] + tmpQ2[30]*tmpFx[24] + tmpQ2[31]*tmpFx[28] + tmpQ2[32]*tmpFx[32] + tmpQ2[33]*tmpFx[36] + tmpQ2[34]*tmpFx[40] + tmpQ2[35]*tmpFx[44];
tmpQ1[9] = + tmpQ2[24]*tmpFx[1] + tmpQ2[25]*tmpFx[5] + tmpQ2[26]*tmpFx[9] + tmpQ2[27]*tmpFx[13] + tmpQ2[28]*tmpFx[17] + tmpQ2[29]*tmpFx[21] + tmpQ2[30]*tmpFx[25] + tmpQ2[31]*tmpFx[29] + tmpQ2[32]*tmpFx[33] + tmpQ2[33]*tmpFx[37] + tmpQ2[34]*tmpFx[41] + tmpQ2[35]*tmpFx[45];
tmpQ1[10] = + tmpQ2[24]*tmpFx[2] + tmpQ2[25]*tmpFx[6] + tmpQ2[26]*tmpFx[10] + tmpQ2[27]*tmpFx[14] + tmpQ2[28]*tmpFx[18] + tmpQ2[29]*tmpFx[22] + tmpQ2[30]*tmpFx[26] + tmpQ2[31]*tmpFx[30] + tmpQ2[32]*tmpFx[34] + tmpQ2[33]*tmpFx[38] + tmpQ2[34]*tmpFx[42] + tmpQ2[35]*tmpFx[46];
tmpQ1[11] = + tmpQ2[24]*tmpFx[3] + tmpQ2[25]*tmpFx[7] + tmpQ2[26]*tmpFx[11] + tmpQ2[27]*tmpFx[15] + tmpQ2[28]*tmpFx[19] + tmpQ2[29]*tmpFx[23] + tmpQ2[30]*tmpFx[27] + tmpQ2[31]*tmpFx[31] + tmpQ2[32]*tmpFx[35] + tmpQ2[33]*tmpFx[39] + tmpQ2[34]*tmpFx[43] + tmpQ2[35]*tmpFx[47];
tmpQ1[12] = + tmpQ2[36]*tmpFx[0] + tmpQ2[37]*tmpFx[4] + tmpQ2[38]*tmpFx[8] + tmpQ2[39]*tmpFx[12] + tmpQ2[40]*tmpFx[16] + tmpQ2[41]*tmpFx[20] + tmpQ2[42]*tmpFx[24] + tmpQ2[43]*tmpFx[28] + tmpQ2[44]*tmpFx[32] + tmpQ2[45]*tmpFx[36] + tmpQ2[46]*tmpFx[40] + tmpQ2[47]*tmpFx[44];
tmpQ1[13] = + tmpQ2[36]*tmpFx[1] + tmpQ2[37]*tmpFx[5] + tmpQ2[38]*tmpFx[9] + tmpQ2[39]*tmpFx[13] + tmpQ2[40]*tmpFx[17] + tmpQ2[41]*tmpFx[21] + tmpQ2[42]*tmpFx[25] + tmpQ2[43]*tmpFx[29] + tmpQ2[44]*tmpFx[33] + tmpQ2[45]*tmpFx[37] + tmpQ2[46]*tmpFx[41] + tmpQ2[47]*tmpFx[45];
tmpQ1[14] = + tmpQ2[36]*tmpFx[2] + tmpQ2[37]*tmpFx[6] + tmpQ2[38]*tmpFx[10] + tmpQ2[39]*tmpFx[14] + tmpQ2[40]*tmpFx[18] + tmpQ2[41]*tmpFx[22] + tmpQ2[42]*tmpFx[26] + tmpQ2[43]*tmpFx[30] + tmpQ2[44]*tmpFx[34] + tmpQ2[45]*tmpFx[38] + tmpQ2[46]*tmpFx[42] + tmpQ2[47]*tmpFx[46];
tmpQ1[15] = + tmpQ2[36]*tmpFx[3] + tmpQ2[37]*tmpFx[7] + tmpQ2[38]*tmpFx[11] + tmpQ2[39]*tmpFx[15] + tmpQ2[40]*tmpFx[19] + tmpQ2[41]*tmpFx[23] + tmpQ2[42]*tmpFx[27] + tmpQ2[43]*tmpFx[31] + tmpQ2[44]*tmpFx[35] + tmpQ2[45]*tmpFx[39] + tmpQ2[46]*tmpFx[43] + tmpQ2[47]*tmpFx[47];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[132];
tmpR2[1] = +tmpObjS[133];
tmpR2[2] = +tmpObjS[134];
tmpR2[3] = +tmpObjS[135];
tmpR2[4] = +tmpObjS[136];
tmpR2[5] = +tmpObjS[137];
tmpR2[6] = +tmpObjS[138];
tmpR2[7] = +tmpObjS[139];
tmpR2[8] = +tmpObjS[140];
tmpR2[9] = +tmpObjS[141];
tmpR2[10] = +tmpObjS[142];
tmpR2[11] = +tmpObjS[143];
tmpR2[12] = +tmpObjS[120];
tmpR2[13] = +tmpObjS[121];
tmpR2[14] = +tmpObjS[122];
tmpR2[15] = +tmpObjS[123];
tmpR2[16] = +tmpObjS[124];
tmpR2[17] = +tmpObjS[125];
tmpR2[18] = +tmpObjS[126];
tmpR2[19] = +tmpObjS[127];
tmpR2[20] = +tmpObjS[128];
tmpR2[21] = +tmpObjS[129];
tmpR2[22] = +tmpObjS[130];
tmpR2[23] = +tmpObjS[131];
tmpR1[0] = + tmpR2[11];
tmpR1[1] = + tmpR2[10];
tmpR1[2] = + tmpR2[23];
tmpR1[3] = + tmpR2[22];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = 0.0;
;
tmpQN2[10] = 0.0;
;
tmpQN2[11] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = 0.0;
;
tmpQN1[4] = + tmpQN2[3];
tmpQN1[5] = + tmpQN2[4];
tmpQN1[6] = + tmpQN2[5];
tmpQN1[7] = 0.0;
;
tmpQN1[8] = + tmpQN2[6];
tmpQN1[9] = + tmpQN2[7];
tmpQN1[10] = + tmpQN2[8];
tmpQN1[11] = 0.0;
;
tmpQN1[12] = + tmpQN2[9];
tmpQN1[13] = + tmpQN2[10];
tmpQN1[14] = + tmpQN2[11];
tmpQN1[15] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 15];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 15 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 15 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 15 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 15 + 4];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 15 + 5];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 15 + 6];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 15 + 7];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 15 + 8];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 15 + 9];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 15 + 10];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 15 + 11];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 15 + 12];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 15 + 13];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 15 + 14];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 12] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 12 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 12 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 12 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 12 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 12 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 12 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 12 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 12 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 12 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 12 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 12 + 11] = acadoWorkspace.objValueOut[11];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 12 ]), acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 16 ]), &(acadoWorkspace.Q2[ runObj * 48 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 24 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.od[450];
acadoWorkspace.objValueIn[5] = acadoVariables.od[451];
acadoWorkspace.objValueIn[6] = acadoVariables.od[452];
acadoWorkspace.objValueIn[7] = acadoVariables.od[453];
acadoWorkspace.objValueIn[8] = acadoVariables.od[454];
acadoWorkspace.objValueIn[9] = acadoVariables.od[455];
acadoWorkspace.objValueIn[10] = acadoVariables.od[456];
acadoWorkspace.objValueIn[11] = acadoVariables.od[457];
acadoWorkspace.objValueIn[12] = acadoVariables.od[458];
acadoWorkspace.objValueIn[13] = acadoVariables.od[459];
acadoWorkspace.objValueIn[14] = acadoVariables.od[460];
acadoWorkspace.objValueIn[15] = acadoVariables.od[461];
acadoWorkspace.objValueIn[16] = acadoVariables.od[462];
acadoWorkspace.objValueIn[17] = acadoVariables.od[463];
acadoWorkspace.objValueIn[18] = acadoVariables.od[464];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] += + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[8] + Gx1[3]*Gx2[12];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[9] + Gx1[3]*Gx2[13];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[14];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[15];
Gx3[4] = + Gx1[4]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[12];
Gx3[5] = + Gx1[4]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[13];
Gx3[6] = + Gx1[4]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[6]*Gx2[10] + Gx1[7]*Gx2[14];
Gx3[7] = + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[6]*Gx2[11] + Gx1[7]*Gx2[15];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[12];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[13];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[14];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[15];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[4] + Gx1[14]*Gx2[8] + Gx1[15]*Gx2[12];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[5] + Gx1[14]*Gx2[9] + Gx1[15]*Gx2[13];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[10] + Gx1[15]*Gx2[14];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[11] + Gx1[15]*Gx2[15];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7];
Gu2[2] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[6]*Gu1[4] + Gx1[7]*Gu1[6];
Gu2[3] = + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[6]*Gu1[5] + Gx1[7]*Gu1[7];
Gu2[4] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[6];
Gu2[5] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[7];
Gu2[6] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[6];
Gu2[7] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[7];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = R11[0] + (real_t)1.0000000000000000e-08;
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = R11[2];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = R11[3] + (real_t)1.0000000000000000e-08;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] = + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] = + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] = + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3];
dNew[1] = + acadoWorkspace.QN1[4]*dOld[0] + acadoWorkspace.QN1[5]*dOld[1] + acadoWorkspace.QN1[6]*dOld[2] + acadoWorkspace.QN1[7]*dOld[3];
dNew[2] = + acadoWorkspace.QN1[8]*dOld[0] + acadoWorkspace.QN1[9]*dOld[1] + acadoWorkspace.QN1[10]*dOld[2] + acadoWorkspace.QN1[11]*dOld[3];
dNew[3] = + acadoWorkspace.QN1[12]*dOld[0] + acadoWorkspace.QN1[13]*dOld[1] + acadoWorkspace.QN1[14]*dOld[2] + acadoWorkspace.QN1[15]*dOld[3];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11];
RDy1[1] = + R2[12]*Dy1[0] + R2[13]*Dy1[1] + R2[14]*Dy1[2] + R2[15]*Dy1[3] + R2[16]*Dy1[4] + R2[17]*Dy1[5] + R2[18]*Dy1[6] + R2[19]*Dy1[7] + R2[20]*Dy1[8] + R2[21]*Dy1[9] + R2[22]*Dy1[10] + R2[23]*Dy1[11];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11];
QDy1[1] = + Q2[12]*Dy1[0] + Q2[13]*Dy1[1] + Q2[14]*Dy1[2] + Q2[15]*Dy1[3] + Q2[16]*Dy1[4] + Q2[17]*Dy1[5] + Q2[18]*Dy1[6] + Q2[19]*Dy1[7] + Q2[20]*Dy1[8] + Q2[21]*Dy1[9] + Q2[22]*Dy1[10] + Q2[23]*Dy1[11];
QDy1[2] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5] + Q2[30]*Dy1[6] + Q2[31]*Dy1[7] + Q2[32]*Dy1[8] + Q2[33]*Dy1[9] + Q2[34]*Dy1[10] + Q2[35]*Dy1[11];
QDy1[3] = + Q2[36]*Dy1[0] + Q2[37]*Dy1[1] + Q2[38]*Dy1[2] + Q2[39]*Dy1[3] + Q2[40]*Dy1[4] + Q2[41]*Dy1[5] + Q2[42]*Dy1[6] + Q2[43]*Dy1[7] + Q2[44]*Dy1[8] + Q2[45]*Dy1[9] + Q2[46]*Dy1[10] + Q2[47]*Dy1[11];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[4] + E1[4]*Gx1[8] + E1[6]*Gx1[12];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[5] + E1[4]*Gx1[9] + E1[6]*Gx1[13];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[6] + E1[4]*Gx1[10] + E1[6]*Gx1[14];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[7] + E1[4]*Gx1[11] + E1[6]*Gx1[15];
H101[4] += + E1[1]*Gx1[0] + E1[3]*Gx1[4] + E1[5]*Gx1[8] + E1[7]*Gx1[12];
H101[5] += + E1[1]*Gx1[1] + E1[3]*Gx1[5] + E1[5]*Gx1[9] + E1[7]*Gx1[13];
H101[6] += + E1[1]*Gx1[2] + E1[3]*Gx1[6] + E1[5]*Gx1[10] + E1[7]*Gx1[14];
H101[7] += + E1[1]*Gx1[3] + E1[3]*Gx1[7] + E1[5]*Gx1[11] + E1[7]*Gx1[15];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 8; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[4] + Hx[2]*Gx[8] + Hx[3]*Gx[12];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[5] + Hx[2]*Gx[9] + Hx[3]*Gx[13];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[6] + Hx[2]*Gx[10] + Hx[3]*Gx[14];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[7] + Hx[2]*Gx[11] + Hx[3]*Gx[15];
A01[4] = + Hx[4]*Gx[0] + Hx[5]*Gx[4] + Hx[6]*Gx[8] + Hx[7]*Gx[12];
A01[5] = + Hx[4]*Gx[1] + Hx[5]*Gx[5] + Hx[6]*Gx[9] + Hx[7]*Gx[13];
A01[6] = + Hx[4]*Gx[2] + Hx[5]*Gx[6] + Hx[6]*Gx[10] + Hx[7]*Gx[14];
A01[7] = + Hx[4]*Gx[3] + Hx[5]*Gx[7] + Hx[6]*Gx[11] + Hx[7]*Gx[15];
A01[8] = + Hx[8]*Gx[0] + Hx[9]*Gx[4] + Hx[10]*Gx[8] + Hx[11]*Gx[12];
A01[9] = + Hx[8]*Gx[1] + Hx[9]*Gx[5] + Hx[10]*Gx[9] + Hx[11]*Gx[13];
A01[10] = + Hx[8]*Gx[2] + Hx[9]*Gx[6] + Hx[10]*Gx[10] + Hx[11]*Gx[14];
A01[11] = + Hx[8]*Gx[3] + Hx[9]*Gx[7] + Hx[10]*Gx[11] + Hx[11]*Gx[15];
A01[12] = + Hx[12]*Gx[0] + Hx[13]*Gx[4] + Hx[14]*Gx[8] + Hx[15]*Gx[12];
A01[13] = + Hx[12]*Gx[1] + Hx[13]*Gx[5] + Hx[14]*Gx[9] + Hx[15]*Gx[13];
A01[14] = + Hx[12]*Gx[2] + Hx[13]*Gx[6] + Hx[14]*Gx[10] + Hx[15]*Gx[14];
A01[15] = + Hx[12]*Gx[3] + Hx[13]*Gx[7] + Hx[14]*Gx[11] + Hx[15]*Gx[15];
A01[16] = + Hx[16]*Gx[0] + Hx[17]*Gx[4] + Hx[18]*Gx[8] + Hx[19]*Gx[12];
A01[17] = + Hx[16]*Gx[1] + Hx[17]*Gx[5] + Hx[18]*Gx[9] + Hx[19]*Gx[13];
A01[18] = + Hx[16]*Gx[2] + Hx[17]*Gx[6] + Hx[18]*Gx[10] + Hx[19]*Gx[14];
A01[19] = + Hx[16]*Gx[3] + Hx[17]*Gx[7] + Hx[18]*Gx[11] + Hx[19]*Gx[15];
A01[20] = + Hx[20]*Gx[0] + Hx[21]*Gx[4] + Hx[22]*Gx[8] + Hx[23]*Gx[12];
A01[21] = + Hx[20]*Gx[1] + Hx[21]*Gx[5] + Hx[22]*Gx[9] + Hx[23]*Gx[13];
A01[22] = + Hx[20]*Gx[2] + Hx[21]*Gx[6] + Hx[22]*Gx[10] + Hx[23]*Gx[14];
A01[23] = + Hx[20]*Gx[3] + Hx[21]*Gx[7] + Hx[22]*Gx[11] + Hx[23]*Gx[15];
A01[24] = + Hx[24]*Gx[0] + Hx[25]*Gx[4] + Hx[26]*Gx[8] + Hx[27]*Gx[12];
A01[25] = + Hx[24]*Gx[1] + Hx[25]*Gx[5] + Hx[26]*Gx[9] + Hx[27]*Gx[13];
A01[26] = + Hx[24]*Gx[2] + Hx[25]*Gx[6] + Hx[26]*Gx[10] + Hx[27]*Gx[14];
A01[27] = + Hx[24]*Gx[3] + Hx[25]*Gx[7] + Hx[26]*Gx[11] + Hx[27]*Gx[15];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 420) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6];
acadoWorkspace.A[(row * 420) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7];
acadoWorkspace.A[(row * 420 + 60) + (col * 2)] = + Hx[4]*E[0] + Hx[5]*E[2] + Hx[6]*E[4] + Hx[7]*E[6];
acadoWorkspace.A[(row * 420 + 60) + (col * 2 + 1)] = + Hx[4]*E[1] + Hx[5]*E[3] + Hx[6]*E[5] + Hx[7]*E[7];
acadoWorkspace.A[(row * 420 + 120) + (col * 2)] = + Hx[8]*E[0] + Hx[9]*E[2] + Hx[10]*E[4] + Hx[11]*E[6];
acadoWorkspace.A[(row * 420 + 120) + (col * 2 + 1)] = + Hx[8]*E[1] + Hx[9]*E[3] + Hx[10]*E[5] + Hx[11]*E[7];
acadoWorkspace.A[(row * 420 + 180) + (col * 2)] = + Hx[12]*E[0] + Hx[13]*E[2] + Hx[14]*E[4] + Hx[15]*E[6];
acadoWorkspace.A[(row * 420 + 180) + (col * 2 + 1)] = + Hx[12]*E[1] + Hx[13]*E[3] + Hx[14]*E[5] + Hx[15]*E[7];
acadoWorkspace.A[(row * 420 + 240) + (col * 2)] = + Hx[16]*E[0] + Hx[17]*E[2] + Hx[18]*E[4] + Hx[19]*E[6];
acadoWorkspace.A[(row * 420 + 240) + (col * 2 + 1)] = + Hx[16]*E[1] + Hx[17]*E[3] + Hx[18]*E[5] + Hx[19]*E[7];
acadoWorkspace.A[(row * 420 + 300) + (col * 2)] = + Hx[20]*E[0] + Hx[21]*E[2] + Hx[22]*E[4] + Hx[23]*E[6];
acadoWorkspace.A[(row * 420 + 300) + (col * 2 + 1)] = + Hx[20]*E[1] + Hx[21]*E[3] + Hx[22]*E[5] + Hx[23]*E[7];
acadoWorkspace.A[(row * 420 + 360) + (col * 2)] = + Hx[24]*E[0] + Hx[25]*E[2] + Hx[26]*E[4] + Hx[27]*E[6];
acadoWorkspace.A[(row * 420 + 360) + (col * 2 + 1)] = + Hx[24]*E[1] + Hx[25]*E[3] + Hx[26]*E[5] + Hx[27]*E[7];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3];
acadoWorkspace.evHxd[1] = + Hx[4]*tmpd[0] + Hx[5]*tmpd[1] + Hx[6]*tmpd[2] + Hx[7]*tmpd[3];
acadoWorkspace.evHxd[2] = + Hx[8]*tmpd[0] + Hx[9]*tmpd[1] + Hx[10]*tmpd[2] + Hx[11]*tmpd[3];
acadoWorkspace.evHxd[3] = + Hx[12]*tmpd[0] + Hx[13]*tmpd[1] + Hx[14]*tmpd[2] + Hx[15]*tmpd[3];
acadoWorkspace.evHxd[4] = + Hx[16]*tmpd[0] + Hx[17]*tmpd[1] + Hx[18]*tmpd[2] + Hx[19]*tmpd[3];
acadoWorkspace.evHxd[5] = + Hx[20]*tmpd[0] + Hx[21]*tmpd[1] + Hx[22]*tmpd[2] + Hx[23]*tmpd[3];
acadoWorkspace.evHxd[6] = + Hx[24]*tmpd[0] + Hx[25]*tmpd[1] + Hx[26]*tmpd[2] + Hx[27]*tmpd[3];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
lbA[2] -= acadoWorkspace.evHxd[2];
lbA[3] -= acadoWorkspace.evHxd[3];
lbA[4] -= acadoWorkspace.evHxd[4];
lbA[5] -= acadoWorkspace.evHxd[5];
lbA[6] -= acadoWorkspace.evHxd[6];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
ubA[2] -= acadoWorkspace.evHxd[2];
ubA[3] -= acadoWorkspace.evHxd[3];
ubA[4] -= acadoWorkspace.evHxd[4];
ubA[5] -= acadoWorkspace.evHxd[5];
ubA[6] -= acadoWorkspace.evHxd[6];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 105. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (sqrt(((((xd[0]-od[0])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[0])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[1])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[1])+(real_t)(5.0000000000000003e-02))))));
a[1] = (sqrt(((((xd[0]-od[2])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[2])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[3])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[3])+(real_t)(5.0000000000000003e-02))))));
a[2] = (sqrt(((((xd[0]-od[4])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[4])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[5])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[5])+(real_t)(5.0000000000000003e-02))))));
a[3] = (sqrt(((((xd[0]-od[6])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[6])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[7])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[7])+(real_t)(5.0000000000000003e-02))))));
a[4] = (sqrt(((((xd[0]-od[8])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[8])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[9])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[9])+(real_t)(5.0000000000000003e-02))))));
a[5] = (sqrt(((((xd[0]-od[10])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[10])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[11])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[11])+(real_t)(5.0000000000000003e-02))))));
a[6] = (sqrt(((((xd[0]-od[12])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[12])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[13])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[13])+(real_t)(5.0000000000000003e-02))))));
a[7] = ((xd[0]-od[0])+(real_t)(5.0000000000000003e-02));
a[8] = ((xd[0]-od[0])+(real_t)(5.0000000000000003e-02));
a[9] = (a[7]+a[8]);
a[10] = (1.0/sqrt(((((xd[0]-od[0])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[0])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[1])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[1])+(real_t)(5.0000000000000003e-02))))));
a[11] = (a[10]*(real_t)(5.0000000000000000e-01));
a[12] = (a[9]*a[11]);
a[13] = ((xd[1]-od[1])+(real_t)(5.0000000000000003e-02));
a[14] = ((xd[1]-od[1])+(real_t)(5.0000000000000003e-02));
a[15] = (a[13]+a[14]);
a[16] = (a[15]*a[11]);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = ((xd[0]-od[2])+(real_t)(5.0000000000000003e-02));
a[20] = ((xd[0]-od[2])+(real_t)(5.0000000000000003e-02));
a[21] = (a[19]+a[20]);
a[22] = (1.0/sqrt(((((xd[0]-od[2])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[2])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[3])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[3])+(real_t)(5.0000000000000003e-02))))));
a[23] = (a[22]*(real_t)(5.0000000000000000e-01));
a[24] = (a[21]*a[23]);
a[25] = ((xd[1]-od[3])+(real_t)(5.0000000000000003e-02));
a[26] = ((xd[1]-od[3])+(real_t)(5.0000000000000003e-02));
a[27] = (a[25]+a[26]);
a[28] = (a[27]*a[23]);
a[29] = (real_t)(0.0000000000000000e+00);
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = ((xd[0]-od[4])+(real_t)(5.0000000000000003e-02));
a[32] = ((xd[0]-od[4])+(real_t)(5.0000000000000003e-02));
a[33] = (a[31]+a[32]);
a[34] = (1.0/sqrt(((((xd[0]-od[4])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[4])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[5])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[5])+(real_t)(5.0000000000000003e-02))))));
a[35] = (a[34]*(real_t)(5.0000000000000000e-01));
a[36] = (a[33]*a[35]);
a[37] = ((xd[1]-od[5])+(real_t)(5.0000000000000003e-02));
a[38] = ((xd[1]-od[5])+(real_t)(5.0000000000000003e-02));
a[39] = (a[37]+a[38]);
a[40] = (a[39]*a[35]);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(0.0000000000000000e+00);
a[43] = ((xd[0]-od[6])+(real_t)(5.0000000000000003e-02));
a[44] = ((xd[0]-od[6])+(real_t)(5.0000000000000003e-02));
a[45] = (a[43]+a[44]);
a[46] = (1.0/sqrt(((((xd[0]-od[6])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[6])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[7])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[7])+(real_t)(5.0000000000000003e-02))))));
a[47] = (a[46]*(real_t)(5.0000000000000000e-01));
a[48] = (a[45]*a[47]);
a[49] = ((xd[1]-od[7])+(real_t)(5.0000000000000003e-02));
a[50] = ((xd[1]-od[7])+(real_t)(5.0000000000000003e-02));
a[51] = (a[49]+a[50]);
a[52] = (a[51]*a[47]);
a[53] = (real_t)(0.0000000000000000e+00);
a[54] = (real_t)(0.0000000000000000e+00);
a[55] = ((xd[0]-od[8])+(real_t)(5.0000000000000003e-02));
a[56] = ((xd[0]-od[8])+(real_t)(5.0000000000000003e-02));
a[57] = (a[55]+a[56]);
a[58] = (1.0/sqrt(((((xd[0]-od[8])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[8])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[9])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[9])+(real_t)(5.0000000000000003e-02))))));
a[59] = (a[58]*(real_t)(5.0000000000000000e-01));
a[60] = (a[57]*a[59]);
a[61] = ((xd[1]-od[9])+(real_t)(5.0000000000000003e-02));
a[62] = ((xd[1]-od[9])+(real_t)(5.0000000000000003e-02));
a[63] = (a[61]+a[62]);
a[64] = (a[63]*a[59]);
a[65] = (real_t)(0.0000000000000000e+00);
a[66] = (real_t)(0.0000000000000000e+00);
a[67] = ((xd[0]-od[10])+(real_t)(5.0000000000000003e-02));
a[68] = ((xd[0]-od[10])+(real_t)(5.0000000000000003e-02));
a[69] = (a[67]+a[68]);
a[70] = (1.0/sqrt(((((xd[0]-od[10])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[10])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[11])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[11])+(real_t)(5.0000000000000003e-02))))));
a[71] = (a[70]*(real_t)(5.0000000000000000e-01));
a[72] = (a[69]*a[71]);
a[73] = ((xd[1]-od[11])+(real_t)(5.0000000000000003e-02));
a[74] = ((xd[1]-od[11])+(real_t)(5.0000000000000003e-02));
a[75] = (a[73]+a[74]);
a[76] = (a[75]*a[71]);
a[77] = (real_t)(0.0000000000000000e+00);
a[78] = (real_t)(0.0000000000000000e+00);
a[79] = ((xd[0]-od[12])+(real_t)(5.0000000000000003e-02));
a[80] = ((xd[0]-od[12])+(real_t)(5.0000000000000003e-02));
a[81] = (a[79]+a[80]);
a[82] = (1.0/sqrt(((((xd[0]-od[12])+(real_t)(5.0000000000000003e-02))*((xd[0]-od[12])+(real_t)(5.0000000000000003e-02)))+(((xd[1]-od[13])+(real_t)(5.0000000000000003e-02))*((xd[1]-od[13])+(real_t)(5.0000000000000003e-02))))));
a[83] = (a[82]*(real_t)(5.0000000000000000e-01));
a[84] = (a[81]*a[83]);
a[85] = ((xd[1]-od[13])+(real_t)(5.0000000000000003e-02));
a[86] = ((xd[1]-od[13])+(real_t)(5.0000000000000003e-02));
a[87] = (a[85]+a[86]);
a[88] = (a[87]*a[83]);
a[89] = (real_t)(0.0000000000000000e+00);
a[90] = (real_t)(0.0000000000000000e+00);
a[91] = (real_t)(0.0000000000000000e+00);
a[92] = (real_t)(0.0000000000000000e+00);
a[93] = (real_t)(0.0000000000000000e+00);
a[94] = (real_t)(0.0000000000000000e+00);
a[95] = (real_t)(0.0000000000000000e+00);
a[96] = (real_t)(0.0000000000000000e+00);
a[97] = (real_t)(0.0000000000000000e+00);
a[98] = (real_t)(0.0000000000000000e+00);
a[99] = (real_t)(0.0000000000000000e+00);
a[100] = (real_t)(0.0000000000000000e+00);
a[101] = (real_t)(0.0000000000000000e+00);
a[102] = (real_t)(0.0000000000000000e+00);
a[103] = (real_t)(0.0000000000000000e+00);
a[104] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = a[0];
out[1] = a[1];
out[2] = a[2];
out[3] = a[3];
out[4] = a[4];
out[5] = a[5];
out[6] = a[6];
out[7] = a[12];
out[8] = a[16];
out[9] = a[17];
out[10] = a[18];
out[11] = a[24];
out[12] = a[28];
out[13] = a[29];
out[14] = a[30];
out[15] = a[36];
out[16] = a[40];
out[17] = a[41];
out[18] = a[42];
out[19] = a[48];
out[20] = a[52];
out[21] = a[53];
out[22] = a[54];
out[23] = a[60];
out[24] = a[64];
out[25] = a[65];
out[26] = a[66];
out[27] = a[72];
out[28] = a[76];
out[29] = a[77];
out[30] = a[78];
out[31] = a[84];
out[32] = a[88];
out[33] = a[89];
out[34] = a[90];
out[35] = a[91];
out[36] = a[92];
out[37] = a[93];
out[38] = a[94];
out[39] = a[95];
out[40] = a[96];
out[41] = a[97];
out[42] = a[98];
out[43] = a[99];
out[44] = a[100];
out[45] = a[101];
out[46] = a[102];
out[47] = a[103];
out[48] = a[104];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 16 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 4-4 ]), &(acadoWorkspace.evGx[ lRun1 * 16 ]), &(acadoWorkspace.d[ lRun1 * 4 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 16-16 ]), &(acadoWorkspace.evGx[ lRun1 * 16 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.E[ lRun3 * 8 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 8 ]), &(acadoWorkspace.E[ lRun3 * 8 ]) );
}

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( &(acadoWorkspace.Q1[ lRun1 * 16 + 16 ]), &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.QE[ lRun3 * 8 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.QE[ lRun3 * 8 ]) );
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 8 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 8 ]), &(acadoWorkspace.evGx[ lRun2 * 16 ]), &(acadoWorkspace.H10[ lRun1 * 8 ]) );
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1, &(acadoWorkspace.R1[ lRun1 * 4 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.QE[ lRun5 * 8 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.QE[ lRun5 * 8 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

acado_multQ1d( &(acadoWorkspace.Q1[ 16 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.Qd[ 4 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.Qd[ 8 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.Qd[ 16 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.Qd[ 28 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.Qd[ 32 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.Qd[ 44 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.Qd[ 52 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.Qd[ 56 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.Qd[ 64 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.Qd[ 68 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.d[ 76 ]), &(acadoWorkspace.Qd[ 76 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 336 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.Qd[ 80 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 352 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.Qd[ 84 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 368 ]), &(acadoWorkspace.d[ 88 ]), &(acadoWorkspace.Qd[ 88 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.d[ 92 ]), &(acadoWorkspace.Qd[ 92 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.Qd[ 96 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 416 ]), &(acadoWorkspace.d[ 100 ]), &(acadoWorkspace.Qd[ 100 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.d[ 104 ]), &(acadoWorkspace.Qd[ 104 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.Qd[ 108 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 464 ]), &(acadoWorkspace.d[ 112 ]), &(acadoWorkspace.Qd[ 112 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 116 ]), &(acadoWorkspace.Qd[ 116 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 8 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.conValueIn[6] = acadoVariables.od[lRun1 * 15];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun1 * 15 + 1];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun1 * 15 + 2];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun1 * 15 + 3];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun1 * 15 + 4];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun1 * 15 + 5];
acadoWorkspace.conValueIn[12] = acadoVariables.od[lRun1 * 15 + 6];
acadoWorkspace.conValueIn[13] = acadoVariables.od[lRun1 * 15 + 7];
acadoWorkspace.conValueIn[14] = acadoVariables.od[lRun1 * 15 + 8];
acadoWorkspace.conValueIn[15] = acadoVariables.od[lRun1 * 15 + 9];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun1 * 15 + 10];
acadoWorkspace.conValueIn[17] = acadoVariables.od[lRun1 * 15 + 11];
acadoWorkspace.conValueIn[18] = acadoVariables.od[lRun1 * 15 + 12];
acadoWorkspace.conValueIn[19] = acadoVariables.od[lRun1 * 15 + 13];
acadoWorkspace.conValueIn[20] = acadoVariables.od[lRun1 * 15 + 14];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 7] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 7 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 7 + 2] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evH[lRun1 * 7 + 3] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evH[lRun1 * 7 + 4] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evH[lRun1 * 7 + 5] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evH[lRun1 * 7 + 6] = acadoWorkspace.conValueOut[6];

acadoWorkspace.evHx[lRun1 * 28] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 28 + 1] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 28 + 2] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 28 + 3] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 28 + 4] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 28 + 5] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 28 + 6] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 28 + 7] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 28 + 8] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHx[lRun1 * 28 + 9] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHx[lRun1 * 28 + 10] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHx[lRun1 * 28 + 11] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHx[lRun1 * 28 + 12] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHx[lRun1 * 28 + 13] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHx[lRun1 * 28 + 14] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHx[lRun1 * 28 + 15] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHx[lRun1 * 28 + 16] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHx[lRun1 * 28 + 17] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHx[lRun1 * 28 + 18] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHx[lRun1 * 28 + 19] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evHx[lRun1 * 28 + 20] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evHx[lRun1 * 28 + 21] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evHx[lRun1 * 28 + 22] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evHx[lRun1 * 28 + 23] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evHx[lRun1 * 28 + 24] = acadoWorkspace.conValueOut[31];
acadoWorkspace.evHx[lRun1 * 28 + 25] = acadoWorkspace.conValueOut[32];
acadoWorkspace.evHx[lRun1 * 28 + 26] = acadoWorkspace.conValueOut[33];
acadoWorkspace.evHx[lRun1 * 28 + 27] = acadoWorkspace.conValueOut[34];
acadoWorkspace.evHu[lRun1 * 14] = acadoWorkspace.conValueOut[35];
acadoWorkspace.evHu[lRun1 * 14 + 1] = acadoWorkspace.conValueOut[36];
acadoWorkspace.evHu[lRun1 * 14 + 2] = acadoWorkspace.conValueOut[37];
acadoWorkspace.evHu[lRun1 * 14 + 3] = acadoWorkspace.conValueOut[38];
acadoWorkspace.evHu[lRun1 * 14 + 4] = acadoWorkspace.conValueOut[39];
acadoWorkspace.evHu[lRun1 * 14 + 5] = acadoWorkspace.conValueOut[40];
acadoWorkspace.evHu[lRun1 * 14 + 6] = acadoWorkspace.conValueOut[41];
acadoWorkspace.evHu[lRun1 * 14 + 7] = acadoWorkspace.conValueOut[42];
acadoWorkspace.evHu[lRun1 * 14 + 8] = acadoWorkspace.conValueOut[43];
acadoWorkspace.evHu[lRun1 * 14 + 9] = acadoWorkspace.conValueOut[44];
acadoWorkspace.evHu[lRun1 * 14 + 10] = acadoWorkspace.conValueOut[45];
acadoWorkspace.evHu[lRun1 * 14 + 11] = acadoWorkspace.conValueOut[46];
acadoWorkspace.evHu[lRun1 * 14 + 12] = acadoWorkspace.conValueOut[47];
acadoWorkspace.evHu[lRun1 * 14 + 13] = acadoWorkspace.conValueOut[48];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];
acadoWorkspace.A01[7] = acadoWorkspace.evHx[7];
acadoWorkspace.A01[8] = acadoWorkspace.evHx[8];
acadoWorkspace.A01[9] = acadoWorkspace.evHx[9];
acadoWorkspace.A01[10] = acadoWorkspace.evHx[10];
acadoWorkspace.A01[11] = acadoWorkspace.evHx[11];
acadoWorkspace.A01[12] = acadoWorkspace.evHx[12];
acadoWorkspace.A01[13] = acadoWorkspace.evHx[13];
acadoWorkspace.A01[14] = acadoWorkspace.evHx[14];
acadoWorkspace.A01[15] = acadoWorkspace.evHx[15];
acadoWorkspace.A01[16] = acadoWorkspace.evHx[16];
acadoWorkspace.A01[17] = acadoWorkspace.evHx[17];
acadoWorkspace.A01[18] = acadoWorkspace.evHx[18];
acadoWorkspace.A01[19] = acadoWorkspace.evHx[19];
acadoWorkspace.A01[20] = acadoWorkspace.evHx[20];
acadoWorkspace.A01[21] = acadoWorkspace.evHx[21];
acadoWorkspace.A01[22] = acadoWorkspace.evHx[22];
acadoWorkspace.A01[23] = acadoWorkspace.evHx[23];
acadoWorkspace.A01[24] = acadoWorkspace.evHx[24];
acadoWorkspace.A01[25] = acadoWorkspace.evHx[25];
acadoWorkspace.A01[26] = acadoWorkspace.evHx[26];
acadoWorkspace.A01[27] = acadoWorkspace.evHx[27];

acado_multHxC( &(acadoWorkspace.evHx[ 28 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 28 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.A01[ 56 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.A01[ 84 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 112 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.A01[ 112 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.A01[ 140 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.A01[ 168 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 196 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.A01[ 196 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.A01[ 224 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.A01[ 252 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 280 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.A01[ 280 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 308 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.A01[ 308 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 336 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.A01[ 336 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 364 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.A01[ 364 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 392 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.A01[ 392 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 420 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.A01[ 420 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.A01[ 448 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 476 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.A01[ 476 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 504 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.A01[ 504 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 532 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.A01[ 532 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 560 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.A01[ 560 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 588 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.A01[ 588 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 616 ]), &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.A01[ 616 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 644 ]), &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.A01[ 644 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 672 ]), &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.A01[ 672 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 700 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.A01[ 700 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 728 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.A01[ 728 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 756 ]), &(acadoWorkspace.evGx[ 416 ]), &(acadoWorkspace.A01[ 756 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 784 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.A01[ 784 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 812 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.A01[ 812 ]) );

for (lRun2 = 0; lRun2 < 29; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun3);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 28 + 28 ]), &(acadoWorkspace.E[ lRun4 * 8 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[60] = acadoWorkspace.evHu[2];
acadoWorkspace.A[61] = acadoWorkspace.evHu[3];
acadoWorkspace.A[120] = acadoWorkspace.evHu[4];
acadoWorkspace.A[121] = acadoWorkspace.evHu[5];
acadoWorkspace.A[180] = acadoWorkspace.evHu[6];
acadoWorkspace.A[181] = acadoWorkspace.evHu[7];
acadoWorkspace.A[240] = acadoWorkspace.evHu[8];
acadoWorkspace.A[241] = acadoWorkspace.evHu[9];
acadoWorkspace.A[300] = acadoWorkspace.evHu[10];
acadoWorkspace.A[301] = acadoWorkspace.evHu[11];
acadoWorkspace.A[360] = acadoWorkspace.evHu[12];
acadoWorkspace.A[361] = acadoWorkspace.evHu[13];
acadoWorkspace.A[422] = acadoWorkspace.evHu[14];
acadoWorkspace.A[423] = acadoWorkspace.evHu[15];
acadoWorkspace.A[482] = acadoWorkspace.evHu[16];
acadoWorkspace.A[483] = acadoWorkspace.evHu[17];
acadoWorkspace.A[542] = acadoWorkspace.evHu[18];
acadoWorkspace.A[543] = acadoWorkspace.evHu[19];
acadoWorkspace.A[602] = acadoWorkspace.evHu[20];
acadoWorkspace.A[603] = acadoWorkspace.evHu[21];
acadoWorkspace.A[662] = acadoWorkspace.evHu[22];
acadoWorkspace.A[663] = acadoWorkspace.evHu[23];
acadoWorkspace.A[722] = acadoWorkspace.evHu[24];
acadoWorkspace.A[723] = acadoWorkspace.evHu[25];
acadoWorkspace.A[782] = acadoWorkspace.evHu[26];
acadoWorkspace.A[783] = acadoWorkspace.evHu[27];
acadoWorkspace.A[844] = acadoWorkspace.evHu[28];
acadoWorkspace.A[845] = acadoWorkspace.evHu[29];
acadoWorkspace.A[904] = acadoWorkspace.evHu[30];
acadoWorkspace.A[905] = acadoWorkspace.evHu[31];
acadoWorkspace.A[964] = acadoWorkspace.evHu[32];
acadoWorkspace.A[965] = acadoWorkspace.evHu[33];
acadoWorkspace.A[1024] = acadoWorkspace.evHu[34];
acadoWorkspace.A[1025] = acadoWorkspace.evHu[35];
acadoWorkspace.A[1084] = acadoWorkspace.evHu[36];
acadoWorkspace.A[1085] = acadoWorkspace.evHu[37];
acadoWorkspace.A[1144] = acadoWorkspace.evHu[38];
acadoWorkspace.A[1145] = acadoWorkspace.evHu[39];
acadoWorkspace.A[1204] = acadoWorkspace.evHu[40];
acadoWorkspace.A[1205] = acadoWorkspace.evHu[41];
acadoWorkspace.A[1266] = acadoWorkspace.evHu[42];
acadoWorkspace.A[1267] = acadoWorkspace.evHu[43];
acadoWorkspace.A[1326] = acadoWorkspace.evHu[44];
acadoWorkspace.A[1327] = acadoWorkspace.evHu[45];
acadoWorkspace.A[1386] = acadoWorkspace.evHu[46];
acadoWorkspace.A[1387] = acadoWorkspace.evHu[47];
acadoWorkspace.A[1446] = acadoWorkspace.evHu[48];
acadoWorkspace.A[1447] = acadoWorkspace.evHu[49];
acadoWorkspace.A[1506] = acadoWorkspace.evHu[50];
acadoWorkspace.A[1507] = acadoWorkspace.evHu[51];
acadoWorkspace.A[1566] = acadoWorkspace.evHu[52];
acadoWorkspace.A[1567] = acadoWorkspace.evHu[53];
acadoWorkspace.A[1626] = acadoWorkspace.evHu[54];
acadoWorkspace.A[1627] = acadoWorkspace.evHu[55];
acadoWorkspace.A[1688] = acadoWorkspace.evHu[56];
acadoWorkspace.A[1689] = acadoWorkspace.evHu[57];
acadoWorkspace.A[1748] = acadoWorkspace.evHu[58];
acadoWorkspace.A[1749] = acadoWorkspace.evHu[59];
acadoWorkspace.A[1808] = acadoWorkspace.evHu[60];
acadoWorkspace.A[1809] = acadoWorkspace.evHu[61];
acadoWorkspace.A[1868] = acadoWorkspace.evHu[62];
acadoWorkspace.A[1869] = acadoWorkspace.evHu[63];
acadoWorkspace.A[1928] = acadoWorkspace.evHu[64];
acadoWorkspace.A[1929] = acadoWorkspace.evHu[65];
acadoWorkspace.A[1988] = acadoWorkspace.evHu[66];
acadoWorkspace.A[1989] = acadoWorkspace.evHu[67];
acadoWorkspace.A[2048] = acadoWorkspace.evHu[68];
acadoWorkspace.A[2049] = acadoWorkspace.evHu[69];
acadoWorkspace.A[2110] = acadoWorkspace.evHu[70];
acadoWorkspace.A[2111] = acadoWorkspace.evHu[71];
acadoWorkspace.A[2170] = acadoWorkspace.evHu[72];
acadoWorkspace.A[2171] = acadoWorkspace.evHu[73];
acadoWorkspace.A[2230] = acadoWorkspace.evHu[74];
acadoWorkspace.A[2231] = acadoWorkspace.evHu[75];
acadoWorkspace.A[2290] = acadoWorkspace.evHu[76];
acadoWorkspace.A[2291] = acadoWorkspace.evHu[77];
acadoWorkspace.A[2350] = acadoWorkspace.evHu[78];
acadoWorkspace.A[2351] = acadoWorkspace.evHu[79];
acadoWorkspace.A[2410] = acadoWorkspace.evHu[80];
acadoWorkspace.A[2411] = acadoWorkspace.evHu[81];
acadoWorkspace.A[2470] = acadoWorkspace.evHu[82];
acadoWorkspace.A[2471] = acadoWorkspace.evHu[83];
acadoWorkspace.A[2532] = acadoWorkspace.evHu[84];
acadoWorkspace.A[2533] = acadoWorkspace.evHu[85];
acadoWorkspace.A[2592] = acadoWorkspace.evHu[86];
acadoWorkspace.A[2593] = acadoWorkspace.evHu[87];
acadoWorkspace.A[2652] = acadoWorkspace.evHu[88];
acadoWorkspace.A[2653] = acadoWorkspace.evHu[89];
acadoWorkspace.A[2712] = acadoWorkspace.evHu[90];
acadoWorkspace.A[2713] = acadoWorkspace.evHu[91];
acadoWorkspace.A[2772] = acadoWorkspace.evHu[92];
acadoWorkspace.A[2773] = acadoWorkspace.evHu[93];
acadoWorkspace.A[2832] = acadoWorkspace.evHu[94];
acadoWorkspace.A[2833] = acadoWorkspace.evHu[95];
acadoWorkspace.A[2892] = acadoWorkspace.evHu[96];
acadoWorkspace.A[2893] = acadoWorkspace.evHu[97];
acadoWorkspace.A[2954] = acadoWorkspace.evHu[98];
acadoWorkspace.A[2955] = acadoWorkspace.evHu[99];
acadoWorkspace.A[3014] = acadoWorkspace.evHu[100];
acadoWorkspace.A[3015] = acadoWorkspace.evHu[101];
acadoWorkspace.A[3074] = acadoWorkspace.evHu[102];
acadoWorkspace.A[3075] = acadoWorkspace.evHu[103];
acadoWorkspace.A[3134] = acadoWorkspace.evHu[104];
acadoWorkspace.A[3135] = acadoWorkspace.evHu[105];
acadoWorkspace.A[3194] = acadoWorkspace.evHu[106];
acadoWorkspace.A[3195] = acadoWorkspace.evHu[107];
acadoWorkspace.A[3254] = acadoWorkspace.evHu[108];
acadoWorkspace.A[3255] = acadoWorkspace.evHu[109];
acadoWorkspace.A[3314] = acadoWorkspace.evHu[110];
acadoWorkspace.A[3315] = acadoWorkspace.evHu[111];
acadoWorkspace.A[3376] = acadoWorkspace.evHu[112];
acadoWorkspace.A[3377] = acadoWorkspace.evHu[113];
acadoWorkspace.A[3436] = acadoWorkspace.evHu[114];
acadoWorkspace.A[3437] = acadoWorkspace.evHu[115];
acadoWorkspace.A[3496] = acadoWorkspace.evHu[116];
acadoWorkspace.A[3497] = acadoWorkspace.evHu[117];
acadoWorkspace.A[3556] = acadoWorkspace.evHu[118];
acadoWorkspace.A[3557] = acadoWorkspace.evHu[119];
acadoWorkspace.A[3616] = acadoWorkspace.evHu[120];
acadoWorkspace.A[3617] = acadoWorkspace.evHu[121];
acadoWorkspace.A[3676] = acadoWorkspace.evHu[122];
acadoWorkspace.A[3677] = acadoWorkspace.evHu[123];
acadoWorkspace.A[3736] = acadoWorkspace.evHu[124];
acadoWorkspace.A[3737] = acadoWorkspace.evHu[125];
acadoWorkspace.A[3798] = acadoWorkspace.evHu[126];
acadoWorkspace.A[3799] = acadoWorkspace.evHu[127];
acadoWorkspace.A[3858] = acadoWorkspace.evHu[128];
acadoWorkspace.A[3859] = acadoWorkspace.evHu[129];
acadoWorkspace.A[3918] = acadoWorkspace.evHu[130];
acadoWorkspace.A[3919] = acadoWorkspace.evHu[131];
acadoWorkspace.A[3978] = acadoWorkspace.evHu[132];
acadoWorkspace.A[3979] = acadoWorkspace.evHu[133];
acadoWorkspace.A[4038] = acadoWorkspace.evHu[134];
acadoWorkspace.A[4039] = acadoWorkspace.evHu[135];
acadoWorkspace.A[4098] = acadoWorkspace.evHu[136];
acadoWorkspace.A[4099] = acadoWorkspace.evHu[137];
acadoWorkspace.A[4158] = acadoWorkspace.evHu[138];
acadoWorkspace.A[4159] = acadoWorkspace.evHu[139];
acadoWorkspace.A[4220] = acadoWorkspace.evHu[140];
acadoWorkspace.A[4221] = acadoWorkspace.evHu[141];
acadoWorkspace.A[4280] = acadoWorkspace.evHu[142];
acadoWorkspace.A[4281] = acadoWorkspace.evHu[143];
acadoWorkspace.A[4340] = acadoWorkspace.evHu[144];
acadoWorkspace.A[4341] = acadoWorkspace.evHu[145];
acadoWorkspace.A[4400] = acadoWorkspace.evHu[146];
acadoWorkspace.A[4401] = acadoWorkspace.evHu[147];
acadoWorkspace.A[4460] = acadoWorkspace.evHu[148];
acadoWorkspace.A[4461] = acadoWorkspace.evHu[149];
acadoWorkspace.A[4520] = acadoWorkspace.evHu[150];
acadoWorkspace.A[4521] = acadoWorkspace.evHu[151];
acadoWorkspace.A[4580] = acadoWorkspace.evHu[152];
acadoWorkspace.A[4581] = acadoWorkspace.evHu[153];
acadoWorkspace.A[4642] = acadoWorkspace.evHu[154];
acadoWorkspace.A[4643] = acadoWorkspace.evHu[155];
acadoWorkspace.A[4702] = acadoWorkspace.evHu[156];
acadoWorkspace.A[4703] = acadoWorkspace.evHu[157];
acadoWorkspace.A[4762] = acadoWorkspace.evHu[158];
acadoWorkspace.A[4763] = acadoWorkspace.evHu[159];
acadoWorkspace.A[4822] = acadoWorkspace.evHu[160];
acadoWorkspace.A[4823] = acadoWorkspace.evHu[161];
acadoWorkspace.A[4882] = acadoWorkspace.evHu[162];
acadoWorkspace.A[4883] = acadoWorkspace.evHu[163];
acadoWorkspace.A[4942] = acadoWorkspace.evHu[164];
acadoWorkspace.A[4943] = acadoWorkspace.evHu[165];
acadoWorkspace.A[5002] = acadoWorkspace.evHu[166];
acadoWorkspace.A[5003] = acadoWorkspace.evHu[167];
acadoWorkspace.A[5064] = acadoWorkspace.evHu[168];
acadoWorkspace.A[5065] = acadoWorkspace.evHu[169];
acadoWorkspace.A[5124] = acadoWorkspace.evHu[170];
acadoWorkspace.A[5125] = acadoWorkspace.evHu[171];
acadoWorkspace.A[5184] = acadoWorkspace.evHu[172];
acadoWorkspace.A[5185] = acadoWorkspace.evHu[173];
acadoWorkspace.A[5244] = acadoWorkspace.evHu[174];
acadoWorkspace.A[5245] = acadoWorkspace.evHu[175];
acadoWorkspace.A[5304] = acadoWorkspace.evHu[176];
acadoWorkspace.A[5305] = acadoWorkspace.evHu[177];
acadoWorkspace.A[5364] = acadoWorkspace.evHu[178];
acadoWorkspace.A[5365] = acadoWorkspace.evHu[179];
acadoWorkspace.A[5424] = acadoWorkspace.evHu[180];
acadoWorkspace.A[5425] = acadoWorkspace.evHu[181];
acadoWorkspace.A[5486] = acadoWorkspace.evHu[182];
acadoWorkspace.A[5487] = acadoWorkspace.evHu[183];
acadoWorkspace.A[5546] = acadoWorkspace.evHu[184];
acadoWorkspace.A[5547] = acadoWorkspace.evHu[185];
acadoWorkspace.A[5606] = acadoWorkspace.evHu[186];
acadoWorkspace.A[5607] = acadoWorkspace.evHu[187];
acadoWorkspace.A[5666] = acadoWorkspace.evHu[188];
acadoWorkspace.A[5667] = acadoWorkspace.evHu[189];
acadoWorkspace.A[5726] = acadoWorkspace.evHu[190];
acadoWorkspace.A[5727] = acadoWorkspace.evHu[191];
acadoWorkspace.A[5786] = acadoWorkspace.evHu[192];
acadoWorkspace.A[5787] = acadoWorkspace.evHu[193];
acadoWorkspace.A[5846] = acadoWorkspace.evHu[194];
acadoWorkspace.A[5847] = acadoWorkspace.evHu[195];
acadoWorkspace.A[5908] = acadoWorkspace.evHu[196];
acadoWorkspace.A[5909] = acadoWorkspace.evHu[197];
acadoWorkspace.A[5968] = acadoWorkspace.evHu[198];
acadoWorkspace.A[5969] = acadoWorkspace.evHu[199];
acadoWorkspace.A[6028] = acadoWorkspace.evHu[200];
acadoWorkspace.A[6029] = acadoWorkspace.evHu[201];
acadoWorkspace.A[6088] = acadoWorkspace.evHu[202];
acadoWorkspace.A[6089] = acadoWorkspace.evHu[203];
acadoWorkspace.A[6148] = acadoWorkspace.evHu[204];
acadoWorkspace.A[6149] = acadoWorkspace.evHu[205];
acadoWorkspace.A[6208] = acadoWorkspace.evHu[206];
acadoWorkspace.A[6209] = acadoWorkspace.evHu[207];
acadoWorkspace.A[6268] = acadoWorkspace.evHu[208];
acadoWorkspace.A[6269] = acadoWorkspace.evHu[209];
acadoWorkspace.A[6330] = acadoWorkspace.evHu[210];
acadoWorkspace.A[6331] = acadoWorkspace.evHu[211];
acadoWorkspace.A[6390] = acadoWorkspace.evHu[212];
acadoWorkspace.A[6391] = acadoWorkspace.evHu[213];
acadoWorkspace.A[6450] = acadoWorkspace.evHu[214];
acadoWorkspace.A[6451] = acadoWorkspace.evHu[215];
acadoWorkspace.A[6510] = acadoWorkspace.evHu[216];
acadoWorkspace.A[6511] = acadoWorkspace.evHu[217];
acadoWorkspace.A[6570] = acadoWorkspace.evHu[218];
acadoWorkspace.A[6571] = acadoWorkspace.evHu[219];
acadoWorkspace.A[6630] = acadoWorkspace.evHu[220];
acadoWorkspace.A[6631] = acadoWorkspace.evHu[221];
acadoWorkspace.A[6690] = acadoWorkspace.evHu[222];
acadoWorkspace.A[6691] = acadoWorkspace.evHu[223];
acadoWorkspace.A[6752] = acadoWorkspace.evHu[224];
acadoWorkspace.A[6753] = acadoWorkspace.evHu[225];
acadoWorkspace.A[6812] = acadoWorkspace.evHu[226];
acadoWorkspace.A[6813] = acadoWorkspace.evHu[227];
acadoWorkspace.A[6872] = acadoWorkspace.evHu[228];
acadoWorkspace.A[6873] = acadoWorkspace.evHu[229];
acadoWorkspace.A[6932] = acadoWorkspace.evHu[230];
acadoWorkspace.A[6933] = acadoWorkspace.evHu[231];
acadoWorkspace.A[6992] = acadoWorkspace.evHu[232];
acadoWorkspace.A[6993] = acadoWorkspace.evHu[233];
acadoWorkspace.A[7052] = acadoWorkspace.evHu[234];
acadoWorkspace.A[7053] = acadoWorkspace.evHu[235];
acadoWorkspace.A[7112] = acadoWorkspace.evHu[236];
acadoWorkspace.A[7113] = acadoWorkspace.evHu[237];
acadoWorkspace.A[7174] = acadoWorkspace.evHu[238];
acadoWorkspace.A[7175] = acadoWorkspace.evHu[239];
acadoWorkspace.A[7234] = acadoWorkspace.evHu[240];
acadoWorkspace.A[7235] = acadoWorkspace.evHu[241];
acadoWorkspace.A[7294] = acadoWorkspace.evHu[242];
acadoWorkspace.A[7295] = acadoWorkspace.evHu[243];
acadoWorkspace.A[7354] = acadoWorkspace.evHu[244];
acadoWorkspace.A[7355] = acadoWorkspace.evHu[245];
acadoWorkspace.A[7414] = acadoWorkspace.evHu[246];
acadoWorkspace.A[7415] = acadoWorkspace.evHu[247];
acadoWorkspace.A[7474] = acadoWorkspace.evHu[248];
acadoWorkspace.A[7475] = acadoWorkspace.evHu[249];
acadoWorkspace.A[7534] = acadoWorkspace.evHu[250];
acadoWorkspace.A[7535] = acadoWorkspace.evHu[251];
acadoWorkspace.A[7596] = acadoWorkspace.evHu[252];
acadoWorkspace.A[7597] = acadoWorkspace.evHu[253];
acadoWorkspace.A[7656] = acadoWorkspace.evHu[254];
acadoWorkspace.A[7657] = acadoWorkspace.evHu[255];
acadoWorkspace.A[7716] = acadoWorkspace.evHu[256];
acadoWorkspace.A[7717] = acadoWorkspace.evHu[257];
acadoWorkspace.A[7776] = acadoWorkspace.evHu[258];
acadoWorkspace.A[7777] = acadoWorkspace.evHu[259];
acadoWorkspace.A[7836] = acadoWorkspace.evHu[260];
acadoWorkspace.A[7837] = acadoWorkspace.evHu[261];
acadoWorkspace.A[7896] = acadoWorkspace.evHu[262];
acadoWorkspace.A[7897] = acadoWorkspace.evHu[263];
acadoWorkspace.A[7956] = acadoWorkspace.evHu[264];
acadoWorkspace.A[7957] = acadoWorkspace.evHu[265];
acadoWorkspace.A[8018] = acadoWorkspace.evHu[266];
acadoWorkspace.A[8019] = acadoWorkspace.evHu[267];
acadoWorkspace.A[8078] = acadoWorkspace.evHu[268];
acadoWorkspace.A[8079] = acadoWorkspace.evHu[269];
acadoWorkspace.A[8138] = acadoWorkspace.evHu[270];
acadoWorkspace.A[8139] = acadoWorkspace.evHu[271];
acadoWorkspace.A[8198] = acadoWorkspace.evHu[272];
acadoWorkspace.A[8199] = acadoWorkspace.evHu[273];
acadoWorkspace.A[8258] = acadoWorkspace.evHu[274];
acadoWorkspace.A[8259] = acadoWorkspace.evHu[275];
acadoWorkspace.A[8318] = acadoWorkspace.evHu[276];
acadoWorkspace.A[8319] = acadoWorkspace.evHu[277];
acadoWorkspace.A[8378] = acadoWorkspace.evHu[278];
acadoWorkspace.A[8379] = acadoWorkspace.evHu[279];
acadoWorkspace.A[8440] = acadoWorkspace.evHu[280];
acadoWorkspace.A[8441] = acadoWorkspace.evHu[281];
acadoWorkspace.A[8500] = acadoWorkspace.evHu[282];
acadoWorkspace.A[8501] = acadoWorkspace.evHu[283];
acadoWorkspace.A[8560] = acadoWorkspace.evHu[284];
acadoWorkspace.A[8561] = acadoWorkspace.evHu[285];
acadoWorkspace.A[8620] = acadoWorkspace.evHu[286];
acadoWorkspace.A[8621] = acadoWorkspace.evHu[287];
acadoWorkspace.A[8680] = acadoWorkspace.evHu[288];
acadoWorkspace.A[8681] = acadoWorkspace.evHu[289];
acadoWorkspace.A[8740] = acadoWorkspace.evHu[290];
acadoWorkspace.A[8741] = acadoWorkspace.evHu[291];
acadoWorkspace.A[8800] = acadoWorkspace.evHu[292];
acadoWorkspace.A[8801] = acadoWorkspace.evHu[293];
acadoWorkspace.A[8862] = acadoWorkspace.evHu[294];
acadoWorkspace.A[8863] = acadoWorkspace.evHu[295];
acadoWorkspace.A[8922] = acadoWorkspace.evHu[296];
acadoWorkspace.A[8923] = acadoWorkspace.evHu[297];
acadoWorkspace.A[8982] = acadoWorkspace.evHu[298];
acadoWorkspace.A[8983] = acadoWorkspace.evHu[299];
acadoWorkspace.A[9042] = acadoWorkspace.evHu[300];
acadoWorkspace.A[9043] = acadoWorkspace.evHu[301];
acadoWorkspace.A[9102] = acadoWorkspace.evHu[302];
acadoWorkspace.A[9103] = acadoWorkspace.evHu[303];
acadoWorkspace.A[9162] = acadoWorkspace.evHu[304];
acadoWorkspace.A[9163] = acadoWorkspace.evHu[305];
acadoWorkspace.A[9222] = acadoWorkspace.evHu[306];
acadoWorkspace.A[9223] = acadoWorkspace.evHu[307];
acadoWorkspace.A[9284] = acadoWorkspace.evHu[308];
acadoWorkspace.A[9285] = acadoWorkspace.evHu[309];
acadoWorkspace.A[9344] = acadoWorkspace.evHu[310];
acadoWorkspace.A[9345] = acadoWorkspace.evHu[311];
acadoWorkspace.A[9404] = acadoWorkspace.evHu[312];
acadoWorkspace.A[9405] = acadoWorkspace.evHu[313];
acadoWorkspace.A[9464] = acadoWorkspace.evHu[314];
acadoWorkspace.A[9465] = acadoWorkspace.evHu[315];
acadoWorkspace.A[9524] = acadoWorkspace.evHu[316];
acadoWorkspace.A[9525] = acadoWorkspace.evHu[317];
acadoWorkspace.A[9584] = acadoWorkspace.evHu[318];
acadoWorkspace.A[9585] = acadoWorkspace.evHu[319];
acadoWorkspace.A[9644] = acadoWorkspace.evHu[320];
acadoWorkspace.A[9645] = acadoWorkspace.evHu[321];
acadoWorkspace.A[9706] = acadoWorkspace.evHu[322];
acadoWorkspace.A[9707] = acadoWorkspace.evHu[323];
acadoWorkspace.A[9766] = acadoWorkspace.evHu[324];
acadoWorkspace.A[9767] = acadoWorkspace.evHu[325];
acadoWorkspace.A[9826] = acadoWorkspace.evHu[326];
acadoWorkspace.A[9827] = acadoWorkspace.evHu[327];
acadoWorkspace.A[9886] = acadoWorkspace.evHu[328];
acadoWorkspace.A[9887] = acadoWorkspace.evHu[329];
acadoWorkspace.A[9946] = acadoWorkspace.evHu[330];
acadoWorkspace.A[9947] = acadoWorkspace.evHu[331];
acadoWorkspace.A[10006] = acadoWorkspace.evHu[332];
acadoWorkspace.A[10007] = acadoWorkspace.evHu[333];
acadoWorkspace.A[10066] = acadoWorkspace.evHu[334];
acadoWorkspace.A[10067] = acadoWorkspace.evHu[335];
acadoWorkspace.A[10128] = acadoWorkspace.evHu[336];
acadoWorkspace.A[10129] = acadoWorkspace.evHu[337];
acadoWorkspace.A[10188] = acadoWorkspace.evHu[338];
acadoWorkspace.A[10189] = acadoWorkspace.evHu[339];
acadoWorkspace.A[10248] = acadoWorkspace.evHu[340];
acadoWorkspace.A[10249] = acadoWorkspace.evHu[341];
acadoWorkspace.A[10308] = acadoWorkspace.evHu[342];
acadoWorkspace.A[10309] = acadoWorkspace.evHu[343];
acadoWorkspace.A[10368] = acadoWorkspace.evHu[344];
acadoWorkspace.A[10369] = acadoWorkspace.evHu[345];
acadoWorkspace.A[10428] = acadoWorkspace.evHu[346];
acadoWorkspace.A[10429] = acadoWorkspace.evHu[347];
acadoWorkspace.A[10488] = acadoWorkspace.evHu[348];
acadoWorkspace.A[10489] = acadoWorkspace.evHu[349];
acadoWorkspace.A[10550] = acadoWorkspace.evHu[350];
acadoWorkspace.A[10551] = acadoWorkspace.evHu[351];
acadoWorkspace.A[10610] = acadoWorkspace.evHu[352];
acadoWorkspace.A[10611] = acadoWorkspace.evHu[353];
acadoWorkspace.A[10670] = acadoWorkspace.evHu[354];
acadoWorkspace.A[10671] = acadoWorkspace.evHu[355];
acadoWorkspace.A[10730] = acadoWorkspace.evHu[356];
acadoWorkspace.A[10731] = acadoWorkspace.evHu[357];
acadoWorkspace.A[10790] = acadoWorkspace.evHu[358];
acadoWorkspace.A[10791] = acadoWorkspace.evHu[359];
acadoWorkspace.A[10850] = acadoWorkspace.evHu[360];
acadoWorkspace.A[10851] = acadoWorkspace.evHu[361];
acadoWorkspace.A[10910] = acadoWorkspace.evHu[362];
acadoWorkspace.A[10911] = acadoWorkspace.evHu[363];
acadoWorkspace.A[10972] = acadoWorkspace.evHu[364];
acadoWorkspace.A[10973] = acadoWorkspace.evHu[365];
acadoWorkspace.A[11032] = acadoWorkspace.evHu[366];
acadoWorkspace.A[11033] = acadoWorkspace.evHu[367];
acadoWorkspace.A[11092] = acadoWorkspace.evHu[368];
acadoWorkspace.A[11093] = acadoWorkspace.evHu[369];
acadoWorkspace.A[11152] = acadoWorkspace.evHu[370];
acadoWorkspace.A[11153] = acadoWorkspace.evHu[371];
acadoWorkspace.A[11212] = acadoWorkspace.evHu[372];
acadoWorkspace.A[11213] = acadoWorkspace.evHu[373];
acadoWorkspace.A[11272] = acadoWorkspace.evHu[374];
acadoWorkspace.A[11273] = acadoWorkspace.evHu[375];
acadoWorkspace.A[11332] = acadoWorkspace.evHu[376];
acadoWorkspace.A[11333] = acadoWorkspace.evHu[377];
acadoWorkspace.A[11394] = acadoWorkspace.evHu[378];
acadoWorkspace.A[11395] = acadoWorkspace.evHu[379];
acadoWorkspace.A[11454] = acadoWorkspace.evHu[380];
acadoWorkspace.A[11455] = acadoWorkspace.evHu[381];
acadoWorkspace.A[11514] = acadoWorkspace.evHu[382];
acadoWorkspace.A[11515] = acadoWorkspace.evHu[383];
acadoWorkspace.A[11574] = acadoWorkspace.evHu[384];
acadoWorkspace.A[11575] = acadoWorkspace.evHu[385];
acadoWorkspace.A[11634] = acadoWorkspace.evHu[386];
acadoWorkspace.A[11635] = acadoWorkspace.evHu[387];
acadoWorkspace.A[11694] = acadoWorkspace.evHu[388];
acadoWorkspace.A[11695] = acadoWorkspace.evHu[389];
acadoWorkspace.A[11754] = acadoWorkspace.evHu[390];
acadoWorkspace.A[11755] = acadoWorkspace.evHu[391];
acadoWorkspace.A[11816] = acadoWorkspace.evHu[392];
acadoWorkspace.A[11817] = acadoWorkspace.evHu[393];
acadoWorkspace.A[11876] = acadoWorkspace.evHu[394];
acadoWorkspace.A[11877] = acadoWorkspace.evHu[395];
acadoWorkspace.A[11936] = acadoWorkspace.evHu[396];
acadoWorkspace.A[11937] = acadoWorkspace.evHu[397];
acadoWorkspace.A[11996] = acadoWorkspace.evHu[398];
acadoWorkspace.A[11997] = acadoWorkspace.evHu[399];
acadoWorkspace.A[12056] = acadoWorkspace.evHu[400];
acadoWorkspace.A[12057] = acadoWorkspace.evHu[401];
acadoWorkspace.A[12116] = acadoWorkspace.evHu[402];
acadoWorkspace.A[12117] = acadoWorkspace.evHu[403];
acadoWorkspace.A[12176] = acadoWorkspace.evHu[404];
acadoWorkspace.A[12177] = acadoWorkspace.evHu[405];
acadoWorkspace.A[12238] = acadoWorkspace.evHu[406];
acadoWorkspace.A[12239] = acadoWorkspace.evHu[407];
acadoWorkspace.A[12298] = acadoWorkspace.evHu[408];
acadoWorkspace.A[12299] = acadoWorkspace.evHu[409];
acadoWorkspace.A[12358] = acadoWorkspace.evHu[410];
acadoWorkspace.A[12359] = acadoWorkspace.evHu[411];
acadoWorkspace.A[12418] = acadoWorkspace.evHu[412];
acadoWorkspace.A[12419] = acadoWorkspace.evHu[413];
acadoWorkspace.A[12478] = acadoWorkspace.evHu[414];
acadoWorkspace.A[12479] = acadoWorkspace.evHu[415];
acadoWorkspace.A[12538] = acadoWorkspace.evHu[416];
acadoWorkspace.A[12539] = acadoWorkspace.evHu[417];
acadoWorkspace.A[12598] = acadoWorkspace.evHu[418];
acadoWorkspace.A[12599] = acadoWorkspace.evHu[419];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = acadoVariables.lbAValues[30] - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = acadoVariables.lbAValues[31] - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = acadoVariables.lbAValues[32] - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = acadoVariables.lbAValues[33] - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = acadoVariables.lbAValues[34] - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = acadoVariables.lbAValues[35] - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = acadoVariables.lbAValues[36] - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = acadoVariables.lbAValues[37] - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = acadoVariables.lbAValues[38] - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = acadoVariables.lbAValues[39] - acadoWorkspace.evH[39];
acadoWorkspace.lbA[40] = acadoVariables.lbAValues[40] - acadoWorkspace.evH[40];
acadoWorkspace.lbA[41] = acadoVariables.lbAValues[41] - acadoWorkspace.evH[41];
acadoWorkspace.lbA[42] = acadoVariables.lbAValues[42] - acadoWorkspace.evH[42];
acadoWorkspace.lbA[43] = acadoVariables.lbAValues[43] - acadoWorkspace.evH[43];
acadoWorkspace.lbA[44] = acadoVariables.lbAValues[44] - acadoWorkspace.evH[44];
acadoWorkspace.lbA[45] = acadoVariables.lbAValues[45] - acadoWorkspace.evH[45];
acadoWorkspace.lbA[46] = acadoVariables.lbAValues[46] - acadoWorkspace.evH[46];
acadoWorkspace.lbA[47] = acadoVariables.lbAValues[47] - acadoWorkspace.evH[47];
acadoWorkspace.lbA[48] = acadoVariables.lbAValues[48] - acadoWorkspace.evH[48];
acadoWorkspace.lbA[49] = acadoVariables.lbAValues[49] - acadoWorkspace.evH[49];
acadoWorkspace.lbA[50] = acadoVariables.lbAValues[50] - acadoWorkspace.evH[50];
acadoWorkspace.lbA[51] = acadoVariables.lbAValues[51] - acadoWorkspace.evH[51];
acadoWorkspace.lbA[52] = acadoVariables.lbAValues[52] - acadoWorkspace.evH[52];
acadoWorkspace.lbA[53] = acadoVariables.lbAValues[53] - acadoWorkspace.evH[53];
acadoWorkspace.lbA[54] = acadoVariables.lbAValues[54] - acadoWorkspace.evH[54];
acadoWorkspace.lbA[55] = acadoVariables.lbAValues[55] - acadoWorkspace.evH[55];
acadoWorkspace.lbA[56] = acadoVariables.lbAValues[56] - acadoWorkspace.evH[56];
acadoWorkspace.lbA[57] = acadoVariables.lbAValues[57] - acadoWorkspace.evH[57];
acadoWorkspace.lbA[58] = acadoVariables.lbAValues[58] - acadoWorkspace.evH[58];
acadoWorkspace.lbA[59] = acadoVariables.lbAValues[59] - acadoWorkspace.evH[59];
acadoWorkspace.lbA[60] = acadoVariables.lbAValues[60] - acadoWorkspace.evH[60];
acadoWorkspace.lbA[61] = acadoVariables.lbAValues[61] - acadoWorkspace.evH[61];
acadoWorkspace.lbA[62] = acadoVariables.lbAValues[62] - acadoWorkspace.evH[62];
acadoWorkspace.lbA[63] = acadoVariables.lbAValues[63] - acadoWorkspace.evH[63];
acadoWorkspace.lbA[64] = acadoVariables.lbAValues[64] - acadoWorkspace.evH[64];
acadoWorkspace.lbA[65] = acadoVariables.lbAValues[65] - acadoWorkspace.evH[65];
acadoWorkspace.lbA[66] = acadoVariables.lbAValues[66] - acadoWorkspace.evH[66];
acadoWorkspace.lbA[67] = acadoVariables.lbAValues[67] - acadoWorkspace.evH[67];
acadoWorkspace.lbA[68] = acadoVariables.lbAValues[68] - acadoWorkspace.evH[68];
acadoWorkspace.lbA[69] = acadoVariables.lbAValues[69] - acadoWorkspace.evH[69];
acadoWorkspace.lbA[70] = acadoVariables.lbAValues[70] - acadoWorkspace.evH[70];
acadoWorkspace.lbA[71] = acadoVariables.lbAValues[71] - acadoWorkspace.evH[71];
acadoWorkspace.lbA[72] = acadoVariables.lbAValues[72] - acadoWorkspace.evH[72];
acadoWorkspace.lbA[73] = acadoVariables.lbAValues[73] - acadoWorkspace.evH[73];
acadoWorkspace.lbA[74] = acadoVariables.lbAValues[74] - acadoWorkspace.evH[74];
acadoWorkspace.lbA[75] = acadoVariables.lbAValues[75] - acadoWorkspace.evH[75];
acadoWorkspace.lbA[76] = acadoVariables.lbAValues[76] - acadoWorkspace.evH[76];
acadoWorkspace.lbA[77] = acadoVariables.lbAValues[77] - acadoWorkspace.evH[77];
acadoWorkspace.lbA[78] = acadoVariables.lbAValues[78] - acadoWorkspace.evH[78];
acadoWorkspace.lbA[79] = acadoVariables.lbAValues[79] - acadoWorkspace.evH[79];
acadoWorkspace.lbA[80] = acadoVariables.lbAValues[80] - acadoWorkspace.evH[80];
acadoWorkspace.lbA[81] = acadoVariables.lbAValues[81] - acadoWorkspace.evH[81];
acadoWorkspace.lbA[82] = acadoVariables.lbAValues[82] - acadoWorkspace.evH[82];
acadoWorkspace.lbA[83] = acadoVariables.lbAValues[83] - acadoWorkspace.evH[83];
acadoWorkspace.lbA[84] = acadoVariables.lbAValues[84] - acadoWorkspace.evH[84];
acadoWorkspace.lbA[85] = acadoVariables.lbAValues[85] - acadoWorkspace.evH[85];
acadoWorkspace.lbA[86] = acadoVariables.lbAValues[86] - acadoWorkspace.evH[86];
acadoWorkspace.lbA[87] = acadoVariables.lbAValues[87] - acadoWorkspace.evH[87];
acadoWorkspace.lbA[88] = acadoVariables.lbAValues[88] - acadoWorkspace.evH[88];
acadoWorkspace.lbA[89] = acadoVariables.lbAValues[89] - acadoWorkspace.evH[89];
acadoWorkspace.lbA[90] = acadoVariables.lbAValues[90] - acadoWorkspace.evH[90];
acadoWorkspace.lbA[91] = acadoVariables.lbAValues[91] - acadoWorkspace.evH[91];
acadoWorkspace.lbA[92] = acadoVariables.lbAValues[92] - acadoWorkspace.evH[92];
acadoWorkspace.lbA[93] = acadoVariables.lbAValues[93] - acadoWorkspace.evH[93];
acadoWorkspace.lbA[94] = acadoVariables.lbAValues[94] - acadoWorkspace.evH[94];
acadoWorkspace.lbA[95] = acadoVariables.lbAValues[95] - acadoWorkspace.evH[95];
acadoWorkspace.lbA[96] = acadoVariables.lbAValues[96] - acadoWorkspace.evH[96];
acadoWorkspace.lbA[97] = acadoVariables.lbAValues[97] - acadoWorkspace.evH[97];
acadoWorkspace.lbA[98] = acadoVariables.lbAValues[98] - acadoWorkspace.evH[98];
acadoWorkspace.lbA[99] = acadoVariables.lbAValues[99] - acadoWorkspace.evH[99];
acadoWorkspace.lbA[100] = acadoVariables.lbAValues[100] - acadoWorkspace.evH[100];
acadoWorkspace.lbA[101] = acadoVariables.lbAValues[101] - acadoWorkspace.evH[101];
acadoWorkspace.lbA[102] = acadoVariables.lbAValues[102] - acadoWorkspace.evH[102];
acadoWorkspace.lbA[103] = acadoVariables.lbAValues[103] - acadoWorkspace.evH[103];
acadoWorkspace.lbA[104] = acadoVariables.lbAValues[104] - acadoWorkspace.evH[104];
acadoWorkspace.lbA[105] = acadoVariables.lbAValues[105] - acadoWorkspace.evH[105];
acadoWorkspace.lbA[106] = acadoVariables.lbAValues[106] - acadoWorkspace.evH[106];
acadoWorkspace.lbA[107] = acadoVariables.lbAValues[107] - acadoWorkspace.evH[107];
acadoWorkspace.lbA[108] = acadoVariables.lbAValues[108] - acadoWorkspace.evH[108];
acadoWorkspace.lbA[109] = acadoVariables.lbAValues[109] - acadoWorkspace.evH[109];
acadoWorkspace.lbA[110] = acadoVariables.lbAValues[110] - acadoWorkspace.evH[110];
acadoWorkspace.lbA[111] = acadoVariables.lbAValues[111] - acadoWorkspace.evH[111];
acadoWorkspace.lbA[112] = acadoVariables.lbAValues[112] - acadoWorkspace.evH[112];
acadoWorkspace.lbA[113] = acadoVariables.lbAValues[113] - acadoWorkspace.evH[113];
acadoWorkspace.lbA[114] = acadoVariables.lbAValues[114] - acadoWorkspace.evH[114];
acadoWorkspace.lbA[115] = acadoVariables.lbAValues[115] - acadoWorkspace.evH[115];
acadoWorkspace.lbA[116] = acadoVariables.lbAValues[116] - acadoWorkspace.evH[116];
acadoWorkspace.lbA[117] = acadoVariables.lbAValues[117] - acadoWorkspace.evH[117];
acadoWorkspace.lbA[118] = acadoVariables.lbAValues[118] - acadoWorkspace.evH[118];
acadoWorkspace.lbA[119] = acadoVariables.lbAValues[119] - acadoWorkspace.evH[119];
acadoWorkspace.lbA[120] = acadoVariables.lbAValues[120] - acadoWorkspace.evH[120];
acadoWorkspace.lbA[121] = acadoVariables.lbAValues[121] - acadoWorkspace.evH[121];
acadoWorkspace.lbA[122] = acadoVariables.lbAValues[122] - acadoWorkspace.evH[122];
acadoWorkspace.lbA[123] = acadoVariables.lbAValues[123] - acadoWorkspace.evH[123];
acadoWorkspace.lbA[124] = acadoVariables.lbAValues[124] - acadoWorkspace.evH[124];
acadoWorkspace.lbA[125] = acadoVariables.lbAValues[125] - acadoWorkspace.evH[125];
acadoWorkspace.lbA[126] = acadoVariables.lbAValues[126] - acadoWorkspace.evH[126];
acadoWorkspace.lbA[127] = acadoVariables.lbAValues[127] - acadoWorkspace.evH[127];
acadoWorkspace.lbA[128] = acadoVariables.lbAValues[128] - acadoWorkspace.evH[128];
acadoWorkspace.lbA[129] = acadoVariables.lbAValues[129] - acadoWorkspace.evH[129];
acadoWorkspace.lbA[130] = acadoVariables.lbAValues[130] - acadoWorkspace.evH[130];
acadoWorkspace.lbA[131] = acadoVariables.lbAValues[131] - acadoWorkspace.evH[131];
acadoWorkspace.lbA[132] = acadoVariables.lbAValues[132] - acadoWorkspace.evH[132];
acadoWorkspace.lbA[133] = acadoVariables.lbAValues[133] - acadoWorkspace.evH[133];
acadoWorkspace.lbA[134] = acadoVariables.lbAValues[134] - acadoWorkspace.evH[134];
acadoWorkspace.lbA[135] = acadoVariables.lbAValues[135] - acadoWorkspace.evH[135];
acadoWorkspace.lbA[136] = acadoVariables.lbAValues[136] - acadoWorkspace.evH[136];
acadoWorkspace.lbA[137] = acadoVariables.lbAValues[137] - acadoWorkspace.evH[137];
acadoWorkspace.lbA[138] = acadoVariables.lbAValues[138] - acadoWorkspace.evH[138];
acadoWorkspace.lbA[139] = acadoVariables.lbAValues[139] - acadoWorkspace.evH[139];
acadoWorkspace.lbA[140] = acadoVariables.lbAValues[140] - acadoWorkspace.evH[140];
acadoWorkspace.lbA[141] = acadoVariables.lbAValues[141] - acadoWorkspace.evH[141];
acadoWorkspace.lbA[142] = acadoVariables.lbAValues[142] - acadoWorkspace.evH[142];
acadoWorkspace.lbA[143] = acadoVariables.lbAValues[143] - acadoWorkspace.evH[143];
acadoWorkspace.lbA[144] = acadoVariables.lbAValues[144] - acadoWorkspace.evH[144];
acadoWorkspace.lbA[145] = acadoVariables.lbAValues[145] - acadoWorkspace.evH[145];
acadoWorkspace.lbA[146] = acadoVariables.lbAValues[146] - acadoWorkspace.evH[146];
acadoWorkspace.lbA[147] = acadoVariables.lbAValues[147] - acadoWorkspace.evH[147];
acadoWorkspace.lbA[148] = acadoVariables.lbAValues[148] - acadoWorkspace.evH[148];
acadoWorkspace.lbA[149] = acadoVariables.lbAValues[149] - acadoWorkspace.evH[149];
acadoWorkspace.lbA[150] = acadoVariables.lbAValues[150] - acadoWorkspace.evH[150];
acadoWorkspace.lbA[151] = acadoVariables.lbAValues[151] - acadoWorkspace.evH[151];
acadoWorkspace.lbA[152] = acadoVariables.lbAValues[152] - acadoWorkspace.evH[152];
acadoWorkspace.lbA[153] = acadoVariables.lbAValues[153] - acadoWorkspace.evH[153];
acadoWorkspace.lbA[154] = acadoVariables.lbAValues[154] - acadoWorkspace.evH[154];
acadoWorkspace.lbA[155] = acadoVariables.lbAValues[155] - acadoWorkspace.evH[155];
acadoWorkspace.lbA[156] = acadoVariables.lbAValues[156] - acadoWorkspace.evH[156];
acadoWorkspace.lbA[157] = acadoVariables.lbAValues[157] - acadoWorkspace.evH[157];
acadoWorkspace.lbA[158] = acadoVariables.lbAValues[158] - acadoWorkspace.evH[158];
acadoWorkspace.lbA[159] = acadoVariables.lbAValues[159] - acadoWorkspace.evH[159];
acadoWorkspace.lbA[160] = acadoVariables.lbAValues[160] - acadoWorkspace.evH[160];
acadoWorkspace.lbA[161] = acadoVariables.lbAValues[161] - acadoWorkspace.evH[161];
acadoWorkspace.lbA[162] = acadoVariables.lbAValues[162] - acadoWorkspace.evH[162];
acadoWorkspace.lbA[163] = acadoVariables.lbAValues[163] - acadoWorkspace.evH[163];
acadoWorkspace.lbA[164] = acadoVariables.lbAValues[164] - acadoWorkspace.evH[164];
acadoWorkspace.lbA[165] = acadoVariables.lbAValues[165] - acadoWorkspace.evH[165];
acadoWorkspace.lbA[166] = acadoVariables.lbAValues[166] - acadoWorkspace.evH[166];
acadoWorkspace.lbA[167] = acadoVariables.lbAValues[167] - acadoWorkspace.evH[167];
acadoWorkspace.lbA[168] = acadoVariables.lbAValues[168] - acadoWorkspace.evH[168];
acadoWorkspace.lbA[169] = acadoVariables.lbAValues[169] - acadoWorkspace.evH[169];
acadoWorkspace.lbA[170] = acadoVariables.lbAValues[170] - acadoWorkspace.evH[170];
acadoWorkspace.lbA[171] = acadoVariables.lbAValues[171] - acadoWorkspace.evH[171];
acadoWorkspace.lbA[172] = acadoVariables.lbAValues[172] - acadoWorkspace.evH[172];
acadoWorkspace.lbA[173] = acadoVariables.lbAValues[173] - acadoWorkspace.evH[173];
acadoWorkspace.lbA[174] = acadoVariables.lbAValues[174] - acadoWorkspace.evH[174];
acadoWorkspace.lbA[175] = acadoVariables.lbAValues[175] - acadoWorkspace.evH[175];
acadoWorkspace.lbA[176] = acadoVariables.lbAValues[176] - acadoWorkspace.evH[176];
acadoWorkspace.lbA[177] = acadoVariables.lbAValues[177] - acadoWorkspace.evH[177];
acadoWorkspace.lbA[178] = acadoVariables.lbAValues[178] - acadoWorkspace.evH[178];
acadoWorkspace.lbA[179] = acadoVariables.lbAValues[179] - acadoWorkspace.evH[179];
acadoWorkspace.lbA[180] = acadoVariables.lbAValues[180] - acadoWorkspace.evH[180];
acadoWorkspace.lbA[181] = acadoVariables.lbAValues[181] - acadoWorkspace.evH[181];
acadoWorkspace.lbA[182] = acadoVariables.lbAValues[182] - acadoWorkspace.evH[182];
acadoWorkspace.lbA[183] = acadoVariables.lbAValues[183] - acadoWorkspace.evH[183];
acadoWorkspace.lbA[184] = acadoVariables.lbAValues[184] - acadoWorkspace.evH[184];
acadoWorkspace.lbA[185] = acadoVariables.lbAValues[185] - acadoWorkspace.evH[185];
acadoWorkspace.lbA[186] = acadoVariables.lbAValues[186] - acadoWorkspace.evH[186];
acadoWorkspace.lbA[187] = acadoVariables.lbAValues[187] - acadoWorkspace.evH[187];
acadoWorkspace.lbA[188] = acadoVariables.lbAValues[188] - acadoWorkspace.evH[188];
acadoWorkspace.lbA[189] = acadoVariables.lbAValues[189] - acadoWorkspace.evH[189];
acadoWorkspace.lbA[190] = acadoVariables.lbAValues[190] - acadoWorkspace.evH[190];
acadoWorkspace.lbA[191] = acadoVariables.lbAValues[191] - acadoWorkspace.evH[191];
acadoWorkspace.lbA[192] = acadoVariables.lbAValues[192] - acadoWorkspace.evH[192];
acadoWorkspace.lbA[193] = acadoVariables.lbAValues[193] - acadoWorkspace.evH[193];
acadoWorkspace.lbA[194] = acadoVariables.lbAValues[194] - acadoWorkspace.evH[194];
acadoWorkspace.lbA[195] = acadoVariables.lbAValues[195] - acadoWorkspace.evH[195];
acadoWorkspace.lbA[196] = acadoVariables.lbAValues[196] - acadoWorkspace.evH[196];
acadoWorkspace.lbA[197] = acadoVariables.lbAValues[197] - acadoWorkspace.evH[197];
acadoWorkspace.lbA[198] = acadoVariables.lbAValues[198] - acadoWorkspace.evH[198];
acadoWorkspace.lbA[199] = acadoVariables.lbAValues[199] - acadoWorkspace.evH[199];
acadoWorkspace.lbA[200] = acadoVariables.lbAValues[200] - acadoWorkspace.evH[200];
acadoWorkspace.lbA[201] = acadoVariables.lbAValues[201] - acadoWorkspace.evH[201];
acadoWorkspace.lbA[202] = acadoVariables.lbAValues[202] - acadoWorkspace.evH[202];
acadoWorkspace.lbA[203] = acadoVariables.lbAValues[203] - acadoWorkspace.evH[203];
acadoWorkspace.lbA[204] = acadoVariables.lbAValues[204] - acadoWorkspace.evH[204];
acadoWorkspace.lbA[205] = acadoVariables.lbAValues[205] - acadoWorkspace.evH[205];
acadoWorkspace.lbA[206] = acadoVariables.lbAValues[206] - acadoWorkspace.evH[206];
acadoWorkspace.lbA[207] = acadoVariables.lbAValues[207] - acadoWorkspace.evH[207];
acadoWorkspace.lbA[208] = acadoVariables.lbAValues[208] - acadoWorkspace.evH[208];
acadoWorkspace.lbA[209] = acadoVariables.lbAValues[209] - acadoWorkspace.evH[209];

acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = acadoVariables.ubAValues[30] - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = acadoVariables.ubAValues[31] - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = acadoVariables.ubAValues[32] - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = acadoVariables.ubAValues[33] - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = acadoVariables.ubAValues[34] - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = acadoVariables.ubAValues[35] - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = acadoVariables.ubAValues[36] - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = acadoVariables.ubAValues[37] - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = acadoVariables.ubAValues[38] - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = acadoVariables.ubAValues[39] - acadoWorkspace.evH[39];
acadoWorkspace.ubA[40] = acadoVariables.ubAValues[40] - acadoWorkspace.evH[40];
acadoWorkspace.ubA[41] = acadoVariables.ubAValues[41] - acadoWorkspace.evH[41];
acadoWorkspace.ubA[42] = acadoVariables.ubAValues[42] - acadoWorkspace.evH[42];
acadoWorkspace.ubA[43] = acadoVariables.ubAValues[43] - acadoWorkspace.evH[43];
acadoWorkspace.ubA[44] = acadoVariables.ubAValues[44] - acadoWorkspace.evH[44];
acadoWorkspace.ubA[45] = acadoVariables.ubAValues[45] - acadoWorkspace.evH[45];
acadoWorkspace.ubA[46] = acadoVariables.ubAValues[46] - acadoWorkspace.evH[46];
acadoWorkspace.ubA[47] = acadoVariables.ubAValues[47] - acadoWorkspace.evH[47];
acadoWorkspace.ubA[48] = acadoVariables.ubAValues[48] - acadoWorkspace.evH[48];
acadoWorkspace.ubA[49] = acadoVariables.ubAValues[49] - acadoWorkspace.evH[49];
acadoWorkspace.ubA[50] = acadoVariables.ubAValues[50] - acadoWorkspace.evH[50];
acadoWorkspace.ubA[51] = acadoVariables.ubAValues[51] - acadoWorkspace.evH[51];
acadoWorkspace.ubA[52] = acadoVariables.ubAValues[52] - acadoWorkspace.evH[52];
acadoWorkspace.ubA[53] = acadoVariables.ubAValues[53] - acadoWorkspace.evH[53];
acadoWorkspace.ubA[54] = acadoVariables.ubAValues[54] - acadoWorkspace.evH[54];
acadoWorkspace.ubA[55] = acadoVariables.ubAValues[55] - acadoWorkspace.evH[55];
acadoWorkspace.ubA[56] = acadoVariables.ubAValues[56] - acadoWorkspace.evH[56];
acadoWorkspace.ubA[57] = acadoVariables.ubAValues[57] - acadoWorkspace.evH[57];
acadoWorkspace.ubA[58] = acadoVariables.ubAValues[58] - acadoWorkspace.evH[58];
acadoWorkspace.ubA[59] = acadoVariables.ubAValues[59] - acadoWorkspace.evH[59];
acadoWorkspace.ubA[60] = acadoVariables.ubAValues[60] - acadoWorkspace.evH[60];
acadoWorkspace.ubA[61] = acadoVariables.ubAValues[61] - acadoWorkspace.evH[61];
acadoWorkspace.ubA[62] = acadoVariables.ubAValues[62] - acadoWorkspace.evH[62];
acadoWorkspace.ubA[63] = acadoVariables.ubAValues[63] - acadoWorkspace.evH[63];
acadoWorkspace.ubA[64] = acadoVariables.ubAValues[64] - acadoWorkspace.evH[64];
acadoWorkspace.ubA[65] = acadoVariables.ubAValues[65] - acadoWorkspace.evH[65];
acadoWorkspace.ubA[66] = acadoVariables.ubAValues[66] - acadoWorkspace.evH[66];
acadoWorkspace.ubA[67] = acadoVariables.ubAValues[67] - acadoWorkspace.evH[67];
acadoWorkspace.ubA[68] = acadoVariables.ubAValues[68] - acadoWorkspace.evH[68];
acadoWorkspace.ubA[69] = acadoVariables.ubAValues[69] - acadoWorkspace.evH[69];
acadoWorkspace.ubA[70] = acadoVariables.ubAValues[70] - acadoWorkspace.evH[70];
acadoWorkspace.ubA[71] = acadoVariables.ubAValues[71] - acadoWorkspace.evH[71];
acadoWorkspace.ubA[72] = acadoVariables.ubAValues[72] - acadoWorkspace.evH[72];
acadoWorkspace.ubA[73] = acadoVariables.ubAValues[73] - acadoWorkspace.evH[73];
acadoWorkspace.ubA[74] = acadoVariables.ubAValues[74] - acadoWorkspace.evH[74];
acadoWorkspace.ubA[75] = acadoVariables.ubAValues[75] - acadoWorkspace.evH[75];
acadoWorkspace.ubA[76] = acadoVariables.ubAValues[76] - acadoWorkspace.evH[76];
acadoWorkspace.ubA[77] = acadoVariables.ubAValues[77] - acadoWorkspace.evH[77];
acadoWorkspace.ubA[78] = acadoVariables.ubAValues[78] - acadoWorkspace.evH[78];
acadoWorkspace.ubA[79] = acadoVariables.ubAValues[79] - acadoWorkspace.evH[79];
acadoWorkspace.ubA[80] = acadoVariables.ubAValues[80] - acadoWorkspace.evH[80];
acadoWorkspace.ubA[81] = acadoVariables.ubAValues[81] - acadoWorkspace.evH[81];
acadoWorkspace.ubA[82] = acadoVariables.ubAValues[82] - acadoWorkspace.evH[82];
acadoWorkspace.ubA[83] = acadoVariables.ubAValues[83] - acadoWorkspace.evH[83];
acadoWorkspace.ubA[84] = acadoVariables.ubAValues[84] - acadoWorkspace.evH[84];
acadoWorkspace.ubA[85] = acadoVariables.ubAValues[85] - acadoWorkspace.evH[85];
acadoWorkspace.ubA[86] = acadoVariables.ubAValues[86] - acadoWorkspace.evH[86];
acadoWorkspace.ubA[87] = acadoVariables.ubAValues[87] - acadoWorkspace.evH[87];
acadoWorkspace.ubA[88] = acadoVariables.ubAValues[88] - acadoWorkspace.evH[88];
acadoWorkspace.ubA[89] = acadoVariables.ubAValues[89] - acadoWorkspace.evH[89];
acadoWorkspace.ubA[90] = acadoVariables.ubAValues[90] - acadoWorkspace.evH[90];
acadoWorkspace.ubA[91] = acadoVariables.ubAValues[91] - acadoWorkspace.evH[91];
acadoWorkspace.ubA[92] = acadoVariables.ubAValues[92] - acadoWorkspace.evH[92];
acadoWorkspace.ubA[93] = acadoVariables.ubAValues[93] - acadoWorkspace.evH[93];
acadoWorkspace.ubA[94] = acadoVariables.ubAValues[94] - acadoWorkspace.evH[94];
acadoWorkspace.ubA[95] = acadoVariables.ubAValues[95] - acadoWorkspace.evH[95];
acadoWorkspace.ubA[96] = acadoVariables.ubAValues[96] - acadoWorkspace.evH[96];
acadoWorkspace.ubA[97] = acadoVariables.ubAValues[97] - acadoWorkspace.evH[97];
acadoWorkspace.ubA[98] = acadoVariables.ubAValues[98] - acadoWorkspace.evH[98];
acadoWorkspace.ubA[99] = acadoVariables.ubAValues[99] - acadoWorkspace.evH[99];
acadoWorkspace.ubA[100] = acadoVariables.ubAValues[100] - acadoWorkspace.evH[100];
acadoWorkspace.ubA[101] = acadoVariables.ubAValues[101] - acadoWorkspace.evH[101];
acadoWorkspace.ubA[102] = acadoVariables.ubAValues[102] - acadoWorkspace.evH[102];
acadoWorkspace.ubA[103] = acadoVariables.ubAValues[103] - acadoWorkspace.evH[103];
acadoWorkspace.ubA[104] = acadoVariables.ubAValues[104] - acadoWorkspace.evH[104];
acadoWorkspace.ubA[105] = acadoVariables.ubAValues[105] - acadoWorkspace.evH[105];
acadoWorkspace.ubA[106] = acadoVariables.ubAValues[106] - acadoWorkspace.evH[106];
acadoWorkspace.ubA[107] = acadoVariables.ubAValues[107] - acadoWorkspace.evH[107];
acadoWorkspace.ubA[108] = acadoVariables.ubAValues[108] - acadoWorkspace.evH[108];
acadoWorkspace.ubA[109] = acadoVariables.ubAValues[109] - acadoWorkspace.evH[109];
acadoWorkspace.ubA[110] = acadoVariables.ubAValues[110] - acadoWorkspace.evH[110];
acadoWorkspace.ubA[111] = acadoVariables.ubAValues[111] - acadoWorkspace.evH[111];
acadoWorkspace.ubA[112] = acadoVariables.ubAValues[112] - acadoWorkspace.evH[112];
acadoWorkspace.ubA[113] = acadoVariables.ubAValues[113] - acadoWorkspace.evH[113];
acadoWorkspace.ubA[114] = acadoVariables.ubAValues[114] - acadoWorkspace.evH[114];
acadoWorkspace.ubA[115] = acadoVariables.ubAValues[115] - acadoWorkspace.evH[115];
acadoWorkspace.ubA[116] = acadoVariables.ubAValues[116] - acadoWorkspace.evH[116];
acadoWorkspace.ubA[117] = acadoVariables.ubAValues[117] - acadoWorkspace.evH[117];
acadoWorkspace.ubA[118] = acadoVariables.ubAValues[118] - acadoWorkspace.evH[118];
acadoWorkspace.ubA[119] = acadoVariables.ubAValues[119] - acadoWorkspace.evH[119];
acadoWorkspace.ubA[120] = acadoVariables.ubAValues[120] - acadoWorkspace.evH[120];
acadoWorkspace.ubA[121] = acadoVariables.ubAValues[121] - acadoWorkspace.evH[121];
acadoWorkspace.ubA[122] = acadoVariables.ubAValues[122] - acadoWorkspace.evH[122];
acadoWorkspace.ubA[123] = acadoVariables.ubAValues[123] - acadoWorkspace.evH[123];
acadoWorkspace.ubA[124] = acadoVariables.ubAValues[124] - acadoWorkspace.evH[124];
acadoWorkspace.ubA[125] = acadoVariables.ubAValues[125] - acadoWorkspace.evH[125];
acadoWorkspace.ubA[126] = acadoVariables.ubAValues[126] - acadoWorkspace.evH[126];
acadoWorkspace.ubA[127] = acadoVariables.ubAValues[127] - acadoWorkspace.evH[127];
acadoWorkspace.ubA[128] = acadoVariables.ubAValues[128] - acadoWorkspace.evH[128];
acadoWorkspace.ubA[129] = acadoVariables.ubAValues[129] - acadoWorkspace.evH[129];
acadoWorkspace.ubA[130] = acadoVariables.ubAValues[130] - acadoWorkspace.evH[130];
acadoWorkspace.ubA[131] = acadoVariables.ubAValues[131] - acadoWorkspace.evH[131];
acadoWorkspace.ubA[132] = acadoVariables.ubAValues[132] - acadoWorkspace.evH[132];
acadoWorkspace.ubA[133] = acadoVariables.ubAValues[133] - acadoWorkspace.evH[133];
acadoWorkspace.ubA[134] = acadoVariables.ubAValues[134] - acadoWorkspace.evH[134];
acadoWorkspace.ubA[135] = acadoVariables.ubAValues[135] - acadoWorkspace.evH[135];
acadoWorkspace.ubA[136] = acadoVariables.ubAValues[136] - acadoWorkspace.evH[136];
acadoWorkspace.ubA[137] = acadoVariables.ubAValues[137] - acadoWorkspace.evH[137];
acadoWorkspace.ubA[138] = acadoVariables.ubAValues[138] - acadoWorkspace.evH[138];
acadoWorkspace.ubA[139] = acadoVariables.ubAValues[139] - acadoWorkspace.evH[139];
acadoWorkspace.ubA[140] = acadoVariables.ubAValues[140] - acadoWorkspace.evH[140];
acadoWorkspace.ubA[141] = acadoVariables.ubAValues[141] - acadoWorkspace.evH[141];
acadoWorkspace.ubA[142] = acadoVariables.ubAValues[142] - acadoWorkspace.evH[142];
acadoWorkspace.ubA[143] = acadoVariables.ubAValues[143] - acadoWorkspace.evH[143];
acadoWorkspace.ubA[144] = acadoVariables.ubAValues[144] - acadoWorkspace.evH[144];
acadoWorkspace.ubA[145] = acadoVariables.ubAValues[145] - acadoWorkspace.evH[145];
acadoWorkspace.ubA[146] = acadoVariables.ubAValues[146] - acadoWorkspace.evH[146];
acadoWorkspace.ubA[147] = acadoVariables.ubAValues[147] - acadoWorkspace.evH[147];
acadoWorkspace.ubA[148] = acadoVariables.ubAValues[148] - acadoWorkspace.evH[148];
acadoWorkspace.ubA[149] = acadoVariables.ubAValues[149] - acadoWorkspace.evH[149];
acadoWorkspace.ubA[150] = acadoVariables.ubAValues[150] - acadoWorkspace.evH[150];
acadoWorkspace.ubA[151] = acadoVariables.ubAValues[151] - acadoWorkspace.evH[151];
acadoWorkspace.ubA[152] = acadoVariables.ubAValues[152] - acadoWorkspace.evH[152];
acadoWorkspace.ubA[153] = acadoVariables.ubAValues[153] - acadoWorkspace.evH[153];
acadoWorkspace.ubA[154] = acadoVariables.ubAValues[154] - acadoWorkspace.evH[154];
acadoWorkspace.ubA[155] = acadoVariables.ubAValues[155] - acadoWorkspace.evH[155];
acadoWorkspace.ubA[156] = acadoVariables.ubAValues[156] - acadoWorkspace.evH[156];
acadoWorkspace.ubA[157] = acadoVariables.ubAValues[157] - acadoWorkspace.evH[157];
acadoWorkspace.ubA[158] = acadoVariables.ubAValues[158] - acadoWorkspace.evH[158];
acadoWorkspace.ubA[159] = acadoVariables.ubAValues[159] - acadoWorkspace.evH[159];
acadoWorkspace.ubA[160] = acadoVariables.ubAValues[160] - acadoWorkspace.evH[160];
acadoWorkspace.ubA[161] = acadoVariables.ubAValues[161] - acadoWorkspace.evH[161];
acadoWorkspace.ubA[162] = acadoVariables.ubAValues[162] - acadoWorkspace.evH[162];
acadoWorkspace.ubA[163] = acadoVariables.ubAValues[163] - acadoWorkspace.evH[163];
acadoWorkspace.ubA[164] = acadoVariables.ubAValues[164] - acadoWorkspace.evH[164];
acadoWorkspace.ubA[165] = acadoVariables.ubAValues[165] - acadoWorkspace.evH[165];
acadoWorkspace.ubA[166] = acadoVariables.ubAValues[166] - acadoWorkspace.evH[166];
acadoWorkspace.ubA[167] = acadoVariables.ubAValues[167] - acadoWorkspace.evH[167];
acadoWorkspace.ubA[168] = acadoVariables.ubAValues[168] - acadoWorkspace.evH[168];
acadoWorkspace.ubA[169] = acadoVariables.ubAValues[169] - acadoWorkspace.evH[169];
acadoWorkspace.ubA[170] = acadoVariables.ubAValues[170] - acadoWorkspace.evH[170];
acadoWorkspace.ubA[171] = acadoVariables.ubAValues[171] - acadoWorkspace.evH[171];
acadoWorkspace.ubA[172] = acadoVariables.ubAValues[172] - acadoWorkspace.evH[172];
acadoWorkspace.ubA[173] = acadoVariables.ubAValues[173] - acadoWorkspace.evH[173];
acadoWorkspace.ubA[174] = acadoVariables.ubAValues[174] - acadoWorkspace.evH[174];
acadoWorkspace.ubA[175] = acadoVariables.ubAValues[175] - acadoWorkspace.evH[175];
acadoWorkspace.ubA[176] = acadoVariables.ubAValues[176] - acadoWorkspace.evH[176];
acadoWorkspace.ubA[177] = acadoVariables.ubAValues[177] - acadoWorkspace.evH[177];
acadoWorkspace.ubA[178] = acadoVariables.ubAValues[178] - acadoWorkspace.evH[178];
acadoWorkspace.ubA[179] = acadoVariables.ubAValues[179] - acadoWorkspace.evH[179];
acadoWorkspace.ubA[180] = acadoVariables.ubAValues[180] - acadoWorkspace.evH[180];
acadoWorkspace.ubA[181] = acadoVariables.ubAValues[181] - acadoWorkspace.evH[181];
acadoWorkspace.ubA[182] = acadoVariables.ubAValues[182] - acadoWorkspace.evH[182];
acadoWorkspace.ubA[183] = acadoVariables.ubAValues[183] - acadoWorkspace.evH[183];
acadoWorkspace.ubA[184] = acadoVariables.ubAValues[184] - acadoWorkspace.evH[184];
acadoWorkspace.ubA[185] = acadoVariables.ubAValues[185] - acadoWorkspace.evH[185];
acadoWorkspace.ubA[186] = acadoVariables.ubAValues[186] - acadoWorkspace.evH[186];
acadoWorkspace.ubA[187] = acadoVariables.ubAValues[187] - acadoWorkspace.evH[187];
acadoWorkspace.ubA[188] = acadoVariables.ubAValues[188] - acadoWorkspace.evH[188];
acadoWorkspace.ubA[189] = acadoVariables.ubAValues[189] - acadoWorkspace.evH[189];
acadoWorkspace.ubA[190] = acadoVariables.ubAValues[190] - acadoWorkspace.evH[190];
acadoWorkspace.ubA[191] = acadoVariables.ubAValues[191] - acadoWorkspace.evH[191];
acadoWorkspace.ubA[192] = acadoVariables.ubAValues[192] - acadoWorkspace.evH[192];
acadoWorkspace.ubA[193] = acadoVariables.ubAValues[193] - acadoWorkspace.evH[193];
acadoWorkspace.ubA[194] = acadoVariables.ubAValues[194] - acadoWorkspace.evH[194];
acadoWorkspace.ubA[195] = acadoVariables.ubAValues[195] - acadoWorkspace.evH[195];
acadoWorkspace.ubA[196] = acadoVariables.ubAValues[196] - acadoWorkspace.evH[196];
acadoWorkspace.ubA[197] = acadoVariables.ubAValues[197] - acadoWorkspace.evH[197];
acadoWorkspace.ubA[198] = acadoVariables.ubAValues[198] - acadoWorkspace.evH[198];
acadoWorkspace.ubA[199] = acadoVariables.ubAValues[199] - acadoWorkspace.evH[199];
acadoWorkspace.ubA[200] = acadoVariables.ubAValues[200] - acadoWorkspace.evH[200];
acadoWorkspace.ubA[201] = acadoVariables.ubAValues[201] - acadoWorkspace.evH[201];
acadoWorkspace.ubA[202] = acadoVariables.ubAValues[202] - acadoWorkspace.evH[202];
acadoWorkspace.ubA[203] = acadoVariables.ubAValues[203] - acadoWorkspace.evH[203];
acadoWorkspace.ubA[204] = acadoVariables.ubAValues[204] - acadoWorkspace.evH[204];
acadoWorkspace.ubA[205] = acadoVariables.ubAValues[205] - acadoWorkspace.evH[205];
acadoWorkspace.ubA[206] = acadoVariables.ubAValues[206] - acadoWorkspace.evH[206];
acadoWorkspace.ubA[207] = acadoVariables.ubAValues[207] - acadoWorkspace.evH[207];
acadoWorkspace.ubA[208] = acadoVariables.ubAValues[208] - acadoWorkspace.evH[208];
acadoWorkspace.ubA[209] = acadoVariables.ubAValues[209] - acadoWorkspace.evH[209];

acado_macHxd( &(acadoWorkspace.evHx[ 28 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 7 ]), &(acadoWorkspace.ubA[ 7 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.lbA[ 21 ]), &(acadoWorkspace.ubA[ 21 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 112 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.lbA[ 35 ]), &(acadoWorkspace.ubA[ 35 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.lbA[ 42 ]), &(acadoWorkspace.ubA[ 42 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 196 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.lbA[ 49 ]), &(acadoWorkspace.ubA[ 49 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.lbA[ 56 ]), &(acadoWorkspace.ubA[ 56 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.lbA[ 63 ]), &(acadoWorkspace.ubA[ 63 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 280 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.lbA[ 70 ]), &(acadoWorkspace.ubA[ 70 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 308 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.lbA[ 77 ]), &(acadoWorkspace.ubA[ 77 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 336 ]), &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.lbA[ 84 ]), &(acadoWorkspace.ubA[ 84 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 364 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.lbA[ 91 ]), &(acadoWorkspace.ubA[ 91 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 392 ]), &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.lbA[ 98 ]), &(acadoWorkspace.ubA[ 98 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 420 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.lbA[ 105 ]), &(acadoWorkspace.ubA[ 105 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 112 ]), &(acadoWorkspace.ubA[ 112 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 476 ]), &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.lbA[ 119 ]), &(acadoWorkspace.ubA[ 119 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 504 ]), &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.lbA[ 126 ]), &(acadoWorkspace.ubA[ 126 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 532 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.lbA[ 133 ]), &(acadoWorkspace.ubA[ 133 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 560 ]), &(acadoWorkspace.d[ 76 ]), &(acadoWorkspace.lbA[ 140 ]), &(acadoWorkspace.ubA[ 140 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 588 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.lbA[ 147 ]), &(acadoWorkspace.ubA[ 147 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 616 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.lbA[ 154 ]), &(acadoWorkspace.ubA[ 154 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 644 ]), &(acadoWorkspace.d[ 88 ]), &(acadoWorkspace.lbA[ 161 ]), &(acadoWorkspace.ubA[ 161 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 672 ]), &(acadoWorkspace.d[ 92 ]), &(acadoWorkspace.lbA[ 168 ]), &(acadoWorkspace.ubA[ 168 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 700 ]), &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.lbA[ 175 ]), &(acadoWorkspace.ubA[ 175 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 728 ]), &(acadoWorkspace.d[ 100 ]), &(acadoWorkspace.lbA[ 182 ]), &(acadoWorkspace.ubA[ 182 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 756 ]), &(acadoWorkspace.d[ 104 ]), &(acadoWorkspace.lbA[ 189 ]), &(acadoWorkspace.ubA[ 189 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 784 ]), &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.lbA[ 196 ]), &(acadoWorkspace.ubA[ 196 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 812 ]), &(acadoWorkspace.d[ 112 ]), &(acadoWorkspace.lbA[ 203 ]), &(acadoWorkspace.ubA[ 203 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];

for (lRun2 = 0; lRun2 < 360; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 24 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 96 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 192 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 264 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 312 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 336 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 360 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 384 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 408 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 432 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 456 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 480 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 504 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 528 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 552 ]), &(acadoWorkspace.Dy[ 276 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 576 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 600 ]), &(acadoWorkspace.Dy[ 300 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 624 ]), &(acadoWorkspace.Dy[ 312 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 648 ]), &(acadoWorkspace.Dy[ 324 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 672 ]), &(acadoWorkspace.Dy[ 336 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 696 ]), &(acadoWorkspace.Dy[ 348 ]), &(acadoWorkspace.g[ 58 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 48 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 96 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 144 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 192 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 288 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 336 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 384 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 432 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 528 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 44 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 576 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 624 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 672 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 768 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 816 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.QDy[ 68 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 912 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.QDy[ 76 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 960 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1008 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1056 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.QDy[ 88 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1104 ]), &(acadoWorkspace.Dy[ 276 ]), &(acadoWorkspace.QDy[ 92 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1152 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1200 ]), &(acadoWorkspace.Dy[ 300 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1248 ]), &(acadoWorkspace.Dy[ 312 ]), &(acadoWorkspace.QDy[ 104 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1296 ]), &(acadoWorkspace.Dy[ 324 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1344 ]), &(acadoWorkspace.Dy[ 336 ]), &(acadoWorkspace.QDy[ 112 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1392 ]), &(acadoWorkspace.Dy[ 348 ]), &(acadoWorkspace.QDy[ 116 ]) );

acadoWorkspace.QDy[120] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[121] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[122] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[123] = + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[2];

acadoWorkspace.QDy[4] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[5] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[79];
acadoWorkspace.QDy[84] += acadoWorkspace.Qd[80];
acadoWorkspace.QDy[85] += acadoWorkspace.Qd[81];
acadoWorkspace.QDy[86] += acadoWorkspace.Qd[82];
acadoWorkspace.QDy[87] += acadoWorkspace.Qd[83];
acadoWorkspace.QDy[88] += acadoWorkspace.Qd[84];
acadoWorkspace.QDy[89] += acadoWorkspace.Qd[85];
acadoWorkspace.QDy[90] += acadoWorkspace.Qd[86];
acadoWorkspace.QDy[91] += acadoWorkspace.Qd[87];
acadoWorkspace.QDy[92] += acadoWorkspace.Qd[88];
acadoWorkspace.QDy[93] += acadoWorkspace.Qd[89];
acadoWorkspace.QDy[94] += acadoWorkspace.Qd[90];
acadoWorkspace.QDy[95] += acadoWorkspace.Qd[91];
acadoWorkspace.QDy[96] += acadoWorkspace.Qd[92];
acadoWorkspace.QDy[97] += acadoWorkspace.Qd[93];
acadoWorkspace.QDy[98] += acadoWorkspace.Qd[94];
acadoWorkspace.QDy[99] += acadoWorkspace.Qd[95];
acadoWorkspace.QDy[100] += acadoWorkspace.Qd[96];
acadoWorkspace.QDy[101] += acadoWorkspace.Qd[97];
acadoWorkspace.QDy[102] += acadoWorkspace.Qd[98];
acadoWorkspace.QDy[103] += acadoWorkspace.Qd[99];
acadoWorkspace.QDy[104] += acadoWorkspace.Qd[100];
acadoWorkspace.QDy[105] += acadoWorkspace.Qd[101];
acadoWorkspace.QDy[106] += acadoWorkspace.Qd[102];
acadoWorkspace.QDy[107] += acadoWorkspace.Qd[103];
acadoWorkspace.QDy[108] += acadoWorkspace.Qd[104];
acadoWorkspace.QDy[109] += acadoWorkspace.Qd[105];
acadoWorkspace.QDy[110] += acadoWorkspace.Qd[106];
acadoWorkspace.QDy[111] += acadoWorkspace.Qd[107];
acadoWorkspace.QDy[112] += acadoWorkspace.Qd[108];
acadoWorkspace.QDy[113] += acadoWorkspace.Qd[109];
acadoWorkspace.QDy[114] += acadoWorkspace.Qd[110];
acadoWorkspace.QDy[115] += acadoWorkspace.Qd[111];
acadoWorkspace.QDy[116] += acadoWorkspace.Qd[112];
acadoWorkspace.QDy[117] += acadoWorkspace.Qd[113];
acadoWorkspace.QDy[118] += acadoWorkspace.Qd[114];
acadoWorkspace.QDy[119] += acadoWorkspace.Qd[115];
acadoWorkspace.QDy[120] += acadoWorkspace.Qd[116];
acadoWorkspace.QDy[121] += acadoWorkspace.Qd[117];
acadoWorkspace.QDy[122] += acadoWorkspace.Qd[118];
acadoWorkspace.QDy[123] += acadoWorkspace.Qd[119];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.QDy[ lRun2 * 4 + 4 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[1] += + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[2] += + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[3] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[4] += + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[5] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[6] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[7] += + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[8] += + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[9] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[10] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[11] += + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[12] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[13] += + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[14] += + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[15] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[16] += + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[17] += + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[18] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[19] += + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[20] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[21] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[22] += + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[23] += + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[24] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[25] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[26] += + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[27] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[28] += + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[29] += + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[30] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[31] += + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[32] += + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[33] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[34] += + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[35] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[36] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[37] += + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[38] += + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[39] += + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[40] += + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[41] += + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[42] += + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[43] += + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[44] += + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[45] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[46] += + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[47] += + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[48] += + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[49] += + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[50] += + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[51] += + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[52] += + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[53] += + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[54] += + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[55] += + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[56] += + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[57] += + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[58] += + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[59] += + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[3];

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[50] = + acadoWorkspace.A01[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[203]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[51] = + acadoWorkspace.A01[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[207]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[52] = + acadoWorkspace.A01[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[211]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[53] = + acadoWorkspace.A01[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[215]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[54] = + acadoWorkspace.A01[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[219]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[55] = + acadoWorkspace.A01[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[223]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[56] = + acadoWorkspace.A01[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[227]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[57] = + acadoWorkspace.A01[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[231]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[58] = + acadoWorkspace.A01[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[235]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[59] = + acadoWorkspace.A01[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[239]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[60] = + acadoWorkspace.A01[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[243]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[61] = + acadoWorkspace.A01[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[247]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[62] = + acadoWorkspace.A01[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[251]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[63] = + acadoWorkspace.A01[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[255]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[64] = + acadoWorkspace.A01[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[259]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[65] = + acadoWorkspace.A01[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[263]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[66] = + acadoWorkspace.A01[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[267]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[67] = + acadoWorkspace.A01[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[271]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[68] = + acadoWorkspace.A01[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[275]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[69] = + acadoWorkspace.A01[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[279]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[70] = + acadoWorkspace.A01[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[283]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[71] = + acadoWorkspace.A01[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[287]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[72] = + acadoWorkspace.A01[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[291]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[73] = + acadoWorkspace.A01[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[295]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[74] = + acadoWorkspace.A01[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[299]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[75] = + acadoWorkspace.A01[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[303]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[76] = + acadoWorkspace.A01[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[307]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[77] = + acadoWorkspace.A01[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[311]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[78] = + acadoWorkspace.A01[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[315]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[79] = + acadoWorkspace.A01[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[319]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[80] = + acadoWorkspace.A01[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[323]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[81] = + acadoWorkspace.A01[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[327]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[82] = + acadoWorkspace.A01[328]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[329]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[330]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[331]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[83] = + acadoWorkspace.A01[332]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[333]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[334]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[335]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[84] = + acadoWorkspace.A01[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[339]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[85] = + acadoWorkspace.A01[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[343]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[86] = + acadoWorkspace.A01[344]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[345]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[346]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[347]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[87] = + acadoWorkspace.A01[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[351]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[88] = + acadoWorkspace.A01[352]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[353]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[354]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[355]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[89] = + acadoWorkspace.A01[356]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[357]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[358]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[359]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[90] = + acadoWorkspace.A01[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[363]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[91] = + acadoWorkspace.A01[364]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[365]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[366]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[367]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[92] = + acadoWorkspace.A01[368]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[369]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[370]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[371]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[93] = + acadoWorkspace.A01[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[375]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[94] = + acadoWorkspace.A01[376]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[377]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[378]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[379]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[95] = + acadoWorkspace.A01[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[383]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[96] = + acadoWorkspace.A01[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[387]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[97] = + acadoWorkspace.A01[388]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[389]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[390]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[391]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[98] = + acadoWorkspace.A01[392]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[393]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[394]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[395]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[99] = + acadoWorkspace.A01[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[399]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[100] = + acadoWorkspace.A01[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[403]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[101] = + acadoWorkspace.A01[404]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[405]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[406]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[407]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[102] = + acadoWorkspace.A01[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[411]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[103] = + acadoWorkspace.A01[412]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[413]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[414]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[415]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[104] = + acadoWorkspace.A01[416]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[417]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[418]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[419]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[105] = + acadoWorkspace.A01[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[423]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[106] = + acadoWorkspace.A01[424]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[425]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[426]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[427]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[107] = + acadoWorkspace.A01[428]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[429]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[430]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[431]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[108] = + acadoWorkspace.A01[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[435]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[109] = + acadoWorkspace.A01[436]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[437]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[438]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[439]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[110] = + acadoWorkspace.A01[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[443]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[111] = + acadoWorkspace.A01[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[447]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[112] = + acadoWorkspace.A01[448]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[449]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[450]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[451]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[113] = + acadoWorkspace.A01[452]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[453]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[454]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[455]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[114] = + acadoWorkspace.A01[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[459]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[115] = + acadoWorkspace.A01[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[463]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[116] = + acadoWorkspace.A01[464]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[465]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[466]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[467]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[117] = + acadoWorkspace.A01[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[471]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[118] = + acadoWorkspace.A01[472]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[473]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[474]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[475]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[119] = + acadoWorkspace.A01[476]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[477]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[478]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[479]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[120] = + acadoWorkspace.A01[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[483]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[121] = + acadoWorkspace.A01[484]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[485]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[486]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[487]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[122] = + acadoWorkspace.A01[488]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[489]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[490]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[491]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[123] = + acadoWorkspace.A01[492]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[493]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[494]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[495]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[124] = + acadoWorkspace.A01[496]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[497]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[498]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[499]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[125] = + acadoWorkspace.A01[500]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[501]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[502]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[503]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[126] = + acadoWorkspace.A01[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[507]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[127] = + acadoWorkspace.A01[508]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[509]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[510]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[511]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[128] = + acadoWorkspace.A01[512]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[513]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[514]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[515]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[129] = + acadoWorkspace.A01[516]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[517]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[518]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[519]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[130] = + acadoWorkspace.A01[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[523]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[131] = + acadoWorkspace.A01[524]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[525]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[526]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[527]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[132] = + acadoWorkspace.A01[528]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[529]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[530]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[531]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[133] = + acadoWorkspace.A01[532]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[533]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[534]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[535]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[134] = + acadoWorkspace.A01[536]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[537]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[538]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[539]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[135] = + acadoWorkspace.A01[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[543]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[136] = + acadoWorkspace.A01[544]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[545]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[546]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[547]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[137] = + acadoWorkspace.A01[548]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[549]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[550]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[551]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[138] = + acadoWorkspace.A01[552]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[553]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[554]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[555]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[139] = + acadoWorkspace.A01[556]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[557]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[558]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[559]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[140] = + acadoWorkspace.A01[560]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[561]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[562]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[563]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[141] = + acadoWorkspace.A01[564]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[565]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[566]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[567]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[142] = + acadoWorkspace.A01[568]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[569]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[570]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[571]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[143] = + acadoWorkspace.A01[572]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[573]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[574]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[575]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[144] = + acadoWorkspace.A01[576]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[577]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[578]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[579]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[145] = + acadoWorkspace.A01[580]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[581]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[582]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[583]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[146] = + acadoWorkspace.A01[584]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[585]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[586]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[587]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[147] = + acadoWorkspace.A01[588]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[589]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[590]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[591]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[148] = + acadoWorkspace.A01[592]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[593]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[594]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[595]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[149] = + acadoWorkspace.A01[596]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[597]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[598]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[599]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[150] = + acadoWorkspace.A01[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[603]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[151] = + acadoWorkspace.A01[604]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[605]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[606]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[607]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[152] = + acadoWorkspace.A01[608]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[609]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[610]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[611]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[153] = + acadoWorkspace.A01[612]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[613]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[614]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[615]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[154] = + acadoWorkspace.A01[616]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[617]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[618]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[619]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[155] = + acadoWorkspace.A01[620]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[621]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[622]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[623]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[156] = + acadoWorkspace.A01[624]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[625]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[626]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[627]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[157] = + acadoWorkspace.A01[628]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[629]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[630]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[631]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[158] = + acadoWorkspace.A01[632]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[633]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[634]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[635]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[159] = + acadoWorkspace.A01[636]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[637]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[638]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[639]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[160] = + acadoWorkspace.A01[640]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[641]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[642]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[643]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[161] = + acadoWorkspace.A01[644]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[645]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[646]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[647]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[162] = + acadoWorkspace.A01[648]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[649]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[650]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[651]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[163] = + acadoWorkspace.A01[652]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[653]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[654]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[655]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[164] = + acadoWorkspace.A01[656]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[657]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[658]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[659]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[165] = + acadoWorkspace.A01[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[663]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[166] = + acadoWorkspace.A01[664]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[665]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[666]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[667]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[167] = + acadoWorkspace.A01[668]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[669]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[670]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[671]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[168] = + acadoWorkspace.A01[672]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[673]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[674]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[675]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[169] = + acadoWorkspace.A01[676]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[677]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[678]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[679]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[170] = + acadoWorkspace.A01[680]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[681]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[682]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[683]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[171] = + acadoWorkspace.A01[684]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[685]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[686]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[687]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[172] = + acadoWorkspace.A01[688]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[689]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[690]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[691]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[173] = + acadoWorkspace.A01[692]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[693]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[694]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[695]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[174] = + acadoWorkspace.A01[696]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[697]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[698]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[699]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[175] = + acadoWorkspace.A01[700]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[701]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[702]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[703]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[176] = + acadoWorkspace.A01[704]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[705]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[706]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[707]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[177] = + acadoWorkspace.A01[708]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[709]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[710]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[711]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[178] = + acadoWorkspace.A01[712]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[713]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[714]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[715]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[179] = + acadoWorkspace.A01[716]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[717]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[718]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[719]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[180] = + acadoWorkspace.A01[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[723]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[181] = + acadoWorkspace.A01[724]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[725]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[726]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[727]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[182] = + acadoWorkspace.A01[728]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[729]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[730]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[731]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[183] = + acadoWorkspace.A01[732]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[733]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[734]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[735]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[184] = + acadoWorkspace.A01[736]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[737]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[738]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[739]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[185] = + acadoWorkspace.A01[740]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[741]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[742]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[743]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[186] = + acadoWorkspace.A01[744]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[745]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[746]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[747]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[187] = + acadoWorkspace.A01[748]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[749]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[750]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[751]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[188] = + acadoWorkspace.A01[752]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[753]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[754]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[755]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[189] = + acadoWorkspace.A01[756]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[757]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[758]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[759]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[190] = + acadoWorkspace.A01[760]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[761]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[762]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[763]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[191] = + acadoWorkspace.A01[764]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[765]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[766]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[767]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[192] = + acadoWorkspace.A01[768]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[769]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[770]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[771]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[193] = + acadoWorkspace.A01[772]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[773]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[774]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[775]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[194] = + acadoWorkspace.A01[776]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[777]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[778]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[779]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[195] = + acadoWorkspace.A01[780]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[781]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[782]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[783]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[196] = + acadoWorkspace.A01[784]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[785]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[786]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[787]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[197] = + acadoWorkspace.A01[788]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[789]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[790]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[791]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[198] = + acadoWorkspace.A01[792]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[793]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[794]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[795]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[199] = + acadoWorkspace.A01[796]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[797]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[798]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[799]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[200] = + acadoWorkspace.A01[800]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[801]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[802]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[803]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[201] = + acadoWorkspace.A01[804]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[805]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[806]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[807]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[202] = + acadoWorkspace.A01[808]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[809]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[810]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[811]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[203] = + acadoWorkspace.A01[812]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[813]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[814]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[815]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[204] = + acadoWorkspace.A01[816]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[817]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[818]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[819]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[205] = + acadoWorkspace.A01[820]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[821]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[822]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[823]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[206] = + acadoWorkspace.A01[824]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[825]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[826]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[827]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[207] = + acadoWorkspace.A01[828]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[829]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[830]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[831]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[208] = + acadoWorkspace.A01[832]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[833]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[834]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[835]*acadoWorkspace.Dx0[3];
acadoWorkspace.pacA01Dx0[209] = + acadoWorkspace.A01[836]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[837]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[838]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[839]*acadoWorkspace.Dx0[3];
for (lRun2 = 0; lRun2 < 210; ++lRun2)
acadoWorkspace.lbA[lRun2] -= acadoWorkspace.pacA01Dx0[lRun2];


for (lRun2 = 0; lRun2 < 210; ++lRun2)
acadoWorkspace.ubA[lRun2] -= acadoWorkspace.pacA01Dx0[lRun2];


}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];

acadoVariables.x[4] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[0];
acadoVariables.x[5] += + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[1];
acadoVariables.x[6] += + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[2];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[3];
acadoVariables.x[8] += + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[4];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[5];
acadoVariables.x[10] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[6];
acadoVariables.x[11] += + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[7];
acadoVariables.x[12] += + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[8];
acadoVariables.x[13] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[9];
acadoVariables.x[14] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[10];
acadoVariables.x[15] += + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[11];
acadoVariables.x[16] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[12];
acadoVariables.x[17] += + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[13];
acadoVariables.x[18] += + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[14];
acadoVariables.x[19] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[15];
acadoVariables.x[20] += + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[16];
acadoVariables.x[21] += + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[17];
acadoVariables.x[22] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[18];
acadoVariables.x[23] += + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[19];
acadoVariables.x[24] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[20];
acadoVariables.x[25] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[21];
acadoVariables.x[26] += + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[22];
acadoVariables.x[27] += + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[23];
acadoVariables.x[28] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[24];
acadoVariables.x[29] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[25];
acadoVariables.x[30] += + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[26];
acadoVariables.x[31] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[27];
acadoVariables.x[32] += + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[28];
acadoVariables.x[33] += + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[29];
acadoVariables.x[34] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[30];
acadoVariables.x[35] += + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[31];
acadoVariables.x[36] += + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[32];
acadoVariables.x[37] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[33];
acadoVariables.x[38] += + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[34];
acadoVariables.x[39] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[35];
acadoVariables.x[40] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[36];
acadoVariables.x[41] += + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[37];
acadoVariables.x[42] += + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[38];
acadoVariables.x[43] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[39];
acadoVariables.x[44] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[40];
acadoVariables.x[45] += + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[41];
acadoVariables.x[46] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[42];
acadoVariables.x[47] += + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[43];
acadoVariables.x[48] += + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[44];
acadoVariables.x[49] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[45];
acadoVariables.x[50] += + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[46];
acadoVariables.x[51] += + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[47];
acadoVariables.x[52] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[48];
acadoVariables.x[53] += + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[49];
acadoVariables.x[54] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[50];
acadoVariables.x[55] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[51];
acadoVariables.x[56] += + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[52];
acadoVariables.x[57] += + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[53];
acadoVariables.x[58] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[54];
acadoVariables.x[59] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[55];
acadoVariables.x[60] += + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[56];
acadoVariables.x[61] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[57];
acadoVariables.x[62] += + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[58];
acadoVariables.x[63] += + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[59];
acadoVariables.x[64] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[60];
acadoVariables.x[65] += + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[61];
acadoVariables.x[66] += + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[62];
acadoVariables.x[67] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[63];
acadoVariables.x[68] += + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[64];
acadoVariables.x[69] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[65];
acadoVariables.x[70] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[66];
acadoVariables.x[71] += + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[67];
acadoVariables.x[72] += + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[68];
acadoVariables.x[73] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[69];
acadoVariables.x[74] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[70];
acadoVariables.x[75] += + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[71];
acadoVariables.x[76] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[72];
acadoVariables.x[77] += + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[73];
acadoVariables.x[78] += + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[74];
acadoVariables.x[79] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[75];
acadoVariables.x[80] += + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[76];
acadoVariables.x[81] += + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[77];
acadoVariables.x[82] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[78];
acadoVariables.x[83] += + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[79];
acadoVariables.x[84] += + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[80];
acadoVariables.x[85] += + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[81];
acadoVariables.x[86] += + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[82];
acadoVariables.x[87] += + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[83];
acadoVariables.x[88] += + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[84];
acadoVariables.x[89] += + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[85];
acadoVariables.x[90] += + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[86];
acadoVariables.x[91] += + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[87];
acadoVariables.x[92] += + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[88];
acadoVariables.x[93] += + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[89];
acadoVariables.x[94] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[90];
acadoVariables.x[95] += + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[91];
acadoVariables.x[96] += + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[92];
acadoVariables.x[97] += + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[93];
acadoVariables.x[98] += + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[94];
acadoVariables.x[99] += + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[95];
acadoVariables.x[100] += + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[96];
acadoVariables.x[101] += + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[97];
acadoVariables.x[102] += + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[98];
acadoVariables.x[103] += + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[99];
acadoVariables.x[104] += + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[100];
acadoVariables.x[105] += + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[101];
acadoVariables.x[106] += + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[102];
acadoVariables.x[107] += + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[103];
acadoVariables.x[108] += + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[104];
acadoVariables.x[109] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[105];
acadoVariables.x[110] += + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[106];
acadoVariables.x[111] += + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[107];
acadoVariables.x[112] += + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[108];
acadoVariables.x[113] += + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[109];
acadoVariables.x[114] += + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[110];
acadoVariables.x[115] += + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[111];
acadoVariables.x[116] += + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[112];
acadoVariables.x[117] += + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[113];
acadoVariables.x[118] += + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[114];
acadoVariables.x[119] += + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[115];
acadoVariables.x[120] += + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[116];
acadoVariables.x[121] += + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[117];
acadoVariables.x[122] += + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[118];
acadoVariables.x[123] += + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[119];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.x[ lRun2 * 2 ]), &(acadoVariables.x[ lRun1 * 4 + 4 ]) );
}
}
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -5.0000000000000000e-01;
acadoVariables.lbValues[1] = -5.9999999999999998e-01;
acadoVariables.lbValues[2] = -5.0000000000000000e-01;
acadoVariables.lbValues[3] = -5.9999999999999998e-01;
acadoVariables.lbValues[4] = -5.0000000000000000e-01;
acadoVariables.lbValues[5] = -5.9999999999999998e-01;
acadoVariables.lbValues[6] = -5.0000000000000000e-01;
acadoVariables.lbValues[7] = -5.9999999999999998e-01;
acadoVariables.lbValues[8] = -5.0000000000000000e-01;
acadoVariables.lbValues[9] = -5.9999999999999998e-01;
acadoVariables.lbValues[10] = -5.0000000000000000e-01;
acadoVariables.lbValues[11] = -5.9999999999999998e-01;
acadoVariables.lbValues[12] = -5.0000000000000000e-01;
acadoVariables.lbValues[13] = -5.9999999999999998e-01;
acadoVariables.lbValues[14] = -5.0000000000000000e-01;
acadoVariables.lbValues[15] = -5.9999999999999998e-01;
acadoVariables.lbValues[16] = -5.0000000000000000e-01;
acadoVariables.lbValues[17] = -5.9999999999999998e-01;
acadoVariables.lbValues[18] = -5.0000000000000000e-01;
acadoVariables.lbValues[19] = -5.9999999999999998e-01;
acadoVariables.lbValues[20] = -5.0000000000000000e-01;
acadoVariables.lbValues[21] = -5.9999999999999998e-01;
acadoVariables.lbValues[22] = -5.0000000000000000e-01;
acadoVariables.lbValues[23] = -5.9999999999999998e-01;
acadoVariables.lbValues[24] = -5.0000000000000000e-01;
acadoVariables.lbValues[25] = -5.9999999999999998e-01;
acadoVariables.lbValues[26] = -5.0000000000000000e-01;
acadoVariables.lbValues[27] = -5.9999999999999998e-01;
acadoVariables.lbValues[28] = -5.0000000000000000e-01;
acadoVariables.lbValues[29] = -5.9999999999999998e-01;
acadoVariables.lbValues[30] = -5.0000000000000000e-01;
acadoVariables.lbValues[31] = -5.9999999999999998e-01;
acadoVariables.lbValues[32] = -5.0000000000000000e-01;
acadoVariables.lbValues[33] = -5.9999999999999998e-01;
acadoVariables.lbValues[34] = -5.0000000000000000e-01;
acadoVariables.lbValues[35] = -5.9999999999999998e-01;
acadoVariables.lbValues[36] = -5.0000000000000000e-01;
acadoVariables.lbValues[37] = -5.9999999999999998e-01;
acadoVariables.lbValues[38] = -5.0000000000000000e-01;
acadoVariables.lbValues[39] = -5.9999999999999998e-01;
acadoVariables.lbValues[40] = -5.0000000000000000e-01;
acadoVariables.lbValues[41] = -5.9999999999999998e-01;
acadoVariables.lbValues[42] = -5.0000000000000000e-01;
acadoVariables.lbValues[43] = -5.9999999999999998e-01;
acadoVariables.lbValues[44] = -5.0000000000000000e-01;
acadoVariables.lbValues[45] = -5.9999999999999998e-01;
acadoVariables.lbValues[46] = -5.0000000000000000e-01;
acadoVariables.lbValues[47] = -5.9999999999999998e-01;
acadoVariables.lbValues[48] = -5.0000000000000000e-01;
acadoVariables.lbValues[49] = -5.9999999999999998e-01;
acadoVariables.lbValues[50] = -5.0000000000000000e-01;
acadoVariables.lbValues[51] = -5.9999999999999998e-01;
acadoVariables.lbValues[52] = -5.0000000000000000e-01;
acadoVariables.lbValues[53] = -5.9999999999999998e-01;
acadoVariables.lbValues[54] = -5.0000000000000000e-01;
acadoVariables.lbValues[55] = -5.9999999999999998e-01;
acadoVariables.lbValues[56] = -5.0000000000000000e-01;
acadoVariables.lbValues[57] = -5.9999999999999998e-01;
acadoVariables.lbValues[58] = -5.0000000000000000e-01;
acadoVariables.lbValues[59] = -5.9999999999999998e-01;
acadoVariables.ubValues[0] = 5.0000000000000000e-01;
acadoVariables.ubValues[1] = 5.9999999999999998e-01;
acadoVariables.ubValues[2] = 5.0000000000000000e-01;
acadoVariables.ubValues[3] = 5.9999999999999998e-01;
acadoVariables.ubValues[4] = 5.0000000000000000e-01;
acadoVariables.ubValues[5] = 5.9999999999999998e-01;
acadoVariables.ubValues[6] = 5.0000000000000000e-01;
acadoVariables.ubValues[7] = 5.9999999999999998e-01;
acadoVariables.ubValues[8] = 5.0000000000000000e-01;
acadoVariables.ubValues[9] = 5.9999999999999998e-01;
acadoVariables.ubValues[10] = 5.0000000000000000e-01;
acadoVariables.ubValues[11] = 5.9999999999999998e-01;
acadoVariables.ubValues[12] = 5.0000000000000000e-01;
acadoVariables.ubValues[13] = 5.9999999999999998e-01;
acadoVariables.ubValues[14] = 5.0000000000000000e-01;
acadoVariables.ubValues[15] = 5.9999999999999998e-01;
acadoVariables.ubValues[16] = 5.0000000000000000e-01;
acadoVariables.ubValues[17] = 5.9999999999999998e-01;
acadoVariables.ubValues[18] = 5.0000000000000000e-01;
acadoVariables.ubValues[19] = 5.9999999999999998e-01;
acadoVariables.ubValues[20] = 5.0000000000000000e-01;
acadoVariables.ubValues[21] = 5.9999999999999998e-01;
acadoVariables.ubValues[22] = 5.0000000000000000e-01;
acadoVariables.ubValues[23] = 5.9999999999999998e-01;
acadoVariables.ubValues[24] = 5.0000000000000000e-01;
acadoVariables.ubValues[25] = 5.9999999999999998e-01;
acadoVariables.ubValues[26] = 5.0000000000000000e-01;
acadoVariables.ubValues[27] = 5.9999999999999998e-01;
acadoVariables.ubValues[28] = 5.0000000000000000e-01;
acadoVariables.ubValues[29] = 5.9999999999999998e-01;
acadoVariables.ubValues[30] = 5.0000000000000000e-01;
acadoVariables.ubValues[31] = 5.9999999999999998e-01;
acadoVariables.ubValues[32] = 5.0000000000000000e-01;
acadoVariables.ubValues[33] = 5.9999999999999998e-01;
acadoVariables.ubValues[34] = 5.0000000000000000e-01;
acadoVariables.ubValues[35] = 5.9999999999999998e-01;
acadoVariables.ubValues[36] = 5.0000000000000000e-01;
acadoVariables.ubValues[37] = 5.9999999999999998e-01;
acadoVariables.ubValues[38] = 5.0000000000000000e-01;
acadoVariables.ubValues[39] = 5.9999999999999998e-01;
acadoVariables.ubValues[40] = 5.0000000000000000e-01;
acadoVariables.ubValues[41] = 5.9999999999999998e-01;
acadoVariables.ubValues[42] = 5.0000000000000000e-01;
acadoVariables.ubValues[43] = 5.9999999999999998e-01;
acadoVariables.ubValues[44] = 5.0000000000000000e-01;
acadoVariables.ubValues[45] = 5.9999999999999998e-01;
acadoVariables.ubValues[46] = 5.0000000000000000e-01;
acadoVariables.ubValues[47] = 5.9999999999999998e-01;
acadoVariables.ubValues[48] = 5.0000000000000000e-01;
acadoVariables.ubValues[49] = 5.9999999999999998e-01;
acadoVariables.ubValues[50] = 5.0000000000000000e-01;
acadoVariables.ubValues[51] = 5.9999999999999998e-01;
acadoVariables.ubValues[52] = 5.0000000000000000e-01;
acadoVariables.ubValues[53] = 5.9999999999999998e-01;
acadoVariables.ubValues[54] = 5.0000000000000000e-01;
acadoVariables.ubValues[55] = 5.9999999999999998e-01;
acadoVariables.ubValues[56] = 5.0000000000000000e-01;
acadoVariables.ubValues[57] = 5.9999999999999998e-01;
acadoVariables.ubValues[58] = 5.0000000000000000e-01;
acadoVariables.ubValues[59] = 5.9999999999999998e-01;
acadoVariables.lbAValues[0] = 1.0000000000000000e+00;
acadoVariables.lbAValues[1] = 1.0000000000000000e+00;
acadoVariables.lbAValues[2] = 1.0000000000000000e+00;
acadoVariables.lbAValues[3] = 1.0000000000000000e+00;
acadoVariables.lbAValues[4] = 1.0000000000000000e+00;
acadoVariables.lbAValues[5] = 1.0000000000000000e+00;
acadoVariables.lbAValues[6] = 1.0000000000000000e+00;
acadoVariables.lbAValues[7] = 1.0000000000000000e+00;
acadoVariables.lbAValues[8] = 1.0000000000000000e+00;
acadoVariables.lbAValues[9] = 1.0000000000000000e+00;
acadoVariables.lbAValues[10] = 1.0000000000000000e+00;
acadoVariables.lbAValues[11] = 1.0000000000000000e+00;
acadoVariables.lbAValues[12] = 1.0000000000000000e+00;
acadoVariables.lbAValues[13] = 1.0000000000000000e+00;
acadoVariables.lbAValues[14] = 1.0000000000000000e+00;
acadoVariables.lbAValues[15] = 1.0000000000000000e+00;
acadoVariables.lbAValues[16] = 1.0000000000000000e+00;
acadoVariables.lbAValues[17] = 1.0000000000000000e+00;
acadoVariables.lbAValues[18] = 1.0000000000000000e+00;
acadoVariables.lbAValues[19] = 1.0000000000000000e+00;
acadoVariables.lbAValues[20] = 1.0000000000000000e+00;
acadoVariables.lbAValues[21] = 1.0000000000000000e+00;
acadoVariables.lbAValues[22] = 1.0000000000000000e+00;
acadoVariables.lbAValues[23] = 1.0000000000000000e+00;
acadoVariables.lbAValues[24] = 1.0000000000000000e+00;
acadoVariables.lbAValues[25] = 1.0000000000000000e+00;
acadoVariables.lbAValues[26] = 1.0000000000000000e+00;
acadoVariables.lbAValues[27] = 1.0000000000000000e+00;
acadoVariables.lbAValues[28] = 1.0000000000000000e+00;
acadoVariables.lbAValues[29] = 1.0000000000000000e+00;
acadoVariables.lbAValues[30] = 1.0000000000000000e+00;
acadoVariables.lbAValues[31] = 1.0000000000000000e+00;
acadoVariables.lbAValues[32] = 1.0000000000000000e+00;
acadoVariables.lbAValues[33] = 1.0000000000000000e+00;
acadoVariables.lbAValues[34] = 1.0000000000000000e+00;
acadoVariables.lbAValues[35] = 1.0000000000000000e+00;
acadoVariables.lbAValues[36] = 1.0000000000000000e+00;
acadoVariables.lbAValues[37] = 1.0000000000000000e+00;
acadoVariables.lbAValues[38] = 1.0000000000000000e+00;
acadoVariables.lbAValues[39] = 1.0000000000000000e+00;
acadoVariables.lbAValues[40] = 1.0000000000000000e+00;
acadoVariables.lbAValues[41] = 1.0000000000000000e+00;
acadoVariables.lbAValues[42] = 1.0000000000000000e+00;
acadoVariables.lbAValues[43] = 1.0000000000000000e+00;
acadoVariables.lbAValues[44] = 1.0000000000000000e+00;
acadoVariables.lbAValues[45] = 1.0000000000000000e+00;
acadoVariables.lbAValues[46] = 1.0000000000000000e+00;
acadoVariables.lbAValues[47] = 1.0000000000000000e+00;
acadoVariables.lbAValues[48] = 1.0000000000000000e+00;
acadoVariables.lbAValues[49] = 1.0000000000000000e+00;
acadoVariables.lbAValues[50] = 1.0000000000000000e+00;
acadoVariables.lbAValues[51] = 1.0000000000000000e+00;
acadoVariables.lbAValues[52] = 1.0000000000000000e+00;
acadoVariables.lbAValues[53] = 1.0000000000000000e+00;
acadoVariables.lbAValues[54] = 1.0000000000000000e+00;
acadoVariables.lbAValues[55] = 1.0000000000000000e+00;
acadoVariables.lbAValues[56] = 1.0000000000000000e+00;
acadoVariables.lbAValues[57] = 1.0000000000000000e+00;
acadoVariables.lbAValues[58] = 1.0000000000000000e+00;
acadoVariables.lbAValues[59] = 1.0000000000000000e+00;
acadoVariables.lbAValues[60] = 1.0000000000000000e+00;
acadoVariables.lbAValues[61] = 1.0000000000000000e+00;
acadoVariables.lbAValues[62] = 1.0000000000000000e+00;
acadoVariables.lbAValues[63] = 1.0000000000000000e+00;
acadoVariables.lbAValues[64] = 1.0000000000000000e+00;
acadoVariables.lbAValues[65] = 1.0000000000000000e+00;
acadoVariables.lbAValues[66] = 1.0000000000000000e+00;
acadoVariables.lbAValues[67] = 1.0000000000000000e+00;
acadoVariables.lbAValues[68] = 1.0000000000000000e+00;
acadoVariables.lbAValues[69] = 1.0000000000000000e+00;
acadoVariables.lbAValues[70] = 1.0000000000000000e+00;
acadoVariables.lbAValues[71] = 1.0000000000000000e+00;
acadoVariables.lbAValues[72] = 1.0000000000000000e+00;
acadoVariables.lbAValues[73] = 1.0000000000000000e+00;
acadoVariables.lbAValues[74] = 1.0000000000000000e+00;
acadoVariables.lbAValues[75] = 1.0000000000000000e+00;
acadoVariables.lbAValues[76] = 1.0000000000000000e+00;
acadoVariables.lbAValues[77] = 1.0000000000000000e+00;
acadoVariables.lbAValues[78] = 1.0000000000000000e+00;
acadoVariables.lbAValues[79] = 1.0000000000000000e+00;
acadoVariables.lbAValues[80] = 1.0000000000000000e+00;
acadoVariables.lbAValues[81] = 1.0000000000000000e+00;
acadoVariables.lbAValues[82] = 1.0000000000000000e+00;
acadoVariables.lbAValues[83] = 1.0000000000000000e+00;
acadoVariables.lbAValues[84] = 1.0000000000000000e+00;
acadoVariables.lbAValues[85] = 1.0000000000000000e+00;
acadoVariables.lbAValues[86] = 1.0000000000000000e+00;
acadoVariables.lbAValues[87] = 1.0000000000000000e+00;
acadoVariables.lbAValues[88] = 1.0000000000000000e+00;
acadoVariables.lbAValues[89] = 1.0000000000000000e+00;
acadoVariables.lbAValues[90] = 1.0000000000000000e+00;
acadoVariables.lbAValues[91] = 1.0000000000000000e+00;
acadoVariables.lbAValues[92] = 1.0000000000000000e+00;
acadoVariables.lbAValues[93] = 1.0000000000000000e+00;
acadoVariables.lbAValues[94] = 1.0000000000000000e+00;
acadoVariables.lbAValues[95] = 1.0000000000000000e+00;
acadoVariables.lbAValues[96] = 1.0000000000000000e+00;
acadoVariables.lbAValues[97] = 1.0000000000000000e+00;
acadoVariables.lbAValues[98] = 1.0000000000000000e+00;
acadoVariables.lbAValues[99] = 1.0000000000000000e+00;
acadoVariables.lbAValues[100] = 1.0000000000000000e+00;
acadoVariables.lbAValues[101] = 1.0000000000000000e+00;
acadoVariables.lbAValues[102] = 1.0000000000000000e+00;
acadoVariables.lbAValues[103] = 1.0000000000000000e+00;
acadoVariables.lbAValues[104] = 1.0000000000000000e+00;
acadoVariables.lbAValues[105] = 1.0000000000000000e+00;
acadoVariables.lbAValues[106] = 1.0000000000000000e+00;
acadoVariables.lbAValues[107] = 1.0000000000000000e+00;
acadoVariables.lbAValues[108] = 1.0000000000000000e+00;
acadoVariables.lbAValues[109] = 1.0000000000000000e+00;
acadoVariables.lbAValues[110] = 1.0000000000000000e+00;
acadoVariables.lbAValues[111] = 1.0000000000000000e+00;
acadoVariables.lbAValues[112] = 1.0000000000000000e+00;
acadoVariables.lbAValues[113] = 1.0000000000000000e+00;
acadoVariables.lbAValues[114] = 1.0000000000000000e+00;
acadoVariables.lbAValues[115] = 1.0000000000000000e+00;
acadoVariables.lbAValues[116] = 1.0000000000000000e+00;
acadoVariables.lbAValues[117] = 1.0000000000000000e+00;
acadoVariables.lbAValues[118] = 1.0000000000000000e+00;
acadoVariables.lbAValues[119] = 1.0000000000000000e+00;
acadoVariables.lbAValues[120] = 1.0000000000000000e+00;
acadoVariables.lbAValues[121] = 1.0000000000000000e+00;
acadoVariables.lbAValues[122] = 1.0000000000000000e+00;
acadoVariables.lbAValues[123] = 1.0000000000000000e+00;
acadoVariables.lbAValues[124] = 1.0000000000000000e+00;
acadoVariables.lbAValues[125] = 1.0000000000000000e+00;
acadoVariables.lbAValues[126] = 1.0000000000000000e+00;
acadoVariables.lbAValues[127] = 1.0000000000000000e+00;
acadoVariables.lbAValues[128] = 1.0000000000000000e+00;
acadoVariables.lbAValues[129] = 1.0000000000000000e+00;
acadoVariables.lbAValues[130] = 1.0000000000000000e+00;
acadoVariables.lbAValues[131] = 1.0000000000000000e+00;
acadoVariables.lbAValues[132] = 1.0000000000000000e+00;
acadoVariables.lbAValues[133] = 1.0000000000000000e+00;
acadoVariables.lbAValues[134] = 1.0000000000000000e+00;
acadoVariables.lbAValues[135] = 1.0000000000000000e+00;
acadoVariables.lbAValues[136] = 1.0000000000000000e+00;
acadoVariables.lbAValues[137] = 1.0000000000000000e+00;
acadoVariables.lbAValues[138] = 1.0000000000000000e+00;
acadoVariables.lbAValues[139] = 1.0000000000000000e+00;
acadoVariables.lbAValues[140] = 1.0000000000000000e+00;
acadoVariables.lbAValues[141] = 1.0000000000000000e+00;
acadoVariables.lbAValues[142] = 1.0000000000000000e+00;
acadoVariables.lbAValues[143] = 1.0000000000000000e+00;
acadoVariables.lbAValues[144] = 1.0000000000000000e+00;
acadoVariables.lbAValues[145] = 1.0000000000000000e+00;
acadoVariables.lbAValues[146] = 1.0000000000000000e+00;
acadoVariables.lbAValues[147] = 1.0000000000000000e+00;
acadoVariables.lbAValues[148] = 1.0000000000000000e+00;
acadoVariables.lbAValues[149] = 1.0000000000000000e+00;
acadoVariables.lbAValues[150] = 1.0000000000000000e+00;
acadoVariables.lbAValues[151] = 1.0000000000000000e+00;
acadoVariables.lbAValues[152] = 1.0000000000000000e+00;
acadoVariables.lbAValues[153] = 1.0000000000000000e+00;
acadoVariables.lbAValues[154] = 1.0000000000000000e+00;
acadoVariables.lbAValues[155] = 1.0000000000000000e+00;
acadoVariables.lbAValues[156] = 1.0000000000000000e+00;
acadoVariables.lbAValues[157] = 1.0000000000000000e+00;
acadoVariables.lbAValues[158] = 1.0000000000000000e+00;
acadoVariables.lbAValues[159] = 1.0000000000000000e+00;
acadoVariables.lbAValues[160] = 1.0000000000000000e+00;
acadoVariables.lbAValues[161] = 1.0000000000000000e+00;
acadoVariables.lbAValues[162] = 1.0000000000000000e+00;
acadoVariables.lbAValues[163] = 1.0000000000000000e+00;
acadoVariables.lbAValues[164] = 1.0000000000000000e+00;
acadoVariables.lbAValues[165] = 1.0000000000000000e+00;
acadoVariables.lbAValues[166] = 1.0000000000000000e+00;
acadoVariables.lbAValues[167] = 1.0000000000000000e+00;
acadoVariables.lbAValues[168] = 1.0000000000000000e+00;
acadoVariables.lbAValues[169] = 1.0000000000000000e+00;
acadoVariables.lbAValues[170] = 1.0000000000000000e+00;
acadoVariables.lbAValues[171] = 1.0000000000000000e+00;
acadoVariables.lbAValues[172] = 1.0000000000000000e+00;
acadoVariables.lbAValues[173] = 1.0000000000000000e+00;
acadoVariables.lbAValues[174] = 1.0000000000000000e+00;
acadoVariables.lbAValues[175] = 1.0000000000000000e+00;
acadoVariables.lbAValues[176] = 1.0000000000000000e+00;
acadoVariables.lbAValues[177] = 1.0000000000000000e+00;
acadoVariables.lbAValues[178] = 1.0000000000000000e+00;
acadoVariables.lbAValues[179] = 1.0000000000000000e+00;
acadoVariables.lbAValues[180] = 1.0000000000000000e+00;
acadoVariables.lbAValues[181] = 1.0000000000000000e+00;
acadoVariables.lbAValues[182] = 1.0000000000000000e+00;
acadoVariables.lbAValues[183] = 1.0000000000000000e+00;
acadoVariables.lbAValues[184] = 1.0000000000000000e+00;
acadoVariables.lbAValues[185] = 1.0000000000000000e+00;
acadoVariables.lbAValues[186] = 1.0000000000000000e+00;
acadoVariables.lbAValues[187] = 1.0000000000000000e+00;
acadoVariables.lbAValues[188] = 1.0000000000000000e+00;
acadoVariables.lbAValues[189] = 1.0000000000000000e+00;
acadoVariables.lbAValues[190] = 1.0000000000000000e+00;
acadoVariables.lbAValues[191] = 1.0000000000000000e+00;
acadoVariables.lbAValues[192] = 1.0000000000000000e+00;
acadoVariables.lbAValues[193] = 1.0000000000000000e+00;
acadoVariables.lbAValues[194] = 1.0000000000000000e+00;
acadoVariables.lbAValues[195] = 1.0000000000000000e+00;
acadoVariables.lbAValues[196] = 1.0000000000000000e+00;
acadoVariables.lbAValues[197] = 1.0000000000000000e+00;
acadoVariables.lbAValues[198] = 1.0000000000000000e+00;
acadoVariables.lbAValues[199] = 1.0000000000000000e+00;
acadoVariables.lbAValues[200] = 1.0000000000000000e+00;
acadoVariables.lbAValues[201] = 1.0000000000000000e+00;
acadoVariables.lbAValues[202] = 1.0000000000000000e+00;
acadoVariables.lbAValues[203] = 1.0000000000000000e+00;
acadoVariables.lbAValues[204] = 1.0000000000000000e+00;
acadoVariables.lbAValues[205] = 1.0000000000000000e+00;
acadoVariables.lbAValues[206] = 1.0000000000000000e+00;
acadoVariables.lbAValues[207] = 1.0000000000000000e+00;
acadoVariables.lbAValues[208] = 1.0000000000000000e+00;
acadoVariables.lbAValues[209] = 1.0000000000000000e+00;
acadoVariables.ubAValues[0] = 1.0000000000000000e+04;
acadoVariables.ubAValues[1] = 1.0000000000000000e+04;
acadoVariables.ubAValues[2] = 1.0000000000000000e+04;
acadoVariables.ubAValues[3] = 1.0000000000000000e+04;
acadoVariables.ubAValues[4] = 1.0000000000000000e+04;
acadoVariables.ubAValues[5] = 1.0000000000000000e+04;
acadoVariables.ubAValues[6] = 1.0000000000000000e+04;
acadoVariables.ubAValues[7] = 1.0000000000000000e+04;
acadoVariables.ubAValues[8] = 1.0000000000000000e+04;
acadoVariables.ubAValues[9] = 1.0000000000000000e+04;
acadoVariables.ubAValues[10] = 1.0000000000000000e+04;
acadoVariables.ubAValues[11] = 1.0000000000000000e+04;
acadoVariables.ubAValues[12] = 1.0000000000000000e+04;
acadoVariables.ubAValues[13] = 1.0000000000000000e+04;
acadoVariables.ubAValues[14] = 1.0000000000000000e+04;
acadoVariables.ubAValues[15] = 1.0000000000000000e+04;
acadoVariables.ubAValues[16] = 1.0000000000000000e+04;
acadoVariables.ubAValues[17] = 1.0000000000000000e+04;
acadoVariables.ubAValues[18] = 1.0000000000000000e+04;
acadoVariables.ubAValues[19] = 1.0000000000000000e+04;
acadoVariables.ubAValues[20] = 1.0000000000000000e+04;
acadoVariables.ubAValues[21] = 1.0000000000000000e+04;
acadoVariables.ubAValues[22] = 1.0000000000000000e+04;
acadoVariables.ubAValues[23] = 1.0000000000000000e+04;
acadoVariables.ubAValues[24] = 1.0000000000000000e+04;
acadoVariables.ubAValues[25] = 1.0000000000000000e+04;
acadoVariables.ubAValues[26] = 1.0000000000000000e+04;
acadoVariables.ubAValues[27] = 1.0000000000000000e+04;
acadoVariables.ubAValues[28] = 1.0000000000000000e+04;
acadoVariables.ubAValues[29] = 1.0000000000000000e+04;
acadoVariables.ubAValues[30] = 1.0000000000000000e+04;
acadoVariables.ubAValues[31] = 1.0000000000000000e+04;
acadoVariables.ubAValues[32] = 1.0000000000000000e+04;
acadoVariables.ubAValues[33] = 1.0000000000000000e+04;
acadoVariables.ubAValues[34] = 1.0000000000000000e+04;
acadoVariables.ubAValues[35] = 1.0000000000000000e+04;
acadoVariables.ubAValues[36] = 1.0000000000000000e+04;
acadoVariables.ubAValues[37] = 1.0000000000000000e+04;
acadoVariables.ubAValues[38] = 1.0000000000000000e+04;
acadoVariables.ubAValues[39] = 1.0000000000000000e+04;
acadoVariables.ubAValues[40] = 1.0000000000000000e+04;
acadoVariables.ubAValues[41] = 1.0000000000000000e+04;
acadoVariables.ubAValues[42] = 1.0000000000000000e+04;
acadoVariables.ubAValues[43] = 1.0000000000000000e+04;
acadoVariables.ubAValues[44] = 1.0000000000000000e+04;
acadoVariables.ubAValues[45] = 1.0000000000000000e+04;
acadoVariables.ubAValues[46] = 1.0000000000000000e+04;
acadoVariables.ubAValues[47] = 1.0000000000000000e+04;
acadoVariables.ubAValues[48] = 1.0000000000000000e+04;
acadoVariables.ubAValues[49] = 1.0000000000000000e+04;
acadoVariables.ubAValues[50] = 1.0000000000000000e+04;
acadoVariables.ubAValues[51] = 1.0000000000000000e+04;
acadoVariables.ubAValues[52] = 1.0000000000000000e+04;
acadoVariables.ubAValues[53] = 1.0000000000000000e+04;
acadoVariables.ubAValues[54] = 1.0000000000000000e+04;
acadoVariables.ubAValues[55] = 1.0000000000000000e+04;
acadoVariables.ubAValues[56] = 1.0000000000000000e+04;
acadoVariables.ubAValues[57] = 1.0000000000000000e+04;
acadoVariables.ubAValues[58] = 1.0000000000000000e+04;
acadoVariables.ubAValues[59] = 1.0000000000000000e+04;
acadoVariables.ubAValues[60] = 1.0000000000000000e+04;
acadoVariables.ubAValues[61] = 1.0000000000000000e+04;
acadoVariables.ubAValues[62] = 1.0000000000000000e+04;
acadoVariables.ubAValues[63] = 1.0000000000000000e+04;
acadoVariables.ubAValues[64] = 1.0000000000000000e+04;
acadoVariables.ubAValues[65] = 1.0000000000000000e+04;
acadoVariables.ubAValues[66] = 1.0000000000000000e+04;
acadoVariables.ubAValues[67] = 1.0000000000000000e+04;
acadoVariables.ubAValues[68] = 1.0000000000000000e+04;
acadoVariables.ubAValues[69] = 1.0000000000000000e+04;
acadoVariables.ubAValues[70] = 1.0000000000000000e+04;
acadoVariables.ubAValues[71] = 1.0000000000000000e+04;
acadoVariables.ubAValues[72] = 1.0000000000000000e+04;
acadoVariables.ubAValues[73] = 1.0000000000000000e+04;
acadoVariables.ubAValues[74] = 1.0000000000000000e+04;
acadoVariables.ubAValues[75] = 1.0000000000000000e+04;
acadoVariables.ubAValues[76] = 1.0000000000000000e+04;
acadoVariables.ubAValues[77] = 1.0000000000000000e+04;
acadoVariables.ubAValues[78] = 1.0000000000000000e+04;
acadoVariables.ubAValues[79] = 1.0000000000000000e+04;
acadoVariables.ubAValues[80] = 1.0000000000000000e+04;
acadoVariables.ubAValues[81] = 1.0000000000000000e+04;
acadoVariables.ubAValues[82] = 1.0000000000000000e+04;
acadoVariables.ubAValues[83] = 1.0000000000000000e+04;
acadoVariables.ubAValues[84] = 1.0000000000000000e+04;
acadoVariables.ubAValues[85] = 1.0000000000000000e+04;
acadoVariables.ubAValues[86] = 1.0000000000000000e+04;
acadoVariables.ubAValues[87] = 1.0000000000000000e+04;
acadoVariables.ubAValues[88] = 1.0000000000000000e+04;
acadoVariables.ubAValues[89] = 1.0000000000000000e+04;
acadoVariables.ubAValues[90] = 1.0000000000000000e+04;
acadoVariables.ubAValues[91] = 1.0000000000000000e+04;
acadoVariables.ubAValues[92] = 1.0000000000000000e+04;
acadoVariables.ubAValues[93] = 1.0000000000000000e+04;
acadoVariables.ubAValues[94] = 1.0000000000000000e+04;
acadoVariables.ubAValues[95] = 1.0000000000000000e+04;
acadoVariables.ubAValues[96] = 1.0000000000000000e+04;
acadoVariables.ubAValues[97] = 1.0000000000000000e+04;
acadoVariables.ubAValues[98] = 1.0000000000000000e+04;
acadoVariables.ubAValues[99] = 1.0000000000000000e+04;
acadoVariables.ubAValues[100] = 1.0000000000000000e+04;
acadoVariables.ubAValues[101] = 1.0000000000000000e+04;
acadoVariables.ubAValues[102] = 1.0000000000000000e+04;
acadoVariables.ubAValues[103] = 1.0000000000000000e+04;
acadoVariables.ubAValues[104] = 1.0000000000000000e+04;
acadoVariables.ubAValues[105] = 1.0000000000000000e+04;
acadoVariables.ubAValues[106] = 1.0000000000000000e+04;
acadoVariables.ubAValues[107] = 1.0000000000000000e+04;
acadoVariables.ubAValues[108] = 1.0000000000000000e+04;
acadoVariables.ubAValues[109] = 1.0000000000000000e+04;
acadoVariables.ubAValues[110] = 1.0000000000000000e+04;
acadoVariables.ubAValues[111] = 1.0000000000000000e+04;
acadoVariables.ubAValues[112] = 1.0000000000000000e+04;
acadoVariables.ubAValues[113] = 1.0000000000000000e+04;
acadoVariables.ubAValues[114] = 1.0000000000000000e+04;
acadoVariables.ubAValues[115] = 1.0000000000000000e+04;
acadoVariables.ubAValues[116] = 1.0000000000000000e+04;
acadoVariables.ubAValues[117] = 1.0000000000000000e+04;
acadoVariables.ubAValues[118] = 1.0000000000000000e+04;
acadoVariables.ubAValues[119] = 1.0000000000000000e+04;
acadoVariables.ubAValues[120] = 1.0000000000000000e+04;
acadoVariables.ubAValues[121] = 1.0000000000000000e+04;
acadoVariables.ubAValues[122] = 1.0000000000000000e+04;
acadoVariables.ubAValues[123] = 1.0000000000000000e+04;
acadoVariables.ubAValues[124] = 1.0000000000000000e+04;
acadoVariables.ubAValues[125] = 1.0000000000000000e+04;
acadoVariables.ubAValues[126] = 1.0000000000000000e+04;
acadoVariables.ubAValues[127] = 1.0000000000000000e+04;
acadoVariables.ubAValues[128] = 1.0000000000000000e+04;
acadoVariables.ubAValues[129] = 1.0000000000000000e+04;
acadoVariables.ubAValues[130] = 1.0000000000000000e+04;
acadoVariables.ubAValues[131] = 1.0000000000000000e+04;
acadoVariables.ubAValues[132] = 1.0000000000000000e+04;
acadoVariables.ubAValues[133] = 1.0000000000000000e+04;
acadoVariables.ubAValues[134] = 1.0000000000000000e+04;
acadoVariables.ubAValues[135] = 1.0000000000000000e+04;
acadoVariables.ubAValues[136] = 1.0000000000000000e+04;
acadoVariables.ubAValues[137] = 1.0000000000000000e+04;
acadoVariables.ubAValues[138] = 1.0000000000000000e+04;
acadoVariables.ubAValues[139] = 1.0000000000000000e+04;
acadoVariables.ubAValues[140] = 1.0000000000000000e+04;
acadoVariables.ubAValues[141] = 1.0000000000000000e+04;
acadoVariables.ubAValues[142] = 1.0000000000000000e+04;
acadoVariables.ubAValues[143] = 1.0000000000000000e+04;
acadoVariables.ubAValues[144] = 1.0000000000000000e+04;
acadoVariables.ubAValues[145] = 1.0000000000000000e+04;
acadoVariables.ubAValues[146] = 1.0000000000000000e+04;
acadoVariables.ubAValues[147] = 1.0000000000000000e+04;
acadoVariables.ubAValues[148] = 1.0000000000000000e+04;
acadoVariables.ubAValues[149] = 1.0000000000000000e+04;
acadoVariables.ubAValues[150] = 1.0000000000000000e+04;
acadoVariables.ubAValues[151] = 1.0000000000000000e+04;
acadoVariables.ubAValues[152] = 1.0000000000000000e+04;
acadoVariables.ubAValues[153] = 1.0000000000000000e+04;
acadoVariables.ubAValues[154] = 1.0000000000000000e+04;
acadoVariables.ubAValues[155] = 1.0000000000000000e+04;
acadoVariables.ubAValues[156] = 1.0000000000000000e+04;
acadoVariables.ubAValues[157] = 1.0000000000000000e+04;
acadoVariables.ubAValues[158] = 1.0000000000000000e+04;
acadoVariables.ubAValues[159] = 1.0000000000000000e+04;
acadoVariables.ubAValues[160] = 1.0000000000000000e+04;
acadoVariables.ubAValues[161] = 1.0000000000000000e+04;
acadoVariables.ubAValues[162] = 1.0000000000000000e+04;
acadoVariables.ubAValues[163] = 1.0000000000000000e+04;
acadoVariables.ubAValues[164] = 1.0000000000000000e+04;
acadoVariables.ubAValues[165] = 1.0000000000000000e+04;
acadoVariables.ubAValues[166] = 1.0000000000000000e+04;
acadoVariables.ubAValues[167] = 1.0000000000000000e+04;
acadoVariables.ubAValues[168] = 1.0000000000000000e+04;
acadoVariables.ubAValues[169] = 1.0000000000000000e+04;
acadoVariables.ubAValues[170] = 1.0000000000000000e+04;
acadoVariables.ubAValues[171] = 1.0000000000000000e+04;
acadoVariables.ubAValues[172] = 1.0000000000000000e+04;
acadoVariables.ubAValues[173] = 1.0000000000000000e+04;
acadoVariables.ubAValues[174] = 1.0000000000000000e+04;
acadoVariables.ubAValues[175] = 1.0000000000000000e+04;
acadoVariables.ubAValues[176] = 1.0000000000000000e+04;
acadoVariables.ubAValues[177] = 1.0000000000000000e+04;
acadoVariables.ubAValues[178] = 1.0000000000000000e+04;
acadoVariables.ubAValues[179] = 1.0000000000000000e+04;
acadoVariables.ubAValues[180] = 1.0000000000000000e+04;
acadoVariables.ubAValues[181] = 1.0000000000000000e+04;
acadoVariables.ubAValues[182] = 1.0000000000000000e+04;
acadoVariables.ubAValues[183] = 1.0000000000000000e+04;
acadoVariables.ubAValues[184] = 1.0000000000000000e+04;
acadoVariables.ubAValues[185] = 1.0000000000000000e+04;
acadoVariables.ubAValues[186] = 1.0000000000000000e+04;
acadoVariables.ubAValues[187] = 1.0000000000000000e+04;
acadoVariables.ubAValues[188] = 1.0000000000000000e+04;
acadoVariables.ubAValues[189] = 1.0000000000000000e+04;
acadoVariables.ubAValues[190] = 1.0000000000000000e+04;
acadoVariables.ubAValues[191] = 1.0000000000000000e+04;
acadoVariables.ubAValues[192] = 1.0000000000000000e+04;
acadoVariables.ubAValues[193] = 1.0000000000000000e+04;
acadoVariables.ubAValues[194] = 1.0000000000000000e+04;
acadoVariables.ubAValues[195] = 1.0000000000000000e+04;
acadoVariables.ubAValues[196] = 1.0000000000000000e+04;
acadoVariables.ubAValues[197] = 1.0000000000000000e+04;
acadoVariables.ubAValues[198] = 1.0000000000000000e+04;
acadoVariables.ubAValues[199] = 1.0000000000000000e+04;
acadoVariables.ubAValues[200] = 1.0000000000000000e+04;
acadoVariables.ubAValues[201] = 1.0000000000000000e+04;
acadoVariables.ubAValues[202] = 1.0000000000000000e+04;
acadoVariables.ubAValues[203] = 1.0000000000000000e+04;
acadoVariables.ubAValues[204] = 1.0000000000000000e+04;
acadoVariables.ubAValues[205] = 1.0000000000000000e+04;
acadoVariables.ubAValues[206] = 1.0000000000000000e+04;
acadoVariables.ubAValues[207] = 1.0000000000000000e+04;
acadoVariables.ubAValues[208] = 1.0000000000000000e+04;
acadoVariables.ubAValues[209] = 1.0000000000000000e+04;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[28] = acadoVariables.u[index * 2];
acadoWorkspace.state[29] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[30] = acadoVariables.od[index * 15];
acadoWorkspace.state[31] = acadoVariables.od[index * 15 + 1];
acadoWorkspace.state[32] = acadoVariables.od[index * 15 + 2];
acadoWorkspace.state[33] = acadoVariables.od[index * 15 + 3];
acadoWorkspace.state[34] = acadoVariables.od[index * 15 + 4];
acadoWorkspace.state[35] = acadoVariables.od[index * 15 + 5];
acadoWorkspace.state[36] = acadoVariables.od[index * 15 + 6];
acadoWorkspace.state[37] = acadoVariables.od[index * 15 + 7];
acadoWorkspace.state[38] = acadoVariables.od[index * 15 + 8];
acadoWorkspace.state[39] = acadoVariables.od[index * 15 + 9];
acadoWorkspace.state[40] = acadoVariables.od[index * 15 + 10];
acadoWorkspace.state[41] = acadoVariables.od[index * 15 + 11];
acadoWorkspace.state[42] = acadoVariables.od[index * 15 + 12];
acadoWorkspace.state[43] = acadoVariables.od[index * 15 + 13];
acadoWorkspace.state[44] = acadoVariables.od[index * 15 + 14];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 4 + 4] = acadoWorkspace.state[0];
acadoVariables.x[index * 4 + 5] = acadoWorkspace.state[1];
acadoVariables.x[index * 4 + 6] = acadoWorkspace.state[2];
acadoVariables.x[index * 4 + 7] = acadoWorkspace.state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[120] = xEnd[0];
acadoVariables.x[121] = xEnd[1];
acadoVariables.x[122] = xEnd[2];
acadoVariables.x[123] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[120];
acadoWorkspace.state[1] = acadoVariables.x[121];
acadoWorkspace.state[2] = acadoVariables.x[122];
acadoWorkspace.state[3] = acadoVariables.x[123];
if (uEnd != 0)
{
acadoWorkspace.state[28] = uEnd[0];
acadoWorkspace.state[29] = uEnd[1];
}
else
{
acadoWorkspace.state[28] = acadoVariables.u[58];
acadoWorkspace.state[29] = acadoVariables.u[59];
}
acadoWorkspace.state[30] = acadoVariables.od[450];
acadoWorkspace.state[31] = acadoVariables.od[451];
acadoWorkspace.state[32] = acadoVariables.od[452];
acadoWorkspace.state[33] = acadoVariables.od[453];
acadoWorkspace.state[34] = acadoVariables.od[454];
acadoWorkspace.state[35] = acadoVariables.od[455];
acadoWorkspace.state[36] = acadoVariables.od[456];
acadoWorkspace.state[37] = acadoVariables.od[457];
acadoWorkspace.state[38] = acadoVariables.od[458];
acadoWorkspace.state[39] = acadoVariables.od[459];
acadoWorkspace.state[40] = acadoVariables.od[460];
acadoWorkspace.state[41] = acadoVariables.od[461];
acadoWorkspace.state[42] = acadoVariables.od[462];
acadoWorkspace.state[43] = acadoVariables.od[463];
acadoWorkspace.state[44] = acadoVariables.od[464];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[120] = acadoWorkspace.state[0];
acadoVariables.x[121] = acadoWorkspace.state[1];
acadoVariables.x[122] = acadoWorkspace.state[2];
acadoVariables.x[123] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[58] = uEnd[0];
acadoVariables.u[59] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 210; ++index)
{
prd = acadoWorkspace.y[index + 60];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 12 */
real_t tmpDy[ 12 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 15];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 15 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 15 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 15 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 15 + 4];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 15 + 5];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 15 + 6];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 15 + 7];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 15 + 8];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 15 + 9];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 15 + 10];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 15 + 11];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 15 + 12];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 15 + 13];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 15 + 14];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 12] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 12];
acadoWorkspace.Dy[lRun1 * 12 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 12 + 1];
acadoWorkspace.Dy[lRun1 * 12 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 12 + 2];
acadoWorkspace.Dy[lRun1 * 12 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 12 + 3];
acadoWorkspace.Dy[lRun1 * 12 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 12 + 4];
acadoWorkspace.Dy[lRun1 * 12 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 12 + 5];
acadoWorkspace.Dy[lRun1 * 12 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 12 + 6];
acadoWorkspace.Dy[lRun1 * 12 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 12 + 7];
acadoWorkspace.Dy[lRun1 * 12 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 12 + 8];
acadoWorkspace.Dy[lRun1 * 12 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 12 + 9];
acadoWorkspace.Dy[lRun1 * 12 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 12 + 10];
acadoWorkspace.Dy[lRun1 * 12 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 12 + 11];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.od[450];
acadoWorkspace.objValueIn[5] = acadoVariables.od[451];
acadoWorkspace.objValueIn[6] = acadoVariables.od[452];
acadoWorkspace.objValueIn[7] = acadoVariables.od[453];
acadoWorkspace.objValueIn[8] = acadoVariables.od[454];
acadoWorkspace.objValueIn[9] = acadoVariables.od[455];
acadoWorkspace.objValueIn[10] = acadoVariables.od[456];
acadoWorkspace.objValueIn[11] = acadoVariables.od[457];
acadoWorkspace.objValueIn[12] = acadoVariables.od[458];
acadoWorkspace.objValueIn[13] = acadoVariables.od[459];
acadoWorkspace.objValueIn[14] = acadoVariables.od[460];
acadoWorkspace.objValueIn[15] = acadoVariables.od[461];
acadoWorkspace.objValueIn[16] = acadoVariables.od[462];
acadoWorkspace.objValueIn[17] = acadoVariables.od[463];
acadoWorkspace.objValueIn[18] = acadoVariables.od[464];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[13];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[26];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[39];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[52];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[65];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[78];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[91];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[104];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[117];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[130];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[143];
objVal += + acadoWorkspace.Dy[lRun1 * 12]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 12 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 12 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 12 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 12 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 12 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 12 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 12 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 12 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 12 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 12 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 12 + 11]*tmpDy[11];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[4];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[8];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

