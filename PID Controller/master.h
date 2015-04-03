#ifndef MASTER_H
#define MASTER_H

#include <avr/pgmspace.h>

volatile uint8_t index = 0;
//Position Configuration Constants of Potentiometers
//Procedure to Measure
// 1 - Move potentiometer arm to 0 degrees
// 2 - Record the value measured by microcontroller
// 3 - Repeat for 0 and 90 degrees for both potentiometers
float v_left_0 = 140;
float v_left_90 = 519;
float v_right_0 = 127;
float v_right_90 = 519;

volatile float x1, y1, x2, y2, m, d, e, mtheta, p, xE, yE;

//Potentiometer Angles
volatile float q1=0.0;
volatile float q2=0.0;

//5 Bar Constants
float a=25;
float l=60;
float k=90;

//Look-up Table Size
const int N = 256;

//Indexing Increments for Look-up Tables
float inc_sin = (float)(100-(-45))/(N - 1);//Max Angle: 100 degrees, Min Angle: -45 degrees
float inc_cos = (float)(100-(-45))/(N - 1);//Max Angle: 100 degrees, Min Angle: -45 degrees
float inc_acos = (float)(1-0)/(N - 1);//Max Value: 1, Min Value: 0
float inc_atan = (float)(1-(-1))/(N - 1);//Max Value 1, Min Value -1

//Look up tables

const float sin_table[N] PROGMEM =
{-0.7071,-0.7001,-0.6929,-0.6857,-0.6785,-0.6712,-0.6638,-0.6563,-0.6488,-0.6412,
	-0.6336,-0.6259,-0.6181,-0.6103,-0.6024,-0.5944,-0.5864,-0.5783,-0.5702,-0.5620,
	-0.5538,-0.5455,-0.5372,-0.5288,-0.5203,-0.5118,-0.5033,-0.4947,-0.4860,-0.4773,
	-0.4686,-0.4598,-0.4509,-0.4421,-0.4331,-0.4242,-0.4152,-0.4061,-0.3970,-0.3879,
	-0.3787,-0.3695,-0.3603,-0.3510,-0.3417,-0.3324,-0.3230,-0.3136,-0.3041,-0.2947,
	-0.2852,-0.2756,-0.2661,-0.2565,-0.2469,-0.2373,-0.2276,-0.2179,-0.2082,-0.1985,
	-0.1888,-0.1790,-0.1693,-0.1595,-0.1497,-0.1399,-0.1300,-0.1202,-0.1103,-0.1004,
	-0.0906,-0.0807,-0.0708,-0.0609,-0.0510,-0.0411,-0.0311,-0.0212,-0.0113,-0.0014,
	0.0086,0.0185,0.0284,0.0383,0.0482,0.0581,0.0680,0.0779,0.0878,0.0977,
	0.1076,0.1175,0.1273,0.1371,0.1470,0.1568,0.1666,0.1763,0.1861,0.1958,
	0.2056,0.2153,0.2250,0.2346,0.2442,0.2539,0.2634,0.2730,0.2825,0.2920,
	0.3015,0.3110,0.3204,0.3298,0.3391,0.3484,0.3577,0.3670,0.3762,0.3854,
	0.3945,0.4036,0.4127,0.4217,0.4307,0.4396,0.4485,0.4573,0.4661,0.4749,
	0.4836,0.4923,0.5009,0.5095,0.5180,0.5264,0.5348,0.5432,0.5515,0.5598,
	0.5680,0.5761,0.5842,0.5922,0.6002,0.6081,0.6159,0.6237,0.6314,0.6391,
	0.6467,0.6542,0.6617,0.6691,0.6765,0.6837,0.6910,0.6981,0.7052,0.7122,
	0.7191,0.7260,0.7328,0.7395,0.7461,0.7527,0.7592,0.7656,0.7720,0.7782,
	0.7844,0.7905,0.7966,0.8025,0.8084,0.8142,0.8199,0.8256,0.8311,0.8366,
	0.8420,0.8473,0.8526,0.8577,0.8628,0.8677,0.8726,0.8774,0.8821,0.8868,
	0.8913,0.8958,0.9001,0.9044,0.9086,0.9127,0.9167,0.9206,0.9245,0.9282,
	0.9319,0.9354,0.9389,0.9422,0.9455,0.9487,0.9518,0.9548,0.9577,0.9605,
	0.9632,0.9658,0.9684,0.9708,0.9731,0.9754,0.9775,0.9795,0.9815,0.9833,
	0.9851,0.9868,0.9883,0.9898,0.9912,0.9924,0.9936,0.9947,0.9956,0.9965,
	0.9973,0.9980,0.9986,0.9990,0.9994,0.9997,0.9999,1.0000,1.0000,0.9999,
	0.9997,0.9994,0.9990,0.9985,0.9979,0.9972,0.9964,0.9955,0.9945,0.9934,
0.9922,0.9909,0.9895,0.9881,0.9865,0.9848};

const float cos_table[N] PROGMEM =
{0.7071,0.7141,0.7210,0.7278,0.7346,0.7413,0.7479,0.7545,0.7610,0.7674,
	0.7737,0.7799,0.7861,0.7922,0.7982,0.8042,0.8100,0.8158,0.8215,0.8271,
	0.8327,0.8381,0.8435,0.8488,0.8540,0.8591,0.8641,0.8691,0.8740,0.8787,
	0.8834,0.8880,0.8926,0.8970,0.9013,0.9056,0.9097,0.9138,0.9178,0.9217,
	0.9255,0.9292,0.9328,0.9364,0.9398,0.9432,0.9464,0.9496,0.9526,0.9556,
	0.9585,0.9613,0.9639,0.9665,0.9690,0.9714,0.9738,0.9760,0.9781,0.9801,
	0.9820,0.9838,0.9856,0.9872,0.9887,0.9902,0.9915,0.9928,0.9939,0.9949,
	0.9959,0.9967,0.9975,0.9981,0.9987,0.9992,0.9995,0.9998,0.9999,1.0000,
	1.0000,0.9998,0.9996,0.9993,0.9988,0.9983,0.9977,0.9970,0.9961,0.9952,
	0.9942,0.9931,0.9919,0.9906,0.9891,0.9876,0.9860,0.9843,0.9825,0.9806,
	0.9786,0.9766,0.9744,0.9721,0.9697,0.9672,0.9647,0.9620,0.9593,0.9564,
	0.9535,0.9504,0.9473,0.9441,0.9407,0.9373,0.9338,0.9302,0.9265,0.9228,
	0.9189,0.9149,0.9109,0.9067,0.9025,0.8982,0.8938,0.8893,0.8847,0.8800,
	0.8753,0.8704,0.8655,0.8605,0.8554,0.8502,0.8450,0.8396,0.8342,0.8287,
	0.8231,0.8174,0.8116,0.8058,0.7999,0.7939,0.7878,0.7816,0.7754,0.7691,
	0.7627,0.7563,0.7497,0.7431,0.7365,0.7297,0.7229,0.7160,0.7090,0.7020,
	0.6949,0.6877,0.6805,0.6732,0.6658,0.6584,0.6509,0.6433,0.6357,0.6280,
	0.6202,0.6124,0.6045,0.5966,0.5886,0.5806,0.5725,0.5643,0.5561,0.5478,
	0.5395,0.5311,0.5226,0.5142,0.5056,0.4970,0.4884,0.4797,0.4710,0.4622,
	0.4534,0.4445,0.4356,0.4266,0.4176,0.4086,0.3995,0.3904,0.3813,0.3721,
	0.3628,0.3536,0.3443,0.3349,0.3256,0.3162,0.3067,0.2973,0.2878,0.2783,
	0.2687,0.2591,0.2496,0.2399,0.2303,0.2206,0.2109,0.2012,0.1915,0.1817,
	0.1720,0.1622,0.1524,0.1426,0.1327,0.1229,0.1130,0.1032,0.0933,0.0834,
	0.0735,0.0636,0.0537,0.0438,0.0339,0.0240,0.0140,0.0041,-0.0058,-0.0157,
	-0.0257,-0.0356,-0.0455,-0.0554,-0.0653,-0.0752,-0.0851,-0.0950,-0.1049,-0.1147,
-0.1246,-0.1344,-0.1443,-0.1541,-0.1639,-0.1736};

const float acos_table[N] PROGMEM =
{90.0000,89.7753,89.5506,89.3259,89.1012,88.8765,88.6517,88.4270,88.2022,87.9774,
	87.7525,87.5277,87.3027,87.0778,86.8528,86.6277,86.4026,86.1774,85.9522,85.7269,
	85.5016,85.2762,85.0507,84.8251,84.5995,84.3737,84.1479,83.9220,83.6960,83.4699,
	83.2437,83.0174,82.7909,82.5644,82.3377,82.1110,81.8841,81.6570,81.4299,81.2026,
	80.9752,80.7476,80.5199,80.2920,80.0639,79.8358,79.6074,79.3789,79.1502,78.9213,
	78.6923,78.4630,78.2336,78.0040,77.7742,77.5442,77.3140,77.0836,76.8530,76.6221,
	76.3910,76.1597,75.9282,75.6965,75.4645,75.2322,74.9997,74.7670,74.5340,74.3007,
	74.0672,73.8334,73.5993,73.3650,73.1303,72.8954,72.6601,72.4246,72.1887,71.9526,
	71.7161,71.4793,71.2422,71.0047,70.7669,70.5288,70.2903,70.0514,69.8122,69.5726,
	69.3327,69.0923,68.8516,68.6105,68.3690,68.1271,67.8848,67.6420,67.3989,67.1553,
	66.9112,66.6667,66.4218,66.1764,65.9306,65.6843,65.4375,65.1902,64.9424,64.6941,
	64.4453,64.1960,63.9462,63.6958,63.4449,63.1934,62.9414,62.6888,62.4356,62.1819,
	61.9275,61.6726,61.4170,61.1608,60.9040,60.6465,60.3884,60.1296,59.8702,59.6101,
	59.3492,59.0877,58.8254,58.5625,58.2988,58.0343,57.7690,57.5030,57.2362,56.9686,
	56.7002,56.4310,56.1609,55.8899,55.6181,55.3454,55.0718,54.7973,54.5219,54.2455,
	53.9681,53.6898,53.4105,53.1301,52.8487,52.5663,52.2828,51.9982,51.7125,51.4257,
	51.1377,50.8485,50.5582,50.2666,49.9738,49.6798,49.3844,49.0878,48.7898,48.4904,
	48.1897,47.8875,47.5839,47.2788,46.9722,46.6641,46.3544,46.0431,45.7301,45.4155,
	45.0991,44.7810,44.4612,44.1394,43.8159,43.4904,43.1629,42.8334,42.5019,42.1683,
	41.8325,41.4945,41.1542,40.8116,40.4666,40.1192,39.7692,39.4167,39.0614,38.7035,
	38.3427,37.9791,37.6125,37.2428,36.8699,36.4938,36.1143,35.7313,35.3447,34.9544,
	34.5603,34.1622,33.7600,33.3535,32.9426,32.5271,32.1068,31.6815,31.2511,30.8153,
	30.3738,29.9264,29.4729,29.0130,28.5463,28.0725,27.5912,27.1021,26.6047,26.0985,
	25.5830,25.0576,24.5217,23.9746,23.4155,22.8435,22.2576,21.6567,21.0395,20.4045,
	19.7499,19.0739,18.3739,17.6472,16.8903,16.0989,15.2677,14.3898,13.4560,12.4537,
11.3649,10.1617,8.7974,7.1807,5.0759,0.0000};

const float atan_table[N] PROGMEM =
{-45.0000,-44.7744,-44.5471,-44.3179,-44.0870,-43.8542,-43.6196,-43.3832,-43.1449,-42.9047,
	-42.6627,-42.4187,-42.1729,-41.9251,-41.6754,-41.4237,-41.1700,-40.9144,-40.6568,-40.3971,
	-40.1355,-39.8718,-39.6061,-39.3383,-39.0685,-38.7966,-38.5225,-38.2464,-37.9682,-37.6878,
	-37.4054,-37.1207,-36.8339,-36.5450,-36.2538,-35.9605,-35.6650,-35.3673,-35.0673,-34.7652,
	-34.4608,-34.1542,-33.8453,-33.5342,-33.2209,-32.9052,-32.5874,-32.2672,-31.9448,-31.6200,
	-31.2930,-30.9638,-30.6322,-30.2983,-29.9622,-29.6237,-29.2830,-28.9400,-28.5947,-28.2471,
	-27.8973,-27.5451,-27.1907,-26.8340,-26.4751,-26.1139,-25.7505,-25.3848,-25.0169,-24.6468,
	-24.2744,-23.8999,-23.5232,-23.1443,-22.7633,-22.3801,-21.9948,-21.6075,-21.2180,-20.8264,
	-20.4328,-20.0372,-19.6396,-19.2400,-18.8384,-18.4349,-18.0296,-17.6223,-17.2132,-16.8023,
	-16.3895,-15.9751,-15.5589,-15.1410,-14.7214,-14.3003,-13.8775,-13.4532,-13.0274,-12.6002,
	-12.1715,-11.7414,-11.3099,-10.8772,-10.4432,-10.0080,-9.5716,-9.1341,-8.6955,-8.2559,
	-7.8153,-7.3738,-6.9314,-6.4881,-6.0441,-5.5993,-5.1539,-4.7079,-4.2612,-3.8141,
	-3.3665,-2.9184,-2.4701,-2.0214,-1.5724,-1.1233,-0.6740,-0.2247,0.2247,0.6740,
	1.1233,1.5724,2.0214,2.4701,2.9184,3.3665,3.8141,4.2612,4.7079,5.1539,
	5.5993,6.0441,6.4881,6.9314,7.3738,7.8153,8.2559,8.6955,9.1341,9.5716,
	10.0080,10.4432,10.8772,11.3099,11.7414,12.1715,12.6002,13.0274,13.4532,13.8775,
	14.3003,14.7214,15.1410,15.5589,15.9751,16.3895,16.8023,17.2132,17.6223,18.0296,
	18.4349,18.8384,19.2400,19.6396,20.0372,20.4328,20.8264,21.2180,21.6075,21.9948,
	22.3801,22.7633,23.1443,23.5232,23.8999,24.2744,24.6468,25.0169,25.3848,25.7505,
	26.1139,26.4751,26.8340,27.1907,27.5451,27.8973,28.2471,28.5947,28.9400,29.2830,
	29.6237,29.9622,30.2983,30.6322,30.9638,31.2930,31.6200,31.9448,32.2672,32.5874,
	32.9052,33.2209,33.5342,33.8453,34.1542,34.4608,34.7652,35.0673,35.3673,35.6650,
	35.9605,36.2538,36.5450,36.8339,37.1207,37.4054,37.6878,37.9682,38.2464,38.5225,
	38.7966,39.0685,39.3383,39.6061,39.8718,40.1355,40.3971,40.6568,40.9144,41.1700,
	41.4237,41.6754,41.9251,42.1729,42.4187,42.6627,42.9047,43.1449,43.3832,43.6196,
43.8542,44.0870,44.3179,44.5471,44.7744,45.0000};

void translate(float left_value, float right_value, volatile float *xOut, volatile float *yOut) {
	 
	 // [Voltage]->[Angle] by a linear function y=mx+b
	 q1 = 90 / (2*(v_left_90-v_left_0)) * (left_value-v_left_0);
	 q2 = 90 / (2*(v_right_90-v_right_0)) * (right_value-v_right_0);
	 
	 //--------------------------------------------------------
	 //Forward Kinematics [q1,q2]->[xE,yE]
	 index = q1 / inc_cos + 80; // Get Index for sin()
	 x1 = a + l * pgm_read_float(&(cos_table[index]));
	 
	 index = q2 / inc_cos + 80; // Get Index for cos()
	 x2 = -a -l  * pgm_read_float(&(cos_table[index]));
	 
	 index = q1 / inc_sin + 80; // Get Index for sin()
	 y1 = l * pgm_read_float(&(sin_table[index]));
	 
	 index = q2 / inc_sin + 80; // Get Index for cos()
	 y2 = l * pgm_read_float(&(sin_table[index]));
	 
	 m = (y1-y2)/(x1-x2);
	 d = sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
	 e = ( d )/(2*k);
	 
	 index = e/inc_acos;//Get Index for acos()
	 mtheta = pgm_read_float(&(acos_table[ index ]));
	 
	 index = m/inc_atan + N/2;//Get Index for atan()
	 p = mtheta - pgm_read_float(&(atan_table[index]));
	 
	 index = p/inc_cos+80;//Get Index for cos()
	 xE = x1 -k * pgm_read_float(&(cos_table[index]));
	 
	 index = p/inc_sin+80;//Get Index for sin()
	 yE = y1 + k * pgm_read_float(&(sin_table[index]));
	 
	 *xOut = xE;
	 *yOut = yE - 56;
}

#endif