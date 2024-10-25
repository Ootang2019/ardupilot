
#pragma once
#ifndef __NN_PARAMETERS_DEF_H__
#define __NN_PARAMETERS_DEF_H__
#include <vector>

namespace NN{

static constexpr int N_ACT = 3;
static constexpr int N_OBS = 12;
static constexpr int N_TASK = 5;
static constexpr int N_CONTEXT = 2;
static constexpr int N_LATENT = 2;
static constexpr int N_HIDDEN = 16;
static constexpr int N_TCN_LAYER = 1;
static constexpr int N_TCN_HIDDEN = 16;
static constexpr int N_KERNEL = 2;
static constexpr int N_STACK = 25;
static constexpr int N_PADD = 1;

static constexpr float AVEL_LIM = 25.100000;
static constexpr float VEL_LIM = 30.000000;
static constexpr float POS_LIM = 999999.900000;
static constexpr float AUTHORITY = 0.100000;

std::vector<float> OBS = {
0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, };
std::vector<float> TASK = {
0.200000, 0.200000, 0.200000, 0.200000, 0.200000, };

std::vector<std::vector<std::vector<float>>> CNN_W1 = {
{
{-0.016408, -0.116827, },
{-0.155324, -0.109530, },
{-0.030761, -0.025734, },
{0.014996, 0.143095, },
{-0.132703, 0.151410, },
{-0.030715, 0.015744, },
{-0.050181, -0.024236, },
{-0.100711, -0.102877, },
{-0.005630, -0.000447, },
{-0.018454, 0.546818, },
{0.082951, 0.155209, },
{-0.071895, 0.333650, },
},
{
{-0.100400, 0.065843, },
{0.013666, -0.080675, },
{-0.139465, -0.054916, },
{0.110339, -0.103853, },
{-0.125621, -0.080124, },
{0.117722, -0.094025, },
{0.151044, 0.119233, },
{0.120752, -0.181791, },
{-0.135258, 0.128239, },
{0.007443, -0.090717, },
{-0.066294, 0.005507, },
{0.027490, 0.007946, },
},
{
{-0.057161, -0.257522, },
{0.007555, -0.009684, },
{-0.043739, -0.239057, },
{0.075125, 0.251221, },
{-0.003076, 0.018144, },
{0.056028, 0.189818, },
{-0.018936, -0.047170, },
{0.033289, 0.022112, },
{-0.011849, 0.016639, },
{-0.015969, -0.103338, },
{-0.057593, 0.659641, },
{-0.078234, -0.125633, },
},
{
{0.054365, 0.005027, },
{-0.148931, -0.011300, },
{0.113918, 0.079411, },
{-0.121728, -0.014788, },
{0.192640, -0.077689, },
{0.070040, -0.148905, },
{0.040316, -0.095665, },
{0.080065, -0.025238, },
{0.133912, -0.078243, },
{-0.086046, -0.001105, },
{-0.002679, -0.005489, },
{0.033120, -0.079786, },
},
{
{-0.001491, -0.069141, },
{-0.026610, 0.022672, },
{0.126429, 0.019724, },
{-0.055197, -0.102388, },
{-0.108806, 0.043834, },
{-0.063187, 0.044095, },
{-0.052713, 0.006343, },
{0.116245, 0.084861, },
{0.176818, -0.128012, },
{0.076348, 0.012316, },
{0.008460, 0.016640, },
{-0.038322, -0.100803, },
},
{
{0.021170, 0.227053, },
{-0.134754, -0.040959, },
{0.063650, 0.121561, },
{-0.008151, -0.139718, },
{-0.005396, 0.053643, },
{-0.005641, 0.129048, },
{0.000128, 0.143900, },
{-0.025457, 0.106831, },
{-0.111541, 0.118749, },
{-0.064925, -0.055890, },
{0.042550, -0.062207, },
{-0.026552, 0.009911, },
},
{
{-0.104535, -0.095871, },
{0.050885, -0.189073, },
{0.140993, -0.213141, },
{-0.004671, 0.120934, },
{-0.062161, 0.178543, },
{-0.038002, -0.000907, },
{0.149311, -0.010846, },
{0.112448, -0.024968, },
{-0.147334, 0.141558, },
{-0.022365, 0.115558, },
{0.069655, 0.165536, },
{0.070115, 0.022317, },
},
{
{0.098412, 0.008528, },
{0.055675, 0.070994, },
{0.112529, 0.024515, },
{0.040887, -0.097808, },
{-0.099531, 0.012678, },
{0.011784, 0.001996, },
{-0.074693, 0.136423, },
{0.035527, 0.075664, },
{0.232383, -0.100268, },
{0.013797, -0.038447, },
{-0.000639, -0.055207, },
{-0.062747, -0.005975, },
},
{
{0.273688, -0.195892, },
{0.303660, 0.209701, },
{0.020135, -0.177951, },
{-0.265915, 0.193519, },
{-0.296582, -0.226185, },
{-0.024620, 0.174925, },
{-0.237637, -0.336450, },
{0.293666, 0.196280, },
{-0.390855, -0.269462, },
{0.043113, -0.296347, },
{-0.054079, 0.038267, },
{0.055870, -0.245931, },
},
{
{-0.139708, 0.102088, },
{-0.101043, 0.004614, },
{-0.012150, 0.115167, },
{0.102192, -0.030771, },
{-0.124824, -0.012971, },
{-0.098872, -0.014893, },
{0.178274, 0.142150, },
{0.009820, -0.012536, },
{0.155494, 0.086735, },
{-0.023506, -0.090937, },
{-0.009839, 0.137837, },
{-0.129708, 0.007812, },
},
{
{0.058969, 0.038147, },
{-0.006343, -0.145031, },
{-0.058945, 0.045682, },
{0.095498, -0.143153, },
{-0.143659, -0.040666, },
{-0.216464, -0.065760, },
{-0.124572, 0.119502, },
{0.200383, -0.123898, },
{0.023232, -0.043307, },
{0.107519, -0.176954, },
{-0.051955, -0.148339, },
{-0.161078, 0.002825, },
},
{
{-0.210387, 0.049777, },
{0.180755, 0.115929, },
{0.045693, -0.249306, },
{0.042964, 0.275385, },
{-0.004242, 0.099205, },
{-0.013946, 0.081143, },
{-0.043086, -0.130875, },
{-0.037452, -0.038014, },
{-0.164529, -0.077231, },
{-0.053631, 0.040767, },
{-0.041021, 0.164096, },
{-0.106897, -0.416187, },
},
};

std::vector<float> CNN_B1 = {
0.193416, 0.058597, 0.027406, 0.023454, -0.202878, -0.067775, 0.035624, 0.140355, 0.588046, 0.033370, -0.118640, 0.033417, };

std::vector<std::vector<float>> CNN_LW = {
{-0.677200, -0.469054, 0.273436, -0.335824, 0.203998, 0.261129, 0.378393, -0.154833, -0.193414, 0.321739, -0.269394, 0.350923, },
{0.557153, -0.084147, 0.357335, -0.443001, -0.225724, -0.178563, 0.123692, -0.194859, 0.418857, -0.241863, -0.219860, -0.147203, },
};

std::vector<float> CNN_LB = {
-0.141321, 0.082690, };

std::vector<std::vector<float>> CNN_LIN_W = {
{0.560959, -0.014497, },
{0.303474, -0.002378, },
{0.228458, -0.404870, },
{0.104024, -0.194069, },
{-0.097778, 0.127161, },
{-0.287915, -0.518011, },
{0.225458, 0.089430, },
{0.431279, 0.071918, },
{-0.457925, -0.437659, },
{0.160337, -0.041986, },
{0.297866, -0.523153, },
{-0.475587, -0.456609, },
{0.159715, 0.023118, },
{0.100506, -0.179246, },
{-0.115284, -0.434495, },
{-0.079170, 0.010039, },
};

std::vector<float> CNN_LIN_B = {
-0.018317, -0.009824, -0.026342, -0.009147, -0.165380, -0.062135, -0.026793, -0.022735, -0.120196, -0.007965, -0.035155, -0.144706, -0.016356, -0.009403, -0.060364, -0.074220, };

std::vector<std::vector<float>> CNN_LOUT_W = {
{0.059566, 0.424252, -0.219343, -0.416216, 0.197817, -0.348578, 0.015442, -0.226914, 0.445343, -0.346565, 0.222445, 0.409410, -0.070274, 0.294522, 0.187544, -0.131200, },
{-0.233155, 0.400171, -0.459604, 0.487604, -0.007803, -0.053509, -0.299946, 0.358381, 0.145816, -0.207887, 0.287262, 0.295972, -0.306556, -0.308873, -0.243751, -0.150973, },
};

std::vector<float> CNN_LOUT_B = {
0.028358, -0.021162, };

std::vector<std::vector<float>> WIN_W = {
{-0.034436, -0.030320, -0.251650, -0.265789, 0.075675, -0.004312, 0.011130, 0.548058, -0.325217, 0.079315, 0.051429, -0.414439, 0.136955, 0.088500, 0.410998, 0.200683, 0.435556, 0.077036, -0.102902, },
{0.785998, 0.093600, 1.156052, -0.388292, -0.036649, -0.826199, -0.036458, -0.405804, 1.748853, 0.017893, -0.010221, 0.004180, -0.070488, 0.111199, 0.074870, 0.038940, -0.014549, 0.139788, 0.076130, },
{0.272827, -0.022064, -0.285398, 0.069073, 0.191067, -0.003289, 0.299283, 0.206263, -0.644580, 0.056049, -0.039862, -0.190710, 0.231975, 0.286211, 0.016241, 0.094059, 0.510124, 0.132009, 0.088852, },
{0.347259, -0.003837, 0.217450, 0.100973, 0.258067, 0.506115, 0.214001, 0.693174, -0.233570, 0.127001, -0.365346, -0.116303, -0.254901, 0.015303, -0.073171, -0.132904, 0.226009, -0.262341, 0.035571, },
{-1.270189, 0.444665, -0.213185, 1.109153, -0.634087, -0.388175, -0.955617, 0.067416, 0.143832, 0.026322, -0.035038, -0.044327, -0.184264, -0.288387, 0.002415, -0.004118, -0.236583, -0.167834, 0.243456, },
{0.363442, 0.082288, 1.183297, -0.135868, 0.156588, -1.556810, 0.288109, -0.221928, 1.425584, 0.018242, -0.006748, 0.022052, 0.068603, 0.216429, 0.123175, 0.207289, 0.436579, 0.086786, 0.211137, },
{0.975247, -0.051930, -1.300529, -0.868352, -0.328528, 0.713088, 0.935262, -0.467138, -0.847980, 0.013856, 0.009660, 0.025303, 0.023256, 0.167553, -0.030195, -0.017251, -0.018498, -0.138746, 0.121596, },
{-0.076091, -0.397456, -0.027237, -0.187902, -0.092081, 0.242869, 0.362907, -0.259553, -0.465001, 0.071316, 0.269008, 0.081575, 0.350526, 0.111651, 0.523206, 0.592514, 0.353252, 0.016910, -0.034244, },
{0.087395, -0.454707, 1.417355, -0.107191, 0.316816, -1.820153, -0.283498, -0.163405, 2.210264, 0.015094, -0.010635, 0.023683, 0.343100, -0.154721, 0.148053, 0.082467, 0.170578, 0.143158, -0.063207, },
{0.382443, -0.017591, 1.834312, 0.061750, -0.181411, -2.059182, 0.326647, -0.292304, 1.973664, 0.024875, -0.004558, 0.036056, -0.363719, 0.270402, 0.160477, 0.197610, 0.301409, 0.049743, 0.356582, },
{0.235005, -0.410446, 0.181843, 0.056463, -0.130551, 0.495615, 0.516759, 0.261929, -0.332081, -0.022518, 0.240128, 0.169227, 0.169565, 0.309872, 0.045367, 0.233565, 0.647010, 0.772833, 0.406362, },
{0.162651, -0.138442, -1.827850, -0.614447, -0.241873, 1.805212, 0.301464, -0.373830, -1.095393, 0.000878, 0.010419, -0.002331, 0.118398, 0.207045, -0.128734, -0.139323, -0.168487, -0.052176, -0.159323, },
{0.616570, 0.163256, -1.262034, -0.617493, -0.426788, 1.548077, 0.458055, 0.161033, -1.199941, 0.007314, 0.015625, 0.022732, 0.082339, -0.251499, -0.143998, -0.128707, 0.117290, -0.176876, -0.300273, },
{-0.154407, -0.118543, -0.365887, -0.397458, 0.045281, 0.013997, 0.052670, 0.098484, -0.859671, 0.120237, -0.322337, -0.009413, -0.411524, -0.261668, 0.409242, 0.120936, 0.503905, 0.431610, 0.308741, },
{-0.078907, -0.415796, -0.257266, -0.218795, 0.216017, -0.057597, 0.334635, 0.395727, -0.441014, -0.147573, 0.061571, 0.192517, -0.053741, 0.126085, 0.468281, 0.568611, 0.523883, 0.256839, -0.147094, },
{0.503540, 0.458884, -0.078833, -1.233366, -0.371230, 0.550495, 0.814944, -0.126615, 0.091595, 0.028473, -0.070902, 0.026403, 0.322925, 0.321994, 0.029803, -0.056815, -0.542078, -0.152678, 0.234917, },
};

std::vector<float> WIN_B = {
0.285724, 0.086438, 0.286029, 0.131355, 0.100743, -0.009988, -0.113603, 0.264072, 0.105637, 0.036589, 0.329884, -0.099218, -0.054913, 0.238592, 0.278001, -0.048550, };

std::vector<std::vector<float>> WOUT_W = {
{0.609901, -1.210414, 0.645873, 0.053730, -1.183200, -1.224021, -0.233134, 0.411618, -1.083629, -1.230797, 0.302194, -0.715731, -0.800099, 0.687920, 0.724216, -0.218458, },
{-0.091100, 0.466639, -0.442819, -0.445374, 0.795810, 0.872366, 1.334452, -0.018425, 1.109444, 1.205138, -0.371592, 1.304915, 1.418046, -0.143652, -0.572071, 1.053306, },
};

std::vector<float> WOUT_B = {
0.210833, -0.210833, };

std::vector<std::vector<std::vector<float>>> LIN_W = {
{
{-0.287780, -0.547124, -0.422991, 0.343980, 0.820584, 0.292024, -0.999950, -2.337895, -0.083945, -0.024156, 0.008874, 0.029317, 0.030272, -0.040679, },
{-0.010296, 0.995284, -0.332455, 0.094117, -0.891521, 0.157872, -0.318276, 2.553469, -0.546147, 0.016474, -0.003780, 0.064807, 0.037160, 0.014325, },
{-0.788321, -0.613881, -0.082103, 0.610785, 0.669913, 0.095922, -2.098419, -2.141670, 0.033721, 0.015268, -0.022117, 0.006639, -0.073957, -0.046699, },
{-0.066186, 1.149301, 0.080699, 0.027477, -0.937940, -0.161702, -0.428898, 2.879846, -0.056682, -0.007036, 0.008617, 0.002138, 0.075413, 0.046125, },
{0.142845, -0.058150, -1.038149, -0.021088, 0.076444, 0.809386, -0.755110, 0.408734, -1.318601, 0.025664, -0.000452, 0.095967, -0.086141, 0.064462, },
{-0.832607, -0.789742, -0.475657, 0.793648, 0.744612, 0.258008, -2.566058, -2.687906, -0.165594, 0.018893, -0.025998, 0.018385, 0.010971, -0.118449, },
{1.007865, 0.652535, 0.371956, -1.129235, -0.655178, -0.176830, 3.007400, 2.768347, 0.276598, 0.010575, 0.026192, 0.003986, 0.057524, 0.043792, },
{-0.153644, -1.286036, 0.318522, 0.320649, 1.295300, -0.400381, -0.267522, -2.878307, 0.455721, 0.026684, -0.011473, -0.039903, 0.037922, 0.034815, },
{1.159554, -0.423417, -0.161849, -1.339435, 0.638573, -0.052345, 3.317433, -0.991474, 0.211982, 0.025084, 0.005402, 0.013386, 0.033757, -0.139746, },
{0.326423, -1.198756, -0.059393, -0.207258, 1.281509, 0.180121, 1.240722, -3.023412, 0.045656, -0.011043, 0.001854, 0.014693, 0.069843, 0.101548, },
{-0.868356, 0.970236, 0.206920, 0.866691, -0.825406, -0.021771, -2.427076, 2.663612, -0.288799, 0.000538, -0.011826, -0.004429, 0.142777, 0.012146, },
{1.226312, -0.207463, 0.151292, -1.268850, 0.088166, -0.127401, 3.774184, 0.564944, 0.397330, 0.023659, 0.020173, 0.005224, -0.002492, -0.117369, },
{-0.202815, -0.709824, 0.570466, 0.058453, 0.791631, -0.520714, 0.295875, 0.469494, 0.628703, -0.004281, -0.003068, -0.030840, 0.053307, 0.045625, },
{-0.475066, 0.375222, 0.523366, 0.454894, -0.272540, -0.584033, 0.114423, -0.565356, 1.074473, -0.004228, -0.000245, -0.050677, -0.072498, -0.035033, },
{-1.093550, 0.849026, 0.155440, 1.034796, -0.687140, -0.116880, -2.855805, 1.879323, -0.352648, -0.005469, -0.009785, -0.014194, 0.029625, 0.159468, },
{-0.917108, 1.291619, 0.192545, 0.868646, -1.297130, -0.068371, -2.676129, 3.242692, -0.361655, -0.002940, 0.006714, 0.003607, 0.007227, 0.057292, },
},
{
{-0.014382, -0.105815, -0.065757, 0.056717, 0.057942, 0.198157, -0.175070, -1.399380, -0.343724, 0.059809, -0.024286, 0.026723, 0.008761, -0.062856, },
{0.118471, 0.148269, -0.198581, 0.099690, -0.170727, 0.161880, -0.163535, 1.511225, -0.469034, -0.000173, 0.003668, 0.010242, -0.085934, -0.017548, },
{-0.250734, 0.019107, 0.060300, 0.236873, 0.238555, -0.057721, -0.942747, -1.326810, -0.056264, -0.014221, -0.019358, 0.027146, -0.113103, 0.007205, },
{-0.243003, 0.253730, 0.029962, 0.305350, -0.353900, -0.190565, -0.487589, 2.103027, 0.170950, -0.012863, -0.045203, 0.039278, -0.141120, 0.073138, },
{-0.019708, 0.455904, -0.472836, -0.030921, -0.312677, 0.384891, -0.571298, 0.588906, -0.464560, -0.043182, 0.008174, -0.043810, 0.061008, -0.057322, },
{-0.379395, -0.035713, -0.030882, 0.356391, 0.180998, 0.152848, -1.253705, -1.574322, 0.009712, -0.025323, -0.011412, 0.023902, 0.141079, 0.038792, },
{0.535778, 0.098091, 0.058925, -0.492931, -0.333094, -0.221549, 1.912067, 1.931319, 0.069043, 0.023942, 0.032835, -0.030169, 0.137466, 0.067976, },
{0.077200, -0.130062, 0.047353, -0.013195, 0.115667, -0.050700, 0.234979, -1.793042, 0.341162, 0.024006, -0.027388, 0.014478, -0.000669, 0.113605, },
{0.414394, 0.142042, 0.069133, -0.408599, -0.106784, 0.123761, 2.157638, -0.714515, -0.292838, 0.012652, 0.006102, -0.018598, -0.003920, -0.042366, },
{0.105528, -0.041147, -0.027391, 0.000380, 0.086812, 0.079453, 0.535245, -2.369559, -0.046308, -0.023459, 0.013257, -0.009126, 0.013273, 0.048148, },
{-0.275974, -0.219004, 0.391175, 0.211112, 0.055089, -0.120492, -1.264357, 1.653145, 0.319813, -0.011895, 0.114263, 0.024092, -0.105520, -0.075054, },
{0.609460, 0.026265, 0.080462, -0.716985, -0.033686, 0.045597, 2.708308, 0.237492, -0.148620, -0.025688, -0.004377, -0.030715, 0.143148, 0.008465, },
{0.569843, -0.394359, 0.313755, -0.380356, 0.364936, -0.310895, 0.937326, 1.041674, 0.398123, -0.030219, -0.015240, -0.023379, -0.059374, -0.133952, },
{-0.145088, 0.136729, 0.337146, 0.056175, -0.062353, -0.189021, 0.836137, -0.922365, 0.714007, 0.023096, 0.012399, 0.002949, 0.035139, 0.165494, },
{-0.376104, -0.004819, 0.317281, 0.242608, 0.155343, -0.190394, -1.890700, 1.151724, 0.139665, -0.021407, -0.045641, 0.018473, -0.147074, -0.104365, },
{-0.487702, 0.298296, 0.218430, 0.395841, -0.024789, -0.147781, -2.080910, 1.970250, 0.288586, 0.026211, 0.008356, 0.016211, 0.058296, -0.046278, },
},
};

std::vector<std::vector<float>> LIN_B = {
{0.245669, 0.167926, 0.302820, 0.155538, 0.186355, 0.251322, 0.220648, 0.198160, 0.217074, 0.228573, 0.199902, 0.218238, 0.004573, 0.053010, 0.173672, 0.242929, },
{0.195879, 0.084638, 0.219043, 0.084354, -0.055464, 0.215009, 0.035025, 0.306903, 0.049230, 0.169603, 0.200247, 0.098274, 0.188642, 0.273510, 0.187231, 0.153885, },
};

std::vector<std::vector<std::vector<float>>> MEAN_W = {
{
{0.133447, 0.043469, 0.309830, 0.056153, 0.108761, 0.423614, -0.715695, 0.050577, -0.910840, -0.163566, 0.365826, -1.161774, -0.337927, -0.482892, 0.623440, 0.620149, },
{0.386072, -0.431864, 0.371726, -0.653974, -0.498973, 0.490345, -0.607151, 0.460321, 0.210175, 0.755852, -0.330774, -0.102661, -0.889170, 0.748343, -0.299329, -0.742047, },
{0.164546, 0.241797, 0.089279, 0.094942, 0.971404, 0.174344, 0.044805, -0.082718, 0.142647, 0.092800, 0.061341, 0.046388, -0.656419, -0.761527, 0.038380, 0.076118, },
},
{
{0.043435, 0.049107, 0.173374, 0.178616, -0.007404, 0.228550, -0.288469, 0.010820, -0.356625, -0.056041, 0.184283, -0.671952, -0.316735, -0.122210, 0.275602, 0.425632, },
{0.188044, -0.152053, 0.195225, -0.333959, -0.118925, 0.240519, -0.311612, 0.142577, 0.081697, 0.381072, -0.119689, -0.090239, -0.538287, 0.312558, -0.113944, -0.370314, },
{-0.142271, 0.009144, -0.238420, -0.156662, 0.715606, -0.148551, -0.218768, -0.341732, -0.185525, -0.245731, -0.247451, -0.233770, -0.293852, -0.506384, -0.265003, -0.212596, },
},
};

}
#endif
