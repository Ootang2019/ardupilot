
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
static constexpr int N_TCN_HIDDEN = 32;
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
{-0.135238, -0.166810, },
{0.093254, 0.013441, },
{-0.030841, -0.106496, },
{0.020983, 0.167137, },
{0.092475, -0.064100, },
{0.054629, -0.107674, },
{-0.025224, 0.230879, },
{-0.068415, 0.016435, },
{-0.150337, -0.066256, },
{-0.004280, 0.043749, },
{0.100925, -0.057034, },
{0.108744, 0.066220, },
},
{
{-0.044223, 0.009844, },
{-0.116483, -0.266612, },
{0.347507, 0.338655, },
{0.051358, -0.052427, },
{0.117766, 0.253145, },
{-0.326552, -0.233059, },
{-0.133475, -0.125339, },
{0.128696, 0.054457, },
{-0.096546, -0.138563, },
{-0.064598, -0.116567, },
{0.118812, -0.010568, },
{-0.029803, -0.278157, },
},
{
{0.022104, -0.223313, },
{0.052849, 0.017507, },
{0.078358, -0.094518, },
{0.167401, 0.083237, },
{-0.161223, -0.112785, },
{0.011261, 0.227185, },
{0.054288, 0.163980, },
{-0.031258, -0.057233, },
{-0.003290, -0.054785, },
{0.001318, 0.091815, },
{0.207945, 0.088585, },
{0.216776, 0.332628, },
},
{
{-0.004391, -0.055543, },
{0.052849, 0.056636, },
{0.005776, -0.024959, },
{0.067474, -0.177611, },
{-0.026965, 0.030209, },
{0.012066, 0.033550, },
{0.113334, 0.003359, },
{0.079570, -0.139827, },
{-0.165555, 0.127649, },
{-0.080984, -0.131173, },
{0.001253, 0.038088, },
{0.032230, 0.012928, },
},
{
{-0.101456, -0.027739, },
{-0.020555, -0.020706, },
{0.024701, -0.030399, },
{-0.021956, 0.067711, },
{-0.096272, -0.108672, },
{0.044507, 0.096232, },
{-0.076403, -0.121424, },
{0.073709, -0.094553, },
{-0.063827, 0.033891, },
{-0.103147, -0.007105, },
{0.084443, 0.220878, },
{-0.080182, -0.024837, },
},
{
{-0.028071, -0.038658, },
{0.016233, -0.003701, },
{0.016673, 0.001172, },
{-0.004388, 0.017557, },
{-0.015157, -0.017644, },
{-0.007998, -0.013137, },
{0.061444, 0.045518, },
{-0.060345, -0.059765, },
{-0.095944, -0.050857, },
{-0.004582, -0.014355, },
{-0.044246, -0.028450, },
{0.020799, 0.015923, },
},
{
{0.075046, 0.103260, },
{0.193429, 0.168773, },
{0.056356, -0.029064, },
{-0.059304, -0.320589, },
{-0.196679, -0.116934, },
{-0.076523, -0.004114, },
{-0.260857, -0.284944, },
{0.086603, 0.054276, },
{0.099456, -0.042933, },
{0.021965, -0.147878, },
{-0.062622, 0.190799, },
{-0.154907, -0.075179, },
},
{
{0.110348, -0.122976, },
{-0.090932, 0.200435, },
{0.078563, -0.006054, },
{0.118465, -0.061004, },
{0.167188, 0.120728, },
{0.055465, -0.006838, },
{-0.002977, 0.094159, },
{-0.002639, 0.138008, },
{-0.057319, -0.111633, },
{-0.025651, -0.017930, },
{0.046319, 0.021618, },
{-0.020725, -0.002472, },
},
{
{-0.020456, 0.040039, },
{-0.008809, -0.036014, },
{-0.024856, 0.020270, },
{-0.005659, 0.006603, },
{0.023438, -0.006888, },
{-0.059295, -0.032886, },
{-0.075444, 0.074502, },
{0.020222, -0.058162, },
{0.041916, 0.068043, },
{0.017732, 0.028170, },
{-0.026826, -0.050532, },
{-0.003819, -0.004800, },
},
{
{0.109209, -0.118779, },
{0.098081, 0.204649, },
{0.033820, -0.133194, },
{0.039139, 0.190013, },
{0.127219, 0.051152, },
{0.172958, -0.051376, },
{-0.119629, -0.191730, },
{-0.174287, -0.184616, },
{-0.171402, -0.166944, },
{0.057019, -0.048196, },
{0.087201, -0.050141, },
{-0.078304, 0.015138, },
},
{
{0.034155, -0.062771, },
{0.026023, -0.076644, },
{0.045154, -0.056528, },
{0.111936, -0.017264, },
{-0.034109, -0.002056, },
{-0.020087, 0.021823, },
{0.087432, -0.087877, },
{0.024907, -0.098997, },
{0.107367, 0.096224, },
{-0.099960, 0.061207, },
{-0.050077, -0.105374, },
{0.003442, 0.043641, },
},
{
{0.087736, 0.057644, },
{-0.000173, -0.037152, },
{0.035405, 0.020925, },
{-0.052708, 0.007568, },
{-0.013052, -0.084656, },
{0.049268, -0.027560, },
{-0.035614, -0.122131, },
{-0.178806, -0.077333, },
{0.109046, 0.141347, },
{0.008354, -0.094407, },
{0.023149, -0.050126, },
{-0.003514, -0.060990, },
},
};

std::vector<float> CNN_B1 = {
-0.012099, -0.178561, 0.052364, -0.314710, 0.024608, -0.147429, 0.116171, -0.239548, -0.122295, -0.000512, -0.078816, -0.374110, };

std::vector<std::vector<float>> CNN_LW = {
{0.114098, 0.119233, -0.216487, 0.054038, -0.430242, -0.050830, -0.669194, 0.326705, 0.089928, -0.176937, 0.337008, 0.050809, },
{0.026871, -0.224274, 0.243683, -0.149030, -0.359617, 0.124791, -0.447700, 0.254238, -0.157511, -0.115975, 0.380532, -0.274168, },
};

std::vector<float> CNN_LB = {
-0.005664, -0.048865, };

std::vector<std::vector<float>> CNN_LIN_W = {
{0.106011, 0.142235, },
{0.320523, 0.286240, },
{0.196289, -0.244722, },
{0.211824, 0.067603, },
{0.233166, -0.009603, },
{0.152412, -0.099414, },
{0.060301, 0.273664, },
{0.043782, 0.077879, },
{0.210872, -0.105514, },
{-0.162905, 0.064952, },
{0.140352, -0.045984, },
{0.153316, -0.158024, },
{0.079708, 0.048884, },
{0.082994, 0.163164, },
{0.003786, -0.014261, },
{0.103752, -0.079020, },
{0.024452, -0.158065, },
{0.378125, -0.209116, },
{-0.122856, 0.068437, },
{-0.134708, 0.153841, },
{0.024969, 0.286025, },
{0.227091, 0.116194, },
{-0.153548, 0.328479, },
{0.035338, 0.252280, },
{-0.060391, 0.260347, },
{0.045248, -0.053912, },
{-0.065047, 0.162439, },
{0.008228, -0.140048, },
{-0.037293, 0.224999, },
{0.197645, 0.235346, },
{0.250163, -0.143831, },
{0.281781, -0.162685, },
};

std::vector<float> CNN_LIN_B = {
-0.050976, -0.098557, -0.133690, -0.052725, -0.064405, -0.082925, -0.104057, -0.052049, -0.073351, -0.136066, -0.079153, -0.084266, -0.070128, -0.079876, -0.073919, -0.054120, -0.094273, -0.130948, -0.083739, -0.079178, -0.084072, -0.067762, -0.092990, -0.052213, -0.067834, -0.060055, -0.047401, -0.135383, -0.096873, -0.094182, -0.077349, -0.078927, };

std::vector<std::vector<float>> CNN_LOUT_W = {
{0.067606, 0.127593, 0.059856, 0.239474, -0.099762, -0.155930, -0.055321, 0.194484, -0.192583, -0.104013, 0.188362, -0.309555, 0.325853, 0.187221, -0.187755, -0.198671, -0.030731, 0.284823, 0.178807, -0.025798, -0.177138, -0.170766, -0.011407, 0.041250, 0.055568, 0.235961, -0.031060, -0.099278, 0.128552, -0.318038, -0.304157, 0.255125, },
{-0.018136, 0.123014, 0.177312, -0.333314, 0.332639, 0.041549, -0.044725, -0.222037, 0.332859, 0.027544, -0.312714, -0.051925, 0.346537, 0.316436, -0.234313, 0.375151, 0.015052, -0.300102, -0.002231, 0.038058, 0.070284, 0.214553, -0.018428, -0.170318, -0.039449, 0.225495, 0.019019, 0.051738, 0.096952, -0.335778, -0.284466, 0.112656, },
};

std::vector<float> CNN_LOUT_B = {
0.017313, 0.009407, };

std::vector<std::vector<float>> WIN_W = {
{-0.055333, -0.069494, -0.107202, -0.243649, 0.114850, -0.146857, 0.826375, 0.249914, -1.169552, 0.024840, 0.017564, -0.196416, 0.063165, 0.067333, 0.403516, 0.232309, 0.495115, },
{0.121966, 0.552577, 0.931244, 0.275070, -0.495626, -0.599848, -1.748392, -0.129278, 2.565317, -0.042655, 0.018610, -0.169358, -0.067903, 0.307100, 0.147466, 0.028855, -0.194444, },
{0.314139, -0.145345, -0.104157, 0.027002, 0.314352, -0.182853, 1.184345, -0.179911, -1.420482, 0.014899, 0.008337, 0.047727, 0.154467, 0.279185, 0.018464, 0.222982, 0.465798, },
{0.307843, -0.107849, 0.455756, 0.139136, 0.362083, 0.265847, 1.382116, -0.128236, -1.333844, 0.551486, -0.434217, -0.070967, -0.311969, -0.145498, -0.056733, -0.096456, 0.331542, },
{-1.725092, 0.207746, -0.028220, 1.561719, -0.397166, -0.594864, 0.403277, 0.074055, 2.676073, -0.191660, 0.003558, 0.008358, 0.040990, -0.005868, -0.015290, -0.036774, 0.086998, },
{0.449063, 0.447677, 0.302905, -0.219812, -0.208799, -0.679498, -1.041573, 0.249236, 1.415087, -0.058131, 0.027243, -0.127434, 0.064694, 0.357360, 0.020004, 0.285820, 0.487266, },
{0.138931, -0.377629, -0.343884, -0.031528, -0.002827, -0.240911, 1.158894, -0.504871, -1.588007, -0.074616, -0.208126, -0.187235, -0.167210, 0.049935, 0.376317, 0.354131, 0.051457, },
{-0.079115, -0.356991, 0.057451, -0.184616, -0.132542, 0.158591, 1.124489, -0.638809, -1.316118, 0.051124, 0.266277, 0.196377, 0.247575, 0.057186, 0.492983, 0.618000, 0.336533, },
{0.134098, -0.269392, 0.300503, -0.153119, 0.131506, -0.706315, -1.055128, 1.052151, 0.930191, 0.104463, -0.036714, -0.023899, 0.326636, 0.009101, -0.281226, -0.369615, 0.162959, },
{0.316554, 0.147958, 0.667909, 0.129386, -0.346959, -0.896067, -1.035076, 0.100284, 2.066129, -0.049356, 0.016254, -0.144668, -0.391496, 0.429184, 0.168430, 0.293891, 0.135539, },
{0.221906, -0.401246, 0.283084, 0.068907, -0.139750, 0.392868, 1.417488, 0.016717, -1.185545, -0.006187, 0.324675, 0.306642, 0.055202, 0.321542, -0.091643, 0.266630, 0.628546, },
{0.423535, -0.470138, -0.767445, -0.875320, 0.089821, 0.742274, 0.109885, -0.501505, -0.700342, -0.013199, -0.171477, 0.117007, 0.211593, 0.380054, -0.412966, -0.416486, 0.089440, },
{0.684192, -0.309907, -0.585342, -0.687400, 0.046373, 0.872478, -0.082721, 0.033303, -1.094941, 0.010661, -0.148134, 0.110709, 0.254083, -0.069456, -0.436597, -0.405027, 0.276144, },
{-0.160618, -0.229230, -0.223236, -0.390330, 0.155967, -0.126978, 0.856433, -0.190750, -1.580679, 0.168718, -0.246475, 0.104683, -0.459360, -0.394121, 0.462866, 0.260118, 0.349506, },
{-0.040295, -0.371757, -0.152590, -0.257723, 0.171978, -0.160399, 1.121059, 0.222835, -1.246083, -0.144990, 0.102496, 0.336427, -0.138339, 0.099672, 0.502061, 0.644486, 0.451870, },
{-0.290794, 0.065073, 0.230169, -0.437889, 0.022590, 0.238898, 0.944321, -0.651564, -1.364225, -0.054345, 0.107149, 0.124594, 0.258843, 0.279569, 0.559283, -0.006650, -0.211334, },
};

std::vector<float> WIN_B = {
0.314207, 0.093818, };

std::vector<std::vector<float>> WOUT_W = {
{0.638175, -1.216746, 0.704702, 0.177520, -1.789497, -0.885062, 0.774089, 0.440872, -0.347187, -0.719234, 0.322938, -0.146584, -0.405828, 0.684965, 0.759570, 0.583179, },
{-0.119376, 0.472968, -0.501647, -0.569167, 1.402101, 0.533404, 0.327227, -0.047680, 0.373004, 0.693576, -0.392336, 0.735757, 1.023770, -0.140698, -0.607426, 0.251668, },
};

std::vector<float> WOUT_B = {
0.299078, -0.299078, };

std::vector<std::vector<std::vector<float>>> LIN_W = {
{
{-0.145547, 0.062767, -0.927598, 0.204233, 0.210700, 0.794876, 0.293015, -2.291297, -4.444376, 0.038090, -0.040934, 0.256680, 0.099607, -0.050891, },
{0.700244, 0.502203, 0.315749, -0.619594, -0.398448, -0.487393, 1.451498, 2.599908, 0.249512, 0.035803, 0.066586, -0.061081, -0.187275, -0.033054, },
{-1.018270, 0.638130, 0.234390, 0.842824, -0.582098, -0.229702, -2.904757, 2.003697, 4.088828, -0.171750, 0.117196, -0.192734, -0.128718, 0.078160, },
{0.593459, 0.525102, -0.434101, -0.631687, -0.313752, 0.354247, 1.399001, 0.494960, -4.558537, 0.042511, -0.054525, 0.089721, 0.002440, -0.097481, },
{-0.045541, -0.023425, -0.496956, 0.164710, 0.041719, 0.269111, 0.490790, -0.008804, -4.923505, -0.009359, -0.034901, -0.132432, -0.141776, 0.004653, },
{-1.641948, -0.051022, -0.135344, 1.604746, 0.005896, -0.085445, -3.410335, -0.126427, 3.621383, -0.196098, 0.054280, -0.265343, 0.080039, 0.069551, },
{-1.476846, -0.001256, -0.816019, 1.360678, -0.001392, 1.006642, -2.689644, -1.530725, -3.903639, 0.100721, 0.028644, 0.350193, 0.205534, 0.176890, },
{0.283732, -0.627932, 0.220326, -0.116162, 0.637200, -0.299051, 1.282673, -2.610527, 2.174703, 0.060394, -0.078026, -0.043015, 0.006715, 0.062403, },
{0.812089, 0.438879, 0.427247, -1.000248, -0.223733, -0.635450, 2.395920, 2.710851, 0.496826, 0.077507, 0.036554, -0.061536, -0.118620, -0.056010, },
{0.303053, -0.281678, 0.415471, -0.185290, 0.364430, -0.295428, 1.066775, -1.520726, 3.254370, 0.013187, -0.051958, -0.129698, -0.024234, 0.082667, },
{0.115889, 0.787680, 0.181755, -0.118487, -0.642853, 0.000135, -0.295450, 3.559366, -0.918216, -0.004823, 0.012653, -0.030105, 0.053124, 0.081973, },
{0.961913, -0.330061, 0.432614, -1.004217, 0.210762, -0.397966, 3.949260, -2.012924, 0.596724, 0.023952, -0.028445, -0.049317, -0.083666, 0.039250, },
{0.676424, 0.687950, 0.684501, -0.821687, -0.606149, -0.632404, 1.151691, 2.353268, 2.615260, 0.048208, 0.082353, 0.025113, -0.126647, 0.154156, },
{-0.166108, -0.443401, 0.032263, 0.146304, 0.546087, -0.089145, 0.306391, -2.918671, 2.073400, 0.045847, -0.079329, 0.005507, -0.000776, -0.009950, },
{-0.770789, -0.351135, -0.224682, 0.713977, 0.513028, 0.263125, -1.312878, -2.531433, 1.639485, 0.026519, -0.000503, 0.083654, 0.164927, 0.180043, },
{-1.354377, 0.414627, -0.465586, 1.306863, -0.420137, 0.579851, -4.282884, 1.225868, 0.144781, 0.023629, 0.008759, 0.053874, 0.073187, 0.293838, },
},
{
{0.606898, -1.283936, 0.247883, -0.562247, 1.236071, -0.118788, 0.221593, -3.250859, -1.797083, 0.197138, -0.064506, 0.028790, 0.054775, -0.144949, },
{0.240061, 0.401194, 0.139520, -0.025668, -0.423651, -0.172458, 0.915667, 3.374994, 0.192979, 0.060701, 0.092289, 0.154952, -0.246727, 0.023336, },
{0.110989, 1.102753, 0.046067, -0.125691, -0.845087, -0.044810, -2.140913, 1.614839, 1.684462, -0.126214, 0.185843, -0.273949, -0.136563, 0.136701, },
{-0.226120, -0.700270, -0.672199, 0.288998, 0.600101, 0.511006, 0.151128, 0.002520, -3.458792, 0.103166, -0.045991, -0.010214, -0.187149, -0.053949, },
{-0.364143, 0.000892, -0.261105, 0.313999, 0.142337, 0.171813, 2.230168, 0.963282, -2.984729, 0.071049, -0.009327, 0.042947, -0.024679, -0.148483, },
{-0.132731, 0.639425, -0.032619, 0.111390, -0.494136, 0.153716, -2.887147, -0.684781, 1.953494, -0.164755, 0.150079, -0.253842, 0.219950, 0.201024, },
{0.709553, -0.246110, -0.269824, -0.661795, 0.011104, 0.104683, -2.529476, -1.263584, -2.495032, -0.072099, 0.038387, -0.123868, 0.295300, 0.106228, },
{0.566356, 0.165875, -0.879012, -0.501427, -0.180269, 0.876550, 0.917198, -2.924971, 1.443507, -0.064074, -0.274444, 0.280811, -0.068145, 0.076850, },
{0.118629, 0.180549, 0.224663, -0.116545, -0.145293, -0.028084, 2.032181, 3.235255, 0.175337, 0.066800, 0.033550, 0.201379, -0.107663, -0.033476, },
{-0.632143, 0.243061, 0.005140, 0.737383, -0.197395, 0.042202, 0.539201, -2.605116, 2.023671, -0.139675, -0.068643, -0.040971, -0.098318, -0.022489, },
{0.035524, 0.399391, 0.234336, -0.101573, -0.563304, 0.037055, -0.505526, 4.047593, -0.294239, 0.021660, 0.122168, 0.017333, -0.119660, 0.064098, },
{-0.172313, -0.273300, -0.082502, 0.066375, 0.265878, 0.213753, 4.147152, -2.174393, 0.241705, 0.097435, -0.141297, 0.217129, 0.033372, 0.022896, },
{0.438576, 0.370413, 0.247232, -0.250090, -0.399839, -0.241464, 0.407139, 2.894065, 0.965523, 0.031857, 0.079340, -0.017734, -0.137943, 0.048633, },
{-0.131179, 0.092174, -0.245694, 0.044464, -0.017801, 0.394954, 0.297310, -3.397445, 1.408169, 0.045514, -0.154620, 0.025068, 0.043327, 0.177329, },
{-0.351564, 0.301539, 0.109880, 0.220746, -0.151013, 0.015913, -1.136992, -3.408959, 1.035261, -0.190590, -0.004654, -0.115656, -0.012297, -0.035039, },
{-0.059454, 0.364380, 0.111655, -0.033010, -0.090873, -0.044931, -4.020681, 1.288975, -0.048177, -0.028411, 0.155018, -0.313216, 0.159751, 0.225069, },
},
};

std::vector<std::vector<float>> LIN_B = {
{0.383373, 0.153289, 0.264598, 0.397477, 0.207010, 0.230623, 0.300895, 0.217509, 0.167265, 0.247572, 0.195504, 0.229769, 0.135176, 0.185227, 0.167606, 0.208995, },
{0.252486, 0.211191, 0.499702, -0.232194, 0.015490, 0.482869, 0.052863, -0.001518, 0.161869, 0.144717, 0.227220, 0.043197, 0.221099, 0.091927, 0.146663, 0.358994, },
};

std::vector<std::vector<std::vector<float>>> MEAN_W = {
{
{0.053329, -0.256800, 0.413904, -0.148487, -0.327693, 0.573822, 0.500763, -0.259197, -0.609646, -0.193007, 0.022477, -1.124555, -0.366160, -0.032820, 0.393954, 1.064924, },
{0.219150, -0.385368, -0.201224, -0.105505, -0.653464, 0.135664, 0.131800, 0.629575, -0.599034, 0.345133, -0.592641, 0.528689, -0.517549, 0.981437, 0.750910, -0.265919, },
{0.931039, -0.132180, -0.885696, 0.823628, 1.936226, -0.716584, 0.924874, -0.465717, -0.273637, -0.682138, 0.111529, -0.171652, -0.852029, -0.560469, -0.305920, 0.057290, },
},
{
{0.120818, -0.194567, 0.260469, 0.112443, -0.351563, 0.372293, 0.403076, -0.045550, -0.481348, 0.052389, 0.078029, -0.857941, -0.113883, 0.028405, 0.319556, 0.957974, },
{0.348139, -0.394928, -0.373812, -0.036644, -0.052403, 0.015845, -0.093136, 0.526210, -0.575727, 0.419109, -0.596899, 0.527686, -0.646893, 0.635900, 0.478013, -0.305865, },
{0.806142, -0.329647, -0.996509, 0.446127, 1.086691, -0.857799, 0.532213, -0.081940, -0.257872, -0.520660, -0.263377, -0.057241, -0.347532, -0.797351, -0.430047, -0.291359, },
},
};

}
#endif
