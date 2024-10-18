
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
{-0.067977, -0.075081, },
{-0.037318, -0.163688, },
{-0.010240, -0.029271, },
{-0.118277, 0.132326, },
{0.277960, 0.040849, },
{0.045923, -0.213013, },
{-0.100583, 0.065421, },
{-0.031550, 0.090020, },
{-0.221209, 0.005216, },
{0.005413, 0.066979, },
{0.018943, -0.188704, },
{0.123688, -0.023453, },
},
{
{0.131189, 0.022524, },
{-0.303257, -0.311579, },
{0.216173, 0.614263, },
{-0.119157, -0.033104, },
{0.305163, 0.311145, },
{-0.214742, -0.451435, },
{-0.078210, -0.083549, },
{0.009901, 0.009922, },
{0.008855, 0.054175, },
{0.064016, 0.152044, },
{0.057396, -0.126668, },
{-0.162062, -0.456065, },
},
{
{0.063308, -0.027360, },
{-0.009481, 0.001421, },
{0.016977, -0.229747, },
{-0.068390, -0.058179, },
{-0.081623, -0.049342, },
{0.101609, 0.353586, },
{0.017650, 0.042142, },
{0.097641, -0.038847, },
{0.057117, -0.042097, },
{0.029740, -0.075727, },
{-0.106491, 0.186400, },
{0.218224, 0.369410, },
},
{
{-0.022123, -0.097482, },
{0.070199, 0.077778, },
{0.025509, -0.008086, },
{0.106736, -0.202178, },
{0.022450, 0.093191, },
{-0.006541, 0.017155, },
{0.138231, -0.033634, },
{0.134729, -0.149586, },
{-0.140178, 0.175504, },
{-0.065201, -0.100583, },
{0.002733, 0.031022, },
{-0.042310, -0.060211, },
},
{
{0.006877, 0.030565, },
{-0.085398, -0.123364, },
{0.071348, -0.061632, },
{-0.171852, 0.031733, },
{-0.114025, 0.012780, },
{0.051768, 0.190830, },
{-0.011558, -0.122124, },
{0.189715, 0.084493, },
{-0.155488, 0.043152, },
{-0.147317, 0.041137, },
{0.116980, 0.061059, },
{-0.050136, 0.058899, },
},
{
{-0.078354, -0.133744, },
{0.134476, 0.141673, },
{-0.046507, -0.081415, },
{-0.024237, 0.061408, },
{-0.120648, -0.124989, },
{0.086893, 0.055311, },
{0.056302, 0.155280, },
{-0.121710, -0.133758, },
{-0.207104, 0.008010, },
{0.086756, -0.046260, },
{-0.104571, -0.129372, },
{0.108062, 0.048634, },
},
{
{-0.015133, -0.022851, },
{0.126754, 0.111396, },
{-0.071744, -0.064096, },
{0.036265, -0.237335, },
{-0.130141, -0.056523, },
{0.054865, 0.112315, },
{-0.266268, -0.271303, },
{0.155271, 0.146221, },
{0.074870, -0.068860, },
{0.108322, -0.079225, },
{0.028485, 0.247099, },
{-0.006400, -0.034560, },
},
{
{0.074045, -0.130653, },
{-0.089664, 0.169635, },
{0.099908, 0.011965, },
{0.125098, -0.031370, },
{0.167639, 0.121970, },
{0.049787, -0.027330, },
{0.026550, 0.081229, },
{-0.020656, 0.117372, },
{-0.086959, -0.119685, },
{-0.015532, -0.003962, },
{0.053258, 0.017143, },
{0.001841, 0.006800, },
},
{
{-0.089477, 0.030742, },
{-0.168779, -0.182499, },
{-0.032379, 0.074127, },
{0.033053, 0.051338, },
{0.160928, 0.221609, },
{-0.124287, -0.085991, },
{-0.123530, 0.126454, },
{0.099146, 0.064339, },
{0.002284, 0.118507, },
{-0.023170, 0.003635, },
{0.122331, -0.137563, },
{0.004157, 0.069223, },
},
{
{0.078934, -0.002047, },
{0.076813, 0.127792, },
{0.023807, -0.104552, },
{-0.009829, 0.082253, },
{0.091856, 0.020033, },
{0.128221, -0.027881, },
{-0.042267, -0.175088, },
{-0.111974, -0.149902, },
{-0.176074, -0.178833, },
{0.042018, 0.023147, },
{0.117298, -0.039174, },
{-0.083338, 0.020071, },
},
{
{0.079620, -0.056922, },
{0.073949, -0.084336, },
{-0.032475, -0.079462, },
{0.139068, -0.067280, },
{-0.085783, -0.034887, },
{0.048934, 0.073629, },
{0.126948, -0.093640, },
{-0.021273, -0.135224, },
{0.107933, 0.115940, },
{-0.033058, 0.136023, },
{-0.072965, -0.131312, },
{0.003460, -0.010428, },
},
{
{0.049725, -0.086387, },
{-0.048914, -0.121238, },
{0.107803, 0.089486, },
{0.019144, 0.197723, },
{0.025079, -0.113776, },
{0.093885, -0.120186, },
{0.043128, -0.101320, },
{-0.052272, 0.007400, },
{-0.022858, -0.212146, },
{0.271078, -0.115559, },
{0.051986, -0.092575, },
{0.200142, -0.335661, },
},
};

std::vector<float> CNN_B1 = {
-0.051479, 0.032076, -0.007027, -0.305885, -0.055845, -0.045952, 0.017775, -0.251270, -0.053278, -0.076846, -0.056984, -0.163692, };

std::vector<std::vector<float>> CNN_LW = {
{0.124864, 0.179854, -0.309830, -0.048226, -0.501617, 0.011885, -0.626471, 0.354366, 0.198539, -0.057031, 0.365493, 0.214180, },
{0.209847, -0.211048, 0.283809, 0.164929, -0.184913, 0.305788, -0.343646, 0.518412, 0.299692, -0.038204, 0.256336, -0.311105, },
};

std::vector<float> CNN_LB = {
0.040955, -0.057935, };

std::vector<std::vector<float>> CNN_LIN_W = {
{-0.308942, -0.243631, },
{0.337434, -0.073427, },
{-0.038714, -0.192468, },
{-0.070947, 0.322240, },
{-0.012805, 0.390636, },
{-0.436955, 0.398892, },
{-0.231736, -0.169226, },
{-0.166810, 0.357173, },
{-0.396487, 0.412772, },
{-0.169827, -0.009713, },
{0.252536, 0.099582, },
{0.337365, -0.194309, },
{-0.154303, 0.448261, },
{-0.312390, 0.311766, },
{-0.040762, -0.116885, },
{-0.328634, -0.032417, },
};

std::vector<float> CNN_LIN_B = {
-0.033525, -0.101096, -0.088867, -0.010381, -0.008125, -0.000759, -0.024351, -0.026373, 0.002104, -0.047341, -0.053190, -0.120598, -0.005824, 0.001552, -0.055573, -0.003737, };

std::vector<std::vector<float>> CNN_LOUT_W = {
{-0.205697, -0.181114, 0.139864, 0.276408, -0.013399, 0.092932, 0.251238, -0.003933, 0.110506, 0.033123, -0.151580, -0.231220, -0.123375, -0.330264, -0.195739, 0.072754, },
{-0.279593, 0.014317, 0.411573, 0.050559, 0.178578, -0.136998, 0.387167, -0.013019, 0.309359, -0.016748, 0.085522, -0.566051, -0.324913, 0.008066, 0.194479, -0.090880, },
};

std::vector<float> CNN_LOUT_B = {
0.014562, 0.009033, };

std::vector<std::vector<float>> WIN_W = {
{-0.055333, -0.069494, -0.107202, -0.243649, 0.114850, -0.146857, 0.826375, 0.249914, -1.169552, 0.024840, 0.017564, -0.196416, 0.063165, 0.067333, 0.403516, 0.232309, 0.495115, 0.180440, -0.021192, },
{0.121966, 0.552577, 0.931244, 0.275070, -0.495626, -0.599848, -1.748392, -0.129278, 2.565317, -0.042655, 0.018610, -0.169358, -0.067903, 0.307100, 0.147466, 0.028855, -0.194444, 0.239793, 0.193792, },
{0.314139, -0.145345, -0.104157, 0.027002, 0.314352, -0.182853, 1.184345, -0.179911, -1.420482, 0.014899, 0.008337, 0.047727, 0.154467, 0.279185, 0.018464, 0.222982, 0.465798, 0.190474, 0.218957, },
{0.307843, -0.107849, 0.455756, 0.139136, 0.362083, 0.265847, 1.382116, -0.128236, -1.333844, 0.551486, -0.434217, -0.070967, -0.311969, -0.145498, -0.056733, -0.096456, 0.331542, -0.048627, 0.052977, },
{-1.725092, 0.207746, -0.028220, 1.561719, -0.397166, -0.594864, 0.403277, 0.074055, 2.676073, -0.191660, 0.003558, 0.008358, 0.040990, -0.005868, -0.015290, -0.036774, 0.086998, 0.022315, -0.208102, },
{0.449063, 0.447677, 0.302905, -0.219812, -0.208799, -0.679498, -1.041573, 0.249236, 1.415087, -0.058131, 0.027243, -0.127434, 0.064694, 0.357360, 0.020004, 0.285820, 0.487266, -0.051768, -0.066802, },
{0.138931, -0.377629, -0.343884, -0.031528, -0.002827, -0.240911, 1.158894, -0.504871, -1.588007, -0.074616, -0.208126, -0.187235, -0.167210, 0.049935, 0.376317, 0.354131, 0.051457, 0.026844, 0.552769, },
{-0.079115, -0.356991, 0.057451, -0.184616, -0.132542, 0.158591, 1.124489, -0.638809, -1.316118, 0.051124, 0.266277, 0.196377, 0.247575, 0.057186, 0.492983, 0.618000, 0.336533, 0.026969, -0.016132, },
{0.134098, -0.269392, 0.300503, -0.153119, 0.131506, -0.706315, -1.055128, 1.052151, 0.930191, 0.104463, -0.036714, -0.023899, 0.326636, 0.009101, -0.281226, -0.369615, 0.162959, -0.265209, -0.757394, },
{0.316554, 0.147958, 0.667909, 0.129386, -0.346959, -0.896067, -1.035076, 0.100284, 2.066129, -0.049356, 0.016254, -0.144668, -0.391496, 0.429184, 0.168430, 0.293891, 0.135539, -0.044463, 0.498321, },
{0.221906, -0.401246, 0.283084, 0.068907, -0.139750, 0.392868, 1.417488, 0.016717, -1.185545, -0.006187, 0.324675, 0.306642, 0.055202, 0.321542, -0.091643, 0.266630, 0.628546, 0.775851, 0.429284, },
{0.423535, -0.470138, -0.767445, -0.875320, 0.089821, 0.742274, 0.109885, -0.501505, -0.700342, -0.013199, -0.171477, 0.117007, 0.211593, 0.380054, -0.412966, -0.416486, 0.089440, -0.164311, -0.783941, },
{0.684192, -0.309907, -0.585342, -0.687400, 0.046373, 0.872478, -0.082721, 0.033303, -1.094941, 0.010661, -0.148134, 0.110709, 0.254083, -0.069456, -0.436597, -0.405027, 0.276144, -0.360575, -0.874490, },
{-0.160618, -0.229230, -0.223236, -0.390330, 0.155967, -0.126978, 0.856433, -0.190750, -1.580679, 0.168718, -0.246475, 0.104683, -0.459360, -0.394121, 0.462866, 0.260118, 0.349506, 0.484682, 0.518466, },
{-0.040295, -0.371757, -0.152590, -0.257723, 0.171978, -0.160399, 1.121059, 0.222835, -1.246083, -0.144990, 0.102496, 0.336427, -0.138339, 0.099672, 0.502061, 0.644486, 0.451870, 0.256035, -0.009081, },
{-0.290794, 0.065073, 0.230169, -0.437889, 0.022590, 0.238898, 0.944321, -0.651564, -1.364225, -0.054345, 0.107149, 0.124594, 0.258843, 0.279569, 0.559283, -0.006650, -0.211334, 0.393777, 0.336428, },
};

std::vector<float> WIN_B = {
0.314207, 0.093818, 0.325748, 0.180078, 0.074506, 0.040621, 0.241765, 0.256141, -0.293762, 0.031240, 0.300557, -0.272884, -0.309628, 0.261931, 0.297484, 0.257190, };

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
