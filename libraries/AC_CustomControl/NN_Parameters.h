#pragma once
#ifndef __NN_PARAMETERS_DEF_H__
#define __NN_PARAMETERS_DEF_H__

#include <vector>

namespace NN {


static constexpr bool USE_LAYERNORM = 0;
static constexpr int N_ACT = 3;
static constexpr int N_STATE = 6;
static constexpr int N_GOAL = 3;
static constexpr int N_TASK = 7;
static constexpr int N_HIDDEN = 16;

static constexpr float AVEL_LIM = 15.700000;
static constexpr float POS_LIM = 999999.900000;
static constexpr float AUTHORITY = 0.250000;

std::vector<float> OBS = {
0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, };
std::vector<float> TASK = {
0.250000, 0.250000, 0.250000, 0.250000, 0.000000, 0.000000, 0.000000, };
static const std::vector<float> A = {
    1.0f,
    -0.47466427087783813f,
    -0.0008633044199086726f,
    1.0f,
    -0.04767982289195061f,
    -0.07276630401611328f,
    -0.825597882270813f,
    -0.44393429160118103f,
    0.044065922498703f,
    0.4736970067024231f,
    0.4523405432701111f,
    -0.4840397536754608f,
    -0.5179069638252258f,
    0.451914519071579f,
    -0.5377469062805176f,
    0.44498804211616516f,
    0.500432014465332f,
    -0.390064001083374f,
    -0.38206353783607483f,
    -0.49622347950935364f,
    1.0f,
    0.0057171182706952095f,
    -0.017093610018491745f,
    1.0f,
    -0.11342088878154755f,
    -0.41079944372177124f,
    -1.0f,
    0.04241831228137016f,
    0.4503895044326782f,
    0.42827141284942627f,
    -0.5059121251106262f,
    -0.5326877236366272f,
    0.4160313606262207f,
    -0.5695338845252991f,
    0.41909077763557434f,
    -0.3457670509815216f,
    0.5244275331497192f,
    -0.3392198085784912f,
    -0.3675517141819f,
    -0.4366764426231384f,
    0.005651258397847414f,
    -0.030946100130677223f,
    -0.10026733577251434f,
    1.0f,
    -0.3643993139266968f,
    -0.2782893776893616f,
    -1.0f,
    -0.525468647480011f,
    -0.5476929545402527f,
    0.479951411485672f,
    0.4292879104614258f,
    0.421464204788208f,
    -0.5617241263389587f,
    0.4178393483161926f,
    -0.33903083205223083f,
    -0.3505151867866516f,
    0.5779876112937927f,
};

static const std::vector<float> ACT_EMB_W = {
    -0.5172700881958008f, 
    0.3684983253479004f, 
    0.10819420963525772f, 
    -0.2177058309316635f, 
    0.16988444328308105f, 
    0.36162251234054565f, 
    -0.3621881604194641f, 
    -0.5193852782249451f, 
    -0.2171596735715866f, 
    -0.2989281117916107f, 
    -0.04971601441502571f, 
    0.46380478143692017f, 
    0.49087727069854736f, 
    0.3101992607116699f, 
    -0.3688192665576935f, 
    -0.2096870094537735f, 
};

static const std::vector<float> ACT_EMB_B = {
    0.04352409765124321f,
    -0.03066634014248848f,
    -0.07199449092149734f,
    0.054283399134874344f,
    -0.048514362424612045f,
    0.059397097676992416f,
    -0.12232331931591034f,
    -0.0331861637532711f,
    -0.05201239138841629f,
    0.044219907373189926f,
    -0.11051936447620392f,
    0.16143091022968292f,
    -0.082970030605793f,
    -0.0934651717543602f,
    -0.09064105898141861f,
    0.03255099803209305f,
};

static const std::vector<float> ANG_EMB_W = {
    -0.5036250948905945f, 
    0.8591762185096741f, 
    -0.2772735059261322f, 
    1.079877257347107f, 
    -1.1987749338150024f, 
    -0.5615005493164062f, 
    0.19500744342803955f, 
    0.8413981199264526f, 
    -1.3446675539016724f, 
    -0.7943435311317444f, 
    0.7867031097412109f, 
    0.4307127296924591f, 
    -0.09152797609567642f, 
    0.6209562420845032f, 
    1.0359843969345093f, 
    -1.6917846202850342f, 
};

static const std::vector<float> ANG_EMB_B = {
    0.03376384824514389f,
    0.0003001229197252542f,
    -0.10011354833841324f,
    0.07318325340747833f,
    -0.06589189916849136f,
    0.061587557196617126f,
    -0.11041871458292007f,
    -0.02559555135667324f,
    -0.07034046202898026f,
    0.028112739324569702f,
    -0.0965384766459465f,
    0.18544721603393555f,
    -0.10072814673185349f,
    -0.0791095644235611f,
    -0.07314501702785492f,
    0.020137332379817963f,
};

static const std::vector<float> ANGVEL_EMB_W = {
    -2.7438783645629883f, 
    1.4044981002807617f, 
    -1.859175443649292f, 
    2.8548500537872314f, 
    -3.6473026275634766f, 
    -1.3406025171279907f, 
    0.6285648941993713f, 
    3.34641695022583f, 
    -2.839496374130249f, 
    -3.4383533000946045f, 
    2.275716781616211f, 
    0.652683436870575f, 
    -2.6634557247161865f, 
    1.628265142440796f, 
    3.041377305984497f, 
    -4.44993257522583f, 
};

static const std::vector<float> ANGVEL_EMB_B = {
    0.012559259310364723f,
    0.003663107519969344f,
    0.0041672298684716225f,
    0.036582257598638535f,
    -0.06849435716867447f,
    -0.026673000305891037f,
    0.02489358000457287f,
    -0.003394934581592679f,
    -0.04307685047388077f,
    0.0032320574391633272f,
    0.0017968704923987389f,
    -0.06879908591508865f,
    -0.038426365703344345f,
    0.02407233417034149f,
    -0.025679871439933777f,
    0.0043227593414485455f,
};

static const std::vector<float> TASK_EMB_W = {
    -0.35178714990615845f, 
    -0.15885570645332336f, 
    0.40525177121162415f, 
    -0.16051964461803436f, 
    0.2536218762397766f, 
    -0.074232317507267f, 
    0.20271539688110352f, 
    -0.5662066340446472f, 
    -0.31300976872444153f, 
    0.11201275885105133f, 
    -0.09527330100536346f, 
    -0.5643627643585205f, 
    0.007109710946679115f, 
    0.41626039147377014f, 
    -0.1903800219297409f, 
    -0.23682862520217896f, 
};

static const std::vector<float> TASK_EMB_B = {
    -0.0398259311914444f,
    0.004976778756827116f,
    0.05264798924326897f,
    -0.04795251414179802f,
    0.05112456530332565f,
    -0.03839446231722832f,
    0.08662041276693344f,
    0.026535827666521072f,
    0.04582493007183075f,
    -0.03162156417965889f,
    0.085802361369133f,
    -0.10142409801483154f,
    0.06809300184249878f,
    0.06534626334905624f,
    0.07919225841760635f,
    -0.025243084877729416f,
};

static const std::vector<float> GCN0_W = {
    -0.19060467183589935f,     0.23047108948230743f,     -0.14960429072380066f,     0.02129817195236683f,     -0.7005875706672668f,     -0.28851553797721863f,     0.4542129635810852f,     0.5204823613166809f,     -0.708145260810852f,     -0.6078556180000305f,     0.13915880024433136f,     -0.00810747779905796f,     -0.026890428736805916f,     0.5050345063209534f,     0.5288873910903931f,     -0.6392835974693298f, 
    -0.3677683472633362f,     0.15529775619506836f,     0.40698960423469543f,     -0.48079895973205566f,     -0.10874982923269272f,     -0.24279014766216278f,     -0.18452750146389008f,     0.07955560088157654f,     0.013144232332706451f,     -0.09356704354286194f,     0.13787099719047546f,     -0.4954257607460022f,     -0.018885398283600807f,     -0.006581578403711319f,     0.39934924244880676f,     0.3352985382080078f, 
    1.0124813318252563f,     -0.8385495543479919f,     0.26324787735939026f,     -0.35746830701828003f,     0.004231580067425966f,     0.9878959059715271f,     -0.08858473598957062f,     -0.3976735472679138f,     0.7104858160018921f,     0.32398656010627747f,     -1.8222140073776245f,     0.127995565533638f,     -0.23607401549816132f,     -1.3609488010406494f,     -0.9991940259933472f,     0.5489894151687622f, 
    -0.46008819341659546f,     -0.3397294580936432f,     0.24670885503292084f,     -0.07439956068992615f,     0.015312401577830315f,     -0.2584303915500641f,     0.39052295684814453f,     0.3849608302116394f,     0.24872420728206635f,     0.008994943462312222f,     -0.06742429733276367f,     -0.41140663623809814f,     0.18471759557724f,     0.11413465440273285f,     -0.2568180561065674f,     -0.3071610629558563f, 
    0.23640409111976624f,     0.22332675755023956f,     0.49980056285858154f,     -0.3533743917942047f,     0.6111268401145935f,     0.3649525046348572f,     0.41972148418426514f,     -0.7272204160690308f,     0.5369576811790466f,     0.1643289476633072f,     0.3236524164676666f,     -0.010791102424263954f,     0.10777844488620758f,     -0.2349061816930771f,     -0.08557222038507462f,     1.488247036933899f, 
    0.12767373025417328f,     0.05736447870731354f,     -0.5093511939048767f,     0.6736966967582703f,     -0.10837625712156296f,     0.3424963355064392f,     -0.14966876804828644f,     0.0371377132833004f,     -0.5467544198036194f,     -0.5653094053268433f,     0.4630592465400696f,     0.28950971364974976f,     -0.5023133158683777f,     -0.10467711091041565f,     0.2303192913532257f,     -0.45996809005737305f, 
    0.3450765311717987f,     0.03970896452665329f,     0.581195592880249f,     -0.22013747692108154f,     0.25824296474456787f,     -0.3684963285923004f,     0.5119215250015259f,     -0.2644634544849396f,     0.43672868609428406f,     0.1092248484492302f,     0.5356956720352173f,     -0.29929405450820923f,     0.6983280181884766f,     0.44293251633644104f,     0.3819359242916107f,     -0.1683197021484375f, 
    -0.2449890375137329f,     0.13642023503780365f,     -0.2878904342651367f,     0.722226619720459f,     -0.38966062664985657f,     0.12439430505037308f,     0.24438002705574036f,     0.3064180016517639f,     -0.34862279891967773f,     -0.5636700391769409f,     0.6420276761054993f,     -0.2816774845123291f,     -0.0061719100922346115f,     0.5633185505867004f,     0.7211707234382629f,     -1.2820680141448975f, 
    0.024093758314847946f,     0.441226601600647f,     0.4231090247631073f,     0.10568287968635559f,     -0.23249687254428864f,     -0.4778434634208679f,     -0.1022561639547348f,     0.42979735136032104f,     0.020535724237561226f,     -0.3199964463710785f,     0.18220284581184387f,     -0.5424180030822754f,     0.38594427704811096f,     0.02742617391049862f,     -0.27714380621910095f,     -0.06935426592826843f, 
    0.47318530082702637f,     0.25756147503852844f,     -0.051489971578121185f,     -0.4602455794811249f,     0.5511751770973206f,     0.3639707565307617f,     0.3050408959388733f,     -0.40598443150520325f,     0.3941340148448944f,     0.6688079833984375f,     0.09148836880922318f,     -0.19326737523078918f,     0.20417903363704681f,     0.17627975344657898f,     -0.389603853225708f,     0.8435112833976746f, 
    -0.3131507635116577f,     -0.37128499150276184f,     0.23215878009796143f,     -0.417702853679657f,     0.27370500564575195f,     -0.44730812311172485f,     0.09152153879404068f,     0.5814492702484131f,     0.3194158673286438f,     -0.3671468198299408f,     0.15387177467346191f,     0.17139297723770142f,     0.309746652841568f,     -0.09019303321838379f,     0.1468556970357895f,     -0.4452357292175293f, 
    -0.056081969290971756f,     -0.3244647681713104f,     0.003273516660556197f,     -0.48990023136138916f,     0.4862566888332367f,     -0.08763112872838974f,     0.09233790636062622f,     -0.488010048866272f,     0.5250319838523865f,     0.2569471001625061f,     -0.33983126282691956f,     -0.3126937747001648f,     0.5703978538513184f,     0.134945347905159f,     -0.060429006814956665f,     1.6891674995422363f, 
    -0.04785352945327759f,     0.4002915322780609f,     0.13224884867668152f,     0.1971440464258194f,     -0.6331833004951477f,     0.11766183376312256f,     0.5133954286575317f,     0.39203429222106934f,     -0.5651509165763855f,     -0.4106147289276123f,     0.4352194368839264f,     -0.1428295373916626f,     -0.19297412037849426f,     -0.026329031214118004f,     0.15315337479114532f,     -1.1358113288879395f, 
    0.15154720842838287f,     0.3354199528694153f,     0.46902668476104736f,     -0.04554171487689018f,     0.4127401113510132f,     0.2875913083553314f,     0.3652837872505188f,     -0.4101406931877136f,     0.3469972312450409f,     0.5853502154350281f,     -0.18274787068367004f,     -0.15679340064525604f,     0.3060206472873688f,     0.1617232859134674f,     -0.14960920810699463f,     1.5338410139083862f, 
    0.4053172469139099f,     -0.05040499567985535f,     0.3539307415485382f,     0.041987668722867966f,     -0.010183547623455524f,     0.04339887201786041f,     0.049762945622205734f,     -0.41993534564971924f,     0.3933386504650116f,     -0.38392096757888794f,     0.3441823422908783f,     -0.3577294945716858f,     0.2739314138889313f,     -0.09690754860639572f,     0.05079963058233261f,     -0.2801799178123474f, 
    -0.08321898430585861f,     0.17021003365516663f,     -0.8935583829879761f,     0.7114365100860596f,     -0.5398293137550354f,     -0.023791640996932983f,     0.1271941214799881f,     -0.27606329321861267f,     -0.674717903137207f,     -0.015978801995515823f,     -0.022275367751717567f,     0.4583420753479004f,     -0.01204267144203186f,     -0.2327316850423813f,     0.16403846442699432f,     -0.41146060824394226f, 
};

static const std::vector<float> GCN0_B = {
    0.03951781243085861f,
    0.18921585381031036f,
    -0.2320135235786438f,
    0.12256553769111633f,
    0.05380775406956673f,
    -0.22488975524902344f,
    0.2504974901676178f,
    0.08208426833152771f,
    0.15897004306316376f,
    0.06949431449174881f,
    0.20714648067951202f,
    0.035948775708675385f,
    0.10879917442798615f,
    0.04919787496328354f,
    0.18184471130371094f,
    -0.18952786922454834f,
};

static const std::vector<float> GCN0_LN_W = {
};

static const std::vector<float> GCN0_LN_B = {
};

static const std::vector<float> LOUT_W = {
    -0.22376230359077454f,     -0.05347425118088722f,     0.1298263669013977f,     0.006605303380638361f,     0.20965774357318878f,     -0.0032516501378268003f,     0.07800843566656113f,     -0.1909133642911911f,     0.045349426567554474f,     0.20404350757598877f,     0.023079006001353264f,     0.038291219621896744f,     -0.16837501525878906f,     0.05059598386287689f,     -0.08581220358610153f,     0.020692477002739906f, 
    -0.20614007115364075f,     0.1912284642457962f,     0.025136571377515793f,     0.1997120976448059f,     0.27300330996513367f,     0.09516068547964096f,     0.09471249580383301f,     -0.4005352854728699f,     -0.0614723339676857f,     0.19916178286075592f,     0.11664377897977829f,     0.059301771223545074f,     -0.4273686408996582f,     0.19645245373249054f,     0.14670279622077942f,     0.09693736582994461f, 
    -0.1795472800731659f,     -0.09469980001449585f,     0.24977117776870728f,     -0.06702081114053726f,     0.09562798589468002f,     0.01868419721722603f,     -0.2614458203315735f,     0.00011368034029146656f,     -0.25930559635162354f,     -0.03495769575238228f,     -0.4441142678260803f,     0.06404466927051544f,     0.021116334944963455f,     0.0446811243891716f,     -0.2545183598995209f,     -0.052382975816726685f, 
    0.09564878791570663f,     0.18729794025421143f,     -0.017576074227690697f,     0.05654625967144966f,     -0.10484317690134048f,     0.15019215643405914f,     0.17248791456222534f,     0.13837863504886627f,     0.1812148243188858f,     -0.14653852581977844f,     0.13277365267276764f,     -0.3620527386665344f,     0.2624119520187378f,     -0.03836125507950783f,     -0.00817792396992445f,     -0.23666206002235413f, 
    0.1732226461172104f,     0.11143286526203156f,     0.03902395814657211f,     0.1863851100206375f,     -0.2840500473976135f,     -0.038606446236371994f,     0.0641932561993599f,     0.26528051495552063f,     0.1866159439086914f,     -0.1529736965894699f,     0.18586231768131256f,     -0.29607129096984863f,     -0.00824762787669897f,     -0.13149599730968475f,     0.16944488883018494f,     -0.08489491790533066f, 
    -0.20220860838890076f,     -0.1562354415655136f,     0.18425984680652618f,     -0.22762878239154816f,     0.05347564071416855f,     -0.023403732106089592f,     -0.12034688889980316f,     -0.15982848405838013f,     -0.3639881908893585f,     -0.05996705964207649f,     -0.17147314548492432f,     0.07397893071174622f,     -0.006233793217688799f,     0.058711063116788864f,     -0.11488603055477142f,     -0.08766813576221466f, 
    0.22763077914714813f,     0.16226312518119812f,     0.12704125046730042f,     0.11480515450239182f,     -0.2242090404033661f,     0.08335891366004944f,     0.18434648215770721f,     0.09932927787303925f,     0.08656495809555054f,     -0.22770348191261292f,     0.12286514043807983f,     -0.3708949387073517f,     0.26755139231681824f,     -0.0820765420794487f,     0.21610380709171295f,     -0.07307227700948715f, 
    0.08435548841953278f,     0.2270636260509491f,     0.031803376972675323f,     0.07597872614860535f,     -0.14735214412212372f,     -0.011661932803690434f,     0.10452481359243393f,     0.35472822189331055f,     0.09026601165533066f,     -0.0868719294667244f,     0.19764403998851776f,     -0.3423610329627991f,     0.2332320660352707f,     -0.23050476610660553f,     0.006133062765002251f,     -0.1845446228981018f, 
    -0.29840758442878723f,     0.20053435862064362f,     0.127947598695755f,     0.011821208521723747f,     0.28659588098526f,     -0.047883059829473495f,     0.1751343011856079f,     -0.40377262234687805f,     0.17021124064922333f,     0.3112218677997589f,     0.042674120515584946f,     0.35831308364868164f,     -0.24154134094715118f,     0.23614415526390076f,     0.13880862295627594f,     0.08852646499872208f, 
    -0.24899105727672577f,     0.03942892700433731f,     0.18544702231884003f,     -0.07781504094600677f,     0.06739024817943573f,     0.06163661926984787f,     -0.07020052522420883f,     -0.1684480756521225f,     -0.10553193837404251f,     0.09537098556756973f,     -0.09882288426160812f,     0.17155133187770844f,     0.005767076276242733f,     0.04949988052248955f,     0.03218470513820648f,     -0.0903615653514862f, 
    -0.0010656588710844517f,     0.10622511804103851f,     0.010001682676374912f,     0.1301107555627823f,     -0.20800691843032837f,     0.1403529942035675f,     0.08235397189855576f,     0.0313849076628685f,     0.1587371677160263f,     -0.05771995708346367f,     0.1603628396987915f,     -0.3289385735988617f,     0.23395557701587677f,     -0.051549021154642105f,     0.2243291586637497f,     -0.13977886736392975f, 
    0.0798458680510521f,     -0.19018632173538208f,     -0.06309421360492706f,     -0.3134048283100128f,     -0.24743863940238953f,     0.23597432672977448f,     -0.2699219584465027f,     0.11593718081712723f,     0.020864153280854225f,     -0.2724343240261078f,     -0.020253362134099007f,     0.025306107476353645f,     0.1338283121585846f,     -0.14869321882724762f,     -0.24642111361026764f,     0.27925601601600647f, 
    0.2593359649181366f,     -0.03775710612535477f,     0.015504621900618076f,     0.16711656749248505f,     -0.24588413536548615f,     0.3808533251285553f,     0.21834559738636017f,     0.32384878396987915f,     0.1720898151397705f,     -0.09076879918575287f,     0.13692936301231384f,     -0.18046678602695465f,     0.2985481917858124f,     -0.21415859460830688f,     0.19948771595954895f,     0.056631144136190414f, 
    0.04969315603375435f,     -0.12350796908140182f,     0.03729686513543129f,     -0.14644163846969604f,     -0.25768738985061646f,     0.26911890506744385f,     -0.24076564610004425f,     0.15203246474266052f,     0.021482687443494797f,     -0.09771012514829636f,     -0.18960535526275635f,     0.103028304874897f,     0.1660117208957672f,     -0.10174691677093506f,     -0.23169665038585663f,     0.21776436269283295f, 
    -0.10005342215299606f,     -0.07178027182817459f,     0.07069950550794601f,     0.08981184661388397f,     0.06760700792074203f,     0.0721188485622406f,     -0.07304079085588455f,     -0.12075161188840866f,     -0.10805823653936386f,     -0.12046638876199722f,     -0.045234449207782745f,     -0.11440553516149521f,     0.10967930406332016f,     -0.0950818806886673f,     -0.0059750922955572605f,     -0.06863559037446976f, 
    -0.0739203542470932f,     0.0867292657494545f,     -0.026600569486618042f,     0.00047069945139810443f,     0.251356840133667f,     -0.0983545258641243f,     0.08905130624771118f,     -0.4377022683620453f,     0.10226895660161972f,     0.23771335184574127f,     0.023830287158489227f,     0.3231491148471832f,     -0.28825581073760986f,     0.258116751909256f,     0.14824047684669495f,     0.09083326905965805f, 
    0.040766775608062744f,     -0.11454834789037704f,     -0.010508419945836067f,     0.002047448419034481f,     0.1743880808353424f,     0.2537156939506531f,     -0.06695909798145294f,     -0.13852079212665558f,     0.011618475429713726f,     0.2040284276008606f,     0.04156997799873352f,     0.1298738420009613f,     -0.03599987551569939f,     0.18031468987464905f,     0.039798907935619354f,     0.3052416741847992f, 
    0.12992869317531586f,     -0.19071416556835175f,     0.10862981528043747f,     -0.19444429874420166f,     -0.13721762597560883f,     0.22652000188827515f,     -0.15612685680389404f,     0.07818248122930527f,     0.02040562964975834f,     -0.19830675423145294f,     -0.2094251662492752f,     0.0783725380897522f,     0.06384923309087753f,     0.1493590921163559f,     -0.16596835851669312f,     0.39002525806427f, 
    -0.1721741110086441f,     0.0883866474032402f,     -0.04216097295284271f,     -0.0031311979983001947f,     0.044003985822200775f,     -0.11089862883090973f,     0.13797318935394287f,     -0.15206566452980042f,     -0.0648789182305336f,     -0.05072512477636337f,     0.09094246476888657f,     0.20369195938110352f,     -0.29167574644088745f,     0.16796112060546875f,     0.12430023401975632f,     0.08943424373865128f, 
    -0.11968131363391876f,     0.004559544380754232f,     -0.054522398859262466f,     0.12387482076883316f,     0.18826593458652496f,     0.10219646990299225f,     0.012650709599256516f,     -0.2879399061203003f,     0.11458536982536316f,     0.2156873345375061f,     0.10566692799329758f,     0.13177356123924255f,     -0.251067578792572f,     0.03641333431005478f,     -0.0036919317208230495f,     0.03300728648900986f, 
    -0.16372472047805786f,     -0.00935368612408638f,     -0.054777421057224274f,     0.07284185290336609f,     -0.14145687222480774f,     -0.06412358582019806f,     0.03778660669922829f,     -0.03479146584868431f,     -0.013253835029900074f,     -0.13795357942581177f,     -0.10267779976129532f,     0.09897465258836746f,     -0.03129637613892555f,     0.04067564010620117f,     0.04518415406346321f,     -0.08862481266260147f, 
    0.3083493709564209f,     0.022308316081762314f,     -0.16697482764720917f,     0.24442337453365326f,     -0.24650709331035614f,     0.3286497890949249f,     0.09175087511539459f,     0.26486220955848694f,     0.10953138768672943f,     -0.1068844273686409f,     0.22602461278438568f,     -0.29255518317222595f,     0.14742803573608398f,     -0.24909430742263794f,     0.05549733340740204f,     -0.31881922483444214f, 
    0.22804081439971924f,     0.09487525373697281f,     -0.052576757967472076f,     0.08610868453979492f,     -0.1349717676639557f,     -0.03849757835268974f,     0.06021494045853615f,     0.23717521131038666f,     0.1703176647424698f,     -0.24935829639434814f,     0.2069595605134964f,     -0.31592389941215515f,     0.2850465774536133f,     -0.2871931195259094f,     0.18923351168632507f,     -0.26775845885276794f, 
    -0.1598634570837021f,     0.0561109334230423f,     -0.015491985715925694f,     0.11003866046667099f,     0.2784007787704468f,     0.05720274895429611f,     0.1843038648366928f,     -0.35201993584632874f,     0.14001066982746124f,     0.23997507989406586f,     -0.03490098565816879f,     0.3221515119075775f,     -0.23833155632019043f,     0.27825987339019775f,     0.02113655023276806f,     0.03218945115804672f, 
    -0.035466425120830536f,     0.06280624866485596f,     -0.08894740790128708f,     0.04333852604031563f,     0.14305660128593445f,     0.025936929509043694f,     0.0863933265209198f,     -0.33220174908638f,     -0.04432252049446106f,     0.1706710308790207f,     0.04689232259988785f,     0.04664267227053642f,     -0.14410820603370667f,     0.20929335057735443f,     0.1415894776582718f,     -0.062271133065223694f, 
    -0.2076500505208969f,     0.1894812285900116f,     -0.07739697396755219f,     0.0377083346247673f,     0.18376624584197998f,     0.04839913547039032f,     0.14643041789531708f,     -0.3261638581752777f,     0.08535962551832199f,     0.09319368004798889f,     0.06542365998029709f,     0.10883189737796783f,     -0.26548847556114197f,     0.23788884282112122f,     0.03330715373158455f,     -0.1098497286438942f, 
    -0.21978548169136047f,     0.012678131461143494f,     -0.0737852081656456f,     -0.025233928114175797f,     0.13658204674720764f,     0.02111455798149109f,     0.07158522307872772f,     -0.12687963247299194f,     0.1683603674173355f,     0.11093725264072418f,     -0.05302884802222252f,     0.16992303729057312f,     -0.2402815967798233f,     0.10289701819419861f,     0.13454243540763855f,     -0.06941697001457214f, 
    -0.02594686672091484f,     0.17125239968299866f,     0.14356067776679993f,     0.04352835938334465f,     0.1364205926656723f,     -0.00039201806066557765f,     0.12831270694732666f,     -0.1988401710987091f,     0.011509635485708714f,     0.027963757514953613f,     -0.058119356632232666f,     0.035266876220703125f,     -0.2604929506778717f,     0.12559464573860168f,     -0.1035238727927208f,     0.08941497653722763f, 
    0.2593325674533844f,     0.1984795480966568f,     0.05326027795672417f,     0.04202191159129143f,     -0.15961016714572906f,     0.26604461669921875f,     0.18128785490989685f,     0.3244820833206177f,     0.10154514014720917f,     -0.281060129404068f,     0.12267101556062698f,     -0.3894590139389038f,     0.34007924795150757f,     -0.08547420799732208f,     0.21690364181995392f,     -0.23092423379421234f, 
    0.0956396833062172f,     -0.14632491767406464f,     0.02409345656633377f,     -0.1428254395723343f,     0.10026951134204865f,     -0.1523629128932953f,     -0.07686769217252731f,     -0.08317992091178894f,     -0.016389798372983932f,     -0.02561659924685955f,     0.01768767274916172f,     -0.08059487491846085f,     -0.03378068655729294f,     -0.03306421637535095f,     -0.050536755472421646f,     -0.12197384983301163f, 
    0.15205849707126617f,     0.226610466837883f,     0.14116288721561432f,     0.08583700656890869f,     -0.002749043982475996f,     0.44593268632888794f,     0.03702931106090546f,     0.3599402904510498f,     -5.149027128936723e-05f,     -0.21318629384040833f,     0.09544089436531067f,     -0.28111380338668823f,     0.24902954697608948f,     -0.20013995468616486f,     0.0943559855222702f,     0.0625309869647026f, 
    -0.15463562309741974f,     -0.05103137716650963f,     -0.1588493138551712f,     -0.018780358135700226f,     0.08447815477848053f,     -0.1007755920290947f,     0.10977557301521301f,     -0.2585476338863373f,     -0.0026593233924359083f,     0.0629323199391365f,     0.08634233474731445f,     0.20484299957752228f,     -0.28783494234085083f,     0.04375136271119118f,     -0.07999918609857559f,     0.10950031876564026f, 
    0.04883076995611191f,     0.029517510905861855f,     0.02933359146118164f,     -0.13003911077976227f,     -0.05143268406391144f,     -0.018656311556696892f,     0.021892836317420006f,     -0.10449468344449997f,     0.022451143711805344f,     -0.09616847336292267f,     -0.12419508397579193f,     -0.09001770615577698f,     -0.04458761587738991f,     -0.14130017161369324f,     -0.08453132212162018f,     -0.12138838320970535f, 
    -0.1828593909740448f,     0.03536814823746681f,     -0.5093079209327698f,     0.1169481948018074f,     0.030635451897978783f,     -0.13050808012485504f,     0.12018005549907684f,     -0.27949902415275574f,     0.02030208706855774f,     0.15787269175052643f,     0.21597841382026672f,     0.09556873142719269f,     -0.30343687534332275f,     0.2385539710521698f,     0.1270637959241867f,     -0.14370034635066986f, 
    -0.012958819046616554f,     -0.1132952868938446f,     0.10979238152503967f,     -0.06951605528593063f,     0.012603608891367912f,     -0.13221004605293274f,     -0.14243370294570923f,     -0.01984671875834465f,     0.12691858410835266f,     -0.004523304291069508f,     -0.023020807653665543f,     -0.06257864087820053f,     -0.01286687795072794f,     -0.14054320752620697f,     -0.10151415318250656f,     0.02485818974673748f, 
    0.06345295906066895f,     0.042182806879282f,     -0.031356558203697205f,     -0.10698244720697403f,     -0.09691555798053741f,     -0.03517110273241997f,     -0.1021927073597908f,     -0.12222423404455185f,     0.08491627126932144f,     0.08210765570402145f,     -0.0980236753821373f,     0.07366199046373367f,     -0.024518631398677826f,     -0.10685202479362488f,     0.009031740948557854f,     -0.07422739267349243f, 
    0.17557671666145325f,     -0.29166993498802185f,     0.10726049542427063f,     -0.16643129289150238f,     -0.19533641636371613f,     0.2814500331878662f,     -0.07983259111642838f,     0.0965023785829544f,     0.03552309423685074f,     -0.23765189945697784f,     -0.1805034577846527f,     -0.16549067199230194f,     0.22555623948574066f,     -0.35861775279045105f,     -0.3827430307865143f,     0.3313612937927246f, 
    0.20789675414562225f,     0.09352507442235947f,     -0.23886077105998993f,     0.14699852466583252f,     -0.17139199376106262f,     -0.5303450226783752f,     0.11495201289653778f,     0.1310456544160843f,     0.09904623031616211f,     -0.4194261431694031f,     0.1632491499185562f,     -0.36423617601394653f,     0.24336102604866028f,     -0.38822054862976074f,     0.027728483080863953f,     -0.4481571912765503f, 
    -0.30011653900146484f,     0.15059049427509308f,     -0.7866028547286987f,     -0.0019016440492123365f,     0.24342520534992218f,     -0.27243322134017944f,     0.14525312185287476f,     -0.30112433433532715f,     0.1987728625535965f,     0.27903464436531067f,     0.07161558419466019f,     0.24281570315361023f,     -0.35422900319099426f,     0.0851203128695488f,     0.12389044463634491f,     -0.11365530639886856f, 
    0.05999715253710747f,     -0.0469757504761219f,     -0.03539664298295975f,     -0.11625437438488007f,     -0.10988166928291321f,     0.22230663895606995f,     -0.15810427069664001f,     0.06924017518758774f,     -0.16041681170463562f,     -0.13015387952327728f,     -0.20362746715545654f,     -0.0040253084152936935f,     0.16073867678642273f,     0.06541426479816437f,     -0.03822436183691025f,     0.1059064120054245f, 
    0.09734544903039932f,     0.1162867620587349f,     0.027151698246598244f,     0.08305931091308594f,     -0.2285233736038208f,     -0.3901847004890442f,     0.15172070264816284f,     0.08546146005392075f,     0.16320128738880157f,     -0.2985798120498657f,     0.05767671391367912f,     -0.3770996630191803f,     0.06789013743400574f,     -0.30095595121383667f,     0.023400304839015007f,     -0.42222410440444946f, 
    -0.23164460062980652f,     0.16489066183567047f,     -0.7056443095207214f,     0.047305427491664886f,     0.27999693155288696f,     -0.21533159911632538f,     0.10175434499979019f,     -0.2935120165348053f,     0.05796286091208458f,     0.19134938716888428f,     0.09544852375984192f,     0.136641263961792f,     -0.415322870016098f,     0.2259678840637207f,     0.23087595403194427f,     -0.22054092586040497f, 
    0.13182899355888367f,     0.03584263101220131f,     -0.1926092952489853f,     -0.006730929017066956f,     -0.07635464519262314f,     0.15469902753829956f,     -0.1906193494796753f,     -0.03789886459708214f,     -0.06424169987440109f,     -0.12632110714912415f,     -0.07857358455657959f,     -0.05450446158647537f,     -0.10100779682397842f,     0.04415508732199669f,     0.07867831736803055f,     0.06840475648641586f, 
    0.12169704586267471f,     0.08238163590431213f,     -0.23368558287620544f,     0.15236017107963562f,     -0.28619644045829773f,     -0.4265957772731781f,     0.19147415459156036f,     0.15412892401218414f,     0.07086615264415741f,     -0.3762686252593994f,     0.1635165959596634f,     -0.47930532693862915f,     0.1905667632818222f,     -0.45286300778388977f,     0.20540446043014526f,     -0.2802508473396301f, 
    0.10336900502443314f,     0.13107411563396454f,     0.6869267821311951f,     0.11120493710041046f,     0.005075082182884216f,     -0.342405766248703f,     -0.00035676054540090263f,     0.05037388578057289f,     0.01671171560883522f,     -0.005250063259154558f,     0.07814972847700119f,     -0.13946032524108887f,     0.17486780881881714f,     -0.14567632973194122f,     -0.0970347449183464f,     -0.4386260509490967f, 
    -0.2962362468242645f,     -0.022630954161286354f,     -0.5080605745315552f,     0.1406196504831314f,     0.10400690138339996f,     -0.09594757854938507f,     0.20776867866516113f,     -0.163270503282547f,     0.0015948893269523978f,     0.023884305730462074f,     0.15879933536052704f,     0.13685517013072968f,     -0.2607874870300293f,     0.27205467224121094f,     -0.03456394374370575f,     0.030550146475434303f, 
    -0.09951868653297424f,     -0.015294765122234821f,     -0.136331245303154f,     -0.07422687113285065f,     -0.034901831299066544f,     -0.10860699415206909f,     0.07458541542291641f,     -0.01424929779022932f,     0.0042733135633170605f,     0.019564785063266754f,     0.05849844217300415f,     -0.02185679040849209f,     -0.008362121880054474f,     -0.12220192700624466f,     -0.08126146346330643f,     0.1020437702536583f, 
    0.27550530433654785f,     -0.03982677310705185f,     -0.04564175754785538f,     0.15476304292678833f,     -0.1991003006696701f,     -0.4391084909439087f,     0.07253095507621765f,     0.08616018295288086f,     0.1693849116563797f,     -0.22286134958267212f,     0.214393749833107f,     -0.4018832743167877f,     0.27303004264831543f,     -0.4141865372657776f,     0.08488497883081436f,     -0.41857412457466125f, 
};

static const std::vector<float> LOUT_B = {
    0.05840639770030975f,     0.09699375182390213f,     -0.20331890881061554f,     0.10394946485757828f,     0.10539331287145615f,     -0.18313029408454895f,     0.1437305212020874f,     0.16069743037223816f,     0.0771905779838562f,     -0.06425589323043823f,     0.13451194763183594f,     -0.15248551964759827f,     0.08517429977655411f,     -0.15602776408195496f,     -0.014553334563970566f,     0.1148352399468422f, 
    -0.0719439759850502f,     -0.2062883824110031f,     0.04104301705956459f,     0.09393791854381561f,     -0.03355282172560692f,     0.16224084794521332f,     0.1519922912120819f,     0.04994078353047371f,     0.07519323378801346f,     0.08313796669244766f,     0.0733129233121872f,     0.058763306587934494f,     0.10965371131896973f,     -0.016334980726242065f,     0.09992407262325287f,     0.06706123054027557f, 
    -0.024802489206194878f,     0.16102655231952667f,     0.011354113928973675f,     -0.004179380834102631f,     -0.20374397933483124f,     0.10138081014156342f,     0.16761288046836853f,     -0.08197296410799026f,     0.08699513226747513f,     0.17904062569141388f,     -0.03891785815358162f,     0.10729958117008209f,     0.01345222070813179f,     0.12283797562122345f,     -0.011629458516836166f,     0.09582096338272095f, 
};

static const std::vector<float> MEAN_W = {
    0.06691636145114899f,     0.16536250710487366f,     -0.24602581560611725f,     -0.2050352543592453f,     -0.1353711187839508f,     -0.23390662670135498f,     -0.18528875708580017f,     -0.24723802506923676f,     0.2695273458957672f,     0.20123501121997833f,     -0.2056490182876587f,     0.3039359152317047f,     -0.23474226891994476f,     0.2737022042274475f,     -0.020686887204647064f,     0.18734179437160492f, 
    0.10905693471431732f,     0.3721468448638916f,     0.11728370934724808f,     0.1024593636393547f,     0.02536638081073761f,     -0.2478395253419876f,     -0.19471363723278046f,     0.23133967816829681f,     0.11472207307815552f,     0.15024802088737488f,     0.16173262894153595f,     0.03741053491830826f,     -0.21602115035057068f,     0.007630185689777136f,     -0.2335798442363739f,     0.09615509957075119f, 
    0.01642332226037979f,     0.2844693064689636f,     -0.011534837074577808f,     -0.0006894593825563788f,     0.3428631126880646f,     -0.2910614013671875f,     0.3320816159248352f,     0.14376479387283325f,     -0.23591242730617523f,     0.29713284969329834f,     0.004593071527779102f,     -0.3090851604938507f,     -0.116366446018219f,     0.2842707931995392f,     0.002591810654848814f,     -0.2658930718898773f, 
};

} // namespace NN

#endif // __NN_PARAMETERS_DEF_H__
